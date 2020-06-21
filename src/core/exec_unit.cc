/*****************************************************************************
 *                                McPAT
 *                      SOFTWARE LICENSE AGREEMENT
 *            Copyright 2012 Hewlett-Packard Development Company, L.P.
 *                          All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.”
 *
 ***************************************************************************/

#include "exec_unit.h"

#include "XML_Parse.h"
#include "basic_circuit.h"
#include "const.h"
#include "io.h"
#include "parameter.h"

#include <algorithm>
#include <assert.h>
#include <cmath>
#include <iostream>
#include <string>

EXECU::EXECU(const ParseXML *XML_interface,
             int ithCore_,
             InputParameter *interface_ip_,
             double lsq_height_,
             const CoreDynParam &dyn_p_,
             bool exist_)
    : XML(XML_interface), ithCore(ithCore_), interface_ip(*interface_ip_),
      lsq_height(lsq_height_), coredynp(dyn_p_), rfu(0), scheu(0), int_bypass(0), intTagBypass(0), int_mul_bypass(0),
      intTag_mul_Bypass(0), fp_bypass(0), fpTagBypass(0), exist(exist_) {
  bool exist_flag = true;
  if (!exist) {
    return;
  }
  double fu_height = 0.0;
  clockRate = coredynp.clockRate;
  executionTime = coredynp.executionTime;
  rfu = new RegFU();
  rfu->set_params(XML, ithCore, &interface_ip, coredynp);
  rfu->computeArea();
  rfu->set_stats(XML);
  scheu = new SchedulerU();
  scheu->set_params(XML, ithCore, &interface_ip, coredynp);
  scheu->computeArea();
  scheu->set_stats(XML);
  exeu.set_params(XML, ithCore, &interface_ip, coredynp, ALU);
  exeu.set_stats(XML);
  exeu.computeArea();
  area.set_area(area.get_area() + exeu.area.get_area() + rfu->area.get_area() +
                scheu->area.get_area());
  fu_height = exeu.FU_height;
  if (coredynp.num_fpus > 0) {
    fp_u.set_params(XML, ithCore, &interface_ip, coredynp, FPU);
    fp_u.set_stats(XML);
    fp_u.computeArea();
    area.set_area(area.get_area() + fp_u.area.get_area());
  }
  if (coredynp.num_muls > 0) {
    mul.set_params(XML, ithCore, &interface_ip, coredynp, MUL);
    mul.set_stats(XML);
    mul.computeArea();
    area.set_area(area.get_area() + mul.area.get_area());
    fu_height += mul.FU_height;
  }
  /*
   * broadcast logic, including int-broadcast; int_tag-broadcast; fp-broadcast;
   * fp_tag-broadcast integer by pass has two paths and fp has 3 paths. on the
   * same bus there are multiple tri-state drivers and muxes that go to
   * different components on the same bus
   */
  if (XML->sys.Embedded) {
    interface_ip.wt = Global_30;
    interface_ip.wire_is_mat_type = 0;
    interface_ip.wire_os_mat_type = 0;
    interface_ip.throughput = 1.0 / clockRate;
    interface_ip.latency = 1.0 / clockRate;
  } else {
    interface_ip.wt = Global;
    interface_ip.wire_is_mat_type =
        2; // start from semi-global since local wires are already used
    interface_ip.wire_os_mat_type = 2;
    interface_ip.throughput = 10.0 / clockRate; // Do not care
    interface_ip.latency = 10.0 / clockRate;
  }

  if (coredynp.core_ty == Inorder) {
    int_bypass =
        new interconnect("Int Bypass Data",
                         Core_device,
                         1,
                         1,
                         int(ceil(XML->sys.machine_bits / 32.0) * 32),
                         rfu->int_regfile_height + exeu.FU_height + lsq_height,
                         &interface_ip,
                         3,
                         false,
                         1.0,
                         coredynp.opt_local,
                         coredynp.core_ty);
    bypass.area.set_area(bypass.area.get_area() + int_bypass->area.get_area());
    intTagBypass = new interconnect("Int Bypass tag",
                                    Core_device,
                                    1,
                                    1,
                                    coredynp.perThreadState,
                                    rfu->int_regfile_height + exeu.FU_height +
                                        lsq_height + scheu->Iw_height,
                                    &interface_ip,
                                    3,
                                    false,
                                    1.0,
                                    coredynp.opt_local,
                                    coredynp.core_ty);
    bypass.area.set_area(bypass.area.get_area() +
                         intTagBypass->area.get_area());

    if (coredynp.num_muls > 0) {
      int_mul_bypass =
          new interconnect("Mul Bypass Data",
                           Core_device,
                           1,
                           1,
                           int(ceil(XML->sys.machine_bits / 32.0) * 32 * 1.5),
                           rfu->fp_regfile_height + exeu.FU_height +
                               mul.FU_height + lsq_height,
                           &interface_ip,
                           3,
                           false,
                           1.0,
                           coredynp.opt_local,
                           coredynp.core_ty);
      bypass.area.set_area(bypass.area.get_area() +
                           int_mul_bypass->area.get_area());
      intTag_mul_Bypass =
          new interconnect("Mul Bypass tag",
                           Core_device,
                           1,
                           1,
                           coredynp.perThreadState,
                           rfu->fp_regfile_height + exeu.FU_height +
                               mul.FU_height + lsq_height + scheu->Iw_height,
                           &interface_ip,
                           3,
                           false,
                           1.0,
                           coredynp.opt_local,
                           coredynp.core_ty);
      bypass.area.set_area(bypass.area.get_area() +
                           intTag_mul_Bypass->area.get_area());
    }

    if (coredynp.num_fpus > 0) {
      fp_bypass =
          new interconnect("FP Bypass Data",
                           Core_device,
                           1,
                           1,
                           int(ceil(XML->sys.machine_bits / 32.0) * 32 * 1.5),
                           rfu->fp_regfile_height + fp_u.FU_height,
                           &interface_ip,
                           3,
                           false,
                           1.0,
                           coredynp.opt_local,
                           coredynp.core_ty);
      bypass.area.set_area(bypass.area.get_area() + fp_bypass->area.get_area());
      fpTagBypass = new interconnect("FP Bypass tag",
                                     Core_device,
                                     1,
                                     1,
                                     coredynp.perThreadState,
                                     rfu->fp_regfile_height + fp_u.FU_height +
                                         lsq_height + scheu->Iw_height,
                                     &interface_ip,
                                     3,
                                     false,
                                     1.0,
                                     coredynp.opt_local,
                                     coredynp.core_ty);
      bypass.area.set_area(bypass.area.get_area() +
                           fpTagBypass->area.get_area());
    }
  } else { // OOO
    if (coredynp.scheu_ty == PhysicalRegFile) {
      /* For physical register based OOO,
       * data broadcast interconnects cover across functional units, lsq, inst
       * windows and register files, while tag broadcast interconnects also
       * cover across ROB
       */
      int_bypass = new interconnect("Int Bypass Data",
                                    Core_device,
                                    1,
                                    1,
                                    int(ceil(coredynp.int_data_width)),
                                    rfu->int_regfile_height + exeu.FU_height +
                                        lsq_height,
                                    &interface_ip,
                                    3,
                                    false,
                                    1.0,
                                    coredynp.opt_local,
                                    coredynp.core_ty);
      bypass.area.set_area(bypass.area.get_area() +
                           int_bypass->area.get_area());
      intTagBypass = new interconnect("Int Bypass tag",
                                      Core_device,
                                      1,
                                      1,
                                      coredynp.phy_ireg_width,
                                      rfu->int_regfile_height +
                                          exeu.FU_height + lsq_height +
                                          scheu->Iw_height + scheu->ROB_height,
                                      &interface_ip,
                                      3,
                                      false,
                                      1.0,
                                      coredynp.opt_local,
                                      coredynp.core_ty);
      bypass.area.set_area(bypass.area.get_area() +
                           intTagBypass->area.get_area());

      if (coredynp.num_muls > 0) {
        int_mul_bypass =
            new interconnect("Mul Bypass Data",
                             Core_device,
                             1,
                             1,
                             int(ceil(coredynp.int_data_width)),
                             rfu->int_regfile_height + exeu.FU_height +
                                 mul.FU_height + lsq_height,
                             &interface_ip,
                             3,
                             false,
                             1.0,
                             coredynp.opt_local,
                             coredynp.core_ty);
        intTag_mul_Bypass = new interconnect(
            "Mul Bypass tag",
            Core_device,
            1,
            1,
            coredynp.phy_ireg_width,
            rfu->int_regfile_height + exeu.FU_height + mul.FU_height +
                lsq_height + scheu->Iw_height + scheu->ROB_height,
            &interface_ip,
            3,
            false,
            1.0,
            coredynp.opt_local,
            coredynp.core_ty);
        bypass.area.set_area(bypass.area.get_area() +
                             int_mul_bypass->area.get_area());
        bypass.area.set_area(bypass.area.get_area() +
                             intTag_mul_Bypass->area.get_area());
      }

      if (coredynp.num_fpus > 0) {
        fp_bypass = new interconnect("FP Bypass Data",
                                     Core_device,
                                     1,
                                     1,
                                     int(ceil(coredynp.fp_data_width)),
                                     rfu->fp_regfile_height + fp_u.FU_height,
                                     &interface_ip,
                                     3,
                                     false,
                                     1.0,
                                     coredynp.opt_local,
                                     coredynp.core_ty);
        fpTagBypass = new interconnect(
            "FP Bypass tag",
            Core_device,
            1,
            1,
            coredynp.phy_freg_width,
            rfu->fp_regfile_height + fp_u.FU_height + lsq_height +
                scheu->fp_Iw_height + scheu->ROB_height,
            &interface_ip,
            3,
            false,
            1.0,
            coredynp.opt_local,
            coredynp.core_ty);
        bypass.area.set_area(bypass.area.get_area() +
                             fp_bypass->area.get_area());
        bypass.area.set_area(bypass.area.get_area() +
                             fpTagBypass->area.get_area());
      }
    } else {
      /*
       * In RS based processor both data and tag are broadcast together,
       * covering functional units, lsq, nst windows, register files, and ROBs
       */
      int_bypass = new interconnect("Int Bypass Data",
                                    Core_device,
                                    1,
                                    1,
                                    int(ceil(coredynp.int_data_width)),
                                    rfu->int_regfile_height + exeu.FU_height +
                                        lsq_height + scheu->Iw_height +
                                        scheu->ROB_height,
                                    &interface_ip,
                                    3,
                                    false,
                                    1.0,
                                    coredynp.opt_local,
                                    coredynp.core_ty);
      intTagBypass = new interconnect("Int Bypass tag",
                                      Core_device,
                                      1,
                                      1,
                                      coredynp.phy_ireg_width,
                                      rfu->int_regfile_height +
                                          exeu.FU_height + lsq_height +
                                          scheu->Iw_height + scheu->ROB_height,
                                      &interface_ip,
                                      3,
                                      false,
                                      1.0,
                                      coredynp.opt_local,
                                      coredynp.core_ty);
      bypass.area.set_area(bypass.area.get_area() +
                           int_bypass->area.get_area());
      bypass.area.set_area(bypass.area.get_area() +
                           intTagBypass->area.get_area());
      if (coredynp.num_muls > 0) {
        int_mul_bypass = new interconnect(
            "Mul Bypass Data",
            Core_device,
            1,
            1,
            int(ceil(coredynp.int_data_width)),
            rfu->int_regfile_height + exeu.FU_height + mul.FU_height +
                lsq_height + scheu->Iw_height + scheu->ROB_height,
            &interface_ip,
            3,
            false,
            1.0,
            coredynp.opt_local,
            coredynp.core_ty);
        intTag_mul_Bypass = new interconnect(
            "Mul Bypass tag",
            Core_device,
            1,
            1,
            coredynp.phy_ireg_width,
            rfu->int_regfile_height + exeu.FU_height + mul.FU_height +
                lsq_height + scheu->Iw_height + scheu->ROB_height,
            &interface_ip,
            3,
            false,
            1.0,
            coredynp.opt_local,
            coredynp.core_ty);
        bypass.area.set_area(bypass.area.get_area() +
                             int_mul_bypass->area.get_area());
        bypass.area.set_area(bypass.area.get_area() +
                             intTag_mul_Bypass->area.get_area());
      }

      if (coredynp.num_fpus > 0) {
        fp_bypass = new interconnect("FP Bypass Data",
                                     Core_device,
                                     1,
                                     1,
                                     int(ceil(coredynp.fp_data_width)),
                                     rfu->fp_regfile_height + fp_u.FU_height +
                                         lsq_height + scheu->fp_Iw_height +
                                         scheu->ROB_height,
                                     &interface_ip,
                                     3,
                                     false,
                                     1.0,
                                     coredynp.opt_local,
                                     coredynp.core_ty);
        fpTagBypass = new interconnect(
            "FP Bypass tag",
            Core_device,
            1,
            1,
            coredynp.phy_freg_width,
            rfu->fp_regfile_height + fp_u.FU_height + lsq_height +
                scheu->fp_Iw_height + scheu->ROB_height,
            &interface_ip,
            3,
            false,
            1.0,
            coredynp.opt_local,
            coredynp.core_ty);
        bypass.area.set_area(bypass.area.get_area() +
                             fp_bypass->area.get_area());
        bypass.area.set_area(bypass.area.get_area() +
                             fpTagBypass->area.get_area());
      }
    }
  }
  area.set_area(area.get_area() + bypass.area.get_area());
}

void EXECU::computeEnergy(bool is_tdp) {
  if (!exist)
    return;
  double pppm_t[4] = {1, 1, 1, 1};
  //	rfu->power.reset();
  //	rfu->rt_power.reset();
  //	scheu->power.reset();
  //	scheu->rt_power.reset();
  //	exeu.power.reset();
  //	exeu.rt_power.reset();

  rfu->computeDynamicPower(is_tdp);
  scheu->computeDynamicPower(is_tdp);
  if(is_tdp) {
    exeu.computePower();
  }
  else {
    exeu.computeRuntimeDynamicPower();
  }
  if (coredynp.num_fpus > 0) {
    if(is_tdp) {
      fp_u.computePower();
    }
    else {
      fp_u.computeRuntimeDynamicPower();
    }
  }
  if (coredynp.num_muls > 0) {
    if(is_tdp) {
      mul.computePower();
    }
    else {
      mul.computeRuntimeDynamicPower();
    }
  }

  if (is_tdp) {
    set_pppm(
        pppm_t,
        2 * coredynp.ALU_cdb_duty_cycle,
        2,
        2,
        2 * coredynp
                .ALU_cdb_duty_cycle); // 2 means two source operands needs to be
                                      // passed for each int instruction.
    bypass.power = bypass.power + intTagBypass->power * pppm_t +
                   int_bypass->power * pppm_t;
    if (coredynp.num_muls > 0) {
      set_pppm(
          pppm_t,
          2 * coredynp.MUL_cdb_duty_cycle,
          2,
          2,
          2 * coredynp
                  .MUL_cdb_duty_cycle); // 2 means two source operands needs to
                                        // be passed for each int instruction.
      bypass.power = bypass.power + intTag_mul_Bypass->power * pppm_t +
                     int_mul_bypass->power * pppm_t;
      power = power + mul.power;
    }
    if (coredynp.num_fpus > 0) {
      set_pppm(
          pppm_t,
          3 * coredynp.FPU_cdb_duty_cycle,
          3,
          3,
          3 * coredynp
                  .FPU_cdb_duty_cycle); // 3 means three source operands needs
                                        // to be passed for each fp instruction.
      bypass.power = bypass.power + fp_bypass->power * pppm_t +
                     fpTagBypass->power * pppm_t;
      power = power + fp_u.power;
    }

    power = power + rfu->power + exeu.power + bypass.power + scheu->power;
  } else {
    set_pppm(pppm_t,
             XML->sys.core[ithCore].cdb_alu_accesses,
             2,
             2,
             XML->sys.core[ithCore].cdb_alu_accesses);
    bypass.rt_power = bypass.rt_power + intTagBypass->power * pppm_t;
    bypass.rt_power = bypass.rt_power + int_bypass->power * pppm_t;

    if (coredynp.num_muls > 0) {
      set_pppm(pppm_t,
               XML->sys.core[ithCore].cdb_mul_accesses,
               2,
               2,
               XML->sys.core[ithCore]
                   .cdb_mul_accesses); // 2 means two source operands needs to
                                       // be passed for each int instruction.
      bypass.rt_power = bypass.rt_power + intTag_mul_Bypass->power * pppm_t +
                        int_mul_bypass->power * pppm_t;
      rt_power = rt_power + mul.rt_power;
    }

    if (coredynp.num_fpus > 0) {
      set_pppm(pppm_t,
               XML->sys.core[ithCore].cdb_fpu_accesses,
               3,
               3,
               XML->sys.core[ithCore].cdb_fpu_accesses);
      bypass.rt_power = bypass.rt_power + fp_bypass->power * pppm_t;
      bypass.rt_power = bypass.rt_power + fpTagBypass->power * pppm_t;
      rt_power = rt_power + fp_u.rt_power;
    }
    rt_power = rt_power + rfu->rt_power + exeu.rt_power + bypass.rt_power +
               scheu->rt_power;
  }
}

void EXECU::displayEnergy(uint32_t indent, int plevel, bool is_tdp) {
  if (!exist)
    return;
  string indent_str(indent, ' ');
  string indent_str_next(indent + 2, ' ');
  bool long_channel = XML->sys.longer_channel_device;
  bool power_gating = XML->sys.power_gating;

  //	cout << indent_str_next << "Results Broadcast Bus Area = " <<
  // bypass->area.get_area() *1e-6 << " mm^2" << endl;
  if (is_tdp) {
    cout << indent_str << "Register Files:" << endl;
    cout << indent_str_next << "Area = " << rfu->area.get_area() * 1e-6
         << " mm^2" << endl;
    cout << indent_str_next
         << "Peak Dynamic = " << rfu->power.readOp.dynamic * clockRate << " W"
         << endl;
    cout << indent_str_next << "Subthreshold Leakage = "
         << (long_channel ? rfu->power.readOp.longer_channel_leakage
                          : rfu->power.readOp.leakage)
         << " W" << endl;
    if (power_gating)
      cout << indent_str_next << "Subthreshold Leakage with power gating = "
           << (long_channel
                   ? rfu->power.readOp.power_gated_with_long_channel_leakage
                   : rfu->power.readOp.power_gated_leakage)
           << " W" << endl;
    cout << indent_str_next
         << "Gate Leakage = " << rfu->power.readOp.gate_leakage << " W" << endl;
    cout << indent_str_next
         << "Runtime Dynamic = " << rfu->rt_power.readOp.dynamic / executionTime
         << " W" << endl;
    cout << endl;
    if (plevel > 3) {
      rfu->displayEnergy(indent + 4, is_tdp);
    }
    cout << indent_str << "Instruction Scheduler:" << endl;
    cout << indent_str_next << "Area = " << scheu->area.get_area() * 1e-6
         << " mm^2" << endl;
    cout << indent_str_next
         << "Peak Dynamic = " << scheu->power.readOp.dynamic * clockRate << " W"
         << endl;
    cout << indent_str_next << "Subthreshold Leakage = "
         << (long_channel ? scheu->power.readOp.longer_channel_leakage
                          : scheu->power.readOp.leakage)
         << " W" << endl;
    if (power_gating)
      cout << indent_str_next << "Subthreshold Leakage with power gating = "
           << (long_channel
                   ? scheu->power.readOp.power_gated_with_long_channel_leakage
                   : scheu->power.readOp.power_gated_leakage)
           << " W" << endl;
    cout << indent_str_next
         << "Gate Leakage = " << scheu->power.readOp.gate_leakage << " W"
         << endl;
    cout << indent_str_next << "Runtime Dynamic = "
         << scheu->rt_power.readOp.dynamic / executionTime << " W" << endl;
    cout << endl;
    if (plevel > 3) {
      scheu->displayEnergy(indent + 4, is_tdp);
    }
    exeu.display(indent, is_tdp);
    if (coredynp.num_fpus > 0) {
      fp_u.display(indent, is_tdp);
    }
    if (coredynp.num_muls > 0) {
      mul.display(indent, is_tdp);
    }
    cout << indent_str << "Results Broadcast Bus:" << endl;
    cout << indent_str_next
         << "Area Overhead = " << bypass.area.get_area() * 1e-6 << " mm^2"
         << endl;
    cout << indent_str_next
         << "Peak Dynamic = " << bypass.power.readOp.dynamic * clockRate << " W"
         << endl;
    cout << indent_str_next << "Subthreshold Leakage = "
         << (long_channel ? bypass.power.readOp.longer_channel_leakage
                          : bypass.power.readOp.leakage)
         << " W" << endl;
    if (power_gating)
      cout << indent_str_next << "Subthreshold Leakage with power gating = "
           << (long_channel
                   ? bypass.power.readOp.power_gated_with_long_channel_leakage
                   : bypass.power.readOp.power_gated_leakage)
           << " W" << endl;
    cout << indent_str_next
         << "Gate Leakage = " << bypass.power.readOp.gate_leakage << " W"
         << endl;
    cout << indent_str_next << "Runtime Dynamic = "
         << bypass.rt_power.readOp.dynamic / executionTime << " W" << endl;
    cout << endl;
  } else {
    cout << indent_str_next << "Register Files    Peak Dynamic = "
         << rfu->rt_power.readOp.dynamic * clockRate << " W" << endl;
    cout << indent_str_next << "Register Files    Subthreshold Leakage = "
         << rfu->rt_power.readOp.leakage << " W" << endl;
    cout << indent_str_next << "Register Files    Gate Leakage = "
         << rfu->rt_power.readOp.gate_leakage << " W" << endl;
    cout << indent_str_next << "Instruction Sheduler   Peak Dynamic = "
         << scheu->rt_power.readOp.dynamic * clockRate << " W" << endl;
    cout << indent_str_next << "Instruction Sheduler   Subthreshold Leakage = "
         << scheu->rt_power.readOp.leakage << " W" << endl;
    cout << indent_str_next << "Instruction Sheduler   Gate Leakage = "
         << scheu->rt_power.readOp.gate_leakage << " W" << endl;
    cout << indent_str_next << "Results Broadcast Bus   Peak Dynamic = "
         << bypass.rt_power.readOp.dynamic * clockRate << " W" << endl;
    cout << indent_str_next << "Results Broadcast Bus   Subthreshold Leakage = "
         << bypass.rt_power.readOp.leakage << " W" << endl;
    cout << indent_str_next << "Results Broadcast Bus   Gate Leakage = "
         << bypass.rt_power.readOp.gate_leakage << " W" << endl;
  }
}

EXECU ::~EXECU() {

  if (!exist)
    return;
  if (int_bypass) {
    delete int_bypass;
    int_bypass = 0;
  }
  if (intTagBypass) {
    delete intTagBypass;
    intTagBypass = 0;
  }
  if (int_mul_bypass) {
    delete int_mul_bypass;
    int_mul_bypass = 0;
  }
  if (intTag_mul_Bypass) {
    delete intTag_mul_Bypass;
    intTag_mul_Bypass = 0;
  }
  if (fp_bypass) {
    delete fp_bypass;
    fp_bypass = 0;
  }
  if (fpTagBypass) {
    delete fpTagBypass;
    fpTagBypass = 0;
  }
  if (rfu) {
    delete rfu;
    rfu = 0;
  }
  if (scheu) {
    delete scheu;
    scheu = 0;
  }
}
