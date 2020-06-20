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

#include "regfile.h"

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

RegFU::RegFU(const ParseXML *XML_interface,
             int ithCore_,
             InputParameter *interface_ip_,
             const CoreDynParam &dyn_p_,
             bool exist_)
    : XML(XML_interface), ithCore(ithCore_), interface_ip(*interface_ip_),
      coredynp(dyn_p_), IRF(0), FRF(0), RFWIN(0), exist(exist_) {
  /*
   * processors have separate architectural register files for each thread.
   * therefore, the bypass buses need to travel across all the register files.
   */

  if (!exist)
    return;
  int data;

  clockRate = coredynp.clockRate;
  executionTime = coredynp.executionTime;
  //**********************************IRF***************************************
  data = coredynp.int_data_width;
  interface_ip.is_cache = false;
  interface_ip.pure_cam = false;
  interface_ip.pure_ram = true;
  interface_ip.line_sz = int(ceil(data / 32.0)) * 4;
  interface_ip.cache_sz = coredynp.num_IRF_entry * interface_ip.line_sz;
  interface_ip.assoc = 1;
  interface_ip.nbanks = 1;
  interface_ip.out_w = interface_ip.line_sz * 8;
  interface_ip.access_mode = 1;
  interface_ip.throughput = 1.0 / clockRate;
  interface_ip.latency = 1.0 / clockRate;
  interface_ip.obj_func_dyn_energy = 0;
  interface_ip.obj_func_dyn_power = 0;
  interface_ip.obj_func_leak_power = 0;
  interface_ip.obj_func_cycle_t = 1;
  interface_ip.num_rw_ports =
      1; // this is the transfer port for saving/restoring states when
         // exceptions happen.
  interface_ip.num_rd_ports = 2 * coredynp.peak_issueW;
  interface_ip.num_wr_ports = coredynp.peak_issueW;
  interface_ip.num_se_rd_ports = 0;
  IRF = new ArrayST(&interface_ip,
                    "Integer Register File",
                    Core_device,
                    coredynp.opt_local,
                    coredynp.core_ty);
  IRF->area.set_area(IRF->area.get_area() +
                     IRF->local_result.area * coredynp.num_pipelines *
                         cdb_overhead *
                         ((coredynp.scheu_ty == ReservationStation)
                              ? XML->sys.core[ithCore].number_hardware_threads
                              : 1));
  area.set_area(area.get_area() +
                IRF->local_result.area * coredynp.num_pipelines * cdb_overhead *
                    ((coredynp.scheu_ty == ReservationStation)
                         ? XML->sys.core[ithCore].number_hardware_threads
                         : 1));
  // area.set_area(area.get_area()*cdb_overhead);
  // output_data_csv(IRF.RF.local_result);

  //**********************************FRF***************************************
  data = coredynp.fp_data_width;
  interface_ip.is_cache = false;
  interface_ip.pure_cam = false;
  interface_ip.pure_ram = true;
  interface_ip.line_sz = int(ceil(data / 32.0)) * 4;
  interface_ip.cache_sz = coredynp.num_FRF_entry * interface_ip.line_sz;
  interface_ip.assoc = 1;
  interface_ip.nbanks = 1;
  interface_ip.out_w = interface_ip.line_sz * 8;
  interface_ip.access_mode = 1;
  interface_ip.throughput = 1.0 / clockRate;
  interface_ip.latency = 1.0 / clockRate;
  interface_ip.obj_func_dyn_energy = 0;
  interface_ip.obj_func_dyn_power = 0;
  interface_ip.obj_func_leak_power = 0;
  interface_ip.obj_func_cycle_t = 1;
  interface_ip.num_rw_ports =
      1; // this is the transfer port for saving/restoring states when
         // exceptions happen.
  interface_ip.num_rd_ports = 2 * XML->sys.core[ithCore].issue_width;
  interface_ip.num_wr_ports = XML->sys.core[ithCore].issue_width;
  interface_ip.num_se_rd_ports = 0;
  FRF = new ArrayST(&interface_ip,
                    "Floating point Register File",
                    Core_device,
                    coredynp.opt_local,
                    coredynp.core_ty);
  FRF->area.set_area(FRF->area.get_area() +
                     FRF->local_result.area * coredynp.num_fp_pipelines *
                         cdb_overhead *
                         ((coredynp.scheu_ty == ReservationStation)
                              ? XML->sys.core[ithCore].number_hardware_threads
                              : 1));
  area.set_area(area.get_area() +
                FRF->local_result.area * coredynp.num_fp_pipelines *
                    cdb_overhead *
                    ((coredynp.scheu_ty == ReservationStation)
                         ? XML->sys.core[ithCore].number_hardware_threads
                         : 1));
  // area.set_area(area.get_area()*cdb_overhead);
  // output_data_csv(FRF.RF.local_result);
  int_regfile_height = IRF->local_result.cache_ht *
                       ((coredynp.scheu_ty == ReservationStation)
                            ? XML->sys.core[ithCore].number_hardware_threads
                            : 1) *
                       sqrt(cdb_overhead);
  fp_regfile_height = FRF->local_result.cache_ht *
                      ((coredynp.scheu_ty == ReservationStation)
                           ? XML->sys.core[ithCore].number_hardware_threads
                           : 1) *
                      sqrt(cdb_overhead);
  // since a EXU is associated with each pipeline, the cdb should not have
  // longer length.
  if (coredynp.regWindowing) {
    //*********************************REG_WIN************************************
    data =
        coredynp
            .int_data_width; // ECC, and usually 2 regs are transfered together
                             // during window shifting.Niagara Mega cell
    interface_ip.is_cache = false;
    interface_ip.pure_cam = false;
    interface_ip.pure_ram = true;
    interface_ip.line_sz = int(ceil(data / 8.0));
    interface_ip.cache_sz = XML->sys.core[ithCore].register_windows_size *
                            IRF->l_ip.cache_sz *
                            XML->sys.core[ithCore].number_hardware_threads;
    interface_ip.assoc = 1;
    interface_ip.nbanks = 1;
    interface_ip.out_w = interface_ip.line_sz * 8;
    interface_ip.access_mode = 1;
    interface_ip.throughput = 4.0 / clockRate;
    interface_ip.latency = 4.0 / clockRate;
    interface_ip.obj_func_dyn_energy = 0;
    interface_ip.obj_func_dyn_power = 0;
    interface_ip.obj_func_leak_power = 0;
    interface_ip.obj_func_cycle_t = 1;
    interface_ip.num_rw_ports =
        1; // this is the transfer port for saving/restoring states when
           // exceptions happen.
    interface_ip.num_rd_ports = 0;
    interface_ip.num_wr_ports = 0;
    interface_ip.num_se_rd_ports = 0;
    RFWIN = new ArrayST(&interface_ip,
                        "RegWindow",
                        Core_device,
                        coredynp.opt_local,
                        coredynp.core_ty);
    RFWIN->area.set_area(RFWIN->area.get_area() +
                         RFWIN->local_result.area * coredynp.num_pipelines);
    area.set_area(area.get_area() +
                  RFWIN->local_result.area * coredynp.num_pipelines);
    // output_data_csv(RFWIN.RF.local_result);
  }
}

void RegFU::computeEnergy(bool is_tdp) {
  /*
   * Architecture RF and physical RF cannot be present at the same time.
   * Therefore, the RF stats can only refer to either ARF or PRF;
   * And the same stats can be used for both.
   */
  if (!exist)
    return;
  if (is_tdp) {
    // init stats for Peak
    IRF->stats_t.readAc.access =
        coredynp.issueW * 2 *
        (coredynp.ALU_duty_cycle * 1.1 +
         (coredynp.num_muls > 0 ? coredynp.MUL_duty_cycle : 0)) *
        coredynp.num_pipelines;
    IRF->stats_t.writeAc.access =
        coredynp.issueW *
        (coredynp.ALU_duty_cycle * 1.1 +
         (coredynp.num_muls > 0 ? coredynp.MUL_duty_cycle : 0)) *
        coredynp.num_pipelines;
    // Rule of Thumb: about 10% RF related instructions do not need to access
    // ALUs
    IRF->tdp_stats = IRF->stats_t;

    FRF->stats_t.readAc.access = FRF->l_ip.num_rd_ports *
                                 coredynp.FPU_duty_cycle * 1.05 *
                                 coredynp.num_fp_pipelines;
    FRF->stats_t.writeAc.access = FRF->l_ip.num_wr_ports *
                                  coredynp.FPU_duty_cycle * 1.05 *
                                  coredynp.num_fp_pipelines;
    FRF->tdp_stats = FRF->stats_t;
    if (coredynp.regWindowing) {
      RFWIN->stats_t.readAc.access = 0;  // 0.5*RFWIN->l_ip.num_rw_ports;
      RFWIN->stats_t.writeAc.access = 0; // 0.5*RFWIN->l_ip.num_rw_ports;
      RFWIN->tdp_stats = RFWIN->stats_t;
    }
  } else {
    // init stats for Runtime Dynamic (RTP)
    IRF->stats_t.readAc.access =
        XML->sys.core[ithCore]
            .int_regfile_reads; // TODO: no diff on archi and phy
    IRF->stats_t.writeAc.access = XML->sys.core[ithCore].int_regfile_writes;
    IRF->rtp_stats = IRF->stats_t;

    FRF->stats_t.readAc.access = XML->sys.core[ithCore].float_regfile_reads;
    FRF->stats_t.writeAc.access = XML->sys.core[ithCore].float_regfile_writes;
    FRF->rtp_stats = FRF->stats_t;
    if (coredynp.regWindowing) {
      RFWIN->stats_t.readAc.access = XML->sys.core[ithCore].function_calls * 16;
      RFWIN->stats_t.writeAc.access =
          XML->sys.core[ithCore].function_calls * 16;
      RFWIN->rtp_stats = RFWIN->stats_t;

      IRF->stats_t.readAc.access = XML->sys.core[ithCore].int_regfile_reads +
                                   XML->sys.core[ithCore].function_calls * 16;
      IRF->stats_t.writeAc.access = XML->sys.core[ithCore].int_regfile_writes +
                                    XML->sys.core[ithCore].function_calls * 16;
      IRF->rtp_stats = IRF->stats_t;

      FRF->stats_t.readAc.access = XML->sys.core[ithCore].float_regfile_reads +
                                   XML->sys.core[ithCore].function_calls * 16;
      ;
      FRF->stats_t.writeAc.access =
          XML->sys.core[ithCore].float_regfile_writes +
          XML->sys.core[ithCore].function_calls * 16;
      ;
      FRF->rtp_stats = FRF->stats_t;
    }
  }
  IRF->power_t.reset();
  FRF->power_t.reset();
  IRF->power_t.readOp.dynamic +=
      (IRF->stats_t.readAc.access * IRF->local_result.power.readOp.dynamic +
       IRF->stats_t.writeAc.access * IRF->local_result.power.writeOp.dynamic);
  FRF->power_t.readOp.dynamic +=
      (FRF->stats_t.readAc.access * FRF->local_result.power.readOp.dynamic +
       FRF->stats_t.writeAc.access * FRF->local_result.power.writeOp.dynamic);
  if (coredynp.regWindowing) {
    RFWIN->power_t.reset();
    RFWIN->power_t.readOp.dynamic +=
        (RFWIN->stats_t.readAc.access *
             RFWIN->local_result.power.readOp.dynamic +
         RFWIN->stats_t.writeAc.access *
             RFWIN->local_result.power.writeOp.dynamic);
  }

  if (is_tdp) {
    IRF->power = IRF->power_t +
                 ((coredynp.scheu_ty == ReservationStation)
                      ? (IRF->local_result.power * coredynp.pppm_lkg_multhread)
                      : IRF->local_result.power);
    FRF->power = FRF->power_t +
                 ((coredynp.scheu_ty == ReservationStation)
                      ? (FRF->local_result.power * coredynp.pppm_lkg_multhread)
                      : FRF->local_result.power);
    power = power + (IRF->power + FRF->power);
    if (coredynp.regWindowing) {
      RFWIN->power = RFWIN->power_t + RFWIN->local_result.power * pppm_lkg;
      power = power + RFWIN->power;
    }
  } else {
    IRF->rt_power =
        IRF->power_t +
        ((coredynp.scheu_ty == ReservationStation)
             ? (IRF->local_result.power * coredynp.pppm_lkg_multhread)
             : IRF->local_result.power);
    FRF->rt_power =
        FRF->power_t +
        ((coredynp.scheu_ty == ReservationStation)
             ? (FRF->local_result.power * coredynp.pppm_lkg_multhread)
             : FRF->local_result.power);
    rt_power = rt_power + (IRF->power_t + FRF->power_t);
    if (coredynp.regWindowing) {
      RFWIN->rt_power = RFWIN->power_t + RFWIN->local_result.power * pppm_lkg;
      rt_power = rt_power + RFWIN->rt_power;
    }
  }
}

void RegFU::displayEnergy(uint32_t indent, int plevel, bool is_tdp) {
  if (!exist)
    return;
  string indent_str(indent, ' ');
  string indent_str_next(indent + 2, ' ');
  bool long_channel = XML->sys.longer_channel_device;
  bool power_gating = XML->sys.power_gating;

  if (is_tdp) {
    cout << indent_str << "Integer RF:" << endl;
    cout << indent_str_next << "Area = " << IRF->area.get_area() * 1e-6
         << " mm^2" << endl;
    cout << indent_str_next
         << "Peak Dynamic = " << IRF->power.readOp.dynamic * clockRate << " W"
         << endl;
    cout << indent_str_next << "Subthreshold Leakage = "
         << (long_channel ? IRF->power.readOp.longer_channel_leakage
                          : IRF->power.readOp.leakage)
         << " W" << endl;
    if (power_gating)
      cout << indent_str_next << "Subthreshold Leakage with power gating = "
           << (long_channel
                   ? IRF->power.readOp.power_gated_with_long_channel_leakage
                   : IRF->power.readOp.power_gated_leakage)
           << " W" << endl;
    cout << indent_str_next
         << "Gate Leakage = " << IRF->power.readOp.gate_leakage << " W" << endl;
    cout << indent_str_next
         << "Runtime Dynamic = " << IRF->rt_power.readOp.dynamic / executionTime
         << " W" << endl;
    cout << endl;
    cout << indent_str << "Floating Point RF:" << endl;
    cout << indent_str_next << "Area = " << FRF->area.get_area() * 1e-6
         << " mm^2" << endl;
    cout << indent_str_next
         << "Peak Dynamic = " << FRF->power.readOp.dynamic * clockRate << " W"
         << endl;
    cout << indent_str_next << "Subthreshold Leakage = "
         << (long_channel ? FRF->power.readOp.longer_channel_leakage
                          : FRF->power.readOp.leakage)
         << " W" << endl;
    if (power_gating)
      cout << indent_str_next << "Subthreshold Leakage with power gating = "
           << (long_channel
                   ? FRF->power.readOp.power_gated_with_long_channel_leakage
                   : FRF->power.readOp.power_gated_leakage)
           << " W" << endl;
    cout << indent_str_next
         << "Gate Leakage = " << FRF->power.readOp.gate_leakage << " W" << endl;
    cout << indent_str_next
         << "Runtime Dynamic = " << FRF->rt_power.readOp.dynamic / executionTime
         << " W" << endl;
    cout << endl;
    if (coredynp.regWindowing) {
      cout << indent_str << "Register Windows:" << endl;
      cout << indent_str_next << "Area = " << RFWIN->area.get_area() * 1e-6
           << " mm^2" << endl;
      cout << indent_str_next
           << "Peak Dynamic = " << RFWIN->power.readOp.dynamic * clockRate
           << " W" << endl;
      cout << indent_str_next << "Subthreshold Leakage = "
           << (long_channel ? RFWIN->power.readOp.longer_channel_leakage
                            : RFWIN->power.readOp.leakage)
           << " W" << endl;
      if (power_gating)
        cout << indent_str_next << "Subthreshold Leakage with power gating = "
             << (long_channel
                     ? RFWIN->power.readOp.power_gated_with_long_channel_leakage
                     : RFWIN->power.readOp.power_gated_leakage)
             << " W" << endl;
      cout << indent_str_next
           << "Gate Leakage = " << RFWIN->power.readOp.gate_leakage << " W"
           << endl;
      cout << indent_str_next << "Runtime Dynamic = "
           << RFWIN->rt_power.readOp.dynamic / executionTime << " W" << endl;
      cout << endl;
    }
  } else {
    cout << indent_str_next << "Integer RF    Peak Dynamic = "
         << IRF->rt_power.readOp.dynamic * clockRate << " W" << endl;
    cout << indent_str_next << "Integer RF    Subthreshold Leakage = "
         << IRF->rt_power.readOp.leakage << " W" << endl;
    cout << indent_str_next
         << "Integer RF    Gate Leakage = " << IRF->rt_power.readOp.gate_leakage
         << " W" << endl;
    cout << indent_str_next << "Floating Point RF   Peak Dynamic = "
         << FRF->rt_power.readOp.dynamic * clockRate << " W" << endl;
    cout << indent_str_next << "Floating Point RF   Subthreshold Leakage = "
         << FRF->rt_power.readOp.leakage << " W" << endl;
    cout << indent_str_next << "Floating Point RF   Gate Leakage = "
         << FRF->rt_power.readOp.gate_leakage << " W" << endl;
    if (coredynp.regWindowing) {
      cout << indent_str_next << "Register Windows   Peak Dynamic = "
           << RFWIN->rt_power.readOp.dynamic * clockRate << " W" << endl;
      cout << indent_str_next << "Register Windows   Subthreshold Leakage = "
           << RFWIN->rt_power.readOp.leakage << " W" << endl;
      cout << indent_str_next << "Register Windows   Gate Leakage = "
           << RFWIN->rt_power.readOp.gate_leakage << " W" << endl;
    }
  }
}

RegFU ::~RegFU() {

  if (!exist)
    return;
  if (IRF) {
    delete IRF;
    IRF = 0;
  }
  if (FRF) {
    delete FRF;
    FRF = 0;
  }
  if (RFWIN) {
    delete RFWIN;
    RFWIN = 0;
  }
}
