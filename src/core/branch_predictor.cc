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

#include "branch_predictor.h"

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

BranchPredictor::BranchPredictor(const ParseXML *XML_interface,
                                 int ithCore_,
                                 InputParameter *interface_ip_,
                                 const CoreDynParam &dyn_p_,
                                 bool exist_)
    : XML(XML_interface), ithCore(ithCore_), interface_ip(*interface_ip_),
      coredynp(dyn_p_), globalBPT(0), localBPT(0), L1_localBPT(0),
      L2_localBPT(0), chooser(0), RAS(0), exist(exist_) {
  /*
   * Branch Predictor, accessed during ID stage.
   * McPAT's branch predictor model is the tournament branch predictor used in
   * Alpha 21264, including global predictor, local two level predictor, and
   * Chooser. The Branch predictor also includes a RAS (return address stack)
   * for function calls Branch predictors are tagged by thread ID and modeled as
   * 1-way associative cache. However RAS return address stacks are duplicated
   * for each thread.
   * TODO:Data Width need to be computed more precisely	 *
   */
  if (!exist)
    return;
  int tag, data;

  clockRate = coredynp.clockRate;
  executionTime = coredynp.executionTime;
  interface_ip.assoc = 1;
  interface_ip.pure_cam = false;
  if (coredynp.multithreaded) {

    tag = int(log2(coredynp.num_hthreads) + EXTRA_TAG_BITS);
    interface_ip.specific_tag = 1;
    interface_ip.tag_w = tag;

    interface_ip.is_cache = true;
    interface_ip.pure_ram = false;
  } else {
    interface_ip.is_cache = false;
    interface_ip.pure_ram = true;
  }
  // Global predictor
  data =
      int(ceil(XML->sys.core[ithCore].predictor.global_predictor_bits / 8.0));
  interface_ip.line_sz = data;
  interface_ip.cache_sz =
      data * XML->sys.core[ithCore].predictor.global_predictor_entries;
  interface_ip.nbanks = 1;
  interface_ip.out_w = interface_ip.line_sz * 8;
  interface_ip.access_mode = 2;
  interface_ip.throughput = 1.0 / clockRate;
  interface_ip.latency = 1.0 / clockRate;
  interface_ip.obj_func_dyn_energy = 0;
  interface_ip.obj_func_dyn_power = 0;
  interface_ip.obj_func_leak_power = 0;
  interface_ip.obj_func_cycle_t = 1;
  interface_ip.num_rw_ports = 0;
  interface_ip.num_rd_ports = coredynp.predictionW;
  interface_ip.num_wr_ports = coredynp.predictionW;
  interface_ip.num_se_rd_ports = 0;
  globalBPT = new ArrayST(&interface_ip,
                          "Global Predictor",
                          Core_device,
                          coredynp.opt_local,
                          coredynp.core_ty);
  globalBPT->area.set_area(globalBPT->area.get_area() +
                           globalBPT->local_result.area);
  area.set_area(area.get_area() + globalBPT->local_result.area);

  // Local BPT (Level 1)
  data =
      int(ceil(XML->sys.core[ithCore].predictor.local_predictor_size[0] / 8.0));
  interface_ip.line_sz = data;
  interface_ip.cache_sz =
      data * XML->sys.core[ithCore].predictor.local_predictor_entries;
  interface_ip.nbanks = 1;
  interface_ip.out_w = interface_ip.line_sz * 8;
  interface_ip.access_mode = 2;
  interface_ip.throughput = 1.0 / clockRate;
  interface_ip.latency = 1.0 / clockRate;
  interface_ip.obj_func_dyn_energy = 0;
  interface_ip.obj_func_dyn_power = 0;
  interface_ip.obj_func_leak_power = 0;
  interface_ip.obj_func_cycle_t = 1;
  interface_ip.num_rw_ports = 0;
  interface_ip.num_rd_ports = coredynp.predictionW;
  interface_ip.num_wr_ports = coredynp.predictionW;
  interface_ip.num_se_rd_ports = 0;
  L1_localBPT = new ArrayST(&interface_ip,
                            "L1 local Predictor",
                            Core_device,
                            coredynp.opt_local,
                            coredynp.core_ty);
  L1_localBPT->area.set_area(L1_localBPT->area.get_area() +
                             L1_localBPT->local_result.area);
  area.set_area(area.get_area() + L1_localBPT->local_result.area);

  // Local BPT (Level 2)
  data =
      int(ceil(XML->sys.core[ithCore].predictor.local_predictor_size[1] / 8.0));
  interface_ip.line_sz = data;
  interface_ip.cache_sz =
      data * XML->sys.core[ithCore].predictor.local_predictor_entries;
  interface_ip.nbanks = 1;
  interface_ip.out_w = interface_ip.line_sz * 8;
  interface_ip.access_mode = 2;
  interface_ip.throughput = 1.0 / clockRate;
  interface_ip.latency = 1.0 / clockRate;
  interface_ip.obj_func_dyn_energy = 0;
  interface_ip.obj_func_dyn_power = 0;
  interface_ip.obj_func_leak_power = 0;
  interface_ip.obj_func_cycle_t = 1;
  interface_ip.num_rw_ports = 0;
  interface_ip.num_rd_ports = coredynp.predictionW;
  interface_ip.num_wr_ports = coredynp.predictionW;
  interface_ip.num_se_rd_ports = 0;
  L2_localBPT = new ArrayST(&interface_ip,
                            "L2 local Predictor",
                            Core_device,
                            coredynp.opt_local,
                            coredynp.core_ty);
  L2_localBPT->area.set_area(L2_localBPT->area.get_area() +
                             L2_localBPT->local_result.area);
  area.set_area(area.get_area() + L2_localBPT->local_result.area);

  // Chooser
  data =
      int(ceil(XML->sys.core[ithCore].predictor.chooser_predictor_bits / 8.0));
  interface_ip.line_sz = data;
  interface_ip.cache_sz =
      data * XML->sys.core[ithCore].predictor.chooser_predictor_entries;
  interface_ip.nbanks = 1;
  interface_ip.out_w = interface_ip.line_sz * 8;
  interface_ip.access_mode = 2;
  interface_ip.throughput = 1.0 / clockRate;
  interface_ip.latency = 1.0 / clockRate;
  interface_ip.obj_func_dyn_energy = 0;
  interface_ip.obj_func_dyn_power = 0;
  interface_ip.obj_func_leak_power = 0;
  interface_ip.obj_func_cycle_t = 1;
  interface_ip.num_rw_ports = 0;
  interface_ip.num_rd_ports = coredynp.predictionW;
  interface_ip.num_wr_ports = coredynp.predictionW;
  interface_ip.num_se_rd_ports = 0;
  chooser = new ArrayST(&interface_ip,
                        "Predictor Chooser",
                        Core_device,
                        coredynp.opt_local,
                        coredynp.core_ty);
  chooser->area.set_area(chooser->area.get_area() + chooser->local_result.area);
  area.set_area(area.get_area() + chooser->local_result.area);

  // RAS return address stacks are Duplicated for each thread.
  interface_ip.is_cache = false;
  interface_ip.pure_ram = true;
  data = int(ceil(coredynp.pc_width / 8.0));
  interface_ip.line_sz = data;
  interface_ip.cache_sz = data * XML->sys.core[ithCore].RAS_size;
  interface_ip.assoc = 1;
  interface_ip.nbanks = 1;
  interface_ip.out_w = interface_ip.line_sz * 8;
  interface_ip.access_mode = 2;
  interface_ip.throughput = 1.0 / clockRate;
  interface_ip.latency = 1.0 / clockRate;
  interface_ip.obj_func_dyn_energy = 0;
  interface_ip.obj_func_dyn_power = 0;
  interface_ip.obj_func_leak_power = 0;
  interface_ip.obj_func_cycle_t = 1;
  interface_ip.num_rw_ports = 0;
  interface_ip.num_rd_ports = coredynp.predictionW;
  interface_ip.num_wr_ports = coredynp.predictionW;
  interface_ip.num_se_rd_ports = 0;
  RAS = new ArrayST(
      &interface_ip, "RAS", Core_device, coredynp.opt_local, coredynp.core_ty);
  RAS->area.set_area(RAS->area.get_area() +
                     RAS->local_result.area * coredynp.num_hthreads);
  area.set_area(area.get_area() +
                RAS->local_result.area * coredynp.num_hthreads);
}

void BranchPredictor::computeEnergy(bool is_tdp) {
  if (!exist)
    return;
  double r_access;
  double w_access;
  if (is_tdp) {
    r_access = coredynp.predictionW * coredynp.BR_duty_cycle;
    w_access = 0 * coredynp.BR_duty_cycle;
    globalBPT->stats_t.readAc.access = r_access;
    globalBPT->stats_t.writeAc.access = w_access;
    globalBPT->tdp_stats = globalBPT->stats_t;

    L1_localBPT->stats_t.readAc.access = r_access;
    L1_localBPT->stats_t.writeAc.access = w_access;
    L1_localBPT->tdp_stats = L1_localBPT->stats_t;

    L2_localBPT->stats_t.readAc.access = r_access;
    L2_localBPT->stats_t.writeAc.access = w_access;
    L2_localBPT->tdp_stats = L2_localBPT->stats_t;

    chooser->stats_t.readAc.access = r_access;
    chooser->stats_t.writeAc.access = w_access;
    chooser->tdp_stats = chooser->stats_t;

    RAS->stats_t.readAc.access = r_access;
    RAS->stats_t.writeAc.access = w_access;
    RAS->tdp_stats = RAS->stats_t;
  } else {
    // The resolution of BPT accesses is coarse, but this is
    // because most simulators cannot track finer grained details
    r_access = XML->sys.core[ithCore].branch_instructions;
    w_access =
        XML->sys.core[ithCore].branch_mispredictions +
        0.1 * XML->sys.core[ithCore]
                  .branch_instructions; // 10% of BR will flip internal bits//0
    globalBPT->stats_t.readAc.access = r_access;
    globalBPT->stats_t.writeAc.access = w_access;
    globalBPT->rtp_stats = globalBPT->stats_t;

    L1_localBPT->stats_t.readAc.access = r_access;
    L1_localBPT->stats_t.writeAc.access = w_access;
    L1_localBPT->rtp_stats = L1_localBPT->stats_t;

    L2_localBPT->stats_t.readAc.access = r_access;
    L2_localBPT->stats_t.writeAc.access = w_access;
    L2_localBPT->rtp_stats = L2_localBPT->stats_t;

    chooser->stats_t.readAc.access = r_access;
    chooser->stats_t.writeAc.access = w_access;
    chooser->rtp_stats = chooser->stats_t;

    RAS->stats_t.readAc.access = XML->sys.core[ithCore].function_calls;
    RAS->stats_t.writeAc.access = XML->sys.core[ithCore].function_calls;
    RAS->rtp_stats = RAS->stats_t;
  }

  globalBPT->power_t.reset();
  L1_localBPT->power_t.reset();
  L2_localBPT->power_t.reset();
  chooser->power_t.reset();
  RAS->power_t.reset();

  globalBPT->power_t.readOp.dynamic +=
      globalBPT->local_result.power.readOp.dynamic *
          globalBPT->stats_t.readAc.access +
      globalBPT->stats_t.writeAc.access *
          globalBPT->local_result.power.writeOp.dynamic;
  L1_localBPT->power_t.readOp.dynamic +=
      L1_localBPT->local_result.power.readOp.dynamic *
          L1_localBPT->stats_t.readAc.access +
      L1_localBPT->stats_t.writeAc.access *
          L1_localBPT->local_result.power.writeOp.dynamic;

  L2_localBPT->power_t.readOp.dynamic +=
      L2_localBPT->local_result.power.readOp.dynamic *
          L2_localBPT->stats_t.readAc.access +
      L2_localBPT->stats_t.writeAc.access *
          L2_localBPT->local_result.power.writeOp.dynamic;

  chooser->power_t.readOp.dynamic +=
      chooser->local_result.power.readOp.dynamic *
          chooser->stats_t.readAc.access +
      chooser->stats_t.writeAc.access *
          chooser->local_result.power.writeOp.dynamic;
  RAS->power_t.readOp.dynamic +=
      RAS->local_result.power.readOp.dynamic * RAS->stats_t.readAc.access +
      RAS->stats_t.writeAc.access * RAS->local_result.power.writeOp.dynamic;

  if (is_tdp) {
    globalBPT->power =
        globalBPT->power_t + globalBPT->local_result.power * pppm_lkg;
    L1_localBPT->power =
        L1_localBPT->power_t + L1_localBPT->local_result.power * pppm_lkg;
    L2_localBPT->power =
        L2_localBPT->power_t + L2_localBPT->local_result.power * pppm_lkg;
    chooser->power = chooser->power_t + chooser->local_result.power * pppm_lkg;
    RAS->power =
        RAS->power_t + RAS->local_result.power * coredynp.pppm_lkg_multhread;

    power = power + globalBPT->power + L1_localBPT->power + L2_localBPT->power +
            chooser->power + RAS->power;
  } else {
    globalBPT->rt_power =
        globalBPT->power_t + globalBPT->local_result.power * pppm_lkg;
    L1_localBPT->rt_power =
        L1_localBPT->power_t + L1_localBPT->local_result.power * pppm_lkg;
    L2_localBPT->rt_power =
        L2_localBPT->power_t + L2_localBPT->local_result.power * pppm_lkg;
    chooser->rt_power =
        chooser->power_t + chooser->local_result.power * pppm_lkg;
    RAS->rt_power =
        RAS->power_t + RAS->local_result.power * coredynp.pppm_lkg_multhread;
    rt_power = rt_power + globalBPT->rt_power + L1_localBPT->rt_power +
               L2_localBPT->rt_power + chooser->rt_power + RAS->rt_power;
  }
}

void BranchPredictor::displayEnergy(uint32_t indent, int plevel, bool is_tdp) {
  if (!exist)
    return;
  string indent_str(indent, ' ');
  string indent_str_next(indent + 2, ' ');
  bool long_channel = XML->sys.longer_channel_device;
  bool power_gating = XML->sys.power_gating;
  if (is_tdp) {
    cout << indent_str << "Global Predictor:" << endl;
    cout << indent_str_next << "Area = " << globalBPT->area.get_area() * 1e-6
         << " mm^2" << endl;
    cout << indent_str_next
         << "Peak Dynamic = " << globalBPT->power.readOp.dynamic * clockRate
         << " W" << endl;
    cout << indent_str_next << "Subthreshold Leakage = "
         << (long_channel ? globalBPT->power.readOp.longer_channel_leakage
                          : globalBPT->power.readOp.leakage)
         << " W" << endl;
    if (power_gating)
      cout << indent_str_next << "Subthreshold Leakage with power gating = "
           << (long_channel ? globalBPT->power.readOp
                                  .power_gated_with_long_channel_leakage
                            : globalBPT->power.readOp.power_gated_leakage)
           << " W" << endl;
    cout << indent_str_next
         << "Gate Leakage = " << globalBPT->power.readOp.gate_leakage << " W"
         << endl;
    cout << indent_str_next << "Runtime Dynamic = "
         << globalBPT->rt_power.readOp.dynamic / executionTime << " W" << endl;
    cout << endl;
    cout << indent_str << "Local Predictor:" << endl;
    cout << indent_str << "L1_Local Predictor:" << endl;
    cout << indent_str_next << "Area = " << L1_localBPT->area.get_area() * 1e-6
         << " mm^2" << endl;
    cout << indent_str_next
         << "Peak Dynamic = " << L1_localBPT->power.readOp.dynamic * clockRate
         << " W" << endl;
    cout << indent_str_next << "Subthreshold Leakage = "
         << (long_channel ? L1_localBPT->power.readOp.longer_channel_leakage
                          : L1_localBPT->power.readOp.leakage)
         << " W" << endl;
    if (power_gating)
      cout << indent_str_next << "Subthreshold Leakage with power gating = "
           << (long_channel ? L1_localBPT->power.readOp
                                  .power_gated_with_long_channel_leakage
                            : L1_localBPT->power.readOp.power_gated_leakage)
           << " W" << endl;
    cout << indent_str_next
         << "Gate Leakage = " << L1_localBPT->power.readOp.gate_leakage << " W"
         << endl;
    cout << indent_str_next << "Runtime Dynamic = "
         << L1_localBPT->rt_power.readOp.dynamic / executionTime << " W"
         << endl;
    cout << endl;
    cout << indent_str << "L2_Local Predictor:" << endl;
    cout << indent_str_next << "Area = " << L2_localBPT->area.get_area() * 1e-6
         << " mm^2" << endl;
    cout << indent_str_next
         << "Peak Dynamic = " << L2_localBPT->power.readOp.dynamic * clockRate
         << " W" << endl;
    cout << indent_str_next << "Subthreshold Leakage = "
         << (long_channel ? L2_localBPT->power.readOp.longer_channel_leakage
                          : L2_localBPT->power.readOp.leakage)
         << " W" << endl;
    if (power_gating)
      cout << indent_str_next << "Subthreshold Leakage with power gating = "
           << (long_channel ? L2_localBPT->power.readOp
                                  .power_gated_with_long_channel_leakage
                            : L2_localBPT->power.readOp.power_gated_leakage)
           << " W" << endl;
    cout << indent_str_next
         << "Gate Leakage = " << L2_localBPT->power.readOp.gate_leakage << " W"
         << endl;
    cout << indent_str_next << "Runtime Dynamic = "
         << L2_localBPT->rt_power.readOp.dynamic / executionTime << " W"
         << endl;
    cout << endl;

    cout << indent_str << "Chooser:" << endl;
    cout << indent_str_next << "Area = " << chooser->area.get_area() * 1e-6
         << " mm^2" << endl;
    cout << indent_str_next
         << "Peak Dynamic = " << chooser->power.readOp.dynamic * clockRate
         << " W" << endl;
    cout << indent_str_next << "Subthreshold Leakage = "
         << (long_channel ? chooser->power.readOp.longer_channel_leakage
                          : chooser->power.readOp.leakage)
         << " W" << endl;
    if (power_gating)
      cout << indent_str_next << "Subthreshold Leakage with power gating = "
           << (long_channel
                   ? chooser->power.readOp.power_gated_with_long_channel_leakage
                   : chooser->power.readOp.power_gated_leakage)
           << " W" << endl;
    cout << indent_str_next
         << "Gate Leakage = " << chooser->power.readOp.gate_leakage << " W"
         << endl;
    cout << indent_str_next << "Runtime Dynamic = "
         << chooser->rt_power.readOp.dynamic / executionTime << " W" << endl;
    cout << endl;
    cout << indent_str << "RAS:" << endl;
    cout << indent_str_next << "Area = " << RAS->area.get_area() * 1e-6
         << " mm^2" << endl;
    cout << indent_str_next
         << "Peak Dynamic = " << RAS->power.readOp.dynamic * clockRate << " W"
         << endl;
    cout << indent_str_next << "Subthreshold Leakage = "
         << (long_channel ? RAS->power.readOp.longer_channel_leakage
                          : RAS->power.readOp.leakage)
         << " W" << endl;
    if (power_gating)
      cout << indent_str_next << "Subthreshold Leakage with power gating = "
           << (long_channel
                   ? RAS->power.readOp.power_gated_with_long_channel_leakage
                   : RAS->power.readOp.power_gated_leakage)
           << " W" << endl;
    cout << indent_str_next
         << "Gate Leakage = " << RAS->power.readOp.gate_leakage << " W" << endl;
    cout << indent_str_next
         << "Runtime Dynamic = " << RAS->rt_power.readOp.dynamic / executionTime
         << " W" << endl;
    cout << endl;
  } else {
    //		cout << indent_str_next << "Global Predictor    Peak Dynamic = "
    //<< globalBPT->rt_power.readOp.dynamic*clockRate << " W" << endl;
    // cout << indent_str_next << "Global Predictor    Subthreshold Leakage = "
    // << globalBPT->rt_power.readOp.leakage <<" W" << endl; 		cout <<
    // indent_str_next
    //<< "Global Predictor    Gate Leakage = " <<
    // globalBPT->rt_power.readOp.gate_leakage << " W" << endl; 		cout
    // << indent_str_next << "Local Predictor   Peak Dynamic = " <<
    // L1_localBPT->rt_power.readOp.dynamic*clockRate  << " W" << endl; 		cout
    // << indent_str_next << "Local Predictor   Subthreshold Leakage = " <<
    // L1_localBPT->rt_power.readOp.leakage  << " W" << endl; 		cout <<
    // indent_str_next << "Local Predictor   Gate Leakage = " <<
    // L1_localBPT->rt_power.readOp.gate_leakage  << " W" << endl; 		cout
    // << indent_str_next << "Chooser   Peak Dynamic = " <<
    // chooser->rt_power.readOp.dynamic*clockRate  << " W" << endl; 		cout
    // << indent_str_next << "Chooser   Subthreshold Leakage = " <<
    // chooser->rt_power.readOp.leakage  << " W" << endl; 		cout <<
    // indent_str_next
    //<< "Chooser   Gate Leakage = " << chooser->rt_power.readOp.gate_leakage <<
    //" W" << endl; 		cout << indent_str_next << "RAS   Peak Dynamic = "
    //<< RAS->rt_power.readOp.dynamic*clockRate  << " W" << endl;
    // cout << indent_str_next << "RAS   Subthreshold Leakage = " <<
    // RAS->rt_power.readOp.leakage  << " W" << endl; 		cout <<
    // indent_str_next
    // << "RAS   Gate Leakage = " << RAS->rt_power.readOp.gate_leakage  << " W"
    //<< endl;
  }
}

BranchPredictor ::~BranchPredictor() {

  if (!exist)
    return;
  if (globalBPT) {
    delete globalBPT;
    globalBPT = 0;
  }
  if (localBPT) {
    delete localBPT;
    localBPT = 0;
  }
  if (L1_localBPT) {
    delete L1_localBPT;
    L1_localBPT = 0;
  }
  if (L2_localBPT) {
    delete L2_localBPT;
    L2_localBPT = 0;
  }
  if (chooser) {
    delete chooser;
    chooser = 0;
  }
  if (RAS) {
    delete RAS;
    RAS = 0;
  }
}
