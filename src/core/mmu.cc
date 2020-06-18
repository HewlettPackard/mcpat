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

#include "mmu.h"

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

MemManU::MemManU(){
  init_params = false;
  init_stats = false;
}

void MemManU::set_params(ParseXML *XML_interface, int ithCore_, InputParameter *interface_ip_, const CoreDynParam &dyn_p_, bool exist_){
  
  XML = XML_interface;
  interface_ip = *interface_ip_;
  coredynp = dyn_p_;
  ithCore = ithCore_;

  exist = exist_;

  if (!exist)
    return;
  int tag, data;
  bool debug = false;

  clockRate = coredynp.clockRate;
  executionTime = coredynp.executionTime;
  interface_ip.is_cache = true;
  interface_ip.pure_cam = false;
  interface_ip.pure_ram = false;
  interface_ip.specific_tag = 1;
  // Itlb TLBs are partioned among threads according to Nigara and Nehalem
  tag = XML->sys.virtual_address_width -
        int(floor(log2(XML->sys.virtual_memory_page_size))) +
        int(ceil(log2(XML->sys.core[ithCore].number_hardware_threads))) +
        EXTRA_TAG_BITS;
  data = XML->sys.physical_address_width -
         int(floor(log2(XML->sys.virtual_memory_page_size)));
  interface_ip.tag_w = tag;
  interface_ip.line_sz =
      int(ceil(data / 8.0)); // int(ceil(pow(2.0,ceil(log2(data)))/8.0));
  interface_ip.cache_sz =
      XML->sys.core[ithCore].itlb.number_entries *
      interface_ip.line_sz; //*XML->sys.core[ithCore].number_hardware_threads;
  interface_ip.assoc = 0;
  interface_ip.nbanks = 1;
  interface_ip.out_w = interface_ip.line_sz * 8;
  interface_ip.access_mode = 0;
  interface_ip.throughput =
      debug ? 1.0 / clockRate
            : XML->sys.core[ithCore].icache.icache_config[4] / clockRate;
  interface_ip.latency =
      debug ? 1.0 / clockRate
            : XML->sys.core[ithCore].icache.icache_config[5] / clockRate;
  interface_ip.obj_func_dyn_energy = 0;
  interface_ip.obj_func_dyn_power = 0;
  interface_ip.obj_func_leak_power = 0;
  interface_ip.obj_func_cycle_t = 1;
  interface_ip.num_rw_ports = 0;
  interface_ip.num_rd_ports = 0;
  interface_ip.num_wr_ports =
      debug ? 1 : XML->sys.core[ithCore].number_instruction_fetch_ports;
  interface_ip.num_se_rd_ports = 0;
  interface_ip.num_search_ports =
      debug ? 1 : XML->sys.core[ithCore].number_instruction_fetch_ports;
  itlb.set_params(
      &interface_ip, "ITLB", Core_device, coredynp.opt_local, coredynp.core_ty);
  // output_data_csv(itlb.tlb.local_result);

  // dtlb
  tag = XML->sys.virtual_address_width -
        int(floor(log2(XML->sys.virtual_memory_page_size))) +
        int(ceil(log2(XML->sys.core[ithCore].number_hardware_threads))) +
        EXTRA_TAG_BITS;
  data = XML->sys.physical_address_width -
         int(floor(log2(XML->sys.virtual_memory_page_size)));
  interface_ip.specific_tag = 1;
  interface_ip.tag_w = tag;
  interface_ip.line_sz =
      int(ceil(data / 8.0)); // int(ceil(pow(2.0,ceil(log2(data)))/8.0));
  interface_ip.cache_sz =
      XML->sys.core[ithCore].dtlb.number_entries *
      interface_ip.line_sz; //*XML->sys.core[ithCore].number_hardware_threads;
  interface_ip.assoc = 0;
  interface_ip.nbanks = 1;
  interface_ip.out_w = interface_ip.line_sz * 8;
  interface_ip.access_mode = 0;
  interface_ip.throughput =
      debug ? 1.0 / clockRate
            : XML->sys.core[ithCore].dcache.dcache_config[4] / clockRate;
  interface_ip.latency =
      debug ? 1.0 / clockRate
            : XML->sys.core[ithCore].dcache.dcache_config[5] / clockRate;
  interface_ip.obj_func_dyn_energy = 0;
  interface_ip.obj_func_dyn_power = 0;
  interface_ip.obj_func_leak_power = 0;
  interface_ip.obj_func_cycle_t = 1;
  interface_ip.num_rw_ports = 0;
  interface_ip.num_rd_ports = 0;
  interface_ip.num_wr_ports = XML->sys.core[ithCore].memory_ports;
  interface_ip.num_se_rd_ports = 0;
  interface_ip.num_search_ports = XML->sys.core[ithCore].memory_ports;
  dtlb.set_params(
      &interface_ip, "DTLB", Core_device, coredynp.opt_local, coredynp.core_ty);

  init_params = true;
}

void MemManU::computeArea(){
  if (!init_params) {
    std::cerr << "[ MemManU ] Error: must set params before calling "
                 "computeArea()\n";
                
    exit(1);
  }

  dtlb.computeArea();
  dtlb.area.set_area(dtlb.area.get_area() +
                           dtlb.local_result.area);
  area.set_area(area.get_area() + dtlb.local_result.area);

  itlb.computeArea();
  itlb.area.set_area(itlb.area.get_area() +
                           itlb.local_result.area);
  area.set_area(area.get_area() + itlb.local_result.area);

}

void MemManU::set_stats(const ParseXML *XML){
  init_stats = true;
}

void MemManU::computeStaticPower() {
  // NOTE: this does nothing, as the static power is optimized
  // along with the array area.
}


void MemManU::computeDynamicPower(bool is_tdp){
  if (!exist)
    return;
  if (!init_stats) {
    std::cerr << "[ MCFrontEnd ] Error: must set params before calling "
                 "computeDynamicPower()\n";
    exit(1);
  }
  if (is_tdp) {
    // init stats for Peak
    itlb.stats_t.readAc.access =
        itlb.l_ip.num_search_ports * coredynp.IFU_duty_cycle;
    itlb.stats_t.readAc.miss = 0;
    itlb.stats_t.readAc.hit =
        itlb.stats_t.readAc.access - itlb.stats_t.readAc.miss;
    itlb.tdp_stats = itlb.stats_t;

    dtlb.stats_t.readAc.access =
        dtlb.l_ip.num_search_ports * coredynp.LSU_duty_cycle;
    dtlb.stats_t.readAc.miss = 0;
    dtlb.stats_t.readAc.hit =
        dtlb.stats_t.readAc.access - dtlb.stats_t.readAc.miss;
    dtlb.tdp_stats = dtlb.stats_t;
  } else {
    // init stats for Runtime Dynamic (RTP)
    itlb.stats_t.readAc.access = XML->sys.core[ithCore].itlb.total_accesses;
    itlb.stats_t.readAc.miss = XML->sys.core[ithCore].itlb.total_misses;
    itlb.stats_t.readAc.hit =
        itlb.stats_t.readAc.access - itlb.stats_t.readAc.miss;
    itlb.rtp_stats = itlb.stats_t;

    dtlb.stats_t.readAc.access = XML->sys.core[ithCore].dtlb.total_accesses;
    dtlb.stats_t.readAc.miss = XML->sys.core[ithCore].dtlb.total_misses;
    dtlb.stats_t.readAc.hit =
        dtlb.stats_t.readAc.access - dtlb.stats_t.readAc.miss;
    dtlb.rtp_stats = dtlb.stats_t;
  }

  itlb.power_t.reset();
  dtlb.power_t.reset();
  itlb.power_t.readOp.dynamic +=
      itlb.stats_t.readAc.access * itlb.local_result.power.searchOp
                                        .dynamic // FA spent most power in tag,
                                                 // so use total access not hits
      + itlb.stats_t.readAc.miss * itlb.local_result.power.writeOp.dynamic;
  dtlb.power_t.readOp.dynamic +=
      dtlb.stats_t.readAc.access * dtlb.local_result.power.searchOp
                                        .dynamic // FA spent most power in tag,
                                                 // so use total access not hits
      + dtlb.stats_t.readAc.miss * dtlb.local_result.power.writeOp.dynamic;

  if (is_tdp) {
    itlb.power = itlb.power_t + itlb.local_result.power * pppm_lkg;
    dtlb.power = dtlb.power_t + dtlb.local_result.power * pppm_lkg;
    power = power + itlb.power + dtlb.power;
  } else {
    itlb.rt_power = itlb.power_t + itlb.local_result.power * pppm_lkg;
    dtlb.rt_power = dtlb.power_t + dtlb.local_result.power * pppm_lkg;
    rt_power = rt_power + itlb.rt_power + dtlb.rt_power;
  }

}

void MemManU::displayEnergy(uint32_t indent, int plevel, bool is_tdp) {
  if (!exist)
    return;
  string indent_str(indent, ' ');
  string indent_str_next(indent + 2, ' ');
  bool long_channel = XML->sys.longer_channel_device;
  bool power_gating = XML->sys.power_gating;

  if (is_tdp) {
    cout << indent_str << "Itlb:" << endl;
    cout << indent_str_next << "Area = " << itlb.area.get_area() * 1e-6
         << " mm^2" << endl;
    cout << indent_str_next
         << "Peak Dynamic = " << itlb.power.readOp.dynamic * clockRate << " W"
         << endl;
    cout << indent_str_next << "Subthreshold Leakage = "
         << (long_channel ? itlb.power.readOp.longer_channel_leakage
                          : itlb.power.readOp.leakage)
         << " W" << endl;
    if (power_gating)
      cout << indent_str_next << "Subthreshold Leakage with power gating = "
           << (long_channel
                   ? itlb.power.readOp.power_gated_with_long_channel_leakage
                   : itlb.power.readOp.power_gated_leakage)
           << " W" << endl;
    cout << indent_str_next
         << "Gate Leakage = " << itlb.power.readOp.gate_leakage << " W"
         << endl;
    cout << indent_str_next << "Runtime Dynamic = "
         << itlb.rt_power.readOp.dynamic / executionTime << " W" << endl;
    cout << endl;
    cout << indent_str << "Dtlb:" << endl;
    cout << indent_str_next << "Area = " << dtlb.area.get_area() * 1e-6
         << " mm^2" << endl;
    cout << indent_str_next
         << "Peak Dynamic = " << dtlb.power.readOp.dynamic * clockRate << " W"
         << endl;
    cout << indent_str_next << "Subthreshold Leakage = "
         << (long_channel ? dtlb.power.readOp.longer_channel_leakage
                          : dtlb.power.readOp.leakage)
         << " W" << endl;
    if (power_gating)
      cout << indent_str_next << "Subthreshold Leakage with power gating = "
           << (long_channel
                   ? dtlb.power.readOp.power_gated_with_long_channel_leakage
                   : dtlb.power.readOp.power_gated_leakage)
           << " W" << endl;
    cout << indent_str_next
         << "Gate Leakage = " << dtlb.power.readOp.gate_leakage << " W"
         << endl;
    cout << indent_str_next << "Runtime Dynamic = "
         << dtlb.rt_power.readOp.dynamic / executionTime << " W" << endl;
    cout << endl;
  } else {
    cout << indent_str_next << "Itlb    Peak Dynamic = "
         << itlb.rt_power.readOp.dynamic * clockRate << " W" << endl;
    cout << indent_str_next
         << "Itlb    Subthreshold Leakage = " << itlb.rt_power.readOp.leakage
         << " W" << endl;
    cout << indent_str_next
         << "Itlb    Gate Leakage = " << itlb.rt_power.readOp.gate_leakage
         << " W" << endl;
    cout << indent_str_next << "Dtlb   Peak Dynamic = "
         << dtlb.rt_power.readOp.dynamic * clockRate << " W" << endl;
    cout << indent_str_next
         << "Dtlb   Subthreshold Leakage = " << dtlb.rt_power.readOp.leakage
         << " W" << endl;
    cout << indent_str_next
         << "Dtlb   Gate Leakage = " << dtlb.rt_power.readOp.gate_leakage
         << " W" << endl;
  }
}

MemManU ::~MemManU() {

  if (!exist)
    return;
}