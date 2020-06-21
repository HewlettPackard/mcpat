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

#include "noc.h"

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

NoC::NoC() {
  embedded = false;
  init_stats = false;
  init_params = false;
  set_area = false;
  router_exist = false;
  link_bus_exist = false;
  M_traffic_pattern = 0.0;
  link_len = 0.0;
  ithNoC = 0;
  executionTime = 0.0;
  scktRatio = 0.0;
  chip_PR_overhead = 0.0;
  macro_PR_overhead = 0.0;
  long_channel = false;
  power_gating = false;
  total_accesses = 0;
}

void NoC::set_params(const ParseXML *XML,
                     int ithNoC_,
                     InputParameter *interface_ip_,
                     double M_traffic_pattern_,
                     double link_len_) {
  /*
   * initialize, compute and optimize individual components.
   */
  interface_ip = *interface_ip_;
  link_len = link_len_;
  M_traffic_pattern = M_traffic_pattern_;
  ithNoC = ithNoC_;
  embedded = XML->sys.Embedded;
  long_channel = XML->sys.longer_channel_device;
  power_gating = XML->sys.power_gating;
  total_accesses = XML->sys.NoC[ithNoC].total_accesses;

  if (embedded) {
    interface_ip.wt = Global_30;
    interface_ip.wire_is_mat_type = 0;
    interface_ip.wire_os_mat_type = 1;
  } else {
    interface_ip.wt = Global;
    interface_ip.wire_is_mat_type = 2;
    interface_ip.wire_os_mat_type = 2;
  }
  set_noc_param(XML);
  local_result = init_interface(&interface_ip);
  scktRatio = g_tp.sckt_co_eff;

  init_params = true;
}

void NoC::init_router() {
  router.init(nocdynp.flit_size,
              nocdynp.virtual_channel_per_port *
                  nocdynp.input_buffer_entries_per_vc,
              nocdynp.virtual_channel_per_port,
              &(g_tp.peri_global),
              nocdynp.input_ports,
              nocdynp.output_ports,
              M_traffic_pattern);
  // router.print_router();
  area.set_area(area.get_area() + router.area.get_area() * nocdynp.total_nodes);

  double long_channel_device_reduction =
      longer_channel_device_reduction(Uncore_device);
  double pg_reduction_retain =
      power_gating_leakage_reduction(false); // state retaining array structure;
  double pg_reduction_nonretain = power_gating_leakage_reduction(
      false); // non-state-retaining array structure;

  router.power.readOp.longer_channel_leakage =
      router.power.readOp.leakage * long_channel_device_reduction;
  router.buffer.power.readOp.longer_channel_leakage =
      router.buffer.power.readOp.leakage * long_channel_device_reduction;
  router.crossbar.power.readOp.longer_channel_leakage =
      router.crossbar.power.readOp.leakage * long_channel_device_reduction;
  router.arbiter.power.readOp.longer_channel_leakage =
      router.arbiter.power.readOp.leakage * long_channel_device_reduction;

  router.buffer.power.readOp.power_gated_leakage =
      router.buffer.power.readOp.leakage *
      pg_reduction_retain; // TODO: this is a simplified version; should use the
                           // power_gated_leakage generated in buff
  router.crossbar.power.readOp.power_gated_leakage =
      router.crossbar.power.readOp.leakage * pg_reduction_nonretain;
  router.arbiter.power.readOp.power_gated_leakage =
      router.arbiter.power.readOp.leakage * pg_reduction_nonretain;
  router.power.readOp.power_gated_leakage =
      router.buffer.power.readOp.power_gated_leakage +
      router.crossbar.power.readOp.power_gated_leakage +
      router.arbiter.power.readOp.power_gated_leakage;

  router.buffer.power.readOp.power_gated_with_long_channel_leakage =
      router.buffer.power.readOp.power_gated_leakage *
      long_channel_device_reduction; // TODO: this is a simplified version;
                                     // should use the power_gated_leakage
                                     // generated in buff
  router.crossbar.power.readOp.power_gated_with_long_channel_leakage =
      router.crossbar.power.readOp.power_gated_leakage *
      long_channel_device_reduction;
  router.arbiter.power.readOp.power_gated_with_long_channel_leakage =
      router.arbiter.power.readOp.power_gated_leakage *
      long_channel_device_reduction;
  router.power.readOp.power_gated_with_long_channel_leakage =
      router.buffer.power.readOp.power_gated_with_long_channel_leakage +
      router.crossbar.power.readOp.power_gated_with_long_channel_leakage +
      router.arbiter.power.readOp.power_gated_with_long_channel_leakage;

  router_exist = true;
}

void NoC::init_link_bus(double link_len_) {
  //	if (nocdynp.min_ports==1 )
  if (nocdynp.type) {
    link_name = "Links";
  } else {
    link_name = "Bus";
  }

  link_len = link_len_;
  assert(link_len > 0);

  interface_ip.throughput = nocdynp.link_throughput / nocdynp.clockRate;
  interface_ip.latency = nocdynp.link_latency / nocdynp.clockRate;

  link_len /= (nocdynp.horizontal_nodes + nocdynp.vertical_nodes) / 2;

  if (nocdynp.total_nodes > 1) {
    link_len /= 2; // All links are shared by neighbors
  }
  link_bus.init(name,
                Uncore_device,
                1,
                1,
                nocdynp.flit_size,
                link_len,
                &interface_ip,
                3,
                true /*pipelinable*/,
                nocdynp.route_over_perc);

  link_bus_tot_per_Router.area.set_area(
      link_bus_tot_per_Router.area.get_area() +
      link_bus.area.get_area() * nocdynp.global_linked_ports);

  area.set_area(area.get_area() +
                link_bus_tot_per_Router.area.get_area() * nocdynp.total_nodes);
  link_bus_exist = true;
}

void NoC::computeArea() {
  if (nocdynp.type) {
    /*
     * if NOC compute router, router links must be computed
     * separately and called from external since total chip
     * area must be known first
     */
    init_router();
  } else {
    init_link_bus(link_len); // if bus compute bus
  }
  set_area = true;
}

void NoC::set_stats(const ParseXML *XML) {
  total_accesses = XML->sys.NoC[ithNoC].total_accesses;
  init_stats = true;
}

void NoC::computePower() {
  double pppm_t[4] = {1, 1, 1, 1};
  double M = nocdynp.duty_cycle;
  // init stats for TDP
  stats_t.readAc.access = M;
  tdp_stats = stats_t;
  if (router_exist) {
    set_pppm(pppm_t, 1 * M, 1, 1, 1); // reset traffic pattern
    router.power = router.power * pppm_t;
    set_pppm(pppm_t,
             nocdynp.total_nodes,
             nocdynp.total_nodes,
             nocdynp.total_nodes,
             nocdynp.total_nodes);
    power = power + router.power * pppm_t;
  }
  if (link_bus_exist) {
    if (nocdynp.type)
      set_pppm(pppm_t,
               1 * M_traffic_pattern * M * (nocdynp.min_ports - 1),
               nocdynp.global_linked_ports,
               nocdynp.global_linked_ports,
               nocdynp.global_linked_ports);
    // reset traffic pattern; local port do not have router links
    else
      set_pppm(pppm_t,
               1 * M_traffic_pattern * M * (nocdynp.min_ports),
               nocdynp.global_linked_ports,
               nocdynp.global_linked_ports,
               nocdynp.global_linked_ports); // reset traffic pattern

    link_bus_tot_per_Router.power = link_bus.power * pppm_t;

    set_pppm(pppm_t,
             nocdynp.total_nodes,
             nocdynp.total_nodes,
             nocdynp.total_nodes,
             nocdynp.total_nodes);
    power = power + link_bus_tot_per_Router.power * pppm_t;
  }
}

void NoC::computeRuntimeDynamicPower() {
  double pppm_t[4] = {1, 1, 1, 1};
  // init stats for runtime power (RTP)
  stats_t.readAc.access = total_accesses;
  rtp_stats = stats_t;
  set_pppm(pppm_t, 1, 0, 0, 0);
  if (router_exist) {
    router.buffer.rt_power.readOp.dynamic =
        (router.buffer.power.readOp.dynamic +
         router.buffer.power.writeOp.dynamic) *
        rtp_stats.readAc.access;
    router.crossbar.rt_power.readOp.dynamic =
        router.crossbar.power.readOp.dynamic * rtp_stats.readAc.access;
    router.arbiter.rt_power.readOp.dynamic =
        router.arbiter.power.readOp.dynamic * rtp_stats.readAc.access;

    router.rt_power =
        router.rt_power +
        (router.buffer.rt_power + router.crossbar.rt_power +
         router.arbiter.rt_power) *
            pppm_t +
        router.power * pppm_lkg; // TDP power must be calculated first!
    rt_power = rt_power + router.rt_power;
  }
  if (link_bus_exist) {
    set_pppm(pppm_t, rtp_stats.readAc.access, 1, 1, rtp_stats.readAc.access);
    link_bus.rt_power = link_bus.power * pppm_t;
    rt_power = rt_power + link_bus.rt_power;
  }
}

void NoC::display(uint32_t indent, int plevel, bool is_tdp) {
  string indent_str(indent, ' ');
  string indent_str_next(indent + 2, ' ');

  double M = M_traffic_pattern * nocdynp.duty_cycle;
  /*only router as a whole has been applied the M_traffic_pattern(0.6 by
   * default) factor in router.cc; When power of crossbars, arbiters, etc need
   * to be displayed, the M_traffic_pattern factor need to be applied together
   * with McPAT's extra traffic pattern.
   * */
  if (is_tdp) {
    std::cout << name << std::endl;
    std::cout << indent_str << "Area = " << area.get_area() * 1e-6 << " mm^2"
              << std::endl;
    std::cout << indent_str
              << "Peak Dynamic = " << power.readOp.dynamic * nocdynp.clockRate
              << " W" << std::endl;
    std::cout << indent_str << "Subthreshold Leakage = "
              << (long_channel ? power.readOp.longer_channel_leakage
                               : power.readOp.leakage)
              << " W" << std::endl;
    if (power_gating) {
      std::cout << indent_str << "Subthreshold Leakage with power gating = "
                << (power.readOp.power_gated_leakage *
                    (long_channel ? power.readOp.longer_channel_leakage /
                                        power.readOp.leakage
                                  : 1))
                << " W" << std::endl;
    }
    std::cout << indent_str << "Gate Leakage = " << power.readOp.gate_leakage
              << " W" << std::endl;
    std::cout << indent_str << "Runtime Dynamic = "
              << rt_power.readOp.dynamic / nocdynp.executionTime << " W"
              << std::endl;
    std::cout << std::endl;

    if (router_exist) {
      std::cout << indent_str << "Router: " << std::endl;
      std::cout << indent_str_next << "Area = " << router.area.get_area() * 1e-6
                << " mm^2" << std::endl;
      std::cout << indent_str_next << "Peak Dynamic = "
                << router.power.readOp.dynamic * nocdynp.clockRate << " W"
                << std::endl;
      std::cout << indent_str_next << "Subthreshold Leakage = "
                << (long_channel ? router.power.readOp.longer_channel_leakage
                                 : router.power.readOp.leakage)
                << " W" << std::endl;
      if (power_gating) {
        std::cout
            << indent_str_next << "Subthreshold Leakage with power gating = "
            << (long_channel
                    ? router.power.readOp.power_gated_with_long_channel_leakage
                    : router.power.readOp.power_gated_leakage)
            << " W" << std::endl;
      }
      std::cout << indent_str_next
                << "Gate Leakage = " << router.power.readOp.gate_leakage << " W"
                << std::endl;
      std::cout << indent_str_next << "Runtime Dynamic = "
                << router.rt_power.readOp.dynamic / nocdynp.executionTime
                << " W" << std::endl;
      std::cout << std::endl;
      if (plevel > 2) {
        std::cout << indent_str << indent_str
                  << "Virtual Channel Buffer:" << std::endl;
        std::cout << indent_str << indent_str_next << "Area = "
                  << router.buffer.area.get_area() * 1e-6 * nocdynp.input_ports
                  << " mm^2" << std::endl;
        std::cout << indent_str << indent_str_next << "Peak Dynamic = "
                  << (router.buffer.power.readOp.dynamic +
                      router.buffer.power.writeOp.dynamic) *
                         nocdynp.min_ports * M * nocdynp.clockRate
                  << " W" << std::endl;
        std::cout << indent_str << indent_str_next << "Subthreshold Leakage = "
                  << (long_channel
                          ? router.buffer.power.readOp.longer_channel_leakage *
                                nocdynp.input_ports
                          : router.buffer.power.readOp.leakage *
                                nocdynp.input_ports)
                  << " W" << std::endl;
        if (power_gating) {
          std::cout << indent_str << indent_str_next
                    << "Subthreshold Leakage with power gating = "
                    << (long_channel
                            ? router.buffer.power.readOp
                                  .power_gated_with_long_channel_leakage
                            : router.buffer.power.readOp.power_gated_leakage)
                    << " W" << std::endl;
        }
        std::cout << indent_str << indent_str_next << "Gate Leakage = "
                  << router.buffer.power.readOp.gate_leakage *
                         nocdynp.input_ports
                  << " W" << std::endl;
        std::cout << indent_str << indent_str_next << "Runtime Dynamic = "
                  << router.buffer.rt_power.readOp.dynamic /
                         nocdynp.executionTime
                  << " W" << std::endl;
        std::cout << std::endl;
        std::cout << indent_str << indent_str << "Crossbar:" << std::endl;
        std::cout << indent_str << indent_str_next
                  << "Area = " << router.crossbar.area.get_area() * 1e-6
                  << " mm^2" << std::endl;
        std::cout << indent_str << indent_str_next << "Peak Dynamic = "
                  << router.crossbar.power.readOp.dynamic * nocdynp.clockRate *
                         nocdynp.min_ports * M
                  << " W" << std::endl;
        std::cout << indent_str << indent_str_next << "Subthreshold Leakage = "
                  << (long_channel
                          ? router.crossbar.power.readOp.longer_channel_leakage
                          : router.crossbar.power.readOp.leakage)
                  << " W" << std::endl;
        if (power_gating)
          std::cout << indent_str << indent_str_next
                    << "Subthreshold Leakage with power gating = "
                    << (long_channel
                            ? router.crossbar.power.readOp
                                  .power_gated_with_long_channel_leakage
                            : router.crossbar.power.readOp.power_gated_leakage)
                    << " W" << std::endl;
        std::cout << indent_str << indent_str_next << "Gate Leakage = "
                  << router.crossbar.power.readOp.gate_leakage << " W"
                  << std::endl;
        std::cout << indent_str << indent_str_next << "Runtime Dynamic = "
                  << router.crossbar.rt_power.readOp.dynamic /
                         nocdynp.executionTime
                  << " W" << std::endl;
        std::cout << std::endl;
        std::cout << indent_str << indent_str << "Arbiter:" << std::endl;
        std::cout << indent_str << indent_str_next << "Peak Dynamic = "
                  << router.arbiter.power.readOp.dynamic * nocdynp.clockRate *
                         nocdynp.min_ports * M
                  << " W" << std::endl;
        std::cout << indent_str << indent_str_next << "Subthreshold Leakage = "
                  << (long_channel
                          ? router.arbiter.power.readOp.longer_channel_leakage
                          : router.arbiter.power.readOp.leakage)
                  << " W" << std::endl;
        if (power_gating)
          std::cout << indent_str << indent_str_next
                    << "Subthreshold Leakage with power gating = "
                    << (long_channel
                            ? router.arbiter.power.readOp
                                  .power_gated_with_long_channel_leakage
                            : router.arbiter.power.readOp.power_gated_leakage)
                    << " W" << std::endl;
        std::cout << indent_str << indent_str_next << "Gate Leakage = "
                  << router.arbiter.power.readOp.gate_leakage << " W"
                  << std::endl;
        std::cout << indent_str << indent_str_next << "Runtime Dynamic = "
                  << router.arbiter.rt_power.readOp.dynamic /
                         nocdynp.executionTime
                  << " W" << std::endl;
        std::cout << std::endl;
      }
    }
    if (link_bus_exist) {
      std::cout << indent_str << (nocdynp.type ? "Per Router " : "")
                << link_name << ": " << std::endl;
      std::cout << indent_str_next
                << "Area = " << link_bus_tot_per_Router.area.get_area() * 1e-6
                << " mm^2" << std::endl;
      std::cout << indent_str_next << "Peak Dynamic = "
                << link_bus_tot_per_Router.power.readOp.dynamic *
                       nocdynp.clockRate
                << " W" << std::endl;
      std::cout
          << indent_str_next << "Subthreshold Leakage = "
          << (long_channel
                  ? link_bus_tot_per_Router.power.readOp.longer_channel_leakage
                  : link_bus_tot_per_Router.power.readOp.leakage)
          << " W" << std::endl;
      if (power_gating)
        std::cout
            << indent_str_next << "Subthreshold Leakage with power gating = "
            << (long_channel
                    ? link_bus_tot_per_Router.power.readOp
                          .power_gated_with_long_channel_leakage
                    : link_bus_tot_per_Router.power.readOp.power_gated_leakage)
            << " W" << std::endl;
      std::cout << indent_str_next << "Gate Leakage = "
                << link_bus_tot_per_Router.power.readOp.gate_leakage << " W"
                << std::endl;
      std::cout << indent_str_next << "Runtime Dynamic = "
                << link_bus.rt_power.readOp.dynamic / nocdynp.executionTime
                << " W" << std::endl;
      std::cout << std::endl;
    }
  } else {
  }
}

void NoC::set_noc_param(const ParseXML *XML) {
  nocdynp.type = XML->sys.NoC[ithNoC].type;
  nocdynp.clockRate = XML->sys.NoC[ithNoC].clockrate;
  nocdynp.clockRate *= 1e6;
  nocdynp.executionTime =
      XML->sys.total_cycles / (XML->sys.target_core_clockrate * 1e6);

  nocdynp.flit_size = XML->sys.NoC[ithNoC].flit_bits;
  if (nocdynp.type) {
    nocdynp.input_ports = XML->sys.NoC[ithNoC].input_ports;
    nocdynp.output_ports = XML->sys.NoC[ithNoC].output_ports; // later minus 1
    nocdynp.min_ports = min(nocdynp.input_ports, nocdynp.output_ports);
    nocdynp.global_linked_ports =
        (nocdynp.input_ports - 1) + (nocdynp.output_ports - 1);
    /*
     * 	Except local i/o ports, all ports needs links( global_linked_ports);
     *  However only min_ports can be fully active simultaneously
     *  since the fewer number of ports (input or output ) is the bottleneck.
     */
  } else {
    nocdynp.input_ports = 1;
    nocdynp.output_ports = 1;
    nocdynp.min_ports = min(nocdynp.input_ports, nocdynp.output_ports);
    nocdynp.global_linked_ports = 1;
  }

  nocdynp.virtual_channel_per_port =
      XML->sys.NoC[ithNoC].virtual_channel_per_port;
  nocdynp.input_buffer_entries_per_vc =
      XML->sys.NoC[ithNoC].input_buffer_entries_per_vc;

  nocdynp.horizontal_nodes = XML->sys.NoC[ithNoC].horizontal_nodes;
  nocdynp.vertical_nodes = XML->sys.NoC[ithNoC].vertical_nodes;
  nocdynp.total_nodes = nocdynp.horizontal_nodes * nocdynp.vertical_nodes;
  nocdynp.duty_cycle = XML->sys.NoC[ithNoC].duty_cycle;
  nocdynp.has_global_link = XML->sys.NoC[ithNoC].has_global_link;
  nocdynp.link_throughput = XML->sys.NoC[ithNoC].link_throughput;
  nocdynp.link_latency = XML->sys.NoC[ithNoC].link_latency;
  nocdynp.chip_coverage = XML->sys.NoC[ithNoC].chip_coverage;
  nocdynp.route_over_perc = XML->sys.NoC[ithNoC].route_over_perc;

  assert(nocdynp.chip_coverage <= 1);
  assert(nocdynp.route_over_perc <= 1);

  if (nocdynp.type)
    name = "NOC";
  else
    name = "BUSES";

  if (XML->sys.NoC[ithNoC].vdd > 0) {
    interface_ip.specific_hp_vdd = true;
    interface_ip.specific_lop_vdd = true;
    interface_ip.specific_lstp_vdd = true;
    interface_ip.hp_Vdd = XML->sys.NoC[ithNoC].vdd;
    interface_ip.lop_Vdd = XML->sys.NoC[ithNoC].vdd;
    interface_ip.lstp_Vdd = XML->sys.NoC[ithNoC].vdd;
  }

  if (XML->sys.NoC[ithNoC].power_gating_vcc > -1) {
    interface_ip.specific_vcc_min = true;
    interface_ip.user_defined_vcc_min = XML->sys.NoC[ithNoC].power_gating_vcc;
  }
}

NoC ::~NoC() {
  // Do Nothing
}
