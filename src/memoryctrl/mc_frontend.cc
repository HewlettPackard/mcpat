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

#include "mc_frontend.h"

#include "XML_Parse.h"
#include "basic_circuit.h"
#include "basic_components.h"
#include "const.h"
#include "io.h"
#include "logic.h"
#include "parameter.h"

#include <algorithm>
#include <assert.h>
#include <cmath>
#include <iostream>
#include <string>

MCFrontEnd::MCFrontEnd() {
  long_channel = false;
  power_gating = false;
  init_params = false;
  init_stats = false;
  memory_channels_per_mc = 0;
  physical_address_width = 0;
  req_window_size_per_channel = 0;
  IO_buffer_size_per_channel = 0;
  memory_reads = 0;
  memory_writes = 0;
}

void MCFrontEnd::set_params(const ParseXML *XML,
                            InputParameter *interface_ip_,
                            const MCParam &mcp_,
                            enum MemoryCtrl_type mc_type_) {
  fe_ip = *interface_ip_;
  rb_ip = *interface_ip_;
  wb_ip = *interface_ip_;
  mcp = mcp_;
  mc_type = mc_type_;
  memory_channels_per_mc = XML->sys.mc.memory_channels_per_mc;
  physical_address_width = XML->sys.physical_address_width;
  req_window_size_per_channel = XML->sys.mc.req_window_size_per_channel;
  IO_buffer_size_per_channel = XML->sys.mc.IO_buffer_size_per_channel;
  long_channel = XML->sys.longer_channel_device;
  power_gating = XML->sys.power_gating;

  int tag, data;
  bool is_default = true; // indication for default setup

  /* MC frontend engine channels share the same engines but logically
   * partitioned For all hardware inside MC. different channels do not share
   * resources.
   * TODO: add docodeing/mux stage to steer memory requests to different
   * channels.
   */

  // memory request reorder buffer
  tag = mcp.addressBusWidth + EXTRA_TAG_BITS + mcp.opcodeW;
  data = int(ceil((physical_address_width + mcp.opcodeW) / 8.0));
  fe_ip.cache_sz = data * req_window_size_per_channel;
  fe_ip.line_sz = data;
  fe_ip.assoc = 0;
  fe_ip.nbanks = 1;
  fe_ip.out_w = fe_ip.line_sz * 8;
  fe_ip.specific_tag = 1;
  fe_ip.tag_w = tag;
  fe_ip.access_mode = 0;
  fe_ip.throughput = 1.0 / mcp.clockRate;
  fe_ip.latency = 1.0 / mcp.clockRate;
  fe_ip.is_cache = true;
  fe_ip.pure_cam = false;
  fe_ip.pure_ram = false;
  fe_ip.obj_func_dyn_energy = 0;
  fe_ip.obj_func_dyn_power = 0;
  fe_ip.obj_func_leak_power = 0;
  fe_ip.obj_func_cycle_t = 1;
  fe_ip.num_rw_ports = 0;
  fe_ip.num_rd_ports = memory_channels_per_mc;
  fe_ip.num_wr_ports = fe_ip.num_rd_ports;
  fe_ip.num_se_rd_ports = 0;
  fe_ip.num_search_ports = memory_channels_per_mc;
  frontendBuffer.set_params(&fe_ip, "MC ReorderBuffer", Uncore_device);

  // selection and arbitration logic
  fe_ip.assoc =
      1; // reset to prevent unnecessary warning messages when init_interface
  MC_arb = new selection_logic(
      is_default, req_window_size_per_channel, 1, &rb_ip, Uncore_device);

  // TODO: Verify whether or not this is a bug, Originally these were all
  // the same interface_ip which leads to slight differences in power if
  // removed.
  rb_ip = fe_ip;

  // read buffers.
  data = (int)ceil(mcp.dataBusWidth / 8.0); // Support key words first operation
                                            // //8 means converting bit to Byte
  rb_ip.cache_sz = data * IO_buffer_size_per_channel; //*llcBlockSize;
  rb_ip.line_sz = data;
  rb_ip.assoc = 1;
  rb_ip.nbanks = 1;
  rb_ip.out_w = rb_ip.line_sz * 8;
  rb_ip.access_mode = 1;
  rb_ip.throughput = 1.0 / mcp.clockRate;
  rb_ip.latency = 1.0 / mcp.clockRate;
  rb_ip.is_cache = false;
  rb_ip.pure_cam = false;
  rb_ip.pure_ram = true;
  rb_ip.obj_func_dyn_energy = 0;
  rb_ip.obj_func_dyn_power = 0;
  rb_ip.obj_func_leak_power = 0;
  rb_ip.obj_func_cycle_t = 1;
  rb_ip.num_rw_ports =
      0; // memory_channels_per_mc*2>2?2:memory_channels_per_mc*2;
  rb_ip.num_rd_ports = memory_channels_per_mc;
  rb_ip.num_wr_ports = rb_ip.num_rd_ports;
  rb_ip.num_se_rd_ports = 0;
  readBuffer.set_params(&rb_ip, "MC ReadBuffer", Uncore_device);

  // TODO: Verify whether or not this is a bug, Originally these were all
  // the same interface_ip which leads to slight differences in power if
  // removed.
  wb_ip = rb_ip;

  // write buffer
  data = (int)ceil(mcp.dataBusWidth / 8.0); // Support key words first operation
                                            // //8 means converting bit to Byte
  wb_ip.cache_sz = data * IO_buffer_size_per_channel; //*llcBlockSize;
  wb_ip.line_sz = data;
  wb_ip.assoc = 1;
  wb_ip.nbanks = 1;
  wb_ip.out_w = wb_ip.line_sz * 8;
  wb_ip.access_mode = 0;
  wb_ip.throughput = 1.0 / mcp.clockRate;
  wb_ip.latency = 1.0 / mcp.clockRate;
  wb_ip.obj_func_dyn_energy = 0;
  wb_ip.obj_func_dyn_power = 0;
  wb_ip.obj_func_leak_power = 0;
  wb_ip.obj_func_cycle_t = 1;
  wb_ip.num_rw_ports = 0;
  wb_ip.num_rd_ports = memory_channels_per_mc;
  wb_ip.num_wr_ports = wb_ip.num_rd_ports;
  wb_ip.num_se_rd_ports = 0;
  writeBuffer.set_params(&wb_ip, "MC writeBuffer", Uncore_device);
  init_params = true;
}

void MCFrontEnd::set_stats(const ParseXML *XML, const MCParam &mcp_) {
  memory_reads = XML->sys.mc.memory_reads;
  memory_writes = XML->sys.mc.memory_writes;
  mcp = mcp_;
  init_stats = true;
}

void MCFrontEnd::computeArea() {
  if (!init_params) {
    std::cerr << "[ MCFrontEnd ] Error: must set params before calling "
                 "computeArea()\n";
    exit(1);
  }
  // Front End Buffer Area Calculation
  frontendBuffer.computeArea();
  frontendBuffer.area.set_area(frontendBuffer.area.get_area() +
                               frontendBuffer.local_result.area *
                                   memory_channels_per_mc);
  area.set_area(area.get_area() +
                frontendBuffer.local_result.area * memory_channels_per_mc);

  // Read Buffer Area Calculation
  readBuffer.computeArea();
  readBuffer.area.set_area(readBuffer.area.get_area() +
                           readBuffer.local_result.area *
                               memory_channels_per_mc);
  area.set_area(area.get_area() +
                readBuffer.local_result.area * memory_channels_per_mc);

  // Write Buffer Area Calculation
  writeBuffer.computeArea();
  writeBuffer.area.set_area(writeBuffer.area.get_area() +
                            writeBuffer.local_result.area *
                                memory_channels_per_mc);
  area.set_area(area.get_area() +
                writeBuffer.local_result.area * memory_channels_per_mc);
}

void MCFrontEnd::computeStaticPower() {
  // NOTE: this does nothing, as the static power is optimized
  // along with the array area.
}

void MCFrontEnd::computeDynamicPower() {
  if (!init_stats) {
    std::cerr << "[ MCFrontEnd ] Error: must set params before calling "
                 "computeDynamicPower()\n";
    exit(1);
  }
  // stats for peak power (TDP)
  computeFrontEndTDP();
  computeReadBufferTDP();
  computeWriteBufferTDP();

  power = frontendBuffer.power_t + readBuffer.power_t + writeBuffer.power_t +
          (frontendBuffer.local_result.power + readBuffer.local_result.power +
           writeBuffer.local_result.power) *
              pppm_lkg;

  // stats for runtime power (RTP)
  computeFrontEndRTP();
  computeReadBufferRTP();
  computeWriteBufferRTP();

  rt_power = frontendBuffer.power_t + readBuffer.power_t + writeBuffer.power_t +
             (frontendBuffer.local_result.power +
              readBuffer.local_result.power + writeBuffer.local_result.power) *
                 pppm_lkg;
  rt_power.readOp.dynamic =
      rt_power.readOp.dynamic + power.readOp.dynamic * 0.1 * mcp.clockRate *
                                    mcp.num_mcs * mcp.executionTime;
}

void MCFrontEnd::computeFrontEndRTP() {
  frontendBuffer.stats_t.readAc.access = memory_reads * mcp.llcBlockSize * 8.0 /
                                         mcp.dataBusWidth * mcp.dataBusWidth /
                                         72;
  // For each channel, each memory word need to check the address data to
  // achieve best scheduling results. and this need to be done on all physical
  // DIMMs in each logical memory DIMM *mcp.dataBusWidth/72
  frontendBuffer.stats_t.writeAc.access = memory_writes * mcp.llcBlockSize *
                                          8.0 / mcp.dataBusWidth *
                                          mcp.dataBusWidth / 72;
  frontendBuffer.rtp_stats = frontendBuffer.stats_t;

  frontendBuffer.power_t.readOp.dynamic =
      (frontendBuffer.rtp_stats.readAc.access +
       frontendBuffer.rtp_stats.writeAc.access) *
          frontendBuffer.local_result.power.searchOp.dynamic +
      frontendBuffer.rtp_stats.readAc.access *
          frontendBuffer.local_result.power.readOp.dynamic +
      frontendBuffer.rtp_stats.writeAc.access *
          frontendBuffer.local_result.power.writeOp.dynamic;
}

void MCFrontEnd::computeReadBufferRTP() {
  readBuffer.stats_t.readAc.access = memory_reads * mcp.llcBlockSize * 8.0 /
                                     mcp.dataBusWidth; // support key word first
  readBuffer.stats_t.writeAc.access =
      memory_reads * mcp.llcBlockSize * 8.0 /
      mcp.dataBusWidth; // support key word first
  readBuffer.rtp_stats = readBuffer.stats_t;

  readBuffer.power_t.readOp.dynamic =
      (readBuffer.rtp_stats.readAc.access *
           readBuffer.local_result.power.readOp.dynamic +
       readBuffer.rtp_stats.writeAc.access *
           readBuffer.local_result.power.writeOp.dynamic);
}

void MCFrontEnd::computeWriteBufferRTP() {
  writeBuffer.stats_t.readAc.access =
      memory_writes * mcp.llcBlockSize * 8.0 / mcp.dataBusWidth;
  writeBuffer.stats_t.writeAc.access =
      memory_writes * mcp.llcBlockSize * 8.0 / mcp.dataBusWidth;
  writeBuffer.rtp_stats = writeBuffer.stats_t;

  writeBuffer.power_t.readOp.dynamic =
      (writeBuffer.rtp_stats.readAc.access *
           writeBuffer.local_result.power.readOp.dynamic +
       writeBuffer.rtp_stats.writeAc.access *
           writeBuffer.local_result.power.writeOp.dynamic);
}

void MCFrontEnd::computeFrontEndTDP() {
  frontendBuffer.stats_t.readAc.access = frontendBuffer.l_ip.num_search_ports;
  frontendBuffer.stats_t.writeAc.access = frontendBuffer.l_ip.num_wr_ports;
  frontendBuffer.tdp_stats = frontendBuffer.stats_t;

  frontendBuffer.power_t.readOp.dynamic =
      (frontendBuffer.tdp_stats.readAc.access +
       frontendBuffer.tdp_stats.writeAc.access) *
          frontendBuffer.local_result.power.searchOp.dynamic +
      frontendBuffer.tdp_stats.readAc.access *
          frontendBuffer.local_result.power.readOp.dynamic +
      frontendBuffer.tdp_stats.writeAc.access *
          frontendBuffer.local_result.power.writeOp.dynamic;
}

void MCFrontEnd::computeReadBufferTDP() {
  readBuffer.stats_t.readAc.access =
      readBuffer.l_ip.num_rd_ports * mcp.frontend_duty_cycle;
  readBuffer.stats_t.writeAc.access =
      readBuffer.l_ip.num_wr_ports * mcp.frontend_duty_cycle;
  readBuffer.tdp_stats = readBuffer.stats_t;

  readBuffer.power_t.readOp.dynamic =
      (readBuffer.tdp_stats.readAc.access *
           readBuffer.local_result.power.readOp.dynamic +
       readBuffer.tdp_stats.writeAc.access *
           readBuffer.local_result.power.writeOp.dynamic);
}

void MCFrontEnd::computeWriteBufferTDP() {
  writeBuffer.stats_t.readAc.access =
      writeBuffer.l_ip.num_rd_ports * mcp.frontend_duty_cycle;
  writeBuffer.stats_t.writeAc.access =
      writeBuffer.l_ip.num_wr_ports * mcp.frontend_duty_cycle;
  writeBuffer.tdp_stats = writeBuffer.stats_t;

  writeBuffer.power_t.readOp.dynamic =
      (writeBuffer.tdp_stats.readAc.access *
           writeBuffer.local_result.power.readOp.dynamic +
       writeBuffer.tdp_stats.writeAc.access *
           writeBuffer.local_result.power.writeOp.dynamic);
}

void MCFrontEnd::display(uint32_t indent, bool enable, bool detailed) {
  string indent_str(indent, ' ');
  string indent_str_next(indent + 2, ' ');

  if (enable) {
    std::cout << indent_str << "Front End Engine:" << std::endl;
    std::cout << indent_str_next << "Area = " << area.get_area() * 1e-6
              << " mm^2" << std::endl;
    std::cout << indent_str_next
              << "Peak Dynamic = " << power.readOp.dynamic * mcp.clockRate
              << " W" << std::endl;
    std::cout << indent_str_next << "Subthreshold Leakage = "
              << (long_channel ? power.readOp.longer_channel_leakage
                               : power.readOp.leakage)
              << " W" << std::endl;
    if (power_gating) {
      std::cout << indent_str_next
                << "Subthreshold Leakage with power gating = "
                << (long_channel
                        ? power.readOp.power_gated_with_long_channel_leakage
                        : power.readOp.power_gated_leakage)
                << " W" << std::endl;
    }
    std::cout << indent_str_next
              << "Gate Leakage = " << power.readOp.gate_leakage << " W"
              << std::endl;
    std::cout << indent_str_next << "Runtime Dynamic = "
              << rt_power.readOp.dynamic / mcp.executionTime << " W"
              << std::endl;
    std::cout << std::endl;
    if (detailed) {
      indent += 4;
      indent_str = std::string(indent, ' ');
      indent_str_next = std::string(indent + 2, ' ');
      std::cout << indent_str << "Front End ROB:" << std::endl;
      std::cout << indent_str_next
                << "Area = " << frontendBuffer.area.get_area() * 1e-6 << " mm^2"
                << std::endl;
      std::cout << indent_str_next << "Peak Dynamic = "
                << frontendBuffer.power.readOp.dynamic * mcp.clockRate << " W"
                << std::endl;
      std::cout << indent_str_next << "Subthreshold Leakage = "
                << frontendBuffer.power.readOp.leakage << " W" << std::endl;
      if (power_gating) {
        std::cout << indent_str_next
                  << "Subthreshold Leakage with power gating = "
                  << (long_channel
                          ? frontendBuffer.power.readOp
                                .power_gated_with_long_channel_leakage
                          : frontendBuffer.power.readOp.power_gated_leakage)
                  << " W" << std::endl;
      }
      std::cout << indent_str_next
                << "Gate Leakage = " << frontendBuffer.power.readOp.gate_leakage
                << " W" << std::endl;
      std::cout << indent_str_next << "Runtime Dynamic = "
                << frontendBuffer.rt_power.readOp.dynamic / mcp.executionTime
                << " W" << std::endl;

      std::cout << std::endl;
      std::cout << indent_str << "Read Buffer:" << std::endl;
      std::cout << indent_str_next
                << "Area = " << readBuffer.area.get_area() * 1e-6 << " mm^2"
                << std::endl;
      std::cout << indent_str_next << "Peak Dynamic = "
                << readBuffer.power.readOp.dynamic * mcp.clockRate << " W"
                << std::endl;
      std::cout << indent_str_next
                << "Subthreshold Leakage = " << readBuffer.power.readOp.leakage
                << " W" << std::endl;
      if (power_gating) {
        std::cout << indent_str_next
                  << "Subthreshold Leakage with power gating = "
                  << (long_channel
                          ? readBuffer.power.readOp
                                .power_gated_with_long_channel_leakage
                          : readBuffer.power.readOp.power_gated_leakage)
                  << " W" << std::endl;
      }
      std::cout << indent_str_next
                << "Gate Leakage = " << readBuffer.power.readOp.gate_leakage
                << " W" << std::endl;
      std::cout << indent_str_next << "Runtime Dynamic = "
                << readBuffer.rt_power.readOp.dynamic / mcp.executionTime
                << " W" << std::endl;
      std::cout << std::endl;
      std::cout << indent_str << "Write Buffer:" << std::endl;
      std::cout << indent_str_next
                << "Area = " << writeBuffer.area.get_area() * 1e-6 << " mm^2"
                << std::endl;
      std::cout << indent_str_next << "Peak Dynamic = "
                << writeBuffer.power.readOp.dynamic * mcp.clockRate << " W"
                << std::endl;
      std::cout << indent_str_next
                << "Subthreshold Leakage = " << writeBuffer.power.readOp.leakage
                << " W" << std::endl;
      if (power_gating) {
        std::cout << indent_str_next
                  << "Subthreshold Leakage with power gating = "
                  << (long_channel
                          ? writeBuffer.power.readOp
                                .power_gated_with_long_channel_leakage
                          : writeBuffer.power.readOp.power_gated_leakage)
                  << " W" << std::endl;
      }
      std::cout << indent_str_next
                << "Gate Leakage = " << writeBuffer.power.readOp.gate_leakage
                << " W" << std::endl;
      std::cout << indent_str_next << "Runtime Dynamic = "
                << writeBuffer.rt_power.readOp.dynamic / mcp.executionTime
                << " W" << std::endl;
      std::cout << std::endl;
    }
  }
}

MCFrontEnd ::~MCFrontEnd() {
  // Do Nothing
}
