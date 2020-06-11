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

MCFrontEnd::MCFrontEnd(ParseXML *XML_interface,
                       InputParameter *interface_ip_,
                       const MCParam &mcp_,
                       enum MemoryCtrl_type mc_type_)
    : XML(XML_interface), interface_ip(*interface_ip_), mc_type(mc_type_),
      mcp(mcp_), MC_arb(0), frontendBuffer(0), readBuffer(0), writeBuffer(0) {
  /* All computations are for a single MC
   *
   */

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
  data = int(ceil((XML->sys.physical_address_width + mcp.opcodeW) / 8.0));
  interface_ip.cache_sz = data * XML->sys.mc.req_window_size_per_channel;
  interface_ip.line_sz = data;
  interface_ip.assoc = 0;
  interface_ip.nbanks = 1;
  interface_ip.out_w = interface_ip.line_sz * 8;
  interface_ip.specific_tag = 1;
  interface_ip.tag_w = tag;
  interface_ip.access_mode = 0;
  interface_ip.throughput = 1.0 / mcp.clockRate;
  interface_ip.latency = 1.0 / mcp.clockRate;
  interface_ip.is_cache = true;
  interface_ip.pure_cam = false;
  interface_ip.pure_ram = false;
  interface_ip.obj_func_dyn_energy = 0;
  interface_ip.obj_func_dyn_power = 0;
  interface_ip.obj_func_leak_power = 0;
  interface_ip.obj_func_cycle_t = 1;
  interface_ip.num_rw_ports = 0;
  interface_ip.num_rd_ports = XML->sys.mc.memory_channels_per_mc;
  interface_ip.num_wr_ports = interface_ip.num_rd_ports;
  interface_ip.num_se_rd_ports = 0;
  interface_ip.num_search_ports = XML->sys.mc.memory_channels_per_mc;
  frontendBuffer =
      new ArrayST(&interface_ip, "MC ReorderBuffer", Uncore_device);
  frontendBuffer->area.set_area(frontendBuffer->area.get_area() +
                                frontendBuffer->local_result.area *
                                    XML->sys.mc.memory_channels_per_mc);
  area.set_area(area.get_area() + frontendBuffer->local_result.area *
                                      XML->sys.mc.memory_channels_per_mc);

  // selection and arbitration logic
  interface_ip.assoc =
      1; // reset to prevent unnecessary warning messages when init_interface
  MC_arb = new selection_logic(is_default,
                               XML->sys.mc.req_window_size_per_channel,
                               1,
                               &interface_ip,
                               Uncore_device);

  // read buffers.
  data = (int)ceil(mcp.dataBusWidth / 8.0); // Support key words first operation
                                            // //8 means converting bit to Byte
  interface_ip.cache_sz =
      data * XML->sys.mc.IO_buffer_size_per_channel; //*llcBlockSize;
  interface_ip.line_sz = data;
  interface_ip.assoc = 1;
  interface_ip.nbanks = 1;
  interface_ip.out_w = interface_ip.line_sz * 8;
  interface_ip.access_mode = 1;
  interface_ip.throughput = 1.0 / mcp.clockRate;
  interface_ip.latency = 1.0 / mcp.clockRate;
  interface_ip.is_cache = false;
  interface_ip.pure_cam = false;
  interface_ip.pure_ram = true;
  interface_ip.obj_func_dyn_energy = 0;
  interface_ip.obj_func_dyn_power = 0;
  interface_ip.obj_func_leak_power = 0;
  interface_ip.obj_func_cycle_t = 1;
  interface_ip.num_rw_ports =
      0; // XML->sys.mc.memory_channels_per_mc*2>2?2:XML->sys.mc.memory_channels_per_mc*2;
  interface_ip.num_rd_ports = XML->sys.mc.memory_channels_per_mc;
  interface_ip.num_wr_ports = interface_ip.num_rd_ports;
  interface_ip.num_se_rd_ports = 0;
  readBuffer = new ArrayST(&interface_ip, "MC ReadBuffer", Uncore_device);
  readBuffer->area.set_area(readBuffer->area.get_area() +
                            readBuffer->local_result.area *
                                XML->sys.mc.memory_channels_per_mc);
  area.set_area(area.get_area() + readBuffer->local_result.area *
                                      XML->sys.mc.memory_channels_per_mc);

  // write buffer
  data = (int)ceil(mcp.dataBusWidth / 8.0); // Support key words first operation
                                            // //8 means converting bit to Byte
  interface_ip.cache_sz =
      data * XML->sys.mc.IO_buffer_size_per_channel; //*llcBlockSize;
  interface_ip.line_sz = data;
  interface_ip.assoc = 1;
  interface_ip.nbanks = 1;
  interface_ip.out_w = interface_ip.line_sz * 8;
  interface_ip.access_mode = 0;
  interface_ip.throughput = 1.0 / mcp.clockRate;
  interface_ip.latency = 1.0 / mcp.clockRate;
  interface_ip.obj_func_dyn_energy = 0;
  interface_ip.obj_func_dyn_power = 0;
  interface_ip.obj_func_leak_power = 0;
  interface_ip.obj_func_cycle_t = 1;
  interface_ip.num_rw_ports = 0;
  interface_ip.num_rd_ports = XML->sys.mc.memory_channels_per_mc;
  interface_ip.num_wr_ports = interface_ip.num_rd_ports;
  interface_ip.num_se_rd_ports = 0;
  writeBuffer = new ArrayST(&interface_ip, "MC writeBuffer", Uncore_device);
  writeBuffer->area.set_area(writeBuffer->area.get_area() +
                             writeBuffer->local_result.area *
                                 XML->sys.mc.memory_channels_per_mc);
  area.set_area(area.get_area() + writeBuffer->local_result.area *
                                      XML->sys.mc.memory_channels_per_mc);
}

void MCFrontEnd::computeEnergy(bool is_tdp) {
  if (is_tdp) {
    // init stats for Peak
    frontendBuffer->stats_t.readAc.access =
        frontendBuffer->l_ip.num_search_ports;
    frontendBuffer->stats_t.writeAc.access = frontendBuffer->l_ip.num_wr_ports;
    frontendBuffer->tdp_stats = frontendBuffer->stats_t;

    readBuffer->stats_t.readAc.access =
        readBuffer->l_ip.num_rd_ports * mcp.frontend_duty_cycle;
    readBuffer->stats_t.writeAc.access =
        readBuffer->l_ip.num_wr_ports * mcp.frontend_duty_cycle;
    readBuffer->tdp_stats = readBuffer->stats_t;

    writeBuffer->stats_t.readAc.access =
        writeBuffer->l_ip.num_rd_ports * mcp.frontend_duty_cycle;
    writeBuffer->stats_t.writeAc.access =
        writeBuffer->l_ip.num_wr_ports * mcp.frontend_duty_cycle;
    writeBuffer->tdp_stats = writeBuffer->stats_t;

  } else {
    // init stats for runtime power (RTP)
    frontendBuffer->stats_t.readAc.access =
        XML->sys.mc.memory_reads * mcp.llcBlockSize * 8.0 / mcp.dataBusWidth *
        mcp.dataBusWidth / 72;
    // For each channel, each memory word need to check the address data to
    // achieve best scheduling results. and this need to be done on all physical
    // DIMMs in each logical memory DIMM *mcp.dataBusWidth/72
    frontendBuffer->stats_t.writeAc.access =
        XML->sys.mc.memory_writes * mcp.llcBlockSize * 8.0 / mcp.dataBusWidth *
        mcp.dataBusWidth / 72;
    frontendBuffer->rtp_stats = frontendBuffer->stats_t;

    readBuffer->stats_t.readAc.access =
        XML->sys.mc.memory_reads * mcp.llcBlockSize * 8.0 /
        mcp.dataBusWidth; // support key word first
    readBuffer->stats_t.writeAc.access =
        XML->sys.mc.memory_reads * mcp.llcBlockSize * 8.0 /
        mcp.dataBusWidth; // support key word first
    readBuffer->rtp_stats = readBuffer->stats_t;

    writeBuffer->stats_t.readAc.access =
        XML->sys.mc.memory_writes * mcp.llcBlockSize * 8.0 / mcp.dataBusWidth;
    writeBuffer->stats_t.writeAc.access =
        XML->sys.mc.memory_writes * mcp.llcBlockSize * 8.0 / mcp.dataBusWidth;
    writeBuffer->rtp_stats = writeBuffer->stats_t;
  }

  frontendBuffer->power_t.reset();
  readBuffer->power_t.reset();
  writeBuffer->power_t.reset();

  //	frontendBuffer->power_t.readOp.dynamic	+=
  //(frontendBuffer->stats_t.readAc.access*
  //			(frontendBuffer->local_result.power.searchOp.dynamic+frontendBuffer->local_result.power.readOp.dynamic)+
  //    		frontendBuffer->stats_t.writeAc.access*frontendBuffer->local_result.power.writeOp.dynamic);

  frontendBuffer->power_t.readOp.dynamic +=
      (frontendBuffer->stats_t.readAc.access +
       frontendBuffer->stats_t.writeAc.access) *
          frontendBuffer->local_result.power.searchOp.dynamic +
      frontendBuffer->stats_t.readAc.access *
          frontendBuffer->local_result.power.readOp.dynamic +
      frontendBuffer->stats_t.writeAc.access *
          frontendBuffer->local_result.power.writeOp.dynamic;

  readBuffer->power_t.readOp.dynamic +=
      (readBuffer->stats_t.readAc.access *
           readBuffer->local_result.power.readOp.dynamic +
       readBuffer->stats_t.writeAc.access *
           readBuffer->local_result.power.writeOp.dynamic);
  writeBuffer->power_t.readOp.dynamic +=
      (writeBuffer->stats_t.readAc.access *
           writeBuffer->local_result.power.readOp.dynamic +
       writeBuffer->stats_t.writeAc.access *
           writeBuffer->local_result.power.writeOp.dynamic);

  if (is_tdp) {
    power = power + frontendBuffer->power_t + readBuffer->power_t +
            writeBuffer->power_t +
            (frontendBuffer->local_result.power +
             readBuffer->local_result.power + writeBuffer->local_result.power) *
                pppm_lkg;

  } else {
    rt_power =
        rt_power + frontendBuffer->power_t + readBuffer->power_t +
        writeBuffer->power_t +
        (frontendBuffer->local_result.power + readBuffer->local_result.power +
         writeBuffer->local_result.power) *
            pppm_lkg;
    rt_power.readOp.dynamic =
        rt_power.readOp.dynamic + power.readOp.dynamic * 0.1 * mcp.clockRate *
                                      mcp.num_mcs * mcp.executionTime;
  }
}

void MCFrontEnd::displayEnergy(uint32_t indent, int plevel, bool is_tdp) {
  string indent_str(indent, ' ');
  string indent_str_next(indent + 2, ' ');
  bool long_channel = XML->sys.longer_channel_device;
  bool power_gating = XML->sys.power_gating;

  if (is_tdp) {
    cout << indent_str << "Front End ROB:" << endl;
    cout << indent_str_next
         << "Area = " << frontendBuffer->area.get_area() * 1e-6 << " mm^2"
         << endl;
    cout << indent_str_next << "Peak Dynamic = "
         << frontendBuffer->power.readOp.dynamic * mcp.clockRate << " W"
         << endl;
    cout << indent_str_next
         << "Subthreshold Leakage = " << frontendBuffer->power.readOp.leakage
         << " W" << endl;
    if (power_gating)
      cout << indent_str_next << "Subthreshold Leakage with power gating = "
           << (long_channel ? frontendBuffer->power.readOp
                                  .power_gated_with_long_channel_leakage
                            : frontendBuffer->power.readOp.power_gated_leakage)
           << " W" << endl;
    cout << indent_str_next
         << "Gate Leakage = " << frontendBuffer->power.readOp.gate_leakage
         << " W" << endl;
    cout << indent_str_next << "Runtime Dynamic = "
         << frontendBuffer->rt_power.readOp.dynamic / mcp.executionTime << " W"
         << endl;

    cout << endl;
    cout << indent_str << "Read Buffer:" << endl;
    cout << indent_str_next << "Area = " << readBuffer->area.get_area() * 1e-6
         << " mm^2" << endl;
    cout << indent_str_next << "Peak Dynamic = "
         << readBuffer->power.readOp.dynamic * mcp.clockRate << " W" << endl;
    cout << indent_str_next
         << "Subthreshold Leakage = " << readBuffer->power.readOp.leakage
         << " W" << endl;
    if (power_gating)
      cout << indent_str_next << "Subthreshold Leakage with power gating = "
           << (long_channel ? readBuffer->power.readOp
                                  .power_gated_with_long_channel_leakage
                            : readBuffer->power.readOp.power_gated_leakage)
           << " W" << endl;
    cout << indent_str_next
         << "Gate Leakage = " << readBuffer->power.readOp.gate_leakage << " W"
         << endl;
    cout << indent_str_next << "Runtime Dynamic = "
         << readBuffer->rt_power.readOp.dynamic / mcp.executionTime << " W"
         << endl;
    cout << endl;
    cout << indent_str << "Write Buffer:" << endl;
    cout << indent_str_next << "Area = " << writeBuffer->area.get_area() * 1e-6
         << " mm^2" << endl;
    cout << indent_str_next << "Peak Dynamic = "
         << writeBuffer->power.readOp.dynamic * mcp.clockRate << " W" << endl;
    cout << indent_str_next
         << "Subthreshold Leakage = " << writeBuffer->power.readOp.leakage
         << " W" << endl;
    if (power_gating)
      cout << indent_str_next << "Subthreshold Leakage with power gating = "
           << (long_channel ? writeBuffer->power.readOp
                                  .power_gated_with_long_channel_leakage
                            : writeBuffer->power.readOp.power_gated_leakage)
           << " W" << endl;
    cout << indent_str_next
         << "Gate Leakage = " << writeBuffer->power.readOp.gate_leakage << " W"
         << endl;
    cout << indent_str_next << "Runtime Dynamic = "
         << writeBuffer->rt_power.readOp.dynamic / mcp.executionTime << " W"
         << endl;
    cout << endl;
  } else {
    cout << indent_str << "Front End ROB:" << endl;
    cout << indent_str_next
         << "Area = " << frontendBuffer->area.get_area() * 1e-6 << " mm^2"
         << endl;
    cout << indent_str_next << "Peak Dynamic = "
         << frontendBuffer->rt_power.readOp.dynamic * mcp.clockRate << " W"
         << endl;
    cout << indent_str_next
         << "Subthreshold Leakage = " << frontendBuffer->rt_power.readOp.leakage
         << " W" << endl;
    cout << indent_str_next
         << "Gate Leakage = " << frontendBuffer->rt_power.readOp.gate_leakage
         << " W" << endl;
    cout << endl;
    cout << indent_str << "Read Buffer:" << endl;
    cout << indent_str_next << "Area = " << readBuffer->area.get_area() * 1e-6
         << " mm^2" << endl;
    cout << indent_str_next << "Peak Dynamic = "
         << readBuffer->rt_power.readOp.dynamic * mcp.clockRate << " W" << endl;
    cout << indent_str_next
         << "Subthreshold Leakage = " << readBuffer->rt_power.readOp.leakage
         << " W" << endl;
    cout << indent_str_next
         << "Gate Leakage = " << readBuffer->rt_power.readOp.gate_leakage
         << " W" << endl;
    cout << endl;
    cout << indent_str << "Write Buffer:" << endl;
    cout << indent_str_next << "Area = " << writeBuffer->area.get_area() * 1e-6
         << " mm^2" << endl;
    cout << indent_str_next << "Peak Dynamic = "
         << writeBuffer->rt_power.readOp.dynamic * mcp.clockRate << " W"
         << endl;
    cout << indent_str_next
         << "Subthreshold Leakage = " << writeBuffer->rt_power.readOp.leakage
         << " W" << endl;
    cout << indent_str_next
         << "Gate Leakage = " << writeBuffer->rt_power.readOp.gate_leakage
         << " W" << endl;
  }
}

MCFrontEnd ::~MCFrontEnd() {

  if (MC_arb) {
    delete MC_arb;
    MC_arb = 0;
  }
  if (frontendBuffer) {
    delete frontendBuffer;
    frontendBuffer = 0;
  }
  if (readBuffer) {
    delete readBuffer;
    readBuffer = 0;
  }
  if (writeBuffer) {
    delete writeBuffer;
    writeBuffer = 0;
  }
}
