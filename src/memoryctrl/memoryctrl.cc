/*****************************************************************************
 *                                McPAT *                      SOFTWARE LICENSE
 AGREEMENT
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
#include "memoryctrl.h"

#include "XML_Parse.h"
#include "basic_circuit.h"
#include "basic_components.h"
#include "const.h"
#include "io.h"
#include "parameter.h"

#include <algorithm>
#include <assert.h>
#include <cmath>
#include <iostream>
#include <string>

/* overview of MC models:
 * McPAT memory controllers are modeled according to large number of industrial
 * data points. The Basic memory controller architecture is base on the Synopsis
 * designs (DesignWare DDR2/DDR3-Lite memory controllers and DDR2/DDR3-Lite
 * protocol controllers) as in Cadence ChipEstimator Tool
 *
 * An MC has 3 parts as shown in this design. McPAT models both high performance
 * MC based on Niagara processor designs and curving and low power MC based on
 * data points in Cadence ChipEstimator Tool.
 *
 * The frontend is modeled analytically, the backend is modeled empirically
 * according to DDR2/DDR3-Lite protocol controllers in Cadence ChipEstimator
 * Tool The PHY is modeled based on "A 100mW 9.6Gb/s Transceiver in 90nm CMOS
 * for next-generation memory interfaces ," ISSCC 2006, and A 14mW 6.25Gb/s
 * Transceiver in 90nm CMOS for Serial Chip-to-Chip Communication," ISSCC 2007
 *
 * In Cadence ChipEstimator Tool there are two types of memory controllers: the
 * full memory controllers that includes the frontend as the DesignWare
 * DDR2/DDR3-Lite memory controllers and the backend only memory controllers as
 * the DDR2/DDR3-Lite protocol controllers (except DesignWare DDR2/DDR3-Lite
 * memory controllers, all memory controller IP in Cadence ChipEstimator Tool
 * are backend memory controllers such as DDRC 1600A and DDRC 800A). Thus,to
 * some extend the area and power difference between DesignWare DDR2/DDR3-Lite
 * memory controllers and DDR2/DDR3-Lite protocol controllers can be an
 * estimation to the frontend power and area, which is very close the
 * analitically modeled results of the frontend for Niagara2@65nm
 *
 */

MemoryController::MemoryController() {
  long_channel = false;
  power_gating = false;
  init_params = false;
  init_stats = false;
  set_area = false;
  mc_type = MC;
}

void MemoryController::set_params(const ParseXML *XML,
                                  InputParameter *interface_ip_,
                                  enum MemoryCtrl_type mc_type_) {
  long_channel = XML->sys.longer_channel_device;
  power_gating = XML->sys.power_gating;
  interface_ip = *interface_ip_;
  mc_type = mc_type_;

  interface_ip.wire_is_mat_type = 2;
  interface_ip.wire_os_mat_type = 2;
  interface_ip.wt = Global;

  set_mc_param(XML);

  transecEngine.set_params(XML, mcp, &interface_ip, mc_type);
  frontend.set_params(XML, &interface_ip, mcp, mc_type);
  if (mcp.type == 0 || (mcp.type == 1 && mcp.withPHY)) {
    PHY.set_params(XML, mcp, &interface_ip, mc_type);
  }
  init_params = true;
}

void MemoryController::set_stats(const ParseXML *XML) {
  set_mc_param(XML);
  transecEngine.set_stats(mcp);
  frontend.set_stats(XML, mcp);
  if (mcp.type == 0 || (mcp.type == 1 && mcp.withPHY)) {
    PHY.set_stats(mcp);
  }
  init_stats = true;
}

void MemoryController::computeArea() {
  if (!init_params) {
    std::cerr << "[ MemoryController ] Error: must set params before calling "
                 "computeArea()\n";
    exit(1);
  }
  transecEngine.computeArea();
  frontend.computeArea();
  area.set_area(area.get_area() + frontend.area.get_area());
  area.set_area(area.get_area() + transecEngine.area.get_area());
  if (mcp.type == 0 || (mcp.type == 1 && mcp.withPHY)) {
    PHY.computeArea();
    area.set_area(area.get_area() + PHY.area.get_area());
  }
  set_area = true;
}

void MemoryController::computeStaticPower() {
  if (!init_params) {
    std::cerr << "[ MemoryController ] Error: must set params before calling "
                 "computeStaticPower()\n";
    exit(1);
  }
  if (!set_area) {
    std::cerr << "[ MemoryController ] Error: must computeArea before calling "
                 "computeStaticPower()\n";
    exit(1);
  }
  transecEngine.computeStaticPower();
  frontend.computeStaticPower();
  if (mcp.type == 0 || (mcp.type == 1 && mcp.withPHY)) {
    PHY.computeStaticPower();
  }
}

void MemoryController::computeDynamicPower() {
  if (!init_stats) {
    std::cerr << "[ MemoryController ] Error: must set stats before calling "
                 "computeDynamicPower()\n";
    exit(1);
  }
  frontend.computeDynamicPower();
  transecEngine.computeDynamicPower();

  if (mcp.type == 0 || (mcp.type == 1 && mcp.withPHY)) {
    PHY.computeDynamicPower();
  }

  power = frontend.power + transecEngine.power;
  if (mcp.type == 0 || (mcp.type == 1 && mcp.withPHY)) {
    power = power + PHY.power;
  }
  rt_power = frontend.rt_power + transecEngine.rt_power;
  if (mcp.type == 0 || (mcp.type == 1 && mcp.withPHY)) {
    rt_power = rt_power + PHY.rt_power;
  }
}

void MemoryController::display(uint32_t indent, bool enable) {
  string indent_str(indent, ' ');
  string indent_str_next(indent + 2, ' ');
  if (enable) {
    std::cout << "Memory Controller:" << std::endl;
    std::cout << indent_str << "Area = " << area.get_area() * 1e-6 << " mm^2"
              << std::endl;
    std::cout << indent_str
              << "Peak Dynamic = " << power.readOp.dynamic * mcp.clockRate
              << " W" << std::endl;
    std::cout << indent_str << "Subthreshold Leakage = "
              << (long_channel ? power.readOp.longer_channel_leakage
                               : power.readOp.leakage)
              << " W" << std::endl;
    if (power_gating) {
      std::cout << indent_str << "Subthreshold Leakage with power gating = "
                << (long_channel
                        ? power.readOp.power_gated_with_long_channel_leakage
                        : power.readOp.power_gated_leakage)
                << " W" << std::endl;
    }
    std::cout << indent_str << "Gate Leakage = " << power.readOp.gate_leakage
              << " W" << std::endl;
    std::cout << indent_str << "Runtime Dynamic = "
              << rt_power.readOp.dynamic / mcp.executionTime << " W"
              << std::endl;
    std::cout << std::endl;

    frontend.display(indent, true);
    transecEngine.display(indent, true);
    if (mcp.type == 0 || (mcp.type == 1 && mcp.withPHY)) {
      PHY.display(indent, true);
    }
  }
}

void MemoryController::set_mc_param(const ParseXML *XML) {
  if (mc_type == MC) {
    mcp.clockRate = XML->sys.mc.mc_clock * 2; // DDR double pumped
    mcp.clockRate *= 1e6;
    mcp.executionTime =
        XML->sys.total_cycles / (XML->sys.target_core_clockrate * 1e6);

    mcp.llcBlockSize = int(ceil(XML->sys.mc.llc_line_length / 8.0)) +
                       XML->sys.mc.llc_line_length; // ecc overhead
    mcp.dataBusWidth =
        int(ceil(XML->sys.mc.databus_width / 8.0)) + XML->sys.mc.databus_width;
    mcp.addressBusWidth = int(
        ceil(XML->sys.mc.addressbus_width)); // XML->sys.physical_address_width;
    mcp.opcodeW = 16;
    mcp.num_mcs = XML->sys.mc.number_mcs;
    mcp.num_channels = XML->sys.mc.memory_channels_per_mc;
    mcp.reads = XML->sys.mc.memory_reads;
    mcp.writes = XML->sys.mc.memory_writes;
    //+++++++++Transaction engine +++++++++++++++++ ////TODO needs better
    // numbers, Run the RTL code from OpenSparc.
    mcp.peakDataTransferRate = XML->sys.mc.peak_transfer_rate;
    mcp.memRank = XML->sys.mc.number_ranks;
    //++++++++++++++PHY ++++++++++++++++++++++++++ //TODO needs better numbers
    // PHY.memAccesses=PHY.peakDataTransferRate;//this is the max power
    // PHY.llcBlocksize=llcBlockSize;
    mcp.frontend_duty_cycle = 0.5; // for max power, the actual off-chip links
                                   // is bidirectional but time shared
    mcp.LVDS = XML->sys.mc.LVDS;
    mcp.type = XML->sys.mc.type;
    mcp.withPHY = XML->sys.mc.withPHY;

    if (XML->sys.mc.vdd > 0) {
      interface_ip.specific_hp_vdd = true;
      interface_ip.specific_lop_vdd = true;
      interface_ip.specific_lstp_vdd = true;
      interface_ip.hp_Vdd = XML->sys.mc.vdd;
      interface_ip.lop_Vdd = XML->sys.mc.vdd;
      interface_ip.lstp_Vdd = XML->sys.mc.vdd;
    }
    if (XML->sys.mc.power_gating_vcc > -1) {
      interface_ip.specific_vcc_min = true;
      interface_ip.user_defined_vcc_min = XML->sys.mc.power_gating_vcc;
    }
  }
  //	else if (mc_type==FLASHC)
  //	{
  //		mcp.clockRate       =XML->sys.flashc.mc_clock*2;//DDR double
  // pumped 		mcp.clockRate       *= 1e6; 		mcp.executionTime
  // = XML->sys.total_cycles/(XML->sys.target_core_clockrate*1e6);
  //
  //		mcp.llcBlockSize
  //=int(ceil(XML->sys.flashc.llc_line_length/8.0))+XML->sys.flashc.llc_line_length;//ecc
  // overhead 		mcp.dataBusWidth
  // =int(ceil(XML->sys.flashc.databus_width/8.0)) +
  // XML->sys.flashc.databus_width; 		mcp.addressBusWidth
  //=int(ceil(XML->sys.flashc.addressbus_width));//XML->sys.physical_address_width;
  //		mcp.opcodeW         =16;
  //		mcp.num_mcs         = XML->sys.flashc.number_mcs;
  //		mcp.num_channels    = XML->sys.flashc.memory_channels_per_mc;
  //		mcp.reads  = XML->sys.flashc.memory_reads;
  //		mcp.writes = XML->sys.flashc.memory_writes;
  //		//+++++++++Transaction engine +++++++++++++++++ ////TODO needs
  // better numbers, Run the RTL code from OpenSparc.
  // mcp.peakDataTransferRate =
  // XML->sys.flashc.peak_transfer_rate; 		mcp.memRank =
  // XML->sys.flashc.number_ranks;
  //		//++++++++++++++PHY ++++++++++++++++++++++++++ //TODO needs
  // better numbers
  //		//PHY.memAccesses=PHY.peakDataTransferRate;//this is the max
  // power
  //		//PHY.llcBlocksize=llcBlockSize;
  //		mcp.frontend_duty_cycle = 0.5;//for max power, the actual
  // off-chip links is bidirectional but time shared 		mcp.LVDS =
  // XML->sys.flashc.LVDS; 		mcp.type = XML->sys.flashc.type;
  //	}
  else {
    std::cerr << "[ MemoryController ] Unknown memory controller type: neither "
                 "DRAM controller nor Flash "
                 "controller"
              << std::endl;
    exit(1);
  }
}

MemoryController ::~MemoryController() {
  // Do Nothing
}
