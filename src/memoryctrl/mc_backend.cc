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
#include "mc_backend.h"

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

/*
 * FlashController()
 *    Constructor, Initializes the member variables that are shared across
 *    methods.
 */
MCBackend::MCBackend() {
  long_channel = false;
  power_gating = false;
  mc_type = MC;
}

/*
 * computeArea()
 *    Computes the component area based off of the input parameters in the XML.
 *    Side Effects:
 *      Sets the component area member to the calculated area.
 *    Input:
 *      None
 *    Output:
 *      None
 */
void MCBackend::computeArea() {
  if (!init_params) {
    std::cerr << "[ MCBackend ] Error: must set params before calling "
                 "computeArea()\n";
    exit(1);
  }
  local_result = init_interface(&ip);
  if (mc_type == MC) {
    if (mcp.type == 0) {
      // area =
      // (2.2927*log(peakDataTransferRate)-14.504)*memDataWidth/144.0*(ip.F_sz_um/0.09);
      area.set_area((2.7927 * log(mcp.peakDataTransferRate * 2) - 19.862) /
                    2.0 * mcp.dataBusWidth / 128.0 * (ip.F_sz_um / 0.09) *
                    mcp.num_channels * 1e6); // um^2
    } else {
      area.set_area(0.15 * mcp.dataBusWidth / 72.0 * (ip.F_sz_um / 0.065) *
                    (ip.F_sz_um / 0.065) * mcp.num_channels * 1e6); // um^2
    }
  } else { // skip old model
    std::cerr << "[ MCBackend ] Error: Unknown memory controllers" << std::endl;
    exit(1);
  }
}

/*
 * computeStaticPower()
 *    Computes the static power based off of the input parameters from the xml.
 *    It calculates leakage power,
 *
 *    TODO: Add Vdd such that the static power & dynamic power can reflect
 *    changes in the chip power supply.
 *
 *    Side Effects:
 *      Sets the static power, leakage, and power gated leakage
 *    Input:
 *      None
 *    Output:
 *      None
 */
void MCBackend::computeStaticPower() {
  // double max_row_addr_width = 20.0;//Current address 12~18bits
  double C_MCB = 0.0;
  double mc_power = 0.0;
  double backend_dyn = 0.0;
  double backend_gates = 0.0;
  // double refresh_period = 0.0;
  // double refresh_freq = 0.0;
  double pmos_to_nmos_sizing_r = pmos_to_nmos_sz_ratio();
  double NMOS_sizing = 0.0;
  double PMOS_sizing = 0.0;
  if (!init_params) {
    std::cerr << "[ MCBackend ] Error: must set params before calling "
                 "computeStaticPower()\n";
    exit(1);
  }
  local_result = init_interface(&ip);
  if (mc_type == MC) {
    if (mcp.type == 0) {
      // assuming the approximately same scaling factor as seen in processors.
      // C_MCB=0.2/1.3/1.3/266/64/0.09*g_ip.F_sz_um;//based on AMD Geode
      // processor which has a very basic mc on chip. C_MCB
      // = 1.6/200/1e6/144/1.2/1.2*g_ip.F_sz_um/0.19;//Based on Niagara power
      // numbers.The base power (W) is divided by device frequency and vdd and
      // scale to target process. mc_power = 0.0291*2;//29.1mW@200MHz @130nm
      // From Power Analysis of SystemLevel OnChip Communication Architectures
      // by Lahiri et
      mc_power =
          4.32 *
          0.1; // 4.32W@1GhzMHz @65nm Cadence ChipEstimator 10% for backend
      C_MCB = mc_power / 1e9 / 72 / 1.1 / 1.1 * ip.F_sz_um / 0.065;
      power_t.readOp.dynamic =
          C_MCB * g_tp.peri_global.Vdd * g_tp.peri_global.Vdd *
          (mcp.dataBusWidth /*+mcp.addressBusWidth*/); // per access energy in
                                                       // memory controller
      power_t.readOp.leakage =
          area.get_area() / 2 * (g_tp.scaling_factor.core_tx_density) *
          cmos_Isub_leakage(g_tp.min_w_nmos_,
                            g_tp.min_w_nmos_ * pmos_to_nmos_sizing_r,
                            1,
                            inv) *
          g_tp.peri_global.Vdd; // unit W
      power_t.readOp.gate_leakage =
          area.get_area() / 2 * (g_tp.scaling_factor.core_tx_density) *
          cmos_Ig_leakage(g_tp.min_w_nmos_,
                          g_tp.min_w_nmos_ * pmos_to_nmos_sizing_r,
                          1,
                          inv) *
          g_tp.peri_global.Vdd; // unit W

    } else {
      NMOS_sizing = g_tp.min_w_nmos_;
      PMOS_sizing = g_tp.min_w_nmos_ * pmos_to_nmos_sizing_r;
      backend_dyn =
          0.9e-9 / 800e6 * mcp.clockRate / 12800 * mcp.peakDataTransferRate *
          mcp.dataBusWidth / 72.0 * g_tp.peri_global.Vdd / 1.1 *
          g_tp.peri_global.Vdd / 1.1 *
          (ip.F_sz_nm / 65.0); // Average on DDR2/3 protocol controller and
                               // DDRC 1600/800A in Cadence ChipEstimate
      // Scaling to technology and DIMM feature. The base IP support
      // DDR3-1600(PC3 12800)
      backend_gates = 50000 * mcp.dataBusWidth /
                      64.0; // 50000 is from Cadence ChipEstimator

      power_t.readOp.dynamic = backend_dyn;
      power_t.readOp.leakage =
          (backend_gates)*cmos_Isub_leakage(NMOS_sizing, PMOS_sizing, 2, nand) *
          g_tp.peri_global.Vdd; // unit W
      power_t.readOp.gate_leakage =
          (backend_gates)*cmos_Ig_leakage(NMOS_sizing, PMOS_sizing, 2, nand) *
          g_tp.peri_global.Vdd; // unit W
    }
  } else { // skip old model
    std::cerr << "[ MCBackend ] Error: Unknown memory controllers" << std::endl;
    exit(1);
  }
  double long_channel_device_reduction =
      longer_channel_device_reduction(Uncore_device);
  power_t.readOp.longer_channel_leakage =
      power_t.readOp.leakage * long_channel_device_reduction;

  double pg_reduction = power_gating_leakage_reduction(false);
  power_t.readOp.power_gated_leakage = power_t.readOp.leakage * pg_reduction;
  power_t.readOp.power_gated_with_long_channel_leakage =
      power_t.readOp.power_gated_leakage * long_channel_device_reduction;
}

/*
 * computeDynamicPower()
 *    Compute both Peak Power and Runtime Power based on the stats of the input
 *    xml.
 *    Side Effects:
 *      Sets the runtime power, and the peak dynamic power in the component
 *      class
 *    Input:
 *      None
 *    Output:
 *      None
 */
void MCBackend::computeDynamicPower() {
  if (!init_stats) {
    std::cerr << "[ MCBackend ] Error: must set stats before calling "
                 "computeDynamicPower()\n";
    exit(1);
  }
  // backend uses internal data buswidth
  stats_t.readAc.access = 0.5 * mcp.num_channels;
  stats_t.writeAc.access = 0.5 * mcp.num_channels;
  tdp_stats = stats_t;

  stats_t.readAc.access = mcp.reads;
  stats_t.writeAc.access = mcp.writes;
  rtp_stats = stats_t;
  power = power_t;
  power.readOp.dynamic = (tdp_stats.readAc.access + tdp_stats.writeAc.access) *
                         power_t.readOp.dynamic;

  rt_power.readOp.dynamic =
      (rtp_stats.readAc.access + rtp_stats.writeAc.access) * mcp.llcBlockSize *
      8.0 / mcp.dataBusWidth * power_t.readOp.dynamic;
  rt_power = rt_power + power_t * pppm_lkg;
  rt_power.readOp.dynamic =
      rt_power.readOp.dynamic + power.readOp.dynamic * 0.1 * mcp.clockRate *
                                    mcp.num_mcs * mcp.executionTime;
  // Assume 10% of peak power is consumed by routine job including memory
  // refreshing and scrubbing
}

/*
 * set_params(const ParseXML,
 *            const MCParam&,
 *            InputParameter,
 *            const enum MemoryCtrl_type)
 *    Sets the parts of the flash controller params that contribute to area and
 *    static power. Must be called before computing area or static power.
 *    Side Effects:
 *      sets the interface_ip struct, and sets the params struct to the
 *      "params" from the xml file. Also sets init_params to true.
 *    Input:
 *      *XML - Parsed XML
 *      &MCParam - Parsed memory controller object from parent
 *      *interface_ip - Interface from McPAT used in Cacti Library
 *      MemoryCtrl_type - enum for type of memory controller
 *    Output:
 *      None
 */
void MCBackend::set_params(const ParseXML *XML,
                           const MCParam &mcp_,
                           InputParameter *interface_ip,
                           const enum MemoryCtrl_type mc_type_) {
  long_channel = XML->sys.longer_channel_device;
  power_gating = XML->sys.power_gating;
  mcp = mcp_;
  ip = *interface_ip;
  mc_type = mc_type_;
  init_params = true;
}

/*
 * set_stats(const MCParam&)
 *    Sets the parts of the flash controller params that contribute to dynamic
 *    power.
 *    Side Effects:
 *      Store duty cycle and and percentage load into fc params, sets
 *      init_stats to true
 *    Input:
 *      MCParam - Parent Parsed MCParam Object
 *    Output:
 *      None
 */
void MCBackend::set_stats(const MCParam &mcp_) {
  mcp = mcp_;
  init_stats = true;
}

/*
 * display(uint32_t, bool)
 *    Display the Power, Area, and Timing results to the standard output
 *    Side Effects:
 *      None
 *    Input:
 *      indent - How far in to indent
 *      enable - toggle printing
 *    Output:
 *      None
 */
void MCBackend::display(uint32_t indent, bool enable) {
  std::string indent_str(indent, ' ');
  std::string indent_str_next(indent + 2, ' ');

  if (enable) {
    std::cout << indent_str << "Transaction Engine:" << std::endl;
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
  }
}
