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

#include "mc_phy.h"

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
 * MCPHY()
 *    Constructor, Initializes the member variables that are shared across
 *    methods.
 */
MCPHY::MCPHY() {
  long_channel = false;
  power_gating = false;
  init_params = false;
  init_stats = false;
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
void MCPHY::computeArea() {
  if (!init_params) {
    std::cerr << "[ MCPHY ] Error: must set params before calling "
                 "computeArea()\n";
    exit(1);
  }
  local_result = init_interface(&ip);
  if (mc_type == MC) {
    if (mcp.type == 0) {
      // Based on die photos from Niagara 1 and 2.
      // TODO merge this into undifferentiated core.PHY only achieves square
      // root of the ideal scaling. area =
      // (6.4323*log(peakDataTransferRate)-34.76)*memDataWidth/128.0*(ip.F_sz_um/0.09);
      area.set_area((6.4323 * log(mcp.peakDataTransferRate * 2) - 48.134) *
                    mcp.dataBusWidth / 128.0 * (ip.F_sz_um / 0.09) *
                    mcp.num_channels * 1e6 / 2); // TODO:/2
    } else {
      double non_IO_percentage = 0.2;
      area.set_area(1.3 * non_IO_percentage / 2133.0e6 * mcp.clockRate / 17066 *
                    mcp.peakDataTransferRate * mcp.dataBusWidth / 16.0 *
                    (ip.F_sz_um / 0.040) * (ip.F_sz_um / 0.040) *
                    mcp.num_channels * 1e6); // um^2
    }
  } else {
    area.set_area(0.4e6 / 2 * mcp.dataBusWidth /
                  8); // area based on Cadence ChipEstimator for 8bit bus
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
void MCPHY::computeStaticPower() {
  // PHY uses internal data buswidth but the actuall off-chip datawidth is
  // 64bits + ecc
  double pmos_to_nmos_sizing_r = pmos_to_nmos_sz_ratio();
  /*
   * according to "A 100mW 9.6Gb/s Transceiver in 90nm CMOS for next-generation
   * memory interfaces ," ISSCC 2006; From Cadence ChipEstimator for normal I/O
   * around 0.4~0.8 mW/Gb/s
   */
  double power_per_gb_per_s = 0.0;
  double phy_gates = 0.0;
  double NMOS_sizing = 0.0;
  double PMOS_sizing = 0.0;

  if (!init_params) {
    std::cerr << "[ MCPHY ] Error: must set params before calling "
                 "computeStaticPower()\n";
    exit(1);
  }

  if (mcp.type == 0 && mc_type == MC) {
    power_per_gb_per_s = mcp.LVDS ? 0.01 : 0.04;
    // This is from curve fitting based on Niagara 1 and 2's PHY die photo.
    // This is power not energy, 10mw/Gb/s @90nm for each channel and scaling
    // down power.readOp.dynamic = 0.02*memAccesses*llcBlocksize*8;//change
    // from Bytes to bits.
    power_t.readOp.dynamic = power_per_gb_per_s * sqrt(ip.F_sz_um / 0.09) *
                             g_tp.peri_global.Vdd / 1.2 * g_tp.peri_global.Vdd /
                             1.2;
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
    // Designware/synopsis 16bit DDR3 PHY is 1.3mm (WITH IOs) at 40nm for upto
    // DDR3 2133 (PC3 17066)
    phy_gates = 200000 * mcp.dataBusWidth / 64.0;
    power_per_gb_per_s = 0.01;
    // This is power not energy, 10mw/Gb/s @90nm for each channel and scaling
    // down
    power_t.readOp.dynamic = power_per_gb_per_s * (ip.F_sz_um / 0.09) *
                             g_tp.peri_global.Vdd / 1.2 * g_tp.peri_global.Vdd /
                             1.2;
    power_t.readOp.leakage =
        (mcp.withPHY ? phy_gates : 0) *
        cmos_Isub_leakage(NMOS_sizing, PMOS_sizing, 2, nand) *
        g_tp.peri_global.Vdd; // unit W
    power_t.readOp.gate_leakage =
        (mcp.withPHY ? phy_gates : 0) *
        cmos_Ig_leakage(NMOS_sizing, PMOS_sizing, 2, nand) *
        g_tp.peri_global.Vdd; // unit W
  }
  //  double phy_factor = (int)ceil(mcp.dataBusWidth/72.0);//Previous phy power
  //  numbers are based on 72 bit DIMM interface power_t.readOp.dynamic *=
  //  phy_factor; power_t.readOp.leakage *= phy_factor;
  //  power_t.readOp.gate_leakage *= phy_factor;

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
void MCPHY::computeDynamicPower() {
  if (!init_stats) {
    std::cerr << "[ MCPHY ] Error: must set stats before calling "
                 "computeDynamicPower()\n";
    exit(1);
  }
  // init stats for Peak
  stats_t.readAc.access = 0.5 * mcp.num_channels; // time share on buses
  stats_t.writeAc.access = 0.5 * mcp.num_channels;
  tdp_stats = stats_t;

  // init stats for runtime power (RTP)
  stats_t.readAc.access = mcp.reads;
  stats_t.writeAc.access = mcp.writes;
  rtp_stats = stats_t;
  double data_transfer_unit = (mc_type == MC) ? 72 : 16; /*DIMM data width*/
  power = power_t;
  power.readOp.dynamic =
      power.readOp.dynamic *
      (mcp.peakDataTransferRate * 8 * 1e6 / 1e9 /*change to Gbs*/) *
      mcp.dataBusWidth / data_transfer_unit * mcp.num_channels / mcp.clockRate;
  // divide by clock rate is for match the final computation where *clock is
  // used
  //(tdp_stats.readAc.access*power_t.readOp.dynamic+
  //					tdp_stats.writeAc.access*power_t.readOp.dynamic);

  rt_power = power_t;
  //    	rt_power.readOp.dynamic	=
  //    (rtp_stats.readAc.access*power_t.readOp.dynamic+
  //    						rtp_stats.writeAc.access*power_t.readOp.dynamic);

  rt_power.readOp.dynamic =
      power_t.readOp.dynamic *
      (rtp_stats.readAc.access + rtp_stats.writeAc.access) *
      (mcp.llcBlockSize) * 8 / 1e9 / mcp.executionTime * (mcp.executionTime);
  rt_power.readOp.dynamic =
      rt_power.readOp.dynamic + power.readOp.dynamic * 0.1 * mcp.clockRate *
                                    mcp.num_mcs * mcp.executionTime;
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
void MCPHY::set_params(const ParseXML *XML,
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
void MCPHY::set_stats(const MCParam &mcp_) {
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
void MCPHY::display(uint32_t indent, bool enable) {
  std::string indent_str(indent, ' ');
  std::string indent_str_next(indent + 2, ' ');

  if (enable) {
    std::cout << indent_str << "PHY:" << std::endl;
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
