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
 * Author:
 *    Andrew Smith
 ***************************************************************************/

#include "flash_controller.h"

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

/*
 * FlashController()
 *    Constructor, Initializes the member variables that are shared across
 *    methods.
 */
FlashController::FlashController() {
  long_channel = false;
  power_gating = false;
  init_params = false;
  init_stats = false;

  number_channel = 0.0;
  // based On PCIe PHY TSMC65GP from Cadence ChipEstimate @ 65nm, it support
  // 8x lanes with each lane speed up to 250MB/s (PCIe1.1x) This is already
  // saturate the 200MB/s of the flash controller core above.
  ctrl_gates = 129267;
  SerDer_gates = 200000 / 8;

  NMOS_sizing = 0.0;
  PMOS_sizing = 0.0;
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
void FlashController::computeArea() {
  double ctrl_area = 0.0;
  double SerDer_area = 0.0;
  double pmos_to_nmos_sizing_r = pmos_to_nmos_sz_ratio();
  if (!init_params) {
    std::cerr << "[ FlashController ] Error: must set params before calling "
                 "computeArea()\n";
    exit(1);
  }
  /* Assuming PCIe is bit-slice based architecture
   * This is the reason for /8 in both area and power calculation
   * to get per lane numbers
   */
  local_result = init_interface(&ip);
  if (fcp.type == 0) // high performance NIU
  {
    std::cerr
        << "Current McPAT does not support high performance flash contorller "
           "since even low power designs are enough for maintain throughput"
        << endl;
    exit(1);
  }
  ctrl_area = 0.243 * (ip.F_sz_um / 0.065) * (ip.F_sz_um / 0.065);
  // Area estimation based on Cadence ChipEstimate @ 65nm: NANDFLASH-CTRL from
  // CAST
  SerDer_area = 0.36 / 8 * (ip.F_sz_um / 0.065) * (ip.F_sz_um / 0.065);
  area.set_area((ctrl_area + (fcp.withPHY ? SerDer_area : 0)) * 1e6 *
                number_channel);
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
void FlashController::computeStaticPower() {
  double ctrl_dyn = 0.0;
  double SerDer_dyn = 0.0;

  if (!init_params) {
    std::cerr << "[ FlashController ] Error: must set params before calling "
                 "computeStaticPower()\n";
    exit(1);
  }
  if (fcp.type == 0) // high performance NIU
  {
    std::cerr
        << "Current McPAT does not support high performance flash contorller "
           "since even low power designs are enough for maintain throughput"
        << endl;
    exit(1);
  }

  // Power
  // Cadence ChipEstimate using 65nm the controller 125mW for every 200MB/s
  // This is power not energy!
  ctrl_dyn = 0.125 * g_tp.peri_global.Vdd / 1.1 * g_tp.peri_global.Vdd / 1.1 *
             (ip.F_sz_nm / 65.0);
  // SerDer_dyn is power not energy, scaling from 10mw/Gb/s @90nm
  SerDer_dyn = 0.01 * 1.6 * (ip.F_sz_um / 0.09) * g_tp.peri_global.Vdd / 1.2 *
               g_tp.peri_global.Vdd / 1.2;
  // max  Per controller speed is 1.6Gb/s (200MB/s)

  power_t.readOp.dynamic =
      (ctrl_dyn + (fcp.withPHY ? SerDer_dyn : 0)) * number_channel;
  power_t.readOp.leakage =
      ((ctrl_gates + (fcp.withPHY ? SerDer_gates : 0)) * number_channel) *
      cmos_Isub_leakage(NMOS_sizing, PMOS_sizing, 2, nand) *
      g_tp.peri_global.Vdd; // unit W
  double long_channel_device_reduction =
      longer_channel_device_reduction(Uncore_device);
  power_t.readOp.longer_channel_leakage =
      power_t.readOp.leakage * long_channel_device_reduction;
  double pg_reduction = power_gating_leakage_reduction(
      false); // array structure all retain state;
  power_t.readOp.power_gated_leakage = power_t.readOp.leakage * pg_reduction;
  power_t.readOp.power_gated_with_long_channel_leakage =
      power_t.readOp.power_gated_leakage * long_channel_device_reduction;
  power_t.readOp.gate_leakage =
      ((ctrl_gates + (fcp.withPHY ? SerDer_gates : 0)) * number_channel) *
      cmos_Ig_leakage(NMOS_sizing, PMOS_sizing, 2, nand) *
      g_tp.peri_global.Vdd; // unit W
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
void FlashController::computeDynamicPower() {
  if (!init_stats) {
    std::cerr << "[ FlashController ] Error: must set stats before calling "
                 "computeDynamicPower()\n";
    exit(1);
  }
  // Peak Dynamic Power based on Duty Cycle
  power = power_t;
  power.readOp.dynamic *= fcp.duty_cycle;
  // Runtime Dynamic Power based on % Load
  rt_power = power_t;
  rt_power.readOp.dynamic *= fcp.perc_load;
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
void FlashController::display(uint32_t indent, bool enable) {
  string indent_str(indent, ' ');
  string indent_str_next(indent + 2, ' ');
  if (enable) {
    std::cout << "Flash Controller:" << std::endl;
    std::cout << indent_str << "Area = " << area.get_area() * 1e-6 << " mm^2"
              << std::endl;
    std::cout << indent_str << "Peak Dynamic = " << power.readOp.dynamic << " W"
              << std::endl; // no multiply of clock since this is power already
    std::cout << indent_str << "Subthreshold Leakage = "
              << (long_channel ? power.readOp.longer_channel_leakage
                               : power.readOp.leakage)
              << " W" << std::endl;
    if (power_gating)
      std::cout << indent_str << "Subthreshold Leakage with power gating = "
                << (long_channel
                        ? power.readOp.power_gated_with_long_channel_leakage
                        : power.readOp.power_gated_leakage)
                << " W" << std::endl;
    std::cout << indent_str << "Gate Leakage = " << power.readOp.gate_leakage
              << " W" << std::endl;
    std::cout << indent_str << "Runtime Dynamic = " << rt_power.readOp.dynamic
              << " W" << std::endl;
    std::cout << std::endl;
  } else {
  }
}

/*
 * set_params(const ParseXML, InputParameter)
 *    Sets the parts of the flash controller params that contribute to area and
 *    static power. Must be called before computing area or static power.
 *    Side Effects:
 *      sets the interface_ip struct, and sets the params struct to the
 *      "params" from the xml file. Also sets init_params to true.
 *    Input:
 *      *XML - Parsed XML
 *      *interface_ip - Interface from McPAT used in Cacti Library
 *    Output:
 *      None
 */
void FlashController::set_params(const ParseXML *XML,
                                 InputParameter *interface_ip) {
  ip = *interface_ip;
  //  fcp.clockRate = XML->sys.flashc.mc_clock;
  fcp.peakDataTransferRate = XML->sys.flashc.peak_transfer_rate;
  fcp.num_channels = ceil(fcp.peakDataTransferRate / 200);
  fcp.num_mcs = XML->sys.flashc.number_mcs;
  fcp.type = XML->sys.flashc.type;
  fcp.withPHY = XML->sys.flashc.withPHY;
  double pmos_to_nmos_sizing_r = pmos_to_nmos_sz_ratio();

  long_channel = XML->sys.longer_channel_device;
  power_gating = XML->sys.power_gating;

  if (XML->sys.flashc.vdd > 0) {
    ip.specific_hp_vdd = true;
    ip.specific_lop_vdd = true;
    ip.specific_lstp_vdd = true;
    ip.hp_Vdd = XML->sys.flashc.vdd;
    ip.lop_Vdd = XML->sys.flashc.vdd;
    ip.lstp_Vdd = XML->sys.flashc.vdd;
  }
  if (XML->sys.flashc.power_gating_vcc > -1) {
    ip.specific_vcc_min = true;
    ip.user_defined_vcc_min = XML->sys.flashc.power_gating_vcc;
  }
  NMOS_sizing = g_tp.min_w_nmos_;
  PMOS_sizing = g_tp.min_w_nmos_ * pmos_to_nmos_sizing_r;
  number_channel = 1 + (fcp.num_channels - 1) * 0.2;
  init_params = true;
}

/*
 * set_stats(const ParseXML)
 *    Sets the parts of the flash controller params that contribute to dynamic
 *    power.
 *    Side Effects:
 *      Store duty cycle and and percentage load into fc params, sets
 *      init_stats to true
 *    Input:
 *      *XML - Parsed XML
 *    Output:
 *      None
 */
void FlashController::set_stats(const ParseXML *XML) {
  fcp.duty_cycle = XML->sys.flashc.duty_cycle;
  fcp.perc_load = XML->sys.flashc.total_load_perc;
  init_stats = true;
}
