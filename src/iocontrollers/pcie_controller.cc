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
#include "pcie_controller.h"

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
 * PCIeController()
 *    Constructor, Initializes the member variables that are shared across
 *    methods.
 */
PCIeController::PCIeController() {
  long_channel = false;
  power_gating = false;
  init_params = false;
  init_stats = false;
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
void PCIeController::computeArea() {
  double frontend_area = 0.0;
  double phy_area = 0.0;
  double ctrl_area = 0.0;
  double SerDer_area = 0.0;

  if (!init_params) {
    std::cerr << "[ PCIeController ] Error: must set params before calling "
                 "computeArea()\n";
    exit(1);
  }

  /* Assuming PCIe is bit-slice based architecture
   * This is the reason for /8 in both area and power calculation
   * to get per lane numbers
   */
  local_result = init_interface(&ip);
  if (pciep.type == 0) // high performance NIU
  {
    // Area estimation based on average of die photo from Niagara 2 and Cadence
    // ChipEstimate @ 65nm.
    ctrl_area = (5.2 + 0.5) / 2 * (ip.F_sz_um / 0.065) * (ip.F_sz_um / 0.065);
    // Area estimation based on average of die photo from Niagara 2, and Cadence
    // ChipEstimate @ 65nm.
    frontend_area =
        (5.2 + 0.1) / 2 * (ip.F_sz_um / 0.065) * (ip.F_sz_um / 0.065);
    // Area estimation based on average of die photo from Niagara 2 and Cadence
    // ChipEstimate hard IP @65nm. SerDer is very hard to scale
    SerDer_area = (3.03 + 0.36) * (ip.F_sz_um / 0.065); //* (ip.F_sz_um/0.065);
    phy_area = frontend_area + SerDer_area;
    // total area
  } else {
    ctrl_area = 0.412 * (ip.F_sz_um / 0.065) * (ip.F_sz_um / 0.065);
    // Area estimation based on average of die photo from Niagara 2, and Cadence
    // ChipEstimate @ 65nm.
    SerDer_area = 0.36 * (ip.F_sz_um / 0.065) * (ip.F_sz_um / 0.065);
    // total area
  }
  area.set_area(((ctrl_area + (pciep.withPHY ? SerDer_area : 0)) / 8 *
                 pciep.num_channels) *
                1e6);
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
void PCIeController::computeStaticPower() {
  double frontend_dyn = 0.0;
  double ctrl_dyn = 0.0;
  double SerDer_dyn = 0.0;
  double frontend_gates = 0.0;
  double ctrl_gates = 0.0;
  double SerDer_gates = 0.0;
  double NMOS_sizing = 0.0;
  double PMOS_sizing = 0.0;
  double pmos_to_nmos_sizing_r = pmos_to_nmos_sz_ratio();

  if (!init_params) {
    std::cerr << "[ PCIeController ] Error: must set params before calling "
                 "computeStaticPower()\n";
    exit(1);
  }
  if (pciep.type == 0) // high performance NIU
  {
    // Power
    // Cadence ChipEstimate using 65nm the controller includes everything: the
    // PHY, the data link and transaction layer
    ctrl_dyn = 3.75e-9 / 8 * g_tp.peri_global.Vdd / 1.1 * g_tp.peri_global.Vdd /
               1.1 * (ip.F_sz_nm / 65.0);
    //	  //Cadence ChipEstimate using 65nm soft IP;
    //	  frontend_dyn =
    // 0.27e-9/8*g_tp.peri_global.Vdd/1.1*g_tp.peri_global.Vdd/1.1*(ip.F_sz_nm/65.0);
    // SerDer_dyn is power not energy, scaling from 10mw/Gb/s @90nm
    SerDer_dyn = 0.01 * 4 * (ip.F_sz_um / 0.09) * g_tp.peri_global.Vdd / 1.2 *
                 g_tp.peri_global.Vdd /
                 1.2;              // PCIe 2.0 max per lane speed is 4Gb/s
    SerDer_dyn /= pciep.clockRate; // covert to energy per clock cycle

    // power_t.readOp.dynamic = (ctrl_dyn)*pciep.num_channels;
    // Cadence ChipEstimate using 65nm
    ctrl_gates = 900000 / 8 * pciep.num_channels;
    //	  frontend_gates   = 120000/8;
    //	  SerDer_gates     = 200000/8;
    NMOS_sizing = 5 * g_tp.min_w_nmos_;
    PMOS_sizing = 5 * g_tp.min_w_nmos_ * pmos_to_nmos_sizing_r;
  } else {
    // Power
    // Cadence ChipEstimate using 65nm the controller includes everything: the
    // PHY, the data link and transaction layer
    ctrl_dyn = 2.21e-9 / 8 * g_tp.peri_global.Vdd / 1.1 * g_tp.peri_global.Vdd /
               1.1 * (ip.F_sz_nm / 65.0);
    //	  //Cadence ChipEstimate using 65nm soft IP;
    //	  frontend_dyn =
    // 0.27e-9/8*g_tp.peri_global.Vdd/1.1*g_tp.peri_global.Vdd/1.1*(ip.F_sz_nm/65.0);
    // SerDer_dyn is power not energy, scaling from 10mw/Gb/s @90nm
    SerDer_dyn = 0.01 * 4 * (ip.F_sz_um / 0.09) * g_tp.peri_global.Vdd / 1.2 *
                 g_tp.peri_global.Vdd /
                 1.2;              // PCIe 2.0 max per lane speed is 4Gb/s
    SerDer_dyn /= pciep.clockRate; // covert to energy per clock cycle

    // Cadence ChipEstimate using 65nm
    ctrl_gates = 200000 / 8 * pciep.num_channels;
    //	  frontend_gates   = 120000/8;
    SerDer_gates = 200000 / 8 * pciep.num_channels;
    NMOS_sizing = g_tp.min_w_nmos_;
    PMOS_sizing = g_tp.min_w_nmos_ * pmos_to_nmos_sizing_r;
  }
  power_t.readOp.dynamic =
      (ctrl_dyn + (pciep.withPHY ? SerDer_dyn : 0)) * pciep.num_channels;
  power_t.readOp.leakage =
      (ctrl_gates + (pciep.withPHY ? SerDer_gates : 0)) *
      cmos_Isub_leakage(NMOS_sizing, PMOS_sizing, 2, nand) *
      g_tp.peri_global.Vdd; // unit W
  double long_channel_device_reduction =
      longer_channel_device_reduction(Uncore_device);
  double pg_reduction = power_gating_leakage_reduction(false);
  power_t.readOp.longer_channel_leakage =
      power_t.readOp.leakage * long_channel_device_reduction;
  power_t.readOp.power_gated_leakage = power_t.readOp.leakage * pg_reduction;
  power_t.readOp.power_gated_with_long_channel_leakage =
      power_t.readOp.power_gated_leakage * long_channel_device_reduction;
  power_t.readOp.gate_leakage =
      (ctrl_gates + (pciep.withPHY ? SerDer_gates : 0)) *
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
void PCIeController::computeDynamicPower() {
  power = power_t;
  power.readOp.dynamic *= pciep.duty_cycle;
  rt_power = power_t;
  rt_power.readOp.dynamic *= pciep.perc_load;
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
void PCIeController::display(uint32_t indent, bool enable) {
  string indent_str(indent, ' ');
  string indent_str_next(indent + 2, ' ');

  if (enable) {
    cout << "PCIe:" << endl;
    cout << indent_str << "Area = " << area.get_area() * 1e-6 << " mm^2"
         << endl;
    cout << indent_str
         << "Peak Dynamic = " << power.readOp.dynamic * pciep.clockRate << " W"
         << endl;
    cout << indent_str << "Subthreshold Leakage = "
         << (long_channel ? power.readOp.longer_channel_leakage
                          : power.readOp.leakage)
         << " W" << endl;
    if (power_gating)
      cout << indent_str << "Subthreshold Leakage with power gating = "
           << (long_channel ? power.readOp.power_gated_with_long_channel_leakage
                            : power.readOp.power_gated_leakage)
           << " W" << endl;
    cout << indent_str << "Gate Leakage = " << power.readOp.gate_leakage << " W"
         << endl;
    cout << indent_str
         << "Runtime Dynamic = " << rt_power.readOp.dynamic * pciep.clockRate
         << " W" << endl;
    cout << endl;
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
void PCIeController::set_params(const ParseXML *XML,
                                InputParameter *interface_ip) {
  ip = *interface_ip;
  pciep.clockRate = XML->sys.pcie.clockrate;
  pciep.clockRate *= 1e6;
  pciep.num_units = XML->sys.pcie.number_units;
  pciep.num_channels = XML->sys.pcie.num_channels;
  pciep.type = XML->sys.pcie.type;
  pciep.withPHY = XML->sys.pcie.withPHY;

  long_channel = XML->sys.longer_channel_device;
  power_gating = XML->sys.power_gating;

  if (XML->sys.pcie.vdd > 0) {
    ip.specific_hp_vdd = true;
    ip.specific_lop_vdd = true;
    ip.specific_lstp_vdd = true;
    ip.hp_Vdd = XML->sys.pcie.vdd;
    ip.lop_Vdd = XML->sys.pcie.vdd;
    ip.lstp_Vdd = XML->sys.pcie.vdd;
  }
  if (XML->sys.pcie.power_gating_vcc > -1) {
    ip.specific_vcc_min = true;
    ip.user_defined_vcc_min = XML->sys.pcie.power_gating_vcc;
  }
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
void PCIeController::set_stats(const ParseXML *XML) {
  pciep.duty_cycle = XML->sys.pcie.duty_cycle;
  pciep.perc_load = XML->sys.pcie.total_load_perc;
  init_stats = true;
}
