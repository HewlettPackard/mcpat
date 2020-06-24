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

#include "undiff_core.h"

void UndiffCore::set_params(const ParseXML *XML_interface,
                            int ithCore_,
                            InputParameter *interface_ip_,
                            const CoreDynParam &dyn_p_,
                            bool exist_,
                            bool embedded_)
// is_default(_is_default)
{
  XML = XML_interface;
  ithCore = ithCore_;
  interface_ip = *interface_ip_;
  coredynp = dyn_p_;
  core_ty = coredynp.core_ty;
  embedded = XML->sys.Embedded;
  pipeline_stage = coredynp.pipeline_stages;
  num_hthreads = coredynp.num_hthreads;
  issue_width = coredynp.issueW;
  exist = exist_;
  if (!exist)
    return;
}

void UndiffCore::computeArea() {
  double undifferentiated_core = 0;
  double core_tx_density = 0;
  double pmos_to_nmos_sizing_r = pmos_to_nmos_sz_ratio();
  double undifferentiated_core_coe;
  // XML_interface=_XML_interface;
  uca_org_t result2;
  result2 = init_interface(&interface_ip);
  // Compute undifferentiated core area at 90nm.
  if (embedded == false) {
    // Based on the results of polynomial/log curve fitting based on
    // undifferentiated core of Niagara, Niagara2, Merom, Penyrn, Prescott,
    // Opteron die measurements
    if (core_ty == OOO) {
      // undifferentiated_core = (0.0764*pipeline_stage*pipeline_stage
      // -2.3685*pipeline_stage + 10.405);//OOO
      undifferentiated_core = (3.57 * log(pipeline_stage) - 1.2643) > 0
                                  ? (3.57 * log(pipeline_stage) - 1.2643)
                                  : 0;
    } else if (core_ty == Inorder) {
      // undifferentiated_core = (0.1238*pipeline_stage + 7.2572)*0.9;//inorder
      undifferentiated_core = (-2.19 * log(pipeline_stage) + 6.55) > 0
                                  ? (-2.19 * log(pipeline_stage) + 6.55)
                                  : 0;
    } else {
      std::cout << "invalid core type" << std::endl;
      exit(0);
    }
    undifferentiated_core *= (1 + logtwo(num_hthreads) * 0.0716);
  } else {
    // Based on the results in paper "parametrized processor models" Sandia Labs
    if (XML->sys.opt_clockrate)
      undifferentiated_core_coe = 0.05;
    else
      undifferentiated_core_coe = 0;
    undifferentiated_core =
        (0.4109 * pipeline_stage - 0.776) * undifferentiated_core_coe;
    undifferentiated_core *= (1 + logtwo(num_hthreads) * 0.0426);
  }

  undifferentiated_core *= g_tp.scaling_factor.logic_scaling_co_eff *
                           1e6; // change from mm^2 to um^2
  core_tx_density = g_tp.scaling_factor.core_tx_density;
  // undifferentiated_core 		    = 3*1e6;
  // undifferentiated_core			*=
  // g_tp.scaling_factor.logic_scaling_co_eff;//(g_ip->F_sz_um*g_ip->F_sz_um/0.09/0.09)*;
  power.readOp.leakage = undifferentiated_core *
                         (core_tx_density)*cmos_Isub_leakage(
                             5 * g_tp.min_w_nmos_,
                             5 * g_tp.min_w_nmos_ * pmos_to_nmos_sizing_r,
                             1,
                             inv) *
                         g_tp.peri_global.Vdd; // unit W
  power.readOp.gate_leakage = undifferentiated_core *
                              (core_tx_density)*cmos_Ig_leakage(
                                  5 * g_tp.min_w_nmos_,
                                  5 * g_tp.min_w_nmos_ * pmos_to_nmos_sizing_r,
                                  1,
                                  inv) *
                              g_tp.peri_global.Vdd;

  double long_channel_device_reduction =
      longer_channel_device_reduction(Core_device, coredynp.core_ty);
  power.readOp.longer_channel_leakage =
      power.readOp.leakage * long_channel_device_reduction;

  double pg_reduction = power_gating_leakage_reduction(false);
  power.readOp.power_gated_leakage = power.readOp.leakage * pg_reduction;
  power.readOp.power_gated_with_long_channel_leakage =
      power.readOp.power_gated_leakage * long_channel_device_reduction;

  area.set_area(undifferentiated_core);

  scktRatio = g_tp.sckt_co_eff;
  power.readOp.dynamic *= scktRatio;
  power.writeOp.dynamic *= scktRatio;
  power.searchOp.dynamic *= scktRatio;
  macro_PR_overhead = g_tp.macro_layout_overhead;
  area.set_area(area.get_area() * macro_PR_overhead);

  //		double vt=g_tp.peri_global.Vth;
  //		double velocity_index=1.1;
  //		double c_in=gate_C(g_tp.min_w_nmos_,
  // g_tp.min_w_nmos_*pmos_to_nmos_sizing_r , 0.0, false); 		double
  // c_out= drain_C_(g_tp.min_w_nmos_, NCH, 2, 1, g_tp.cell_h_def, false) +
  // drain_C_(g_tp.min_w_nmos_*pmos_to_nmos_sizing_r, PCH, 1, 1,
  // g_tp.cell_h_def, false) + c_in; 		double w_nmos=g_tp.min_w_nmos_;
  // double w_pmos=g_tp.min_w_nmos_*pmos_to_nmos_sizing_r; 		double
  // i_on_n=1.0; 		double
  // i_on_p=1.0; 		double i_on_n_in=1.0; 		double i_on_p_in=1;
  // double vdd=g_tp.peri_global.Vdd;

  //		power.readOp.sc=shortcircuit_simple(vt, velocity_index, c_in,
  // c_out, w_nmos,w_pmos, i_on_n, i_on_p,i_on_n_in, i_on_p_in, vdd);
  //		power.readOp.dynamic=c_out*vdd*vdd/2;

  //		std::cout<<power.readOp.dynamic << "dynamic" <<std::endl;
  //		std::cout<<power.readOp.sc << "sc" << std::endl;

  //		power.readOp.sc=shortcircuit(vt, velocity_index, c_in, c_out,
  // w_nmos,w_pmos, i_on_n, i_on_p,i_on_n_in, i_on_p_in, vdd);
  //		power.readOp.dynamic=c_out*vdd*vdd/2;
  //
  //		std::cout<<power.readOp.dynamic << "dynamic" <<std::endl;
  //		std::cout<<power.readOp.sc << "sc" << std::endl;
}

void UndiffCore::displayEnergy(uint32_t indent, int plevel, bool is_tdp) {
  string indent_str(indent, ' ');
  string indent_str_next(indent + 2, ' ');
  bool long_channel = XML->sys.longer_channel_device;
  bool power_gating = XML->sys.power_gating;

  if (is_tdp) {
    std::cout << indent_str << "UndiffCore:" << std::endl;
    std::cout << indent_str_next << "Area = " << area.get_area() * 1e-6
              << " mm^2" << std::endl;
    std::cout << indent_str_next
              << "Peak Dynamic = " << power.readOp.dynamic * clockRate << " W"
              << std::endl;
    // std::cout << indent_str_next << "Subthreshold Leakage = " <<
    // power.readOp.leakage <<" W" << std::endl;
    std::cout << indent_str_next << "Subthreshold Leakage = "
              << (long_channel ? power.readOp.longer_channel_leakage
                               : power.readOp.leakage)
              << " W" << std::endl;
    if (power_gating)
      std::cout << indent_str_next
                << "Subthreshold Leakage with power gating = "
                << (long_channel
                        ? power.readOp.power_gated_with_long_channel_leakage
                        : power.readOp.power_gated_leakage)
                << " W" << std::endl;
    std::cout << indent_str_next
              << "Gate Leakage = " << power.readOp.gate_leakage << " W"
              << std::endl;
    // std::cout << indent_str_next << "Runtime Dynamic = " <<
    // rt_power.readOp.dynamic/executionTime << " W" << std::endl;
    std::cout << std::endl;
  } else {
    std::cout << indent_str << "UndiffCore:" << std::endl;
    std::cout << indent_str_next << "Area = " << area.get_area() * 1e-6
              << " mm^2" << std::endl;
    std::cout << indent_str_next
              << "Peak Dynamic = " << power.readOp.dynamic * clockRate << " W"
              << std::endl;
    std::cout << indent_str_next
              << "Subthreshold Leakage = " << power.readOp.leakage << " W"
              << std::endl;
    std::cout << indent_str_next
              << "Gate Leakage = " << power.readOp.gate_leakage << " W"
              << std::endl;
    // std::cout << indent_str_next << "Runtime Dynamic = " <<
    // rt_power.readOp.dynamic/executionTime << " W" << std::endl;
    std::cout << std::endl;
  }
}
