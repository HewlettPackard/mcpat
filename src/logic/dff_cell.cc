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

#include "dff_cell.h"

// TODO: add inverter and transmission gate base DFF.
DFFCell::DFFCell(bool _is_dram,
                 double _WdecNANDn,
                 double _WdecNANDp,
                 double _cell_load,
                 const InputParameter *configure_interface)
    : is_dram(_is_dram), cell_load(_cell_load), WdecNANDn(_WdecNANDn),
      WdecNANDp(_WdecNANDp) { // this model is based on the NAND2 based DFF.
  l_ip = *configure_interface;
  //			area.set_area(730*l_ip.F_sz_um*l_ip.F_sz_um);
  area.set_area(
      5 * compute_gate_area(NAND, 2, WdecNANDn, WdecNANDp, g_tp.cell_h_def) +
      compute_gate_area(NAND, 2, WdecNANDn, WdecNANDn, g_tp.cell_h_def));
}

double DFFCell::fpfp_node_cap(unsigned int fan_in, unsigned int fan_out) {
  double Ctotal = 0;
  // printf("WdecNANDn = %E\n", WdecNANDn);

  /* part 1: drain cap of NAND gate */
  Ctotal += drain_C_(WdecNANDn, NCH, 2, 1, g_tp.cell_h_def, is_dram) +
            fan_in * drain_C_(WdecNANDp, PCH, 1, 1, g_tp.cell_h_def, is_dram);

  /* part 2: gate cap of NAND gates */
  Ctotal += fan_out * gate_C(WdecNANDn + WdecNANDp, 0, is_dram);

  return Ctotal;
}

void DFFCell::compute_DFF_cell() {
  double c1, c2, c3, c4, c5, c6;
  /* node 5 and node 6 are identical to node 1 in capacitance */
  c1 = c5 = c6 = fpfp_node_cap(2, 1);
  c2 = fpfp_node_cap(2, 3);
  c3 = fpfp_node_cap(3, 2);
  c4 = fpfp_node_cap(2, 2);

  // cap-load of the clock signal in each Dff, actually the clock signal only
  // connected to one NAND2
  clock_cap = 2 * gate_C(WdecNANDn + WdecNANDp, 0, is_dram);
  e_switch.readOp.dynamic += (c4 + c1 + c2 + c3 + c5 + c6 + 2 * cell_load) *
                             0.5 * g_tp.peri_global.Vdd * g_tp.peri_global.Vdd;
  ;

  /* no 1/2 for e_keep and e_clock because clock signal switches twice in one
   * cycle */
  e_keep_1.readOp.dynamic += c3 * g_tp.peri_global.Vdd * g_tp.peri_global.Vdd;
  e_keep_0.readOp.dynamic += c2 * g_tp.peri_global.Vdd * g_tp.peri_global.Vdd;
  e_clock.readOp.dynamic +=
      clock_cap * g_tp.peri_global.Vdd * g_tp.peri_global.Vdd;
  ;

  /* static power */
  e_switch.readOp.leakage +=
      (cmos_Isub_leakage(WdecNANDn, WdecNANDp, 2, nand) *
           5 // 5 NAND2 and 1 NAND3 in a DFF
       + cmos_Isub_leakage(WdecNANDn, WdecNANDn, 3, nand)) *
      g_tp.peri_global.Vdd;
  e_switch.readOp.gate_leakage +=
      (cmos_Ig_leakage(WdecNANDn, WdecNANDp, 2, nand) *
           5 // 5 NAND2 and 1 NAND3 in a DFF
       + cmos_Ig_leakage(WdecNANDn, WdecNANDn, 3, nand)) *
      g_tp.peri_global.Vdd;
  // printf("leakage =%E\n",cmos_Ileak(1, is_dram) );
}

