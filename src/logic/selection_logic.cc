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

#include "selection_logic.h"

// selection_logic
selection_logic::selection_logic(bool _is_default,
                                 int win_entries_,
                                 int issue_width_,
                                 const InputParameter *configure_interface,
                                 enum Device_ty device_ty_,
                                 enum Core_type core_ty_)
    // const ParseXML *_XML_interface)
    : is_default(_is_default), win_entries(win_entries_),
      issue_width(issue_width_), device_ty(device_ty_), core_ty(core_ty_) {
  // uca_org_t result2;
  l_ip = *configure_interface;
  local_result = init_interface(&l_ip);
  // init_tech_params(l_ip.F_sz_um, false);
  // win_entries=numIBEntries;//IQentries;
  // issue_width=issueWidth;
  selection_power();
  double sckRation = g_tp.sckt_co_eff;
  power.readOp.dynamic *= sckRation;
  power.writeOp.dynamic *= sckRation;
  power.searchOp.dynamic *= sckRation;

  double long_channel_device_reduction =
      longer_channel_device_reduction(device_ty, core_ty);
  power.readOp.longer_channel_leakage =
      power.readOp.leakage * long_channel_device_reduction;

  double pg_reduction = power_gating_leakage_reduction(false);
  power.readOp.power_gated_leakage = power.readOp.leakage * pg_reduction;
  power.readOp.power_gated_with_long_channel_leakage =
      power.readOp.power_gated_leakage * long_channel_device_reduction;
}
void selection_logic::set_params(bool _is_default,
                                 int win_entries_,
                                 int issue_width_,
                                 const InputParameter *configure_interface,
                                 enum Device_ty device_ty_,
                                 enum Core_type core_ty_) {
  is_default = _is_default;
  win_entries = win_entries_;
  issue_width = issue_width_;
  device_ty = device_ty_;
  core_ty = core_ty_;

  l_ip = *configure_interface;
  local_result = init_interface(&l_ip);
  // init_tech_params(l_ip.F_sz_um, false);
  // win_entries=numIBEntries;//IQentries;
  // issue_width=issueWidth;

  selection_power();
  double sckRation = g_tp.sckt_co_eff;
  power.readOp.dynamic *= sckRation;
  power.writeOp.dynamic *= sckRation;
  power.searchOp.dynamic *= sckRation;

  double long_channel_device_reduction =
      longer_channel_device_reduction(device_ty, core_ty);
  power.readOp.longer_channel_leakage =
      power.readOp.leakage * long_channel_device_reduction;

  double pg_reduction = power_gating_leakage_reduction(false);
  power.readOp.power_gated_leakage = power.readOp.leakage * pg_reduction;
  power.readOp.power_gated_with_long_channel_leakage =
      power.readOp.power_gated_leakage * long_channel_device_reduction;
}

void selection_logic::selection_power() { // based on cost effective superscalar
                                          // processor TR pp27-31
  double Ctotal, Cor, Cpencode;
  int num_arbiter;
  double WSelORn, WSelORprequ, WSelPn, WSelPp, WSelEnn, WSelEnp;

  // TODO: the 0.8um process data is used.
  WSelORn = 12.5 * l_ip.F_sz_um; // this was 10 micron for the 0.8 micron
                                 // process
  WSelORprequ =
      50 * l_ip.F_sz_um;        // this was 40 micron for the 0.8 micron process
  WSelPn = 12.5 * l_ip.F_sz_um; // this was 10mcron for the 0.8 micron process
  WSelPp = 18.75 * l_ip.F_sz_um; // this was 15 micron for the 0.8 micron
                                 // process
  WSelEnn = 6.25 * l_ip.F_sz_um; // this was 5 micron for the 0.8 micron process
  WSelEnp = 12.5 * l_ip.F_sz_um; // this was 10 micron for the 0.8 micron
                                 // process

  Ctotal = 0;
  num_arbiter = 1;
  while (win_entries > 4) {
    win_entries = (int)ceil((double)win_entries / 4.0);
    num_arbiter += win_entries;
  }
  // the 4-input OR logic to generate anyreq
  Cor = 4 * drain_C_(WSelORn, NCH, 1, 1, g_tp.cell_h_def) +
        drain_C_(WSelORprequ, PCH, 1, 1, g_tp.cell_h_def);
  power.readOp.gate_leakage =
      cmos_Ig_leakage(WSelORn, WSelORprequ, 4, nor) * g_tp.peri_global.Vdd;

  // The total capacity of the 4-bit priority encoder
  Cpencode =
      drain_C_(WSelPn, NCH, 1, 1, g_tp.cell_h_def) +
      drain_C_(WSelPp, PCH, 1, 1, g_tp.cell_h_def) +
      2 * drain_C_(WSelPn, NCH, 1, 1, g_tp.cell_h_def) +
      drain_C_(WSelPp, PCH, 2, 1, g_tp.cell_h_def) +
      3 * drain_C_(WSelPn, NCH, 1, 1, g_tp.cell_h_def) +
      drain_C_(WSelPp, PCH, 3, 1, g_tp.cell_h_def) +
      4 * drain_C_(WSelPn, NCH, 1, 1, g_tp.cell_h_def) +
      drain_C_(WSelPp,
               PCH,
               4,
               1,
               g_tp.cell_h_def) + // precompute priority logic
      2 * 4 * gate_C(WSelEnn + WSelEnp, 20.0) +
      4 * drain_C_(WSelEnn, NCH, 1, 1, g_tp.cell_h_def) +
      2 * 4 * drain_C_(WSelEnp, PCH, 1, 1, g_tp.cell_h_def) + // enable logic
      (2 * 4 + 2 * 3 + 2 * 2 + 2) *
          gate_C(WSelPn + WSelPp, 10.0); // requests signal

  Ctotal += issue_width * num_arbiter * (Cor + Cpencode);

  power.readOp.dynamic =
      Ctotal * g_tp.peri_global.Vdd * g_tp.peri_global.Vdd *
      2; // 2 means the abitration signal need to travel round trip
  power.readOp.leakage =
      issue_width * num_arbiter *
      (cmos_Isub_leakage(
           WSelPn,
           WSelPp,
           2,
           nor) /*approximate precompute with a nor gate*/ // grant1p
       + cmos_Isub_leakage(WSelPn, WSelPp, 3, nor)         // grant2p
       + cmos_Isub_leakage(WSelPn, WSelPp, 4, nor)         // grant3p
       + cmos_Isub_leakage(WSelEnn, WSelEnp, 2, nor) * 4   // enable logic
       + cmos_Isub_leakage(WSelEnn, WSelEnp, 1, inv) * 2 *
             3 // for each grant there are two inverters, there are 3 grant
               // sIsubnals
       ) *
      g_tp.peri_global.Vdd;
  power.readOp.gate_leakage =
      issue_width * num_arbiter *
      (cmos_Ig_leakage(
           WSelPn,
           WSelPp,
           2,
           nor) /*approximate precompute with a nor gate*/ // grant1p
       + cmos_Ig_leakage(WSelPn, WSelPp, 3, nor)           // grant2p
       + cmos_Ig_leakage(WSelPn, WSelPp, 4, nor)           // grant3p
       + cmos_Ig_leakage(WSelEnn, WSelEnp, 2, nor) * 4     // enable logic
       + cmos_Ig_leakage(WSelEnn, WSelEnp, 1, inv) * 2 *
             3 // for each grant there are two inverters, there are 3 grant
               // signals
       ) *
      g_tp.peri_global.Vdd;
}
