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

#include "dep_resource_conflict_check.h"

dep_resource_conflict_check::dep_resource_conflict_check(
    const InputParameter *configure_interface,
    const CoreDynParam &dyn_p_,
    int compare_bits_,
    bool _is_default)
    : l_ip(*configure_interface), coredynp(dyn_p_), compare_bits(compare_bits_),
      is_default(_is_default) {
  Wcompn = 25 * l_ip.F_sz_um; // this was 20.0 micron for the 0.8 micron process
  Wevalinvp =
      25 * l_ip.F_sz_um; // this was 20.0 micron for the 0.8 micron process
  Wevalinvn =
      100 * l_ip.F_sz_um; // this was 80.0 mcron for the 0.8 micron process
  Wcomppreequ =
      50 * l_ip.F_sz_um; // this was 40.0  micron for the 0.8 micron process
  WNORn = 6.75 * l_ip.F_sz_um; // this was 5.4 micron for the 0.8 micron process
  WNORp =
      38.125 * l_ip.F_sz_um; // this was 30.5 micron for the 0.8 micron process

  local_result = init_interface(&l_ip);

  if (coredynp.core_ty == Inorder)
    compare_bits += 16 + 8 + 8; // TODO: opcode bits + log(shared resources) +
                                // REG TAG BITS-->opcode comparator
  else
    compare_bits += 16 + 8 + 8;

  conflict_check_power();
  double sckRation = g_tp.sckt_co_eff;
  power.readOp.dynamic *= sckRation;
  power.writeOp.dynamic *= sckRation;
  power.searchOp.dynamic *= sckRation;
}

void dep_resource_conflict_check::set_params(const InputParameter *configure_interface,
                              const CoreDynParam &dyn_p_,
                              int compare_bits_,
                              bool _is_default){

  l_ip = *configure_interface;
  coredynp = dyn_p_;
  compare_bits = compare_bits_;
  is_default = _is_default;

  Wcompn = 25 * l_ip.F_sz_um; // this was 20.0 micron for the 0.8 micron process
  Wevalinvp =
      25 * l_ip.F_sz_um; // this was 20.0 micron for the 0.8 micron process
  Wevalinvn =
      100 * l_ip.F_sz_um; // this was 80.0 mcron for the 0.8 micron process
  Wcomppreequ =
      50 * l_ip.F_sz_um; // this was 40.0  micron for the 0.8 micron process
  WNORn = 6.75 * l_ip.F_sz_um; // this was 5.4 micron for the 0.8 micron process
  WNORp =
      38.125 * l_ip.F_sz_um; // this was 30.5 micron for the 0.8 micron process

  local_result = init_interface(&l_ip);

  if (coredynp.core_ty == Inorder) {
    compare_bits += 16 + 8 + 8; // TODO: opcode bits + log(shared resources) +
                                // REG TAG BITS-->opcode comparator
  }
  else {
    compare_bits += 16 + 8 + 8;
  }

  conflict_check_power();
  double sckRation = g_tp.sckt_co_eff;
  power.readOp.dynamic *= sckRation;
  power.writeOp.dynamic *= sckRation;
  power.searchOp.dynamic *= sckRation;
                              }
                              
void dep_resource_conflict_check::conflict_check_power() {
  double Ctotal;
  int num_comparators;
  num_comparators =
      3 * ((coredynp.decodeW) * (coredynp.decodeW) -
           coredynp.decodeW); // 2(N*N-N) is used for source to dest comparison,
                              // (N*N-N) is used for dest to dest comparision.
  // When decode-width ==1, no dcl logic

  Ctotal = num_comparators * compare_cap();
  // printf("%i,%s\n",XML_interface->sys.core[0].predictor.predictor_entries,XML_interface->sys.core[0].predictor.prediction_scheme);

  power.readOp.dynamic =
      Ctotal * /*CLOCKRATE*/ g_tp.peri_global.Vdd * g_tp.peri_global.Vdd /*AF*/;
  power.readOp.leakage = num_comparators * compare_bits * 2 *
                         simplified_nmos_leakage(Wcompn, false);

  double long_channel_device_reduction =
      longer_channel_device_reduction(Core_device, coredynp.core_ty);
  power.readOp.longer_channel_leakage =
      power.readOp.leakage * long_channel_device_reduction;
  power.readOp.gate_leakage =
      num_comparators * compare_bits * 2 * cmos_Ig_leakage(Wcompn, 0, 2, nmos);

  double pg_reduction = power_gating_leakage_reduction(false);
  power.readOp.power_gated_leakage = power.readOp.leakage * pg_reduction;
  power.readOp.power_gated_with_long_channel_leakage =
      power.readOp.power_gated_leakage * long_channel_device_reduction;
}

/* estimate comparator power consumption (this comparator is similar
   to the tag-match structure in a CAM */
double dep_resource_conflict_check::compare_cap() {
  double c1, c2;

  WNORp = WNORp * compare_bits /
          2.0; // resize the big NOR gate at the DCL according to fan in.
  /* bottom part of comparator */
  c2 = (compare_bits) * (drain_C_(Wcompn, NCH, 1, 1, g_tp.cell_h_def) +
                         drain_C_(Wcompn, NCH, 2, 1, g_tp.cell_h_def)) +
       drain_C_(Wevalinvp, PCH, 1, 1, g_tp.cell_h_def) +
       drain_C_(Wevalinvn, NCH, 1, 1, g_tp.cell_h_def);

  /* top part of comparator */
  c1 = (compare_bits) * (drain_C_(Wcompn, NCH, 1, 1, g_tp.cell_h_def) +
                         drain_C_(Wcompn, NCH, 2, 1, g_tp.cell_h_def) +
                         drain_C_(Wcomppreequ, NCH, 1, 1, g_tp.cell_h_def)) +
       gate_C(WNORn + WNORp, 10.0) +
       drain_C_(WNORp, NCH, 2, 1, g_tp.cell_h_def) +
       compare_bits * drain_C_(WNORn, NCH, 2, 1, g_tp.cell_h_def);
  return (c1 + c2);
}

void dep_resource_conflict_check::leakage_feedback(double temperature) {
  l_ip.temp = (unsigned int)round(temperature / 10.0) * 10;
  uca_org_t init_result = init_interface(&l_ip); // init_result is dummy

  // This is part of conflict_check_power()
  int num_comparators =
      3 * ((coredynp.decodeW) * (coredynp.decodeW) -
           coredynp.decodeW); // 2(N*N-N) is used for source to dest comparison,
                              // (N*N-N) is used for dest to dest comparision.
  power.readOp.leakage = num_comparators * compare_bits * 2 *
                         simplified_nmos_leakage(Wcompn, false);

  double long_channel_device_reduction =
      longer_channel_device_reduction(Core_device, coredynp.core_ty);
  power.readOp.longer_channel_leakage =
      power.readOp.leakage * long_channel_device_reduction;
  power.readOp.gate_leakage =
      num_comparators * compare_bits * 2 * cmos_Ig_leakage(Wcompn, 0, 2, nmos);

  double pg_reduction = power_gating_leakage_reduction(false);
  power.readOp.power_gated_leakage = power.readOp.leakage * pg_reduction;
  power.readOp.power_gated_with_long_channel_leakage =
      power.readOp.power_gated_leakage * long_channel_device_reduction;
}
