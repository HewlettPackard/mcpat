/*****************************************************************************
 *                                McPAT/CACTI
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

#include "decoder.h"

#include "area.h"
#include "parameter.h"

#include <assert.h>
#include <iostream>
#include <math.h>

using namespace std;

Decoder::Decoder(int _num_dec_signals,
                 bool flag_way_select,
                 double _C_ld_dec_out,
                 double _R_wire_dec_out,
                 bool fully_assoc_,
                 bool is_dram_,
                 bool is_wl_tr_,
                 const Area &cell_,
                 bool power_gating_,
                 int nodes_DSTN_)
    : exist(false), C_ld_dec_out(_C_ld_dec_out),
      R_wire_dec_out(_R_wire_dec_out), num_gates(0), num_gates_min(2), delay(0),
      // power(),
      fully_assoc(fully_assoc_), is_dram(is_dram_), is_wl_tr(is_wl_tr_),
      total_driver_nwidth(0), total_driver_pwidth(0), sleeptx(NULL),
      nodes_DSTN(nodes_DSTN_), power_gating(power_gating_) {

  for (int i = 0; i < MAX_NUMBER_GATES_STAGE; i++) {
    w_dec_n[i] = 0;
    w_dec_p[i] = 0;
  }

  /*
   * _num_dec_signals is the number of decoded signal as output
   * num_addr_bits_dec is the number of signal to be decoded
   * as the decoders input.
   */
  int num_addr_bits_dec = _log2(_num_dec_signals);

  if (num_addr_bits_dec < 4) {
    if (flag_way_select) {
      exist = true;
      num_in_signals = 2;
    } else {
      num_in_signals = 0;
    }
  } else {
    exist = true;

    if (flag_way_select) {
      num_in_signals = 3;
    } else {
      num_in_signals = 2;
    }
  }

  assert(cell_.h > 0);
  assert(cell_.w > 0);
  // the height of a row-decoder-driver cell is fixed to be 4 * cell.h;
  // area.h = 4 * cell.h;
  area.h = g_tp.h_dec * cell_.h;
  height = cell_.h;
  compute_widths();
  compute_area();
}

void Decoder::set_params(int _num_dec_signals,
                         bool flag_way_select,
                         double _C_ld_dec_out,
                         double _R_wire_dec_out,
                         bool fully_assoc_,
                         bool is_dram_,
                         bool is_wl_tr_,
                         const Area &cell_,
                         bool power_gating_,
                         int nodes_DSTN_) {

  exist = false;
  C_ld_dec_out = _C_ld_dec_out;

  R_wire_dec_out = _R_wire_dec_out;
  num_gates = 0;
  num_gates_min = 2;
  delay = 0;
  fully_assoc = fully_assoc_;
  is_dram = is_dram_;
  is_wl_tr = is_wl_tr_;
  total_driver_nwidth = 0;
  total_driver_pwidth = 0;
  sleeptx = NULL;
  nodes_DSTN = nodes_DSTN_;
  power_gating = power_gating_;

  for (int i = 0; i < MAX_NUMBER_GATES_STAGE; i++) {
    w_dec_n[i] = 0;
    w_dec_p[i] = 0;
  }

  /*
   * _num_dec_signals is the number of decoded signal as output
   * num_addr_bits_dec is the number of signal to be decoded
   * as the decoders input.
   */
  int num_addr_bits_dec = _log2(_num_dec_signals);

  if (num_addr_bits_dec < 4) {
    if (flag_way_select) {
      exist = true;
      num_in_signals = 2;
    } else {
      num_in_signals = 0;
    }
  } else {
    exist = true;

    if (flag_way_select) {
      num_in_signals = 3;
    } else {
      num_in_signals = 2;
    }
  }

  assert(cell_.h > 0);
  assert(cell_.w > 0);
  // the height of a row-decoder-driver cell is fixed to be 4 * cell.h;
  // area.h = 4 * cell.h;
  area.h = g_tp.h_dec * cell_.h;

  height = cell_.h;
}

void Decoder::compute_widths() {
  double F;
  double p_to_n_sz_ratio = pmos_to_nmos_sz_ratio(is_dram, is_wl_tr);
  double gnand2 = (2 + p_to_n_sz_ratio) / (1 + p_to_n_sz_ratio);
  double gnand3 = (3 + p_to_n_sz_ratio) / (1 + p_to_n_sz_ratio);

  if (exist) {
    if (num_in_signals == 2 || fully_assoc) {
      w_dec_n[0] = 2 * g_tp.min_w_nmos_;
      w_dec_p[0] = p_to_n_sz_ratio * g_tp.min_w_nmos_;
      F = gnand2;
    } else {
      w_dec_n[0] = 3 * g_tp.min_w_nmos_;
      w_dec_p[0] = p_to_n_sz_ratio * g_tp.min_w_nmos_;
      F = gnand3;
    }

    F *= C_ld_dec_out / (gate_C(w_dec_n[0], 0, is_dram, false, is_wl_tr) +
                         gate_C(w_dec_p[0], 0, is_dram, false, is_wl_tr));
    num_gates = logical_effort(num_gates_min,
                               num_in_signals == 2 ? gnand2 : gnand3,
                               F,
                               w_dec_n,
                               w_dec_p,
                               C_ld_dec_out,
                               p_to_n_sz_ratio,
                               is_dram,
                               is_wl_tr,
                               g_tp.max_w_nmos_dec);
  }
}

void Decoder::computeArea() {
  compute_widths();
  compute_area();
}
void Decoder::compute_area() {
  double cumulative_area = 0;
  double cumulative_curr = 0;    // cumulative leakage current
  double cumulative_curr_Ig = 0; // cumulative leakage current

  if (exist) { // First check if this decoder exists
    if (num_in_signals == 2) {
      cumulative_area =
          compute_gate_area(NAND, 2, w_dec_p[0], w_dec_n[0], area.h);
      cumulative_curr =
          cmos_Isub_leakage(w_dec_n[0], w_dec_p[0], 2, nand, is_dram);
      cumulative_curr_Ig =
          cmos_Ig_leakage(w_dec_n[0], w_dec_p[0], 2, nand, is_dram);
    } else if (num_in_signals == 3) {
      cumulative_area =
          compute_gate_area(NAND, 3, w_dec_p[0], w_dec_n[0], area.h);
      cumulative_curr =
          cmos_Isub_leakage(w_dec_n[0], w_dec_p[0], 3, nand, is_dram);
      ;
      cumulative_curr_Ig =
          cmos_Ig_leakage(w_dec_n[0], w_dec_p[0], 3, nand, is_dram);
    }

    for (int i = 1; i < num_gates; i++) {
      cumulative_area +=
          compute_gate_area(INV, 1, w_dec_p[i], w_dec_n[i], area.h);
      cumulative_curr +=
          cmos_Isub_leakage(w_dec_n[i], w_dec_p[i], 1, inv, is_dram);
      cumulative_curr_Ig =
          cmos_Ig_leakage(w_dec_n[i], w_dec_p[i], 1, inv, is_dram);
    }
    power.readOp.leakage = cumulative_curr * g_tp.peri_global.Vdd;
    power.readOp.power_gated_leakage =
        cumulative_curr * g_tp.peri_global.Vcc_min;
    power.readOp.gate_leakage = cumulative_curr_Ig * g_tp.peri_global.Vdd;

    area.w = (cumulative_area / area.h);
    if (power_gating) {
      compute_power_gating();
      cumulative_area += sleeptx->area.get_area();
      area.w = (cumulative_area / area.h);
    }
  }
}

void Decoder::compute_power_gating() {
  // For all driver chains there is only one sleep transistors to save area
  // Total transistor width for sleep tx calculation
  for (int i = 0; i < num_gates; i++) {
    total_driver_nwidth += w_dec_n[i];
    total_driver_pwidth += w_dec_p[i];
  }

  // compute sleep tx
  bool is_footer = false;
  double Isat_subarray = simplified_nmos_Isat(total_driver_nwidth);
  double detalV;
  double c_wakeup;

  c_wakeup = drain_C_(total_driver_pwidth, PCH, 1, 1, height); // Psleep tx
  detalV = g_tp.peri_global.Vdd - g_tp.peri_global.Vcc_min;
  //    if (g_ip->power_gating)
  sleeptx = new Sleep_tx(g_ip->perfloss,
                         Isat_subarray,
                         is_footer,
                         c_wakeup,
                         detalV,
                         nodes_DSTN,
                         area);
}

double Decoder::compute_delays(double inrisetime) {
  if (exist) {
    double ret_val = 0; // outrisetime
    int i;
    double rd, tf, this_delay, c_load, c_intrinsic, Vpp;
    double Vdd = g_tp.peri_global.Vdd;

    if ((is_wl_tr) && (is_dram)) {
      Vpp = g_tp.vpp;
    } else if (is_wl_tr) {
      Vpp = g_tp.sram_cell.Vdd;
    } else {
      Vpp = g_tp.peri_global.Vdd;
    }

    // first check whether a decoder is required at all
    rd = tr_R_on(w_dec_n[0], NCH, num_in_signals, is_dram, false, is_wl_tr);
    c_load = gate_C(w_dec_n[1] + w_dec_p[1], 0.0, is_dram, false, is_wl_tr);
    c_intrinsic =
        drain_C_(w_dec_p[0], PCH, 1, 1, area.h, is_dram, false, is_wl_tr) *
            num_in_signals +
        drain_C_(w_dec_n[0],
                 NCH,
                 num_in_signals,
                 1,
                 area.h,
                 is_dram,
                 false,
                 is_wl_tr);
    tf = rd * (c_intrinsic + c_load);
    this_delay = horowitz(inrisetime, tf, 0.5, 0.5, RISE);
    delay += this_delay;
    inrisetime = this_delay / (1.0 - 0.5);
    power.readOp.dynamic += (c_load + c_intrinsic) * Vdd * Vdd;
    //    cout<<"w_dec_n["<<0<<"] = "<<w_dec_n[0]<<" delay = "<<this_delay<<"
    //    tf= "<<tf  <<endl;

    for (i = 1; i < num_gates - 1; ++i) {
      rd = tr_R_on(w_dec_n[i], NCH, 1, is_dram, false, is_wl_tr);
      c_load = gate_C(
          w_dec_p[i + 1] + w_dec_n[i + 1], 0.0, is_dram, false, is_wl_tr);
      c_intrinsic =
          drain_C_(w_dec_p[i], PCH, 1, 1, area.h, is_dram, false, is_wl_tr) +
          drain_C_(w_dec_n[i], NCH, 1, 1, area.h, is_dram, false, is_wl_tr);
      tf = rd * (c_intrinsic + c_load);
      this_delay = horowitz(inrisetime, tf, 0.5, 0.5, RISE);
      delay += this_delay;
      inrisetime = this_delay / (1.0 - 0.5);
      power.readOp.dynamic += (c_load + c_intrinsic) * Vdd * Vdd;
      //      cout<<"w_dec_n["<<i<<"] = "<<w_dec_n[i]<<" delay =
      //      "<<this_delay<<" tf= "<<tf  <<endl;
    }
    //    cout<<endl;

    // add delay of final inverter that drives the wordline
    i = num_gates - 1;
    c_load = C_ld_dec_out;
    rd = tr_R_on(w_dec_n[i], NCH, 1, is_dram, false, is_wl_tr);
    c_intrinsic =
        drain_C_(w_dec_p[i], PCH, 1, 1, area.h, is_dram, false, is_wl_tr) +
        drain_C_(w_dec_n[i], NCH, 1, 1, area.h, is_dram, false, is_wl_tr);
    tf = rd * (c_intrinsic + c_load) + R_wire_dec_out * c_load / 2;
    this_delay = horowitz(inrisetime, tf, 0.5, 0.5, RISE);
    delay += this_delay;
    ret_val = this_delay / (1.0 - 0.5);
    power.readOp.dynamic += c_load * Vpp * Vpp + c_intrinsic * Vdd * Vdd;

    //    cout<<"w_dec_n["<<i<<"] = "<<w_dec_n[i]<<" delay = "<<this_delay<<"
    //    tf= "<<tf  << " tr_R_on "<< rd <<" R_wire_dec_out = " <<R_wire_dec_out
    //    <<endl;
    return ret_val;
  } else {
    return 0.0;
  }
}

void Decoder::leakage_feedback(double temperature) {
  double cumulative_curr = 0;    // cumulative leakage current
  double cumulative_curr_Ig = 0; // cumulative leakage current

  if (exist) { // First check if this decoder exists
    if (num_in_signals == 2) {
      cumulative_curr =
          cmos_Isub_leakage(w_dec_n[0], w_dec_p[0], 2, nand, is_dram);
      cumulative_curr_Ig =
          cmos_Ig_leakage(w_dec_n[0], w_dec_p[0], 2, nand, is_dram);
    } else if (num_in_signals == 3) {
      cumulative_curr =
          cmos_Isub_leakage(w_dec_n[0], w_dec_p[0], 3, nand, is_dram);
      ;
      cumulative_curr_Ig =
          cmos_Ig_leakage(w_dec_n[0], w_dec_p[0], 3, nand, is_dram);
    }

    for (int i = 1; i < num_gates; i++) {
      cumulative_curr +=
          cmos_Isub_leakage(w_dec_n[i], w_dec_p[i], 1, inv, is_dram);
      cumulative_curr_Ig =
          cmos_Ig_leakage(w_dec_n[i], w_dec_p[i], 1, inv, is_dram);
    }

    power.readOp.leakage = cumulative_curr * g_tp.peri_global.Vdd;
    power.readOp.gate_leakage = cumulative_curr_Ig * g_tp.peri_global.Vdd;
  }
}

// TODO: add sleep tx in predec/predecblk/predecdriver
