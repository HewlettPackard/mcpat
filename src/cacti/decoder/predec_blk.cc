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

#include "predec_blk.h"

#include "area.h"
#include "decoder.h"
#include "parameter.h"

#include <assert.h>
#include <iostream>
#include <math.h>

PredecBlk::PredecBlk(int num_dec_signals,
                     Decoder *dec_,
                     double C_wire_predec_blk_out,
                     double R_wire_predec_blk_out_,
                     int num_dec_per_predec,
                     bool is_dram,
                     bool is_blk1) {
  dec = dec_;
  exist = false;
  number_input_addr_bits = 0;
  C_ld_predec_blk_out = 0;
  R_wire_predec_blk_out = 0;
  branch_effort_nand2_gate_output = 1;
  branch_effort_nand3_gate_output = 1;
  flag_two_unique_paths = false;
  flag_L2_gate = 0;
  number_inputs_L1_gate = 0;
  number_gates_L1_nand2_path = 0;
  number_gates_L1_nand3_path = 0;
  number_gates_L2 = 0;
  min_number_gates_L1 = 2;
  min_number_gates_L2 = 2;
  num_L1_active_nand2_path = 0;
  num_L1_active_nand3_path = 0;
  delay_nand2_path = 0;
  delay_nand3_path = 0;
  is_dram_ = is_dram;

  int branch_effort_predec_out;
  double C_ld_dec_gate;
  int num_addr_bits_dec = _log2(num_dec_signals);
  int blk1_num_input_addr_bits = (num_addr_bits_dec + 1) / 2;
  int blk2_num_input_addr_bits = num_addr_bits_dec - blk1_num_input_addr_bits;

  w_L1_nand2_n[0] = 0;
  w_L1_nand2_p[0] = 0;
  w_L1_nand3_n[0] = 0;
  w_L1_nand3_p[0] = 0;

  if (is_blk1 == true) {
    if (num_addr_bits_dec <= 0) {
      return;
    } else if (num_addr_bits_dec < 4) {
      // Just one predecoder block is required with NAND2 gates. No decoder
      // required. The first level of predecoding directly drives the decoder
      // output load
      exist = true;
      number_input_addr_bits = num_addr_bits_dec;
      R_wire_predec_blk_out = dec->R_wire_dec_out;
      C_ld_predec_blk_out = dec->C_ld_dec_out;
    } else {
      exist = true;
      number_input_addr_bits = blk1_num_input_addr_bits;
      branch_effort_predec_out = (1 << blk2_num_input_addr_bits);
      C_ld_dec_gate =
          num_dec_per_predec *
          gate_C(dec->w_dec_n[0] + dec->w_dec_p[0], 0, is_dram_, false, false);
      R_wire_predec_blk_out = R_wire_predec_blk_out_;
      C_ld_predec_blk_out =
          branch_effort_predec_out * C_ld_dec_gate + C_wire_predec_blk_out;
    }
  } else {
    if (num_addr_bits_dec >= 4) {
      exist = true;
      number_input_addr_bits = blk2_num_input_addr_bits;
      branch_effort_predec_out = (1 << blk1_num_input_addr_bits);
      C_ld_dec_gate =
          num_dec_per_predec *
          gate_C(dec->w_dec_n[0] + dec->w_dec_p[0], 0, is_dram_, false, false);
      R_wire_predec_blk_out = R_wire_predec_blk_out_;
      C_ld_predec_blk_out =
          branch_effort_predec_out * C_ld_dec_gate + C_wire_predec_blk_out;
    }
  }

  compute_widths();
  compute_area();
}

void PredecBlk::set_params(int num_dec_signals,
                           Decoder *dec_,
                           double C_wire_predec_blk_out,
                           double R_wire_predec_blk_out_,
                           int num_dec_per_predec,
                           bool is_dram,
                           bool is_blk1) {
  dec = dec_;
  exist = false;
  number_input_addr_bits = 0;
  C_ld_predec_blk_out = 0;
  R_wire_predec_blk_out = 0;
  branch_effort_nand2_gate_output = 1;
  branch_effort_nand3_gate_output = 1;
  flag_two_unique_paths = false;
  flag_L2_gate = 0;
  number_inputs_L1_gate = 0;
  number_gates_L1_nand2_path = 0;
  number_gates_L1_nand3_path = 0;
  number_gates_L2 = 0;
  min_number_gates_L1 = 2;
  min_number_gates_L2 = 2;
  num_L1_active_nand2_path = 0;
  num_L1_active_nand3_path = 0;
  delay_nand2_path = 0;
  delay_nand3_path = 0;
  is_dram_ = is_dram;

  int branch_effort_predec_out;
  double C_ld_dec_gate;
  int num_addr_bits_dec = _log2(num_dec_signals);
  int blk1_num_input_addr_bits = (num_addr_bits_dec + 1) / 2;
  int blk2_num_input_addr_bits = num_addr_bits_dec - blk1_num_input_addr_bits;

  w_L1_nand2_n[0] = 0;
  w_L1_nand2_p[0] = 0;
  w_L1_nand3_n[0] = 0;
  w_L1_nand3_p[0] = 0;

  if (is_blk1 == true) {
    if (num_addr_bits_dec <= 0) {
      return;
    } else if (num_addr_bits_dec < 4) {
      // Just one predecoder block is required with NAND2 gates. No decoder
      // required. The first level of predecoding directly drives the decoder
      // output load
      exist = true;
      number_input_addr_bits = num_addr_bits_dec;
      R_wire_predec_blk_out = dec->R_wire_dec_out;
      C_ld_predec_blk_out = dec->C_ld_dec_out;
    } else {
      exist = true;
      number_input_addr_bits = blk1_num_input_addr_bits;
      branch_effort_predec_out = (1 << blk2_num_input_addr_bits);
      C_ld_dec_gate =
          num_dec_per_predec *
          gate_C(dec->w_dec_n[0] + dec->w_dec_p[0], 0, is_dram_, false, false);
      R_wire_predec_blk_out = R_wire_predec_blk_out_;
      C_ld_predec_blk_out =
          branch_effort_predec_out * C_ld_dec_gate + C_wire_predec_blk_out;
    }
  } else {
    if (num_addr_bits_dec >= 4) {
      exist = true;
      number_input_addr_bits = blk2_num_input_addr_bits;
      branch_effort_predec_out = (1 << blk1_num_input_addr_bits);
      C_ld_dec_gate =
          num_dec_per_predec *
          gate_C(dec->w_dec_n[0] + dec->w_dec_p[0], 0, is_dram_, false, false);
      R_wire_predec_blk_out = R_wire_predec_blk_out_;
      C_ld_predec_blk_out =
          branch_effort_predec_out * C_ld_dec_gate + C_wire_predec_blk_out;
    }
  }

  compute_widths();
  compute_area();
}

void PredecBlk::compute_widths() {
  double F, c_load_nand3_path, c_load_nand2_path;
  double p_to_n_sz_ratio = pmos_to_nmos_sz_ratio(is_dram_);
  double gnand2 = (2 + p_to_n_sz_ratio) / (1 + p_to_n_sz_ratio);
  double gnand3 = (3 + p_to_n_sz_ratio) / (1 + p_to_n_sz_ratio);

  if (exist == false)
    return;

  switch (number_input_addr_bits) {
    case 1:
      flag_two_unique_paths = false;
      number_inputs_L1_gate = 2;
      flag_L2_gate = 0;
      break;
    case 2:
      flag_two_unique_paths = false;
      number_inputs_L1_gate = 2;
      flag_L2_gate = 0;
      break;
    case 3:
      flag_two_unique_paths = false;
      number_inputs_L1_gate = 3;
      flag_L2_gate = 0;
      break;
    case 4:
      flag_two_unique_paths = false;
      number_inputs_L1_gate = 2;
      flag_L2_gate = 2;
      branch_effort_nand2_gate_output = 4;
      break;
    case 5:
      flag_two_unique_paths = true;
      flag_L2_gate = 2;
      branch_effort_nand2_gate_output = 8;
      branch_effort_nand3_gate_output = 4;
      break;
    case 6:
      flag_two_unique_paths = false;
      number_inputs_L1_gate = 3;
      flag_L2_gate = 2;
      branch_effort_nand3_gate_output = 8;
      break;
    case 7:
      flag_two_unique_paths = true;
      flag_L2_gate = 3;
      branch_effort_nand2_gate_output = 32;
      branch_effort_nand3_gate_output = 16;
      break;
    case 8:
      flag_two_unique_paths = true;
      flag_L2_gate = 3;
      branch_effort_nand2_gate_output = 64;
      branch_effort_nand3_gate_output = 32;
      break;
    case 9:
      flag_two_unique_paths = false;
      number_inputs_L1_gate = 3;
      flag_L2_gate = 3;
      branch_effort_nand3_gate_output = 64;
      break;
    default:
      assert(0);
      break;
  }

  // find the number of gates and sizing in second level of predecoder (if there
  // is a second level)
  if (flag_L2_gate) {
    if (flag_L2_gate == 2) { // 2nd level is a NAND2 gate
      w_L2_n[0] = 2 * g_tp.min_w_nmos_;
      F = gnand2;
    } else { // 2nd level is a NAND3 gate
      w_L2_n[0] = 3 * g_tp.min_w_nmos_;
      F = gnand3;
    }
    w_L2_p[0] = p_to_n_sz_ratio * g_tp.min_w_nmos_;
    F *= C_ld_predec_blk_out /
         (gate_C(w_L2_n[0], 0, is_dram_) + gate_C(w_L2_p[0], 0, is_dram_));
    number_gates_L2 = logical_effort(min_number_gates_L2,
                                     flag_L2_gate == 2 ? gnand2 : gnand3,
                                     F,
                                     w_L2_n,
                                     w_L2_p,
                                     C_ld_predec_blk_out,
                                     p_to_n_sz_ratio,
                                     is_dram_,
                                     false,
                                     g_tp.max_w_nmos_);

    // Now find the number of gates and widths in first level of predecoder
    if ((flag_two_unique_paths) ||
        (number_inputs_L1_gate ==
         2)) { // Whenever flag_two_unique_paths is true, it means first level
               // of decoder employs
      // both NAND2 and NAND3 gates. Or when number_inputs_L1_gate is 2, it
      // means a NAND2 gate is used in the first level of the predecoder
      c_load_nand2_path =
          branch_effort_nand2_gate_output *
          (gate_C(w_L2_n[0], 0, is_dram_) + gate_C(w_L2_p[0], 0, is_dram_));
      w_L1_nand2_n[0] = 2 * g_tp.min_w_nmos_;
      w_L1_nand2_p[0] = p_to_n_sz_ratio * g_tp.min_w_nmos_;
      F = gnand2 * c_load_nand2_path /
          (gate_C(w_L1_nand2_n[0], 0, is_dram_) +
           gate_C(w_L1_nand2_p[0], 0, is_dram_));
      number_gates_L1_nand2_path = logical_effort(min_number_gates_L1,
                                                  gnand2,
                                                  F,
                                                  w_L1_nand2_n,
                                                  w_L1_nand2_p,
                                                  c_load_nand2_path,
                                                  p_to_n_sz_ratio,
                                                  is_dram_,
                                                  false,
                                                  g_tp.max_w_nmos_);
    }

    // Now find widths of gates along path in which first gate is a NAND3
    if ((flag_two_unique_paths) ||
        (number_inputs_L1_gate ==
         3)) { // Whenever flag_two_unique_paths is TRUE, it means first level
               // of decoder employs
      // both NAND2 and NAND3 gates. Or when number_inputs_L1_gate is 3, it
      // means a NAND3 gate is used in the first level of the predecoder
      c_load_nand3_path =
          branch_effort_nand3_gate_output *
          (gate_C(w_L2_n[0], 0, is_dram_) + gate_C(w_L2_p[0], 0, is_dram_));
      w_L1_nand3_n[0] = 3 * g_tp.min_w_nmos_;
      w_L1_nand3_p[0] = p_to_n_sz_ratio * g_tp.min_w_nmos_;
      F = gnand3 * c_load_nand3_path /
          (gate_C(w_L1_nand3_n[0], 0, is_dram_) +
           gate_C(w_L1_nand3_p[0], 0, is_dram_));
      number_gates_L1_nand3_path = logical_effort(min_number_gates_L1,
                                                  gnand3,
                                                  F,
                                                  w_L1_nand3_n,
                                                  w_L1_nand3_p,
                                                  c_load_nand3_path,
                                                  p_to_n_sz_ratio,
                                                  is_dram_,
                                                  false,
                                                  g_tp.max_w_nmos_);
    }
  } else { // find number of gates and widths in first level of predecoder block
           // when there is no second level
    if (number_inputs_L1_gate == 2) {
      w_L1_nand2_n[0] = 2 * g_tp.min_w_nmos_;
      w_L1_nand2_p[0] = p_to_n_sz_ratio * g_tp.min_w_nmos_;
      F = gnand2 * C_ld_predec_blk_out /
          (gate_C(w_L1_nand2_n[0], 0, is_dram_) +
           gate_C(w_L1_nand2_p[0], 0, is_dram_));
      number_gates_L1_nand2_path = logical_effort(min_number_gates_L1,
                                                  gnand2,
                                                  F,
                                                  w_L1_nand2_n,
                                                  w_L1_nand2_p,
                                                  C_ld_predec_blk_out,
                                                  p_to_n_sz_ratio,
                                                  is_dram_,
                                                  false,
                                                  g_tp.max_w_nmos_);
    } else if (number_inputs_L1_gate == 3) {
      w_L1_nand3_n[0] = 3 * g_tp.min_w_nmos_;
      w_L1_nand3_p[0] = p_to_n_sz_ratio * g_tp.min_w_nmos_;
      F = gnand3 * C_ld_predec_blk_out /
          (gate_C(w_L1_nand3_n[0], 0, is_dram_) +
           gate_C(w_L1_nand3_p[0], 0, is_dram_));
      number_gates_L1_nand3_path = logical_effort(min_number_gates_L1,
                                                  gnand3,
                                                  F,
                                                  w_L1_nand3_n,
                                                  w_L1_nand3_p,
                                                  C_ld_predec_blk_out,
                                                  p_to_n_sz_ratio,
                                                  is_dram_,
                                                  false,
                                                  g_tp.max_w_nmos_);
    }
  }
}

void PredecBlk::compute_area() {
  if (exist) { // First check whether a predecoder block is needed
    int num_L1_nand2 = 0;
    int num_L1_nand3 = 0;
    int num_L2 = 0;
    double tot_area_L1_nand3 = 0;
    double leak_L1_nand3 = 0;
    double gate_leak_L1_nand3 = 0;

    double tot_area_L1_nand2 = compute_gate_area(
        NAND, 2, w_L1_nand2_p[0], w_L1_nand2_n[0], g_tp.cell_h_def);
    double leak_L1_nand2 =
        cmos_Isub_leakage(w_L1_nand2_n[0], w_L1_nand2_p[0], 2, nand, is_dram_);
    double gate_leak_L1_nand2 =
        cmos_Ig_leakage(w_L1_nand2_n[0], w_L1_nand2_p[0], 2, nand, is_dram_);
    if (number_inputs_L1_gate != 3) {
      tot_area_L1_nand3 = 0;
      leak_L1_nand3 = 0;
      gate_leak_L1_nand3 = 0;
    } else {
      tot_area_L1_nand3 = compute_gate_area(
          NAND, 3, w_L1_nand3_p[0], w_L1_nand3_n[0], g_tp.cell_h_def);
      leak_L1_nand3 =
          cmos_Isub_leakage(w_L1_nand3_n[0], w_L1_nand3_p[0], 3, nand);
      gate_leak_L1_nand3 =
          cmos_Ig_leakage(w_L1_nand3_n[0], w_L1_nand3_p[0], 3, nand);
    }

    switch (number_input_addr_bits) {
      case 1: // 2 NAND2 gates
        num_L1_nand2 = 2;
        num_L2 = 0;
        num_L1_active_nand2_path = 1;
        num_L1_active_nand3_path = 0;
        break;
      case 2: // 4 NAND2 gates
        num_L1_nand2 = 4;
        num_L2 = 0;
        num_L1_active_nand2_path = 1;
        num_L1_active_nand3_path = 0;
        break;
      case 3: // 8 NAND3 gates
        num_L1_nand3 = 8;
        num_L2 = 0;
        num_L1_active_nand2_path = 0;
        num_L1_active_nand3_path = 1;
        break;
      case 4: // 4 + 4 NAND2 gates
        num_L1_nand2 = 8;
        num_L2 = 16;
        num_L1_active_nand2_path = 2;
        num_L1_active_nand3_path = 0;
        break;
      case 5: // 4 NAND2 gates, 8 NAND3 gates
        num_L1_nand2 = 4;
        num_L1_nand3 = 8;
        num_L2 = 32;
        num_L1_active_nand2_path = 1;
        num_L1_active_nand3_path = 1;
        break;
      case 6: // 8 + 8 NAND3 gates
        num_L1_nand3 = 16;
        num_L2 = 64;
        num_L1_active_nand2_path = 0;
        num_L1_active_nand3_path = 2;
        break;
      case 7: // 4 + 4 NAND2 gates, 8 NAND3 gates
        num_L1_nand2 = 8;
        num_L1_nand3 = 8;
        num_L2 = 128;
        num_L1_active_nand2_path = 2;
        num_L1_active_nand3_path = 1;
        break;
      case 8: // 4 NAND2 gates, 8 + 8 NAND3 gates
        num_L1_nand2 = 4;
        num_L1_nand3 = 16;
        num_L2 = 256;
        num_L1_active_nand2_path = 2;
        num_L1_active_nand3_path = 2;
        break;
      case 9: // 8 + 8 + 8 NAND3 gates
        num_L1_nand3 = 24;
        num_L2 = 512;
        num_L1_active_nand2_path = 0;
        num_L1_active_nand3_path = 3;
        break;
      default:
        break;
    }

    for (int i = 1; i < number_gates_L1_nand2_path; ++i) {
      tot_area_L1_nand2 += compute_gate_area(
          INV, 1, w_L1_nand2_p[i], w_L1_nand2_n[i], g_tp.cell_h_def);
      leak_L1_nand2 += cmos_Isub_leakage(
          w_L1_nand2_n[i], w_L1_nand2_p[i], 2, nand, is_dram_);
      gate_leak_L1_nand2 +=
          cmos_Ig_leakage(w_L1_nand2_n[i], w_L1_nand2_p[i], 2, nand, is_dram_);
    }
    tot_area_L1_nand2 *= num_L1_nand2;
    leak_L1_nand2 *= num_L1_nand2;
    gate_leak_L1_nand2 *= num_L1_nand2;

    for (int i = 1; i < number_gates_L1_nand3_path; ++i) {
      tot_area_L1_nand3 += compute_gate_area(
          INV, 1, w_L1_nand3_p[i], w_L1_nand3_n[i], g_tp.cell_h_def);
      leak_L1_nand3 += cmos_Isub_leakage(
          w_L1_nand3_n[i], w_L1_nand3_p[i], 3, nand, is_dram_);
      gate_leak_L1_nand3 +=
          cmos_Ig_leakage(w_L1_nand3_n[i], w_L1_nand3_p[i], 3, nand, is_dram_);
    }
    tot_area_L1_nand3 *= num_L1_nand3;
    leak_L1_nand3 *= num_L1_nand3;
    gate_leak_L1_nand3 *= num_L1_nand3;

    double cumulative_area_L1 = tot_area_L1_nand2 + tot_area_L1_nand3;
    double cumulative_area_L2 = 0.0;
    double leakage_L2 = 0.0;
    double gate_leakage_L2 = 0.0;

    if (flag_L2_gate == 2) {
      cumulative_area_L2 =
          compute_gate_area(NAND, 2, w_L2_p[0], w_L2_n[0], g_tp.cell_h_def);
      leakage_L2 = cmos_Isub_leakage(w_L2_n[0], w_L2_p[0], 2, nand, is_dram_);
      gate_leakage_L2 =
          cmos_Ig_leakage(w_L2_n[0], w_L2_p[0], 2, nand, is_dram_);
    } else if (flag_L2_gate == 3) {
      cumulative_area_L2 =
          compute_gate_area(NAND, 3, w_L2_p[0], w_L2_n[0], g_tp.cell_h_def);
      leakage_L2 = cmos_Isub_leakage(w_L2_n[0], w_L2_p[0], 3, nand, is_dram_);
      gate_leakage_L2 =
          cmos_Ig_leakage(w_L2_n[0], w_L2_p[0], 3, nand, is_dram_);
    }

    for (int i = 1; i < number_gates_L2; ++i) {
      cumulative_area_L2 +=
          compute_gate_area(INV, 1, w_L2_p[i], w_L2_n[i], g_tp.cell_h_def);
      leakage_L2 += cmos_Isub_leakage(w_L2_n[i], w_L2_p[i], 2, inv, is_dram_);
      gate_leakage_L2 +=
          cmos_Ig_leakage(w_L2_n[i], w_L2_p[i], 2, inv, is_dram_);
    }
    cumulative_area_L2 *= num_L2;
    leakage_L2 *= num_L2;
    gate_leakage_L2 *= num_L2;

    power_nand2_path.readOp.leakage = leak_L1_nand2 * g_tp.peri_global.Vdd;
    power_nand3_path.readOp.leakage = leak_L1_nand3 * g_tp.peri_global.Vdd;
    power_L2.readOp.leakage = leakage_L2 * g_tp.peri_global.Vdd;

    power_nand2_path.readOp.power_gated_leakage =
        leak_L1_nand2 * g_tp.peri_global.Vcc_min;
    power_nand3_path.readOp.power_gated_leakage =
        leak_L1_nand3 * g_tp.peri_global.Vcc_min;
    power_L2.readOp.power_gated_leakage = leakage_L2 * g_tp.peri_global.Vcc_min;

    area.set_area(cumulative_area_L1 + cumulative_area_L2);
    power_nand2_path.readOp.gate_leakage =
        gate_leak_L1_nand2 * g_tp.peri_global.Vdd;
    power_nand3_path.readOp.gate_leakage =
        gate_leak_L1_nand3 * g_tp.peri_global.Vdd;
    power_L2.readOp.gate_leakage = gate_leakage_L2 * g_tp.peri_global.Vdd;
  }
}

pair<double, double>
PredecBlk::compute_delays(pair<double, double> inrisetime) // <nand2, nand3>
{
  pair<double, double> ret_val;
  ret_val.first = 0;  // outrisetime_nand2_path
  ret_val.second = 0; // outrisetime_nand3_path

  double inrisetime_nand2_path = inrisetime.first;
  double inrisetime_nand3_path = inrisetime.second;
  int i;
  double rd, c_load, c_intrinsic, tf, this_delay;
  double Vdd = g_tp.peri_global.Vdd;

  // TODO: following delay calculation part can be greatly simplified.
  // first check whether a predecoder block is required
  if (exist) {
    // Find delay in first level of predecoder block
    // First find delay in path
    if ((flag_two_unique_paths) || (number_inputs_L1_gate == 2)) {
      // First gate is a NAND2 gate
      rd = tr_R_on(w_L1_nand2_n[0], NCH, 2, is_dram_);
      c_load = gate_C(w_L1_nand2_n[1] + w_L1_nand2_p[1], 0.0, is_dram_);
      c_intrinsic =
          2 * drain_C_(w_L1_nand2_p[0], PCH, 1, 1, g_tp.cell_h_def, is_dram_) +
          drain_C_(w_L1_nand2_n[0], NCH, 2, 1, g_tp.cell_h_def, is_dram_);
      tf = rd * (c_intrinsic + c_load);
      this_delay = horowitz(inrisetime_nand2_path, tf, 0.5, 0.5, RISE);
      delay_nand2_path += this_delay;
      inrisetime_nand2_path = this_delay / (1.0 - 0.5);
      power_nand2_path.readOp.dynamic += (c_load + c_intrinsic) * Vdd * Vdd;

      // Add delays of all but the last inverter in the chain
      for (i = 1; i < number_gates_L1_nand2_path - 1; ++i) {
        rd = tr_R_on(w_L1_nand2_n[i], NCH, 1, is_dram_);
        c_load =
            gate_C(w_L1_nand2_n[i + 1] + w_L1_nand2_p[i + 1], 0.0, is_dram_);
        c_intrinsic =
            drain_C_(w_L1_nand2_p[i], PCH, 1, 1, g_tp.cell_h_def, is_dram_) +
            drain_C_(w_L1_nand2_n[i], NCH, 1, 1, g_tp.cell_h_def, is_dram_);
        tf = rd * (c_intrinsic + c_load);
        this_delay = horowitz(inrisetime_nand2_path, tf, 0.5, 0.5, RISE);
        delay_nand2_path += this_delay;
        inrisetime_nand2_path = this_delay / (1.0 - 0.5);
        power_nand2_path.readOp.dynamic += (c_intrinsic + c_load) * Vdd * Vdd;
      }

      // Add delay of the last inverter
      i = number_gates_L1_nand2_path - 1;
      rd = tr_R_on(w_L1_nand2_n[i], NCH, 1, is_dram_);
      if (flag_L2_gate) {
        c_load =
            branch_effort_nand2_gate_output *
            (gate_C(w_L2_n[0], 0, is_dram_) + gate_C(w_L2_p[0], 0, is_dram_));
        c_intrinsic =
            drain_C_(w_L1_nand2_p[i], PCH, 1, 1, g_tp.cell_h_def, is_dram_) +
            drain_C_(w_L1_nand2_n[i], NCH, 1, 1, g_tp.cell_h_def, is_dram_);
        tf = rd * (c_intrinsic + c_load);
        this_delay = horowitz(inrisetime_nand2_path, tf, 0.5, 0.5, RISE);
        delay_nand2_path += this_delay;
        inrisetime_nand2_path = this_delay / (1.0 - 0.5);
        power_nand2_path.readOp.dynamic += (c_intrinsic + c_load) * Vdd * Vdd;
      } else { // First level directly drives decoder output load
        c_load = C_ld_predec_blk_out;
        c_intrinsic =
            drain_C_(w_L1_nand2_p[i], PCH, 1, 1, g_tp.cell_h_def, is_dram_) +
            drain_C_(w_L1_nand2_n[i], NCH, 1, 1, g_tp.cell_h_def, is_dram_);
        tf = rd * (c_intrinsic + c_load) + R_wire_predec_blk_out * c_load / 2;
        this_delay = horowitz(inrisetime_nand2_path, tf, 0.5, 0.5, RISE);
        delay_nand2_path += this_delay;
        ret_val.first = this_delay / (1.0 - 0.5);
        power_nand2_path.readOp.dynamic += (c_intrinsic + c_load) * Vdd * Vdd;
      }
    }

    if ((flag_two_unique_paths) ||
        (number_inputs_L1_gate == 3)) { // Check if the number of gates in the
                                        // first level is more than 1.
      // First gate is a NAND3 gate
      rd = tr_R_on(w_L1_nand3_n[0], NCH, 3, is_dram_);
      c_load = gate_C(w_L1_nand3_n[1] + w_L1_nand3_p[1], 0.0, is_dram_);
      c_intrinsic =
          3 * drain_C_(w_L1_nand3_p[0], PCH, 1, 1, g_tp.cell_h_def, is_dram_) +
          drain_C_(w_L1_nand3_n[0], NCH, 3, 1, g_tp.cell_h_def, is_dram_);
      tf = rd * (c_intrinsic + c_load);
      this_delay = horowitz(inrisetime_nand3_path, tf, 0.5, 0.5, RISE);
      delay_nand3_path += this_delay;
      inrisetime_nand3_path = this_delay / (1.0 - 0.5);
      power_nand3_path.readOp.dynamic += (c_intrinsic + c_load) * Vdd * Vdd;

      // Add delays of all but the last inverter in the chain
      for (i = 1; i < number_gates_L1_nand3_path - 1; ++i) {
        rd = tr_R_on(w_L1_nand3_n[i], NCH, 1, is_dram_);
        c_load =
            gate_C(w_L1_nand3_n[i + 1] + w_L1_nand3_p[i + 1], 0.0, is_dram_);
        c_intrinsic =
            drain_C_(w_L1_nand3_p[i], PCH, 1, 1, g_tp.cell_h_def, is_dram_) +
            drain_C_(w_L1_nand3_n[i], NCH, 1, 1, g_tp.cell_h_def, is_dram_);
        tf = rd * (c_intrinsic + c_load);
        this_delay = horowitz(inrisetime_nand3_path, tf, 0.5, 0.5, RISE);
        delay_nand3_path += this_delay;
        inrisetime_nand3_path = this_delay / (1.0 - 0.5);
        power_nand3_path.readOp.dynamic += (c_intrinsic + c_load) * Vdd * Vdd;
      }

      // Add delay of the last inverter
      i = number_gates_L1_nand3_path - 1;
      rd = tr_R_on(w_L1_nand3_n[i], NCH, 1, is_dram_);
      if (flag_L2_gate) {
        c_load =
            branch_effort_nand3_gate_output *
            (gate_C(w_L2_n[0], 0, is_dram_) + gate_C(w_L2_p[0], 0, is_dram_));
        c_intrinsic =
            drain_C_(w_L1_nand3_p[i], PCH, 1, 1, g_tp.cell_h_def, is_dram_) +
            drain_C_(w_L1_nand3_n[i], NCH, 1, 1, g_tp.cell_h_def, is_dram_);
        tf = rd * (c_intrinsic + c_load);
        this_delay = horowitz(inrisetime_nand3_path, tf, 0.5, 0.5, RISE);
        delay_nand3_path += this_delay;
        inrisetime_nand3_path = this_delay / (1.0 - 0.5);
        power_nand3_path.readOp.dynamic += (c_intrinsic + c_load) * Vdd * Vdd;
      } else { // First level directly drives decoder output load
        c_load = C_ld_predec_blk_out;
        c_intrinsic =
            drain_C_(w_L1_nand3_p[i], PCH, 1, 1, g_tp.cell_h_def, is_dram_) +
            drain_C_(w_L1_nand3_n[i], NCH, 1, 1, g_tp.cell_h_def, is_dram_);
        tf = rd * (c_intrinsic + c_load) + R_wire_predec_blk_out * c_load / 2;
        this_delay = horowitz(inrisetime_nand3_path, tf, 0.5, 0.5, RISE);
        delay_nand3_path += this_delay;
        ret_val.second = this_delay / (1.0 - 0.5);
        power_nand3_path.readOp.dynamic += (c_intrinsic + c_load) * Vdd * Vdd;
      }
    }

    // Find delay through second level
    if (flag_L2_gate) {
      if (flag_L2_gate == 2) {
        rd = tr_R_on(w_L2_n[0], NCH, 2, is_dram_);
        c_load = gate_C(w_L2_n[1] + w_L2_p[1], 0.0, is_dram_);
        c_intrinsic =
            2 * drain_C_(w_L2_p[0], PCH, 1, 1, g_tp.cell_h_def, is_dram_) +
            drain_C_(w_L2_n[0], NCH, 2, 1, g_tp.cell_h_def, is_dram_);
        tf = rd * (c_intrinsic + c_load);
        this_delay = horowitz(inrisetime_nand2_path, tf, 0.5, 0.5, RISE);
        delay_nand2_path += this_delay;
        inrisetime_nand2_path = this_delay / (1.0 - 0.5);
        power_L2.readOp.dynamic += (c_intrinsic + c_load) * Vdd * Vdd;
      } else { // flag_L2_gate = 3
        rd = tr_R_on(w_L2_n[0], NCH, 3, is_dram_);
        c_load = gate_C(w_L2_n[1] + w_L2_p[1], 0.0, is_dram_);
        c_intrinsic =
            3 * drain_C_(w_L2_p[0], PCH, 1, 1, g_tp.cell_h_def, is_dram_) +
            drain_C_(w_L2_n[0], NCH, 3, 1, g_tp.cell_h_def, is_dram_);
        tf = rd * (c_intrinsic + c_load);
        this_delay = horowitz(inrisetime_nand3_path, tf, 0.5, 0.5, RISE);
        delay_nand3_path += this_delay;
        inrisetime_nand3_path = this_delay / (1.0 - 0.5);
        power_L2.readOp.dynamic += (c_intrinsic + c_load) * Vdd * Vdd;
      }

      for (i = 1; i < number_gates_L2 - 1; ++i) {
        rd = tr_R_on(w_L2_n[i], NCH, 1, is_dram_);
        c_load = gate_C(w_L2_n[i + 1] + w_L2_p[i + 1], 0.0, is_dram_);
        c_intrinsic =
            drain_C_(w_L2_p[i], PCH, 1, 1, g_tp.cell_h_def, is_dram_) +
            drain_C_(w_L2_n[i], NCH, 1, 1, g_tp.cell_h_def, is_dram_);
        tf = rd * (c_intrinsic + c_load);
        this_delay = horowitz(inrisetime_nand2_path, tf, 0.5, 0.5, RISE);
        delay_nand2_path += this_delay;
        inrisetime_nand2_path = this_delay / (1.0 - 0.5);
        this_delay = horowitz(inrisetime_nand3_path, tf, 0.5, 0.5, RISE);
        delay_nand3_path += this_delay;
        inrisetime_nand3_path = this_delay / (1.0 - 0.5);
        power_L2.readOp.dynamic += (c_intrinsic + c_load) * Vdd * Vdd;
      }

      // Add delay of final inverter that drives the wordline decoders
      i = number_gates_L2 - 1;
      c_load = C_ld_predec_blk_out;
      rd = tr_R_on(w_L2_n[i], NCH, 1, is_dram_);
      c_intrinsic = drain_C_(w_L2_p[i], PCH, 1, 1, g_tp.cell_h_def, is_dram_) +
                    drain_C_(w_L2_n[i], NCH, 1, 1, g_tp.cell_h_def, is_dram_);
      tf = rd * (c_intrinsic + c_load) + R_wire_predec_blk_out * c_load / 2;
      this_delay = horowitz(inrisetime_nand2_path, tf, 0.5, 0.5, RISE);
      delay_nand2_path += this_delay;
      ret_val.first = this_delay / (1.0 - 0.5);
      this_delay = horowitz(inrisetime_nand3_path, tf, 0.5, 0.5, RISE);
      delay_nand3_path += this_delay;
      ret_val.second = this_delay / (1.0 - 0.5);
      power_L2.readOp.dynamic += (c_intrinsic + c_load) * Vdd * Vdd;
    }
  }

  delay = (ret_val.first > ret_val.second) ? ret_val.first : ret_val.second;
  return ret_val;
}

void PredecBlk::leakage_feedback(double temperature) {
  if (exist) { // First check whether a predecoder block is needed
    int num_L1_nand2 = 0;
    int num_L1_nand3 = 0;
    int num_L2 = 0;
    double leak_L1_nand3 = 0;
    double gate_leak_L1_nand3 = 0;

    double leak_L1_nand2 =
        cmos_Isub_leakage(w_L1_nand2_n[0], w_L1_nand2_p[0], 2, nand, is_dram_);
    double gate_leak_L1_nand2 =
        cmos_Ig_leakage(w_L1_nand2_n[0], w_L1_nand2_p[0], 2, nand, is_dram_);
    if (number_inputs_L1_gate != 3) {
      leak_L1_nand3 = 0;
      gate_leak_L1_nand3 = 0;
    } else {
      leak_L1_nand3 =
          cmos_Isub_leakage(w_L1_nand3_n[0], w_L1_nand3_p[0], 3, nand);
      gate_leak_L1_nand3 =
          cmos_Ig_leakage(w_L1_nand3_n[0], w_L1_nand3_p[0], 3, nand);
    }

    switch (number_input_addr_bits) {
      case 1: // 2 NAND2 gates
        num_L1_nand2 = 2;
        num_L2 = 0;
        num_L1_active_nand2_path = 1;
        num_L1_active_nand3_path = 0;
        break;
      case 2: // 4 NAND2 gates
        num_L1_nand2 = 4;
        num_L2 = 0;
        num_L1_active_nand2_path = 1;
        num_L1_active_nand3_path = 0;
        break;
      case 3: // 8 NAND3 gates
        num_L1_nand3 = 8;
        num_L2 = 0;
        num_L1_active_nand2_path = 0;
        num_L1_active_nand3_path = 1;
        break;
      case 4: // 4 + 4 NAND2 gates
        num_L1_nand2 = 8;
        num_L2 = 16;
        num_L1_active_nand2_path = 2;
        num_L1_active_nand3_path = 0;
        break;
      case 5: // 4 NAND2 gates, 8 NAND3 gates
        num_L1_nand2 = 4;
        num_L1_nand3 = 8;
        num_L2 = 32;
        num_L1_active_nand2_path = 1;
        num_L1_active_nand3_path = 1;
        break;
      case 6: // 8 + 8 NAND3 gates
        num_L1_nand3 = 16;
        num_L2 = 64;
        num_L1_active_nand2_path = 0;
        num_L1_active_nand3_path = 2;
        break;
      case 7: // 4 + 4 NAND2 gates, 8 NAND3 gates
        num_L1_nand2 = 8;
        num_L1_nand3 = 8;
        num_L2 = 128;
        num_L1_active_nand2_path = 2;
        num_L1_active_nand3_path = 1;
        break;
      case 8: // 4 NAND2 gates, 8 + 8 NAND3 gates
        num_L1_nand2 = 4;
        num_L1_nand3 = 16;
        num_L2 = 256;
        num_L1_active_nand2_path = 2;
        num_L1_active_nand3_path = 2;
        break;
      case 9: // 8 + 8 + 8 NAND3 gates
        num_L1_nand3 = 24;
        num_L2 = 512;
        num_L1_active_nand2_path = 0;
        num_L1_active_nand3_path = 3;
        break;
      default:
        break;
    }

    for (int i = 1; i < number_gates_L1_nand2_path; ++i) {
      leak_L1_nand2 += cmos_Isub_leakage(
          w_L1_nand2_n[i], w_L1_nand2_p[i], 2, nand, is_dram_);
      gate_leak_L1_nand2 +=
          cmos_Ig_leakage(w_L1_nand2_n[i], w_L1_nand2_p[i], 2, nand, is_dram_);
    }
    leak_L1_nand2 *= num_L1_nand2;
    gate_leak_L1_nand2 *= num_L1_nand2;

    for (int i = 1; i < number_gates_L1_nand3_path; ++i) {
      leak_L1_nand3 += cmos_Isub_leakage(
          w_L1_nand3_n[i], w_L1_nand3_p[i], 3, nand, is_dram_);
      gate_leak_L1_nand3 +=
          cmos_Ig_leakage(w_L1_nand3_n[i], w_L1_nand3_p[i], 3, nand, is_dram_);
    }
    leak_L1_nand3 *= num_L1_nand3;
    gate_leak_L1_nand3 *= num_L1_nand3;

    double leakage_L2 = 0.0;
    double gate_leakage_L2 = 0.0;

    if (flag_L2_gate == 2) {
      leakage_L2 = cmos_Isub_leakage(w_L2_n[0], w_L2_p[0], 2, nand, is_dram_);
      gate_leakage_L2 =
          cmos_Ig_leakage(w_L2_n[0], w_L2_p[0], 2, nand, is_dram_);
    } else if (flag_L2_gate == 3) {
      leakage_L2 = cmos_Isub_leakage(w_L2_n[0], w_L2_p[0], 3, nand, is_dram_);
      gate_leakage_L2 =
          cmos_Ig_leakage(w_L2_n[0], w_L2_p[0], 3, nand, is_dram_);
    }

    for (int i = 1; i < number_gates_L2; ++i) {
      leakage_L2 += cmos_Isub_leakage(w_L2_n[i], w_L2_p[i], 2, inv, is_dram_);
      gate_leakage_L2 +=
          cmos_Ig_leakage(w_L2_n[i], w_L2_p[i], 2, inv, is_dram_);
    }
    leakage_L2 *= num_L2;
    gate_leakage_L2 *= num_L2;

    power_nand2_path.readOp.leakage = leak_L1_nand2 * g_tp.peri_global.Vdd;
    power_nand3_path.readOp.leakage = leak_L1_nand3 * g_tp.peri_global.Vdd;
    power_L2.readOp.leakage = leakage_L2 * g_tp.peri_global.Vdd;

    power_nand2_path.readOp.gate_leakage =
        gate_leak_L1_nand2 * g_tp.peri_global.Vdd;
    power_nand3_path.readOp.gate_leakage =
        gate_leak_L1_nand3 * g_tp.peri_global.Vdd;
    power_L2.readOp.gate_leakage = gate_leakage_L2 * g_tp.peri_global.Vdd;
  }
}
