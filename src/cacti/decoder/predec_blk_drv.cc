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

#include "predec_blk_drv.h"

#include "area.h"
#include "decoder.h"
#include "parameter.h"
#include "predec_blk.h"

#include <assert.h>
#include <iostream>
#include <math.h>

void PredecBlkDrv::set_params(int way_select_, PredecBlk *blk_, bool is_dram) {
  flag_driver_exists = 0;
  number_gates_nand2_path = 0;
  number_gates_nand3_path = 0;
  min_number_gates = 2;
  num_buffers_driving_1_nand2_load = 0;
  num_buffers_driving_2_nand2_load = 0;
  num_buffers_driving_4_nand2_load = 0;
  num_buffers_driving_2_nand3_load = 0;
  num_buffers_driving_8_nand3_load = 0;
  num_buffers_nand3_path = 0;
  c_load_nand2_path_out = 0;
  c_load_nand3_path_out = 0;
  r_load_nand2_path_out = 0;
  r_load_nand3_path_out = 0;
  delay_nand2_path = 0;
  delay_nand3_path = 0;
  blk = blk_;
  dec = blk->dec;
  is_dram_ = is_dram;
  way_select = way_select_;
  for (int i = 0; i < MAX_NUMBER_GATES_STAGE; i++) {
    width_nand2_path_n[i] = 0;
    width_nand2_path_p[i] = 0;
    width_nand3_path_n[i] = 0;
    width_nand3_path_p[i] = 0;
  }

  number_input_addr_bits = blk->number_input_addr_bits;

  if (way_select > 1) {
    flag_driver_exists = 1;
    number_input_addr_bits = way_select;
    if (dec->num_in_signals == 2) {
      c_load_nand2_path_out =
          gate_C(dec->w_dec_n[0] + dec->w_dec_p[0], 0, is_dram_);
      num_buffers_driving_2_nand2_load = number_input_addr_bits;
    } else if (dec->num_in_signals == 3) {
      c_load_nand3_path_out =
          gate_C(dec->w_dec_n[0] + dec->w_dec_p[0], 0, is_dram_);
      num_buffers_driving_2_nand3_load = number_input_addr_bits;
    }
  } else if (way_select == 0) {
    if (blk->exist) {
      flag_driver_exists = 1;
    }
  }

  compute_widths();
  compute_area();
}

PredecBlkDrv::PredecBlkDrv(int way_select_, PredecBlk *blk_, bool is_dram)
    : flag_driver_exists(0), number_gates_nand2_path(0),
      number_gates_nand3_path(0), min_number_gates(2),
      num_buffers_driving_1_nand2_load(0), num_buffers_driving_2_nand2_load(0),
      num_buffers_driving_4_nand2_load(0), num_buffers_driving_2_nand3_load(0),
      num_buffers_driving_8_nand3_load(0), num_buffers_nand3_path(0),
      c_load_nand2_path_out(0), c_load_nand3_path_out(0),
      r_load_nand2_path_out(0), r_load_nand3_path_out(0), delay_nand2_path(0),
      delay_nand3_path(0), power_nand2_path(), power_nand3_path(), blk(blk_),
      dec(blk->dec), is_dram_(is_dram), way_select(way_select_) {
  for (int i = 0; i < MAX_NUMBER_GATES_STAGE; i++) {
    width_nand2_path_n[i] = 0;
    width_nand2_path_p[i] = 0;
    width_nand3_path_n[i] = 0;
    width_nand3_path_p[i] = 0;
  }

  number_input_addr_bits = blk->number_input_addr_bits;

  if (way_select > 1) {
    flag_driver_exists = 1;
    number_input_addr_bits = way_select;
    if (dec->num_in_signals == 2) {
      c_load_nand2_path_out =
          gate_C(dec->w_dec_n[0] + dec->w_dec_p[0], 0, is_dram_);
      num_buffers_driving_2_nand2_load = number_input_addr_bits;
    } else if (dec->num_in_signals == 3) {
      c_load_nand3_path_out =
          gate_C(dec->w_dec_n[0] + dec->w_dec_p[0], 0, is_dram_);
      num_buffers_driving_2_nand3_load = number_input_addr_bits;
    }
  } else if (way_select == 0) {
    if (blk->exist) {
      flag_driver_exists = 1;
    }
  }

  compute_widths();
  compute_area();
}

void PredecBlkDrv::compute_widths() {
  // The predecode block driver accepts as input the address bits from the
  // h-tree network. For each addr bit it then generates addr and addrbar as
  // outputs. For now ignore the effect of inversion to generate addrbar and
  // simply treat addrbar as addr.

  double F;
  double p_to_n_sz_ratio = pmos_to_nmos_sz_ratio(is_dram_);

  if (flag_driver_exists) {
    double C_nand2_gate_blk =
        gate_C(blk->w_L1_nand2_n[0] + blk->w_L1_nand2_p[0], 0, is_dram_);
    double C_nand3_gate_blk =
        gate_C(blk->w_L1_nand3_n[0] + blk->w_L1_nand3_p[0], 0, is_dram_);

    if (way_select == 0) {
      if (blk->number_input_addr_bits == 1) { // 2 NAND2 gates
        num_buffers_driving_2_nand2_load = 1;
        c_load_nand2_path_out = 2 * C_nand2_gate_blk;
      } else if (blk->number_input_addr_bits ==
                 2) { // 4 NAND2 gates  one 2-4 decoder
        num_buffers_driving_4_nand2_load = 2;
        c_load_nand2_path_out = 4 * C_nand2_gate_blk;
      } else if (blk->number_input_addr_bits ==
                 3) { // 8 NAND3 gates  one 3-8 decoder
        num_buffers_driving_8_nand3_load = 3;
        c_load_nand3_path_out = 8 * C_nand3_gate_blk;
      } else if (blk->number_input_addr_bits ==
                 4) { // 4 + 4 NAND2 gates two 2-4 decoder
        num_buffers_driving_4_nand2_load = 4;
        c_load_nand2_path_out = 4 * C_nand2_gate_blk;
      } else if (blk->number_input_addr_bits ==
                 5) { // 4 NAND2 gates, 8 NAND3 gates one 2-4 decoder and one
                      // 3-8 decoder
        num_buffers_driving_4_nand2_load = 2;
        num_buffers_driving_8_nand3_load = 3;
        c_load_nand2_path_out = 4 * C_nand2_gate_blk;
        c_load_nand3_path_out = 8 * C_nand3_gate_blk;
      } else if (blk->number_input_addr_bits ==
                 6) { // 8 + 8 NAND3 gates two 3-8 decoder
        num_buffers_driving_8_nand3_load = 6;
        c_load_nand3_path_out = 8 * C_nand3_gate_blk;
      } else if (blk->number_input_addr_bits ==
                 7) { // 4 + 4 NAND2 gates, 8 NAND3 gates two 2-4 decoder and
                      // one 3-8 decoder
        num_buffers_driving_4_nand2_load = 4;
        num_buffers_driving_8_nand3_load = 3;
        c_load_nand2_path_out = 4 * C_nand2_gate_blk;
        c_load_nand3_path_out = 8 * C_nand3_gate_blk;
      } else if (blk->number_input_addr_bits ==
                 8) { // 4 NAND2 gates, 8 + 8 NAND3 gates one 2-4 decoder and
                      // two 3-8 decoder
        num_buffers_driving_4_nand2_load = 2;
        num_buffers_driving_8_nand3_load = 6;
        c_load_nand2_path_out = 4 * C_nand2_gate_blk;
        c_load_nand3_path_out = 8 * C_nand3_gate_blk;
      } else if (blk->number_input_addr_bits ==
                 9) { // 8 + 8 + 8 NAND3 gates three 3-8 decoder
        num_buffers_driving_8_nand3_load = 9;
        c_load_nand3_path_out = 8 * C_nand3_gate_blk;
      }
    }

    if ((blk->flag_two_unique_paths) || (blk->number_inputs_L1_gate == 2) ||
        (number_input_addr_bits == 0) ||
        ((way_select) &&
         (dec->num_in_signals ==
          2))) { // this means that way_select is driving NAND2 in decoder.
      width_nand2_path_n[0] = g_tp.min_w_nmos_;
      width_nand2_path_p[0] = p_to_n_sz_ratio * width_nand2_path_n[0];
      F = c_load_nand2_path_out /
          gate_C(width_nand2_path_n[0] + width_nand2_path_p[0], 0, is_dram_);
      number_gates_nand2_path = logical_effort(min_number_gates,
                                               1,
                                               F,
                                               width_nand2_path_n,
                                               width_nand2_path_p,
                                               c_load_nand2_path_out,
                                               p_to_n_sz_ratio,
                                               is_dram_,
                                               false,
                                               g_tp.max_w_nmos_);
    }

    if ((blk->flag_two_unique_paths) || (blk->number_inputs_L1_gate == 3) ||
        ((way_select) &&
         (dec->num_in_signals ==
          3))) { // this means that way_select is driving NAND3 in decoder.
      width_nand3_path_n[0] = g_tp.min_w_nmos_;
      width_nand3_path_p[0] = p_to_n_sz_ratio * width_nand3_path_n[0];
      F = c_load_nand3_path_out /
          gate_C(width_nand3_path_n[0] + width_nand3_path_p[0], 0, is_dram_);
      number_gates_nand3_path = logical_effort(min_number_gates,
                                               1,
                                               F,
                                               width_nand3_path_n,
                                               width_nand3_path_p,
                                               c_load_nand3_path_out,
                                               p_to_n_sz_ratio,
                                               is_dram_,
                                               false,
                                               g_tp.max_w_nmos_);
    }
  }
}

void PredecBlkDrv::compute_area() {
  double area_nand2_path = 0;
  double area_nand3_path = 0;
  double leak_nand2_path = 0;
  double leak_nand3_path = 0;
  double gate_leak_nand2_path = 0;
  double gate_leak_nand3_path = 0;

  if (flag_driver_exists) { // first check whether a predecoder block driver is
                            // needed
    for (int i = 0; i < number_gates_nand2_path; ++i) {
      area_nand2_path += compute_gate_area(INV,
                                           1,
                                           width_nand2_path_p[i],
                                           width_nand2_path_n[i],
                                           g_tp.cell_h_def);
      leak_nand2_path += cmos_Isub_leakage(
          width_nand2_path_n[i], width_nand2_path_p[i], 1, inv, is_dram_);
      gate_leak_nand2_path += cmos_Ig_leakage(
          width_nand2_path_n[i], width_nand2_path_p[i], 1, inv, is_dram_);
    }
    area_nand2_path *=
        (num_buffers_driving_1_nand2_load + num_buffers_driving_2_nand2_load +
         num_buffers_driving_4_nand2_load);
    leak_nand2_path *=
        (num_buffers_driving_1_nand2_load + num_buffers_driving_2_nand2_load +
         num_buffers_driving_4_nand2_load);
    gate_leak_nand2_path *=
        (num_buffers_driving_1_nand2_load + num_buffers_driving_2_nand2_load +
         num_buffers_driving_4_nand2_load);

    for (int i = 0; i < number_gates_nand3_path; ++i) {
      area_nand3_path += compute_gate_area(INV,
                                           1,
                                           width_nand3_path_p[i],
                                           width_nand3_path_n[i],
                                           g_tp.cell_h_def);
      leak_nand3_path += cmos_Isub_leakage(
          width_nand3_path_n[i], width_nand3_path_p[i], 1, inv, is_dram_);
      gate_leak_nand3_path += cmos_Ig_leakage(
          width_nand3_path_n[i], width_nand3_path_p[i], 1, inv, is_dram_);
    }
    area_nand3_path *=
        (num_buffers_driving_2_nand3_load + num_buffers_driving_8_nand3_load);
    leak_nand3_path *=
        (num_buffers_driving_2_nand3_load + num_buffers_driving_8_nand3_load);
    gate_leak_nand3_path *=
        (num_buffers_driving_2_nand3_load + num_buffers_driving_8_nand3_load);

    power_nand2_path.readOp.leakage = leak_nand2_path * g_tp.peri_global.Vdd;
    power_nand3_path.readOp.leakage = leak_nand3_path * g_tp.peri_global.Vdd;
    power_nand2_path.readOp.power_gated_leakage =
        leak_nand2_path * g_tp.peri_global.Vcc_min;
    power_nand3_path.readOp.power_gated_leakage =
        leak_nand3_path * g_tp.peri_global.Vcc_min;
    power_nand2_path.readOp.gate_leakage =
        gate_leak_nand2_path * g_tp.peri_global.Vdd;
    power_nand3_path.readOp.gate_leakage =
        gate_leak_nand3_path * g_tp.peri_global.Vdd;
    area.set_area(area_nand2_path + area_nand3_path);
  }
}

pair<double, double>
PredecBlkDrv::compute_delays(double inrisetime_nand2_path,
                             double inrisetime_nand3_path) {
  pair<double, double> ret_val;
  ret_val.first = 0;  // outrisetime_nand2_path
  ret_val.second = 0; // outrisetime_nand3_path
  int i;
  double rd, c_gate_load, c_load, c_intrinsic, tf, this_delay;
  double Vdd = g_tp.peri_global.Vdd;

  if (flag_driver_exists) {
    for (i = 0; i < number_gates_nand2_path - 1; ++i) {
      rd = tr_R_on(width_nand2_path_n[i], NCH, 1, is_dram_);
      c_gate_load = gate_C(
          width_nand2_path_p[i + 1] + width_nand2_path_n[i + 1], 0.0, is_dram_);
      c_intrinsic =
          drain_C_(
              width_nand2_path_p[i], PCH, 1, 1, g_tp.cell_h_def, is_dram_) +
          drain_C_(width_nand2_path_n[i], NCH, 1, 1, g_tp.cell_h_def, is_dram_);
      tf = rd * (c_intrinsic + c_gate_load);
      this_delay = horowitz(inrisetime_nand2_path, tf, 0.5, 0.5, RISE);
      delay_nand2_path += this_delay;
      inrisetime_nand2_path = this_delay / (1.0 - 0.5);
      power_nand2_path.readOp.dynamic +=
          (c_gate_load + c_intrinsic) * 0.5 * Vdd * Vdd;
    }

    // Final inverter drives the predecoder block or the decoder output load
    if (number_gates_nand2_path != 0) {
      i = number_gates_nand2_path - 1;
      rd = tr_R_on(width_nand2_path_n[i], NCH, 1, is_dram_);
      c_intrinsic =
          drain_C_(
              width_nand2_path_p[i], PCH, 1, 1, g_tp.cell_h_def, is_dram_) +
          drain_C_(width_nand2_path_n[i], NCH, 1, 1, g_tp.cell_h_def, is_dram_);
      c_load = c_load_nand2_path_out;
      tf = rd * (c_intrinsic + c_load) + r_load_nand2_path_out * c_load / 2;
      this_delay = horowitz(inrisetime_nand2_path, tf, 0.5, 0.5, RISE);
      delay_nand2_path += this_delay;
      ret_val.first = this_delay / (1.0 - 0.5);
      power_nand2_path.readOp.dynamic +=
          (c_intrinsic + c_load) * 0.5 * Vdd * Vdd;
      //      cout<< "c_intrinsic = " << c_intrinsic << "c_load" << c_load
      //      <<endl;
    }

    for (i = 0; i < number_gates_nand3_path - 1; ++i) {
      rd = tr_R_on(width_nand3_path_n[i], NCH, 1, is_dram_);
      c_gate_load = gate_C(
          width_nand3_path_p[i + 1] + width_nand3_path_n[i + 1], 0.0, is_dram_);
      c_intrinsic =
          drain_C_(
              width_nand3_path_p[i], PCH, 1, 1, g_tp.cell_h_def, is_dram_) +
          drain_C_(width_nand3_path_n[i], NCH, 1, 1, g_tp.cell_h_def, is_dram_);
      tf = rd * (c_intrinsic + c_gate_load);
      this_delay = horowitz(inrisetime_nand3_path, tf, 0.5, 0.5, RISE);
      delay_nand3_path += this_delay;
      inrisetime_nand3_path = this_delay / (1.0 - 0.5);
      power_nand3_path.readOp.dynamic +=
          (c_gate_load + c_intrinsic) * 0.5 * Vdd * Vdd;
    }

    // Final inverter drives the predecoder block or the decoder output load
    if (number_gates_nand3_path != 0) {
      i = number_gates_nand3_path - 1;
      rd = tr_R_on(width_nand3_path_n[i], NCH, 1, is_dram_);
      c_intrinsic =
          drain_C_(
              width_nand3_path_p[i], PCH, 1, 1, g_tp.cell_h_def, is_dram_) +
          drain_C_(width_nand3_path_n[i], NCH, 1, 1, g_tp.cell_h_def, is_dram_);
      c_load = c_load_nand3_path_out;
      tf = rd * (c_intrinsic + c_load) + r_load_nand3_path_out * c_load / 2;
      this_delay = horowitz(inrisetime_nand3_path, tf, 0.5, 0.5, RISE);
      delay_nand3_path += this_delay;
      ret_val.second = this_delay / (1.0 - 0.5);
      power_nand3_path.readOp.dynamic +=
          (c_intrinsic + c_load) * 0.5 * Vdd * Vdd;
    }
  }
  return ret_val;
}

double PredecBlkDrv::get_rdOp_dynamic_E(int num_act_mats_hor_dir) {
  return (num_addr_bits_nand2_path() * power_nand2_path.readOp.dynamic +
          num_addr_bits_nand3_path() * power_nand3_path.readOp.dynamic) *
         num_act_mats_hor_dir;
}

void PredecBlkDrv::leakage_feedback(double temperature) {
  double leak_nand2_path = 0;
  double leak_nand3_path = 0;
  double gate_leak_nand2_path = 0;
  double gate_leak_nand3_path = 0;

  if (flag_driver_exists) { // first check whether a predecoder block driver is
                            // needed
    for (int i = 0; i < number_gates_nand2_path; ++i) {
      leak_nand2_path += cmos_Isub_leakage(
          width_nand2_path_n[i], width_nand2_path_p[i], 1, inv, is_dram_);
      gate_leak_nand2_path += cmos_Ig_leakage(
          width_nand2_path_n[i], width_nand2_path_p[i], 1, inv, is_dram_);
    }
    leak_nand2_path *=
        (num_buffers_driving_1_nand2_load + num_buffers_driving_2_nand2_load +
         num_buffers_driving_4_nand2_load);
    gate_leak_nand2_path *=
        (num_buffers_driving_1_nand2_load + num_buffers_driving_2_nand2_load +
         num_buffers_driving_4_nand2_load);

    for (int i = 0; i < number_gates_nand3_path; ++i) {
      leak_nand3_path += cmos_Isub_leakage(
          width_nand3_path_n[i], width_nand3_path_p[i], 1, inv, is_dram_);
      gate_leak_nand3_path += cmos_Ig_leakage(
          width_nand3_path_n[i], width_nand3_path_p[i], 1, inv, is_dram_);
    }
    leak_nand3_path *=
        (num_buffers_driving_2_nand3_load + num_buffers_driving_8_nand3_load);
    gate_leak_nand3_path *=
        (num_buffers_driving_2_nand3_load + num_buffers_driving_8_nand3_load);

    power_nand2_path.readOp.leakage = leak_nand2_path * g_tp.peri_global.Vdd;
    power_nand3_path.readOp.leakage = leak_nand3_path * g_tp.peri_global.Vdd;
    power_nand2_path.readOp.gate_leakage =
        gate_leak_nand2_path * g_tp.peri_global.Vdd;
    power_nand3_path.readOp.gate_leakage =
        gate_leak_nand3_path * g_tp.peri_global.Vdd;
  }
}
