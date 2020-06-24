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

#include "driver.h"

#include "area.h"
#include "parameter.h"

#include <assert.h>
#include <iostream>
#include <math.h>

Driver::Driver(double c_gate_load_,
               double c_wire_load_,
               double r_wire_load_,
               bool is_dram,
               bool power_gating_,
               int nodes_DSTN_)
    : number_gates(0), min_number_gates(2), c_gate_load(c_gate_load_),
      c_wire_load(c_wire_load_), r_wire_load(r_wire_load_), delay(0),
      //  power(),
      is_dram_(is_dram), total_driver_nwidth(0), total_driver_pwidth(0),
      sleeptx(NULL), nodes_DSTN(nodes_DSTN_), power_gating(power_gating_) {
  for (int i = 0; i < MAX_NUMBER_GATES_STAGE; i++) {
    width_n[i] = 0;
    width_p[i] = 0;
  }

  compute_widths();
  compute_area();
}

void Driver::compute_widths() {
  double p_to_n_sz_ratio = pmos_to_nmos_sz_ratio(is_dram_);
  double c_load = c_gate_load + c_wire_load;
  width_n[0] = g_tp.min_w_nmos_;
  width_p[0] = p_to_n_sz_ratio * g_tp.min_w_nmos_;

  double F = c_load / gate_C(width_n[0] + width_p[0], 0, is_dram_);
  number_gates = logical_effort(min_number_gates,
                                1,
                                F,
                                width_n,
                                width_p,
                                c_load,
                                p_to_n_sz_ratio,
                                is_dram_,
                                false,
                                g_tp.max_w_nmos_);
}

void Driver::compute_area() {
  double cumulative_area = 0;

  area.h = g_tp.cell_h_def;
  for (int i = 0; i < number_gates; i++) {
    cumulative_area +=
        compute_gate_area(INV, 1, width_p[i], width_n[i], area.h);
  }
  area.w = (cumulative_area / area.h);
  if (power_gating) {
    compute_power_gating();
    cumulative_area += sleeptx->area.get_area();
    area.w = (cumulative_area / area.h);
  }
}

void Driver::compute_power_gating() {
  // For all driver chains there is only one sleep transistors to save area
  // Total transistor width for sleep tx calculation
  for (int i = 0; i < number_gates; i++) {
    total_driver_nwidth += width_n[i];
    total_driver_pwidth += width_p[i];
  }

  // compute sleep tx
  bool is_footer = false;
  double Isat_subarray = simplified_nmos_Isat(total_driver_nwidth);
  double detalV;
  double c_wakeup;

  c_wakeup = drain_C_(total_driver_pwidth, PCH, 1, 1, area.h); // Psleep tx
  detalV = g_tp.peri_global.Vdd - g_tp.peri_global.Vcc_min;
  //    if (g_ip->power_gating)
  sleeptx = new Sleep_tx(g_ip->perfloss,
                         Isat_subarray,
                         is_footer,
                         c_wakeup,
                         detalV,
                         nodes_DSTN, // default is 1 for drivers
                         area);
}

double Driver::compute_delay(double inrisetime) {
  int i;
  double rd, c_load, c_intrinsic, tf;
  double this_delay = 0;

  for (i = 0; i < number_gates - 1; ++i) {
    rd = tr_R_on(width_n[i], NCH, 1, is_dram_);
    c_load = gate_C(width_n[i + 1] + width_p[i + 1], 0.0, is_dram_);
    c_intrinsic = drain_C_(width_p[i], PCH, 1, 1, g_tp.cell_h_def, is_dram_) +
                  drain_C_(width_n[i], NCH, 1, 1, g_tp.cell_h_def, is_dram_);
    tf = rd * (c_intrinsic + c_load);
    this_delay = horowitz(inrisetime, tf, 0.5, 0.5, RISE);
    delay += this_delay;
    inrisetime = this_delay / (1.0 - 0.5);
    power.readOp.dynamic +=
        (c_intrinsic + c_load) * g_tp.peri_global.Vdd * g_tp.peri_global.Vdd;
    power.readOp.leakage +=
        cmos_Isub_leakage(width_n[i], width_p[i], 1, inv, is_dram_) *
        g_tp.peri_global.Vdd;
    power.readOp.power_gated_leakage +=
        cmos_Isub_leakage(width_n[i], width_p[i], 1, inv, is_dram_) *
        g_tp.peri_global.Vcc_min;
    power.readOp.gate_leakage +=
        cmos_Ig_leakage(width_n[i], width_p[i], 1, inv, is_dram_) *
        g_tp.peri_global.Vdd;
  }

  i = number_gates - 1;
  c_load = c_gate_load + c_wire_load;
  rd = tr_R_on(width_n[i], NCH, 1, is_dram_);
  c_intrinsic = drain_C_(width_p[i], PCH, 1, 1, g_tp.cell_h_def, is_dram_) +
                drain_C_(width_n[i], NCH, 1, 1, g_tp.cell_h_def, is_dram_);
  tf = rd * (c_intrinsic + c_load) +
       r_wire_load * (c_wire_load / 2 + c_gate_load);
  this_delay = horowitz(inrisetime, tf, 0.5, 0.5, RISE);
  delay += this_delay;
  power.readOp.dynamic +=
      (c_intrinsic + c_load) * g_tp.peri_global.Vdd * g_tp.peri_global.Vdd;
  power.readOp.leakage +=
      cmos_Isub_leakage(width_n[i], width_p[i], 1, inv, is_dram_) *
      g_tp.peri_global.Vdd;
  power.readOp.power_gated_leakage +=
      cmos_Isub_leakage(width_n[i], width_p[i], 1, inv, is_dram_) *
      g_tp.peri_global.Vcc_min;
  power.readOp.gate_leakage +=
      cmos_Ig_leakage(width_n[i], width_p[i], 1, inv, is_dram_) *
      g_tp.peri_global.Vdd;

  return this_delay / (1.0 - 0.5);
}
