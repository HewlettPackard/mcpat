
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

#include "inst_decoder.h"

void inst_decoder::set_params(bool _is_default,
                              const InputParameter *configure_interface,
                              int opcode_length_,
                              int num_decoders_,
                              bool x86_,
                              enum Device_ty device_ty_,
                              enum Core_type core_ty_) {
  /*
   * Instruction decoder is different from n to 2^n decoders
   * that are commonly used in row decoders in memory arrays.
   * The RISC instruction decoder is typically a very simple device.
   * We can decode an instruction by simply
   * separating the machine word into small parts using wire slices
   * The RISC instruction decoder can be approximate by the n to 2^n decoders,
   * although this approximation usually underestimate power since each decoded
   * instruction normally has more than 1 active signal.
   *
   * However, decoding a CISC instruction word is much more difficult
   * than the RISC case. A CISC decoder is typically set up as a state machine.
   * The machine reads the opcode field to determine
   * what type of instruction it is,
   * and where the other data values are.
   * The instruction word is read in piece by piece,
   * and decisions are made at each stage as to
   * how the remainder of the instruction word will be read.
   * (sequencer and ROM are usually needed)
   * An x86 decoder can be even more complex since
   * it involve  both decoding instructions into u-ops and
   * merge u-ops when doing micro-ops fusion.
   */
  is_default = _is_default;
  opcode_length = opcode_length_;
  num_decoders = num_decoders_;
  x86 = x86_;
  device_ty = device_ty_;
  core_ty = core_ty_;
  bool is_dram = false;
  double pmos_to_nmos_sizing_r;
  double load_nmos_width, load_pmos_width;
  double C_driver_load, R_wire_load;
  Area cell;

  l_ip = *configure_interface;
  local_result = init_interface(&l_ip);
  cell.h = g_tp.cell_h_def;
  cell.w = g_tp.cell_h_def;

  num_decoder_segments = (int)ceil(opcode_length / 18.0);
  if (opcode_length > 18)
    opcode_length = 18;
  num_decoded_signals = (int)pow(2.0, opcode_length);
  pmos_to_nmos_sizing_r = pmos_to_nmos_sz_ratio();
  load_nmos_width = g_tp.max_w_nmos_ / 2;
  load_pmos_width = g_tp.max_w_nmos_ * pmos_to_nmos_sizing_r;
  C_driver_load =
      1024 * gate_C(load_nmos_width + load_pmos_width,
                    0,
                    is_dram); // TODO: this number 1024 needs to be revisited
  R_wire_load = 3000 * l_ip.F_sz_um * g_tp.wire_outside_mat.R_per_um;

  final_dec.set_params(num_decoded_signals,
                       false,
                       C_driver_load,
                       R_wire_load,
                       false /*is_fa*/,
                       false /*is_dram*/,
                       false /*wl_tr*/, // to use peri device
                       cell);
  final_dec.computeArea();

  predec_blk1.set_params(num_decoded_signals,
                         &final_dec,
                         0, // Assuming predec and dec are back to back
                         0,
                         1, // Each Predec only drives one final dec
                         false /*is_dram*/,
                         true);

  predec_blk2.set_params(num_decoded_signals,
                         &final_dec,
                         0, // Assuming predec and dec are back to back
                         0,
                         1, // Each Predec only drives one final dec
                         false /*is_dram*/,
                         false);

  predec_blk_drv1.set_params(0, &predec_blk1, false);

  predec_blk_drv2.set_params(0, &predec_blk2, false);

  pre_dec.set_params(&predec_blk_drv1, &predec_blk_drv2);
  init_params = true;
}

void inst_decoder::computeArea() {
  if (!init_params) {
    std::cerr << "[ Inst_decoder ] Error: must set params before calling "
                 "computeArea()\n";

    exit(1);
  }
  double area_decoder = final_dec.area.get_area() * num_decoded_signals *
                        num_decoder_segments * num_decoders;
  // double w_decoder    = area_decoder / area.get_h();
  double area_pre_dec =
      (predec_blk_drv1.area.get_area() + predec_blk_drv2.area.get_area() +
       predec_blk1.area.get_area() + predec_blk2.area.get_area()) *
      num_decoder_segments * num_decoders;
  area.set_area(area.get_area() + area_decoder + area_pre_dec);
  double macro_layout_overhead = g_tp.macro_layout_overhead;
  double chip_PR_overhead = g_tp.chip_layout_overhead;
  area.set_area(area.get_area() * macro_layout_overhead * chip_PR_overhead);
}

void inst_decoder::computeDynamicPower() {
  inst_decoder_delay_power();

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

void inst_decoder::inst_decoder_delay_power() {

  double dec_outrisetime;
  double inrisetime = 0, outrisetime;
  double pppm_t[4] = {1, 1, 1, 1};
  double squencer_passes = x86 ? 2 : 1;

  outrisetime = pre_dec.compute_delays(inrisetime);
  dec_outrisetime = final_dec.compute_delays(outrisetime);
  set_pppm(pppm_t,
           squencer_passes * num_decoder_segments,
           num_decoder_segments,
           squencer_passes * num_decoder_segments,
           num_decoder_segments);
  power = power + pre_dec.power * pppm_t;
  set_pppm(pppm_t,
           squencer_passes * num_decoder_segments,
           num_decoder_segments * num_decoded_signals,
           num_decoder_segments * num_decoded_signals,
           squencer_passes * num_decoder_segments);
  power = power + final_dec.power * pppm_t;
}

void inst_decoder::leakage_feedback(double temperature) {
  l_ip.temp = (unsigned int)round(temperature / 10.0) * 10;
  uca_org_t init_result = init_interface(&l_ip); // init_result is dummy

  final_dec.leakage_feedback(temperature);
  pre_dec.leakage_feedback(temperature);

  double pppm_t[4] = {1, 1, 1, 1};
  double squencer_passes = x86 ? 2 : 1;

  set_pppm(pppm_t,
           squencer_passes * num_decoder_segments,
           num_decoder_segments,
           squencer_passes * num_decoder_segments,
           num_decoder_segments);
  power = pre_dec.power * pppm_t;

  set_pppm(pppm_t,
           squencer_passes * num_decoder_segments,
           num_decoder_segments * num_decoded_signals,
           num_decoder_segments * num_decoded_signals,
           squencer_passes * num_decoder_segments);
  power = power + final_dec.power * pppm_t;

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

inst_decoder::~inst_decoder() {
  local_result.cleanup();

  delete pre_dec.blk1;
  delete pre_dec.blk2;
  delete pre_dec.drv1;
  delete pre_dec.drv2;
}
