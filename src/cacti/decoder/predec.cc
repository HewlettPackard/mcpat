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

#include "predec.h"

#include "area.h"
#include "parameter.h"

#include <assert.h>
#include <iostream>
#include <math.h>

void Predec::set_params(PredecBlkDrv *drv1_, PredecBlkDrv *drv2_) {
  blk1 = drv1_->blk;
  blk2 = drv2_->blk;
  drv1 = drv1_;
  drv2 = drv2_;

  driver_power.readOp.leakage = drv1->power_nand2_path.readOp.leakage +
                                drv1->power_nand3_path.readOp.leakage +
                                drv2->power_nand2_path.readOp.leakage +
                                drv2->power_nand3_path.readOp.leakage;
  block_power.readOp.leakage =
      blk1->power_nand2_path.readOp.leakage +
      blk1->power_nand3_path.readOp.leakage + blk1->power_L2.readOp.leakage +
      blk2->power_nand2_path.readOp.leakage +
      blk2->power_nand3_path.readOp.leakage + blk2->power_L2.readOp.leakage;

  driver_power.readOp.power_gated_leakage =
      drv1->power_nand2_path.readOp.power_gated_leakage +
      drv1->power_nand3_path.readOp.power_gated_leakage +
      drv2->power_nand2_path.readOp.power_gated_leakage +
      drv2->power_nand3_path.readOp.power_gated_leakage;
  block_power.readOp.power_gated_leakage =
      blk1->power_nand2_path.readOp.power_gated_leakage +
      blk1->power_nand3_path.readOp.power_gated_leakage +
      blk1->power_L2.readOp.power_gated_leakage +
      blk2->power_nand2_path.readOp.power_gated_leakage +
      blk2->power_nand3_path.readOp.power_gated_leakage +
      blk2->power_L2.readOp.power_gated_leakage;

  power.readOp.leakage =
      driver_power.readOp.leakage + block_power.readOp.leakage;

  power.readOp.power_gated_leakage = driver_power.readOp.power_gated_leakage +
                                     block_power.readOp.power_gated_leakage;

  driver_power.readOp.gate_leakage =
      drv1->power_nand2_path.readOp.gate_leakage +
      drv1->power_nand3_path.readOp.gate_leakage +
      drv2->power_nand2_path.readOp.gate_leakage +
      drv2->power_nand3_path.readOp.gate_leakage;
  block_power.readOp.gate_leakage = blk1->power_nand2_path.readOp.gate_leakage +
                                    blk1->power_nand3_path.readOp.gate_leakage +
                                    blk1->power_L2.readOp.gate_leakage +
                                    blk2->power_nand2_path.readOp.gate_leakage +
                                    blk2->power_nand3_path.readOp.gate_leakage +
                                    blk2->power_L2.readOp.gate_leakage;
  power.readOp.gate_leakage =
      driver_power.readOp.gate_leakage + block_power.readOp.gate_leakage;
}

Predec::Predec(PredecBlkDrv *drv1_, PredecBlkDrv *drv2_)
    : blk1(drv1_->blk), blk2(drv2_->blk), drv1(drv1_), drv2(drv2_) {
  driver_power.readOp.leakage = drv1->power_nand2_path.readOp.leakage +
                                drv1->power_nand3_path.readOp.leakage +
                                drv2->power_nand2_path.readOp.leakage +
                                drv2->power_nand3_path.readOp.leakage;
  block_power.readOp.leakage =
      blk1->power_nand2_path.readOp.leakage +
      blk1->power_nand3_path.readOp.leakage + blk1->power_L2.readOp.leakage +
      blk2->power_nand2_path.readOp.leakage +
      blk2->power_nand3_path.readOp.leakage + blk2->power_L2.readOp.leakage;

  driver_power.readOp.power_gated_leakage =
      drv1->power_nand2_path.readOp.power_gated_leakage +
      drv1->power_nand3_path.readOp.power_gated_leakage +
      drv2->power_nand2_path.readOp.power_gated_leakage +
      drv2->power_nand3_path.readOp.power_gated_leakage;
  block_power.readOp.power_gated_leakage =
      blk1->power_nand2_path.readOp.power_gated_leakage +
      blk1->power_nand3_path.readOp.power_gated_leakage +
      blk1->power_L2.readOp.power_gated_leakage +
      blk2->power_nand2_path.readOp.power_gated_leakage +
      blk2->power_nand3_path.readOp.power_gated_leakage +
      blk2->power_L2.readOp.power_gated_leakage;

  power.readOp.leakage =
      driver_power.readOp.leakage + block_power.readOp.leakage;

  power.readOp.power_gated_leakage = driver_power.readOp.power_gated_leakage +
                                     block_power.readOp.power_gated_leakage;

  driver_power.readOp.gate_leakage =
      drv1->power_nand2_path.readOp.gate_leakage +
      drv1->power_nand3_path.readOp.gate_leakage +
      drv2->power_nand2_path.readOp.gate_leakage +
      drv2->power_nand3_path.readOp.gate_leakage;
  block_power.readOp.gate_leakage = blk1->power_nand2_path.readOp.gate_leakage +
                                    blk1->power_nand3_path.readOp.gate_leakage +
                                    blk1->power_L2.readOp.gate_leakage +
                                    blk2->power_nand2_path.readOp.gate_leakage +
                                    blk2->power_nand3_path.readOp.gate_leakage +
                                    blk2->power_L2.readOp.gate_leakage;
  power.readOp.gate_leakage =
      driver_power.readOp.gate_leakage + block_power.readOp.gate_leakage;
}

double Predec::compute_delays(double inrisetime) {
  // TODO: Jung Ho thinks that predecoder block driver locates between decoder
  // and predecoder block.
  pair<double, double> tmp_pair1, tmp_pair2;
  tmp_pair1 = drv1->compute_delays(inrisetime, inrisetime);
  tmp_pair1 = blk1->compute_delays(tmp_pair1);
  tmp_pair2 = drv2->compute_delays(inrisetime, inrisetime);
  tmp_pair2 = blk2->compute_delays(tmp_pair2);
  tmp_pair1 = get_max_delay_before_decoder(tmp_pair1, tmp_pair2);

  driver_power.readOp.dynamic =
      drv1->num_addr_bits_nand2_path() * drv1->power_nand2_path.readOp.dynamic +
      drv1->num_addr_bits_nand3_path() * drv1->power_nand3_path.readOp.dynamic +
      drv2->num_addr_bits_nand2_path() * drv2->power_nand2_path.readOp.dynamic +
      drv2->num_addr_bits_nand3_path() * drv2->power_nand3_path.readOp.dynamic;

  block_power.readOp.dynamic =
      blk1->power_nand2_path.readOp.dynamic * blk1->num_L1_active_nand2_path +
      blk1->power_nand3_path.readOp.dynamic * blk1->num_L1_active_nand3_path +
      blk1->power_L2.readOp.dynamic +
      blk2->power_nand2_path.readOp.dynamic * blk1->num_L1_active_nand2_path +
      blk2->power_nand3_path.readOp.dynamic * blk1->num_L1_active_nand3_path +
      blk2->power_L2.readOp.dynamic;

  power.readOp.dynamic =
      driver_power.readOp.dynamic + block_power.readOp.dynamic;

  delay = tmp_pair1.first;
  return tmp_pair1.second;
}

void Predec::leakage_feedback(double temperature) {
  drv1->leakage_feedback(temperature);
  drv2->leakage_feedback(temperature);
  blk1->leakage_feedback(temperature);
  blk2->leakage_feedback(temperature);

  driver_power.readOp.leakage = drv1->power_nand2_path.readOp.leakage +
                                drv1->power_nand3_path.readOp.leakage +
                                drv2->power_nand2_path.readOp.leakage +
                                drv2->power_nand3_path.readOp.leakage;
  block_power.readOp.leakage =
      blk1->power_nand2_path.readOp.leakage +
      blk1->power_nand3_path.readOp.leakage + blk1->power_L2.readOp.leakage +
      blk2->power_nand2_path.readOp.leakage +
      blk2->power_nand3_path.readOp.leakage + blk2->power_L2.readOp.leakage;
  power.readOp.leakage =
      driver_power.readOp.leakage + block_power.readOp.leakage;

  driver_power.readOp.gate_leakage =
      drv1->power_nand2_path.readOp.gate_leakage +
      drv1->power_nand3_path.readOp.gate_leakage +
      drv2->power_nand2_path.readOp.gate_leakage +
      drv2->power_nand3_path.readOp.gate_leakage;
  block_power.readOp.gate_leakage = blk1->power_nand2_path.readOp.gate_leakage +
                                    blk1->power_nand3_path.readOp.gate_leakage +
                                    blk1->power_L2.readOp.gate_leakage +
                                    blk2->power_nand2_path.readOp.gate_leakage +
                                    blk2->power_nand3_path.readOp.gate_leakage +
                                    blk2->power_L2.readOp.gate_leakage;
  power.readOp.gate_leakage =
      driver_power.readOp.gate_leakage + block_power.readOp.gate_leakage;
}

// returns <delay, risetime>
pair<double, double>
Predec::get_max_delay_before_decoder(pair<double, double> input_pair1,
                                     pair<double, double> input_pair2) {
  pair<double, double> ret_val;
  double delay;

  delay = drv1->delay_nand2_path + blk1->delay_nand2_path;
  ret_val.first = delay;
  ret_val.second = input_pair1.first;
  delay = drv1->delay_nand3_path + blk1->delay_nand3_path;
  if (ret_val.first < delay) {
    ret_val.first = delay;
    ret_val.second = input_pair1.second;
  }
  delay = drv2->delay_nand2_path + blk2->delay_nand2_path;
  if (ret_val.first < delay) {
    ret_val.first = delay;
    ret_val.second = input_pair2.first;
  }
  delay = drv2->delay_nand3_path + blk2->delay_nand3_path;
  if (ret_val.first < delay) {
    ret_val.first = delay;
    ret_val.second = input_pair2.second;
  }

  return ret_val;
}
