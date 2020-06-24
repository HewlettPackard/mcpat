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

#ifndef __PREDEC_BLK_H__
#define __PREDEC_BLK_H__

#include "area.h"
#include "component.h"
#include "decoder.h"
#include "parameter.h"
#include "powergating.h"

#include <boost/serialization/assume_abstract.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/utility.hpp>
#include <vector>

class PredecBlk : public Component {
public:
  PredecBlk(){};
  PredecBlk(int num_dec_signals,
            Decoder *dec,
            double C_wire_predec_blk_out,
            double R_wire_predec_blk_out,
            int num_dec_per_predec,
            bool is_dram_,
            bool is_blk1);
  void set_params(int num_dec_signals,
                  Decoder *dec,
                  double C_wire_predec_blk_out,
                  double R_wire_predec_blk_out,
                  int num_dec_per_predec,
                  bool is_dram_,
                  bool is_blk1);

  Decoder *dec;
  bool exist;
  int number_input_addr_bits;
  double C_ld_predec_blk_out;
  double R_wire_predec_blk_out;
  int branch_effort_nand2_gate_output;
  int branch_effort_nand3_gate_output;
  bool flag_two_unique_paths;
  int flag_L2_gate;
  int number_inputs_L1_gate;
  int number_gates_L1_nand2_path;
  int number_gates_L1_nand3_path;
  int number_gates_L2;
  int min_number_gates_L1;
  int min_number_gates_L2;
  int num_L1_active_nand2_path;
  int num_L1_active_nand3_path;
  double w_L1_nand2_n[MAX_NUMBER_GATES_STAGE];
  double w_L1_nand2_p[MAX_NUMBER_GATES_STAGE];
  double w_L1_nand3_n[MAX_NUMBER_GATES_STAGE];
  double w_L1_nand3_p[MAX_NUMBER_GATES_STAGE];
  double w_L2_n[MAX_NUMBER_GATES_STAGE];
  double w_L2_p[MAX_NUMBER_GATES_STAGE];
  double delay_nand2_path;
  double delay_nand3_path;
  powerDef power_nand2_path;
  powerDef power_nand3_path;
  powerDef power_L2;

  bool is_dram_;

  void compute_widths();
  void compute_area();

  void leakage_feedback(double temperature);

  pair<double, double>
  compute_delays(pair<double, double> inrisetime); // <nand2, nand3>
  // return <outrise_nand2, outrise_nand3>

private:
  // Serialization
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive &ar, const unsigned int version) {
    Component::serialize(ar, version);
  }
};

#endif //__PREDEC_BLK_H__
