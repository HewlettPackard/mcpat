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

#ifndef __PREDEC_BLK_DRV_H__
#define __PREDEC_BLK_DRV_H__

#include "area.h"
#include "component.h"
#include "decoder.h"
#include "parameter.h"
#include "powergating.h"
#include "predec_blk.h"

#include <boost/serialization/assume_abstract.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/utility.hpp>
#include <vector>

class PredecBlkDrv : public Component {
public:
  void set_params(int way_select_, PredecBlk *blk_, bool is_dram);
  PredecBlkDrv(){};
  PredecBlkDrv(int way_select_, PredecBlk *blk_, bool is_dram);

  int flag_driver_exists;
  int number_input_addr_bits;
  int number_gates_nand2_path;
  int number_gates_nand3_path;
  int min_number_gates;
  int num_buffers_driving_1_nand2_load;
  int num_buffers_driving_2_nand2_load;
  int num_buffers_driving_4_nand2_load;
  int num_buffers_driving_2_nand3_load;
  int num_buffers_driving_8_nand3_load;
  int num_buffers_nand3_path;
  double c_load_nand2_path_out;
  double c_load_nand3_path_out;
  double r_load_nand2_path_out;
  double r_load_nand3_path_out;
  double width_nand2_path_n[MAX_NUMBER_GATES_STAGE];
  double width_nand2_path_p[MAX_NUMBER_GATES_STAGE];
  double width_nand3_path_n[MAX_NUMBER_GATES_STAGE];
  double width_nand3_path_p[MAX_NUMBER_GATES_STAGE];
  double delay_nand2_path;
  double delay_nand3_path;
  powerDef power_nand2_path;
  powerDef power_nand3_path;

  PredecBlk *blk;
  Decoder *dec;
  bool is_dram_;
  int way_select;

  void compute_widths();
  void compute_area();

  void leakage_feedback(double temperature);

  pair<double, double> compute_delays(
      double inrisetime_nand2_path,
      double inrisetime_nand3_path); // return <outrise_nand2, outrise_nand3>

  inline int num_addr_bits_nand2_path() {
    return num_buffers_driving_1_nand2_load + num_buffers_driving_2_nand2_load +
           num_buffers_driving_4_nand2_load;
  }
  inline int num_addr_bits_nand3_path() {
    return num_buffers_driving_2_nand3_load + num_buffers_driving_8_nand3_load;
  }
  double get_rdOp_dynamic_E(int num_act_mats_hor_dir);

private:
  // Serialization
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive &ar, const unsigned int version) {
    Component::serialize(ar, version);
  }
};

#endif // __PREDEC_BLK_DRV_H__
