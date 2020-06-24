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

#ifndef __DECODER_H__
#define __DECODER_H__

#include "area.h"
#include "component.h"
#include "parameter.h"
#include "powergating.h"

#include <boost/serialization/assume_abstract.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/utility.hpp>
#include <vector>

using namespace std;

class Decoder : public Component {
public:
  Decoder(int _num_dec_signals,
          bool flag_way_select,
          double _C_ld_dec_out,
          double _R_wire_dec_out,
          bool fully_assoc_,
          bool is_dram_,
          bool is_wl_tr_,
          const Area &cell_,
          bool power_gating_ = false,
          int nodes_DSTN_ = 1);
  Decoder(){};
  void set_params(int _num_dec_signals,
                  bool flag_way_select,
                  double _C_ld_dec_out,
                  double _R_wire_dec_out,
                  bool fully_assoc_,
                  bool is_dram_,
                  bool is_wl_tr_,
                  const Area &cell_,
                  bool power_gating_ = false,
                  int nodes_DSTN_ = 1);
  bool exist;
  int num_in_signals;
  double C_ld_dec_out;
  double R_wire_dec_out;
  int num_gates;
  int num_gates_min;
  double w_dec_n[MAX_NUMBER_GATES_STAGE];
  double w_dec_p[MAX_NUMBER_GATES_STAGE];
  double delay;
  // powerDef power;
  bool fully_assoc;
  bool is_dram;
  bool is_wl_tr;

  double height;
  double total_driver_nwidth;
  double total_driver_pwidth;
  Sleep_tx *sleeptx;

  int nodes_DSTN;
  bool power_gating;

  void computeArea();
  void compute_widths();
  void compute_area();
  double compute_delays(double inrisetime); // return outrisetime
  void compute_power_gating();

  void leakage_feedback(double temperature);

  ~Decoder() {
    if (sleeptx != 0)
      delete sleeptx;
  };

private:
  // Serialization
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive &ar, const unsigned int version) {
    ar &exist;
    ar &num_in_signals;
    ar &C_ld_dec_out;
    ar &R_wire_dec_out;
    ar &num_gates;
    ar &num_gates_min;
    ar &w_dec_n;
    ar &w_dec_p;
    ar &delay;
    ar &fully_assoc;
    ar &is_dram;
    ar &is_wl_tr;

    ar &height;
    ar &total_driver_nwidth;
    ar &total_driver_pwidth;

    ar &nodes_DSTN;
    ar &power_gating;
    Component::serialize(ar, version);
  }
};

#endif
