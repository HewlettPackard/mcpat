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

#ifndef __DRIVER_H__
#define __DRIVER_H__

#include "area.h"
#include "component.h"
#include "parameter.h"
#include "powergating.h"

#include <boost/serialization/assume_abstract.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/utility.hpp>
#include <vector>

class Driver : public Component {
public:
  Driver(double c_gate_load_,
         double c_wire_load_,
         double r_wire_load_,
         bool is_dram,
         bool power_gating_ = false,
         int nodes_DSTN_ = 1);

  int number_gates;
  int min_number_gates;
  double width_n[MAX_NUMBER_GATES_STAGE];
  double width_p[MAX_NUMBER_GATES_STAGE];
  double c_gate_load;
  double c_wire_load;
  double r_wire_load;
  double delay;
  //  powerDef power;
  bool is_dram_;

  double total_driver_nwidth;
  double total_driver_pwidth;
  Sleep_tx *sleeptx;

  int nodes_DSTN;
  bool power_gating;

  void compute_widths();
  void compute_area();
  double compute_delay(double inrisetime);

  void compute_power_gating();

  ~Driver() {
    if (sleeptx != 0)
      delete sleeptx;
  };

private:
  // Serialization
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive &ar, const unsigned int version) {
    ar &number_gates;
    ar &min_number_gates;
    ar &width_n;
    ar &width_p;
    ar &c_gate_load;
    ar &c_wire_load;
    ar &r_wire_load;
    ar &delay;
    ar &is_dram_;
    ar &total_driver_nwidth;
    ar &total_driver_pwidth;
    ar &nodes_DSTN;
    ar &power_gating;
    Component::serialize(ar, version);
  }
};

#endif // __DRIVER_H__
