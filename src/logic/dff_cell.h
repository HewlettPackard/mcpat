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
#ifndef __DFF_CELL_H__
#define __DFF_CELL_H__

#include "XML_Parse.h"
#include "arch_const.h"
#include "basic_circuit.h"
#include "basic_components.h"
#include "cacti_interface.h"
#include "component.h"
#include "const.h"
#include "decoder.h"
#include "parameter.h"
#include "xmlParser.h"

#include <boost/serialization/assume_abstract.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/utility.hpp>
#include <cassert>
#include <cmath>
#include <cstring>
#include <iostream>

class DFFCell : public Component {
public:
  DFFCell(bool _is_dram,
          double _WdecNANDn,
          double _WdecNANDp,
          double _cell_load,
          const InputParameter *configure_interface);
  InputParameter l_ip;
  bool is_dram;
  double cell_load;
  double WdecNANDn;
  double WdecNANDp;
  double clock_cap;
  int model;
  int n_switch;
  int n_keep_1;
  int n_keep_0;
  int n_clock;
  powerDef e_switch;
  powerDef e_keep_1;
  powerDef e_keep_0;
  powerDef e_clock;

  double fpfp_node_cap(unsigned int fan_in, unsigned int fan_out);
  void compute_DFF_cell(void);

private:
  // Serialization
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive &ar, const unsigned int version) {
    ar &is_dram;
    ar &cell_load;
    ar &WdecNANDn;
    ar &WdecNANDp;
    ar &clock_cap;
    ar &model;
    ar &n_switch;
    ar &n_keep_1;
    ar &n_keep_0;
    ar &n_clock;
    ar &e_switch;
    ar &e_keep_1;
    ar &e_keep_0;
    ar &e_clock;
    Component::serialize(ar, version);
  }
};

#endif //__DFF_CELL_H__
