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

#ifndef NOC_H_
#define NOC_H_

#include "XML_Parse.h"
#include "array.h"
#include "basic_components.h"
#include "interconnect.h"
#include "parameter.h"
#include "router.h"

#include <boost/serialization/assume_abstract.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/utility.hpp>

class NoC : public Component {
public:
  int ithNoC;
  InputParameter interface_ip;
  double link_len;
  double executionTime;
  double scktRatio;
  double chip_PR_overhead;
  double macro_PR_overhead;
  Router router;
  interconnect link_bus;
  NoCParam nocdynp;
  uca_org_t local_result;
  statsDef tdp_stats;
  statsDef rtp_stats;
  statsDef stats_t;
  powerDef power_t;
  Component link_bus_tot_per_Router;
  bool link_bus_exist;
  bool router_exist;
  string name, link_name;
  double M_traffic_pattern;
  NoC();
  void set_params(const ParseXML *XML,
                  int ithNoC_,
                  InputParameter *interface_ip_,
                  double M_traffic_pattern_ = 0.6,
                  double link_len_ = 0);
  void set_stats(const ParseXML *XML);
  void computeArea();
  void computePower(bool cp = false);
  void computeRuntimeDynamicPower();
  void init_link_bus(double link_len_);
  void display(uint32_t indent = 0, int plevel = 100, bool is_tdp = true);
  // TODO
  void computeEnergy_link_bus(bool is_tdp = true);
  void displayEnergy_link_bus(uint32_t indent = 0,
                              int plevel = 100,
                              bool is_tdp = true);
  ~NoC();

private:
  bool embedded;
  bool init_stats;
  bool init_params;
  bool set_area;
  bool long_channel;
  bool power_gating;

  unsigned int total_accesses;

  void set_noc_param(const ParseXML *XML);
  void init_router();

  // Serialization
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive &ar, const unsigned int version) {
    ar &name;
    ar &link_name;
    ar &router;
    ar &link_bus;
    ar &router_exist;
    ar &link_bus_exist;
    ar &link_bus_tot_per_Router;
    // ar &link_bus_tot_per_Router.area;
    ar &Component::area;
    // Component::serialize(ar, version);
  }
};

#endif /* NOC_H_ */
