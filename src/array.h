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

#ifndef __ARRAY_H__
#define __ARRAY_H__

#include "basic_components.h"
#include "cacti_interface.h"
#include "component.h"
#include "const.h"
#include "parameter.h"

#include <boost/serialization/assume_abstract.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/utility.hpp>
#include <iostream>
#include <string>

using namespace std;

class ArrayST : public Component {
public:
  ArrayST(){};
  ArrayST(const InputParameter *configure_interface,
          string _name,
          enum Device_ty device_ty_,
          bool opt_local_ = true,
          enum Core_type core_ty_ = Inorder,
          bool _is_default = true);

  InputParameter l_ip;
  string name;
  enum Device_ty device_ty;
  bool opt_local;
  enum Core_type core_ty;
  bool is_default;
  uca_org_t local_result;

  statsDef tdp_stats;
  statsDef rtp_stats;
  statsDef stats_t;
  powerDef power_t;

  virtual void set_params(const InputParameter *configure_interface,
                          string _name,
                          enum Device_ty device_ty_,
                          bool opt_local_ = true,
                          enum Core_type core_ty_ = Inorder,
                          bool _is_default = true);
  virtual void computeArea();
  virtual ~ArrayST();

protected:
  virtual void optimize_array();
  virtual void compute_base_power();
  void leakage_feedback(double temperature);

  // Serialization
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive &ar, const unsigned int version) {
    ar &name;
    ar &device_ty;
    ar &opt_local;
    ar &core_ty;
    ar &is_default;
    ar &tdp_stats;
    ar &rtp_stats;
    ar &stats_t;
    ar &power_t;
    Component::serialize(ar, version);
  }
};

#endif /* __ARRAY_H__ */
