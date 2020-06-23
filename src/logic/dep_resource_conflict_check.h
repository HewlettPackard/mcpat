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
#ifndef __DEP_RESOURCE_CONFLICT_CHECK_H__
#define __DEP_RESOURCE_CONFLICT_CHECK_H__

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

#include <cassert>
#include <cmath>
#include <cstring>
#include <iostream>

class dep_resource_conflict_check : public Component {
public:
  dep_resource_conflict_check(){};

  dep_resource_conflict_check(const InputParameter *configure_interface,
                              const CoreDynParam &dyn_p_,
                              int compare_bits_,
                              bool _is_default = true);
  InputParameter l_ip;
  uca_org_t local_result;
  double WNORn, WNORp, Wevalinvp, Wevalinvn, Wcompn, Wcompp, Wcomppreequ;
  CoreDynParam coredynp;
  int compare_bits;
  bool is_default;
  statsDef tdp_stats;
  statsDef rtp_stats;
  statsDef stats_t;
  powerDef power_t;

  void set_params(const InputParameter *configure_interface,
                  const CoreDynParam &dyn_p_,
                  int compare_bits_,
                  bool _is_default = true);

  void conflict_check_power();
  double compare_cap();
  ~dep_resource_conflict_check() { local_result.cleanup(); }

  void leakage_feedback(double temperature);
};

#endif // __DEP_RESOURCE_CONFLICT_CHECK_H__
