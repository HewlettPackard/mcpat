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

#ifndef __CACHE_PARAM_H__
#define __CACHE_PARAM_H__

#include "XML_Parse.h"
#include "basic_components.h"
#include "parameter.h"

#include <vector>

enum cache_level { L2, L3, L1Directory, L2Directory };

class CacheDynParam {
public:
  string name;
  enum Dir_type dir_ty;
  double clockRate;
  double executionTime;
  double capacity;
  double blockW;
  double assoc;
  double nbanks;
  double throughput;
  double latency;
  double duty_cycle;
  double dir_duty_cycle;
  // double duty_cycle;
  int missb_size;
  int fu_size;
  int prefetchb_size;
  int wbb_size;
  double vdd;
  double power_gating_vcc;
  CacheDynParam(){};
  ~CacheDynParam(){};
  void set_params_l2_cache(const ParseXML *XML, const int ithCache);
  void set_params_l3_cache(const ParseXML *XML, const int ithCache);
  void set_params_l1_directory(const ParseXML *XML, const int ithCache);
  void set_params_l2_directory(const ParseXML *XML, const int ithCache);
};

#endif
