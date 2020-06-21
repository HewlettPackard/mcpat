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

#ifndef __SHAREDCACHE_H__
#define __SHAREDCACHE_H__

#include "XML_Parse.h"
#include "area.h"
#include "array.h"
#include "basic_components.h"
#include "cache_param.h"
#include "datacache.h"
#include "parameter.h"

#include <vector>

class SharedCache : public Component {
public:
  InputParameter interface_ip;
  DataCache unicache; // Shared cache
  CacheDynParam cachep;
  statsDef homenode_tdp_stats;
  statsDef homenode_rtp_stats;
  statsDef homenode_stats_t;

  SharedCache();
  void set_params(const ParseXML *XML,
                  const int ithCache_,
                  InputParameter *interface_ip_,
                  const enum cache_level cacheL_ = L2);
  void set_stats(const ParseXML *XML);
  void computeArea();
  void computeStaticPower(bool is_tdp = false);
  void computeDynamicPower();
  void display(uint32_t indent = 0, bool enable = true);
  ~SharedCache(){};

private:
  const ParseXML *XML;
  int ithCache;
  enum cache_level cacheL;
  double dir_overhead;
  double scktRatio;
  double executionTime;

  bool long_channel;
  bool power_gating;
  bool init_params;
  bool init_stats;
  bool set_area;
  bool debug;
  bool is_default;

  double size;
  double line;
  double assoc;
  double banks;

  enum Device_ty device_t;
  enum Core_type core_t;
};

#endif /* SHAREDCACHE_H_ */
