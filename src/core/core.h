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

#ifndef CORE_H_
#define CORE_H_

#include "XML_Parse.h"
#include "array.h"
#include "basic_components.h"
#include "branch_predictor.h"
#include "exec_unit.h"
#include "instfetch.h"
#include "interconnect.h"
#include "loadstore.h"
#include "mmu.h"
#include "parameter.h"
#include "pipeline.h"
#include "renaming_unit.h"
#include "sharedcache.h"
#include "undiff_core.h"

class Core : public Component {
public:
  const ParseXML *XML;
  int ithCore;
  InputParameter interface_ip;
  double clockRate;
  double executionTime;
  double scktRatio;
  double chip_PR_overhead;
  double macro_PR_overhead;
  InstFetchU ifu;
  LoadStoreU lsu;
  MemManU mmu;
  EXECU exu;
  RENAMINGU rnu;
  Pipeline corepipe;
  UndiffCore undiffCore;
  SharedCache l2cache;
  CoreDynParam coredynp;

  double pipeline_area_per_unit;

  // full_decoder 	inst_decoder;
  // clock_network	clockNetwork;
  Core(){};
  void set_params(const ParseXML *XML_interface,
                  int ithCore_,
                  InputParameter *interface_ip_,
                  bool cp = false);
  void computeArea();
  void set_stats(const ParseXML *XML_interface);
  void set_core_param();
  void computeDynamicPower(bool is_tdp = true);
  void displayEnergy(uint32_t indent = 0, int plevel = 100, bool is_tdp = true);
  ~Core();

  // Serialization
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive &ar, const unsigned int version) {
    ar &clockRate;
    ar &executionTime;
    ar &clockRate;
    ar &executionTime;
    ar &scktRatio;
    ar &chip_PR_overhead;
    ar &macro_PR_overhead;
    ar &ifu;
    ar &lsu;
    ar &mmu;
    ar &exu;
    ar &rnu;
    ar &corepipe;
    ar &undiffCore;
    ar &l2cache;
    ar &pipeline_area_per_unit;
    ar &Component::area;
    // Component::serialize(ar, version);
  }
};

#endif /* CORE_H_ */
