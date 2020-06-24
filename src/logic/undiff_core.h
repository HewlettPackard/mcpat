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
#ifndef __UNDIFF_CORE_H__
#define __UNDIFF_CORE_H__

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

class UndiffCore : public Component {
public:  
  
  UndiffCore(){};
  void set_params(const ParseXML *XML_interface,
             int ithCore_,
             InputParameter *interface_ip_,
             const CoreDynParam &dyn_p_,
             bool exist_ = true,
             bool embedded_ = false);
  const ParseXML *XML;
  int ithCore;
  InputParameter interface_ip;
  CoreDynParam coredynp;
  double clockRate, executionTime;
  double scktRatio, chip_PR_overhead, macro_PR_overhead;
  enum Core_type core_ty;
  bool opt_performance, embedded;
  double pipeline_stage, num_hthreads, issue_width;
  bool is_default;
  void computeArea();


  void displayEnergy(uint32_t indent = 0, int plevel = 100, bool is_tdp = true);
  ~UndiffCore(){};
  bool exist;
};

#endif //__UNDIFF_CORE_H__
