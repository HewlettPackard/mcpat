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
#ifndef __PIPELINE_H__
#define __PIPELINE_H__

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

class Pipeline : public Component {
public:
  Pipeline(const InputParameter *configure_interface,
           const CoreDynParam &dyn_p_,
           enum Device_ty device_ty_ = Core_device,
           bool _is_core_pipeline = true,
           bool _is_default = true);
  InputParameter l_ip;
  uca_org_t local_result;
  CoreDynParam coredynp;
  enum Device_ty device_ty;
  bool is_core_pipeline, is_default;
  double num_piperegs;
  //	int pipeline_stages;
  //	int tot_stage_vector, per_stage_vector;
  bool process_ind;
  double WNANDn;
  double WNANDp;
  double load_per_pipeline_stage;
  //	int  Hthread,  num_thread, fetchWidth, decodeWidth, issueWidth,
  // commitWidth, instruction_length; 	int  PC_width, opcode_length,
  // num_arch_reg_tag, data_width,num_phsical_reg_tag, address_width; 	bool
  // thread_clock_gated; 	bool in_order, multithreaded;
  void compute_stage_vector();
  void compute();
  ~Pipeline() { local_result.cleanup(); };
};

#endif // __PIPELINE_H__
