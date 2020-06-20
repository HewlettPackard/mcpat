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
#ifndef __INST_DECODER_H__
#define __INST_DECODER_H__

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

class inst_decoder : public Component {
public:
  inst_decoder(bool _is_default,
               const InputParameter *configure_interface,
               int opcode_length_,
               int num_decoders_,
               bool x86_,
               enum Device_ty device_ty_ = Core_device,
               enum Core_type core_ty_ = Inorder);
  inst_decoder();
  bool is_default;
  int opcode_length;
  int num_decoders;
  bool x86;
  int num_decoder_segments;
  int num_decoded_signals;
  InputParameter l_ip;
  uca_org_t local_result;
  enum Device_ty device_ty;
  enum Core_type core_ty;

  Decoder *final_dec;
  Predec *pre_dec;

  statsDef tdp_stats;
  statsDef rtp_stats;
  statsDef stats_t;
  powerDef power_t;
  void inst_decoder_delay_power();
  ~inst_decoder();
  void leakage_feedback(double temperature);
};

#endif //__INST_DECODER_H__
