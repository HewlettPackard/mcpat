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

#ifndef __MC_FRONTEND_H__
#define __MC_FRONTEND_H__

#include "XML_Parse.h"
#include "array.h"
#include "basic_components.h"
#include "parameter.h"
#include "selection_logic.h"

#include <vector>

class MCFrontEnd : public Component {
public:
  InputParameter sl_ip;
  InputParameter fe_ip;
  InputParameter rb_ip;
  InputParameter wb_ip;
  enum MemoryCtrl_type mc_type;
  MCParam mcp;
  selection_logic *MC_arb;
  ArrayST frontendBuffer;
  ArrayST readBuffer;
  ArrayST writeBuffer;

  MCFrontEnd();
  void set_params(const ParseXML *XML,
                  InputParameter *interface_ip_,
                  const MCParam &mcp_,
                  enum MemoryCtrl_type mc_type_);
  void set_stats(const ParseXML *XML, const MCParam &mcp_);
  void computeArea();
  void computeStaticPower();
  void computeDynamicPower();
  void display(uint32_t indent = 0, bool enable = true, bool detailed = false);
  ~MCFrontEnd();

private:
  bool long_channel;
  bool power_gating;
  bool init_params;
  bool init_stats;
  int memory_channels_per_mc;
  int physical_address_width;
  int req_window_size_per_channel;
  int IO_buffer_size_per_channel;
  int memory_reads;
  int memory_writes;

  void computeFrontEndRTP();
  void computeReadBufferRTP();
  void computeWriteBufferRTP();
  void computeFrontEndTDP();
  void computeReadBufferTDP();
  void computeWriteBufferTDP();

  // Serialization
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive &ar, const unsigned int version) {
    ar &frontendBuffer;
    ar &readBuffer;
    ar &writeBuffer;
    Component::serialize(ar, version);
  }
};

#endif // __MC_FRONTEND_H__
