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

#ifndef __LOAD_STORE_U_H__
#define __LOAD_STORE_U_H__

#include "XML_Parse.h"
#include "array.h"
#include "basic_components.h"
#include "datacache.h"
#include "interconnect.h"
#include "parameter.h"

class LoadStoreU : public Component {
public:
  const ParseXML *XML;
  int ithCore;
  InputParameter interface_ip;
  CoreDynParam coredynp;
  enum Cache_policy cache_p;
  double clockRate;
  double executionTime;
  double scktRatio;
  double chip_PR_overhead;
  double macro_PR_overhead;
  double lsq_height;
  DataCache dcache;
  ArrayST *LSQ; // it is actually the store queue but for inorder processors it
                // serves as both loadQ and StoreQ
  ArrayST *LoadQ;
  bool exist;

  LoadStoreU(const ParseXML *XML_interface,
             int ithCore_,
             InputParameter *interface_ip_,
             const CoreDynParam &dyn_p_,
             bool exist_ = true);
  void computeEnergy(bool is_tdp = true);
  void displayEnergy(uint32_t indent = 0, int plevel = 100, bool is_tdp = true);
  ~LoadStoreU();
};

#endif // __LOAD_STORE_U_H__
