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

#ifndef __RENAMING_U_H__
#define __RENAMING_U_H__

#include "XML_Parse.h"
#include "array.h"
#include "basic_components.h"
#include "interconnect.h"
#include "logic.h"
#include "parameter.h"

class RENAMINGU : public Component {
public:
  ParseXML *XML;
  int ithCore;
  InputParameter interface_ip;
  double clockRate;
  double executionTime;
  CoreDynParam coredynp;
  ArrayST iFRAT;
  ArrayST fFRAT;
  ArrayST iRRAT;
  ArrayST fRRAT;
  ArrayST ifreeL;
  ArrayST ffreeL;
  dep_resource_conflict_check *idcl;
  dep_resource_conflict_check *fdcl;
  ArrayST RAHT; // register alias history table Used to store GC
  bool exist;

  RENAMINGU();
  void set_params(ParseXML *XML_interface,
                  int ithCore_,
                  InputParameter *interface_ip_,
                  const CoreDynParam &dyn_p_,
                  bool exist_ = true);
  void set_stats(const ParseXML *XML);
  void computeArea();
  void computeStaticPower();
  void computeDynamicPower(bool is_tdp);
    void displayEnergy(uint32_t indent = 0, int plevel = 100, bool is_tdp = true);
   ~RENAMINGU();
  
  private:

  bool init_params;
  bool init_stats;
};

#endif // __RENAMING_U_H__
