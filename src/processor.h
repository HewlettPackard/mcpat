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
#ifndef PROCESSOR_H_
#define PROCESSOR_H_

#include "XML_Parse.h"
#include "arbiter.h"
#include "area.h"
#include "array.h"
#include "basic_components.h"
#include "core.h"
#include "decoder.h"
#include "flash_controller.h"
#include "memoryctrl.h"
#include "niu_controller.h"
#include "noc.h"
#include "parameter.h"
#include "pcie_controller.h"
#include "router.h"
#include "sharedcache.h"

#include <vector>

class Processor : public Component {
public:
  Processor();
  void init(const ParseXML* XML);
  void computeArea();
  void computePower();
  void computeRuntimeDynamicPower();
  void displayEnergy(uint32_t indent = 0, int plevel = 100, bool is_tdp = true);
  ~Processor();

private:
  const ParseXML *XML;
  vector<Core *> cores;
  vector<SharedCache *> l2array;
  vector<SharedCache *> l3array;
  vector<SharedCache *> l1dirarray;
  vector<SharedCache *> l2dirarray;
  vector<NoC *> nocs;
  MemoryController mc;
  NIUController niu;
  PCIeController pcie;
  FlashController flashcontroller;
  InputParameter interface_ip;
  ProcParam procdynp;

  // Used for total Area Calcs:
  Component core;
  Component l2;
  Component l3;
  Component l1dir;
  Component l2dir;
  Component noc;
  Component mcs;
  Component cc;
  Component nius;
  Component pcies;
  Component flashcontrollers;
  int numCore;
  int numL2;
  int numL3;
  int numNOC;
  int numL1Dir;
  int numL2Dir;
  void set_proc_param();
  void displayDeviceType(int device_type_, uint32_t indent = 0);
  void displayInterconnectType(int interconnect_type_, uint32_t indent = 0);
};

#endif /* PROCESSOR_H_ */
