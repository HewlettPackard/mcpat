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

#include "cache_param.h"

#include <vector>

void CacheDynParam::set_params_l2_cache(const ParseXML *XML,
                                        const int ithCache) {
  this->name = "L2";
  this->clockRate = XML->sys.L2[ithCache].clockrate;
  this->clockRate *= 1e6;
  this->executionTime =
      XML->sys.total_cycles / (XML->sys.target_core_clockrate * 1e6);
  this->capacity = XML->sys.L2[ithCache].L2_config[0];
  this->blockW = XML->sys.L2[ithCache].L2_config[1];
  this->assoc = XML->sys.L2[ithCache].L2_config[2];
  this->nbanks = XML->sys.L2[ithCache].L2_config[3];
  this->throughput = XML->sys.L2[ithCache].L2_config[4] / this->clockRate;
  this->latency = XML->sys.L2[ithCache].L2_config[5] / this->clockRate;
  this->missb_size = XML->sys.L2[ithCache].buffer_sizes[0];
  this->fu_size = XML->sys.L2[ithCache].buffer_sizes[1];
  this->prefetchb_size = XML->sys.L2[ithCache].buffer_sizes[2];
  this->wbb_size = XML->sys.L2[ithCache].buffer_sizes[3];
  this->duty_cycle = XML->sys.L2[ithCache].duty_cycle;
  if (!XML->sys.L2[ithCache].merged_dir) {
    this->dir_ty = NonDir;
  } else {
    this->dir_ty = SBT;
    this->dir_duty_cycle = XML->sys.L2[ithCache].dir_duty_cycle;
  }
}

void CacheDynParam::set_params_l3_cache(const ParseXML *XML,
                                        const int ithCache) {
  this->name = "L3";
  this->clockRate = XML->sys.L3[ithCache].clockrate;
  this->clockRate *= 1e6;
  this->executionTime =
      XML->sys.total_cycles / (XML->sys.target_core_clockrate * 1e6);
  this->capacity = XML->sys.L3[ithCache].L3_config[0];
  this->blockW = XML->sys.L3[ithCache].L3_config[1];
  this->assoc = XML->sys.L3[ithCache].L3_config[2];
  this->nbanks = XML->sys.L3[ithCache].L3_config[3];
  this->throughput = XML->sys.L3[ithCache].L3_config[4] / this->clockRate;
  this->latency = XML->sys.L3[ithCache].L3_config[5] / this->clockRate;
  this->missb_size = XML->sys.L3[ithCache].buffer_sizes[0];
  this->fu_size = XML->sys.L3[ithCache].buffer_sizes[1];
  this->prefetchb_size = XML->sys.L3[ithCache].buffer_sizes[2];
  this->wbb_size = XML->sys.L3[ithCache].buffer_sizes[3];
  this->duty_cycle = XML->sys.L3[ithCache].duty_cycle;
  if (!XML->sys.L2[ithCache].merged_dir) {
    this->dir_ty = NonDir;
  } else {
    this->dir_ty = SBT;
    this->dir_duty_cycle = XML->sys.L2[ithCache].dir_duty_cycle;
  }
}

void CacheDynParam::set_params_l1_directory(const ParseXML *XML,
                                            const int ithCache) {
  this->name = "First Level Directory";
  this->dir_ty = (enum Dir_type)XML->sys.L1Directory[ithCache].Directory_type;
  this->clockRate = XML->sys.L1Directory[ithCache].clockrate;
  this->clockRate *= 1e6;
  this->executionTime =
      XML->sys.total_cycles / (XML->sys.target_core_clockrate * 1e6);
  this->capacity = XML->sys.L1Directory[ithCache].Dir_config[0];
  this->blockW = XML->sys.L1Directory[ithCache].Dir_config[1];
  this->assoc = XML->sys.L1Directory[ithCache].Dir_config[2];
  this->nbanks = XML->sys.L1Directory[ithCache].Dir_config[3];
  this->throughput =
      XML->sys.L1Directory[ithCache].Dir_config[4] / this->clockRate;
  this->latency =
      XML->sys.L1Directory[ithCache].Dir_config[5] / this->clockRate;
  this->missb_size = XML->sys.L1Directory[ithCache].buffer_sizes[0];
  this->fu_size = XML->sys.L1Directory[ithCache].buffer_sizes[1];
  this->prefetchb_size = XML->sys.L1Directory[ithCache].buffer_sizes[2];
  this->wbb_size = XML->sys.L1Directory[ithCache].buffer_sizes[3];
  this->duty_cycle = XML->sys.L1Directory[ithCache].duty_cycle;
}

void CacheDynParam::set_params_l2_directory(const ParseXML *XML,
                                            const int ithCache) {
  this->name = "Second Level Directory";
  this->dir_ty = (enum Dir_type)XML->sys.L2Directory[ithCache].Directory_type;
  this->clockRate = XML->sys.L2Directory[ithCache].clockrate;
  this->clockRate *= 1e6;
  this->executionTime =
      XML->sys.total_cycles / (XML->sys.target_core_clockrate * 1e6);
  this->capacity = XML->sys.L2Directory[ithCache].Dir_config[0];
  this->blockW = XML->sys.L2Directory[ithCache].Dir_config[1];
  this->assoc = XML->sys.L2Directory[ithCache].Dir_config[2];
  this->nbanks = XML->sys.L2Directory[ithCache].Dir_config[3];
  this->throughput =
      XML->sys.L2Directory[ithCache].Dir_config[4] / this->clockRate;
  this->latency =
      XML->sys.L2Directory[ithCache].Dir_config[5] / this->clockRate;
  this->missb_size = XML->sys.L2Directory[ithCache].buffer_sizes[0];
  this->fu_size = XML->sys.L2Directory[ithCache].buffer_sizes[1];
  this->prefetchb_size = XML->sys.L2Directory[ithCache].buffer_sizes[2];
  this->wbb_size = XML->sys.L2Directory[ithCache].buffer_sizes[3];
  this->duty_cycle = XML->sys.L2Directory[ithCache].duty_cycle;
}
