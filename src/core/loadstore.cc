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

#include "loadstore.h"

#include "XML_Parse.h"
#include "basic_circuit.h"
#include "const.h"
#include "io.h"
#include "parameter.h"

#include <algorithm>
#include <assert.h>
#include <cmath>
#include <iostream>
#include <string>

LoadStoreU::LoadStoreU(ParseXML *XML_interface,
                       int ithCore_,
                       InputParameter *interface_ip_,
                       const CoreDynParam &dyn_p_,
                       bool exist_)
    : XML(XML_interface), ithCore(ithCore_), interface_ip(*interface_ip_),
      coredynp(dyn_p_), LSQ(0), LoadQ(0), exist(exist_) {
  if (!exist)
    return;
  int idx, tag, data, size, line, assoc, banks;
  bool debug = false;
  int ldst_opcode = XML->sys.core[ithCore].opcode_width; // 16;

  clockRate = coredynp.clockRate;
  executionTime = coredynp.executionTime;
  cache_p = (Cache_policy)XML->sys.core[ithCore].dcache.dcache_config[7];

  interface_ip.num_search_ports = XML->sys.core[ithCore].memory_ports;
  interface_ip.is_cache = true;
  interface_ip.pure_cam = false;
  interface_ip.pure_ram = false;
  // Dcache
  size = (int)XML->sys.core[ithCore].dcache.dcache_config[0];
  line = (int)XML->sys.core[ithCore].dcache.dcache_config[1];
  assoc = (int)XML->sys.core[ithCore].dcache.dcache_config[2];
  banks = (int)XML->sys.core[ithCore].dcache.dcache_config[3];
  idx = debug ? 9 : int(ceil(log2(size / line / assoc)));
  tag = debug ? 51
              : XML->sys.physical_address_width - idx - int(ceil(log2(line))) +
                    EXTRA_TAG_BITS;
  interface_ip.specific_tag = 1;
  interface_ip.tag_w = tag;
  interface_ip.cache_sz =
      debug ? 32768 : (int)XML->sys.core[ithCore].dcache.dcache_config[0];
  interface_ip.line_sz =
      debug ? 64 : (int)XML->sys.core[ithCore].dcache.dcache_config[1];
  interface_ip.assoc =
      debug ? 8 : (int)XML->sys.core[ithCore].dcache.dcache_config[2];
  interface_ip.nbanks =
      debug ? 1 : (int)XML->sys.core[ithCore].dcache.dcache_config[3];
  interface_ip.out_w = interface_ip.line_sz * 8;
  interface_ip.access_mode =
      0; // debug?0:XML->sys.core[ithCore].dcache.dcache_config[5];
  interface_ip.throughput =
      debug ? 1.0 / clockRate
            : XML->sys.core[ithCore].dcache.dcache_config[4] / clockRate;
  interface_ip.latency =
      debug ? 3.0 / clockRate
            : XML->sys.core[ithCore].dcache.dcache_config[5] / clockRate;
  interface_ip.is_cache = true;
  interface_ip.obj_func_dyn_energy = 0;
  interface_ip.obj_func_dyn_power = 0;
  interface_ip.obj_func_leak_power = 0;
  interface_ip.obj_func_cycle_t = 1;
  interface_ip.num_rw_ports =
      debug
          ? 1
          : XML->sys.core[ithCore]
                .memory_ports; // usually In-order has 1 and OOO has 2 at least.
  interface_ip.num_rd_ports = 0;
  interface_ip.num_wr_ports = 0;
  interface_ip.num_se_rd_ports = 0;
  dcache.caches = new ArrayST(&interface_ip,
                              "dcache",
                              Core_device,
                              coredynp.opt_local,
                              coredynp.core_ty);
  dcache.area.set_area(dcache.area.get_area() +
                       dcache.caches->local_result.area);
  area.set_area(area.get_area() + dcache.caches->local_result.area);
  // output_data_csv(dcache.caches.local_result);

  // dCache controllers
  // miss buffer
  tag = XML->sys.physical_address_width + EXTRA_TAG_BITS;
  data = (XML->sys.physical_address_width) + int(ceil(log2(size / line))) +
         dcache.caches->l_ip.line_sz * 8;
  interface_ip.specific_tag = 1;
  interface_ip.tag_w = tag;
  interface_ip.line_sz =
      int(ceil(data / 8.0)); // int(ceil(pow(2.0,ceil(log2(data)))/8.0));
  interface_ip.cache_sz =
      XML->sys.core[ithCore].dcache.buffer_sizes[0] * interface_ip.line_sz;
  interface_ip.assoc = 0;
  interface_ip.nbanks = 1;
  interface_ip.out_w = interface_ip.line_sz * 8;
  interface_ip.access_mode = 2;
  interface_ip.throughput =
      debug ? 1.0 / clockRate
            : XML->sys.core[ithCore].dcache.dcache_config[4] / clockRate;
  interface_ip.latency =
      debug ? 1.0 / clockRate
            : XML->sys.core[ithCore].dcache.dcache_config[5] / clockRate;
  interface_ip.obj_func_dyn_energy = 0;
  interface_ip.obj_func_dyn_power = 0;
  interface_ip.obj_func_leak_power = 0;
  interface_ip.obj_func_cycle_t = 1;
  interface_ip.num_rw_ports = debug ? 1 : XML->sys.core[ithCore].memory_ports;
  ;
  interface_ip.num_rd_ports = 0;
  interface_ip.num_wr_ports = 0;
  interface_ip.num_se_rd_ports = 0;
  dcache.missb = new ArrayST(&interface_ip,
                             "dcacheMissBuffer",
                             Core_device,
                             coredynp.opt_local,
                             coredynp.core_ty);
  dcache.area.set_area(dcache.area.get_area() +
                       dcache.missb->local_result.area);
  area.set_area(area.get_area() + dcache.missb->local_result.area);
  // output_data_csv(dcache.missb.local_result);

  // fill buffer
  tag = XML->sys.physical_address_width + EXTRA_TAG_BITS;
  data = dcache.caches->l_ip.line_sz;
  interface_ip.specific_tag = 1;
  interface_ip.tag_w = tag;
  interface_ip.line_sz = data; // int(pow(2.0,ceil(log2(data))));
  interface_ip.cache_sz = data * XML->sys.core[ithCore].dcache.buffer_sizes[1];
  interface_ip.assoc = 0;
  interface_ip.nbanks = 1;
  interface_ip.out_w = interface_ip.line_sz * 8;
  interface_ip.access_mode = 2;
  interface_ip.throughput =
      debug ? 1.0 / clockRate
            : XML->sys.core[ithCore].dcache.dcache_config[4] / clockRate;
  interface_ip.latency =
      debug ? 1.0 / clockRate
            : XML->sys.core[ithCore].dcache.dcache_config[5] / clockRate;
  interface_ip.obj_func_dyn_energy = 0;
  interface_ip.obj_func_dyn_power = 0;
  interface_ip.obj_func_leak_power = 0;
  interface_ip.obj_func_cycle_t = 1;
  interface_ip.num_rw_ports = debug ? 1 : XML->sys.core[ithCore].memory_ports;
  ;
  interface_ip.num_rd_ports = 0;
  interface_ip.num_wr_ports = 0;
  interface_ip.num_se_rd_ports = 0;
  dcache.ifb = new ArrayST(&interface_ip,
                           "dcacheFillBuffer",
                           Core_device,
                           coredynp.opt_local,
                           coredynp.core_ty);
  dcache.area.set_area(dcache.area.get_area() + dcache.ifb->local_result.area);
  area.set_area(area.get_area() + dcache.ifb->local_result.area);
  // output_data_csv(dcache.ifb.local_result);

  // prefetch buffer
  tag = XML->sys.physical_address_width +
        EXTRA_TAG_BITS; // check with previous entries to decide wthether to
                        // merge.
  data = dcache.caches->l_ip
             .line_sz; // separate queue to prevent from cache polution.
  interface_ip.specific_tag = 1;
  interface_ip.tag_w = tag;
  interface_ip.line_sz = data; // int(pow(2.0,ceil(log2(data))));
  interface_ip.cache_sz =
      XML->sys.core[ithCore].dcache.buffer_sizes[2] * interface_ip.line_sz;
  interface_ip.assoc = 0;
  interface_ip.nbanks = 1;
  interface_ip.out_w = interface_ip.line_sz * 8;
  interface_ip.access_mode = 2;
  interface_ip.throughput =
      debug ? 1.0 / clockRate
            : XML->sys.core[ithCore].dcache.dcache_config[4] / clockRate;
  interface_ip.latency =
      debug ? 1.0 / clockRate
            : XML->sys.core[ithCore].dcache.dcache_config[5] / clockRate;
  interface_ip.obj_func_dyn_energy = 0;
  interface_ip.obj_func_dyn_power = 0;
  interface_ip.obj_func_leak_power = 0;
  interface_ip.obj_func_cycle_t = 1;
  interface_ip.num_rw_ports = debug ? 1 : XML->sys.core[ithCore].memory_ports;
  ;
  interface_ip.num_rd_ports = 0;
  interface_ip.num_wr_ports = 0;
  interface_ip.num_se_rd_ports = 0;
  dcache.prefetchb = new ArrayST(&interface_ip,
                                 "dcacheprefetchBuffer",
                                 Core_device,
                                 coredynp.opt_local,
                                 coredynp.core_ty);
  dcache.area.set_area(dcache.area.get_area() +
                       dcache.prefetchb->local_result.area);
  area.set_area(area.get_area() + dcache.prefetchb->local_result.area);
  // output_data_csv(dcache.prefetchb.local_result);

  // WBB

  if (cache_p == Write_back) {
    tag = XML->sys.physical_address_width + EXTRA_TAG_BITS;
    data = dcache.caches->l_ip.line_sz;
    interface_ip.specific_tag = 1;
    interface_ip.tag_w = tag;
    interface_ip.line_sz = data;
    interface_ip.cache_sz =
        XML->sys.core[ithCore].dcache.buffer_sizes[3] * interface_ip.line_sz;
    interface_ip.assoc = 0;
    interface_ip.nbanks = 1;
    interface_ip.out_w = interface_ip.line_sz * 8;
    interface_ip.access_mode = 2;
    interface_ip.throughput =
        debug ? 1.0 / clockRate
              : XML->sys.core[ithCore].dcache.dcache_config[4] / clockRate;
    interface_ip.latency =
        debug ? 1.0 / clockRate
              : XML->sys.core[ithCore].dcache.dcache_config[5] / clockRate;
    interface_ip.obj_func_dyn_energy = 0;
    interface_ip.obj_func_dyn_power = 0;
    interface_ip.obj_func_leak_power = 0;
    interface_ip.obj_func_cycle_t = 1;
    interface_ip.num_rw_ports = XML->sys.core[ithCore].memory_ports;
    interface_ip.num_rd_ports = 0;
    interface_ip.num_wr_ports = 0;
    interface_ip.num_se_rd_ports = 0;
    dcache.wbb = new ArrayST(&interface_ip,
                             "dcacheWBB",
                             Core_device,
                             coredynp.opt_local,
                             coredynp.core_ty);
    dcache.area.set_area(dcache.area.get_area() +
                         dcache.wbb->local_result.area);
    area.set_area(area.get_area() + dcache.wbb->local_result.area);
    // output_data_csv(dcache.wbb.local_result);
  }

  /*
   * LSU--in-order processors do not have separate load queue: unified lsq
   * partitioned among threads
   * it is actually the store queue but for inorder processors it serves as both
   * loadQ and StoreQ
   */
  tag = ldst_opcode + XML->sys.virtual_address_width +
        int(ceil(log2(XML->sys.core[ithCore].number_hardware_threads))) +
        EXTRA_TAG_BITS;
  data = XML->sys.machine_bits;
  interface_ip.is_cache = true;
  interface_ip.line_sz = int(ceil(data / 32.0)) * 4;
  interface_ip.specific_tag = 1;
  interface_ip.tag_w = tag;
  interface_ip.cache_sz =
      XML->sys.core[ithCore].store_buffer_size * interface_ip.line_sz;
  interface_ip.assoc = 0;
  interface_ip.nbanks = 1;
  interface_ip.out_w = interface_ip.line_sz * 8;
  interface_ip.access_mode = 1;
  interface_ip.throughput = 1.0 / clockRate;
  interface_ip.latency = 1.0 / clockRate;
  interface_ip.obj_func_dyn_energy = 0;
  interface_ip.obj_func_dyn_power = 0;
  interface_ip.obj_func_leak_power = 0;
  interface_ip.obj_func_cycle_t = 1;
  interface_ip.num_rw_ports = 0;
  interface_ip.num_rd_ports = XML->sys.core[ithCore].memory_ports;
  interface_ip.num_wr_ports = XML->sys.core[ithCore].memory_ports;
  interface_ip.num_se_rd_ports = 0;
  interface_ip.num_search_ports = XML->sys.core[ithCore].memory_ports;
  LSQ = new ArrayST(&interface_ip,
                    "Load(Store)Queue",
                    Core_device,
                    coredynp.opt_local,
                    coredynp.core_ty);
  LSQ->area.set_area(LSQ->area.get_area() + LSQ->local_result.area);
  area.set_area(area.get_area() + LSQ->local_result.area);
  // output_data_csv(LSQ.LSQ.local_result);
  lsq_height =
      LSQ->local_result.cache_ht *
      sqrt(cdb_overhead); /*XML->sys.core[ithCore].number_hardware_threads*/

  if ((coredynp.core_ty == OOO) &&
      (XML->sys.core[ithCore].load_buffer_size > 0)) {
    interface_ip.line_sz = int(ceil(data / 32.0)) * 4;
    interface_ip.specific_tag = 1;
    interface_ip.tag_w = tag;
    interface_ip.cache_sz =
        XML->sys.core[ithCore].load_buffer_size * interface_ip.line_sz;
    interface_ip.assoc = 0;
    interface_ip.nbanks = 1;
    interface_ip.out_w = interface_ip.line_sz * 8;
    interface_ip.access_mode = 1;
    interface_ip.throughput = 1.0 / clockRate;
    interface_ip.latency = 1.0 / clockRate;
    interface_ip.obj_func_dyn_energy = 0;
    interface_ip.obj_func_dyn_power = 0;
    interface_ip.obj_func_leak_power = 0;
    interface_ip.obj_func_cycle_t = 1;
    interface_ip.num_rw_ports = 0;
    interface_ip.num_rd_ports = XML->sys.core[ithCore].memory_ports;
    interface_ip.num_wr_ports = XML->sys.core[ithCore].memory_ports;
    interface_ip.num_se_rd_ports = 0;
    interface_ip.num_search_ports = XML->sys.core[ithCore].memory_ports;
    LoadQ = new ArrayST(&interface_ip,
                        "LoadQueue",
                        Core_device,
                        coredynp.opt_local,
                        coredynp.core_ty);
    LoadQ->area.set_area(LoadQ->area.get_area() + LoadQ->local_result.area);
    area.set_area(area.get_area() + LoadQ->local_result.area);
    // output_data_csv(LoadQ.LoadQ.local_result);
    lsq_height =
        (LSQ->local_result.cache_ht + LoadQ->local_result.cache_ht) *
        sqrt(cdb_overhead); /*XML->sys.core[ithCore].number_hardware_threads*/
  }
  area.set_area(area.get_area() * cdb_overhead);
}

void LoadStoreU::computeEnergy(bool is_tdp) {
  if (!exist)
    return;
  if (is_tdp) {
    // init stats for Peak
    dcache.caches->stats_t.readAc.access =
        0.67 * dcache.caches->l_ip.num_rw_ports * coredynp.LSU_duty_cycle;
    dcache.caches->stats_t.readAc.miss = 0;
    dcache.caches->stats_t.readAc.hit = dcache.caches->stats_t.readAc.access -
                                        dcache.caches->stats_t.readAc.miss;
    dcache.caches->stats_t.writeAc.access =
        0.33 * dcache.caches->l_ip.num_rw_ports * coredynp.LSU_duty_cycle;
    dcache.caches->stats_t.writeAc.miss = 0;
    dcache.caches->stats_t.writeAc.hit = dcache.caches->stats_t.writeAc.access -
                                         dcache.caches->stats_t.writeAc.miss;
    dcache.caches->tdp_stats = dcache.caches->stats_t;

    dcache.missb->stats_t.readAc.access =
        dcache.missb->l_ip.num_search_ports * coredynp.LSU_duty_cycle;
    dcache.missb->stats_t.writeAc.access =
        dcache.missb->l_ip.num_search_ports * coredynp.LSU_duty_cycle;
    dcache.missb->tdp_stats = dcache.missb->stats_t;

    dcache.ifb->stats_t.readAc.access =
        dcache.ifb->l_ip.num_search_ports * coredynp.LSU_duty_cycle;
    dcache.ifb->stats_t.writeAc.access =
        dcache.ifb->l_ip.num_search_ports * coredynp.LSU_duty_cycle;
    dcache.ifb->tdp_stats = dcache.ifb->stats_t;

    dcache.prefetchb->stats_t.readAc.access =
        dcache.prefetchb->l_ip.num_search_ports * coredynp.LSU_duty_cycle;
    dcache.prefetchb->stats_t.writeAc.access =
        dcache.ifb->l_ip.num_search_ports * coredynp.LSU_duty_cycle;
    dcache.prefetchb->tdp_stats = dcache.prefetchb->stats_t;
    if (cache_p == Write_back) {
      dcache.wbb->stats_t.readAc.access = dcache.wbb->l_ip.num_search_ports;
      dcache.wbb->stats_t.writeAc.access = dcache.wbb->l_ip.num_search_ports;
      dcache.wbb->tdp_stats = dcache.wbb->stats_t;
    }

    LSQ->stats_t.readAc.access = LSQ->stats_t.writeAc.access =
        LSQ->l_ip.num_search_ports * coredynp.LSU_duty_cycle;
    LSQ->tdp_stats = LSQ->stats_t;
    if ((coredynp.core_ty == OOO) &&
        (XML->sys.core[ithCore].load_buffer_size > 0)) {
      LoadQ->stats_t.readAc.access = LoadQ->stats_t.writeAc.access =
          LoadQ->l_ip.num_search_ports * coredynp.LSU_duty_cycle;
      LoadQ->tdp_stats = LoadQ->stats_t;
    }
  } else {
    // init stats for Runtime Dynamic (RTP)
    dcache.caches->stats_t.readAc.access =
        XML->sys.core[ithCore].dcache.read_accesses;
    dcache.caches->stats_t.readAc.miss =
        XML->sys.core[ithCore].dcache.read_misses;
    dcache.caches->stats_t.readAc.hit = dcache.caches->stats_t.readAc.access -
                                        dcache.caches->stats_t.readAc.miss;
    dcache.caches->stats_t.writeAc.access =
        XML->sys.core[ithCore].dcache.write_accesses;
    dcache.caches->stats_t.writeAc.miss =
        XML->sys.core[ithCore].dcache.write_misses;
    dcache.caches->stats_t.writeAc.hit = dcache.caches->stats_t.writeAc.access -
                                         dcache.caches->stats_t.writeAc.miss;
    dcache.caches->rtp_stats = dcache.caches->stats_t;

    if (cache_p == Write_back) {
      dcache.missb->stats_t.readAc.access = dcache.caches->stats_t.writeAc.miss;
      dcache.missb->stats_t.writeAc.access =
          dcache.caches->stats_t.writeAc.miss;
      dcache.missb->rtp_stats = dcache.missb->stats_t;

      dcache.ifb->stats_t.readAc.access = dcache.caches->stats_t.writeAc.miss;
      dcache.ifb->stats_t.writeAc.access = dcache.caches->stats_t.writeAc.miss;
      dcache.ifb->rtp_stats = dcache.ifb->stats_t;

      dcache.prefetchb->stats_t.readAc.access =
          dcache.caches->stats_t.writeAc.miss;
      dcache.prefetchb->stats_t.writeAc.access =
          dcache.caches->stats_t.writeAc.miss;
      dcache.prefetchb->rtp_stats = dcache.prefetchb->stats_t;

      dcache.wbb->stats_t.readAc.access = dcache.caches->stats_t.writeAc.miss;
      dcache.wbb->stats_t.writeAc.access = dcache.caches->stats_t.writeAc.miss;
      dcache.wbb->rtp_stats = dcache.wbb->stats_t;
    } else {
      dcache.missb->stats_t.readAc.access = dcache.caches->stats_t.readAc.miss;
      dcache.missb->stats_t.writeAc.access = dcache.caches->stats_t.readAc.miss;
      dcache.missb->rtp_stats = dcache.missb->stats_t;

      dcache.ifb->stats_t.readAc.access = dcache.caches->stats_t.readAc.miss;
      dcache.ifb->stats_t.writeAc.access = dcache.caches->stats_t.readAc.miss;
      dcache.ifb->rtp_stats = dcache.ifb->stats_t;

      dcache.prefetchb->stats_t.readAc.access =
          dcache.caches->stats_t.readAc.miss;
      dcache.prefetchb->stats_t.writeAc.access =
          dcache.caches->stats_t.readAc.miss;
      dcache.prefetchb->rtp_stats = dcache.prefetchb->stats_t;
    }

    LSQ->stats_t.readAc.access = (XML->sys.core[ithCore].load_instructions +
                                  XML->sys.core[ithCore].store_instructions) *
                                 2; // flush overhead considered
    LSQ->stats_t.writeAc.access = (XML->sys.core[ithCore].load_instructions +
                                   XML->sys.core[ithCore].store_instructions) *
                                  2;
    LSQ->rtp_stats = LSQ->stats_t;

    if ((coredynp.core_ty == OOO) &&
        (XML->sys.core[ithCore].load_buffer_size > 0)) {
      LoadQ->stats_t.readAc.access = XML->sys.core[ithCore].load_instructions +
                                     XML->sys.core[ithCore].store_instructions;
      LoadQ->stats_t.writeAc.access = XML->sys.core[ithCore].load_instructions +
                                      XML->sys.core[ithCore].store_instructions;
      LoadQ->rtp_stats = LoadQ->stats_t;
    }
  }

  dcache.power_t.reset();
  LSQ->power_t.reset();
  dcache.power_t.readOp.dynamic +=
      (dcache.caches->stats_t.readAc.hit *
           dcache.caches->local_result.power.readOp.dynamic +
       dcache.caches->stats_t.readAc.miss *
           dcache.caches->local_result.power.readOp
               .dynamic + // assuming D cache is in the fast model which read
                          // tag and data together
       dcache.caches->stats_t.writeAc.miss *
           dcache.caches->local_result.tag_array2->power.readOp.dynamic +
       dcache.caches->stats_t.writeAc.access *
           dcache.caches->local_result.power.writeOp.dynamic);

  if (cache_p == Write_back) { // write miss will generate a write later
    dcache.power_t.readOp.dynamic +=
        dcache.caches->stats_t.writeAc.miss *
        dcache.caches->local_result.power.writeOp.dynamic;
  }

  dcache.power_t.readOp.dynamic +=
      dcache.missb->stats_t.readAc.access *
          dcache.missb->local_result.power.searchOp.dynamic +
      dcache.missb->stats_t.writeAc.access *
          dcache.missb->local_result.power.writeOp
              .dynamic; // each access to missb involves a CAM and a write
  dcache.power_t.readOp.dynamic +=
      dcache.ifb->stats_t.readAc.access *
          dcache.ifb->local_result.power.searchOp.dynamic +
      dcache.ifb->stats_t.writeAc.access *
          dcache.ifb->local_result.power.writeOp.dynamic;
  dcache.power_t.readOp.dynamic +=
      dcache.prefetchb->stats_t.readAc.access *
          dcache.prefetchb->local_result.power.searchOp.dynamic +
      dcache.prefetchb->stats_t.writeAc.access *
          dcache.prefetchb->local_result.power.writeOp.dynamic;
  if (cache_p == Write_back) {
    dcache.power_t.readOp.dynamic +=
        dcache.wbb->stats_t.readAc.access *
            dcache.wbb->local_result.power.searchOp.dynamic +
        dcache.wbb->stats_t.writeAc.access *
            dcache.wbb->local_result.power.writeOp.dynamic;
  }

  if ((coredynp.core_ty == OOO) &&
      (XML->sys.core[ithCore].load_buffer_size > 0)) {
    LoadQ->power_t.reset();
    LoadQ->power_t.readOp.dynamic +=
        LoadQ->stats_t.readAc.access *
            (LoadQ->local_result.power.searchOp.dynamic +
             LoadQ->local_result.power.readOp.dynamic) +
        LoadQ->stats_t.writeAc.access *
            LoadQ->local_result.power.writeOp
                .dynamic; // every memory access invloves at least two
                          // operations on LoadQ

    LSQ->power_t.readOp.dynamic +=
        LSQ->stats_t.readAc.access * (LSQ->local_result.power.searchOp.dynamic +
                                      LSQ->local_result.power.readOp.dynamic) +
        LSQ->stats_t.writeAc.access *
            LSQ->local_result.power.writeOp
                .dynamic; // every memory access invloves at least two
                          // operations on LSQ

  } else {
    LSQ->power_t.readOp.dynamic +=
        LSQ->stats_t.readAc.access * (LSQ->local_result.power.searchOp.dynamic +
                                      LSQ->local_result.power.readOp.dynamic) +
        LSQ->stats_t.writeAc.access *
            LSQ->local_result.power.writeOp
                .dynamic; // every memory access invloves at least two
                          // operations on LSQ
  }

  if (is_tdp) {
    //    	dcache.power = dcache.power_t +
    //    (dcache.caches->local_result.power)*pppm_lkg +
    //    			(dcache.missb->local_result.power +
    //    			dcache.ifb->local_result.power +
    //    			dcache.prefetchb->local_result.power +
    //    			dcache.wbb->local_result.power)*pppm_Isub;
    dcache.power = dcache.power_t + (dcache.caches->local_result.power +
                                     dcache.missb->local_result.power +
                                     dcache.ifb->local_result.power +
                                     dcache.prefetchb->local_result.power) *
                                        pppm_lkg;
    if (cache_p == Write_back) {
      dcache.power = dcache.power + dcache.wbb->local_result.power * pppm_lkg;
    }

    LSQ->power = LSQ->power_t + LSQ->local_result.power * pppm_lkg;
    power = power + dcache.power + LSQ->power;

    if ((coredynp.core_ty == OOO) &&
        (XML->sys.core[ithCore].load_buffer_size > 0)) {
      LoadQ->power = LoadQ->power_t + LoadQ->local_result.power * pppm_lkg;
      power = power + LoadQ->power;
    }
  } else {
    //    	dcache.rt_power = dcache.power_t +
    //    (dcache.caches->local_result.power +
    //    dcache.missb->local_result.power
    //    + 			dcache.ifb->local_result.power +
    //    			dcache.prefetchb->local_result.power +
    //    			dcache.wbb->local_result.power)*pppm_lkg;
    dcache.rt_power = dcache.power_t + (dcache.caches->local_result.power +
                                        dcache.missb->local_result.power +
                                        dcache.ifb->local_result.power +
                                        dcache.prefetchb->local_result.power) *
                                           pppm_lkg;

    if (cache_p == Write_back) {
      dcache.rt_power =
          dcache.rt_power + dcache.wbb->local_result.power * pppm_lkg;
    }

    LSQ->rt_power = LSQ->power_t + LSQ->local_result.power * pppm_lkg;
    rt_power = rt_power + dcache.rt_power + LSQ->rt_power;

    if ((coredynp.core_ty == OOO) &&
        (XML->sys.core[ithCore].load_buffer_size > 0)) {
      LoadQ->rt_power = LoadQ->power_t + LoadQ->local_result.power * pppm_lkg;
      rt_power = rt_power + LoadQ->rt_power;
    }
  }
}

void LoadStoreU::displayEnergy(uint32_t indent, int plevel, bool is_tdp) {
  if (!exist)
    return;
  string indent_str(indent, ' ');
  string indent_str_next(indent + 2, ' ');
  bool long_channel = XML->sys.longer_channel_device;
  bool power_gating = XML->sys.power_gating;

  if (is_tdp) {
    cout << indent_str << "Data Cache:" << endl;
    cout << indent_str_next << "Area = " << dcache.area.get_area() * 1e-6
         << " mm^2" << endl;
    cout << indent_str_next
         << "Peak Dynamic = " << dcache.power.readOp.dynamic * clockRate << " W"
         << endl;
    cout << indent_str_next << "Subthreshold Leakage = "
         << (long_channel ? dcache.power.readOp.longer_channel_leakage
                          : dcache.power.readOp.leakage)
         << " W" << endl;
    if (power_gating)
      cout << indent_str_next << "Subthreshold Leakage with power gating = "
           << (long_channel
                   ? dcache.power.readOp.power_gated_with_long_channel_leakage
                   : dcache.power.readOp.power_gated_leakage)
           << " W" << endl;
    cout << indent_str_next
         << "Gate Leakage = " << dcache.power.readOp.gate_leakage << " W"
         << endl;
    cout << indent_str_next << "Runtime Dynamic = "
         << dcache.rt_power.readOp.dynamic / executionTime << " W" << endl;
    cout << endl;
    if (coredynp.core_ty == Inorder) {
      cout << indent_str << "Load/Store Queue:" << endl;
      cout << indent_str_next << "Area = " << LSQ->area.get_area() * 1e-6
           << " mm^2" << endl;
      cout << indent_str_next
           << "Peak Dynamic = " << LSQ->power.readOp.dynamic * clockRate << " W"
           << endl;
      cout << indent_str_next << "Subthreshold Leakage = "
           << (long_channel ? LSQ->power.readOp.longer_channel_leakage
                            : LSQ->power.readOp.leakage)
           << " W" << endl;
      if (power_gating)
        cout << indent_str_next << "Subthreshold Leakage with power gating = "
             << (long_channel
                     ? LSQ->power.readOp.power_gated_with_long_channel_leakage
                     : LSQ->power.readOp.power_gated_leakage)
             << " W" << endl;
      cout << indent_str_next
           << "Gate Leakage = " << LSQ->power.readOp.gate_leakage << " W"
           << endl;
      cout << indent_str_next << "Runtime Dynamic = "
           << LSQ->rt_power.readOp.dynamic / executionTime << " W" << endl;
      cout << endl;
    } else

    {
      if (XML->sys.core[ithCore].load_buffer_size > 0) {
        cout << indent_str << "LoadQ:" << endl;
        cout << indent_str_next << "Area = " << LoadQ->area.get_area() * 1e-6
             << " mm^2" << endl;
        cout << indent_str_next
             << "Peak Dynamic = " << LoadQ->power.readOp.dynamic * clockRate
             << " W" << endl;
        cout << indent_str_next << "Subthreshold Leakage = "
             << (long_channel ? LoadQ->power.readOp.longer_channel_leakage
                              : LoadQ->power.readOp.leakage)
             << " W" << endl;
        if (power_gating)
          cout << indent_str_next << "Subthreshold Leakage with power gating = "
               << (long_channel ? LoadQ->power.readOp
                                      .power_gated_with_long_channel_leakage
                                : LoadQ->power.readOp.power_gated_leakage)
               << " W" << endl;
        cout << indent_str_next
             << "Gate Leakage = " << LoadQ->power.readOp.gate_leakage << " W"
             << endl;
        cout << indent_str_next << "Runtime Dynamic = "
             << LoadQ->rt_power.readOp.dynamic / executionTime << " W" << endl;
        cout << endl;
      }
      cout << indent_str << "StoreQ:" << endl;
      cout << indent_str_next << "Area = " << LSQ->area.get_area() * 1e-6
           << " mm^2" << endl;
      cout << indent_str_next
           << "Peak Dynamic = " << LSQ->power.readOp.dynamic * clockRate << " W"
           << endl;
      cout << indent_str_next << "Subthreshold Leakage = "
           << (long_channel ? LSQ->power.readOp.longer_channel_leakage
                            : LSQ->power.readOp.leakage)
           << " W" << endl;
      if (power_gating)
        cout << indent_str_next << "Subthreshold Leakage with power gating = "
             << (long_channel
                     ? LSQ->power.readOp.power_gated_with_long_channel_leakage
                     : LSQ->power.readOp.power_gated_leakage)
             << " W" << endl;
      cout << indent_str_next
           << "Gate Leakage = " << LSQ->power.readOp.gate_leakage << " W"
           << endl;
      cout << indent_str_next << "Runtime Dynamic = "
           << LSQ->rt_power.readOp.dynamic / executionTime << " W" << endl;
      cout << endl;
    }
  } else {
    cout << indent_str_next << "Data Cache    Peak Dynamic = "
         << dcache.rt_power.readOp.dynamic * clockRate << " W" << endl;
    cout << indent_str_next << "Data Cache    Subthreshold Leakage = "
         << dcache.rt_power.readOp.leakage << " W" << endl;
    cout << indent_str_next << "Data Cache    Gate Leakage = "
         << dcache.rt_power.readOp.gate_leakage << " W" << endl;
    if (coredynp.core_ty == Inorder) {
      cout << indent_str_next << "Load/Store Queue   Peak Dynamic = "
           << LSQ->rt_power.readOp.dynamic * clockRate << " W" << endl;
      cout << indent_str_next << "Load/Store Queue   Subthreshold Leakage = "
           << LSQ->rt_power.readOp.leakage << " W" << endl;
      cout << indent_str_next << "Load/Store Queue   Gate Leakage = "
           << LSQ->rt_power.readOp.gate_leakage << " W" << endl;
    } else {
      cout << indent_str_next << "LoadQ   Peak Dynamic = "
           << LoadQ->rt_power.readOp.dynamic * clockRate << " W" << endl;
      cout << indent_str_next << "LoadQ   Subthreshold Leakage = "
           << LoadQ->rt_power.readOp.leakage << " W" << endl;
      cout << indent_str_next
           << "LoadQ   Gate Leakage = " << LoadQ->rt_power.readOp.gate_leakage
           << " W" << endl;
      cout << indent_str_next << "StoreQ   Peak Dynamic = "
           << LSQ->rt_power.readOp.dynamic * clockRate << " W" << endl;
      cout << indent_str_next
           << "StoreQ   Subthreshold Leakage = " << LSQ->rt_power.readOp.leakage
           << " W" << endl;
      cout << indent_str_next
           << "StoreQ   Gate Leakage = " << LSQ->rt_power.readOp.gate_leakage
           << " W" << endl;
    }
  }
}

LoadStoreU ::~LoadStoreU() {

  if (!exist)
    return;
  if (LSQ) {
    delete LSQ;
    LSQ = 0;
  }
  if (LoadQ) {
    delete LoadQ;
    LoadQ = 0;
  }
}
