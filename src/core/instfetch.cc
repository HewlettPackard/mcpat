// /*****************************************************************************
//  *                                McPAT
//  *                      SOFTWARE LICENSE AGREEMENT
//  *            Copyright 2012 Hewlett-Packard Development Company, L.P.
//  *                          All Rights Reserved
//  *
//  * Redistribution and use in source and binary forms, with or without
//  * modification, are permitted provided that the following conditions are
//  * met: redistributions of source code must retain the above copyright
//  * notice, this list of conditions and the following disclaimer;
//  * redistributions in binary form must reproduce the above copyright
//  * notice, this list of conditions and the following disclaimer in the
//  * documentation and/or other materials provided with the distribution;
//  * neither the name of the copyright holders nor the names of its
//  * contributors may be used to endorse or promote products derived from
//  * this software without specific prior written permission.

//  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//  * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//  * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.”
//  *
//  ***************************************************************************/

// #include "instfetch.h"

// #include "XML_Parse.h"
// #include "basic_circuit.h"
// #include "const.h"
// #include "io.h"
// #include "parameter.h"

// #include <algorithm>
// #include <assert.h>
// #include <cmath>
// #include <iostream>
// #include <string>

// InstFetchU::InstFetchU(const ParseXML *XML_interface,
//                        int ithCore_,
//                        InputParameter *interface_ip_,
//                        const CoreDynParam &dyn_p_,
//                        bool exist_)
//     : XML(XML_interface), ithCore(ithCore_), interface_ip(*interface_ip_),
//       coredynp(dyn_p_), IB(0), BTB(0), exist(exist_) {
//   if (!exist)
//     return;
//   int idx, tag, data, size, line, assoc, banks;
//   bool debug = false, is_default = true;

//   clockRate = coredynp.clockRate;
//   executionTime = coredynp.executionTime;
//   cache_p = (Cache_policy)XML->sys.core[ithCore].icache.icache_config[7];
//   // Assuming all L1 caches are virtually idxed physically tagged.
//   // cache

//   size = (int)XML->sys.core[ithCore].icache.icache_config[0];
//   line = (int)XML->sys.core[ithCore].icache.icache_config[1];
//   assoc = (int)XML->sys.core[ithCore].icache.icache_config[2];
//   banks = (int)XML->sys.core[ithCore].icache.icache_config[3];
//   idx = debug ? 9 : int(ceil(log2(size / line / assoc)));
//   tag = debug ? 51
//               : (int)XML->sys.physical_address_width - idx -
//                     int(ceil(log2(line))) + EXTRA_TAG_BITS;
//   interface_ip.specific_tag = 1;
//   interface_ip.tag_w = tag;
//   interface_ip.cache_sz =
//       debug ? 32768 : (int)XML->sys.core[ithCore].icache.icache_config[0];
//   interface_ip.line_sz =
//       debug ? 64 : (int)XML->sys.core[ithCore].icache.icache_config[1];
//   interface_ip.assoc =
//       debug ? 8 : (int)XML->sys.core[ithCore].icache.icache_config[2];
//   interface_ip.nbanks =
//       debug ? 1 : (int)XML->sys.core[ithCore].icache.icache_config[3];
//   interface_ip.out_w = interface_ip.line_sz * 8;
//   interface_ip.access_mode =
//       0; // debug?0:XML->sys.core[ithCore].icache.icache_config[5];
//   interface_ip.throughput =
//       debug ? 1.0 / clockRate
//             : XML->sys.core[ithCore].icache.icache_config[4] / clockRate;
//   interface_ip.latency =
//       debug ? 3.0 / clockRate
//             : XML->sys.core[ithCore].icache.icache_config[5] / clockRate;
//   interface_ip.is_cache = true;
//   interface_ip.pure_cam = false;
//   interface_ip.pure_ram = false;
//   //  interface_ip.obj_func_dyn_energy = 0;
//   //  interface_ip.obj_func_dyn_power  = 0;
//   //  interface_ip.obj_func_leak_power = 0;
//   //  interface_ip.obj_func_cycle_t    = 1;
//   interface_ip.num_rw_ports =
//       debug ? 1 : XML->sys.core[ithCore].number_instruction_fetch_ports;
//   interface_ip.num_rd_ports = 0;
//   interface_ip.num_wr_ports = 0;
//   interface_ip.num_se_rd_ports = 0;
//   icache.caches = new ArrayST(&interface_ip,
//                               "icache",
//                               Core_device,
//                               coredynp.opt_local,
//                               coredynp.core_ty);
//   scktRatio = g_tp.sckt_co_eff;
//   chip_PR_overhead = g_tp.chip_layout_overhead;
//   macro_PR_overhead = g_tp.macro_layout_overhead;
//   icache.area.set_area(icache.area.get_area() +
//                        icache.caches->local_result.area);
//   area.set_area(area.get_area() + icache.caches->local_result.area);
//   // output_data_csv(icache.caches.local_result);

//   /*
//    *iCache controllers
//    *miss buffer Each MSHR contains enough state
//    *to handle one or more accesses of any type to a single memory line.
//    *Due to the generality of the MSHR mechanism,
//    *the amount of state involved is non-trivial:
//    *including the address, pointers to the cache entry and destination register,
//    *written data, and various other pieces of state.
//    */
//   interface_ip.num_search_ports =
//       debug ? 1 : XML->sys.core[ithCore].number_instruction_fetch_ports;
//   tag = XML->sys.physical_address_width + EXTRA_TAG_BITS;
//   data = (XML->sys.physical_address_width) + int(ceil(log2(size / line))) +
//          icache.caches->l_ip.line_sz * 8;
//   interface_ip.specific_tag = 1;
//   interface_ip.tag_w = tag;
//   interface_ip.line_sz =
//       int(ceil(data / 8.0)); // int(ceil(pow(2.0,ceil(log2(data)))/8.0));
//   interface_ip.cache_sz =
//       XML->sys.core[ithCore].icache.buffer_sizes[0] * interface_ip.line_sz;
//   interface_ip.assoc = 0;
//   interface_ip.nbanks = 1;
//   interface_ip.out_w = interface_ip.line_sz * 8;
//   interface_ip.access_mode = 0;
//   interface_ip.throughput =
//       debug ? 1.0 / clockRate
//             : XML->sys.core[ithCore].icache.icache_config[4] /
//                   clockRate; // means cycle time
//   interface_ip.latency = debug
//                              ? 1.0 / clockRate
//                              : XML->sys.core[ithCore].icache.icache_config[5] /
//                                    clockRate; // means access time
//   interface_ip.obj_func_dyn_energy = 0;
//   interface_ip.obj_func_dyn_power = 0;
//   interface_ip.obj_func_leak_power = 0;
//   interface_ip.obj_func_cycle_t = 1;
//   interface_ip.num_rw_ports =
//       debug ? 1 : XML->sys.core[ithCore].number_instruction_fetch_ports;
//   interface_ip.num_rd_ports = 0;
//   interface_ip.num_wr_ports = 0;
//   interface_ip.num_se_rd_ports = 0;
//   interface_ip.num_search_ports =
//       XML->sys.core[ithCore].number_instruction_fetch_ports;
//   icache.missb = new ArrayST(&interface_ip,
//                              "icacheMissBuffer",
//                              Core_device,
//                              coredynp.opt_local,
//                              coredynp.core_ty);
//   icache.area.set_area(icache.area.get_area() +
//                        icache.missb->local_result.area);
//   area.set_area(area.get_area() + icache.missb->local_result.area);
//   // output_data_csv(icache.missb.local_result);

//   // fill buffer
//   tag = XML->sys.physical_address_width + EXTRA_TAG_BITS;
//   data = icache.caches->l_ip.line_sz;
//   interface_ip.specific_tag = 1;
//   interface_ip.tag_w = tag;
//   interface_ip.line_sz = data; // int(pow(2.0,ceil(log2(data))));
//   interface_ip.cache_sz = data * XML->sys.core[ithCore].icache.buffer_sizes[1];
//   interface_ip.assoc = 0;
//   interface_ip.nbanks = 1;
//   interface_ip.out_w = interface_ip.line_sz * 8;
//   interface_ip.access_mode = 0;
//   interface_ip.throughput =
//       debug ? 1.0 / clockRate
//             : XML->sys.core[ithCore].icache.icache_config[4] / clockRate;
//   interface_ip.latency =
//       debug ? 1.0 / clockRate
//             : XML->sys.core[ithCore].icache.icache_config[5] / clockRate;
//   interface_ip.obj_func_dyn_energy = 0;
//   interface_ip.obj_func_dyn_power = 0;
//   interface_ip.obj_func_leak_power = 0;
//   interface_ip.obj_func_cycle_t = 1;
//   interface_ip.num_rw_ports =
//       debug ? 1 : XML->sys.core[ithCore].number_instruction_fetch_ports;
//   interface_ip.num_rd_ports = 0;
//   interface_ip.num_wr_ports = 0;
//   interface_ip.num_se_rd_ports = 0;
//   interface_ip.num_search_ports =
//       XML->sys.core[ithCore].number_instruction_fetch_ports;
//   icache.ifb = new ArrayST(&interface_ip,
//                            "icacheFillBuffer",
//                            Core_device,
//                            coredynp.opt_local,
//                            coredynp.core_ty);
//   icache.area.set_area(icache.area.get_area() + icache.ifb->local_result.area);
//   area.set_area(area.get_area() + icache.ifb->local_result.area);
//   // output_data_csv(icache.ifb.local_result);

//   // prefetch buffer
//   tag = XML->sys.physical_address_width +
//         EXTRA_TAG_BITS; // check with previous entries to decide wthether to
//                         // merge.
//   data = icache.caches->l_ip
//              .line_sz; // separate queue to prevent from cache polution.
//   interface_ip.specific_tag = 1;
//   interface_ip.tag_w = tag;
//   interface_ip.line_sz = data; // int(pow(2.0,ceil(log2(data))));
//   interface_ip.cache_sz =
//       XML->sys.core[ithCore].icache.buffer_sizes[2] * interface_ip.line_sz;
//   interface_ip.assoc = 0;
//   interface_ip.nbanks = 1;
//   interface_ip.out_w = interface_ip.line_sz * 8;
//   interface_ip.access_mode = 0;
//   interface_ip.throughput =
//       debug ? 1.0 / clockRate
//             : XML->sys.core[ithCore].icache.icache_config[4] / clockRate;
//   interface_ip.latency =
//       debug ? 1.0 / clockRate
//             : XML->sys.core[ithCore].icache.icache_config[5] / clockRate;
//   interface_ip.obj_func_dyn_energy = 0;
//   interface_ip.obj_func_dyn_power = 0;
//   interface_ip.obj_func_leak_power = 0;
//   interface_ip.obj_func_cycle_t = 1;
//   interface_ip.num_rw_ports =
//       debug ? 1 : XML->sys.core[ithCore].number_instruction_fetch_ports;
//   interface_ip.num_rd_ports = 0;
//   interface_ip.num_wr_ports = 0;
//   interface_ip.num_se_rd_ports = 0;
//   interface_ip.num_search_ports =
//       XML->sys.core[ithCore].number_instruction_fetch_ports;
//   icache.prefetchb = new ArrayST(&interface_ip,
//                                  "icacheprefetchBuffer",
//                                  Core_device,
//                                  coredynp.opt_local,
//                                  coredynp.core_ty);
//   icache.area.set_area(icache.area.get_area() +
//                        icache.prefetchb->local_result.area);
//   area.set_area(area.get_area() + icache.prefetchb->local_result.area);
//   // output_data_csv(icache.prefetchb.local_result);

//   // Instruction buffer
//   data =
//       XML->sys.core[ithCore].instruction_length *
//       XML->sys.core[ithCore]
//           .peak_issue_width; // icache.caches.l_ip.line_sz; //multiple
//                              // threads timing sharing the instruction buffer.
//   interface_ip.is_cache = false;
//   interface_ip.pure_ram = true;
//   interface_ip.pure_cam = false;
//   interface_ip.line_sz = int(ceil(data / 8.0));
//   interface_ip.cache_sz =
//       XML->sys.core[ithCore].number_hardware_threads *
//                   XML->sys.core[ithCore].instruction_buffer_size *
//                   interface_ip.line_sz >
//               64
//           ? XML->sys.core[ithCore].number_hardware_threads *
//                 XML->sys.core[ithCore].instruction_buffer_size *
//                 interface_ip.line_sz
//           : 64;
//   interface_ip.assoc = 1;
//   interface_ip.nbanks = 1;
//   interface_ip.out_w = interface_ip.line_sz * 8;
//   interface_ip.access_mode = 0;
//   interface_ip.throughput = 1.0 / clockRate;
//   interface_ip.latency = 1.0 / clockRate;
//   interface_ip.obj_func_dyn_energy = 0;
//   interface_ip.obj_func_dyn_power = 0;
//   interface_ip.obj_func_leak_power = 0;
//   interface_ip.obj_func_cycle_t = 1;
//   // NOTE: Assuming IB is time slice shared among threads, every fetch op will
//   // at least fetch "fetch width" instructions.
//   interface_ip.num_rw_ports =
//       debug
//           ? 1
//           : XML->sys.core[ithCore]
//                 .number_instruction_fetch_ports; // XML->sys.core[ithCore].fetch_width;
//   interface_ip.num_rd_ports = 0;
//   interface_ip.num_wr_ports = 0;
//   interface_ip.num_se_rd_ports = 0;
//   IB = new ArrayST(&interface_ip,
//                    "InstBuffer",
//                    Core_device,
//                    coredynp.opt_local,
//                    coredynp.core_ty);
//   IB->area.set_area(IB->area.get_area() + IB->local_result.area);
//   area.set_area(area.get_area() + IB->local_result.area);
//   // output_data_csv(IB.IB.local_result);

//   //	  inst_decoder.opcode_length = XML->sys.core[ithCore].opcode_width;
//   //	  inst_decoder.init_decoder(is_default, &interface_ip);
//   //	  inst_decoder.full_decoder_power();

//   if (coredynp.predictionW > 0) {
//     /*
//      * BTB branch target buffer, accessed during IF stage. Virtually indexed and
//      * virtually tagged It is only a cache without all the buffers in the cache
//      * controller since it is more like a look up table than a cache with cache
//      * controller. When access miss, no load from other places such as main
//      * memory (not actively fill the misses), it is passively updated under two
//      * circumstances: 1)  when BPT@ID stage finds out current is a taken branch
//      * while BTB missed 2)  When BPT@ID stage predicts differently than BTB 3)
//      * When ID stage finds out current instruction is not a branch while BTB had
//      * a hit.(mark as invalid) 4)  when EXEU find out wrong target has been
//      * provided from BTB.
//      *
//      */
//     size = XML->sys.core[ithCore].BTB.BTB_config[0];
//     line = XML->sys.core[ithCore].BTB.BTB_config[1];
//     assoc = XML->sys.core[ithCore].BTB.BTB_config[2];
//     banks = XML->sys.core[ithCore].BTB.BTB_config[3];
//     idx = debug ? 9 : int(ceil(log2(size / line / assoc)));
//     //    	  tag							   =
//     //    debug?51:XML->sys.virtual_address_width-idx-int(ceil(log2(line))) +
//     //    int(ceil(log2(XML->sys.core[ithCore].number_hardware_threads)))
//     //    +EXTRA_TAG_BITS;
//     tag = debug ? 51
//                 : XML->sys.virtual_address_width +
//                       int(ceil(log2(
//                           XML->sys.core[ithCore].number_hardware_threads))) +
//                       EXTRA_TAG_BITS;
//     interface_ip.is_cache = true;
//     interface_ip.pure_ram = false;
//     interface_ip.pure_cam = false;
//     interface_ip.specific_tag = 1;
//     interface_ip.tag_w = tag;
//     interface_ip.cache_sz = debug ? 32768 : size;
//     interface_ip.line_sz = debug ? 64 : line;
//     interface_ip.assoc = debug ? 8 : assoc;
//     interface_ip.nbanks = debug ? 1 : banks;
//     interface_ip.out_w = interface_ip.line_sz * 8;
//     interface_ip.access_mode =
//         0; // debug?0:XML->sys.core[ithCore].dcache.dcache_config[5];
//     interface_ip.throughput =
//         debug ? 1.0 / clockRate
//               : XML->sys.core[ithCore].BTB.BTB_config[4] / clockRate;
//     interface_ip.latency =
//         debug ? 3.0 / clockRate
//               : XML->sys.core[ithCore].BTB.BTB_config[5] / clockRate;
//     interface_ip.obj_func_dyn_energy = 0;
//     interface_ip.obj_func_dyn_power = 0;
//     interface_ip.obj_func_leak_power = 0;
//     interface_ip.obj_func_cycle_t = 1;
//     interface_ip.num_rw_ports = 1;
//     interface_ip.num_rd_ports = coredynp.predictionW;
//     interface_ip.num_wr_ports = coredynp.predictionW;
//     interface_ip.num_se_rd_ports = 0;
//     BTB = new ArrayST(&interface_ip,
//                       "Branch Target Buffer",
//                       Core_device,
//                       coredynp.opt_local,
//                       coredynp.core_ty);
//     BTB->area.set_area(BTB->area.get_area() + BTB->local_result.area);
//     area.set_area(area.get_area() + BTB->local_result.area);
//     /// cout<<"area="<<area<<endl;

//     BPT = new BranchPredictor();
//     BPT->set_params(XML, ithCore, &interface_ip, coredynp);
//     BPT->computeArea();
//     BPT->set_stats(XML);
//     area.set_area(area.get_area() + BPT->area.get_area());
//   }

//   ID_inst.set_params(is_default,
//                              &interface_ip,
//                              coredynp.opcode_length,
//                              1 /*Decoder should not know how many by itself*/,
//                              coredynp.x86,
//                              Core_device,
//                              coredynp.core_ty);

//   ID_operand.set_params(is_default,
//                                 &interface_ip,
//                                 coredynp.arch_ireg_width,
//                                 1,
//                                 coredynp.x86,
//                                 Core_device,
//                                 coredynp.core_ty);

//   ID_misc.set_params(is_default,
//                              &interface_ip,
//                              8 /* Prefix field etc upto 14B*/,
//                              1,
//                              coredynp.x86,
//                              Core_device,
//                              coredynp.core_ty);
//   ID_inst.computeArea();
//   ID_inst.computeDynamicPower();
//   ID_operand.computeArea();
//   ID_operand.computeDynamicPower();
//   ID_misc.computeArea();
//   ID_misc.computeDynamicPower();
//   // TODO: X86 decoder should decode the inst in cyclic mode under the control
//   // of squencer. So the dynamic power should be multiplied by a few times.
//   area.set_area(area.get_area() +
//                 (ID_inst.area.get_area() + ID_operand.area.get_area() +
//                  ID_misc.area.get_area()) *
//                     coredynp.decodeW);
// }

// void InstFetchU::computeEnergy(bool is_tdp) {
//   if (!exist)
//     return;
//   if (is_tdp) {
//     // init stats for Peak
//     icache.caches->stats_t.readAc.access =
//         icache.caches->l_ip.num_rw_ports * coredynp.IFU_duty_cycle;
//     icache.caches->stats_t.readAc.miss = 0;
//     icache.caches->stats_t.readAc.hit = icache.caches->stats_t.readAc.access -
//                                         icache.caches->stats_t.readAc.miss;
//     icache.caches->tdp_stats = icache.caches->stats_t;

//     icache.missb->stats_t.readAc.access = icache.missb->stats_t.readAc.hit =
//         icache.missb->l_ip.num_search_ports * coredynp.IFU_duty_cycle;
//     icache.missb->stats_t.writeAc.access = icache.missb->stats_t.writeAc.hit =
//         icache.missb->l_ip.num_search_ports * coredynp.IFU_duty_cycle;
//     icache.missb->tdp_stats = icache.missb->stats_t;

//     icache.ifb->stats_t.readAc.access = icache.ifb->stats_t.readAc.hit =
//         icache.ifb->l_ip.num_search_ports * coredynp.IFU_duty_cycle;
//     icache.ifb->stats_t.writeAc.access = icache.ifb->stats_t.writeAc.hit =
//         icache.ifb->l_ip.num_search_ports * coredynp.IFU_duty_cycle;
//     icache.ifb->tdp_stats = icache.ifb->stats_t;

//     icache.prefetchb->stats_t.readAc.access =
//         icache.prefetchb->stats_t.readAc.hit =
//             icache.prefetchb->l_ip.num_search_ports * coredynp.IFU_duty_cycle;
//     icache.prefetchb->stats_t.writeAc.access = icache.ifb->stats_t.writeAc.hit =
//         icache.ifb->l_ip.num_search_ports * coredynp.IFU_duty_cycle;
//     icache.prefetchb->tdp_stats = icache.prefetchb->stats_t;

//     IB->stats_t.readAc.access = IB->stats_t.writeAc.access =
//         XML->sys.core[ithCore].peak_issue_width;
//     IB->tdp_stats = IB->stats_t;

//     if (coredynp.predictionW > 0) {
//       BTB->stats_t.readAc.access =
//           coredynp.predictionW; // XML->sys.core[ithCore].BTB.read_accesses;
//       BTB->stats_t.writeAc.access =
//           0; // XML->sys.core[ithCore].BTB.write_accesses;
//     }

//     ID_inst.stats_t.readAc.access = coredynp.decodeW;
//     ID_operand.stats_t.readAc.access = coredynp.decodeW;
//     ID_misc.stats_t.readAc.access = coredynp.decodeW;
//     ID_inst.tdp_stats = ID_inst.stats_t;
//     ID_operand.tdp_stats = ID_operand.stats_t;
//     ID_misc.tdp_stats = ID_misc.stats_t;

//   } else {
//     // init stats for Runtime Dynamic (RTP)
//     icache.caches->stats_t.readAc.access =
//         XML->sys.core[ithCore].icache.read_accesses;
//     icache.caches->stats_t.readAc.miss =
//         XML->sys.core[ithCore].icache.read_misses;
//     icache.caches->stats_t.readAc.hit = icache.caches->stats_t.readAc.access -
//                                         icache.caches->stats_t.readAc.miss;
//     icache.caches->rtp_stats = icache.caches->stats_t;

//     icache.missb->stats_t.readAc.access = icache.caches->stats_t.readAc.miss;
//     icache.missb->stats_t.writeAc.access = icache.caches->stats_t.readAc.miss;
//     icache.missb->rtp_stats = icache.missb->stats_t;

//     icache.ifb->stats_t.readAc.access = icache.caches->stats_t.readAc.miss;
//     icache.ifb->stats_t.writeAc.access = icache.caches->stats_t.readAc.miss;
//     icache.ifb->rtp_stats = icache.ifb->stats_t;

//     icache.prefetchb->stats_t.readAc.access =
//         icache.caches->stats_t.readAc.miss;
//     icache.prefetchb->stats_t.writeAc.access =
//         icache.caches->stats_t.readAc.miss;
//     icache.prefetchb->rtp_stats = icache.prefetchb->stats_t;

//     IB->stats_t.readAc.access = IB->stats_t.writeAc.access =
//         XML->sys.core[ithCore].total_instructions;
//     IB->rtp_stats = IB->stats_t;

//     if (coredynp.predictionW > 0) {
//       BTB->stats_t.readAc.access =
//           XML->sys.core[ithCore]
//               .BTB.read_accesses; // XML->sys.core[ithCore].branch_instructions;
//       BTB->stats_t.writeAc.access =
//           XML->sys.core[ithCore]
//               .BTB
//               .write_accesses; // XML->sys.core[ithCore].branch_mispredictions;
//       BTB->rtp_stats = BTB->stats_t;
//     }

//     ID_inst.stats_t.readAc.access = XML->sys.core[ithCore].total_instructions;
//     ID_operand.stats_t.readAc.access =
//         XML->sys.core[ithCore].total_instructions;
//     ID_misc.stats_t.readAc.access = XML->sys.core[ithCore].total_instructions;
//     ID_inst.rtp_stats = ID_inst.stats_t;
//     ID_operand.rtp_stats = ID_operand.stats_t;
//     ID_misc.rtp_stats = ID_misc.stats_t;
//   }

//   icache.power_t.reset();
//   IB->power_t.reset();
//   //	ID_inst.power_t.reset();
//   //	ID_operand.power_t.reset();
//   //	ID_misc.power_t.reset();
//   if (coredynp.predictionW > 0) {
//     BTB->power_t.reset();
//   }

//   icache.power_t.readOp.dynamic +=
//       (icache.caches->stats_t.readAc.hit *
//            icache.caches->local_result.power.readOp.dynamic +
//        // icache.caches->stats_t.readAc.miss*icache.caches->local_result.tag_array2->power.readOp.dynamic+
//        icache.caches->stats_t.readAc.miss *
//            icache.caches->local_result.power.readOp
//                .dynamic + // assume tag data accessed in parallel
//        icache.caches->stats_t.readAc.miss *
//            icache.caches->local_result.power.writeOp
//                .dynamic); // read miss in Icache cause a write to Icache
//   icache.power_t.readOp.dynamic +=
//       icache.missb->stats_t.readAc.access *
//           icache.missb->local_result.power.searchOp.dynamic +
//       icache.missb->stats_t.writeAc.access *
//           icache.missb->local_result.power.writeOp
//               .dynamic; // each access to missb involves a CAM and a write
//   icache.power_t.readOp.dynamic +=
//       icache.ifb->stats_t.readAc.access *
//           icache.ifb->local_result.power.searchOp.dynamic +
//       icache.ifb->stats_t.writeAc.access *
//           icache.ifb->local_result.power.writeOp.dynamic;
//   icache.power_t.readOp.dynamic +=
//       icache.prefetchb->stats_t.readAc.access *
//           icache.prefetchb->local_result.power.searchOp.dynamic +
//       icache.prefetchb->stats_t.writeAc.access *
//           icache.prefetchb->local_result.power.writeOp.dynamic;

//   IB->power_t.readOp.dynamic +=
//       IB->local_result.power.readOp.dynamic * IB->stats_t.readAc.access +
//       IB->stats_t.writeAc.access * IB->local_result.power.writeOp.dynamic;

//   if (coredynp.predictionW > 0) {
//     BTB->power_t.readOp.dynamic +=
//         BTB->local_result.power.readOp.dynamic * BTB->stats_t.readAc.access +
//         BTB->stats_t.writeAc.access * BTB->local_result.power.writeOp.dynamic;

//     BPT->computeDynamicPower(is_tdp);
//   }

//   if (is_tdp) {
//     //    	icache.power = icache.power_t +
//     //    	        (icache.caches->local_result.power)*pppm_lkg +
//     //    			(icache.missb->local_result.power +
//     //    			icache.ifb->local_result.power +
//     //    			icache.prefetchb->local_result.power)*pppm_Isub;
//     icache.power = icache.power_t + (icache.caches->local_result.power +
//                                      icache.missb->local_result.power +
//                                      icache.ifb->local_result.power +
//                                      icache.prefetchb->local_result.power) *
//                                         pppm_lkg;

//     IB->power = IB->power_t + IB->local_result.power * pppm_lkg;
//     power = power + icache.power + IB->power;
//     if (coredynp.predictionW > 0) {
//       BTB->power = BTB->power_t + BTB->local_result.power * pppm_lkg;
//       power = power + BTB->power + BPT->power;
//     }

//     ID_inst.power_t.readOp.dynamic = ID_inst.power.readOp.dynamic;
//     ID_operand.power_t.readOp.dynamic = ID_operand.power.readOp.dynamic;
//     ID_misc.power_t.readOp.dynamic = ID_misc.power.readOp.dynamic;

//     ID_inst.power.readOp.dynamic *= ID_inst.tdp_stats.readAc.access;
//     ID_operand.power.readOp.dynamic *= ID_operand.tdp_stats.readAc.access;
//     ID_misc.power.readOp.dynamic *= ID_misc.tdp_stats.readAc.access;

//     power = power + (ID_inst.power + ID_operand.power + ID_misc.power);
//   } else {
//     //    	icache.rt_power = icache.power_t +
//     //    	        (icache.caches->local_result.power)*pppm_lkg +
//     //    			(icache.missb->local_result.power +
//     //    			icache.ifb->local_result.power +
//     //    			icache.prefetchb->local_result.power)*pppm_Isub;

//     icache.rt_power = icache.power_t + (icache.caches->local_result.power +
//                                         icache.missb->local_result.power +
//                                         icache.ifb->local_result.power +
//                                         icache.prefetchb->local_result.power) *
//                                            pppm_lkg;

//     IB->rt_power = IB->power_t + IB->local_result.power * pppm_lkg;
//     rt_power = rt_power + icache.rt_power + IB->rt_power;
//     if (coredynp.predictionW > 0) {
//       BTB->rt_power = BTB->power_t + BTB->local_result.power * pppm_lkg;
//       rt_power = rt_power + BTB->rt_power + BPT->rt_power;
//     }

//     ID_inst.rt_power.readOp.dynamic =
//         ID_inst.power_t.readOp.dynamic * ID_inst.rtp_stats.readAc.access;
//     ID_operand.rt_power.readOp.dynamic = ID_operand.power_t.readOp.dynamic *
//                                           ID_operand.rtp_stats.readAc.access;
//     ID_misc.rt_power.readOp.dynamic =
//         ID_misc.power_t.readOp.dynamic * ID_misc.rtp_stats.readAc.access;

//     rt_power = rt_power +
//                (ID_inst.rt_power + ID_operand.rt_power + ID_misc.rt_power);
//   }
// }

// void InstFetchU::displayEnergy(uint32_t indent, int plevel, bool is_tdp) {
//   if (!exist)
//     return;
//   string indent_str(indent, ' ');
//   string indent_str_next(indent + 2, ' ');
//   bool long_channel = XML->sys.longer_channel_device;
//   bool power_gating = XML->sys.power_gating;

//   if (is_tdp) {

//     cout << indent_str << "Instruction Cache:" << endl;
//     cout << indent_str_next << "Area = " << icache.area.get_area() * 1e-6
//          << " mm^2" << endl;
//     cout << indent_str_next
//          << "Peak Dynamic = " << icache.power.readOp.dynamic * clockRate << " W"
//          << endl;
//     cout << indent_str_next << "Subthreshold Leakage = "
//          << (long_channel ? icache.power.readOp.longer_channel_leakage
//                           : icache.power.readOp.leakage)
//          << " W" << endl;
//     if (power_gating)
//       cout << indent_str_next << "Subthreshold Leakage with power gating = "
//            << (long_channel
//                    ? icache.power.readOp.power_gated_with_long_channel_leakage
//                    : icache.power.readOp.power_gated_leakage)
//            << " W" << endl;
//     cout << indent_str_next
//          << "Gate Leakage = " << icache.power.readOp.gate_leakage << " W"
//          << endl;
//     cout << indent_str_next << "Runtime Dynamic = "
//          << icache.rt_power.readOp.dynamic / executionTime << " W" << endl;
//     cout << endl;
//     if (coredynp.predictionW > 0) {
//       cout << indent_str << "Branch Target Buffer:" << endl;
//       cout << indent_str_next << "Area = " << BTB->area.get_area() * 1e-6
//            << " mm^2" << endl;
//       cout << indent_str_next
//            << "Peak Dynamic = " << BTB->power.readOp.dynamic * clockRate << " W"
//            << endl;
//       cout << indent_str_next << "Subthreshold Leakage = "
//            << (long_channel ? BTB->power.readOp.longer_channel_leakage
//                             : BTB->power.readOp.leakage)
//            << " W" << endl;
//       if (power_gating)
//         cout << indent_str_next << "Subthreshold Leakage with power gating = "
//              << (long_channel
//                      ? BTB->power.readOp.power_gated_with_long_channel_leakage
//                      : BTB->power.readOp.power_gated_leakage)
//              << " W" << endl;
//       cout << indent_str_next
//            << "Gate Leakage = " << BTB->power.readOp.gate_leakage << " W"
//            << endl;
//       cout << indent_str_next << "Runtime Dynamic = "
//            << BTB->rt_power.readOp.dynamic / executionTime << " W" << endl;
//       cout << endl;
//       if (BPT->exist) {
//         cout << indent_str << "Branch Predictor:" << endl;
//         cout << indent_str_next << "Area = " << BPT->area.get_area() * 1e-6
//              << " mm^2" << endl;
//         cout << indent_str_next
//              << "Peak Dynamic = " << BPT->power.readOp.dynamic * clockRate
//              << " W" << endl;
//         cout << indent_str_next << "Subthreshold Leakage = "
//              << (long_channel ? BPT->power.readOp.longer_channel_leakage
//                               : BPT->power.readOp.leakage)
//              << " W" << endl;
//         if (power_gating)
//           cout << indent_str_next << "Subthreshold Leakage with power gating = "
//                << (long_channel
//                        ? BPT->power.readOp.power_gated_with_long_channel_leakage
//                        : BPT->power.readOp.power_gated_leakage)
//                << " W" << endl;
//         cout << indent_str_next
//              << "Gate Leakage = " << BPT->power.readOp.gate_leakage << " W"
//              << endl;
//         cout << indent_str_next << "Runtime Dynamic = "
//              << BPT->rt_power.readOp.dynamic / executionTime << " W" << endl;
//         cout << endl;
//         if (plevel > 3) {
//           BPT->displayEnergy(indent + 4, plevel, is_tdp);
//         }
//       }
//     }
//     cout << indent_str << "Instruction Buffer:" << endl;
//     cout << indent_str_next << "Area = " << IB->area.get_area() * 1e-6
//          << " mm^2" << endl;
//     cout << indent_str_next
//          << "Peak Dynamic = " << IB->power.readOp.dynamic * clockRate << " W"
//          << endl;
//     cout << indent_str_next << "Subthreshold Leakage = "
//          << (long_channel ? IB->power.readOp.longer_channel_leakage
//                           : IB->power.readOp.leakage)
//          << " W" << endl;
//     if (power_gating)
//       cout << indent_str_next << "Subthreshold Leakage with power gating = "
//            << (long_channel
//                    ? IB->power.readOp.power_gated_with_long_channel_leakage
//                    : IB->power.readOp.power_gated_leakage)
//            << " W" << endl;
//     cout << indent_str_next
//          << "Gate Leakage = " << IB->power.readOp.gate_leakage << " W" << endl;
//     cout << indent_str_next
//          << "Runtime Dynamic = " << IB->rt_power.readOp.dynamic / executionTime
//          << " W" << endl;
//     cout << endl;
//     cout << indent_str << "Instruction Decoder:" << endl;
//     cout << indent_str_next << "Area = "
//          << (ID_inst.area.get_area() + ID_operand.area.get_area() +
//              ID_misc.area.get_area()) *
//                 coredynp.decodeW * 1e-6
//          << " mm^2" << endl;
//     cout << indent_str_next << "Peak Dynamic = "
//          << (ID_inst.power.readOp.dynamic + ID_operand.power.readOp.dynamic +
//              ID_misc.power.readOp.dynamic) *
//                 clockRate
//          << " W" << endl;
//     cout << indent_str_next << "Subthreshold Leakage = "
//          << (long_channel ? (ID_inst.power.readOp.longer_channel_leakage +
//                              ID_operand.power.readOp.longer_channel_leakage +
//                              ID_misc.power.readOp.longer_channel_leakage)
//                           : (ID_inst.power.readOp.leakage +
//                              ID_operand.power.readOp.leakage +
//                              ID_misc.power.readOp.leakage))
//          << " W" << endl;

//     double tot_leakage =
//         (ID_inst.power.readOp.leakage + ID_operand.power.readOp.leakage +
//          ID_misc.power.readOp.leakage);
//     double tot_leakage_longchannel =
//         (ID_inst.power.readOp.longer_channel_leakage +
//          ID_operand.power.readOp.longer_channel_leakage +
//          ID_misc.power.readOp.longer_channel_leakage);
//     double tot_leakage_pg = (ID_inst.power.readOp.power_gated_leakage +
//                              ID_operand.power.readOp.power_gated_leakage +
//                              ID_misc.power.readOp.power_gated_leakage);
//     double tot_leakage_pg_with_long_channel =
//         (ID_inst.power.readOp.power_gated_with_long_channel_leakage +
//          ID_operand.power.readOp.power_gated_with_long_channel_leakage +
//          ID_misc.power.readOp.power_gated_with_long_channel_leakage);

//     if (power_gating)
//       cout << indent_str_next << "Subthreshold Leakage with power gating = "
//            << (long_channel ? tot_leakage_pg_with_long_channel : tot_leakage_pg)
//            << " W" << endl;
//     cout << indent_str_next << "Gate Leakage = "
//          << (ID_inst.power.readOp.gate_leakage +
//              ID_operand.power.readOp.gate_leakage +
//              ID_misc.power.readOp.gate_leakage)
//          << " W" << endl;
//     cout << indent_str_next << "Runtime Dynamic = "
//          << (ID_inst.rt_power.readOp.dynamic +
//              ID_operand.rt_power.readOp.dynamic +
//              ID_misc.rt_power.readOp.dynamic) /
//                 executionTime
//          << " W" << endl;
//     cout << endl;
//   } else {
//     //		cout << indent_str_next << "Instruction Cache    Peak Dynamic = "
//     //<< icache.rt_power.readOp.dynamic*clockRate << " W" << endl;
//     // cout << indent_str_next << "Instruction Cache    Subthreshold Leakage = "
//     // << icache.rt_power.readOp.leakage <<" W" << endl; 		cout <<
//     // indent_str_next << "Instruction Cache    Gate Leakage = " <<
//     // icache.rt_power.readOp.gate_leakage << " W" << endl; 		cout <<
//     // indent_str_next << "Instruction Buffer   Peak Dynamic = " <<
//     // IB->rt_power.readOp.dynamic*clockRate  << " W" << endl; 		cout <<
//     // indent_str_next << "Instruction Buffer   Subthreshold Leakage = " <<
//     // IB->rt_power.readOp.leakage  << " W" << endl; 		cout << indent_str_next
//     // << "Instruction Buffer   Gate Leakage = " <<
//     // IB->rt_power.readOp.gate_leakage
//     //<< " W" << endl; 		cout << indent_str_next << "Branch Target Buffer
//     // Peak Dynamic = " << BTB->rt_power.readOp.dynamic*clockRate  << " W" <<
//     // endl; 		cout << indent_str_next << "Branch Target Buffer   Subthreshold
//     // Leakage = " << BTB->rt_power.readOp.leakage  << " W" << endl; 		cout
//     // << indent_str_next << "Branch Target Buffer   Gate Leakage = " <<
//     // BTB->rt_power.readOp.gate_leakage  << " W" << endl; 		cout <<
//     // indent_str_next << "Branch Predictor   Peak Dynamic = " <<
//     // BPT->rt_power.readOp.dynamic*clockRate  << " W" << endl; 		cout
//     // << indent_str_next << "Branch Predictor   Subthreshold Leakage = " <<
//     // BPT->rt_power.readOp.leakage  << " W" << endl; 		cout <<
//     // indent_str_next
//     // << "Branch Predictor   Gate Leakage = " <<
//     // BPT->rt_power.readOp.gate_leakage
//     //<< " W" << endl;
//   }
// }

// InstFetchU ::~InstFetchU() {

//   if (!exist)
//     return;
//   if (IB) {
//     delete IB;
//     IB = 0;
//   }
//   if (coredynp.predictionW > 0) {
//     if (BTB) {
//       delete BTB;
//       BTB = 0;
//     }
//     if (BPT) {
//       delete BPT;
//       BPT = 0;
//     }
//   }
// }


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

#include "instfetch.h"

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

InstFetchU::InstFetchU(const ParseXML *XML_interface,
                       int ithCore_,
                       InputParameter *interface_ip_,
                       const CoreDynParam &dyn_p_,
                       bool exist_)
    : XML(XML_interface), ithCore(ithCore_), interface_ip(*interface_ip_),
      coredynp(dyn_p_), IB(0), BTB(0), ID_inst(0), ID_operand(0), ID_misc(0),
      exist(exist_) {
  if (!exist)
    return;
  int idx, tag, data, size, line, assoc, banks;
  bool debug = false, is_default = true;

  clockRate = coredynp.clockRate;
  executionTime = coredynp.executionTime;
  cache_p = (Cache_policy)XML->sys.core[ithCore].icache.icache_config[7];
  // Assuming all L1 caches are virtually idxed physically tagged.
  // cache

  size = (int)XML->sys.core[ithCore].icache.icache_config[0];
  line = (int)XML->sys.core[ithCore].icache.icache_config[1];
  assoc = (int)XML->sys.core[ithCore].icache.icache_config[2];
  banks = (int)XML->sys.core[ithCore].icache.icache_config[3];
  idx = debug ? 9 : int(ceil(log2(size / line / assoc)));
  tag = debug ? 51
              : (int)XML->sys.physical_address_width - idx -
                    int(ceil(log2(line))) + EXTRA_TAG_BITS;
  interface_ip.specific_tag = 1;
  interface_ip.tag_w = tag;
  interface_ip.cache_sz =
      debug ? 32768 : (int)XML->sys.core[ithCore].icache.icache_config[0];
  interface_ip.line_sz =
      debug ? 64 : (int)XML->sys.core[ithCore].icache.icache_config[1];
  interface_ip.assoc =
      debug ? 8 : (int)XML->sys.core[ithCore].icache.icache_config[2];
  interface_ip.nbanks =
      debug ? 1 : (int)XML->sys.core[ithCore].icache.icache_config[3];
  interface_ip.out_w = interface_ip.line_sz * 8;
  interface_ip.access_mode =
      0; // debug?0:XML->sys.core[ithCore].icache.icache_config[5];
  interface_ip.throughput =
      debug ? 1.0 / clockRate
            : XML->sys.core[ithCore].icache.icache_config[4] / clockRate;
  interface_ip.latency =
      debug ? 3.0 / clockRate
            : XML->sys.core[ithCore].icache.icache_config[5] / clockRate;
  interface_ip.is_cache = true;
  interface_ip.pure_cam = false;
  interface_ip.pure_ram = false;
  //  interface_ip.obj_func_dyn_energy = 0;
  //  interface_ip.obj_func_dyn_power  = 0;
  //  interface_ip.obj_func_leak_power = 0;
  //  interface_ip.obj_func_cycle_t    = 1;
  interface_ip.num_rw_ports =
      debug ? 1 : XML->sys.core[ithCore].number_instruction_fetch_ports;
  interface_ip.num_rd_ports = 0;
  interface_ip.num_wr_ports = 0;
  interface_ip.num_se_rd_ports = 0;
  icache.caches = new ArrayST(&interface_ip,
                              "icache",
                              Core_device,
                              coredynp.opt_local,
                              coredynp.core_ty);
  scktRatio = g_tp.sckt_co_eff;
  chip_PR_overhead = g_tp.chip_layout_overhead;
  macro_PR_overhead = g_tp.macro_layout_overhead;
  icache.area.set_area(icache.area.get_area() +
                       icache.caches->local_result.area);
  area.set_area(area.get_area() + icache.caches->local_result.area);
  // output_data_csv(icache.caches.local_result);

  /*
   *iCache controllers
   *miss buffer Each MSHR contains enough state
   *to handle one or more accesses of any type to a single memory line.
   *Due to the generality of the MSHR mechanism,
   *the amount of state involved is non-trivial:
   *including the address, pointers to the cache entry and destination register,
   *written data, and various other pieces of state.
   */
  interface_ip.num_search_ports =
      debug ? 1 : XML->sys.core[ithCore].number_instruction_fetch_ports;
  tag = XML->sys.physical_address_width + EXTRA_TAG_BITS;
  data = (XML->sys.physical_address_width) + int(ceil(log2(size / line))) +
         icache.caches->l_ip.line_sz * 8;
  interface_ip.specific_tag = 1;
  interface_ip.tag_w = tag;
  interface_ip.line_sz =
      int(ceil(data / 8.0)); // int(ceil(pow(2.0,ceil(log2(data)))/8.0));
  interface_ip.cache_sz =
      XML->sys.core[ithCore].icache.buffer_sizes[0] * interface_ip.line_sz;
  interface_ip.assoc = 0;
  interface_ip.nbanks = 1;
  interface_ip.out_w = interface_ip.line_sz * 8;
  interface_ip.access_mode = 0;
  interface_ip.throughput =
      debug ? 1.0 / clockRate
            : XML->sys.core[ithCore].icache.icache_config[4] /
                  clockRate; // means cycle time
  interface_ip.latency = debug
                             ? 1.0 / clockRate
                             : XML->sys.core[ithCore].icache.icache_config[5] /
                                   clockRate; // means access time
  interface_ip.obj_func_dyn_energy = 0;
  interface_ip.obj_func_dyn_power = 0;
  interface_ip.obj_func_leak_power = 0;
  interface_ip.obj_func_cycle_t = 1;
  interface_ip.num_rw_ports =
      debug ? 1 : XML->sys.core[ithCore].number_instruction_fetch_ports;
  interface_ip.num_rd_ports = 0;
  interface_ip.num_wr_ports = 0;
  interface_ip.num_se_rd_ports = 0;
  interface_ip.num_search_ports =
      XML->sys.core[ithCore].number_instruction_fetch_ports;
  icache.missb = new ArrayST(&interface_ip,
                             "icacheMissBuffer",
                             Core_device,
                             coredynp.opt_local,
                             coredynp.core_ty);
  icache.area.set_area(icache.area.get_area() +
                       icache.missb->local_result.area);
  area.set_area(area.get_area() + icache.missb->local_result.area);
  // output_data_csv(icache.missb.local_result);

  // fill buffer
  tag = XML->sys.physical_address_width + EXTRA_TAG_BITS;
  data = icache.caches->l_ip.line_sz;
  interface_ip.specific_tag = 1;
  interface_ip.tag_w = tag;
  interface_ip.line_sz = data; // int(pow(2.0,ceil(log2(data))));
  interface_ip.cache_sz = data * XML->sys.core[ithCore].icache.buffer_sizes[1];
  interface_ip.assoc = 0;
  interface_ip.nbanks = 1;
  interface_ip.out_w = interface_ip.line_sz * 8;
  interface_ip.access_mode = 0;
  interface_ip.throughput =
      debug ? 1.0 / clockRate
            : XML->sys.core[ithCore].icache.icache_config[4] / clockRate;
  interface_ip.latency =
      debug ? 1.0 / clockRate
            : XML->sys.core[ithCore].icache.icache_config[5] / clockRate;
  interface_ip.obj_func_dyn_energy = 0;
  interface_ip.obj_func_dyn_power = 0;
  interface_ip.obj_func_leak_power = 0;
  interface_ip.obj_func_cycle_t = 1;
  interface_ip.num_rw_ports =
      debug ? 1 : XML->sys.core[ithCore].number_instruction_fetch_ports;
  interface_ip.num_rd_ports = 0;
  interface_ip.num_wr_ports = 0;
  interface_ip.num_se_rd_ports = 0;
  interface_ip.num_search_ports =
      XML->sys.core[ithCore].number_instruction_fetch_ports;
  icache.ifb = new ArrayST(&interface_ip,
                           "icacheFillBuffer",
                           Core_device,
                           coredynp.opt_local,
                           coredynp.core_ty);
  icache.area.set_area(icache.area.get_area() + icache.ifb->local_result.area);
  area.set_area(area.get_area() + icache.ifb->local_result.area);
  // output_data_csv(icache.ifb.local_result);

  // prefetch buffer
  tag = XML->sys.physical_address_width +
        EXTRA_TAG_BITS; // check with previous entries to decide wthether to
                        // merge.
  data = icache.caches->l_ip
             .line_sz; // separate queue to prevent from cache polution.
  interface_ip.specific_tag = 1;
  interface_ip.tag_w = tag;
  interface_ip.line_sz = data; // int(pow(2.0,ceil(log2(data))));
  interface_ip.cache_sz =
      XML->sys.core[ithCore].icache.buffer_sizes[2] * interface_ip.line_sz;
  interface_ip.assoc = 0;
  interface_ip.nbanks = 1;
  interface_ip.out_w = interface_ip.line_sz * 8;
  interface_ip.access_mode = 0;
  interface_ip.throughput =
      debug ? 1.0 / clockRate
            : XML->sys.core[ithCore].icache.icache_config[4] / clockRate;
  interface_ip.latency =
      debug ? 1.0 / clockRate
            : XML->sys.core[ithCore].icache.icache_config[5] / clockRate;
  interface_ip.obj_func_dyn_energy = 0;
  interface_ip.obj_func_dyn_power = 0;
  interface_ip.obj_func_leak_power = 0;
  interface_ip.obj_func_cycle_t = 1;
  interface_ip.num_rw_ports =
      debug ? 1 : XML->sys.core[ithCore].number_instruction_fetch_ports;
  interface_ip.num_rd_ports = 0;
  interface_ip.num_wr_ports = 0;
  interface_ip.num_se_rd_ports = 0;
  interface_ip.num_search_ports =
      XML->sys.core[ithCore].number_instruction_fetch_ports;
  icache.prefetchb = new ArrayST(&interface_ip,
                                 "icacheprefetchBuffer",
                                 Core_device,
                                 coredynp.opt_local,
                                 coredynp.core_ty);
  icache.area.set_area(icache.area.get_area() +
                       icache.prefetchb->local_result.area);
  area.set_area(area.get_area() + icache.prefetchb->local_result.area);
  // output_data_csv(icache.prefetchb.local_result);

  // Instruction buffer
  data =
      XML->sys.core[ithCore].instruction_length *
      XML->sys.core[ithCore]
          .peak_issue_width; // icache.caches.l_ip.line_sz; //multiple
                             // threads timing sharing the instruction buffer.
  interface_ip.is_cache = false;
  interface_ip.pure_ram = true;
  interface_ip.pure_cam = false;
  interface_ip.line_sz = int(ceil(data / 8.0));
  interface_ip.cache_sz =
      XML->sys.core[ithCore].number_hardware_threads *
                  XML->sys.core[ithCore].instruction_buffer_size *
                  interface_ip.line_sz >
              64
          ? XML->sys.core[ithCore].number_hardware_threads *
                XML->sys.core[ithCore].instruction_buffer_size *
                interface_ip.line_sz
          : 64;
  interface_ip.assoc = 1;
  interface_ip.nbanks = 1;
  interface_ip.out_w = interface_ip.line_sz * 8;
  interface_ip.access_mode = 0;
  interface_ip.throughput = 1.0 / clockRate;
  interface_ip.latency = 1.0 / clockRate;
  interface_ip.obj_func_dyn_energy = 0;
  interface_ip.obj_func_dyn_power = 0;
  interface_ip.obj_func_leak_power = 0;
  interface_ip.obj_func_cycle_t = 1;
  // NOTE: Assuming IB is time slice shared among threads, every fetch op will
  // at least fetch "fetch width" instructions.
  interface_ip.num_rw_ports =
      debug
          ? 1
          : XML->sys.core[ithCore]
                .number_instruction_fetch_ports; // XML->sys.core[ithCore].fetch_width;
  interface_ip.num_rd_ports = 0;
  interface_ip.num_wr_ports = 0;
  interface_ip.num_se_rd_ports = 0;
  IB = new ArrayST(&interface_ip,
                   "InstBuffer",
                   Core_device,
                   coredynp.opt_local,
                   coredynp.core_ty);
  IB->area.set_area(IB->area.get_area() + IB->local_result.area);
  area.set_area(area.get_area() + IB->local_result.area);
  // output_data_csv(IB.IB.local_result);

  //	  inst_decoder.opcode_length = XML->sys.core[ithCore].opcode_width;
  //	  inst_decoder.init_decoder(is_default, &interface_ip);
  //	  inst_decoder.full_decoder_power();

  if (coredynp.predictionW > 0) {
    /*
     * BTB branch target buffer, accessed during IF stage. Virtually indexed and
     * virtually tagged It is only a cache without all the buffers in the cache
     * controller since it is more like a look up table than a cache with cache
     * controller. When access miss, no load from other places such as main
     * memory (not actively fill the misses), it is passively updated under two
     * circumstances: 1)  when BPT@ID stage finds out current is a taken branch
     * while BTB missed 2)  When BPT@ID stage predicts differently than BTB 3)
     * When ID stage finds out current instruction is not a branch while BTB had
     * a hit.(mark as invalid) 4)  when EXEU find out wrong target has been
     * provided from BTB.
     *
     */
    size = XML->sys.core[ithCore].BTB.BTB_config[0];
    line = XML->sys.core[ithCore].BTB.BTB_config[1];
    assoc = XML->sys.core[ithCore].BTB.BTB_config[2];
    banks = XML->sys.core[ithCore].BTB.BTB_config[3];
    idx = debug ? 9 : int(ceil(log2(size / line / assoc)));
    //    	  tag							   =
    //    debug?51:XML->sys.virtual_address_width-idx-int(ceil(log2(line))) +
    //    int(ceil(log2(XML->sys.core[ithCore].number_hardware_threads)))
    //    +EXTRA_TAG_BITS;
    tag = debug ? 51
                : XML->sys.virtual_address_width +
                      int(ceil(log2(
                          XML->sys.core[ithCore].number_hardware_threads))) +
                      EXTRA_TAG_BITS;
    interface_ip.is_cache = true;
    interface_ip.pure_ram = false;
    interface_ip.pure_cam = false;
    interface_ip.specific_tag = 1;
    interface_ip.tag_w = tag;
    interface_ip.cache_sz = debug ? 32768 : size;
    interface_ip.line_sz = debug ? 64 : line;
    interface_ip.assoc = debug ? 8 : assoc;
    interface_ip.nbanks = debug ? 1 : banks;
    interface_ip.out_w = interface_ip.line_sz * 8;
    interface_ip.access_mode =
        0; // debug?0:XML->sys.core[ithCore].dcache.dcache_config[5];
    interface_ip.throughput =
        debug ? 1.0 / clockRate
              : XML->sys.core[ithCore].BTB.BTB_config[4] / clockRate;
    interface_ip.latency =
        debug ? 3.0 / clockRate
              : XML->sys.core[ithCore].BTB.BTB_config[5] / clockRate;
    interface_ip.obj_func_dyn_energy = 0;
    interface_ip.obj_func_dyn_power = 0;
    interface_ip.obj_func_leak_power = 0;
    interface_ip.obj_func_cycle_t = 1;
    interface_ip.num_rw_ports = 1;
    interface_ip.num_rd_ports = coredynp.predictionW;
    interface_ip.num_wr_ports = coredynp.predictionW;
    interface_ip.num_se_rd_ports = 0;
    BTB = new ArrayST(&interface_ip,
                      "Branch Target Buffer",
                      Core_device,
                      coredynp.opt_local,
                      coredynp.core_ty);
    BTB->area.set_area(BTB->area.get_area() + BTB->local_result.area);
    area.set_area(area.get_area() + BTB->local_result.area);
    /// cout<<"area="<<area<<endl;

    BPT = new BranchPredictor();
    BPT->set_params(XML, ithCore, &interface_ip, coredynp);
    BPT->computeArea();
    BPT->set_stats(XML);
    area.set_area(area.get_area() + BPT->area.get_area());
  }

  ID_inst = new inst_decoder(is_default,
                             &interface_ip,
                             coredynp.opcode_length,
                             1 /*Decoder should not know how many by itself*/,
                             coredynp.x86,
                             Core_device,
                             coredynp.core_ty);

  ID_operand = new inst_decoder(is_default,
                                &interface_ip,
                                coredynp.arch_ireg_width,
                                1,
                                coredynp.x86,
                                Core_device,
                                coredynp.core_ty);

  ID_misc = new inst_decoder(is_default,
                             &interface_ip,
                             8 /* Prefix field etc upto 14B*/,
                             1,
                             coredynp.x86,
                             Core_device,
                             coredynp.core_ty);
  // TODO: X86 decoder should decode the inst in cyclic mode under the control
  // of squencer. So the dynamic power should be multiplied by a few times.
  area.set_area(area.get_area() +
                (ID_inst->area.get_area() + ID_operand->area.get_area() +
                 ID_misc->area.get_area()) *
                    coredynp.decodeW);
}

void InstFetchU::computeEnergy(bool is_tdp) {
  if (!exist)
    return;
  if (is_tdp) {
    // init stats for Peak
    icache.caches->stats_t.readAc.access =
        icache.caches->l_ip.num_rw_ports * coredynp.IFU_duty_cycle;
    icache.caches->stats_t.readAc.miss = 0;
    icache.caches->stats_t.readAc.hit = icache.caches->stats_t.readAc.access -
                                        icache.caches->stats_t.readAc.miss;
    icache.caches->tdp_stats = icache.caches->stats_t;

    icache.missb->stats_t.readAc.access = icache.missb->stats_t.readAc.hit =
        icache.missb->l_ip.num_search_ports * coredynp.IFU_duty_cycle;
    icache.missb->stats_t.writeAc.access = icache.missb->stats_t.writeAc.hit =
        icache.missb->l_ip.num_search_ports * coredynp.IFU_duty_cycle;
    icache.missb->tdp_stats = icache.missb->stats_t;

    icache.ifb->stats_t.readAc.access = icache.ifb->stats_t.readAc.hit =
        icache.ifb->l_ip.num_search_ports * coredynp.IFU_duty_cycle;
    icache.ifb->stats_t.writeAc.access = icache.ifb->stats_t.writeAc.hit =
        icache.ifb->l_ip.num_search_ports * coredynp.IFU_duty_cycle;
    icache.ifb->tdp_stats = icache.ifb->stats_t;

    icache.prefetchb->stats_t.readAc.access =
        icache.prefetchb->stats_t.readAc.hit =
            icache.prefetchb->l_ip.num_search_ports * coredynp.IFU_duty_cycle;
    icache.prefetchb->stats_t.writeAc.access = icache.ifb->stats_t.writeAc.hit =
        icache.ifb->l_ip.num_search_ports * coredynp.IFU_duty_cycle;
    icache.prefetchb->tdp_stats = icache.prefetchb->stats_t;

    IB->stats_t.readAc.access = IB->stats_t.writeAc.access =
        XML->sys.core[ithCore].peak_issue_width;
    IB->tdp_stats = IB->stats_t;

    if (coredynp.predictionW > 0) {
      BTB->stats_t.readAc.access =
          coredynp.predictionW; // XML->sys.core[ithCore].BTB.read_accesses;
      BTB->stats_t.writeAc.access =
          0; // XML->sys.core[ithCore].BTB.write_accesses;
    }

    ID_inst->stats_t.readAc.access = coredynp.decodeW;
    ID_operand->stats_t.readAc.access = coredynp.decodeW;
    ID_misc->stats_t.readAc.access = coredynp.decodeW;
    ID_inst->tdp_stats = ID_inst->stats_t;
    ID_operand->tdp_stats = ID_operand->stats_t;
    ID_misc->tdp_stats = ID_misc->stats_t;

  } else {
    // init stats for Runtime Dynamic (RTP)
    icache.caches->stats_t.readAc.access =
        XML->sys.core[ithCore].icache.read_accesses;
    icache.caches->stats_t.readAc.miss =
        XML->sys.core[ithCore].icache.read_misses;
    icache.caches->stats_t.readAc.hit = icache.caches->stats_t.readAc.access -
                                        icache.caches->stats_t.readAc.miss;
    icache.caches->rtp_stats = icache.caches->stats_t;

    icache.missb->stats_t.readAc.access = icache.caches->stats_t.readAc.miss;
    icache.missb->stats_t.writeAc.access = icache.caches->stats_t.readAc.miss;
    icache.missb->rtp_stats = icache.missb->stats_t;

    icache.ifb->stats_t.readAc.access = icache.caches->stats_t.readAc.miss;
    icache.ifb->stats_t.writeAc.access = icache.caches->stats_t.readAc.miss;
    icache.ifb->rtp_stats = icache.ifb->stats_t;

    icache.prefetchb->stats_t.readAc.access =
        icache.caches->stats_t.readAc.miss;
    icache.prefetchb->stats_t.writeAc.access =
        icache.caches->stats_t.readAc.miss;
    icache.prefetchb->rtp_stats = icache.prefetchb->stats_t;

    IB->stats_t.readAc.access = IB->stats_t.writeAc.access =
        XML->sys.core[ithCore].total_instructions;
    IB->rtp_stats = IB->stats_t;

    if (coredynp.predictionW > 0) {
      BTB->stats_t.readAc.access =
          XML->sys.core[ithCore]
              .BTB.read_accesses; // XML->sys.core[ithCore].branch_instructions;
      BTB->stats_t.writeAc.access =
          XML->sys.core[ithCore]
              .BTB
              .write_accesses; // XML->sys.core[ithCore].branch_mispredictions;
      BTB->rtp_stats = BTB->stats_t;
    }

    ID_inst->stats_t.readAc.access = XML->sys.core[ithCore].total_instructions;
    ID_operand->stats_t.readAc.access =
        XML->sys.core[ithCore].total_instructions;
    ID_misc->stats_t.readAc.access = XML->sys.core[ithCore].total_instructions;
    ID_inst->rtp_stats = ID_inst->stats_t;
    ID_operand->rtp_stats = ID_operand->stats_t;
    ID_misc->rtp_stats = ID_misc->stats_t;
  }

  icache.power_t.reset();
  IB->power_t.reset();
  //	ID_inst->power_t.reset();
  //	ID_operand->power_t.reset();
  //	ID_misc->power_t.reset();
  if (coredynp.predictionW > 0) {
    BTB->power_t.reset();
  }

  icache.power_t.readOp.dynamic +=
      (icache.caches->stats_t.readAc.hit *
           icache.caches->local_result.power.readOp.dynamic +
       // icache.caches->stats_t.readAc.miss*icache.caches->local_result.tag_array2->power.readOp.dynamic+
       icache.caches->stats_t.readAc.miss *
           icache.caches->local_result.power.readOp
               .dynamic + // assume tag data accessed in parallel
       icache.caches->stats_t.readAc.miss *
           icache.caches->local_result.power.writeOp
               .dynamic); // read miss in Icache cause a write to Icache
  icache.power_t.readOp.dynamic +=
      icache.missb->stats_t.readAc.access *
          icache.missb->local_result.power.searchOp.dynamic +
      icache.missb->stats_t.writeAc.access *
          icache.missb->local_result.power.writeOp
              .dynamic; // each access to missb involves a CAM and a write
  icache.power_t.readOp.dynamic +=
      icache.ifb->stats_t.readAc.access *
          icache.ifb->local_result.power.searchOp.dynamic +
      icache.ifb->stats_t.writeAc.access *
          icache.ifb->local_result.power.writeOp.dynamic;
  icache.power_t.readOp.dynamic +=
      icache.prefetchb->stats_t.readAc.access *
          icache.prefetchb->local_result.power.searchOp.dynamic +
      icache.prefetchb->stats_t.writeAc.access *
          icache.prefetchb->local_result.power.writeOp.dynamic;

  IB->power_t.readOp.dynamic +=
      IB->local_result.power.readOp.dynamic * IB->stats_t.readAc.access +
      IB->stats_t.writeAc.access * IB->local_result.power.writeOp.dynamic;

  if (coredynp.predictionW > 0) {
    BTB->power_t.readOp.dynamic +=
        BTB->local_result.power.readOp.dynamic * BTB->stats_t.readAc.access +
        BTB->stats_t.writeAc.access * BTB->local_result.power.writeOp.dynamic;

    BPT->computeDynamicPower(is_tdp);
  }

  if (is_tdp) {
    //    	icache.power = icache.power_t +
    //    	        (icache.caches->local_result.power)*pppm_lkg +
    //    			(icache.missb->local_result.power +
    //    			icache.ifb->local_result.power +
    //    			icache.prefetchb->local_result.power)*pppm_Isub;
    icache.power = icache.power_t + (icache.caches->local_result.power +
                                     icache.missb->local_result.power +
                                     icache.ifb->local_result.power +
                                     icache.prefetchb->local_result.power) *
                                        pppm_lkg;

    IB->power = IB->power_t + IB->local_result.power * pppm_lkg;
    power = power + icache.power + IB->power;
    if (coredynp.predictionW > 0) {
      BTB->power = BTB->power_t + BTB->local_result.power * pppm_lkg;
      power = power + BTB->power + BPT->power;
    }

    ID_inst->power_t.readOp.dynamic = ID_inst->power.readOp.dynamic;
    ID_operand->power_t.readOp.dynamic = ID_operand->power.readOp.dynamic;
    ID_misc->power_t.readOp.dynamic = ID_misc->power.readOp.dynamic;

    ID_inst->power.readOp.dynamic *= ID_inst->tdp_stats.readAc.access;
    ID_operand->power.readOp.dynamic *= ID_operand->tdp_stats.readAc.access;
    ID_misc->power.readOp.dynamic *= ID_misc->tdp_stats.readAc.access;

    power = power + (ID_inst->power + ID_operand->power + ID_misc->power);
  } else {
    //    	icache.rt_power = icache.power_t +
    //    	        (icache.caches->local_result.power)*pppm_lkg +
    //    			(icache.missb->local_result.power +
    //    			icache.ifb->local_result.power +
    //    			icache.prefetchb->local_result.power)*pppm_Isub;

    icache.rt_power = icache.power_t + (icache.caches->local_result.power +
                                        icache.missb->local_result.power +
                                        icache.ifb->local_result.power +
                                        icache.prefetchb->local_result.power) *
                                           pppm_lkg;

    IB->rt_power = IB->power_t + IB->local_result.power * pppm_lkg;
    rt_power = rt_power + icache.rt_power + IB->rt_power;
    if (coredynp.predictionW > 0) {
      BTB->rt_power = BTB->power_t + BTB->local_result.power * pppm_lkg;
      rt_power = rt_power + BTB->rt_power + BPT->rt_power;
    }

    ID_inst->rt_power.readOp.dynamic =
        ID_inst->power_t.readOp.dynamic * ID_inst->rtp_stats.readAc.access;
    ID_operand->rt_power.readOp.dynamic = ID_operand->power_t.readOp.dynamic *
                                          ID_operand->rtp_stats.readAc.access;
    ID_misc->rt_power.readOp.dynamic =
        ID_misc->power_t.readOp.dynamic * ID_misc->rtp_stats.readAc.access;

    rt_power = rt_power +
               (ID_inst->rt_power + ID_operand->rt_power + ID_misc->rt_power);
  }
}

void InstFetchU::displayEnergy(uint32_t indent, int plevel, bool is_tdp) {
  if (!exist)
    return;
  string indent_str(indent, ' ');
  string indent_str_next(indent + 2, ' ');
  bool long_channel = XML->sys.longer_channel_device;
  bool power_gating = XML->sys.power_gating;

  if (is_tdp) {

    cout << indent_str << "Instruction Cache:" << endl;
    cout << indent_str_next << "Area = " << icache.area.get_area() * 1e-6
         << " mm^2" << endl;
    cout << indent_str_next
         << "Peak Dynamic = " << icache.power.readOp.dynamic * clockRate << " W"
         << endl;
    cout << indent_str_next << "Subthreshold Leakage = "
         << (long_channel ? icache.power.readOp.longer_channel_leakage
                          : icache.power.readOp.leakage)
         << " W" << endl;
    if (power_gating)
      cout << indent_str_next << "Subthreshold Leakage with power gating = "
           << (long_channel
                   ? icache.power.readOp.power_gated_with_long_channel_leakage
                   : icache.power.readOp.power_gated_leakage)
           << " W" << endl;
    cout << indent_str_next
         << "Gate Leakage = " << icache.power.readOp.gate_leakage << " W"
         << endl;
    cout << indent_str_next << "Runtime Dynamic = "
         << icache.rt_power.readOp.dynamic / executionTime << " W" << endl;
    cout << endl;
    if (coredynp.predictionW > 0) {
      cout << indent_str << "Branch Target Buffer:" << endl;
      cout << indent_str_next << "Area = " << BTB->area.get_area() * 1e-6
           << " mm^2" << endl;
      cout << indent_str_next
           << "Peak Dynamic = " << BTB->power.readOp.dynamic * clockRate << " W"
           << endl;
      cout << indent_str_next << "Subthreshold Leakage = "
           << (long_channel ? BTB->power.readOp.longer_channel_leakage
                            : BTB->power.readOp.leakage)
           << " W" << endl;
      if (power_gating)
        cout << indent_str_next << "Subthreshold Leakage with power gating = "
             << (long_channel
                     ? BTB->power.readOp.power_gated_with_long_channel_leakage
                     : BTB->power.readOp.power_gated_leakage)
             << " W" << endl;
      cout << indent_str_next
           << "Gate Leakage = " << BTB->power.readOp.gate_leakage << " W"
           << endl;
      cout << indent_str_next << "Runtime Dynamic = "
           << BTB->rt_power.readOp.dynamic / executionTime << " W" << endl;
      cout << endl;
      if (BPT->exist) {
        cout << indent_str << "Branch Predictor:" << endl;
        cout << indent_str_next << "Area = " << BPT->area.get_area() * 1e-6
             << " mm^2" << endl;
        cout << indent_str_next
             << "Peak Dynamic = " << BPT->power.readOp.dynamic * clockRate
             << " W" << endl;
        cout << indent_str_next << "Subthreshold Leakage = "
             << (long_channel ? BPT->power.readOp.longer_channel_leakage
                              : BPT->power.readOp.leakage)
             << " W" << endl;
        if (power_gating)
          cout << indent_str_next << "Subthreshold Leakage with power gating = "
               << (long_channel
                       ? BPT->power.readOp.power_gated_with_long_channel_leakage
                       : BPT->power.readOp.power_gated_leakage)
               << " W" << endl;
        cout << indent_str_next
             << "Gate Leakage = " << BPT->power.readOp.gate_leakage << " W"
             << endl;
        cout << indent_str_next << "Runtime Dynamic = "
             << BPT->rt_power.readOp.dynamic / executionTime << " W" << endl;
        cout << endl;
        if (plevel > 3) {
          BPT->displayEnergy(indent + 4, plevel, is_tdp);
        }
      }
    }
    cout << indent_str << "Instruction Buffer:" << endl;
    cout << indent_str_next << "Area = " << IB->area.get_area() * 1e-6
         << " mm^2" << endl;
    cout << indent_str_next
         << "Peak Dynamic = " << IB->power.readOp.dynamic * clockRate << " W"
         << endl;
    cout << indent_str_next << "Subthreshold Leakage = "
         << (long_channel ? IB->power.readOp.longer_channel_leakage
                          : IB->power.readOp.leakage)
         << " W" << endl;
    if (power_gating)
      cout << indent_str_next << "Subthreshold Leakage with power gating = "
           << (long_channel
                   ? IB->power.readOp.power_gated_with_long_channel_leakage
                   : IB->power.readOp.power_gated_leakage)
           << " W" << endl;
    cout << indent_str_next
         << "Gate Leakage = " << IB->power.readOp.gate_leakage << " W" << endl;
    cout << indent_str_next
         << "Runtime Dynamic = " << IB->rt_power.readOp.dynamic / executionTime
         << " W" << endl;
    cout << endl;
    cout << indent_str << "Instruction Decoder:" << endl;
    cout << indent_str_next << "Area = "
         << (ID_inst->area.get_area() + ID_operand->area.get_area() +
             ID_misc->area.get_area()) *
                coredynp.decodeW * 1e-6
         << " mm^2" << endl;
    cout << indent_str_next << "Peak Dynamic = "
         << (ID_inst->power.readOp.dynamic + ID_operand->power.readOp.dynamic +
             ID_misc->power.readOp.dynamic) *
                clockRate
         << " W" << endl;
    cout << indent_str_next << "Subthreshold Leakage = "
         << (long_channel ? (ID_inst->power.readOp.longer_channel_leakage +
                             ID_operand->power.readOp.longer_channel_leakage +
                             ID_misc->power.readOp.longer_channel_leakage)
                          : (ID_inst->power.readOp.leakage +
                             ID_operand->power.readOp.leakage +
                             ID_misc->power.readOp.leakage))
         << " W" << endl;

    double tot_leakage =
        (ID_inst->power.readOp.leakage + ID_operand->power.readOp.leakage +
         ID_misc->power.readOp.leakage);
    double tot_leakage_longchannel =
        (ID_inst->power.readOp.longer_channel_leakage +
         ID_operand->power.readOp.longer_channel_leakage +
         ID_misc->power.readOp.longer_channel_leakage);
    double tot_leakage_pg = (ID_inst->power.readOp.power_gated_leakage +
                             ID_operand->power.readOp.power_gated_leakage +
                             ID_misc->power.readOp.power_gated_leakage);
    double tot_leakage_pg_with_long_channel =
        (ID_inst->power.readOp.power_gated_with_long_channel_leakage +
         ID_operand->power.readOp.power_gated_with_long_channel_leakage +
         ID_misc->power.readOp.power_gated_with_long_channel_leakage);

    if (power_gating)
      cout << indent_str_next << "Subthreshold Leakage with power gating = "
           << (long_channel ? tot_leakage_pg_with_long_channel : tot_leakage_pg)
           << " W" << endl;
    cout << indent_str_next << "Gate Leakage = "
         << (ID_inst->power.readOp.gate_leakage +
             ID_operand->power.readOp.gate_leakage +
             ID_misc->power.readOp.gate_leakage)
         << " W" << endl;
    cout << indent_str_next << "Runtime Dynamic = "
         << (ID_inst->rt_power.readOp.dynamic +
             ID_operand->rt_power.readOp.dynamic +
             ID_misc->rt_power.readOp.dynamic) /
                executionTime
         << " W" << endl;
    cout << endl;
  } else {
    //		cout << indent_str_next << "Instruction Cache    Peak Dynamic = "
    //<< icache.rt_power.readOp.dynamic*clockRate << " W" << endl;
    // cout << indent_str_next << "Instruction Cache    Subthreshold Leakage = "
    // << icache.rt_power.readOp.leakage <<" W" << endl; 		cout <<
    // indent_str_next << "Instruction Cache    Gate Leakage = " <<
    // icache.rt_power.readOp.gate_leakage << " W" << endl; 		cout <<
    // indent_str_next << "Instruction Buffer   Peak Dynamic = " <<
    // IB->rt_power.readOp.dynamic*clockRate  << " W" << endl; 		cout <<
    // indent_str_next << "Instruction Buffer   Subthreshold Leakage = " <<
    // IB->rt_power.readOp.leakage  << " W" << endl; 		cout << indent_str_next
    // << "Instruction Buffer   Gate Leakage = " <<
    // IB->rt_power.readOp.gate_leakage
    //<< " W" << endl; 		cout << indent_str_next << "Branch Target Buffer
    // Peak Dynamic = " << BTB->rt_power.readOp.dynamic*clockRate  << " W" <<
    // endl; 		cout << indent_str_next << "Branch Target Buffer   Subthreshold
    // Leakage = " << BTB->rt_power.readOp.leakage  << " W" << endl; 		cout
    // << indent_str_next << "Branch Target Buffer   Gate Leakage = " <<
    // BTB->rt_power.readOp.gate_leakage  << " W" << endl; 		cout <<
    // indent_str_next << "Branch Predictor   Peak Dynamic = " <<
    // BPT->rt_power.readOp.dynamic*clockRate  << " W" << endl; 		cout
    // << indent_str_next << "Branch Predictor   Subthreshold Leakage = " <<
    // BPT->rt_power.readOp.leakage  << " W" << endl; 		cout <<
    // indent_str_next
    // << "Branch Predictor   Gate Leakage = " <<
    // BPT->rt_power.readOp.gate_leakage
    //<< " W" << endl;
  }
}

InstFetchU ::~InstFetchU() {

  if (!exist)
    return;
  if (IB) {
    delete IB;
    IB = 0;
  }
  if (ID_inst) {
    delete ID_inst;
    ID_inst = 0;
  }
  if (ID_operand) {
    delete ID_operand;
    ID_operand = 0;
  }
  if (ID_misc) {
    delete ID_misc;
    ID_misc = 0;
  }
  if (coredynp.predictionW > 0) {
    if (BTB) {
      delete BTB;
      BTB = 0;
    }
    if (BPT) {
      delete BPT;
      BPT = 0;
    }
  }
}