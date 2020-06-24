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

#include "renaming_unit.h"

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

RENAMINGU::RENAMINGU() {
  init_stats = false;
  init_params = false;
  set_area = false;
  long_channel = false;
  power_gating = false;
  fp_rename_writes = 0;
  fp_rename_reads = 0;
  rename_writes = 0;
  rename_reads = 0;
  int_instructions = 0;
  fp_instructions = 0;
}

/*
 * Although renaming logic maybe be used in in-order processors, McPAT
 * assumes no renaming logic is used since the performance gain is very
 * limited and the only major inorder processor with renaming logic is
 * Itainium that is a VLIW processor and different from current McPAT's
 * model. physical register base OOO must have Dual-RAT architecture or
 * equivalent structure.FRAT:FrontRAT, RRAT:RetireRAT; i,f prefix mean int
 * and fp RAT for all Renaming logic, random accessible checkpointing is
 * used, but only update when instruction retires. FRAT will be read twice
 * and written once per instruction; RRAT will be write once per instruction
 * when committing and reads out all when context switch
 *
 * RAM scheme has # ARchi Reg entry with each entry hold phy reg tag,
 * CAM scheme has # Phy Reg entry with each entry hold ARchi reg tag,
 *
 * RAM-based RAT is duplicated/partitioned for each different hardware
 * threads CAM-based RAT is shared for all hardware threads With SMT, RAT is
 * partitioned and tagged. RAM-based RAT needs to have N (N-way SMT) sets of
 * entries, with each set for a thread. The RAT control logic will determine
 * different sets to use for different threads. But it does not need extra
 * tag bits in the entries. However, CAM-based RAT need extra tag bits to
 * distinguish the architecture register ids for different threads.
 *
 * checkpointing of RAT and RRAT are both for architecture state recovery
 * with events including mis-speculation; Checkpointing is easier to
 * implement in CAM than in RAM based RAT, despite of the inferior scalabilty
 * of the CAM-based RATs.  McPAT assumes at least 1 checkpoint for CAM-based
 * RATs, and no more than 4 checkpoints (based on MIPS designs) for RAM based
 * RATs, thus CAM-based RAT does not need RRAT Although no Dual-RAT is needed
 * in RS-based OOO processors, since archi RegFile contains the committed
 * register values, a RRAT or GC (not both) will speedup the mis-speculation
 * recovery. Thus, when RAM-RAT does not have any GC, McPAT assumes the
 * existence of a RRAT.
 *
 * RAM-base RAT does not need to scan/search all contents during instruction
 * commit, since the ROB for RAM-based RAT contains the ARF-PRF mapping that
 * is used for index the RAT entry to be updated.
 *
 * Both RAM and CAM have same DCL
 */
void RENAMINGU::set_params(const ParseXML *XML,
                           int ithCore_,
                           InputParameter *interface_ip_,
                           const CoreDynParam &dyn_p_,
                           bool exist_) {
  int tag = 0;
  int data = 0;
  int out_w = 0;
  exist = exist_;
  if (!exist_) {
    return;
  }
  ithCore = ithCore_;
  interface_ip = *interface_ip_;
  coredynp = dyn_p_;
  long_channel = XML->sys.longer_channel_device;
  power_gating = XML->sys.power_gating;
  // interface_ip.wire_is_mat_type = 0;
  // interface_ip.wire_os_mat_type = 0;
  // interface_ip.wt               = Global_30;
  clockRate = coredynp.clockRate;
  executionTime = coredynp.executionTime;
  if (coredynp.core_ty == OOO) {
    // integer pipeline
    if (coredynp.scheu_ty == PhysicalRegFile) {
      if (coredynp.rm_ty ==
          RAMbased) { // FRAT with global checkpointing (GCs) please see paper
                      // tech report for detailed explanation.
        data = int(ceil(coredynp.phy_ireg_width *
                        (1 + coredynp.globalCheckpoint) / 8.0)); // 33;
        out_w = int(ceil(coredynp.phy_ireg_width / 8.0));        // bytes
        interface_ip.is_cache = false;
        interface_ip.pure_cam = false;
        interface_ip.pure_ram = true;
        interface_ip.line_sz = data;
        interface_ip.cache_sz = data *
                                XML->sys.core[ithCore].archi_Regs_IRF_size *
                                XML->sys.core[ithCore].number_hardware_threads;
        interface_ip.assoc = 1;
        interface_ip.nbanks = 1;
        interface_ip.out_w = out_w * 8;
        interface_ip.access_mode = 2;
        interface_ip.throughput = 1.0 / clockRate;
        interface_ip.latency = 1.0 / clockRate;
        interface_ip.obj_func_dyn_energy = 0;
        interface_ip.obj_func_dyn_power = 0;
        interface_ip.obj_func_leak_power = 0;
        interface_ip.obj_func_cycle_t = 1;
        interface_ip.num_rw_ports = 1; // the extra one port is for GCs
        interface_ip.num_rd_ports = 2 * coredynp.decodeW;
        interface_ip.num_wr_ports = coredynp.decodeW;
        interface_ip.num_se_rd_ports = 0;
        iFRAT.set_params(&interface_ip,
                         "Int FrontRAT",
                         Core_device,
                         coredynp.opt_local,
                         coredynp.core_ty);
        // FRAT floating point
        data = int(ceil(coredynp.phy_freg_width *
                        (1 + coredynp.globalCheckpoint) / 8.0));
        out_w = int(ceil(coredynp.phy_freg_width / 8.0));
        interface_ip.is_cache = false;
        interface_ip.pure_cam = false;
        interface_ip.pure_ram = true;
        interface_ip.line_sz = data;
        interface_ip.cache_sz = data *
                                XML->sys.core[ithCore].archi_Regs_FRF_size *
                                XML->sys.core[ithCore].number_hardware_threads;
        interface_ip.assoc = 1;
        interface_ip.nbanks = 1;
        interface_ip.out_w = out_w * 8;
        interface_ip.access_mode = 2;
        interface_ip.throughput = 1.0 / clockRate;
        interface_ip.latency = 1.0 / clockRate;
        interface_ip.obj_func_dyn_energy = 0;
        interface_ip.obj_func_dyn_power = 0;
        interface_ip.obj_func_leak_power = 0;
        interface_ip.obj_func_cycle_t = 1;
        interface_ip.num_rw_ports = 1; // the extra one port is for GCs
        interface_ip.num_rd_ports = 2 * coredynp.fp_decodeW;
        interface_ip.num_wr_ports = coredynp.fp_decodeW;
        interface_ip.num_se_rd_ports = 0;
        fFRAT.set_params(&interface_ip,
                         "FP FrontRAT",
                         Core_device,
                         coredynp.opt_local,
                         coredynp.core_ty);
      } else if (coredynp.rm_ty == CAMbased) {
        // FRAT
        tag = coredynp.arch_ireg_width + coredynp.hthread_width;
        data = int(
            ceil((coredynp.arch_ireg_width + 1 * coredynp.globalCheckpoint) /
                 8.0)); // each checkpoint in the CAM-based RAT design needs
                        // only 1 bit, see "a power-aware hybrid ram-cam
                        // renaming mechanism for fast recovery"
        out_w = int(ceil(coredynp.arch_ireg_width / 8.0));
        interface_ip.is_cache = true;
        interface_ip.pure_cam = false;
        interface_ip.pure_ram = false;
        interface_ip.line_sz = data;
        interface_ip.cache_sz = data * XML->sys.core[ithCore].phy_Regs_IRF_size;
        interface_ip.assoc = 0;
        interface_ip.nbanks = 1;
        interface_ip.out_w = out_w * 8;
        interface_ip.specific_tag = 1;
        interface_ip.tag_w = tag;
        interface_ip.access_mode = 2;
        interface_ip.throughput = 1.0 / clockRate;
        interface_ip.latency = 1.0 / clockRate;
        interface_ip.obj_func_dyn_energy = 0;
        interface_ip.obj_func_dyn_power = 0;
        interface_ip.obj_func_leak_power = 0;
        interface_ip.obj_func_cycle_t = 1;
        interface_ip.num_rw_ports = 1; // for GCs
        interface_ip.num_rd_ports = coredynp.decodeW;
        interface_ip.num_wr_ports = coredynp.decodeW;
        interface_ip.num_se_rd_ports = 0;
        interface_ip.num_search_ports = 2 * coredynp.decodeW;
        iFRAT.set_params(&interface_ip,
                         "Int FrontRAT",
                         Core_device,
                         coredynp.opt_local,
                         coredynp.core_ty);
        // FRAT for FP
        tag = coredynp.arch_freg_width + coredynp.hthread_width;
        data = int(
            ceil((coredynp.arch_freg_width + 1 * coredynp.globalCheckpoint) /
                 8.0)); // each checkpoint in the CAM-based RAT design needs
                        // only 1 bit, see "a power-aware hybrid ram-cam
                        // renaming mechanism for fast recovery"
        out_w = int(ceil(coredynp.arch_freg_width / 8.0));
        interface_ip.is_cache = true;
        interface_ip.pure_cam = false;
        interface_ip.pure_ram = false;
        interface_ip.line_sz = data;
        interface_ip.cache_sz = data * XML->sys.core[ithCore].phy_Regs_FRF_size;
        interface_ip.assoc = 0;
        interface_ip.nbanks = 1;
        interface_ip.out_w = out_w * 8;
        interface_ip.specific_tag = 1;
        interface_ip.tag_w = tag;
        interface_ip.access_mode = 2;
        interface_ip.throughput = 1.0 / clockRate;
        interface_ip.latency = 1.0 / clockRate;
        interface_ip.obj_func_dyn_energy = 0;
        interface_ip.obj_func_dyn_power = 0;
        interface_ip.obj_func_leak_power = 0;
        interface_ip.obj_func_cycle_t = 1;
        interface_ip.num_rw_ports = 1; // for GCs
        interface_ip.num_rd_ports = coredynp.fp_decodeW;
        interface_ip.num_wr_ports = coredynp.fp_decodeW;
        interface_ip.num_se_rd_ports = 0;
        interface_ip.num_search_ports = 2 * coredynp.fp_decodeW;
        fFRAT.set_params(&interface_ip,
                         "FP FrontRAT",
                         Core_device,
                         coredynp.opt_local,
                         coredynp.core_ty);
      }

      // RRAT is always RAM based, does not have GCs, and is used only for
      // record latest non-speculative mapping RRAT is not needed for CAM-based
      // RAT (McPAT assumes CAM-based RAT to have at least 1 checkpoint), it is
      // not needed for RAM-based RAT with checkpoints McPAT assumes renaming
      // unit to have RRAT when there is no checkpoints in FRAT, while MIPS
      // R1000 has 4 GCs, according to Intel Netburst Archi, combine GC with
      // FRAT is very costly, especially for high issue width and high clock
      // rate.

      if ((coredynp.rm_ty == RAMbased) && (coredynp.globalCheckpoint < 1)) {
        data = int(ceil(coredynp.phy_ireg_width / 8.0));
        interface_ip.is_cache = false;
        interface_ip.pure_cam = false;
        interface_ip.pure_ram = true;
        interface_ip.line_sz = data;
        interface_ip.cache_sz =
            data * XML->sys.core[ithCore].archi_Regs_IRF_size * 2 *
            XML->sys.core[ithCore]
                .number_hardware_threads; // HACK--2 to make it as least 64B
        interface_ip.assoc = 1;
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
        interface_ip.num_rd_ports = XML->sys.core[ithCore].commit_width;
        interface_ip.num_wr_ports = XML->sys.core[ithCore].commit_width;
        interface_ip.num_se_rd_ports = 0;
        iRRAT.set_params(&interface_ip,
                         "Int RetireRAT",
                         Core_device,
                         coredynp.opt_local,
                         coredynp.core_ty);
        // RRAT for FP
        data = int(ceil(coredynp.phy_freg_width / 8.0));
        interface_ip.is_cache = false;
        interface_ip.pure_cam = false;
        interface_ip.pure_ram = true;
        interface_ip.line_sz = data;
        interface_ip.cache_sz =
            data * XML->sys.core[ithCore].archi_Regs_FRF_size * 2 *
            XML->sys.core[ithCore]
                .number_hardware_threads; // HACK--2 to make it as least 64B
        interface_ip.assoc = 1;
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
        interface_ip.num_rd_ports = coredynp.fp_decodeW;
        interface_ip.num_wr_ports = coredynp.fp_decodeW;
        interface_ip.num_se_rd_ports = 0;
        fRRAT.set_params(&interface_ip,
                         "FP RetireRAT",
                         Core_device,
                         coredynp.opt_local,
                         coredynp.core_ty);
      }
      // Freelist of renaming unit always RAM based and needed for RAM-based
      // RATs. Although it can be implemented within the CAM-based RAT, Current
      // McPAT does not have the free bits in the CAM but use the same external
      // free list as a close approximation for CAM RAT. Recycle happens at two
      // places: 1)when DCL check there are WAW, the Phy-registers/ROB directly
      // recycles into freelist
      // 2)When instruction commits the Phyregisters/ROB needed to be recycled.
      // therefore num_wr port = decode-1(-1 means at least one phy reg will be
      // used for the current renaming group) + commit width
      data = int(ceil(coredynp.phy_ireg_width / 8.0));
      interface_ip.is_cache = false;
      interface_ip.pure_cam = false;
      interface_ip.pure_ram = true;
      interface_ip.line_sz = data;
      interface_ip.cache_sz = data * coredynp.num_ifreelist_entries;
      interface_ip.assoc = 1;
      interface_ip.nbanks = 1;
      interface_ip.out_w = interface_ip.line_sz * 8;
      interface_ip.access_mode = 1;
      interface_ip.throughput = 1.0 / clockRate;
      interface_ip.latency = 1.0 / clockRate;
      interface_ip.obj_func_dyn_energy = 0;
      interface_ip.obj_func_dyn_power = 0;
      interface_ip.obj_func_leak_power = 0;
      interface_ip.obj_func_cycle_t = 1;
      interface_ip.num_rw_ports = 1; // TODO
      interface_ip.num_rd_ports = coredynp.decodeW;
      interface_ip.num_wr_ports =
          coredynp.decodeW - 1 + XML->sys.core[ithCore].commit_width;
      // every cycle, (coredynp.decodeW -1) inst may need to send back it dest
      // tags, committW insts needs to update freelist buffers
      interface_ip.num_se_rd_ports = 0;
      ifreeL.set_params(&interface_ip,
                        "Int Free List",
                        Core_device,
                        coredynp.opt_local,
                        coredynp.core_ty);

      // freelist for FP
      data = int(ceil(coredynp.phy_freg_width / 8.0));
      interface_ip.is_cache = false;
      interface_ip.pure_cam = false;
      interface_ip.pure_ram = true;
      interface_ip.line_sz = data;
      interface_ip.cache_sz = data * coredynp.num_ffreelist_entries;
      interface_ip.assoc = 1;
      interface_ip.nbanks = 1;
      interface_ip.out_w = interface_ip.line_sz * 8;
      interface_ip.access_mode = 1;
      interface_ip.throughput = 1.0 / clockRate;
      interface_ip.latency = 1.0 / clockRate;
      interface_ip.obj_func_dyn_energy = 0;
      interface_ip.obj_func_dyn_power = 0;
      interface_ip.obj_func_leak_power = 0;
      interface_ip.obj_func_cycle_t = 1;
      interface_ip.num_rw_ports = 1;
      interface_ip.num_rd_ports = coredynp.fp_decodeW;
      interface_ip.num_wr_ports =
          coredynp.fp_decodeW - 1 + XML->sys.core[ithCore].commit_width;
      interface_ip.num_se_rd_ports = 0;
      ffreeL.set_params(&interface_ip,
                        "FP Free List",
                        Core_device,
                        coredynp.opt_local,
                        coredynp.core_ty);

      idcl.set_params(
          &interface_ip,
          coredynp,
          coredynp.phy_ireg_width); // TODO:Separate 2 sections See TR
      fdcl.set_params(&interface_ip, coredynp, coredynp.phy_freg_width);

    } else if (coredynp.scheu_ty == ReservationStation) {
      if (coredynp.rm_ty == RAMbased) {

        data = int(ceil(coredynp.phy_ireg_width *
                        (1 + coredynp.globalCheckpoint) / 8.0));
        out_w = int(ceil(coredynp.phy_ireg_width /
                         8.0)); // GC does not need to be readout
        interface_ip.is_cache = false;
        interface_ip.pure_cam = false;
        interface_ip.pure_ram = true;
        interface_ip.line_sz = data;
        interface_ip.cache_sz = data *
                                XML->sys.core[ithCore].archi_Regs_IRF_size *
                                XML->sys.core[ithCore].number_hardware_threads;
        interface_ip.assoc = 1;
        interface_ip.nbanks = 1;
        interface_ip.out_w = out_w * 8;
        interface_ip.access_mode = 2;
        interface_ip.throughput = 1.0 / clockRate;
        interface_ip.latency = 1.0 / clockRate;
        interface_ip.obj_func_dyn_energy = 0;
        interface_ip.obj_func_dyn_power = 0;
        interface_ip.obj_func_leak_power = 0;
        interface_ip.obj_func_cycle_t = 1;
        interface_ip.num_rw_ports = 1; // the extra one port is for GCs
        interface_ip.num_rd_ports = 2 * coredynp.decodeW;
        interface_ip.num_wr_ports = coredynp.decodeW;
        interface_ip.num_se_rd_ports = 0;
        iFRAT.set_params(&interface_ip,
                         "Int FrontRAT",
                         Core_device,
                         coredynp.opt_local,
                         coredynp.core_ty);
        //   iFRAT.local_result.power.readOp.dynamic *=
        // 1+0.2*0.05;//1+mis-speculation% TODO
        //   iFRAT.local_result.power.writeOp.dynamic
        //*=1+0.2*0.05;//compensate for GC
        // FP
        data = int(ceil(coredynp.phy_freg_width *
                        (1 + coredynp.globalCheckpoint) / 8.0));
        out_w = int(ceil(coredynp.phy_freg_width / 8.0));
        interface_ip.is_cache = false;
        interface_ip.pure_cam = false;
        interface_ip.pure_ram = true;
        interface_ip.line_sz = data;
        interface_ip.cache_sz = data *
                                XML->sys.core[ithCore].archi_Regs_FRF_size *
                                XML->sys.core[ithCore].number_hardware_threads;
        interface_ip.assoc = 1;
        interface_ip.nbanks = 1;
        interface_ip.out_w = out_w * 8;
        interface_ip.access_mode = 2;
        interface_ip.throughput = 1.0 / clockRate;
        interface_ip.latency = 1.0 / clockRate;
        interface_ip.obj_func_dyn_energy = 0;
        interface_ip.obj_func_dyn_power = 0;
        interface_ip.obj_func_leak_power = 0;
        interface_ip.obj_func_cycle_t = 1;
        interface_ip.num_rw_ports = 1; // the extra one port is for GCs
        interface_ip.num_rd_ports = 2 * coredynp.fp_decodeW;
        interface_ip.num_wr_ports = coredynp.fp_decodeW;
        interface_ip.num_se_rd_ports = 0;
        fFRAT.set_params(&interface_ip,
                         "FP FrontRAT",
                         Core_device,
                         coredynp.opt_local,
                         coredynp.core_ty);
        //   fFRAT.local_result.power.readOp.dynamic *=
        // 1+0.2*0.05;//1+mis-speculation% TODO
        //   fFRAT.local_result.power.writeOp.dynamic
        //*=1+0.2*0.05;//compensate for GC

      } else if (coredynp.rm_ty == CAMbased) {
        // FRAT
        tag = coredynp.arch_ireg_width + coredynp.hthread_width;
        data = int(ceil(
            (coredynp.arch_ireg_width + 1 * coredynp.globalCheckpoint) / 8.0));
        out_w = int(ceil(coredynp.arch_ireg_width /
                         8.0)); // GC bits does not need to be sent out
        interface_ip.is_cache = true;
        interface_ip.pure_cam = false;
        interface_ip.pure_ram = false;
        interface_ip.line_sz = data;
        interface_ip.cache_sz = data * XML->sys.core[ithCore].phy_Regs_IRF_size;
        interface_ip.assoc = 0;
        interface_ip.nbanks = 1;
        interface_ip.out_w = out_w * 8;
        interface_ip.specific_tag = 1;
        interface_ip.tag_w = tag;
        interface_ip.access_mode = 2;
        interface_ip.throughput = 1.0 / clockRate;
        interface_ip.latency = 1.0 / clockRate;
        interface_ip.obj_func_dyn_energy = 0;
        interface_ip.obj_func_dyn_power = 0;
        interface_ip.obj_func_leak_power = 0;
        interface_ip.obj_func_cycle_t = 1;
        interface_ip.num_rw_ports = 1; // for GCs
        interface_ip.num_rd_ports = coredynp.decodeW;
        interface_ip.num_wr_ports = coredynp.decodeW;
        interface_ip.num_se_rd_ports = 0;
        interface_ip.num_search_ports = 2 * coredynp.decodeW;
        iFRAT.set_params(&interface_ip,
                         "Int FrontRAT",
                         Core_device,
                         coredynp.opt_local,
                         coredynp.core_ty);

        // FRAT
        tag = coredynp.arch_freg_width + coredynp.hthread_width;
        data = int(
            ceil((coredynp.arch_freg_width + 1 * coredynp.globalCheckpoint) /
                 8.0)); // the address of CAM needed to be sent out
        out_w = int(ceil(coredynp.arch_freg_width / 8.0));
        interface_ip.is_cache = true;
        interface_ip.pure_cam = false;
        interface_ip.pure_ram = false;
        interface_ip.line_sz = data;
        interface_ip.cache_sz = data * XML->sys.core[ithCore].phy_Regs_FRF_size;
        interface_ip.assoc = 0;
        interface_ip.nbanks = 1;
        interface_ip.out_w = out_w * 8;
        interface_ip.specific_tag = 1;
        interface_ip.tag_w = tag;
        interface_ip.access_mode = 2;
        interface_ip.throughput = 1.0 / clockRate;
        interface_ip.latency = 1.0 / clockRate;
        interface_ip.obj_func_dyn_energy = 0;
        interface_ip.obj_func_dyn_power = 0;
        interface_ip.obj_func_leak_power = 0;
        interface_ip.obj_func_cycle_t = 1;
        interface_ip.num_rw_ports = 1; // for GCs
        interface_ip.num_rd_ports =
            XML->sys.core[ithCore].decode_width; // 0;TODO;
        interface_ip.num_wr_ports = coredynp.fp_decodeW;
        interface_ip.num_se_rd_ports = 0;
        interface_ip.num_search_ports = 2 * coredynp.fp_decodeW;
        fFRAT.set_params(&interface_ip,
                         "FP FrontRAT",
                         Core_device,
                         coredynp.opt_local,
                         coredynp.core_ty);
      }
      // Although no RRAT for RS based OOO is really needed since the archiRF
      // always holds the non-speculative data, having the RRAT or GC (not both)
      // can help the recovery of mis-speculations.

      if ((coredynp.rm_ty == RAMbased) && (coredynp.globalCheckpoint < 1)) {
        data = int(ceil(coredynp.phy_ireg_width / 8.0));
        interface_ip.is_cache = false;
        interface_ip.pure_cam = false;
        interface_ip.pure_ram = true;
        interface_ip.line_sz = data;
        interface_ip.cache_sz =
            data * XML->sys.core[ithCore].archi_Regs_IRF_size * 2 *
            XML->sys.core[ithCore]
                .number_hardware_threads; // HACK--2 to make it as least 64B
        interface_ip.assoc = 1;
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
        interface_ip.num_rd_ports = XML->sys.core[ithCore].commit_width;
        interface_ip.num_wr_ports = XML->sys.core[ithCore].commit_width;
        interface_ip.num_se_rd_ports = 0;
        iRRAT.set_params(&interface_ip,
                         "Int RetireRAT",
                         Core_device,
                         coredynp.opt_local,
                         coredynp.core_ty);
        // RRAT for FP
        data = int(ceil(coredynp.phy_freg_width / 8.0));
        interface_ip.is_cache = false;
        interface_ip.pure_cam = false;
        interface_ip.pure_ram = true;
        interface_ip.line_sz = data;
        interface_ip.cache_sz =
            data * XML->sys.core[ithCore].archi_Regs_FRF_size * 2 *
            XML->sys.core[ithCore]
                .number_hardware_threads; // HACK--2 to make it as least 64B
        interface_ip.assoc = 1;
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
        interface_ip.num_rd_ports = coredynp.fp_decodeW;
        interface_ip.num_wr_ports = coredynp.fp_decodeW;
        interface_ip.num_se_rd_ports = 0;
        fRRAT.set_params(&interface_ip,
                         "FP RetireRAT",
                         Core_device,
                         coredynp.opt_local,
                         coredynp.core_ty);
      }

      // Freelist of renaming unit of RS based OOO is unifed for both int and fp
      // renaming unit since the ROB is unified
      data = int(ceil(coredynp.phy_ireg_width / 8.0));
      interface_ip.is_cache = false;
      interface_ip.pure_cam = false;
      interface_ip.pure_ram = true;
      interface_ip.line_sz = data;
      interface_ip.cache_sz = data * coredynp.num_ifreelist_entries;
      interface_ip.assoc = 1;
      interface_ip.nbanks = 1;
      interface_ip.out_w = interface_ip.line_sz * 8;
      interface_ip.access_mode = 1;
      interface_ip.throughput = 1.0 / clockRate;
      interface_ip.latency = 1.0 / clockRate;
      interface_ip.obj_func_dyn_energy = 0;
      interface_ip.obj_func_dyn_power = 0;
      interface_ip.obj_func_leak_power = 0;
      interface_ip.obj_func_cycle_t = 1;
      interface_ip.num_rw_ports = 1; // TODO
      interface_ip.num_rd_ports = coredynp.decodeW;
      interface_ip.num_wr_ports =
          coredynp.decodeW - 1 + XML->sys.core[ithCore].commit_width;
      interface_ip.num_se_rd_ports = 0;
      ifreeL.set_params(&interface_ip,
                        "Unified Free List",
                        Core_device,
                        coredynp.opt_local,
                        coredynp.core_ty);
      // ifreeL.area.set_area(ifreeL.area.get_area()+
      // ifreeL.local_result.area*XML->sys.core[ithCore].number_hardware_threads);

      idcl.set_params(
          &interface_ip,
          coredynp,
          coredynp.phy_ireg_width); // TODO:Separate 2 sections See TR
      fdcl.set_params(&interface_ip, coredynp, coredynp.phy_freg_width);
    }
  }
  if (coredynp.core_ty == Inorder && coredynp.issueW > 1) {
    /* Dependency check logic will only present when decode(issue) width>1.
     *  Multiple issue in order processor can do without renaming, but dcl is a
     * must.
     */
    idcl.set_params(&interface_ip,
                    coredynp,
                    coredynp.phy_ireg_width); // TODO:Separate 2 sections See TR
    fdcl.set_params(&interface_ip, coredynp, coredynp.phy_freg_width);
  }
  init_params = true;
}

void RENAMINGU::set_stats(const ParseXML *XML) {
  fp_rename_writes = XML->sys.core[ithCore].fp_rename_writes;
  fp_rename_reads = XML->sys.core[ithCore].fp_rename_reads;
  rename_writes = XML->sys.core[ithCore].rename_writes;
  rename_reads = XML->sys.core[ithCore].rename_reads;
  int_instructions = XML->sys.core[ithCore].int_instructions;
  fp_instructions = XML->sys.core[ithCore].fp_instructions;
  init_stats = true;
}

void RENAMINGU::computeArea() {
  if (!init_params) {
    std::cerr << "[ RENAMINGU ] Error: must set params before calling "
                 "computeArea()\n";
    exit(1);
  }
  set_area = true;
  if (coredynp.core_ty == OOO) {
    if (coredynp.scheu_ty == PhysicalRegFile) {
      if (coredynp.rm_ty ==
          RAMbased) { // FRAT with global checkpointing (GCs) please see paper
                      // tech report for detailed explanation.
        iFRAT.computeArea();
        iFRAT.area.set_area(iFRAT.area.get_area() + iFRAT.local_result.area);
        area.set_area(area.get_area() + iFRAT.area.get_area());

        fFRAT.computeArea();
        fFRAT.area.set_area(fFRAT.area.get_area() + fFRAT.local_result.area);
        area.set_area(area.get_area() + fFRAT.area.get_area());

      } else if (coredynp.rm_ty == CAMbased) {
        iFRAT.computeArea();
        iFRAT.area.set_area(iFRAT.area.get_area() + iFRAT.local_result.area);
        area.set_area(area.get_area() + iFRAT.area.get_area());

        fFRAT.computeArea();
        fFRAT.area.set_area(fFRAT.area.get_area() + fFRAT.local_result.area);
        area.set_area(area.get_area() + fFRAT.area.get_area());
      }

      // RRAT is always RAM based, does not have GCs, and is used only for
      // record latest non-speculative mapping RRAT is not needed for CAM-based
      // RAT (McPAT assumes CAM-based RAT to have at least 1 checkpoint), it is
      // not needed for RAM-based RAT with checkpoints McPAT assumes renaming
      // unit to have RRAT when there is no checkpoints in FRAT, while MIPS
      // R1000 has 4 GCs, according to Intel Netburst Archi, combine GC with
      // FRAT is very costly, especially for high issue width and high clock
      // rate.

      if ((coredynp.rm_ty == RAMbased) && (coredynp.globalCheckpoint < 1)) {
        iRRAT.computeArea();
        iRRAT.area.set_area(iRRAT.area.get_area() + iRRAT.local_result.area);
        area.set_area(area.get_area() + iRRAT.area.get_area());

        fRRAT.computeArea();
        fRRAT.area.set_area(fRRAT.area.get_area() + fRRAT.local_result.area);
        area.set_area(area.get_area() + fRRAT.area.get_area());
      }
      // Freelist of renaming unit always RAM based and needed for RAM-based
      // RATs. Although it can be implemented within the CAM-based RAT, Current
      // McPAT does not have the free bits in the CAM but use the same external
      // free list as a close approximation for CAM RAT. Recycle happens at two
      // places: 1)when DCL check there are WAW, the Phy-registers/ROB directly
      // recycles into freelist
      // 2)When instruction commits the Phyregisters/ROB needed to be recycled.
      // therefore num_wr port = decode-1(-1 means at least one phy reg will be
      // used for the current renaming group) + commit width
      ifreeL.computeArea();
      ifreeL.area.set_area(ifreeL.area.get_area() + ifreeL.local_result.area);
      area.set_area(area.get_area() + ifreeL.area.get_area());

      ffreeL.computeArea();
      ffreeL.area.set_area(ffreeL.area.get_area() + ffreeL.local_result.area);
      area.set_area(area.get_area() + ffreeL.area.get_area());

    } else if (coredynp.scheu_ty == ReservationStation) {
      if (coredynp.rm_ty == RAMbased) {
        iFRAT.computeArea();
        iFRAT.local_result.adjust_area();
        //			iFRAT.local_result.power.readOp.dynamic *=
        // 1+0.2*0.05;//1+mis-speculation% TODO
        //			iFRAT.local_result.power.writeOp.dynamic
        //*=1+0.2*0.05;//compensate for GC
        iFRAT.area.set_area(iFRAT.area.get_area() + iFRAT.local_result.area);
        area.set_area(area.get_area() + iFRAT.area.get_area());

        fFRAT.computeArea();
        fFRAT.local_result.adjust_area();
        //			fFRAT.local_result.power.readOp.dynamic *=
        // 1+0.2*0.05;//1+mis-speculation% TODO
        //			fFRAT.local_result.power.writeOp.dynamic
        //*=1+0.2*0.05;//compensate for GC
        fFRAT.area.set_area(fFRAT.area.get_area() + fFRAT.local_result.area);
        area.set_area(area.get_area() + fFRAT.area.get_area());

      } else if (coredynp.rm_ty == CAMbased) {
        // FRAT
        iFRAT.computeArea();
        iFRAT.area.set_area(iFRAT.area.get_area() + iFRAT.local_result.area);
        area.set_area(area.get_area() + iFRAT.area.get_area());

        // FRAT
        fFRAT.computeArea();
        fFRAT.area.set_area(fFRAT.area.get_area() + fFRAT.local_result.area);
        area.set_area(area.get_area() + fFRAT.area.get_area());
      }
      // Although no RRAT for RS based OOO is really needed since the archiRF
      // always holds the non-speculative data, having the RRAT or GC (not both)
      // can help the recovery of mis-speculations.

      if ((coredynp.rm_ty == RAMbased) && (coredynp.globalCheckpoint < 1)) {
        iRRAT.computeArea();
        iRRAT.area.set_area(iRRAT.area.get_area() + iRRAT.local_result.area);
        area.set_area(area.get_area() + iRRAT.area.get_area());

        // RRAT for FP
        fRRAT.computeArea();
        fRRAT.area.set_area(fRRAT.area.get_area() + fRRAT.local_result.area);
        area.set_area(area.get_area() + fRRAT.area.get_area());
      }

      // Freelist of renaming unit of RS based OOO is unifed for both int and fp
      // renaming unit since the ROB is unified
      ifreeL.computeArea();
      // ifreeL.area.set_area(ifreeL.area.get_area()+
      // ifreeL.local_result.area*XML->sys.core[ithCore].number_hardware_threads);
      area.set_area(area.get_area() + ifreeL.area.get_area());
    }
  }
}

void RENAMINGU::computeStaticPower(bool is_tdp) {
  if (!exist) {
    return;
  }
  if (!init_params) {
    std::cerr << "[ RENAMINGU ] Error: must set params before calling "
                 "computeStaticPower()\n";
    exit(1);
  }
  if (!set_area) {
    std::cerr << "[ RENAMINGU ] Error: must computeArea before calling "
                 "computeStaticPower()\n";
    exit(1);
  }
  double pppm_t[4] = {1, 1, 1, 1};
  if (is_tdp) { // init stats for Peak
    if (coredynp.core_ty == OOO) {
      if (coredynp.scheu_ty == PhysicalRegFile) {
        if (coredynp.rm_ty == RAMbased) {
          iFRAT.stats_t.readAc.access = iFRAT.l_ip.num_rd_ports;
          iFRAT.stats_t.writeAc.access = iFRAT.l_ip.num_wr_ports;
          iFRAT.tdp_stats = iFRAT.stats_t;

          fFRAT.stats_t.readAc.access = fFRAT.l_ip.num_rd_ports;
          fFRAT.stats_t.writeAc.access = fFRAT.l_ip.num_wr_ports;
          fFRAT.tdp_stats = fFRAT.stats_t;

        } else if (coredynp.rm_ty == CAMbased) {
          iFRAT.stats_t.readAc.access = iFRAT.l_ip.num_search_ports;
          iFRAT.stats_t.writeAc.access = iFRAT.l_ip.num_wr_ports;
          iFRAT.tdp_stats = iFRAT.stats_t;

          fFRAT.stats_t.readAc.access = fFRAT.l_ip.num_search_ports;
          fFRAT.stats_t.writeAc.access = fFRAT.l_ip.num_wr_ports;
          fFRAT.tdp_stats = fFRAT.stats_t;
        }
        if ((coredynp.rm_ty == RAMbased) && (coredynp.globalCheckpoint < 1)) {
          iRRAT.stats_t.readAc.access = iRRAT.l_ip.num_rd_ports;
          iRRAT.stats_t.writeAc.access = iRRAT.l_ip.num_wr_ports;
          iRRAT.tdp_stats = iRRAT.stats_t;

          fRRAT.stats_t.readAc.access = fRRAT.l_ip.num_rd_ports;
          fRRAT.stats_t.writeAc.access = fRRAT.l_ip.num_wr_ports;
          fRRAT.tdp_stats = fRRAT.stats_t;
        }
        ifreeL.stats_t.readAc.access =
            coredynp.decodeW; // ifreeL.l_ip.num_rd_ports;;
        ifreeL.stats_t.writeAc.access =
            coredynp.decodeW; // ifreeL.l_ip.num_wr_ports;
        ifreeL.tdp_stats = ifreeL.stats_t;

        ffreeL.stats_t.readAc.access =
            coredynp.decodeW; // ffreeL.l_ip.num_rd_ports;
        ffreeL.stats_t.writeAc.access =
            coredynp.decodeW; // ffreeL.l_ip.num_wr_ports;
        ffreeL.tdp_stats = ffreeL.stats_t;
      } else if (coredynp.scheu_ty == ReservationStation) {
        if (coredynp.rm_ty == RAMbased) {
          iFRAT.stats_t.readAc.access = iFRAT.l_ip.num_rd_ports;
          iFRAT.stats_t.writeAc.access = iFRAT.l_ip.num_wr_ports;
          iFRAT.tdp_stats = iFRAT.stats_t;

          fFRAT.stats_t.readAc.access = fFRAT.l_ip.num_rd_ports;
          fFRAT.stats_t.writeAc.access = fFRAT.l_ip.num_wr_ports;
          fFRAT.tdp_stats = fFRAT.stats_t;

        } else if (coredynp.rm_ty == CAMbased) {
          iFRAT.stats_t.readAc.access = iFRAT.l_ip.num_search_ports;
          iFRAT.stats_t.writeAc.access = iFRAT.l_ip.num_wr_ports;
          iFRAT.tdp_stats = iFRAT.stats_t;

          fFRAT.stats_t.readAc.access = fFRAT.l_ip.num_search_ports;
          fFRAT.stats_t.writeAc.access = fFRAT.l_ip.num_wr_ports;
          fFRAT.tdp_stats = fFRAT.stats_t;
        }

        if ((coredynp.rm_ty == RAMbased) && (coredynp.globalCheckpoint < 1)) {
          iRRAT.stats_t.readAc.access = iRRAT.l_ip.num_rd_ports;
          iRRAT.stats_t.writeAc.access = iRRAT.l_ip.num_wr_ports;
          iRRAT.tdp_stats = iRRAT.stats_t;

          fRRAT.stats_t.readAc.access = fRRAT.l_ip.num_rd_ports;
          fRRAT.stats_t.writeAc.access = fRRAT.l_ip.num_wr_ports;
          fRRAT.tdp_stats = fRRAT.stats_t;
        }
        // Unified free list for both int and fp
        ifreeL.stats_t.readAc.access =
            coredynp.decodeW; // ifreeL.l_ip.num_rd_ports;
        ifreeL.stats_t.writeAc.access =
            coredynp.decodeW; // ifreeL.l_ip.num_wr_ports;
        ifreeL.tdp_stats = ifreeL.stats_t;
      }
      idcl.stats_t.readAc.access = coredynp.decodeW;
      fdcl.stats_t.readAc.access = coredynp.decodeW;
      idcl.tdp_stats = idcl.stats_t;
      fdcl.tdp_stats = fdcl.stats_t;
    } else {
      if (coredynp.issueW > 1) {
        idcl.stats_t.readAc.access = coredynp.decodeW;
        fdcl.stats_t.readAc.access = coredynp.decodeW;
        idcl.tdp_stats = idcl.stats_t;
        fdcl.tdp_stats = fdcl.stats_t;
      }
    }

  } else { // init stats for Runtime Dynamic (RTP)
    if (coredynp.core_ty == OOO) {
      if (coredynp.scheu_ty == PhysicalRegFile) {
        if (coredynp.rm_ty == RAMbased) {
          iFRAT.stats_t.readAc.access = rename_reads;
          iFRAT.stats_t.writeAc.access = rename_writes;
          iFRAT.rtp_stats = iFRAT.stats_t;

          fFRAT.stats_t.readAc.access = fp_rename_reads;
          fFRAT.stats_t.writeAc.access = fp_rename_writes;
          fFRAT.rtp_stats = fFRAT.stats_t;
        } else if (coredynp.rm_ty == CAMbased) {
          iFRAT.stats_t.readAc.access = rename_reads;
          iFRAT.stats_t.writeAc.access = rename_writes;
          iFRAT.rtp_stats = iFRAT.stats_t;

          fFRAT.stats_t.readAc.access = fp_rename_reads;
          fFRAT.stats_t.writeAc.access = fp_rename_writes;
          fFRAT.rtp_stats = fFRAT.stats_t;
        }
        if ((coredynp.rm_ty == RAMbased) && (coredynp.globalCheckpoint < 1)) {
          // HACK, should be (context switch + branch mispredictions)*16
          iRRAT.stats_t.readAc.access = rename_writes;
          iRRAT.stats_t.writeAc.access = rename_writes;
          iRRAT.rtp_stats = iRRAT.stats_t;

          // HACK, should be (context switch + branch mispredictions)*16
          fRRAT.stats_t.readAc.access = fp_rename_writes;
          fRRAT.stats_t.writeAc.access = fp_rename_writes;
          fRRAT.rtp_stats = fRRAT.stats_t;
        }
        ifreeL.stats_t.readAc.access = rename_reads;
        ifreeL.stats_t.writeAc.access = 2 * rename_writes;
        ifreeL.rtp_stats = ifreeL.stats_t;

        ffreeL.stats_t.readAc.access = fp_rename_reads;
        ffreeL.stats_t.writeAc.access = 2 * fp_rename_writes;
        ffreeL.rtp_stats = ffreeL.stats_t;
      } else if (coredynp.scheu_ty == ReservationStation) {
        if (coredynp.rm_ty == RAMbased) {
          iFRAT.stats_t.readAc.access = rename_reads;
          iFRAT.stats_t.writeAc.access = rename_writes;
          iFRAT.rtp_stats = iFRAT.stats_t;

          fFRAT.stats_t.readAc.access = fp_rename_reads;
          fFRAT.stats_t.writeAc.access = fp_rename_writes;
          fFRAT.rtp_stats = fFRAT.stats_t;
        } else if (coredynp.rm_ty == CAMbased) {
          iFRAT.stats_t.readAc.access = rename_reads;
          iFRAT.stats_t.writeAc.access = rename_writes;
          iFRAT.rtp_stats = iFRAT.stats_t;

          fFRAT.stats_t.readAc.access = fp_rename_reads;
          fFRAT.stats_t.writeAc.access = fp_rename_writes;
          fFRAT.rtp_stats = fFRAT.stats_t;
        }

        if ((coredynp.rm_ty == RAMbased) && (coredynp.globalCheckpoint < 1)) {
          // HACK, should be (context switch + branch mispredictions)*16
          iRRAT.stats_t.readAc.access = rename_writes;
          iRRAT.stats_t.writeAc.access = rename_writes;
          iRRAT.rtp_stats = iRRAT.stats_t;

          // HACK, should be (context switch + branch mispredictions)*16
          fRRAT.stats_t.readAc.access = fp_rename_writes;
          fRRAT.stats_t.writeAc.access = fp_rename_writes;
          fRRAT.rtp_stats = fRRAT.stats_t;
        }
        // Unified free list for both int and fp since the ROB act as physcial
        // registers
        ifreeL.stats_t.readAc.access = rename_reads + fp_rename_reads;
        // HACK: 2-> since some of renaming in the same group are terminated
        // early
        ifreeL.stats_t.writeAc.access = 2 * (rename_writes + fp_rename_writes);
        ifreeL.rtp_stats = ifreeL.stats_t;
      }
      idcl.stats_t.readAc.access =
          3 * coredynp.decodeW * coredynp.decodeW * rename_reads;
      fdcl.stats_t.readAc.access =
          3 * coredynp.fp_issueW * coredynp.fp_issueW * fp_rename_writes;
      idcl.rtp_stats = idcl.stats_t;
      fdcl.rtp_stats = fdcl.stats_t;
    } else {
      if (coredynp.issueW > 1) {
        idcl.stats_t.readAc.access = 2 * int_instructions;
        fdcl.stats_t.readAc.access = fp_instructions;
        idcl.rtp_stats = idcl.stats_t;
        fdcl.rtp_stats = fdcl.stats_t;
      }
    }
  }
  /* Compute engine */
  if (coredynp.core_ty == OOO) {
    if (coredynp.scheu_ty == PhysicalRegFile) {
      if (coredynp.rm_ty == RAMbased) {
        iFRAT.power_t.reset();
        fFRAT.power_t.reset();

        iFRAT.power_t.readOp.dynamic +=
            (iFRAT.stats_t.readAc.access *
                 (iFRAT.local_result.power.readOp.dynamic +
                  idcl.power.readOp.dynamic) +
             iFRAT.stats_t.writeAc.access *
                 iFRAT.local_result.power.writeOp.dynamic);
        fFRAT.power_t.readOp.dynamic +=
            (fFRAT.stats_t.readAc.access *
                 (fFRAT.local_result.power.readOp.dynamic +
                  fdcl.power.readOp.dynamic) +
             fFRAT.stats_t.writeAc.access *
                 fFRAT.local_result.power.writeOp.dynamic);
      } else if (coredynp.rm_ty == CAMbased) {
        iFRAT.power_t.reset();
        fFRAT.power_t.reset();
        iFRAT.power_t.readOp.dynamic +=
            (iFRAT.stats_t.readAc.access *
                 (iFRAT.local_result.power.searchOp.dynamic +
                  idcl.power.readOp.dynamic) +
             iFRAT.stats_t.writeAc.access *
                 iFRAT.local_result.power.writeOp.dynamic);
        fFRAT.power_t.readOp.dynamic +=
            (fFRAT.stats_t.readAc.access *
                 (fFRAT.local_result.power.searchOp.dynamic +
                  fdcl.power.readOp.dynamic) +
             fFRAT.stats_t.writeAc.access *
                 fFRAT.local_result.power.writeOp.dynamic);
      }
      if ((coredynp.rm_ty == RAMbased) && (coredynp.globalCheckpoint < 1)) {
        iRRAT.power_t.reset();
        fRRAT.power_t.reset();

        iRRAT.power_t.readOp.dynamic +=
            (iRRAT.stats_t.readAc.access *
                 iRRAT.local_result.power.readOp.dynamic +
             iRRAT.stats_t.writeAc.access *
                 iRRAT.local_result.power.writeOp.dynamic);
        fRRAT.power_t.readOp.dynamic +=
            (fRRAT.stats_t.readAc.access *
                 fRRAT.local_result.power.readOp.dynamic +
             fRRAT.stats_t.writeAc.access *
                 fRRAT.local_result.power.writeOp.dynamic);
      }

      ifreeL.power_t.reset();
      ffreeL.power_t.reset();
      ifreeL.power_t.readOp.dynamic +=
          (ifreeL.stats_t.readAc.access *
               ifreeL.local_result.power.readOp.dynamic +
           ifreeL.stats_t.writeAc.access *
               ifreeL.local_result.power.writeOp.dynamic);
      ffreeL.power_t.readOp.dynamic +=
          (ffreeL.stats_t.readAc.access *
               ffreeL.local_result.power.readOp.dynamic +
           ffreeL.stats_t.writeAc.access *
               ffreeL.local_result.power.writeOp.dynamic);

    } else if (coredynp.scheu_ty == ReservationStation) {
      if (coredynp.rm_ty == RAMbased) {
        iFRAT.power_t.reset();
        fFRAT.power_t.reset();

        iFRAT.power_t.readOp.dynamic +=
            (iFRAT.stats_t.readAc.access *
                 (iFRAT.local_result.power.readOp.dynamic +
                  idcl.power.readOp.dynamic) +
             iFRAT.stats_t.writeAc.access *
                 iFRAT.local_result.power.writeOp.dynamic);
        fFRAT.power_t.readOp.dynamic +=
            (fFRAT.stats_t.readAc.access *
                 (fFRAT.local_result.power.readOp.dynamic +
                  fdcl.power.readOp.dynamic) +
             fFRAT.stats_t.writeAc.access *
                 fFRAT.local_result.power.writeOp.dynamic);
      } else if (coredynp.rm_ty == CAMbased) {
        iFRAT.power_t.reset();
        fFRAT.power_t.reset();
        iFRAT.power_t.readOp.dynamic +=
            (iFRAT.stats_t.readAc.access *
                 (iFRAT.local_result.power.searchOp.dynamic +
                  idcl.power.readOp.dynamic) +
             iFRAT.stats_t.writeAc.access *
                 iFRAT.local_result.power.writeOp.dynamic);
        fFRAT.power_t.readOp.dynamic +=
            (fFRAT.stats_t.readAc.access *
                 (fFRAT.local_result.power.searchOp.dynamic +
                  fdcl.power.readOp.dynamic) +
             fFRAT.stats_t.writeAc.access *
                 fFRAT.local_result.power.writeOp.dynamic);
      }

      if ((coredynp.rm_ty == RAMbased) && (coredynp.globalCheckpoint < 1)) {
        iRRAT.power_t.reset();
        fRRAT.power_t.reset();

        iRRAT.power_t.readOp.dynamic +=
            (iRRAT.stats_t.readAc.access *
                 iRRAT.local_result.power.readOp.dynamic +
             iRRAT.stats_t.writeAc.access *
                 iRRAT.local_result.power.writeOp.dynamic);
        fRRAT.power_t.readOp.dynamic +=
            (fRRAT.stats_t.readAc.access *
                 fRRAT.local_result.power.readOp.dynamic +
             fRRAT.stats_t.writeAc.access *
                 fRRAT.local_result.power.writeOp.dynamic);
      }

      ifreeL.power_t.reset();
      ifreeL.power_t.readOp.dynamic +=
          (ifreeL.stats_t.readAc.access *
               ifreeL.local_result.power.readOp.dynamic +
           ifreeL.stats_t.writeAc.access *
               ifreeL.local_result.power.writeOp.dynamic);
    }

  } else {
    if (coredynp.issueW > 1) {
      idcl.power_t.reset();
      fdcl.power_t.reset();
      set_pppm(pppm_t,
               idcl.stats_t.readAc.access,
               coredynp.num_hthreads,
               coredynp.num_hthreads,
               idcl.stats_t.readAc.access);
      idcl.power_t = idcl.power * pppm_t;
      set_pppm(pppm_t,
               fdcl.stats_t.readAc.access,
               coredynp.num_hthreads,
               coredynp.num_hthreads,
               idcl.stats_t.readAc.access);
      fdcl.power_t = fdcl.power * pppm_t;
    }
  }

  // assign value to tpd and rtp
  if (is_tdp) {
    if (coredynp.core_ty == OOO) {
      if (coredynp.scheu_ty == PhysicalRegFile) {
        iFRAT.power = iFRAT.power_t + (iFRAT.local_result.power) + idcl.power_t;
        fFRAT.power = fFRAT.power_t + (fFRAT.local_result.power) + fdcl.power_t;
        ifreeL.power = ifreeL.power_t + ifreeL.local_result.power;
        ffreeL.power = ffreeL.power_t + ffreeL.local_result.power;
        power = power +
                (iFRAT.power + fFRAT.power)
                //+ (iRRAT.power + fRRAT.power)
                + (ifreeL.power + ffreeL.power);
        if ((coredynp.rm_ty == RAMbased) && (coredynp.globalCheckpoint < 1)) {
          iRRAT.power = iRRAT.power_t + iRRAT.local_result.power;
          fRRAT.power = fRRAT.power_t + fRRAT.local_result.power;
          power = power + (iRRAT.power + fRRAT.power);
        }
      } else if (coredynp.scheu_ty == ReservationStation) {
        iFRAT.power = iFRAT.power_t + (iFRAT.local_result.power) + idcl.power_t;
        fFRAT.power = fFRAT.power_t + (fFRAT.local_result.power) + fdcl.power_t;
        ifreeL.power = ifreeL.power_t + ifreeL.local_result.power;
        power = power + (iFRAT.power + fFRAT.power) + ifreeL.power;
        if ((coredynp.rm_ty == RAMbased) && (coredynp.globalCheckpoint < 1)) {
          iRRAT.power = iRRAT.power_t + iRRAT.local_result.power;
          fRRAT.power = fRRAT.power_t + fRRAT.local_result.power;
          power = power + (iRRAT.power + fRRAT.power);
        }
      }
    } else {
      power = power + idcl.power_t + fdcl.power_t;
    }

  } else {
    if (coredynp.core_ty == OOO) {
      if (coredynp.scheu_ty == PhysicalRegFile) {
        iFRAT.rt_power =
            iFRAT.power_t + (iFRAT.local_result.power) + idcl.power_t;
        fFRAT.rt_power =
            fFRAT.power_t + (fFRAT.local_result.power) + fdcl.power_t;

        ifreeL.rt_power = ifreeL.power_t + ifreeL.local_result.power;
        ffreeL.rt_power = ffreeL.power_t + ffreeL.local_result.power;
        rt_power = rt_power +
                   (iFRAT.rt_power + fFRAT.rt_power)
                   //			                   + (iRRAT.rt_power +
                   // fRRAT.rt_power)
                   + (ifreeL.rt_power + ffreeL.rt_power);

        if ((coredynp.rm_ty == RAMbased) && (coredynp.globalCheckpoint < 1)) {
          iRRAT.rt_power = iRRAT.power_t + iRRAT.local_result.power;
          fRRAT.rt_power = fRRAT.power_t + fRRAT.local_result.power;
          rt_power = rt_power + (iRRAT.rt_power + fRRAT.rt_power);
        }
      } else if (coredynp.scheu_ty == ReservationStation) {
        iFRAT.rt_power =
            iFRAT.power_t + (iFRAT.local_result.power) + idcl.power_t;
        fFRAT.rt_power =
            fFRAT.power_t + (fFRAT.local_result.power) + fdcl.power_t;
        ifreeL.rt_power = ifreeL.power_t + ifreeL.local_result.power;
        rt_power =
            rt_power + (iFRAT.rt_power + fFRAT.rt_power) + ifreeL.rt_power;
        if ((coredynp.rm_ty == RAMbased) && (coredynp.globalCheckpoint < 1)) {
          iRRAT.rt_power = iRRAT.power_t + iRRAT.local_result.power;
          fRRAT.rt_power = fRRAT.power_t + fRRAT.local_result.power;
          rt_power = rt_power + (iRRAT.rt_power + fRRAT.rt_power);
        }
      }
    } else {
      rt_power = rt_power + idcl.power_t + fdcl.power_t;
    }
  }
}

void RENAMINGU::display(uint32_t indent, int plevel, bool is_tdp) {
  if (!exist) {
    return;
  }
  string indent_str(indent, ' ');
  string indent_str_next(indent + 2, ' ');

  if (is_tdp) {

    if (coredynp.core_ty == OOO) {
      cout << indent_str << "Int Front End RAT with "
           << coredynp.globalCheckpoint << " internal checkpoints:" << endl;
      cout << indent_str_next << "Area = " << iFRAT.area.get_area() * 1e-6
           << " mm^2" << endl;
      cout << indent_str_next
           << "Peak Dynamic = " << iFRAT.power.readOp.dynamic * clockRate
           << " W" << endl;
      cout << indent_str_next << "Subthreshold Leakage = "
           << (long_channel ? iFRAT.power.readOp.longer_channel_leakage
                            : iFRAT.power.readOp.leakage)
           << " W" << endl;
      if (power_gating) {
        cout << indent_str_next << "Subthreshold Leakage with power gating = "
             << (long_channel
                     ? iFRAT.power.readOp.power_gated_with_long_channel_leakage
                     : iFRAT.power.readOp.power_gated_leakage)
             << " W" << endl;
      }
      cout << indent_str_next
           << "Gate Leakage = " << iFRAT.power.readOp.gate_leakage << " W"
           << endl;
      cout << indent_str_next << "Runtime Dynamic = "
           << iFRAT.rt_power.readOp.dynamic / executionTime << " W" << endl;
      cout << endl;
      cout << indent_str << "FP Front End RAT with "
           << coredynp.globalCheckpoint << " internal checkpoints:" << endl;
      cout << indent_str_next << "Area = " << fFRAT.area.get_area() * 1e-6
           << " mm^2" << endl;
      cout << indent_str_next
           << "Peak Dynamic = " << fFRAT.power.readOp.dynamic * clockRate
           << " W" << endl;
      cout << indent_str_next << "Subthreshold Leakage = "
           << (long_channel ? fFRAT.power.readOp.longer_channel_leakage
                            : fFRAT.power.readOp.leakage)
           << " W" << endl;
      if (power_gating) {
        cout << indent_str_next << "Subthreshold Leakage with power gating = "
             << (long_channel
                     ? fFRAT.power.readOp.power_gated_with_long_channel_leakage
                     : fFRAT.power.readOp.power_gated_leakage)
             << " W" << endl;
      }
      cout << indent_str_next
           << "Gate Leakage = " << fFRAT.power.readOp.gate_leakage << " W"
           << endl;
      cout << indent_str_next << "Runtime Dynamic = "
           << fFRAT.rt_power.readOp.dynamic / executionTime << " W" << endl;
      cout << endl;
      cout << indent_str << "Free List:" << endl;
      cout << indent_str_next << "Area = " << ifreeL.area.get_area() * 1e-6
           << " mm^2" << endl;
      cout << indent_str_next
           << "Peak Dynamic = " << ifreeL.power.readOp.dynamic * clockRate
           << " W" << endl;
      cout << indent_str_next << "Subthreshold Leakage = "
           << (long_channel ? ifreeL.power.readOp.longer_channel_leakage
                            : ifreeL.power.readOp.leakage)
           << " W" << endl;
      if (power_gating) {
        cout << indent_str_next << "Subthreshold Leakage with power gating = "
             << (long_channel
                     ? ifreeL.power.readOp.power_gated_with_long_channel_leakage
                     : ifreeL.power.readOp.power_gated_leakage)
             << " W" << endl;
      }
      cout << indent_str_next
           << "Gate Leakage = " << ifreeL.power.readOp.gate_leakage << " W"
           << endl;
      cout << indent_str_next << "Runtime Dynamic = "
           << ifreeL.rt_power.readOp.dynamic / executionTime << " W" << endl;
      cout << endl;
      if ((coredynp.rm_ty == RAMbased) && (coredynp.globalCheckpoint < 1)) {
        cout << indent_str << "Int Retire RAT: " << endl;
        cout << indent_str_next << "Area = " << iRRAT.area.get_area() * 1e-6
             << " mm^2" << endl;
        cout << indent_str_next
             << "Peak Dynamic = " << iRRAT.power.readOp.dynamic * clockRate
             << " W" << endl;
        cout << indent_str_next << "Subthreshold Leakage = "
             << (long_channel ? iRRAT.power.readOp.longer_channel_leakage
                              : iRRAT.power.readOp.leakage)
             << " W" << endl;
        if (power_gating) {
          cout
              << indent_str_next << "Subthreshold Leakage with power gating = "
              << (long_channel
                      ? iRRAT.power.readOp.power_gated_with_long_channel_leakage
                      : iRRAT.power.readOp.power_gated_leakage)
              << " W" << endl;
        }
        cout << indent_str_next
             << "Gate Leakage = " << iRRAT.power.readOp.gate_leakage << " W"
             << endl;
        cout << indent_str_next << "Runtime Dynamic = "
             << iRRAT.rt_power.readOp.dynamic / executionTime << " W" << endl;
        cout << endl;
        cout << indent_str << "FP Retire RAT:" << endl;
        cout << indent_str_next << "Area = " << fRRAT.area.get_area() * 1e-6
             << " mm^2" << endl;
        cout << indent_str_next
             << "Peak Dynamic = " << fRRAT.power.readOp.dynamic * clockRate
             << " W" << endl;
        cout << indent_str_next << "Subthreshold Leakage = "
             << (long_channel ? fRRAT.power.readOp.longer_channel_leakage
                              : fRRAT.power.readOp.leakage)
             << " W" << endl;
        if (power_gating) {
          cout
              << indent_str_next << "Subthreshold Leakage with power gating = "
              << (long_channel
                      ? fRRAT.power.readOp.power_gated_with_long_channel_leakage
                      : fRRAT.power.readOp.power_gated_leakage)
              << " W" << endl;
        }
        cout << indent_str_next
             << "Gate Leakage = " << fRRAT.power.readOp.gate_leakage << " W"
             << endl;
        cout << indent_str_next << "Runtime Dynamic = "
             << fRRAT.rt_power.readOp.dynamic / executionTime << " W" << endl;
        cout << endl;
      }
      if (coredynp.scheu_ty == PhysicalRegFile) {
        cout << indent_str << "FP Free List:" << endl;
        cout << indent_str_next << "Area = " << ffreeL.area.get_area() * 1e-6
             << " mm^2" << endl;
        cout << indent_str_next
             << "Peak Dynamic = " << ffreeL.power.readOp.dynamic * clockRate
             << " W" << endl;
        cout << indent_str_next << "Subthreshold Leakage = "
             << (long_channel ? ffreeL.power.readOp.longer_channel_leakage
                              : ffreeL.power.readOp.leakage)
             << " W" << endl;
        if (power_gating) {
          cout << indent_str_next << "Subthreshold Leakage with power gating = "
               << (long_channel ? ffreeL.power.readOp
                                      .power_gated_with_long_channel_leakage
                                : ffreeL.power.readOp.power_gated_leakage)
               << " W" << endl;
        }
        cout << indent_str_next
             << "Gate Leakage = " << ffreeL.power.readOp.gate_leakage << " W"
             << endl;
        cout << indent_str_next << "Runtime Dynamic = "
             << ffreeL.rt_power.readOp.dynamic / executionTime << " W" << endl;
        cout << endl;
      }
    } else {
      cout << indent_str << "Int DCL:" << endl;
      cout << indent_str_next
           << "Peak Dynamic = " << idcl.power.readOp.dynamic * clockRate << " W"
           << endl;
      cout << indent_str_next << "Subthreshold Leakage = "
           << (long_channel ? idcl.power.readOp.longer_channel_leakage
                            : idcl.power.readOp.leakage)
           << " W" << endl;
      if (power_gating) {
        cout << indent_str_next << "Subthreshold Leakage with power gating = "
             << (long_channel
                     ? idcl.power.readOp.power_gated_with_long_channel_leakage
                     : idcl.power.readOp.power_gated_leakage)
             << " W" << endl;
      }
      cout << indent_str_next
           << "Gate Leakage = " << idcl.power.readOp.gate_leakage << " W"
           << endl;
      cout << indent_str_next << "Runtime Dynamic = "
           << idcl.rt_power.readOp.dynamic / executionTime << " W" << endl;
      cout << indent_str << "FP DCL:" << endl;
      cout << indent_str_next
           << "Peak Dynamic = " << fdcl.power.readOp.dynamic * clockRate << " W"
           << endl;
      cout << indent_str_next << "Subthreshold Leakage = "
           << (long_channel ? fdcl.power.readOp.longer_channel_leakage
                            : fdcl.power.readOp.leakage)
           << " W" << endl;
      if (power_gating) {
        cout << indent_str_next << "Subthreshold Leakage with power gating = "
             << (long_channel
                     ? fdcl.power.readOp.power_gated_with_long_channel_leakage
                     : fdcl.power.readOp.power_gated_leakage)
             << " W" << endl;
      }
      cout << indent_str_next
           << "Gate Leakage = " << fdcl.power.readOp.gate_leakage << " W"
           << endl;
      cout << indent_str_next << "Runtime Dynamic = "
           << fdcl.rt_power.readOp.dynamic / executionTime << " W" << endl;
    }
  } else {
    if (coredynp.core_ty == OOO) {
      cout << indent_str_next << "Int Front End RAT    Peak Dynamic = "
           << iFRAT.rt_power.readOp.dynamic * clockRate << " W" << endl;
      cout << indent_str_next << "Int Front End RAT    Subthreshold Leakage = "
           << iFRAT.rt_power.readOp.leakage << " W" << endl;
      cout << indent_str_next << "Int Front End RAT    Gate Leakage = "
           << iFRAT.rt_power.readOp.gate_leakage << " W" << endl;
      cout << indent_str_next << "FP Front End RAT   Peak Dynamic = "
           << fFRAT.rt_power.readOp.dynamic * clockRate << " W" << endl;
      cout << indent_str_next << "FP Front End RAT   Subthreshold Leakage = "
           << fFRAT.rt_power.readOp.leakage << " W" << endl;
      cout << indent_str_next << "FP Front End RAT   Gate Leakage = "
           << fFRAT.rt_power.readOp.gate_leakage << " W" << endl;
      cout << indent_str_next << "Free List   Peak Dynamic = "
           << ifreeL.rt_power.readOp.dynamic * clockRate << " W" << endl;
      cout << indent_str_next << "Free List   Subthreshold Leakage = "
           << ifreeL.rt_power.readOp.leakage << " W" << endl;
      cout << indent_str_next << "Free List   Gate Leakage = "
           << fFRAT.rt_power.readOp.gate_leakage << " W" << endl;
      if (coredynp.scheu_ty == PhysicalRegFile) {
        if ((coredynp.rm_ty == RAMbased) && (coredynp.globalCheckpoint < 1)) {
          cout << indent_str_next << "Int Retire RAT   Peak Dynamic = "
               << iRRAT.rt_power.readOp.dynamic * clockRate << " W" << endl;
          cout << indent_str_next << "Int Retire RAT   Subthreshold Leakage = "
               << iRRAT.rt_power.readOp.leakage << " W" << endl;
          cout << indent_str_next << "Int Retire RAT   Gate Leakage = "
               << iRRAT.rt_power.readOp.gate_leakage << " W" << endl;
          cout << indent_str_next << "FP Retire RAT   Peak Dynamic = "
               << fRRAT.rt_power.readOp.dynamic * clockRate << " W" << endl;
          cout << indent_str_next << "FP Retire RAT   Subthreshold Leakage = "
               << fRRAT.rt_power.readOp.leakage << " W" << endl;
          cout << indent_str_next << "FP Retire RAT   Gate Leakage = "
               << fRRAT.rt_power.readOp.gate_leakage << " W" << endl;
        }
        cout << indent_str_next << "FP Free List   Peak Dynamic = "
             << ffreeL.rt_power.readOp.dynamic * clockRate << " W" << endl;
        cout << indent_str_next << "FP Free List   Subthreshold Leakage = "
             << ffreeL.rt_power.readOp.leakage << " W" << endl;
        cout << indent_str_next << "FP Free List   Gate Leakage = "
             << fFRAT.rt_power.readOp.gate_leakage << " W" << endl;
      }
    } else {
      cout << indent_str_next << "Int DCL   Peak Dynamic = "
           << idcl.rt_power.readOp.dynamic * clockRate << " W" << endl;
      cout << indent_str_next << "Int DCL   Subthreshold Leakage = "
           << idcl.rt_power.readOp.leakage << " W" << endl;
      cout << indent_str_next
           << "Int DCL   Gate Leakage = " << idcl.rt_power.readOp.gate_leakage
           << " W" << endl;
      cout << indent_str_next << "FP DCL   Peak Dynamic = "
           << fdcl.rt_power.readOp.dynamic * clockRate << " W" << endl;
      cout << indent_str_next
           << "FP DCL   Subthreshold Leakage = " << fdcl.rt_power.readOp.leakage
           << " W" << endl;
      cout << indent_str_next
           << "FP DCL   Gate Leakage = " << fdcl.rt_power.readOp.gate_leakage
           << " W" << endl;
    }
  }
}

RENAMINGU ::~RENAMINGU() {
  if (!exist) {
    return;
  }
}
