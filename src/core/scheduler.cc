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

#include "scheduler.h"

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

SchedulerU::SchedulerU() {
  init_params = false;
  init_stats = false;
}

void SchedulerU::set_params(const ParseXML *XML_interface,
                            int ithCore_,
                            InputParameter *interface_ip_,
                            const CoreDynParam &dyn_p_,
                            bool exist_) {

  XML = XML_interface;
  interface_ip = *interface_ip_;
  coredynp = dyn_p_;
  ithCore = ithCore_;

  exist = exist_;

  if (!exist)
    return;
  int tag, data;
  bool is_default = true;
  string tmp_name;

  clockRate = coredynp.clockRate;
  executionTime = coredynp.executionTime;
  if ((coredynp.core_ty == Inorder && coredynp.multithreaded)) {
    // Instruction issue queue, in-order multi-issue or multithreaded processor
    // also has this structure. Unified window for Inorder processors
    tag = int(log2(XML->sys.core[ithCore].number_hardware_threads) *
              coredynp.perThreadState); // This is the normal thread state bits
                                        // based on Niagara Design
    data = XML->sys.core[ithCore].instruction_length;
    // NOTE: x86 inst can be very lengthy, up to 15B. Source: Intel® 64 and
    // IA-32 Architectures Software Developer’s Manual
    interface_ip.is_cache = true;
    interface_ip.pure_cam = false;
    interface_ip.pure_ram = false;
    interface_ip.line_sz = int(ceil(data / 8.0));
    interface_ip.specific_tag = 1;
    interface_ip.tag_w = tag;
    interface_ip.cache_sz =
        XML->sys.core[ithCore].instruction_window_size * interface_ip.line_sz >
                64
            ? XML->sys.core[ithCore].instruction_window_size *
                  interface_ip.line_sz
            : 64;
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
    interface_ip.num_rd_ports = coredynp.peak_issueW;
    interface_ip.num_wr_ports = coredynp.peak_issueW;
    interface_ip.num_se_rd_ports = 0;
    interface_ip.num_search_ports = coredynp.peak_issueW;
    int_inst_window.set_params(&interface_ip,
                               "InstFetchQueue",
                               Core_device,
                               coredynp.opt_local,
                               coredynp.core_ty);
    /*
     * selection logic
     * In a single-issue Inorder multithreaded processor like Niagara, issue
     * width=1*number_of_threads since the processor does need to pick up
     * instructions from multiple ready ones(although these ready ones are from
     * different threads).While SMT processors do not distinguish which thread
     * belongs to who at the issue stage.
     */
    interface_ip.assoc =
        1; // reset to prevent unnecessary warning messages when init_interface
    instruction_selection.set_params(
        is_default,
        XML->sys.core[ithCore].instruction_window_size,
        coredynp.peak_issueW * XML->sys.core[ithCore].number_hardware_threads,
        &interface_ip,
        Core_device,
        coredynp.core_ty);
  }

  if (coredynp.core_ty == OOO) {
    /*
     * CAM based instruction window
     * For physicalRegFilebased OOO it is the instruction issue queue, where
     * only tags of phy regs are stored For RS based OOO it is the Reservation
     * station, where both tags and values of phy regs are stored It is written
     * once and read twice(two operands) before an instruction can be issued.
     * X86 instruction can be very long up to 15B. add instruction length in XML
     */
    if (coredynp.scheu_ty == PhysicalRegFile) {
      tag = coredynp.phy_ireg_width;
      // Each time only half of the tag is compared, but two tag should be
      // stored. This underestimate the search power
      data =
          int((ceil((coredynp.instruction_length +
                     2 * (coredynp.phy_ireg_width - coredynp.arch_ireg_width)) /
                    2.0) /
               8.0));
      // Data width being divided by 2 means only after both operands available
      // the whole data will be read out. This is modeled using two equivalent
      // readouts with half of the data width
      tmp_name = "InstIssueQueue";
    } else {
      tag = coredynp.phy_ireg_width;
      // Each time only half of the tag is compared, but two tag should be
      // stored. This underestimate the search power
      data =
          int(ceil(((coredynp.instruction_length +
                     2 * (coredynp.phy_ireg_width - coredynp.arch_ireg_width) +
                     2 * coredynp.int_data_width) /
                    2.0) /
                   8.0));
      // Data width being divided by 2 means only after both operands available
      // the whole data will be read out. This is modeled using two equivalent
      // readouts with half of the data width

      tmp_name = "IntReservationStation";
    }
    interface_ip.is_cache = true;
    interface_ip.pure_cam = false;
    interface_ip.pure_ram = false;
    interface_ip.line_sz = data;
    interface_ip.cache_sz =
        data * XML->sys.core[ithCore].instruction_window_size;
    interface_ip.assoc = 0;
    interface_ip.nbanks = 1;
    interface_ip.out_w = interface_ip.line_sz * 8;
    interface_ip.specific_tag = 1;
    interface_ip.tag_w = tag;
    interface_ip.access_mode = 0;
    interface_ip.throughput = 2 * 1.0 / clockRate;
    interface_ip.latency = 2 * 1.0 / clockRate;
    interface_ip.obj_func_dyn_energy = 0;
    interface_ip.obj_func_dyn_power = 0;
    interface_ip.obj_func_leak_power = 0;
    interface_ip.obj_func_cycle_t = 1;
    interface_ip.num_rw_ports = 0;
    interface_ip.num_rd_ports = coredynp.peak_issueW;
    interface_ip.num_wr_ports = coredynp.peak_issueW;
    interface_ip.num_se_rd_ports = 0;
    interface_ip.num_search_ports = coredynp.peak_issueW;
    int_inst_window.set_params(&interface_ip,
                               tmp_name,
                               Core_device,
                               coredynp.opt_local,
                               coredynp.core_ty);
    // FU inst window
    if (coredynp.scheu_ty == PhysicalRegFile) {
      tag = 2 * coredynp.phy_freg_width; // TODO: each time only half of the tag
                                         // is compared
      data =
          int(ceil((coredynp.instruction_length +
                    2 * (coredynp.phy_freg_width - coredynp.arch_freg_width)) /
                   8.0));
      tmp_name = "FPIssueQueue";
    } else {
      tag = 2 * coredynp.phy_ireg_width;
      data =
          int(ceil((coredynp.instruction_length +
                    2 * (coredynp.phy_freg_width - coredynp.arch_freg_width) +
                    2 * coredynp.fp_data_width) /
                   8.0));
      tmp_name = "FPReservationStation";
    }
    interface_ip.is_cache = true;
    interface_ip.pure_cam = false;
    interface_ip.pure_ram = false;
    interface_ip.line_sz = data;
    interface_ip.cache_sz =
        data * XML->sys.core[ithCore].fp_instruction_window_size;
    interface_ip.assoc = 0;
    interface_ip.nbanks = 1;
    interface_ip.out_w = interface_ip.line_sz * 8;
    interface_ip.specific_tag = 1;
    interface_ip.tag_w = tag;
    interface_ip.access_mode = 0;
    interface_ip.throughput = 1.0 / clockRate;
    interface_ip.latency = 1.0 / clockRate;
    interface_ip.obj_func_dyn_energy = 0;
    interface_ip.obj_func_dyn_power = 0;
    interface_ip.obj_func_leak_power = 0;
    interface_ip.obj_func_cycle_t = 1;
    interface_ip.num_rw_ports = 0;
    interface_ip.num_rd_ports = coredynp.fp_issueW;
    interface_ip.num_wr_ports = coredynp.fp_issueW;
    interface_ip.num_se_rd_ports = 0;
    interface_ip.num_search_ports = coredynp.fp_issueW;
    fp_inst_window.set_params(&interface_ip,
                              tmp_name,
                              Core_device,
                              coredynp.opt_local,
                              coredynp.core_ty);
    if (XML->sys.core[ithCore].ROB_size > 0) {
      /*
       *  if ROB_size = 0, then the target processor does not support
       *hardware-based speculation, i.e. , the processor allow OOO issue as well
       *as OOO completion, which means branch must be resolved before
       *instruction issued into instruction window, since there is no change to
       *flush miss-predict branch path after instructions are issued in this
       *situation.
       *
       *  ROB.ROB size = inflight inst. ROB is unified for int and fp inst.
       *  One old approach is to combine the RAT and ROB as a huge CAM structure
       *as in AMD K7. However, this approach is abandoned due to its high power
       *and poor scalability. McPAT uses current implementation of ROB as
       *circular buffer. ROB is written once when instruction is issued and read
       *once when the instruction is committed.         *
       */

      int robExtra = int(ceil(5 + log2(coredynp.num_hthreads)));
      data = int(ceil(
          (robExtra + coredynp.pc_width +
           ((coredynp.rm_ty == RAMbased)
                ? (coredynp.phy_ireg_width + coredynp.phy_freg_width)
                : fmax(coredynp.phy_ireg_width, coredynp.phy_freg_width)) +
           ((coredynp.scheu_ty == PhysicalRegFile) ? 0
                                                   : coredynp.fp_data_width)) /
          8.0));
      /*
       * 	5 bits are: busy, Issued, Finished, speculative, valid;
       * 	PC is to id the instruction for recover
       * exception/mis-prediction. When using RAM-based RAT, ROB needs to
       * contain the ARF-PRF mapping to index the correct entry in the RAT, so
       * that the correct architecture register (and freelist) can be found and
       * the RAT can be appropriately updated; otherwise, the RAM-based RAT
       * needs to support search ops to identify the target architecture
       * register that needs to be updated, or the physical resigner that needs
       * to be recycled; When using CAM-based RAT, ROB only needs to contain
       * destination physical register since the CAM-base RAT can search for the
       * corresponding ARF-PRF mapping to find the correct entry in the RAT, so
       * that the correct architecture register (and freelist/bits) can be found
       * and the RAT can be appropriately updated. ROB phy_reg entry should use
       * the larger one from phy_ireg and phy_freg; fdata_width is always
       * larger. Latest Intel Processors may have different ROB/RS designs.
       */

      /*
                              if(coredynp.scheu_ty==PhysicalRegFile)
                              {
                                      //PC is to id the instruction for recover
      exception.
                                      //inst is used to map the renamed dest.
      registers.so that commit stage can know which reg/RRAT to update
      //				data =
      int(ceil((robExtra+coredynp.pc_width
      +
      //						coredynp.instruction_length
      + 2*coredynp.phy_ireg_width)/8.0));

                                      if (coredynp.rm_ty ==RAMbased)
                                      {
                                              data = int(ceil((robExtra +
      coredynp.pc_width + (coredynp.phy_ireg_width,
      coredynp.phy_freg_width))/8.0));
                                              //When using RAM-based RAT, ROB
      needs to contain the ARF-PRF mapping to index the correct entry in the
      RAT,
                                              //so that the correct architecture
      register (and freelist) can be found and the RAT can be appropriately
      updated.
                                      }
                                      else if ((coredynp.rm_ty ==CAMbased))
                                      {
                                              data =
      int(ceil((robExtra+coredynp.pc_width + fmax(coredynp.phy_ireg_width,
      coredynp.phy_freg_width))/8.0));
                                              //When using CAM-based RAT, ROB
      needs to contain the ARF-PRF mapping to index the correct entry in the
      RAT,
                                                                                      //so that the correct architecture register (and freelist) can be found and the RAT can be appropriately updated.
                                      }
                              }
                              else
                              {
                                      //in RS based OOO, ROB also contains value
      of destination reg
      //				data  =
      int(ceil((robExtra+coredynp.pc_width
      +
      //						coredynp.instruction_length + 2*coredynp.phy_ireg_width
      + coredynp.fp_data_width)/8.0));

                                      //using phy_reg number to search in the
      RAT, the correct architecture register can be found and the RAT can be
      appropriately updated.
                                      //ROB phy_reg entry should use the larger
      one from ireg and freg; fdata_width is always larger; Latest Intel
      Processors may have different ROB/RS designs. data  = int(ceil((robExtra +
      coredynp.pc_width + fmax(coredynp.phy_ireg_width, coredynp.phy_freg_width)
      + coredynp.fp_data_width)/8.0));
                              }
      */

      interface_ip.is_cache = false;
      interface_ip.pure_cam = false;
      interface_ip.pure_ram = true;
      interface_ip.line_sz = data;
      interface_ip.cache_sz =
          data * XML->sys.core[ithCore]
                     .ROB_size; // The XML ROB size is for all threads
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
      interface_ip.num_rd_ports = coredynp.peak_commitW;
      interface_ip.num_wr_ports = coredynp.peak_issueW;
      interface_ip.num_se_rd_ports = 0;
      interface_ip.num_search_ports = 0;
      ROB.set_params(&interface_ip,
                     "ReorderBuffer",
                     Core_device,
                     coredynp.opt_local,
                     coredynp.core_ty);
    }
    instruction_selection.set_params(
        is_default,
        XML->sys.core[ithCore].instruction_window_size,
        coredynp.peak_issueW,
        &interface_ip,
        Core_device,
        coredynp.core_ty);
  }

  init_params = true;
}

void SchedulerU::computeStaticPower() {
  // NOTE: this does nothing, as the static power is optimized
  // along with the array area.
}

void SchedulerU::set_stats(const ParseXML *XML) {
  init_stats = true;
  if ((coredynp.core_ty == Inorder && coredynp.multithreaded)) {
    Iw_height = int_inst_window.local_result.cache_ht;
  }

  if (coredynp.core_ty == OOO) {
    Iw_height = int_inst_window.local_result.cache_ht;
    fp_Iw_height = fp_inst_window.local_result.cache_ht;
    if (XML->sys.core[ithCore].ROB_size > 0) {
      ROB_height = ROB.local_result.cache_ht;
    }
  }
}

void SchedulerU::computeArea() {

  if (!init_params) {
    std::cerr << "[ SchedulerU ] Error: must set params before calling "
                 "computeArea()\n";

    exit(1);
  }

  if ((coredynp.core_ty == Inorder && coredynp.multithreaded)) {
    int_inst_window.computeArea();
    int_inst_window.area.set_area(int_inst_window.area.get_area() +
                                  int_inst_window.local_result.area *
                                      coredynp.num_pipelines);
    area.set_area(area.get_area() +
                  int_inst_window.local_result.area * coredynp.num_pipelines);
  }

  if (coredynp.core_ty == OOO) {
    int_inst_window.computeArea();
    int_inst_window.area.set_area(int_inst_window.area.get_area() +
                                  int_inst_window.local_result.area *
                                      coredynp.num_pipelines);
    area.set_area(area.get_area() +
                  int_inst_window.local_result.area * coredynp.num_pipelines);

    fp_inst_window.computeArea();
    fp_inst_window.area.set_area(fp_inst_window.area.get_area() +
                                 fp_inst_window.local_result.area *
                                     coredynp.num_fp_pipelines);
    area.set_area(area.get_area() +
                  fp_inst_window.local_result.area * coredynp.num_fp_pipelines);

    if (XML->sys.core[ithCore].ROB_size > 0) {
      ROB.computeArea();
      ROB.area.set_area(ROB.area.get_area() +
                        ROB.local_result.area * coredynp.num_pipelines);
      area.set_area(area.get_area() +
                    ROB.local_result.area * coredynp.num_pipelines);
    }
  }
}

void SchedulerU::displayEnergy(uint32_t indent, int plevel, bool is_tdp) {
  if (!exist)
    return;
  string indent_str(indent, ' ');
  string indent_str_next(indent + 2, ' ');
  bool long_channel = XML->sys.longer_channel_device;
  bool power_gating = XML->sys.power_gating;

  if (is_tdp) {
    if (coredynp.core_ty == OOO) {
      cout << indent_str << "Instruction Window:" << endl;
      cout << indent_str_next
           << "Area = " << int_inst_window.area.get_area() * 1e-6 << " mm^2"
           << endl;
      cout << indent_str_next << "Peak Dynamic = "
           << int_inst_window.power.readOp.dynamic * clockRate << " W" << endl;
      cout << indent_str_next << "Subthreshold Leakage = "
           << (long_channel
                   ? int_inst_window.power.readOp.longer_channel_leakage
                   : int_inst_window.power.readOp.leakage)
           << " W" << endl;
      if (power_gating)
        cout << indent_str_next << "Subthreshold Leakage with power gating = "
             << (long_channel
                     ? int_inst_window.power.readOp
                           .power_gated_with_long_channel_leakage
                     : int_inst_window.power.readOp.power_gated_leakage)
             << " W" << endl;
      cout << indent_str_next
           << "Gate Leakage = " << int_inst_window.power.readOp.gate_leakage
           << " W" << endl;
      cout << indent_str_next << "Runtime Dynamic = "
           << int_inst_window.rt_power.readOp.dynamic / executionTime << " W"
           << endl;
      cout << endl;
      cout << indent_str << "FP Instruction Window:" << endl;
      cout << indent_str_next
           << "Area = " << fp_inst_window.area.get_area() * 1e-6 << " mm^2"
           << endl;
      cout << indent_str_next << "Peak Dynamic = "
           << fp_inst_window.power.readOp.dynamic * clockRate << " W" << endl;
      cout << indent_str_next << "Subthreshold Leakage = "
           << (long_channel ? fp_inst_window.power.readOp.longer_channel_leakage
                            : fp_inst_window.power.readOp.leakage)
           << " W" << endl;
      if (power_gating)
        cout << indent_str_next << "Subthreshold Leakage with power gating = "
             << (long_channel ? fp_inst_window.power.readOp
                                    .power_gated_with_long_channel_leakage
                              : fp_inst_window.power.readOp.power_gated_leakage)
             << " W" << endl;
      cout << indent_str_next
           << "Gate Leakage = " << fp_inst_window.power.readOp.gate_leakage
           << " W" << endl;
      cout << indent_str_next << "Runtime Dynamic = "
           << fp_inst_window.rt_power.readOp.dynamic / executionTime << " W"
           << endl;
      cout << endl;
      if (XML->sys.core[ithCore].ROB_size > 0) {
        cout << indent_str << "ROB:" << endl;
        cout << indent_str_next << "Area = " << ROB.area.get_area() * 1e-6
             << " mm^2" << endl;
        cout << indent_str_next
             << "Peak Dynamic = " << ROB.power.readOp.dynamic * clockRate
             << " W" << endl;
        cout << indent_str_next << "Subthreshold Leakage = "
             << (long_channel ? ROB.power.readOp.longer_channel_leakage
                              : ROB.power.readOp.leakage)
             << " W" << endl;
        if (power_gating)
          cout << indent_str_next << "Subthreshold Leakage with power gating = "
               << (long_channel
                       ? ROB.power.readOp.power_gated_with_long_channel_leakage
                       : ROB.power.readOp.power_gated_leakage)
               << " W" << endl;
        cout << indent_str_next
             << "Gate Leakage = " << ROB.power.readOp.gate_leakage << " W"
             << endl;
        cout << indent_str_next << "Runtime Dynamic = "
             << ROB.rt_power.readOp.dynamic / executionTime << " W" << endl;
        cout << endl;
      }
    } else if (coredynp.multithreaded) {
      cout << indent_str << "Instruction Window:" << endl;
      cout << indent_str_next
           << "Area = " << int_inst_window.area.get_area() * 1e-6 << " mm^2"
           << endl;
      cout << indent_str_next << "Peak Dynamic = "
           << int_inst_window.power.readOp.dynamic * clockRate << " W" << endl;
      cout << indent_str_next << "Subthreshold Leakage = "
           << (long_channel
                   ? int_inst_window.power.readOp.longer_channel_leakage
                   : int_inst_window.power.readOp.leakage)
           << " W" << endl;
      if (power_gating)
        cout << indent_str_next << "Subthreshold Leakage with power gating = "
             << (long_channel
                     ? int_inst_window.power.readOp
                           .power_gated_with_long_channel_leakage
                     : int_inst_window.power.readOp.power_gated_leakage)
             << " W" << endl;
      cout << indent_str_next
           << "Gate Leakage = " << int_inst_window.power.readOp.gate_leakage
           << " W" << endl;
      cout << indent_str_next << "Runtime Dynamic = "
           << int_inst_window.rt_power.readOp.dynamic / executionTime << " W"
           << endl;
      cout << endl;
    }
  } else {
    if (coredynp.core_ty == OOO) {
      cout << indent_str_next << "Instruction Window    Peak Dynamic = "
           << int_inst_window.rt_power.readOp.dynamic * clockRate << " W"
           << endl;
      cout << indent_str_next << "Instruction Window    Subthreshold Leakage = "
           << int_inst_window.rt_power.readOp.leakage << " W" << endl;
      cout << indent_str_next << "Instruction Window    Gate Leakage = "
           << int_inst_window.rt_power.readOp.gate_leakage << " W" << endl;
      cout << indent_str_next << "FP Instruction Window   Peak Dynamic = "
           << fp_inst_window.rt_power.readOp.dynamic * clockRate << " W"
           << endl;
      cout << indent_str_next
           << "FP Instruction Window   Subthreshold Leakage = "
           << fp_inst_window.rt_power.readOp.leakage << " W" << endl;
      cout << indent_str_next << "FP Instruction Window   Gate Leakage = "
           << fp_inst_window.rt_power.readOp.gate_leakage << " W" << endl;
      if (XML->sys.core[ithCore].ROB_size > 0) {
        cout << indent_str_next << "ROB   Peak Dynamic = "
             << ROB.rt_power.readOp.dynamic * clockRate << " W" << endl;
        cout << indent_str_next
             << "ROB   Subthreshold Leakage = " << ROB.rt_power.readOp.leakage
             << " W" << endl;
        cout << indent_str_next
             << "ROB   Gate Leakage = " << ROB.rt_power.readOp.gate_leakage
             << " W" << endl;
      }
    } else if (coredynp.multithreaded) {
      cout << indent_str_next << "Instruction Window    Peak Dynamic = "
           << int_inst_window.rt_power.readOp.dynamic * clockRate << " W"
           << endl;
      cout << indent_str_next << "Instruction Window    Subthreshold Leakage = "
           << int_inst_window.rt_power.readOp.leakage << " W" << endl;
      cout << indent_str_next << "Instruction Window    Gate Leakage = "
           << int_inst_window.rt_power.readOp.gate_leakage << " W" << endl;
    }
  }
}
void SchedulerU::computeDynamicPower(bool is_tdp) {
  if (!exist)
    return;
  if (!init_stats) {
    std::cerr << "[ SchedulerU ] Error: must set stats before calling "
                 "computeDynamicPower()\n";

    exit(1);
  }
  double ROB_duty_cycle;
  //	ROB_duty_cycle = ((coredynp.ALU_duty_cycle +
  // coredynp.num_muls>0?coredynp.MUL_duty_cycle:0
  //			+ coredynp.num_fpus>0?coredynp.FPU_duty_cycle:0))*1.1<1
  //? (coredynp.ALU_duty_cycle + coredynp.num_muls>0?coredynp.MUL_duty_cycle:0
  //					+
  // coredynp.num_fpus>0?coredynp.FPU_duty_cycle:0)*1.1:1;
  ROB_duty_cycle = 1;
  // init stats
  if (is_tdp) {
    if (coredynp.core_ty == OOO) {
      int_inst_window.stats_t.readAc.access =
          coredynp.issueW *
          coredynp.num_pipelines; // int_inst_window.l_ip.num_search_ports;
      int_inst_window.stats_t.writeAc.access =
          coredynp.issueW *
          coredynp.num_pipelines; // int_inst_window.l_ip.num_wr_ports;
      int_inst_window.stats_t.searchAc.access =
          coredynp.issueW * coredynp.num_pipelines;
      int_inst_window.tdp_stats = int_inst_window.stats_t;
      fp_inst_window.stats_t.readAc.access =
          fp_inst_window.l_ip.num_rd_ports * coredynp.num_fp_pipelines;
      fp_inst_window.stats_t.writeAc.access =
          fp_inst_window.l_ip.num_wr_ports * coredynp.num_fp_pipelines;
      fp_inst_window.stats_t.searchAc.access =
          fp_inst_window.l_ip.num_search_ports * coredynp.num_fp_pipelines;
      fp_inst_window.tdp_stats = fp_inst_window.stats_t;

      if (XML->sys.core[ithCore].ROB_size > 0) {
        ROB.stats_t.readAc.access =
            coredynp.commitW * coredynp.num_pipelines * ROB_duty_cycle;
        ROB.stats_t.writeAc.access =
            coredynp.issueW * coredynp.num_pipelines * ROB_duty_cycle;
        ROB.tdp_stats = ROB.stats_t;

        /*
         * When inst commits, ROB must be read.
         * Because for Physcial register based cores, physical register tag in
         * ROB need to be read out and write into RRAT/CAM based RAT. For RS
         * based cores, register content that stored in ROB must be read out and
         * stored in architectural registers.
         *
         * if no-register is involved, the ROB read out operation when
         * instruction commits can be ignored. assuming 20% insts. belong this
         * type.
         * TODO: ROB duty_cycle need to be revisited
         */
      }

    } else if (coredynp.multithreaded) {
      int_inst_window.stats_t.readAc.access =
          coredynp.issueW *
          coredynp.num_pipelines; // int_inst_window.l_ip.num_search_ports;
      int_inst_window.stats_t.writeAc.access =
          coredynp.issueW *
          coredynp.num_pipelines; // int_inst_window.l_ip.num_wr_ports;
      int_inst_window.stats_t.searchAc.access =
          coredynp.issueW * coredynp.num_pipelines;
      int_inst_window.tdp_stats = int_inst_window.stats_t;
    }

  } else { // rtp
    if (coredynp.core_ty == OOO) {
      int_inst_window.stats_t.readAc.access =
          XML->sys.core[ithCore].inst_window_reads;
      int_inst_window.stats_t.writeAc.access =
          XML->sys.core[ithCore].inst_window_writes;
      int_inst_window.stats_t.searchAc.access =
          XML->sys.core[ithCore].inst_window_wakeup_accesses;
      int_inst_window.rtp_stats = int_inst_window.stats_t;
      fp_inst_window.stats_t.readAc.access =
          XML->sys.core[ithCore].fp_inst_window_reads;
      fp_inst_window.stats_t.writeAc.access =
          XML->sys.core[ithCore].fp_inst_window_writes;
      fp_inst_window.stats_t.searchAc.access =
          XML->sys.core[ithCore].fp_inst_window_wakeup_accesses;
      fp_inst_window.rtp_stats = fp_inst_window.stats_t;

      if (XML->sys.core[ithCore].ROB_size > 0) {

        ROB.stats_t.readAc.access = XML->sys.core[ithCore].ROB_reads;
        ROB.stats_t.writeAc.access = XML->sys.core[ithCore].ROB_writes;
        /* ROB need to be updated in RS based OOO when new values are produced,
         * this update may happen before the commit stage when ROB entry is
         * released
         * 1. ROB write at instruction inserted in
         * 2. ROB write as results produced (for RS based OOO only)
         * 3. ROB read  as instruction committed. For RS based OOO, data values
         * are read out and sent to ARF For Physical reg based OOO, no data
         * stored in ROB, but register tags need to be read out and used to set
         * the RRAT and to recycle the register tag to free list buffer
         */
        ROB.rtp_stats = ROB.stats_t;
      }

    } else if (coredynp.multithreaded) {
      int_inst_window.stats_t.readAc.access =
          XML->sys.core[ithCore].int_instructions +
          XML->sys.core[ithCore].fp_instructions;
      int_inst_window.stats_t.writeAc.access =
          XML->sys.core[ithCore].int_instructions +
          XML->sys.core[ithCore].fp_instructions;
      int_inst_window.stats_t.searchAc.access =
          2 * (XML->sys.core[ithCore].int_instructions +
               XML->sys.core[ithCore].fp_instructions);
      int_inst_window.rtp_stats = int_inst_window.stats_t;
    }
  }

  // computation engine
  if (coredynp.core_ty == OOO) {
    int_inst_window.power_t.reset();
    fp_inst_window.power_t.reset();

    /* each instruction needs to write to scheduler, read out when all resources
     * and source operands are ready two search ops with one for each source
     * operand
     *
     */
    int_inst_window.power_t.readOp.dynamic +=
        int_inst_window.local_result.power.readOp.dynamic *
            int_inst_window.stats_t.readAc.access +
        int_inst_window.local_result.power.searchOp.dynamic *
            int_inst_window.stats_t.searchAc.access +
        int_inst_window.local_result.power.writeOp.dynamic *
            int_inst_window.stats_t.writeAc.access +
        int_inst_window.stats_t.readAc.access *
            instruction_selection.power.readOp.dynamic;

    fp_inst_window.power_t.readOp.dynamic +=
        fp_inst_window.local_result.power.readOp.dynamic *
            fp_inst_window.stats_t.readAc.access +
        fp_inst_window.local_result.power.searchOp.dynamic *
            fp_inst_window.stats_t.searchAc.access +
        fp_inst_window.local_result.power.writeOp.dynamic *
            fp_inst_window.stats_t.writeAc.access +
        fp_inst_window.stats_t.writeAc.access *
            instruction_selection.power.readOp.dynamic;

    if (XML->sys.core[ithCore].ROB_size > 0) {
      ROB.power_t.reset();
      ROB.power_t.readOp.dynamic +=
          ROB.local_result.power.readOp.dynamic * ROB.stats_t.readAc.access +
          ROB.stats_t.writeAc.access * ROB.local_result.power.writeOp.dynamic;
    }

  } else if (coredynp.multithreaded) {
    int_inst_window.power_t.reset();
    int_inst_window.power_t.readOp.dynamic +=
        int_inst_window.local_result.power.readOp.dynamic *
            int_inst_window.stats_t.readAc.access +
        int_inst_window.local_result.power.searchOp.dynamic *
            int_inst_window.stats_t.searchAc.access +
        int_inst_window.local_result.power.writeOp.dynamic *
            int_inst_window.stats_t.writeAc.access +
        int_inst_window.stats_t.writeAc.access *
            instruction_selection.power.readOp.dynamic;
  }

  // assign values
  if (is_tdp) {
    if (coredynp.core_ty == OOO) {
      int_inst_window.power =
          int_inst_window.power_t +
          (int_inst_window.local_result.power + instruction_selection.power) *
              pppm_lkg;
      fp_inst_window.power =
          fp_inst_window.power_t +
          (fp_inst_window.local_result.power + instruction_selection.power) *
              pppm_lkg;
      power = power + int_inst_window.power + fp_inst_window.power;
      if (XML->sys.core[ithCore].ROB_size > 0) {
        ROB.power = ROB.power_t + ROB.local_result.power * pppm_lkg;
        power = power + ROB.power;
      }

    } else if (coredynp.multithreaded) {
      //			set_pppm(pppm_t,
      // XML->sys.core[ithCore].issue_width,1, 1, 1);
      int_inst_window.power =
          int_inst_window.power_t +
          (int_inst_window.local_result.power + instruction_selection.power) *
              pppm_lkg;
      power = power + int_inst_window.power;
    }

  } else { // rtp
    if (coredynp.core_ty == OOO) {
      int_inst_window.rt_power =
          int_inst_window.power_t +
          (int_inst_window.local_result.power + instruction_selection.power) *
              pppm_lkg;
      fp_inst_window.rt_power =
          fp_inst_window.power_t +
          (fp_inst_window.local_result.power + instruction_selection.power) *
              pppm_lkg;
      rt_power = rt_power + int_inst_window.rt_power + fp_inst_window.rt_power;
      if (XML->sys.core[ithCore].ROB_size > 0) {
        ROB.rt_power = ROB.power_t + ROB.local_result.power * pppm_lkg;
        rt_power = rt_power + ROB.rt_power;
      }

    } else if (coredynp.multithreaded) {
      //			set_pppm(pppm_t,
      // XML->sys.core[ithCore].issue_width,1, 1, 1);
      int_inst_window.rt_power =
          int_inst_window.power_t +
          (int_inst_window.local_result.power + instruction_selection.power) *
              pppm_lkg;
      rt_power = rt_power + int_inst_window.rt_power;
    }
  }
  //	set_pppm(pppm_t, XML->sys.core[ithCore].issue_width,1, 1, 1);
  //	cout<<"Scheduler
  // power="<<power.readOp.dynamic<<"leakage="<<power.readOp.leakage<<endl;
  //	cout<<"IW="<<int_inst_window.local_result.power.searchOp.dynamic *
  // int_inst_window.stats_t.readAc.access +
  //    + int_inst_window.local_result.power.writeOp.dynamic *
  //    int_inst_window.stats_t.writeAc.access<<"leakage="<<int_inst_window.local_result.power.readOp.leakage<<endl;
  //	cout<<"selection"<<instruction_selection.power.readOp.dynamic<<"leakage"<<instruction_selection.power.readOp.leakage<<endl;
}

SchedulerU ::~SchedulerU() {}
