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

#include "pipeline.h"

#include "dff_cell.h"

Pipeline::Pipeline(const InputParameter *configure_interface,
                   const CoreDynParam &dyn_p_,
                   enum Device_ty device_ty_,
                   bool _is_core_pipeline,
                   bool _is_default)
    : l_ip(*configure_interface), coredynp(dyn_p_), device_ty(device_ty_),
      is_core_pipeline(_is_core_pipeline), is_default(_is_default),
      num_piperegs(0.0)

{
  local_result = init_interface(&l_ip);
  if (!coredynp.Embedded)
    process_ind = true;
  else
    process_ind = false;
  WNANDn =
      (process_ind)
          ? 25 * l_ip.F_sz_um
          : g_tp.min_w_nmos_; // this was  20 micron for the 0.8 micron process
  WNANDp = (process_ind)
               ? 37.5 * l_ip.F_sz_um
               : g_tp.min_w_nmos_ *
                     pmos_to_nmos_sz_ratio(); // this was  30 micron for the 0.8
                                              // micron process
  load_per_pipeline_stage = 2 * gate_C(WNANDn + WNANDp, 0, false);
  compute();
}

void Pipeline::compute() {
  compute_stage_vector();
  DFFCell pipe_reg(false, WNANDn, WNANDp, load_per_pipeline_stage, &l_ip);
  pipe_reg.compute_DFF_cell();

  double clock_power_pipereg = num_piperegs * pipe_reg.e_clock.readOp.dynamic;
  //******************pipeline power: currently, we average all the
  // possibilities of the states of DFFs in the pipeline. A better way to do it
  // is to consider the harming distance of two consecutive signals, However
  // McPAT does not have plan to do this in near future as it focuses on worst
  // case power.
  double pipe_reg_power =
      num_piperegs *
          (pipe_reg.e_switch.readOp.dynamic + pipe_reg.e_keep_0.readOp.dynamic +
           pipe_reg.e_keep_1.readOp.dynamic) /
          3 +
      clock_power_pipereg;
  double pipe_reg_leakage = num_piperegs * pipe_reg.e_switch.readOp.leakage;
  double pipe_reg_gate_leakage =
      num_piperegs * pipe_reg.e_switch.readOp.gate_leakage;
  power.readOp.dynamic += pipe_reg_power;
  power.readOp.leakage += pipe_reg_leakage;
  power.readOp.gate_leakage += pipe_reg_gate_leakage;
  area.set_area(num_piperegs * pipe_reg.area.get_area());

  double long_channel_device_reduction =
      longer_channel_device_reduction(device_ty, coredynp.core_ty);
  power.readOp.longer_channel_leakage =
      power.readOp.leakage * long_channel_device_reduction;

  double pg_reduction = power_gating_leakage_reduction(false);
  power.readOp.power_gated_leakage = power.readOp.leakage * pg_reduction;
  power.readOp.power_gated_with_long_channel_leakage =
      power.readOp.power_gated_leakage * long_channel_device_reduction;

  double sckRation = g_tp.sckt_co_eff;
  power.readOp.dynamic *= sckRation;
  power.writeOp.dynamic *= sckRation;
  power.searchOp.dynamic *= sckRation;
  double macro_layout_overhead = g_tp.macro_layout_overhead;
  if (!coredynp.Embedded)
    area.set_area(area.get_area() * macro_layout_overhead);
}

void Pipeline::compute_stage_vector() {
  double num_stages, tot_stage_vector, per_stage_vector;
  int opcode_length =
      coredynp.x86 ? coredynp.micro_opcode_length : coredynp.opcode_length;
  // Hthread = thread_clock_gated? 1:num_thread;

  if (!is_core_pipeline) {
    num_piperegs = l_ip.pipeline_stages *
                   l_ip.per_stage_vector; // The number of pipeline stages are
                                          // calculated based on the achievable
                                          // throughput and required throughput
  } else {
    if (coredynp.core_ty == Inorder) {
      /* assume 6 pipe stages and try to estimate bits per pipe stage */
      /* pipe stage 0/IF */
      num_piperegs += coredynp.pc_width * 2 * coredynp.num_hthreads;
      /* pipe stage IF/ID */
      num_piperegs += coredynp.fetchW *
                      (coredynp.instruction_length + coredynp.pc_width) *
                      coredynp.num_hthreads;
      /* pipe stage IF/ThreadSEL */
      if (coredynp.multithreaded)
        num_piperegs += coredynp.num_hthreads *
                        coredynp.perThreadState; // 8 bit thread states
      /* pipe stage ID/EXE */
      num_piperegs += coredynp.decodeW *
                      (coredynp.instruction_length + coredynp.pc_width +
                       pow(2.0, opcode_length) + 2 * coredynp.int_data_width) *
                      coredynp.num_hthreads;
      /* pipe stage EXE/MEM */
      num_piperegs +=
          coredynp.issueW *
          (3 * coredynp.arch_ireg_width + pow(2.0, opcode_length) +
           8 * 2 * coredynp.int_data_width /*+2*powers (2,reg_length)*/);
      /* pipe stage MEM/WB the 2^opcode_length means the total decoded signal
       * for the opcode*/
      num_piperegs +=
          coredynp.issueW *
          (2 * coredynp.int_data_width + pow(2.0, opcode_length) +
           8 * 2 * coredynp.int_data_width /*+2*powers (2,reg_length)*/);
      //		/* pipe stage 5/6 */
      //		num_piperegs += issueWidth*(data_width + powers
      //(2,opcode_length)/*+2*powers (2,reg_length)*/);
      //		/* pipe stage 6/7 */
      //		num_piperegs += issueWidth*(data_width + powers
      //(2,opcode_length)/*+2*powers (2,reg_length)*/);
      //		/* pipe stage 7/8 */
      //		num_piperegs += issueWidth*(data_width + powers
      //(2,opcode_length)/**2*powers (2,reg_length)*/);
      //		/* assume 50% extra in control signals (rule of thumb)
      //*/
      num_stages = 6;

    } else {
      /* assume 12 stage pipe stages and try to estimate bits per pipe stage */
      /*OOO: Fetch, decode, rename, IssueQ, dispatch, regread, EXE, MEM, WB, CM
       */

      /* pipe stage 0/1F*/
      num_piperegs +=
          coredynp.pc_width * 2 * coredynp.num_hthreads; // PC and Next PC
      /* pipe stage IF/ID */
      num_piperegs +=
          coredynp.fetchW * (coredynp.instruction_length + coredynp.pc_width) *
          coredynp.num_hthreads; // PC is used to feed branch predictor in ID
      /* pipe stage 1D/Renaming*/
      num_piperegs +=
          coredynp.decodeW * (coredynp.instruction_length + coredynp.pc_width) *
          coredynp.num_hthreads; // PC is for branch exe in later stage.
      /* pipe stage Renaming/wire_drive */
      num_piperegs +=
          coredynp.decodeW * (coredynp.instruction_length + coredynp.pc_width);
      /* pipe stage Renaming/IssueQ */
      num_piperegs += coredynp.issueW *
                      (coredynp.instruction_length + coredynp.pc_width +
                       3 * coredynp.phy_ireg_width) *
                      coredynp.num_hthreads; // 3*coredynp.phy_ireg_width means
                                             // 2 sources and 1 dest
      /* pipe stage IssueQ/Dispatch */
      num_piperegs += coredynp.issueW * (coredynp.instruction_length +
                                         3 * coredynp.phy_ireg_width);
      /* pipe stage Dispatch/EXE */

      num_piperegs += coredynp.issueW *
                      (3 * coredynp.phy_ireg_width + coredynp.pc_width +
                       pow(2.0, opcode_length) /*+2*powers (2,reg_length)*/);
      /* 2^opcode_length means the total decoded signal for the opcode*/
      num_piperegs += coredynp.issueW *
                      (2 * coredynp.int_data_width +
                       pow(2.0, opcode_length) /*+2*powers (2,reg_length)*/);
      /*2 source operands in EXE; Assume 2EXE stages* since we do not really
       * distinguish OP*/
      num_piperegs += coredynp.issueW *
                      (2 * coredynp.int_data_width +
                       pow(2.0, opcode_length) /*+2*powers (2,reg_length)*/);
      /* pipe stage EXE/MEM, data need to be read/write, address*/
      num_piperegs +=
          coredynp.issueW *
          (coredynp.int_data_width + coredynp.v_address_width +
           pow(2.0,
               opcode_length) /*+2*powers (2,reg_length)*/); // memory Opcode
                                                             // still need to be
                                                             // passed
      /* pipe stage MEM/WB; result data, writeback regs */
      num_piperegs +=
          coredynp.issueW * (coredynp.int_data_width + coredynp.phy_ireg_width /* powers (2,opcode_length) + (2,opcode_length)+2*powers (2,reg_length)*/);
      /* pipe stage WB/CM ; result data, regs need to be updated, address for
       * resolve memory ops in ROB's top*/
      num_piperegs +=
          coredynp.commitW *
          (coredynp.int_data_width + coredynp.v_address_width + coredynp.phy_ireg_width /*+ powers (2,opcode_length)*2*powers (2,reg_length)*/) *
          coredynp.num_hthreads;
      //		if (multithreaded)
      //		{
      //
      //		}
      num_stages = 12;
    }

    /* assume 50% extra in control registers and interrupt registers (rule of
     * thumb) */
    num_piperegs = num_piperegs * 1.5;
    tot_stage_vector = num_piperegs;
    per_stage_vector = tot_stage_vector / num_stages;

    if (coredynp.core_ty == Inorder) {
      if (coredynp.pipeline_stages > 6)
        num_piperegs = per_stage_vector * coredynp.pipeline_stages;
    } else // OOO
    {
      if (coredynp.pipeline_stages > 12)
        num_piperegs = per_stage_vector * coredynp.pipeline_stages;
    }
  }
}
