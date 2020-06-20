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

#include "functional_unit.h"

FunctionalUnit::FunctionalUnit(const ParseXML *XML,
                               int ithCore_,
                               InputParameter *interface_ip_,
                               const CoreDynParam &dyn_p_,
                               enum FU_type fu_type_)
    : ithCore(ithCore_), interface_ip(*interface_ip_),
      coredynp(dyn_p_), fu_type(fu_type_) {

  long_channel = false;
  power_gating = false;
  embedded = false;
  mul_accesses = 0;
  ialu_accesses = 0;
  fpu_accesses = 0;

  long_channel = XML->sys.longer_channel_device;
  power_gating = XML->sys.power_gating;
  embedded = XML->sys.Embedded;

  double area_t; //, leakage, gate_leakage;
  double pmos_to_nmos_sizing_r = pmos_to_nmos_sz_ratio();
  clockRate = coredynp.clockRate;
  executionTime = coredynp.executionTime;

  // XML_interface=_XML_interface;
  uca_org_t result2;
  result2 = init_interface(&interface_ip);
  if (embedded) {
    if (fu_type == FPU) {
      num_fu = coredynp.num_fpus;
      // area_t = 8.47*1e6*g_tp.scaling_factor.logic_scaling_co_eff;//this is
      // um^2
      area_t = 4.47 * 1e6 *
               (g_ip->F_sz_nm * g_ip->F_sz_nm / 90.0 /
                90.0); // this is um^2 The base number
      // 4.47 contains both VFP and NEON processing unit, VFP is about 40% and
      // NEON is about 60%
      if (g_ip->F_sz_nm > 90)
        area_t = 4.47 * 1e6 *
                 g_tp.scaling_factor.logic_scaling_co_eff; // this is um^2
      leakage = area_t * (g_tp.scaling_factor.core_tx_density) *
                cmos_Isub_leakage(5 * g_tp.min_w_nmos_,
                                  5 * g_tp.min_w_nmos_ * pmos_to_nmos_sizing_r,
                                  1,
                                  inv) *
                g_tp.peri_global.Vdd / 2; // unit W
      gate_leakage =
          area_t * (g_tp.scaling_factor.core_tx_density) *
          cmos_Ig_leakage(5 * g_tp.min_w_nmos_,
                          5 * g_tp.min_w_nmos_ * pmos_to_nmos_sizing_r,
                          1,
                          inv) *
          g_tp.peri_global.Vdd / 2; // unit W
      // energy = 0.3529/10*1e-9;//this is the energy(nJ) for a FP instruction
      // in FPU usually it can have up to 20 cycles.
      //			base_energy = coredynp.core_ty==Inorder? 0:
      // 89e-3*3; //W The base energy of ALU average numbers from Intel 4G and
      // 773Mhz (Wattch) 			base_energy
      //*=(g_tp.peri_global.Vdd*g_tp.peri_global.Vdd/1.2/1.2);
      base_energy = 0;
      per_access_energy =
          1.15 / 1e9 / 4 / 1.3 / 1.3 * g_tp.peri_global.Vdd *
          g_tp.peri_global.Vdd *
          (g_ip->F_sz_nm /
           90.0); // g_tp.peri_global.Vdd*g_tp.peri_global.Vdd/1.2/1.2);//0.00649*1e-9;
                  // //This is per Hz energy(nJ)
      // FPU power from Sandia's processor sizing tech report
      FU_height = (18667 * num_fu) * interface_ip.F_sz_um; // FPU from Sun's
                                                           // data
    } else if (fu_type == ALU) {
      num_fu = coredynp.num_alus;
      area_t =
          280 * 260 *
          g_tp.scaling_factor.logic_scaling_co_eff; // this is um^2 ALU + MUl
      leakage = area_t * (g_tp.scaling_factor.core_tx_density) *
                cmos_Isub_leakage(20 * g_tp.min_w_nmos_,
                                  20 * g_tp.min_w_nmos_ * pmos_to_nmos_sizing_r,
                                  1,
                                  inv) *
                g_tp.peri_global.Vdd / 2; // unit W
      gate_leakage =
          area_t * (g_tp.scaling_factor.core_tx_density) *
          cmos_Ig_leakage(20 * g_tp.min_w_nmos_,
                          20 * g_tp.min_w_nmos_ * pmos_to_nmos_sizing_r,
                          1,
                          inv) *
          g_tp.peri_global.Vdd / 2;
      //			base_energy = coredynp.core_ty==Inorder?
      // 0:89e-3; //W The base energy of ALU average numbers from Intel 4G and
      // 773Mhz (Wattch) 			base_energy
      //*=(g_tp.peri_global.Vdd*g_tp.peri_global.Vdd/1.2/1.2);
      base_energy = 0;
      per_access_energy =
          1.15 / 3 / 1e9 / 4 / 1.3 / 1.3 * g_tp.peri_global.Vdd *
          g_tp.peri_global.Vdd *
          (g_ip->F_sz_nm /
           90.0); //(g_tp.peri_global.Vdd*g_tp.peri_global.Vdd/1.2/1.2);//0.00649*1e-9;
                  ////This is per cycle energy(nJ)
      FU_height = (6222 * num_fu) * interface_ip.F_sz_um; // integer ALU

    } else if (fu_type == MUL) {
      num_fu = coredynp.num_muls;
      area_t =
          280 * 260 * 3 *
          g_tp.scaling_factor.logic_scaling_co_eff; // this is um^2 ALU + MUl
      leakage = area_t * (g_tp.scaling_factor.core_tx_density) *
                cmos_Isub_leakage(20 * g_tp.min_w_nmos_,
                                  20 * g_tp.min_w_nmos_ * pmos_to_nmos_sizing_r,
                                  1,
                                  inv) *
                g_tp.peri_global.Vdd / 2; // unit W
      gate_leakage =
          area_t * (g_tp.scaling_factor.core_tx_density) *
          cmos_Ig_leakage(20 * g_tp.min_w_nmos_,
                          20 * g_tp.min_w_nmos_ * pmos_to_nmos_sizing_r,
                          1,
                          inv) *
          g_tp.peri_global.Vdd / 2;
      //			base_energy = coredynp.core_ty==Inorder?
      // 0:89e-3*2; //W The base energy of ALU average numbers from Intel 4G and
      // 773Mhz (Wattch) 			base_energy
      //*=(g_tp.peri_global.Vdd*g_tp.peri_global.Vdd/1.2/1.2);
      base_energy = 0;
      per_access_energy =
          1.15 * 2 / 3 / 1e9 / 4 / 1.3 / 1.3 * g_tp.peri_global.Vdd *
          g_tp.peri_global.Vdd *
          (g_ip->F_sz_nm /
           90.0); //(g_tp.peri_global.Vdd*g_tp.peri_global.Vdd/1.2/1.2);//0.00649*1e-9;
                  ////This is per cycle energy(nJ), coefficient based on Wattch
      FU_height =
          (9334 * num_fu) * interface_ip.F_sz_um; // divider/mul from Sun's data
    } else {
      std::cout << "Unknown Functional Unit Type" << std::endl;
      exit(0);
    }
    per_access_energy *= 0.5; // According to ARM data embedded processor has
                              // much lower per acc energy
  } else {
    if (fu_type == FPU) {
      num_fu = coredynp.num_fpus;
      // area_t = 8.47*1e6*g_tp.scaling_factor.logic_scaling_co_eff;//this is
      // um^2
      area_t = 8.47 * 1e6 *
               (g_ip->F_sz_nm * g_ip->F_sz_nm / 90.0 / 90.0); // this is um^2
      if (g_ip->F_sz_nm > 90)
        area_t = 8.47 * 1e6 *
                 g_tp.scaling_factor.logic_scaling_co_eff; // this is um^2
      leakage = area_t * (g_tp.scaling_factor.core_tx_density) *
                cmos_Isub_leakage(5 * g_tp.min_w_nmos_,
                                  5 * g_tp.min_w_nmos_ * pmos_to_nmos_sizing_r,
                                  1,
                                  inv) *
                g_tp.peri_global.Vdd / 2; // unit W
      gate_leakage =
          area_t * (g_tp.scaling_factor.core_tx_density) *
          cmos_Ig_leakage(5 * g_tp.min_w_nmos_,
                          5 * g_tp.min_w_nmos_ * pmos_to_nmos_sizing_r,
                          1,
                          inv) *
          g_tp.peri_global.Vdd / 2; // unit W
      // energy = 0.3529/10*1e-9;//this is the energy(nJ) for a FP instruction
      // in FPU usually it can have up to 20 cycles.
      base_energy = coredynp.core_ty == Inorder
                        ? 0
                        : 89e-3 * 3; // W The base energy of ALU average numbers
                                     // from Intel 4G and 773Mhz (Wattch)
      base_energy *= (g_tp.peri_global.Vdd * g_tp.peri_global.Vdd / 1.2 / 1.2);
      per_access_energy =
          1.15 * 3 / 1e9 / 4 / 1.3 / 1.3 * g_tp.peri_global.Vdd *
          g_tp.peri_global.Vdd *
          (g_ip->F_sz_nm /
           90.0); // g_tp.peri_global.Vdd*g_tp.peri_global.Vdd/1.2/1.2);//0.00649*1e-9;
                  // //This is per op energy(nJ)
      FU_height = (38667 * num_fu) * interface_ip.F_sz_um; // FPU from Sun's
                                                           // data
    } else if (fu_type == ALU) {
      num_fu = coredynp.num_alus;
      area_t =
          280 * 260 * 2 *
          g_tp.scaling_factor.logic_scaling_co_eff; // this is um^2 ALU + MUl
      leakage = area_t * (g_tp.scaling_factor.core_tx_density) *
                cmos_Isub_leakage(20 * g_tp.min_w_nmos_,
                                  20 * g_tp.min_w_nmos_ * pmos_to_nmos_sizing_r,
                                  1,
                                  inv) *
                g_tp.peri_global.Vdd / 2; // unit W
      gate_leakage =
          area_t * (g_tp.scaling_factor.core_tx_density) *
          cmos_Ig_leakage(20 * g_tp.min_w_nmos_,
                          20 * g_tp.min_w_nmos_ * pmos_to_nmos_sizing_r,
                          1,
                          inv) *
          g_tp.peri_global.Vdd / 2;
      base_energy = coredynp.core_ty == Inorder
                        ? 0
                        : 89e-3; // W The base energy of ALU average numbers
                                 // from Intel 4G and 773Mhz (Wattch)
      base_energy *= (g_tp.peri_global.Vdd * g_tp.peri_global.Vdd / 1.2 / 1.2);
      per_access_energy =
          1.15 / 1e9 / 4 / 1.3 / 1.3 * g_tp.peri_global.Vdd *
          g_tp.peri_global.Vdd *
          (g_ip->F_sz_nm /
           90.0); //(g_tp.peri_global.Vdd*g_tp.peri_global.Vdd/1.2/1.2);//0.00649*1e-9;
                  ////This is per cycle energy(nJ)
      FU_height = (6222 * num_fu) * interface_ip.F_sz_um; // integer ALU

    } else if (fu_type == MUL) {
      num_fu = coredynp.num_muls;
      area_t =
          280 * 260 * 2 * 3 *
          g_tp.scaling_factor.logic_scaling_co_eff; // this is um^2 ALU + MUl
      leakage = area_t * (g_tp.scaling_factor.core_tx_density) *
                cmos_Isub_leakage(20 * g_tp.min_w_nmos_,
                                  20 * g_tp.min_w_nmos_ * pmos_to_nmos_sizing_r,
                                  1,
                                  inv) *
                g_tp.peri_global.Vdd / 2; // unit W
      gate_leakage =
          area_t * (g_tp.scaling_factor.core_tx_density) *
          cmos_Ig_leakage(20 * g_tp.min_w_nmos_,
                          20 * g_tp.min_w_nmos_ * pmos_to_nmos_sizing_r,
                          1,
                          inv) *
          g_tp.peri_global.Vdd / 2;
      base_energy = coredynp.core_ty == Inorder
                        ? 0
                        : 89e-3 * 2; // W The base energy of ALU average numbers
                                     // from Intel 4G and 773Mhz (Wattch)
      base_energy *= (g_tp.peri_global.Vdd * g_tp.peri_global.Vdd / 1.2 / 1.2);
      per_access_energy =
          1.15 * 2 / 1e9 / 4 / 1.3 / 1.3 * g_tp.peri_global.Vdd *
          g_tp.peri_global.Vdd *
          (g_ip->F_sz_nm /
           90.0); //(g_tp.peri_global.Vdd*g_tp.peri_global.Vdd/1.2/1.2);//0.00649*1e-9;
                  ////This is per cycle energy(nJ), coefficient based on Wattch
      FU_height =
          (9334 * num_fu) * interface_ip.F_sz_um; // divider/mul from Sun's data
    } else {
      std::cout << "Unknown Functional Unit Type" << std::endl;
      exit(0);
    }
  }
  set_stats(XML);


  // IEXEU, simple ALU and FPU
  //  double C_ALU, C_EXEU, C_FPU; //Lum Equivalent capacitance of IEXEU and
  //  FPU. Based on Intel and Sun 90nm process fabracation.
  //
  //  C_ALU	  = 0.025e-9;//F
  //  C_EXEU  = 0.05e-9; //F
  //  C_FPU	  = 0.35e-9;//F
  area.set_area(area_t * num_fu);
  leakage *= num_fu;
  gate_leakage *= num_fu;
  double macro_layout_overhead = g_tp.macro_layout_overhead;
  area.set_area(area.get_area() * macro_layout_overhead);
}

void FunctionalUnit::set_stats(const ParseXML* XML) {
  mul_accesses = XML->sys.core[ithCore].mul_accesses;
  ialu_accesses = XML->sys.core[ithCore].ialu_accesses;
  fpu_accesses = XML->sys.core[ithCore].fpu_accesses;
}

void FunctionalUnit::computeEnergy(bool is_tdp) {
  double pppm_t[4] = {1, 1, 1, 1};
  double FU_duty_cycle = 0.0;
  if (is_tdp) {

    set_pppm(pppm_t, 2, 2, 2, 2); // 2 means two source operands needs to be
                                  // passed for each int instruction.
    if (fu_type == FPU) {
      stats_t.readAc.access = num_fu;
      tdp_stats = stats_t;
      FU_duty_cycle = coredynp.FPU_duty_cycle;
    } else if (fu_type == ALU) {
      stats_t.readAc.access = 1 * num_fu;
      tdp_stats = stats_t;
      FU_duty_cycle = coredynp.ALU_duty_cycle;
    } else if (fu_type == MUL) {
      stats_t.readAc.access = num_fu;
      tdp_stats = stats_t;
      FU_duty_cycle = coredynp.MUL_duty_cycle;
    }

    // power.readOp.dynamic = base_energy/clockRate +
    // energy*stats_t.readAc.access;
    power.readOp.dynamic =
        per_access_energy * stats_t.readAc.access + base_energy / clockRate;
    double sckRation = g_tp.sckt_co_eff;
    power.readOp.dynamic *= sckRation * FU_duty_cycle;
    power.writeOp.dynamic *= sckRation;
    power.searchOp.dynamic *= sckRation;

    power.readOp.leakage = leakage;
    power.readOp.gate_leakage = gate_leakage;
    double long_channel_device_reduction =
        longer_channel_device_reduction(Core_device, coredynp.core_ty);
    power.readOp.longer_channel_leakage =
        power.readOp.leakage * long_channel_device_reduction;

    double pg_reduction = power_gating_leakage_reduction(false);
    power.readOp.power_gated_leakage = power.readOp.leakage * pg_reduction;
    power.readOp.power_gated_with_long_channel_leakage =
        power.readOp.power_gated_leakage * long_channel_device_reduction;

  } else {
    if (fu_type == FPU) {
      stats_t.readAc.access = fpu_accesses;
      rtp_stats = stats_t;
    } else if (fu_type == ALU) {
      stats_t.readAc.access = ialu_accesses;
      rtp_stats = stats_t;
    } else if (fu_type == MUL) {
      stats_t.readAc.access = mul_accesses;
      rtp_stats = stats_t;
    }

    // rt_power.readOp.dynamic = base_energy*executionTime +
    // energy*stats_t.readAc.access;
    rt_power.readOp.dynamic =
        per_access_energy * stats_t.readAc.access + base_energy * executionTime;
    double sckRation = g_tp.sckt_co_eff;
    rt_power.readOp.dynamic *= sckRation;
    rt_power.writeOp.dynamic *= sckRation;
    rt_power.searchOp.dynamic *= sckRation;
  }
}

void FunctionalUnit::displayEnergy(uint32_t indent, int plevel, bool is_tdp) {
  string indent_str(indent, ' ');
  string indent_str_next(indent + 2, ' ');

  //	std::cout << indent_str_next << "Results Broadcast Bus Area = " <<
  // bypass->area.get_area() *1e-6 << " mm^2" << std::endl;
  if (is_tdp) {
    if (fu_type == FPU) {
      std::cout << indent_str
           << "Floating Point Units (FPUs) (Count: " << coredynp.num_fpus
           << " ):" << std::endl;
      std::cout << indent_str_next << "Area = " << area.get_area() * 1e-6 << " mm^2"
           << std::endl;
      std::cout << indent_str_next
           << "Peak Dynamic = " << power.readOp.dynamic * clockRate << " W"
           << std::endl;
      //			std::cout << indent_str_next << "Subthreshold Leakage
      //= " << power.readOp.leakage  << " W" << std::endl;
      std::cout << indent_str_next << "Subthreshold Leakage = "
           << (long_channel ? power.readOp.longer_channel_leakage
                            : power.readOp.leakage)
           << " W" << std::endl;
      if (power_gating) {
        std::cout << indent_str_next << "Subthreshold Leakage with power gating = "
             << (long_channel
                     ? power.readOp.power_gated_with_long_channel_leakage
                     : power.readOp.power_gated_leakage)
             << " W" << std::endl;
      }
      std::cout << indent_str_next << "Gate Leakage = " << power.readOp.gate_leakage
           << " W" << std::endl;
      std::cout << indent_str_next
           << "Runtime Dynamic = " << rt_power.readOp.dynamic / executionTime
           << " W" << std::endl;
      std::cout << std::endl;
    } else if (fu_type == ALU) {
      std::cout << indent_str << "Integer ALUs (Count: " << coredynp.num_alus
           << " ):" << std::endl;
      std::cout << indent_str_next << "Area = " << area.get_area() * 1e-6 << " mm^2"
           << std::endl;
      std::cout << indent_str_next
           << "Peak Dynamic = " << power.readOp.dynamic * clockRate << " W"
           << std::endl;
      //			std::cout << indent_str_next << "Subthreshold Leakage
      //= " << power.readOp.leakage  << " W" << std::endl;
      std::cout << indent_str_next << "Subthreshold Leakage = "
           << (long_channel ? power.readOp.longer_channel_leakage
                            : power.readOp.leakage)
           << " W" << std::endl;
      if (power_gating) {
        std::cout << indent_str_next << "Subthreshold Leakage with power gating = "
             << (long_channel
                     ? power.readOp.power_gated_with_long_channel_leakage
                     : power.readOp.power_gated_leakage)
             << " W" << std::endl;
      }
      std::cout << indent_str_next << "Gate Leakage = " << power.readOp.gate_leakage
           << " W" << std::endl;
      std::cout << indent_str_next
           << "Runtime Dynamic = " << rt_power.readOp.dynamic / executionTime
           << " W" << std::endl;
      std::cout << std::endl;
    } else if (fu_type == MUL) {
      std::cout << indent_str
           << "Complex ALUs (Mul/Div) (Count: " << coredynp.num_muls
           << " ):" << std::endl;
      std::cout << indent_str_next << "Area = " << area.get_area() * 1e-6 << " mm^2"
           << std::endl;
      std::cout << indent_str_next
           << "Peak Dynamic = " << power.readOp.dynamic * clockRate << " W"
           << std::endl;
      //			std::cout << indent_str_next << "Subthreshold Leakage
      //= " << power.readOp.leakage  << " W" << std::endl;
      std::cout << indent_str_next << "Subthreshold Leakage = "
           << (long_channel ? power.readOp.longer_channel_leakage
                            : power.readOp.leakage)
           << " W" << std::endl;
      if (power_gating) {
        std::cout << indent_str_next << "Subthreshold Leakage with power gating = "
             << (long_channel
                     ? power.readOp.power_gated_with_long_channel_leakage
                     : power.readOp.power_gated_leakage)
             << " W" << std::endl;
      }
      std::cout << indent_str_next << "Gate Leakage = " << power.readOp.gate_leakage
           << " W" << std::endl;
      std::cout << indent_str_next
           << "Runtime Dynamic = " << rt_power.readOp.dynamic / executionTime
           << " W" << std::endl;
      std::cout << std::endl;
    }

  } else {
  }
}

void FunctionalUnit::leakage_feedback(double temperature) {
  // Update the temperature and initialize the global interfaces.
  interface_ip.temp = (unsigned int)round(temperature / 10.0) * 10;

  uca_org_t init_result = init_interface(&interface_ip); // init_result is dummy

  // This is part of FunctionalUnit()
  double area_t, leakage, gate_leakage;
  double pmos_to_nmos_sizing_r = pmos_to_nmos_sz_ratio();

  if (fu_type == FPU) {
    area_t = 4.47 * 1e6 *
             (g_ip->F_sz_nm * g_ip->F_sz_nm / 90.0 /
              90.0); // this is um^2 The base number
    if (g_ip->F_sz_nm > 90)
      area_t =
          4.47 * 1e6 * g_tp.scaling_factor.logic_scaling_co_eff; // this is um^2
    leakage = area_t * (g_tp.scaling_factor.core_tx_density) *
              cmos_Isub_leakage(5 * g_tp.min_w_nmos_,
                                5 * g_tp.min_w_nmos_ * pmos_to_nmos_sizing_r,
                                1,
                                inv) *
              g_tp.peri_global.Vdd / 2; // unit W
    gate_leakage = area_t * (g_tp.scaling_factor.core_tx_density) *
                   cmos_Ig_leakage(5 * g_tp.min_w_nmos_,
                                   5 * g_tp.min_w_nmos_ * pmos_to_nmos_sizing_r,
                                   1,
                                   inv) *
                   g_tp.peri_global.Vdd / 2; // unit W
  } else if (fu_type == ALU) {
    area_t = 280 * 260 * 2 * num_fu *
             g_tp.scaling_factor.logic_scaling_co_eff; // this is um^2 ALU + MUl
    leakage = area_t * (g_tp.scaling_factor.core_tx_density) *
              cmos_Isub_leakage(20 * g_tp.min_w_nmos_,
                                20 * g_tp.min_w_nmos_ * pmos_to_nmos_sizing_r,
                                1,
                                inv) *
              g_tp.peri_global.Vdd / 2; // unit W
    gate_leakage =
        area_t * (g_tp.scaling_factor.core_tx_density) *
        cmos_Ig_leakage(20 * g_tp.min_w_nmos_,
                        20 * g_tp.min_w_nmos_ * pmos_to_nmos_sizing_r,
                        1,
                        inv) *
        g_tp.peri_global.Vdd / 2;
  } else if (fu_type == MUL) {
    area_t = 280 * 260 * 2 * 3 * num_fu *
             g_tp.scaling_factor.logic_scaling_co_eff; // this is um^2 ALU + MUl
    leakage = area_t * (g_tp.scaling_factor.core_tx_density) *
              cmos_Isub_leakage(20 * g_tp.min_w_nmos_,
                                20 * g_tp.min_w_nmos_ * pmos_to_nmos_sizing_r,
                                1,
                                inv) *
              g_tp.peri_global.Vdd / 2; // unit W
    gate_leakage =
        area_t * (g_tp.scaling_factor.core_tx_density) *
        cmos_Ig_leakage(20 * g_tp.min_w_nmos_,
                        20 * g_tp.min_w_nmos_ * pmos_to_nmos_sizing_r,
                        1,
                        inv) *
        g_tp.peri_global.Vdd / 2;
  } else {
    std::cout << "Unknown Functional Unit Type" << std::endl;
    exit(1);
  }

  power.readOp.leakage = leakage * num_fu;
  power.readOp.gate_leakage = gate_leakage * num_fu;
  power.readOp.longer_channel_leakage =
      longer_channel_device_reduction(Core_device, coredynp.core_ty) *
      power.readOp.leakage;

  double pg_reduction = power_gating_leakage_reduction(false);
  power.readOp.power_gated_leakage = power.readOp.leakage * pg_reduction;
}

