#ifndef __INST_DECODER_H__
#define __INST_DECODER_H__

#include "XML_Parse.h"
#include "arch_const.h"
#include "basic_circuit.h"
#include "basic_components.h"
#include "cacti_interface.h"
#include "component.h"
#include "const.h"
#include "decoder.h"
#include "parameter.h"
#include "xmlParser.h"

#include <cassert>
#include <cmath>
#include <cstring>
#include <iostream>

class inst_decoder : public Component {
public:
  void set_params(bool _is_default,
                  const InputParameter *configure_interface,
                  int opcode_length_,
                  int num_decoders_,
                  bool x86_,
                  enum Device_ty device_ty_ = Core_device,
                  enum Core_type core_ty_ = Inorder);
  inst_decoder() { init_params = false; };
  bool is_default;
  int opcode_length;
  int num_decoders;
  bool x86;
  int num_decoder_segments;
  int num_decoded_signals;
  InputParameter l_ip;
  uca_org_t local_result;
  enum Device_ty device_ty;
  enum Core_type core_ty;

  Decoder final_dec;
  Predec pre_dec;
  PredecBlk predec_blk1;
  PredecBlk predec_blk2;
  PredecBlkDrv predec_blk_drv1;
  PredecBlkDrv predec_blk_drv2;
  statsDef tdp_stats;
  statsDef rtp_stats;
  statsDef stats_t;
  powerDef power_t;

  void computeArea();
  void computeDynamicPower();

  void inst_decoder_delay_power();
  ~inst_decoder();
  void leakage_feedback(double temperature);

private:
  bool init_params;
};

#endif //__INST_DECODER_H__