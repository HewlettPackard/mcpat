/*****************************************************************************
 *                                McPAT/CACTI
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

#ifndef __PREDEC_H__
#define __PREDEC_H__

#include "area.h"
#include "component.h"
#include "parameter.h"
#include "powergating.h"
#include "predec_blk.h"
#include "predec_blk_drv.h"

#include <boost/serialization/assume_abstract.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/utility.hpp>
#include <vector>

class Predec : public Component {
public:
  Predec(){};
  void set_params(PredecBlkDrv *drv1, PredecBlkDrv *drv2);
  Predec(PredecBlkDrv *drv1, PredecBlkDrv *drv2);

  double compute_delays(double inrisetime); // return outrisetime

  void leakage_feedback(double temperature);
  PredecBlk *blk1;
  PredecBlk *blk2;
  PredecBlkDrv *drv1;
  PredecBlkDrv *drv2;

  powerDef block_power;
  powerDef driver_power;

private:
  // returns <delay, risetime>
  pair<double, double>
  get_max_delay_before_decoder(pair<double, double> input_pair1,
                               pair<double, double> input_pair2);
  // Serialization
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive &ar, const unsigned int version) {
    ar &block_power;
    ar &driver_power;
    Component::serialize(ar, version);
  }
};

#endif // __PREDEC_H__
