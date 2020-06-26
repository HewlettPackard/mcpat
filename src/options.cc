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
 *
 * Author:
 *    Andrew Smith
 ***************************************************************************/
#include "options.h"

#include <boost/program_options.hpp>
#include <iostream>
#include <string>

bool mcpat::Options::parse(int argc, char **argv) {
  // clang-format off
  po::options_description desc("General Options");
  desc.add_options()
    ("help,h", "Display help message")
  ;

  po::options_description io("IO Options");
  io.add_options()
    ("infile,i", po::value<std::string>(&input_xml), "Input XML File")
    ("print_level,p", po::value<int>(&print_level), "How detailed to print device tree; [1,5] being most detailed");
  ;

  po::options_description serialization("Serialization Options");
  serialization.add_options()
    ("serial_file", po::value<std::string>(&serialization_file)->default_value("mcpat_cp.txt"), "file name to serialize to")
    ("serial_create", po::value<bool>(&serialization_create)->default_value(false), "Create A Serialization Checkpoint") 
    ("serial_restore", po::value<bool>(&serialization_restore)->default_value(false), "Restore from a Serialization Checkpoint")
  ;

  po::options_description optimization("Optimization Options");
  optimization.add_options()
    ("opt_for_clk,o", po::value<bool>(&opt_for_clk)->default_value(true), "0: optimize for ED^2P only; 1: optimzed for target clock rate")
  ;
  // clang-format on

  po::options_description all_options;
  all_options.add(desc);
  all_options.add(io);
  all_options.add(serialization);
  all_options.add(optimization);

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, all_options), vm);
  po::notify(vm);

  if (vm.count("help")) {
    std::cout << all_options << "\n";
    return false;
  }
  if (input_xml == "") {
    std::cerr << "Must specify an Input XML File; \"./mcpat --help\" for more "
                 "options\n";
    return false;
  }
  return true;
}
