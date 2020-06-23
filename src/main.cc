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
#include "XML_Parse.h"
#include "globalvar.h"
#include "io.h"
#include "options.h"
#include "processor.h"
#include "version.h"
#include "xmlParser.h"

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <fstream>
#include <iostream>

void save(const Processor &s, std::string name) {
  // Make an archive
  std::ofstream ofs(name.c_str());
  boost::archive::text_oarchive oa(ofs);
  oa << s;
}

void restore(Processor &s, std::string name) {
  // Restore from the Archive
  // std::cerr << "Archive " << name << "\n";
  std::ifstream ifs(name.c_str());
  if (ifs.good()) {
    boost::archive::text_iarchive ia(ifs);
    ia >> s;
  } else {
    std::cerr << "Archive " << name << " cannot be used\n";
    assert(false);
  }
}

using namespace std;

int main(int argc, char *argv[]) {
  mcpat::Options opt;
  if (!opt.parse(argc, argv)) {
    return 1;
  }

  opt_for_clk = opt.opt_for_clk;

  cout << "McPAT (version " << VER_MAJOR << "." << VER_MINOR << " of "
       << VER_UPDATE << ") is computing the target processor...\n " << endl;

  // parse XML-based interface
  ParseXML *p1 = new ParseXML();
  Processor proc;
  p1->parse(opt.input_xml);
  if (opt.serialization_create) {
    proc.init(p1);
    save(proc, opt.serialization_file);
    std::cout << "Checkpoint generated @: " << opt.serialization_file << "\n";
    return 0;
  } else if (opt.serialization_restore) {
    restore(proc, opt.serialization_file);
    proc.init(p1, true);
  } else {
    proc.init(p1);
  }
  proc.displayEnergy(2, opt.print_level);
  delete p1;
  return 0;
}
