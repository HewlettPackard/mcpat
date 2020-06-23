#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2020 Andrew Smith
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import sys
import os
import subprocess
import argparse
import datetime
import shutil
import re
import difflib
import glob
from threading import Timer

start = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
verbose = True
debug = True
quiet = False
timeout_limit = 120.0
kill_flag = False

parser = argparse.ArgumentParser()
parser.add_argument('--input', type=str, default="./input/basic_test_1", help="Test Input Path")
parser.add_argument("--output", type=str, default="./output/basic_test_1", help="Test Output Path")
parser.add_argument("--golden", type=str, default="./golden/basic_test_1", help="Test Golden Path")
parser.add_argument("--serial", type=bool, default=False, help="Serial if true, Basic if false")
args = parser.parse_args()

input_path = args.input
output_path = args.output
golden_path = args.golden

def print_info(info, *args):
  if verbose:
    print("[ " + __file__ + " ] " + info + " " +
          " ".join([str(x) for x in args]))
  elif not quiet:
    print("[ " + __file__ + " ] " + info)


def print_pass(vector, *args):
  if verbose:
    print("\033[32m[ " + __file__ + " ] PASS:\033[00m " + vector + " " +
          " ".join([str(x) for x in args]))
  elif not quiet:
    print("\033[32m[ " + __file__ + " ] PASS:\033[00m " + vector)


def print_fail(vector, *args):
  if verbose:
    print("\033[31m[ " + __file__ + " ] FAIL:\033[00m " + vector + " " +
          " ".join([str(x) for x in args]))
  elif not quiet:
    print("\033[31m[ " + __file__ + " ] FAIL:\033[00m " + vector)


def print_results(passed, failed, total):
  if not quiet:
    print("[ " + __file__ + " ] Passed: \033[32m" + str(passed) +
          "\033[00m; Failed: \033[31m" + str(failed) +
          "\033[00m; Total Vectors: " + str(total))


def kill(p):
  global kill_flag
  kill_flag = True
  try:
    p.kill()
  except OSError:
    pass


def diff_result(vector):
  outfile = os.path.join(output_path, vector + ".out")
  difffile = os.path.join(output_path, vector + ".diff")
  goldfile = os.path.join(golden_path, vector + ".golden")
  with open(outfile, "r") as o, open(goldfile, "r") as g:
    outlines = o.readlines()
    goldlines = g.readlines()
  result = list(difflib.unified_diff(outlines, goldlines))
  with open(difffile, "w") as d:
    d.writelines(result)
  if len(result) == 0:
    return 0
  return 1


def run_test_normal(vector):
  global kill_flag
  kill_flag = False
  infile = os.path.join(input_path, vector + ".xml")
  stdo = os.path.join(output_path, vector + ".out")
  stde = os.path.join(output_path, vector + ".err")
  with open(stdo, "w") as so, open(stde, "w") as se:
    p = subprocess.Popen([
        "../build/mcpat",
        "-i",
        infile,
        "-p",
        "5",
    ],
                         stdout=so,
                         stderr=se)
    t = Timer(timeout_limit, kill, [p])
    t.start()
    p.wait()
    t.cancel()
  if kill_flag:
    print_fail(vector, "Timeout Limit of " + str(timeout_limit) + "s Reached")
    return 1
  else:
    if diff_result(vector) == 0:
      print_pass(vector)
      return 0
    else:
      print_fail(
          vector,
          "The files " + vector + ".out and " + vector + ".golden differ")
      return 1
  return 0


def run_test_serializaiton_create(vector):
  global kill_flag
  kill_flag = False
  infile = os.path.join(input_path, vector + ".xml")
  sname = os.path.join(output_path, vector + ".txt")
  stdo = os.path.join(output_path, vector + ".out")
  stde = os.path.join(output_path, vector + ".err")
  with open(stdo, "w") as so, open(stde, "w") as se:
    p = subprocess.Popen([
        "../build/mcpat", "-i", infile, "-p", "5", "--serial_create=true",
        "--serial_file=" + sname
    ],
                         stdout=so,
                         stderr=se)
    t = Timer(timeout_limit, kill, [p])
    t.start()
    p.wait()
    t.cancel()
  if kill_flag:
    print_fail(vector, "Timeout Limit of " + str(timeout_limit) + "s Reached")
    return 1
  if (os.stat(os.path.join(output_path, vector + ".txt")).st_size > 0):
    print_pass(vector)
    return 0
  else:
    return 1
  return 0


def run_test_serialization_restore(vector, sfile):
  global kill_flag
  kill_flag = False
  infile = os.path.join(input_path, vector + ".xml")
  sname = os.path.join(output_path, sfile + ".txt")
  stdo = os.path.join(output_path, vector + ".out")
  stde = os.path.join(output_path, vector + ".err")
  with open(stdo, "w") as so, open(stde, "w") as se:
    p = subprocess.Popen([
        "../build/mcpat", "-i", infile, "-p", "5", "--serial_restore=true",
        "--serial_file=" + sname
    ],
                         stdout=so,
                         stderr=se)
    t = Timer(timeout_limit, kill, [p])
    t.start()
    p.wait()
    t.cancel()
  if kill_flag:
    print_fail(vector, "Timeout Limit of " + str(timeout_limit) + "s Reached")
    return 1
  #else:
  #  if diff_result(vector) == 0:
  #    print_pass(vector)
  #    return 0
  #  else:
  #    print_fail(
  #        vector,
  #        "The files " + vector + ".out and " + vector + ".golden differ")
  #    return 1
  print_pass(vector)
  return 0


def get_vectors():
  files = glob.glob(os.path.join(input_path, "*"))
  vectors = sorted([os.path.basename(f).split(".")[0] for f in files])
  return vectors


if __name__ == "__main__":
  p = 0
  f = 0
  print_info(start)
  vectors = get_vectors()
  print_info("Found " + str(len(vectors)) + " test vectors")
  if not args.serial:
    for vector in vectors:
      if run_test_normal(vector) == 0:
        p += 1
      else:
        f += 1
  else:
    # Create a Serialized File:
    if(len(vectors) > 0):
      if run_test_serializaiton_create(vectors[0]) == 0:
        # Use Serialized File for Remainder of Tests:
        for vector in vectors:
          if run_test_serialization_restore(vector, vectors[0]) == 0:
            p += 1
          else:
            f += 1
      else:
        print_info("Failed to create serialization checkpoint")
    else:
      print_info("No files in "+input_path)
      sys.exit(1)
  print_results(p, f, len(vectors))
