#!/bin/bash
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

script_name="unit_test.sh"

print_info () {
  echo -e "[ $script_name ] $1"
}

#--------------------------------------------------------------------
#
#
#
#--------------------------------------------------------------------
TESTS=("basic_test_1"
       "serialization_test_1"
       "serialization_test_2"
       "serialization_test_3"
       "serialization_test_4")

#--------------------------------------------------------------------
# Output Directories
#   ___  _   _ _____ ____  _   _ _____   ____ ___ ____  
#  / _ \| | | |_   _|  _ \| | | |_   _| |  _ \_ _|  _ \ 
# | | | | | | | | | | |_) | | | | | |   | | | | || |_) |
# | |_| | |_| | | | |  __/| |_| | | |   | |_| | ||  _ < 
#  \___/ \___/  |_| |_|    \___/  |_|   |____/___|_| \_\
#                                                       
#--------------------------------------------------------------------
OUTPUT="./output"
if [ ! -d $OUTPUT ]; then
  print_info "Creating $OUTPUT"
  mkdir -p $OUTPUT
#else
#  print_info "Cleaning $OUTPUT"
#  rm -f $OUTPUT/*
fi
for test_set in ${TESTS[@]}; do
  mkdir -p $OUTPUT/$test_set
done

#--------------------------------------------------------------------
# Run Tests
#  _____ _____ ____ _____ ____  
# |_   _| ____/ ___|_   _/ ___| 
#   | | |  _| \___ \ | | \___ \ 
#   | | | |___ ___) || |  ___) |
#   |_| |_____|____/ |_| |____/ 
#                             
#--------------------------------------------------------------------
./unit_test.py
