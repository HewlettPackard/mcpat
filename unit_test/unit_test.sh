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

print_pass () {
  green="\e[32m"
  nc="\e[0m"
  echo -e "$green[ $script_name ] PASS:$nc $1"
}

print_error () {
  red="\e[31m"
  nc="\e[0m"
  echo -e "$red[ $script_name ] ERROR:$nc $1"
}

print_test_results () {
  green="\e[32m"
  red="\e[31m"
  nc="\e[0m"
  echo -e "[ $script_name ] Passed $green$1$nc; Failed $red$2$nc; out of $3 Unit Tests"
}


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
else
  print_info "Cleaning $OUTPUT"
  rm -f $OUTPUT/*
fi

GOLDEN="./golden"

#--------------------------------------------------------------------
# Run Tests
#  _____ _____ ____ _____ ____  
# |_   _| ____/ ___|_   _/ ___| 
#   | | |  _| \___ \ | | \___ \ 
#   | | | |___ ___) || |  ___) |
#   |_| |_____|____/ |_| |____/ 
#                             
#--------------------------------------------------------------------
INPUT="./input"
PASS_COUNT=0
TOTAL_COUNT=0
FAIL_COUNT=0
for t in $(ls $INPUT); do 
  test_name=$(basename $t .xml)
  TOTAL_COUNT=$((TOTAL_COUNT + 1))
  ../mcpat -infile $INPUT/$test_name.xml -print_level 5 -opt_for_clk 1 > $OUTPUT/$test_name.out 2> $OUTPUT/$test_name.err
  if [ -s $OUTPUT/$test_name.err ] || [ ! -s $OUTPUT/$test_name.out ];
  then
    print_error "$test_name; check $OUTPUT/$test_name.err"
    FAIL_COUNT=$((FAIL_COUNT + 1))
  else
    if [ $(grep -rnI "nan\|inf" $OUTPUT/${test_name}.out | wc -l) -ne 0 ];
    then
      print_pass "$test_name; nan, inf present in output; check $OUTPUT/$test_name.out"
      FAIL_COUNT=$((FAIL_COUNT + 1))
    else
      if [ $(diff $GOLDEN/$test_name.golden $OUTPUT/$test_name.out | wc -l) -eq 0 ];
      then
        print_pass "$test_name"
        PASS_COUNT=$((PASS_COUNT + 1))
      else
        print_error "$test_name; output differs from golden output"
        FAIL_COUNT=$((FAIL_COUNT + 1))
      fi
    fi
  fi
done
print_test_results $PASS_COUNT $FAIL_COUNT $TOTAL_COUNT
