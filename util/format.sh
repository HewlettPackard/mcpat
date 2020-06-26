#!/bin/bash

SCRIPT="$(readlink -f $0)"
SCRIPT_PATH="$(dirname $SCRIPT)"
SRC_PATH="$SCRIPT_PATH/../src"

# Format C Code:
find $SRC_PATH -name '*.cpp' \
  -o -name '*.h' \
  -o -name '*.hh' \
  -o -name '*.c' \
  -o -name '*.cc' \
  | grep -vE "build" \
  | xargs clang-format -i --verbose

# Format Python Code:
yapf -ir -vv . 
