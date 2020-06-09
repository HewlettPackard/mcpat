#!/bin/bash

# Format C Code:
find -name '*.cpp' -o -name '*.h' -o -name '*.hh' -o -name '*.c' -o -name '*.cc' | xargs clang-format -i --verbose

# Format Python Code:
yapf -ir -vv . 
