#!/bin/bash

new_name=""
old_name=""

#for i in {1..50..1}; do
#  printf -v new_name "mp_%02d.xml" $i
#  if [ -f "$1/mp_$i.xml" ]; then 
#    echo "mv "$1/mp_$i.xml" $1/$new_name"
#    mv "$1/mp_$i.xml" $1/$new_name
#  fi
#done

for i in {1..50..1}; do
  printf -v old_name "mp_%02d.out" $i
  printf -v new_name "mp_%02d.golden" $i
  if [ -f "$1/$old_name" ]; then 
    echo "mv $1/$old_name $1/$new_name"
    mv $1/$old_name $1/$new_name
  fi
done
