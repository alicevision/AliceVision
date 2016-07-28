#!/bin/bash

declare -a arr=("bark" "bikes" "boat" "graf" "leuven" "trees" "ubc" "wall")

for i in "${arr[@]}"
do
    for (( j = 1 ; j <= 6; j++ ))
    do
       echo "OCV $i $j"
       ./sift.sh OCV opencv $i $j
    done
    for (( j = 1 ; j <= 6; j++ ))
    do
       echo "VL $i $j"
       ./sift.sh VL vlfeat $i $j
    done
done
