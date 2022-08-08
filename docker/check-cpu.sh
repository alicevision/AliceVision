#!/bin/sh

FREE_MEM=$(free -g | grep Mem | awk '
{
print $7
}
' | sed "s/Gi//")



test ${FREE_MEM} -ge 1 || FREE_MEM=$((${FREE_MEM} + 1))


test ${FREE_MEM} -lt $(nproc) || FREE_MEM=$(nproc)

echo ${FREE_MEM}