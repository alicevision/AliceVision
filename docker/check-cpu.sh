#!/bin/sh

# This script is used to check the amount of available memory to ensure 2 GB of memory per process.
# If the number of processes is greater than the amount of available memory (considering 2 GB per core) we restrict the number of cores to use.
#
# Sample usage:
# CPU_CORES=`check-cpu.sh`
# make -j${CPU_CORES}


FREE_MEM=$(free -g | grep Mem | awk '
{
print $7
}
' | sed "s/Gi//")



test ${FREE_MEM} -ge 2 || FREE_MEM=$((${FREE_MEM} + 2))


test ${FREE_MEM} -lt $(nproc) || FREE_MEM=$(nproc)

echo $((${FREE_MEM}/2))