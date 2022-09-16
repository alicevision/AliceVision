#!/bin/sh

# This script is used to check the amount of available memory to ensure 2 GB of memory per process.
# If the number of processes is greater than the amount of available memory (considering 2 GB per core) we restrict the number of cores to use.
#
# Sample usage:
# CPU_CORES=`check-cpu.sh`
# make -j${CPU_CORES}


NPROC=$(nproc)

FREE_MEM=$(free -g | grep Mem | awk '
{
print $7
}
' | sed "s/Gi//")

# Arbitrary minimum of RAM per process
MIN_RAM_PER_CORE=2

# The number of processes we can run in parallel to guarantee at least N GB of RAM per process
MAX_CORES_RAM=$((${FREE_MEM} / ${MIN_RAM_PER_CORE}))

# Maximum usable cores regarding the number of available cores and the amount of available memory
USABLE_CORES=$((${MAX_CORES_RAM} < ${NPROC} ? ${MAX_CORES_RAM} : ${NPROC}))

# echo "NPROC=${NPROC}"
# echo "FREE_MEM=${FREE_MEM}"
# echo "MIN_RAM_PER_CORE=${MIN_RAM_PER_CORE}"
# echo "MAX_CORES_RAM=${MAX_CORES_RAM}"
# echo " => USABLE_CORES=${USABLE_CORES}"
# echo ""

echo ${USABLE_CORES}