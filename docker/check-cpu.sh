#!/bin/sh

# This script is used to check the amount of available memory to ensure 2 GB of memory per process.
# If the number of processes is greater than the amount of available memory (considering 2 GB per core) we restrict the number of cores to use.
#
# Sample usage:
# CPU_CORES=`check-cpu.sh`
# make -j${CPU_CORES}


NPROC=$(nproc)

FREE_MEM=$(awk '/^MemAvailable:/{printf("%d",$2/1024/1024)}' /proc/meminfo)

>&2 echo "The amount of available memory on the system is ${FREE_MEM} GB" 

# Arbitrary minimum of RAM per process
MIN_RAM_PER_CORE=2

#Arbitrary minimum reserve RAM for the system. We will keep this at 2GB.
MIN_RESERVE_RAM=2

# The number of processes we can run in parallel to guarantee at least N GB of RAM per process in keeping with the minimum RAM requirement for the system to function.
MAX_CORES_RAM=$(($((${FREE_MEM}-${MIN_RESERVE_RAM})) / ${MIN_RAM_PER_CORE}))

#If the number of cores goes negative, that means the system has either 2 GB of RAM or less than that. We can warn the user of a potential for the build to fail the system, since this does not meet the minimum requirement to build AliceVision.

test ${MAX_CORES_RAM} -ge 0 || >&2 echo "Warning: The system does not support the minimum amount of RAM needed to run the build system. We will attempt to build it anyway, but the system may fail to have enough memory for operations, or may end up using copious amounts of swap space."

PRACTICAL_MAX_CORES_RAM=$((${MAX_CORES_RAM} < 0 ? 1 : ${MAX_CORES_RAM}))

# Maximum usable cores regarding the number of available cores and the amount of available memory
USABLE_CORES=$((${PRACTICAL_MAX_CORES_RAM} < ${NPROC} ? ${PRACTICAL_MAX_CORES_RAM} : ${NPROC}))

# echo "NPROC=${NPROC}"
# echo "FREE_MEM=${FREE_MEM}"
# echo "MIN_RAM_PER_CORE=${MIN_RAM_PER_CORE}"
# echo "MAX_CORES_RAM=${MAX_CORES_RAM}"
# echo " => USABLE_CORES=${USABLE_CORES}"
# echo ""

echo -n ${USABLE_CORES}
