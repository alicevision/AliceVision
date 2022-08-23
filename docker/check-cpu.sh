#!/bin/sh

# This script is used to check the amount of available memory and restricts the number of concurrent jobs/processes (-j${CPU_CORES}) used
# to 1 GB per process. If the number of processes are less than the amount of available memory (considering 2GB per core) we restrict the number
# of cores to the total number of cores present on the system.


FREE_MEM=$(free -g | grep Mem | awk '
{
print $7
}
' | sed "s/Gi//")



test ${FREE_MEM} -ge 2 || FREE_MEM=$((${FREE_MEM} + 2))


test ${FREE_MEM} -lt $(nproc) || FREE_MEM=$(nproc)

echo $((${FREE_MEM}/2))