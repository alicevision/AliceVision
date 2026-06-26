#!/usr/bin/env bash

# This script is used to check the amount of available memory  to ensure 2 GB
# of memory per process. If the number of processes is greater than the amount
# of available memory (considering 2 GB per core) we restrict the number of
# parallel make jobs to avoid resource contention.
#
# Sample usage:
# CPU_CORES=$(check-cpu.sh)
# make -j${CPU_CORES}

MIN_RAM_PER_CORE=2048 # Arbitrary minimum of RAM per process
MIN_RESERVE_RAM=2048  # Arbitrary minimum reserve RAM for the system
DEBUG=${DEBUG:-0}
NPROC=$(nproc || getconf _NPROCESSORS_ONLN)
FREE_MEM=$(awk '/^MemAvailable:/{printf("%d",$2/1024)}' /proc/meminfo)

>&2 echo "The amount of available memory on the system is ${FREE_MEM} MB"

# The number of processes we can run in parallel to guarantee at least N GB
# of RAM per process in keeping with the minimum RAM requirement for the
# system to function.
MAX_CORES_RAM=$(((FREE_MEM - MIN_RESERVE_RAM) / MIN_RAM_PER_CORE))

# If the number of cores goes negative, that means the system has either 2 GB
# of RAM or less than that. We can warn the user of a potential for the build
# to fail the system, since this does not meet the minimum requirement
# to build AliceVision.

if [[ "$MAX_CORES_RAM" -lt 0 ]]; then
  >&2 echo "Warning: The system does not support the minimum amount of RAM needed to run"
  >&2 echo "the build system. We will attempt to build it anyway, but the system may fail"
  >&2 echo "to have enough memory for operations, or may end up using copious amounts of swap space."
fi

PRACTICAL_MAX_CORES_RAM=$((MAX_CORES_RAM < 0 ? 1 : MAX_CORES_RAM))

# Maximum usable cores regarding the number of available cores and
# the amount of available memory
USABLE_CORES=$((PRACTICAL_MAX_CORES_RAM < NPROC ? PRACTICAL_MAX_CORES_RAM : NPROC))

if [[ "$DEBUG" -eq 1 ]]; then
  >&2 cat <<EOF
Available RAM : ${FREE_MEM} MB
CPU cores     : ${NPROC}
RAM limit     : ${MAX_CORES_RAM}
Using         : ${USABLE_CORES}
EOF
fi

printf '%s' "${USABLE_CORES}"
