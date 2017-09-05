// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#pragma once

namespace openMVG {
namespace system {

/**
 * @brief Returns the CPU clock, as reported by the OS.
 *
 * Taken from https://github.com/anrieff/libcpuid.
 * Duplicated to avoid the dependency for one function.
 */
int cpu_clock_by_os();

/**
 * @brief Returns the total number of CPUs.
 */
int get_total_cpus();

}
}


