#pragma once

namespace openMVG {
namespace system {

/**
 * Taken from https://github.com/anrieff/libcpuid.
 * Duplicated to avoid the dependency for one function.
 */
int cpu_clock_by_os();

int get_total_cpus();

}
}


