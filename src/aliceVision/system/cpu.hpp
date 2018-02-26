// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

namespace aliceVision {
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


