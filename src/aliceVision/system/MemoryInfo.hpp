// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <cstddef>
#include <ostream>

namespace aliceVision {
namespace system {

struct MemoryInfo
{
    std::size_t totalRam;
    std::size_t freeRam;
    //	std::size_t sharedRam;
    //	std::size_t bufferRam;
    std::size_t totalSwap;
    std::size_t freeSwap;
};

MemoryInfo getMemoryInfo();

std::ostream& operator<<(std::ostream& os, const MemoryInfo& infos);

}
}

