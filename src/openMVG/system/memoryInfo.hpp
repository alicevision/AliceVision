#pragma once

#include <cstddef>
#include <ostream>

namespace openMVG {
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

