#include "hardwareContext.hpp"

#include "cpu.hpp"
#include "MemoryInfo.hpp"
#include <aliceVision/alicevision_omp.hpp>

namespace aliceVision {

void HardwareContext::displayHardware()
{
    std::cout << "Hardware : " << std::endl;

    std::cout << "\tDetected core count : " << system::get_total_cpus() << std::endl;

    if (_maxUserCoresAvailable < std::numeric_limits<unsigned int>::max())
    {
        std::cout << "\tUser upper limit on core count : " << _maxUserCoresAvailable << std::endl;
    }

    std::cout << "\tOpenMP will use " << omp_get_max_threads() << " cores" << std::endl;

    auto meminfo = system::getMemoryInfo();

    std::cout << "\tDetected available memory : " << meminfo.availableRam / (1024 * 1024) << " Mo" << std::endl;

    if (_maxUserMemoryAvailable < std::numeric_limits<size_t>::max())
    {
        std::cout << "\tUser upper limit on memory available : " << _maxUserMemoryAvailable / (1024 * 1024) << " Mo" << std::endl;
    }

    std::cout << std::endl;
}

unsigned int HardwareContext::getMaxThreads() const
{
    // Get hardware limit on threads
    unsigned int count = system::get_total_cpus();

    // Get User max threads
    if (count > _maxUserCoresAvailable)
    {
        count = _maxUserCoresAvailable;
    }

    // Get User limit max threads
    if (_limitUserCores > 0 && count > _limitUserCores)
    {
        count = _limitUserCores;
    }

    return count;
}

size_t HardwareContext::getMaxMemory() const
{
    auto meminfo = system::getMemoryInfo();

    size_t ret = meminfo.availableRam;
    ret = std::min(ret, _maxUserMemoryAvailable);

    return ret;
}

}  // namespace aliceVision