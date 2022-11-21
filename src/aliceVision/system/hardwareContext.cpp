#include "hardwareContext.hpp"

#include "cpu.hpp"
#include "MemoryInfo.hpp"
#include <aliceVision/alicevision_omp.hpp>

namespace aliceVision {

void HardwareContext::setOptions(boost::program_options::options_description & options)
{
    options.add_options()
        ("maxMemoryAvailable", boost::program_options::value<size_t>(&_maxUserMemoryAvailable)->default_value(_maxUserMemoryAvailable), "User specified available RAM")
        ("maxCoresAvailable", boost::program_options::value<unsigned int>(&_maxUserCoresAvailable)->default_value(_maxUserCoresAvailable), "User specified available number of cores");
}

void HardwareContext::displayHardware()
{
    std::cout << "Hardware : " << std::endl;
    
    std::cout << "\tDetected core count : " << system::get_total_cpus() << std::endl;

    if (_maxUserCoresAvailable < std::numeric_limits<size_t>::max())
    {
        std::cout << "\tUser upper limit on core count : " << _maxUserCoresAvailable << std::endl;
    }

    std::cout << "\tOpenMP will use " << omp_get_max_threads() << " cores" << std::endl;

    auto meminfo = system::getMemoryInfo();
    
    std::cout << "\tDetected available memory : " << meminfo.availableRam / (1024 * 1024)  << " Mo" << std::endl;

    if (_maxUserMemoryAvailable < std::numeric_limits<unsigned int>::max())
    {
        std::cout << "\tUser upper limit on memory available : " << _maxUserMemoryAvailable / (1024 * 1024) << " Mo" << std::endl;
    }

    std::cout << std::endl;
}

unsigned int HardwareContext::getMaxThreads() const
{   
    //Get hardware limit on threads
    unsigned int count = system::get_total_cpus();

    //Get User max threads
    if (count > _maxUserCoresAvailable)
    {
        count = _maxUserCoresAvailable;
    }

    //Get User limit max threads
    if (_limitUserCores > 0)
    {
        if (count > _limitUserCores)
        {
            count = _limitUserCores;
        }
    }

    return count;
}

}