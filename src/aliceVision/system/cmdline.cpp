#include "cmdline.hpp"

#include "cpu.hpp"
#include "MemoryInfo.hpp"
#include <aliceVision/alicevision_omp.hpp>

namespace aliceVision {

bool CmdLine::execute(int argc, char** argv)
{
    std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());

    boost::program_options::options_description logParams("Log parameters");
    logParams.add_options()
        ("verboseLevel,v", boost::program_options::value<std::string>(&verboseLevel)->default_value(verboseLevel), "verbosity level (fatal, error, warning, info, debug, trace).");

    _allParams.add(logParams);

    boost::program_options::options_description hardwareParams("Hardware parameters");
    hardwareParams.add_options()
        ("maxMemoryAvailable", boost::program_options::value<size_t>(&_maxMemoryAvailable)->default_value(_maxMemoryAvailable), "User specified available RAM")
        ("maxCoresAvailable", boost::program_options::value<unsigned int>(&_maxCoresAvailable)->default_value(_maxCoresAvailable), "User specified available number of cores");

    _allParams.add(hardwareParams);

    boost::program_options::variables_map vm;
    try
    {
        boost::program_options::store(boost::program_options::parse_command_line(argc, argv, _allParams), vm);

        if (vm.count("help") || (argc == 1))
        {
            ALICEVISION_COUT(_allParams);
            return false;
        }
        boost::program_options::notify(vm);
    }
    catch (boost::program_options::required_option& e)
    {
        ALICEVISION_CERR("ERROR: " << e.what());
        ALICEVISION_COUT("Usage:\n\n" << _allParams);
        return false;
    }
    catch (boost::program_options::error& e)
    {
        ALICEVISION_CERR("ERROR: " << e.what());
        ALICEVISION_COUT("Usage:\n\n" << _allParams);
        return false;
    }

    ALICEVISION_COUT("Program called with the following parameters:");
    ALICEVISION_COUT(vm);

    // set verbose level
    system::Logger::get()->setLogLevel(verboseLevel);

    // Limit globally the maximum number of core used by openmp
    omp_set_num_threads(std::min(_maxCoresAvailable, unsigned int(system::get_total_cpus())));

    displayHardware();

    return true;
}

void CmdLine::displayHardware()
{
    std::cout << "Hardware : " << std::endl;
    
    std::cout << "\tDetected core count : " << system::get_total_cpus() << std::endl;

    if (_maxMemoryAvailable < std::numeric_limits<size_t>::max())
    {
        std::cout << "\tUser upper limit on core count : " << _maxCoresAvailable << std::endl;
    }

    std::cout << "\tOpenMP will use " << omp_get_max_threads() << " cores" << std::endl;

    auto meminfo = system::getMemoryInfo();
    
    std::cout << "\tDetected available memory : " << meminfo.availableRam / (1024 * 1024)  << " Mo" << std::endl;

    if (_maxCoresAvailable < std::numeric_limits<unsigned int>::max())
    {
        std::cout << "\tUser upper limit on memory available : " << _maxMemoryAvailable / (1024 * 1024) << " Mo" << std::endl;
    }

    std::cout << std::endl;
}

}