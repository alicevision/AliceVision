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
    _hContext.setupFromCommandLine(hardwareParams);

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

    _hContext.displayHardware();

    return true;
}

}