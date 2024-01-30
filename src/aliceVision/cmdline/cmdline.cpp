#include "cmdline.hpp"

#include <aliceVision/system/cpu.hpp>
#include <aliceVision/alicevision_omp.hpp>

namespace aliceVision {

bool CmdLine::execute(int argc, char** argv)
{
    std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());

    boost::program_options::options_description logParams("Log parameters");
    logParams.add_options()("verboseLevel,v",
                            boost::program_options::value<std::string>(&verboseLevel)->default_value(verboseLevel),
                            "verbosity level (fatal, error, warning, info, debug, trace).");

    _allParams.add(logParams);

    boost::program_options::options_description hardwareParams("Hardware parameters");

    size_t uma = _hContext.getUserMaxMemoryAvailable();
    unsigned int uca = _hContext.getUserMaxCoresAvailable();

    hardwareParams.add_options()(
      "maxMemoryAvailable", boost::program_options::value<size_t>(&uma)->default_value(uma), "User specified available RAM")(
      "maxCoresAvailable", boost::program_options::value<unsigned int>(&uca)->default_value(uca), "User specified available number of cores");

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

    _hContext.setUserMaxMemoryAvailable(uma);
    _hContext.setUserMaxCoresAvailable(uca);
    _hContext.displayHardware();

    return true;
}

}  // namespace aliceVision