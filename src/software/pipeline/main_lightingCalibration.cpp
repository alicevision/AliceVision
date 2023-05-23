#include <aliceVision/image/all.hpp>
#include <aliceVision/image/io.hpp>

// SFMData
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

// Lighting calibration
#include <aliceVision/lightingEstimation/lightingCalibration.hpp>

// Command line parameters
#include <aliceVision/system/main.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

// Eigen
//#include <Eigen/Dense>
//#include <Eigen/Core>

//OpenCV
// #include <opencv2/core/core.hpp>
// #include <opencv2/core/eigen.hpp>
// #include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <array>
#include <sstream>
#include <fstream>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 0
#define ALICEVISION_SOFTWARE_VERSION_MINOR 1

// Namespaces
namespace po = boost::program_options;
namespace fs = boost::filesystem;

using namespace aliceVision;

int aliceVision_main(int argc, char **argv)
{
    std::string inputPath;
    std::string inputJSON;
    std::string ouputJSON;
    std::string method;
    bool saveAsModel;

    po::options_description allParams("AliceVision lighting calibration");
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
    ("inputPath,i", po::value<std::string>(&inputPath)->required(), "Path to input. Could be SfMData file or folder with pictures")
    ("inputJSON, j", po::value<std::string>(&inputJSON)->required(), "Path to JSON which describes sphere positions and radius")
    ("outputFile, o", po::value<std::string>(&ouputJSON)->required(), "Path to JSON output file");

    po::options_description optionalParams("Optional parameters");

    optionalParams.add_options()
    ("saveAsModel, s", po::value<bool>(&saveAsModel)->default_value(false), "Calibration used for several datasets")
    ("method, m", po::value<std::string>(&method)->default_value("brightestPoint"), "Method for light estimation");

    allParams.add(requiredParams).add(optionalParams);

    po::variables_map vm;
    try
    {
        po::store(po::parse_command_line(argc, argv, allParams), vm);

        if(vm.count("help") || (argc == 1))
        {
          ALICEVISION_COUT(allParams);
          return EXIT_SUCCESS;
        }
        po::notify(vm);
    }
    catch(po::required_option& e)
    {
      ALICEVISION_CERR("ERROR: " << e.what());
      ALICEVISION_COUT("Usage:\n\n" << allParams);
      return EXIT_FAILURE;
    }
    catch(po::error& e)
    {
      ALICEVISION_CERR("ERROR: " << e.what());
      ALICEVISION_COUT("Usage:\n\n" << allParams);
      return EXIT_FAILURE;
    }

    ALICEVISION_COUT("Program called with the following parameters:");
    ALICEVISION_COUT(vm);

    if(fs::is_directory(inputPath))
    {
        std::cout << "Directory input : WIP" << std::endl;
    }
    else
    {
        sfmData::SfMData sfmData;
        if(!sfmDataIO::Load(sfmData, inputPath, sfmDataIO::ESfMData(sfmDataIO::VIEWS|sfmDataIO::INTRINSICS)))
        {
            ALICEVISION_LOG_ERROR("The input file '" + inputPath + "' cannot be read");
            return EXIT_FAILURE;
        }
        lightingEstimation::lightCalibration(sfmData, inputJSON, ouputJSON, method, saveAsModel);
    }

    return 0;
}
