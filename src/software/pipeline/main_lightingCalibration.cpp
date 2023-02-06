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

int aliceVision_main(int argc, char **argv)
{
    namespace po = boost::program_options;
    namespace fs = boost::filesystem;

    std::string inputPath;
    std::string inputJSON;
    std::string ouputJSON;
    Eigen::Vector2f sphereCenterOffset(0, 0);
    double sphereRadius = 1.0;

    po::options_description allParams("AliceVision lighting calibration");
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
    ("inputPath,i", po::value<std::string>(&inputPath)->required(), "Path to input. Could be SfMData file or folder with pictures")
    ("inputJSON, j", po::value<std::string>(&inputJSON)->required(), "Path to JSON which describes sphere positions and radius")
    ("outputFile, o", po::value<std::string>(&ouputJSON)->required(), "Path to JSON output file");

    allParams.add(requiredParams);

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
    catch(boost::program_options::required_option& e)
    {
      ALICEVISION_CERR("ERROR: " << e.what());
      ALICEVISION_COUT("Usage:\n\n" << allParams);
      return EXIT_FAILURE;
    }
    catch(boost::program_options::error& e)
    {
      ALICEVISION_CERR("ERROR: " << e.what());
      ALICEVISION_COUT("Usage:\n\n" << allParams);
      return EXIT_FAILURE;
    }

    ALICEVISION_COUT("Program called with the following parameters:");
    ALICEVISION_COUT(vm);

    if(boost::filesystem::is_directory(inputPath))
    {
        std::cout << "Directory input : WIP" << std::endl;
    }
    else
    {
        aliceVision::sfmData::SfMData sfmData;
        if(!aliceVision::sfmDataIO::Load(sfmData, inputPath, aliceVision::sfmDataIO::ESfMData(aliceVision::sfmDataIO::VIEWS|aliceVision::sfmDataIO::INTRINSICS)))
        {
            ALICEVISION_LOG_ERROR("The input file '" + inputPath + "' cannot be read");
            return EXIT_FAILURE;
        }
        lightCalibration(sfmData, inputJSON, ouputJSON);
    }

    return 0;
}
