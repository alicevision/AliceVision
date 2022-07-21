#include <aliceVision/image/all.hpp>
#include <aliceVision/image/io.hpp>

// SFMData
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

// Photometric Stereo
#include <aliceVision/photometricStereo/photometricDataIO.hpp>
#include <aliceVision/photometricStereo/normalIntegration.hpp>

// Command line parameters
#include <aliceVision/system/main.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Core>

//OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>

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

    bool isPerspective(true);
    std::string outputFolder;
    std::string pathToK;
    std::string inputPath;
    std::string sfmDataFile;

    // image downscale factor during process
    int downscale = 1;

    po::options_description allParams("AliceVision normal integration");
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
    ("inputPath,i", po::value<std::string>(&inputPath)->required(), "Path to input : a folder containing the normal map and the mask")
    ("outputPath,o", po::value<std::string>(&outputFolder)->required(), "outputFolder.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
    ("sfmDataFile,s", po::value<std::string>(&sfmDataFile)->default_value(""), "Path to SfmData file")
    ("downscale, d", po::value<int>(&downscale)->default_value(downscale), "Downscale factor for faster results" );

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

    if(sfmDataFile.compare("") == 0)
    {
        normalIntegration(inputPath, isPerspective, downscale, outputFolder);
    }
    else
    {
      aliceVision::sfmData::SfMData sfmData;
      if(!aliceVision::sfmDataIO::Load(sfmData, sfmDataFile, aliceVision::sfmDataIO::ESfMData::ALL))
      {
          ALICEVISION_LOG_ERROR("The input file '" + sfmDataFile + "' cannot be read");
          return EXIT_FAILURE;
      }
      normalIntegration(sfmData, inputPath, isPerspective, downscale, outputFolder);
    }

    return 0;
};

