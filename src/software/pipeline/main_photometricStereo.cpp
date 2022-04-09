#include <aliceVision/image/all.hpp>
#include <aliceVision/image/io.hpp>

// SFMData
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

// Photometric Stereo
#include <aliceVision/photometricStereo/photometricDataIO.hpp>
#include <aliceVision/photometricStereo/normalIntegration.hpp>
#include <aliceVision/photometricStereo/photometricStereo.hpp>


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

    std::string inputPath;
    std::string outputPath;
    std::string pathToLightData;
    size_t HS_order;

    po::options_description allParams("AliceVision photometricStereo");
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
    ("inputPath,i", po::value<std::string>(&inputPath)->required(), "Path to input; could be SfMData file or folder with pictures")
    ("pathToLightData,l", po::value<std::string>(&pathToLightData)->required(), "Path to input; could be SfMData file or folder with pictures")
    ("HSOrder,h", po::value<size_t>(&HS_order)->default_value(0), "HS order, 0 = directional, 1 = directional + ambiant")
    ("outputPath,o", po::value<std::string>(&outputPath)->default_value(""), "output path");

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

    aliceVision::image::Image<aliceVision::image::RGBfColor> normalsIm;
    aliceVision::image::Image<aliceVision::image::RGBfColor> albedoIm;
    
    if(boost::filesystem::is_directory(inputPath))
    {
        photometricStereo(inputPath, pathToLightData, outputPath, HS_order, normalsIm, albedoIm);
    }
    else
    {
      aliceVision::sfmData::SfMData sfmData;
      if(!aliceVision::sfmDataIO::Load(sfmData, inputPath, aliceVision::sfmDataIO::ESfMData(aliceVision::sfmDataIO::VIEWS|aliceVision::sfmDataIO::INTRINSICS)))
      {
          ALICEVISION_LOG_ERROR("The input file '" + inputPath + "' cannot be read");
          return EXIT_FAILURE;
      }

      photometricStereo(sfmData, pathToLightData, outputPath, HS_order, normalsIm, albedoIm);

    }

    return 0;
}
