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
    bool isPerspective;
    std::string pathToDM;
    std::string pathToK;

    po::options_description allParams("AliceVision photometricStereo");
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
    ("inputPath,i", po::value<std::string>(&inputPath)->required(), "Path to input. Could be SfMData file or folder with pictures")
    ("pathToK,k", po::value<std::string>(&pathToK)->default_value(""), "pathToK.")
    ("isPerspective,c", po::value<bool>(&isPerspective)->default_value(false), "isPerspective")
    ("pathToDM,o", po::value<std::string>(&pathToDM)->default_value(""), "pathToDM.");

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
    
    photometricStero(inputPath, isPerspective, normalsIm);

    int pictCols = normalsIm.Width();
    int pictRows = normalsIm.Height();

    Eigen::MatrixXf K = Eigen::MatrixXf::Zero(3,3);
    readMatrix(pathToK, K);

    aliceVision::image::Image<float> depth(pictCols, pictRows);
    normalIntegration(normalsIm, depth, isPerspective, K);
    
    oiio::ParamValueList metadata;
    metadata.attribute("AliceVision:storageDataType", aliceVision::image::EStorageDataType_enumToString(aliceVision::image::EStorageDataType::Float));
    aliceVision::image::writeImage(pathToDM, depth, aliceVision::image::EImageColorSpace::NO_CONVERSION, metadata);


    return 0;
}
