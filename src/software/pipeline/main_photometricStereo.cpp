#include <aliceVision/image/all.hpp>
#include <aliceVision/image/io.hpp>

// SFMData
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

// Photometric Stereo
#include <aliceVision/photometricStereo/photometricDataIO.hpp>

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

    std::string dataFolder;
    std::vector<int> usedPictures;

    po::options_description allParams("AliceVision photometricStereo");
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
    ("dataFolder,d", po::value<std::string>(&dataFolder)->required(), "Data folder")
    ("usedPictures,i", po::value<std::vector<int>>(&usedPictures)->required()->multitoken(), "usedPictures.");

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


    // Load light instensities
    std::vector<std::array<float, 3>> intList;
    const std::string intFileName = dataFolder + "light_intensities.txt";
    loadLightIntensities(intFileName, usedPictures, intList);

    // Load light directions
    Eigen::MatrixXf lightMat(3*usedPictures.size(), 3);
    const std::string dirFileName = dataFolder + "light_directions.txt";
    loadLightDirections(dirFileName, usedPictures, lightMat);

    // Read mask
    const std::string maskName = dataFolder + "mask.png";
    aliceVision::image::Image<float> mask;
    loadMask(maskName, mask);

    std::vector<int> indexes;
    getIndMask(mask, indexes);
    size_t maskSize = indexes.size();

    const int pictRows = mask.rows();
    const int pictCols = mask.cols();

    Eigen::MatrixXf currentPicture(3,pictRows*pictCols);
    Eigen::MatrixXf allPictures(3*usedPictures.size(), pictRows*pictCols);
    Eigen::MatrixXf imMat(3*usedPictures.size(), maskSize);

    std::string picturePath;
    std::string pictureName;

    // Read pictures
    for (size_t i = 0; i < usedPictures.size(); ++i)
    {
        std::ostringstream ss;
        ss << std::setw(3) << std::setfill('0') << usedPictures.at(i);
        pictureName = ss.str();

        picturePath = dataFolder + pictureName +".png";

        aliceVision::image::Image<aliceVision::image::RGBfColor> imageFloat;
        aliceVision::image::ImageReadOptions options;
        options.outputColorSpace = aliceVision::image::EImageColorSpace::SRGB;

        aliceVision::image::readImage(picturePath, imageFloat, options);
        intensityScaling(intList.at(i), imageFloat);

        image2PsMatrix(imageFloat, currentPicture);

        allPictures.block(3*i,0,3,allPictures.cols()) << currentPicture;
    }

    return 0;
}
