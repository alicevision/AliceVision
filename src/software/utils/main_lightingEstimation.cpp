// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/mvsUtils/MultiViewParams.hpp>
#include <aliceVision/mvsUtils/fileIO.hpp>
#include <aliceVision/lightingEstimation/lightingEstimation.hpp>
#include <aliceVision/image/io.hpp>

#include <boost/program_options.hpp> 
#include <boost/filesystem.hpp>

#include <string>
#include <vector>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;
namespace fs = boost::filesystem;

int main(int argc, char** argv)
{
  system::Timer timer;

  // command-line parameters

  std::string verboseLevel = aliceVision::system::EVerboseLevel_enumToString(aliceVision::system::Logger::getDefaultVerboseLevel());
  std::string sfmDataFilename;
  std::string depthMapsFilterFolder;
  std::string imagesFolder;
  std::string outputFolder;

  po::options_description allParams("AliceVision lighthingEstimation");

  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
      ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
        "SfMData file.")
      ("depthMapsFilterFolder", po::value<std::string>(&depthMapsFilterFolder)->required(),
        "Filtered depth maps folder.")
      ("imagesFolder", po::value<std::string>(&imagesFolder)->required(),
        "Images used for depth map computation.\n"
        "Filename should be the image uid.")
      ("output,o", po::value<std::string>(&outputFolder)->required(),
        "Folder for output lighting vector files.");

  //po::options_description optionalParams("Optional parameters");
  //optionalParams.add_options();
  
  po::options_description logParams("Log parameters");
  logParams.add_options()
    ("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
      "verbosity level (fatal, error, warning, info, debug, trace).");

  allParams.add(requiredParams).add(logParams); //.add(optionalParams)

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

  // set verbose level
  aliceVision::system::Logger::get()->setLogLevel(verboseLevel);

  // read the input SfM scene
  sfmData::SfMData sfmData;
  if(!sfmDataIO::Load(sfmData, sfmDataFilename, sfmDataIO::ESfMData::ALL))
  {
    ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFilename << "' cannot be read.");
    return EXIT_FAILURE;
  }

  // initialization
  mvsUtils::MultiViewParams mp(sfmData, imagesFolder, "", depthMapsFilterFolder, false);

  for(const auto& viewPair : sfmData.getViews())
  {
    const IndexT viewId = viewPair.first;

    const std::string picturePath = mp.getImagePath(mp.getIndexFromViewId(viewId));
    const std::string normalsPath = mvsUtils::getFileNameFromViewId(&mp, viewId, mvsUtils::EFileType::normalMap, 0);

    lightingEstimation::LightingVector shl;
    image::Image<image::RGBfColor> picture;
    image::Image<image::RGBfColor> normals;

    image::readImage(picturePath, picture);
    image::readImage(normalsPath, normals);

    lightingEstimation::estimateLigthing(shl, picture, picture, normals);

    std::ofstream file((fs::path(outputFolder) / (std::to_string(viewId) + ".shl")).string());
    if(file.is_open())
      file << shl;
  }
          
  ALICEVISION_LOG_INFO("Task done in (s): " + std::to_string(timer.elapsed()));
  return EXIT_SUCCESS;
}
