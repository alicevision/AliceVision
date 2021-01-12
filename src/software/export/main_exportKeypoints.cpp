// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/matching/IndMatch.hpp>
#include <aliceVision/matching/io.hpp>
#include <aliceVision/matching/svgVisualization.hpp>
#include <aliceVision/sfm/pipeline/regionsIO.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/image/all.hpp>

#include <dependencies/vectorGraphics/svgDrawer.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/progress.hpp>

#include <cstdlib>
#include <string>
#include <vector>
#include <fstream>
#include <map>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;
using namespace aliceVision::feature;
using namespace aliceVision::matching;
using namespace aliceVision::sfmData;
using namespace svg;

namespace po = boost::program_options;
namespace fs = boost::filesystem;

int aliceVision_main(int argc, char ** argv)
{
  // command-line parameters

  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  std::string sfmDataFilename;
  std::string outputFolder;
  std::vector<std::string> featuresFolders;

  // user optional parameters

  std::string describerTypesName = feature::EImageDescriberType_enumToString(feature::EImageDescriberType::SIFT);

  po::options_description allParams("AliceVision exportKeypoints");

  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
      "SfMData file.")
    ("output,o", po::value<std::string>(&outputFolder)->required(),
      "Output path for keypoints.")
    ("featuresFolders,f", po::value<std::vector<std::string>>(&featuresFolders)->multitoken()->required(),
      "Path to folder(s) containing the extracted features.");

  po::options_description optionalParams("Optional parameters");
  optionalParams.add_options()
    ("describerTypes,d", po::value<std::string>(&describerTypesName)->default_value(describerTypesName),
      feature::EImageDescriberType_informations().c_str());

  po::options_description logParams("Log parameters");
  logParams.add_options()
    ("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
      "verbosity level (fatal,  error, warning, info, debug, trace).");

  allParams.add(requiredParams).add(optionalParams).add(logParams);

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
  system::Logger::get()->setLogLevel(verboseLevel);

  if(outputFolder.empty())
  {
    ALICEVISION_LOG_ERROR("Invalid output folder");
    return EXIT_FAILURE;
  }

  // read SfM Scene (image view names)

  SfMData sfmData;
  if(!sfmDataIO::Load(sfmData, sfmDataFilename, sfmDataIO::ESfMData(sfmDataIO::VIEWS|sfmDataIO::INTRINSICS)))
  {
    ALICEVISION_LOG_ERROR("The input SfMData file '"<< sfmDataFilename << "' cannot be read.");
    return EXIT_FAILURE;
  }

  // load SfM Scene regions
  // get imageDescriberMethodType
  std::vector<EImageDescriberType> describerMethodTypes = EImageDescriberType_stringToEnums(describerTypesName);

  // read the features
  feature::FeaturesPerView featuresPerView;
  if(!sfm::loadFeaturesPerView(featuresPerView, sfmData, featuresFolders, describerMethodTypes))
  {
    ALICEVISION_LOG_ERROR("Invalid features");
    return EXIT_FAILURE;
  }

  // for each image, export visually the keypoints

  fs::create_directory(outputFolder);
  ALICEVISION_LOG_INFO("Export extracted keypoints for all images");
  boost::progress_display myProgressBar(sfmData.views.size());
  for(const auto &iterViews : sfmData.views)
  {
    const View * view = iterViews.second.get();
    const std::string viewImagePath = view->getImagePath();

    const std::pair<size_t, size_t>
      dimImage = std::make_pair(view->getWidth(), view->getHeight());

    const MapFeaturesPerDesc& features = featuresPerView.getData().at(view->getViewId());

    // output filename
    fs::path outputFilename = fs::path(outputFolder) / std::string(std::to_string(view->getViewId()) + "_" + std::to_string(features.size()) + ".svg");

    matching::saveFeatures2SVG(viewImagePath,
                               dimImage,
                               featuresPerView.getData().at(view->getViewId()),
                               outputFilename.string());

    ++myProgressBar;
  }
  
  return EXIT_SUCCESS;
}
