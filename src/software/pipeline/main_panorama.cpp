// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/sfm/pipeline/regionsIO.hpp>
#include <aliceVision/feature/imageDescriberCommon.hpp>
#include <aliceVision/sfm/pipeline/panorama/ReconstructionEngine_panorama.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <cstdlib>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;
namespace fs = boost::filesystem;

int main(int argc, char **argv)
{
  // command-line parameters

  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  std::string sfmDataFilename;
  std::vector<std::string> featuresFolders;
  std::vector<std::string> matchesFolders;
  std::string outDirectory;

  // user optional parameters

  std::string outSfMDataFilename = "SfmData.json";
  std::string describerTypesName = feature::EImageDescriberType_enumToString(feature::EImageDescriberType::SIFT);
  sfm::ERotationAveragingMethod rotationAveragingMethod = sfm::ROTATION_AVERAGING_L2;
  sfm::ERelativeRotationMethod relativeRotationMethod = sfm::RELATIVE_ROTATION_FROM_E;
  bool lockAllIntrinsics = false;

  po::options_description allParams(
    "Perform detection of cameras for 360° panorama stitching\n"
    "Based on Global SfM method but with no translations between cameras\n"
    "AliceVision panorama");

  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
      "SfMData file.")
    ("output,o", po::value<std::string>(&outDirectory)->required(),
      "Path of the output folder.")
    ("featuresFolders,f", po::value<std::vector<std::string>>(&featuresFolders)->multitoken()->required(),
      "Path to folder(s) containing the extracted features.")
    ("matchesFolders,m", po::value<std::vector<std::string>>(&matchesFolders)->multitoken()->required(),
      "Path to folder(s) in which computed matches are stored.");

  po::options_description optionalParams("Optional parameters");
  optionalParams.add_options()
    ("outSfMDataFilename", po::value<std::string>(&outSfMDataFilename)->default_value(outSfMDataFilename),
      "Filename of the output SfMData file.")
    ("describerTypes,d", po::value<std::string>(&describerTypesName)->default_value(describerTypesName),
      feature::EImageDescriberType_informations().c_str())
    ("rotationAveraging", po::value<sfm::ERotationAveragingMethod>(&rotationAveragingMethod)->default_value(rotationAveragingMethod),
      "* 1: L1 minimization\n"
      "* 2: L2 minimization")
    ("relativeRotation", po::value<sfm::ERelativeRotationMethod>(&relativeRotationMethod)->default_value(relativeRotationMethod),
      "* from essential matrix"
      "* from homography matrix")
    ("lockAllIntrinsics", po::value<bool>(&lockAllIntrinsics)->default_value(lockAllIntrinsics),
      "Force lock of all camera intrinsic parameters, so they will not be refined during Bundle Adjustment.");

  po::options_description logParams("Log parameters");
  logParams.add_options()
    ("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
      "verbosity level (fatal, error, warning, info, debug, trace).");

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

  if (rotationAveragingMethod < sfm::ROTATION_AVERAGING_L1 ||
      rotationAveragingMethod > sfm::ROTATION_AVERAGING_L2 )
  {
    ALICEVISION_LOG_ERROR("Rotation averaging method is invalid");
    return EXIT_FAILURE;
  }

  if (relativeRotationMethod < sfm::RELATIVE_ROTATION_FROM_E ||
      relativeRotationMethod > sfm::RELATIVE_ROTATION_FROM_H )
  {
    ALICEVISION_LOG_ERROR("Relative rotation method is invalid");
    return EXIT_FAILURE;
  }

  // load input SfMData scene
  sfmData::SfMData sfmData;
  if(!sfmDataIO::Load(sfmData, sfmDataFilename, sfmDataIO::ESfMData(sfmDataIO::VIEWS|sfmDataIO::INTRINSICS)))
  {
    ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFilename << "' cannot be read.");
    return EXIT_FAILURE;
  }

  if(!sfmData.structure.empty())
  {
    ALICEVISION_LOG_ERROR("Part computed SfMData are not currently supported in Global SfM." << std::endl << "Please use Incremental SfM. Aborted");
    return EXIT_FAILURE;
  }

  if(!sfmData.getRigs().empty())
  {
    ALICEVISION_LOG_ERROR("Rigs are not currently supported in Global SfM." << std::endl << "Please use Incremental SfM. Aborted");
    return EXIT_FAILURE;
  }

  // get describerTypes
  const std::vector<feature::EImageDescriberType> describerTypes = feature::EImageDescriberType_stringToEnums(describerTypesName);

  // features reading
  feature::FeaturesPerView featuresPerView;
  if(!sfm::loadFeaturesPerView(featuresPerView, sfmData, featuresFolders, describerTypes))
  {
    ALICEVISION_LOG_ERROR("Invalid features");
    return EXIT_FAILURE;
  }

  // matches reading
  // Load the match file (try to read the two matches file formats).
  matching::PairwiseMatches pairwiseMatches;
  if(!sfm::loadPairwiseMatches(pairwiseMatches, sfmData, matchesFolders, describerTypes))
  {
    ALICEVISION_LOG_ERROR("Unable to load matches files from: " << matchesFolders);
    return EXIT_FAILURE;
  }

  if(outDirectory.empty())
  {
    ALICEVISION_LOG_ERROR("It is an invalid output folder");
    return EXIT_FAILURE;
  }

  if(!fs::exists(outDirectory))
    fs::create_directory(outDirectory);

  // global SfM reconstruction process
  aliceVision::system::Timer timer;
  sfm::ReconstructionEngine_panorama sfmEngine(
    sfmData,
    outDirectory,
    (fs::path(outDirectory) / "sfm_log.html").string());

  // configure the featuresPerView & the matches_provider
  sfmEngine.SetFeaturesProvider(&featuresPerView);
  sfmEngine.SetMatchesProvider(&pairwiseMatches);

  // configure reconstruction parameters
  sfmEngine.setLockAllIntrinsics(lockAllIntrinsics); // TODO: rename param

  // configure motion averaging method
  sfmEngine.SetRotationAveragingMethod(sfm::ERotationAveragingMethod(rotationAveragingMethod));

  // configure relative rotation method (from essential or from homography matrix)
  sfmEngine.SetRelativeRotationMethod(sfm::ERelativeRotationMethod(relativeRotationMethod));

  if(!sfmEngine.process())
    return EXIT_FAILURE;

  // get the color for the 3D points
  sfmEngine.colorize();

  // set featuresFolders and matchesFolders relative paths
  {
    sfmEngine.getSfMData().addFeaturesFolders(featuresFolders);
    sfmEngine.getSfMData().addMatchesFolders(matchesFolders);
    sfmEngine.getSfMData().setAbsolutePath(outDirectory);
  }

  sfmData::SfMData& outSfmData = sfmEngine.getSfMData();

  ALICEVISION_LOG_INFO("Global structure from motion took (s): " << timer.elapsed());
  ALICEVISION_LOG_INFO("Generating HTML report...");

  sfm::generateSfMReport(outSfmData, (fs::path(outDirectory) / "sfm_report.html").string());

  // export to disk computed scene (data & visualizable results)
  ALICEVISION_LOG_INFO("Export SfMData to disk");

  sfmDataIO::Save(outSfmData, outSfMDataFilename, sfmDataIO::ESfMData::ALL);
  sfmDataIO::Save(outSfmData, (fs::path(outDirectory) / "cloud_and_poses.ply").string(), sfmDataIO::ESfMData::ALL);

  ALICEVISION_LOG_INFO("Structure from Motion results:" << std::endl
    << "\t- # input images: " << outSfmData.getViews().size() << std::endl
    << "\t- # cameras calibrated: " << outSfmData.getPoses().size() << std::endl
    << "\t- # landmarks: " << outSfmData.getLandmarks().size());

  /*
  // Create panorama buffer

  for(auto& viewIt: sfmData.getViews())
  {
    IndexT viewId = viewIt.first;
    const View& view = *viewIt.second.get();
    if(!sfmData.isPoseAndIntrinsicDefined(view))
      continue;

    const CameraPose camPose = sfmData.getPose(view);
    const camera::IntrinsicBase& intrinsic = *sfmData.getIntrinsicPtr(view.getIntrinsicId());

    std::string imagePath = view.getImagePath();
    // Image RGB(A)f

    for(int y = 0; y <; ++y)
    {
      for(int x = 0; x <; ++x)
      {
        // equirectangular to unit vector
        // unit vector to camera
        // is in camera frustrum?
        // camera to image (pixel coord with distortion)
        // add_disto + cam2ima
      }
    }
  }
  */

  return EXIT_SUCCESS;
}
