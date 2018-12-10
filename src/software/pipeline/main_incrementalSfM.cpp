// This file is part of the AliceVision project.
// Copyright (c) 2015 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/sfm/sfm.hpp>
#include <aliceVision/sfm/pipeline/regionsIO.hpp>
#include <aliceVision/feature/imageDescriberCommon.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/types.hpp>
#include <aliceVision/config.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <cstdlib>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 2
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;
namespace fs = boost::filesystem;

/**
 * @brief Retrieve the view id in the sfmData from the image filename.
 * @param[in] sfmData the SfM scene
 * @param[in] name the image name to find (uid or filename or path)
 * @param[out] out_viewId the id found
 * @return if a view is found
 */
bool retrieveViewIdFromImageName(const sfmData::SfMData& sfmData,
                                 const std::string& name,
                                 IndexT& out_viewId)
{
  out_viewId = UndefinedIndexT;

  // list views uid / filenames and find the one that correspond to the user ones
  for(const auto& viewPair : sfmData.getViews())
  {
    const sfmData::View& v = *(viewPair.second.get());
    
    if(name == std::to_string(v.getViewId()) ||
       name == fs::path(v.getImagePath()).filename().string() ||
       name == v.getImagePath())
    {
      out_viewId = v.getViewId();
      break;
    }
  }

  if(out_viewId == UndefinedIndexT)
    ALICEVISION_LOG_ERROR("Can't find the given initial pair view: " << name);

  return out_viewId != UndefinedIndexT;
}


int main(int argc, char **argv)
{
  // command-line parameters

  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  std::string sfmDataFilename;
  std::vector<std::string> featuresFolders;
  std::vector<std::string> matchesFolders;
  std::string outputSfM;

  // user optional parameters

  std::string outputSfMViewsAndPoses;
  std::string extraInfoFolder;
  std::string describerTypesName = feature::EImageDescriberType_enumToString(feature::EImageDescriberType::SIFT);
  std::string outInterFileExtension = ".ply";
  std::pair<std::string,std::string> initialPairString("","");
  int maxNbMatches = 0;
  int minInputTrackLength = 2;
  std::size_t minNbObservationsForTriangulation = 2;
  double minAngleForTriangulation = 3.0;
  double minAngleForLandmark = 2.0;
  double maxReprojectionError = 4.0;
  float minAngleInitialPair = 5.0f;
  float maxAngleInitialPair = 40.0f;
  bool refineIntrinsics = true;
  bool useLocalBundleAdjustment = false;
  bool useOnlyMatchesFromInputFolder = false;
  bool useTrackFiltering = true;
  bool useRigsCalibration = true;
  bool lockScenePreviouslyReconstructed = true;
  std::size_t localBundelAdjustementGraphDistanceLimit = 1;
  std::string localizerEstimatorName = robustEstimation::ERobustEstimator_enumToString(robustEstimation::ERobustEstimator::ACRANSAC);

  po::options_description allParams(
    "Sequential/Incremental reconstruction\n"
    "Perform incremental SfM (Initial Pair Essential + Resection)\n"
    "AliceVision incrementalSfM");

  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
      "SfMData file.")
    ("output,o", po::value<std::string>(&outputSfM)->required(),
      "Path to the output SfMData file.")
    ("featuresFolders,f", po::value<std::vector<std::string>>(&featuresFolders)->multitoken()->required(),
      "Path to folder(s) containing the extracted features.")
    ("matchesFolders,m", po::value<std::vector<std::string>>(&matchesFolders)->multitoken()->required(),
      "Path to folder(s) in which computed matches are stored.");

  po::options_description optionalParams("Optional parameters");
  optionalParams.add_options()
    ("outputViewsAndPoses", po::value<std::string>(&outputSfMViewsAndPoses)->default_value(outputSfMViewsAndPoses),
      "Path to the output SfMData file (with only views and poses).")
    ("extraInfoFolder", po::value<std::string>(&extraInfoFolder)->default_value(extraInfoFolder),
      "Folder for intermediate reconstruction files and additional reconstruction information files.")
    ("describerTypes,d", po::value<std::string>(&describerTypesName)->default_value(describerTypesName),
      feature::EImageDescriberType_informations().c_str())
    ("interFileExtension", po::value<std::string>(&outInterFileExtension)->default_value(outInterFileExtension),
      "Extension of the intermediate file export.")
    ("maxNumberOfMatches", po::value<int>(&maxNbMatches)->default_value(maxNbMatches),
      "Maximum number of matches per image pair (and per feature type). "
      "This can be useful to have a quick reconstruction overview. 0 means no limit.")
    ("minInputTrackLength", po::value<int>(&minInputTrackLength)->default_value(minInputTrackLength),
      "Minimum track length in input of SfM.")
    ("minAngleForTriangulation", po::value<double>(&minAngleForTriangulation)->default_value(minAngleForTriangulation),
      "Minimum angle for triangulation.")
    ("minAngleForLandmark", po::value<double>(&minAngleForLandmark)->default_value(minAngleForLandmark),
      "Minimum angle for landmark.")
    ("maxReprojectionError", po::value<double>(&maxReprojectionError)->default_value(maxReprojectionError),
      "Maximum reprojection error.")
    ("minAngleInitialPair", po::value<float>(&minAngleInitialPair)->default_value(minAngleInitialPair),
      "Minimum angle for the initial pair.")
    ("maxAngleInitialPair", po::value<float>(&maxAngleInitialPair)->default_value(maxAngleInitialPair),
      "Maximum angle for the initial pair.")
    ("minNumberOfObservationsForTriangulation", po::value<std::size_t>(&minNbObservationsForTriangulation)->default_value(minNbObservationsForTriangulation),
      "Minimum number of observations to triangulate a point.\n"
      "Set it to 3 (or more) reduces drastically the noise in the point cloud, but the number of final poses is a little bit reduced (from 1.5% to 11% on the tested datasets).\n"
      "Note: set it to 0 or 1 to use the old triangulation algorithm (using 2 views only) during resection.")
    ("initialPairA", po::value<std::string>(&initialPairString.first)->default_value(initialPairString.first),
      "UID or filepath or filename of the first image.")
    ("initialPairB", po::value<std::string>(&initialPairString.second)->default_value(initialPairString.second),
      "UID or filepath or filename of the second image.")
    ("refineIntrinsics", po::value<bool>(&refineIntrinsics)->default_value(refineIntrinsics),
      "Refine intrinsic parameters.")
    ("useLocalBA,l", po::value<bool>(&useLocalBundleAdjustment)->default_value(useLocalBundleAdjustment),
      "Enable/Disable the Local bundle adjustment strategy.\n"
      "It reduces the reconstruction time, especially for big datasets (500+ images).")
    ("localBAGraphDistance", po::value<std::size_t>(&localBundelAdjustementGraphDistanceLimit)->default_value(localBundelAdjustementGraphDistanceLimit),
      "Graph-distance limit setting the Active region in the Local Bundle Adjustment strategy.")
    ("localizerEstimator", po::value<std::string>(&localizerEstimatorName)->default_value(localizerEstimatorName),
      "Estimator type used to localize cameras (acransac (default), ransac, lsmeds, loransac, maxconsensus)")
    ("useOnlyMatchesFromInputFolder", po::value<bool>(&useOnlyMatchesFromInputFolder)->default_value(useOnlyMatchesFromInputFolder),
      "Use only matches from the input matchesFolder parameter.\n"
      "Matches folders previously added to the SfMData file will be ignored.")
    ("useTrackFiltering", po::value<bool>(&useTrackFiltering)->default_value(useTrackFiltering),
      "Enable/Disable the track filtering.\n")
    ("useRigsCalibration", po::value<bool>(&useRigsCalibration)->default_value(useRigsCalibration),
      "Enable/Disable rigs calibration.\n")
    ("lockScenePreviouslyReconstructed", po::value<bool>(&lockScenePreviouslyReconstructed)->default_value(lockScenePreviouslyReconstructed),
      "Lock/Unlock scene previously reconstructed.\n");

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

  // load input SfMData scene
  sfmData::SfMData sfmData;
  if(!sfmDataIO::Load(sfmData, sfmDataFilename, sfmDataIO::ESfMData::ALL))
  {
    ALICEVISION_LOG_ERROR("The input SfMData file '" + sfmDataFilename + "' cannot be read.");
    return EXIT_FAILURE;
  }

  // lock scene previously reconstructed
  if(lockScenePreviouslyReconstructed)
  {
    // lock all reconstructed camera poses
    for(auto& cameraPosePair : sfmData.getPoses())
      cameraPosePair.second.lock();

    for(const auto& viewPair : sfmData.getViews())
    {
      // lock all reconstructed views intrinsics
      const sfmData::View& view = *(viewPair.second);
      if(sfmData.isPoseAndIntrinsicDefined(&view))
        sfmData.getIntrinsics().at(view.getIntrinsicId())->lock();
    }
  }

  // get imageDescriber type
  const std::vector<feature::EImageDescriberType> describerTypes = feature::EImageDescriberType_stringToEnums(describerTypesName);

  // features reading
  feature::FeaturesPerView featuresPerView;
  if(!sfm::loadFeaturesPerView(featuresPerView, sfmData, featuresFolders, describerTypes))
  {
    ALICEVISION_LOG_ERROR("Invalid features.");
    return EXIT_FAILURE;
  }
  
  // matches reading
  matching::PairwiseMatches pairwiseMatches;
  if(!sfm::loadPairwiseMatches(pairwiseMatches, sfmData, matchesFolders, describerTypes, maxNbMatches, useOnlyMatchesFromInputFolder))
  {
    ALICEVISION_LOG_ERROR("Unable to load matches.");
    return EXIT_FAILURE;
  }

  if(extraInfoFolder.empty())
    extraInfoFolder = fs::path(outputSfM).parent_path().string();

  if (!fs::exists(extraInfoFolder))
    fs::create_directory(extraInfoFolder);

  // sequential reconstruction process
  aliceVision::system::Timer timer;
  sfm::ReconstructionEngine_sequentialSfM sfmEngine(
    sfmData,
    extraInfoFolder,
    (fs::path(extraInfoFolder) / "sfm_log.html").string());

  // configure the featuresPerView & the matches_provider
  sfmEngine.setFeatures(&featuresPerView);
  sfmEngine.setMatches(&pairwiseMatches);

  // configure reconstruction parameters
  sfmEngine.setFixedIntrinsics(!refineIntrinsics);
  sfmEngine.setMinInputTrackLength(minInputTrackLength);
  sfmEngine.setMinAngleForTriangulation(minAngleForTriangulation);
  sfmEngine.setMinAngleForLandmark(minAngleForLandmark);
  sfmEngine.setMaxReprojectionError(maxReprojectionError);
  sfmEngine.setMinAngleInitialPair(minAngleInitialPair);
  sfmEngine.setMaxAngleInitialPair(maxAngleInitialPair);
  sfmEngine.setIntermediateFileExtension(outInterFileExtension);
  sfmEngine.setUseLocalBundleAdjustmentStrategy(useLocalBundleAdjustment);
  sfmEngine.setLocalBundleAdjustmentGraphDistance(localBundelAdjustementGraphDistanceLimit);
  sfmEngine.setLocalizerEstimator(robustEstimation::ERobustEstimator_stringToEnum(localizerEstimatorName));
  sfmEngine.useTrackFiltering(useTrackFiltering);
  sfmEngine.useRigsCalibration(useRigsCalibration);

  if(minNbObservationsForTriangulation < 2)
  {
      // allows to use to the old triangulatation algorithm (using 2 views only) during resection.
      minNbObservationsForTriangulation = 0;
      // ALICEVISION_LOG_ERROR("The value associated to the argument '--minNbObservationsForTriangulation' must be >= 2 ");
      // return EXIT_FAILURE;
  }
  sfmEngine.setNbOfObservationsForTriangulation(minNbObservationsForTriangulation);

  // handle Initial pair parameter
  if(!initialPairString.first.empty() && !initialPairString.second.empty())
  {
    if(initialPairString.first == initialPairString.second)
    {
      ALICEVISION_LOG_ERROR("Invalid image names. You cannot use the same image to initialize a pair.");
      return EXIT_FAILURE;
    }

    Pair initialPairIndex;
    if(!retrieveViewIdFromImageName(sfmData, initialPairString.first, initialPairIndex.first) ||
       !retrieveViewIdFromImageName(sfmData, initialPairString.second, initialPairIndex.second))
    {
      ALICEVISION_LOG_ERROR("Could not find the initial pairs (" + initialPairString.first + ", " + initialPairString.second + ") !");
      return EXIT_FAILURE;
    }

    sfmEngine.setInitialPair(initialPairIndex);
  }

  if(!sfmEngine.process())
    return EXIT_FAILURE;

  // get the color for the 3D points
  if(!sfmEngine.colorize())
    ALICEVISION_LOG_ERROR("SfM Colorization failed.");

  // set featuresFolders and matchesFolders relative paths
  {
    const fs::path sfmFolder = fs::path(outputSfM).remove_filename();

    for(const std::string& featuresFolder : featuresFolders)
       sfmEngine.getSfMData().addFeaturesFolder(fs::relative(fs::path(featuresFolder), sfmFolder).string());

    for(const std::string& matchesFolder : matchesFolders)
       sfmEngine.getSfMData().addMatchesFolder(fs::relative(fs::path(matchesFolder), sfmFolder).string());

    sfmEngine.getSfMData().setAbsolutePath(outputSfM);
  }

  ALICEVISION_LOG_INFO("Structure from motion took (s): " + std::to_string(timer.elapsed()));
  ALICEVISION_LOG_INFO("Generating HTML report...");

  sfm::generateSfMReport(sfmEngine.getSfMData(), (fs::path(extraInfoFolder) / "sfm_report.html").string());

  // export to disk computed scene (data & visualizable results)
  ALICEVISION_LOG_INFO("Export SfMData to disk: " + outputSfM);

  sfmDataIO::Save(sfmEngine.getSfMData(), (fs::path(extraInfoFolder) / ("cloud_and_poses" + outInterFileExtension)).string(), sfmDataIO::ESfMData(sfmDataIO::VIEWS|sfmDataIO::EXTRINSICS|sfmDataIO::INTRINSICS|sfmDataIO::STRUCTURE));
  sfmDataIO::Save(sfmEngine.getSfMData(), outputSfM, sfmDataIO::ESfMData::ALL);

  if(!outputSfMViewsAndPoses.empty())
   sfmDataIO:: Save(sfmEngine.getSfMData(), outputSfMViewsAndPoses, sfmDataIO::ESfMData(sfmDataIO::VIEWS|sfmDataIO::EXTRINSICS|sfmDataIO::INTRINSICS));

  ALICEVISION_LOG_INFO("Structure from Motion results:" << std::endl
    << "\t- # input images: " << sfmEngine.getSfMData().getViews().size() << std::endl
    << "\t- # cameras calibrated: " << sfmEngine.getSfMData().getValidViews().size() << std::endl
    << "\t- # poses: " << sfmEngine.getSfMData().getPoses().size() << std::endl
    << "\t- # landmarks: " << sfmEngine.getSfMData().getLandmarks().size());

  return EXIT_SUCCESS;
}
