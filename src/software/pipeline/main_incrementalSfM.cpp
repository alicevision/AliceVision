// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/feature/imageDescriberCommon.hpp>
#include <aliceVision/sfm/sfm.hpp>
#include <aliceVision/sfm/pipeline/regionsIO.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>

#include <dependencies/stlplus3/filesystemSimplified/file_system.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <cstdlib>

using namespace aliceVision;
using namespace aliceVision::camera;
using namespace aliceVision::sfm;
using namespace aliceVision::feature;
using namespace std;
namespace po = boost::program_options;

/**
 * @brief Retrieve the view id in the sfmData from the image filename.
 *
 * @param[in] sfm_data the SfM scene
 * @param[in] initialName the image name to find (filename or path)
 * @param[out] out_viewId the id found
 * @return if a view is found
 */
bool retrieveViewIdFromImageName(
  const SfMData & sfm_data,
  const std::string& initialName,
  IndexT& out_viewId)
{
  out_viewId = UndefinedIndexT;

  bool isName = (initialName == stlplus::filename_part(initialName));
  
  /// List views filenames and find the one that correspond to the user ones:
  for(Views::const_iterator it = sfm_data.GetViews().begin();
    it != sfm_data.GetViews().end(); ++it)
  {
    const View * v = it->second.get();
    std::string filename;
    
    if(isName)
      filename = stlplus::filename_part(v->getImagePath());
    else if(stlplus::is_full_path(v->getImagePath()))
      filename = v->getImagePath();
    else
      filename = sfm_data.s_root_path + v->getImagePath();
    
    if (filename == initialName)
    {
      if(out_viewId == UndefinedIndexT)
          out_viewId = v->getViewId();
      else
        std::cout<<"Error: Two pictures named :" << initialName << " !" << std::endl;
    }
  }
  return out_viewId != UndefinedIndexT;
}


int main(int argc, char **argv)
{
  // command-line parameters

  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  std::string sfmDataFilename;
  std::string featuresFolder;
  std::string matchesFolder;
  std::string outputSfM;

  // user optional parameters

  std::string outputSfMViewsAndPoses;
  std::string extraInfoFolder;
  std::string describerTypesName = feature::EImageDescriberType_enumToString(feature::EImageDescriberType::SIFT);
  std::string outInterFileExtension = ".ply";
  std::pair<std::string,std::string> initialPairString("","");
  int minInputTrackLength = 2;
  int maxNbMatches = 0;
  int userCameraModel = static_cast<int>(PINHOLE_CAMERA_RADIAL3);
  bool refineIntrinsics = true;
  bool allowUserInteraction = true;
  bool useLocalBundleAdjustment = false;
  std::size_t localBundelAdjustementGraphDistanceLimit = 1;

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
    ("featuresFolder,f", po::value<std::string>(&featuresFolder)->required(),
      "Path to a folder containing the extracted features.")
    ("matchesFolder,m", po::value<std::string>(&matchesFolder)->required(),
      "Path to a folder in which computed matches are stored.");

  po::options_description optionalParams("Optional parameters");
  optionalParams.add_options()
    ("outputViewsAndPoses", po::value<std::string>(&outputSfMViewsAndPoses)->default_value(outputSfMViewsAndPoses),
      "Path to the output SfMData (with only views and poses) file")
    ("extraInfoFolder", po::value<std::string>(&extraInfoFolder)->default_value(extraInfoFolder),
      "Folder for intermediate reconstruction files and additional reconstruction information files.")
    ("describerTypes,d", po::value<std::string>(&describerTypesName)->default_value(describerTypesName),
      feature::EImageDescriberType_informations().c_str())
    ("interFileExtension", po::value<std::string>(&outInterFileExtension)->default_value(outInterFileExtension),
      "Extension of the intermediate file export.")
    ("minInputTrackLength", po::value<int>(&minInputTrackLength)->default_value(minInputTrackLength),
      "Minimum track length in input of SfM")
    ("maxNumberOfMatches", po::value<int>(&maxNbMatches)->default_value(maxNbMatches),
      "Maximum number of matches per image pair (and per feature type). "
      "This can be useful to have a quick reconstruction overview. 0 means no limit.")
    ("cameraModel", po::value<int>(&userCameraModel)->default_value(userCameraModel),
      "* 1: Pinhole\n"
      "* 2: Pinhole radial 1\n"
      "* 3: Pinhole radial 3")
    ("initialPairA", po::value<std::string>(&initialPairString.first)->default_value(initialPairString.first),
      "filename of the first image (without path).")
    ("initialPairB", po::value<std::string>(&initialPairString.second)->default_value(initialPairString.second),
      "filename of the second image (without path).")
    ("refineIntrinsics", po::value<bool>(&refineIntrinsics)->default_value(refineIntrinsics),
      "Refine intrinsic parameters.")
    ("allowUserInteraction", po::value<bool>(&allowUserInteraction)->default_value(allowUserInteraction),
      "Enable/Disable user interactions.\n"
      "If the process is done on renderfarm, it doesn't make sense to wait for user inputs")
    ("useLocalBA,l", po::value<bool>(&useLocalBundleAdjustment)->default_value(useLocalBundleAdjustment),
      "Enable/Disable the Local bundle adjustment strategy.\n"
      "It reduces the reconstruction time, especially for big datasets (500+ images).\n")
    ("localBAGraphDistance", po::value<std::size_t>(&localBundelAdjustementGraphDistanceLimit)->default_value(localBundelAdjustementGraphDistanceLimit),
      "Graph-distance limit setting the Active region in the Local Bundle Adjustment strategy (by default: 1).\n");

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

  // check output SfM path
  if(outputSfM.empty())
  {
    ALICEVISION_LOG_ERROR("Error: Invalid output SfMData file path.");
    return EXIT_FAILURE;
  }

  // Check feature folder
  if(featuresFolder.empty()) {
    featuresFolder = matchesFolder;
  }

  // Load input SfMData scene
  SfMData sfmData;
  if(!Load(sfmData, sfmDataFilename, ESfMData(ALL))) {
    ALICEVISION_LOG_ERROR("Error: The input SfMData file '" + sfmDataFilename + "' cannot be read.");
    return EXIT_FAILURE;
  }

  // Get imageDescriberMethodType
  const std::vector<feature::EImageDescriberType> describerTypes = feature::EImageDescriberType_stringToEnums(describerTypesName);

  // Features reading
  feature::FeaturesPerView featuresPerView;
  if(!sfm::loadFeaturesPerView(featuresPerView, sfmData, featuresFolder, describerTypes))
  {
    ALICEVISION_LOG_ERROR("Error: Invalid features.");
    return EXIT_FAILURE;
  }
  
  // Matches reading
  matching::PairwiseMatches pairwiseMatches;

  if(!loadPairwiseMatches(pairwiseMatches, sfmData, matchesFolder, describerTypes, "f", maxNbMatches))
  {
    ALICEVISION_LOG_ERROR("Error: Unable to load matches file from '" + matchesFolder + "'.");
    return EXIT_FAILURE;
  }

  if(extraInfoFolder.empty())
  {
    namespace bfs = boost::filesystem;
    extraInfoFolder = bfs::path(outputSfM).parent_path().string();
  }

  if (!stlplus::folder_exists(extraInfoFolder))
    stlplus::folder_create(extraInfoFolder);

  // Sequential reconstruction process
  
  aliceVision::system::Timer timer;
  ReconstructionEngine_sequentialSfM sfmEngine(
    sfmData,
    extraInfoFolder,
    stlplus::create_filespec(extraInfoFolder, "sfm_log.html"));

  // Configure the featuresPerView & the matches_provider
  sfmEngine.setFeatures(&featuresPerView);
  sfmEngine.setMatches(&pairwiseMatches);

  // Configure reconstruction parameters
  sfmEngine.Set_bFixedIntrinsics(!refineIntrinsics);
  sfmEngine.SetUnknownCameraType(EINTRINSIC(userCameraModel));
  sfmEngine.setMinInputTrackLength(minInputTrackLength);
  sfmEngine.setSfmdataInterFileExtension(outInterFileExtension);
  sfmEngine.setAllowUserInteraction(allowUserInteraction);
  sfmEngine.setUseLocalBundleAdjustmentStrategy(useLocalBundleAdjustment);
  sfmEngine.setLocalBundleAdjustmentGraphDistance(localBundelAdjustementGraphDistanceLimit);

  // Handle Initial pair parameter
  if(!initialPairString.first.empty() && !initialPairString.second.empty())
  {
    if(initialPairString.first == initialPairString.second)
    {
      ALICEVISION_LOG_ERROR("Error: Invalid image names. You cannot use the same image to initialize a pair.");
      return EXIT_FAILURE;
    }

    Pair initialPairIndex;
    if(!retrieveViewIdFromImageName(sfmData, initialPairString.first, initialPairIndex.first)
            || !retrieveViewIdFromImageName(sfmData, initialPairString.second, initialPairIndex.second))
    {
      ALICEVISION_LOG_ERROR("Could not find the initial pairs (" + initialPairString.first + ", " + initialPairString.second + ") !");
      return EXIT_FAILURE;
    }
    sfmEngine.setInitialPair(initialPairIndex);
  }

  if(!sfmEngine.Process())
    return EXIT_FAILURE;

  // Get the color for the 3D points
  if(!sfmEngine.Colorize())
    ALICEVISION_LOG_ERROR("Error: Colorize failed !");

  sfmEngine.Get_SfMData().setFeatureFolder(featuresFolder);
  sfmEngine.Get_SfMData().setMatchingFolder(matchesFolder);

  ALICEVISION_LOG_INFO("Structure from motion took (s): " + std::to_string(timer.elapsed()));
  ALICEVISION_LOG_INFO("Generating HTML report...");

  Generate_SfM_Report(sfmEngine.Get_SfMData(),
    stlplus::create_filespec(extraInfoFolder, "sfm_report.html"));

  // Export to disk computed scene (data & visualizable results)
  ALICEVISION_LOG_INFO("Export SfMData to disk:" + outputSfM);

  Save(sfmEngine.Get_SfMData(), stlplus::create_filespec(extraInfoFolder, "cloud_and_poses", outInterFileExtension), ESfMData(VIEWS | EXTRINSICS | INTRINSICS | STRUCTURE));
  Save(sfmEngine.Get_SfMData(), outputSfM, ESfMData(ALL));

  if(!outputSfMViewsAndPoses.empty())
    Save(sfmEngine.Get_SfMData(), outputSfMViewsAndPoses, ESfMData(VIEWS | EXTRINSICS | INTRINSICS));

  return EXIT_SUCCESS;
}
