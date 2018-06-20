// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfm/pipeline/regionsIO.hpp>
#include <aliceVision/feature/imageDescriberCommon.hpp>
#include <aliceVision/sfm/pipeline/global/ReconstructionEngine_globalSfM.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <cstdlib>

using namespace aliceVision;
using namespace aliceVision::sfm;
using namespace aliceVision::feature;
using namespace std;

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
  int rotationAveragingMethod = static_cast<int>(ROTATION_AVERAGING_L2);
  int translationAveragingMethod = static_cast<int>(TRANSLATION_AVERAGING_SOFTL1);
  bool refineIntrinsics = true;

  po::options_description allParams("Implementation of the paper\n"
    "\"Global Fusion of Relative Motions for "
    "Robust, Accurate and Scalable Structure from Motion.\"\n"
    "Pierre Moulon, Pascal Monasse and Renaud Marlet ICCV 2013.\n"
    "AliceVision globalSfM");

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
    ("rotationAveraging", po::value<int>(&rotationAveragingMethod)->default_value(rotationAveragingMethod),
      "* 1: L1 minimization\n"
      "* 2: L2 minimization")
    ("translationAveraging", po::value<int>(&translationAveragingMethod)->default_value(translationAveragingMethod),
      "* 1: L1 minimization\n"
      "* 2: L2 minimization of sum of squared Chordal distances")
    ("refineIntrinsics", po::value<bool>(&refineIntrinsics)->default_value(refineIntrinsics),
      "Refine intrinsic parameters.");

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

  if (rotationAveragingMethod < ROTATION_AVERAGING_L1 ||
      rotationAveragingMethod > ROTATION_AVERAGING_L2 )
  {
    ALICEVISION_LOG_ERROR("Rotation averaging method is invalid");
    return EXIT_FAILURE;
  }

  if (translationAveragingMethod < TRANSLATION_AVERAGING_L1 ||
      translationAveragingMethod > TRANSLATION_AVERAGING_SOFTL1 )
  {
    ALICEVISION_LOG_ERROR("Translation averaging method is invalid");
    return EXIT_FAILURE;
  }

  // load input SfMData scene
  SfMData sfmData;
  if (!Load(sfmData, sfmDataFilename, ESfMData(VIEWS|INTRINSICS)))
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
  FeaturesPerView featuresPerView;
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

  if (outDirectory.empty())
  {
    ALICEVISION_LOG_ERROR("It is an invalid output folder");
    return EXIT_FAILURE;
  }

  if (!fs::exists(outDirectory))
    fs::create_directory(outDirectory);

  // global SfM reconstruction process
  aliceVision::system::Timer timer;
  ReconstructionEngine_globalSfM sfmEngine(
    sfmData,
    outDirectory,
    (fs::path(outDirectory) / "sfm_log.html").string());

  // configure the featuresPerView & the matches_provider
  sfmEngine.SetFeaturesProvider(&featuresPerView);
  sfmEngine.SetMatchesProvider(&pairwiseMatches);

  // configure reconstruction parameters
  sfmEngine.setFixedIntrinsics(!refineIntrinsics);

  // configure motion averaging method
  sfmEngine.SetRotationAveragingMethod(ERotationAveragingMethod(rotationAveragingMethod));
  sfmEngine.SetTranslationAveragingMethod(ETranslationAveragingMethod(translationAveragingMethod));

  if(!sfmEngine.process())
    return EXIT_FAILURE;

  // get the color for the 3D points
  if(!sfmEngine.colorize())
    ALICEVISION_LOG_ERROR("SfM Colorization failed.");

  // set featuresFolders and matchesFolders relative paths
  {
    for(const std::string& featuresFolder : featuresFolders)
       sfmEngine.getSfMData().addFeaturesFolder(fs::relative(fs::path(featuresFolder), outDirectory).string());

    for(const std::string& matchesFolder : matchesFolders)
       sfmEngine.getSfMData().addMatchesFolder(fs::relative(fs::path(matchesFolder), outDirectory).string());

    sfmEngine.getSfMData().setAbsolutePath(outDirectory);
  }

  ALICEVISION_LOG_INFO("Global structure from motion took (s): " << timer.elapsed());
  ALICEVISION_LOG_INFO("Generating HTML report...");

  generateSfMReport(sfmEngine.getSfMData(), (fs::path(outDirectory) / "sfm_report.html").string());

  // export to disk computed scene (data & visualizable results)
  ALICEVISION_LOG_INFO("Export SfMData to disk");

  Save(sfmEngine.getSfMData(), outSfMDataFilename, ESfMData::ALL);
  Save(sfmEngine.getSfMData(), (fs::path(outDirectory) / "cloud_and_poses.ply").string(), ESfMData::ALL);

  ALICEVISION_LOG_INFO("Structure from Motion results:" << std::endl
    << "\t- # input images: " << sfmEngine.getSfMData().getViews().size() << std::endl
    << "\t- # cameras calibrated: " << sfmEngine.getSfMData().getPoses().size() << std::endl
    << "\t- # landmarks: " << sfmEngine.getSfMData().getLandmarks().size());

  return EXIT_SUCCESS;
}
