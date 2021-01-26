// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/config.hpp>
#include <aliceVision/localization/ILocalizer.hpp>
#include <aliceVision/localization/VoctreeLocalizer.hpp>
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_CCTAG)
#include <aliceVision/localization/CCTagLocalizer.hpp>
#endif
#include <aliceVision/localization/LocalizationResult.hpp>
#include <aliceVision/localization/optimization.hpp>
#include <aliceVision/image/io.hpp>
#include <aliceVision/dataio/FeedProvider.hpp>
#include <aliceVision/feature/ImageDescriber.hpp>
#include <aliceVision/feature/imageDescriberCommon.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/robustEstimation/estimators.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/system/cmdline.hpp>

#include <boost/filesystem.hpp>
#include <boost/progress.hpp>
#include <boost/program_options.hpp> 
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/sum.hpp>

#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <memory>

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_ALEMBIC)
#include <aliceVision/sfmDataIO/AlembicExporter.hpp>
#endif // ALICEVISION_HAVE_ALEMBIC

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace bfs = boost::filesystem;
namespace bacc = boost::accumulators;
namespace po = boost::program_options;

std::string myToString(std::size_t i, std::size_t zeroPadding)
{
  std::stringstream ss;
  ss << std::setw(zeroPadding) << std::setfill('0') << i;
  return ss.str();
}

int aliceVision_main(int argc, char** argv)
{
  /// the calibration file
  std::string calibFile;
  /// the AliceVision .json data file
  std::string sfmFilePath;
  /// the folder containing the descriptors
  std::string descriptorsFolder;
  /// the media file to localize
  std::string mediaFilepath;
 
  /// the describer types name to use for the matching
  std::string matchDescTypeNames = feature::EImageDescriberType_enumToString(feature::EImageDescriberType::SIFT);
  /// the preset for the feature extractor
  feature::ConfigurationPreset featDescPreset;
  /// the describer types to use for the matching
  std::vector<feature::EImageDescriberType> matchDescTypes;
  /// the estimator to use for resection
  robustEstimation::ERobustEstimator resectionEstimator = robustEstimation::ERobustEstimator::ACRANSAC;
  /// the estimator to use for matching
  robustEstimation::ERobustEstimator matchingEstimator = robustEstimation::ERobustEstimator::ACRANSAC;
  /// the possible choices for the estimators as strings
  const std::string str_estimatorChoices = robustEstimation::ERobustEstimator_enumToString(robustEstimation::ERobustEstimator::ACRANSAC)
                                          +", "+robustEstimation::ERobustEstimator_enumToString(robustEstimation::ERobustEstimator::LORANSAC);
  bool refineIntrinsics = false;
  /// the maximum reprojection error allowed for resection
  double resectionErrorMax = 4.0;  
  /// the maximum reprojection error allowed for image matching with geometric validation
  double matchingErrorMax = 4.0;   
  /// whether to use the voctreeLocalizer or cctagLocalizer
  bool useVoctreeLocalizer = true;
  
  // voctree parameters
  std::string algostring = "AllResults";
  /// number of similar images to search when querying the voctree
  std::size_t numResults = 4;
  /// maximum number of successfully matched similar images
  std::size_t maxResults = 10;      
  std::size_t numCommonViews = 3;
  /// the vocabulary tree file
  std::string vocTreeFilepath;
  /// the vocabulary tree weights file
  std::string weightsFilepath;
  /// Number of previous frame of the sequence to use for matching
  std::size_t nbFrameBufferMatching = 10;
  /// enable/disable the robust matching (geometric validation) when matching query image
  /// and databases images
  bool robustMatching = true;
  
  /// the Alembic export file
  std::string exportAlembicFile = "trackedcameras.abc";
  /// the JSON export file
  std::string exportJsonFile = "";

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_CCTAG)
  // parameters for cctag localizer
  std::size_t nNearestKeyFrames = 5;   
#endif
  // parameters for the final bundle adjustment
  /// If !refineIntrinsics it can run a final global bundle to refine the scene
  bool globalBundle = false;
  /// It does not count the distortion
  bool noDistortion = false;
  /// It does not refine intrinsics during BA
  bool noBArefineIntrinsics = false;
  /// remove the points that does not have a minimum visibility over the sequence
  /// ie that are seen at least by minPointVisibility frames of the sequence
  std::size_t minPointVisibility = 0;
  
  /// whether to save visual debug info
  std::string visualDebug = "";
  int randomSeed = std::mt19937::default_seed;

  po::options_description allParams(
      "This program takes as input a media (image, image sequence, video) and a database (vocabulary tree, 3D scene data) \n"
      "and returns for each frame a pose estimation for the camera.");

  po::options_description inputParams("Required input parameters");
  
  inputParams.add_options()
      ("sfmdata", po::value<std::string>(&sfmFilePath)->required(), 
          "The sfm_data.json kind of file generated by AliceVision.")
      ("mediafile", po::value<std::string>(&mediaFilepath)->required(), 
          "The folder path or the filename for the media to track");
  
  po::options_description commonParams(
      "Common optional parameters for the localizer");
  commonParams.add_options()
      ("descriptorPath", po::value<std::string>(&descriptorsFolder),
          "Folder containing the descriptors for all the images (ie the *.desc.)")
      ("matchDescTypes", po::value<std::string>(&matchDescTypeNames)->default_value(matchDescTypeNames),
          "The describer types to use for the matching")
      ("preset", po::value<feature::EImageDescriberPreset>(&featDescPreset.descPreset)->default_value(featDescPreset.descPreset),
          "Preset for the feature extractor when localizing a new image "
          "{LOW,MEDIUM,NORMAL,HIGH,ULTRA}")
      ("resectionEstimator", po::value<robustEstimation::ERobustEstimator>(&resectionEstimator)->default_value(resectionEstimator),
          std::string("The type of *sac framework to use for resection "
          "("+str_estimatorChoices+")").c_str())
      ("matchingEstimator", po::value<robustEstimation::ERobustEstimator>(&matchingEstimator)->default_value(matchingEstimator),
          std::string("The type of *sac framework to use for matching "
          "("+str_estimatorChoices+")").c_str())
      ("calibration", po::value<std::string>(&calibFile)/*->required( )*/, 
          "Calibration file")
      ("refineIntrinsics", po::value<bool>(&refineIntrinsics), 
          "Enable/Disable camera intrinsics refinement for each localized image")
      ("reprojectionError", po::value<double>(&resectionErrorMax)->default_value(resectionErrorMax), 
          "Maximum reprojection error (in pixels) allowed for resectioning. If set "
          "to 0 it lets the ACRansac select an optimal value.")
      ("randomSeed", po::value<int>(&randomSeed)->default_value(randomSeed),
          "This seed value will generate a sequence using a linear random generator. Set -1 to use a random seed.")
          ;
  
// voctree specific options
  po::options_description voctreeParams("Parameters specific for the vocabulary tree-based localizer");
  voctreeParams.add_options()
      ("nbImageMatch", po::value<std::size_t>(&numResults)->default_value(numResults), 
          "[voctree] Number of images to retrieve in database")
      ("maxResults", po::value<std::size_t>(&maxResults)->default_value(maxResults), 
          "[voctree] For algorithm AllResults, it stops the image matching when "
          "this number of matched images is reached. If 0 it is ignored.")
      ("commonviews", po::value<std::size_t>(&numCommonViews)->default_value(numCommonViews), 
          "[voctree] Number of minimum images in which a point must be seen to "
          "be used in cluster tracking")
      ("voctree", po::value<std::string>(&vocTreeFilepath), 
          "[voctree] Filename for the vocabulary tree")
      ("voctreeWeights", po::value<std::string>(&weightsFilepath), 
          "[voctree] Filename for the vocabulary tree weights")
      ("algorithm", po::value<std::string>(&algostring)->default_value(algostring), 
          "[voctree] Algorithm type: FirstBest, AllResults" )
      ("matchingError", po::value<double>(&matchingErrorMax)->default_value(matchingErrorMax), 
          "[voctree] Maximum matching error (in pixels) allowed for image matching with "
          "geometric verification. If set to 0 it lets the ACRansac select "
          "an optimal value.")
      ("nbFrameBufferMatching", po::value<std::size_t>(&nbFrameBufferMatching)->default_value(nbFrameBufferMatching),
          "[voctree] Number of previous frame of the sequence to use for matching "
          "(0 = Disable)")
      ("robustMatching", po::value<bool>(&robustMatching)->default_value(robustMatching), 
          "[voctree] Enable/Disable the robust matching between query and database images, "
          "all putative matches will be considered.")
// cctag specific options
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_CCTAG)
      ("nNearestKeyFrames", po::value<size_t>(&nNearestKeyFrames)->default_value(nNearestKeyFrames), 
          "[cctag] Number of images to retrieve in the database")
#endif
  ;
  
// final bundle adjustment options
  po::options_description bundleParams("Parameters specific for final (optional) bundle adjustment optimization of the sequence");
  bundleParams.add_options()
      ("globalBundle", po::value<bool>(&globalBundle), 
          "[bundle adjustment] If --refineIntrinsics is not set, this option "
          "allows to run a final global budndle adjustment to refine the scene")
      ("noDistortion", po::value<bool>(&noDistortion), 
          "[bundle adjustment] It does not take into account distortion during "
          "the BA, it consider the distortion coefficients all equal to 0")
      ("noBArefineIntrinsics", po::value<bool>(&noBArefineIntrinsics), 
          "[bundle adjustment] It does not refine intrinsics during BA")
      ("minPointVisibility", po::value<size_t>(&minPointVisibility)->default_value(minPointVisibility), 
          "[bundle adjustment] Minimum number of observation that a point must "
          "have in order to be considered for bundle adjustment");
  
// output options
  po::options_description outputParams("Options for the output of the localizer");
  outputParams.add_options()
      ("help,h", "Print this message")
      ("visualDebug", po::value<std::string>(&visualDebug), 
          "If a folder is provided it enables visual debug and saves all the "
          "debugging info in that folder")

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_ALEMBIC)
      ("outputAlembic", po::value<std::string>(&exportAlembicFile)->default_value(exportAlembicFile),
          "Filename for the SfMData export file (where camera poses will be stored). "
          "Default : trackedcameras.abc.")
#endif
      ("outputJSON", po::value<std::string>(&exportJsonFile)->default_value(exportJsonFile),
          "Filename for the localization results (raw data) as .json")

      ;
  
  allParams.add(inputParams).add(outputParams).add(commonParams).add(voctreeParams).add(bundleParams);

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
    ALICEVISION_CERR("ERROR: " << e.what() << std::endl);
    ALICEVISION_COUT("Usage:\n\n" << allParams);
    return EXIT_FAILURE;
  }
  catch(boost::program_options::error& e)
  {
    ALICEVISION_CERR("ERROR: " << e.what() << std::endl);
    ALICEVISION_COUT("Usage:\n\n" << allParams);
    return EXIT_FAILURE;
  }

  std::mt19937 generator(randomSeed == -1 ? std::random_device()() : randomSeed);

  const double defaultLoRansacMatchingError = 4.0;
  const double defaultLoRansacResectionError = 4.0;
  if(!robustEstimation::adjustRobustEstimatorThreshold(matchingEstimator, matchingErrorMax, defaultLoRansacMatchingError) ||
     !robustEstimation::adjustRobustEstimatorThreshold(resectionEstimator, resectionErrorMax, defaultLoRansacResectionError))
  {
    return EXIT_FAILURE;
  }

  // Init descTypes from command-line string
  matchDescTypes = feature::EImageDescriberType_stringToEnums(matchDescTypeNames);

  // decide the localizer to use based on the type of feature
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_CCTAG)
  useVoctreeLocalizer = !(matchDescTypes.size() == 1 &&
                        ((matchDescTypes.front() == feature::EImageDescriberType::CCTAG3) ||
                        (matchDescTypes.front() == feature::EImageDescriberType::CCTAG4)));
#endif
  
  // the bundle adjustment can be run for now only if the refine intrinsics option is not set
  globalBundle = (globalBundle && !refineIntrinsics);
  ALICEVISION_COUT("Program called with the following parameters:");
  ALICEVISION_COUT(vm);

  // if the provided folder for visual debugging does not exist create it
  // recursively
  if((!visualDebug.empty()) && (!bfs::exists(visualDebug)))
  {
    bfs::create_directories(visualDebug);
  }
 
  // this contains the full path and the root name of the file without the extension
  const bool wantsJsonOutput = exportJsonFile.empty();
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_ALEMBIC)
  std::string basenameAlembic = (bfs::path(exportJsonFile).parent_path() / bfs::path(exportJsonFile).stem()).string();
#endif
  std::string basenameJson;
  if(wantsJsonOutput)
  {
    basenameJson = (bfs::path(exportJsonFile).parent_path() / bfs::path(exportJsonFile).stem()).string();
  }

  // load SfMData
  sfmData::SfMData sfmData;
  if(!sfmDataIO::Load(sfmData, sfmFilePath, sfmDataIO::ESfMData::ALL))
  {
    ALICEVISION_LOG_ERROR("The input SfMData file '" + sfmFilePath + "' cannot be read.");
    return EXIT_FAILURE;
  }
  
  //***********************************************************************
  // Localizer initialization
  //***********************************************************************
  
  std::unique_ptr<localization::LocalizerParameters> param;
  
  std::unique_ptr<localization::ILocalizer> localizer;
  
  // initialize the localizer according to the chosen type of describer

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_CCTAG)
  if(!useVoctreeLocalizer)
  {
    localization::CCTagLocalizer* tmpLoc = new localization::CCTagLocalizer(sfmData, descriptorsFolder);
    localizer.reset(tmpLoc);

    localization::CCTagLocalizer::Parameters* tmpParam = new localization::CCTagLocalizer::Parameters();
    param.reset(tmpParam);
    tmpParam->_nNearestKeyFrames = nNearestKeyFrames;
  }
  else
#endif
  {

    localization::VoctreeLocalizer* tmpLoc = new localization::VoctreeLocalizer(sfmData,
                                                   descriptorsFolder,
                                                   vocTreeFilepath,
                                                   weightsFilepath,
                                                   matchDescTypes);

    localizer.reset(tmpLoc);
    
    localization::VoctreeLocalizer::Parameters *tmpParam = new localization::VoctreeLocalizer::Parameters();
    param.reset(tmpParam);
    tmpParam->_algorithm = localization::VoctreeLocalizer::initFromString(algostring);;
    tmpParam->_numResults = numResults;
    tmpParam->_maxResults = maxResults;
    tmpParam->_numCommonViews = numCommonViews;
    tmpParam->_ccTagUseCuda = false;
    tmpParam->_matchingError = matchingErrorMax;
    tmpParam->_nbFrameBufferMatching = nbFrameBufferMatching;
    tmpParam->_useRobustMatching = robustMatching;
  }
  
  assert(localizer);
  assert(param);
  
  // set other common parameters
  param->_featurePreset = featDescPreset;
  param->_refineIntrinsics = refineIntrinsics;
  param->_visualDebug = visualDebug;
  param->_errorMax = resectionErrorMax;
  param->_resectionEstimator = resectionEstimator;
  param->_matchingEstimator = matchingEstimator;
  
  
  if(!localizer->isInit())
  {
    ALICEVISION_CERR("ERROR while initializing the localizer!");
    return EXIT_FAILURE;
  }
  
  // create the feedProvider
  dataio::FeedProvider feed(mediaFilepath, calibFile);
  if(!feed.isInit())
  {
    ALICEVISION_CERR("ERROR while initializing the FeedProvider!");
    return EXIT_FAILURE;
  }
  
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_ALEMBIC)
  // init alembic exporter
  sfmDataIO::AlembicExporter exporter(exportAlembicFile);
  exporter.initAnimatedCamera("camera");
#endif
  
  image::Image<float> imageGrey;
  camera::PinholeRadialK3 queryIntrinsics;
  bool hasIntrinsics = false;
  
  std::size_t frameCounter = 0;
  std::size_t goodFrameCounter = 0;
  std::vector<std::string> goodFrameList;
  std::string currentImgName;
  
  //***********************************************************************
  // Main loop
  //***********************************************************************
  
  // Define an accumulator set for computing the mean and the
  // standard deviation of the time taken for localization
  bacc::accumulator_set<double, bacc::stats<bacc::tag::mean, bacc::tag::min, bacc::tag::max, bacc::tag::sum > > stats;
  
  std::vector<localization::LocalizationResult> vec_localizationResults;
  
  while(feed.readImage(imageGrey, queryIntrinsics, currentImgName, hasIntrinsics))
  {
    ALICEVISION_COUT("******************************");
    ALICEVISION_COUT("FRAME " << myToString(frameCounter,4));
    ALICEVISION_COUT("******************************");
    localization::LocalizationResult localizationResult;
    auto detect_start = std::chrono::steady_clock::now();
    localizer->localize(imageGrey, 
                       param.get(),
                       generator,
                       hasIntrinsics /*useInputIntrinsics*/,
                       queryIntrinsics,
                       localizationResult,
                       currentImgName);
    auto detect_end = std::chrono::steady_clock::now();
    auto detect_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(detect_end - detect_start);
    ALICEVISION_COUT("\nLocalization took  " << detect_elapsed.count() << " [ms]");
    stats(detect_elapsed.count());
    
    vec_localizationResults.emplace_back(localizationResult);

    // save data
    if(localizationResult.isValid())
    {
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_ALEMBIC)
      exporter.addCameraKeyframe(localizationResult.getPose(), &queryIntrinsics, currentImgName, frameCounter, frameCounter);
#endif
      
      goodFrameCounter++;
      goodFrameList.push_back(currentImgName + " : " + std::to_string(localizationResult.getIndMatch3D2D().size()) );
    }
    else
    {
      ALICEVISION_CERR("Unable to localize frame " << frameCounter);
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_ALEMBIC)
      exporter.jumpKeyframe(currentImgName);
#endif
    }
    ++frameCounter;
    feed.goToNextFrame();
  }

  if(wantsJsonOutput)
  {
    localization::LocalizationResult::save(vec_localizationResults, basenameJson + ".json");
  }

  //***********************************************************************
  // Global bundle
  //***********************************************************************
  
  if(globalBundle)
  {
    ALICEVISION_COUT("\n\n\n***********************************************");
    ALICEVISION_COUT("Bundle Adjustment - Refining the whole sequence");
    ALICEVISION_COUT("***********************************************\n\n");
    // run a bundle adjustment
    const bool b_allTheSame = true;
    const bool b_refineStructure = false;
    const bool b_refinePose = true;
    const bool BAresult = localization::refineSequence(vec_localizationResults,
                                                       b_allTheSame, 
                                                       !noBArefineIntrinsics, 
                                                       noDistortion, 
                                                       b_refinePose,
                                                       b_refineStructure,
                                                       basenameJson + ".sfmdata.BUNDLE",
                                                       minPointVisibility);
    if(!BAresult)
    {
      ALICEVISION_CERR("Bundle Adjustment failed!");
    }
    else
    {
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_ALEMBIC)
      // now copy back in a new abc with the same name file and BUNDLE appended at the end
      sfmDataIO::AlembicExporter exporterBA( basenameAlembic +".BUNDLE.abc" );
      exporterBA.initAnimatedCamera("camera");
      std::size_t idx = 0;
      for(const localization::LocalizationResult &res : vec_localizationResults)
      {
        if(res.isValid())
        {
          assert(idx < vec_localizationResults.size());
          exporterBA.addCameraKeyframe(res.getPose(), &res.getIntrinsics(), currentImgName, idx, idx);
        }
        else
        {
          exporterBA.jumpKeyframe(currentImgName);
        }
        idx++;
      }

#endif
      if(wantsJsonOutput)
      {
        localization::LocalizationResult::save(vec_localizationResults, basenameJson +".BUNDLE.json");
      }

    }
  }
  
  // print out some time stats
  ALICEVISION_COUT("\n\n******************************");
  ALICEVISION_COUT("Localized " << goodFrameCounter << "/" << frameCounter << " images");
  ALICEVISION_COUT("Images localized with the number of 2D/3D matches during localization :");
  for(std::size_t i = 0; i < goodFrameList.size(); ++i)
    ALICEVISION_COUT(goodFrameList[i]);
  ALICEVISION_COUT("Processing took " << bacc::sum(stats)/1000 << " [s] overall");
  ALICEVISION_COUT("Mean time for localization:   " << bacc::mean(stats) << " [ms]");
  ALICEVISION_COUT("Max time for localization:   " << bacc::max(stats) << " [ms]");
  ALICEVISION_COUT("Min time for localization:   " << bacc::min(stats) << " [ms]");

  return EXIT_SUCCESS;
}
