// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/config.hpp>
#include <aliceVision/localization/VoctreeLocalizer.hpp>
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_CCTAG)
#include <aliceVision/localization/CCTagLocalizer.hpp>
#endif
#include <aliceVision/rig/Rig.hpp>
#include <aliceVision/image/io.hpp>
#include <aliceVision/dataio/FeedProvider.hpp>
#include <aliceVision/feature/ImageDescriber.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/robustEstimation/estimators.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/main.hpp>

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
  // common parameters
  /// the AliceVision .json/abc data file
  std::string sfmFilePath;
  /// the the folder containing the descriptors
  std::string descriptorsFolder;
  /// the media file to localize
  std::vector<std::string> mediaPath;
  /// the calibration file for each camera
  std::vector<std::string> cameraIntrinsics;                  
  /// the name of the file where to store the calibration data
  std::string outputFile;
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
  const std::string str_estimatorChoices = ""+robustEstimation::ERobustEstimator_enumToString(robustEstimation::ERobustEstimator::ACRANSAC)
                                          +","+robustEstimation::ERobustEstimator_enumToString(robustEstimation::ERobustEstimator::LORANSAC);
  bool refineIntrinsics = false;

  /// the maximum error allowed for resection
  double resectionErrorMax = 4.0;
  /// the maximum error allowed for image matching with geometric validation
  double matchingErrorMax = 4.0;
  /// the maximum number of frames in input
  std::size_t maxInputFrames = 0;


  // parameters for voctree localizer
  /// whether to use the voctreeLocalizer or cctagLocalizer
  bool useVoctreeLocalizer = true;
  /// the vocabulary tree file
  std::string vocTreeFilepath;
  /// the vocabulary tree weights file
  std::string weightsFilepath;
  /// the localization algorithm to use for the voctree localizer
  std::string algostring = "AllResults";
  /// number of documents to search when querying the voctree
  std::size_t numResults = 4;
  /// maximum number of matching documents to retain
  std::size_t maxResults = 10;
  
  // parameters for cctag localizer
  std::size_t nNearestKeyFrames = 5;

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_ALEMBIC)
  /// the export file
  std::string exportFile = "trackedcameras.abc"; 
#endif
  int randomSeed = std::mt19937::default_seed;

  std::size_t numCameras = 0;
  po::options_description allParams("This program is used to calibrate a camera rig composed of internally calibrated cameras."
  "It takes as input a synchronized sequence of N cameras and it saves the estimated "
  "rig calibration to a text file");

  po::options_description ioParams("Required input and output parameters");  
  ioParams.add_options()
      ("sfmdata", po::value<std::string>(&sfmFilePath)->required(),
          "The sfm_data.json kind of file generated by AliceVision.")
      ("mediapath", po::value<std::vector<std::string> >(&mediaPath)->multitoken()->required(),
          "The path to the video file, the folder of the image sequence or a text "
          "file (one image path per line) for each camera of the rig "
          "(eg. --mediapath /path/to/cam1.mov /path/to/cam2.mov).")
      ("cameraIntrinsics", po::value<std::vector<std::string> >(&cameraIntrinsics)->multitoken()->required(),
          "The intrinsics calibration file for each camera of the rig. "
          "(eg. --cameraIntrinsics /path/to/calib1.txt /path/to/calib2.txt).")
      ("outfile,o", po::value<std::string>(&outputFile)->required(),
          "The name of the file where to store the calibration data");
   
  po::options_description commonParams("Common optional parameters for the localizer");
  commonParams.add_options()
      ("descriptorPath", po::value<std::string>(&descriptorsFolder),
          "Folder containing the .desc.")
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
      ("refineIntrinsics", po::value<bool>(&refineIntrinsics),
          "Enable/Disable camera intrinsics refinement for each localized image")
      ("reprojectionError", po::value<double>(&resectionErrorMax)->default_value(resectionErrorMax), 
          "Maximum reprojection error (in pixels) allowed for resectioning. If set "
          "to 0 it lets the ACRansac select an optimal value.")
      ("maxInputFrames", po::value<std::size_t>(&maxInputFrames)->default_value(maxInputFrames), 
          "Maximum number of frames to read in input. 0 means no limit.")
      ("randomSeed", po::value<int>(&randomSeed)->default_value(randomSeed),
          "This seed value will generate a sequence using a linear random generator. Set -1 to use a random seed.")
      ;

  // parameters for voctree localizer
  po::options_description voctreeParams("Parameters specific for the vocabulary tree-based localizer");
  voctreeParams.add_options()
      ("voctree", po::value<std::string>(&vocTreeFilepath),
          "[voctree] Filename for the vocabulary tree")
      ("voctreeWeights", po::value<std::string>(&weightsFilepath),
          "[voctree] Filename for the vocabulary tree weights")
      ("algorithm", po::value<std::string>(&algostring)->default_value(algostring),
          "[voctree] Algorithm type: {FirstBest,AllResults}" )
      ("nbImageMatch", po::value<std::size_t>(&numResults)->default_value(numResults),
          "[voctree] Number of images to retrieve in the database")
      ("maxResults", po::value<std::size_t>(&maxResults)->default_value(maxResults), 
          "[voctree] For algorithm AllResults, it stops the image matching when "
          "this number of matched images is reached. If 0 it is ignored.")
      ("matchingError", po::value<double>(&matchingErrorMax)->default_value(matchingErrorMax), 
          "[voctree] Maximum matching error (in pixels) allowed for image matching with "
          "geometric verification. If set to 0 it lets the ACRansac select "
          "an optimal value.")
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_CCTAG)
  // parameters for cctag localizer
      ("nNearestKeyFrames", po::value<std::size_t>(&nNearestKeyFrames)->default_value(nNearestKeyFrames),
          "[cctag] Number of images to retrieve in database")
#endif
      ;

  // output options
  po::options_description outputParams("Options for the output of the localizer");
  outputParams.add_options()  
      ("help,h", "Print this message")
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_ALEMBIC)
      ("export,e", po::value<std::string>(&exportFile)->default_value(exportFile),
          "Filename for the alembic file containing the rig poses with the 3D points. "
          "It also saves a file for each camera named 'filename.cam##.abc'.")
#endif
          ;
  
  allParams.add(ioParams).add(outputParams).add(commonParams).add(voctreeParams);
  
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
  catch(po::required_option& e)
  {
    ALICEVISION_CERR("ERROR: " << e.what() << std::endl);
    ALICEVISION_COUT("Usage:\n\n" << allParams);
    return EXIT_FAILURE;
  }
  catch(po::error& e)
  {
    ALICEVISION_CERR("ERROR: " << e.what() << std::endl);
    ALICEVISION_COUT("Usage:\n\n" << allParams);
    return EXIT_FAILURE;
  }

  const double defaultLoRansacMatchingError = 4.0;
  const double defaultLoRansacResectionError = 4.0;
  if(!adjustRobustEstimatorThreshold(matchingEstimator, matchingErrorMax, defaultLoRansacMatchingError) ||
     !adjustRobustEstimatorThreshold(resectionEstimator, resectionErrorMax, defaultLoRansacResectionError))
  {
    return EXIT_FAILURE;
  }
  
  // check that we have the same number of feeds as the intrinsics
  if((mediaPath.size() != cameraIntrinsics.size()))
  {
    ALICEVISION_CERR("The number of intrinsics and the number of cameras are not the same." << std::endl);
    return EXIT_FAILURE;
  }
  numCameras = mediaPath.size();

  // Init descTypes from command-line string
  matchDescTypes = feature::EImageDescriberType_stringToEnums(matchDescTypeNames);

  // decide the localizer to use based on the type of feature
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_CCTAG)
  useVoctreeLocalizer = !(matchDescTypes.size() == 1 &&
                        ((matchDescTypes.front() == feature::EImageDescriberType::CCTAG3) ||
                        (matchDescTypes.front() == feature::EImageDescriberType::CCTAG4)));
#endif

  ALICEVISION_COUT("Program called with the following parameters:");
  ALICEVISION_COUT(vm);

  std::mt19937 randomNumberGenerator(randomSeed == -1 ? std::random_device()() : randomSeed);

  std::unique_ptr<localization::LocalizerParameters> param;
  
  std::unique_ptr<localization::ILocalizer> localizer;

  // load SfMData
  sfmData::SfMData sfmData;
  if(!sfmDataIO::Load(sfmData, sfmFilePath, sfmDataIO::ESfMData::ALL))
  {
    ALICEVISION_LOG_ERROR("The input SfMData file '" + sfmFilePath + "' cannot be read.");
    return EXIT_FAILURE;
  }
  
  // initialize the localizer according to the chosen type of describer
  if(useVoctreeLocalizer)
  {
    ALICEVISION_COUT("Calibrating sequence using the voctree localizer");
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
    tmpParam->_ccTagUseCuda = false;
    tmpParam->_matchingError = matchingErrorMax;
    
  }
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_CCTAG)
  else
  {
    localization::CCTagLocalizer* tmpLoc = new localization::CCTagLocalizer(sfmData, descriptorsFolder);
    localizer.reset(tmpLoc);
    
    localization::CCTagLocalizer::Parameters *tmpParam = new localization::CCTagLocalizer::Parameters();
    param.reset(tmpParam);
    tmpParam->_nNearestKeyFrames = nNearestKeyFrames;
  }
#endif 

  assert(localizer);
  assert(param);
  
  // set other common parameters
  param->_featurePreset = featDescPreset;
  param->_refineIntrinsics = refineIntrinsics;
  param->_errorMax = resectionErrorMax;
  param->_resectionEstimator = resectionEstimator;
  param->_matchingEstimator = matchingEstimator;

  if(!localizer->isInit())
  {
    ALICEVISION_CERR("ERROR while initializing the localizer!");
    return EXIT_FAILURE;
  }

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_ALEMBIC)
  sfmDataIO::AlembicExporter exporter(exportFile);
  exporter.addLandmarks(localizer->getSfMData().getLandmarks());
#endif

  // Create a camera rig
  rig::Rig rig;

  // Loop over all cameras of the rig
  for(std::size_t idCamera = 0; idCamera < numCameras; ++idCamera)
  {
    ALICEVISION_COUT("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
    ALICEVISION_COUT("CAMERA " << idCamera);
    ALICEVISION_COUT("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n\n");
    const std::string &calibFile = cameraIntrinsics[idCamera];
    const std::string &feedPath = mediaPath[idCamera];
    // contains the folder where the video, the images or the filelist is
    const std::string subMediaFilepath = 
        bfs::is_directory(bfs::path(mediaPath[idCamera])) ? 
          (mediaPath[idCamera]) : 
          (bfs::path(mediaPath[idCamera]).parent_path().string());

    // create the feedProvider
    dataio::FeedProvider feed(feedPath, calibFile);
    if(!feed.isInit())
    {
      ALICEVISION_CERR("ERROR while initializing the FeedProvider!");
      return EXIT_FAILURE;
    }

    double step = 1.0;
    const int nbFrames = feed.nbFrames();
    int nbFramesToProcess = nbFrames;

    // Compute the discretization's step
    if (maxInputFrames && feed.nbFrames() > maxInputFrames)
    {
      step = feed.nbFrames() / (double) maxInputFrames;
      nbFramesToProcess = maxInputFrames;
    }
    ALICEVISION_COUT("Input stream length is " << feed.nbFrames() << ".");

    //std::string featureFile, cameraResultFile, pointsFile;
    //featureFile = subMediaFilepath + "/cctag" + std::to_string(nRings) + "CC.out";
    //cameraResultFile = inputFolder + "/" + std::to_string(i) + "/cameras.txt";
    //std::ofstream result;
    //result.open(cameraResultFile);
    //pointsFile = inputFolder + "/points.txt";

    image::Image<float> imageGrey;
    std::shared_ptr<camera::PinholeRadialK3> queryIntrinsics = std::make_shared<camera::PinholeRadialK3>();
    bool hasIntrinsics = false;

    std::size_t iInputFrame = 0;
    std::string currentImgName;

    // Define an accumulator set for computing the mean and the
    // standard deviation of the time taken for localization
    bacc::accumulator_set<double, bacc::stats<bacc::tag::mean, bacc::tag::min, bacc::tag::max, bacc::tag::sum > > stats;

    // used to collect the match data result
    std::vector<localization::LocalizationResult> vLocalizationResults;
    std::size_t currentFrame = 0;
    while(feed.readImage(imageGrey, *queryIntrinsics, currentImgName, hasIntrinsics))
    {
      ALICEVISION_COUT("******************************");
      ALICEVISION_COUT("Stream " << idCamera << " Frame " << myToString(currentFrame, 4) << "/" << nbFrames << " : (" << iInputFrame << "/" << nbFramesToProcess << ")");
      ALICEVISION_COUT("******************************");
      auto detect_start = std::chrono::steady_clock::now();
      localization::LocalizationResult localizationResult;
      localizer->setCudaPipe( idCamera );
      const bool ok = localizer->localize(imageGrey,
                                          param.get(),
                                          randomNumberGenerator,
                                          hasIntrinsics/*useInputIntrinsics*/,
                                          *queryIntrinsics,
                                          localizationResult);
      assert( ok == localizationResult.isValid() );
      vLocalizationResults.emplace_back(localizationResult);
      sfmData::CameraPose pose(localizationResult.getPose());
      auto detect_end = std::chrono::steady_clock::now();
      auto detect_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(detect_end - detect_start);
      ALICEVISION_COUT("Localization took  " << detect_elapsed.count() << " [ms]");
      stats(detect_elapsed.count());
      
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_ALEMBIC)
      if(localizationResult.isValid())
      {
        exporter.addCamera("camera"+std::to_string(idCamera)+"."+myToString(currentFrame,4),
                           sfmData::View(subMediaFilepath, currentFrame, currentFrame),
                           &pose,
                           queryIntrinsics);
      }
      else
      {
        // @fixme for now just add a fake camera so that it still can be see in MAYA
        exporter.addCamera("camera"+std::to_string(idCamera)+".V."+myToString(currentFrame,4),
                           sfmData::View(subMediaFilepath, currentFrame, currentFrame),
                           &pose,
                           queryIntrinsics);
      }
#endif
      ++iInputFrame;
      currentFrame = std::floor(iInputFrame * step);
      feed.goToFrame(currentFrame);
    }

    rig.setTrackingResult(vLocalizationResults, idCamera);

    // print out some time stats
    ALICEVISION_COUT("\n\n******************************");
    ALICEVISION_COUT("Processed " << iInputFrame << " images for camera " << idCamera);
    ALICEVISION_COUT("Processing took " << bacc::sum(stats) / 1000 << " [s] overall");
    ALICEVISION_COUT("Mean time for localization:   " << bacc::mean(stats) << " [ms]");
    ALICEVISION_COUT("Max time for localization:   " << bacc::max(stats) << " [ms]");
    ALICEVISION_COUT("Min time for localization:   " << bacc::min(stats) << " [ms]");
  }
  
  {
    // just for statistics purposes
    const std::size_t numRigCam = rig.nCams();
    ALICEVISION_COUT("\n\n******************************");
    for(std::size_t idCam = 0; idCam < numRigCam; ++idCam)
    {
      auto & currResult = rig.getLocalizationResults(idCam);
      std::size_t numLocalized = 0;
      for(const auto &curr : currResult)
      {
        if(curr.isValid())
          ++numLocalized;
      }
      ALICEVISION_COUT("Camera " << idCam << " localized " 
              << numLocalized << "/" << currResult.size());
    }
    
  }
  
  ALICEVISION_COUT("Rig calibration initialization...");
  if(!rig.initializeCalibration())
  {
    ALICEVISION_CERR("Unable to find a proper initialization for the relative poses! Aborting...");
    return EXIT_FAILURE;
  }
  ALICEVISION_COUT("Rig calibration optimization...");
  if(!rig.optimizeCalibration())
  {
    ALICEVISION_CERR("Unable to optimize the relative poses! Aborting...");
    return EXIT_FAILURE;
  }
  
  // save the rig calibration (subposes)
  rig.saveCalibration(outputFile);
  
  
  // just print out the results
  // the first rig pose
  if(rig.getPosesSize() > 0)
  {
    ALICEVISION_COUT("First pose of the rig");
    const geometry::Pose3 &pose = rig.getPose(0); 
    ALICEVISION_COUT("R\n" << pose.rotation());
    ALICEVISION_COUT("center\n" << pose.center());
    ALICEVISION_COUT("t\n" << pose.translation());
  }
  
  // get the subposes of the cameras inside the rig
  const std::vector<geometry::Pose3>& subposes = rig.getRelativePoses();
  assert(numCameras-1 == subposes.size());
  for(std::size_t i = 0; i < subposes.size(); ++i)
  {
    const geometry::Pose3 &pose = subposes[i];
    ALICEVISION_COUT("--------------------");
    ALICEVISION_COUT("Subpose p0" << i+1); // from camera 0 to camera i+1
    ALICEVISION_COUT("R\n" << pose.rotation());
    ALICEVISION_COUT("center\n" << pose.center());
    ALICEVISION_COUT("t\n" << pose.translation());
    ALICEVISION_COUT("--------------------\n");
  }
  
  return EXIT_SUCCESS;
}
