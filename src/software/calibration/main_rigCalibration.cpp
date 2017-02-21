#include <openMVG/localization/VoctreeLocalizer.hpp>
#ifdef HAVE_CCTAG
#include <openMVG/localization/CCTagLocalizer.hpp>
#endif
#include <openMVG/rig/Rig.hpp>
#include <openMVG/image/image_io.hpp>
#include <openMVG/dataio/FeedProvider.hpp>
#include <openMVG/features/image_describer.hpp>
#include <openMVG/robust_estimation/robust_estimators.hpp>
#include <openMVG/logger.hpp>

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

#ifdef HAVE_ALEMBIC
#include <openMVG/sfm/AlembicExporter.hpp>
#endif // HAVE_ALEMBIC


namespace bfs = boost::filesystem;
namespace bacc = boost::accumulators;
namespace po = boost::program_options;

using namespace openMVG;

enum DescriberType
{
  SIFT
#ifdef HAVE_CCTAG
  ,CCTAG,
  SIFT_CCTAG
#endif
};

inline DescriberType stringToDescriberType(const std::string& describerType)
{
  if(describerType == "SIFT")
    return DescriberType::SIFT;
#ifdef HAVE_CCTAG
  if (describerType == "CCTAG")
    return DescriberType::CCTAG;
  if(describerType == "SIFT_CCTAG")
    return DescriberType::SIFT_CCTAG;
#endif
  throw std::invalid_argument("Unsupported describer type "+describerType);
}

inline std::string describerTypeToString(DescriberType describerType)
{
  if(describerType == DescriberType::SIFT)
    return "SIFT";
#ifdef HAVE_CCTAG
  if (describerType == DescriberType::CCTAG)
    return "CCTAG";
  if(describerType == DescriberType::SIFT_CCTAG)
    return "SIFT_CCTAG";
#endif
  throw std::invalid_argument("Unrecognized DescriberType "+std::to_string(describerType));
}

std::ostream& operator<<(std::ostream& os, const DescriberType describerType)
{
  os << describerTypeToString(describerType);
  return os;
}

std::istream& operator>>(std::istream &in, DescriberType &describerType)
{
  std::string token;
  in >> token;
  describerType = stringToDescriberType(token);
  return in;
}

std::string myToString(std::size_t i, std::size_t zeroPadding)
{
  std::stringstream ss;
  ss << std::setw(zeroPadding) << std::setfill('0') << i;
  return ss.str();
}

/**
 * @brief It checks if the value for the reprojection error or the matching error
 * is compatible with the given robust estimator. The value cannot be 0 for 
 * LORansac, for ACRansac a value of 0 means to use infinity (ie estimate the 
 * threshold during ransac process)
 * @param e The estimator to be checked.
 * @param value The value for the reprojection or matching error.
 * @return true if the value is compatible
 */
bool checkRobustEstimator(robust::EROBUST_ESTIMATOR e, double &value)
{
  if(e != robust::EROBUST_ESTIMATOR::ROBUST_ESTIMATOR_LORANSAC &&
     e != robust::EROBUST_ESTIMATOR::ROBUST_ESTIMATOR_ACRANSAC)
  {
    OPENMVG_CERR("Only " << robust::EROBUST_ESTIMATOR::ROBUST_ESTIMATOR_ACRANSAC 
            << " and " << robust::EROBUST_ESTIMATOR::ROBUST_ESTIMATOR_LORANSAC 
            << " are supported.");
    return false;
  }
  if(value == 0 && 
     e == robust::EROBUST_ESTIMATOR::ROBUST_ESTIMATOR_ACRANSAC)
  {
    // for acransac set it to infinity
    value = std::numeric_limits<double>::infinity();
  }
  // for loransac we need thresholds > 0
  if(e == robust::EROBUST_ESTIMATOR::ROBUST_ESTIMATOR_LORANSAC)
  {
    const double minThreshold = 1e-6;
    if(value <= minThreshold)
    {
      OPENMVG_CERR("Error: errorMax and matchingError cannot be 0 with " 
              << robust::EROBUST_ESTIMATOR::ROBUST_ESTIMATOR_LORANSAC 
              << " estimator.");
      return false;     
    }
  }

  return true;
}

int main(int argc, char** argv)
{
  // common parameters
  /// the OpenMVG .json/abc data file
  std::string sfmFilePath;
  /// the the folder containing the descriptors
  std::string descriptorsFolder;
  /// the media file to localize
  std::vector<std::string> mediaPath;
  /// the calibration file for each camera
  std::vector<std::string> cameraIntrinsics;                  
  /// the name of the file where to store the calibration data
  std::string outputFile;
  /// the preset for the feature extractor
  features::EDESCRIBER_PRESET featurePreset = features::EDESCRIBER_PRESET::NORMAL_PRESET;     
  /// the type of features to use for localization
  DescriberType descriptorType = DescriberType::SIFT;        
  /// the estimator to use for resection
  robust::EROBUST_ESTIMATOR resectionEstimator = robust::EROBUST_ESTIMATOR::ROBUST_ESTIMATOR_ACRANSAC;        
  /// the estimator to use for matching
  robust::EROBUST_ESTIMATOR matchingEstimator = robust::EROBUST_ESTIMATOR::ROBUST_ESTIMATOR_ACRANSAC;        
  /// the possible choices for the estimators as strings
  const std::string str_estimatorChoices = ""+robust::EROBUST_ESTIMATOR_enumToString(robust::EROBUST_ESTIMATOR::ROBUST_ESTIMATOR_ACRANSAC)
                                          +","+robust::EROBUST_ESTIMATOR_enumToString(robust::EROBUST_ESTIMATOR::ROBUST_ESTIMATOR_LORANSAC);
  bool refineIntrinsics = false;

  /// the maximum error allowed for resection
  double resectionErrorMax = 4.0;
  /// the maximum error allowed for image matching with geometric validation
  double matchingErrorMax = 4.0;
  /// the maximum number of frames in input
  int maxInputFrames = 0;


  // parameters for voctree localizer
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

#ifdef HAVE_ALEMBIC
  /// the export file
  std::string exportFile = "trackedcameras.abc"; 
#endif
  
  std::size_t numCameras = 0;
  po::options_description allParams("This program is used to calibrate a camera rig composed of internally calibrated cameras."
  "It takes as input a synchronized sequence of N cameras and it saves the estimated "
  "rig calibration to a text file");

  po::options_description ioParams("Required input and output parameters");  
  ioParams.add_options()
      ("sfmdata", po::value<std::string>(&sfmFilePath)->required(),
          "The sfm_data.json kind of file generated by OpenMVG.")
      ("descriptorPath", po::value<std::string>(&descriptorsFolder)->required(),
          "Folder containing the .desc.")
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
      ("descriptors", po::value<DescriberType>(&descriptorType)->default_value(descriptorType), 
        "Type of descriptors to use {SIFT"
#ifdef HAVE_CCTAG
        ", CCTAG, SIFT_CCTAG"
#endif
        "}")
      ("preset", po::value<features::EDESCRIBER_PRESET>(&featurePreset)->default_value(featurePreset), 
          "Preset for the feature extractor when localizing a new image "
          "{LOW,MEDIUM,NORMAL,HIGH,ULTRA}")
      ("resectionEstimator", po::value<robust::EROBUST_ESTIMATOR>(&resectionEstimator)->default_value(resectionEstimator), 
          std::string("The type of *sac framework to use for resection "
          "{"+str_estimatorChoices+"}").c_str())
      ("matchingEstimator", po::value<robust::EROBUST_ESTIMATOR>(&matchingEstimator)->default_value(matchingEstimator), 
          std::string("The type of *sac framework to use for matching "
          "{"+str_estimatorChoices+"}").c_str())
      ("refineIntrinsics", po::bool_switch(&refineIntrinsics),
          "Enable/Disable camera intrinsics refinement for each localized image")
      ("reprojectionError", po::value<double>(&resectionErrorMax)->default_value(resectionErrorMax), 
          "Maximum reprojection error (in pixels) allowed for resectioning. If set "
          "to 0 it lets the ACRansac select an optimal value.")
      ("maxInputFrames", po::value<int>(&maxInputFrames)->default_value(maxInputFrames), 
          "Maximum number of frames to read in input. 0 means no limit.");

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
#ifdef HAVE_CCTAG
  // parameters for cctag localizer
      ("nNearestKeyFrames", po::value<std::size_t>(&nNearestKeyFrames)->default_value(nNearestKeyFrames),
          "[cctag] Number of images to retrieve in database")
#endif
  ;
  
  // output options
  po::options_description outputParams("Options for the output of the localizer");
  outputParams.add_options()  
      ("help,h", "Print this message")
#ifdef HAVE_ALEMBIC
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
      OPENMVG_COUT(allParams);
      return EXIT_SUCCESS;
    }

    po::notify(vm);
  }
  catch(po::required_option& e)
  {
    OPENMVG_CERR("ERROR: " << e.what() << std::endl);
    OPENMVG_COUT("Usage:\n\n" << allParams);
    return EXIT_FAILURE;
  }
  catch(po::error& e)
  {
    OPENMVG_CERR("ERROR: " << e.what() << std::endl);
    OPENMVG_COUT("Usage:\n\n" << allParams);
    return EXIT_FAILURE;
  }

  if(!checkRobustEstimator(matchingEstimator, matchingErrorMax) || 
     !checkRobustEstimator(resectionEstimator, resectionErrorMax))
  {
    return EXIT_FAILURE;
  }
  
  // check that we have the same number of feeds as the intrinsics
  if((mediaPath.size() != cameraIntrinsics.size()))
  {
    OPENMVG_CERR("The number of intrinsics and the number of cameras are not the same." << std::endl);
    return EXIT_FAILURE;
  }
  numCameras = mediaPath.size();

  // just debugging prints, print out all the parameters
  {
    OPENMVG_COUT("Program called with the following parameters:");
    OPENMVG_COUT("\tsfmdata: " << sfmFilePath);
    OPENMVG_COUT("\tpreset: " << featurePreset);
    OPENMVG_COUT("\tmediapath: " << mediaPath);
    OPENMVG_COUT("\tcameraIntrinsics: " << cameraIntrinsics);
    OPENMVG_COUT("\tresectionEstimator: " << resectionEstimator);
    OPENMVG_COUT("\tmatchingEstimator: " << matchingEstimator);
    OPENMVG_COUT("\tdescriptorPath: " << descriptorsFolder);
    OPENMVG_COUT("\trefineIntrinsics: " << refineIntrinsics);
    OPENMVG_COUT("\treprojectionError: " << resectionErrorMax);
    OPENMVG_COUT("\tdescriptors: " << descriptorType);
    if((DescriberType::SIFT==descriptorType)
#ifdef HAVE_CCTAG
            ||(DescriberType::SIFT_CCTAG==descriptorType)
#endif
      )
    {
      // parameters for voctree localizer
      OPENMVG_COUT("\tvoctree: " << vocTreeFilepath);
      OPENMVG_COUT("\tweights: " << weightsFilepath);
      OPENMVG_COUT("\tnbImageMatch: " << numResults);
      OPENMVG_COUT("\tmaxResults: " << maxResults);
      OPENMVG_COUT("\talgorithm: " << algostring);
      OPENMVG_COUT("\tmatchingError: " << matchingErrorMax);
    }
#ifdef HAVE_CCTAG
    else
    {
      OPENMVG_COUT("\tnNearestKeyFrames: " << nNearestKeyFrames);
    }
#endif

  }

  std::unique_ptr<localization::LocalizerParameters> param;
  
  std::unique_ptr<localization::ILocalizer> localizer;
  
  // initialize the localizer according to the chosen type of describer
  if((DescriberType::SIFT==descriptorType)
#ifdef HAVE_CCTAG
            ||(DescriberType::SIFT_CCTAG==descriptorType)
#endif
      )
  {
    OPENMVG_COUT("Calibrating sequence using the voctree localizer");
    localization::VoctreeLocalizer* tmpLoc = new localization::VoctreeLocalizer(sfmFilePath,
                                                            descriptorsFolder,
                                                            vocTreeFilepath,
                                                            weightsFilepath
#ifdef HAVE_CCTAG
                                                            , DescriberType::SIFT_CCTAG==descriptorType
#endif
                                                            );
    localizer.reset(tmpLoc);
    
    localization::VoctreeLocalizer::Parameters *tmpParam = new localization::VoctreeLocalizer::Parameters();
    param.reset(tmpParam);
    tmpParam->_algorithm = localization::VoctreeLocalizer::initFromString(algostring);;
    tmpParam->_numResults = numResults;
    tmpParam->_maxResults = maxResults;
    tmpParam->_ccTagUseCuda = false;
    tmpParam->_matchingError = matchingErrorMax;
    
  }
#ifdef HAVE_CCTAG
  else
  {
    localization::CCTagLocalizer* tmpLoc = new localization::CCTagLocalizer(sfmFilePath, descriptorsFolder);
    localizer.reset(tmpLoc);
    
    localization::CCTagLocalizer::Parameters *tmpParam = new localization::CCTagLocalizer::Parameters();
    param.reset(tmpParam);
    tmpParam->_nNearestKeyFrames = nNearestKeyFrames;
  }
#endif 

  assert(localizer);
  assert(param);
  
  // set other common parameters
  param->_featurePreset = featurePreset;
  param->_refineIntrinsics = refineIntrinsics;
  param->_errorMax = resectionErrorMax;
  param->_resectionEstimator = resectionEstimator;
  param->_matchingEstimator = matchingEstimator;

  if(!localizer->isInit())
  {
    OPENMVG_CERR("ERROR while initializing the localizer!");
    return EXIT_FAILURE;
  }

#ifdef HAVE_ALEMBIC
  sfm::AlembicExporter exporter(exportFile);
  exporter.addPoints(localizer->getSfMData().GetLandmarks());
#endif

  // Create a camera rig
  rig::Rig rig;

  // Loop over all cameras of the rig
  for(std::size_t idCamera = 0; idCamera < numCameras; ++idCamera)
  {
    OPENMVG_COUT("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
    OPENMVG_COUT("CAMERA " << idCamera);
    OPENMVG_COUT("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n\n");
    const std::string &calibFile = cameraIntrinsics[idCamera];
    const std::string &feedPath = mediaPath[idCamera];
    // contains the directory where the video, the images or the filelist is
    const std::string subMediaFilepath = 
        bfs::is_directory(bfs::path(mediaPath[idCamera])) ? 
          (mediaPath[idCamera]) : 
          (bfs::path(mediaPath[idCamera]).parent_path().string());

    // create the feedProvider
    dataio::FeedProvider feed(feedPath, calibFile);
    if(!feed.isInit())
    {
      OPENMVG_CERR("ERROR while initializing the FeedProvider!");
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
    OPENMVG_COUT("Input stream length is " << feed.nbFrames() << ".");

    //std::string featureFile, cameraResultFile, pointsFile;
    //featureFile = subMediaFilepath + "/cctag" + std::to_string(nRings) + "CC.out";
    //cameraResultFile = inputFolder + "/" + std::to_string(i) + "/cameras.txt";
    //std::ofstream result;
    //result.open(cameraResultFile);
    //pointsFile = inputFolder + "/points.txt";

    image::Image<unsigned char> imageGrey;
    cameras::Pinhole_Intrinsic_Radial_K3 queryIntrinsics;
    bool hasIntrinsics = false;

    std::size_t iInputFrame = 0;
    std::string currentImgName;

    // Define an accumulator set for computing the mean and the
    // standard deviation of the time taken for localization
    bacc::accumulator_set<double, bacc::stats<bacc::tag::mean, bacc::tag::min, bacc::tag::max, bacc::tag::sum > > stats;

    // used to collect the match data result
    std::vector<localization::LocalizationResult> vLocalizationResults;
    std::size_t currentFrame = 0;
    while(feed.readImage(imageGrey, queryIntrinsics, currentImgName, hasIntrinsics))
    {
      OPENMVG_COUT("******************************");
      OPENMVG_COUT("Stream " << idCamera << " Frame " << myToString(currentFrame, 4) << "/" << nbFrames << " : (" << iInputFrame << "/" << nbFramesToProcess << ")");
      OPENMVG_COUT("******************************");
      auto detect_start = std::chrono::steady_clock::now();
      localization::LocalizationResult localizationResult;
      localizer->setCudaPipe( idCamera );
      const bool ok = localizer->localize(imageGrey,
                                          param.get(),
                                          hasIntrinsics/*useInputIntrinsics*/,
                                          queryIntrinsics,
                                          localizationResult);
      assert( ok == localizationResult.isValid() );
      vLocalizationResults.emplace_back(localizationResult);
      auto detect_end = std::chrono::steady_clock::now();
      auto detect_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(detect_end - detect_start);
      OPENMVG_COUT("Localization took  " << detect_elapsed.count() << " [ms]");
      stats(detect_elapsed.count());
      
#ifdef HAVE_ALEMBIC
      if(localizationResult.isValid())
      {
        exporter.appendCamera("camera"+std::to_string(idCamera)+"."+myToString(currentFrame,4), localizationResult.getPose(), &queryIntrinsics, subMediaFilepath, currentFrame, currentFrame);
      }
      else
      {
        // @fixme for now just add a fake camera so that it still can be see in MAYA
        exporter.appendCamera("camera"+std::to_string(idCamera)+".V."+myToString(currentFrame,4), geometry::Pose3(), &queryIntrinsics, subMediaFilepath, currentFrame, currentFrame);
      }
#endif
      ++iInputFrame;
      currentFrame = std::floor(iInputFrame * step);
      feed.goToFrame(currentFrame);
    }

    rig.setTrackingResult(vLocalizationResults, idCamera);

    // print out some time stats
    OPENMVG_COUT("\n\n******************************");
    OPENMVG_COUT("Processed " << iInputFrame << " images for camera " << idCamera);
    OPENMVG_COUT("Processing took " << bacc::sum(stats) / 1000 << " [s] overall");
    OPENMVG_COUT("Mean time for localization:   " << bacc::mean(stats) << " [ms]");
    OPENMVG_COUT("Max time for localization:   " << bacc::max(stats) << " [ms]");
    OPENMVG_COUT("Min time for localization:   " << bacc::min(stats) << " [ms]");
  }
  
  {
    // just for statistics purposes
    const std::size_t numRigCam = rig.nCams();
    OPENMVG_COUT("\n\n******************************");
    for(std::size_t idCam = 0; idCam < numRigCam; ++idCam)
    {
      auto & currResult = rig.getLocalizationResults(idCam);
      std::size_t numLocalized = 0;
      for(const auto &curr : currResult)
      {
        if(curr.isValid())
          ++numLocalized;
      }
      OPENMVG_COUT("Camera " << idCam << " localized " 
              << numLocalized << "/" << currResult.size());
    }
    
  }
  
  OPENMVG_COUT("Rig calibration initialization...");
  if(!rig.initializeCalibration())
  {
    OPENMVG_CERR("Unable to find a proper initialization for the relative poses! Aborting...");
    return EXIT_FAILURE;
  }
  OPENMVG_COUT("Rig calibration optimization...");
  if(!rig.optimizeCalibration())
  {
    OPENMVG_CERR("Unable to optimize the relative poses! Aborting...");
    return EXIT_FAILURE;
  }
  
  // save the rig calibration (subposes)
  rig.saveCalibration(outputFile);
  
  
  // just print out the results
  // the first rig pose
  if(rig.getPosesSize() > 0)
  {
    OPENMVG_COUT("First pose of the rig");
    const geometry::Pose3 &pose = rig.getPose(0); 
    OPENMVG_COUT("R\n" << pose.rotation());
    OPENMVG_COUT("center\n" << pose.center());
    OPENMVG_COUT("t\n" << pose.translation());
  }
  
  // get the subposes of the cameras inside the rig
  const std::vector<geometry::Pose3>& subposes = rig.getRelativePoses();
  assert(numCameras-1 == subposes.size());
  for(std::size_t i = 0; i < subposes.size(); ++i)
  {
    const geometry::Pose3 &pose = subposes[i];
    OPENMVG_COUT("--------------------");
    OPENMVG_COUT("Subpose p0" << i+1); // from camera 0 to camera i+1
    OPENMVG_COUT("R\n" << pose.rotation());
    OPENMVG_COUT("center\n" << pose.center());
    OPENMVG_COUT("t\n" << pose.translation());
    OPENMVG_COUT("--------------------\n");
  }
  
  return EXIT_SUCCESS;
}
