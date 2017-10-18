// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

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
    {
      filename = stlplus::filename_part(v->getImagePath());
    }
    else
    {
      if(stlplus::is_full_path(v->getImagePath()))
      {
        filename = v->getImagePath();
      }
      else
      {
        filename = sfm_data.s_root_path + v->getImagePath();
      }
    }
    
    if (filename == initialName)
    {
      if(out_viewId == UndefinedIndexT)
      {
          out_viewId = v->getViewId();
      }
      else
      {
        std::cout<<"Error: Two pictures named :" << initialName << " !" << std::endl;
      }
    }
  }
  return out_viewId != UndefinedIndexT;
}


int main(int argc, char **argv)
{
  // command-line parameters

  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  std::string sfmDataFilename;
  std::string featuresDirectory;
  std::string matchesDirectory;
  std::string outputSfM;

  // user optional parameters

  std::string extraInfoDirectory;
  std::string describerTypesName = feature::EImageDescriberType_enumToString(feature::EImageDescriberType::SIFT);
  std::string outInterFileExtension = ".ply";
  std::pair<std::string,std::string> initialPairString("","");
  int minInputTrackLength = 2;
  int userCameraModel = static_cast<int>(PINHOLE_CAMERA_RADIAL3);
  bool refineIntrinsics = true;
  bool allowUserInteraction = true;

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
    ("featuresDirectory,f", po::value<std::string>(&featuresDirectory)->required(),
      "Path to a directory containing the extracted features.")
    ("matchesDirectory,m", po::value<std::string>(&matchesDirectory)->required(),
      "Path to a directory in which computed matches are stored.");

  po::options_description optionalParams("Optional parameters");
  optionalParams.add_options()
    ("extraInfoDirectory", po::value<std::string>(&extraInfoDirectory)->default_value(extraInfoDirectory),
      "Directory for intermediate reconstruction files and additional reconstruction information files.")
    ("describerTypes,d", po::value<std::string>(&describerTypesName)->default_value(describerTypesName),
      feature::EImageDescriberType_informations().c_str())
    ("interFileExtension", po::value<std::string>(&outInterFileExtension)->default_value(outInterFileExtension),
      "Extension of the intermediate file export.")
    ("minInputTrackLength", po::value<int>(&minInputTrackLength)->default_value(minInputTrackLength),
      "Minimum track length in input of SfM")
    ("cameraModel", po::value<int>(&userCameraModel)->default_value(userCameraModel),
      "* 1: Pinhole\n"
      "* 2: Pinhole radial 1\n"
      "* 3: Pinhole radial 3")
    ("initialPairA", po::value<std::string>(&initialPairString.first)->default_value(initialPairString.first),
      "filename of the first image (without path).")
    ("initialPairB", po::value<std::string>(&initialPairString.second)->default_value(initialPairString.second),
      "filename of the second image (without path).")
    ("refineIntrinsics", po::value<bool>(&refineIntrinsics)->default_value(refineIntrinsics),
      "Refine intrinsic parameters.");
    ("allowUserInteraction", po::value<bool>(&allowUserInteraction)->default_value(allowUserInteraction),
      "Enable/Disable user interactions.\n"
      "If the process is done on renderfarm, it doesn't make sense to wait for user inputs");

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

  // Load input SfMData scene
  SfMData sfmData;
  if (!Load(sfmData, sfmDataFilename, ESfMData(ALL))) {
    std::cerr << std::endl
      << "The input SfMData file \""<< sfmDataFilename << "\" cannot be read." << std::endl;
    return EXIT_FAILURE;
  }

  if(featuresDirectory.empty()) {
    featuresDirectory = matchesDirectory;
  }

  // Get imageDescriberMethodType
  const std::vector<feature::EImageDescriberType> describerTypes = feature::EImageDescriberType_stringToEnums(describerTypesName);

  // Features reading
  feature::FeaturesPerView featuresPerView;
  if(!sfm::loadFeaturesPerView(featuresPerView, sfmData, featuresDirectory, describerTypes))
  {
    std::cerr << std::endl
      << "Invalid features." << std::endl;
    return EXIT_FAILURE;
  }
  
  // Matches reading
  matching::PairwiseMatches pairwiseMatches;

  if(!loadPairwiseMatches(pairwiseMatches, sfmData, matchesDirectory, describerTypes, "f"))
  {
    std::cerr << std::endl << "Unable to load matches file from: " << matchesDirectory << std::endl;
    return EXIT_FAILURE;
  }

  if (outputSfM.empty())
  {
    std::cerr << "\nIt is an invalid SfMData file path." << std::endl;
    return EXIT_FAILURE;
  }
  if(extraInfoDirectory.empty())
  {
    namespace bfs = boost::filesystem;
    extraInfoDirectory = bfs::path(outputSfM).parent_path().string();
  }

  if (!stlplus::folder_exists(extraInfoDirectory))
    stlplus::folder_create(extraInfoDirectory);

  //---------------------------------------
  // Sequential reconstruction process
  //---------------------------------------

  aliceVision::system::Timer timer;
  ReconstructionEngine_sequentialSfM sfmEngine(
    sfmData,
    extraInfoDirectory,
    stlplus::create_filespec(extraInfoDirectory, "sfm_log.html"));

  // Configure the featuresPerView & the matches_provider
  sfmEngine.setFeatures(&featuresPerView);
  sfmEngine.setMatches(&pairwiseMatches);

  // Configure reconstruction parameters
  sfmEngine.Set_bFixedIntrinsics(!refineIntrinsics);
  sfmEngine.SetUnknownCameraType(EINTRINSIC(userCameraModel));
  sfmEngine.setMinInputTrackLength(minInputTrackLength);
  sfmEngine.setSfmdataInterFileExtension(outInterFileExtension);
  sfmEngine.setAllowUserInteraction(allowUserInteraction);

  // Handle Initial pair parameter
  if (!initialPairString.first.empty() && !initialPairString.second.empty())
  {
    if (initialPairString.first == initialPairString.second)
    {
      std::cerr << "\nInvalid image names. You cannot use the same image to initialize a pair." << std::endl;
      return EXIT_FAILURE;
    }

    Pair initialPairIndex;
    if(!retrieveViewIdFromImageName(sfmData, initialPairString.first, initialPairIndex.first)
            || !retrieveViewIdFromImageName(sfmData, initialPairString.second, initialPairIndex.second))
    {
        std::cerr << "Could not find the initial pairs <" << initialPairString.first
          <<  ", " << initialPairString.second << ">!\n";
        return EXIT_FAILURE;
    }
 
    sfmEngine.setInitialPair(initialPairIndex);
  }

  if (!sfmEngine.Process())
  {
    return EXIT_FAILURE;
  }

  // get the color for the 3D points
  if(!sfmEngine.Colorize())
  {
    std::cerr << "Colorize failed!" << std::endl;
  }
  
  sfmEngine.Get_SfMData().setFeatureFolder(featuresDirectory);
  sfmEngine.Get_SfMData().setMatchingFolder(matchesDirectory);

  std::cout << std::endl << " Total Ac-Sfm took (s): " << timer.elapsed() << std::endl;

  std::cout << "...Generating SfM_Report.html" << std::endl;
  Generate_SfM_Report(sfmEngine.Get_SfMData(),
    stlplus::create_filespec(extraInfoDirectory, "sfm_report.html"));

  //-- Export to disk computed scene (data & visualizable results)
  std::cout << "...Export SfMData to disk:" << outputSfM << std::endl;

  Save(sfmEngine.Get_SfMData(), outputSfM, ESfMData(ALL));

  Save(sfmEngine.Get_SfMData(), stlplus::create_filespec(extraInfoDirectory, "cloud_and_poses", outInterFileExtension), ESfMData(VIEWS | EXTRINSICS | INTRINSICS | STRUCTURE));

  return EXIT_SUCCESS;
}
