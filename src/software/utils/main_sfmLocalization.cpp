// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// Copyright (c) 2015 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/sfm/sfm.hpp>
#include <aliceVision/sfm/pipeline/regionsIO.hpp>
#include <aliceVision/feature/feature.hpp>
#include <aliceVision/image/all.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/main.hpp>

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

// Image localization API sample:
// - Allow to locate an image to an existing SfM_reconstruction
//   if 3D-2D matches are found
// - A demonstration mode (default):
//   - try to locate all the view of the SfM_Data reconstruction
int aliceVision_main(int argc, char **argv)
{
  // command-line parameters

  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  std::string sfmDataFilename;
  std::vector<std::string> featuresFolders;
  std::string outputFolder;
  std::string queryImage;

  // user optional parameters

  std::string describerTypesName = feature::EImageDescriberType_enumToString(feature::EImageDescriberType::SIFT);
  double maxResidualError = std::numeric_limits<double>::infinity();

  po::options_description allParams(
    "Image localization in an existing SfM reconstruction\n"
    "AliceVision sfmLocalization");

  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
      "SfMData file.")
    ("output,o", po::value<std::string>(&outputFolder)->required(),
      "Output path.")
    ("featuresFolders,f", po::value<std::vector<std::string>>(&featuresFolders)->multitoken()->required(),
      "Path to folder(s) containing the extracted features.")
    ("queryImage", po::value<std::string>(&queryImage)->required(),
      "Path to the image that must be localized.");

  po::options_description optionalParams("Optional parameters");
  optionalParams.add_options()
    ("describerTypes,d", po::value<std::string>(&describerTypesName)->default_value(describerTypesName),
      feature::EImageDescriberType_informations().c_str())
    ("maxResidualError", po::value<double>(&maxResidualError)->default_value(maxResidualError),
      "Upper bound of the residual error tolerance.");

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

  std::mt19937 randomNumberGenerator;

  // set verbose level
  system::Logger::get()->setLogLevel(verboseLevel);

  // Load input SfM_Data scene
  sfmData::SfMData sfmData;
  if(!sfmDataIO::Load(sfmData, sfmDataFilename, sfmDataIO::ESfMData::ALL))
  {
    ALICEVISION_LOG_ERROR("The input SfMData file '"<< sfmDataFilename << "' cannot be read");
    return EXIT_FAILURE;
  }

  if(sfmData.getPoses().empty() || sfmData.getLandmarks().empty())
  {
    ALICEVISION_LOG_ERROR("The input SfM_Data file have not 3D content to match with.");
    return EXIT_FAILURE;
  }

  // ---------------
  // Initialization
  // ---------------

  // Init the regions_type from the image describer file (used for image regions extraction)
  using namespace aliceVision::feature;

  // Get imageDescriberMethodType
  EImageDescriberType describerType = EImageDescriberType_stringToEnum(describerTypesName);
  
  // Get imageDecriber fom type
  std::unique_ptr<ImageDescriber> imageDescribers = createImageDescriber(describerType);

  // Load the SfM_Data region's views
  //-
  //-- Localization
  // - init the retrieval database
  // - Go along the sfm_data view
  // - extract the regions of the view
  // - try to locate the image
  //-
  sfm::SfMLocalizationSingle3DTrackObservationDatabase localizer;
  {
    feature::RegionsPerView regionsPerView;
    if (!sfm::loadRegionsPerView(regionsPerView, sfmData, featuresFolders, {describerType}))
    {
      ALICEVISION_LOG_ERROR("Invalid regions.");
      return EXIT_FAILURE;
    }

    if (outputFolder.empty())
    {
      ALICEVISION_LOG_ERROR("It is an invalid output folder");
      return EXIT_FAILURE;
    }

    if (!fs::exists(outputFolder))
      fs::create_directory(outputFolder);
    
    if(!localizer.Init(sfmData, regionsPerView))
    {
      ALICEVISION_LOG_ERROR("Cannot initialize the SfM localizer");
    }
  }
  
  std::vector<Vec3> vec_found_poses;

  ALICEVISION_LOG_INFO("SfM::localization => try with image: " << queryImage);
  std::unique_ptr<Regions> query_regions;
  image::Image<unsigned char> imageGray;
  {
    image::readImage(queryImage, imageGray, image::EImageColorSpace::NO_CONVERSION);

    // Compute features and descriptors
    imageDescribers->describe(imageGray, query_regions);
    ALICEVISION_LOG_INFO("# regions detected in query image: " << query_regions->RegionCount());
  }

  // Suppose intrinsic as unknown
  std::shared_ptr<camera::IntrinsicBase> optional_intrinsic (nullptr);

  geometry::Pose3 pose;
  sfm::ImageLocalizerMatchData matching_data;
  matching_data.error_max = maxResidualError;

  // Try to localize the image in the database thanks to its regions
  if (!localizer.Localize(
    Pair(imageGray.Width(), imageGray.Height()),
    optional_intrinsic.get(),
    *(query_regions.get()),
    randomNumberGenerator,
    pose,
    &matching_data))
  {
    ALICEVISION_LOG_ERROR("Cannot locate the image");
  }
  else
  {
    const bool b_new_intrinsic = (optional_intrinsic == nullptr);
    // A valid pose has been found (try to refine it):
    // If not intrinsic as input:
    //  init a new one from the projection matrix decomposition
    // Else use the existing one and consider as static.
    if (b_new_intrinsic)
    {
      // setup a default camera model from the found projection matrix
      Mat3 K, R;
      Vec3 t;
      KRt_from_P(matching_data.projection_matrix, &K, &R, &t);

      const double focal = (K(0,0) + K(1,1))/2.0;
      const Vec2 principal_point(K(0,2), K(1,2));
      optional_intrinsic = std::make_shared<camera::PinholeRadialK3>(
        imageGray.Width(), imageGray.Height(),
        focal, principal_point(0), principal_point(1));
    }
    sfm::SfMLocalizer::RefinePose
    (
      optional_intrinsic.get(),
      pose, matching_data,
      true, b_new_intrinsic
    );

    vec_found_poses.push_back(pose.center());
  }

  // export the found camera position
  const std::string out_file_name = (fs::path(outputFolder) / "found_pose_centers.ply").string();
  {
    std::ofstream outfile;
    outfile.open(out_file_name.c_str(), std::ios_base::out);
    if (outfile.is_open()) {
      outfile << "ply"
       << "\n" << "format ascii 1.0"
       << "\n" << "element vertex " << vec_found_poses.size()
       << "\n" << "property float x"
       << "\n" << "property float y"
       << "\n" << "property float z"
       << "\n" << "property uchar red"
       << "\n" << "property uchar green"
       << "\n" << "property uchar blue"
       << "\n" << "end_header" << "\n";

      for(const Vec3 & pose_center: vec_found_poses) {
        outfile << pose_center.transpose() << " " << "255 0 0" << "\n";
      }

      outfile.close();
    }
  }
  return EXIT_FAILURE;
}
