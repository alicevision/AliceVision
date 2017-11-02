// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfm/sfm.hpp>
#include <aliceVision/sfm/pipeline/regionsIO.hpp>
#include <aliceVision/feature/feature.hpp>
#include <aliceVision/image/image.hpp>
#include <aliceVision/system/Timer.hpp>

#include <dependencies/stlplus3/filesystemSimplified/file_system.hpp>

#include <boost/program_options.hpp>

#include <cstdlib>

using namespace aliceVision;
using namespace aliceVision::sfm;
using namespace std;
namespace po = boost::program_options;

// ---------------------------------------------------------------------------
// Image localization API sample:
// ---------------------------------------------------------------------------
// - Allow to locate an image to an existing SfM_reconstruction
//   if 3D-2D matches are found
// - A demonstration mode (default):
//   - try to locate all the view of the SfM_Data reconstruction
// ---------------------------------------------------------------------------
//
int main(int argc, char **argv)
{
  // command-line parameters

  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  std::string sfmDataFilename;
  std::string describerTypesName = feature::EImageDescriberType_enumToString(feature::EImageDescriberType::SIFT);
  std::string matchesDirectory;
  std::string outputDirectory;
  std::string queryImage;
  double maxResidualError = std::numeric_limits<double>::infinity();

  po::options_description allParams(
    "Image localization in an existing SfM reconstruction\n"
    "AliceVision sfmLocalization");

  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
      "SfMData file.")
    ("output,o", po::value<std::string>(&outputDirectory)->required(),
      "Output path.")
    ("matchesDirectory,m", po::value<std::string>(&matchesDirectory)->required(),
      "Path to a directory in which computed matches are stored.")
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

  // set verbose level
  system::Logger::get()->setLogLevel(verboseLevel);

  // Load input SfM_Data scene
  SfMData sfmData;
  if (!Load(sfmData, sfmDataFilename, ESfMData(ALL))) {
    std::cerr << std::endl
      << "The input SfM_Data file \""<< sfmDataFilename << "\" cannot be read." << std::endl;
    return EXIT_FAILURE;
  }

  if (sfmData.GetPoses().empty() || sfmData.GetLandmarks().empty())
  {
    std::cerr << std::endl
      << "The input SfM_Data file have not 3D content to match with." << std::endl;
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
    RegionsPerView regionsPerView;

    if (!sfm::loadRegionsPerView(regionsPerView, sfmData, matchesDirectory, {describerType}))
    {
      std::cerr << std::endl << "Invalid regions." << std::endl;
      return EXIT_FAILURE;
    }

    if (outputDirectory.empty())
    {
      std::cerr << "\nIt is an invalid output directory" << std::endl;
      return EXIT_FAILURE;
    }

    if (!stlplus::folder_exists(outputDirectory))
      stlplus::folder_create(outputDirectory);
    
    if (!localizer.Init(sfmData, regionsPerView))
    {
      std::cerr << "Cannot initialize the SfM localizer" << std::endl;
    }
  }
  
  std::vector<Vec3> vec_found_poses;

  if (!queryImage.empty())
  {
    std::cout << "SfM::localization => try with image: " << queryImage << std::endl;
    std::unique_ptr<Regions> query_regions;
    image::Image<unsigned char> imageGray;
    {
      if (!image::ReadImage(queryImage.c_str(), &imageGray))
      {
        std::cerr << "Cannot open the input provided image" << std::endl;
        return EXIT_FAILURE;
      }
      // Compute features and descriptors
      imageDescribers->Describe(imageGray, query_regions);
      std::cout << "#regions detected in query image: " << query_regions->RegionCount() << std::endl;
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
      pose,
      &matching_data))
    {
      std::cerr << "Cannot locate the image" << std::endl;
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
        KRt_From_P(matching_data.projection_matrix, &K, &R, &t);

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
  }
  else
  {
    std::cout << "----------------\n"
      << " DEMONSTRATION \n"
      << "----------------\n" << std::endl;
    std::cout << "Will try to locate all the view of the loaded sfm_data file." << std::endl;

    for (const auto & viewIter : sfmData.GetViews())
    {
      const View * view = viewIter.second.get();
      // Load an image, extract the regions and match
      const std::string sImagePath = sfmData.s_root_path + view->getImagePath();

      std::cout << "SfM::localization => try with image: " << sImagePath << std::endl;

      std::unique_ptr<Regions> query_regions;
      imageDescribers->Allocate(query_regions);
      const std::string basename = stlplus::basename_part(sImagePath);
      const std::string featFile = stlplus::create_filespec(matchesDirectory, basename, ".feat");
      const std::string descFile = stlplus::create_filespec(matchesDirectory, basename, ".desc");
      if (!query_regions->Load(featFile, descFile))
      {
        std::cerr << "Invalid regions files for the view: " << sImagePath << std::endl;
        continue;
      }

      // Initialize intrinsics data for the view if any
      std::shared_ptr<camera::IntrinsicBase> optional_intrinsic (nullptr);
      if (sfmData.GetIntrinsics().count(view->getIntrinsicId()))
      {
        optional_intrinsic = sfmData.GetIntrinsics().at(view->getIntrinsicId());
      }

      geometry::Pose3 pose;
      sfm::ImageLocalizerMatchData matching_data;
      matching_data.error_max = maxResidualError;

      // Try to localize the image in the database thanks to its regions
      if (!localizer.Localize(
        Pair(view->getWidth(), view->getHeight()),
        optional_intrinsic.get(),
        *(query_regions.get()),
        pose,
        &matching_data))
      {
        std::cerr << "Cannot locate the image" << std::endl;
      }
      else
      {
        const bool b_new_intrinsic = (optional_intrinsic == nullptr);
        // A valid pose has been found (try to refine it):
        // If no valid intrinsic as input:
        //  init a new one from the projection matrix decomposition
        // Else use the existing one and consider it as constant
        if (b_new_intrinsic)
        {
          // setup a default camera model from the found projection matrix
          Mat3 K, R;
          Vec3 t;
          KRt_From_P(matching_data.projection_matrix, &K, &R, &t);

          const double focal = (K(0,0) + K(1,1))/2.0;
          const Vec2 principal_point(K(0,2), K(1,2));
          optional_intrinsic = std::make_shared<camera::PinholeRadialK3>(
            view->getWidth(), view->getHeight(),
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
    }
    std::cout
      << "\n#images found: " << vec_found_poses.size()
      << "\n#sfm_data view count: " << sfmData.GetViews().size()
      << std::endl;
  }

  // Export the found camera position
  const std::string out_file_name = stlplus::create_filespec(outputDirectory, "found_pose_centers", "ply");
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

      for (const Vec3 & pose_center: vec_found_poses) {
        outfile << pose_center.transpose() << " " << "255 0 0" << "\n";
      }

      outfile.close();
    }
  }

  return EXIT_FAILURE;
}
