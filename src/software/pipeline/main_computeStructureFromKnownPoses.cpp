// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#include "aliceVision/config.hpp"
#include "aliceVision/sfm/sfm.hpp"
#include "aliceVision/sfm/pipelines/RegionsIO.hpp"
#include "aliceVision/matching/indMatch.hpp"
#include "aliceVision/system/timer.hpp"

#include "dependencies/cmdLine/cmdLine.h"
#include "dependencies/stlplus3/filesystemSimplified/file_system.hpp"

using namespace aliceVision;
using namespace aliceVision::sfm;

/// Compute the structure of a scene according existing camera poses.
int main(int argc, char **argv)
{
  using namespace std;
  std::cout << "Compute Structure from the provided poses" << std::endl;

  CmdLine cmd;

  std::string sSfM_Data_Filename;
  std::string describerMethods = "SIFT";
  std::string sFeaturesDir;
  std::string sMatchesDir;
  std::string sMatchesGeometricModel = "f";
  std::string sOutFile = "";

  cmd.add( make_option('i', sSfM_Data_Filename, "input_file") );
  cmd.add( make_option('d', describerMethods, "describerMethods") );
  cmd.add( make_option('f', sFeaturesDir, "feat_dir") );
  cmd.add( make_option('m', sMatchesDir, "match_dir") );
  cmd.add( make_option('o', sOutFile, "output_file") );
  cmd.add( make_option('g', sMatchesGeometricModel, "matchesGeometricModel"));

  try {
    if (argc == 1) throw std::string("Invalid command line parameter.");
    cmd.process(argc, argv);
  } catch(const std::string& s) {
    std::cerr << "Usage: " << argv[0] << "\n"
        "[-i|--input_file] path to a SfM_Data scene\n"
        "[-d|--describerMethods]\n"
        "  (methods to use to describe an image):\n"
        "   SIFT (default),\n"
        "   SIFT_FLOAT to use SIFT stored as float,\n"
        "   AKAZE: AKAZE with floating point descriptors,\n"
        "   AKAZE_MLDB:  AKAZE with binary descriptors\n"
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_CCTAG)
        "   CCTAG3: CCTAG markers with 3 crowns\n"
        "   CCTAG4: CCTAG markers with 4 crowns\n"
#endif
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_OPENCV)
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_OCVSIFT)
        "   SIFT_OCV: OpenCV SIFT\n"
#endif
        "   AKAZE_OCV: OpenCV AKAZE\n"
#endif
        "[-o|--output_file] file where the output data will be stored "
           "(i.e. path/sfm_data_structure.bin)\n"
        "[-f|--feat_dir] path to the features and descriptors that "
           "corresponds to the provided SfM_Data scene\n"
        "\n[Optional]\n"
        "[-m|--match_dir] path to the matches files "
        "(if not provided the images connections will be computed from Frustrums intersections)\n"
        "[-g|--matchesGeometricModel MODEL] matching geometric model used: 'f' (default), 'e' or 'h'"
    << std::endl;

    std::cerr << s << std::endl;
    return EXIT_FAILURE;
  }
  
  // Load input SfM_Data scene
  SfM_Data sfm_data;
  if (!Load(sfm_data, sSfM_Data_Filename, ESfM_Data(VIEWS|INTRINSICS|EXTRINSICS))) {
    std::cerr << std::endl
      << "The input SfM_Data file \""<< sSfM_Data_Filename << "\" cannot be read." << std::endl;
    return EXIT_FAILURE;
  }

  // Init the regions_type from the image describer file (used for image regions extraction)
  using namespace aliceVision::feature;
  
  // Get imageDescriberMethodType
  std::vector<EImageDescriberType> describerMethodTypes = EImageDescriberType_stringToEnums(describerMethods);

  // Prepare the Regions provider
  RegionsPerView regionsPerView;
  if(!sfm::loadRegionsPerView(regionsPerView, sfm_data, sFeaturesDir, describerMethodTypes))
  {
    std::cerr << std::endl
      << "Invalid regions." << std::endl;
    return EXIT_FAILURE;
  }

  //--
  //- Pair selection method:
  //  - geometry guided -> camera frustum intersection,
  //  - putative matches guided (photometric matches)
  //     (keep pairs that have valid Intrinsic & Pose ids).
  //--
  Pair_Set pairs;
  if (sMatchesDir.empty())
  {
    // No image pair provided, so we use cameras frustum intersection.
    // Build the list of connected images pairs from frustum intersections
    pairs = Frustum_Filter(sfm_data).getFrustumIntersectionPairs();
  }
  else
  {
    // Load pre-computed matches
    matching::PairwiseMatches matches;
    if (!matching::Load(matches, sfm_data.GetViewsKeys(), sMatchesDir, describerMethodTypes, sMatchesGeometricModel))
    {
      std::cerr<< "Unable to read the matches file." << std::endl;
      return EXIT_FAILURE;
    }
    pairs = matching::getImagePairs(matches);
    // Keep only Pairs that belong to valid view indexes.
    const std::set<IndexT> valid_viewIdx = sfm_data.getValidViews();
    pairs = Pair_filter(pairs, valid_viewIdx);
  }

  aliceVision::system::Timer timer;

  // Clear previous 3D landmarks
  sfm_data.structure.clear();

  //------------------------------------------
  // Compute Structure from known camera poses
  //------------------------------------------
  SfM_Data_Structure_Estimation_From_Known_Poses structure_estimator;
  structure_estimator.match(sfm_data, pairs, regionsPerView);

  // Unload descriptors before triangulation
  regionsPerView.clearDescriptors();

  // Filter matches
  structure_estimator.filter(sfm_data, pairs, regionsPerView);
  // Create 3D landmarks
  structure_estimator.triangulate(sfm_data, regionsPerView);

  RemoveOutliers_AngleError(sfm_data, 2.0);

  std::cout
    << "\nStructure estimation took (s): " << timer.elapsed() << "." << std::endl
    << "#landmark found: " << sfm_data.GetLandmarks().size() << std::endl;

  if (stlplus::extension_part(sOutFile) != "ply")
  {
    Save(sfm_data,
      stlplus::create_filespec(
        stlplus::folder_part(sOutFile),
        stlplus::basename_part(sOutFile), "ply"),
      ESfM_Data(ALL));
  }

  if (Save(sfm_data, sOutFile, ESfM_Data(ALL)))
    return EXIT_SUCCESS;
  
  std::cout << "Error while saving the sfm data!" << std::endl;
  return EXIT_FAILURE;
}
