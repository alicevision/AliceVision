// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#include <cstdlib>

#include "aliceVision/sfm/pipelines/RegionsIO.hpp"
#include "aliceVision/features/ImageDescriberCommon.hpp"
#include "aliceVision/sfm/pipelines/global/sfm_global_engine_relative_motions.hpp"
#include "aliceVision/system/timer.hpp"

using namespace aliceVision;
using namespace aliceVision::sfm;
using namespace aliceVision::features;

#include "third_party/cmdLine/cmdLine.h"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"


int main(int argc, char **argv)
{
  using namespace std;
  std::cout << std::endl
    << "-----------------------------------------------------------\n"
    << "Global Structure from Motion:\n"
    << "-----------------------------------------------------------\n"
    << "Open Source implementation of the paper:\n"
    << "\"Global Fusion of Relative Motions for "
    << "Robust, Accurate and Scalable Structure from Motion.\"\n"
    << "Pierre Moulon, Pascal Monasse and Renaud Marlet. "
    << " ICCV 2013." << std::endl
    << "------------------------------------------------------------"
    << std::endl;


  CmdLine cmd;

  std::string sSfM_Data_Filename;
  std::string describerMethods = "SIFT";
  std::string sMatchesDir;
  std::string sOutDir = "";
  std::string sOutSfMDataFilepath = "";
  int iRotationAveragingMethod = int (ROTATION_AVERAGING_L2);
  int iTranslationAveragingMethod = int (TRANSLATION_AVERAGING_SOFTL1);
  bool bRefineIntrinsics = true;

  cmd.add( make_option('i', sSfM_Data_Filename, "input_file") );
  cmd.add( make_option('d', describerMethods, "describerMethods") );
  cmd.add( make_option('m', sMatchesDir, "matchdir") );
  cmd.add( make_option('o', sOutDir, "outdir") );
  cmd.add( make_option('s', sOutSfMDataFilepath, "out_sfmdata_file") );
  cmd.add( make_option('r', iRotationAveragingMethod, "rotationAveraging") );
  cmd.add( make_option('t', iTranslationAveragingMethod, "translationAveraging") );
  cmd.add( make_option('f', bRefineIntrinsics, "refineIntrinsics") );

  try {
    if (argc == 1) throw std::string("Invalid parameter.");
    cmd.process(argc, argv);
  } catch(const std::string& s) {
    std::cerr << "Usage: " << argv[0] << '\n'
    << "[-i|--input_file] path to a SfM_Data scene\n"
    << "[-d|--describerMethods]\n"
    << "  (methods to use to describe an image):\n"
    << "   SIFT (default),\n"
    << "   SIFT_FLOAT to use SIFT stored as float,\n"
    << "   AKAZE: AKAZE with floating point descriptors,\n"
    << "   AKAZE_MLDB:  AKAZE with binary descriptors\n"
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_CCTAG)
    << "   CCTAG3: CCTAG markers with 3 crowns\n"
    << "   CCTAG4: CCTAG markers with 4 crowns\n"
#endif //ALICEVISION_HAVE_CCTAG
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_OPENCV)
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_OCVSIFT)
    << "   SIFT_OCV: OpenCV SIFT\n"
#endif //ALICEVISION_HAVE_OCVSIFT
    << "   AKAZE_OCV: OpenCV AKAZE\n"
#endif //ALICEVISION_HAVE_OPENCV
    << "[-m|--matchdir] path to the matches that corresponds to the provided SfM_Data scene\n"
    << "[-o|--outdir] path where the output data will be stored\n"
    << "\n[Optional]\n"
    << "[-s|--out_sfmdata_file] path of the output sfmdata file (default: $outdir/sfm_data.json)\n"
    << "[-r|--rotationAveraging]\n"
    << "\t 1 -> L1 minimization\n"
    << "\t 2 -> L2 minimization (default)\n"
    << "[-t|--translationAveraging]:\n"
    << "\t 1 -> L1 minimization\n"
    << "\t 2 -> L2 minimization of sum of squared Chordal distances\n"
    << "\t 3 -> SoftL1 minimization (default)\n"
    << "[-f|--refineIntrinsics]\n"
    << "\t 0-> intrinsic parameters are kept as constant\n"
    << "\t 1-> refine intrinsic parameters (default). \n"
    << std::endl;

    std::cerr << s << std::endl;
    return EXIT_FAILURE;
  }

  if(sOutSfMDataFilepath.empty())
    sOutSfMDataFilepath = stlplus::create_filespec(sOutDir, "sfm_data", "json");

  if (iRotationAveragingMethod < ROTATION_AVERAGING_L1 ||
      iRotationAveragingMethod > ROTATION_AVERAGING_L2 )  {
    std::cerr << "\n Rotation averaging method is invalid" << std::endl;
    return EXIT_FAILURE;
  }

  if (iTranslationAveragingMethod < TRANSLATION_AVERAGING_L1 ||
      iTranslationAveragingMethod > TRANSLATION_AVERAGING_SOFTL1 )  {
    std::cerr << "\n Translation averaging method is invalid" << std::endl;
    return EXIT_FAILURE;
  }

  // Load input SfM_Data scene
  SfM_Data sfmData;
  if (!Load(sfmData, sSfM_Data_Filename, ESfM_Data(VIEWS|INTRINSICS))) {
    std::cerr << std::endl
      << "The input SfM_Data file \""<< sSfM_Data_Filename << "\" cannot be read." << std::endl;
    return EXIT_FAILURE;
  }

  if(!sfmData.getRigs().empty() > 0)
  {
    std::cerr << "Rigs are not currently supported in Global SfM." << std::endl
              << "Please use Incremental SfM. Aborted" << std::endl;

    return EXIT_FAILURE;
  }

  // Get describerTypes
  const std::vector<features::EImageDescriberType> describerTypes = features::EImageDescriberType_stringToEnums(describerMethods);

  // Features reading
  FeaturesPerView featuresPerView;
  if (!sfm::loadFeaturesPerView(featuresPerView, sfmData, sMatchesDir, describerTypes)) {
    std::cerr << std::endl
      << "Invalid features." << std::endl;
    return EXIT_FAILURE;
  }
  // Matches reading
  matching::PairwiseMatches pairwiseMatches;
  // Load the match file (try to read the two matches file formats)

  if(!sfm::loadPairwiseMatches(pairwiseMatches, sfmData, sMatchesDir, describerTypes, "e"))
  {
    std::cerr << std::endl << "Unable to load matches files from: " << sMatchesDir << std::endl;
    return EXIT_FAILURE;
  }

  if (sOutDir.empty())
  {
    std::cerr << "\nIt is an invalid output directory" << std::endl;
    return EXIT_FAILURE;
  }

  if (!stlplus::folder_exists(sOutDir))
    stlplus::folder_create(sOutDir);

  //---------------------------------------
  // Global SfM reconstruction process
  //---------------------------------------

  aliceVision::system::Timer timer;
  GlobalSfMReconstructionEngine_RelativeMotions sfmEngine(
    sfmData,
    sOutDir,
    stlplus::create_filespec(sOutDir, "Reconstruction_Report.html"));

  // Configure the featuresPerView & the matches_provider
  sfmEngine.SetFeaturesProvider(&featuresPerView);
  sfmEngine.SetMatchesProvider(&pairwiseMatches);

  // Configure reconstruction parameters
  sfmEngine.Set_bFixedIntrinsics(!bRefineIntrinsics);

  // Configure motion averaging method
  sfmEngine.SetRotationAveragingMethod(
    ERotationAveragingMethod(iRotationAveragingMethod));
  sfmEngine.SetTranslationAveragingMethod(
    ETranslationAveragingMethod(iTranslationAveragingMethod));

  if (!sfmEngine.Process())
  {
    return EXIT_FAILURE;
  }

  // get the color for the 3D points
  if(!sfmEngine.Colorize())
  {
    std::cerr << "Colorize failed!" << std::endl;
  }

  std::cout << std::endl << " Total Ac-Global-Sfm took (s): " << timer.elapsed() << std::endl;

  std::cout << "...Generating SfM_Report.html" << std::endl;
  Generate_SfM_Report(sfmEngine.Get_SfM_Data(),
    stlplus::create_filespec(sOutDir, "SfMReconstruction_Report.html"));

  //-- Export to disk computed scene (data & visualizable results)
  std::cout << "...Export SfM_Data to disk." << std::endl;
  Save(sfmEngine.Get_SfM_Data(), sOutSfMDataFilepath, ESfM_Data(ALL));

  Save(sfmEngine.Get_SfM_Data(),
    stlplus::create_filespec(sOutDir, "cloud_and_poses", ".ply"),
    ESfM_Data(ALL));

  return EXIT_SUCCESS;
}
