
// Copyright (c) 2012, 2013 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include <cstdlib>

#include "openMVG/features/ImageDescriberCommon.hpp"
#include "openMVG/sfm/sfm.hpp"
#include "openMVG/sfm/pipelines/RegionsIO.hpp"
#include "openMVG/system/timer.hpp"

#include "third_party/cmdLine/cmdLine.h"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

using namespace openMVG;
using namespace openMVG::cameras;
using namespace openMVG::sfm;
using namespace openMVG::features;

/**
 * @brief Retrieve the view id in the sfmData from the image filename.
 *
 * @param[in] sfm_data the SfM scene
 * @param[in] initialName the image name to find (filename or path)
 * @param[out] out_viewId the id found
 * @return if a view is found
 */
bool retrieveViewIdFromImageName(
  const SfM_Data & sfm_data,
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
  using namespace std;
  std::cout << "Sequential/Incremental reconstruction" << std::endl
            << " Perform incremental SfM (Initial Pair Essential + Resection)." << std::endl
            << std::endl;

  CmdLine cmd;

  std::string sSfM_Data_Filename;
  std::string describerMethods = "SIFT";
  std::string sMatchesDir;
  std::string sFeaturesDir;
  std::string sOutDir = "";
  std::string sOutSfMDataFilepath = "";
  std::string sOutInterFileExtension = ".ply";
  std::pair<std::string,std::string> initialPairString("","");
  bool bRefineIntrinsics = true;
  int minInputTrackLength = 2;
  int i_User_camera_model = PINHOLE_CAMERA_RADIAL3;
  bool allowUserInteraction = true;
  bool localBA = false;

  cmd.add( make_option('i', sSfM_Data_Filename, "input_file") );
  cmd.add( make_option('d', describerMethods, "describerMethods") );
  cmd.add( make_option('m', sMatchesDir, "matchdir") );
  cmd.add( make_option('F', sFeaturesDir, "featuresDir") );
  cmd.add( make_option('o', sOutDir, "outdir") );
  cmd.add( make_option('s', sOutSfMDataFilepath, "out_sfmdata_file") );
  cmd.add( make_option('e', sOutInterFileExtension, "inter_file_extension") );
  cmd.add( make_option('a', initialPairString.first, "initialPairA") );
  cmd.add( make_option('b', initialPairString.second, "initialPairB") );
  cmd.add( make_option('c', i_User_camera_model, "camera_model") );
  cmd.add( make_option('f', bRefineIntrinsics, "refineIntrinsics") );
  cmd.add( make_option('t', minInputTrackLength, "minInputTrackLength") );
  cmd.add( make_option('u', allowUserInteraction, "allowUserInteraction") );
  cmd.add( make_option('l', localBA, "localBA") );

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
#if OPENMVG_IS_DEFINED(OPENMVG_HAVE_CCTAG)
    << "   CCTAG3: CCTAG markers with 3 crowns\n"
    << "   CCTAG4: CCTAG markers with 4 crowns\n"
#endif //OPENMVG_HAVE_CCTAG
#if OPENMVG_IS_DEFINED(OPENMVG_HAVE_OPENCV)
#if OPENMVG_IS_DEFINED(OPENMVG_HAVE_OCVSIFT)
    << "   SIFT_OCV: OpenCV SIFT\n"
#endif //OPENMVG_HAVE_OCVSIFT
    << "   AKAZE_OCV: OpenCV AKAZE\n"
#endif //OPENMVG_HAVE_OPENCV
    << "[-m|--matchdir] path to the matches that corresponds to the provided SfM_Data scene\n"
    << "[-F|--featuresDir] path to directory containing the extracted features (default: $matchdir)\n"
    << "[-o|--outdir] path where the output data will be stored\n"
    << "[-s|--out_sfmdata_file] path of the output sfmdata file (default: $outdir/sfm_data.json)\n"
    << "[-e|--inter_file_extension] extension of the intermediate file export (default: .ply)\n"
    << "[-a|--initialPairA] filename of the first image (without path)\n"
    << "[-b|--initialPairB] filename of the second image (without path)\n"
    << "[-c|--camera_model] Camera model type for view with unknown intrinsic:\n"
      << "\t 1: Pinhole \n"
      << "\t 2: Pinhole radial 1\n"
      << "\t 3: Pinhole radial 3 (default)\n"
    << "[-f|--refineIntrinsics] \n"
    << "\t 0-> intrinsic parameters are kept as constant\n"
    << "\t 1-> refine intrinsic parameters (default). \n"
    << "[-t|--minInputTrackLength N] minimum track length in input of SfM (default: 2)\n"
    << "[-p|--matchFilePerImage] \n"
    << "\t To use one match file per image instead of a global file.\n"
    << "[-u|--allowUserInteraction] Enable/Disable user interactions. (default: true)\n"
    << "\t If the process is done on renderfarm, it doesn't make sense to wait for user inputs.\n"    
    << "[-l|--localBA] Enable/Disable the Local Bundle Adjustment strategy. (default: false)\n"
    << "\t It may be helpfull in case of big dataset of images.\n"
    << std::endl;

    std::cerr << s << std::endl;
    return EXIT_FAILURE;
  }
    
  if(sOutSfMDataFilepath.empty())
    sOutSfMDataFilepath = stlplus::create_filespec(sOutDir, "sfm_data", "json");

  // Load input SfM_Data scene
  SfM_Data sfm_data;
  if (!Load(sfm_data, sSfM_Data_Filename, ESfM_Data(ALL))) {
    std::cerr << std::endl
      << "The input SfM_Data file \""<< sSfM_Data_Filename << "\" cannot be read." << std::endl;
    return EXIT_FAILURE;
  }

  if(sFeaturesDir.empty()) {
    sFeaturesDir = sMatchesDir;
  }

  // Get imageDescriberMethodType
  const std::vector<features::EImageDescriberType> describerTypes = features::EImageDescriberType_stringToEnums(describerMethods);

  // Features reading
  features::FeaturesPerView featuresPerView;
  if(!sfm::loadFeaturesPerView(featuresPerView, sfm_data, sFeaturesDir, describerTypes))
  {
    std::cerr << std::endl
      << "Invalid features." << std::endl;
    return EXIT_FAILURE;
  }
  
  // Matches reading
  matching::PairwiseMatches pairwiseMatches;

  if(!loadPairwiseMatches(pairwiseMatches, sfm_data, sMatchesDir, describerTypes, "f"))
  {
    std::cerr << std::endl << "Unable to load matches file from: " << sMatchesDir << std::endl;
    return EXIT_FAILURE;
  }

  if (sOutDir.empty())
  {
    std::cerr << "\nIt is an invalid output directory" << std::endl;
    return EXIT_FAILURE;
  }

  if (!stlplus::folder_exists(sOutDir))
    stlplus::folder_create(sOutDir);

  // Local bundle adjustment 
  if (localBA)
  {
    if (!stlplus::folder_exists(sOutDir+"/LocalBA/"))
      stlplus::folder_create(sOutDir+"/LocalBA/");
    if (!stlplus::folder_exists(sOutDir+"/LocalBA/K/"))
      stlplus::folder_create(sOutDir+"/LocalBA/K/");  
  }
    
  //---------------------------------------
  // Sequential reconstruction process
  //---------------------------------------

  openMVG::system::Timer timer;
  SequentialSfMReconstructionEngine sfmEngine(
    sfm_data,
    sOutDir,
    stlplus::create_filespec(sOutDir, "Reconstruction_Report.html"));

  // Configure the featuresPerView & the matches_provider
  sfmEngine.setFeatures(&featuresPerView);
  sfmEngine.setMatches(&pairwiseMatches);

  // Configure reconstruction parameters
  sfmEngine.Set_bFixedIntrinsics(!bRefineIntrinsics);
  sfmEngine.SetUnknownCameraType(EINTRINSIC(i_User_camera_model));
  sfmEngine.setMinInputTrackLength(minInputTrackLength);
  sfmEngine.setSfmdataInterFileExtension(sOutInterFileExtension);
  sfmEngine.setAllowUserInteraction(allowUserInteraction);
  sfmEngine.setUseLocalBundleAdjustmentStrategy(localBA);

  // Handle Initial pair parameter
  if (!initialPairString.first.empty() && !initialPairString.second.empty())
  {
    if (initialPairString.first == initialPairString.second)
    {
      std::cerr << "\nInvalid image names. You cannot use the same image to initialize a pair." << std::endl;
      return EXIT_FAILURE;
    }

    Pair initialPairIndex;
    if(!retrieveViewIdFromImageName(sfm_data, initialPairString.first, initialPairIndex.first)
            || !retrieveViewIdFromImageName(sfm_data, initialPairString.second, initialPairIndex.second))
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
  
  sfmEngine.Get_SfM_Data().setFeatureFolder(sFeaturesDir);
  sfmEngine.Get_SfM_Data().setMatchingFolder(sMatchesDir);

  std::cout << std::endl << " Total Ac-Sfm took (s): " << timer.elapsed() << std::endl;

  std::cout << "...Generating SfM_Report.html" << std::endl;
  Generate_SfM_Report(sfmEngine.Get_SfM_Data(),
    stlplus::create_filespec(sOutDir, "SfMReconstruction_Report.html"));

  //-- Export to disk computed scene (data & visualizable results)
  std::cout << "...Export SfM_Data to disk:" << std::endl;
  std::cout << "   " << sOutSfMDataFilepath << std::endl;

  Save(sfmEngine.Get_SfM_Data(), sOutSfMDataFilepath, ESfM_Data(ALL));

  Save(sfmEngine.Get_SfM_Data(), stlplus::create_filespec(sOutDir, "cloud_and_poses", sOutInterFileExtension), ESfM_Data(VIEWS | EXTRINSICS | INTRINSICS | STRUCTURE));

  return EXIT_SUCCESS;
}
