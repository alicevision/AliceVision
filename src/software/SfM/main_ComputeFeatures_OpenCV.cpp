
// Copyright (c) 2012, 2013 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.


#include "openMVG/image/image.hpp"
#include "openMVG/sfm/sfm.hpp"
#include "openMVG/system/timer.hpp"

/// Feature/Regions & Image describer interfaces
#include "openMVG/features/features.hpp"
#include "nonFree/sift/SIFT_OPENCV_Image_describer.hpp"
#include "openMVG/features/akaze/AKAZE_OCV_Image_describer.hpp"

#include <cereal/archives/json.hpp>

#include "third_party/cmdLine/cmdLine.h"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"
#include "third_party/progress/progress.hpp"

#include <cstdlib>
#include <fstream>
#include <algorithm>

using namespace openMVG;
using namespace openMVG::image;
using namespace openMVG::features;
using namespace openMVG::sfm;
using namespace std;

enum eGeometricModel
{
  FUNDAMENTAL_MATRIX = 0,
  ESSENTIAL_MATRIX   = 1,
  HOMOGRAPHY_MATRIX  = 2
};

enum ePairMode
{
  PAIR_EXHAUSTIVE = 0,
  PAIR_CONTIGUOUS = 1,
  PAIR_FROM_FILE  = 2
};

/// Compute between the Views
/// Compute view image description (feature & descriptor extraction using OpenCV)
/// Compute putative local feature matches (descriptor matching)
/// Compute geometric coherent feature matches (robust model estimation from putative matches)
/// Export computed data
int main(int argc, char **argv)
{
  CmdLine cmd;

  std::string sSfM_Data_Filename;
  std::string sOutDir = "";
  bool bForce = false;
#ifdef USE_OCVSIFT
  std::string sImage_Describer_Method = "AKAZE_OPENCV";
#endif
  std::string sFeaturePreset = "NORMAL";
  int rangeStart = -1;
  int rangeSize = 1;

  // required
  cmd.add( make_option('i', sSfM_Data_Filename, "input_file") );
  cmd.add( make_option('o', sOutDir, "outdir") );
  // Optional
  cmd.add( make_option('f', bForce, "force") );
#ifdef USE_OCVSIFT
  cmd.add( make_option('m', sImage_Describer_Method, "describerMethod") );
#endif
  cmd.add( make_option('p', sFeaturePreset, "describerPreset") );
  cmd.add( make_option('s', rangeStart, "range_start") );
  cmd.add( make_option('r', rangeSize, "range_size") );

  try {
      if (argc == 1) throw std::string("Invalid command line parameter.");
      cmd.process(argc, argv);
  } catch(const std::string& s) {
      std::cerr << "Usage: " << argv[0] << '\n'
      << "[-i|--input_file]: a SfM_Data file \n"
      << "[-o|--outdir] path \n"
      << "\n[Optional]\n"
      << "[-f|--force: Force to recompute data]\n"
#ifdef USE_OCVSIFT
      << "[-m|--describerMethod\n"
      << "  (method to use to describe an image):\n"
      << "   AKAZE_OPENCV (default),\n"
      << "   SIFT_OPENCV: SIFT FROM OPENCV,\n"
#endif
      << "[-p|--describerPreset]\n"
      << "  (used to control the Image_describer configuration):\n"
      << "   LOW,\n"
      << "   MEDIUM,\n"
      << "   NORMAL (default),\n"
      << "   HIGH,\n"
      << "   ULTRA: !!Can take long time!!\n"
      << "[-s]--range_start] range image index start\n"
      << "[-r]--range_size] range size\n"
      << std::endl;

      std::cerr << s << std::endl;
      return EXIT_FAILURE;
  }

  std::cout << " You called : " <<std::endl
            << argv[0] << std::endl
            << "--input_file " << sSfM_Data_Filename << std::endl
            << "--outdir " << sOutDir << std::endl
#ifdef USE_OCVSIFT
            << "--describerMethod " << sImage_Describer_Method << std::endl
#endif
            << "--describerPreset " << sFeaturePreset << std::endl
            << "--range_start " << rangeStart << std::endl
            << "--range_size " << rangeSize << std::endl;

  if (sOutDir.empty())  {
    std::cerr << "\nIt is an invalid output directory" << std::endl;
    return EXIT_FAILURE;
  }

  // Create output dir
  if (!stlplus::folder_exists(sOutDir))
  {
    if (!stlplus::folder_create(sOutDir))
    {
      std::cerr << "Cannot create output directory" << std::endl;
      return EXIT_FAILURE;
    }
  }

  //---------------------------------------
  // a. Load input scene
  //---------------------------------------
  SfM_Data sfm_data;
  if (!Load(sfm_data, sSfM_Data_Filename, ESfM_Data(VIEWS|INTRINSICS))) {
    std::cerr << std::endl
      << "The input file \""<< sSfM_Data_Filename << "\" cannot be read" << std::endl;
    return false;
  }

  // Init the image_describer
  // - retrieve the used one in case of pre-computed features
  // - else create the desired one

  using namespace openMVG::features;
  std::unique_ptr<Image_describer> image_describer;

  const std::string sImage_describer = stlplus::create_filespec(sOutDir, "image_describer", "json");
  if (stlplus::is_file(sImage_describer))
  {
    // Dynamically load the image_describer from the file (will restore old used settings)
    std::ifstream stream(sImage_describer.c_str());
    if (!stream.is_open())
      return false;

    cereal::JSONInputArchive archive(stream);
    archive(cereal::make_nvp("image_describer", image_describer));
  }
  else
  {
#ifdef USE_OCVSIFT
    if (sImage_Describer_Method == "AKAZE_OPENCV")
    {
      image_describer.reset(new AKAZE_OCV_Image_describer);
    }
    else
    if (sImage_Describer_Method == "SIFT_OPENCV")
    {
      image_describer.reset(new SIFT_OPENCV_Image_describer());
    }
    else
    {
      std::cerr << "Unknown image describer method." << std::endl;
      return EXIT_FAILURE;
    }
#else
    image_describer.reset(new AKAZE_OCV_Image_describer);
#endif

    image_describer->Set_configuration_preset(sFeaturePreset);

    // Export the used Image_describer and region type for:
    // - dynamic future regions computation and/or loading
    {
      std::ofstream stream(sImage_describer.c_str());
      if (!stream.is_open())
        return false;

      cereal::JSONOutputArchive archive(stream);
      archive(cereal::make_nvp("image_describer", image_describer));
      std::unique_ptr<Regions> regionsType;
      image_describer->Allocate(regionsType);
      archive(cereal::make_nvp("regions_type", regionsType));
    }
  }

  // Feature extraction routines
  // For each View of the SfM_Data container:
  // - if regions file exist continue,
  // - if no file, compute features
  {
    system::Timer timer;
    Image<unsigned char> imageGray;
    C_Progress_display my_progress_bar( sfm_data.GetViews().size(),
      std::cout, "\n- EXTRACT FEATURES -\n" );
    Views::const_iterator iterViews = sfm_data.views.begin();
    Views::const_iterator iterViewsEnd = sfm_data.views.end();
    if(rangeStart != -1)
    {
      if(rangeStart < 0 || rangeStart > sfm_data.views.size())
      {
        std::cerr << "Bad specific index" << std::endl;
        return EXIT_FAILURE;
      }
      if(rangeSize < 0 || rangeSize > sfm_data.views.size())
      {
        std::cerr << "Bad range size. " << std::endl;
        return EXIT_FAILURE;
      }
      if(rangeStart + rangeSize > sfm_data.views.size())
        rangeSize = sfm_data.views.size() - rangeStart;

      std::advance(iterViews, rangeStart);
      iterViewsEnd = iterViews;
      std::advance(iterViewsEnd, rangeSize);
    }
    for(;
        iterViews != iterViewsEnd;
        ++iterViews, ++my_progress_bar)
    {
      const View * view = iterViews->second.get();
      const std::string sView_filename = stlplus::create_filespec(sfm_data.s_root_path,
        view->s_Img_path);
      const std::string sFeat = stlplus::create_filespec(sOutDir,
        stlplus::basename_part(std::to_string(view->id_view)), "feat");
      const std::string sDesc = stlplus::create_filespec(sOutDir,
        stlplus::basename_part(std::to_string(view->id_view)), "desc");
      

      //If features or descriptors file are missing, compute them
      if (bForce || !stlplus::file_exists(sFeat) || !stlplus::file_exists(sDesc))
      {
        if (!ReadImage(sView_filename.c_str(), &imageGray))
        {
          std::cout << "Error: can't read image: " << sView_filename << std::endl;
          continue;
        }

        // Compute features and descriptors and export them to files
        std::cout << "Extracting features from image " << view->id_view << " - (" << sFeat << ")" << std::endl;
        std::unique_ptr<Regions> regions;

        image_describer->Describe(imageGray, regions);
        image_describer->Save(regions.get(), sFeat, sDesc);
      }
    }
    std::cout << "Task done in (s): " << timer.elapsed() << std::endl;
  }
  return EXIT_SUCCESS;
}
