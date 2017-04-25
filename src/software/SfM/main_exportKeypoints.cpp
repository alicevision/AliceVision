
// Copyright (c) 2012, 2013, 2015 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "openMVG/matching/indMatch.hpp"
#include "openMVG/matching/indMatch_utils.hpp"
#include "openMVG/features/svgVisualization.hpp"
#include "openMVG/image/image.hpp"
#include "openMVG/sfm/sfm.hpp"
#include "openMVG/sfm/pipelines/RegionsIO.hpp"

#include "third_party/cmdLine/cmdLine.h"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"
#include "third_party/progress/progress.hpp"
#include "third_party/vectorGraphics/svgDrawer.hpp"

#include <cstdlib>
#include <string>
#include <vector>
#include <fstream>
#include <map>

using namespace openMVG;
using namespace openMVG::matching;
using namespace openMVG::sfm;
using namespace svg;


int main(int argc, char ** argv)
{
  CmdLine cmd;

  std::string sSfM_Data_Filename;
  std::string describerMethods = "SIFT";
  std::string sMatchesDir = "";
  std::string sOutDir = "";

  cmd.add( make_option('i', sSfM_Data_Filename, "input_file") );
  cmd.add( make_option('m', describerMethods, "describerMethods") );
  cmd.add( make_option('d', sMatchesDir, "matchdir") );
  cmd.add( make_option('o', sOutDir, "outdir") );

  try {
      if (argc == 1) throw std::string("Invalid command line parameter.");
      cmd.process(argc, argv);
  } catch(const std::string& s) {
      std::cerr << "Export pairwise matches.\nUsage: " << argv[0] << "\n"
      << "[-i|--input_file file] path to a SfM_Data scene\n"
      << "[-m|--describerMethods]\n"
      << "  (methods to use to describe an image):\n"
      << "   SIFT (default),\n"
      << "   SIFT_FLOAT to use SIFT stored as float,\n"
      << "   AKAZE_FLOAT: AKAZE with floating point descriptors,\n"
      << "   AKAZE_MLDB:  AKAZE with binary descriptors\n"
#ifdef HAVE_CCTAG
      << "   CCTAG3: CCTAG markers with 3 crowns\n"
      << "   CCTAG4: CCTAG markers with 4 crowns\n"
      << "   SIFT_CCTAG3: CCTAG markers with 3 crowns\n" 
      << "   SIFT_CCTAG4: CCTAG markers with 4 crowns\n" 
#endif
      << "[-d|--matchdir path]\n"
      << "[-o|--outdir path]\n"
      << std::endl;

      std::cerr << s << std::endl;
      return EXIT_FAILURE;
  }

  if (sOutDir.empty())
  {
    std::cerr << "\nIt is an invalid output directory" << std::endl;
    return EXIT_FAILURE;
  }


  //---------------------------------------
  // Read SfM Scene (image view names)
  //---------------------------------------
  SfM_Data sfm_data;
  if (!Load(sfm_data, sSfM_Data_Filename, ESfM_Data(VIEWS|INTRINSICS))) {
    std::cerr << std::endl
      << "The input SfM_Data file \""<< sSfM_Data_Filename << "\" cannot be read." << std::endl;
    return EXIT_FAILURE;
  }

  //---------------------------------------
  // Load SfM Scene regions
  //---------------------------------------
  using namespace openMVG::features;
  
  // Get imageDescriberMethodType
  std::vector<EImageDescriberType> describerMethodTypes = EImageDescriberType_stringToEnums(describerMethods);

  // Read the features
  features::FeaturesPerView featuresPerView;
  if (!sfm::loadFeaturesPerView(featuresPerView, sfm_data, sMatchesDir, describerMethodTypes)) {
    std::cerr << std::endl
      << "Invalid features." << std::endl;
    return EXIT_FAILURE;
  }

  // ------------
  // For each image, export visually the keypoints
  // ------------

  stlplus::folder_create(sOutDir);
  std::cout << "\n Export extracted keypoints for all images" << std::endl;
  C_Progress_display my_progress_bar( sfm_data.views.size() );
  for(const auto &iterViews : sfm_data.views)
  {
    const View * view = iterViews.second.get();
    const std::string sView_filename = stlplus::create_filespec(sfm_data.s_root_path,
      view->s_Img_path);

    const std::pair<size_t, size_t>
      dimImage = std::make_pair(view->ui_width, view->ui_height);

    const MapFeaturesPerDesc& features = featuresPerView.getData().at(view->id_view);

    // output filename
    std::ostringstream os;
    os << stlplus::folder_append_separator(sOutDir)
      << stlplus::basename_part(sView_filename)
      << "_" << features.size() << "_.svg"; //TO CHECK

    features::saveFeatures2SVG(sView_filename,
                               dimImage,
                               featuresPerView.getData().at(view->id_view),
                               os.str());

    ++my_progress_bar;
  }
  
  return EXIT_SUCCESS;
}
