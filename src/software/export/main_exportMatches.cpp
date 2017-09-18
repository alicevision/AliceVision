// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#include "aliceVision/matching/IndMatch.hpp"
#include "aliceVision/matching/io.hpp"
#include "aliceVision/image/image.hpp"
#include "aliceVision/sfm/sfm.hpp"
#include "aliceVision/sfm/pipelines/RegionsIO.hpp"
#include "aliceVision/feature/svgVisualization.hpp"

#include "dependencies/cmdLine/cmdLine.h"
#include "dependencies/stlplus3/filesystemSimplified/file_system.hpp"
#include "dependencies/progress/progress.hpp"
#include "dependencies/vectorGraphics/svgDrawer.hpp"

#include <cstdlib>
#include <string>
#include <vector>
#include <fstream>
#include <map>

using namespace aliceVision;
using namespace aliceVision::feature;
using namespace aliceVision::matching;
using namespace aliceVision::sfm;
using namespace svg;

// Convert HUE color to RGB
inline float hue2rgb(float p, float q, float t){
  if(t < 0) t += 1;
  if(t > 1) t -= 1;
  if(t < 1.f/6.f) return p + (q - p) * 6.f * t;
  if(t < 1.f/2.f) return q;
  if(t < 2.f/3.f) return p + (q - p) * (2.f/3.f - t) * 6.f;
  return p;
}

 //
 // Converts an HSL color value to RGB. Conversion formula
 // adapted from http://en.wikipedia.org/wiki/HSL_color_space.
 // Assumes h, s, and l are contained in the set [0, 1] and
 // returns r, g, and b in the set [0, 255].
 void hslToRgb(
   float h, float s, float l,
   unsigned char & r, unsigned char & g, unsigned char & b)
{
  if(s == 0){
    r = g = b = static_cast<unsigned char>(l * 255.f); // achromatic
  }else{
    const float q = l < 0.5f ? l * (1 + s) : l + s - l * s;
    const float p = 2.f * l - q;
    r = static_cast<unsigned char>(hue2rgb(p, q, h + 1.f/3.f) * 255.f);
    g = static_cast<unsigned char>(hue2rgb(p, q, h) * 255.f);
    b = static_cast<unsigned char>(hue2rgb(p, q, h - 1.f/3.f) * 255.f);
  }
}


int main(int argc, char ** argv)
{
  CmdLine cmd;

  std::string sSfM_Data_Filename;
  std::string describerMethods = "SIFT";
  std::string sMatchesDir = "";
  std::string sMatchGeometricModel = "f";
  std::string sOutDir = "";

  cmd.add( make_option('i', sSfM_Data_Filename, "input_file") );
  cmd.add( make_option('m', describerMethods, "describerMethods") );
  cmd.add( make_option('d', sMatchesDir, "matchdir") );
  cmd.add( make_option('g', sMatchGeometricModel, "geometric_model") );
  cmd.add( make_option('o', sOutDir, "outdir") );

  try {
      if (argc == 1) throw std::string("Invalid command line parameter.");
      cmd.process(argc, argv);
  } catch(const std::string& s) {
      std::cerr << "Export pairwise matches.\nUsage: " << argv[0] << "\n"
      << "[-i|--input_file FILE] path to a SfM_Data scene\n"
      << "[-m|--describerMethods]\n"
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
      << "[-d|--matchdir PATH] path to the folder with all features and match files\n"
      << "[-g|--geometric_model MODEL] model used for the matching:\n"
      << "   f: (default) fundamental matrix,\n"
      << "   e: essential matrix,\n"
      << "   h: homography matrix.\n"
      << "[-o|--outdir PATH]\n"
      << std::endl;

      std::cerr << s << std::endl;
      return EXIT_FAILURE;
  }

  if (sOutDir.empty())  {
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
  // Get imageDescriberMethodType
  std::vector<EImageDescriberType> describerMethodTypes = EImageDescriberType_stringToEnums(describerMethods);

  // Read the features
  feature::FeaturesPerView featuresPerView;
  if(!sfm::loadFeaturesPerView(featuresPerView, sfm_data, sMatchesDir, describerMethodTypes))
  {
    std::cerr << std::endl
      << "Invalid features." << std::endl;
    return EXIT_FAILURE;
  }

  matching::PairwiseMatches pairwiseMatches;
  if(!sfm::loadPairwiseMatches(pairwiseMatches, sfm_data, sMatchesDir, describerMethodTypes, sMatchGeometricModel))
  {
    std::cerr << "\nInvalid matches file." << std::endl;
    return EXIT_FAILURE;
  }

  // ------------
  // For each pair, export the matches
  // ------------

  stlplus::folder_create(sOutDir);
  std::cout << "\n Export pairwise matches" << std::endl;
  const Pair_Set pairs = matching::getImagePairs(pairwiseMatches);
  C_Progress_display my_progress_bar( pairs.size() );
  for (Pair_Set::const_iterator iter = pairs.begin();
    iter != pairs.end();
    ++iter, ++my_progress_bar)
  {
    const size_t I = iter->first;
    const size_t J = iter->second;

    const View * view_I = sfm_data.GetViews().at(I).get();
    const std::string sView_I= stlplus::create_filespec(sfm_data.s_root_path,
      view_I->getImagePath());
    const View * view_J = sfm_data.GetViews().at(J).get();
    const std::string sView_J= stlplus::create_filespec(sfm_data.s_root_path,
      view_J->getImagePath());

    const std::pair<size_t, size_t>
      dimImage_I = std::make_pair(view_I->getWidth(), view_I->getHeight()),
      dimImage_J = std::make_pair(view_J->getWidth(), view_J->getHeight());

    svgDrawer svgStream( dimImage_I.first + dimImage_J.first, max(dimImage_I.second, dimImage_J.second));
    svgStream.drawImage(sView_I,
      dimImage_I.first,
      dimImage_I.second);
    svgStream.drawImage(sView_J,
      dimImage_J.first,
      dimImage_J.second, dimImage_I.first);

    const matching::MatchesPerDescType& vec_FilteredMatches = pairwiseMatches.at(*iter);

    std::cout << "nb describer : " << vec_FilteredMatches.size() << std::endl;

    if(vec_FilteredMatches.empty())
      continue;

    for(const auto& matchesIt: vec_FilteredMatches)
    {
      const feature::EImageDescriberType descType = matchesIt.first;
      assert(descType != feature::EImageDescriberType::UNINITIALIZED);
      const matching::IndMatches& matches = matchesIt.second;
      std::cout << EImageDescriberType_enumToString(matchesIt.first) << ": " << matches.size() << " matches" << std::endl;

      const PointFeatures& vec_feat_I = featuresPerView.getFeatures(view_I->getViewId(), descType);
      const PointFeatures& vec_feat_J = featuresPerView.getFeatures(view_J->getViewId(), descType);

      //-- Draw link between features :
      for(std::size_t i = 0; i < matches.size(); ++i)
      {
        const PointFeature & imaA = vec_feat_I[matches[i]._i];
        const PointFeature & imaB = vec_feat_J[matches[i]._j];

        // Compute a flashy colour for the correspondence
        unsigned char r,g,b;
        hslToRgb( (rand() % 360) / 360., 1.0, .5, r, g, b);
        std::ostringstream osCol;
        osCol << "rgb(" << (int)r <<',' << (int)g << ',' << (int)b <<")";
        svgStream.drawLine(imaA.x(), imaA.y(),
          imaB.x()+dimImage_I.first, imaB.y(), svgStyle().stroke(osCol.str(), 2.0));
      }

      const std::string featColor = describerTypeColor(descType);
      //-- Draw features (in two loop, in order to have the features upper the link, svg layer order):
      for(std::size_t i=0; i< matches.size(); ++i)
      {
        const PointFeature & imaA = vec_feat_I[matches[i]._i];
        const PointFeature & imaB = vec_feat_J[matches[i]._j];
        svgStream.drawCircle(imaA.x(), imaA.y(), 5.0,
          svgStyle().stroke(featColor, 2.0));
        svgStream.drawCircle(imaB.x() + dimImage_I.first, imaB.y(), 5.0,
          svgStyle().stroke(featColor, 2.0));
      }
    }

    std::ostringstream os;
    os << stlplus::folder_append_separator(sOutDir)
      << iter->first << "_" << iter->second
      << "_" << vec_FilteredMatches.getNbAllMatches() << "_.svg";
    ofstream svgFile( os.str().c_str() );
    svgFile << svgStream.closeSvgFile().str();
    svgFile.close();
  }
  return EXIT_SUCCESS;
}
