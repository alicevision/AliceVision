// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#include "aliceVision/matching/indMatch.hpp"
#include "aliceVision/matching/indMatch_utils.hpp"
#include "aliceVision/image/image.hpp"
#include "aliceVision/features/features.hpp"
#include "aliceVision/tracks/tracks.hpp"
#include "aliceVision/sfm/sfm.hpp"
#include "aliceVision/sfm/pipelines/RegionsIO.hpp"
#include "aliceVision/features/svgVisualization.hpp"

#include "software/utils/sfmHelper/sfmIOHelper.hpp"
#include "dependencies/cmdLine/cmdLine.h"
#include "dependencies/stlplus3/filesystemSimplified/file_system.hpp"
#include "dependencies/progress/progress.hpp"
#include "dependencies/vectorGraphics/svgDrawer.hpp"

using namespace aliceVision;
using namespace aliceVision::matching;
using namespace aliceVision::sfm;
using namespace aliceVision::tracks;
using namespace svg;

int main(int argc, char ** argv)
{
  CmdLine cmd;

  std::string sSfM_Data_Filename;
  std::string describerMethods = "SIFT";
  std::string sMatchesDir;
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
      std::cerr << "Export pairwise tracks.\nUsage: " << argv[0] << "\n"
      << "[-i|--input_file file] path to a SfM_Data scene\n"
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
      << "[-o|--outdir path]\n"
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
  using namespace aliceVision::features;
  
  // Get imageDescriberMethodType
  std::vector<EImageDescriberType> describerMethodTypes = EImageDescriberType_stringToEnums(describerMethods);

  // Read the features
  features::FeaturesPerView featuresPerView;
  if (!sfm::loadFeaturesPerView(featuresPerView, sfm_data, sMatchesDir, describerMethodTypes)) {
    std::cerr << std::endl
      << "Invalid features." << std::endl;
    return EXIT_FAILURE;
  }

  // Read the matches
  matching::PairwiseMatches pairwiseMatches;
  if (!loadPairwiseMatches(pairwiseMatches, sfm_data, sMatchesDir, describerMethodTypes, sMatchGeometricModel))
  {
    std::cerr << "\nInvalid matches file." << std::endl;
    return EXIT_FAILURE;
  }

  //---------------------------------------
  // Compute tracks from matches
  //---------------------------------------
  tracks::TracksMap map_tracks;
  {
    const aliceVision::matching::PairwiseMatches & map_Matches = pairwiseMatches;
    tracks::TracksBuilder tracksBuilder;
    tracksBuilder.Build(map_Matches);
    tracksBuilder.Filter();
    tracksBuilder.ExportToSTL(map_tracks);
  }

  // ------------
  // For each pair, export the matches
  // ------------
  const size_t viewCount = sfm_data.GetViews().size();

  stlplus::folder_create(sOutDir);
  std::cout << "\n viewCount: " << viewCount << std::endl;
  std::cout << "\n Export pairwise tracks" << std::endl;
  C_Progress_display my_progress_bar( (viewCount*(viewCount-1)) / 2.0 );

  for (size_t I = 0; I < viewCount; ++I)
  {
    for (size_t J = I+1; J < viewCount; ++J, ++my_progress_bar)
    {

      const View * view_I = sfm_data.GetViews().at(I).get();
      const std::string sView_I= stlplus::create_filespec(sfm_data.s_root_path,
        view_I->getImagePath());
      const View * view_J = sfm_data.GetViews().at(J).get();
      const std::string sView_J= stlplus::create_filespec(sfm_data.s_root_path,
        view_J->getImagePath());

      const std::pair<size_t, size_t>
        dimImage_I = std::make_pair(view_I->getWidth(), view_I->getHeight()),
        dimImage_J = std::make_pair(view_J->getWidth(), view_J->getHeight());

      //Get common tracks between view I and J
      tracks::TracksMap map_tracksCommon;
      std::set<size_t> set_imageIndex;
      set_imageIndex.insert(I);
      set_imageIndex.insert(J);
      TracksUtilsMap::GetTracksInImages(set_imageIndex, map_tracks, map_tracksCommon);

      if (!map_tracksCommon.empty())
      {
        svgDrawer svgStream( dimImage_I.first + dimImage_J.first, max(dimImage_I.second, dimImage_J.second));
        svgStream.drawImage(sView_I,
          dimImage_I.first,
          dimImage_I.second);
        svgStream.drawImage(sView_J,
          dimImage_J.first,
          dimImage_J.second, dimImage_I.first);

        //-- Draw link between features :
        for (tracks::TracksMap::const_iterator tracksIt = map_tracksCommon.begin();
          tracksIt != map_tracksCommon.end(); ++tracksIt)
        {
          const features::EImageDescriberType descType = tracksIt->second.descType;
          assert(descType != features::EImageDescriberType::UNINITIALIZED);
          tracks::Track::FeatureIdPerView::const_iterator obsIt = tracksIt->second.featPerView.begin();

          const PointFeatures& vec_feat_I = featuresPerView.getFeatures(view_I->getViewId(), descType);
          const PointFeatures& vec_feat_J = featuresPerView.getFeatures(view_J->getViewId(), descType);

          const PointFeature& imaA = vec_feat_I[obsIt->second];
          ++obsIt;
          const PointFeature& imaB = vec_feat_J[obsIt->second];

          svgStream.drawLine(imaA.x(), imaA.y(),
            imaB.x()+dimImage_I.first, imaB.y(),
            svgStyle().stroke("green", 2.0));
        }

        //-- Draw features (in two loop, in order to have the features upper the link, svg layer order):
        for (tracks::TracksMap::const_iterator tracksIt = map_tracksCommon.begin();
          tracksIt != map_tracksCommon.end(); ++ tracksIt)
        {
          const features::EImageDescriberType descType = tracksIt->second.descType;
          assert(descType != features::EImageDescriberType::UNINITIALIZED);
          tracks::Track::FeatureIdPerView::const_iterator obsIt = tracksIt->second.featPerView.begin();

          const PointFeatures& vec_feat_I = featuresPerView.getFeatures(view_I->getViewId(), descType);
          const PointFeatures& vec_feat_J = featuresPerView.getFeatures(view_J->getViewId(), descType);

          const PointFeature& imaA = vec_feat_I[obsIt->second];
          ++obsIt;
          const PointFeature& imaB = vec_feat_J[obsIt->second];

          const std::string featColor = describerTypeColor(descType);

          svgStream.drawCircle(imaA.x(), imaA.y(),
            3.0, svgStyle().stroke(featColor, 2.0));
          svgStream.drawCircle(imaB.x() + dimImage_I.first,imaB.y(),
            3.0, svgStyle().stroke(featColor, 2.0));
        }
        std::ostringstream os;
        os << stlplus::folder_append_separator(sOutDir)
           << I << "_" << J
           << "_" << map_tracksCommon.size() << "_.svg";
        ofstream svgFile( os.str().c_str() );
        svgFile << svgStream.closeSvgFile().str();
      }
    }
  }
  return EXIT_SUCCESS;
}
