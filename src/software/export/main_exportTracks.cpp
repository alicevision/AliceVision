// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "aliceVision/matching/IndMatch.hpp"
#include "aliceVision/matching/io.hpp"
#include "aliceVision/image/all.hpp"
#include "aliceVision/feature/feature.hpp"
#include "aliceVision/track/Track.hpp"
#include "aliceVision/sfm/sfm.hpp"
#include "aliceVision/sfm/pipeline/regionsIO.hpp"
#include "aliceVision/feature/svgVisualization.hpp"

#include "software/utils/sfmHelper/sfmIOHelper.hpp"

#include "dependencies/stlplus3/filesystemSimplified/file_system.hpp"
#include "dependencies/vectorGraphics/svgDrawer.hpp"

#include <boost/program_options.hpp>
#include <boost/progress.hpp>

using namespace aliceVision;
using namespace aliceVision::matching;
using namespace aliceVision::sfm;
using namespace aliceVision::track;
using namespace svg;
namespace po = boost::program_options;

int main(int argc, char ** argv)
{
  // command-line parameters

  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  std::string sfmDataFilename;
  std::string outputFolder;
  std::string matchesFolder;
  std::string describerTypesName = feature::EImageDescriberType_enumToString(feature::EImageDescriberType::SIFT);
  std::string matchesGeometricModel = "f";

  po::options_description allParams("AliceVision exportTracks");

  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
      "SfMData file.")
    ("output,o", po::value<std::string>(&outputFolder)->required(),
      "Output path for tracks.")
    ("matchesFolder,m", po::value<std::string>(&matchesFolder)->required(),
      "Path to a folder in which computed matches are stored.");

  po::options_description optionalParams("Optional parameters");
  optionalParams.add_options()
    ("describerTypes,d", po::value<std::string>(&describerTypesName)->default_value(describerTypesName),
      feature::EImageDescriberType_informations().c_str());
    ("matchesGeometricModel,g", po::value<std::string>(&matchesGeometricModel)->default_value(matchesGeometricModel),
      "Matches geometric Model :\n"
      "- f: fundamental matrix\n"
      "- e: essential matrix\n"
      "- h: homography matrix");

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

  if (outputFolder.empty())  {
    std::cerr << "\nIt is an invalid output folder" << std::endl;
    return EXIT_FAILURE;
  }

  //---------------------------------------
  // Read SfM Scene (image view names)
  //---------------------------------------
  SfMData sfm_data;
  if (!Load(sfm_data, sfmDataFilename, ESfMData(VIEWS|INTRINSICS))) {
    std::cerr << std::endl
      << "The input SfMData file \""<< sfmDataFilename << "\" cannot be read." << std::endl;
    return EXIT_FAILURE;
  }

  //---------------------------------------
  // Load SfM Scene regions
  //---------------------------------------
  using namespace aliceVision::feature;
  
  // Get imageDescriberMethodType
  std::vector<EImageDescriberType> describerMethodTypes = EImageDescriberType_stringToEnums(describerTypesName);

  // Read the features
  feature::FeaturesPerView featuresPerView;
  if (!sfm::loadFeaturesPerView(featuresPerView, sfm_data, matchesFolder, describerMethodTypes)) {
    std::cerr << std::endl
      << "Invalid features." << std::endl;
    return EXIT_FAILURE;
  }

  // Read the matches
  matching::PairwiseMatches pairwiseMatches;
  if (!loadPairwiseMatches(pairwiseMatches, sfm_data, matchesFolder, describerMethodTypes, matchesGeometricModel))
  {
    std::cerr << "\nInvalid matches file." << std::endl;
    return EXIT_FAILURE;
  }

  //---------------------------------------
  // Compute tracks from matches
  //---------------------------------------
  track::TracksMap map_tracks;
  {
    const aliceVision::matching::PairwiseMatches & map_Matches = pairwiseMatches;
    track::TracksBuilder tracksBuilder;
    tracksBuilder.Build(map_Matches);
    tracksBuilder.Filter();
    tracksBuilder.ExportToSTL(map_tracks);
  }

  // ------------
  // For each pair, export the matches
  // ------------
  const size_t viewCount = sfm_data.GetViews().size();

  stlplus::folder_create(outputFolder);
  std::cout << "\n viewCount: " << viewCount << std::endl;
  std::cout << "\n Export pairwise tracks" << std::endl;
  boost::progress_display my_progress_bar( (viewCount*(viewCount-1)) / 2.0 );

  for (size_t I = 0; I < viewCount; ++I)
  {
    for (size_t J = I+1; J < viewCount; ++J, ++my_progress_bar)
    {

      const View * view_I = sfm_data.GetViews().at(I).get();
      const std::string sView_I= view_I->getImagePath();
      const View * view_J = sfm_data.GetViews().at(J).get();
      const std::string sView_J= view_J->getImagePath();

      const std::pair<size_t, size_t>
        dimImage_I = std::make_pair(view_I->getWidth(), view_I->getHeight()),
        dimImage_J = std::make_pair(view_J->getWidth(), view_J->getHeight());

      //Get common tracks between view I and J
      track::TracksMap map_tracksCommon;
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
        for (track::TracksMap::const_iterator tracksIt = map_tracksCommon.begin();
          tracksIt != map_tracksCommon.end(); ++tracksIt)
        {
          const feature::EImageDescriberType descType = tracksIt->second.descType;
          assert(descType != feature::EImageDescriberType::UNINITIALIZED);
          track::Track::FeatureIdPerView::const_iterator obsIt = tracksIt->second.featPerView.begin();

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
        for (track::TracksMap::const_iterator tracksIt = map_tracksCommon.begin();
          tracksIt != map_tracksCommon.end(); ++ tracksIt)
        {
          const feature::EImageDescriberType descType = tracksIt->second.descType;
          assert(descType != feature::EImageDescriberType::UNINITIALIZED);
          track::Track::FeatureIdPerView::const_iterator obsIt = tracksIt->second.featPerView.begin();

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
        os << stlplus::folder_append_separator(outputFolder)
           << I << "_" << J
           << "_" << map_tracksCommon.size() << "_.svg";
        ofstream svgFile( os.str().c_str() );
        svgFile << svgStream.closeSvgFile().str();
      }
    }
  }
  return EXIT_SUCCESS;
}
