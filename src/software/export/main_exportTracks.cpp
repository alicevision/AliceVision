// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/matching/IndMatch.hpp>
#include <aliceVision/matching/io.hpp>
#include <aliceVision/image/all.hpp>
#include <aliceVision/feature/feature.hpp>
#include <aliceVision/track/Track.hpp>
#include <aliceVision/sfm/sfm.hpp>
#include <aliceVision/sfm/pipeline/regionsIO.hpp>
#include <aliceVision/feature/svgVisualization.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>

#include "software/utils/sfmHelper/sfmIOHelper.hpp"

#include "dependencies/vectorGraphics/svgDrawer.hpp"

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/progress.hpp>

using namespace aliceVision;
using namespace aliceVision::matching;
using namespace aliceVision::sfm;
using namespace aliceVision::track;
using namespace svg;

namespace po = boost::program_options;
namespace fs = boost::filesystem;

int main(int argc, char ** argv)
{
  // command-line parameters

  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  std::string sfmDataFilename;
  std::string outputFolder;
  std::string featuresFolder;
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
    ("featuresFolder,f", po::value<std::string>(&featuresFolder)->required(),
      "Path to a folder containing the extracted features.")
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

  ALICEVISION_COUT("Program called with the following parameters:");
  ALICEVISION_COUT(vm);

  // set verbose level
  system::Logger::get()->setLogLevel(verboseLevel);

  if (outputFolder.empty())
  {
    ALICEVISION_LOG_ERROR("It is an invalid output folder");
    return EXIT_FAILURE;
  }

  // read SfM Scene (image view names)
  SfMData sfmData;
  if (!Load(sfmData, sfmDataFilename, ESfMData(VIEWS|INTRINSICS)))
  {
    ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFilename << "' cannot be read.");
    return EXIT_FAILURE;
  }

  // load SfM Scene regions
  using namespace aliceVision::feature;
  
  // get imageDescriberMethodType
  std::vector<EImageDescriberType> describerMethodTypes = EImageDescriberType_stringToEnums(describerTypesName);

  // read the features
  std::vector<std::string> featuresFolders = sfmData.getFeaturesFolders();
  featuresFolders.emplace_back(featuresFolder);

  feature::FeaturesPerView featuresPerView;
  if (!sfm::loadFeaturesPerView(featuresPerView, sfmData, featuresFolders, describerMethodTypes))
  {
    ALICEVISION_LOG_ERROR("Invalid features");
    return EXIT_FAILURE;
  }

  // read the matches
  matching::PairwiseMatches pairwiseMatches;
  if (!loadPairwiseMatches(pairwiseMatches, sfmData, matchesFolder, describerMethodTypes, matchesGeometricModel))
  {
    ALICEVISION_LOG_ERROR("Invalid matches file");
    return EXIT_FAILURE;
  }

  // compute tracks from matches
  track::TracksMap map_tracks;
  {
    const aliceVision::matching::PairwiseMatches & map_Matches = pairwiseMatches;
    track::TracksBuilder tracksBuilder;
    tracksBuilder.Build(map_Matches);
    tracksBuilder.Filter();
    tracksBuilder.ExportToSTL(map_tracks);
  }

  // for each pair, export the matches
  const size_t viewCount = sfmData.GetViews().size();

  fs::create_directory(outputFolder);
  ALICEVISION_LOG_INFO("viewCount: " << viewCount);
  ALICEVISION_LOG_INFO("Export pairwise tracks");
  boost::progress_display my_progress_bar( (viewCount*(viewCount-1)) / 2.0 );

  for(size_t I = 0; I < viewCount; ++I)
  {
    for(size_t J = I+1; J < viewCount; ++J, ++my_progress_bar)
    {
      const View* viewI = sfmData.GetViews().at(I).get();
      const View* viewJ = sfmData.GetViews().at(J).get();

      const std::string viewImagePathI = viewI->getImagePath();
      const std::string viewImagePathJ = viewJ->getImagePath();

      const std::pair<size_t, size_t> dimImageI = std::make_pair(viewI->getWidth(), viewI->getHeight());
      const std::pair<size_t, size_t> dimImageJ = std::make_pair(viewJ->getWidth(), viewJ->getHeight());

      // get common tracks between view I and J
      track::TracksMap map_tracksCommon;
      std::set<size_t> set_imageIndex;

      set_imageIndex.insert(I);
      set_imageIndex.insert(J);
      TracksUtilsMap::GetCommonTracksInImages(set_imageIndex, map_tracks, map_tracksCommon);

      if (!map_tracksCommon.empty())
      {
        svgDrawer svgStream( dimImageI.first + dimImageJ.first, max(dimImageI.second, dimImageJ.second));
        svgStream.drawImage(viewImagePathI, dimImageI.first, dimImageI.second);
        svgStream.drawImage(viewImagePathJ, dimImageJ.first, dimImageJ.second, dimImageI.first);

        // draw link between features :
        for (track::TracksMap::const_iterator tracksIt = map_tracksCommon.begin();
          tracksIt != map_tracksCommon.end(); ++tracksIt)
        {
          const feature::EImageDescriberType descType = tracksIt->second.descType;
          assert(descType != feature::EImageDescriberType::UNINITIALIZED);
          track::Track::FeatureIdPerView::const_iterator obsIt = tracksIt->second.featPerView.begin();

          const PointFeatures& featuresI = featuresPerView.getFeatures(viewI->getViewId(), descType);
          const PointFeatures& featuresJ = featuresPerView.getFeatures(viewJ->getViewId(), descType);

          const PointFeature& imaA = featuresI[obsIt->second];
          ++obsIt;
          const PointFeature& imaB = featuresJ[obsIt->second];

          svgStream.drawLine(imaA.x(), imaA.y(),
            imaB.x()+dimImageI.first, imaB.y(),
            svgStyle().stroke("green", 2.0));
        }

        // draw features (in two loop, in order to have the features upper the link, svg layer order):
        for (track::TracksMap::const_iterator tracksIt = map_tracksCommon.begin();
          tracksIt != map_tracksCommon.end(); ++ tracksIt)
        {
          const feature::EImageDescriberType descType = tracksIt->second.descType;
          assert(descType != feature::EImageDescriberType::UNINITIALIZED);
          track::Track::FeatureIdPerView::const_iterator obsIt = tracksIt->second.featPerView.begin();

          const PointFeatures& featuresI = featuresPerView.getFeatures(viewI->getViewId(), descType);
          const PointFeatures& featuresJ = featuresPerView.getFeatures(viewJ->getViewId(), descType);

          const PointFeature& imaA = featuresI[obsIt->second];
          ++obsIt;
          const PointFeature& imaB = featuresJ[obsIt->second];

          const std::string featColor = describerTypeColor(descType);

          svgStream.drawCircle(imaA.x(), imaA.y(),
            3.0, svgStyle().stroke(featColor, 2.0));
          svgStream.drawCircle(imaB.x() + dimImageI.first,imaB.y(),
            3.0, svgStyle().stroke(featColor, 2.0));
        }

        fs::path outputFilename = fs::path(outputFolder) / std::string(std::to_string(I) + "_" + std::to_string(J) + "_" + std::to_string(map_tracksCommon.size()) + ".svg");

        ofstream svgFile(outputFilename.string());
        svgFile << svgStream.closeSvgFile().str();
      }
    }
  }
  return EXIT_SUCCESS;
}
