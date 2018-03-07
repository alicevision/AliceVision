// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfm/SfMData.hpp>
#include <aliceVision/sfm/sfmDataIO.hpp>
#include <aliceVision/sfm/pipeline/regionsIO.hpp>
#include <aliceVision/sfm/pipeline/ReconstructionEngine.hpp>
#include <aliceVision/feature/FeaturesPerView.hpp>
#include <aliceVision/feature/RegionsPerView.hpp>
#include <aliceVision/feature/ImageDescriber.hpp>
#include <aliceVision/feature/imageDescriberCommon.hpp>
#include <aliceVision/matchingImageCollection/matchingCommon.hpp>
#include <aliceVision/matchingImageCollection/ImageCollectionMatcher_generic.hpp>
#include <aliceVision/matchingImageCollection/ImageCollectionMatcher_cascadeHashing.hpp>
#include <aliceVision/matchingImageCollection/GeometricFilter.hpp>
#include <aliceVision/matchingImageCollection/GeometricFilterMatrix_F_AC.hpp>
#include <aliceVision/matchingImageCollection/GeometricFilterMatrix_E_AC.hpp>
#include <aliceVision/matchingImageCollection/GeometricFilterMatrix_H_AC.hpp>
#include <aliceVision/matching/pairwiseAdjacencyDisplay.hpp>
#include <aliceVision/matching/io.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/feature/selection.hpp>
#include <aliceVision/graph/graph.hpp>
#include <aliceVision/stl/stl.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <cstdlib>
#include <fstream>
#include <cctype>

using namespace aliceVision;
using namespace aliceVision::camera;
using namespace aliceVision::feature;
using namespace aliceVision::matching;
using namespace aliceVision::robustEstimation;
using namespace aliceVision::sfm;
using namespace aliceVision::matchingImageCollection;
using namespace std;

namespace po = boost::program_options;
namespace fs = boost::filesystem;

enum EGeometricModel
{
  FUNDAMENTAL_MATRIX = 0,
  ESSENTIAL_MATRIX   = 1,
  HOMOGRAPHY_MATRIX  = 2
};

void getStatsMap(const PairwiseMatches& map)
{
#ifdef ALICEVISION_DEBUG_MATCHING
  std::map<int,int> stats;
  for(const auto& imgMatches: map)
  {
    for(const auto& featMatchesPerDesc: imgMatches.second)
    {
      for(const matching::IndMatch& featMatches: featMatchesPerDesc.second)
      {
        int d = std::floor(featMatches._distance / 1000.0);
        if( stats.find(d) != stats.end() )
          stats[d] += 1;
        else
          stats[d] = 1;
      }
    }
  }
  for(const auto& stat: stats)
  {
    ALICEVISION_LOG_DEBUG(std::to_string(stat.first) + "\t" + std::to_string(stat.second));
  }
#endif
}

/// Compute corresponding features between a series of views:
/// - Load view images description (regions: features & descriptors)
/// - Compute putative local feature matches (descriptors matching)
/// - Compute geometric coherent feature matches (robust model estimation from putative matches)
/// - Export computed data
int main(int argc, char **argv)
{
  // command-line parameters

  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  std::string sfmDataFilename;
  std::string matchesFolder;

  // user optional parameters

  std::string featuresFolder = "";
  std::string geometricModel = "f";
  std::string describerTypesName = feature::EImageDescriberType_enumToString(feature::EImageDescriberType::SIFT);
  float distRatio = 0.8f;
  std::string predefinedPairList;
  int rangeStart = -1;
  int rangeSize = 0;
  std::string nearestMatchingMethod = "ANN_L2";
  std::string geometricEstimatorName = robustEstimation::ERobustEstimator_enumToString(robustEstimation::ERobustEstimator::ACRANSAC);
  double geometricErrorMax = 0.0; //< the maximum reprojection error allowed for image matching with geometric validation
  bool savePutativeMatches = false;
  bool guidedMatching = false;
  int maxIteration = 2048;
  bool matchFilePerImage = true;
  size_t numMatchesToKeep = 0;
  bool useGridSort = true;
  bool exportDebugFiles = false;
  const std::string fileExtension = "txt";

  po::options_description allParams(
     "Compute corresponding features between a series of views:\n"
     "- Load view images description (regions: features & descriptors)\n"
     "- Compute putative local feature matches (descriptors matching)\n"
     "- Compute geometric coherent feature matches (robust model estimation from putative matches)\n"
     "- Export computed data\n"
     "AliceVision featureMatching");

  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
      "SfMData file.")
    ("output,o", po::value<std::string>(&matchesFolder)->required(),
      "Path to a folder in which computed matches will be stored.");

  po::options_description optionalParams("Optional parameters");
  optionalParams.add_options()
    ("geometricModel,g", po::value<std::string>(&geometricModel)->default_value(geometricModel),
      "Pairwise correspondences filtering thanks to robust model estimation:\n"
      "* f: fundamental matrix\n"
      "* e: essential matrix\n"
      "* h: homography matrix")
    ("describerTypes,d", po::value<std::string>(&describerTypesName)->default_value(describerTypesName),
      feature::EImageDescriberType_informations().c_str())
    ("featuresFolder,f", po::value<std::string>(&featuresFolder)->default_value(featuresFolder),
      "Path to a folder containing the extracted features.")
    ("imagePairsList,l", po::value<std::string>(&predefinedPairList)->default_value(predefinedPairList),
      "Path to a file which contains the list of image pairs to match.")
    ("photometricMatchingMethod,p", po::value<std::string>(&nearestMatchingMethod)->default_value(nearestMatchingMethod),
      "For Scalar based regions descriptor:\n"
      "* BRUTE_FORCE_L2: L2 BruteForce matching\n"
      "* ANN_L2: L2 Approximate Nearest Neighbor matching\n"
      "* CASCADE_HASHING_L2: L2 Cascade Hashing matching\n"
      "* FAST_CASCADE_HASHING_L2: L2 Cascade Hashing with precomputed hashed regions\n"
      "(faster than CASCADE_HASHING_L2 but use more memory)\n"
      "For Binary based descriptor:\n"
      "* BRUTE_FORCE_HAMMING: BruteForce Hamming matching")
    ("geometricEstimator", po::value<std::string>(&geometricEstimatorName)->default_value(geometricEstimatorName),
      "Geometric estimator:\n"
      "* acransac: A-Contrario Ransac\n"
      "* loransac: LO-Ransac (only available for fundamental matrix). Need to set '--geometricError'")
    ("geometricError", po::value<double>(&geometricErrorMax)->default_value(geometricErrorMax), 
          "Maximum matching error (in pixels) allowed for image matching with geometric verification. "
          "If set to 0 it lets the ACRansac select an optimal value.")
    ("savePutativeMatches", po::value<bool>(&savePutativeMatches)->default_value(savePutativeMatches),
      "Save putative matches.")
    ("guidedMatching", po::value<bool>(&guidedMatching)->default_value(guidedMatching),
      "Use the found model to improve the pairwise correspondences.")
    ("matchFilePerImage", po::value<bool>(&matchFilePerImage)->default_value(matchFilePerImage),
      "Save matches in a separate file per image.")
    ("distanceRatio", po::value<float>(&distRatio)->default_value(distRatio),
      "Distance ratio to discard non meaningful matches.")
    ("maxIteration", po::value<int>(&maxIteration)->default_value(maxIteration),
      "Maximum number of iterations allowed in ransac step.")
    ("useGridSort", po::value<bool>(&useGridSort)->default_value(useGridSort),
      "Use matching grid sort.")
    ("exportDebugFiles", po::value<bool>(&exportDebugFiles)->default_value(exportDebugFiles),
      "Export debug files (svg, dot).")
    ("maxMatches", po::value<std::size_t>(&numMatchesToKeep)->default_value(numMatchesToKeep),
      "Maximum number pf matches to keep.")
    ("rangeStart", po::value<int>(&rangeStart)->default_value(rangeStart),
      "Range image index start.")
    ("rangeSize", po::value<int>(&rangeSize)->default_value(rangeSize),
      "Range size.");

  po::options_description logParams("Log parameters");
  logParams.add_options()
    ("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
      "verbosity level (fatal, error, warning, info, debug, trace).");

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

  robustEstimation::ERobustEstimator geometricEstimator = robustEstimation::ERobustEstimator_stringToEnum(geometricEstimatorName);
  if(!checkRobustEstimator(geometricEstimator, geometricErrorMax))
    return EXIT_FAILURE;

  ALICEVISION_COUT("Program called with the following parameters:");
  ALICEVISION_COUT(vm);

  // set verbose level
  system::Logger::get()->setLogLevel(verboseLevel);

  // check and set input options
  if(matchesFolder.empty() || !fs::is_directory(matchesFolder))
  {
    ALICEVISION_LOG_ERROR("Invalid output matches folder: " + matchesFolder);
    return EXIT_FAILURE;
  }

  if(featuresFolder.empty())
  {
    ALICEVISION_LOG_INFO("Using matchesFolder as featuresFolder");
    featuresFolder = matchesFolder;
  }

  if(describerTypesName.empty())
  {
    ALICEVISION_LOG_ERROR("Empty option: --describerMethods");
    return EXIT_FAILURE;
  }

  EGeometricModel geometricModelToCompute = FUNDAMENTAL_MATRIX;
  if(geometricModel.size() != 1)
  {
      ALICEVISION_LOG_ERROR("Unknown geometric model: " + geometricModel);
      return EXIT_FAILURE;
  }

  const std::string geometricMode = std::string(1, std::tolower(geometricModel[0]));
  switch(geometricMode[0])
  {
    case 'f':
      geometricModelToCompute = FUNDAMENTAL_MATRIX;
      break;
    case 'e':
      geometricModelToCompute = ESSENTIAL_MATRIX;
      break;
    case 'h':
      geometricModelToCompute = HOMOGRAPHY_MATRIX;
      break;
    default:
      ALICEVISION_LOG_ERROR("Unknown geometric model: " + geometricMode);
      return EXIT_FAILURE;
  }

  // Feature matching
  // a. Load SfMData Views & intrinsics data
  // b. Compute putative descriptor matches
  // c. Geometric filtering of putative matches
  // d. Export some statistics

  // a. Load SfMData (image view & intrinsics data)

  SfMData sfmData;
  if(!Load(sfmData, sfmDataFilename, ESfMData(VIEWS|INTRINSICS)))
  {
    ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFilename << "' cannot be read.");
    return EXIT_FAILURE;
  }

  // b. Compute putative descriptor matches
  //    - Descriptor matching (according user method choice)
  //    - Keep correspondences only if NearestNeighbor ratio is ok

  // from matching mode compute the pair list that have to be matched
  PairSet pairs;
  std::set<IndexT> filter;

  if(predefinedPairList.empty())
  {
    pairs = exhaustivePairs(sfmData.GetViews(), rangeStart, rangeSize);
  }
  else
  {
    ALICEVISION_LOG_INFO("Load pair list from file: " << predefinedPairList);
    if(!loadPairs(predefinedPairList, pairs, rangeStart, rangeSize))
        return EXIT_FAILURE;
  }

  if(pairs.empty())
  {
    ALICEVISION_LOG_INFO("No image pair to match.");
    // if we only compute a selection of matches, we may have no match.
    return rangeSize ? EXIT_SUCCESS : EXIT_FAILURE;
  }

  ALICEVISION_LOG_INFO("Number of pairs: " << pairs.size());

  // filter creation
  for(const auto& pair: pairs)
  {
    filter.insert(pair.first);
    filter.insert(pair.second);
  }

  ALICEVISION_LOG_INFO("Putative matches");

  PairwiseMatches mapPutativesMatches;

  // allocate the right Matcher according the Matching requested method
  EMatcherType collectionMatcherType = EMatcherType_stringToEnum(nearestMatchingMethod);
  std::unique_ptr<IImageCollectionMatcher> imageCollectionMatcher = createImageCollectionMatcher(collectionMatcherType, distRatio);

  const std::vector<feature::EImageDescriberType> describerTypes = feature::EImageDescriberType_stringToEnums(describerTypesName);

  ALICEVISION_LOG_INFO("There are " + std::to_string(sfmData.GetViews().size()) + " views and " + std::to_string(pairs.size()) + " image pairs.");

  // load the corresponding view regions
  RegionsPerView regionPerView;;
  if(!sfm::loadRegionsPerView(regionPerView, sfmData, featuresFolder, describerTypes, filter))
  {
    ALICEVISION_LOG_ERROR("Invalid regions in '" + sfmDataFilename + "'");
    return EXIT_FAILURE;
  }

  // perform the matching
  system::Timer timer;

  for(const feature::EImageDescriberType descType : describerTypes)
  {
    assert(descType != feature::EImageDescriberType::UNINITIALIZED);
    ALICEVISION_LOG_INFO(EImageDescriberType_enumToString(descType) + " Regions Matching");

    // photometric matching of putative pairs
    imageCollectionMatcher->Match(regionPerView, pairs, descType, mapPutativesMatches);

    // TODO: DELI
    // if(!guided_matching) regionPerView.clearDescriptors()
  }

  if(mapPutativesMatches.empty())
  {
    ALICEVISION_LOG_INFO("No putative matches.");
    // If we only compute a selection of matches, we may have no match.
    return rangeSize ? EXIT_SUCCESS : EXIT_FAILURE;
  }

  ALICEVISION_LOG_INFO(std::to_string(mapPutativesMatches.size()) << " putative image pair matches");

  for(const auto& imageMatch: mapPutativesMatches)
    ALICEVISION_LOG_INFO("\t- image pair (" + std::to_string(imageMatch.first.first) << ", " + std::to_string(imageMatch.first.second) + ") contains " + std::to_string(imageMatch.second.getNbAllMatches()) + " putative matches.");

  // export putative matches
  if(savePutativeMatches)
    Save(mapPutativesMatches, matchesFolder, "putative", fileExtension, matchFilePerImage);

  ALICEVISION_LOG_INFO("Task (Regions Matching) done in (s): " + std::to_string(timer.elapsed()));

  /*
  // TODO: DELI
  if(exportDebugFiles)
  {
    //-- export putative matches Adjacency matrix
    PairwiseMatchingToAdjacencyMatrixSVG(sfmData.GetViews().size(),
      mapPutativesMatches,
      (fs::path(matchesFolder) / "PutativeAdjacencyMatrix.svg").string());
    //-- export view pair graph once putative graph matches have been computed
    {
      std::set<IndexT> set_ViewIds;

      std::transform(sfmData.GetViews().begin(), sfmData.GetViews().end(),
        std::inserter(set_ViewIds, set_ViewIds.begin()), stl::RetrieveKey());

      graph::indexedGraph putativeGraph(set_ViewIds, getPairs(mapPutativesMatches));

      graph::exportToGraphvizData(
        (fs::path(matchesFolder) / "putative_matches.dot").string(),
        putativeGraph.g);
    }
  }
  */

#ifdef ALICEVISION_DEBUG_MATCHING
    {
      ALICEVISION_LOG_DEBUG("PUTATIVE");
      getStatsMap(mapPutativesMatches);
    }
#endif

  // c. Geometric filtering of putative matches
  //    - AContrario Estimation of the desired geometric model
  //    - Use an upper bound for the a contrario estimated threshold

  GeometricFilter geometricFilter(&sfmData, regionPerView);

  timer.reset();
  ALICEVISION_LOG_INFO("Geometric filtering");

  matching::PairwiseMatches map_GeometricMatches;
  switch(geometricModelToCompute)
  {
    case HOMOGRAPHY_MATRIX:
    {
      const bool bGeometric_only_guided_matching = true;
      geometricFilter.Robust_model_estimation(GeometricFilterMatrix_H_AC(std::numeric_limits<double>::infinity(), maxIteration),
        mapPutativesMatches, guidedMatching,
        bGeometric_only_guided_matching ? -1.0 : 0.6);
      map_GeometricMatches = geometricFilter.Get_geometric_matches();
    }
    break;
    case FUNDAMENTAL_MATRIX:
    {
      geometricFilter.Robust_model_estimation(GeometricFilterMatrix_F_AC(geometricErrorMax, maxIteration, geometricEstimator),
        mapPutativesMatches, guidedMatching);
      map_GeometricMatches = geometricFilter.Get_geometric_matches();
    }
    break;
    case ESSENTIAL_MATRIX:
    {
      geometricFilter.Robust_model_estimation(GeometricFilterMatrix_E_AC(std::numeric_limits<double>::infinity(), maxIteration),
        mapPutativesMatches, guidedMatching);
      map_GeometricMatches = geometricFilter.Get_geometric_matches();

      // perform an additional check to remove pairs with poor overlap
      std::vector<PairwiseMatches::key_type> vec_toRemove;
      for(PairwiseMatches::const_iterator iterMap = map_GeometricMatches.begin();
        iterMap != map_GeometricMatches.end(); ++iterMap)
      {
        const size_t putativePhotometricCount = mapPutativesMatches.find(iterMap->first)->second.getNbAllMatches();
        const size_t putativeGeometricCount = iterMap->second.getNbAllMatches();
        const float ratio = putativeGeometricCount / (float)putativePhotometricCount;
        if (putativeGeometricCount < 50 || ratio < .3f)
        {
          // the image pair will be removed
          vec_toRemove.push_back(iterMap->first);
        }
      }
      // remove discarded pairs
      for(std::vector<PairwiseMatches::key_type>::const_iterator
        iter =  vec_toRemove.begin(); iter != vec_toRemove.end(); ++iter)
      {
        map_GeometricMatches.erase(*iter);
      }
    }
    break;
  }

  ALICEVISION_LOG_INFO(std::to_string(map_GeometricMatches.size()) + " geometric image pair matches:");
  for(const auto& matchGeo: map_GeometricMatches)
    ALICEVISION_LOG_INFO("\t- image pair (" + std::to_string(matchGeo.first.first) + ", " + std::to_string(matchGeo.first.second) + ") contains " + std::to_string(matchGeo.second.getNbAllMatches()) + " geometric matches.");

  // grid filtering

  PairwiseMatches finalMatches;

  {
    for(const auto& matchGeo: map_GeometricMatches)
    {
      //Get the image pair and their matches.
      const Pair& indexImagePair = matchGeo.first;
      const aliceVision::matching::MatchesPerDescType& matchesPerDesc = matchGeo.second;

      for(const auto& match: matchesPerDesc)
      {
        const feature::EImageDescriberType descType = match.first;
        assert(descType != feature::EImageDescriberType::UNINITIALIZED);
        const aliceVision::matching::IndMatches& inputMatches = match.second;

        const feature::FeatRegions<feature::SIOPointFeature>* rRegions = dynamic_cast<const feature::FeatRegions<feature::SIOPointFeature>*>(&regionPerView.getRegions(indexImagePair.second, descType));
        const feature::FeatRegions<feature::SIOPointFeature>* lRegions = dynamic_cast<const feature::FeatRegions<feature::SIOPointFeature>*>(&regionPerView.getRegions(indexImagePair.first, descType));

        // get the regions for the current view pair:
        if(rRegions && lRegions)
        {
          // sorting function:
          aliceVision::matching::IndMatches outMatches;
          sortMatches(inputMatches, *lRegions, *rRegions, outMatches);

          if(useGridSort)
          {
            // TODO: rename as matchesGridOrdering
            matchesGridFiltering(*lRegions, *rRegions, indexImagePair, sfmData, outMatches);
          }
          if(numMatchesToKeep > 0)
          {
            size_t finalSize = std::min(numMatchesToKeep, outMatches.size());
            outMatches.resize(finalSize);
          }

          // std::cout << "Left features: " << lRegions->Features().size() << ", right features: " << rRegions->Features().size() << ", num matches: " << inputMatches.size() << ", num filtered matches: " << outMatches.size() << std::endl;
          finalMatches[indexImagePair].insert(std::make_pair(descType, outMatches));
        }
        else
        {
          ALICEVISION_LOG_INFO("You cannot perform the grid filtering with these regions");
        }
      }
    }

    ALICEVISION_LOG_INFO("After grid filtering:");
    for(const auto& matchGridFiltering: finalMatches)
      ALICEVISION_LOG_INFO("\t- image pair (" + std::to_string(matchGridFiltering.first.first) + ", " + std::to_string(matchGridFiltering.first.second) + ") contains " + std::to_string(matchGridFiltering.second.getNbAllMatches()) + " geometric matches.");

  }

  // export geometric filtered matches

  ALICEVISION_LOG_INFO("Save geometric matches.");
  Save(finalMatches, matchesFolder, geometricMode, fileExtension, matchFilePerImage);
  ALICEVISION_LOG_INFO("Task done in (s): " + std::to_string(timer.elapsed()));

  // d. Export some statistics

  if(exportDebugFiles)
  {
    // export Adjacency matrix
    ALICEVISION_LOG_INFO("Export Adjacency Matrix of the pairwise's geometric matches");
    PairwiseMatchingToAdjacencyMatrixSVG(sfmData.GetViews().size(),
      finalMatches,(fs::path(matchesFolder) / "GeometricAdjacencyMatrix.svg").string());

    /*
    // export view pair graph once geometric filter have been done
    {
      std::set<IndexT> set_ViewIds;
      std::transform(sfmData.GetViews().begin(), sfmData.GetViews().end(),
        std::inserter(set_ViewIds, set_ViewIds.begin()), stl::RetrieveKey());
      graph::indexedGraph putativeGraph(set_ViewIds, getPairs(finalMatches));
      graph::exportToGraphvizData(
        (fs::path(matchesFolder) / "geometric_matches.dot").string(),
        putativeGraph.g);
    }
    */
  }

#ifdef ALICEVISION_DEBUG_MATCHING
  {
    ALICEVISION_LOG_DEBUG("GEOMETRIC");
    getStatsMap(map_GeometricMatches);
  }
#endif

  return EXIT_SUCCESS;
}
