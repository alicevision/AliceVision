// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

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

#include <dependencies/stlplus3/filesystemSimplified/file_system.hpp>

#include <boost/program_options.hpp>

#include <cstdlib>
#include <fstream>
#include <cctype>

using namespace aliceVision;
using namespace aliceVision::camera;
using namespace aliceVision::matching;
using namespace aliceVision::robustEstimation;
using namespace aliceVision::sfm;
using namespace aliceVision::matchingImageCollection;
using namespace std;
namespace po = boost::program_options;

enum EGeometricModel
{
  FUNDAMENTAL_MATRIX = 0,
  ESSENTIAL_MATRIX   = 1,
  HOMOGRAPHY_MATRIX  = 2
};

enum EPairMode
{
  PAIR_EXHAUSTIVE = 0,
  PAIR_CONTIGUOUS = 1,
  PAIR_FROM_FILE  = 2
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
    std::cout << stat.first << "\t" << stat.second << std::endl;
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
  std::string matchesDirectory;

  // user optional parameters

  std::string featuresDirectory = "";
  std::string geometricModel = "f";
  std::string describerTypesName = feature::EImageDescriberType_enumToString(feature::EImageDescriberType::SIFT);
  float distRatio = 0.8f;
  int matchingVideoMode = 0;
  std::string predefinedPairList;
  int rangeStart = -1;
  int rangeSize = 0;
  std::string nearestMatchingMethod = "ANN_L2";
  std::string geometricEstimatorName = robustEstimation::ERobustEstimator_enumToString(robustEstimation::ERobustEstimator::ACRANSAC);
  bool savePutativeMatches = false;
  bool guidedMatching = false;
  int maxIteration = 2048;
  bool matchFilePerImage = false;
  size_t numMatchesToKeep = 0;
  bool useGridSort = false;
  bool exportDebugFiles = false;
  std::string fileExtension = "bin";

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
    ("output,o", po::value<std::string>(&matchesDirectory)->required(),
      "Path to a directory in which computed matches will be stored.");

  po::options_description optionalParams("Optional parameters");
  optionalParams.add_options()
    ("geometricModel,g", po::value<std::string>(&geometricModel)->default_value(geometricModel),
      "Pairwise correspondences filtering thanks to robust model estimation:\n"
      "* f: fundamental matrix\n"
      "* e: essential matrix\n"
      "* h: homography matrix")
    ("describerTypes,d", po::value<std::string>(&describerTypesName)->default_value(describerTypesName),
      feature::EImageDescriberType_informations().c_str())
    ("featuresDirectory,f", po::value<std::string>(&featuresDirectory)->default_value(featuresDirectory),
      "Path to a directory containing the extracted features.")
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
      "* loransac: LO-Ransac (only available for fundamental matrix)")
    ("savePutativeMatches", po::value<bool>(&savePutativeMatches)->default_value(savePutativeMatches),
      "Save putative matches.")
    ("guidedMatching", po::value<bool>(&guidedMatching)->default_value(guidedMatching),
      "Use the found model to improve the pairwise correspondences.")
    ("matchFilePerImage", po::value<bool>(&matchFilePerImage)->default_value(matchFilePerImage),
      "Save matches in a separate file per image.")
    ("distanceRatio", po::value<float>(&distRatio)->default_value(distRatio),
      "Distance ratio to discard non meaningful matches.")
    ("videoModeMatching", po::value<int>(&matchingVideoMode)->default_value(matchingVideoMode),
      "sequence matching with an overlap of X images:\n"
      "* 0: will match 0 with (1->X), ...\n"
      "* 2: will match 0 with (1,2), 1 with (2,3), ...\n"
      "* 3: will match 0 with (1,2,3), 1 with (2,3,4), ...")
    ("maxIteration", po::value<int>(&maxIteration)->default_value(maxIteration),
      "Maximum number of iterations allowed in ransac step.")
    ("useGridSort", po::value<bool>(&useGridSort)->default_value(useGridSort),
      "Use matching grid sort.")
    ("exportDebugFiles", po::value<bool>(&exportDebugFiles)->default_value(exportDebugFiles),
      "Export debug files (svg, dot).")
    ("fileExtension", po::value<std::string>(&fileExtension)->default_value(fileExtension),
      "File extension to store matches (bin or txt).")
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

  ALICEVISION_COUT("Program called with the following parameters:");
  ALICEVISION_COUT(vm);

  // set verbose level
  system::Logger::get()->setLogLevel(verboseLevel);

  EPairMode pairMode = (matchingVideoMode == 0 ) ? PAIR_EXHAUSTIVE : PAIR_CONTIGUOUS;

  if(predefinedPairList.length()) {
    pairMode = PAIR_FROM_FILE;
    if (matchingVideoMode>0) {
      std::cerr << "\nIncompatible options: --videoModeMatching and --pairList" << std::endl;
      return EXIT_FAILURE;
    }
  }

  if(matchesDirectory.empty() || !stlplus::is_folder(matchesDirectory))  {
    std::cerr << "\nIt is an invalid output directory" << std::endl;
    return EXIT_FAILURE;
  }

  if(describerTypesName.empty())
  {
    std::cerr << "\nError: describerMethods argument is empty." << std::endl;
    return EXIT_FAILURE;
  }

  if(featuresDirectory.empty()) {
    featuresDirectory = matchesDirectory;
  }

  EGeometricModel geometricModelToCompute = FUNDAMENTAL_MATRIX;
  if(geometricModel.size() != 1)
  {
      std::cerr << "Unknown geometric model: " << geometricModel << std::endl;
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
      std::cerr << "Unknown geometric model: " << geometricMode << std::endl;
      return EXIT_FAILURE;
  }
  robustEstimation::ERobustEstimator geometricEstimator = robustEstimation::ERobustEstimator_stringToEnum(geometricEstimatorName);

  // -----------------------------
  // - Load SfMData Views & intrinsics data
  // a. Compute putative descriptor matches
  // b. Geometric filtering of putative matches
  // + Export some statistics
  // -----------------------------

  //---------------------------------------
  // Read SfM Scene (image view & intrinsics data)
  //---------------------------------------
  SfMData sfmData;
  if(!Load(sfmData, sfmDataFilename, ESfMData(VIEWS|INTRINSICS))) {
    std::cerr << std::endl
      << "The input SfMData file \""<< sfmDataFilename << "\" cannot be read." << std::endl;
    return EXIT_FAILURE;
  }

  //---------------------------------------
  // Load SfM Scene regions
  //---------------------------------------
  using namespace aliceVision::feature;

  //---------------------------------------
  // a. Compute putative descriptor matches
  //    - Descriptor matching (according user method choice)
  //    - Keep correspondences only if NearestNeighbor ratio is ok
  //---------------------------------------

  // From matching mode compute the pair list that have to be matched:
  PairSet pairs;
  std::set<IndexT> filter;
  switch(pairMode)
  {
    case PAIR_EXHAUSTIVE: pairs = exhaustivePairs(sfmData.GetViews(), rangeStart, rangeSize); break;
    case PAIR_CONTIGUOUS: pairs = contiguousWithOverlap(sfmData.GetViews(), matchingVideoMode); break;
    case PAIR_FROM_FILE:
      std::cout << "Load pairList from file: " << predefinedPairList << std::endl;
      if(!loadPairs(predefinedPairList, pairs, rangeStart, rangeSize))
      {
          return EXIT_FAILURE;
      }
      break;
  }

  if(pairs.empty())
  {
    std::cout << "No image pair to match." << std::endl;
    // If we only compute a selection of matches, we may have no match.
    return rangeSize ? EXIT_SUCCESS : EXIT_FAILURE;
  }

  std::cout << "Number of pairs: " << pairs.size() << std::endl;

  //Creation of the filter
  for(const auto& pair: pairs)
  {
    filter.insert(pair.first);
    filter.insert(pair.second);
  }

  std::cout << std::endl << " - PUTATIVE MATCHES - " << std::endl;
  std::cout << "Use: ";
  switch(pairMode)
  {
    case PAIR_EXHAUSTIVE: std::cout << "exhaustive pairwise matching" << std::endl; break;
    case PAIR_CONTIGUOUS: std::cout << "sequence pairwise matching" << std::endl; break;
    case PAIR_FROM_FILE:  std::cout << "user defined pairwise matching" << std::endl; break;
  }

  PairwiseMatches mapPutativesMatches;

  // Allocate the right Matcher according the Matching requested method
  EMatcherType collectionMatcherType = EMatcherType_stringToEnum(nearestMatchingMethod);
  std::unique_ptr<IImageCollectionMatcher> imageCollectionMatcher = createImageCollectionMatcher(collectionMatcherType, distRatio);

  const std::vector<feature::EImageDescriberType> describerTypes = feature::EImageDescriberType_stringToEnums(describerTypesName);

  std::cout << "There are " << sfmData.GetViews().size() << " views and " << pairs.size() << " image pairs." << std::endl;

  // Load the corresponding view regions
  RegionsPerView regionPerView;

  // Perform the matching
  system::Timer timer;

  if(!sfm::loadRegionsPerView(regionPerView, sfmData, featuresDirectory, describerTypes, filter))
  {
    std::cerr << std::endl << "Invalid regions." << std::endl;
    return EXIT_FAILURE;
  }

  for(const feature::EImageDescriberType descType : describerTypes)
  {
    assert(descType != feature::EImageDescriberType::UNINITIALIZED);
    std::cout << "-> " << EImageDescriberType_enumToString(descType) << " Regions Matching" << std::endl;

    // Photometric matching of putative pairs
    imageCollectionMatcher->Match(sfmData, regionPerView, pairs, descType, mapPutativesMatches);

    // TODO: DELI
    // if(!guided_matching) regionPerView.clearDescriptors()
  }

  if(mapPutativesMatches.empty())
  {
    std::cout << "No putative matches." << std::endl;
    // If we only compute a selection of matches, we may have no match.
    return rangeSize ? EXIT_SUCCESS : EXIT_FAILURE;
  }

  std::cout << mapPutativesMatches.size() << " putative image pair matches" << std::endl;

  for(const auto& imageMatch: mapPutativesMatches)
  {
    std::cout << " * image pair " << imageMatch.first.first << ", " << imageMatch.first.second << ": " << imageMatch.second.getNbAllMatches() << " putative matches." << std::endl;
  }

  //---------------------------------------
  //-- Export putative matches
  //---------------------------------------
  if(savePutativeMatches)
    Save(mapPutativesMatches, matchesDirectory, "putative", fileExtension, matchFilePerImage);

  std::cout << "Task (Regions Matching) done in (s): " << timer.elapsed() << std::endl;

  /*
  TODO: DELI
  if(exportDebugFiles)
  {
    //-- export putative matches Adjacency matrix
    PairwiseMatchingToAdjacencyMatrixSVG(sfmData.GetViews().size(),
      mapPutativesMatches,
      stlplus::create_filespec(matchesDirectory, "PutativeAdjacencyMatrix", "svg"));
    //-- export view pair graph once putative graph matches have been computed
    {
      std::set<IndexT> set_ViewIds;

      std::transform(sfmData.GetViews().begin(), sfmData.GetViews().end(),
        std::inserter(set_ViewIds, set_ViewIds.begin()), stl::RetrieveKey());

      graph::indexedGraph putativeGraph(set_ViewIds, getPairs(mapPutativesMatches));

      graph::exportToGraphvizData(
        stlplus::create_filespec(matchesDirectory, "putative_matches.dot"),
        putativeGraph.g);
    }
  }
  */

#ifdef ALICEVISION_DEBUG_MATCHING
    {
      std::cout << "PUTATIVE" << std::endl;
      getStatsMap(mapPutativesMatches);
    }
#endif

  //---------------------------------------
  // b. Geometric filtering of putative matches
  //    - AContrario Estimation of the desired geometric model
  //    - Use an upper bound for the a contrario estimated threshold
  //---------------------------------------

  GeometricFilter geometricFilter(&sfmData, regionPerView);

  timer.reset();
  std::cout << std::endl << " - Geometric filtering - " << std::endl;

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
      geometricFilter.Robust_model_estimation(GeometricFilterMatrix_F_AC(std::numeric_limits<double>::infinity(), maxIteration, geometricEstimator),
        mapPutativesMatches, guidedMatching);
      map_GeometricMatches = geometricFilter.Get_geometric_matches();
    }
    break;
    case ESSENTIAL_MATRIX:
    {
      geometricFilter.Robust_model_estimation(GeometricFilterMatrix_E_AC(std::numeric_limits<double>::infinity(), maxIteration),
        mapPutativesMatches, guidedMatching);
      map_GeometricMatches = geometricFilter.Get_geometric_matches();

      //-- Perform an additional check to remove pairs with poor overlap
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
      //-- remove discarded pairs
      for(std::vector<PairwiseMatches::key_type>::const_iterator
        iter =  vec_toRemove.begin(); iter != vec_toRemove.end(); ++iter)
      {
        map_GeometricMatches.erase(*iter);
      }
    }
    break;
  }

  std::cout << map_GeometricMatches.size() << " geometric image pair matches:" << std::endl;
  for(const auto& matchGeo: map_GeometricMatches)
  {
    std::cout << " * Image pair (" << matchGeo.first.first << ", " << matchGeo.first.second << ") contains " << matchGeo.second.getNbAllMatches() << " geometric matches." << std::endl;
  }

  //---------------------------------------
  //-- Grid Filtering
  //---------------------------------------
  PairwiseMatches finalMatches;

  if(numMatchesToKeep == 0)
  {
    finalMatches.swap(map_GeometricMatches);
  }
  else
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

        //Get the regions for the current view pair:
        if(rRegions && lRegions)
        {
          //Sorting function:
          aliceVision::matching::IndMatches outMatches;
          sortMatches(inputMatches, *lRegions, *rRegions, outMatches);

          if(useGridSort)
          {
            matchesGridFiltering(*lRegions, *rRegions, indexImagePair, sfmData, outMatches);
          }

          size_t finalSize = std::min(numMatchesToKeep, outMatches.size());
          outMatches.resize(finalSize);

          // std::cout << "Left features: " << lRegions->Features().size() << ", right features: " << rRegions->Features().size() << ", num matches: " << inputMatches.size() << ", num filtered matches: " << outMatches.size() << std::endl;
          finalMatches[indexImagePair].insert(std::make_pair(descType, outMatches));
        }
        else
        {
          std::cout << "You cannot perform the grid filtering with these regions" << std::endl;
        }
      }
    }

    std::cout << "After grid filtering:" << std::endl;
    for(const auto& matchGridFiltering: finalMatches)
    {
      std::cout << " * Image pair (" << matchGridFiltering.first.first << ", " << matchGridFiltering.first.second << ") contains " << matchGridFiltering.second.getNbAllMatches() << " geometric matches." << std::endl;
    }
  }

  //---------------------------------------
  //-- Export geometric filtered matches
  //---------------------------------------
  std::cout << "Save geometric matches." << std::endl;
  Save(finalMatches, matchesDirectory, geometricMode, fileExtension, matchFilePerImage);

  std::cout << "Task done in (s): " << timer.elapsed() << std::endl;

  if(exportDebugFiles)
  {
    //-- export Adjacency matrix
    std::cout << "\n Export Adjacency Matrix of the pairwise's geometric matches"
      << std::endl;
    PairwiseMatchingToAdjacencyMatrixSVG(sfmData.GetViews().size(),
      finalMatches,
      stlplus::create_filespec(matchesDirectory, "GeometricAdjacencyMatrix", "svg"));
    /*
    //-- export view pair graph once geometric filter have been done
    {
      std::set<IndexT> set_ViewIds;
      std::transform(sfmData.GetViews().begin(), sfmData.GetViews().end(),
        std::inserter(set_ViewIds, set_ViewIds.begin()), stl::RetrieveKey());
      graph::indexedGraph putativeGraph(set_ViewIds, getPairs(finalMatches));
      graph::exportToGraphvizData(
        stlplus::create_filespec(matchesDirectory, "geometric_matches.dot"),
        putativeGraph.g);
    }
    */
  }

#ifdef ALICEVISION_DEBUG_MATCHING
  {
    std::cout << "GEOMETRIC" << std::endl;
    getStatsMap(map_GeometricMatches);
  }
#endif

  return EXIT_SUCCESS;
}
