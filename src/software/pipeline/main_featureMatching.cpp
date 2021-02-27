// This file is part of the AliceVision project.
// Copyright (c) 2015 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/sfm/pipeline/regionsIO.hpp>
#include <aliceVision/sfm/pipeline/ReconstructionEngine.hpp>
#include <aliceVision/sfm/pipeline/structureFromKnownPoses/StructureEstimationFromKnownPoses.hpp>
#include <aliceVision/matching/matchesFiltering.hpp>
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
#include <aliceVision/matchingImageCollection/GeometricFilterMatrix_HGrowing.hpp>
#include <aliceVision/matchingImageCollection/GeometricFilterType.hpp>
#include <aliceVision/matching/pairwiseAdjacencyDisplay.hpp>
#include <aliceVision/matching/io.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/graph/graph.hpp>
#include <aliceVision/stl/stl.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <cstdlib>
#include <fstream>
#include <cctype>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 2
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;
using namespace aliceVision::camera;
using namespace aliceVision::feature;
using namespace aliceVision::matching;
using namespace aliceVision::robustEstimation;
using namespace aliceVision::sfm;
using namespace aliceVision::sfmData;
using namespace aliceVision::matchingImageCollection;
using namespace std;

namespace po = boost::program_options;
namespace fs = boost::filesystem;

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
int aliceVision_main(int argc, char **argv)
{
  // command-line parameters

  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  std::string sfmDataFilename;
  std::string matchesFolder;
  std::vector<std::string> featuresFolders;

  // user optional parameters

  std::string geometricFilterTypeName = matchingImageCollection::EGeometricFilterType_enumToString(matchingImageCollection::EGeometricFilterType::FUNDAMENTAL_MATRIX);
  std::string describerTypesName = feature::EImageDescriberType_enumToString(feature::EImageDescriberType::SIFT);
  float distRatio = 0.8f;
  std::vector<std::string> predefinedPairList;
  int rangeStart = -1;
  int rangeSize = 0;
  std::string nearestMatchingMethod = "ANN_L2";
  robustEstimation::ERobustEstimator geometricEstimator = robustEstimation::ERobustEstimator::ACRANSAC;
  double geometricErrorMax = 0.0; //< the maximum reprojection error allowed for image matching with geometric validation
  double knownPosesGeometricErrorMax = 4.0;
  bool savePutativeMatches = false;
  bool guidedMatching = false;
  bool crossMatching = false;
  int maxIteration = 2048;
  bool matchFilePerImage = false;
  size_t numMatchesToKeep = 0;
  bool useGridSort = true;
  bool exportDebugFiles = false;
  bool matchFromKnownCameraPoses = false;
  const std::string fileExtension = "txt";
  int randomSeed = std::mt19937::default_seed;

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
      "Path to a folder in which computed matches will be stored.")
    ("featuresFolders,f", po::value<std::vector<std::string>>(&featuresFolders)->multitoken()->required(),
      "Path to folder(s) containing the extracted features.");

  po::options_description optionalParams("Optional parameters");
  optionalParams.add_options()
    ("geometricFilterType,g", po::value<std::string>(&geometricFilterTypeName)->default_value(geometricFilterTypeName),
      matchingImageCollection::EGeometricFilterType_informations().c_str())
    ("describerTypes,d", po::value<std::string>(&describerTypesName)->default_value(describerTypesName),
      feature::EImageDescriberType_informations().c_str())
    ("imagePairsList,l", po::value<std::vector<std::string>>(&predefinedPairList)->multitoken(),
      "Path(s) to one or more files which contain the list of image pairs to match.")
    ("photometricMatchingMethod,p", po::value<std::string>(&nearestMatchingMethod)->default_value(nearestMatchingMethod),
      "For Scalar based regions descriptor:\n"
      "* BRUTE_FORCE_L2: L2 BruteForce matching\n"
      "* ANN_L2: L2 Approximate Nearest Neighbor matching\n"
      "* CASCADE_HASHING_L2: L2 Cascade Hashing matching\n"
      "* FAST_CASCADE_HASHING_L2: L2 Cascade Hashing with precomputed hashed regions\n"
      "(faster than CASCADE_HASHING_L2 but use more memory)\n"
      "For Binary based descriptor:\n"
      "* BRUTE_FORCE_HAMMING: BruteForce Hamming matching")
    ("geometricEstimator", po::value<robustEstimation::ERobustEstimator>(&geometricEstimator)->default_value(geometricEstimator),
      "Geometric estimator:\n"
      "* acransac: A-Contrario Ransac\n"
      "* loransac: LO-Ransac (only available for fundamental matrix). Need to set '--geometricError'")
    ("geometricError", po::value<double>(&geometricErrorMax)->default_value(geometricErrorMax), 
      "Maximum error (in pixels) allowed for features matching during geometric verification. "
      "If set to 0 it lets the ACRansac select an optimal value.")
    ("matchFromKnownCameraPoses", po::value<bool>(&matchFromKnownCameraPoses)->default_value(matchFromKnownCameraPoses),
      "Enable the usage of geometric information from known camera poses to guide the feature matching. "
      "If some cameras have unknown poses (so there is no geometric prior), the standard feature matching will be performed.")
    ("knownPosesGeometricErrorMax", po::value<double>(&knownPosesGeometricErrorMax)->default_value(knownPosesGeometricErrorMax),
      "Maximum error (in pixels) allowed for features matching guided by geometric information from known camera poses. "
      "If set to 0 it lets the ACRansac select an optimal value.")
    ("savePutativeMatches", po::value<bool>(&savePutativeMatches)->default_value(savePutativeMatches),
      "Save putative matches.")
    ("guidedMatching", po::value<bool>(&guidedMatching)->default_value(guidedMatching),
      "Use the found model to improve the pairwise correspondences.")
    ("crossMatching", po::value<bool>(&crossMatching)->default_value(crossMatching),
      "Make sure that the matching process is symmetric (same matches for I->J than fo J->I).")
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
      "Range size.")
    ("randomSeed", po::value<int>(&randomSeed)->default_value(randomSeed),
      "This seed value will generate a sequence using a linear random generator. Set -1 to use a random seed.")
    ;

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

  const double defaultLoRansacMatchingError = 20.0;
  if(!adjustRobustEstimatorThreshold(geometricEstimator, geometricErrorMax, defaultLoRansacMatchingError))
    return EXIT_FAILURE;

  ALICEVISION_COUT("Program called with the following parameters:");
  ALICEVISION_COUT(vm);

  // set verbose level
  system::Logger::get()->setLogLevel(verboseLevel);

  std::mt19937 randomNumberGenerator(randomSeed == -1 ? std::random_device()() : randomSeed);

  // check and set input options
  if(matchesFolder.empty() || !fs::is_directory(matchesFolder))
  {
    ALICEVISION_LOG_ERROR("Invalid output matches folder: " + matchesFolder);
    return EXIT_FAILURE;
  }
  

  const matchingImageCollection::EGeometricFilterType geometricFilterType = matchingImageCollection::EGeometricFilterType_stringToEnum(geometricFilterTypeName);

  if(describerTypesName.empty())
  {
    ALICEVISION_LOG_ERROR("Empty option: --describerMethods");
    return EXIT_FAILURE;
  }

  // Feature matching
  // a. Load SfMData Views & intrinsics data
  // b. Compute putative descriptor matches
  // c. Geometric filtering of putative matches
  // d. Export some statistics

  // a. Load SfMData (image view & intrinsics data)

  SfMData sfmData;
  if(!sfmDataIO::Load(sfmData, sfmDataFilename, sfmDataIO::ESfMData(sfmDataIO::VIEWS|sfmDataIO::INTRINSICS|sfmDataIO::EXTRINSICS)))
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

  
  // We assume that there is only one pair for (I,J) and (J,I)
  if(predefinedPairList.empty())
  {
    pairs = exhaustivePairs(sfmData.getViews(), rangeStart, rangeSize);
  }
  else
  {
    for(const std::string& imagePairsFile: predefinedPairList)
    {
      ALICEVISION_LOG_INFO("Load pair list from file: " << imagePairsFile);
      if(!loadPairs(imagePairsFile, pairs, rangeStart, rangeSize))
          return EXIT_FAILURE;
    }
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

  PairwiseMatches mapPutativesMatches;

  // allocate the right Matcher according the Matching requested method
  EMatcherType collectionMatcherType = EMatcherType_stringToEnum(nearestMatchingMethod);
  std::unique_ptr<IImageCollectionMatcher> imageCollectionMatcher = createImageCollectionMatcher(collectionMatcherType, distRatio, crossMatching);

  const std::vector<feature::EImageDescriberType> describerTypes = feature::EImageDescriberType_stringToEnums(describerTypesName);

  ALICEVISION_LOG_INFO("There are " << sfmData.getViews().size() << " views and " << pairs.size() << " image pairs.");

  ALICEVISION_LOG_INFO("Load features and descriptors");

  // load the corresponding view regions
  RegionsPerView regionPerView;
  if(!sfm::loadRegionsPerView(regionPerView, sfmData, featuresFolders, describerTypes, filter))
  {
    ALICEVISION_LOG_ERROR("Invalid regions in '" + sfmDataFilename + "'");
    return EXIT_FAILURE;
  }

  // perform the matching
  system::Timer timer;
  PairSet pairsPoseKnown;
  PairSet pairsPoseUnknown;

  if(matchFromKnownCameraPoses)
  {
      for(const auto& p: pairs)
      {
        if(sfmData.isPoseAndIntrinsicDefined(p.first) && sfmData.isPoseAndIntrinsicDefined(p.second))
        {
            pairsPoseKnown.insert(p);
        }
        else
        {
            pairsPoseUnknown.insert(p);
        }
      }
  }
  else
  {
      pairsPoseUnknown = pairs;
  }

  if(!pairsPoseKnown.empty())
  {
    // compute matches from known camera poses when you have an initialization on the camera poses
    ALICEVISION_LOG_INFO("Putative matches from known poses: " << pairsPoseKnown.size() << " image pairs.");

    sfm::StructureEstimationFromKnownPoses structureEstimator;
    structureEstimator.match(sfmData, pairsPoseKnown, regionPerView, knownPosesGeometricErrorMax);
    mapPutativesMatches = structureEstimator.getPutativesMatches();
  }

  if(!pairsPoseUnknown.empty())
  {
      ALICEVISION_LOG_INFO("Putative matches (unknown poses): " << pairsPoseUnknown.size() << " image pairs.");
      // match feature descriptors between them without geometric notion

      for(const feature::EImageDescriberType descType : describerTypes)
      {
        assert(descType != feature::EImageDescriberType::UNINITIALIZED);
        ALICEVISION_LOG_INFO(EImageDescriberType_enumToString(descType) + " Regions Matching");

        // photometric matching of putative pairs
        imageCollectionMatcher->Match(randomNumberGenerator, regionPerView, pairsPoseUnknown, descType, mapPutativesMatches);

        // TODO: DELI
        // if(!guided_matching) regionPerView.clearDescriptors()
      }

  }

  if(mapPutativesMatches.empty())
  {
    ALICEVISION_LOG_INFO("No putative feature matches.");
    // If we only compute a selection of matches, we may have no match.
    return rangeSize ? EXIT_SUCCESS : EXIT_FAILURE;
  }

  if(geometricFilterType == EGeometricFilterType::HOMOGRAPHY_GROWING)
  {
    // sort putative matches according to their Lowe ratio
    // This is suggested by [F.Srajer, 2016]: the matches used to be the seeds of the homographies growing are chosen according
    // to the putative matches order. This modification should improve recall.
    for(auto& imgPair: mapPutativesMatches)
    {
      for(auto& descType: imgPair.second)
      {
        IndMatches & matches = descType.second;
        sortMatches_byDistanceRatio(matches);
      }
    }
  }

  // when a range is specified, generate a file prefix to reflect the current iteration (rangeStart/rangeSize)
  // => with matchFilePerImage: avoids overwriting files if a view is present in several iterations
  // => without matchFilePerImage: avoids overwriting the unique resulting file
  const std::string filePrefix = rangeSize > 0 ? std::to_string(rangeStart/rangeSize) + "." : "";

  ALICEVISION_LOG_INFO(std::to_string(mapPutativesMatches.size()) << " putative image pair matches");

  for(const auto& imageMatch: mapPutativesMatches)
    ALICEVISION_LOG_INFO("\t- image pair (" + std::to_string(imageMatch.first.first) << ", " + std::to_string(imageMatch.first.second) + ") contains " + std::to_string(imageMatch.second.getNbAllMatches()) + " putative matches.");

  // export putative matches
  if(savePutativeMatches)
    Save(mapPutativesMatches, (fs::path(matchesFolder) / "putativeMatches").string(), fileExtension, matchFilePerImage, filePrefix);

  ALICEVISION_LOG_INFO("Task (Regions Matching) done in (s): " + std::to_string(timer.elapsed()));

  /*
  // TODO: DELI
  if(exportDebugFiles)
  {
    //-- export putative matches Adjacency matrix
    PairwiseMatchingToAdjacencyMatrixSVG(sfmData.getViews().size(),
      mapPutativesMatches,
      (fs::path(matchesFolder) / "PutativeAdjacencyMatrix.svg").string());
    //-- export view pair graph once putative graph matches have been computed
    {
      std::set<IndexT> set_ViewIds;

      std::transform(sfmData.getViews().begin(), sfmData.getViews().end(),
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

  timer.reset();

  matching::PairwiseMatches geometricMatches;

  ALICEVISION_LOG_INFO("Geometric filtering: using " << matchingImageCollection::EGeometricFilterType_enumToString(geometricFilterType));

  switch(geometricFilterType)
  {

    case EGeometricFilterType::NO_FILTERING:
      geometricMatches = mapPutativesMatches;
    break;

    case EGeometricFilterType::FUNDAMENTAL_MATRIX:
    {
      matchingImageCollection::robustModelEstimation(geometricMatches,
        &sfmData,
        regionPerView,
        GeometricFilterMatrix_F_AC(geometricErrorMax, maxIteration, geometricEstimator),
        mapPutativesMatches,
        randomNumberGenerator,
        guidedMatching);
    }
    break;

  case EGeometricFilterType::FUNDAMENTAL_WITH_DISTORTION:
  {
    matchingImageCollection::robustModelEstimation(geometricMatches,
      &sfmData,
      regionPerView,
      GeometricFilterMatrix_F_AC(geometricErrorMax, maxIteration, geometricEstimator, true),
      mapPutativesMatches,
      randomNumberGenerator,
      guidedMatching);
  }
  break;

    case EGeometricFilterType::ESSENTIAL_MATRIX:
    {
      matchingImageCollection::robustModelEstimation(geometricMatches,
        &sfmData,
        regionPerView,
        GeometricFilterMatrix_E_AC(geometricErrorMax, maxIteration),
        mapPutativesMatches,
        randomNumberGenerator,
        guidedMatching);

      // perform an additional check to remove pairs with poor overlap
      std::vector<PairwiseMatches::key_type> toRemoveVec;
      for(PairwiseMatches::const_iterator iterMap = geometricMatches.begin();
        iterMap != geometricMatches.end(); ++iterMap)
      {
        const size_t putativePhotometricCount = mapPutativesMatches.find(iterMap->first)->second.getNbAllMatches();
        const size_t putativeGeometricCount = iterMap->second.getNbAllMatches();
        const float ratio = putativeGeometricCount / (float)putativePhotometricCount;
        if (putativeGeometricCount < 50 || ratio < .3f)
          toRemoveVec.push_back(iterMap->first); // the image pair will be removed
      }

      // remove discarded pairs
      for(std::vector<PairwiseMatches::key_type>::const_iterator iter = toRemoveVec.begin();
          iter != toRemoveVec.end(); ++iter)
        geometricMatches.erase(*iter);
    }
    break;

    case EGeometricFilterType::HOMOGRAPHY_MATRIX:
    {
      const bool onlyGuidedMatching = true;
      matchingImageCollection::robustModelEstimation(geometricMatches,
        &sfmData,
        regionPerView,
        GeometricFilterMatrix_H_AC(geometricErrorMax, maxIteration),
        mapPutativesMatches, randomNumberGenerator, guidedMatching,
        onlyGuidedMatching ? -1.0 : 0.6);
    }
    break;

    case EGeometricFilterType::HOMOGRAPHY_GROWING:
    {
      matchingImageCollection::robustModelEstimation(geometricMatches,
        &sfmData,
        regionPerView,
        GeometricFilterMatrix_HGrowing(geometricErrorMax, maxIteration),
        mapPutativesMatches,
        randomNumberGenerator,
        guidedMatching);
    }
    break;
  }

  ALICEVISION_LOG_INFO(std::to_string(geometricMatches.size()) + " geometric image pair matches:");
  for(const auto& matchGeo: geometricMatches)
    ALICEVISION_LOG_INFO("\t- image pair (" + std::to_string(matchGeo.first.first) + ", " + std::to_string(matchGeo.first.second) + ") contains " + std::to_string(matchGeo.second.getNbAllMatches()) + " geometric matches.");

  // grid filtering
  ALICEVISION_LOG_INFO("Grid filtering");

  PairwiseMatches finalMatches;
  
  {
    for(const auto& geometricMatch: geometricMatches)
    {
      //Get the image pair and their matches.
      const Pair& indexImagePair = geometricMatch.first;
      const aliceVision::matching::MatchesPerDescType& matchesPerDesc = geometricMatch.second;

      for(const auto& match: matchesPerDesc)
      {
        const feature::EImageDescriberType descType = match.first;
        assert(descType != feature::EImageDescriberType::UNINITIALIZED);
        const aliceVision::matching::IndMatches& inputMatches = match.second;

        const feature::Regions* rRegions = &regionPerView.getRegions(indexImagePair.second, descType);
        const feature::Regions* lRegions = &regionPerView.getRegions(indexImagePair.first, descType);

        // get the regions for the current view pair:
        if(rRegions && lRegions)
        {
          // sorting function:
          aliceVision::matching::IndMatches outMatches;
          sortMatches_byFeaturesScale(inputMatches, *lRegions, *rRegions, outMatches);

          if(useGridSort)
          {
            // TODO: rename as matchesGridOrdering
              matchesGridFiltering(*lRegions, sfmData.getView(indexImagePair.first).getImgSize(),
                                   *rRegions, sfmData.getView(indexImagePair.second).getImgSize(),
                                   indexImagePair, outMatches);
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
  Save(finalMatches, matchesFolder, fileExtension, matchFilePerImage, filePrefix);
  ALICEVISION_LOG_INFO("Task done in (s): " + std::to_string(timer.elapsed()));

  // d. Export some statistics
  if(exportDebugFiles)
  {
    // export Adjacency matrix
    ALICEVISION_LOG_INFO("Export Adjacency Matrix of the pairwise's geometric matches");
    PairwiseMatchingToAdjacencyMatrixSVG(sfmData.getViews().size(),
      finalMatches,(fs::path(matchesFolder) / "GeometricAdjacencyMatrix.svg").string());

    /*
    // export view pair graph once geometric filter have been done
    {
      std::set<IndexT> set_ViewIds;
      std::transform(sfmData.getViews().begin(), sfmData.getViews().end(),
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
    getStatsMap(geometricMatches);
  }
#endif

  return EXIT_SUCCESS;
}
