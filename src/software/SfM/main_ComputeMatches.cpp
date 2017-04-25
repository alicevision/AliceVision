
// Copyright (c) 2012, 2013 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
#include "openMVG/sfm/pipelines/RegionsIO.hpp"
#include "openMVG/sfm/pipelines/sfm_engine.hpp"
#include "openMVG/features/FeaturesPerView.hpp"
#include "openMVG/features/RegionsPerView.hpp"

/// Generic Image Collection image matching
#include "openMVG/features/image_describer.hpp"
#include "openMVG/features/ImageDescriberCommon.hpp"
#include "openMVG/matching_image_collection/MatchingCommon.hpp"
#include "openMVG/matching_image_collection/Matcher_Regions_AllInMemory.hpp"
#include "openMVG/matching_image_collection/Cascade_Hashing_Matcher_Regions_AllInMemory.hpp"
#include "openMVG/matching_image_collection/GeometricFilter.hpp"
#include "openMVG/matching_image_collection/F_ACRobust.hpp"
#include "openMVG/matching_image_collection/E_ACRobust.hpp"
#include "openMVG/matching_image_collection/H_ACRobust.hpp"
#include "openMVG/matching/pairwiseAdjacencyDisplay.hpp"
#include "openMVG/matching/indMatch_utils.hpp"
#include "openMVG/system/timer.hpp"

#include "openMVG/graph/graph.hpp"
#include "openMVG/stl/stl.hpp"
#include "third_party/cmdLine/cmdLine.h"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

#include "openMVG/features/selection.hpp"

#include <cstdlib>
#include <fstream>
#include <cctype>

using namespace openMVG;
using namespace openMVG::cameras;
using namespace openMVG::matching;
using namespace openMVG::robust;
using namespace openMVG::sfm;
using namespace openMVG::matching_image_collection;
using namespace std;

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
#ifdef OPENMVG_DEBUG_MATCHING
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
  CmdLine cmd;

  std::string sfmDataFilename;
  std::string matchesDirectory = "";
  std::string geometricModel = "f";
  std::string describerMethods = "SIFT";
  float distRatio = 0.8f;
  int matchingVideoMode = -1;
  std::string predefinedPairList = "";
  int rangeStart = -1;
  int rangeSize = 0;
  std::string nearestMatchingMethod = "ANN_L2";
  bool savePutativeMatches = false;
  bool guidedMatching = false;
  int maxIteration = 2048;
  bool matchFilePerImage = false;
  size_t numMatchesToKeep = 0;
  bool useGridSort = false;
  bool exportDebugFiles = false;

  //required
  cmd.add( make_option('i', sfmDataFilename, "input_file") );
  cmd.add( make_option('o', matchesDirectory, "out_dir") );
  // Options
  cmd.add( make_option('m', describerMethods, "describerMethods") );
  cmd.add( make_option('r', distRatio, "ratio") );
  cmd.add( make_option('g', geometricModel, "geometric_model") );
  cmd.add( make_option('v', matchingVideoMode, "video_mode_matching") );
  cmd.add( make_option('l', predefinedPairList, "pair_list") );
  cmd.add( make_option('s', rangeStart, "range_start") );
  cmd.add( make_option('d', rangeSize, "range_size") );
  cmd.add( make_option('n', nearestMatchingMethod, "nearest_matching_method") );
  cmd.add( make_option('f', savePutativeMatches, "save_putative_matches") );
  cmd.add( make_option('M', guidedMatching, "guided_matching") );
  cmd.add( make_option('I', maxIteration, "max_iteration") );
  cmd.add( make_option('x', matchFilePerImage, "match_file_per_image") );
  cmd.add(make_option('u', numMatchesToKeep, "max_matches"));
  cmd.add(make_option('y', useGridSort, "use_grid_sort"));
  cmd.add(make_option('e', exportDebugFiles, "export_debug_files"));

  try
  {
    if(argc == 1) throw std::string("Invalid command line parameter.");
    cmd.process(argc, argv);
  }
  catch(const std::string& s)
  {
    // TODO
    std::cerr << "Usage: " << argv[0] << '\n'
      << "[-i|--input_file] a SfM_Data file\n"
      << "[-o|--out_dir path]\n"
      << "   path of the directory containing the extracted features and in which computed matches will be stored\n"
      << "\n[Optional]\n"
      << "[-f|--save_putative_matches] Save putative matches\n"
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
      << "  use the found model to improve the pairwise correspondences.\n"  
      << "[-r|--ratio] Distance ratio to discard non meaningful matches\n"
      << "   0.8: (default).\n"
      << "[-g|--geometric_model]\n"
      << "  (pairwise correspondences filtering thanks to robust model estimation):\n"
      << "   f: (default) fundamental matrix,\n"
      << "   e: essential matrix,\n"
      << "   h: homography matrix.\n"
      << "[-v|--video_mode_matching]\n"
      << "  (sequence matching with an overlap of X images)\n"
      << "   X: with match 0 with (1->X), ...]\n"
      << "   2: will match 0 with (1,2), 1 with (2,3), ...\n"
      << "   3: will match 0 with (1,2,3), 1 with (2,3,4), ...\n"
      << "[-l]--pair_list] filepath\n"
      << "  A file which contains the list of matches to perform.\n"
      << "[-s]--range_start] range image index start\n"
      << "  To compute only the matches for specified range.\n"
      << "  This allows to compute different matches on different computers in parallel.\n"
      << "[-d]--range_size] range size\n"
      << "  To compute only the matches for specified range.\n"
      << "  This allows to compute different matches on different computers in parallel.\n"
      << "[-n|--nearest_matching_method]\n"
      << "  For Scalar based regions descriptor:\n"
      << "    BRUTE_FORCE_L2: L2 BruteForce matching,\n"
      << "    ANN_L2: L2 Approximate Nearest Neighbor matching (default),\n"
      << "    CASCADE_HASHING_L2: L2 Cascade Hashing matching.\n"
      << "    FAST_CASCADE_HASHING_L2:\n"
      << "      L2 Cascade Hashing with precomputed hashed regions\n"
      << "     (faster than CASCADE_HASHING_L2 but use more memory).\n"
      << "  For Binary based descriptor:\n"
      << "    BRUTE_FORCE_HAMMING: BruteForce Hamming matching.\n"
      << "[-M|--guided_matching]\n"
      << "  use the found model to improve the pairwise correspondences.\n"
      << "[-I|--max_iteration]\n"
      << "  number of maximum iterations allowed in ransac step.\n"
      << "[-x|--match_file_per_image]\n"
      << "  Save matches in a separate file per image\n"
      << "[-u|--max_matches]\n"
      << "[-y|--use_grid_sort]\n"
      << "  Use matching grid sort\n"
      << "[-e|--export_debug_files] Export debug files (svg, dot)"
      << std::endl;

    std::cerr << s << std::endl;
    return EXIT_FAILURE;
  }

  std::cout << " You called : " << "\n"
            << argv[0] << "\n"
            << "--input_file " << sfmDataFilename << "\n"
            << "--out_dir " << matchesDirectory << "\n"
            << "Optional parameters:" << "\n"
            << "--save_putative_matches " << savePutativeMatches << "\n"
            << "--describerMethods " << describerMethods << "\n"
            << "--ratio " << distRatio << "\n"
            << "--geometric_model " << geometricModel << "\n"
            << "--video_mode_matching " << matchingVideoMode << "\n"
            << "--pair_list " << predefinedPairList << "\n"
            << "--range_start " << rangeStart <<  "\n"
            << "--range_size " << rangeSize <<  "\n"
            << "--nearest_matching_method " << nearestMatchingMethod << "\n"
            << "--guided_matching " << guidedMatching << "\n"
            << "--match_file_per_image " << matchFilePerImage << "\n"
            << "--max_matches " << numMatchesToKeep  << "\n"
            << "--use_grid_sort " << useGridSort << "\n"
            << "--max_iteration " << maxIteration << "\n"
            << "--export_debug_files " << exportDebugFiles
            << std::endl;

  EPairMode pairMode = (matchingVideoMode == -1 ) ? PAIR_EXHAUSTIVE : PAIR_CONTIGUOUS;

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
  
  if(describerMethods.empty())
  {
    std::cerr << "\nError: describerMethods argument is empty." << std::endl;
    return EXIT_FAILURE;
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

  // -----------------------------
  // - Load SfM_Data Views & intrinsics data
  // a. Compute putative descriptor matches
  // b. Geometric filtering of putative matches
  // + Export some statistics
  // -----------------------------

  //---------------------------------------
  // Read SfM Scene (image view & intrinsics data)
  //---------------------------------------
  SfM_Data sfmData;
  if(!Load(sfmData, sfmDataFilename, ESfM_Data(VIEWS|INTRINSICS))) {
    std::cerr << std::endl
      << "The input SfM_Data file \""<< sfmDataFilename << "\" cannot be read." << std::endl;
    return EXIT_FAILURE;
  }

  //---------------------------------------
  // Load SfM Scene regions
  //---------------------------------------
  using namespace openMVG::features;

  //---------------------------------------
  // a. Compute putative descriptor matches
  //    - Descriptor matching (according user method choice)
  //    - Keep correspondences only if NearestNeighbor ratio is ok
  //---------------------------------------

  // From matching mode compute the pair list that have to be matched:
  Pair_Set pairs;
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
  std::unique_ptr<Matcher> collectionMatcher = createMatcher(collectionMatcherType, distRatio);
  
  const std::vector<features::EImageDescriberType> describerTypes = features::EImageDescriberType_stringToEnums(describerMethods);
  
  std::cout << "There are " << sfmData.GetViews().size() << " views and " << pairs.size() << " image pairs." << std::endl;
  
  // Load the corresponding view regions
  RegionsPerView regionPerView;

  // Perform the matching
  system::Timer timer;

  if(!sfm::loadRegionsPerView(regionPerView, sfmData, matchesDirectory, describerTypes, filter))
  {
    std::cerr << std::endl << "Invalid regions." << std::endl;
    return EXIT_FAILURE;
  }

  for(const features::EImageDescriberType descType : describerTypes)
  {
    std::cout << "-> " << EImageDescriberType_enumToString(descType) << " Regions Matching" << std::endl;

    // Photometric matching of putative pairs
    collectionMatcher->Match(sfmData, regionPerView, pairs, descType, mapPutativesMatches);

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
    Save(mapPutativesMatches, matchesDirectory, "putative", "txt", matchFilePerImage);

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
  
#ifdef OPENMVG_DEBUG_MATCHING
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

  std::unique_ptr<ImageCollectionGeometricFilter> filter_ptr(
    new ImageCollectionGeometricFilter(&sfmData, regionPerView));

  if(!filter_ptr)
  {
    std::cerr << "An error occurred while generating the geometric filter! Aborting..." << std::endl;
    return EXIT_FAILURE;
  }

  timer.reset();
  std::cout << std::endl << " - Geometric filtering - " << std::endl;

  matching::PairwiseMatches map_GeometricMatches;
  switch(geometricModelToCompute)
  {
    case HOMOGRAPHY_MATRIX:
    {
      const bool bGeometric_only_guided_matching = true;
      filter_ptr->Robust_model_estimation(GeometricFilter_HMatrix_AC(std::numeric_limits<double>::infinity(), maxIteration),
        mapPutativesMatches, guidedMatching,
        bGeometric_only_guided_matching ? -1.0 : 0.6);
      map_GeometricMatches = filter_ptr->Get_geometric_matches();
    }
    break;
    case FUNDAMENTAL_MATRIX:
    {
      filter_ptr->Robust_model_estimation(GeometricFilter_FMatrix_AC(std::numeric_limits<double>::infinity(), maxIteration),
        mapPutativesMatches, guidedMatching);
      map_GeometricMatches = filter_ptr->Get_geometric_matches();
    }
    break;
    case ESSENTIAL_MATRIX:
    {
      filter_ptr->Robust_model_estimation(GeometricFilter_EMatrix_AC(std::numeric_limits<double>::infinity(), maxIteration),
        mapPutativesMatches, guidedMatching);
      map_GeometricMatches = filter_ptr->Get_geometric_matches();

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
      const openMVG::matching::MatchesPerDescType& matchesPerDesc = matchGeo.second;

      for(const auto& match: matchesPerDesc)
      {
        const features::EImageDescriberType descType = match.first;
        const openMVG::matching::IndMatches& inputMatches = match.second;

        const features::Feat_Regions<features::SIOPointFeature>* rRegions = dynamic_cast<const features::Feat_Regions<features::SIOPointFeature>*>(&regionPerView.getRegions(indexImagePair.second, descType));
        const features::Feat_Regions<features::SIOPointFeature>* lRegions = dynamic_cast<const features::Feat_Regions<features::SIOPointFeature>*>(&regionPerView.getRegions(indexImagePair.first, descType));

        //Get the regions for the current view pair:
        if(rRegions && lRegions)
        {
          //Sorting function:
          openMVG::matching::IndMatches outMatches;
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
      std::cout << " * Image pair (" << matchGridFiltering.first.first << ", " << matchGridFiltering.first.second << ") contains " << matchGridFiltering.second.size() << " geometric matches." << std::endl;
    }
  }

  //---------------------------------------
  //-- Export geometric filtered matches
  //---------------------------------------
  std::cout << "Save geometric matches." << std::endl;
  Save(finalMatches, matchesDirectory, geometricMode, "txt", matchFilePerImage);

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

#ifdef OPENMVG_DEBUG_MATCHING
  {
    std::cout << "GEOMETRIC" << std::endl;
    getStatsMap(map_GeometricMatches);
  }
#endif

  return EXIT_SUCCESS;
}
