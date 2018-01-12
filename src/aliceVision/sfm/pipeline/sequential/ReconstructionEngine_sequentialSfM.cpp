// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "aliceVision/sfm/pipeline/sequential/ReconstructionEngine_sequentialSfM.hpp"
#include "aliceVision/sfm/pipeline/RelativePoseInfo.hpp"
#include "aliceVision/sfm/utils/statistics.hpp"
#include "aliceVision/sfm/sfmDataIO.hpp"
#include "aliceVision/sfm/BundleAdjustmentCeres.hpp"
#include "aliceVision/sfm/LocalBundleAdjustmentCeres.hpp"
#include "aliceVision/sfm/sfmDataFilters.hpp"

#include "aliceVision/feature/FeaturesPerView.hpp"
#include "aliceVision/matching/IndMatch.hpp"
#include "aliceVision/multiview/essential.hpp"
#include "aliceVision/multiview/triangulation/triangulationDLT.hpp"
#include "aliceVision/multiview/triangulation/Triangulation.hpp"
#include "aliceVision/multiview/triangulation/NViewsTriangulationLORansac.hpp"
#include "aliceVision/graph/connectedComponent.hpp"
#include "aliceVision/stl/stl.hpp"
#include "aliceVision/system/Timer.hpp"
#include "aliceVision/system/cpu.hpp"
#include "aliceVision/system/MemoryInfo.hpp"
#include <aliceVision/config.hpp>
#include "aliceVision/robustEstimation/LORansac.hpp"
#include "aliceVision/robustEstimation/ScoreEvaluator.hpp"

#include "dependencies/htmlDoc/htmlDoc.hpp"

#include <boost/progress.hpp>
#include <boost/format.hpp>

#include <tuple>
#include <iostream>
#include <algorithm>

#ifdef _MSC_VER
#pragma warning( once : 4267 ) //warning C4267: 'argument' : conversion from 'size_t' to 'const int', possible loss of data
#endif

//#define ALICEVISION_NEXTBESTVIEW_WITHOUT_SCORE

namespace aliceVision {
namespace sfm {

using namespace aliceVision::geometry;
using namespace aliceVision::camera;

/**
 * @brief Compute indexes of all features in a fixed size pyramid grid.
 * These precomputed values are useful to the next best view selection for incremental SfM.
 *
 * @param[in] tracksPerView: The list of TrackID per view
 * @param[in] map_tracks: All putative tracks
 * @param[in] views: All views
 * @param[in] featuresProvider: Input features and descriptors
 * @param[in] pyramidDepth: Depth of the pyramid.
 * @param[out] tracksPyramidPerView:
 *             Precomputed list of pyramid cells ID for each track in each view.
 */
void computeTracksPyramidPerView(
    const track::TracksPerView& tracksPerView,
    const track::TracksMap& map_tracks,
    const Views& views,
    const feature::FeaturesPerView& featuresProvider,
    const std::size_t pyramidBase,
    const std::size_t pyramidDepth,
    track::TracksPyramidPerView& tracksPyramidPerView)
{
  std::vector<std::size_t> widthPerLevel(pyramidDepth);
  std::vector<std::size_t> startPerLevel(pyramidDepth);
  std::size_t start = 0;
  for(size_t level = 0; level < pyramidDepth; ++level)
  {
    startPerLevel[level] = start;
    widthPerLevel[level] = std::pow(pyramidBase, level+1);
    start += Square(widthPerLevel[level]);
  }

  tracksPyramidPerView.reserve(tracksPerView.size());
  for(const auto& viewTracks: tracksPerView)
  {
    auto& trackPyramid = tracksPyramidPerView[viewTracks.first];
    // reserve 500 tracks in each view
    trackPyramid.reserve(500 * pyramidDepth);
  }

  for(const auto& viewTracks: tracksPerView)
  {
    const auto viewId = viewTracks.first;
    auto& tracksPyramidIndex = tracksPyramidPerView[viewId];
    const View& view = *views.at(viewId).get();
    std::vector<double> cellWidthPerLevel(pyramidDepth);
    std::vector<double> cellHeightPerLevel(pyramidDepth);
    for(std::size_t level = 0; level < pyramidDepth; ++level)
    {
      cellWidthPerLevel[level] = (double)view.getWidth() / (double)widthPerLevel[level];
      cellHeightPerLevel[level] = (double)view.getHeight() / (double)widthPerLevel[level];
    }
    for(std::size_t i = 0; i < viewTracks.second.size(); ++i)
    {
      const std::size_t trackId = viewTracks.second[i];
      const track::Track& track = map_tracks.at(trackId);
      const std::size_t featIndex = track.featPerView.at(viewId);
      const auto& feature = featuresProvider.getFeatures(viewId, track.descType)[featIndex]; 
      
      for(std::size_t level = 0; level < pyramidDepth; ++level)
      {
        std::size_t xCell = std::floor(std::max(feature.x(), 0.0f) / cellWidthPerLevel[level]);
        std::size_t yCell = std::floor(std::max(feature.y(), 0.0f) / cellHeightPerLevel[level]);
        xCell = std::min(xCell, widthPerLevel[level] - 1);
        yCell = std::min(yCell, widthPerLevel[level] - 1);
        const std::size_t levelIndex = xCell + yCell * widthPerLevel[level];
        assert(levelIndex < Square(widthPerLevel[level]));
        tracksPyramidIndex[trackId * pyramidDepth + level] = startPerLevel[level] + levelIndex;
      }
    }
  }
}

ReconstructionEngine_sequentialSfM::ReconstructionEngine_sequentialSfM(
  const SfMData & sfm_data,
  const std::string & soutDirectory,
  const std::string & sloggingFile)
  : ReconstructionEngine(sfm_data, soutDirectory),
    _sLoggingFile(sloggingFile),
    _userInitialImagePair(Pair(0,0)),
    _camType(EINTRINSIC(PINHOLE_CAMERA_RADIAL3))
{
  if (!_sLoggingFile.empty())
  {
    // setup HTML logger
    _htmlDocStream = std::make_shared<htmlDocument::htmlDocumentStream>("[log] Sequential SfM reconstruction");
    _htmlDocStream->pushInfo(
          htmlDocument::htmlMarkup("h1", std::string("[log] Sequential SfM reconstruction")));
    _htmlDocStream->pushInfo("<hr>");
    
    _htmlDocStream->pushInfo( "Dataset info:");
    _htmlDocStream->pushInfo( "Views count: " +
                              htmlDocument::toString( sfm_data.GetViews().size()) + "<br>");
  }
  
  // Init remaining image list
  for (Views::const_iterator itV = sfm_data.GetViews().begin();
       itV != sfm_data.GetViews().end(); ++itV)
  {
    _set_remainingViewId.insert(itV->second.get()->getViewId());
  }
}

ReconstructionEngine_sequentialSfM::~ReconstructionEngine_sequentialSfM()
{
  if (!_sLoggingFile.empty())
  {
    // Save the reconstruction Log
    std::ofstream htmlFileStream(_sLoggingFile.c_str());
    htmlFileStream << _htmlDocStream->getDoc();
  }
}

// Compute robust Resection of remaining images
// - group of images will be selected and resection + scene completion will be tried
void ReconstructionEngine_sequentialSfM::robustResectionOfImages(
  const std::set<size_t>& viewIds,
  std::set<size_t>& set_reconstructedViewId,
  std::set<size_t>& set_rejectedViewId)
{
  static const std::size_t maxImagesPerGroup = 30;
  
  size_t imageIndex = 0;
  size_t resectionGroupIndex = 0;
  std::set<size_t> set_remainingViewId(viewIds);
  std::vector<size_t> vec_possible_resection_indexes;
  
  while (findNextImagesGroupForResection(vec_possible_resection_indexes, set_remainingViewId))
  {
    if(vec_possible_resection_indexes.empty())
    {
      break;
    }
    
    // The beginning of the incremental SfM is a well known risky and
    // unstable step which has a big impact on the final result.
    // The Bundle Adjustment is an intensive computing step so we only use it
    // every N cameras.
    // We make an exception for the first 'nbFirstUnstableCameras' cameras
    // and perform a BA for each camera because it makes the results
    // more stable and it's quite cheap because we have few data.
    static const std::size_t nbFirstUnstableCameras = 30;
    
    if (_sfm_data.GetPoses().size() < nbFirstUnstableCameras)
    {
      // Add images one by one to reconstruct the first cameras.
      ALICEVISION_LOG_DEBUG("RobustResectionOfImages : beginning of the incremental SfM" << std::endl
                        << "Only the first image of the resection group is used." << std::endl
                        << "First image ViewId : " << vec_possible_resection_indexes.front() << std::endl
                        << "# unstable poses : " << _sfm_data.GetPoses().size() << " / " << nbFirstUnstableCameras << std::endl);
      
      vec_possible_resection_indexes.resize(1);
    }
  
    ALICEVISION_LOG_DEBUG("Resection group start " << resectionGroupIndex << " with " << vec_possible_resection_indexes.size() << " images.\n");
    auto chrono_start = std::chrono::steady_clock::now();
    bool bImageAdded = false;
    
    // get reconstructed views before resection
    const std::set<IndexT> prevReconstructedViews = _sfm_data.getValidViews();
    
    // Limit to a maximum number of cameras added to ensure that
    // we don't add too much data in one step without bundle adjustment.
    if(vec_possible_resection_indexes.size() > maxImagesPerGroup)
      vec_possible_resection_indexes.resize(maxImagesPerGroup);
      
    // add images to the 3D reconstruction
#pragma omp parallel for 
    for (std::size_t i = 0; i < vec_possible_resection_indexes.size(); i++)
    {
      const std::size_t possible_resection_index = vec_possible_resection_indexes.at(i);
      const std::size_t currentIndex = imageIndex;
      ++imageIndex;
      
      {
        const View& view = *_sfm_data.views.at(possible_resection_index);
        
        if(view.isPartOfRig())
        {
          // Some views can become indirectly localized when the sub-pose becomes defined
          if(_sfm_data.IsPoseAndIntrinsicDefined(view.getViewId()))
          {
            ALICEVISION_LOG_DEBUG("Resection of image " << currentIndex << " ID=" << possible_resection_index << " was skipped." << std::endl
                                  << "RigID=" << view.getRigId() << " Sub-poseID=" << view.getSubPoseId()
                                  << " sub-pose and pose defined.");
#pragma omp critical                              
            set_remainingViewId.erase(possible_resection_index);
            
            continue;
          }
          
          // We cannot localize a view if it is part of an initialized RIG with unknown Rig Pose
          const bool knownPose = _sfm_data.existsPose(view);
          const Rig& rig = _sfm_data.getRig(view);
          const RigSubPose& subpose = rig.getSubPose(view.getSubPoseId());
          
          if(rig.isInitialized() &&
             !knownPose &&
             (subpose.status == ERigSubPoseStatus::UNINITIALIZED))
          {
            ALICEVISION_LOG_DEBUG("Resection of image " << currentIndex << " ID=" << possible_resection_index << " was skipped." << std::endl
                                  << "RigID=" << view.getRigId() << " Sub-poseID=" << view.getSubPoseId()
                                  << " Rig initialized but unkown pose and sub-pose.");
#pragma omp critical   
            set_remainingViewId.erase(possible_resection_index);
            
            continue;
          }
        }
      }
      
      ResectionData newResectionData;
      bool bResect = computeResection(possible_resection_index, newResectionData);
#pragma omp critical      
      {
        if (bResect)
        {
          bImageAdded |= bResect;
          updateScene(possible_resection_index, newResectionData);
          set_reconstructedViewId.insert(possible_resection_index);
          ALICEVISION_LOG_DEBUG("Resection of image: " << currentIndex << " ID=" << possible_resection_index << " succeed.");
          _sfm_data.GetViews().at(possible_resection_index)->setResectionId(resectionGroupIndex);
        }
        else
        {
          set_rejectedViewId.insert(possible_resection_index);
          ALICEVISION_LOG_DEBUG("Resection of image " << currentIndex << " ID=" << possible_resection_index << " was not possible.");
        }
        set_remainingViewId.erase(possible_resection_index);
      }
    }
    ALICEVISION_LOG_DEBUG("Resection of " << vec_possible_resection_indexes.size() << " new images took " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - chrono_start).count() << " msec.");
     
    // get new reconstructed views
    std::set<IndexT> newReconstructedViews;
    {
      // get reconstructed views after resection
      const std::set<IndexT> reconstructedViews = _sfm_data.getValidViews();
      
      std::set_difference(
            reconstructedViews.begin(),
            reconstructedViews.end(),
            prevReconstructedViews.begin(),
            prevReconstructedViews.end(),
            std::inserter(newReconstructedViews, newReconstructedViews.end()));
    }

    // Triangulate
    chrono_start = std::chrono::steady_clock::now();
    
    // TEMPORARY HACK: allows to keep an access to the old triangulatation algorithm (using 2 views only) 
    if (_minNbObservationsForTriangulation == 0)
      triangulate(_sfm_data, prevReconstructedViews, newReconstructedViews);
    else
      triangulateMultiViews_LORANSAC(_sfm_data, prevReconstructedViews, newReconstructedViews);
    
    ALICEVISION_LOG_DEBUG("Triangulation of the " << newReconstructedViews.size() << " newly reconstructed views took " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - chrono_start).count() << " msec.");
    
    if (bImageAdded)
    {
      if((resectionGroupIndex % 10) == 0)
      {
        chrono_start = std::chrono::steady_clock::now();
        // Scene logging as ply for visual debug
        std::ostringstream os;
        os << std::setw(8) << std::setfill('0') << resectionGroupIndex << "_Resection";
        Save(_sfm_data, stlplus::create_filespec(_sOutDirectory, os.str(), _sfmdataInterFileExtension), _sfmdataInterFilter);
        ALICEVISION_LOG_DEBUG("Save of file " << os.str() << " took " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - chrono_start).count() << " msec.");
      }
      ALICEVISION_LOG_DEBUG("Global Bundle start, resection group index: " << resectionGroupIndex << ".");
      chrono_start = std::chrono::steady_clock::now();
      std::size_t bundleAdjustmentIteration = 0;
      const std::size_t nbOutliersThreshold = 50;
      // Perform BA until all point are under the given precision
      do
      {
        auto chrono2_start = std::chrono::steady_clock::now();
        
        if (_uselocalBundleAdjustment)
          localBundleAdjustment(newReconstructedViews);
        else
          BundleAdjustment(_bFixedIntrinsics);
        
        ALICEVISION_LOG_DEBUG("Resection group index: " << resectionGroupIndex << ", bundle iteration: " << bundleAdjustmentIteration
                  << " took " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - chrono2_start).count() << " msec.");
        ++bundleAdjustmentIteration;
      }
      while (badTrackRejector(4.0, nbOutliersThreshold));
      ALICEVISION_LOG_DEBUG("Bundle with " << bundleAdjustmentIteration << " iterations took " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - chrono_start).count() << " msec.");
      chrono_start = std::chrono::steady_clock::now();
      
      std::set<IndexT> removedPosesId;
      bool contentRemoved = eraseUnstablePosesAndObservations(this->_sfm_data, _minPointsPerPose, _minTrackLength, &removedPosesId);
      
      if (_uselocalBundleAdjustment && contentRemoved)
      {
        // Get removed VIEWS index
        std::set<IndexT> removedViewsId;
        for (const auto& x : _sfm_data.GetViews())
        {
          if (removedPosesId.find(x.second->getPoseId()) != removedPosesId.end())
          {
            if (!_sfm_data.IsPoseAndIntrinsicDefined(x.second->getViewId()))
              removedViewsId.insert(x.second->getViewId());
            else
              ALICEVISION_LOG_WARNING("The view #" << x.second->getViewId() << " is set as Removed while it is still in the scene.");
          }
        }
        
        // Remove removed views to the graph
        _localBA_data->removeViewsToTheGraph(removedViewsId);
        ALICEVISION_LOG_DEBUG("Poses (index) removed to the reconstruction: " << removedPosesId);
      }
      ALICEVISION_LOG_DEBUG("eraseUnstablePosesAndObservations took " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - chrono_start).count() << " msec.");
    }
    ++resectionGroupIndex;
  }
  // Ensure there is no remaining outliers
  badTrackRejector(4.0, 0);
  eraseUnstablePosesAndObservations(this->_sfm_data, _minPointsPerPose, _minTrackLength);
}

bool ReconstructionEngine_sequentialSfM::Process()
{
  // Update cache values
  if(_pyramidWeights.size() != _pyramidDepth)
  {
    _pyramidWeights.resize(_pyramidDepth);
    std::size_t maxWeight = 0;
    for(std::size_t level = 0; level < _pyramidDepth; ++level)
    {
      std::size_t nbCells = Square(std::pow(_pyramidBase, level+1));
      // We use a different weighting strategy than [Schonberger 2016].
      // They use w = 2^l with l={1...L} (even if there is a typo in the text where they say to use w=2^{2*l}.
      // We prefer to give more importance to the first levels of the pyramid, so:
      // w = 2^{L-l} with L the number of levels in the pyramid.
      _pyramidWeights[level] = std::pow(2.0, (_pyramidDepth-(level+1)));
      maxWeight += nbCells * _pyramidWeights[level];
    }
    _pyramidThreshold = maxWeight * 0.2;
  }
  
  //-------------------
  //-- Incremental reconstruction
  //-------------------
  if (!initLandmarkTracks())
    return false;
  
  // Initial pair choice
  std::vector<Pair> initialImagePairCandidates;
  if(_userInitialImagePair == Pair(0,0))
  {
    if(!getBestInitialImagePairs(initialImagePairCandidates))
    {
      if(_userInteraction)
      {
        // Cannot find a valid initial pair, try to set it by hand?
        if(!chooseInitialPair(_userInitialImagePair))
          return false;
      }
      else
      {
        return false;
      }
    }
  }
  if(_userInitialImagePair != Pair(0,0))
  {
    //double, double, double, std::size_t, Pair
    initialImagePairCandidates.emplace_back(_userInitialImagePair);
  }
  
  bool successfullInitialization = false;
  // Initial pair Essential Matrix and [R|t] estimation.
  for(const auto& initialPairCandidate: initialImagePairCandidates)
  {
    if(makeInitialPair3D(initialPairCandidate))
    {
      // Successfully found an initial image pair
      ALICEVISION_LOG_DEBUG("Initial pair is: " << initialPairCandidate.first << ", " << initialPairCandidate.second);
      successfullInitialization = true;
      break;
    }
  }
  if(!successfullInitialization)
  {
    ALICEVISION_LOG_ERROR("Initialization failed after trying all possible initial image pairs.");
    return false;
  }
  
  // timer for stats
  aliceVision::system::Timer timer_sfm;

  std::set<std::size_t> reconstructedViewIds;
  std::set<std::size_t> rejectedViewIds;
  std::size_t nbRejectedLoops = 0;
  do
  {
    reconstructedViewIds.clear();
    rejectedViewIds.clear();
    
    // Compute robust Resection of remaining images
    // - group of images will be selected and resection + scene completion will be tried
    robustResectionOfImages(
          _set_remainingViewId,
          reconstructedViewIds,
          rejectedViewIds);
    // Remove all reconstructed views from the remaining views
    for(const std::size_t v: reconstructedViewIds)
    {
      _set_remainingViewId.erase(v);
    }

    ALICEVISION_LOG_DEBUG("SequentialSfM -- nbRejectedLoops: " << nbRejectedLoops);
    ALICEVISION_LOG_DEBUG("SequentialSfM -- reconstructedViewIds: " << reconstructedViewIds.size());
    ALICEVISION_LOG_DEBUG("SequentialSfM -- rejectedViewIds: " << rejectedViewIds.size());
    ALICEVISION_LOG_DEBUG("SequentialSfM -- _set_remainingViewId: " << _set_remainingViewId.size());
    
    ++nbRejectedLoops;
    // Retry to perform the resectioning of all the rejected views,
    // as long as new views are successfully added.
  } while( !reconstructedViewIds.empty() && !_set_remainingViewId.empty() );
  
  // timer for stats
  const double time_sfm = timer_sfm.elapsed();
  
  //-- Reconstruction done.
  //-- Display some statistics
  ALICEVISION_LOG_INFO(
    "-------------------------------\n"
    "-- Structure from Motion (statistics):\n"
    "-- #Camera calibrated: " << _sfm_data.GetPoses().size() <<
    " from " << _sfm_data.GetViews().size() << " input images.\n"
    "-- #Tracks, #3D points: " << _sfm_data.GetLandmarks().size() << "\n"
    "-------------------------------");

  Histogram<double> h;
  computeResidualsHistogram(&h);
  ALICEVISION_LOG_INFO("Histogram of residuals:" << h.ToString());
  
  Histogram<double> hTracks;
  computeTracksLengthsHistogram(&hTracks);
  ALICEVISION_LOG_INFO("Histogram of tracks length:" << hTracks.ToString());
  
  if (!_sLoggingFile.empty())
  {
    using namespace htmlDocument;
    std::ostringstream os;
    os << "Structure from Motion process finished.";
    _htmlDocStream->pushInfo("<hr>");
    _htmlDocStream->pushInfo(htmlMarkup("h3",os.str()));

    os.str("");
    os<< "-- Structure from Motion (statistics):<br>"
      << "-- # Camera calibrated: " << _sfm_data.GetPoses().size()
      << " from " <<_sfm_data.GetViews().size() << " input images.<br>"
      << "-- # Tracks, #3D points: " << _sfm_data.GetLandmarks().size() << "<br>";

    _htmlDocStream->pushInfo(os.str());
    
    _htmlDocStream->pushInfo(htmlMarkup("h2","Histogram of reprojection-residuals"));
    
    
    const std::vector<double> xBin = h.GetXbinsValue();
    htmlDocument::JSXGraphWrapper jsxGraph;
    _htmlDocStream->pushXYChart(xBin, h.GetHist(),"3DtoImageResiduals");
    
    const std::vector<double> xBinTracks = hTracks.GetXbinsValue();
    htmlDocument::JSXGraphWrapper jsxGraphTracks;
    _htmlDocStream->pushXYChart(xBinTracks, hTracks.GetHist(),"3DtoTracksSize");
  }
  exportStatistics(time_sfm);

  return true;
}

/// Select a candidate initial pair
bool ReconstructionEngine_sequentialSfM::chooseInitialPair(Pair & initialPairIndex) const
{
  if (_userInitialImagePair != Pair(0,0))
  {
    // Internal initial pair is already initialized (so return it)
    initialPairIndex = _userInitialImagePair;
  }
  else
  {
    // List Views that supports valid intrinsic
    std::set<IndexT> valid_views;
    for (Views::const_iterator it = _sfm_data.GetViews().begin();
         it != _sfm_data.GetViews().end(); ++it)
    {
      const View * v = it->second.get();
      if( _sfm_data.GetIntrinsics().find(v->getIntrinsicId()) != _sfm_data.GetIntrinsics().end())
        valid_views.insert(v->getViewId());
    }
    
    if (_sfm_data.GetIntrinsics().empty() || valid_views.empty())
    {
      ALICEVISION_CERR("There is no defined intrinsic data in order to compute an essential matrix for the initial pair.");
      return false;
    }

    ALICEVISION_LOG_DEBUG(
      "----------------------------------------------------\n"
      "ReconstructionEngine_sequentialSfM::ChooseInitialPair\n"
      "----------------------------------------------------\n"
      " Pairs that have valid intrinsic and high support of points are displayed:\n"
      " Choose one pair manually by typing the two integer indexes\n"
      "----------------------------------------------------"
      );

    // Try to list the 10 top pairs that have:
    //  - valid intrinsics,
    //  - valid estimated Fundamental matrix.
    std::vector< size_t > vec_NbMatchesPerPair;
    std::vector<aliceVision::matching::PairwiseMatches::const_iterator> vec_MatchesIterator;
    for (aliceVision::matching::PairwiseMatches::const_iterator
      iter = _pairwiseMatches->begin();
      iter != _pairwiseMatches->end(); ++iter)
    {
      const Pair current_pair = iter->first;
      if (valid_views.count(current_pair.first) &&
          valid_views.count(current_pair.second) )
      {
        vec_NbMatchesPerPair.push_back(iter->second.size());
        vec_MatchesIterator.push_back(iter);
      }
    }
    // sort the Pairs in descending order according their correspondences count
    using namespace stl::indexed_sort;
    std::vector< sort_index_packet_descend< size_t, size_t> > packet_vec(vec_NbMatchesPerPair.size());
    sort_index_helper(packet_vec, &vec_NbMatchesPerPair[0], std::min((size_t)10, vec_NbMatchesPerPair.size()));
    
    for (size_t i = 0; i < std::min((size_t)10, vec_NbMatchesPerPair.size()); ++i) {
      const size_t index = packet_vec[i].index;
      aliceVision::matching::PairwiseMatches::const_iterator iter = vec_MatchesIterator[index];
      ALICEVISION_COUT("(" << iter->first.first << "," << iter->first.second <<")\t\t"
        << iter->second.getNbAllMatches() << " matches");
    }
    
    // Ask the user to choose an initial pair (by set some view ids)
    ALICEVISION_COUT(std::endl << " type INITIAL pair ids: X enter Y enter");
    int val, val2;
    if ( std::cin >> val && std::cin >> val2) {
      initialPairIndex.first = val;
      initialPairIndex.second = val2;
    }
  }

  ALICEVISION_LOG_DEBUG("\nPutative starting pair is: (" << initialPairIndex.first
      << "," << initialPairIndex.second << ")");

  // Check validity of the initial pair indices:
  if (!_featuresPerView->viewExist(initialPairIndex.first)  ||
      !_featuresPerView->viewExist(initialPairIndex.second))
  {
    ALICEVISION_LOG_WARNING("At least one of the initial pair indices is invalid.");
    return false;
  }
  return true;
}

bool ReconstructionEngine_sequentialSfM::initLandmarkTracks()
{
  // Compute tracks from matches
  track::TracksBuilder tracksBuilder;

  {
    // List of features matches for each couple of images
    const aliceVision::matching::PairwiseMatches & map_Matches = *_pairwiseMatches;
    ALICEVISION_LOG_DEBUG("Track building");

    tracksBuilder.Build(map_Matches);
    ALICEVISION_LOG_DEBUG("Track filtering");
    tracksBuilder.Filter(_minInputTrackLength);

    ALICEVISION_LOG_DEBUG("Track export to internal struct");
    //-- Build tracks with STL compliant type :
    tracksBuilder.ExportToSTL(_map_tracks);
    ALICEVISION_LOG_DEBUG("Build tracks per view");
    track::TracksUtilsMap::computeTracksPerView(_map_tracks, _map_tracksPerView);
    
    ALICEVISION_LOG_DEBUG("Build tracks pyramid per view");
    computeTracksPyramidPerView(
            _map_tracksPerView, _map_tracks, _sfm_data.views, *_featuresPerView, _pyramidBase, _pyramidDepth, _map_featsPyramidPerView);

    {
      //-- Display stats :
      //    - number of images
      //    - number of tracks
      std::set<size_t> set_imagesId;
      track::TracksUtilsMap::ImageIdInTracks(_map_tracksPerView, set_imagesId);
      ALICEVISION_LOG_INFO("Number of tracks: " << tracksBuilder.NbTracks());
      ALICEVISION_LOG_INFO("Number of images in tracks: " << set_imagesId.size());

      std::map<size_t, size_t> map_Occurence_TrackLength;
      track::TracksUtilsMap::TracksLength(_map_tracks, map_Occurence_TrackLength);
      ALICEVISION_LOG_INFO("TrackLength, Occurrence");
      for(const auto& iter: map_Occurence_TrackLength)
      {
        ALICEVISION_LOG_INFO("\t" << iter.first << "\t" << iter.second);
        // Add input tracks histogram
        _tree.add("sfm.inputtracks_histogram."
          + std::to_string(iter.first), iter.second);
      }
    }
  }
  return _map_tracks.size() > 0;
}

bool ReconstructionEngine_sequentialSfM::getBestInitialImagePairs(std::vector<Pair>& out_bestImagePairs) const
{
  // From the k view pairs with the highest number of verified matches
  // select a pair that have the largest baseline (mean angle between its bearing vectors).
  
  const unsigned iMin_inliers_count = 100;
  // Use a min angle limit to ensure quality of the geometric evaluation.
  const float fRequired_min_angle = 5.0f;
  // Use a max angle limit to ensure good matching quality.
  const float fLimit_max_angle = 40.0f;
  
  // List Views that support valid intrinsic (view that could be used for Essential matrix computation)
  std::set<IndexT> valid_views;
  for(const auto& it : _sfm_data.GetViews())
  {
    
    const View * v = it.second.get();
    if (_sfm_data.GetIntrinsics().count(v->getIntrinsicId()) &&
        _sfm_data.GetIntrinsics().at(v->getIntrinsicId())->isValid())
      valid_views.insert(v->getViewId());
  }
  
  if (valid_views.size() < 2)
  {
    ALICEVISION_LOG_WARNING("Failed to find an initial pair automatically. There is no view with valid intrinsics.");
    return false;
  }
  
  /// ImagePairScore contains <imagePairScore*scoring_angle, imagePairScore, scoring_angle, numberOfInliers, imagePair>
  typedef std::tuple<double, double, double, std::size_t, Pair> ImagePairScore;
  std::vector<ImagePairScore> bestImagePairs;
  bestImagePairs.reserve(_pairwiseMatches->size());
  
  // Compute the relative pose & the 'baseline score'
  boost::progress_display my_progress_bar( _pairwiseMatches->size(),
    std::cout,
    "Automatic selection of an initial pair:\n" );

  #pragma omp parallel for schedule(dynamic)

  for (int i = 0; i < _pairwiseMatches->size(); ++i)
  {
    matching::PairwiseMatches::const_iterator iter = _pairwiseMatches->begin();
    std::advance(iter, i);
    
#pragma omp critical
    ++my_progress_bar;
    
    const Pair current_pair = iter->first;

    const size_t I = std::min(current_pair.first, current_pair.second);
    const size_t J = std::max(current_pair.first, current_pair.second);

    if (!valid_views.count(I) || !valid_views.count(J))
      continue;
    
    const View * view_I = _sfm_data.GetViews().at(I).get();
    const Intrinsics::const_iterator iterIntrinsic_I = _sfm_data.GetIntrinsics().find(view_I->getIntrinsicId());
    const View * view_J = _sfm_data.GetViews().at(J).get();
    const Intrinsics::const_iterator iterIntrinsic_J = _sfm_data.GetIntrinsics().find(view_J->getIntrinsicId());

    const Pinhole * cam_I = dynamic_cast<const Pinhole*>(iterIntrinsic_I->second.get());
    const Pinhole * cam_J = dynamic_cast<const Pinhole*>(iterIntrinsic_J->second.get());
    if (cam_I == nullptr || cam_J == nullptr)
      continue;

    aliceVision::track::TracksMap map_tracksCommon;
    const std::set<size_t> set_imageIndex= {I, J};
    track::TracksUtilsMap::GetCommonTracksInImagesFast(set_imageIndex, _map_tracks, _map_tracksPerView, map_tracksCommon);

    // Copy points correspondences to arrays for relative pose estimation
    const size_t n = map_tracksCommon.size();
    ALICEVISION_LOG_INFO("AutomaticInitialPairChoice, test I: " << I << ", J: " << J << ", nbCommonTracks: " << n);
    Mat xI(2,n), xJ(2,n);
    size_t cptIndex = 0;
    std::vector<std::size_t> commonTracksIds(n);
    for (aliceVision::track::TracksMap::const_iterator
      iterT = map_tracksCommon.begin(); iterT != map_tracksCommon.end();
      ++iterT, ++cptIndex)
    {
      auto iter = iterT->second.featPerView.begin();
      const size_t i = iter->second;
      const size_t j = (++iter)->second;
      commonTracksIds[cptIndex] = iterT->first;
      
      const auto& viewI = _featuresPerView->getFeatures(I, iterT->second.descType); 
      const auto& viewJ = _featuresPerView->getFeatures(J, iterT->second.descType);
      
      Vec2 feat = viewI[i].coords().cast<double>();
      xI.col(cptIndex) = cam_I->get_ud_pixel(feat);
      feat = viewJ[j].coords().cast<double>();
      xJ.col(cptIndex) = cam_J->get_ud_pixel(feat);
    }
    
    // Robust estimation of the relative pose
    RelativePoseInfo relativePose_info;
    relativePose_info.initial_residual_tolerance = Square(4.0);
    
    const bool relativePoseSuccess = robustRelativePose(
          cam_I->K(), cam_J->K(),
          xI, xJ, relativePose_info,
          std::make_pair(cam_I->w(), cam_I->h()), std::make_pair(cam_J->w(), cam_J->h()),
          1024);
    
    if (relativePoseSuccess && relativePose_info.vec_inliers.size() > iMin_inliers_count)
    {
      // Triangulate inliers & compute angle between bearing vectors
      std::vector<float> vec_angles(relativePose_info.vec_inliers.size());
      std::vector<std::size_t> validCommonTracksIds(relativePose_info.vec_inliers.size());
      const Pose3 pose_I = Pose3(Mat3::Identity(), Vec3::Zero());
      const Pose3 pose_J = relativePose_info.relativePose;
      const Mat34 PI = cam_I->get_projective_equivalent(pose_I);
      const Mat34 PJ = cam_J->get_projective_equivalent(pose_J);
      std::size_t i = 0;
      for (const size_t inlier_idx: relativePose_info.vec_inliers)
      {
        Vec3 X;
        TriangulateDLT(PI, xI.col(inlier_idx), PJ, xJ.col(inlier_idx), &X);
        IndexT trackId = commonTracksIds[inlier_idx];
        auto iter = map_tracksCommon[trackId].featPerView.begin();
        const Vec2 featI = _featuresPerView->getFeatures(I, map_tracksCommon[trackId].descType)[iter->second].coords().cast<double>();
        const Vec2 featJ = _featuresPerView->getFeatures(J, map_tracksCommon[trackId].descType)[(++iter)->second].coords().cast<double>();
        vec_angles[i] = AngleBetweenRay(pose_I, cam_I, pose_J, cam_J, featI, featJ);
        validCommonTracksIds[i] = trackId;
        ++i;
      }
      // Compute the median triangulation angle
      const unsigned median_index = vec_angles.size() / 2;
      std::nth_element(
            vec_angles.begin(),
            vec_angles.begin() + median_index,
            vec_angles.end());
      const float scoring_angle = vec_angles[median_index];
      const double imagePairScore = std::min(computeImageScore(I, validCommonTracksIds), computeImageScore(J, validCommonTracksIds));
      double score = scoring_angle * imagePairScore;

      // If the image pair is outside the reasonable angle range: [fRequired_min_angle;fLimit_max_angle]
      // we put it in negative to ensure that image pairs with reasonable angle will win,
      // but keep the score ordering.
      if (scoring_angle < fRequired_min_angle ||
          scoring_angle > fLimit_max_angle)
        score = - 1.0 / score;

      #pragma omp critical
      bestImagePairs.emplace_back(score, imagePairScore, scoring_angle, relativePose_info.vec_inliers.size(), current_pair);
    }
  }
  // We print the N best scores and return the best one.
  const std::size_t nBestScores = std::min(std::size_t(50), bestImagePairs.size());
  std::sort(bestImagePairs.begin(), bestImagePairs.end(), std::greater<ImagePairScore>());
  ALICEVISION_LOG_DEBUG(bestImagePairs.size() << " possible image pairs. " << nBestScores << " best possibles image pairs are:");
  ALICEVISION_LOG_DEBUG(boost::format("%=15s | %=15s | %=15s | %=15s | %=15s") % "Pair" % "Score" % "ImagePairScore" % "Angle" % "NbMatches");
  ALICEVISION_LOG_DEBUG(std::string(15*5+3*3, '-'));
  for(std::size_t i = 0; i < nBestScores; ++i)
  {
    const ImagePairScore& s = bestImagePairs[i];
    const Pair& currPair = std::get<4>(s);
    const std::string pairIdx = std::to_string(currPair.first) + ", " + std::to_string(currPair.second);
    ALICEVISION_LOG_DEBUG(boost::format("%=15s | %+15.1f | %+15.1f | %+15.1f | %+15f") % pairIdx % std::get<0>(s) % std::get<1>(s) % std::get<2>(s) % std::get<3>(s));
  }
  if (bestImagePairs.empty())
  {
    ALICEVISION_LOG_ERROR("Error: No valid initial pair found automatically.");
    return false;
  }
  out_bestImagePairs.reserve(bestImagePairs.size());
  for(const auto& imagePair: bestImagePairs)
    out_bestImagePairs.push_back(std::get<4>(imagePair));
  
  return true;
}

/// Compute the initial 3D seed (First camera t=0; R=Id, second estimated by 5 point algorithm)
bool ReconstructionEngine_sequentialSfM::makeInitialPair3D(const Pair& current_pair)
{
  // Compute robust Essential matrix for ImageId [I,J]
  // use min max to have I < J
  const std::size_t I = std::min(current_pair.first, current_pair.second);
  const std::size_t J = std::max(current_pair.first, current_pair.second);

  // a. Assert we have valid pinhole cameras
  const View * viewI = _sfm_data.GetViews().at(I).get();
  const Intrinsics::const_iterator iterIntrinsicI = _sfm_data.GetIntrinsics().find(viewI->getIntrinsicId());
  const View * viewJ = _sfm_data.GetViews().at(J).get();
  const Intrinsics::const_iterator iterIntrinsicJ = _sfm_data.GetIntrinsics().find(viewJ->getIntrinsicId());

  ALICEVISION_LOG_INFO("Initial pair is:\n"
          << "  A - Id: " << I << " - " << " filepath: " << viewI->getImagePath() << "\n"
          << "  B - Id: " << J << " - " << " filepath: " << viewJ->getImagePath());

  if (iterIntrinsicI == _sfm_data.GetIntrinsics().end() ||
      iterIntrinsicJ == _sfm_data.GetIntrinsics().end() )
  {
    ALICEVISION_LOG_WARNING("Can't find initial image pair intrinsics: " << viewI->getIntrinsicId() << ", "  << viewJ->getIntrinsicId());
    return false;
  }

  const Pinhole * camI = dynamic_cast<const Pinhole*>(iterIntrinsicI->second.get());
  const Pinhole * camJ = dynamic_cast<const Pinhole*>(iterIntrinsicJ->second.get());

  if (camI == nullptr || camJ == nullptr || !camI->isValid() || !camJ->isValid())
  {
    ALICEVISION_LOG_WARNING("Can't find initial image pair intrinsics (NULL ptr): " << viewI->getIntrinsicId() << ", "  << viewJ->getIntrinsicId());
    return false;
  }
  
  // b. Get common features between the two views
  // use the track to have a more dense match correspondence set
  aliceVision::track::TracksMap map_tracksCommon;
  const std::set<std::size_t> set_imageIndex= {I, J};
  track::TracksUtilsMap::GetCommonTracksInImagesFast(set_imageIndex, _map_tracks, _map_tracksPerView, map_tracksCommon);
  
  //-- Copy point to arrays
  const std::size_t n = map_tracksCommon.size();
  Mat xI(2,n), xJ(2,n);
  std::size_t cptIndex = 0;
  for (aliceVision::track::TracksMap::const_iterator
    iterT = map_tracksCommon.begin(); iterT != map_tracksCommon.end();
    ++iterT, ++cptIndex)
  {
    assert(iterT->second.featPerView.size() == 2);
    auto iter = iterT->second.featPerView.begin();
    const std::size_t i = iter->second;
    const std::size_t j = (++iter)->second;
    
    Vec2 feat = _featuresPerView->getFeatures(I, iterT->second.descType)[i].coords().cast<double>();
    xI.col(cptIndex) = camI->get_ud_pixel(feat);
    feat = _featuresPerView->getFeatures(J, iterT->second.descType)[j].coords().cast<double>();
    xJ.col(cptIndex) = camJ->get_ud_pixel(feat);
  }
  ALICEVISION_LOG_INFO(n << " matches in the image pair for the initial pose estimation.");

  // c. Robust estimation of the relative pose
  RelativePoseInfo relativePose_info;

  const std::pair<std::size_t, std::size_t> imageSizeI(camI->w(), camI->h());
  const std::pair<std::size_t, std::size_t> imageSizeJ(camJ->w(), camJ->h());
  
  if (!robustRelativePose(
        camI->K(), camJ->K(), xI, xJ, relativePose_info, imageSizeI, imageSizeJ, 4096))
  {
    ALICEVISION_LOG_WARNING(" /!\\ Robust estimation failed to compute E for this pair");
    return false;
  }
  ALICEVISION_LOG_DEBUG("A-Contrario initial pair residual: "
    << relativePose_info.found_residual_precision);

  // Bound min precision at 1 pix.
  relativePose_info.found_residual_precision = std::max(relativePose_info.found_residual_precision, 1.0);
  
  {
    // Init poses
    const Pose3& initPoseI = Pose3(Mat3::Identity(), Vec3::Zero());
    const Pose3& initPoseJ = relativePose_info.relativePose;
    
    _sfm_data.setPose(*viewI, initPoseI);
    _sfm_data.setPose(*viewJ, initPoseJ);
    
    // Triangulate
    const std::set<IndexT> prevImageIndex = {static_cast<IndexT>(I)};
    const std::set<IndexT> newImageIndex = {static_cast<IndexT>(J)};
    
    triangulate(_sfm_data, prevImageIndex, newImageIndex);
    
    Save(_sfm_data, stlplus::create_filespec(_sOutDirectory, "initialPair", _sfmdataInterFileExtension), _sfmdataInterFilter);
    
    /*
    // - refine only Structure and Rotations & translations (keep intrinsic constant)
    BundleAdjustmentCeres::BA_options options(true);
    options.setDenseBA();
    BundleAdjustmentCeres bundle_adjustment_obj(options);
    if (!bundle_adjustment_obj.Adjust(_sfm_data, BA_REFINE_ROTATION | BA_REFINE_TRANSLATION | BA_REFINE_STRUCTURE))
    {
      ALICEVISION_LOG_WARNING("BA of initial pair " << current_pair.first << ", " << current_pair.second << " failed.");

      // Clear poses, RIGs, landmarks
      _sfm_data.GetPoses().clear();
      _sfm_data.GetLandmarks().clear();
      _sfm_data.resetRigs();
      
      return false;
    }
    */
    std::size_t bundleAdjustmentIteration = 0;
    const std::size_t nbOutliersThreshold = 0;
    
    do
    {
      auto chrono2_start = std::chrono::steady_clock::now();
      bool baStatus = BundleAdjustment(true);
      if(baStatus == false)
      {
          ALICEVISION_LOG_WARNING("BA of initial pair " << current_pair.first << ", " << current_pair.second << " failed.");

          // Clear poses, RIGs, landmarks
          _sfm_data.GetPoses().clear();
          _sfm_data.GetLandmarks().clear();
          _sfm_data.resetRigs();

          return false;
      }
      ALICEVISION_LOG_DEBUG("Initial Pair, bundle iteration: " << bundleAdjustmentIteration
           << " took " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - chrono2_start).count() << " msec.");
      ++bundleAdjustmentIteration;
    }
    while (badTrackRejector(4.0, nbOutliersThreshold));
    
    
    Save(_sfm_data, stlplus::create_filespec(_sOutDirectory, "initialPair_afterBA", _sfmdataInterFileExtension), _sfmdataInterFilter);
    
    // Save outlier residual information
    Histogram<double> histoResiduals;
    ALICEVISION_LOG_DEBUG(
      "=========================\n"
      " MSE Residual InitialPair Inlier: " << computeResidualsHistogram(&histoResiduals) << "\n"
      "=========================");

    if (!_sLoggingFile.empty())
    {
      using namespace htmlDocument;
      _htmlDocStream->pushInfo(htmlMarkup("h3","Essential Matrix."));
      std::ostringstream os;
      os << std::endl
        << "<b>Robust Essential matrix:</b>" << "<br>"
        << "-> View I:<br>id: " << I << "<br>image path: " << viewI->getImagePath() << "<br>"
        << "-> View J:<br>id: " << J << "<br>image path: " << viewJ->getImagePath() << "<br><br>"
        << "- Threshold: " << relativePose_info.found_residual_precision << "<br>"
        << "- Resection status: " << "OK" << "<br>"
        << "- # points used for robust Essential matrix estimation: "
        << xI.cols() << "<br>"
        << "- # points validated by robust estimation: "
        << _sfm_data.structure.size() << "<br>"
        << "- % points validated: "
        << _sfm_data.structure.size()/static_cast<float>(xI.cols())
        << "<br>";
      _htmlDocStream->pushInfo(os.str());

      _htmlDocStream->pushInfo(htmlMarkup("h3",
        "Initial triangulation - Residual of the robust estimation.<br>Thresholded at: "
        + toString(relativePose_info.found_residual_precision)));

      _htmlDocStream->pushInfo(htmlMarkup("h3","Histogram of residuals"));

      std::vector<double> xBin = histoResiduals.GetXbinsValue();
      std::pair< std::pair<double,double>, std::pair<double,double> > range =
          autoJSXGraphViewport<double>(xBin, histoResiduals.GetHist());
      
      htmlDocument::JSXGraphWrapper jsxGraph;
      jsxGraph.init("InitialPairTriangulationKeptInfo",600,300);
      jsxGraph.addXYChart(xBin, histoResiduals.GetHist(), "line,point");
      jsxGraph.addLine(relativePose_info.found_residual_precision, 0,
                       relativePose_info.found_residual_precision, histoResiduals.GetHist().front());
      jsxGraph.UnsuspendUpdate();
      jsxGraph.setViewport(range);
      jsxGraph.close();
      
      _htmlDocStream->pushInfo(jsxGraph.toStr());
      _htmlDocStream->pushInfo("<hr>");

      std::ofstream htmlFileStream( std::string(stlplus::folder_append_separator(_sOutDirectory) + _sLoggingFile).c_str());
      htmlFileStream << _htmlDocStream->getDoc();
    }
  }
  
  Save(_sfm_data, stlplus::create_filespec(_sOutDirectory, "initialPair_sfmData", _sfmdataInterFileExtension), _sfmdataInterFilter);
  
  return !_sfm_data.structure.empty();
}

double ReconstructionEngine_sequentialSfM::computeResidualsHistogram(Histogram<double> * histo) const
{
  if (_sfm_data.GetLandmarks().empty())
    return -1.0;
  
  // Collect residuals for each observation
  std::vector<float> vec_residuals;
  vec_residuals.reserve(_sfm_data.structure.size());
  for(const auto &track : _sfm_data.GetLandmarks())
  {
    const Observations & observations = track.second.observations;
    for(const auto& obs: observations)
    {
      const View * view = _sfm_data.GetViews().find(obs.first)->second.get();
      const Pose3 pose = _sfm_data.getPose(*view);
      const std::shared_ptr<IntrinsicBase> intrinsic = _sfm_data.GetIntrinsics().find(view->getIntrinsicId())->second;
      const Vec2 residual = intrinsic->residual(pose, track.second.X, obs.second.x);
      vec_residuals.push_back( fabs(residual(0)) );
      vec_residuals.push_back( fabs(residual(1)) );
    }
  }
  
  assert(!vec_residuals.empty());

  MinMaxMeanMedian<float> stats(vec_residuals.begin(), vec_residuals.end());
  
  if (histo)  {
    *histo = Histogram<double>(stats.min, stats.max, 10);
    histo->Add(vec_residuals.begin(), vec_residuals.end());
  }

  ALICEVISION_LOG_DEBUG(
    "\nReconstructionEngine_sequentialSfM::ComputeResidualsMSE.\n"
    "\t-- #Tracks:\t" << _sfm_data.GetLandmarks().size() << "\n"
    "\t-- Residual min:\t" << stats.min << "\n"
    "\t-- Residual median:\t" << stats.median << "\n"
    "\t-- Residual max:\t "  << stats.max << "\n"
    "\t-- Residual mean:\t " << stats.mean);

  return stats.mean;
}

double ReconstructionEngine_sequentialSfM::computeTracksLengthsHistogram(Histogram<double> * histo) const
{
  if (_sfm_data.GetLandmarks().empty())
    return -1.0;
  
  // Collect tracks size: number of 2D observations per 3D points
  std::vector<float> vec_nbTracks;
  vec_nbTracks.reserve(_sfm_data.GetLandmarks().size());
  
  for(const auto &track : _sfm_data.GetLandmarks())
  {
    const Observations & observations = track.second.observations;
    vec_nbTracks.push_back(observations.size());
  }
  
  assert(!vec_nbTracks.empty());

  MinMaxMeanMedian<float> stats(vec_nbTracks.begin(), vec_nbTracks.end());

  if (histo)
  {
    *histo = Histogram<double>(stats.min, stats.max + 1, stats.max - stats.min + 1);
    histo->Add(vec_nbTracks.begin(), vec_nbTracks.end());
  }

  ALICEVISION_LOG_INFO("Tracks: " << _sfm_data.GetLandmarks().size());
  ALICEVISION_LOG_INFO("Tracks Length min: " << stats.min);
  ALICEVISION_LOG_INFO("Tracks Length median: " << stats.median);
  ALICEVISION_LOG_INFO("Tracks Length max: "  << stats.max);
  ALICEVISION_LOG_INFO("Tracks Length mean: " << stats.mean);

  return stats.mean;
}

std::size_t ReconstructionEngine_sequentialSfM::computeImageScore(std::size_t viewId, const std::vector<std::size_t>& trackIds) const
{
#ifdef ALICEVISION_NEXTBESTVIEW_WITHOUT_SCORE
  return trackIds.size();
#else
  std::size_t score = 0;
  // The number of cells of the pyramid grid represent the score
  // and ensure a proper repartition of features in images.
  const auto& featsPyramid = _map_featsPyramidPerView.at(viewId);
  for(std::size_t level = 0; level < _pyramidDepth; ++level)
  {
    std::set<std::size_t> featIndexes; // Set of grid cell indexes in the pyramid
    for(std::size_t trackId: trackIds)
    {
      std::size_t pyramidIndex = featsPyramid.at(trackId * _pyramidDepth + level);
      featIndexes.insert(pyramidIndex);
    }
    score += featIndexes.size() * _pyramidWeights[level];
  }
  return score;
#endif
}

bool ReconstructionEngine_sequentialSfM::findConnectedViews(
  std::vector<ViewConnectionScore>& out_connectedViews,
  const std::set<size_t>& remainingViewIds) const
{
  out_connectedViews.clear();
  
  if (remainingViewIds.empty() || _sfm_data.GetLandmarks().empty())
    return false;
  
  // Collect tracksIds
  std::set<size_t> reconstructed_trackId;
  std::transform(_sfm_data.GetLandmarks().begin(), _sfm_data.GetLandmarks().end(),
                 std::inserter(reconstructed_trackId, reconstructed_trackId.begin()),
                 stl::RetrieveKey());
  
  const std::set<IndexT> reconstructedIntrinsics = _sfm_data.getReconstructedIntrinsics();
  
#pragma omp parallel for
  for (int i = 0; i < remainingViewIds.size(); ++i)
  {
    std::set<size_t>::const_iterator iter = remainingViewIds.cbegin();
    std::advance(iter, i);
    const size_t viewId = *iter;
    const size_t intrinsicId = _sfm_data.GetViews().at(viewId)->getIntrinsicId();
    const bool isIntrinsicsReconstructed = reconstructedIntrinsics.count(intrinsicId);
    
    // Compute 2D - 3D possible content
    aliceVision::track::TracksPerView::const_iterator tracksIdsIt = _map_tracksPerView.find(viewId);
    if(tracksIdsIt == _map_tracksPerView.end())
      continue;

    const aliceVision::track::TrackIdSet& set_tracksIds = tracksIdsIt->second;
    if (set_tracksIds.empty())
      continue;
    
    // Check if the view is part of a rig
    {
      const View& view = *_sfm_data.views.at(viewId);
      
      if(view.isPartOfRig())
      {
        // Some views can become indirectly localized when the sub-pose becomes defined
        if(_sfm_data.IsPoseAndIntrinsicDefined(view.getViewId()))
        {
          continue;
        }
        
        // We cannot localize a view if it is part of an initialized RIG with unknown Rig Pose
        const bool knownPose = _sfm_data.existsPose(view);
        const Rig& rig = _sfm_data.getRig(view);
        const RigSubPose& subpose = rig.getSubPose(view.getSubPoseId());
        
        if(rig.isInitialized() &&
           !knownPose &&
           (subpose.status == ERigSubPoseStatus::UNINITIALIZED))
        {
          continue;
        }
      }
    }
    
    // Count the common possible putative point
    //  with the already 3D reconstructed trackId
    std::vector<size_t> vec_trackIdForResection;
    vec_trackIdForResection.reserve(set_tracksIds.size());
    std::set_intersection(set_tracksIds.begin(), set_tracksIds.end(),
                          reconstructed_trackId.begin(),
                          reconstructed_trackId.end(),
                          std::back_inserter(vec_trackIdForResection));
    // Compute an image score based on the number of matches to the 3D scene
    // and the repartition of these features in the image.
    std::size_t score = computeImageScore(viewId, vec_trackIdForResection);
#pragma omp critical
    {
      out_connectedViews.emplace_back(viewId, vec_trackIdForResection.size(), score, isIntrinsicsReconstructed);
    }
  }
  
  // Sort by the image score
  std::sort(out_connectedViews.begin(), out_connectedViews.end(),
            [](const ViewConnectionScore& t1, const ViewConnectionScore& t2) {
    return std::get<2>(t1) > std::get<2>(t2);
  });
  return !out_connectedViews.empty();
}

bool ReconstructionEngine_sequentialSfM::findNextImagesGroupForResection(
  std::vector<size_t> & out_selectedViewIds,
  const std::set<size_t>& remainingViewIds) const
{
  out_selectedViewIds.clear();
  auto chrono_start = std::chrono::steady_clock::now();
  std::vector<ViewConnectionScore> vec_viewsScore;
  if(!findConnectedViews(vec_viewsScore, remainingViewIds))
  {
    ALICEVISION_LOG_DEBUG("FindConnectedViews does not find connected new views ");
    return false;
  }
  
  // Impose a minimal number of points to ensure that it makes sense to try the pose estimation.
  static const std::size_t minPointsThreshold = 30;

  ALICEVISION_LOG_DEBUG("FindNextImagesGroupForResection -- Scores (features): ");
  // print the 30 best scores
  for(std::size_t i = 0; i < vec_viewsScore.size() && i < 30; ++i)
  {
    ALICEVISION_LOG_DEBUG_OBJ << std::get<2>(vec_viewsScore[i]) << "(" << std::get<1>(vec_viewsScore[i]) << "), ";
  }
  ALICEVISION_LOG_DEBUG_OBJ << std::endl;

  // If the list is empty or if the list contains images with no correspondences
  // -> (no resection will be possible)
  if (vec_viewsScore.empty() || std::get<1>(vec_viewsScore[0]) == 0)
  {
    ALICEVISION_LOG_DEBUG("FindNextImagesGroupForResection failed:");
    if(vec_viewsScore.empty())
    {
      ALICEVISION_LOG_DEBUG("No putative image.");
    }
    else
    {
      ALICEVISION_LOG_DEBUG_OBJ << "Not enough point in the putative images: ";
      for(auto v: vec_viewsScore)
        ALICEVISION_LOG_DEBUG_OBJ << std::get<1>(v) << ", ";
    }
    ALICEVISION_LOG_DEBUG_OBJ << std::endl;
    // All remaining images cannot be used for pose estimation
    return false;
  }
  
  // Add the image view index with the best score
  out_selectedViewIds.push_back(std::get<0>(vec_viewsScore[0]));
  
  #ifdef ALICEVISION_NEXTBESTVIEW_WITHOUT_SCORE
    static const float dThresholdGroup = 0.75f;
    // Number of 2D-3D correspondences for the best view.
    const IndexT bestScore = std::get<2>(vec_viewsScore[0]);
    // Add all the image view indexes that have at least N% of the score of the best image.
    const size_t scoreThreshold = dThresholdGroup * bestScore;
  #else
    const size_t scoreThreshold = _pyramidThreshold;
  #endif
    
  for (size_t i = 1;
       i < vec_viewsScore.size() &&
       std::get<1>(vec_viewsScore[i]) > minPointsThreshold && // ensure min number of points
       std::get<2>(vec_viewsScore[i]) > scoreThreshold; // ensure score level
       ++i)
  {
    out_selectedViewIds.push_back(std::get<0>(vec_viewsScore[i]));
    if(!std::get<3>(vec_viewsScore[i]))
    {
      // If we add a new intrinsic, it is a sensitive stage in the process,
      // so it is better to perform a Bundle Adjustment just after.
      break;
    }
  }
  ALICEVISION_LOG_DEBUG(
    "FindNextImagesGroupForResection with " << out_selectedViewIds.size() << " images took: " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - chrono_start).count() << " msec\n"
    " - Scores: " << std::get<2>(vec_viewsScore.front()) << " to " << std::get<2>(vec_viewsScore[out_selectedViewIds.size()-1]) << " (threshold was " << scoreThreshold << ")\n"
    " - Features: " << std::get<1>(vec_viewsScore.front()) << " to " << std::get<1>(vec_viewsScore[out_selectedViewIds.size()-1]) << " (threshold was " << minPointsThreshold << ")");
  return true;
}

/**
 * @brief Add one image to the 3D reconstruction. To the resectioning of
 * the camera.
 * @param[in] viewIndex: image index to add to the reconstruction.
 * @param[out] resectionData: contains the camera pose and all data used during the resection.
 *
 * A. Compute 2D/3D matches
 * B. Look if intrinsic data is known or not
 * C. Do the resectioning: compute the camera pose.
 * D. Refine the pose of the found camera
 */
bool ReconstructionEngine_sequentialSfM::computeResection(const std::size_t viewIndex, ResectionData & resectionData)
{
  using namespace track;

  // A. Compute 2D/3D matches
  // A1. list tracks ids used by the view
  const aliceVision::track::TrackIdSet& set_tracksIds = _map_tracksPerView.at(viewIndex);

  // A2. intersects the track list with the reconstructed
  std::set<std::size_t> reconstructed_trackId;
  std::transform(_sfm_data.GetLandmarks().begin(), _sfm_data.GetLandmarks().end(),
                 std::inserter(reconstructed_trackId, reconstructed_trackId.begin()),
                 stl::RetrieveKey());
  
  // Get the ids of the already reconstructed tracks
  std::set_intersection(set_tracksIds.begin(), set_tracksIds.end(),
                        reconstructed_trackId.begin(),
                        reconstructed_trackId.end(),
                        std::inserter(resectionData.tracksId, resectionData.tracksId.begin()));
  
  if (resectionData.tracksId.empty())
  {
    // No match. The image has no connection with already reconstructed points.
    ALICEVISION_LOG_DEBUG("Resection failed as there is no connection with already reconstructed points");
    return false;
  }
  
  // Get back featId associated to a tracksID already reconstructed.
  // These 2D/3D associations will be used for the resection.
  TracksUtilsMap::GetFeatureIdInViewPerTrack(_map_tracks,
                                             resectionData.tracksId,
                                             viewIndex,
                                             &resectionData.featuresId);
  
  // Localize the image inside the SfM reconstruction
  resectionData.pt2D.resize(2, resectionData.tracksId.size());
  resectionData.pt3D.resize(3, resectionData.tracksId.size());
  resectionData.vec_descType.resize(resectionData.tracksId.size());
  
  // B. Look if intrinsic data is known or not
  const View * view_I = _sfm_data.GetViews().at(viewIndex).get();
  resectionData.optionalIntrinsic = _sfm_data.GetIntrinsicSharedPtr(view_I->getIntrinsicId());
  
  std::size_t cpt = 0;
  std::set<std::size_t>::const_iterator iterTrackId = resectionData.tracksId.begin();
  for (std::vector<TracksUtilsMap::FeatureId>::const_iterator iterfeatId = resectionData.featuresId.begin();
       iterfeatId != resectionData.featuresId.end();
       ++iterfeatId, ++iterTrackId, ++cpt)
  {
    const feature::EImageDescriberType descType = iterfeatId->first;
    resectionData.pt3D.col(cpt) = _sfm_data.GetLandmarks().at(*iterTrackId).X;
    resectionData.pt2D.col(cpt) = _featuresPerView->getFeatures(viewIndex, descType)[iterfeatId->second].coords().cast<double>();
    resectionData.vec_descType.at(cpt) = descType;
  }
  
  // C. Do the resectioning: compute the camera pose.
  ALICEVISION_LOG_DEBUG(
    "-------------------------------\n"
    "-- Robust Resection of view: " << viewIndex);

  const bool bResection = sfm::SfMLocalizer::Localize(
      Pair(view_I->getWidth(), view_I->getHeight()),
      resectionData.optionalIntrinsic.get(),
      resectionData,
      resectionData.pose, 
      _localizerEstimator
    );

  if (!_sLoggingFile.empty())
  {
    using namespace htmlDocument;
    std::ostringstream os;
    os << "Robust resection of view " << viewIndex << ": <br>";
    _htmlDocStream->pushInfo(htmlMarkup("h4",os.str()));

    os.str("");
    os << std::endl
      << "- Image path: " << view_I->getImagePath() << "<br>"
      << "- Threshold: " << resectionData.error_max << "<br>"
      << "- Resection status: " << (bResection ? "OK" : "FAILED") << "<br>"
      << "- # points used for Resection: " << resectionData.featuresId.size() << "<br>"
      << "- # points validated by robust estimation: " << resectionData.vec_inliers.size() << "<br>"
      << "- % points validated: "
      << resectionData.vec_inliers.size()/static_cast<float>(resectionData.featuresId.size()) << "<br>";

    _htmlDocStream->pushInfo(os.str());
  }
  
  if (!bResection)
  {
    ALICEVISION_LOG_DEBUG("Resection of view " << viewIndex << " failed.");
    return false;
  }
  
  // D. Refine the pose of the found camera.
  // We use a local scene with only the 3D points and the new camera.
  {
    camera::Pinhole * pinhole_cam = dynamic_cast<camera::Pinhole *>(resectionData.optionalIntrinsic.get());
    resectionData.isNewIntrinsic = (resectionData.optionalIntrinsic == nullptr) || (pinhole_cam && !pinhole_cam->isValid());
    // A valid pose has been found (try to refine it):
    // If no valid intrinsic as input:
    //  init a new one from the projection matrix decomposition
    // Else use the existing one and consider it as constant.
    if (resectionData.isNewIntrinsic)
    {
      // setup a default camera model from the found projection matrix
      Mat3 K, R;
      Vec3 t;
      KRt_From_P(resectionData.projection_matrix, &K, &R, &t);
      
      const double focal = (K(0,0) + K(1,1))/2.0;
      const Vec2 principal_point(K(0,2), K(1,2));
      
      if(resectionData.optionalIntrinsic == nullptr)
      {
        // Create the new camera intrinsic group
        resectionData.optionalIntrinsic = createPinholeIntrinsic(_camType, view_I->getWidth(), view_I->getHeight(), focal, principal_point(0), principal_point(1));
        
      }
      else if(pinhole_cam)
      {
        // Fill the uninitialized camera intrinsic group
        pinhole_cam->setK(focal, principal_point(0), principal_point(1));
      }
    }
    const std::set<IndexT> reconstructedIntrinsics = _sfm_data.getReconstructedIntrinsics();
    // If we use a camera intrinsic for the first time we need to refine it.
    const bool intrinsicsFirstUsage = (reconstructedIntrinsics.count(view_I->getIntrinsicId()) == 0);

    if(!sfm::SfMLocalizer::RefinePose(
      resectionData.optionalIntrinsic.get(), resectionData.pose,
      resectionData, true, resectionData.isNewIntrinsic || intrinsicsFirstUsage))
    {
      ALICEVISION_LOG_DEBUG("Resection of view " << viewIndex << " failed during pose refinement.");
      return false;
    }
  }
  return true;
}

void ReconstructionEngine_sequentialSfM::updateScene(const std::size_t viewIndex, const ResectionData & resectionData)
{ 
  // A. Update the global scene with the new found camera pose, intrinsic (if not defined)
  
  // update the view pose or rig pose/sub-pose
  _map_ACThreshold.insert(std::make_pair(viewIndex, resectionData.error_max));
  
  const View& view = *_sfm_data.views.at(viewIndex);
  _sfm_data.setPose(view, resectionData.pose);
  
  if (resectionData.isNewIntrinsic)
  {
    // Since the view have not yet an intrinsic group before, create a new one
    IndexT new_intrinsic_id = 0;
    if (!_sfm_data.GetIntrinsics().empty())
    {
      // Since some intrinsic Id already exists,
      //  we have to create a new unique identifier following the existing one
      std::set<IndexT> existing_intrinsicId;
      std::transform(_sfm_data.GetIntrinsics().begin(), _sfm_data.GetIntrinsics().end(),
                     std::inserter(existing_intrinsicId, existing_intrinsicId.begin()),
                     stl::RetrieveKey());
      new_intrinsic_id = (*existing_intrinsicId.rbegin())+1;
    }
    _sfm_data.views.at(viewIndex).get()->setIntrinsicId(new_intrinsic_id);
    _sfm_data.intrinsics[new_intrinsic_id] = resectionData.optionalIntrinsic;
  }
  
  // B. Update the observations into the global scene structure
  // - Add the new 2D observations to the reconstructed tracks
  std::set<std::size_t>::const_iterator iterTrackId = resectionData.tracksId.begin();
  for (std::size_t i = 0; i < resectionData.pt2D.cols(); ++i, ++iterTrackId)
  {
    const Vec3 X = resectionData.pt3D.col(i);
    const Vec2 x = resectionData.pt2D.col(i);
    const Vec2 residual = resectionData.optionalIntrinsic->residual(resectionData.pose, X, x);
    if (residual.norm() < resectionData.error_max &&
        resectionData.pose.depth(X) > 0)
    {
      // Inlier, add the point to the reconstructed track
      _sfm_data.structure[*iterTrackId].observations[viewIndex] = Observation(x, resectionData.featuresId[i].second);
    }
  }
}

bool ReconstructionEngine_sequentialSfM::checkChieralities(
  const Vec3& pt3D, 
  const std::set<IndexT> & viewsId, 
  const SfMData& scene)
{
  for (const IndexT & viewId : viewsId)
  {
    const View* view = scene.GetViews().at(viewId).get();
    const Pose3 pose = scene.getPose(*view);
    // Check that the point is in front of all the cameras.
    if (pose.depth(pt3D) < 0) 
      return false;
  }
  return true;
}

bool ReconstructionEngine_sequentialSfM::checkAngles(const Vec3 &pt3D, const std::set<IndexT> &viewsId, const SfMData &scene, const double &kMinAngle)
{ 
    for (const std::size_t & viewIdA : viewsId)  
    {
      for (const std::size_t & viewIdB : viewsId)  
      {
        if (viewIdA < viewIdB)
        {
          double angle_deg = AngleBetweenRay(scene.getPose(*scene.GetViews().at(viewIdA).get()), 
                                             scene.getPose(*scene.GetViews().at(viewIdB).get()),
                                             pt3D);
          if (angle_deg >= kMinAngle)
            return true;
        }      
      }
    }
    return false;
}

/* Previous algo. */
//void ReconstructionEngine_sequentialSfM::getTracksToTriangulate(const std::set<IndexT>& previousReconstructedViews, 
//                                                                const std::set<IndexT>& newReconstructedViews, 
//                                                                std::map<IndexT, std::set<IndexT>> & mapTracksToTriangulate)
//{
//  std::set<IndexT> allReconstructedViews;
//  allReconstructedViews.insert(previousReconstructedViews.begin(), previousReconstructedViews.end());
//  allReconstructedViews.insert(newReconstructedViews.begin(), newReconstructedViews.end());
  
//#pragma omp parallel for 
//  for (ptrdiff_t i = 0; i < static_cast<ptrdiff_t>(allReconstructedViews.size()); ++i)
//  {
//    std::set<IndexT>::const_iterator iter = allReconstructedViews.begin();
//    std::advance(iter, i);
//    const IndexT indexAll = *iter;
    
//    for(IndexT indexNew: newReconstructedViews)
//    {
//      if(indexAll == indexNew)
//        continue;
      
//      const std::size_t I = std::min((IndexT)indexNew, indexAll);
//      const std::size_t J = std::max((IndexT)indexNew, indexAll);
//      // Find track correspondences between I and J
//      const std::set<std::size_t> set_viewIndex = { I, J };
//      track::TracksMap map_tracksCommonIJ;
//      track::TracksUtilsMap::GetCommonTracksInImagesFast(set_viewIndex, _map_tracks, _map_tracksPerView, map_tracksCommonIJ);
      
//      if (map_tracksCommonIJ.empty())
//        continue;
        
//      // Collect tracksIds
//      std::set<std::size_t> commonTracksId;
//      std::transform(map_tracksCommonIJ.begin(), map_tracksCommonIJ.end(),
//                     std::inserter(commonTracksId, commonTracksId.begin()),
//                     stl::RetrieveKey());
      
//#pragma omp critical        
//      for (std::size_t trackId : commonTracksId)
//      {
//        mapTracksToTriangulate[trackId].insert(I);      
//        mapTracksToTriangulate[trackId].insert(J);      
//      } // tracks to reconstruct/update
//    } // new reconstructed view
//  } // all recontrusted views
//}

void ReconstructionEngine_sequentialSfM::getTracksToTriangulate(const std::set<IndexT>& previousReconstructedViews, 
                                                                const std::set<IndexT>& newReconstructedViews, 
                                                                std::map<IndexT, std::set<IndexT>> & mapTracksToTriangulate) const
{
  std::set<IndexT> allReconstructedViews;
  allReconstructedViews.insert(previousReconstructedViews.begin(), previousReconstructedViews.end());
  allReconstructedViews.insert(newReconstructedViews.begin(), newReconstructedViews.end());
  
  std::set<IndexT> allTracksInNewViews;
  track::TracksUtilsMap::GetTracksInImagesFast(newReconstructedViews, _map_tracksPerView, allTracksInNewViews);
  
  std::set<IndexT>::iterator it;
#pragma omp parallel private(it)
  {
    for (it = allTracksInNewViews.begin(); it != allTracksInNewViews.end(); ++it)
    {
#pragma omp single nowait
      {
        const std::size_t trackId = *it;
        
        const track::Track & track = _map_tracks.at(trackId);
        
        std::set<IndexT> allViewsSharingTheTrack;
        std::transform(track.featPerView.begin(), track.featPerView.end(),
                       std::inserter(allViewsSharingTheTrack, allViewsSharingTheTrack.begin()),
                       stl::RetrieveKey());
        
        std::set<IndexT> allReconstructedViewsSharingTheTrack;
        std::set_intersection(allViewsSharingTheTrack.begin(), allViewsSharingTheTrack.end(),
                              allReconstructedViews.begin(), allReconstructedViews.end(),
                              std::inserter(allReconstructedViewsSharingTheTrack, allReconstructedViewsSharingTheTrack.begin()));
        
        if (allReconstructedViewsSharingTheTrack.size() >= _minNbObservationsForTriangulation)
        {
#pragma omp critical        
          mapTracksToTriangulate[trackId] = allReconstructedViewsSharingTheTrack;
        }
      }
    }
  }
}

void ReconstructionEngine_sequentialSfM::triangulateMultiViews_LORANSAC(SfMData& scene, const std::set<IndexT>& previousReconstructedViews, const std::set<IndexT>& newReconstructedViews)
{
  ALICEVISION_LOG_DEBUG("Triangulating (mode: multi-view LO-RANSAC)... ");

  // -- Identify the track to triangulate :
  // This map contains all the tracks that will be triangulated (for the first time, or not)
  // These tracks are seen by at least one new reconstructed view.  
  std::map<IndexT, std::set<IndexT>> mapTracksToTriangulate; // <trackId, observations> 
  getTracksToTriangulate(previousReconstructedViews, newReconstructedViews, mapTracksToTriangulate);
  
  std::vector<IndexT> setTracksId; // <trackId>
  std::transform(mapTracksToTriangulate.begin(), mapTracksToTriangulate.end(),
                 std::inserter(setTracksId, setTracksId.begin()),
                 stl::RetrieveKey());
                   
#pragma omp parallel for 
  for (int i = 0; i < setTracksId.size(); i++) // each track (already reconstructed or not)
  {
    const IndexT trackId = setTracksId.at(i);
    bool isValidTrack = true;
    const track::Track & track = _map_tracks.at(trackId);
    std::set<IndexT> & observations = mapTracksToTriangulate.at(trackId); // all the posed views possessing the track
    
    // The track needs to be seen by a min. number of views to be triangulated
    if (observations.size() < _minNbObservationsForTriangulation)
      continue;
    
    Vec3 X_euclidean = Vec3::Zero();
    std::set<IndexT> inliers;
    
    if (observations.size() == 2) 
    {
      /* --------------------------------------------
       *    2 observations : triangulation using DLT
       * -------------------------------------------- */ 
       
      inliers = observations;
      
      // -- Prepare:
      IndexT I =  *(observations.begin());
      IndexT J =  *(observations.rbegin());
      const View* viewI = scene.GetViews().at(I).get();
      const View* viewJ = scene.GetViews().at(J).get();
      const IntrinsicBase* camI = scene.GetIntrinsics().at(viewI->getIntrinsicId()).get();
      const IntrinsicBase* camJ = scene.GetIntrinsics().at(viewJ->getIntrinsicId()).get();
      const Pose3 poseI = scene.getPose(*viewI);
      const Pose3 poseJ = scene.getPose(*viewJ);
      const Vec2 xI = _featuresPerView->getFeatures(I, track.descType)[track.featPerView.at(I)].coords().cast<double>();
      const Vec2 xJ = _featuresPerView->getFeatures(J, track.descType)[track.featPerView.at(J)].coords().cast<double>();
  
      // -- Triangulate:
      TriangulateDLT(camI->get_projective_equivalent(poseI), 
                     camI->get_ud_pixel(xI), 
                     camJ->get_projective_equivalent(poseJ), 
                     camI->get_ud_pixel(xJ), 
                     &X_euclidean);
      
      // -- Check:
      //  - angle (small angle leads imprecise triangulation)
      //  - positive depth
      //  - residual values
      // TODO assert(acThresholdIt != _map_ACThreshold.end());
      const auto& acThresholdItI = _map_ACThreshold.find(I);
      const auto& acThresholdItJ = _map_ACThreshold.find(J);
      const double& acThresholdI = (acThresholdItI != _map_ACThreshold.end()) ? acThresholdItI->second : 4.0;
      const double& acThresholdJ = (acThresholdItJ != _map_ACThreshold.end()) ? acThresholdItJ->second : 4.0;
      
      if (AngleBetweenRay(poseI, camI, poseJ, camJ, xI, xJ) < _minAngleForTriangulation || 
          poseI.depth(X_euclidean) < 0 || 
          poseJ.depth(X_euclidean) < 0 || 
          camI->residual(poseI, X_euclidean, xI).norm() > acThresholdI || 
          camJ->residual(poseJ, X_euclidean, xJ).norm() > acThresholdJ)
        isValidTrack = false;
    }
    else 
    {
      /* -------------------------------------------------------
       *    N obsevations (N>2) : triangulation using LORANSAC 
       * ------------------------------------------------------- */ 
     
      // -- Prepare:
      Mat2X features(2, observations.size()); // undistorted 2D features (one per pose)
      std::vector< Mat34 > Ps; // projective matrices (one per pose)
      {
        const track::Track & track = _map_tracks.at(trackId);
        
        int i = 0;
        for (const IndexT & viewId : observations)
        {
          const View* view = scene.GetViews().at(viewId).get();
          const IntrinsicBase* cam = scene.GetIntrinsics().at(view->getIntrinsicId()).get();
          const Vec2 x_ud = cam->get_ud_pixel(_featuresPerView->getFeatures(viewId, track.descType)[track.featPerView.at(viewId)].coords().cast<double>()); // undistorted 2D point
          features(0,i) = x_ud(0); 
          features(1,i) = x_ud(1);  
          Ps.push_back(cam->get_projective_equivalent(scene.getPose(*view)));
          i++;
        }
      }
      
      // -- Triangulate: 
      Vec4 X_homogeneous = Vec4::Zero();
      std::vector<std::size_t> inliersIndex;
      
      TriangulateNViewLORANSAC(features, Ps, &X_homogeneous, &inliersIndex, 8.0);
      
      HomogeneousToEuclidean(X_homogeneous, &X_euclidean);     
      
      // observations = {350, 380, 442} | inliersIndex = [0, 1] | inliers = {350, 380}
      for (const auto & id : inliersIndex)
        inliers.insert(*std::next(observations.begin(), id));

      // -- Check:
      //  - nb of cameras validing the track 
      //  - angle (small angle leads imprecise triangulation)
      //  - positive depth (chierality)
      if (inliers.size() < _minNbObservationsForTriangulation ||
          !checkAngles(X_euclidean, inliers, scene, _minAngleForTriangulation) ||
          !checkChieralities(X_euclidean, inliers, scene))
        isValidTrack = false;
    }  

    // -- Add the tringulated point to the scene
    if (isValidTrack)
    {
      Landmark landmark;
      landmark.X = X_euclidean;
      landmark.descType = track.descType;
      for (const IndexT & viewId : inliers) // add inliers as observations
      {
        const Vec2 x = _featuresPerView->getFeatures(viewId, track.descType)[track.featPerView.at(viewId)].coords().cast<double>();
        landmark.observations[viewId] = Observation(x, track.featPerView.at(viewId));
      }
#pragma omp critical
      {
        scene.structure[trackId] = landmark;
      }      
    }
    else
    {
#pragma omp critical
      {
        if (scene.structure.find(trackId) != scene.structure.end()) 
          scene.structure.erase(trackId);
      }
    }
  } // for all shared tracks 
}    

void ReconstructionEngine_sequentialSfM::triangulate(SfMData& scene, const std::set<IndexT>& previousReconstructedViews, const std::set<IndexT>& newReconstructedViews)
{
  {
    std::vector<IndexT> intersection;
    std::set_intersection(
          newReconstructedViews.begin(),
          newReconstructedViews.end(),
          previousReconstructedViews.begin(),
          previousReconstructedViews.end(),
          std::back_inserter(intersection));
    
    assert(intersection.empty());
  }
  
  std::set<IndexT> allReconstructedViews;
  allReconstructedViews.insert(previousReconstructedViews.begin(), previousReconstructedViews.end());
  allReconstructedViews.insert(newReconstructedViews.begin(), newReconstructedViews.end());
  
#pragma omp parallel for schedule(dynamic)
  for (ptrdiff_t i = 0; i < static_cast<ptrdiff_t>(allReconstructedViews.size()); ++i)
  {
    std::set<IndexT>::const_iterator iter = allReconstructedViews.begin();
    std::advance(iter, i);
    const IndexT indexAll = *iter;
    
    for(IndexT indexNew: newReconstructedViews)
    {
      if(indexAll == indexNew)
        continue;
      
      const std::size_t I = std::min((IndexT)indexNew, indexAll);
      const std::size_t J = std::max((IndexT)indexNew, indexAll);
      
      // Find track correspondences between I and J
      const std::set<std::size_t> set_viewIndex = { I, J };
      track::TracksMap map_tracksCommonIJ;
      track::TracksUtilsMap::GetCommonTracksInImagesFast(set_viewIndex, _map_tracks, _map_tracksPerView, map_tracksCommonIJ);

      const View* viewI = scene.GetViews().at(I).get();
      const View* viewJ = scene.GetViews().at(J).get();
      const IntrinsicBase* camI = scene.GetIntrinsics().at(viewI->getIntrinsicId()).get();
      const IntrinsicBase* camJ = scene.GetIntrinsics().at(viewJ->getIntrinsicId()).get();
      const Pose3 poseI = scene.getPose(*viewI);
      const Pose3 poseJ = scene.getPose(*viewJ);
      
      std::size_t new_putative_track = 0, new_added_track = 0, extented_track = 0;
      for (const std::pair<std::size_t, track::Track >& trackIt : map_tracksCommonIJ)
      {
        const std::size_t trackId = trackIt.first;
        const track::Track & track = trackIt.second;

        const Vec2 xI = _featuresPerView->getFeatures(I, track.descType)[track.featPerView.at(I)].coords().cast<double>();
        const Vec2 xJ = _featuresPerView->getFeatures(J, track.descType)[track.featPerView.at(J)].coords().cast<double>();
        
        // test if the track already exists in 3D
        bool trackIdExists;
#pragma omp critical
        {
          trackIdExists = scene.structure.find(trackId) != scene.structure.end();
        }
        if (trackIdExists)
        {
          // 3D point triangulated before, only add image observation if needed
#pragma omp critical
          {
            Landmark& landmark = scene.structure.at(trackId);
            if (landmark.observations.count(I) == 0)
            {
              const Vec2 residual = camI->residual(poseI, landmark.X, xI);
              const auto& acThresholdIt = _map_ACThreshold.find(I);
              // TODO assert(acThresholdIt != _map_ACThreshold.end());
              const double acThreshold = (acThresholdIt != _map_ACThreshold.end()) ? acThresholdIt->second : 4.0;
              if (poseI.depth(landmark.X) > 0 && residual.norm() < std::max(4.0, acThreshold))
              {
                landmark.observations[I] = Observation(xI, track.featPerView.at(I));
                ++extented_track;
              }
            }
            if (landmark.observations.count(J) == 0)
            {
              const Vec2 residual = camJ->residual(poseJ, landmark.X, xJ);
              const auto& acThresholdIt = _map_ACThreshold.find(J);
              // TODO assert(acThresholdIt != _map_ACThreshold.end());
              const double acThreshold = (acThresholdIt != _map_ACThreshold.end()) ? acThresholdIt->second : 4.0;
              if (poseJ.depth(landmark.X) > 0 && residual.norm() < std::max(4.0, acThreshold))
              {
                landmark.observations[J] = Observation(xJ, track.featPerView.at(J));
                ++extented_track;
              }
            }
          }
        }
        else
        {
          // A new 3D point must be added
#pragma omp critical
          {
            ++new_putative_track;
          }
          
          Vec3 X_euclidean = Vec3::Zero();
          const Vec2 xI_ud = camI->get_ud_pixel(xI);
          const Vec2 xJ_ud = camJ->get_ud_pixel(xJ);
          const Mat34 pI = camI->get_projective_equivalent(poseI);
          const Mat34 pJ = camJ->get_projective_equivalent(poseJ);
          
          TriangulateDLT(pI, xI_ud, pJ, xJ_ud, &X_euclidean);
          
          // Check triangulation results
          //  - Check angle (small angle leads imprecise triangulation)
          //  - Check positive depth
          //  - Check residual values
          const double angle = AngleBetweenRay(poseI, camI, poseJ, camJ, xI, xJ);
          const Vec2 residualI = camI->residual(poseI, X_euclidean, xI);
          const Vec2 residualJ = camJ->residual(poseJ, X_euclidean, xJ);
          
          // TODO assert(acThresholdIt != _map_ACThreshold.end());
          
          const auto& acThresholdItI = _map_ACThreshold.find(I);
          const auto& acThresholdItJ = _map_ACThreshold.find(J);
          
          const double& acThresholdI = (acThresholdItI != _map_ACThreshold.end()) ? acThresholdItI->second : 4.0;
          const double& acThresholdJ = (acThresholdItJ != _map_ACThreshold.end()) ? acThresholdItJ->second : 4.0;
          
          if (angle > _minAngleForTriangulation &&
              poseI.depth(X_euclidean) > 0 &&
              poseJ.depth(X_euclidean) > 0 &&
              residualI.norm() < acThresholdI &&
              residualJ.norm() < acThresholdJ)
          {
#pragma omp critical
            {
              // Add a new track
              Landmark & landmark = scene.structure[trackId];
              landmark.X = X_euclidean;
              landmark.descType = track.descType;
              
              landmark.observations[I] = Observation(xI, track.featPerView.at(I));
              landmark.observations[J] = Observation(xJ, track.featPerView.at(J));
              
              ++new_added_track;
            } // critical
          } // 3D point is valid
        } // else (New 3D point)
      }// for all correspondences
    }

//  #pragma omp critical
//  if (!map_tracksCommonIJ.empty())
//  {
//    ALICEVISION_LOG_DEBUG("--Triangulated 3D points [" << I << "-" << J << "]:\n"
//                      "\t#Track extented: " << extented_track << "\n"
//                      "\t#Validated/#Possible: " << new_added_track << "/" << new_putative_track << "\n"
//                      "\t#3DPoint for the entire scene: " << scene.GetLandmarks().size());
//  }
  }
}

/// Bundle adjustment to refine Structure; Motion and Intrinsics
bool ReconstructionEngine_sequentialSfM::BundleAdjustment(bool fixedIntrinsics)
{
  BundleAdjustmentCeres::BA_options options;
  if (_sfm_data.GetPoses().size() > 100)
  {
    ALICEVISION_LOG_DEBUG("Global BundleAdjustment sparse");
    options.setSparseBA();
  }
  else
  {
    ALICEVISION_LOG_DEBUG("Global BundleAdjustment dense");
    options.setDenseBA();
  }
  BundleAdjustmentCeres bundle_adjustment_obj(options);
  BA_Refine refineOptions = BA_REFINE_ROTATION | BA_REFINE_TRANSLATION | BA_REFINE_STRUCTURE;
  if(!fixedIntrinsics)
    refineOptions |= BA_REFINE_INTRINSICS_ALL;
  return bundle_adjustment_obj.Adjust(_sfm_data, refineOptions);
}

bool ReconstructionEngine_sequentialSfM::localBundleAdjustment(const std::set<IndexT>& newReconstructedViews)
{
  
  // -- Manage Ceres options (parameter ordering, local BA, sparse/dense mode, etc.)
  
  LocalBundleAdjustmentCeres::LocalBA_options options;
  options.enableParametersOrdering();
  
  if (_sfm_data.GetPoses().size() > 100) // default value: 100 
  {
    options.setSparseBA();
    options.enableLocalBA();
  }
  else
  {
    options.setDenseBA();
  }
  
  const std::size_t kMinNbOfMatches = 50; // default value: 50 
  bool isBaSucceed;
  
  // Add the new reconstructed views to the graph
  _localBA_data->updateGraphWithNewViews(_sfm_data, _map_tracksPerView, newReconstructedViews, kMinNbOfMatches);
  
  // -- Prepare Local BA & Adjust
  LocalBundleAdjustmentCeres localBA_ceres;
  
  if (options.isLocalBAEnabled()) // Local Bundle Adjustment
  {
    ALICEVISION_LOG_DEBUG("Local BA is activated: YES");
    
    // Compute the graph-distance between each newly reconstructed views and all the reconstructed views
    _localBA_data->computeGraphDistances(_sfm_data, newReconstructedViews);

    // Use the graph-distances to assign a LBA state (Refine, Constant & Ignore) for each parameter (poses, intrinsics & landmarks)
    _localBA_data->convertDistancesToLBAStates(_sfm_data); 

    // Check Ceres mode: 
    // Restore the Ceres Dense mode if the number of cameras in the solver is <= 100
    if (_localBA_data->getNumOfRefinedPoses() + _localBA_data->getNumOfConstantPoses() <= 100)
    {
      options.setDenseBA();
    }
    
    localBA_ceres = LocalBundleAdjustmentCeres(*_localBA_data, options, newReconstructedViews);
    
    // -- Refine:
    
    // Parameters are refined only if the number of cameras to refine is > to the number of newly added cameras.
    // - if there are equal: it meens that none of the new cameras is connected to the local BA graph,
    //    so the refinement would be done on those cameras only, without any constant parameters.
    // - the number of cameras to refine cannot be < to the number of newly added cameras (set to 'refine' by default)
    if (_localBA_data->getNumOfRefinedPoses() > newReconstructedViews.size())
    {
      isBaSucceed = localBA_ceres.Adjust(_sfm_data, *_localBA_data);
    }
    else
      ALICEVISION_LOG_WARNING("The refinement has not been done: the new cameras are not connected to the rest of the local BA graph.");
  }   
  else // Classic Bundle Adjustment
  {
    ALICEVISION_LOG_DEBUG("Local BA is activated: NO");

    _localBA_data->setAllParametersToRefine(_sfm_data);
    
    localBA_ceres = LocalBundleAdjustmentCeres(*_localBA_data, options, newReconstructedViews);
    
    isBaSucceed = localBA_ceres.Adjust(_sfm_data, *_localBA_data);
  }
  
  // Save the current focal lengths values (for each intrinsic) in the history  
  _localBA_data->saveFocallengthsToHistory(_sfm_data);
  // (optional) Export the current focallengths value to a txt file. 
  _localBA_data->exportFocalLengths(_localBA_data->getOutDirectory());
  
  // -- Export data about the refinement
  std::string namecomplement = "_M" + std::to_string(kMinNbOfMatches) + "_D" + std::to_string(_localBA_data->getGraphDistanceLimit());
  std::string filename =  "BaStats" + namecomplement + ".txt";
  localBA_ceres.exportStatistics(_localBA_data->getOutDirectory(), filename);
  
  return isBaSucceed;
}

/**
 * @brief Discard tracks with too large residual error
 *
 * Remove observation/tracks that have:
 *  - too large residual error
 *  - too small angular value
 *
 * @return True if more than 'count' outliers have been removed.
 */
bool ReconstructionEngine_sequentialSfM::badTrackRejector(double dPrecision, std::size_t count)
{
  const std::size_t nbOutliers_residualErr = RemoveOutliers_PixelResidualError(_sfm_data, dPrecision, 2);
  const std::size_t nbOutliers_angleErr = RemoveOutliers_AngleError(_sfm_data, 2.0);

  ALICEVISION_LOG_DEBUG("badTrackRejector: nbOutliers_residualErr: " << nbOutliers_residualErr << ", nbOutliers_angleErr: " << nbOutliers_angleErr);

  return (nbOutliers_residualErr + nbOutliers_angleErr) > count;
}

/**
 * @brief Export a JSON file containing various statistics about the SfM reconstruction
 *
 * @param[in] time_sfm time in seconds of the reconstruction process
 */
void ReconstructionEngine_sequentialSfM::exportStatistics(double time_sfm)
{
  // Put nb images, nb poses, nb points
  _tree.put("sfm.views", _sfm_data.GetViews().size());
  _tree.put("sfm.validViews", _sfm_data.getValidViews().size());
  _tree.put("sfm.poses", _sfm_data.GetPoses().size());
  _tree.put("sfm.points", _sfm_data.GetLandmarks().size());

  const double residual = RMSE(_sfm_data);
  ALICEVISION_LOG_INFO("RMSE residual: " << residual);
  _tree.put("sfm.residual", residual);

  // Add observations histogram
  std::map<std::size_t, std::size_t> obsHistogram;
  for (const auto & iterTracks : _sfm_data.GetLandmarks())
  {
    const Observations & obs = iterTracks.second.observations;
    if(obsHistogram.count(obs.size()))
      obsHistogram[obs.size()]++;
    else
      obsHistogram[obs.size()] = 1;
  }
  for(std::size_t i = 2; i < obsHistogram.size(); i++)
  {
    _tree.add("sfm.observationsHistogram."
              + std::to_string(i), obsHistogram[i]);
  }
  
  // Add process time
  _tree.put("sfm.time", time_sfm);
  
  // CPU frequency
  _tree.put("hardware.cpu.freq", system::cpu_clock_by_os());
  
  // CPU cores
  _tree.put("hardware.cpu.cores", system::get_total_cpus());
  
  _tree.put("hardware.ram.size", system::getMemoryInfo().totalRam);
  
  // Write json on disk
  pt::write_json(stlplus::folder_append_separator(_sOutDirectory)+"stats.json", _tree);
}

} // namespace sfm
} // namespace aliceVision
