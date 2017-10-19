
// Copyright (c) 2015 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.


#include "openMVG/sfm/pipelines/sequential/sequential_SfM.hpp"
#include "openMVG/sfm/pipelines/sfm_robust_model_estimation.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
#include "openMVG/sfm/sfm_data_BA_ceres.hpp"
#include "openMVG/sfm/sfm_data_BA_local_ceres.hpp"
#include "openMVG/sfm/sfm_data_filters.hpp"
#include "openMVG/sfm/pipelines/localization/SfM_Localizer.hpp"

#include "openMVG/features/FeaturesPerView.hpp"
#include "openMVG/matching/indMatch.hpp"
#include "openMVG/multiview/essential.hpp"
#include "openMVG/multiview/triangulation.hpp"
#include "openMVG/multiview/triangulation_nview.hpp"
#include "openMVG/graph/connectedComponent.hpp"
#include "openMVG/stl/stl.hpp"
#include "openMVG/system/timer.hpp"
#include "openMVG/system/cpu.hpp"
#include "openMVG/system/memoryInfo.hpp"
#include <openMVG/config.hpp>

#include "third_party/htmlDoc/htmlDoc.hpp"
#include "third_party/progress/progress.hpp"

#if OPENMVG_IS_DEFINED(OPENMVG_HAVE_BOOST)
#include <boost/format.hpp>
#endif

#include <tuple>

#ifdef _MSC_VER
#pragma warning( once : 4267 ) //warning C4267: 'argument' : conversion from 'size_t' to 'const int', possible loss of data
#endif

//#define OPENMVG_NEXTBESTVIEW_WITHOUT_SCORE

namespace openMVG {
namespace sfm {

using namespace openMVG::geometry;
using namespace openMVG::cameras;

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
    const tracks::TracksPerView& tracksPerView,
    const tracks::TracksMap& map_tracks,
    const Views& views,
    const features::FeaturesPerView& featuresProvider,
    const std::size_t pyramidBase,
    const std::size_t pyramidDepth,
    tracks::TracksPyramidPerView& tracksPyramidPerView)
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
  
#if OPENMVG_IS_DEFINED(OPENMVG_HAVE_BOOST)
  tracksPyramidPerView.reserve(tracksPerView.size());
  for(const auto& viewTracks: tracksPerView)
  {
    auto& trackPyramid = tracksPyramidPerView[viewTracks.first];
    // reserve 500 tracks in each view
    trackPyramid.reserve(500 * pyramidDepth);
  }
#endif
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
      const tracks::Track& track = map_tracks.at(trackId);
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

SequentialSfMReconstructionEngine::SequentialSfMReconstructionEngine(
    const SfM_Data & sfm_data,
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
    _htmlDocStream = std::make_shared<htmlDocument::htmlDocumentStream>("SequentialReconstructionEngine SFM report.");
    _htmlDocStream->pushInfo(
          htmlDocument::htmlMarkup("h1", std::string("SequentialSfMReconstructionEngine")));
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

SequentialSfMReconstructionEngine::~SequentialSfMReconstructionEngine()
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
void SequentialSfMReconstructionEngine::RobustResectionOfImages(
    const std::set<size_t>& viewIds,
    std::set<size_t>& set_reconstructedViewId,
    std::set<size_t>& set_rejectedViewId)
{
  static const std::size_t maxImagesPerGroup = 30;

  IndexT resectionId = 0;
  size_t imageIndex = 0;
  size_t resectionGroupIndex = 0;
  std::set<size_t> set_remainingViewId(viewIds);
  std::vector<size_t> vec_possible_resection_indexes;
  
  if (_uselocalBundleAdjustment)  
    _localBA_data->exportIntrinsicsHistory(_sOutDirectory + "/LocalBA/"); // export EXIF
  
  while (FindNextImagesGroupForResection(vec_possible_resection_indexes, set_remainingViewId))
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
      OPENMVG_LOG_DEBUG("RobustResectionOfImages : beginning of the incremental SfM" << std::endl
                        << "Only the first image of the resection group is used." << std::endl
                        << "First image ViewId : " << vec_possible_resection_indexes.front() << std::endl
                        << "# unstable poses : " << _sfm_data.GetPoses().size() << " / " << nbFirstUnstableCameras << std::endl);

      vec_possible_resection_indexes.resize(1);
    }

    OPENMVG_LOG_DEBUG("Resection group start " << resectionGroupIndex << " with " << vec_possible_resection_indexes.size() << " images.\n");
    auto chrono_start = std::chrono::steady_clock::now();
    bool bImageAdded = false;

    // get reconstructed views before resection
    const std::set<IndexT> prevReconstructedViews = _sfm_data.getValidViews();

    // add images to the 3D reconstruction

    for (const size_t possible_resection_index: vec_possible_resection_indexes )
    {
      const size_t currentIndex = imageIndex;
      ++imageIndex;

      {
        const View& view = *_sfm_data.views.at(possible_resection_index);

        if(view.isPartOfRig())
        {
          // Some views can become indirectly localized when the sub-pose becomes defined
          if(_sfm_data.IsPoseAndIntrinsicDefined(view.getViewId()))
          {
            OPENMVG_LOG_DEBUG("Resection of image " << currentIndex << " ID=" << possible_resection_index << " was skipped." << std::endl
                              << "RigID=" << view.getRigId() << " Sub-poseID=" << view.getSubPoseId()
                              << " sub-pose and pose defined.");
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
            OPENMVG_LOG_DEBUG("Resection of image " << currentIndex << " ID=" << possible_resection_index << " was skipped." << std::endl
                              << "RigID=" << view.getRigId() << " Sub-poseID=" << view.getSubPoseId()
                              << " Rig initialized but unkown pose and sub-pose.");
            set_remainingViewId.erase(possible_resection_index);
            continue;
          }
        }
      }

      const bool bResect = Resection(possible_resection_index);

      bImageAdded |= bResect;
      if (!bResect)
      {
        set_rejectedViewId.insert(possible_resection_index);
        OPENMVG_LOG_DEBUG("Resection of image " << currentIndex << " ID=" << possible_resection_index << " was not possible.");
      }
      else
      {
        set_reconstructedViewId.insert(possible_resection_index); // contains all the resected views (from the beginning of the reconstruction)
        OPENMVG_LOG_DEBUG("Resection of image: " << currentIndex << " ID=" << possible_resection_index << " succeed.");
        _sfm_data.GetViews().at(possible_resection_index)->setResectionId(resectionId);
        ++resectionId;
      }
      set_remainingViewId.erase(possible_resection_index);

      // Limit to a maximum number of cameras added to ensure that
      // we don't add too much data in one step without bundle adjustment.
      if(_sfm_data.getValidViews().size() - prevReconstructedViews.size() > maxImagesPerGroup)
      {
        break;
      }
    }
    
    OPENMVG_LOG_DEBUG("Resection of " << vec_possible_resection_indexes.size() << " new images took " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - chrono_start).count() << " msec.");

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

    OPENMVG_LOG_DEBUG("Triangulation of the " << newReconstructedViews.size() << " newly reconstructed views.");

    //BundleAdjustment();

    // triangulate
    triangulate(_sfm_data, prevReconstructedViews, newReconstructedViews);
    
    if (_uselocalBundleAdjustment)
      _localBA_data->setNewViewsId(newReconstructedViews);

    if (bImageAdded)
    {
      if((resectionGroupIndex % 10) == 0)
      {
        chrono_start = std::chrono::steady_clock::now();
        // Scene logging as ply for visual debug
        std::ostringstream os;
        os << std::setw(8) << std::setfill('0') << resectionGroupIndex << "_Resection";
        Save(_sfm_data, stlplus::create_filespec(_sOutDirectory, os.str(), _sfmdataInterFileExtension), _sfmdataInterFilter);
        OPENMVG_LOG_DEBUG("Save of file " << os.str() << " took " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - chrono_start).count() << " msec.");
      }
      OPENMVG_LOG_DEBUG("Global Bundle start, resection group index: " << resectionGroupIndex << ".");
      chrono_start = std::chrono::steady_clock::now();
      std::size_t bundleAdjustmentIteration = 0;
      const std::size_t nbOutliersThreshold = 50;
      // Perform BA until all point are under the given precision
      std::size_t nbRejectedTracks;
      
      // [TEMP] used to run Local BA (without save changes due to adjustment) the BA (saving adjustments)
      // It's useful to compare time spent in the different adjustments 
      _compareBAAndLocalBA = false;
      
      do
      {
        auto chrono2_start = std::chrono::steady_clock::now();

        if (_uselocalBundleAdjustment)
        {
          if (_compareBAAndLocalBA)
          {
            localBundleAdjustment("localBA");
            localBundleAdjustment("BA"); 
          }
          else
            localBundleAdjustment();
        }
        else
          BundleAdjustment(_bFixedIntrinsics);
        
        OPENMVG_LOG_DEBUG("Resection group index: " << resectionGroupIndex << ", bundle iteration: " << bundleAdjustmentIteration
                          << " took " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - chrono2_start).count() << " msec.");
        ++bundleAdjustmentIteration;
        nbRejectedTracks = badTrackRejector(4.0, nbOutliersThreshold);   
      }
      while (nbRejectedTracks != 0);
     
      OPENMVG_LOG_DEBUG("Bundle with " << bundleAdjustmentIteration << " iterations took " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - chrono_start).count() << " msec.");
      chrono_start = std::chrono::steady_clock::now();
      
      if (_uselocalBundleAdjustment)
      {
        // Local BA activated 
        // Save the new values of each intrinsic in a backup file
        _localBA_data->exportIntrinsicsHistory(_sOutDirectory + "/LocalBA/");
        
        Poses poses_saved =  _sfm_data.GetPoses();
        std::set<IndexT> removed_posesId, removed_viewsId;
        // Remove unstable poses & extract removed poses id
        if (eraseUnstablePosesAndObservations(this->_sfm_data, _minPointsPerPose, _minTrackLength))
        {
          // If poses have been removed (may be useless ?)
          if (poses_saved !=  _sfm_data.GetPoses())
          {
            // Get removed POSES index
            for (const auto& pose : poses_saved)
            {
              IndexT id_pose = pose.first;
              if (_sfm_data.GetPoses().find(id_pose) == _sfm_data.GetPoses().end()) // id not found = removed
                removed_posesId.insert(id_pose);
            }
            // Get removed VIEWS index
            for (const auto& itView : _sfm_data.GetViews())
            {
              if (_sfm_data.IsPoseAndIntrinsicDefined(itView.second->getViewId()))
              {
                if (removed_posesId.find(itView.second->getPoseId()) != removed_posesId.end())
                  removed_viewsId.insert(itView.second->getViewId());
              }
            }
            // Remove removed views to the graph
            _localBA_data->removeViewsToTheGraph(removed_viewsId);
            OPENMVG_LOG_DEBUG("Poses (index) removed to the reconstruction: " << removed_posesId);
          }
        }
      }
      else
        eraseUnstablePosesAndObservations(this->_sfm_data, _minPointsPerPose, _minTrackLength); // just remove poses & obs.
      
      OPENMVG_LOG_DEBUG("eraseUnstablePosesAndObservations took " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - chrono_start).count() << " msec.");
    }
    
    ++resectionGroupIndex;
  }
  
  // Ensure there is no remaining outliers
  badTrackRejector(4.0, 0);
  eraseUnstablePosesAndObservations(this->_sfm_data, _minPointsPerPose, _minTrackLength);
}

bool SequentialSfMReconstructionEngine::Process()
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
  if (!InitLandmarkTracks())
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
        if(!ChooseInitialPair(_userInitialImagePair))
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
    if(MakeInitialPair3D(initialPairCandidate))
    {
      // Successfully found an initial image pair
      OPENMVG_LOG_DEBUG("Initial pair is: " << initialPairCandidate.first << ", " << initialPairCandidate.second);
      successfullInitialization = true;
      break;
    }
  }
  if(!successfullInitialization)
  {
    OPENMVG_LOG_ERROR("Initialization failed after trying all possible initial image pairs.");
    return false;
  }
  
  // timer for stats
  openMVG::system::Timer timer_sfm;
  
  std::set<std::size_t> reconstructedViewIds;
  std::set<std::size_t> rejectedViewIds;
  std::size_t nbRejectedLoops = 0;
  
  do
  {
    reconstructedViewIds.clear();
    rejectedViewIds.clear();
    
    // Compute robust Resection of remaining images
    // - group of images will be selected and resection + scene completion will be tried
    RobustResectionOfImages(
          _set_remainingViewId,
          reconstructedViewIds,
          rejectedViewIds);
    // Remove all reconstructed views from the remaining views
    for(const std::size_t v: reconstructedViewIds)
    {
      _set_remainingViewId.erase(v);
    }
    
    OPENMVG_LOG_DEBUG("SequentialSfM -- nbRejectedLoops: " << nbRejectedLoops);
    OPENMVG_LOG_DEBUG("SequentialSfM -- reconstructedViewIds: " << reconstructedViewIds.size());
    OPENMVG_LOG_DEBUG("SequentialSfM -- rejectedViewIds: " << rejectedViewIds.size());
    OPENMVG_LOG_DEBUG("SequentialSfM -- _set_remainingViewId: " << _set_remainingViewId.size());
    
    ++nbRejectedLoops;
    // Retry to perform the resectioning of all the rejected views,
    // as long as new views are successfully added.
  } while( !reconstructedViewIds.empty() && !_set_remainingViewId.empty() );
  
  // timer for stats
  const double time_sfm = timer_sfm.elapsed();
  
  //-- Reconstruction done.
  //-- Display some statistics
  OPENMVG_LOG_DEBUG(
        "-------------------------------\n"
        "-- Structure from Motion (statistics):\n"
        "-- #Camera calibrated: " << _sfm_data.GetPoses().size() <<
        " from " << _sfm_data.GetViews().size() << " input images.\n"
                                                   "-- #Tracks, #3D points: " << _sfm_data.GetLandmarks().size() << "\n"
                                                                                                                    "-------------------------------");
  
  Histogram<double> h;
  ComputeResidualsHistogram(&h);
  OPENMVG_LOG_DEBUG("Histogram of residuals:" << h.ToString());
  
  Histogram<double> hTracks;
  ComputeTracksLengthsHistogram(&hTracks);
  OPENMVG_LOG_DEBUG("Histogram of tracks length:" << hTracks.ToString());
  
  if (!_sLoggingFile.empty())
  {
    using namespace htmlDocument;
    std::ostringstream os;
    os << "Structure from Motion process finished.";
    _htmlDocStream->pushInfo("<hr>");
    _htmlDocStream->pushInfo(htmlMarkup("h1",os.str()));
    
    os.str("");
    os << "-------------------------------" << "<br>"
       << "-- Structure from Motion (statistics):<br>"
       << "-- #Camera calibrated: " << _sfm_data.GetPoses().size()
       << " from " <<_sfm_data.GetViews().size() << " input images.<br>"
       << "-- #Tracks, #3D points: " << _sfm_data.GetLandmarks().size() << "<br>"
       << "-------------------------------" << "<br>";
    _htmlDocStream->pushInfo(os.str());
    
    _htmlDocStream->pushInfo(htmlMarkup("h2","Histogram of reprojection-residuals"));
    
    
    const std::vector<double> xBin = h.GetXbinsValue();
    htmlDocument::JSXGraphWrapper jsxGraph;
    _htmlDocStream->pushXYChart(xBin, h.GetHist(),"3DtoImageResiduals");
    
    const std::vector<double> xBinTracks = hTracks.GetXbinsValue();
    htmlDocument::JSXGraphWrapper jsxGraphTracks;
    _htmlDocStream->pushXYChart(xBinTracks, hTracks.GetHist(),"3DtoTracksSize");
  }
  
#if OPENMVG_IS_DEFINED(OPENMVG_HAVE_BOOST)
  exportStatistics(time_sfm);
#endif
  return true;
}

/// Select a candidate initial pair
bool SequentialSfMReconstructionEngine::ChooseInitialPair(Pair & initialPairIndex) const
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
      
      OPENMVG_CERR("There is no defined intrinsic data in order to compute an essential matrix for the initial pair.");
      return false;
    }
    
    OPENMVG_LOG_DEBUG(
          "----------------------------------------------------\n"
          "SequentialSfMReconstructionEngine::ChooseInitialPair\n"
          "----------------------------------------------------\n"
          " Pairs that have valid intrinsic and high support of points are displayed:\n"
          " Choose one pair manually by typing the two integer indexes\n"
          "----------------------------------------------------"
          );
    
    // Try to list the 10 top pairs that have:
    //  - valid intrinsics,
    //  - valid estimated Fundamental matrix.
    std::vector< size_t > vec_NbMatchesPerPair;
    std::vector<openMVG::matching::PairwiseMatches::const_iterator> vec_MatchesIterator;
    for (openMVG::matching::PairwiseMatches::const_iterator
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
      openMVG::matching::PairwiseMatches::const_iterator iter = vec_MatchesIterator[index];
      OPENMVG_COUT("(" << iter->first.first << "," << iter->first.second <<")\t\t"
                   << iter->second.getNbAllMatches() << " matches");
    }
    
    // Ask the user to choose an initial pair (by set some view ids)
    OPENMVG_COUT(std::endl << " type INITIAL pair ids: X enter Y enter");
    int val, val2;
    if ( std::cin >> val && std::cin >> val2) {
      initialPairIndex.first = val;
      initialPairIndex.second = val2;
    }
  }
  
  OPENMVG_LOG_DEBUG("\nPutative starting pair is: (" << initialPairIndex.first
                    << "," << initialPairIndex.second << ")");
  
  // Check validity of the initial pair indices:
  if (!_featuresPerView->viewExist(initialPairIndex.first)  ||
      !_featuresPerView->viewExist(initialPairIndex.second))
  {
    OPENMVG_LOG_WARNING("At least one of the initial pair indices is invalid.");
    return false;
  }
  return true;
}

bool SequentialSfMReconstructionEngine::InitLandmarkTracks()
{
  // Compute tracks from matches
  tracks::TracksBuilder tracksBuilder;
  
  {
    // List of features matches for each couple of images
    const openMVG::matching::PairwiseMatches & map_Matches = *_pairwiseMatches;
    OPENMVG_LOG_DEBUG("Track building");
    
    tracksBuilder.Build(map_Matches);
    OPENMVG_LOG_DEBUG("Track filtering");
    tracksBuilder.Filter(_minInputTrackLength);
    
    OPENMVG_LOG_DEBUG("Track export to internal struct");
    //-- Build tracks with STL compliant type :
    tracksBuilder.ExportToSTL(_map_tracks);
    OPENMVG_LOG_DEBUG("Build tracks per view");
    tracks::TracksUtilsMap::computeTracksPerView(_map_tracks, _map_tracksPerView);
    OPENMVG_LOG_DEBUG("Build tracks pyramid per view");
    computeTracksPyramidPerView(
          _map_tracksPerView, _map_tracks, _sfm_data.views, *_featuresPerView, _pyramidBase, _pyramidDepth, _map_featsPyramidPerView);
    
    OPENMVG_LOG_DEBUG("Track stats");
    {
      std::ostringstream osTrack;
      //-- Display stats :
      //    - number of images
      //    - number of tracks
      std::set<size_t> set_imagesId;
      tracks::TracksUtilsMap::ImageIdInTracks(_map_tracksPerView, set_imagesId);
      osTrack << "------------------" << "\n"
              << "-- Tracks Stats --" << "\n"
              << " Number of tracks: " << tracksBuilder.NbTracks() << "\n"
              << " Number of images in tracks: " << set_imagesId.size() << "\n";
      //        << " Images Id: " << "\n";
      //      std::copy(set_imagesId.begin(),
      //        set_imagesId.end(),
      //        std::ostream_iterator<size_t>(osTrack, ", "));
      osTrack << "\n------------------" << "\n";
      
      std::map<size_t, size_t> map_Occurence_TrackLength;
      tracks::TracksUtilsMap::TracksLength(_map_tracks, map_Occurence_TrackLength);
      osTrack << "TrackLength, Occurrence" << "\n";
      for(const auto& iter: map_Occurence_TrackLength)
      {
        osTrack << "\t" << iter.first << "\t" << iter.second << "\n";
#if OPENMVG_IS_DEFINED(OPENMVG_HAVE_BOOST)
        // Add input tracks histogram
        _tree.add("sfm.inputtracks_histogram."
                  + std::to_string(iter.first), iter.second);
#endif
      }
      osTrack << "\n";
      OPENMVG_LOG_DEBUG(osTrack.str());
    }
  }
  return _map_tracks.size() > 0;
}

bool SequentialSfMReconstructionEngine::getBestInitialImagePairs(std::vector<Pair>& out_bestImagePairs) const
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
    OPENMVG_LOG_WARNING("Failed to find an initial pair automatically. There is no view with valid intrinsics.");
    return false;
  }
  
    /// ImagePairScore contains <imagePairScore*scoring_angle, imagePairScore, scoring_angle, numberOfInliers, imagePair>
  typedef std::tuple<double, double, double, std::size_t, Pair> ImagePairScore;
  std::vector<ImagePairScore> bestImagePairs;
  bestImagePairs.reserve(_pairwiseMatches->size());
  
  // Compute the relative pose & the 'baseline score'
  C_Progress_display my_progress_bar( _pairwiseMatches->size(),
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
    
    const size_t I = min(current_pair.first, current_pair.second);
    const size_t J = max(current_pair.first, current_pair.second);
    if (!valid_views.count(I) || !valid_views.count(J))
      continue;
    
    const View * view_I = _sfm_data.GetViews().at(I).get();
    const Intrinsics::const_iterator iterIntrinsic_I = _sfm_data.GetIntrinsics().find(view_I->getIntrinsicId());
    const View * view_J = _sfm_data.GetViews().at(J).get();
    const Intrinsics::const_iterator iterIntrinsic_J = _sfm_data.GetIntrinsics().find(view_J->getIntrinsicId());

    const Pinhole_Intrinsic * cam_I = dynamic_cast<const Pinhole_Intrinsic*>(iterIntrinsic_I->second.get());
    const Pinhole_Intrinsic * cam_J = dynamic_cast<const Pinhole_Intrinsic*>(iterIntrinsic_J->second.get());
    if (cam_I == nullptr || cam_J == nullptr)
      continue;
    
    openMVG::tracks::TracksMap map_tracksCommon;
    const std::set<size_t> set_imageIndex= {I, J};
    tracks::TracksUtilsMap::GetTracksInImagesFast(set_imageIndex, _map_tracks, _map_tracksPerView, map_tracksCommon);
    
    // Copy points correspondences to arrays for relative pose estimation
    const size_t n = map_tracksCommon.size();
    OPENMVG_LOG_INFO("AutomaticInitialPairChoice, test I: " << I << ", J: " << J << ", nbCommonTracks: " << n);
    Mat xI(2,n), xJ(2,n);
    size_t cptIndex = 0;
    std::vector<std::size_t> commonTracksIds(n);
    for (openMVG::tracks::TracksMap::const_iterator
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
    RelativePose_Info relativePose_info;
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
      // Store the pair iff the pair is in the asked angle range [fRequired_min_angle;fLimit_max_angle]
      if (scoring_angle > fRequired_min_angle &&
          scoring_angle < fLimit_max_angle)
      {
        const double imagePairScore = std::min(computeImageScore(I, validCommonTracksIds), computeImageScore(J, validCommonTracksIds));
        const double score = scoring_angle * imagePairScore;

        #pragma omp critical
        bestImagePairs.emplace_back(score, imagePairScore, scoring_angle, relativePose_info.vec_inliers.size(), current_pair);
      }
    }
  }
  // We print the N best scores and return the best one.
  const std::size_t nBestScores = std::min(std::size_t(50), bestImagePairs.size());
  std::sort(bestImagePairs.begin(), bestImagePairs.end(), std::greater<ImagePairScore>());
  OPENMVG_LOG_DEBUG(bestImagePairs.size() << " possible image pairs. " << nBestScores << " best possibles image pairs are:");
#if OPENMVG_IS_DEFINED(OPENMVG_HAVE_BOOST)
  OPENMVG_LOG_DEBUG(boost::format("%=15s | %=15s | %=15s | %=15s | %=15s") % "Pair" % "Score" % "ImagePairScore" % "Angle" % "NbMatches");
  OPENMVG_LOG_DEBUG(std::string(15*5+3*3, '-'));
  for(std::size_t i = 0; i < nBestScores; ++i)
  {
    const ImagePairScore& s = bestImagePairs[i];
    const Pair& currPair = std::get<4>(s);
    const std::string pairIdx = std::to_string(currPair.first) + ", " + std::to_string(currPair.second);
    OPENMVG_LOG_DEBUG(boost::format("%=15s | %+15.1f | %+15.1f | %+15.1f | %+15f") % pairIdx % std::get<0>(s) % std::get<1>(s) % std::get<2>(s) % std::get<3>(s));
  }
#endif
  if (bestImagePairs.empty())
  {
    OPENMVG_LOG_DEBUG("No valid initial pair found automatically.");
    return false;
  }
  out_bestImagePairs.reserve(bestImagePairs.size());
  for(const auto& imagePair: bestImagePairs)
    out_bestImagePairs.push_back(std::get<4>(imagePair));

  return true;
}

/// Compute the initial 3D seed (First camera t=0; R=Id, second estimated by 5 point algorithm)
bool SequentialSfMReconstructionEngine::MakeInitialPair3D(const Pair& current_pair)
{
  // Compute robust Essential matrix for ImageId [I,J]
  // use min max to have I < J
  const std::size_t I = min(current_pair.first, current_pair.second);
  const std::size_t J = max(current_pair.first, current_pair.second);
  
  // a. Assert we have valid pinhole cameras
  const View * viewI = _sfm_data.GetViews().at(I).get();
  const Intrinsics::const_iterator iterIntrinsicI = _sfm_data.GetIntrinsics().find(viewI->getIntrinsicId());
  const View * viewJ = _sfm_data.GetViews().at(J).get();
  const Intrinsics::const_iterator iterIntrinsicJ = _sfm_data.GetIntrinsics().find(viewJ->getIntrinsicId());

  OPENMVG_LOG_DEBUG("Initial pair is:\n"
          << "  A - Id: " << I << " - " << " filepath: " << viewI->getImagePath() << "\n"
          << "  B - Id: " << J << " - " << " filepath: " << viewJ->getImagePath());

  if (iterIntrinsicI == _sfm_data.GetIntrinsics().end() ||
      iterIntrinsicJ == _sfm_data.GetIntrinsics().end() )
  {
    OPENMVG_LOG_WARNING("Can't find initial image pair intrinsics: " << viewI->getIntrinsicId() << ", "  << viewJ->getIntrinsicId());
    return false;
  }

  const Pinhole_Intrinsic * camI = dynamic_cast<const Pinhole_Intrinsic*>(iterIntrinsicI->second.get());
  const Pinhole_Intrinsic * camJ = dynamic_cast<const Pinhole_Intrinsic*>(iterIntrinsicJ->second.get());
  if (camI == nullptr || camJ == nullptr || !camI->isValid() || !camJ->isValid())
  {
    OPENMVG_LOG_WARNING("Can't find initial image pair intrinsics (NULL ptr): " << viewI->getIntrinsicId() << ", "  << viewJ->getIntrinsicId());
    return false;
  }

  // b. Get common features between the two views
  // use the track to have a more dense match correspondence set
  openMVG::tracks::TracksMap map_tracksCommon;
  const std::set<std::size_t> set_imageIndex= {I, J};
  tracks::TracksUtilsMap::GetTracksInImagesFast(set_imageIndex, _map_tracks, _map_tracksPerView, map_tracksCommon);
  
  //-- Copy point to arrays
  const std::size_t n = map_tracksCommon.size();
  Mat xI(2,n), xJ(2,n);
  std::size_t cptIndex = 0;
  for (openMVG::tracks::TracksMap::const_iterator
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
  OPENMVG_LOG_DEBUG(n << " matches in the image pair for the initial pose estimation.");
  
  // c. Robust estimation of the relative pose
  RelativePose_Info relativePose_info;

  const std::pair<std::size_t, std::size_t> imageSizeI(camI->w(), camI->h());
  const std::pair<std::size_t, std::size_t> imageSizeJ(camJ->w(), camJ->h());

  if (!robustRelativePose(
    camI->K(), camJ->K(), xI, xJ, relativePose_info, imageSizeI, imageSizeJ, 4096))
  {
    OPENMVG_LOG_WARNING(" /!\\ Robust estimation failed to compute E for this pair");
    return false;
  }
  OPENMVG_LOG_DEBUG("A-Contrario initial pair residual: "
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
    Bundle_Adjustment_Ceres::BA_options options(true);
    options.setDenseBA();
    Bundle_Adjustment_Ceres bundle_adjustment_obj(options);
    if (!bundle_adjustment_obj.Adjust(_sfm_data, BA_REFINE_ROTATION | BA_REFINE_TRANSLATION | BA_REFINE_STRUCTURE))
    {
      OPENMVG_LOG_WARNING("BA of initial pair " << current_pair.first << ", " << current_pair.second << " failed.");

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
          OPENMVG_LOG_WARNING("BA of initial pair " << current_pair.first << ", " << current_pair.second << " failed.");

          // Clear poses, RIGs, landmarks
          _sfm_data.GetPoses().clear();
          _sfm_data.GetLandmarks().clear();
          _sfm_data.resetRigs();

          return false;
      }
      OPENMVG_LOG_DEBUG("Initial Pair, bundle iteration: " << bundleAdjustmentIteration
           << " took " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - chrono2_start).count() << " msec.");
      ++bundleAdjustmentIteration;
    }
    while (badTrackRejector(4.0, nbOutliersThreshold));


    Save(_sfm_data, stlplus::create_filespec(_sOutDirectory, "initialPair_afterBA", _sfmdataInterFileExtension), _sfmdataInterFilter);

    // Save outlier residual information
    Histogram<double> histoResiduals;
    OPENMVG_LOG_DEBUG(
          "=========================\n"
          " MSE Residual InitialPair Inlier: " << ComputeResidualsHistogram(&histoResiduals) << "\n"
                                                                                                "=========================");
    
    if (!_sLoggingFile.empty())
    {
      using namespace htmlDocument;
      _htmlDocStream->pushInfo(htmlMarkup("h1","Essential Matrix."));
      ostringstream os;
      os << std::endl
        << "-------------------------------" << "<br>"
        << "-- Robust Essential matrix: <"  << I << "," <<J << "> images: "
        << viewI->getImagePath() << ","
        << viewJ->getImagePath() << "<br>"
        << "-- Threshold: " << relativePose_info.found_residual_precision << "<br>"
        << "-- Resection status: " << "OK" << "<br>"
        << "-- Nb points used for robust Essential matrix estimation: "
        << xI.cols() << "<br>"
        << "-- Nb points validated by robust estimation: "
        << _sfm_data.structure.size() << "<br>"
        << "-- % points validated: "
        << _sfm_data.structure.size()/static_cast<float>(xI.cols())
        << "<br>"
        << "-------------------------------" << "<br>";
      _htmlDocStream->pushInfo(os.str());
      
      _htmlDocStream->pushInfo(htmlMarkup("h2",
                                          "Residual of the robust estimation (Initial triangulation). Thresholded at: "
                                          + toString(relativePose_info.found_residual_precision)));
      
      _htmlDocStream->pushInfo(htmlMarkup("h2","Histogram of residuals"));
      
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
      
      ofstream htmlFileStream( string(stlplus::folder_append_separator(_sOutDirectory) +
                                      "Reconstruction_Report.html").c_str());
      htmlFileStream << _htmlDocStream->getDoc();
    }
  }

  Save(_sfm_data, stlplus::create_filespec(_sOutDirectory, "initialPair_sfmData", _sfmdataInterFileExtension), _sfmdataInterFilter);

  return !_sfm_data.structure.empty();
}

double SequentialSfMReconstructionEngine::ComputeResidualsHistogram(Histogram<double> * histo) const
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
  
  float dMin, dMax, dMean, dMedian;
  minMaxMeanMedian<float>(vec_residuals.begin(), vec_residuals.end(),
                          dMin, dMax, dMean, dMedian);
  if (histo)  {
    *histo = Histogram<double>(dMin, dMax, 10);
    histo->Add(vec_residuals.begin(), vec_residuals.end());
  }
  
  OPENMVG_LOG_DEBUG(
        "\nSequentialSfMReconstructionEngine::ComputeResidualsMSE.\n"
        "\t-- #Tracks:\t" << _sfm_data.GetLandmarks().size() << "\n"
        "\t-- Residual min:\t" << dMin << "\n"
        "\t-- Residual median:\t" << dMedian << "\n"
        "\t-- Residual max:\t "  << dMax << "\n"
        "\t-- Residual mean:\t " << dMean);

  return dMean;
}

double SequentialSfMReconstructionEngine::ComputeTracksLengthsHistogram(Histogram<double> * histo) const
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
  
  float dMin, dMax, dMean, dMedian;
  minMaxMeanMedian<float>(
        vec_nbTracks.begin(), vec_nbTracks.end(),
        dMin, dMax, dMean, dMedian);
  
  if (histo)
  {
    *histo = Histogram<double>(dMin, dMax, dMax-dMin+1);
    histo->Add(vec_nbTracks.begin(), vec_nbTracks.end());
  }
  
  OPENMVG_LOG_DEBUG(
        "\nSequentialSfMReconstructionEngine::ComputeTracksLengthsHistogram.\n"
        "\t-- #Tracks:\t" << _sfm_data.GetLandmarks().size() << "\n"
        "\t-- Tracks Length min:\t" << dMin << "\n"
        "\t-- Tracks Length median:\t" << dMedian << "\n"
        "\t-- Tracks Length max:\t "  << dMax << "\n"
        "\t-- Tracks Length mean:\t " << dMean);

  return dMean;
}

std::size_t SequentialSfMReconstructionEngine::computeImageScore(std::size_t viewId, const std::vector<std::size_t>& trackIds) const
{
#ifdef OPENMVG_NEXTBESTVIEW_WITHOUT_SCORE
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

bool SequentialSfMReconstructionEngine::FindConnectedViews(
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
    openMVG::tracks::TracksPerView::const_iterator tracksIdsIt = _map_tracksPerView.find(viewId);
    if(tracksIdsIt == _map_tracksPerView.end())
      continue;
    
    const openMVG::tracks::TrackIdSet& set_tracksIds = tracksIdsIt->second;
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
    return get<2>(t1) > get<2>(t2);
  });
  
  return !out_connectedViews.empty();
}


bool SequentialSfMReconstructionEngine::FindNextImagesGroupForResection(
    std::vector<size_t> & out_selectedViewIds,
    const std::set<size_t>& remainingViewIds) const
{
  out_selectedViewIds.clear();
  auto chrono_start = std::chrono::steady_clock::now();
  std::vector<ViewConnectionScore> vec_viewsScore;
  if(!FindConnectedViews(vec_viewsScore, remainingViewIds))
    return false;
  
  // Impose a minimal number of points to ensure that it makes sense to try the pose estimation.
  static const std::size_t minPointsThreshold = 30;

  OPENMVG_LOG_DEBUG("FindNextImagesGroupForResection -- Scores (features): ");
  // print the 30 best scores
  for(std::size_t i = 0; i < vec_viewsScore.size() && i < 30; ++i)
  {
    OPENMVG_LOG_DEBUG_OBJ << std::get<2>(vec_viewsScore[i]) << "(" << std::get<1>(vec_viewsScore[i]) << "), ";
  }
  OPENMVG_LOG_DEBUG_OBJ << std::endl;
  
  // If the list is empty or if the list contains images with no correspondences
  // -> (no resection will be possible)
  if (vec_viewsScore.empty() || std::get<1>(vec_viewsScore[0]) == 0)
  {
    OPENMVG_LOG_DEBUG("FindNextImagesGroupForResection failed:");
    if(vec_viewsScore.empty())
    {
      OPENMVG_LOG_DEBUG("No putative image.");
    }
    else
    {
      OPENMVG_LOG_DEBUG_OBJ << "Not enough point in the putative images: ";
      for(auto v: vec_viewsScore)
        OPENMVG_LOG_DEBUG_OBJ << std::get<1>(v) << ", ";
    }
    OPENMVG_LOG_DEBUG_OBJ << std::endl;
    // All remaining images cannot be used for pose estimation
    return false;
  }
  
  // Add the image view index with the best score
  out_selectedViewIds.push_back(std::get<0>(vec_viewsScore[0]));
  
#ifdef OPENMVG_NEXTBESTVIEW_WITHOUT_SCORE
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
  OPENMVG_LOG_DEBUG(
        "FindNextImagesGroupForResection with " << out_selectedViewIds.size() << " images took: " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - chrono_start).count() << " msec\n"
                                                                                                                                                                                                                       " - Scores: " << std::get<2>(vec_viewsScore.front()) << " to " << std::get<2>(vec_viewsScore[out_selectedViewIds.size()-1]) << " (threshold was " << scoreThreshold << ")\n"
                                                                                                                                                                                                                                                                                                                                                                                              " - Features: " << std::get<1>(vec_viewsScore.front()) << " to " << std::get<1>(vec_viewsScore[out_selectedViewIds.size()-1]) << " (threshold was " << minPointsThreshold << ")");
  return true;
}

/**
 * @brief Add one image to the 3D reconstruction. To the resectioning of
 * the camera and triangulate all the new possible tracks.
 * @param[in] viewIndex: image index to add to the reconstruction.
 *
 * A. Compute 2D/3D matches
 * B. Look if intrinsic data is known or not
 * C. Do the resectioning: compute the camera pose.
 * D. Refine the pose of the found camera
 * E. Update the global scene with the new camera pose or rig pose/sub-pose
 * F. Update the observations into the global scene structure
 * G. Triangulate new possible 2D tracks
 */
bool SequentialSfMReconstructionEngine::Resection(const std::size_t viewIndex)
{
  using namespace tracks;
  
  // A. Compute 2D/3D matches
  // A1. list tracks ids used by the view
  const openMVG::tracks::TrackIdSet& set_tracksIds = _map_tracksPerView.at(viewIndex);
  
  // A2. intersects the track list with the reconstructed
  std::set<std::size_t> reconstructed_trackId;
  std::transform(_sfm_data.GetLandmarks().begin(), _sfm_data.GetLandmarks().end(),
                 std::inserter(reconstructed_trackId, reconstructed_trackId.begin()),
                 stl::RetrieveKey());
  
  // Get the ids of the already reconstructed tracks
  std::set<std::size_t> set_trackIdForResection;
  std::set_intersection(set_tracksIds.begin(), set_tracksIds.end(),
                        reconstructed_trackId.begin(),
                        reconstructed_trackId.end(),
                        std::inserter(set_trackIdForResection, set_trackIdForResection.begin()));
  
  if (set_trackIdForResection.empty())
  {
    // No match. The image has no connection with already reconstructed points.
    OPENMVG_LOG_DEBUG("Resection failed as there is no connection with already reconstructed points");
    return false;
  }
  
  // Get back featId associated to a tracksID already reconstructed.
  // These 2D/3D associations will be used for the resection.
  std::vector<TracksUtilsMap::FeatureId> vec_featIdForResection;
  
  TracksUtilsMap::GetFeatureIdInViewPerTrack(_map_tracks,
                                             set_trackIdForResection,
                                             viewIndex,
                                             &vec_featIdForResection);
  
  // Localize the image inside the SfM reconstruction
  Image_Localizer_Match_Data resection_data;
  resection_data.pt2D.resize(2, set_trackIdForResection.size());
  resection_data.pt3D.resize(3, set_trackIdForResection.size());
  resection_data.vec_descType.resize(set_trackIdForResection.size());
  
  // B. Look if intrinsic data is known or not
  const View * view_I = _sfm_data.GetViews().at(viewIndex).get();
  std::shared_ptr<cameras::IntrinsicBase> optionalIntrinsic = _sfm_data.GetIntrinsicSharedPtr(view_I->getIntrinsicId());

  std::size_t cpt = 0;
  std::set<std::size_t>::const_iterator iterTrackId = set_trackIdForResection.begin();
  for (std::vector<TracksUtilsMap::FeatureId>::const_iterator iterfeatId = vec_featIdForResection.begin();
       iterfeatId != vec_featIdForResection.end();
       ++iterfeatId, ++iterTrackId, ++cpt)
  {
    const features::EImageDescriberType descType = iterfeatId->first;
    resection_data.pt3D.col(cpt) = _sfm_data.GetLandmarks().at(*iterTrackId).X;
    resection_data.pt2D.col(cpt) = _featuresPerView->getFeatures(viewIndex, descType)[iterfeatId->second].coords().cast<double>();
    resection_data.vec_descType.at(cpt) = descType;
  }
  
  // C. Do the resectioning: compute the camera pose.
  OPENMVG_LOG_DEBUG(
        "-------------------------------\n"
        "-- Robust Resection of view: " << viewIndex);
  
  geometry::Pose3 pose;
  const bool bResection = sfm::SfM_Localizer::Localize(
      Pair(view_I->getWidth(), view_I->getHeight()),
      optionalIntrinsic.get(),
      resection_data,
      pose
    );

  if (!_sLoggingFile.empty())
  {
    using namespace htmlDocument;
    ostringstream os;
    os << "Resection of Image index: <" << viewIndex << "> image: "
      << view_I->getImagePath() <<"<br> \n";
    _htmlDocStream->pushInfo(htmlMarkup("h1",os.str()));
    
    os.str("");
    os << std::endl
      << "-------------------------------" << "<br>"
      << "-- Robust Resection of camera index: <" << viewIndex << "> image: "
      <<  view_I->getImagePath() <<"<br>"
      << "-- Threshold: " << resection_data.error_max << "<br>"
      << "-- Resection status: " << (bResection ? "OK" : "FAILED") << "<br>"
      << "-- Nb points used for Resection: " << vec_featIdForResection.size() << "<br>"
      << "-- Nb points validated by robust estimation: " << resection_data.vec_inliers.size() << "<br>"
      << "-- % points validated: "
      << resection_data.vec_inliers.size()/static_cast<float>(vec_featIdForResection.size()) << "<br>"
      << "-------------------------------" << "<br>";
    _htmlDocStream->pushInfo(os.str());
  }
  
  if (!bResection)
  {
    OPENMVG_LOG_DEBUG("Resection of view " << viewIndex << " failed.");
    return false;
  }

  // D. Refine the pose of the found camera.
  // We use a local scene with only the 3D points and the new camera.
  {
    cameras::Pinhole_Intrinsic * pinhole_cam = dynamic_cast<cameras::Pinhole_Intrinsic *>(optionalIntrinsic.get());
    const bool b_new_intrinsic = (optionalIntrinsic == nullptr) || (pinhole_cam && !pinhole_cam->isValid());
    // A valid pose has been found (try to refine it):
    // If no valid intrinsic as input:
    //  init a new one from the projection matrix decomposition
    // Else use the existing one and consider it as constant.
    if (b_new_intrinsic)
    {
      // setup a default camera model from the found projection matrix
      Mat3 K, R;
      Vec3 t;
      KRt_From_P(resection_data.projection_matrix, &K, &R, &t);
      
      const double focal = (K(0,0) + K(1,1))/2.0;
      const Vec2 principal_point(K(0,2), K(1,2));
      
      if(optionalIntrinsic == nullptr)
      {
        // Create the new camera intrinsic group
        optionalIntrinsic = createPinholeIntrinsic(_camType, view_I->getWidth(), view_I->getHeight(), focal, principal_point(0), principal_point(1));
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
    if(!sfm::SfM_Localizer::RefinePose(
         optionalIntrinsic.get(), pose,
         resection_data, true, b_new_intrinsic || intrinsicsFirstUsage))
    {
      OPENMVG_LOG_DEBUG("Resection of view " << viewIndex << " failed during pose refinement.");
      return false;
    }
    
    // E. Update the global scene with the new found camera pose, intrinsic (if not defined)

    // update the view pose or rig pose/sub-pose
    _map_ACThreshold.insert(std::make_pair(viewIndex, resection_data.error_max));

    const View& view = *_sfm_data.views.at(viewIndex);
    _sfm_data.setPose(view, pose);

    if (b_new_intrinsic)
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
      _sfm_data.intrinsics[new_intrinsic_id]= optionalIntrinsic;
    }
  }
  
  // F. Update the observations into the global scene structure
  // - Add the new 2D observations to the reconstructed tracks
  iterTrackId = set_trackIdForResection.begin();
  for (std::size_t i = 0; i < resection_data.pt2D.cols(); ++i, ++iterTrackId)
  {
    const Vec3 X = resection_data.pt3D.col(i);
    const Vec2 x = resection_data.pt2D.col(i);
    const Vec2 residual = optionalIntrinsic->residual(pose, X, x);
    if (residual.norm() < resection_data.error_max &&
        pose.depth(X) > 0)
    {
      // Inlier, add the point to the reconstructed track
      _sfm_data.structure[*iterTrackId].observations[viewIndex] = Observation(x, vec_featIdForResection[i].second);
    }
  }

  return true;
}

void SequentialSfMReconstructionEngine::triangulate(SfM_Data& scene, const std::set<IndexT>& previousReconstructedViews, const std::set<IndexT>& newReconstructedViews)
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

  std::set<IndexT> reconstructedViews;
  reconstructedViews.insert(previousReconstructedViews.begin(), previousReconstructedViews.end());
  reconstructedViews.insert(newReconstructedViews.begin(), newReconstructedViews.end());

  #pragma omp parallel for schedule(dynamic)
  for (ptrdiff_t i = 0; i < static_cast<ptrdiff_t>(reconstructedViews.size()); ++i)
  {
    std::set<IndexT>::const_iterator iter = reconstructedViews.begin();
    std::advance(iter, i);
    const IndexT indexPrev = *iter;

    for(IndexT indexNew: newReconstructedViews)
    {
      if(indexPrev == indexNew)
        continue;

      const std::size_t I = std::min((IndexT)indexNew, indexPrev);
      const std::size_t J = std::max((IndexT)indexNew, indexPrev);

      // Find track correspondences between I and J
      const std::set<std::size_t> set_viewIndex = { I, J };
      tracks::TracksMap map_tracksCommonIJ;
      tracks::TracksUtilsMap::GetTracksInImagesFast(set_viewIndex, _map_tracks, _map_tracksPerView, map_tracksCommonIJ);

      const View* viewI = scene.GetViews().at(I).get();
      const View* viewJ = scene.GetViews().at(J).get();
      const IntrinsicBase* camI = scene.GetIntrinsics().at(viewI->getIntrinsicId()).get();
      const IntrinsicBase* camJ = scene.GetIntrinsics().at(viewJ->getIntrinsicId()).get();
      const Pose3 poseI = scene.getPose(*viewI);
      const Pose3 poseJ = scene.getPose(*viewJ);
      
      std::size_t new_putative_track = 0, new_added_track = 0, extented_track = 0;
      for (const std::pair<std::size_t, tracks::Track >& trackIt : map_tracksCommonIJ)
      {
        const std::size_t trackId = trackIt.first;
        const tracks::Track & track = trackIt.second;
        
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

          if (angle > 3.0 &&
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
//    OPENMVG_LOG_DEBUG("--Triangulated 3D points [" << I << "-" << J << "]:\n"
//                      "\t#Track extented: " << extented_track << "\n"
//                      "\t#Validated/#Possible: " << new_added_track << "/" << new_putative_track << "\n"
//                      "\t#3DPoint for the entire scene: " << scene.GetLandmarks().size());
//  }
  }
}

/// Bundle adjustment to refine Structure; Motion and Intrinsics
bool SequentialSfMReconstructionEngine::BundleAdjustment(bool fixedIntrinsics)
{
  Bundle_Adjustment_Ceres::BA_options options;
  if (_sfm_data.GetPoses().size() > 100)
  {
    OPENMVG_LOG_DEBUG("Global BundleAdjustment sparse");
    options.setSparseBA();
  }
  else
  {
    OPENMVG_LOG_DEBUG("Global BundleAdjustment dense");
    options.setDenseBA();
  }
  Bundle_Adjustment_Ceres bundle_adjustment_obj(options);
  BA_Refine refineOptions = BA_REFINE_ROTATION | BA_REFINE_TRANSLATION | BA_REFINE_STRUCTURE;
  if(!fixedIntrinsics)
    refineOptions |= BA_REFINE_INTRINSICS_ALL;
  return bundle_adjustment_obj.Adjust(_sfm_data, refineOptions);
}

/// Local Bundle Adjustment to refine only the parameters close to the newly resected views.
bool SequentialSfMReconstructionEngine::localBundleAdjustment(const std::string& name)
{
  LocalBA_timeProfiler times;
  
  Local_Bundle_Adjustment_Ceres::LocalBA_options options;
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
  
  openMVG::system::Timer durationLBA;
  openMVG::system::Timer duration;
  
  Local_Bundle_Adjustment_Ceres localBA_obj(options);
  if (options.isLocalBAEnabled() && name != "BA")
  {
    // Update the 'reconstructionGraph' using the recently added cameras
    _localBA_data->updateGraphWithNewViews(_sfm_data, _map_tracksPerView);
    
    { // -- update timer
      times._graphUpdating = duration.elapsed(); 
      duration.reset();
    }
    
    // Compute the 'connexity-distance' for each poses according to the newly added views
    _localBA_data->computeDistancesMaps(_sfm_data);
    
    { // -- update timer
      times._distMapsComputing = duration.elapsed(); 
      duration.reset();
    }
    
    // Determine the LocalBA_State (reffine, constant, ignored) for all the
    // parameters (landmarks, cameras poses and intrinsics)
    _localBA_data->convertDistancesToLBAStates(_sfm_data);    
    
    // If the number of cameras that will be added to the solver is <= 100
    // we have to modify the BA mode to Dense.
    if (_localBA_data->getNumberOfConstantAndRefinedCameras() <= 100)
      options.setDenseBA();
  }
  
  localBA_obj.initStatistics(_localBA_data->getNewViewsId());
  
  duration.reset();
  
  // Run Bundle Adjustment:
  bool isBaSucceed;
  if (name == "localBA")  // do not save changes in reconstruction
  {
    const SfM_Data& const_sfm_data = Get_SfM_Data();
    isBaSucceed = localBA_obj.Adjust(const_sfm_data, *_localBA_data);
  }
  else // save the changes due to the adjustment
    isBaSucceed = localBA_obj.Adjust(_sfm_data, *_localBA_data);
  
  times._adjusting = duration.elapsed(); 
  times._allLocalBA = durationLBA.elapsed();  
  times.exportTimes(_sOutDirectory + "/LocalBA/times_"+ name +".txt");
  times.showTimes();

  // Update 'map_intrinsicsHistorical' and compute 'map_intrinsicsLimits'
  //  checkIntrinsicParametersLimits(map_intrinsicsHistorical, map_intrinsicsLimits);
  _localBA_data->addIntrinsicsToHistory(_sfm_data);
  
  // Save data about the Ceres Sover refinement:
  localBA_obj.exportStatistics(_sOutDirectory + "/LocalBA/");

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
bool SequentialSfMReconstructionEngine::badTrackRejector(double dPrecision, std::size_t count)
{
  std::cout << "-- badTrackrejector" << std::endl;
  const std::size_t nbOutliers_residualErr = RemoveOutliers_PixelResidualError(_sfm_data, dPrecision, 2);
  const std::size_t nbOutliers_angleErr = RemoveOutliers_AngleError(_sfm_data, 2.0);
  
  OPENMVG_LOG_DEBUG("badTrackRejector: nbOutliers_residualErr: " << nbOutliers_residualErr << ", nbOutliers_angleErr: " << nbOutliers_angleErr);
  std::cout << "-- badTrackrejector: done " << std::endl;
  return nbOutliers_residualErr + nbOutliers_angleErr;
}

/**
 * @brief Export a JSON file containing various statistics about the SfM reconstruction
 *
 * @param[in] time_sfm time in seconds of the reconstruction process
 */
#if OPENMVG_IS_DEFINED(OPENMVG_HAVE_BOOST)
void SequentialSfMReconstructionEngine::exportStatistics(double time_sfm)
{
  // Put nb images, nb poses, nb points
  _tree.put("sfm.views", _sfm_data.GetViews().size());
  _tree.put("sfm.poses", _sfm_data.GetPoses().size());
  _tree.put("sfm.points", _sfm_data.GetLandmarks().size());
  
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
#endif

} // namespace sfm
} // namespace openMVG

