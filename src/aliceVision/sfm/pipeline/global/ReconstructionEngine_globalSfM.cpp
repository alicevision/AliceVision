// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "ReconstructionEngine_globalSfM.hpp"
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/multiview/triangulation/triangulationDLT.hpp>
#include <aliceVision/multiview/triangulation/Triangulation.hpp>
#include <aliceVision/graph/connectedComponent.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/stl/stl.hpp>
#include <aliceVision/multiview/essential.hpp>
#include <aliceVision/track/TracksBuilder.hpp>
#include <aliceVision/track/tracksUtils.hpp>
#include <aliceVision/config.hpp>

#include <dependencies/htmlDoc/htmlDoc.hpp>

#include <boost/progress.hpp>

#ifdef _MSC_VER
#pragma warning( once : 4267 ) //warning C4267: 'argument' : conversion from 'size_t' to 'const int', possible loss of data
#endif

namespace aliceVision {
namespace sfm {

using namespace aliceVision::camera;
using namespace aliceVision::geometry;
using namespace aliceVision::feature;
using namespace aliceVision::sfmData;

ReconstructionEngine_globalSfM::ReconstructionEngine_globalSfM(const SfMData& sfmData,
                                                               const std::string& outDirectory,
                                                               const std::string& loggingFile)
  : ReconstructionEngine(sfmData, outDirectory)
  , _loggingFile(loggingFile)
  , _normalizedFeaturesPerView(nullptr)
{
  if(!_loggingFile.empty())
  {
    // setup HTML logger
    _htmlDocStream = std::make_shared<htmlDocument::htmlDocumentStream>("GlobalReconstructionEngine SFM report.");
    _htmlDocStream->pushInfo(htmlDocument::htmlMarkup("h1", std::string("ReconstructionEngine_globalSfM")));
    _htmlDocStream->pushInfo("<hr>");
    _htmlDocStream->pushInfo( "Dataset info:");
    _htmlDocStream->pushInfo( "Views count: " + htmlDocument::toString( sfmData.getViews().size()) + "<br>");
  }

  // Set default motion Averaging methods
  _eRotationAveragingMethod = ROTATION_AVERAGING_L2;
  _eTranslationAveragingMethod = TRANSLATION_AVERAGING_L1;
}

ReconstructionEngine_globalSfM::~ReconstructionEngine_globalSfM()
{
  if(!_loggingFile.empty())
  {
    // Save the reconstruction Log
    std::ofstream htmlFileStream(_loggingFile.c_str());
    htmlFileStream << _htmlDocStream->getDoc();
  }
}

void ReconstructionEngine_globalSfM::SetFeaturesProvider(feature::FeaturesPerView* featuresPerView)
{
  _featuresPerView = featuresPerView;

  // Copy features and save a normalized version
  _normalizedFeaturesPerView = std::make_shared<FeaturesPerView>(*featuresPerView);
  #pragma omp parallel
  for(MapFeaturesPerView::iterator iter = _normalizedFeaturesPerView->getData().begin();
    iter != _normalizedFeaturesPerView->getData().end(); ++iter)
  {
    #pragma omp single nowait
    {
      // get the related view & camera intrinsic and compute the corresponding bearing vectors
      const View * view = _sfmData.getViews().at(iter->first).get();
      if(_sfmData.getIntrinsics().count(view->getIntrinsicId()))
      {
        const std::shared_ptr<IntrinsicBase> cam = _sfmData.getIntrinsics().find(view->getIntrinsicId())->second;
        for(auto& iterFeatPerDesc: iter->second)
        {
          for (PointFeatures::iterator iterPt = iterFeatPerDesc.second.begin();
            iterPt != iterFeatPerDesc.second.end(); ++iterPt)
          {
            const Vec2 pt = iterPt->coords().cast<double>();
            const Vec3 bearingVector = cam->toUnitSphere(cam->removeDistortion(cam->ima2cam(pt)));
            iterPt->coords() << (bearingVector.head(2) / bearingVector(2)).cast<float>();
          }
        }
      }
    }
  }
}

void ReconstructionEngine_globalSfM::SetMatchesProvider(matching::PairwiseMatches* provider)
{
  _pairwiseMatches = provider;
}

void ReconstructionEngine_globalSfM::SetRotationAveragingMethod(ERotationAveragingMethod eRotationAveragingMethod)
{
  _eRotationAveragingMethod = eRotationAveragingMethod;
}

void ReconstructionEngine_globalSfM::SetTranslationAveragingMethod(ETranslationAveragingMethod eTranslationAveragingMethod)
{
  _eTranslationAveragingMethod = eTranslationAveragingMethod;
}

bool ReconstructionEngine_globalSfM::process()
{
  // keep only the largest biedge connected subgraph
  {
    const PairSet pairs = matching::getImagePairs(*_pairwiseMatches);
    const std::set<IndexT> set_remainingIds = graph::CleanGraph_KeepLargestBiEdge_Nodes<PairSet, IndexT>(pairs, _outputFolder);
    if(set_remainingIds.empty())
    {
      ALICEVISION_LOG_DEBUG("Invalid input image graph for global SfM");
      return false;
    }
    KeepOnlyReferencedElement(set_remainingIds, *_pairwiseMatches);
  }

  aliceVision::rotationAveraging::RelativeRotations relatives_R;
  Compute_Relative_Rotations(relatives_R);

  HashMap<IndexT, Mat3> global_rotations;
  if(!Compute_Global_Rotations(relatives_R, global_rotations))
  {
    ALICEVISION_LOG_WARNING("GlobalSfM:: Rotation Averaging failure!");
    return false;
  }
  matching::PairwiseMatches tripletWise_matches;
  if(!Compute_Global_Translations(global_rotations, tripletWise_matches))
  {
    ALICEVISION_LOG_WARNING("GlobalSfM:: Translation Averaging failure!");
    return false;
  }
  if(!Compute_Initial_Structure(tripletWise_matches))
  {
    ALICEVISION_LOG_WARNING("GlobalSfM:: Cannot initialize an initial structure!");
    return false;
  }
  if(!Adjust())
  {
    ALICEVISION_LOG_WARNING("GlobalSfM:: Non-linear adjustment failure!");
    return false;
  }

  //-- Export statistics about the SfM process
  if (!_loggingFile.empty())
  {
    using namespace htmlDocument;
    std::ostringstream os;
    os << "Structure from Motion statistics.";
    _htmlDocStream->pushInfo("<hr>");
    _htmlDocStream->pushInfo(htmlMarkup("h1",os.str()));

    os.str("");
    os << "-------------------------------" << "<br>"
      << "-- View count: " << _sfmData.getViews().size() << "<br>"
      << "-- Intrinsic count: " << _sfmData.getIntrinsics().size() << "<br>"
      << "-- Pose count: " << _sfmData.getPoses().size() << "<br>"
      << "-- Track count: "  << _sfmData.getLandmarks().size() << "<br>"
      << "-------------------------------" << "<br>";
    _htmlDocStream->pushInfo(os.str());
  }

  return true;
}

/// Compute from relative rotations the global rotations of the camera poses
bool ReconstructionEngine_globalSfM::Compute_Global_Rotations(const rotationAveraging::RelativeRotations& relatives_R,
                                                              HashMap<IndexT, Mat3>& global_rotations)
{
  if(relatives_R.empty())
    return false;
  // Log statistics about the relative rotation graph
  {
    std::set<IndexT> set_pose_ids;
    for (const auto & relative_R : relatives_R)
    {
      set_pose_ids.insert(relative_R.i);
      set_pose_ids.insert(relative_R.j);
    }

    ALICEVISION_LOG_DEBUG("Global rotations computation: " << "\n"
                          "\t- relative rotations: " << relatives_R.size() << "\n"
                          "\t- global rotations: " << set_pose_ids.size());
  }

  // Global Rotation solver:
  const ERelativeRotationInferenceMethod eRelativeRotationInferenceMethod = TRIPLET_ROTATION_INFERENCE_COMPOSITION_ERROR; //TRIPLET_ROTATION_INFERENCE_NONE;

  GlobalSfMRotationAveragingSolver rotationAveraging_solver;
  //-- Rejection triplet that are 'not' identity rotation (error to identity > 5°)
  const bool b_rotationAveraging = rotationAveraging_solver.Run(_eRotationAveragingMethod, eRelativeRotationInferenceMethod, relatives_R, 5.0, global_rotations);

  ALICEVISION_LOG_DEBUG("Found #global_rotations: " << global_rotations.size());

  if(b_rotationAveraging)
  {
    // Log input graph to the HTML report
    if(!_loggingFile.empty() && !_outputFolder.empty())
    {
      // Log a relative pose graph
      {
        std::set<IndexT> set_pose_ids;
        PairSet relative_pose_pairs;
        for(const auto & view : _sfmData.getViews())
        {
          const IndexT pose_id = view.second->getPoseId();
          set_pose_ids.insert(pose_id);
        }
        const std::string sGraph_name = "global_relative_rotation_pose_graph_final";
        graph::indexedGraph putativeGraph(set_pose_ids, rotationAveraging_solver.GetUsedPairs());
        graph::exportToGraphvizData((fs::path(_outputFolder) / (sGraph_name + ".dot")).string(), putativeGraph.g);

        /*
        using namespace htmlDocument;
        std::ostringstream os;
        os << "<br>" << sGraph_name << "<br>"
           << "<img src=\""
           << (fs::path(_sOutDirectory) / (sGraph_name + "svg")).string()
           << "\" height=\"600\">\n";
        _htmlDocStream->pushInfo(os.str());
        */
      }
    }
  }

  return b_rotationAveraging;
}

/// Compute/refine relative translations and compute global translations
bool ReconstructionEngine_globalSfM::Compute_Global_Translations(const HashMap<IndexT, Mat3>& global_rotations,
                                                                 matching::PairwiseMatches& tripletWise_matches)
{
  // Translation averaging (compute translations & update them to a global common coordinates system)
  GlobalSfMTranslationAveragingSolver translation_averaging_solver;
  const bool bTranslationAveraging = translation_averaging_solver.Run(
    _eTranslationAveragingMethod,
    _sfmData,
    *_normalizedFeaturesPerView.get(),
    *_pairwiseMatches,
    global_rotations,
    _randomNumberGenerator,
    tripletWise_matches);

  if(!_loggingFile.empty())
  {
    sfmDataIO::Save(_sfmData,(fs::path(_loggingFile).parent_path() / "cameraPath_translation_averaging.ply").string(), sfmDataIO::ESfMData(sfmDataIO::EXTRINSICS));
  }

  return bTranslationAveraging;
}

/// Compute the initial structure of the scene
bool ReconstructionEngine_globalSfM::Compute_Initial_Structure(matching::PairwiseMatches& tripletWise_matches)
{
  // Build tracks from selected triplets (Union of all the validated triplet tracks (_tripletWise_matches))
  {
    using namespace aliceVision::track;
    TracksBuilder tracksBuilder;
#ifdef USE_ALL_VALID_MATCHES // not used by default
    matching::PairwiseMatches pose_supported_matches;
    for (const auto & pairwiseMatchesIt :  *_pairwiseMatches)
    {
      const View * vI = _sfm_data.getViews().at(pairwiseMatchesIt.first.first).get();
      const View * vJ = _sfm_data.getViews().at(pairwiseMatchesIt.first.second).get();
      if (_sfm_data.isPoseAndIntrinsicDefined(vI) && _sfm_data.isPoseAndIntrinsicDefined(vJ))
      {
        pose_supported_matches.insert(pairwiseMatchesIt);
      }
    }
    tracksBuilder.build(pose_supported_matches);
#else
    // Use triplet validated matches
    tracksBuilder.build(tripletWise_matches);
#endif
    tracksBuilder.filter(true,3);
    TracksMap map_selectedTracks; // reconstructed track (visibility per 3D point)
    tracksBuilder.exportToSTL(map_selectedTracks);

    // Fill sfm_data with the computed tracks (no 3D yet)
    Landmarks & structure = _sfmData.structure;
    IndexT idx(0);
    for (TracksMap::const_iterator itTracks = map_selectedTracks.begin();
      itTracks != map_selectedTracks.end();
      ++itTracks, ++idx)
    {
      const Track & track = itTracks->second;
      Landmark& newLandmark = structure[idx];
      newLandmark.descType = track.descType;
      Observations & obs = newLandmark.observations;
      for (Track::FeatureIdPerView::const_iterator it = track.featPerView.begin(); it != track.featPerView.end(); ++it)
      {
        const size_t imaIndex = it->first;
        const size_t featIndex = it->second;
        const PointFeature & pt = _featuresPerView->getFeatures(imaIndex, track.descType)[featIndex];

        const double scale = (_featureConstraint == EFeatureConstraint::BASIC) ? 0.0 : pt.scale();
        obs[imaIndex] = Observation(pt.coords().cast<double>(), featIndex, scale);
      }
    }

    ALICEVISION_LOG_DEBUG("Track stats");
    {
      std::ostringstream osTrack;
      //-- Display stats:
      //    - number of images
      //    - number of tracks
      std::set<size_t> set_imagesId;
      imageIdInTracks(map_selectedTracks, set_imagesId);
      osTrack << "------------------" << "\n"
        << "-- Tracks Stats --" << "\n"
        << " Tracks number: " << tracksBuilder.nbTracks() << "\n"
        << " Images Id: " << "\n";
      std::copy(set_imagesId.begin(),
        set_imagesId.end(),
        std::ostream_iterator<size_t>(osTrack, ", "));
      osTrack << "\n------------------" << "\n";

      std::map<size_t, size_t> map_Occurence_TrackLength;
      tracksLength(map_selectedTracks, map_Occurence_TrackLength);
      osTrack << "TrackLength, Occurrence" << "\n";
      for (std::map<size_t, size_t>::const_iterator iter = map_Occurence_TrackLength.begin();
        iter != map_Occurence_TrackLength.end(); ++iter)  {
        osTrack << "\t" << iter->first << "\t" << iter->second << "\n";
      }
      osTrack << "\n";
      ALICEVISION_LOG_DEBUG(osTrack.str());
    }
  }

  // Compute 3D position of the landmark of the structure by triangulation of the observations
  {
    aliceVision::system::Timer timer;

    const IndexT trackCountBefore = _sfmData.getLandmarks().size();
    StructureComputation_blind structure_estimator(true);
    structure_estimator.triangulate(_sfmData, _randomNumberGenerator);

    ALICEVISION_LOG_DEBUG("#removed tracks (invalid triangulation): " <<
      trackCountBefore - IndexT(_sfmData.getLandmarks().size()));
    ALICEVISION_LOG_DEBUG("  Triangulation took (s): " << timer.elapsed());

    // Export initial structure
    if (!_loggingFile.empty())
    {
      sfmDataIO::Save(_sfmData,
                     (fs::path(_loggingFile).parent_path() / "initial_structure.ply").string(),
                     sfmDataIO::ESfMData(sfmDataIO::EXTRINSICS | sfmDataIO::STRUCTURE));
    }
  }
  return !_sfmData.structure.empty();
}

// Adjust the scene (& remove outliers)
bool ReconstructionEngine_globalSfM::Adjust()
{
  // refine sfm  scene (in a 3 iteration process (free the parameters regarding their incertainty order)):
  BundleAdjustmentCeres::CeresOptions options; 
  options.useParametersOrdering = false; // disable parameters ordering

  BundleAdjustmentCeres BA(options);
  // - refine only Structure and translations
  bool success = BA.adjust(_sfmData, BundleAdjustment::REFINE_TRANSLATION | BundleAdjustment::REFINE_STRUCTURE);
  if(success)
  {
    if(!_loggingFile.empty())
      sfmDataIO::Save(_sfmData, (fs::path(_loggingFile).parent_path() / "structure_00_refine_T_Xi.ply").string(), sfmDataIO::ESfMData(sfmDataIO::EXTRINSICS | sfmDataIO::STRUCTURE));

    // refine only structure and rotations & translations
    success = BA.adjust(_sfmData, BundleAdjustment::REFINE_ROTATION | BundleAdjustment::REFINE_TRANSLATION | BundleAdjustment::REFINE_STRUCTURE);

    if(success && !_loggingFile.empty())
      sfmDataIO::Save(_sfmData, (fs::path(_loggingFile).parent_path() / "structure_01_refine_RT_Xi.ply").string(), sfmDataIO::ESfMData(sfmDataIO::EXTRINSICS | sfmDataIO::STRUCTURE));
  }

  if(success && !_lockAllIntrinsics)
  {
    // refine all: Structure, motion:{rotations, translations} and optics:{intrinsics}
    success = BA.adjust(_sfmData, BundleAdjustment::REFINE_ALL);
    if(success && !_loggingFile.empty())
      sfmDataIO::Save(_sfmData, (fs::path(_loggingFile).parent_path() / "structure_02_refine_KRT_Xi.ply").string(), sfmDataIO::ESfMData(sfmDataIO::EXTRINSICS | sfmDataIO::STRUCTURE));
  }

  // Remove outliers (max_angle, residual error)
  const size_t pointcount_initial = _sfmData.structure.size();
  RemoveOutliers_PixelResidualError(_sfmData, _featureConstraint, 4.0);
  const size_t pointcount_pixelresidual_filter = _sfmData.structure.size();
  RemoveOutliers_AngleError(_sfmData, 2.0);
  const size_t pointcount_angular_filter = _sfmData.structure.size();
  ALICEVISION_LOG_DEBUG("Outlier removal (remaining points):\n"
                        "\t- # landmarks initial: " << pointcount_initial << "\n"
                        "\t- # landmarks after pixel residual filter: " << pointcount_pixelresidual_filter << "\n"
                        "\t- # landmarks after angular filter: " << pointcount_angular_filter);

  if(!_loggingFile.empty())
    sfmDataIO::Save(_sfmData, (fs::path(_loggingFile).parent_path() / "structure_03_outlier_removed.ply").string(), sfmDataIO::ESfMData(sfmDataIO::EXTRINSICS | sfmDataIO::STRUCTURE));

  // check that poses & intrinsic cover some measures (after outlier removal)
  const IndexT minPointPerPose = 12; // 6 min
  const IndexT minTrackLength = 3; // 2 min todo param@L
  
  if (eraseUnstablePosesAndObservations(_sfmData, minPointPerPose, minTrackLength))
  {
    // TODO: must ensure that track graph is producing a single connected component

    const size_t pointcount_cleaning = _sfmData.structure.size();
    ALICEVISION_LOG_DEBUG("# landmarks after eraseUnstablePosesAndObservations: " << pointcount_cleaning);
  }

  BundleAdjustment::ERefineOptions refineOptions = BundleAdjustment::REFINE_ROTATION | BundleAdjustment::REFINE_TRANSLATION | BundleAdjustment::REFINE_STRUCTURE;
  if(!_lockAllIntrinsics)
    refineOptions |= BundleAdjustment::REFINE_INTRINSICS_ALL;
  success = BA.adjust(_sfmData, refineOptions);

  if(success && !_loggingFile.empty())
    sfmDataIO::Save(_sfmData, (fs::path(_loggingFile).parent_path() / "structure_04_outlier_removed.ply").string(), sfmDataIO::ESfMData(sfmDataIO::EXTRINSICS | sfmDataIO::STRUCTURE));

  return success;
}

void ReconstructionEngine_globalSfM::Compute_Relative_Rotations(rotationAveraging::RelativeRotations& vec_relatives_R)
{
  //
  // Build the Relative pose graph from matches:
  //
  /// pairwise view relation between poseIds
  typedef std::map<Pair, PairSet> PoseWiseMatches;

  // List shared correspondences (pairs) between poses
  PoseWiseMatches poseWiseMatches;
  for (matching::PairwiseMatches::const_iterator iterMatches = _pairwiseMatches->begin();
    iterMatches != _pairwiseMatches->end(); ++iterMatches)
  {
    const Pair pair = iterMatches->first;
    const View* v1 = _sfmData.getViews().at(pair.first).get();
    const View* v2 = _sfmData.getViews().at(pair.second).get();
    poseWiseMatches[Pair(v1->getPoseId(), v2->getPoseId())].insert(pair);
  }

  boost::progress_display progressBar( poseWiseMatches.size(), std::cout, "\n- Relative pose computation -\n" );
  #pragma omp parallel for schedule(dynamic)
  // Compute the relative pose from pairwise point matches:
  for (int i = 0; i < poseWiseMatches.size(); ++i)
  {
    #pragma omp critical
    {
      ++progressBar;
    }
    {
      PoseWiseMatches::const_iterator iter (poseWiseMatches.begin());
      std::advance(iter, i);
      const auto& relative_pose_iterator(*iter);
      const Pair relative_pose_pair = relative_pose_iterator.first;
      const PairSet& match_pairs = relative_pose_iterator.second;

      // If a pair has the same ID, discard it
      if (relative_pose_pair.first == relative_pose_pair.second)
      {
        continue;
      }

      // Select common bearing vectors
      if (match_pairs.size() > 1)
      {
        ALICEVISION_LOG_WARNING("Compute relative pose between more than two view is not supported");
        continue;
      }

      const Pair pairIterator = *(match_pairs.begin());

      const IndexT I = pairIterator.first;
      const IndexT J = pairIterator.second;

      const View* view_I = _sfmData.views[I].get();
      const View* view_J = _sfmData.views[J].get();

      // Check that valid cameras are existing for the pair of view
      if (_sfmData.getIntrinsics().count(view_I->getIntrinsicId()) == 0 ||
        _sfmData.getIntrinsics().count(view_J->getIntrinsicId()) == 0)
        continue;

      // Setup corresponding bearing vector
      const matching::MatchesPerDescType & matchesPerDesc = _pairwiseMatches->at(pairIterator);
      const std::size_t nbBearing = matchesPerDesc.getNbAllMatches();
      std::size_t iBearing = 0;
      Mat x1(2, nbBearing), x2(2, nbBearing);

      for(const auto& matchesPerDescIt: matchesPerDesc)
      {
        const feature::EImageDescriberType descType = matchesPerDescIt.first;
        assert(descType != feature::EImageDescriberType::UNINITIALIZED);
        const matching::IndMatches & matches = matchesPerDescIt.second;

        for (const auto & match : matches)
        {
          x1.col(iBearing) = _normalizedFeaturesPerView->getFeatures(I, descType)[match._i].coords().cast<double>();
          x2.col(iBearing++) = _normalizedFeaturesPerView->getFeatures(J, descType)[match._j].coords().cast<double>();
        }
      }
      assert(nbBearing == iBearing);


      std::shared_ptr<camera::IntrinsicBase> cam_I = _sfmData.getIntrinsics().at(view_I->getIntrinsicId());
      std::shared_ptr<camera::Pinhole> camIPinHole = std::dynamic_pointer_cast<camera::Pinhole>(cam_I);
      if (!camIPinHole) {
        ALICEVISION_LOG_ERROR("Camera is not pinhole in Compute_Relative_Rotations");
        continue;
      }

      std::shared_ptr<camera::IntrinsicBase> cam_J = _sfmData.getIntrinsics().at(view_J->getIntrinsicId());
      std::shared_ptr<camera::Pinhole> camJPinHole = std::dynamic_pointer_cast<camera::Pinhole>(cam_J);
      if (!camJPinHole) {
        ALICEVISION_LOG_ERROR("Camera is not pinhole in Compute_Relative_Rotations");
        continue;
      }

      RelativePoseInfo relativePose_info;
      // Compute max authorized error as geometric mean of camera plane tolerated residual error
      relativePose_info.initial_residual_tolerance = std::pow(
        cam_I->imagePlaneToCameraPlaneError(2.5) *
        cam_J->imagePlaneToCameraPlaneError(2.5),
        1./2.);

      // Since we use normalized features, we will use unit image size and intrinsic matrix:
      const std::pair<size_t, size_t> imageSize(1., 1.);
      const Mat3 K  = Mat3::Identity();


      if(!robustRelativePose(K, K, x1, x2, _randomNumberGenerator, relativePose_info, imageSize, imageSize, 256))
      {
        continue;
      }

      const bool refineUsingBA = true;
      if(refineUsingBA)
      {
        // Refine the defined scene
        SfMData tinyScene;
        tinyScene.views.insert(*_sfmData.getViews().find(view_I->getViewId()));
        tinyScene.views.insert(*_sfmData.getViews().find(view_J->getViewId()));
        tinyScene.intrinsics.insert(*_sfmData.getIntrinsics().find(view_I->getIntrinsicId()));
        tinyScene.intrinsics.insert(*_sfmData.getIntrinsics().find(view_J->getIntrinsicId()));

        // Init poses
        const Pose3& poseI = Pose3(Mat3::Identity(), Vec3::Zero());
        const Pose3& poseJ = relativePose_info.relativePose;

        tinyScene.setPose(*view_I, CameraPose(poseI));
        tinyScene.setPose(*view_J, CameraPose(poseJ));

        // Init structure
        const Mat34 P1 = camIPinHole->getProjectiveEquivalent(poseI);
        const Mat34 P2 = camJPinHole->getProjectiveEquivalent(poseJ);
        Landmarks & landmarks = tinyScene.structure;

        size_t landmarkId = 0;
        for(const auto& matchesPerDescIt: matchesPerDesc)
        {
          const feature::EImageDescriberType descType = matchesPerDescIt.first;
          assert(descType != feature::EImageDescriberType::UNINITIALIZED);
          if(descType == feature::EImageDescriberType::UNINITIALIZED)
            throw std::logic_error("descType UNINITIALIZED");
          const matching::IndMatches & matches = matchesPerDescIt.second;
          for (const matching::IndMatch& match: matches)
          {
            const PointFeature& p1 = _featuresPerView->getFeatures(I, descType)[match._i];
            const PointFeature& p2 = _featuresPerView->getFeatures(J, descType)[match._j];
            const Vec2 x1_ = p1.coords().cast<double>();
            const Vec2 x2_ = p2.coords().cast<double>();
            Vec3 X;
            multiview::TriangulateDLT(P1, x1_, P2, x2_, &X);
            Observations obs;
            const double scaleI = (_featureConstraint == EFeatureConstraint::BASIC) ? 0.0 : p1.scale();
            const double scaleJ = (_featureConstraint == EFeatureConstraint::BASIC) ? 0.0 : p2.scale();
            obs[view_I->getViewId()] = Observation(x1_, match._i, scaleI);
            obs[view_J->getViewId()] = Observation(x2_, match._j, scaleJ);
            Landmark& newLandmark = landmarks[landmarkId++];
            newLandmark.descType = descType;
            newLandmark.observations = obs;
            newLandmark.X = X;
          }
        }
        // - refine only Structure and Rotations & translations (keep intrinsic constant)
        BundleAdjustmentCeres::CeresOptions options(false, false);
        options.linearSolverType = ceres::DENSE_SCHUR;
        BundleAdjustmentCeres bundle_adjustment_obj(options);
        if(bundle_adjustment_obj.adjust(tinyScene, BundleAdjustment::REFINE_ROTATION | BundleAdjustment::REFINE_TRANSLATION | BundleAdjustment::REFINE_STRUCTURE))
        {
          // --> to debug: save relative pair geometry on disk
          // std::ostringstream os;
          // os << relative_pose_pair.first << "_" << relative_pose_pair.second << ".ply";
          // Save(tiny_scene, os.str(), ESfMData(STRUCTURE | EXTRINSICS));
          //

          const geometry::Pose3 poseI = tinyScene.getPose(*view_I).getTransform();
          const geometry::Pose3 poseJ = tinyScene.getPose(*view_J).getTransform();

          const Mat3 R1 = poseI.rotation();
          const Mat3 R2 = poseJ.rotation();
          const Vec3 t1 = poseI.translation();
          const Vec3 t2 = poseJ.translation();
          // Compute relative motion and save it
          Mat3 Rrel;
          Vec3 trel;
          relativeCameraMotion(R1, t1, R2, t2, &Rrel, &trel);
          // Update found relative pose
          relativePose_info.relativePose = Pose3(Rrel, -Rrel.transpose() * trel);
        }
      }
      #pragma omp critical
      {
        // Add the relative rotation to the relative 'rotation' pose graph
        using namespace aliceVision::rotationAveraging;
          vec_relatives_R.emplace_back(
            relative_pose_pair.first, relative_pose_pair.second,
            relativePose_info.relativePose.rotation(), relativePose_info.vec_inliers.size());
      }
    }
  } // for all relative pose

  // Re-weight rotation in [0,1]
  if (vec_relatives_R.size() > 1)
  {
    std::vector<double> vec_count;
    vec_count.reserve(vec_relatives_R.size());
    for(const auto & relative_rotation_info : vec_relatives_R)
    {
      vec_count.push_back(relative_rotation_info.weight);
    }
    std::partial_sort(vec_count.begin(), vec_count.begin() + vec_count.size() / 2, vec_count.end());
    // const float thTrustPair = vec_count[vec_count.size() / 2];
    for(auto & relative_rotation_info : vec_relatives_R)
    {
      relative_rotation_info.weight = std::min(relative_rotation_info.weight, 1.f);
    }
  }

  // Log input graph to the HTML report
  if (!_loggingFile.empty() && !_outputFolder.empty())
  {
    // Log a relative view graph
    {
      std::set<IndexT> set_ViewIds;
      std::transform(_sfmData.getViews().begin(), _sfmData.getViews().end(), std::inserter(set_ViewIds, set_ViewIds.begin()), stl::RetrieveKey());
      graph::indexedGraph putativeGraph(set_ViewIds, getImagePairs(*_pairwiseMatches));
      graph::exportToGraphvizData((fs::path(_outputFolder) / "global_relative_rotation_view_graph.dot").string(), putativeGraph.g);
    }

    // Log a relative pose graph
    {
      std::set<IndexT> set_pose_ids;
      PairSet relative_pose_pairs;
      for(const auto& relative_R : vec_relatives_R)
      {
        const Pair relative_pose_indices(relative_R.i, relative_R.j);
        relative_pose_pairs.insert(relative_pose_indices);
        set_pose_ids.insert(relative_R.i);
        set_pose_ids.insert(relative_R.j);
      }
      const std::string sGraph_name = "global_relative_rotation_pose_graph";
      graph::indexedGraph putativeGraph(set_pose_ids, relative_pose_pairs);
      graph::exportToGraphvizData((fs::path(_outputFolder) / (sGraph_name + ".dot")).string(), putativeGraph.g);
      /*
      using namespace htmlDocument;
      std::ostringstream os;

      os << "<br>" << "global_relative_rotation_pose_graph" << "<br>"
         << "<img src=\""
         << (fs::path(_sOutDirectory) / "global_relative_rotation_pose_graph.svg").string()
         << "\" height=\"600\">\n";
      _htmlDocStream->pushInfo(os.str());
      */
    }
  }
}

} // namespace sfm
} // namespace aliceVision

