// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "StructureEstimationFromKnownPoses.hpp"
#include <aliceVision/feature/metric.hpp>
#include <aliceVision/matching/IndMatch.hpp>
#include <aliceVision/matching/guidedMatching.hpp>
#include <aliceVision/multiview/relativePose/FundamentalError.hpp>
#include <aliceVision/multiview/triangulation/Triangulation.hpp>
#include <aliceVision/graph/graph.hpp>
#include <aliceVision/track/TracksBuilder.hpp>
#include <aliceVision/sfm/sfmTriangulation.hpp>
#include <aliceVision/config.hpp>

#include <boost/progress.hpp>

namespace aliceVision {
namespace sfm {

using namespace aliceVision::camera;
using namespace aliceVision::feature;
using namespace aliceVision::geometry;
using namespace aliceVision::sfmData;

/// Camera pair epipole (Projection of camera center 2 in the image plane 1)
inline Vec3 epipole_from_P(const Mat34& P1, const Pose3& P2)
{
  const Vec3 c = P2.center();
  Vec4 center;
  center << c(0), c(1), c(2), 1.0;
  return P1*center;
}

/// Export point feature based vector to a matrix [(x,y)'T, (x,y)'T]
/// Use the camera intrinsics in order to get undistorted pixel coordinates
template<typename MatT >
void PointsToMat(
  const IntrinsicBase * cam,
  const PointFeatures & vec_feats,
  MatT & m)
{
  m.resize(2, vec_feats.size());
  typedef typename MatT::Scalar Scalar; // Output matrix type

  size_t i = 0;
  for( PointFeatures::const_iterator iter = vec_feats.begin();
    iter != vec_feats.end(); ++iter, ++i)
  {
    if (cam && cam->isValid())
      m.col(i) = cam->get_ud_pixel(Vec2(iter->x(), iter->y()));
    else
      m.col(i) << iter->x(), iter->y();
  }
}

/// Use geometry of the views to compute a putative structure from features and descriptors.
void StructureEstimationFromKnownPoses::run(SfMData& sfmData,
  const PairSet& pairs,
  const feature::RegionsPerView& regionsPerView,
  std::mt19937 &randomNumberGenerator, 
  double geometricErrorMax)
{
  sfmData.structure.clear();

  match(sfmData, pairs, regionsPerView, geometricErrorMax);
  filter(sfmData, pairs, regionsPerView);
  triangulate(sfmData, regionsPerView, randomNumberGenerator);
}

// #define ALICEVISION_EXHAUSTIVE_MATCHING

/// Use guided matching to find corresponding 2-view correspondences
void StructureEstimationFromKnownPoses::match(const SfMData& sfmData,
  const PairSet& pairs,
  const feature::RegionsPerView& regionsPerView,
  double geometricErrorMax)
{
  boost::progress_display my_progress_bar( pairs.size(), std::cout,
    "Compute pairwise fundamental guided matching:\n" );

  #pragma omp parallel
  for (PairSet::const_iterator it = pairs.begin(); it != pairs.end(); ++it)
  {
    #pragma omp single nowait
    {
    // --
    // Perform GUIDED MATCHING
    // --
    // Use the computed model to check valid correspondences
    // - by considering geometric error and descriptor distance ratio.

    const View * viewL = sfmData.getViews().at(it->first).get();
    const Pose3 poseL = sfmData.getPose(*viewL).getTransform();
    const Intrinsics::const_iterator iterIntrinsicL = sfmData.getIntrinsics().find(viewL->getIntrinsicId());
    const View * viewR = sfmData.getViews().at(it->second).get();
    const Pose3 poseR = sfmData.getPose(*viewR).getTransform();
    const Intrinsics::const_iterator iterIntrinsicR = sfmData.getIntrinsics().find(viewR->getIntrinsicId());

    if (sfmData.getIntrinsics().count(viewL->getIntrinsicId()) != 0 ||
        sfmData.getIntrinsics().count(viewR->getIntrinsicId()) != 0)
    {
      std::shared_ptr<IntrinsicBase> camL = iterIntrinsicL->second;
      std::shared_ptr<camera::Pinhole> pinHoleCamL = std::dynamic_pointer_cast<camera::Pinhole>(camL);
      if (!pinHoleCamL) {
        ALICEVISION_LOG_ERROR("Camera is not pinhole in match");
      }

      std::shared_ptr<IntrinsicBase> camR = iterIntrinsicR->second;
      std::shared_ptr<camera::Pinhole> pinHoleCamR = std::dynamic_pointer_cast<camera::Pinhole>(camR);
      if (!pinHoleCamL) {
        ALICEVISION_LOG_ERROR("Camera is not pinhole in match");
      }
      
      const Mat34 P_L = pinHoleCamL->getProjectiveEquivalent(poseL);
      const Mat34 P_R = pinHoleCamR->getProjectiveEquivalent(poseR);

      const Mat3 F_lr = F_from_P(P_L, P_R);
      std::vector<feature::EImageDescriberType> commonDescTypes = regionsPerView.getCommonDescTypes(*it);
      
      matching::MatchesPerDescType allImagePairMatches;
      for(feature::EImageDescriberType descType: commonDescTypes)
      {
        std::vector<matching::IndMatch> matches;
#ifdef ALICEVISION_EXHAUSTIVE_MATCHING
          matching::guidedMatching
          <Mat3, multiview::relativePose::FundamentalEpipolarDistanceError>
          (
            F_lr,
            iterIntrinsicL->second.get(),
            regionsPerView.getRegions(it->first, descType),
            iterIntrinsicR->second.get(),
            regionsPerView.getRegions(it->second, descType),
            // descType,
            Square(thresholdF), Square(0.8),
            matches
          );
      #else
        const Vec3 epipole2  = epipole_from_P(P_R, poseL);

        //const feature::Regions& regions = regionsPerView.getRegions(it->first);
        matching::guidedMatchingFundamentalFast<multiview::relativePose::FundamentalEpipolarDistanceError>
          (
            F_lr,
            epipole2,
            iterIntrinsicL->second.get(),
            regionsPerView.getRegions(it->first, descType),
            iterIntrinsicR->second.get(),
            regionsPerView.getRegions(it->second, descType),
            iterIntrinsicR->second->w(), iterIntrinsicR->second->h(),
            //descType,
            Square(geometricErrorMax), Square(0.8),
            matches
          );
      #endif
         allImagePairMatches[descType] = matches;
      }

      #pragma omp critical
      {
        ++my_progress_bar;
        _putativeMatches[*it] = allImagePairMatches;
      }
    }
    }
  }
}

/// Filter inconsistent correspondences by using 3-view correspondences on view triplets
void StructureEstimationFromKnownPoses::filter(
  const SfMData& sfmData,
  const PairSet& pairs,
  const feature::RegionsPerView& regionsPerView)
{
  // Compute triplets
  // Triangulate triplet tracks
  //  - keep valid one

  typedef std::vector< graph::Triplet > Triplets;
  const Triplets triplets = graph::tripletListing(pairs);

  boost::progress_display my_progress_bar( triplets.size(), std::cout,
    "Per triplet tracks validation (discard spurious correspondences):\n" );
  #pragma omp parallel
  for( Triplets::const_iterator it = triplets.begin(); it != triplets.end(); ++it)
  {
    #pragma omp single nowait
    {
      #pragma omp critical
      {++my_progress_bar;}

      const graph::Triplet & triplet = *it;
      const IndexT I = triplet.i, J = triplet.j , K = triplet.k;

      track::TracksMap map_tracksCommon;
      track::TracksBuilder tracksBuilder;
      {
        matching::PairwiseMatches map_matchesIJK;
        if (_putativeMatches.count(std::make_pair(I,J)))
          map_matchesIJK.insert(*_putativeMatches.find(std::make_pair(I,J)));

        if (_putativeMatches.count(std::make_pair(I,K)))
          map_matchesIJK.insert(*_putativeMatches.find(std::make_pair(I,K)));

        if (_putativeMatches.count(std::make_pair(J,K)))
          map_matchesIJK.insert(*_putativeMatches.find(std::make_pair(J,K)));

        if (map_matchesIJK.size() >= 2) {
          tracksBuilder.build(map_matchesIJK);
          tracksBuilder.filter(true,3, false);
          tracksBuilder.exportToSTL(map_tracksCommon);
        }

        // Triangulate the tracks
        for (track::TracksMap::const_iterator iterTracks = map_tracksCommon.begin();
          iterTracks != map_tracksCommon.end(); ++iterTracks) {
          {
            const track::Track & subTrack = iterTracks->second;
            multiview::Triangulation trianObj;
            for (auto iter = subTrack.featPerView.begin(); iter != subTrack.featPerView.end(); ++iter)
            {
              const size_t imaIndex = iter->first;
              const size_t featIndex = iter->second;
              const View * view = sfmData.getViews().at(imaIndex).get();
              
              std::shared_ptr<camera::IntrinsicBase> cam = sfmData.getIntrinsics().at(view->getIntrinsicId());
              std::shared_ptr<camera::Pinhole> camPinHole = std::dynamic_pointer_cast<camera::Pinhole>(cam);
              if (!camPinHole) {
                ALICEVISION_LOG_ERROR("Camera is not pinhole in filter");
                continue;
              }

              const Pose3 pose = sfmData.getPose(*view).getTransform();
              const Vec2 pt = regionsPerView.getRegions(imaIndex, subTrack.descType).GetRegionPosition(featIndex);
              trianObj.add(camPinHole->getProjectiveEquivalent(pose), cam->get_ud_pixel(pt));
            }
            const Vec3 Xs = trianObj.compute();
            if (trianObj.minDepth() > 0 && trianObj.error()/(double)trianObj.size() < 4.0)
            // TODO: Add an angular check ?
            {
              #pragma omp critical
              {
                track::Track::FeatureIdPerView::const_iterator iterI, iterJ, iterK;
                iterI = iterJ = iterK = subTrack.featPerView.begin();
                std::advance(iterJ,1);
                std::advance(iterK,2);

                _tripletMatches[std::make_pair(I,J)][subTrack.descType].emplace_back(iterI->second, iterJ->second);
                _tripletMatches[std::make_pair(J,K)][subTrack.descType].emplace_back(iterJ->second, iterK->second);
                _tripletMatches[std::make_pair(I,K)][subTrack.descType].emplace_back(iterI->second, iterK->second);
              }
            }
          }
        }
      }
    }
  }
  // Clear putatives matches since they are no longer required
  matching::PairwiseMatches().swap(_putativeMatches);
}

/// Init & triangulate landmark observations from validated 3-view correspondences
void StructureEstimationFromKnownPoses::triangulate(
  SfMData& sfmData,
  const feature::RegionsPerView& regionsPerView, 
  std::mt19937 &randomNumberGenerator)
{
  track::TracksMap map_tracksCommon;
  track::TracksBuilder tracksBuilder;
  tracksBuilder.build(_tripletMatches);
  tracksBuilder.filter(true,3);
  tracksBuilder.exportToSTL(map_tracksCommon);
  matching::PairwiseMatches().swap(_tripletMatches);

  // Generate new Structure tracks
  sfmData.structure.clear();

  // Fill sfm_data with the computed tracks (no 3D yet)
  Landmarks & structure = sfmData.structure;
  IndexT idx(0);
  for (track::TracksMap::const_iterator itTracks = map_tracksCommon.begin();
    itTracks != map_tracksCommon.end();
    ++itTracks, ++idx)
  {
    const track::Track & track = itTracks->second;
    structure[idx] = Landmark(track.descType);
    Observations & observations = structure.at(idx).observations;
    for (auto it = track.featPerView.begin(); it != track.featPerView.end(); ++it)
    {
      const size_t imaIndex = it->first;
      const size_t featIndex = it->second;
      const feature::Regions& regions = regionsPerView.getRegions(imaIndex, track.descType);
      const feature::PointFeature& feat = regions.Features()[featIndex];

      observations[imaIndex] = Observation(feat.coords().cast<double>(), featIndex, feat.scale());
    }
  }

  // Triangulate them using a robust triangulation scheme
  StructureComputation_robust structure_estimator(true);
  structure_estimator.triangulate(sfmData, randomNumberGenerator);
}

} // namespace sfm
} // namespace aliceVision

