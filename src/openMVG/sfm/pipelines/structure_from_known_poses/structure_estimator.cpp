// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#include "aliceVision/sfm/pipelines/structure_from_known_poses/structure_estimator.hpp"

#include "aliceVision/matching/indMatch.hpp"
#include "aliceVision/matching/metric.hpp"
#include "aliceVision/robust_estimation/guided_matching.hpp"
#include "aliceVision/multiview/solver_fundamental_kernel.hpp"
#include "aliceVision/multiview/triangulation_nview.hpp"
#include "aliceVision/graph/graph.hpp"
#include "aliceVision/tracks/tracks.hpp"
#include "aliceVision/sfm/sfm_data_triangulation.hpp"
#include <aliceVision/config.hpp>

#include "third_party/progress/progress.hpp"

namespace aliceVision {
namespace sfm {

using namespace cameras;
using namespace features;
using namespace geometry;


/// Camera pair epipole (Projection of camera center 2 in the image plane 1)
static Vec3 epipole_from_P(const Mat34& P1, const Pose3& P2)
{
  const Vec3 c = P2.center();
  Vec4 center;
  center << c(0), c(1), c(2), 1.0;
  return P1*center;
}

/// Export point feature based vector to a matrix [(x,y)'T, (x,y)'T]
/// Use the camera intrinsics in order to get undistorted pixel coordinates
template<typename MatT >
static void PointsToMat(
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
void SfM_Data_Structure_Estimation_From_Known_Poses::run(
  SfM_Data & sfm_data,
  const Pair_Set & pairs,
  const features::RegionsPerView& regionsPerView)
{
  sfm_data.structure.clear();

  match(sfm_data, pairs, regionsPerView);
  filter(sfm_data, pairs, regionsPerView);
  triangulate(sfm_data, regionsPerView);
}

/// Use guided matching to find corresponding 2-view correspondences
void SfM_Data_Structure_Estimation_From_Known_Poses::match(
  const SfM_Data & sfm_data,
  const Pair_Set & pairs,
  const features::RegionsPerView& regionsPerView)
{
  C_Progress_display my_progress_bar( pairs.size(), std::cout,
    "Compute pairwise fundamental guided matching:\n" );

  #pragma omp parallel
  for (Pair_Set::const_iterator it = pairs.begin(); it != pairs.end(); ++it)
  {
    #pragma omp single nowait
    {
    // --
    // Perform GUIDED MATCHING
    // --
    // Use the computed model to check valid correspondences
    // - by considering geometric error and descriptor distance ratio.

    const View * viewL = sfm_data.GetViews().at(it->first).get();
    const Pose3 poseL = sfm_data.getPose(*viewL);
    const Intrinsics::const_iterator iterIntrinsicL = sfm_data.GetIntrinsics().find(viewL->getIntrinsicId());
    const View * viewR = sfm_data.GetViews().at(it->second).get();
    const Pose3 poseR = sfm_data.getPose(*viewR);
    const Intrinsics::const_iterator iterIntrinsicR = sfm_data.GetIntrinsics().find(viewR->getIntrinsicId());

    if (sfm_data.GetIntrinsics().count(viewL->getIntrinsicId()) != 0 ||
        sfm_data.GetIntrinsics().count(viewR->getIntrinsicId()) != 0)
    {
      const Mat34 P_L = iterIntrinsicL->second.get()->get_projective_equivalent(poseL);
      const Mat34 P_R = iterIntrinsicR->second.get()->get_projective_equivalent(poseR);

      const Mat3 F_lr = F_from_P(P_L, P_R);
      const double thresholdF = 4.0;
      std::vector<features::EImageDescriberType> commonDescTypes = regionsPerView.getCommonDescTypes(*it);
      
      matching::MatchesPerDescType allImagePairMatches;
      for(features::EImageDescriberType descType: commonDescTypes)
      {
        std::vector<matching::IndMatch> matches;
      #ifdef EXHAUSTIVE_MATCHING
        geometry_aware::GuidedMatching
          <Mat3, fundamental::kernel::EpipolarDistanceError>
          (
            F_lr,
            iterIntrinsicL->second.get(),
            regionsPerView.getRegions(it->first),
            iterIntrinsicR->second.get(),
            regionsPerView.getRegions(it->second),
            descType,
            Square(thresholdF), Square(0.8),
            matches
          );
      #else
        const Vec3 epipole2  = epipole_from_P(P_R, poseL);

        //const features::Regions& regions = regionsPerView.getRegions(it->first);
        geometry_aware::GuidedMatching_Fundamental_Fast
          <fundamental::kernel::EpipolarDistanceError>
          (
            F_lr,
            epipole2,
            iterIntrinsicL->second.get(),
            regionsPerView.getRegions(it->first, descType),
            iterIntrinsicR->second.get(),
            regionsPerView.getRegions(it->second, descType),
            iterIntrinsicR->second->w(), iterIntrinsicR->second->h(),
            //descType,
            Square(thresholdF), Square(0.8),
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
void SfM_Data_Structure_Estimation_From_Known_Poses::filter(
  const SfM_Data & sfm_data,
  const Pair_Set & pairs,
  const features::RegionsPerView& regionsPerView)
{
  // Compute triplets
  // Triangulate triplet tracks
  //  - keep valid one

  typedef std::vector< graph::Triplet > Triplets;
  const Triplets triplets = graph::tripletListing(pairs);

  C_Progress_display my_progress_bar( triplets.size(), std::cout,
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

      tracks::TracksMap map_tracksCommon;
      tracks::TracksBuilder tracksBuilder;
      {
        matching::PairwiseMatches map_matchesIJK;
        if (_putativeMatches.count(std::make_pair(I,J)))
          map_matchesIJK.insert(*_putativeMatches.find(std::make_pair(I,J)));

        if (_putativeMatches.count(std::make_pair(I,K)))
          map_matchesIJK.insert(*_putativeMatches.find(std::make_pair(I,K)));

        if (_putativeMatches.count(std::make_pair(J,K)))
          map_matchesIJK.insert(*_putativeMatches.find(std::make_pair(J,K)));

        if (map_matchesIJK.size() >= 2) {
          tracksBuilder.Build(map_matchesIJK);
          tracksBuilder.Filter(3, false);
          tracksBuilder.ExportToSTL(map_tracksCommon);
        }

        // Triangulate the tracks
        for (tracks::TracksMap::const_iterator iterTracks = map_tracksCommon.begin();
          iterTracks != map_tracksCommon.end(); ++iterTracks) {
          {
            const tracks::Track & subTrack = iterTracks->second;
            Triangulation trianObj;
            for (auto iter = subTrack.featPerView.begin(); iter != subTrack.featPerView.end(); ++iter)
            {
              const size_t imaIndex = iter->first;
              const size_t featIndex = iter->second;
              const View * view = sfm_data.GetViews().at(imaIndex).get();
              const IntrinsicBase * cam = sfm_data.GetIntrinsics().at(view->getIntrinsicId()).get();
              const Pose3 pose = sfm_data.getPose(*view);
              const Vec2 pt = regionsPerView.getRegions(imaIndex, subTrack.descType).GetRegionPosition(featIndex);
              trianObj.add(cam->get_projective_equivalent(pose), cam->get_ud_pixel(pt));
            }
            const Vec3 Xs = trianObj.compute();
            if (trianObj.minDepth() > 0 && trianObj.error()/(double)trianObj.size() < 4.0)
            // TODO: Add an angular check ?
            {
              #pragma omp critical
              {
                tracks::Track::FeatureIdPerView::const_iterator iterI, iterJ, iterK;
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
void SfM_Data_Structure_Estimation_From_Known_Poses::triangulate(
  SfM_Data & sfm_data,
  const features::RegionsPerView& regionsPerView)
{
  tracks::TracksMap map_tracksCommon;
  tracks::TracksBuilder tracksBuilder;
  tracksBuilder.Build(_tripletMatches);
  tracksBuilder.Filter(3);
  tracksBuilder.ExportToSTL(map_tracksCommon);
  matching::PairwiseMatches().swap(_tripletMatches);

  // Generate new Structure tracks
  sfm_data.structure.clear();

  // Fill sfm_data with the computed tracks (no 3D yet)
  Landmarks & structure = sfm_data.structure;
  IndexT idx(0);
  for (tracks::TracksMap::const_iterator itTracks = map_tracksCommon.begin();
    itTracks != map_tracksCommon.end();
    ++itTracks, ++idx)
  {
    const tracks::Track & track = itTracks->second;
    structure[idx] = Landmark(track.descType);
    Observations & observations = structure.at(idx).observations;
    for (auto it = track.featPerView.begin(); it != track.featPerView.end(); ++it)
    {
      const size_t imaIndex = it->first;
      const size_t featIndex = it->second;
      const Vec2 pt = regionsPerView.getRegions(imaIndex, track.descType).GetRegionPosition(featIndex);
      observations[imaIndex] = Observation(pt, featIndex);
    }
  }

  // Triangulate them using a robust triangulation scheme
  SfM_Data_Structure_Computation_Robust structure_estimator(true);
  structure_estimator.triangulate(sfm_data);
}

} // namespace sfm
} // namespace aliceVision

