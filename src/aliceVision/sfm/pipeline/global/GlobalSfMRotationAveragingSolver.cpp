// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "GlobalSfMRotationAveragingSolver.hpp"
#include <aliceVision/sfm/sfm.hpp>
#include <aliceVision/graph/graph.hpp>
#include <aliceVision/multiview/rotationAveraging/rotationAveraging.hpp>
#include <aliceVision/stl/mapUtils.hpp>

#include <aliceVision/utils/Histogram.hpp>

namespace aliceVision {
namespace sfm {

using namespace aliceVision::rotationAveraging;

PairSet GlobalSfMRotationAveragingSolver::GetUsedPairs() const
{
  return used_pairs;
}

bool GlobalSfMRotationAveragingSolver::Run(
  ERotationAveragingMethod eRotationAveragingMethod,
  ERelativeRotationInferenceMethod eRelativeRotationInferenceMethod,
  const RelativeRotations& relativeRot_In,
  const double max_angular_error,
  HashMap<IndexT, Mat3>& map_globalR
) const
{
  RelativeRotations relativeRotations = relativeRot_In;
  // We work on a copy, since inference can remove some relative motions

  ALICEVISION_LOG_DEBUG("GlobalSfMRotationAveragingSolver: A) relativeRotations.size(): " << relativeRotations.size());

  //-> Test there is only one graph and at least 3 camera?
  switch(eRelativeRotationInferenceMethod)
  {
    case(TRIPLET_ROTATION_INFERENCE_COMPOSITION_ERROR):
    {
      //-------------------
      // Triplet inference (test over the composition error)
      //-------------------
      PairSet pairs = getPairs(relativeRotations);
      ALICEVISION_LOG_DEBUG("GlobalSfMRotationAveragingSolver: pairs.size(): " << pairs.size());

      std::vector< graph::Triplet > vec_triplets = graph::tripletListing(pairs);
      ALICEVISION_LOG_DEBUG("GlobalSfMRotationAveragingSolver: vec_triplets.size(): " << vec_triplets.size());

      //-- Rejection triplet that are 'not' identity rotation (error to identity > max_angular_error)
      TripletRotationRejection(max_angular_error, vec_triplets, relativeRotations);

      pairs = getPairs(relativeRotations);
      const std::set<IndexT> set_remainingIds = graph::CleanGraph_KeepLargestBiEdge_Nodes<PairSet, IndexT>(pairs);
      if(set_remainingIds.empty())
        return false;
      KeepOnlyReferencedElement(set_remainingIds, relativeRotations);
      break;
    }
    default:
      ALICEVISION_LOG_WARNING(
        "Unknown relative rotation inference method: " << (int) eRelativeRotationInferenceMethod);
  }

  // Compute contiguous index (mapping between sparse index and contiguous index)
  //  from ranging in [min(Id), max(Id)] to  [0, nbCam]

  const PairSet pairs = getPairs(relativeRotations);
  HashMap<IndexT, IndexT> _reindexForward, _reindexBackward;
  reindex(pairs, _reindexForward, _reindexBackward);

  for(RelativeRotations::iterator iter = relativeRotations.begin();  iter != relativeRotations.end(); ++iter)
  {
    RelativeRotation & rel = *iter;
    rel.i = _reindexForward[rel.i];
    rel.j = _reindexForward[rel.j];
  }

  //- B. solve global rotation computation
  bool bSuccess = false;
  std::vector<Mat3> vec_globalR(_reindexForward.size());

  ALICEVISION_LOG_DEBUG("GlobalSfMRotationAveragingSolver: B) vec_globalR.size(): " << vec_globalR.size());

  switch(eRotationAveragingMethod)
  {
    case ROTATION_AVERAGING_L2:
    {
      //- Solve the global rotation estimation problem:
      bSuccess = rotationAveraging::l2::L2RotationAveraging(
        _reindexForward.size(),
        relativeRotations,
        vec_globalR);

      ALICEVISION_LOG_DEBUG("rotationAveraging::l2::L2RotationAveraging: success: " << bSuccess);
      //- Non linear refinement of the global rotations
      if (bSuccess)
      {
        bSuccess = rotationAveraging::l2::L2RotationAveraging_Refine(
          relativeRotations,
          vec_globalR);
        ALICEVISION_LOG_DEBUG("rotationAveraging::l2::L2RotationAveraging_Refine: success: " << bSuccess);
      }

      // save kept pairs (restore original pose indices using the backward reindexing)
      for(RelativeRotations::iterator iter = relativeRotations.begin();  iter != relativeRotations.end(); ++iter)
      {
        RelativeRotation & rel = *iter;
        rel.i = _reindexBackward[rel.i];
        rel.j = _reindexBackward[rel.j];
      }
      used_pairs = getPairs(relativeRotations);
    }
    break;
    case ROTATION_AVERAGING_L1:
    {
      using namespace aliceVision::rotationAveraging::l1;

      //- Solve the global rotation estimation problem:
      const size_t nMainViewID = 0; //arbitrary choice
      std::vector<bool> vec_inliers;
      bSuccess = rotationAveraging::l1::GlobalRotationsRobust(
        relativeRotations, vec_globalR, nMainViewID, 0.0f, &vec_inliers);

      ALICEVISION_LOG_DEBUG("rotationAveraging::l1::GlobalRotationsRobust: success: " << bSuccess);
      ALICEVISION_LOG_DEBUG("inliers: " << vec_inliers);

      // save kept pairs (restore original pose indices using the backward reindexing)
      for (size_t i = 0; i < vec_inliers.size(); ++i)
      {
        if (vec_inliers[i])
        {
          used_pairs.insert(
            Pair(_reindexBackward[relativeRotations[i].i],
                 _reindexBackward[relativeRotations[i].j]));
        }
      }
    }
    break;
    default:
      ALICEVISION_LOG_DEBUG(
        "Unknown rotation averaging method: " << (int) eRotationAveragingMethod);
  }

  if (bSuccess)
  {
    //-- Setup the averaged rotations
    for (size_t i = 0; i < vec_globalR.size(); ++i)  {
      map_globalR[_reindexBackward[i]] = vec_globalR[i];
    }
  }
  else{
    ALICEVISION_LOG_WARNING("Global rotation solving failed.");
  }

  return bSuccess;
}

/// Reject edges of the view graph that do not produce triplets with tiny
///  angular error once rotation composition have been computed.
void GlobalSfMRotationAveragingSolver::TripletRotationRejection(
  const double max_angular_error,
  std::vector< graph::Triplet > & vec_triplets,
  RelativeRotations & relativeRotations) const
{
  const size_t edges_start_count = relativeRotations.size();

  RelativeRotationsMap map_relatives = getMap(relativeRotations);
  RelativeRotationsMap map_relatives_validated;

  //--
  // ROTATION OUTLIERS DETECTION
  //--

  std::vector< graph::Triplet > vec_triplets_validated;
  vec_triplets_validated.reserve(vec_triplets.size());

  std::vector<float> vec_errToIdentityPerTriplet;
  vec_errToIdentityPerTriplet.reserve(vec_triplets.size());
  // Compute the composition error for each length 3 cycles
  for (size_t i = 0; i < vec_triplets.size(); ++i)
  {
    const graph::Triplet & triplet = vec_triplets[i];
    const IndexT I = triplet.i, J = triplet.j , K = triplet.k;

    ALICEVISION_LOG_DEBUG("GlobalSfMRotationAveragingSolver::TripletRotationRejection: i: " << i << ", (" << I << ", " << J << ", " << K << ").");

    //-- Find the three relative rotations
    const Pair ij(I,J), ji(J,I);
    const Mat3 RIJ = (map_relatives.count(ij)) ?
      map_relatives.at(ij).Rij : Mat3(map_relatives.at(ji).Rij.transpose());

    const Pair jk(J,K), kj(K,J);
    const Mat3 RJK = (map_relatives.count(jk)) ?
      map_relatives.at(jk).Rij : Mat3(map_relatives.at(kj).Rij.transpose());

    const Pair ki(K,I), ik(I,K);
    const Mat3 RKI = (map_relatives.count(ki)) ?
      map_relatives.at(ki).Rij : Mat3(map_relatives.at(ik).Rij.transpose());

    const Mat3 Rot_To_Identity = RIJ * RJK * RKI; // motion composition
    const float angularErrorDegree = static_cast<float>(radianToDegree(getRotationMagnitude(Rot_To_Identity)));
    vec_errToIdentityPerTriplet.push_back(angularErrorDegree);

    if (angularErrorDegree < max_angular_error)
    {
      vec_triplets_validated.push_back(triplet);

      if (map_relatives.count(ij))
        map_relatives_validated[ij] = map_relatives.at(ij);
      else
        map_relatives_validated[ji] = map_relatives.at(ji);

      if (map_relatives.count(jk))
        map_relatives_validated[jk] = map_relatives.at(jk);
      else
        map_relatives_validated[kj] = map_relatives.at(kj);

      if (map_relatives.count(ki))
        map_relatives_validated[ki] = map_relatives.at(ki);
      else
        map_relatives_validated[ik] = map_relatives.at(ik);
    }
    else
    {
      ALICEVISION_LOG_DEBUG("GlobalSfMRotationAveragingSolver::TripletRotationRejection: i: " << i << ", angularErrorDegree: " << angularErrorDegree << ", max_angular_error: " << max_angular_error);
    }
  }
  map_relatives = std::move(map_relatives_validated);

  // update to keep only useful triplets
  relativeRotations.clear();
  relativeRotations.reserve(map_relatives.size());
  std::transform(map_relatives.begin(), map_relatives.end(), std::back_inserter(relativeRotations), stl::RetrieveValue());
  std::transform(map_relatives.begin(), map_relatives.end(), std::inserter(used_pairs, used_pairs.begin()), stl::RetrieveKey());

  // Display statistics about rotation triplets error:
  ALICEVISION_LOG_DEBUG("Statistics about rotation triplets:");
  ALICEVISION_LOG_DEBUG(BoxStats<float>(vec_errToIdentityPerTriplet.begin(), vec_errToIdentityPerTriplet.end()));

  std::sort(vec_errToIdentityPerTriplet.begin(), vec_errToIdentityPerTriplet.end());

  if (!vec_errToIdentityPerTriplet.empty())
  {
    utils::Histogram<float> histo(0.0f, *max_element(vec_errToIdentityPerTriplet.begin(), vec_errToIdentityPerTriplet.end()), 20);
    histo.Add(vec_errToIdentityPerTriplet.begin(), vec_errToIdentityPerTriplet.end());
    ALICEVISION_LOG_DEBUG(histo.ToString());
  }

  {
    ALICEVISION_LOG_DEBUG("Triplets filtering based on composition error on unit cycles");
    ALICEVISION_LOG_DEBUG(
      "#Triplets before: " << vec_triplets.size() << "\n"
      "#Triplets after: " << vec_triplets_validated.size());
  }

  vec_triplets = std::move(vec_triplets_validated);

  const size_t edges_end_count = relativeRotations.size();
  ALICEVISION_LOG_DEBUG("#Edges removed by triplet inference: " << edges_start_count - edges_end_count);
}

} // namespace sfm
} // namespace aliceVision

