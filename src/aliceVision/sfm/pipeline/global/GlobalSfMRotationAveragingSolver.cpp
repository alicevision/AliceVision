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

PairSet GlobalSfMRotationAveragingSolver::getUsedPairs() const { return usedPairs; }

bool GlobalSfMRotationAveragingSolver::run(ERotationAveragingMethod eRotationAveragingMethod,
                                           ERelativeRotationInferenceMethod eRelativeRotationInferenceMethod,
                                           const RelativeRotations& relativeRotIn,
                                           const double maxAngularError,
                                           std::map<IndexT, Mat3>& mapGlobalR) const
{
    RelativeRotations relativeRotations = relativeRotIn;
    // We work on a copy, since inference can remove some relative motions

    ALICEVISION_LOG_DEBUG("GlobalSfMRotationAveragingSolver: A) relativeRotations.size(): " << relativeRotations.size());

    //-> Test there is only one graph and at least 3 camera?
    switch (eRelativeRotationInferenceMethod)
    {
        case (TRIPLET_ROTATION_INFERENCE_COMPOSITION_ERROR):
        {
            //-------------------
            // Triplet inference (test over the composition error)
            //-------------------
            PairSet pairs = getPairs(relativeRotations);
            ALICEVISION_LOG_DEBUG("GlobalSfMRotationAveragingSolver: pairs.size(): " << pairs.size());

            std::vector<graph::Triplet> vecTriplets = graph::tripletListing(pairs);
            ALICEVISION_LOG_DEBUG("GlobalSfMRotationAveragingSolver: vecTriplets.size(): " << vecTriplets.size());

            //-- Rejection triplet that are 'not' identity rotation (error to identity > maxAngularError)
            tripletRotationRejection(maxAngularError, vecTriplets, relativeRotations);

            pairs = getPairs(relativeRotations);
            const std::set<IndexT> setRemainingIds = graph::CleanGraph_KeepLargestBiEdge_Nodes<PairSet, IndexT>(pairs);
            if (setRemainingIds.empty())
                return false;
            keepOnlyReferencedElement(setRemainingIds, relativeRotations);
            break;
        }
        default:
            ALICEVISION_LOG_WARNING("Unknown relative rotation inference method: " << (int)eRelativeRotationInferenceMethod);
    }

    // Compute contiguous index (mapping between sparse index and contiguous index)
    //  from ranging in [min(Id), max(Id)] to  [0, nbCam]

    const PairSet pairs = getPairs(relativeRotations);
    std::map<IndexT, IndexT> _reindexForward, _reindexBackward;
    reindex(pairs, _reindexForward, _reindexBackward);

    for (RelativeRotations::iterator iter = relativeRotations.begin(); iter != relativeRotations.end(); ++iter)
    {
        RelativeRotation& rel = *iter;
        rel.i = _reindexForward[rel.i];
        rel.j = _reindexForward[rel.j];
    }

    //- B. solve global rotation computation
    bool bSuccess = false;
    std::vector<Mat3> vecGlobalR(_reindexForward.size());

    ALICEVISION_LOG_DEBUG("GlobalSfMRotationAveragingSolver: B) vecGlobalR.size(): " << vecGlobalR.size());

    switch (eRotationAveragingMethod)
    {
        case ROTATION_AVERAGING_L2:
        {
            //- Solve the global rotation estimation problem:
            bSuccess = rotationAveraging::l2::L2RotationAveraging(_reindexForward.size(), relativeRotations, vecGlobalR);

            ALICEVISION_LOG_DEBUG("rotationAveraging::l2::L2RotationAveraging: success: " << bSuccess);
            //- Non linear refinement of the global rotations
            if (bSuccess)
            {
                bSuccess = rotationAveraging::l2::L2RotationAveraging_Refine(relativeRotations, vecGlobalR);
                ALICEVISION_LOG_DEBUG("rotationAveraging::l2::L2RotationAveraging_Refine: success: " << bSuccess);
            }

            // save kept pairs (restore original pose indices using the backward reindexing)
            for (RelativeRotations::iterator iter = relativeRotations.begin(); iter != relativeRotations.end(); ++iter)
            {
                RelativeRotation& rel = *iter;
                rel.i = _reindexBackward[rel.i];
                rel.j = _reindexBackward[rel.j];
            }
            usedPairs = getPairs(relativeRotations);
        }
        break;
        case ROTATION_AVERAGING_L1:
        {
            using namespace aliceVision::rotationAveraging::l1;

            //- Solve the global rotation estimation problem:
            const size_t nMainViewID = 0;  // arbitrary choice
            std::vector<bool> vecInliers;
            bSuccess = rotationAveraging::l1::GlobalRotationsRobust(relativeRotations, vecGlobalR, nMainViewID, 0.0f, &vecInliers);

            ALICEVISION_LOG_DEBUG("rotationAveraging::l1::GlobalRotationsRobust: success: " << bSuccess);
            ALICEVISION_LOG_DEBUG("inliers: " << vecInliers);

            // save kept pairs (restore original pose indices using the backward reindexing)
            for (size_t i = 0; i < vecInliers.size(); ++i)
            {
                if (vecInliers[i])
                {
                    usedPairs.insert(Pair(_reindexBackward[relativeRotations[i].i], _reindexBackward[relativeRotations[i].j]));
                }
            }
        }
        break;
        default:
            ALICEVISION_LOG_DEBUG("Unknown rotation averaging method: " << (int)eRotationAveragingMethod);
    }

    if (bSuccess)
    {
        //-- Setup the averaged rotations
        for (size_t i = 0; i < vecGlobalR.size(); ++i)
        {
            mapGlobalR[_reindexBackward[i]] = vecGlobalR[i];
        }
    }
    else
    {
        ALICEVISION_LOG_WARNING("Global rotation solving failed.");
    }

    return bSuccess;
}

/// Reject edges of the view graph that do not produce triplets with tiny
///  angular error once rotation composition have been computed.
void GlobalSfMRotationAveragingSolver::tripletRotationRejection(const double maxAngularError,
                                                                std::vector<graph::Triplet>& vecTriplets,
                                                                RelativeRotations& relativeRotations) const
{
    const size_t edgesStartCount = relativeRotations.size();

    RelativeRotationsMap mapRelatives = getMap(relativeRotations);
    RelativeRotationsMap mapRelativesValidated;

    //--
    // ROTATION OUTLIERS DETECTION
    //--

    std::vector<graph::Triplet> vecTripletsValidated;
    vecTripletsValidated.reserve(vecTriplets.size());

    std::vector<float> vecErrToIdentityPerTriplet;
    vecErrToIdentityPerTriplet.reserve(vecTriplets.size());
    // Compute the composition error for each length 3 cycles
    for (size_t i = 0; i < vecTriplets.size(); ++i)
    {
        const graph::Triplet& triplet = vecTriplets[i];
        const IndexT I = triplet.i, J = triplet.j, K = triplet.k;

        ALICEVISION_LOG_DEBUG("GlobalSfMRotationAveragingSolver::tripletRotationRejection: i: " << i << ", (" << I << ", " << J << ", " << K << ").");

        //-- Find the three relative rotations
        const Pair ij(I, J), ji(J, I);
        const Mat3 RIJ = (mapRelatives.count(ij)) ? mapRelatives.at(ij).Rij : Mat3(mapRelatives.at(ji).Rij.transpose());

        const Pair jk(J, K), kj(K, J);
        const Mat3 RJK = (mapRelatives.count(jk)) ? mapRelatives.at(jk).Rij : Mat3(mapRelatives.at(kj).Rij.transpose());

        const Pair ki(K, I), ik(I, K);
        const Mat3 RKI = (mapRelatives.count(ki)) ? mapRelatives.at(ki).Rij : Mat3(mapRelatives.at(ik).Rij.transpose());

        const Mat3 RotToIdentity = RIJ * RJK * RKI;  // motion composition
        const float angularErrorDegree = static_cast<float>(radianToDegree(getRotationMagnitude(RotToIdentity)));
        vecErrToIdentityPerTriplet.push_back(angularErrorDegree);

        if (angularErrorDegree < maxAngularError)
        {
            vecTripletsValidated.push_back(triplet);

            if (mapRelatives.count(ij))
                mapRelativesValidated[ij] = mapRelatives.at(ij);
            else
                mapRelativesValidated[ji] = mapRelatives.at(ji);

            if (mapRelatives.count(jk))
                mapRelativesValidated[jk] = mapRelatives.at(jk);
            else
                mapRelativesValidated[kj] = mapRelatives.at(kj);

            if (mapRelatives.count(ki))
                mapRelativesValidated[ki] = mapRelatives.at(ki);
            else
                mapRelativesValidated[ik] = mapRelatives.at(ik);
        }
        else
        {
            ALICEVISION_LOG_DEBUG("GlobalSfMRotationAveragingSolver::tripletRotationRejection: i: "
                                  << i << ", angularErrorDegree: " << angularErrorDegree << ", maxAngularError: " << maxAngularError);
        }
    }
    mapRelatives = std::move(mapRelativesValidated);

    // update to keep only useful triplets
    relativeRotations.clear();
    relativeRotations.reserve(mapRelatives.size());
    std::transform(mapRelatives.begin(), mapRelatives.end(), std::back_inserter(relativeRotations), stl::RetrieveValue());
    std::transform(mapRelatives.begin(), mapRelatives.end(), std::inserter(usedPairs, usedPairs.begin()), stl::RetrieveKey());

    // Display statistics about rotation triplets error:
    ALICEVISION_LOG_DEBUG("Statistics about rotation triplets:");
    ALICEVISION_LOG_DEBUG(BoxStats<float>(vecErrToIdentityPerTriplet.begin(), vecErrToIdentityPerTriplet.end()));

    std::sort(vecErrToIdentityPerTriplet.begin(), vecErrToIdentityPerTriplet.end());

    if (!vecErrToIdentityPerTriplet.empty())
    {
        utils::Histogram<float> histo(0.0f, *max_element(vecErrToIdentityPerTriplet.begin(), vecErrToIdentityPerTriplet.end()), 20);
        histo.Add(vecErrToIdentityPerTriplet.begin(), vecErrToIdentityPerTriplet.end());
        ALICEVISION_LOG_DEBUG(histo.ToString());
    }

    {
        ALICEVISION_LOG_DEBUG("Triplets filtering based on composition error on unit cycles");
        ALICEVISION_LOG_DEBUG("#Triplets before: " << vecTriplets.size()
                                                   << "\n"
                                                      "#Triplets after: "
                                                   << vecTripletsValidated.size());
    }

    vecTriplets = std::move(vecTripletsValidated);

    const size_t edgesEndCount = relativeRotations.size();
    ALICEVISION_LOG_DEBUG("#Edges removed by triplet inference: " << edgesStartCount - edgesEndCount);
}

}  // namespace sfm
}  // namespace aliceVision
