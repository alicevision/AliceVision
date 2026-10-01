// This file is part of the AliceVision project.
// Copyright (c) 2027 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/sfmData/SfMData.hpp>
#include <array>

namespace aliceVision
{
namespace sfm
{

/**
 * @brief Select a well-conditioned triplet of landmarks to remove the gauge freedom
 *        before computing the BA Jacobian / covariance.
 *
 * Bundle adjustment has a 7-DOF gauge freedom (3 translation + 3 rotation + 1 scale).
 * Fixing the 3D position of three non-collinear landmarks over-constrains these 7 DOF
 * (9 fixed values) and makes the Hessian / information matrix full-rank.
 *
 * The selection maximises the minimum altitude of the candidate triangle:
 *
 * @f[
 *   \text{score} = \frac{\|(p_2 - p_1) \times (p_3 - p_1)\|}
 *                       {\max(\|p_2{-}p_1\|,\, \|p_3{-}p_1\|,\, \|p_3{-}p_2\|)}
 *                 \;\propto\; h_{\min}
 * @f]
 *
 * This metric simultaneously rewards large spread (large triangle area in the numerator)
 * and penalises near-collinear configurations (large longest side in the denominator
 * produces a small score when the triangle is a sliver). A score of 0 corresponds to
 * exactly collinear points, which would leave at least one rotation DOF unresolved.
 *
 * The search is performed by random sampling: @p nbSamples candidate triplets are drawn
 * uniformly at random from the landmark set and the highest-scoring one is returned.
 * More samples reduce the probability of missing a well-conditioned triplet at the cost
 * of a proportionally longer search.
 *
 * @param[in]  sfmData      The reconstruction whose landmarks are searched.
 * @param[out] selectedSet  The three landmark IDs forming the best triplet found.
 * @param[in]  nbSamples    Number of random triplets to evaluate (default: 100,000).
 *
 * @return true if a valid triplet was found (i.e. the scene has at least 3 landmarks);
 *         false otherwise.
 */
bool selectTripletForGaugeRemoval(const sfmData::SfMData& sfmData, std::array<IndexT, 3>& selectedSet, int nbSamples = 100000);

}
}
