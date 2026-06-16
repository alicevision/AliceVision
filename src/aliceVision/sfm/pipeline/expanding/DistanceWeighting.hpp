// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/sfmData/SfMData.hpp>

namespace aliceVision {
namespace sfm {

/**
 * @brief Weight observations based on their spatial distribution in the image.
 * Points that are in sparse areas (far from their neighbors) get a higher weight,
 * while points in dense clusters get a lower weight. This helps to balance the
 * influence of features across the image.
 *
 * The weight is calculated based on the squared distance to the K-th nearest neighbor (neighboorRank),
 * clamped to a maximum area (ratioDefaultArea * averageFeatureArea).
 * Weights are normalized per view so that their mean is 1.0.
 *
 * @param[in,out] sfmData The SfM data containing views and landmarks to be weighted.
 * @param[in] neighboorRank The rank (K) of the neighbor to use for distance calculation.
 * @param[in] ratioDefaultArea Multiplier for clamping the maximum area to consider for a point.
 * @return True if successful.
 */
bool weightObservationsFromDistance(sfmData::SfMData & sfmData, size_t neighboorRank, double ratioDefaultArea);

}
}