// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/types.hpp>

#include <cstdint>
#include <vector>

namespace aliceVision {
namespace sfmData {

/**
 * @brief Compact landmark/observation table suitable for contiguous serialization.
 */
struct LandmarkTable
{
    // Per landmark arrays (size: N)
    std::vector<IndexT> ids;
    std::vector<Vec3> points;
    std::vector<Vec3> colors;
    std::vector<std::uint8_t> descTypes;
    std::vector<std::uint8_t> flags;
    std::vector<IndexT> referenceViewIds;

    // CSR over observations: landmark i owns [observationOffsets[i], observationOffsets[i+1])
    std::vector<IndexT> observationOffsets;

    // Per observation arrays (size: M)
    std::vector<IndexT> observationViewIds;
    std::vector<IndexT> observationFeatureIds;
    std::vector<Vec2> observationXY;
    std::vector<float> observationScales;
    std::vector<float> observationDepths;

    // Optional inverse index by view
    std::vector<IndexT> viewIds;
    std::vector<IndexT> viewObservationOffsets;
    std::vector<IndexT> viewObservationLandmarkIndices;
    std::vector<IndexT> viewObservationIndices;
};

/**
 * @brief Build a compact landmark table from SfMData.
 * @param sfmData The source scene.
 * @param buildViewIndex Build optional inverse index by view.
 */
LandmarkTable buildLandmarkTable(const SfMData& sfmData, bool buildViewIndex = false);

}  // namespace sfmData
}  // namespace aliceVision
