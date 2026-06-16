// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/LandmarkTable.hpp>

#include <aliceVision/image/pixelTypes.hpp>

#include <map>
#include <utility>
#include <vector>

namespace aliceVision {
namespace sfmData {

namespace {

constexpr std::uint8_t kParallaxRobustBit = 1u << 0;
constexpr std::uint8_t kLockedBit = 1u << 1;

}  // namespace

LandmarkTable buildLandmarkTable(const SfMData& sfmData, bool buildViewIndex)
{
    LandmarkTable table;

    const std::size_t landmarkCount = sfmData.getLandmarks().size();
    table.ids.reserve(landmarkCount);
    table.points.reserve(landmarkCount);
    table.colors.reserve(landmarkCount);
    table.descTypes.reserve(landmarkCount);
    table.flags.reserve(landmarkCount);
    table.referenceViewIds.reserve(landmarkCount);
    table.observationOffsets.reserve(landmarkCount + 1);

    std::size_t totalObservationCount = 0;
    for (const auto& [id, landmark] : sfmData.getLandmarks())
    {
        (void)id;
        totalObservationCount += landmark.getObservations().size();
    }

    table.observationViewIds.reserve(totalObservationCount);
    table.observationFeatureIds.reserve(totalObservationCount);
    table.observationXY.reserve(totalObservationCount);
    table.observationScales.reserve(totalObservationCount);
    table.observationDepths.reserve(totalObservationCount);

    std::map<IndexT, std::vector<std::pair<IndexT, IndexT>>> viewToObservations;

    table.observationOffsets.push_back(0);

    IndexT landmarkLinearIndex = 0;
    for (const auto& [landmarkId, landmark] : sfmData.getLandmarks())
    {
        table.ids.push_back(landmarkId);
        table.points.push_back(landmark.getX());

        Vec3 rgb = Vec3::Zero();
        const auto& c = landmark.getRgb();
        rgb(0) = static_cast<float>(c.r()) / 255.0f;
        rgb(1) = static_cast<float>(c.g()) / 255.0f;
        rgb(2) = static_cast<float>(c.b()) / 255.0f;
        table.colors.push_back(rgb);

        table.descTypes.push_back(static_cast<std::uint8_t>(landmark.getDescType()));

        std::uint8_t packedFlags = 0;
        if (landmark.isParallaxRobust())
        {
            packedFlags |= kParallaxRobustBit;
        }
        if (landmark.isLocked())
        {
            packedFlags |= kLockedBit;
        }
        table.flags.push_back(packedFlags);
        table.referenceViewIds.push_back(landmark.getReferenceViewIndex());

        for (const auto& [viewId, observation] : landmark.getObservations())
        {
            table.observationViewIds.push_back(viewId);
            table.observationFeatureIds.push_back(observation.getFeatureId());
            table.observationXY.push_back(observation.getCoordinates());
            table.observationScales.push_back(static_cast<float>(observation.getScale()));
            table.observationDepths.push_back(static_cast<float>(observation.getDepth()));

            if (buildViewIndex)
            {
                const IndexT observationIndex = static_cast<IndexT>(table.observationViewIds.size() - 1);
                viewToObservations[viewId].emplace_back(landmarkLinearIndex, observationIndex);
            }
        }

        table.observationOffsets.push_back(static_cast<IndexT>(table.observationViewIds.size()));
        ++landmarkLinearIndex;
    }

    if (buildViewIndex)
    {
        table.viewIds.reserve(viewToObservations.size());
        table.viewObservationOffsets.reserve(viewToObservations.size() + 1);
        table.viewObservationLandmarkIndices.reserve(totalObservationCount);
        table.viewObservationIndices.reserve(totalObservationCount);

        table.viewObservationOffsets.push_back(0);
        for (const auto& [viewId, pairs] : viewToObservations)
        {
            table.viewIds.push_back(viewId);
            for (const auto& [landmarkIndex, observationIndex] : pairs)
            {
                table.viewObservationLandmarkIndices.push_back(landmarkIndex);
                table.viewObservationIndices.push_back(observationIndex);
            }
            table.viewObservationOffsets.push_back(static_cast<IndexT>(table.viewObservationIndices.size()));
        }
    }

    return table;
}

}  // namespace sfmData
}  // namespace aliceVision
