// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/types.hpp>
#include <aliceVision/sfm/bundle/BundleAdjustment.hpp>

namespace aliceVision {

namespace sfmData {
class SfMData;
}  // namespace sfmData

namespace sfm {

/// Filter a list of pair: Keep only the pair that are defined in index list
template<typename IterablePairs, typename IterableIndex>
inline PairSet filterPairs(const IterablePairs& pairs, const IterableIndex& index)
{
    PairSet keptPairs;
    for (auto& it : pairs)
    {
        if (index.count(it.first) > 0 && index.count(it.second) > 0)
            keptPairs.insert(it);
    }
    return keptPairs;
}

/// Remove observations with too large reprojection error.
/// Return the number of removed tracks.
IndexT removeOutliersWithPixelResidualError(sfmData::SfMData& sfmData,
                                            EFeatureConstraint featureConstraint,
                                            const double dThresholdPixel,
                                            const unsigned int minTrackLength = 2);

// Remove tracks that have a small angle (tracks with tiny angle leads to instable 3D points)
// Return the number of removed tracks
IndexT removeOutliersWithAngleError(sfmData::SfMData& sfmData, const double dMinAcceptedAngle);

bool eraseUnstablePoses(sfmData::SfMData& sfmData, const IndexT minPointsPerPose, std::set<IndexT>* outRemovedViewsId = NULL);

bool eraseObservationsWithMissingPoses(sfmData::SfMData& sfmData, const IndexT minPointsPerLandmark);

/// Remove unstable content from analysis of the sfm_data structure
bool eraseUnstablePosesAndObservations(sfmData::SfMData& sfmData,
                                       const IndexT minPointsPerPose = 6,
                                       const IndexT minPointsPerLandmark = 2,
                                       std::set<IndexT>* outRemovedViewsId = NULL);

}  // namespace sfm
}  // namespace aliceVision
