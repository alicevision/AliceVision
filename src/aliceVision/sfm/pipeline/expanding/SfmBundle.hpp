// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/types.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfm/bundle/BundleAdjustment.hpp>
#include <aliceVision/track/TracksHandler.hpp>
#include <aliceVision/sfm/pipeline/expanding/LbaPolicy.hpp>
#include <aliceVision/sfm/pipeline/expanding/ExpansionHistory.hpp>

namespace aliceVision {
namespace sfm {

class SfmBundle
{
public:
    using uptr = std::unique_ptr<SfmBundle>;
    
public:
    /**
     * @brief Process bundle
     * @param sfmData the scene description to optimize
     * @param tracksHandler the tracks manager
     * @param viewIds the set of views to bundle
     * @return false if an error occured
    */
    bool process(sfmData::SfMData & sfmData, const track::TracksHandler & tracksHandler, const std::set<IndexT> & viewIds);

    /**
     * brief setup the expansion chunk handler
     * @param expansionChunk a unique ptr. Ownership will be taken
     */
    void setLbaPolicyHandler(LbaPolicy::uptr & lbaPolicy)
    {
        _lbaPolicy = std::move(lbaPolicy);
    }


private:
    /**
     * Initialize bundle properties
    */
    bool initialize(sfmData::SfMData & sfmData, const track::TracksHandler & tracksHandler, const std::set<IndexT> & viewIds);

    /**
     * Cleanup sfmData 
     * @param sfmData the scene to clean
     * @return true if enough change occured during the cleaning
    */
    bool cleanup(sfmData::SfMData & sfmData);

private:
    LbaPolicy::uptr _lbaPolicy;

private:

    EFeatureConstraint _featureConstraint = EFeatureConstraint::SCALE;
    double _maxReprojectionError = 40.0;
    double _minAngleForLandmark = 2.0;
    size_t _minTrackLength = 2;
    size_t _minPointsPerPose = 30;
    size_t _bundleAdjustmentMaxOutlier = 50;
    size_t _minNbCamerasToRefinePrincipalPoint = 3;
    bool _useLBA = true;
    size_t _minNbCamerasLBA = 100;
    size_t _LBAGraphDistanceLimit = 1;
    size_t _LBAMinNbOfMatches = 50;
};

} // namespace sfm
} // namespace aliceVision