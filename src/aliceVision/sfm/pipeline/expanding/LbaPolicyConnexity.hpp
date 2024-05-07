// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/sfm/pipeline/expanding/LbaPolicy.hpp>
#include <aliceVision/sfm/pipeline/expanding/ConnexityGraph.hpp>

namespace aliceVision {
namespace sfm {

class LbaPolicyConnexity : public LbaPolicy
{
public:
    using uptr = std::unique_ptr<LbaPolicy>;
    
public:
    /**
     * @brief Build the policy using a scene
     * @param sfmData the scene to process
     * @param tracksHandler the tracks for this scene
     * @param views the list of views of interest
    */
    virtual bool build(sfmData::SfMData & sfmData, const track::TracksHandler & tracksHandler, const std::set<IndexT> & viewIds) override;

private:
    void upgradeSfmData(sfmData::SfMData & sfmData, const ConnexityGraph & graph);
    void setupIntrinsics(sfmData::SfMData & sfmData);

private:
    int _distanceLimit = 1;
};

}
}