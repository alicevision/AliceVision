// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/types.hpp>
#include <aliceVision/track/TracksHandler.hpp>
#include <aliceVision/sfmData/SfMData.hpp>

namespace aliceVision {
namespace sfm {

class ExpansionPolicy
{
public:
    using uptr = std::unique_ptr<ExpansionPolicy>;
public:

    /**
     * @brief Initialize policy for an iteration
     * @param sfmData the scene to process
     * @return true if the init succeeded
    */
    virtual bool initialize(const sfmData::SfMData & sfmData) = 0;

    /**
     * @brief compute policy for an iteration
     * @param sfmData the scene to process
     * @param tracksHandler the tracks for this scene
     * @return true if the policy succeeded
    */
    virtual bool process(const sfmData::SfMData & sfmData, const track::TracksHandler & tracksHandler) = 0;
    
    /**
     * @brief Retrieve the selected next views
     * @return a set of views to process
    */
    virtual std::set<IndexT> getNextViews() = 0;
};

}
}