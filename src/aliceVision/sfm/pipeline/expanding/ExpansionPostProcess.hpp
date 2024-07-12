// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/types.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/track/TracksHandler.hpp>

namespace aliceVision {
namespace sfm {

class ExpansionPostProcess
{
public:
    using uptr = std::unique_ptr<ExpansionPostProcess>;

public:

    /**
     * @brief Perform post process for an iteration
     * @param sfmData the scene to process
     * @param tracksHandler the tracks for this scene
     * @return true if the process succeeded
    */
    virtual bool process(sfmData::SfMData & sfmData, track::TracksHandler & tracksHandler) = 0;

    /**
     * @brief get updated views during process
     * @return a set of updated views
    */
    const std::set<IndexT> & getUpdatedViews() const
    {
        return _updatedViews;
    }

protected:
    std::set<IndexT> _updatedViews;
};

}
}