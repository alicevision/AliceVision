// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "ExpansionIteration.hpp"

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/sfm/pipeline/expanding/ExpansionChunk.hpp>

namespace aliceVision {
namespace sfm {


bool ExpansionIteration::process(sfmData::SfMData & sfmData, track::TracksHandler & tracksHandler)
{
    ALICEVISION_LOG_INFO("ExpansionIteration::process start");

    if (!_chunkHandler)
    {
        return false;
    }

    if (!_policy)
    {
        return false;
    }

    if (!_policy->initialize(sfmData))
    {
        return false;
    }
 
    while (1)
    {
        if (!_historyHandler->beginEpoch(sfmData))
        {
            break;
        }

        if (!_policy->process(sfmData, tracksHandler))
        {
            break;
        }
   
        if (!_chunkHandler->process(sfmData, tracksHandler, _policy->getNextViews()))
        {
            continue;
        }

        //Rollback any views which were ignored (not with errors)
        _policy->rollback(_chunkHandler->getIgnoredViews());
        
        //Save this epoch to history
        _historyHandler->endEpoch(sfmData, _policy->getNextViews());
    }

    ALICEVISION_LOG_INFO("ExpansionIteration::process end");

    return true;
}

} // namespace sfm
} // namespace aliceVision

