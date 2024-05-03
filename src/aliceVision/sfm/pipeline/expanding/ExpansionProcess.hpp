// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/types.hpp>
#include <aliceVision/track/TracksHandler.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/feature/FeaturesPerView.hpp>
#include <aliceVision/sfm/pipeline/expanding/ExpansionIteration.hpp>
#include <aliceVision/sfm/pipeline/expanding/ExpansionHistory.hpp>

namespace aliceVision {
namespace sfm {

class ExpansionProcess{
public:
    using uptr = std::unique_ptr<ExpansionProcess>;

public:

    /**
     * @brief Process a scene, potentially handling multiple groups
     * @param sfmData the in/out scene to process
     * @param tracksHandler the tracks for this scene
    */
    bool process(sfmData::SfMData & sfmData, track::TracksHandler & tracksHandler);

    /**
     * @brief Return iteration handler pointer
     * @return a pointer to the unique_ptr iterationhandler
    */
    ExpansionIteration * getIterationHandler() const
    {
        return _iterationHandler.get();
    }

    /**
     * brief setup the expansion history handler
     * @param expansionHistory a shared ptr
     */
    void setExpansionHistoryHandler(ExpansionHistory::sptr & expansionHistory)
    {
        _historyHandler = expansionHistory;
    }
    /**
     * brief setup the expansion iteration handler
     * @param expansionIteration a unique ptr. Ownership will be taken
     */
    void setExpansionIterationHandler(ExpansionIteration::uptr & expansionIteration)
    {
        _iterationHandler = std::move(expansionIteration);
    }

private:

    /**
     * @brief Process sfmData if something exists inside (previous sfm)
     * @param[in] sfmData the object to update
     * @param[in] tracks the tracks for this scene
     */
    bool prepareExisting(sfmData::SfMData & sfmData, const track::TracksHandler & tracksHandler);

    /**
     * @brief Remap the sfmData landmarks to id compatible with trackmap
     * @param[in] sfmData the object to update
     * @param[in] tracks the tracks for this scene
     */
    void remapExistingLandmarks(sfmData::SfMData & sfmData, const track::TracksHandler & tracksHandler);

    
   
private:
    //Must be declared first for initialization
    std::shared_ptr<ExpansionHistory> _historyHandler;

    /**
     * Handle iteration prcess
    */
    std::unique_ptr<ExpansionIteration> _iterationHandler;
};

} // namespace sfm
} // namespace aliceVision

