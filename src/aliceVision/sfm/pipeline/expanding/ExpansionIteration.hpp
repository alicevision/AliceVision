// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/types.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/track/TracksHandler.hpp>
#include <aliceVision/sfm/pipeline/expanding/ExpansionChunk.hpp>
#include <aliceVision/sfm/pipeline/expanding/ExpansionHistory.hpp>
#include <aliceVision/sfm/pipeline/expanding/ExpansionPolicy.hpp>

namespace aliceVision {
namespace sfm {

class ExpansionIteration
{
public:
    using uptr = std::unique_ptr<ExpansionIteration>;

public:
    
    /**
     * @brief process an iteration of the sfm
     * An iteration is a set of successive chunks selected dynamically.
     * Multiple iteration may be necessary to complete.
     * @param sfmData the scene to process
     * @param tracksHandler the tracks for this scene
     * @return true if the iteration succeeded
    */
    bool process(sfmData::SfMData & sfmData,  track::TracksHandler & tracksHandler);

    /**
     * @return a pointer to the chunk handler
    */
    ExpansionChunk * getChunkHandler() const
    {
        return _chunkHandler.get();
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
     * brief setup the expansion history handler
     * @param expansionPolicy a unique ptr. Ownership will be taken
     */
    void setExpansionPolicyHandler(ExpansionPolicy::uptr & expansionPolicy)
    {
        _policy = std::move(expansionPolicy);
    }

    /**
     * brief setup the expansion chunk handler
     * @param expansionChunk a unique ptr. Ownership will be taken
     */
    void setExpansionChunkHandler(ExpansionChunk::uptr & expansionChunk)
    {
        _chunkHandler = std::move(expansionChunk);
    }

private:
    std::unique_ptr<ExpansionChunk> _chunkHandler;
    std::shared_ptr<ExpansionHistory> _historyHandler;
    std::unique_ptr<ExpansionPolicy> _policy;
};

} // namespace sfm
} // namespace aliceVision

