// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/Parallelization.hpp>

namespace aliceVision {

bool rangeComputation(int & rangeStart, int & rangeEnd, int rangeIteration, int rangeBlocksCount, int itemsCount)
{
    rangeIteration = std::max(0, rangeIteration);
    rangeBlocksCount = std::max(1, rangeBlocksCount);

    if (itemsCount <= rangeBlocksCount)
    {
        //Just split 1 per iteration
        rangeStart = rangeIteration;
        rangeEnd = rangeStart + 1;
    }
    else
    {
        // Distribute data among available chunks
        const int chunkSize = itemsCount / rangeBlocksCount;
        const int reminder = itemsCount % rangeBlocksCount;
        
        // Reminder is distributed among first iterations
        const int shift = std::min(rangeIteration, reminder);
        const int reminded = (rangeIteration < reminder)?1:0;

        rangeStart = rangeIteration * chunkSize + shift;
        rangeEnd = rangeStart + chunkSize + reminded;
    }

    rangeEnd = std::min(rangeEnd, itemsCount);

    return (rangeStart < itemsCount);
}

}  // namespace aliceVision
