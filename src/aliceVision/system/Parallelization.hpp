// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once


namespace aliceVision {

/**
 * Utility function to compute range of values to process
 * Assuming we have itemsCount items in a container.
 * @param[out] rangeStart is the first element index to process 
 * @param[out] rangeEnd is the last element index to process (non included)
 * @param rangeIteration is the iteration number
 * @param rangeBlocksCount is the number of iterations launched in parallel
 * @return false if nothing has to be processed in this iteration
*/
bool rangeComputation(int & rangeStart, int & rangeEnd, int rangeIteration, int rangeBlocksCount, int itemsCount);

}  // namespace aliceVision
