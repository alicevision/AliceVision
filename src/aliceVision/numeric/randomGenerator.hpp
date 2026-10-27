// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <random>

namespace aliceVision {

/**
 * @brief Create a generator dedicated to one work item (an image pair, a track...),
 * so that its random draws do not depend on thread scheduling nor on the other items.
 * @param[in] baseSeed seed shared by all the items, drawn once outside of the parallel loop
 * @param[in] ids the ids identifying the item
 * @return the generator for this item
 */
template<typename... Ids>
inline std::mt19937 createSeededGenerator(std::mt19937::result_type baseSeed, Ids... ids)
{
    std::seed_seq seedSequence{baseSeed, static_cast<std::mt19937::result_type>(ids)...};
    return std::mt19937(seedSequence);
}

}  // namespace aliceVision
