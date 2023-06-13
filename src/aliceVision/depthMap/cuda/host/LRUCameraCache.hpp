// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/depthMap/cuda/host/LRUCache.hpp>

namespace aliceVision {
namespace depthMap {

/**
 * @struct CameraPair
 * @brief Support class for operating an LRU cache of downscaled cameras
 * @note The CameraPair (camera id, downscale) give a unique key for LRU cache.
 */
struct CameraPair : public std::pair<int, int>
{
    CameraPair() : std::pair<int, int>(0, 0) {}
    CameraPair(int i) : std::pair<int, int>(i, i) {}
    CameraPair(int i, int j) : std::pair<int, int>(i, j) {}

    CameraPair& operator=(int i)
    {
        this->first = this->second = i;
        return *this;
    }
};

inline bool operator==(const CameraPair& l, const CameraPair& r)
{
    return (l.first == r.first && l.second == r.second);
}

inline bool operator<(const CameraPair& l, const CameraPair& r)
{
    return (l.first < r.first || (l.first == r.first && l.second < r.second));
}

using LRUCameraCache = LRUCache<CameraPair>;
using LRUCameraIdCache = LRUCache<int>;

} // namespace depthMap
} // namespace aliceVision
