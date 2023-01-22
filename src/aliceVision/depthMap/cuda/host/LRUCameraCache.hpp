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
 * @struct CameraSelection
 * @brief Support class for operating an LRU cache of cameras
 */
struct CameraSelection : public std::pair<int, int>
{
    CameraSelection() : std::pair<int, int>(0, 0) {}
    CameraSelection(int i) : std::pair<int, int>(i, i) {}
    CameraSelection(int i, int j) : std::pair<int, int>(i, j) {}

    CameraSelection& operator=(int i)
    {
        this->first = this->second = i;
        return *this;
    }
};

inline bool operator==(const CameraSelection& l, const CameraSelection& r)
{
    return (l.first == r.first && l.second == r.second);
}

inline bool operator<(const CameraSelection& l, const CameraSelection& r)
{
    return (l.first < r.first || (l.first == r.first && l.second < r.second));
}

using LRUCameraCache = LRUCache<CameraSelection>;

} // namespace depthMap
} // namespace aliceVision
