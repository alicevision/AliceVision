// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/types.hpp>
#include <utility>
#include <memory>

namespace aliceVision {
namespace sfmData {

template<class T>
class HashMapLocked : public HashMap<IndexT, T>
{
  public:
    HashMapLocked<T>() : HashMap<IndexT, T>()
    {
    }

    /**
     * We don't want the user to assume the object is created when the index does not exist in the map
     */
    //std::shared_ptr<T>& operator[](const IndexT& index) = delete;
};

}  // namespace sfmData
}  // namespace aliceVision