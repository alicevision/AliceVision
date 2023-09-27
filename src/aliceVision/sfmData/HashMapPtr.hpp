// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/types.hpp>

namespace aliceVision {
namespace sfmData {

template <class T>
class HashMapPtr : public HashMap<IndexT, std::shared_ptr<T>>
{
public:
    /**
     * We don't want the user to assume the object is created when the index does not exist in the map
    */
    std::shared_ptr<T> & operator[](const IndexT & index) = delete;
};

}
}