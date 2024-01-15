// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/types.hpp>

namespace aliceVision {
namespace sfmData {

template<class T>
auto cloneT(T* ptr, int val) -> decltype(ptr->clone())
{
    if (ptr)
    {
        return ptr->clone();
    }

    return nullptr;
}

template<class T>
T* cloneT(T* ptr, long val)
{
    if (ptr)
    {
        return new T(*ptr);
    }

    return nullptr;
}

template<class T>
class HashMapPtr : public HashMap<IndexT, std::shared_ptr<T>>
{
  public:
    HashMapPtr<T>()
      : HashMap<IndexT, std::shared_ptr<T>>()
    {}

    HashMapPtr(const HashMapPtr<T>& other)
    {
        for (const auto& pair : other)
        {
            this->insert({pair.first, std::shared_ptr<T>(cloneT(pair.second.get(), 0))});
        }
    }

    /**
     * We don't want the user to assume the object is created when the index does not exist in the map
     */
    std::shared_ptr<T>& operator[](const IndexT& index) = delete;
};

}  // namespace sfmData
}  // namespace aliceVision