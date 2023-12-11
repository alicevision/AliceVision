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
  using parent = HashMap<IndexT, T>;
  friend class SfMData;
  
  public:
    HashMapLocked<T>() : parent()
    {
    }

    /**
     * We don't want the user to assume the object is created when the index does not exist in the map
     */
    T & operator[](const IndexT& index) 
    {
        return parent::operator[](index);
    }
    
    const T & operator[](const IndexT& index) const
    {
        return parent::operator[](index);
    }

  private:
    typename parent::iterator find(const IndexT & k)
    {
        return parent::find(k);
    }

    typename parent::const_iterator find(const IndexT & k) const
    {
        return parent::find(k);
    }

    
};

}  // namespace sfmData
}  // namespace aliceVision