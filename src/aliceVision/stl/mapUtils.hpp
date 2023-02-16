// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

// ---------------------------
// Usage example :
// ---------------------------

// std::map<int, double> m;
// m[0] = 2.0;
// m[1] = 1.6;
// std::vector<int> keys;
// // Retrieve all keys
// transform(m.begin(), m.end(), back_inserter(keys), RetrieveKey());
// // Log all keys to console
// copy(keys.begin(), keys.end(), ostream_iterator<int>(cout, "\n"));
// // Retrieve all values
// std::vector<double> values;
// // Retrieve all values
// transform(m.begin(), m.end(), back_inserter(values), RetrieveValue());

#include <map>

namespace stl {

/// Allow to select the Keys of a map.
struct RetrieveKey
{
  template <typename T>
  typename T::first_type operator()(const T & keyValuePair) const
  {
    return keyValuePair.first;
  }
};

/// Allow to select the Values of a map.
struct RetrieveValue
{
  template <typename T>
  typename T::second_type operator()(const T & keyValuePair) const
  {
    return keyValuePair.second;
  }
};

}

namespace aliceVision {

template<typename Map>
const typename Map::mapped_type& map_get_with_default(const Map& m, const typename Map::key_type& key, const typename Map::mapped_type& defval)
{
    auto it = m.find(key);
    if(it == m.end())
        return defval;
    return it->second;
}
template<typename Map>
bool map_has_non_empty_value(const Map& m, const typename Map::key_type& key)
{
    auto it = m.find(key);
    if(it == m.end())
        return false;
    return !it->second.empty();
}

}
