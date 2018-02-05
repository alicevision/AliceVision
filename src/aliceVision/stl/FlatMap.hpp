// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <boost/container/flat_map.hpp>

namespace stl
{
  template <class Key
         ,class T
         ,class Compare = std::less<Key>
         ,class Allocator = std::allocator<std::pair<Key, T> > >
  using flat_map = boost::container::flat_map<Key, T, Compare, Allocator>;
}
