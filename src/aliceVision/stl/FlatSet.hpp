// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <boost/container/flat_set.hpp>

namespace stl
{
  template <class Key
         ,class Compare  = std::less<Key>
         ,class Allocator = std::allocator<Key> >
  using flat_set = boost::container::flat_set<Key, Compare, Allocator>;
}
