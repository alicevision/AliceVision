// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#pragma once

#include <boost/container/flat_set.hpp>

namespace stl
{
  template <class Key
         ,class Compare  = std::less<Key>
         ,class Allocator = std::allocator<Key> >
  using flat_set = boost::container::flat_set<Key, Compare, Allocator>;
}
