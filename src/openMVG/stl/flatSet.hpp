#pragma once

#include <openMVG/config.hpp>

#if OPENMVG_IS_DEFINED(OPENMVG_HAVE_BOOST)
#include <boost/container/flat_set.hpp>
#endif

namespace stl
{
#if OPENMVG_IS_DEFINED(OPENMVG_HAVE_BOOST)
  template <class Key
         ,class Compare  = std::less<Key>
         ,class Allocator = std::allocator<Key> >
  using flat_set = boost::container::flat_set<Key, Compare, Allocator>;
#else
  // Fallback to non-flat implementation
  template <class Key
         ,class Compare  = std::less<Key>
         ,class Allocator = std::allocator<Key> >
  using flat_set = std::set<Key, Compare, Allocator>;
#endif
}
