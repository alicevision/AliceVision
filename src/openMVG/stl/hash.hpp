// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#ifndef ALICEVISION_STL_HASH_H
#define ALICEVISION_STL_HASH_H

#include <functional>

namespace stl
{

// Combine hashing value
// http://www.boost.org/doc/libs/1_37_0/doc/html/hash/reference.html#boost.hash_combine
template <class T>
inline void hash_combine(std::size_t& seed, const T& v)
{
  std::hash<T> hasher;
  seed ^= hasher(v) + 0x9e3779b9 + (seed<<6) + (seed>>2);
}

} // namespace stl

#endif  // ALICEVISION_STL_HASH_H
