// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <mutex>
#include <set>

namespace aliceVision {
namespace sfm {

/// ThreadSafe Set thanks to a mutex
template <typename T>
class MutexSet {

  typedef std::mutex mutexT;
  typedef std::lock_guard<mutexT> lockGuardT;

public:
  void insert(const T& value)
  {
    lockGuardT guard(_mutex);
    _set.insert(value);
  }

  int count(const T& value) const
  {
    lockGuardT guard(_mutex);
    return _set.count(value);
  }

  std::size_t size() const
  {
    lockGuardT guard(_mutex);
    return _set.size();
  }

private:
  std::set<T> _set;
  mutable mutexT _mutex;
};

} // namespace sfm
} // namespace aliceVision
