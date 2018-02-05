// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <mutex>
#include <set>

typedef std::mutex mutexT;
typedef std::lock_guard<mutexT> lock_guardT;

namespace aliceVision {
namespace sfm{

/// ThreadSafe Set thanks to a mutex
template <typename T>
class MutexSet {

public:
    void insert(const T & value) {
      lock_guardT guard(m_Mutex);
      m_Set.insert(value);
    }

    int count(const T & value) const {
      lock_guardT guard(m_Mutex);
      return m_Set.count(value);
    }

    std::size_t size() const {
      lock_guardT guard(m_Mutex);
      return m_Set.size();
    }

private:
    std::set<T> m_Set;
    mutable mutexT m_Mutex;
};

} // namespace sfm
} // namespace aliceVision
