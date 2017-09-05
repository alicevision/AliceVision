// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#pragma once

#include <mutex>
#include <set>

typedef std::mutex mutexT;
typedef std::lock_guard<mutexT> lock_guardT;

namespace openMVG {
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
} // namespace openMVG
