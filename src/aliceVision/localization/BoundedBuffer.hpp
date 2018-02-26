// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <deque>
#include <assert.h>

namespace aliceVision {
namespace localization {

/**
 * @brief This class implements a bounded buffer, a buffer with a given fixed size
 * that allows to push new elements into it. If the buffer is full at the moment
 * of the insertion, the first element of the buffer (in a FIFO strategy) is removed
 * and the new element is appended at the end of the buffer.
 */
template<class T>
class BoundedBuffer
{
  
private:
  
  typedef std::deque<T> Buffer;
  
  Buffer _buffer;

  /// The fixed maximum size for the buffer
  std::size_t _maxSize;
  
public:
  
  /**
   * @brief Build a bounded buffer of the given size.
   * @param[in] maxSize The maximum size of the buffer. Whenever a new element is 
   * pushed into the buffer, if the buffer is full, the first element is removed
   * in a FIFO strategy in order to make place for the new element.
   */
  BoundedBuffer(std::size_t maxSize) : _maxSize(maxSize) { }
  
  typedef typename Buffer::iterator iterator;
  typedef typename Buffer::const_iterator const_iterator;

  /**
   * @brief Returns an iterator pointing to the first element of the buffer.
   * 
   * @return the iterator pointing to the first element of the buffer.
   */
  iterator begin() { return _buffer.begin(); }

  /**
   * @brief Returns an iterator pointing to the last element of the buffer.
   * 
   * @return the iterator pointing to the last element of the buffer.
   */
  iterator end()  { return _buffer.end(); }
  
  /**
   * @brief Returns a const iterator pointing to the first element of the buffer.
   * 
   * @return the const iterator pointing to the first element of the buffer.
   */
  const_iterator begin() const { return _buffer.begin(); }

  /**
   * @brief Returns an iterator pointing to the last element of the buffer.
   * 
   * @return the iterator pointing to the last element of the buffer.
   */
  const_iterator end() const { return _buffer.end(); }
  
  /**
   * @brief It add a new element at the end of the buffer. If the buffer is full
   * the first element is removed before the insertion (FIFO strategy).
   * @param[in] fm The element to add.
   */
  template <typename... Args>
  void emplace_back(Args&&... args)
  {
    assert(_buffer.size() <= _maxSize);

    if(_buffer.size() == _maxSize)
    {
      _buffer.pop_front();
    }
    _buffer.emplace_back(args...);
  }

};

}
}
