// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2014 openMVG contributors.
// Copyright (c) 2013 David Ok.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#ifndef ALICEVISION_SYSTEM_TIMER_HPP
#define ALICEVISION_SYSTEM_TIMER_HPP

#include <chrono>
#include <iostream>
#include <string>

namespace aliceVision {
namespace system {

  /**
   * @brief Timer class with microsecond accuracy.
   * Adapted from DO++, a basic set of libraries in C++ for computer vision, licensed under MPL2.
   * See https://do-cv.github.io/sara
   */
  class Timer
  {
  public:
    //! Default constructor
    Timer();
    //! Reset the timer to zero.
    void reset();
    //! Returns the elapsed time in seconds.
    double elapsed() const;
    //! Returns the elapsed time in milliseconds.
    double elapsedMs() const;
  private:

    std::chrono::high_resolution_clock::time_point start_;
  };
  
  // print the elapsed time
  std::ostream& operator << (std::ostream&, const Timer&);
  
/**
 * @brief Prints the duration in the format #d #h #m #s #ms starting from the non-zero
 * most significant entity (ie it does not print #d if d is 0 and so on...).
 * 
 * @param durationMs the duration in milliseconds.
 * @return a formatted string
 */  
std::string prettyTime(double durationMs);

} // namespace system
} // namespace aliceVision

#endif // ALICEVISION_SYSTEM_TIMER_HPP

