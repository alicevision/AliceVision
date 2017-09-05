// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#ifndef OPENMVG_SYSTEM_TIMER_HPP
#define OPENMVG_SYSTEM_TIMER_HPP

#include <chrono>
#include <iostream>
#include <string>

namespace openMVG {
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
} // namespace openMVG

#endif // OPENMVG_SYSTEM_TIMER_HPP

