// ========================================================================== //
//
// Copyright (C) 2013 David Ok <david.ok8@gmail.com>
// Copyright (C) 2014 Pierre Moulon
//
// Adapted from DO++, a basic set of libraries in C++ for computer
// vision.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License v. 2.0. If a copy of the MPL was not distributed with this file,
// you can obtain one at http://mozilla.org/MPL/2.0/.
// ========================================================================== //

#include <openMVG/system/timer.hpp>
#include <cmath>

#ifdef _WIN32
# include <windows.h>
#endif

namespace openMVG {
namespace system {

Timer::Timer()
{
  reset();
}

void Timer::reset()
{
  start_ = std::chrono::high_resolution_clock::now();
}

double Timer::elapsed() const
{
  return elapsedMs() / 1000.;
}

double Timer::elapsedMs() const
{
  const auto end_ = std::chrono::high_resolution_clock::now();
  return std::chrono::duration_cast<std::chrono::milliseconds>(end_ - start_).count();
}

std::ostream& operator << (std::ostream& str, const Timer& t)
{
  return str << t.elapsed() << " s elapsed";
}

std::string prettyTime(double durationMs)
{
  std::string out;

  const auto msecs = fmod(durationMs, 1000);
  durationMs /= 1000.;
  const std::size_t secs = std::size_t(fmod(durationMs, 60));
  durationMs /= 60.;
  const std::size_t mins = std::size_t(fmod(durationMs, 60));
  durationMs /= 60.;
  const std::size_t hours = std::size_t(fmod(durationMs, 24));
  durationMs /= 24.;
  const std::size_t days = durationMs;

  bool printed_earlier = false;
  if(days >= 1)
  {
    printed_earlier = true;
    out += (std::to_string(days) + "d ");
  }
  if(printed_earlier || hours >= 1)
  {
    printed_earlier = true;
    out += (std::to_string(hours) + "h ");
  }
  if(printed_earlier || mins >= 1)
  {
    printed_earlier = true;
    out += (std::to_string(mins) + "m ");
  }
  if(printed_earlier || secs >= 1)
  {
    printed_earlier = true;
    out += (std::to_string(secs) + "s ");
  }
  if(printed_earlier || msecs >= 1)
  {
    printed_earlier = true;
    out += (std::to_string(msecs) + "ms");
  }
  return out;
}

} // namespace system
} // namespace openMVG
