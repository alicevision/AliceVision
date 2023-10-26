// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <iosfwd>
#include <memory>
#include <string>

namespace aliceVision {
namespace system {

class ProgressDisplayImpl
{
  public:
    virtual ~ProgressDisplayImpl();
    virtual void restart(unsigned long expectedCount) = 0;
    virtual void increment(unsigned long count) = 0;
    virtual unsigned long count() = 0;
    virtual unsigned long expectedCount() = 0;
};

/**
 * This is a generic API to display progress bars. Depending on implementation different
 * destinations for display data may be used. Currently only console output is supported.
 *
 * The API is essentially the same as boost::timer::progress_display
 *
 * For ease of use value semantics are exposed.
 */
class ProgressDisplay
{
  public:
    ProgressDisplay();
    ProgressDisplay(const std::shared_ptr<ProgressDisplayImpl>& impl)
      : _impl{impl}
    {}

    void restart(unsigned long expectedCount) { _impl->restart(expectedCount); }

    // Thread safe with respect to other calls to operator++ and to calls to count()
    void operator++() { _impl->increment(1); }

    // Thread safe with respect to other calls to operator++ and to calls to count()
    void operator+=(unsigned long increment) { _impl->increment(increment); }

    // Thread safe with respect to calls to operator++
    unsigned long count() { return _impl->count(); }

    // Thread safe
    unsigned long expectedCount() { return _impl->expectedCount(); }

  private:
    std::shared_ptr<ProgressDisplayImpl> _impl;
};

/// Creates console-based progress bar
ProgressDisplay createConsoleProgressDisplay(unsigned long expectedCount,
                                             std::ostream& os,
                                             const std::string& s1 = "\n",  // leading strings
                                             const std::string& s2 = "",
                                             const std::string& s3 = "");

}  // namespace system
}  // namespace aliceVision
