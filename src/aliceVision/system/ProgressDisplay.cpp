// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "ProgressDisplay.hpp"
#include <boost/timer/progress_display.hpp>
#include <mutex>

namespace aliceVision {
namespace system {

ProgressDisplayImpl::~ProgressDisplayImpl() = default;

class ProgressDisplayImplEmpty : public ProgressDisplayImpl {
public:
    void restart(unsigned long expectedCount) override {}
    void increment(unsigned long count) override {}
    unsigned long count() override { return 0; }
    unsigned long expectedCount() override { return 0; }
};

ProgressDisplay::ProgressDisplay() : _impl{std::make_shared<ProgressDisplayImplEmpty>()}
{}

class ProgressDisplayImplBoostProgress : public ProgressDisplayImpl {
public:
    ProgressDisplayImplBoostProgress(unsigned long expectedCount,
                                     std::ostream& os,
                                     const std::string& s1,
                                     const std::string& s2,
                                     const std::string& s3) :
        _display{expectedCount, os, s1, s2, s3}
    {
    }

    ~ProgressDisplayImplBoostProgress() override = default;

    void restart(unsigned long expectedCount) override
    {
        _display.restart(expectedCount);
    }

    void increment(unsigned long count) override
    {
        std::lock_guard<std::mutex> lock{_mutex};
        _display += count;
    }

    unsigned long count() override
    {
        std::lock_guard<std::mutex> lock{_mutex};
        return _display.count();
    }

    unsigned long expectedCount() override
    {
        return _display.expected_count();
    }

private:
    std::mutex _mutex;
    boost::timer::progress_display _display;
};


ProgressDisplay createConsoleProgressDisplay(unsigned long expectedCount,
                                             std::ostream& os,
                                             const std::string& s1,
                                             const std::string& s2,
                                             const std::string& s3)
{
    auto impl = std::make_shared<ProgressDisplayImplBoostProgress>(expectedCount, os, s1, s2, s3);
    return ProgressDisplay(impl);
}

} // namespace system
} // namespace aliceVision
