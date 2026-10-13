// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "ProgressDisplay.hpp"
#include <iostream>
#include <mutex>

namespace aliceVision {
namespace system {

ProgressDisplayImpl::~ProgressDisplayImpl() = default;

class ProgressDisplayImplEmpty : public ProgressDisplayImpl
{
  public:
    void restart([[maybe_unused]] unsigned long expectedCount) override {}
    void increment([[maybe_unused]] unsigned long count) override {}
    unsigned long count() override { return 0; }
    unsigned long expectedCount() override { return 0; }
};

ProgressDisplay::ProgressDisplay()
  : _impl{std::make_shared<ProgressDisplayImplEmpty>()}
{}

// Console progress bar, same output as the former boost::timer::progress_display
class ProgressDisplayImplConsole : public ProgressDisplayImpl
{
  public:
    ProgressDisplayImplConsole(unsigned long expectedCount,
                               std::ostream& os,
                               const std::string& s1,
                               const std::string& s2,
                               const std::string& s3)
      : _os{os},
        _s1{s1},
        _s2{s2},
        _s3{s3}
    {
        restart(expectedCount);
    }

    ~ProgressDisplayImplConsole() override = default;

    void restart(unsigned long expectedCount) override
    {
        _count = _nextTicCount = _tic = 0;
        _expectedCount = expectedCount ? expectedCount : 1;  // prevent divide by zero

        _os << _s1 << "0%   10   20   30   40   50   60   70   80   90   100%\n"
            << _s2 << "|----|----|----|----|----|----|----|----|----|----|" << std::endl
            << _s3;
    }

    void increment(unsigned long count) override
    {
        std::lock_guard<std::mutex> lock{_mutex};
        if ((_count += count) >= _nextTicCount)
            displayTic();
    }

    unsigned long count() override
    {
        std::lock_guard<std::mutex> lock{_mutex};
        return _count;
    }

    unsigned long expectedCount() override { return _expectedCount; }

  private:
    void displayTic()
    {
        const unsigned int ticsNeeded = static_cast<unsigned int>(static_cast<double>(_count) / static_cast<double>(_expectedCount) * 50.0);
        do
        {
            _os << '*' << std::flush;
        } while (++_tic < ticsNeeded);
        _nextTicCount = static_cast<unsigned long>((_tic / 50.0) * static_cast<double>(_expectedCount));
        if (_count == _expectedCount)
        {
            if (_tic < 51)
                _os << '*';
            _os << std::endl;
        }
    }

    std::mutex _mutex;
    std::ostream& _os;
    const std::string _s1;
    const std::string _s2;
    const std::string _s3;
    unsigned long _count = 0;
    unsigned long _expectedCount = 1;
    unsigned long _nextTicCount = 0;
    unsigned int _tic = 0;
};

ProgressDisplay createConsoleProgressDisplay(unsigned long expectedCount,
                                             std::ostream& os,
                                             const std::string& s1,
                                             const std::string& s2,
                                             const std::string& s3)
{
    auto impl = std::make_shared<ProgressDisplayImplConsole>(expectedCount, os, s1, s2, s3);
    return ProgressDisplay(impl);
}

ProgressDisplay createConsoleProgressDisplay(unsigned long expectedCount,
                                             const std::string& s1,
                                             const std::string& s2,
                                             const std::string& s3)
{
    return createConsoleProgressDisplay(expectedCount, std::cout, s1, s2, s3);
}

}  // namespace system
}  // namespace aliceVision
