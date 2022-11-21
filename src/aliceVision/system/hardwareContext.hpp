// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "Logger.hpp"
#include "Timer.hpp"

#include <boost/program_options/options_description.hpp>

namespace aliceVision {

class HardwareContext
{
public:
    void displayHardware();

    size_t getUserMaxMemoryAvailable() const
    {
        return _maxUserMemoryAvailable;
    }

    unsigned int getUserMaxCoresAvailable() const
    {
        return _maxUserCoresAvailable;
    }

    void setUserCoresLimit(unsigned int coresLimit)
    {
        _limitUserCores = coresLimit;
    }

    void setOptions(boost::program_options::options_description & options);

    unsigned int getMaxThreads() const;

private:
    size_t _maxUserMemoryAvailable = std::numeric_limits<size_t>::max();
    unsigned int _maxUserCoresAvailable = std::numeric_limits<unsigned int>::max();
    unsigned int _limitUserCores = std::numeric_limits<unsigned int>::max();
};

}
