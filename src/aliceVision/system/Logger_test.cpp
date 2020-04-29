// This file is part of the AliceVision project.
// Copyright (c) 2019 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/system/Logger.hpp>

#define BOOST_TEST_MODULE Logger

#include <boost/test/unit_test.hpp>
#include <boost/algorithm/string.hpp>

#include <array>
#include <exception>

BOOST_AUTO_TEST_CASE(Logger_Enums)
{
    using namespace aliceVision::system;
    const std::array<EVerboseLevel, 6> allLevels{EVerboseLevel::Trace,
                                                 EVerboseLevel::Debug,
                                                 EVerboseLevel::Info,
                                                 EVerboseLevel::Warning,
                                                 EVerboseLevel::Error ,
                                                 EVerboseLevel::Fatal};

    const std::array<std::string, 6> allStringLevels{"trace",
                                                 "debug",
                                                 "info",
                                                 "warning",
                                                 "error" ,
                                                 "fatal"};

    for(std::size_t i = 0; i < allLevels.size(); ++i)
    {
        BOOST_CHECK(allLevels[i] == EVerboseLevel_stringToEnum(allStringLevels[i]));
        BOOST_CHECK(allLevels[i] == EVerboseLevel_stringToEnum(boost::to_upper_copy(allStringLevels[i])));
        BOOST_CHECK(allStringLevels[i] == EVerboseLevel_enumToString(allLevels[i]));
    }

    BOOST_CHECK_THROW(EVerboseLevel_stringToEnum("not a level"), std::out_of_range);
}
