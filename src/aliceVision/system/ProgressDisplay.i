// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

%include <aliceVision/system/ProgressDisplay.hpp>

%{
#include <iostream>
#include <aliceVision/system/ProgressDisplay.hpp>

// Use the full namespace in the wrapper
namespace aliceVision {
namespace system {

inline ProgressDisplay ConsoleProgressDisplay(unsigned long expectedCount, 
                                              const std::string& s1 = "\n",  // leading strings
                                              const std::string& s2 = "",
                                              const std::string& s3 = "")
{
  return createConsoleProgressDisplay(expectedCount, std::cout, s1, s2, s3);
}

} // namespace system
} // namespace aliceVision

%}

// Declare the wrapper so SWIG knows it
namespace aliceVision {
namespace system {
    ProgressDisplay ConsoleProgressDisplay(unsigned long expectedCount, 
                                           const std::string& s1 = "\n", 
                                           const std::string& s2 = "", 
                                           const std::string& s3 = "");
}
}
