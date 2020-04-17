// This file is part of the AliceVision project.
// Copyright (c) 2020 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#define _ALICEVISION_SYSTEM_MAIN_IMPL
#include "main.hpp"

#include "Logger.hpp"

#include <stdexcept>

#undef main

int aliceVision_main_wrapper(int(*realMain)(int, char*[]), int argc, char* argv[])
{
      try
    {
        return realMain(argc, argv);
    }
    catch(const std::exception& e)
    {
        ALICEVISION_LOG_FATAL(e.what());
    }
    catch(...)
    {
        ALICEVISION_LOG_FATAL("Unknown exception");
    }
    return EXIT_FAILURE;
}

