// This file is part of the AliceVision project.
// Copyright (c) 2020 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/system/MemoryInfo.hpp>
#include <aliceVision/gpu/gpu.hpp>
#include <aliceVision/config.hpp>

#include <boost/program_options.hpp>

#include <string>
#include <sstream>
#include <vector>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;

int aliceVision_main(int argc, char** argv)
{
    // command-line parameters
    CmdLine cmdline("AliceVision hardwareResources");
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // print GPU Information
    ALICEVISION_LOG_INFO(gpu::gpuInformationCUDA());

    system::MemoryInfo memoryInformation = system::getMemoryInfo();

    ALICEVISION_LOG_INFO("Memory information: " << std::endl << memoryInformation);

    if (memoryInformation.availableRam == 0)
    {
        ALICEVISION_LOG_WARNING("Cannot find available system memory, this can be due to OS limitation.\n"
                                "Use only one thread for CPU feature extraction.");
    }
    else
    {
        const double oneGB = 1024.0 * 1024.0 * 1024.0;
        if (memoryInformation.availableRam < 0.5 * memoryInformation.totalRam)
        {
            ALICEVISION_LOG_WARNING("More than half of the RAM is used by other applications. It would be more efficient to close them.");
            ALICEVISION_LOG_WARNING(" => " << std::size_t(std::round(double(memoryInformation.totalRam - memoryInformation.availableRam) / oneGB))
                                           << " GB are used by other applications for a total RAM capacity of "
                                           << std::size_t(std::round(double(memoryInformation.totalRam) / oneGB)) << " GB.");
        }
    }

    return EXIT_SUCCESS;
}
