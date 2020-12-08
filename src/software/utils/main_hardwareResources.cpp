// This file is part of the AliceVision project.
// Copyright (c) 2020 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>
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


int aliceVision_main(int argc, char **argv)
{
  // command-line parameters
  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());

  po::options_description allParams("AliceVision hardwareResources");

  po::options_description logParams("Log parameters");
  logParams.add_options()
    ("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
      "verbosity level (fatal,  error, warning, info, debug, trace).");

  allParams.add(logParams);

  po::variables_map vm;
  try
  {
    po::store(po::parse_command_line(argc, argv, allParams), vm);

    if(vm.count("help"))
    {
      ALICEVISION_COUT(allParams);
      return EXIT_SUCCESS;
    }
    po::notify(vm);
  }
  catch(boost::program_options::required_option& e)
  {
    ALICEVISION_CERR("ERROR: " << e.what());
    ALICEVISION_COUT("Usage:\n\n" << allParams);
    return EXIT_FAILURE;
  }
  catch(boost::program_options::error& e)
  {
    ALICEVISION_CERR("ERROR: " << e.what());
    ALICEVISION_COUT("Usage:\n\n" << allParams);
    return EXIT_FAILURE;
  }

  ALICEVISION_COUT("Program called with the following parameters:");
  ALICEVISION_COUT(vm);

  // set verbose level
  system::Logger::get()->setLogLevel(verboseLevel);

  // print GPU Information
  ALICEVISION_LOG_INFO(gpu::gpuInformationCUDA());

  system::MemoryInfo memoryInformation = system::getMemoryInfo();

  ALICEVISION_LOG_INFO("Memory information: " << std::endl << memoryInformation);

  if(memoryInformation.availableRam == 0)
  {
    ALICEVISION_LOG_WARNING("Cannot find available system memory, this can be due to OS limitation.\n"
                            "Use only one thread for CPU feature extraction.");
  }
  else
  {
      const double oneGB = 1024.0 * 1024.0 * 1024.0;
      if(memoryInformation.availableRam < 0.5 * memoryInformation.totalRam)
      {
          ALICEVISION_LOG_WARNING("More than half of the RAM is used by other applications. It would be more efficient to close them.");
          ALICEVISION_LOG_WARNING(" => "
                                  << std::size_t(std::round(double(memoryInformation.totalRam - memoryInformation.availableRam) / oneGB))
                                  << " GB are used by other applications for a total RAM capacity of "
                                  << std::size_t(std::round(double(memoryInformation.totalRam) / oneGB)) << " GB.");
      }
  }

  return EXIT_SUCCESS;
}
