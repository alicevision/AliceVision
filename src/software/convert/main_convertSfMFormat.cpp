// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2015 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfm/sfm.hpp>
#include <aliceVision/sfm/utils/uid.hpp>
#include <aliceVision/config.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>

#include <boost/program_options.hpp>
#include <boost/system/error_code.hpp>
#include <boost/filesystem.hpp>

#include <string>
#include <vector>

using namespace aliceVision;
using namespace aliceVision::sfm;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

// convert from a SfMData format to another
int main(int argc, char **argv)
{
  // command-line parameters

  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  std::string sfmDataFilename;
  std::string outputSfMDataFilename;

  // user optional parameters

  bool flagViews = true;
  bool flagIntrinsics = true;
  bool flagExtrinsics = true;
  bool flagStructure = true;
  bool flagObservations = true;

  po::options_description allParams("AliceVision convertSfMFormat");

  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
      "SfMData file.")
    ("output,o", po::value<std::string>(&outputSfMDataFilename)->required(),
      "Path to the output Alembic file.");

  po::options_description optionalParams("Optional parameters");
  optionalParams.add_options()
    ("views", po::value<bool>(&flagViews)->default_value(flagViews),
      "Export views.")
    ("intrinsics", po::value<bool>(&flagIntrinsics)->default_value(flagIntrinsics),
      "Export intrinsics.")
    ("extrinsics", po::value<bool>(&flagExtrinsics)->default_value(flagExtrinsics),
      "Export extrinsics.")
    ("structure", po::value<bool>(&flagStructure)->default_value(flagStructure),
      "Export structure.")
    ("observations", po::value<bool>(&flagObservations)->default_value(flagObservations),
      "Export observations.");

  po::options_description logParams("Log parameters");
  logParams.add_options()
    ("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
      "verbosity level (fatal,  error, warning, info, debug, trace).");

  allParams.add(requiredParams).add(optionalParams).add(logParams);

  po::variables_map vm;
  try
  {
    po::store(po::parse_command_line(argc, argv, allParams), vm);

    if(vm.count("help") || (argc == 1))
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

  if (sfmDataFilename.empty() || outputSfMDataFilename.empty())
  {
    ALICEVISION_LOG_ERROR("Invalid input or output filename");
    return EXIT_FAILURE;
  }

  int flags = (flagViews   ? VIEWS        : 0)
       | (flagIntrinsics   ? INTRINSICS   : 0)
       | (flagExtrinsics   ? EXTRINSICS   : 0)
       | (flagObservations ? OBSERVATIONS : 0)
       | (flagStructure    ? STRUCTURE    : 0);

  flags = (flags) ? flags : ALL;

  // load input SfMData scene
  SfMData sfmData;
  if (!Load(sfmData, sfmDataFilename, ESfMData::ALL))
  {
    ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFilename << "' cannot be read");
    return EXIT_FAILURE;
  }

  // export the SfMData scene in the expected format
  if(!Save(sfmData, outputSfMDataFilename, ESfMData(flags)))
  {
    ALICEVISION_LOG_ERROR("An error occured while trying to save '" << outputSfMDataFilename << "'");
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
