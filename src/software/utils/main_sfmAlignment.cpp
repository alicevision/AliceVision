// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/sfm/utils/alignment.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/config.hpp>

#include <boost/program_options.hpp>

#include <string>
#include <sstream>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;
using namespace aliceVision::sfm;

namespace po = boost::program_options;

int main(int argc, char **argv)
{
  // command-line parameters

  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  std::string sfmDataFilename;
  std::string outSfMDataFilename;
  std::string sfmDataReferenceFilename;

  po::options_description allParams("AliceVision sfmAlignment");

  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
      "SfMData file to align.")
    ("output,o", po::value<std::string>(&outSfMDataFilename)->required(),
      "Output SfMData scene.")
    ("reference,r", po::value<std::string>(&sfmDataReferenceFilename)->required(),
      "Path to the scene used as the reference coordinate system.");

  po::options_description logParams("Log parameters");
  logParams.add_options()
    ("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
      "verbosity level (fatal,  error, warning, info, debug, trace).");

  allParams.add(requiredParams).add(logParams);

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

  // Load input scene
  sfmData::SfMData sfmDataIn;
  if(!sfmDataIO::Load(sfmDataIn, sfmDataFilename, sfmDataIO::ESfMData::ALL))
  {
    ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFilename << "' cannot be read");
    return EXIT_FAILURE;
  }

  // Load reference scene
  sfmData::SfMData sfmDataInRef;
  if(!sfmDataIO::Load(sfmDataInRef, sfmDataReferenceFilename, sfmDataIO::ESfMData::ALL))
  {
    ALICEVISION_LOG_ERROR("The reference SfMData file '" << sfmDataReferenceFilename << "' cannot be read");
    return EXIT_FAILURE;
  }

  ALICEVISION_LOG_INFO("Search similarity transformation.");

  double S;
  Mat3 R;
  Vec3 t;
  bool hasValidSimilarity = sfm::computeSimilarity(sfmDataIn, sfmDataInRef, &S, &R, &t);

  if(!hasValidSimilarity)
  {
    std::stringstream ss;
    ss << "Failed to find similarity between the 2 SfM scenes:";
    ss << "\t- " << sfmDataFilename << std::endl;
    ss << "\t- " << sfmDataReferenceFilename << std::endl;
    ALICEVISION_LOG_ERROR(ss.str());
    return EXIT_FAILURE;
  }

  {
    std::stringstream ss;
    ss << "Transformation:" << std::endl;
    ss << "\t- Scale: " << S << std::endl;
    ss << "\t- Rotation:\n" << R << std::endl;
    ss << "\t- Translate: " << t.transpose() << std::endl;
    ALICEVISION_LOG_INFO(ss.str());
  }

  sfm::applyTransform(sfmDataIn, S, R, t);

  ALICEVISION_LOG_INFO("Save into '" << outSfMDataFilename << "'");
  
  // Export the SfMData scene in the expected format
  if(!sfmDataIO::Save(sfmDataIn, outSfMDataFilename, sfmDataIO::ESfMData::ALL))
  {
    ALICEVISION_LOG_ERROR("An error occurred while trying to save '" << outSfMDataFilename << "'");
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
