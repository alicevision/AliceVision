// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfm/sfm.hpp>
#include <aliceVision/config.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>

#include <dependencies/stlplus3/filesystemSimplified/file_system.hpp>

#include <boost/program_options.hpp>

#include <string>
#include <vector>

using namespace aliceVision;
using namespace aliceVision::image;
using namespace aliceVision::sfm;
namespace po = boost::program_options;

// Convert from a SfMData format to another
int main(int argc, char **argv)
{
  // command-line parameters

  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  std::string sfmDataFilename;
  std::string outputSfMDataFilename;

  po::options_description allParams("AliceVision computeSfMColor");

  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
      "SfMData file.")
    ("output,o", po::value<std::string>(&outputSfMDataFilename)->required(),
      "Output SfMData filename (.json, .bin, .xml, .ply, .baf"
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_ALEMBIC)
      ", .abc"
#endif
      ").");

  po::options_description logParams("Log parameters");
  logParams.add_options()
    ("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
      "verbosity level (fatal, error, warning, info, debug, trace).");

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

  if (outputSfMDataFilename.empty())
  {
    std::cerr << std::endl
      << "No output filename specified." << std::endl;
    return EXIT_FAILURE;
  }

  // Load input SfMData scene
  SfMData sfm_data;
  std::cout << "Loading sfm data from " << sfmDataFilename << "..." << std::endl;
  if (!Load(sfm_data, sfmDataFilename, ESfMData(ALL)))
  {
    std::cerr << std::endl
      << "The input SfMData file \"" << sfmDataFilename << "\" cannot be read." << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << "Done!" << std::endl;

  // Compute the scene structure color
  if (!ColorizeTracks(sfm_data))
  {
    std::cerr << "Error while trying to colorize the tracks! Aborting..." << std::endl;
    return EXIT_FAILURE;
  }

  // Export the SfMData scene in the expected format
  std::cout << "Saving output result to " << outputSfMDataFilename << "..." << std::endl;
  if (!Save(sfm_data, outputSfMDataFilename.c_str(), ESfMData(ALL)))
  {
    std::cerr << std::endl
      << "An error occured while trying to save \"" << outputSfMDataFilename << "\"." << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << "Done!" << std::endl;

  return EXIT_SUCCESS;
}
