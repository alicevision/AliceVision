// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "aliceVision/sfm/sfm.hpp"
#include "aliceVision/sfm/utils/uid.hpp"
#include <aliceVision/config.hpp>

#include "dependencies/stlplus3/filesystemSimplified/file_system.hpp"

#include <boost/program_options.hpp>
#include <boost/system/error_code.hpp>
#include <boost/filesystem.hpp>

#include <string>
#include <vector>

using namespace aliceVision;
using namespace aliceVision::sfm;
namespace po = boost::program_options;

// Convert from a SfMData format to another
int main(int argc, char **argv)
{
  // command-line parameters

  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  std::string sfmDataFilename;
  std::string outputSfMDataFilename;
  std::string matchesFolder;

  // user optional parameters

  bool flagViews = true;
  bool flagIntrinsics = true;
  bool flagExtrinsics = true;
  bool flagStructure = true;
  bool flagObservations = true;
  bool recomputeUID = false;

  po::options_description allParams("AliceVision convertSfMFormat");

  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
      "SfMData file.")
    ("output,o", po::value<std::string>(&outputSfMDataFilename)->required(),
      "Path to the output Alembic file.")
    ("matchesFolder,m", po::value<std::string>(&matchesFolder)->required(),
      "Path to a folder in which computed matches are stored.");

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
      "Export observations.")
    ("regenerateUID", po::value<bool>(&recomputeUID)->default_value(recomputeUID),
      "Regenerate UID.");

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

  // set verbose level
  system::Logger::get()->setLogLevel(verboseLevel);

  if (sfmDataFilename.empty() || outputSfMDataFilename.empty())
  {
    std::cerr << "Invalid input or output filename." << std::endl;
    return EXIT_FAILURE;
  }

  // OptionSwitch is cloned in cmd.add(),
  // so we must use cmd.used() instead of testing OptionSwitch.used
  int flags =
    (flagViews ? VIEWS      : 0)
  | (flagIntrinsics ? INTRINSICS : 0)
  | (flagExtrinsics ? EXTRINSICS : 0)
  | (flagObservations ? OBSERVATIONS : 0)
  | (flagStructure ? STRUCTURE  : 0);

  flags = (flags) ? flags : ALL;

  // Load input SfMData scene
  SfMData sfm_data;
  if (!Load(sfm_data, sfmDataFilename, ESfMData(ALL)))
  {
    std::cerr << std::endl
      << "The input SfMData file \"" << sfmDataFilename << "\" cannot be read." << std::endl;
    return EXIT_FAILURE;
  }
  
  if(recomputeUID)
  {
    std::cout << "Recomputing the UID of the views..." << std::endl;
    std::map<std::size_t, std::size_t> oldIdToNew;
    regenerateUID(sfm_data, oldIdToNew);
    
    if(!matchesFolder.empty())
    {
      std::cout << "Generating alias for .feat and .desc with the UIDs" << std::endl;
      for(const auto& iter : oldIdToNew)
      {
        const auto oldID = iter.first;
        const auto newID = iter.second;
        
        // nothing to do if the ids are the same
        if(oldID == newID)
          continue;
        
        const auto oldFeatfilename = stlplus::create_filespec(matchesFolder, std::to_string(oldID), ".feat");
        const auto newFeatfilename = stlplus::create_filespec(matchesFolder, std::to_string(newID), ".feat");
        const auto oldDescfilename = stlplus::create_filespec(matchesFolder, std::to_string(oldID), ".desc");
        const auto newDescfilename = stlplus::create_filespec(matchesFolder, std::to_string(newID), ".desc");

        if(!(stlplus::is_file(oldFeatfilename) && stlplus::is_file(oldDescfilename)))
        {
          std::cerr << "Cannot find the features file for view ID " << oldID
                      << std::endl;
          return EXIT_FAILURE;
        }
        boost::system::error_code ec;
        boost::filesystem::create_symlink(oldFeatfilename, newFeatfilename, ec);
        if(ec)
        {
          std::cerr << "Error while creating " << newFeatfilename << ": " << ec.message() << std::endl;
          return EXIT_FAILURE;
        }
        boost::filesystem::create_symlink(oldDescfilename, newDescfilename, ec);
        if(ec)
        {
          std::cerr << "Error while creating " << newDescfilename << ": " << ec.message() << std::endl;
          return EXIT_FAILURE;
        }
      }
    }
  }

  // Export the SfMData scene in the expected format
  if (!Save(sfm_data, outputSfMDataFilename, ESfMData(flags)))
  {
    std::cerr << std::endl
      << "An error occured while trying to save \"" << outputSfMDataFilename << "\"." << std::endl;
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
