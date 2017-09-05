// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#include "openMVG/sfm/sfm.hpp"
#include <openMVG/config.hpp>

#include "third_party/cmdLine/cmdLine.h"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

#include <string>
#include <vector>

using namespace openMVG;
using namespace openMVG::image;
using namespace openMVG::sfm;

// Convert from a SfM_Data format to another
int main(int argc, char **argv)
{
  CmdLine cmd;

  std::string sSfM_Data_Filename_In;
  std::string sOutput_file;

  cmd.add(make_option('i', sSfM_Data_Filename_In, "input_file"));
  cmd.add(make_option('o', sOutput_file, "output_file"));

  try {
      if (argc == 1) throw std::string("Invalid command line parameter.");
      cmd.process(argc, argv);
  } catch(const std::string& s) {
      std::cerr << "Usage: " << argv[0] << '\n'
        << "[-i|--input_file] path to the input SfM_Data scene\n"
        << "[-o|--output_file] path to the output SfM_Data scene\n"
        << "\t .json, .bin, .xml, .ply, .baf"
#if OPENMVG_IS_DEFINED(OPENMVG_HAVE_ALEMBIC)
           ", .abc"
#endif
        << std::endl;

      std::cerr << s << std::endl;
      return EXIT_FAILURE;
  }

  if (sOutput_file.empty())
  {
    std::cerr << std::endl
      << "No output filename specified." << std::endl;
    return EXIT_FAILURE;
  }

  // Load input SfM_Data scene
  SfM_Data sfm_data;
  std::cout << "Loading sfm data from " << sSfM_Data_Filename_In << "..." << std::endl;
  if (!Load(sfm_data, sSfM_Data_Filename_In, ESfM_Data(ALL)))
  {
    std::cerr << std::endl
      << "The input SfM_Data file \"" << sSfM_Data_Filename_In << "\" cannot be read." << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << "Done!" << std::endl;

  // Compute the scene structure color
  if (!ColorizeTracks(sfm_data))
  {
    std::cerr << "Error while trying to colorize the tracks! Aborting..." << std::endl;
    return EXIT_FAILURE;
  }

  // Export the SfM_Data scene in the expected format
  std::cout << "Saving output result to " << sOutput_file << "..." << std::endl;
  if (!Save(sfm_data, sOutput_file.c_str(), ESfM_Data(ALL)))
  {
    std::cerr << std::endl
      << "An error occured while trying to save \"" << sOutput_file << "\"." << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << "Done!" << std::endl;

  return EXIT_SUCCESS;
}
