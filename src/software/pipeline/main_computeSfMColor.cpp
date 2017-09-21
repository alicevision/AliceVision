// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#include "aliceVision/sfm/sfm.hpp"
#include <aliceVision/config.hpp>

#include "dependencies/cmdLine/cmdLine.h"
#include "dependencies/stlplus3/filesystemSimplified/file_system.hpp"

#include <string>
#include <vector>

using namespace aliceVision;
using namespace aliceVision::image;
using namespace aliceVision::sfm;

// Convert from a SfMData format to another
int main(int argc, char **argv)
{
  CmdLine cmd;

  std::string sSfMData_Filename_In;
  std::string sOutput_file;

  cmd.add(make_option('i', sSfMData_Filename_In, "input_file"));
  cmd.add(make_option('o', sOutput_file, "output_file"));

  try {
      if (argc == 1) throw std::string("Invalid command line parameter.");
      cmd.process(argc, argv);
  } catch(const std::string& s) {
      std::cerr << "Usage: " << argv[0] << '\n'
        << "[-i|--input_file] path to the input SfMData scene\n"
        << "[-o|--output_file] path to the output SfMData scene\n"
        << "\t .json, .bin, .xml, .ply, .baf"
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_ALEMBIC)
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

  // Load input SfMData scene
  SfMData sfm_data;
  std::cout << "Loading sfm data from " << sSfMData_Filename_In << "..." << std::endl;
  if (!Load(sfm_data, sSfMData_Filename_In, ESfMData(ALL)))
  {
    std::cerr << std::endl
      << "The input SfMData file \"" << sSfMData_Filename_In << "\" cannot be read." << std::endl;
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
  std::cout << "Saving output result to " << sOutput_file << "..." << std::endl;
  if (!Save(sfm_data, sOutput_file.c_str(), ESfMData(ALL)))
  {
    std::cerr << std::endl
      << "An error occured while trying to save \"" << sOutput_file << "\"." << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << "Done!" << std::endl;

  return EXIT_SUCCESS;
}
