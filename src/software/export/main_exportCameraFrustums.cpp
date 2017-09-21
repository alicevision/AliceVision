// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#include "aliceVision/sfm/sfm.hpp"
#include "aliceVision/system/Timer.hpp"

#include "dependencies/cmdLine/cmdLine.h"
#include "dependencies/stlplus3/filesystemSimplified/file_system.hpp"

using namespace aliceVision;
using namespace aliceVision::sfm;

/// Export camera frustrums as a triangle PLY file
int main(int argc, char **argv)
{
  using namespace std;
  std::cout << "Export camera frustums" << std::endl;

  CmdLine cmd;

  std::string sSfMData_Filename;
  std::string sOutFile = "";

  cmd.add( make_option('i', sSfMData_Filename, "input_file") );
  cmd.add( make_option('o', sOutFile, "output_file") );

  try {
    if (argc == 1) throw std::string("Invalid command line parameter.");
    cmd.process(argc, argv);
  } catch(const std::string& s) {
    std::cerr << "Usage: " << argv[0] << '\n'
    << "[-i|--input_file] path to a SfMData scene\n"
    << "[-o|--output_file] PLY file to store the camera frustums as triangle meshes.\n"
    << std::endl;

    std::cerr << s << std::endl;
    return EXIT_FAILURE;
  }

  // Load input SfMData scene
  SfMData sfm_data;
  if (!Load(sfm_data, sSfMData_Filename, ESfMData(VIEWS|INTRINSICS|EXTRINSICS))) {
    std::cerr << std::endl
      << "The input SfMData file \""<< sSfMData_Filename << "\" cannot be read." << std::endl;
    return EXIT_FAILURE;
  }

  // Assert that we can create the output directory/file
  if (!stlplus::folder_exists( stlplus::folder_part(sOutFile) ))
    if(!stlplus::folder_create( stlplus::folder_part(sOutFile) ))
      return EXIT_FAILURE;

  // If sfm_data have not structure, cameras are displayed as tiny normalized cones
  const FrustumFilter frustum_filter(sfm_data);
  if (!sOutFile.empty())
  {
    if (frustum_filter.export_Ply(sOutFile))
      return EXIT_SUCCESS;
  }
  return EXIT_FAILURE;
}
