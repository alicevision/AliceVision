// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#include "aliceVision/sfm/sfm.hpp"
#include "aliceVision/system/Timer.hpp"

#include "dependencies/stlplus3/filesystemSimplified/file_system.hpp"

#include <boost/program_options.hpp>

using namespace aliceVision;
using namespace aliceVision::sfm;
using namespace std;
namespace po = boost::program_options;

/// Export camera frustrums as a triangle PLY file
int main(int argc, char **argv)
{
  // command-line parameters

  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  std::string sfmDataFilename;
  std::string plyOutFilename;

  po::options_description allParams("AliceVision exportCameraFrustums");

  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
      "SfMData file.")
    ("output,o", po::value<std::string>(&plyOutFilename)->required(),
      "PLY file to store the camera frustums as triangle meshes.");

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

  // set verbose level
  system::Logger::get()->setLogLevel(verboseLevel);

  // export
  {
    // Load input SfMData scene
    SfMData sfmData;
    if (!Load(sfmData, sfmDataFilename, ESfMData(VIEWS|INTRINSICS|EXTRINSICS))) {
      std::cerr << std::endl
        << "The input SfMData file \""<< sfmDataFilename << "\" cannot be read." << std::endl;
      return EXIT_FAILURE;
    }

    // Assert that we can create the output directory/file
    if (!stlplus::folder_exists( stlplus::folder_part(plyOutFilename) ))
      if(!stlplus::folder_create( stlplus::folder_part(plyOutFilename) ))
        return EXIT_FAILURE;

    // If sfm_data have not structure, cameras are displayed as tiny normalized cones
    const FrustumFilter frustum_filter(sfmData);
    if (!plyOutFilename.empty())
    {
      if (frustum_filter.export_Ply(plyOutFilename))
        return EXIT_SUCCESS;
    }
  }
  return EXIT_FAILURE;
}
