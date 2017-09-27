// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#include "aliceVision/system/Timer.hpp"

#include "software/utils/sfmColorHarmonize/colorHarmonizeEngineGlobal.hpp"

#include "dependencies/stlplus3/filesystemSimplified/file_system.hpp"

#include <boost/program_options.hpp>

#include <cstdlib>
#include <memory>

using namespace aliceVision;
using namespace std;
namespace po = boost::program_options;

int main( int argc, char **argv )
{
  // command-line parameters

  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  std::string sfmDataFilename;
  std::string matchesDirectory;
  std::string describerTypesName = feature::EImageDescriberType_enumToString(feature::EImageDescriberType::SIFT);
  std::string matchesGeometricModel = "f";
  std::string outputDirectory ;
  int selectionMethod;
  int imgRef;

  po::options_description allParams("AliceVision sfmColorHarmonize");

  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
      "SfMData file.")
    ("output,o", po::value<std::string>(&outputDirectory)->required(),
      "Output path.")
    ("matchesDirectory,m", po::value<std::string>(&matchesDirectory)->required(),
      "Path to a directory in which computed matches are stored.")
    ("referenceImage", po::value<int>(&imgRef)->required(),
      "Reference image id.")
    ("selectionMethod", po::value<int>(&selectionMethod)->required(),
      "- 0: FullFrame\n"
      "- 1: Matched Points\n"
      "- 2: VLD Segment");

  po::options_description optionalParams("Optional parameters");
  optionalParams.add_options()
    ("describerTypes,d", po::value<std::string>(&describerTypesName)->default_value(describerTypesName),
      feature::EImageDescriberType_informations().c_str());
    ("matchesGeometricModel,g", po::value<std::string>(&matchesGeometricModel)->default_value(matchesGeometricModel),
      "Matches geometric Model :\n"
      "- f: fundamental matrix\n"
      "- e: essential matrix\n"
      "- h: homography matrix");

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

  if (sfmDataFilename.empty())
  {
    std::cerr << "\nIt is an invalid file input" << std::endl;
    return EXIT_FAILURE;
  }

  const std::vector<feature::EImageDescriberType> describerTypes = feature::EImageDescriberType_stringToEnums(describerTypesName);

  if ( !stlplus::folder_exists( outputDirectory ) )
    stlplus::folder_create( outputDirectory );

  //---------------------------------------
  // Harmonization process
  //---------------------------------------

  aliceVision::system::Timer timer;

  ColorHarmonizationEngineGlobal colorHarmonizeEngine(sfmDataFilename,
    matchesDirectory,
    matchesGeometricModel,
    outputDirectory,
    describerTypes,
    selectionMethod,
    imgRef);

  if (colorHarmonizeEngine.Process() )
  {
    clock_t timeEnd = clock();
    std::cout << std::endl
      << " ColorHarmonization took (s): "
      << timer.elapsed() << std::endl;

    return EXIT_SUCCESS;
  }
  else
  {
    std::cerr << "\n Something goes wrong in the process" << std::endl;
  }
  return EXIT_FAILURE;
}
