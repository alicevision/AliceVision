#include <aliceVision/image/all.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 2
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;
namespace po = boost::program_options;
namespace fs = boost::filesystem;
namespace pt = boost::property_tree;

int main(int argc, char * argv[]) {

  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  std::string sfmInputDataFilename = "";
  std::string sfmOutputDataFilename = "";
  double offset_x = 0.0;
  double offset_y = 0.0;
  double radius = 0.0;

  /*****
   * DESCRIBE COMMAND LINE PARAMETERS 
   */
  po::options_description allParams(
    "Parse external information about cameras used in a panorama.\n"
    "AliceVision PanoramaExternalInfo");

  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmInputDataFilename)->required(), "SfMData file input.")
    ("outSfMDataFilename,o", po::value<std::string>(&sfmOutputDataFilename)->required(), "SfMData file output.")
    ("offsetx,ox", po::value<double>(&offset_x)->required(), "Fisheye circle's center offset in X (pixels).")
    ("offsety,oy", po::value<double>(&offset_y)->required(), "Fisheye circle's center offset in Y (pixels).")
    ("radius,r", po::value<double>(&radius)->required(), "Fisheye circle's radius (% of image shortest side).")
    ;

  po::options_description optionalParams("Optional parameters");

  po::options_description logParams("Log parameters");
  logParams.add_options()
    ("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
      "verbosity level (fatal, error, warning, info, debug, trace).");
  
  allParams.add(requiredParams).add(optionalParams).add(logParams);


  /**
   * READ COMMAND LINE
   */
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

  /**
   *  set verbose level
   **/
  system::Logger::get()->setLogLevel(verboseLevel);


  
  /**
   * Update sfm accordingly
   */
  sfmData::SfMData sfmInput;
  if(!sfmDataIO::Load(sfmInput, sfmInputDataFilename, sfmDataIO::ESfMData(sfmDataIO::VIEWS|sfmDataIO::INTRINSICS|sfmDataIO::EXTRINSICS)))
  {
    ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmInputDataFilename << "' cannot be read.");
    return EXIT_FAILURE;
  }

  
  int count_equidistant = 0;

  sfmData::Intrinsics & intrinsics = sfmInput.getIntrinsics();
  for (auto & intrinsic_pair : intrinsics) {
    std::shared_ptr<camera::IntrinsicBase> intrinsic = intrinsic_pair.second;
    std::shared_ptr<camera::EquiDistant> equidistant = std::dynamic_pointer_cast<camera::EquiDistant>(intrinsic);
    if (equidistant == nullptr) {
      continue;
    }

    equidistant->setCenterX(double(equidistant->w()) / 2.0 + offset_x);
    equidistant->setCenterY(double(equidistant->h()) / 2.0 + offset_y);
    equidistant->setRadius(radius / 100.0 * 0.5 * std::min(double(equidistant->w()),double(equidistant->h())));


    count_equidistant++;
  }

  if (count_equidistant == 0) {
    ALICEVISION_LOG_ERROR("None of the camera are equidistant.");
    return EXIT_FAILURE;
  }

  ALICEVISION_LOG_INFO(count_equidistant << " cameras intrinsics have been updated");

  if (!sfmDataIO::Save(sfmInput, sfmOutputDataFilename, sfmDataIO::ESfMData(sfmDataIO::VIEWS | sfmDataIO::INTRINSICS | sfmDataIO::EXTRINSICS))) {
    ALICEVISION_LOG_ERROR("The output SfMData file '" << sfmOutputDataFilename << "' cannot be write.");
    return EXIT_FAILURE;
  }

  return 0;
}
