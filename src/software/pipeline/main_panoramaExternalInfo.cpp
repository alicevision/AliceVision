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
  std::string externalInfoFilename = "";
  std::string sfmInputDataFilename = "";
  std::string sfmOutputDataFilename = "";

  /*****
   * DESCRIBE COMMAND LINE PARAMETERS 
   */
  po::options_description allParams(
    "Parse external information about cameras used in a panorama.\n"
    "AliceVision PanoramaExternalInfo");

  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("config,c", po::value<std::string>(&externalInfoFilename)->required(), "External info xml file.")
    ("input,i", po::value<std::string>(&sfmInputDataFilename)->required(), "SfMData file input.")
    ("outSfMDataFilename,o", po::value<std::string>(&sfmOutputDataFilename)->required(), "SfMData file output.")
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


  pt::ptree tree;

  try {
    pt::read_xml(externalInfoFilename, tree);
  }
  catch (...) {
    ALICEVISION_CERR("Error parsing input file");
  }

  pt::ptree lens = tree.get_child("papywizard.header.lens");
  pt::ptree shoot = tree.get_child("papywizard.shoot");

  std::string lensType = lens.get<std::string>("<xmlattr>.type");
  // double lensFocal = lens.get<double>("focal");

  /*Make sure we control everything for debug purpose*/
  if (lensType != "rectilinear") {
    ALICEVISION_CERR("Lens type not supported: " << lensType);
    return EXIT_FAILURE;
  }
  

  std::map<int, Eigen::Matrix3d> rotations;
  
  for (auto it : shoot) {

    int id = it.second.get<double>("<xmlattr>.id");
    int bracket = it.second.get<double>("<xmlattr>.bracket");

    if (rotations.find(id) != rotations.end()) {
      ALICEVISION_CERR("Multiple xml attributes with a same id: " << id);
      return EXIT_FAILURE;
    }

    double yaw_degree = it.second.get<double>("position.<xmlattr>.yaw");
    double pitch_degree = it.second.get<double>("position.<xmlattr>.pitch");
    double roll_degree = it.second.get<double>("position.<xmlattr>.roll");

    double yaw = degreeToRadian(yaw_degree);
    double pitch = degreeToRadian(pitch_degree);
    double roll = degreeToRadian(roll_degree);

    Eigen::AngleAxis<double> Myaw(yaw, Eigen::Vector3d::UnitY());
    Eigen::AngleAxis<double> Mpitch(pitch, Eigen::Vector3d::UnitX());
    Eigen::AngleAxis<double> Mroll(roll, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxis<double> Mimage(-M_PI_2, Eigen::Vector3d::UnitZ());

    Eigen::Matrix3d cRo = Myaw.toRotationMatrix() * Mpitch.toRotationMatrix() *  Mroll.toRotationMatrix() * Mimage.toRotationMatrix()  ;

    rotations[id] = cRo.transpose();
  }

  
  /**
   * Update sfm accordingly
   */
  sfmData::SfMData sfmData;
  if(!sfmDataIO::Load(sfmData, sfmInputDataFilename, sfmDataIO::ESfMData(sfmDataIO::VIEWS|sfmDataIO::INTRINSICS|sfmDataIO::EXTRINSICS)))
  {
    ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmInputDataFilename << "' cannot be read.");
    return EXIT_FAILURE;
  }

  if (sfmData.getViews().size() != rotations.size()) {
    ALICEVISION_LOG_ERROR("The input SfMData has an incorrect number of views.");
    return EXIT_FAILURE; 
  }


  /**
   * HEURISTIC : 
   * The xml file describe rotations for view ids which are not correlated with Alicevision view id
   * Let assume that the order of xml views ids is the lexicographic order of the image names.
  */
  std::vector<std::pair<std::string, int>> names_with_id;
  for (auto v : sfmData.getViews()) {
    names_with_id.push_back(std::make_pair(v.second->getImagePath(), v.first));
  }
  std::sort(names_with_id.begin(), names_with_id.end());

  size_t index = 0;
  for (auto &item_rotation: rotations) {

    IndexT viewIdx = names_with_id[index].second;


    if (item_rotation.second.trace() != 0) {
      sfmData::CameraPose pose(geometry::Pose3 (item_rotation.second, Eigen::Vector3d::Zero()));
      sfmData.setAbsolutePose(viewIdx, pose);
    }
        
    index++;
  }
  

  if (!sfmDataIO::Save(sfmData, sfmOutputDataFilename, sfmDataIO::ESfMData(sfmDataIO::VIEWS | sfmDataIO::INTRINSICS | sfmDataIO::EXTRINSICS))) {
    ALICEVISION_LOG_ERROR("The output SfMData file '" << sfmOutputDataFilename << "' cannot be write.");
    return EXIT_FAILURE;
  }

  return 0;
}
