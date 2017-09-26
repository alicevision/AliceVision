// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#include <aliceVision/sfm/AlembicExporter.hpp>
#include <aliceVision/sfm/sfmDataIO_gt.hpp>

#include <dependencies/stlplus3/filesystemSimplified/file_system.hpp>

#include <boost/program_options.hpp>

#include <string>
#include <vector>

using namespace aliceVision;
using namespace aliceVision::sfm;
namespace po = boost::program_options;

int main(int argc, char **argv)
{
  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  std::string sfmDataFilename;
  std::string outputSfMDataFilename;

  po::options_description allParams("AliceVision convertAnimatedCamera");

  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
      "SfMData file.")
    ("output,o", po::value<std::string>(&outputSfMDataFilename)->required(),
      "Path to the output Alembic file.");

  allParams.add(requiredParams);

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

  ALICEVISION_COUT("Program called with the following parameters: " << std::endl
    << "\t" << argv[0] << std::endl
    << "\t--input " << sfmDataFilename << std::endl
    << "\t--output " << outputSfMDataFilename << std::endl
    << "\t--verboseLevel "<< verboseLevel);

  // set verbose level
  system::Logger::get()->setLogLevel(verboseLevel);


  if (sfmDataFilename.empty() || outputSfMDataFilename.empty())
  {
    std::cerr << "Invalid input or output filename." << std::endl;
    return EXIT_FAILURE;
  }

  // Load input SfMData scene
  SfMData sfm_data;
  if (!readGt(sfmDataFilename, sfm_data, false))
  {
    std::cerr << std::endl
      << "The input SfMData file \"" << sfmDataFilename << "\" cannot be read." << std::endl;
    return EXIT_FAILURE;
  }

  // init alembic exporter
  sfm::AlembicExporter exporter( outputSfMDataFilename );
  exporter.initAnimatedCamera("camera");

  for(const auto &iter : sfm_data.GetViews())
  {
    const auto &view = iter.second;
    const geometry::Pose3 pose_gt = sfm_data.GetPoses().at(view->getPoseId());
    std::shared_ptr<camera::IntrinsicBase> intrinsic_gt = std::make_shared<camera::Pinhole>();
    intrinsic_gt = sfm_data.GetIntrinsics().at(view->getIntrinsicId());
    exporter.addCameraKeyframe(pose_gt, dynamic_cast<camera::Pinhole*>(intrinsic_gt.get()), view->getImagePath(), view->getViewId(), view->getIntrinsicId());
  }
  return EXIT_SUCCESS;
}
