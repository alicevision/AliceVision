// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfm/sfm.hpp>
#include <aliceVision/sfm/BundleAdjustmentCeres.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/config.hpp>

#include <USfM/usfm_data_Scene.hpp>
#include <USfM/usfm_IO.hpp>
#include <USfM/usfm_IO_Factory.hpp>
#include <USfM/usfm_Statistics.hpp>
#include <USfM/usfm_Algorithm_Factory.hpp>
#include <USfM/usfm_Projection_Radial3.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <string>
#include <sstream>
#include <vector>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;
using namespace aliceVision::sfm;
using namespace aliceVision::sfmData;
using namespace aliceVision::sfmDataIO;
using namespace aliceVision::camera;

namespace po = boost::program_options;
namespace bf = boost::filesystem;


inline usfm::ECameraModel intrinsic_aliceVision2usfm(EINTRINSIC intrinsicType)
{
  switch(intrinsicType)
  {
  case EINTRINSIC::PINHOLE_CAMERA:
    return usfm::ECameraModel::eSimplePinhole;
  case EINTRINSIC::PINHOLE_CAMERA_RADIAL1:
    return usfm::ECameraModel::eRadial1;
  case EINTRINSIC::PINHOLE_CAMERA_RADIAL3:
    return usfm::ECameraModel::eRadial3;
  case EINTRINSIC::PINHOLE_CAMERA_BROWN:
    throw std::runtime_error("PINHOLE_CAMERA_BROWN not implemented");
  case EINTRINSIC::PINHOLE_CAMERA_FISHEYE:
    throw std::runtime_error("PINHOLE_CAMERA_FISHEYE not implemented");
  case EINTRINSIC::PINHOLE_CAMERA_FISHEYE1:
    throw std::runtime_error("PINHOLE_CAMERA_FISHEYE1 not implemented");
  case EINTRINSIC::PINHOLE_CAMERA_END:
  case EINTRINSIC::PINHOLE_CAMERA_START:
    break;
  }
  throw std::runtime_error("intrinsic_aliceVision2usfm not defined");
}


int main(int argc, char **argv)
{
  // command-line parameters

  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  std::string sfmDataFilename;
  std::string outSfMDataFilename;
  std::string outputStats;
  std::string algorithm = "NBUP"; // cov::EAlgorithm_enumToString(cov::eAlgorithmSvdTaylorExpansion);
  bool debug = false;

  po::options_description params("AliceVision Uncertainty");
  params.add_options()
    ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
      "SfMData file to align.")
  ("output,o", po::value<std::string>(&outSfMDataFilename)->required(),
    "Output SfMData scene.")
  ("outputCov,c", po::value<std::string>(&outputStats),
    "Output covariances file.")
  ("algorithm,a", po::value<std::string>(&algorithm)->default_value(algorithm),
    "Algorithm.")
  ("debug,d", po::value<bool>(&debug)->default_value(debug),
    "Enable creation of debug files in the current folder.")
    ("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
      "verbosity level (fatal,  error, warning, info, debug, trace).");

  po::variables_map vm;
  try
  {
    po::store(po::parse_command_line(argc, argv, params), vm);

    if(vm.count("help") || (argc == 1))
    {
      ALICEVISION_COUT(params);
      return EXIT_SUCCESS;
    }
    po::notify(vm);
  }
  catch(boost::program_options::required_option& e)
  {
    ALICEVISION_CERR("ERROR: " << e.what());
    ALICEVISION_COUT("Usage:\n\n" << params);
    return EXIT_FAILURE;
  }
  catch(boost::program_options::error& e)
  {
    ALICEVISION_CERR("ERROR: " << e.what());
    ALICEVISION_COUT("Usage:\n\n" << params);
    return EXIT_FAILURE;
  }
  ALICEVISION_COUT(vm);

  // set verbose level
  system::Logger::get()->setLogLevel(verboseLevel);
  
  if (sfmDataFilename.empty() ||
      outSfMDataFilename.empty())
  {
    std::cerr << "Invalid input or output filename." << std::endl;
    return EXIT_FAILURE;
  }

  // Load input scene
  SfMData sfmData;
  if (!Load(sfmData, sfmDataFilename, ESfMData(ALL)))
  {
    std::cerr << std::endl
      << "The input SfMData file \"" << sfmDataFilename << "\" cannot be read." << std::endl;
    return EXIT_FAILURE;
  }

  usfm::Scene uScene;

  for(auto& intrinsicIt: sfmData.intrinsics)
  {
    std::vector<double> p = intrinsicIt.second->getParams();
    p.insert(p.begin(), (double)intrinsicIt.second->h());
    p.insert(p.begin(), (double)intrinsicIt.second->w());

    uScene._cameras[intrinsicIt.first] = usfm::Camera(intrinsicIt.first, intrinsic_aliceVision2usfm(intrinsicIt.second->getType()), p);
  }

  for(auto& viewIt: sfmData.views)
  {
    if(sfmData.isPoseAndIntrinsicDefined(viewIt.second.get()))
    {
      usfm::Image& img = uScene._images[viewIt.first];
      usfm::ProjectionRadial3 proj;
      proj.initOffsets(&img);

      img._id = viewIt.first;
      img._cam_id = viewIt.second->getIntrinsicId();

      // 
      const geometry::Pose3& pose = sfmData.getPose(*viewIt.second).getTransform();
      Vec3 c = pose.center();

      // img._offset_in_parameters[usfm::e_aa] = 0;
      // img._offset_in_parameters[usfm::e_C] = 3;
      img._parameters.resize(22); // 6?
      ceres::RotationMatrixToAngleAxis(pose.rotation().data(), img._parameters.data()); // 3 angles axis parameters first
      img._parameters[3] = c(0);
      img._parameters[4] = c(1);
      img._parameters[5] = c(2);

      img.setModel(usfm::eAAC);
      // img._point2D will be filled later when iterating on landmarks
    }
  }

  for(auto& landmarkIt: sfmData.structure)
  {
    usfm::Point3D p;
    p._id = landmarkIt.first;
    std::memcpy(p._X, landmarkIt.second.X.data(), 3*sizeof(double));
    uScene._points3D[landmarkIt.first] = p;

    for(auto& obsIt: landmarkIt.second.observations)
    {
      usfm::Point2D p;
      p._id_point3D = landmarkIt.first;
      p._xy[0] = obsIt.second.x(0);
      p._xy[1] = obsIt.second.x(1);

      IndexT viewId = obsIt.first;
      uScene._images[viewId]._point2D.push_back(p); 
    }
  }

  uScene.setInputCovarianceEstimator("UNIT");

  ALICEVISION_LOG_INFO(uScene);

  // init algorithm
  std::shared_ptr<usfm::Algorithm> alg = usfm::Algorithm_Factory::initAlgorithm(algorithm);

  usfm::Statistic statistic;
  // run algorithm
  alg->compute(uScene, statistic);

  usfm::IO io;
  // save the results
  std::string outputFolder = bf::path(outputStats).parent_path().string();
  io.writeCov2File(outputFolder, uScene, statistic);

  std::cout << "Export cov into \"" << outputFolder << "\"" << std::endl;

  // Retrieve uncertainty matrices from usfm and copy them into aliceVision::sfmData
  {
    int matrix_offset = 0;
    for (auto &img : uScene._images)
    {
      std::shared_ptr<usfm::Projection> proj = usfm::Projection_Factory::createProjection(NULL, &img.second, NULL, NULL);
      int num_img_params = proj->numImgParams();
      usfm::DM img_cov_eig = uScene._iZ.block(matrix_offset+3, matrix_offset+3, 3, 3);
      matrix_offset += num_img_params;

      IndexT poseId = sfmData.views.at(img.second._id)->getPoseId();
      sfmData._posesUncertainty[poseId] = img_cov_eig;
    }

    auto pointIt = uScene._points3D.begin();
    for (int k = 0; k < uScene._cov_pts.size(); ++k, ++pointIt)
    {
      sfmData._landmarksUncertainty[pointIt->first] = uScene._cov_pts[k];
    }
  }

  std::cout << "Save into \"" << outSfMDataFilename << "\"" << std::endl;

  // Export the SfMData scene in the expected format
  if (!Save(sfmData, outSfMDataFilename, ESfMData(ALL)))
  {
    std::cerr << std::endl
      << "An error occurred while trying to save \"" << outSfMDataFilename << "\"." << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
