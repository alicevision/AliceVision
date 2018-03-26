// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.
#include <aliceVision/registration/PointcloudRegistration.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <pcl/console/parse.h>

using namespace aliceVision;
using namespace aliceVision::registration;

namespace po = boost::program_options;

int main(int argc, char** argv)
{
	po::options_description allParams(
    "3D 3D registration.\n"
    "Perform registration of 3D models (e.g. SfM & LiDAR model).\n"
    "AliceVision 3d3dRegistration");

  std::string sourceFile, targetFile;
  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("sourceFile,s", po::value<std::string>(&sourceFile)->required(),
      "Path to file source (fixed) 3D model.")
    ("targetFile,t", po::value<std::string>(&targetFile)->required(),
      "Path to the target (moveing) 3D model.");

  float sourceMeasurement = 1.f,
      targetMeasurement = 1.f,
      scaleRatio = 1.f,
      voxelSize = 0.1f;
  po::options_description optionalParams("Optional parameters");
  optionalParams.add_options()
    ("scaleRatio", po::value<float>(&scaleRatio)->default_value(scaleRatio),
      "Scale ratio between the two 3D models (= target size / source size)")
    ("sourceMeasurement", po::value<float>(&sourceMeasurement)->default_value(sourceMeasurement),
      "Measurement made ont the source 3D model (same unit as 'targetMeasurement'). It allowes to compute the scale ratio between 3D models.")
    ("targetMeasurement", po::value<float>(&targetMeasurement)->default_value(targetMeasurement),
      "Measurement made ont the target 3D model (same unit as 'sourceMeasurement'). It allowes to compute the scale ratio between 3D models.")
    ("voxelSize", po::value<float>(&voxelSize)->default_value(voxelSize),
      "Size of the voxel grid applied on each 3D model to downsample them. Downsampling reduces computing duration.");
    ;

  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  po::options_description logParams("Log parameters");
  logParams.add_options()
    ("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
      "verbosity level (fatal, error, warning, info, debug, trace).");

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

  ALICEVISION_COUT("Program called with the following parameters:");
  ALICEVISION_COUT(vm);

	// ===========================================================
	// -- Run alignement
	// ===========================================================

	PointcloudRegistration pa;

	if (pa.loadSourceCloud(sourceFile) == EXIT_FAILURE)
    return EXIT_FAILURE;

	if (pa.loadTargetCloud(targetFile) == EXIT_FAILURE)
    return EXIT_FAILURE;

	if (pa.setSourceMeasurements(sourceMeasurement) == EXIT_FAILURE)
		return EXIT_FAILURE;

	if (pa.setTargetMeasurements(targetMeasurement) == EXIT_FAILURE)
		return EXIT_FAILURE;

	pa.setVoxelSize(voxelSize);
	pa.setVerbose(true);
	pa.setShowPipeline(true);

	pcl::PointCloud<pcl::PointXYZ> regGICP_source;

	pa.align(regGICP_source);

	Eigen::Matrix4f T = pa.getFinalTransformation();
	std::cout << "Transformation (such as: T * source = target) =\n" << T << std::endl;
	
	return EXIT_SUCCESS;
}
