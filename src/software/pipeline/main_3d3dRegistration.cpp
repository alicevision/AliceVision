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
      "Path to file source (moving) 3D model.")
    ("targetFile,t", po::value<std::string>(&targetFile)->required(),
      "Path to the target (fixed) 3D model.");

  // to choose which transformed 3D model to export:
  std::string outputFile;
  
  float sourceMeasurement = 1.f,
      targetMeasurement = 1.f,
      scaleRatio = 1.f,
      voxelSize = 0.1f;
  bool showPipeline = false, 
      showTimeline = true;

  po::options_description optionalParams("Optional parameters");
  optionalParams.add_options()
    ("outputFile,o", po::value<std::string>(&outputFile),
      "Path to save the transformed source 3D model.")
    ("scaleRatio", po::value<float>(&scaleRatio)->default_value(scaleRatio),
      "Scale ratio between the two 3D models (= target size / source size)")
    ("sourceMeasurement", po::value<float>(&sourceMeasurement)->default_value(sourceMeasurement),
      "Measurement made ont the source 3D model (same unit as 'targetMeasurement'). It allowes to compute the scale ratio between 3D models.")
    ("targetMeasurement", po::value<float>(&targetMeasurement)->default_value(targetMeasurement),
      "Measurement made ont the target 3D model (same unit as 'sourceMeasurement'). It allowes to compute the scale ratio between 3D models.")
    ("voxelSize", po::value<float>(&voxelSize)->default_value(voxelSize),
      "Size of the voxel grid applied on each 3D model to downsample them. Downsampling reduces computing duration.")
    ("showPipeline", po::value<bool>(&showPipeline)->default_value(showPipeline),
      "To enable the visualization of each step of the pipeline in external windows.");
    ("showPipeline", po::value<bool>(&showTimeline)->default_value(showTimeline),
      "To show the duration of each time of the algnment pipeline.");

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

	PointcloudRegistration reg;

	if (reg.loadSourceCloud(sourceFile) == EXIT_FAILURE)
    return EXIT_FAILURE;

	if (reg.loadTargetCloud(targetFile) == EXIT_FAILURE)
    return EXIT_FAILURE;
  
  reg.setScaleRatio(scaleRatio);
  reg.setTargetMeasurement(targetMeasurement);
  reg.setSourceMeasurement(sourceMeasurement);
	reg.setVoxelSize(voxelSize);
	reg.setShowPipeline(showPipeline);

	reg.align();

  if (showTimeline)
    reg.showTimeline();
    
	if (!outputFile.empty()) // export a transformed 3D model
	{
  	Eigen::Matrix4f T = reg.getFinalTransformation();
    return reg.tranformAndSaveCloud(sourceFile, T, outputFile);
	}
	else
    return EXIT_SUCCESS;
}
