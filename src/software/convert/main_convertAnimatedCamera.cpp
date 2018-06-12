// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfm/AlembicExporter.hpp>
#include <aliceVision/sfm/sfmDataIO_gt.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>

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
      "SfMData gt file.")
    ("output,o", po::value<std::string>(&outputSfMDataFilename)->required(),
      "Path to the output Alembic file.");

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

  ALICEVISION_COUT("Program called with the following parameters:");
  ALICEVISION_COUT(vm);

  // set verbose level
  system::Logger::get()->setLogLevel(verboseLevel);


  if (sfmDataFilename.empty() || outputSfMDataFilename.empty())
  {
    ALICEVISION_LOG_ERROR("Invalid input or output filename");
    return EXIT_FAILURE;
  }

  // load input SfMData scene
  SfMData sfmData;
  if (!readGt(sfmDataFilename, sfmData, false))
  {
    ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFilename << "' cannot be read");
    return EXIT_FAILURE;
  }

  // init alembic exporter
  sfm::AlembicExporter exporter(outputSfMDataFilename);
  exporter.initAnimatedCamera("camera");

  for(const auto &iter : sfmData.getViews())
  {
    const auto &view = iter.second;
    const geometry::Pose3 pose_gt = sfmData.getPose(*view).getTransform();
    std::shared_ptr<camera::IntrinsicBase> intrinsic_gt = std::make_shared<camera::Pinhole>();
    intrinsic_gt = sfmData.getIntrinsics().at(view->getIntrinsicId());
    exporter.addCameraKeyframe(pose_gt, dynamic_cast<camera::Pinhole*>(intrinsic_gt.get()), view->getImagePath(), view->getViewId(), view->getIntrinsicId());
  }
  return EXIT_SUCCESS;
}
