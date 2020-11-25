// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/config.hpp>

#include <software/utils/precisionEvaluationToGt.hpp>
#include <software/utils/sfmHelper/sfmPlyHelper.hpp>

#include <dependencies/htmlDoc/htmlDoc.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <cstdlib>
#include <iostream>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;
namespace fs = boost::filesystem;

int aliceVision_main(int argc, char **argv)
{
  // command-line parameters

  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  std::string sfmDataFilename;
  std::string outputFolder;
  std::string gtFilename;

  po::options_description allParams("AliceVision qualityEvaluation");

  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
      "SfMData file.")
    ("output,o", po::value<std::string>(&outputFolder)->required(),
      "Output path for statistics.")
    ("groundTruthPath", po::value<std::string>(&gtFilename)->required(),
      "Path to a ground truth reconstructed scene");

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

  if(outputFolder.empty())
  {
    ALICEVISION_LOG_ERROR("Invalid output folder");
    return EXIT_FAILURE;
  }

  if (!fs::exists(outputFolder))
    fs::create_directory(outputFolder);

  // load GT camera rotations & positions [R|C]:
  std::mt19937 randomNumberGenerator;
  sfmData::SfMData sfmData_gt;

  if(!sfmDataIO::Load(sfmData_gt, gtFilename, sfmDataIO::ESfMData(sfmDataIO::VIEWS|sfmDataIO::INTRINSICS|sfmDataIO::EXTRINSICS)))
  {
    ALICEVISION_LOG_ERROR("The input SfMData file '"<< gtFilename << "' cannot be read");
    return EXIT_FAILURE;
  }
  ALICEVISION_LOG_INFO(sfmData_gt.getPoses().size() << " gt cameras have been found");

  // load the camera that we have to evaluate
  sfmData::SfMData sfmData;
  if(!sfmDataIO::Load(sfmData, sfmDataFilename, sfmDataIO::ESfMData(sfmDataIO::VIEWS|sfmDataIO::INTRINSICS|sfmDataIO::EXTRINSICS)))
  {
    ALICEVISION_LOG_ERROR("The input SfMData file '"<< sfmDataFilename << "' cannot be read");
    return EXIT_FAILURE;
  }

  // fill vectors of valid views for evaluation
  std::vector<Vec3> vec_camPosGT, vec_C;
  std::vector<Mat3> vec_camRotGT, vec_camRot;
  for(const auto &iter : sfmData.getViews())
  {
    const auto &view = iter.second;
    // jump to next view if there is no correponding pose in reconstruction
    if(sfmData.getPoses().find(view->getPoseId()) == sfmData.getPoses().end())
    {
      ALICEVISION_LOG_INFO("no pose in input for view " << view->getPoseId());
      continue;
    }

    // jump to next view if there is no corresponding view in GT
    if(sfmData_gt.getViews().find(view->getViewId()) == sfmData_gt.getViews().end())
    {
      ALICEVISION_LOG_INFO("no view in GT for viewId " << view->getViewId());
      continue;
    }
    const int idPoseGT = sfmData_gt.getViews().at(view->getViewId())->getPoseId();

    // gt
    const geometry::Pose3& pose_gt = sfmData_gt.getAbsolutePose(idPoseGT).getTransform();
    vec_camPosGT.push_back(pose_gt.center());
    vec_camRotGT.push_back(pose_gt.rotation());

    // data to evaluate
    const geometry::Pose3 pose_eval = sfmData.getPose(*view).getTransform();
    vec_C.push_back(pose_eval.center());
    vec_camRot.push_back(pose_eval.rotation());
  }

  // visual output of the camera location
  plyHelper::exportToPly(vec_camPosGT, (fs::path(outputFolder) / "camGT.ply").string());
  plyHelper::exportToPly(vec_C, (fs::path(outputFolder) / "camComputed.ply").string());

  // evaluation
  htmlDocument::htmlDocumentStream _htmlDocStream("aliceVision Quality evaluation.");
  EvaluteToGT(vec_camPosGT, vec_C, vec_camRotGT, vec_camRot, outputFolder, randomNumberGenerator, &_htmlDocStream);

  std::ofstream htmlFileStream((fs::path(outputFolder) / "ExternalCalib_Report.html").string());
  htmlFileStream << _htmlDocStream.getDoc();

  return EXIT_SUCCESS;
}

