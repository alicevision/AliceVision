// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "aliceVision/sfm/sfm.hpp"
#include "aliceVision/config.hpp"

#include "software/utils/precisionEvaluationToGt.hpp"
#include "software/utils/sfmHelper/sfmPlyHelper.hpp"

#include "dependencies/htmlDoc/htmlDoc.hpp"

#include <boost/program_options.hpp>

#include <cstdlib>
#include <iostream>

using namespace aliceVision;
using namespace aliceVision::sfm;
namespace po = boost::program_options;
using namespace std;

int main(int argc, char **argv)
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

  // set verbose level
  system::Logger::get()->setLogLevel(verboseLevel);

  if (outputFolder.empty())  {
    std::cerr << "\nIt is an invalid output folder" << std::endl;
    return EXIT_FAILURE;
  }

  if (!stlplus::folder_exists(outputFolder))
    stlplus::folder_create(outputFolder);

  //---------------------------------------
  // Quality evaluation
  //---------------------------------------

  //-- Load GT camera rotations & positions [R|C]:
  SfMData sfm_data_gt;
  // READ DATA FROM GT
  std::cout << "Try to read data from GT" << std::endl;
  if (!Load(sfm_data_gt, gtFilename, ESfMData(VIEWS|INTRINSICS|EXTRINSICS)))
  {
    std::cerr << "The input SfMData file \""<< sfmDataFilename << "\" cannot be read." << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << sfm_data_gt.GetPoses().size() << " gt cameras have been found" << std::endl;

  //-- Load the camera that we have to evaluate
  SfMData sfm_data;
  if (!Load(sfm_data, sfmDataFilename, ESfMData(VIEWS|INTRINSICS|EXTRINSICS)))
  {
    std::cerr << std::endl
      << "The input SfMData file \""<< sfmDataFilename << "\" cannot be read." << std::endl;
    return EXIT_FAILURE;
  }

  // Fill vectors of valid views for evaluation
  std::vector<Vec3> vec_camPosGT, vec_C;
  std::vector<Mat3> vec_camRotGT, vec_camRot;
  for(const auto &iter : sfm_data.GetViews())
  {
    const auto &view = iter.second;
    // Jump to next view if there is no correponding pose in reconstruction
    if(sfm_data.GetPoses().find(view->getPoseId()) == sfm_data.GetPoses().end())
    {
      std::cout << "no pose in input (" << view->getPoseId() << ")" << std::endl;
      continue;
    }

    // Jump to next view if there is no corresponding view in GT
    if(sfm_data_gt.GetViews().find(view->getViewId()) == sfm_data_gt.GetViews().end())
    {
      std::cout << "no view in GT (" << view->getViewId() << ")" << std::endl;
      continue;
    }
    const int idPoseGT = sfm_data_gt.GetViews().at(view->getViewId())->getPoseId();

    //-- GT
    const geometry::Pose3 pose_gt = sfm_data_gt.GetPoses().at(idPoseGT);
    vec_camPosGT.push_back(pose_gt.center());
    vec_camRotGT.push_back(pose_gt.rotation());

    //-- Data to evaluate
    const geometry::Pose3 pose_eval = sfm_data.GetPoses().at(view->getPoseId());
    vec_C.push_back(pose_eval.center());
    vec_camRot.push_back(pose_eval.rotation());
  }

  // Visual output of the camera location
  plyHelper::exportToPly(vec_camPosGT, string(stlplus::folder_append_separator(outputFolder) + "camGT.ply").c_str());
  plyHelper::exportToPly(vec_C, string(stlplus::folder_append_separator(outputFolder) + "camComputed.ply").c_str());

  // Evaluation
  htmlDocument::htmlDocumentStream _htmlDocStream("aliceVision Quality evaluation.");
  EvaluteToGT(vec_camPosGT, vec_C, vec_camRotGT, vec_camRot, outputFolder, &_htmlDocStream);

  ofstream htmlFileStream( string(stlplus::folder_append_separator(outputFolder) +
    "ExternalCalib_Report.html"));
  htmlFileStream << _htmlDocStream.getDoc();

  return EXIT_SUCCESS;
}

