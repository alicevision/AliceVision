// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#include "aliceVision/sfm/sfm.hpp"
#include "aliceVision/config.hpp"

#include "software/utils/precisionEvaluationToGt.hpp"
#include "software/utils/sfmHelper/sfmPlyHelper.hpp"

#include "dependencies/htmlDoc/htmlDoc.hpp"
#include "dependencies/cmdLine/cmdLine.h"

#include <cstdlib>
#include <iostream>

using namespace aliceVision;
using namespace aliceVision::sfm;

int main(int argc, char **argv)
{
  using namespace std;

  CmdLine cmd;

  std::string
    sGTFile,
    sComputedFile,
    sOutDirectory = "";


  cmd.add( make_option('i', sGTFile, "gt") );
  cmd.add( make_option('c', sComputedFile, "computed") );
  cmd.add( make_option('o', sOutDirectory, "outdir") );

  try {
    if (argc == 1) throw std::string("Invalid command line parameter.");
    cmd.process(argc, argv);
  } catch(const std::string& s) {
    std::cerr << "Usage: " << argv[0] << '\n'
      << "[-i|--gt] ground truth path: it could be a json/bin"
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_ALEMBIC)
      << "/abc"
#endif
      << " file containing\n"
      << " a reconstructed scene or localized camera, or the path to a directory\n"
      << " containing the 2 other directories, \"images\" containing the images of\n"
      << " the scene, and \"gt_dense_cameras\" containing the corresponding .camera files\n"
      << "[-c|--computed] path (aliceVision sfm_data.json file)\n"
      << "[-o|--outdir] path (where statistics will be saved)\n"
      << std::endl;

    std::cerr << s << std::endl;
    return EXIT_FAILURE;
  }

  if (sOutDirectory.empty())  {
    std::cerr << "\nIt is an invalid output directory" << std::endl;
    return EXIT_FAILURE;
  }

  if (!stlplus::folder_exists(sOutDirectory))
    stlplus::folder_create(sOutDirectory);

  //---------------------------------------
  // Quality evaluation
  //---------------------------------------

  //-- Load GT camera rotations & positions [R|C]:
  SfMData sfm_data_gt;
  // READ DATA FROM GT
  std::cout << "Try to read data from GT" << std::endl;
  if (!Load(sfm_data_gt, sGTFile, ESfMData(VIEWS|INTRINSICS|EXTRINSICS)))
  {
    std::cerr << "The input SfMData file \""<< sComputedFile << "\" cannot be read." << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << sfm_data_gt.GetPoses().size() << " gt cameras have been found" << std::endl;

  //-- Load the camera that we have to evaluate
  SfMData sfm_data;
  if (!Load(sfm_data, sComputedFile, ESfMData(VIEWS|INTRINSICS|EXTRINSICS)))
  {
    std::cerr << std::endl
      << "The input SfMData file \""<< sComputedFile << "\" cannot be read." << std::endl;
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
  plyHelper::exportToPly(vec_camPosGT, string(stlplus::folder_append_separator(sOutDirectory) + "camGT.ply").c_str());
  plyHelper::exportToPly(vec_C, string(stlplus::folder_append_separator(sOutDirectory) + "camComputed.ply").c_str());

  // Evaluation
  htmlDocument::htmlDocumentStream _htmlDocStream("aliceVision Quality evaluation.");
  EvaluteToGT(vec_camPosGT, vec_C, vec_camRotGT, vec_camRot, sOutDirectory, &_htmlDocStream);

  ofstream htmlFileStream( string(stlplus::folder_append_separator(sOutDirectory) +
    "ExternalCalib_Report.html"));
  htmlFileStream << _htmlDocStream.getDoc();

  return EXIT_SUCCESS;
}

