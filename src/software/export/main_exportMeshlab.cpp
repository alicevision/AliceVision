// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/image/all.hpp>
#include <aliceVision/system/main.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <fstream>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;
using namespace aliceVision::camera;
using namespace aliceVision::geometry;
using namespace aliceVision::image;
using namespace aliceVision::sfmData;

namespace po = boost::program_options;
namespace fs = boost::filesystem;

int aliceVision_main(int argc, char **argv)
{
  // command-line parameters

  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  std::string sfmDataFilename;
  std::string plyPath;
  std::string outDirectory;

  po::options_description allParams("AliceVision exportMeshlab");

  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
      "SfMData file.")
    ("ply", po::value<std::string>(&plyPath)->required(),
      "Ply.")
    ("output,o", po::value<std::string>(&outDirectory)->required(),
      "Output folder.");

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

  // Create output dir
  if(!fs::exists(outDirectory))
    fs::create_directory(outDirectory);

  // Read the SfM scene
  SfMData sfm_data;
  if(!sfmDataIO::Load(sfm_data, sfmDataFilename, sfmDataIO::ESfMData(sfmDataIO::VIEWS|sfmDataIO::INTRINSICS|sfmDataIO::EXTRINSICS)))
  {
    std::cerr << std::endl
      << "The input SfMData file \""<< sfmDataFilename << "\" cannot be read." << std::endl;
    return EXIT_FAILURE;
  }

  std::ofstream outfile((fs::path(outDirectory) / "sceneMeshlab.mlp").string());

  // Init mlp file
  outfile << "<!DOCTYPE MeshLabDocument>" << outfile.widen('\n')
    << "<MeshLabProject>" << outfile.widen('\n')
    << " <MeshGroup>" << outfile.widen('\n')
    << "  <MLMesh label=\"" << plyPath << "\" filename=\"" << plyPath << "\">" << outfile.widen('\n')
    << "   <MLMatrix44>" << outfile.widen('\n')
    << "1 0 0 0 " << outfile.widen('\n')
    << "0 1 0 0 " << outfile.widen('\n')
    << "0 0 1 0 " << outfile.widen('\n')
    << "0 0 0 1 " << outfile.widen('\n')
    << "</MLMatrix44>" << outfile.widen('\n')
    << "  </MLMesh>" << outfile.widen('\n')
    << " </MeshGroup>" << outfile.widen('\n');

  outfile <<  " <RasterGroup>" << outfile.widen('\n');

  for(Views::const_iterator iter = sfm_data.getViews().begin();
      iter != sfm_data.getViews().end(); ++iter)
  {
    const View * view = iter->second.get();
    if (!sfm_data.isPoseAndIntrinsicDefined(view))
      continue;

    const Pose3 pose = sfm_data.getPose(*view).getTransform();
    Intrinsics::const_iterator iterIntrinsic = sfm_data.getIntrinsics().find(view->getIntrinsicId());

    // We have a valid view with a corresponding camera & pose
    const std::string srcImage = view->getImagePath();
    std::shared_ptr<camera::IntrinsicBase> cam = iterIntrinsic->second;
    std::shared_ptr<camera::Pinhole> camPinHole = std::dynamic_pointer_cast<camera::Pinhole>(cam);
    if (!camPinHole) {
      ALICEVISION_LOG_ERROR("Camera is not pinhole in filter");
      continue;
    }
    
    Mat34 P = camPinHole->getProjectiveEquivalent(pose);

    for ( int i = 1; i < 3 ; ++i)
      for ( int j = 0; j < 4; ++j)
        P(i, j) *= -1.;

    Mat3 R, K;
    Vec3 t;
    KRt_from_P( P, &K, &R, &t);

    const Vec3 optical_center = R.transpose() * t;

    outfile
      << "  <MLRaster label=\"" << fs::path(view->getImagePath()).filename().string() << "\">" << std::endl
      << "   <VCGCamera TranslationVector=\""
      << optical_center[0] << " "
      << optical_center[1] << " "
      << optical_center[2] << " "
      << " 1 \""
      << " LensDistortion=\"0 0\""
      << " ViewportPx=\"" << cam->w() << " " << cam->h() << "\""
      << " PixelSizeMm=\"" << 1  << " " << 1 << "\""
      << " CenterPx=\"" << cam->w() / 2.0 << " " << cam->h() / 2.0 << "\""
      << " FocalMm=\"" << (double)K(0, 0 )  << "\""
      << " RotationMatrix=\""
      << R(0, 0) << " " << R(0, 1) << " " << R(0, 2) << " 0 "
      << R(1, 0) << " " << R(1, 1) << " " << R(1, 2) << " 0 "
      << R(2, 0) << " " << R(2, 1) << " " << R(2, 2) << " 0 "
      << "0 0 0 1 \"/>"  << std::endl;

    // Link the image plane
    outfile << "   <Plane semantic=\"\" fileName=\"" << srcImage << "\"/> "<< std::endl;
    outfile << "  </MLRaster>" << std::endl;
  }
  outfile << "   </RasterGroup>" << std::endl
    << "</MeshLabProject>" << std::endl;

  outfile.close();

  return EXIT_SUCCESS;
}
