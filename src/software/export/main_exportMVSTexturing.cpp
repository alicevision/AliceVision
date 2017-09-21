// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#include "aliceVision/sfm/sfm.hpp"
#include "aliceVision/image/image.hpp"

#include "dependencies/cmdLine/cmdLine.h"
#include "dependencies/stlplus3/filesystemSimplified/file_system.hpp"

#include <fstream>

using namespace aliceVision;
using namespace aliceVision::camera;
using namespace aliceVision::geometry;
using namespace aliceVision::sfm;

int main(int argc, char **argv)
{
  CmdLine cmd;

  std::string sSfMData_Filename;
  std::string sOutDir = "";

  cmd.add( make_option('i', sSfMData_Filename, "sfmdata") );
  cmd.add( make_option('o', sOutDir, "outdir") );

  try {
      if (argc == 1) throw std::string("Invalid command line parameter.");
      cmd.process(argc, argv);
  } catch(const std::string& s) {
      std::cerr << "Usage: " << argv[0] << '\n'
      << "[-i|--sfmdata] filename, the SfMData file to convert\n"
      << "[-o|--outdir] path.\n"
      << std::endl;

      std::cerr << s << std::endl;
      return EXIT_FAILURE;
  }

  std::cout << " You called : " <<std::endl
            << argv[0] << std::endl
            << "--sfmdata " << sSfMData_Filename << std::endl
            << "--outdir " << sOutDir << std::endl;

  bool bOneHaveDisto = false;
  
  // Create output dir
  if (!stlplus::folder_exists(sOutDir))
    stlplus::folder_create( sOutDir );

  // Read the SfM scene
  SfMData sfm_data;
  if (!Load(sfm_data, sSfMData_Filename, ESfMData(VIEWS|INTRINSICS|EXTRINSICS))) {
    std::cerr << std::endl
      << "The input SfMData file \""<< sSfMData_Filename << "\" cannot be read." << std::endl;
    return EXIT_FAILURE;
  }

  for(Views::const_iterator iter = sfm_data.GetViews().begin();
      iter != sfm_data.GetViews().end(); ++iter)
  {
    const View * view = iter->second.get();
    if (!sfm_data.IsPoseAndIntrinsicDefined(view))
        continue;
    
    // Valid view, we can ask a pose & intrinsic data
    const Pose3 pose = sfm_data.getPose(*view);
    Intrinsics::const_iterator iterIntrinsic = sfm_data.GetIntrinsics().find(view->getIntrinsicId());
    const IntrinsicBase * cam = iterIntrinsic->second.get();
    
    if (!camera::isPinhole(cam->getType()))
        continue;
    const Pinhole * pinhole_cam = static_cast<const Pinhole *>(cam);
    
    // Extrinsic
    const Vec3 t = pose.translation();
    const Mat3 R = pose.rotation();
    // Intrinsic
    const double f = pinhole_cam->focal();
    const Vec2 pp = pinhole_cam->principal_point();

    // Image size in px
    const int w = pinhole_cam->w();
    const int h = pinhole_cam->h();
    
    // We can now create the .cam file for the View in the output dir 
    std::ofstream outfile( stlplus::create_filespec(
                sOutDir, stlplus::basename_part(view->getImagePath()), "cam" ).c_str() );
    // See https://github.com/nmoehrle/mvs-texturing/blob/master/Arguments.cpp
    // for full specs
    const int largerDim = w > h ? w : h;
    outfile << t(0) << " " << t(1) << " " << t(2) << " "
        << R(0,0) << " " << R(0,1) << " " << R(0,2) << " "
        << R(1,0) << " " << R(1,1) << " " << R(1,2) << " "
        << R(2,0) << " " << R(2,1) << " " << R(2,2) << "\n"
        << f / largerDim << " 0 0 1 " << pp(0) / w << " " << pp(1) / h;
    outfile.close();
    
    if(cam->have_disto())
      bOneHaveDisto = true;
  }
  
  const std::string sUndistMsg = bOneHaveDisto ? "undistorded" : "";
  const std::string sQuitMsg = std::string("Your SfMData file was succesfully converted!\n") +
	  "Now you can copy your " + sUndistMsg + " images in the \"" + sOutDir + "\" directory and run MVS Texturing";
  std::cout << sQuitMsg << std::endl;
  return EXIT_SUCCESS;
}
