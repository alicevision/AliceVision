// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#include "aliceVision/sfm/sfm.hpp"
#include "aliceVision/image/image.hpp"

using namespace aliceVision;
using namespace aliceVision::cameras;
using namespace aliceVision::geometry;
using namespace aliceVision::image;
using namespace aliceVision::sfm;

#include "third_party/cmdLine/cmdLine.h"
#include "third_party/progress/progress.hpp"

#include <stdlib.h>

int main(int argc, char *argv[]) {

  CmdLine cmd;
  std::string sSfM_Data_Filename;
  std::string sOutDir = "";

  cmd.add( make_option('i', sSfM_Data_Filename, "sfmdata") );
  cmd.add( make_option('o', sOutDir, "outdir") );

  try {
      if (argc == 1) throw std::string("Invalid command line parameter.");
      cmd.process(argc, argv);
  } catch(const std::string& s) {
    std::cerr
      << "Export undistorted images related to a sfm_data file.\n"
      << "Usage: " << argv[0] << '\n'
      << "[-i|--sfmdata] filename, the SfM_Data file to convert\n"
      << "[-o|--outdir] path\n"
      << std::endl;

      std::cerr << s << std::endl;
      return EXIT_FAILURE;
  }

  // Create output dir
  if (!stlplus::folder_exists(sOutDir))
    stlplus::folder_create( sOutDir );

  SfM_Data sfm_data;
  if (!Load(sfm_data, sSfM_Data_Filename, ESfM_Data(VIEWS|INTRINSICS))) {
    std::cerr << std::endl
      << "The input SfM_Data file \""<< sSfM_Data_Filename << "\" cannot be read." << std::endl;
    return EXIT_FAILURE;
  }

  bool bOk = true;
  {
    // Export views as undistorted images (those with valid Intrinsics)
    Image<RGBColor> image, image_ud;
    C_Progress_display my_progress_bar( sfm_data.GetViews().size() );
    for(Views::const_iterator iter = sfm_data.GetViews().begin();
      iter != sfm_data.GetViews().end(); ++iter, ++my_progress_bar)
    {
      const View * view = iter->second.get();
      bool bIntrinsicDefined = view->getIntrinsicId() != UndefinedIndexT &&
        sfm_data.GetIntrinsics().find(view->getIntrinsicId()) != sfm_data.GetIntrinsics().end();

      Intrinsics::const_iterator iterIntrinsic = sfm_data.GetIntrinsics().find(view->getIntrinsicId());

      const std::string srcImage = stlplus::create_filespec(sfm_data.s_root_path, view->getImagePath());
      const std::string dstImage = stlplus::create_filespec(
        sOutDir, stlplus::filename_part(srcImage));

      const IntrinsicBase * cam = iterIntrinsic->second.get();
      if (cam->isValid() && cam->have_disto())
      {
        // undistort the image and save it
        if (ReadImage( srcImage.c_str(), &image))
        {
          UndistortImage(image, cam, image_ud, BLACK);
          bOk &= WriteImage(dstImage.c_str(), image_ud);
        }
      }
      else // (no distortion)
      {
        // copy the image since there is no distortion
        stlplus::file_copy(srcImage, dstImage);
      }
    }
  }

  // Exit program
  if (bOk)
    return( EXIT_SUCCESS );
  else
    return( EXIT_FAILURE );
}
