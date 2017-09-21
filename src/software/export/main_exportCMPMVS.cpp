// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#include "aliceVision/sfm/sfm.hpp"
#include "aliceVision/image/image.hpp"

using namespace aliceVision;
using namespace aliceVision::camera;
using namespace aliceVision::geometry;
using namespace aliceVision::image;
using namespace aliceVision::sfm;

#include "dependencies/cmdLine/cmdLine.h"
#include "dependencies/progress/progress.hpp"

#include <stdlib.h>
#include <stdio.h>
#include <cmath>
#include <iterator>
#include <iomanip>

std::string replaceAll( std::string const& original, std::string const& from, std::string const& to )
{
    std::string results;
    std::string::const_iterator end = original.end();
    std::string::const_iterator current = original.begin();
    std::string::const_iterator next = std::search( current, end, from.begin(), from.end() );
    while ( next != end ) {
        results.append( current, next );
        results.append( to );
        current = next + from.size();
        next = std::search( current, end, from.begin(), from.end() );
    }
    results.append( current, next );
    return results;
}

bool exportToCMPMVSFormat(
  const SfMData & sfm_data,
  const std::string & sOutDirectory // Output CMPMVS files directory
  )
{
  bool bOk = true;
  // Create basis directory structure
  if (!stlplus::is_folder(sOutDirectory))
  {
    stlplus::folder_create(sOutDirectory);
    bOk = stlplus::is_folder(sOutDirectory);
  }

  if (!bOk)
  {
    std::cerr << "Cannot access to one of the desired output directory" << std::endl;
    return false;
  }
  else
  {
    // Since CMPMVS requires contiguous camera index, and that some views can have some missing poses,
    // we reindex the poses to ensure a contiguous pose list.
    Hash_Map<IndexT, IndexT> map_viewIdToContiguous;

    // CMPMVS only support images of the same resolution,
    // so select the most used resolution and only export those images.
    std::pair<size_t, size_t> mostCommonResolution;
    {
      std::map<std::pair<size_t, size_t>, size_t> imgResolutions;
      std::size_t nbValidImages = 0;
      for(const auto &iter : sfm_data.GetViews())
      {
        const View * view = iter.second.get();
        if (!sfm_data.IsPoseAndIntrinsicDefined(view))
          continue;
        Intrinsics::const_iterator iterIntrinsic = sfm_data.GetIntrinsics().find(view->getIntrinsicId());
        const IntrinsicBase * cam = iterIntrinsic->second.get();
        if(!cam->isValid())
          continue;
        ++nbValidImages;
        std::pair<size_t, size_t> imgResolution = std::make_pair(cam->w(), cam->h());
        if(imgResolutions.find(imgResolution) == imgResolutions.end())
        {
          imgResolutions[imgResolution] = 1;
        }
        else
        {
          ++imgResolutions[imgResolution];
        }
      }
      std::size_t s = 0;
      for(auto& r: imgResolutions)
      {
        if(r.second > s)
        {
          mostCommonResolution = r.first;
          s = r.second;
        }
      }
      if(imgResolutions.size() > 1)
      {
        std::cerr << "CMPMVS only supports images of the same size, so we export the most common resolution: " << mostCommonResolution.first << "x" << mostCommonResolution.second << std::endl;
        std::cerr << "We will only export " << s << " cameras from a dataset of " << nbValidImages << " cameras." << std::endl;
      }
    }
    // Export valid views as Projective Cameras:
    for(const auto &iter : sfm_data.GetViews())
    {
      const View * view = iter.second.get();
      if (!sfm_data.IsPoseAndIntrinsicDefined(view))
        continue;
      Intrinsics::const_iterator iterIntrinsic = sfm_data.GetIntrinsics().find(view->getIntrinsicId());
      const IntrinsicBase * cam = iterIntrinsic->second.get();
      if(cam->w() != mostCommonResolution.first || cam->h() != mostCommonResolution.second)
        continue;
      // View Id re-indexing
      // Need to start at 1 for CMPMVS
      map_viewIdToContiguous.insert(std::make_pair(view->getViewId(), map_viewIdToContiguous.size() + 1));
    }

    // Export data
    C_Progress_display my_progress_bar(map_viewIdToContiguous.size());

    // Export (calibrated) views as undistorted images
    for(int i = 0; i < map_viewIdToContiguous.size(); ++i)
    {
      auto viewIdToContiguous = map_viewIdToContiguous.cbegin();
      std::advance(viewIdToContiguous, i);
      IndexT viewId = viewIdToContiguous->first;
      const View * view = sfm_data.GetViews().at(viewId).get();
      Intrinsics::const_iterator iterIntrinsic = sfm_data.GetIntrinsics().find(view->getIntrinsicId());
      // We have a valid view with a corresponding camera & pose
      
      // Export camera pose
      {
        const Pose3 pose = sfm_data.getPose(*view);
        const Mat34 P = iterIntrinsic->second.get()->get_projective_equivalent(pose);
        std::ostringstream os;
        os << std::setw(5) << std::setfill('0') << map_viewIdToContiguous[view->getViewId()] << "_P";
        std::ofstream file(
          stlplus::create_filespec(stlplus::folder_append_separator(sOutDirectory),
          os.str() ,"txt").c_str());
        file << "CONTOUR" << "\n"
          << P(0, 0) << " " << P(0, 1) << " "  << P(0, 2) << " "  << P(0, 3) << "\n"
          << P(1, 0) << " " << P(1, 1) << " "  << P(1, 2) << " "  << P(1, 3) << "\n"
          << P(2, 0) << " " << P(2, 1) << " "  << P(2, 2) << " "  << P(2, 3) << "\n";
        file.close();
      }
      // Export undistort image
      {
        const std::string srcImage = stlplus::create_filespec(sfm_data.s_root_path, view->getImagePath());
        std::ostringstream os;
        os << std::setw(5) << std::setfill('0') << map_viewIdToContiguous[view->getViewId()];

        std::string dstImage = stlplus::create_filespec(
          stlplus::folder_append_separator(sOutDirectory), os.str(),"jpg");

        const IntrinsicBase * cam = iterIntrinsic->second.get();
        Image<RGBColor> image, image_ud;
        if (cam->isValid() && cam->have_disto())
        {
          // undistort the image and save it
          ReadImage( srcImage.c_str(), &image);
          UndistortImage(image, cam, image_ud, BLACK);
          WriteImage(dstImage.c_str(), image_ud);
        }
        else // (no distortion)
        {
          // copy the image if extension match
          if (stlplus::extension_part(srcImage) == "JPG" ||
            stlplus::extension_part(srcImage) == "jpg")
          {
            stlplus::file_copy(srcImage, dstImage);
          }
          else
          {
            ReadImage( srcImage.c_str(), &image);
            WriteImage( dstImage.c_str(), image);
          }
        }
      }
      ++my_progress_bar;
    }
    std::string dirName = stlplus::folder_append_separator(sOutDirectory);
    std::cout << "Linux path is: " << dirName << std::endl;
    dirName = replaceAll(dirName, "/s/prods/", "V:\\");
    dirName = replaceAll(dirName, "/s/v/", "V:\\");
    dirName = replaceAll(dirName, "/", "\\");
    std::cout << "Windows path is: " << dirName << std::endl;

    // Write the cmpmvs ini file
    std::ostringstream os;
    os << "[global]" << os.widen('\n')
    << "dirName=\"" << dirName << "\"" << os.widen('\n')
    << "prefix=\"\"" << os.widen('\n')
    << "imgExt=\"jpg\"" << os.widen('\n')
    << "ncams=" << map_viewIdToContiguous.size() << os.widen('\n')
    << "width=" << mostCommonResolution.first << os.widen('\n')
    << "height=" << mostCommonResolution.second << os.widen('\n')
    << "scale=2" << os.widen('\n')
    << "workDirName=\"_tmp_scale2\"" << os.widen('\n')
    << "doPrepareData=TRUE" << os.widen('\n')
    << "doPrematchSifts=TRUE" << os.widen('\n')
    << "doPlaneSweepingSGM=TRUE"  << os.widen('\n')
    << "doFuse=TRUE" << os.widen('\n')
    << "nTimesSimplify=10" << os.widen('\n')
    << os.widen('\n')

    << "[uvatlas]" << os.widen('\n')
    << "texSide=8192" << os.widen('\n')
    << "scale=2" << os.widen('\n')
    << os.widen('\n')
    
    << "[hallucinationsFiltering]" << os.widen('\n')
    << "useSkyPrior=FALSE" << os.widen('\n')
    << "doLeaveLargestFullSegmentOnly=FALSE" << os.widen('\n')
    << "doRemoveHugeTriangles=FALSE" << os.widen('\n')
    << os.widen('\n')

    << "[delanuaycut]" << os.widen('\n')
    << "saveMeshTextured=TRUE" << os.widen('\n')
    << os.widen('\n')

    << "[largeScale]" << os.widen('\n')
    << "workDirName=\"largeScaleMaxPts01024_scale2\"" << os.widen('\n')
    << "doReconstructSpaceAccordingToVoxelsArray=TRUE" << os.widen('\n')
    << "doGenerateAndReconstructSpaceMaxPts=FALSE" << os.widen('\n')
    << "doGenerateSpace=TRUE" << os.widen('\n')
    << "planMaxPts=3000000" << os.widen('\n')
    << "planMaxPtsPerVoxel=3000000" << os.widen('\n')
    // << "doComputeDEMandOrtoPhoto=FALSE" << os.widen('\n')
    // << "doGenerateVideoFrames=FALSE" << os.widen('\n')
    << "nGridHelperVolumePointsDim=10" << os.widen('\n')
    << "joinMeshesSaveTextured=TRUE" << os.widen('\n')
    << os.widen('\n')
    << "[meshEnergyOpt]" << os.widen('\n')
    << "doOptimizeOrSmoothMesh=FALSE" << os.widen('\n')
    << os.widen('\n')
    << "[semiGlobalMatching]" << os.widen('\n')
    << "wsh=4" << os.widen('\n')
    << os.widen('\n')
    << "[refineRc]" << os.widen('\n')
    << "wsh=4" << os.widen('\n')
    << os.widen('\n')
    << "#EOF" << os.widen('\n')
    << os.widen('\n')
    << os.widen('\n');

    std::ofstream file2(
	    stlplus::create_filespec(stlplus::folder_append_separator(sOutDirectory),
	    "cmpmvs_scale2" ,"ini").c_str());
    file2 << os.str();
    file2.close();
  }
  return bOk;
}

int main(int argc, char *argv[])
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
      << "[-o|--outdir] path\n"
      << std::endl;

      std::cerr << s << std::endl;
      return EXIT_FAILURE;
  }

  sOutDir = stlplus::folder_to_path(sOutDir);

  // Create output dir
  if (!stlplus::folder_exists(sOutDir))
    stlplus::folder_create( sOutDir );

  // Read the input SfM scene
  SfMData sfm_data;
  if (!Load(sfm_data, sSfMData_Filename, ESfMData(ALL)))
  {
    std::cerr << std::endl
      << "The input SfMData file \""<< sSfMData_Filename << "\" cannot be read." << std::endl;
    return EXIT_FAILURE;
  }

  if (!exportToCMPMVSFormat(sfm_data, stlplus::filespec_to_path(sOutDir, "CMPMVS")))
    return EXIT_FAILURE;

  return EXIT_SUCCESS;
}
