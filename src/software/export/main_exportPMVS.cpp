// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "aliceVision/sfm/sfm.hpp"
#include "aliceVision/image/image.hpp"

#include <boost/program_options.hpp>
#include <boost/progress.hpp>

#include <stdlib.h>
#include <stdio.h>
#include <cmath>
#include <iterator>
#include <iomanip>

using namespace aliceVision;
using namespace aliceVision::camera;
using namespace aliceVision::geometry;
using namespace aliceVision::image;
using namespace aliceVision::sfm;
namespace po = boost::program_options;

bool exportToPMVSFormat(
  const SfMData & sfm_data,
  const std::string & sOutDirectory,  //Output PMVS files folder
  const int downsampling_factor,
  const int CPU_core_count,
  const bool b_VisData = true
  )
{
  bool bOk = true;
  if (!stlplus::is_folder(sOutDirectory))
  {
    stlplus::folder_create(sOutDirectory);
    bOk = stlplus::is_folder(sOutDirectory);
  }

  // Create basis folder structure
  stlplus::folder_create( stlplus::folder_append_separator(sOutDirectory) + "models");
  stlplus::folder_create( stlplus::folder_append_separator(sOutDirectory) + "txt");
  stlplus::folder_create( stlplus::folder_append_separator(sOutDirectory) + "visualize");

  if (bOk &&
      stlplus::is_folder(stlplus::folder_append_separator(sOutDirectory) + "models") &&
      stlplus::is_folder( stlplus::folder_append_separator(sOutDirectory) + "txt") &&
      stlplus::is_folder( stlplus::folder_append_separator(sOutDirectory) + "visualize")
      )
  {
    bOk = true;
  }
  else  {
    std::cerr << "Cannot access to one of the desired output folder" << std::endl;
  }

  if (bOk)
  {
    boost::progress_display my_progress_bar( sfm_data.GetViews().size()*2 );

    // Since PMVS requires contiguous camera index, and that some views can have some missing poses,
    // we reindex the poses to ensure a contiguous pose list.
    HashMap<IndexT, IndexT> map_viewIdToContiguous;

    // Export valid views as Projective Cameras:
    for(Views::const_iterator iter = sfm_data.GetViews().begin();
      iter != sfm_data.GetViews().end(); ++iter, ++my_progress_bar)
    {
      const View * view = iter->second.get();
      if (!sfm_data.IsPoseAndIntrinsicDefined(view))
        continue;

      const Pose3 pose = sfm_data.getPose(*view);
      Intrinsics::const_iterator iterIntrinsic = sfm_data.GetIntrinsics().find(view->getIntrinsicId());

      // View Id re-indexing
      map_viewIdToContiguous.insert(std::make_pair(view->getViewId(), map_viewIdToContiguous.size()));

      // We have a valid view with a corresponding camera & pose
      const Mat34 P = iterIntrinsic->second.get()->get_projective_equivalent(pose);
      std::ostringstream os;
      os << std::setw(8) << std::setfill('0') << map_viewIdToContiguous[view->getViewId()];
      std::ofstream file(
        stlplus::create_filespec(stlplus::folder_append_separator(sOutDirectory) + "txt",
        os.str() ,"txt").c_str());
      file << "CONTOUR" << os.widen('\n')
        << P.row(0) <<"\n"<< P.row(1) <<"\n"<< P.row(2) << os.widen('\n');
      file.close();
    }

    // Export (calibrated) views as undistorted images
    Image<RGBColor> image, image_ud;
    for(Views::const_iterator iter = sfm_data.GetViews().begin();
      iter != sfm_data.GetViews().end(); ++iter, ++my_progress_bar)
    {
      const View * view = iter->second.get();
      if (!sfm_data.IsPoseAndIntrinsicDefined(view))
        continue;

      Intrinsics::const_iterator iterIntrinsic = sfm_data.GetIntrinsics().find(view->getIntrinsicId());

      // We have a valid view with a corresponding camera & pose
      const std::string srcImage = view->getImagePath();
      std::ostringstream os;
      os << std::setw(8) << std::setfill('0') << map_viewIdToContiguous[view->getViewId()];
      const std::string dstImage = stlplus::create_filespec(
        stlplus::folder_append_separator(sOutDirectory) + "visualize", os.str(),"jpg");

      const IntrinsicBase * cam = iterIntrinsic->second.get();
      if (cam->isValid() && cam->have_disto())
      {
        // undistort the image and save it
        readImage( srcImage, image);
        UndistortImage(image, cam, image_ud, BLACK);
        writeImage(dstImage, image_ud);
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
          readImage(srcImage, image);
          writeImage(dstImage, image);
        }
      }
    }

    //pmvs_options.txt
    std::ostringstream os;
    os << "level " << downsampling_factor << os.widen('\n')
     << "csize 2" << os.widen('\n')
     << "threshold 0.7" << os.widen('\n')
     << "wsize 7" << os.widen('\n')
     << "minImageNum 3" << os.widen('\n')
     << "CPU " << CPU_core_count << os.widen('\n')
     << "setEdge 0" << os.widen('\n')
     << "useBound 0" << os.widen('\n')
     << "useVisData " << (int) b_VisData << os.widen('\n')
     << "sequence -1" << os.widen('\n')
     << "maxAngle 10" << os.widen('\n')
     << "quad 2.0" << os.widen('\n')
     << "timages -1 0 " << map_viewIdToContiguous.size() << os.widen('\n')
     << "oimages 0" << os.widen('\n');

    if (b_VisData)
    {
      std::map< IndexT, std::set<IndexT> > view_shared;
      // From the structure observations, list the putatives pairs (symmetric)
      for (Landmarks::const_iterator itL = sfm_data.GetLandmarks().begin();
        itL != sfm_data.GetLandmarks().end(); ++itL)
      {
        const Landmark & landmark = itL->second;
        const Observations & observations = landmark.observations;
        for (Observations::const_iterator itOb = observations.begin();
          itOb != observations.end(); ++itOb)
        {
          const IndexT viewId = itOb->first;
          Observations::const_iterator itOb2 = itOb;
          ++itOb2;
          for (; itOb2 != observations.end(); ++itOb2)
          {
            const IndexT viewId2 = itOb2->first;
            view_shared[map_viewIdToContiguous[viewId]].insert(map_viewIdToContiguous[viewId2]);
            view_shared[map_viewIdToContiguous[viewId2]].insert(map_viewIdToContiguous[viewId]);
          }
        }
      }
      // Export the vis.dat file
      std::ostringstream osVisData;
      osVisData
        << "VISDATA" << os.widen('\n')
        << view_shared.size() << os.widen('\n'); // #images
      // Export view shared visibility
      for (std::map< IndexT, std::set<IndexT> >::const_iterator it = view_shared.begin();
        it != view_shared.end(); ++it)
      {
        const std::set<IndexT> & setView = it->second;
        osVisData << it->first << ' ' << setView.size();
        for (std::set<IndexT>::const_iterator itV = setView.begin();
          itV != setView.end(); ++itV)
        {
          osVisData << ' ' << *itV;
        }
        osVisData << os.widen('\n');
      }
      std::ofstream file(stlplus::create_filespec(sOutDirectory, "vis", "dat").c_str());
      file << osVisData.str();
      file.close();
    }

    std::ofstream file(stlplus::create_filespec(sOutDirectory, "pmvs_options", "txt").c_str());
    file << os.str();
    file.close();
  }
  return bOk;
}

bool exportToBundlerFormat(
  const SfMData & sfm_data,
  const std::string & sOutFile, //Output Bundle.rd.out file
  const std::string & sOutListFile)  //Output Bundler list.txt file
{
  std::ofstream os(sOutFile.c_str()	);
  std::ofstream osList(sOutListFile.c_str()	);
  if (! os.is_open() || ! osList.is_open())
  {
    return false;
  }
  else
  {
    // Since PMVS requires contiguous camera index, and that some views can have some missing poses,
    // we reindex the poses to ensure a contiguous pose list.
    HashMap<IndexT, IndexT> map_viewIdToContiguous;

    // Count the number of valid cameras and re-index the viewIds
    for(Views::const_iterator iter = sfm_data.GetViews().begin();
      iter != sfm_data.GetViews().end(); ++iter)
    {
      const View * view = iter->second.get();
      if (!sfm_data.IsPoseAndIntrinsicDefined(view))
        continue;

      // View Id re-indexing
      map_viewIdToContiguous.insert(std::make_pair(view->getViewId(), map_viewIdToContiguous.size()));
    }

    // Fill the "Bundle file"
    os << "# Bundle file v0.3" << os.widen('\n')
      << map_viewIdToContiguous.size()  << " " << sfm_data.GetLandmarks().size() << os.widen('\n');

    // Export camera properties & image filenames
    for(Views::const_iterator iter = sfm_data.GetViews().begin();
      iter != sfm_data.GetViews().end(); ++iter)
    {
      const View * view = iter->second.get();
      if (!sfm_data.IsPoseAndIntrinsicDefined(view))
        continue;

      const Pose3 pose = sfm_data.getPose(*view);
      Intrinsics::const_iterator iterIntrinsic = sfm_data.GetIntrinsics().find(view->getIntrinsicId());

      // Must export focal, k1, k2, R, t

      Mat3 D;
      D.fill(0.0);
      D .diagonal() = Vec3(1., -1., -1.); // mapping between our pinhole and Bundler convention
      const double k1 = 0.0, k2 = 0.0; // distortion already removed

      if(isPinhole(iterIntrinsic->second.get()->getType()))
      {
        const Pinhole * cam = dynamic_cast<const Pinhole*>(iterIntrinsic->second.get());
        const double focal = cam->focal();
        const Mat3 R = D * pose.rotation();
        const Vec3 t = D * pose.translation();

        os << focal << " " << k1 << " " << k2 << os.widen('\n') //f k1 k2
          << R(0,0) << " " << R(0, 1) << " " << R(0, 2) << os.widen('\n')  //R.row(0)
          << R(1,0) << " " << R(1, 1) << " " << R(1, 2) << os.widen('\n')  //R.row(1)
          << R(2,0) << " " << R(2, 1) << " " << R(2, 2) << os.widen('\n')  //R.row(2)
          << t(0)   << " " << t(1)    << " " << t(2)    << os.widen('\n'); //t

        osList << stlplus::basename_part(view->getImagePath()) + "." + stlplus::extension_part(view->getImagePath())
          << " 0 " << focal << os.widen('\n');
      }
      else
      {
        std::cerr << "Unsupported camera model for Bundler export." << std::endl;
        return false;
      }
    }
    // Export structure and visibility
    for (Landmarks::const_iterator iter = sfm_data.GetLandmarks().begin();
      iter != sfm_data.GetLandmarks().end(); ++iter)
    {
      const Landmark & landmark = iter->second;
      const Observations & observations = landmark.observations;
      const Vec3 & X = landmark.X;
      // X, color, obsCount
      os << X[0] << " " << X[1] << " " << X[2] << os.widen('\n')
         <<  "255 255 255" << os.widen('\n')
         << observations.size() << " ";
      for(Observations::const_iterator iterObs = observations.begin();
        iterObs != observations.end(); ++iterObs)
      {
        const Observation & ob = iterObs->second;
        // ViewId, FeatId, x, y
        os << map_viewIdToContiguous[iterObs->first] << " " << ob.id_feat << " " << ob.x(0) << " " << ob.x(1) << " ";
      }
      os << os.widen('\n');
    }
    os.close();
    osList.close();
  }
  return true;
}

int main(int argc, char *argv[])
{
  // command-line parameters

  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  std::string sfmDataFilename;
  std::string outputFolder;

  int resolution = 1;
  int nbCore = 8;
  bool useVisData = true;

  po::options_description allParams("AliceVision exportPMVS");

  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
      "SfMData file.")
    ("output,o", po::value<std::string>(&outputFolder)->required(),
      "Output path for keypoints.");

  po::options_description optionalParams("Optional parameters");
  optionalParams.add_options()
    ("resolution", po::value<int>(&resolution)->default_value(resolution),
      "Divide image coefficient")
    ("nbCore", po::value<int>(&nbCore)->default_value(nbCore),
      "Nb core")
    ("useVisData", po::value<bool>(&useVisData)->default_value(useVisData),
      "Use visibility information.");

  po::options_description logParams("Log parameters");
  logParams.add_options()
    ("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
      "verbosity level (fatal,  error, warning, info, debug, trace).");

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

  // set verbose level
  system::Logger::get()->setLogLevel(verboseLevel);

  // Create output dir
  if (!stlplus::folder_exists(outputFolder))
    stlplus::folder_create( outputFolder );

  SfMData sfm_data;
  if (!Load(sfm_data, sfmDataFilename, ESfMData(ALL))) {
    std::cerr << std::endl
      << "The input SfMData file \""<< sfmDataFilename << "\" cannot be read." << std::endl;
    return EXIT_FAILURE;
  }

  {
    exportToPMVSFormat(sfm_data,
      stlplus::folder_append_separator(outputFolder) + "PMVS",
      resolution,
      nbCore,
      useVisData);

    exportToBundlerFormat(sfm_data,
      stlplus::folder_append_separator(outputFolder) +
      stlplus::folder_append_separator("PMVS") + "bundle.rd.out",
      stlplus::folder_append_separator(outputFolder) +
      stlplus::folder_append_separator("PMVS") + "list.txt"
      );

    return( EXIT_SUCCESS );
  }

  // Exit program
  return( EXIT_FAILURE );
}
