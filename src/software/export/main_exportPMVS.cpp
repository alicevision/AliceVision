// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/image/all.hpp>
#include <aliceVision/system/main.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/progress.hpp>

#include <stdlib.h>
#include <stdio.h>
#include <cmath>
#include <iterator>
#include <iomanip>

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

bool exportToPMVSFormat(
  const SfMData & sfm_data,
  const std::string & sOutDirectory,  //Output PMVS files folder
  const int downsampling_factor,
  const int CPU_core_count,
  const bool b_VisData = true
  )
{
  bool bOk = true;
  if (!fs::exists(sOutDirectory))
  {
    fs::create_directory(sOutDirectory);
    bOk = fs::is_directory(sOutDirectory);
  }

  // Create basis folder structure
  fs::create_directory(fs::path(sOutDirectory) / std::string("models"));
  fs::create_directory(fs::path(sOutDirectory) / std::string("txt"));
  fs::create_directory(fs::path(sOutDirectory) / std::string("visualize"));

  if (bOk &&
      fs::is_directory(fs::path(sOutDirectory) / std::string("models")) &&
      fs::is_directory(fs::path(sOutDirectory) / std::string("txt")) &&
      fs::is_directory(fs::path(sOutDirectory) / std::string("visualize")))
  {
    bOk = true;
  }
  else  {
    std::cerr << "Cannot access to one of the desired output folder" << std::endl;
  }

  if (bOk)
  {
    boost::progress_display my_progress_bar( sfm_data.getViews().size()*2 );

    // Since PMVS requires contiguous camera index, and that some views can have some missing poses,
    // we reindex the poses to ensure a contiguous pose list.
    HashMap<IndexT, IndexT> map_viewIdToContiguous;

    // Export valid views as Projective Cameras:
    for(Views::const_iterator iter = sfm_data.getViews().begin();
      iter != sfm_data.getViews().end(); ++iter, ++my_progress_bar)
    {
      const View * view = iter->second.get();
      if (!sfm_data.isPoseAndIntrinsicDefined(view))
        continue;

      const Pose3 pose = sfm_data.getPose(*view).getTransform();
      Intrinsics::const_iterator iterIntrinsic = sfm_data.getIntrinsics().find(view->getIntrinsicId());

      // View Id re-indexing
      map_viewIdToContiguous.insert(std::make_pair(view->getViewId(), map_viewIdToContiguous.size()));

      std::shared_ptr<camera::IntrinsicBase> cam = iterIntrinsic->second;
      std::shared_ptr<camera::Pinhole> camPinHole = std::dynamic_pointer_cast<camera::Pinhole>(cam);
      if (!camPinHole) {
        ALICEVISION_LOG_ERROR("Camera is not pinhole in filter");
        continue;
      }

      // We have a valid view with a corresponding camera & pose
      const Mat34 P = camPinHole->getProjectiveEquivalent(pose);
      std::ostringstream os;
      os << std::setw(8) << std::setfill('0') << map_viewIdToContiguous[view->getViewId()];
      std::ofstream file((fs::path(sOutDirectory) / std::string("txt") / (os.str() + ".txt")).string());
      file << "CONTOUR" << os.widen('\n')
        << P.row(0) <<"\n"<< P.row(1) <<"\n"<< P.row(2) << os.widen('\n');
      file.close();
    }

    // Export (calibrated) views as undistorted images
    Image<RGBColor> image, image_ud;
    for(Views::const_iterator iter = sfm_data.getViews().begin();
      iter != sfm_data.getViews().end(); ++iter, ++my_progress_bar)
    {
      const View * view = iter->second.get();
      if (!sfm_data.isPoseAndIntrinsicDefined(view))
        continue;

      Intrinsics::const_iterator iterIntrinsic = sfm_data.getIntrinsics().find(view->getIntrinsicId());

      // We have a valid view with a corresponding camera & pose
      const std::string srcImage = view->getImagePath();
      std::ostringstream os;
      os << std::setw(8) << std::setfill('0') << map_viewIdToContiguous[view->getViewId()];
      const std::string dstImage = (fs::path(sOutDirectory) / std::string("visualize") / (os.str() + ".jpg")).string();
      const IntrinsicBase * cam = iterIntrinsic->second.get();
      if (cam->isValid() && cam->hasDistortion())
      {
        // undistort the image and save it
        readImage( srcImage, image, image::EImageColorSpace::NO_CONVERSION);
        UndistortImage(image, cam, image_ud, BLACK);
        writeImage(dstImage, image_ud, image::EImageColorSpace::NO_CONVERSION);
      }
      else // (no distortion)
      {
        // copy the image if extension match
        if (fs::extension(srcImage) == ".JPG" ||
          fs::extension(srcImage) == ".jpg")
        {
          fs::copy_file(srcImage, dstImage);
        }
        else
        {
          readImage(srcImage, image, image::EImageColorSpace::NO_CONVERSION);
          writeImage(dstImage, image, image::EImageColorSpace::NO_CONVERSION);
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
      for (Landmarks::const_iterator itL = sfm_data.getLandmarks().begin();
        itL != sfm_data.getLandmarks().end(); ++itL)
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
      std::ofstream file((fs::path(sOutDirectory) / "vis.dat").string());
      file << osVisData.str();
      file.close();
    }

    std::ofstream file((fs::path(sOutDirectory) / "pmvs_options.txt").string());
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
    for(Views::const_iterator iter = sfm_data.getViews().begin();
      iter != sfm_data.getViews().end(); ++iter)
    {
      const View * view = iter->second.get();
      if (!sfm_data.isPoseAndIntrinsicDefined(view))
        continue;

      // View Id re-indexing
      map_viewIdToContiguous.insert(std::make_pair(view->getViewId(), map_viewIdToContiguous.size()));
    }

    // Fill the "Bundle file"
    os << "# Bundle file v0.3" << os.widen('\n')
      << map_viewIdToContiguous.size()  << " " << sfm_data.getLandmarks().size() << os.widen('\n');

    // Export camera properties & image filenames
    for(Views::const_iterator iter = sfm_data.getViews().begin();
      iter != sfm_data.getViews().end(); ++iter)
    {
      const View * view = iter->second.get();
      if (!sfm_data.isPoseAndIntrinsicDefined(view))
        continue;

      const Pose3 pose = sfm_data.getPose(*view).getTransform();
      Intrinsics::const_iterator iterIntrinsic = sfm_data.getIntrinsics().find(view->getIntrinsicId());

      // Must export focal, k1, k2, R, t

      Mat3 D;
      D.fill(0.0);
      D .diagonal() = Vec3(1., -1., -1.); // mapping between our pinhole and Bundler convention
      const double k1 = 0.0, k2 = 0.0; // distortion already removed

      if(isPinhole(iterIntrinsic->second.get()->getType()))
      {
        const Pinhole * cam = dynamic_cast<const Pinhole*>(iterIntrinsic->second.get());

        if (cam->getFocalLengthPixX() == cam->getFocalLengthPixY()) 
        {
          const double focal = cam->getFocalLengthPixX();
          const Mat3 R = D * pose.rotation();
          const Vec3 t = D * pose.translation();

          os << focal << " " << k1 << " " << k2 << os.widen('\n') //f k1 k2
            << R(0,0) << " " << R(0, 1) << " " << R(0, 2) << os.widen('\n')  //R.row(0)
            << R(1,0) << " " << R(1, 1) << " " << R(1, 2) << os.widen('\n')  //R.row(1)
            << R(2,0) << " " << R(2, 1) << " " << R(2, 2) << os.widen('\n')  //R.row(2)
            << t(0)   << " " << t(1)    << " " << t(2)    << os.widen('\n'); //t

          osList << fs::path(view->getImagePath()).filename() << " 0 " << focal << os.widen('\n');
        }
        else 
        {
          std::cerr << "Unsupported anamorphic camera for Bundler export." << std::endl;
          return false;
        }
      }
      else
      {
        std::cerr << "Unsupported camera model for Bundler export." << std::endl;
        return false;
      }
    }
    // Export structure and visibility
    for (Landmarks::const_iterator iter = sfm_data.getLandmarks().begin();
      iter != sfm_data.getLandmarks().end(); ++iter)
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

int aliceVision_main(int argc, char *argv[])
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
  if (!fs::exists(outputFolder))
    fs::create_directory(outputFolder);

  SfMData sfmData;
  if(!sfmDataIO::Load(sfmData, sfmDataFilename, sfmDataIO::ESfMData::ALL))
  {
    std::cerr << std::endl
      << "The input SfMData file \""<< sfmDataFilename << "\" cannot be read." << std::endl;
    return EXIT_FAILURE;
  }

  {
    exportToPMVSFormat(sfmData,
      (fs::path(outputFolder) / std::string("PMVS")).string(),
      resolution,
      nbCore,
      useVisData);

    exportToBundlerFormat(sfmData,
      (fs::path(outputFolder) /
      std::string("PMVS") /
      std::string("bundle.rd.out")).string(),
      (fs::path(outputFolder) /
      std::string("PMVS") /
      std::string("list.txt")).string());

    return EXIT_SUCCESS;
  }

  // Exit program
  return EXIT_FAILURE;
}
