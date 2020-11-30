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
using namespace aliceVision::feature;

namespace po = boost::program_options;
namespace fs = boost::filesystem;

/// Naive image bilinear resampling of an image for thumbnail generation
template <typename ImageT>
ImageT
create_thumbnail
(
  const ImageT & image,
  int thumb_width,
  int thumb_height
);

/* Notes:
 * - An MVE2 scene appears to duplicate camera rot matrix and trans vector per-view data in 'meta.ini'
 *   within the first section of 'synth_0.out'.
 * - We do not save the original, instead we rely on the undistorted image from aliceVision.
 * - We do not output thumbnails or EXIF blobs, as these appear only to be used only for the GUI UMVE.
 * - To avoid encoding loss, aliceVision images should be written as .PNG if undistorted images are *not* computed.
 * - In AliceVision, some views may have some missing poses; MVE does *not* require a contiguous camera index.
 *
 *  For information on the target for this conversion, please see the MVE (v2) File format:
 *  https://github.com/simonfuhrmann/mve/wiki/MVE-File-Format
 */

bool exportToMVE2Format(
  const SfMData & sfm_data,
  const std::string & sOutDirectory // Output MVE2 files folder
  )
{
  bool bOk = true;
  // Create basis folder structure
  if (!fs::is_directory(sOutDirectory))
  {
    std::cout << "\033[1;31mCreating folder:  " << sOutDirectory << "\033[0m\n";
    fs::create_directory(sOutDirectory);
    bOk = fs::is_directory(sOutDirectory);
  }

  if (!bOk)
  {
    std::cerr << "Cannot access one of the desired output directories" << std::endl;
	  return false;
  }

  // Export the SfMData scene to the MVE2 format
  {
    // Create 'views' subfolder
    const std::string sOutViewsDirectory = (fs::path(sOutDirectory) / "views").string();
    if (!fs::exists(sOutViewsDirectory))
    {
      std::cout << "\033[1;31mCreating folder:  " << sOutViewsDirectory << "\033[0m\n";
      fs::create_directory(sOutViewsDirectory);
    }

    // Prepare to write bundle file
    // Get cameras and features from AliceVision
    size_t cameraCount = 0;
    for(const auto& view: sfm_data.getViews())
        if(sfm_data.isPoseAndIntrinsicDefined(view.second.get()))
            ++cameraCount;
    // Tally global set of feature landmarks
    const Landmarks & landmarks = sfm_data.getLandmarks();
    const size_t featureCount = std::distance(landmarks.begin(), landmarks.end());
    const std::string filename = "synth_0.out";
    std::cout << "Writing bundle (" << cameraCount << " cameras, "
        << featureCount << " features): to " << filename << "...\n";
    std::ofstream out((fs::path(sOutDirectory) / filename).string());
    out << "drews 1.0\n";  // MVE expects this header
    out << cameraCount << " " << featureCount << "\n";

    // Export (calibrated) views as undistorted images
    boost::progress_display my_progress_bar(sfm_data.getViews().size());
    std::pair<int,int> w_h_image_size;
    Image<RGBColor> image, image_ud, thumbnail;
    std::string sOutViewIteratorDirectory;
    std::size_t view_index = 0;
    std::map<std::size_t, IndexT> viewIdToviewIndex;
    for(Views::const_iterator iter = sfm_data.getViews().begin();
      iter != sfm_data.getViews().end(); ++iter, ++my_progress_bar)
    {
      const View * view = iter->second.get();

      if (!sfm_data.isPoseAndIntrinsicDefined(view))
        continue;

      viewIdToviewIndex[view->getViewId()] = view_index;
      // Create current view subfolder 'view_xxxx.mve'
      std::ostringstream padding;
      // Warning: We use view_index instead of view->getViewId() because MVE use indexes instead of IDs.
      padding << std::setw(4) << std::setfill('0') << view_index;

      sOutViewIteratorDirectory = (fs::path(sOutViewsDirectory) / ("view_" + padding.str() + ".mve")).string();
      if (!fs::exists(sOutViewIteratorDirectory))
      {
        fs::create_directory(sOutViewIteratorDirectory);
      }

      // We have a valid view with a corresponding camera & pose
      const std::string srcImage = view->getImagePath();
      const std::string dstImage = (fs::path(sOutViewIteratorDirectory) / "undistorted.png").string();

      Intrinsics::const_iterator iterIntrinsic = sfm_data.getIntrinsics().find(view->getIntrinsicId());
      const IntrinsicBase * cam = iterIntrinsic->second.get();
      if (cam->isValid() && cam->hasDistortion())
      {
        // Undistort and save the image
        readImage(srcImage, image, image::EImageColorSpace::NO_CONVERSION);
        UndistortImage(image, cam, image_ud, BLACK);
        writeImage(dstImage, image_ud, image::EImageColorSpace::NO_CONVERSION);
      }
      else // (no distortion)
      {
        // If extensions match, copy the PNG image
        if (fs::extension(srcImage) == ".PNG" ||
          fs::extension(srcImage) == ".png")
        {
          fs::copy_file(srcImage, dstImage);
        }
        else
        {
          readImage( srcImage, image, image::EImageColorSpace::NO_CONVERSION);
          writeImage( dstImage, image, image::EImageColorSpace::NO_CONVERSION);
        }
      }

      // Prepare to write an MVE 'meta.ini' file for the current view
      const Pose3 pose = sfm_data.getPose(*view).getTransform();
      const Pinhole * pinhole_cam = static_cast<const Pinhole *>(cam);

      const Mat3& rotation = pose.rotation();
      const Vec3& translation = pose.translation();
      
      // Focal length and principal point must be normalized (0..1)
      const float flen = pinhole_cam->getFocalLengthPixX() / static_cast<double>(std::max(cam->w(), cam->h()));
      const float pixelAspect = pinhole_cam->getFocalLengthPixX() / pinhole_cam->getFocalLengthPixY();
      const float ppX = std::abs(pinhole_cam->getPrincipalPoint()(0)/cam->w());
      const float ppY = std::abs(pinhole_cam->getPrincipalPoint()(1)/cam->h());

      // For each camera, write to bundle:  focal length, radial distortion[0-1], rotation matrix[0-8], translation vector[0-2]
      std::ostringstream fileOut;
      fileOut
        << "# MVE view meta data is stored in INI-file syntax." << fileOut.widen('\n')
        << "# This file is generated, formatting will get lost." << fileOut.widen('\n')
        << fileOut.widen('\n')
        << "[camera]" << fileOut.widen('\n')
        << "focal_length = " << flen << fileOut.widen('\n')
        << "pixel_aspect = " << pixelAspect << fileOut.widen('\n')
        << "principal_point = " << ppX << " " << ppY << fileOut.widen('\n')
        << "rotation = " << rotation(0, 0) << " " << rotation(0, 1) << " " << rotation(0, 2) << " "
        << rotation(1, 0) << " " << rotation(1, 1) << " " << rotation(1, 2) << " "
        << rotation(2, 0) << " " << rotation(2, 1) << " " << rotation(2, 2) << fileOut.widen('\n')
        << "translation = " << translation[0] << " " << translation[1] << " "
        << translation[2] << " " << fileOut.widen('\n')
        << fileOut.widen('\n')
        << "[view]" << fileOut.widen('\n')
        << "id = " << view_index << fileOut.widen('\n')
        << "name = " << fs::path(srcImage.c_str()).filename().string() << fileOut.widen('\n');

      // To do:  trim any extra separator(s) from aliceVision name we receive, e.g.:
      // '/home/insight/aliceVision_KevinCain/aliceVision_Build/software/SfM/ImageDataset_SceauxCastle/images//100_7100.JPG'
      std::ofstream file((fs::path(sOutViewIteratorDirectory) / "meta.ini").string());
      file << fileOut.str();
      file.close();

      out
        << flen << " " << "0" << " " << "0" << "\n"  // Write '0' distortion values for pre-corrected images
        << rotation(0, 0) << " " << rotation(0, 1) << " " << rotation(0, 2) << "\n"
        << rotation(1, 0) << " " << rotation(1, 1) << " " << rotation(1, 2) << "\n"
        << rotation(2, 0) << " " << rotation(2, 1) << " " << rotation(2, 2) << "\n"
        << translation[0] << " " << translation[1] << " " << translation[2] << "\n";

      // Save a thumbnail image "thumbnail.png", 50x50 pixels
      thumbnail = create_thumbnail(image, 50, 50);
      const std::string dstThumbnailImage = (fs::path(sOutViewIteratorDirectory) / "thumbnail.png").string();
      writeImage(dstThumbnailImage, thumbnail, image::EImageColorSpace::NO_CONVERSION);
      
      ++view_index;
    }

    // For each feature, write to bundle:  position XYZ[0-3], color RGB[0-2], all ref.view_id & ref.feature_id
    // The following method is adapted from Simon Fuhrmann's MVE project:
    // https://github.com/simonfuhrmann/mve/blob/e3db7bc60ce93fe51702ba77ef480e151f927c23/libs/mve/bundle_io.cc

    for (Landmarks::const_iterator iterLandmarks = landmarks.begin(); iterLandmarks != landmarks.end(); ++iterLandmarks)
    {
      const Vec3 exportPoint = iterLandmarks->second.X;
      out << exportPoint.x() << " " << exportPoint.y() << " " << exportPoint.z() << "\n";
      out << 250 << " " << 100 << " " << 150 << "\n";  // Write arbitrary RGB color, see above note

      // Tally set of feature observations
      const Observations & observations = iterLandmarks->second.observations;
      const size_t featureCount = std::distance(observations.begin(), observations.end());
      out << featureCount;

      for (Observations::const_iterator itObs = observations.begin(); itObs != observations.end(); ++itObs)
      {
          const IndexT viewId = itObs->first;
          const IndexT viewIndex = viewIdToviewIndex[viewId];
          const IndexT featId = itObs->second.id_feat;
          out << " " << viewIndex << " " << featId << " 0";
      }
      out << "\n";
    }
    out.close();
  }
  return bOk;
}

int aliceVision_main(int argc, char *argv[])
{
  // command-line parameters

  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  std::string sfmDataFilename;
  std::string outDirectory;

  po::options_description allParams("AliceVision exportMVE2");

  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
      "SfMData file.")
    ("output,o", po::value<std::string>(&outDirectory)->required(),
      "Output folder.\n"
      "Note:  this program writes output in MVE file format");

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
  if (!fs::exists(outDirectory))
    fs::create_directory(outDirectory);

  // Read the input SfM scene
  SfMData sfmData;
  if(!sfmDataIO::Load(sfmData, sfmDataFilename, sfmDataIO::ESfMData::ALL))
  {
    std::cerr << std::endl
      << "The input SfMData file \""<< sfmDataFilename << "\" cannot be read." << std::endl;
    return EXIT_FAILURE;
  }

  if (exportToMVE2Format(sfmData, (fs::path(outDirectory) / "MVE").string()))
    return EXIT_SUCCESS;
  else
    return EXIT_FAILURE;
}

/// Naive image bilinear resampling of an image for thumbnail generation
/// Inspired by create_thumbnail from MVE (cropping is here ignored)
template <typename ImageT>
ImageT
create_thumbnail
(
  const ImageT & image,
  int thumb_width,
  int thumb_height
)
{
  const int width = image.Width();
  const int height = image.Height();
  const float image_aspect = static_cast<float>(width) / height;
  const float thumb_aspect = static_cast<float>(thumb_width) / thumb_height;

  int rescale_width, rescale_height;
  if (image_aspect > thumb_aspect)
  {
    rescale_width = std::ceil(thumb_height * image_aspect);
    rescale_height = thumb_height;
  }
  else
  {
    rescale_width = thumb_width;
    rescale_height = std::ceil(thumb_width / image_aspect);
  }

  // Generation of the sampling grid
  std::vector< std::pair<float,float> > sampling_grid;
  sampling_grid.reserve(rescale_height * rescale_width);
  for ( int i = 0 ; i < rescale_height ; ++i )
  {
    for ( int j = 0 ; j < rescale_width ; ++j )
    {
      const float dx = static_cast<float>(j) * width / rescale_width;
      const float dy = static_cast<float>(i) * height / rescale_height;
      sampling_grid.push_back( std::make_pair( dy , dx ) ) ;
    }
  }

  const Sampler2d<SamplerLinear> sampler;
  ImageT imageOut;
  GenericRessample(image, sampling_grid, rescale_width, rescale_height, sampler, imageOut);
  return imageOut;
}
