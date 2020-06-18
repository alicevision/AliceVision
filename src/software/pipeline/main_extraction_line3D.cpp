// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2015 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/image/all.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/config.hpp>

#include <line3D/configLIBS.h>
#include <line3D/line3D.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <Eigen/Eigen>

#include <boost/program_options.hpp>
#include <boost/system/error_code.hpp>
#include <boost/filesystem.hpp>

#include <algorithm>
#include <string>
#include <regex>


// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 2
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;
namespace fs = boost::filesystem;


// convert from a SfMData format to another
int aliceVision_main(int argc, char **argv)
{
  // command-line parameters

  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  std::string sfmDataFilename;
  std::string outputFolder;
  std::vector<std::string> imagesFolders;
  int rangeStart = -1;
  int rangeSize = 1;

  const bool load_segments = true; // store lines to files
  int max_img_width = 0;
  unsigned int max_line_segments = 10000;
  bool neighbors_by_worldpoints = false;
  bool use_GPU = true;

  po::options_description allParams("AliceVision extraction 3D lines");

  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
      "SfMData file.")
    ("output,o", po::value<std::string>(&outputFolder)->required(),
      "Path of the output folder to store extracted lines.");

  po::options_description optionalParams("Optional parameters");
  optionalParams.add_options()
    ("imagesFolders",  po::value<std::vector<std::string>>(&imagesFolders)->multitoken(),
      "Use images from specific folder(s) instead of those specify in the SfMData file.\n"
      "Filename should be the same or the image uid.")
    ("maxImgWidth", po::value<int>(&max_img_width)->default_value(max_img_width),
      "maximum width (or height, for portrait images) to which images are resized "
      "for line segment detection (coordinates will be upscaled afterwards!)"
      "if <= 0, the images are not resized.")
    ("maxLineSegments", po::value<unsigned int>(&max_line_segments)->default_value(max_line_segments),
      "maximum number of 2D line segments per image (sorted by length).")
    ("useGPU", po::value<bool>(&use_GPU)->default_value(use_GPU),
      "uses the GPU for processing whenever possible (highly recommended, requires CUDA!). ")
    ("rangeStart", po::value<int>(&rangeStart)->default_value(rangeStart),
      "Range image index start.")
    ("rangeSize", po::value<int>(&rangeSize)->default_value(rangeSize),
      "Range size.");


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

  ALICEVISION_COUT("Program called with the following parameters:");
  ALICEVISION_COUT(vm);

  // set verbose level
  system::Logger::get()->setLogLevel(verboseLevel);

  if(sfmDataFilename.empty() || outputFolder.empty())
  {
    ALICEVISION_LOG_ERROR("Invalid input or output filename");
    return EXIT_FAILURE;
  }

  if(imagesFolders.empty())
  {
      throw std::runtime_error("'imagesFolders' is required.");
  }

  // load input SfMData scene
  sfmData::SfMData sfmData;
  if(!sfmDataIO::Load(sfmData, sfmDataFilename, sfmDataIO::ESfMData::ALL))
  {
    ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFilename << "' cannot be read");
    return EXIT_FAILURE;
  }

  // Line3D++ constructor
  // -------------------------------------
  // PARAMETERS:
  // -------------------------------------
  // output_folder            - folder where the temp directory will be created
  // load_segments            - if true  -> detected 2D line segments will be serialized to hard drive
  //                                        and reloaded when the dataset is processed again
  //                            if false -> the line segments are redetected everytime
  // max_img_width            - maximum width (or height, for portrait images) to which images are resized
  //                            for line segment detection (coordinates will be upscaled afterwards!).
  //                            if set to -1, the images are not resized
  // max_line_segments        - maximum number of 2D line segments per image (sorted by length)
  // neighbors_by_worldpoints - if true  -> matching neighbors (images) are derived from the common worldpoints
  //                            if false -> an explicit list of matching neighbors has to be provided
  //                            (--> see void addImage(...))
  // use_GPU                  - uses the GPU for processing whenever possible (highly recommended, requires CUDA!)
  L3DPP::Line3D line3D(outputFolder, load_segments, max_img_width, max_line_segments, neighbors_by_worldpoints, use_GPU);

  // export valid views as projective cameras
  for(auto viewIt : sfmData.getViews())
  {
      const IndexT viewId = viewIt.first;
      const sfmData::View& view = *viewIt.second;
      if (!sfmData.isPoseAndIntrinsicDefined(&view))
        continue;

      std::string srcImagePath;
      {
        bool found = false;
        for(const std::string& folder : imagesFolders)
        {
          const fs::recursive_directory_iterator end;
          const auto findIt = std::find_if(fs::recursive_directory_iterator(folder), end,
                                   [&view](const fs::directory_entry& e) {
                                      return (e.path().stem() == std::to_string(view.getViewId()) ||
                                              e.path().stem() == fs::path(view.getImagePath()).stem());});

          if(findIt != end)
          {
            srcImagePath = (fs::path(folder) / (findIt->path().stem().string() + findIt->path().extension().string())).string();
            found = true;
            break;
          }
        }

        if(!found)
          throw std::runtime_error("Cannot find view " + std::to_string(view.getViewId()) + " image file in given folder(s)");
      }

      ALICEVISION_LOG_INFO("Process view " << viewId << ".");

      image::Image<unsigned char> image; // Should we use RGBColor?
      readImage(srcImagePath, image, image::EImageColorSpace::SRGB);

      cv::Mat imageCv;
      cv::eigen2cv(image.GetMat(), imageCv);

      // get camera pose / projection
      const geometry::Pose3 pose = sfmData.getPose(view).getTransform();
      // get camera intrinsics matrices
      const Mat3 K = dynamic_cast<const camera::Pinhole*>(sfmData.getIntrinsicPtr(view.getIntrinsicId()))->K();
      const Mat3& R = pose.rotation();
      const Vec3& t = pose.translation();

      BoxStats<double> viewDepthsStat = sfmData.getViewLandmarkDepthStat(viewId);
      float median_depth = (float) viewDepthsStat.median;
      std::list<unsigned int> wps_or_neighbors;
      const double minViewAngle = 0.0;
      const double maxViewAngle = 360.0;
      const std::size_t nbNearestCams = 10;
      std::vector<IndexT> neighborViews = sfmData.findNearestViewsByLandmarks(viewId, nbNearestCams, minViewAngle, maxViewAngle);
      for(IndexT v: neighborViews)
          wps_or_neighbors.push_back(v);

      // void addImage(...): add a new image to the system [multithreading safe]
      // -------------------------------------
      // PARAMETERS:
      // -------------------------------------
      // camID            - unique ID of the image
      // image            - the image itself (CV_8U or CV_8UC3 supported)
      // K                - camera intrinsics (3x3)
      // R                - camera rotation (3x3)
      // t                - camera translation (3x1) [camera model: point2D = K [R | t] point3D]
      // median_depth     - median 3D worldpoint depth for this camera (i.e. median
      //                    Euclidean distance of the worldpoints to the camera center)
      // wps_or_neighbors - a list with the IDs of the
      //                    (a) worldpoints seen by this camera                --> if neighbors_by_worldpoints=true (see constructor)
      //                    (b) images with which this image should be matched --> if neighbors_by_worldpoints=false
      // line_segments    - list with the 2D line segments for this image. if it is empty (default) the line segments
      //                    will be detected by the LSD algorithm automatically
      line3D.addImage(viewId, imageCv, K, R, t, median_depth, wps_or_neighbors);
  }

  int rangeEnd = sfmData.getViews().size();

  // set extraction range
  if(rangeStart != -1)
  {
    if(rangeStart < 0 || rangeSize < 0 ||
       rangeStart > sfmData.getViews().size())
    {
      ALICEVISION_LOG_ERROR("Range is incorrect");
      return EXIT_FAILURE;
    }

    if(rangeStart + rangeSize > sfmData.views.size())
      rangeSize = sfmData.views.size() - rangeStart;

    rangeEnd = rangeStart + rangeSize;
  }
  else
  {
    rangeStart = 0;
  }


  return EXIT_SUCCESS;
}
