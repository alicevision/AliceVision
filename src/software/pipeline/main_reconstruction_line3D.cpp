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
  std::string outputMeshFilepath;
  std::string outputEdgePointpath;
  std::string inputLinesFolder;

  bool load_segments = true; // load segments from files
  int max_img_width = -1;
  unsigned int max_line_segments = 10000;
  bool neighbors_by_worldpoints = false;
  bool use_GPU = true;

  float sigma_position = 2.0f;
  float sigma_angle = 5.0f; // in degrees
  unsigned int num_neighbors = 0;
  float epipolar_overlap = 0.0f;
  int kNN = 0;
  float const_regularization_depth = 0.0f;

  unsigned int visibility_t = 2;
  bool perform_diffusion = true;
  float collinearity_t = 6.0f;
  bool use_CERES = true;
  unsigned int max_iter_CERES = 0;

  double pixelStep = 5.0;

  po::options_description allParams("AliceVision convertSfMFormat");

  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
      "SfMData file.")
    ("inputLinesFolder",  po::value<std::string>(&inputLinesFolder)->required(),
     "Folder with extracted lines. Filename should use the image uid.")
    ("output,o", po::value<std::string>(&outputMeshFilepath)->required(),
      "Path to the output Mesh file.")
    ("outputSfM,o", po::value<std::string>(&outputEdgePointpath)->required(),
      "Path to the output Edge Point Cloud file.");

  po::options_description optionalParams("Optional parameters");
  optionalParams.add_options()
     ("sigmaPosition", po::value<float>(&sigma_position)->default_value(sigma_position),
      "spatial regularizer (for scoring and clustering)"
      "if > 0 : in pixels (regularizer derived from image space and unprojected into 3D space) [scale invariant]"
      "if < 0 : in meters (regularizer directly defined in world coordinates) [not scale invariant].")
    ("sigmaAngle", po::value<float>(&sigma_angle)->default_value(sigma_angle),
      "angular regularizer (for scoring and clustering) defined in degrees.")
    ("numNeighbors", po::value<unsigned int>(&num_neighbors)->default_value(num_neighbors),
      "number of neighboring images with which each image is matched.")
    ("epipolarOverlap", po::value<float>(&epipolar_overlap)->default_value(epipolar_overlap),
      " minimum overlap of a line segment with the epipolar beam of another segment, to be considered a potential match (in [0,1]).")
    ("kNN", po::value<int>(&kNN)->default_value(kNN),
      "k-nearest-neighbor matching, if > 0 : keep only the k matches with the highest epipolar overlap (per image)"
      "if <= 0 : keep all matches that fulfill the epipolar_overlap.")
    ("visibility", po::value<unsigned int>(&visibility_t)->default_value(visibility_t),
      "minimum number of different cameras from which clustered 2D segments must originate, "
      "such that the resulting 3D line is considered to be valid.")
    ("collinearity", po::value<float>(&collinearity_t)->default_value(collinearity_t),
      "threshold (in pixels) for segments from one image to be considered potentially collinear, "
      "if <= 0 : collinearity not considered (default).")
    ("maxIterCeres", po::value<unsigned int>(&max_iter_CERES)->default_value(max_iter_CERES),
      "maximum number of iterations for Ceres.")
    ("useGPU", po::value<bool>(&use_GPU)->default_value(use_GPU),
      "uses the GPU for processing whenever possible (highly recommended, requires CUDA!). ")
    ("pixelStep", po::value<double>(&pixelStep)->default_value(pixelStep),
      "step in pixels between each point of extracted lines. ");

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

  if(sfmDataFilename.empty() || outputMeshFilepath.empty())
  {
    ALICEVISION_LOG_ERROR("Invalid input or output filename");
    return EXIT_FAILURE;
  }

  const std::string outputFolder = fs::path(outputMeshFilepath).parent_path().string();

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
  // lines_folder            - folder where the temp directory will be created
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
  L3DPP::Line3D line3D(inputLinesFolder, load_segments, max_img_width, max_line_segments, neighbors_by_worldpoints, use_GPU);

  // export valid views as projective cameras
  for(auto viewIt : sfmData.getViews())
  {
      const IndexT viewId = viewIt.first;
      const sfmData::View& view = *viewIt.second;
      if (!sfmData.isPoseAndIntrinsicDefined(&view))
        continue;

      ALICEVISION_LOG_INFO("Process view " << viewId << ".");

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

      L3DPP::DataArray<float4>* lines = line3D.loadLineSegments(viewId);

      if(lines == nullptr)
      {
          continue;
          //ALICEVISION_LOG_ERROR("Missing segment file for view '" << viewId << "' in folder: " << inputLinesFolder);
          //return EXIT_FAILURE;
      }
      ALICEVISION_LOG_INFO("Loaded " << lines->width() << " lines.");
      cv::Mat imageCv;

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
      line3D.addImage(viewId, imageCv, K, R, t, median_depth, wps_or_neighbors, lines);
  }


  ALICEVISION_LOG_INFO("Match images");
  // void matchImages(...): matches 2D line segments between images
  // -------------------------------------
  // PARAMETERS:
  // -------------------------------------
  // sigma_position             - spatial regularizer (for scoring and clustering)
  //                              if > 0 -> in pixels (regularizer derived from image space and unprojected into 3D space) [scale invariant]
  //                              if < 0 -> in "meters" (regularizer directly defined in world coordinates) [not scale invariant]
  //                              the second method is recommended when the scale is known!
  // sigma_angle                - angular regularizer (for scoring and clustering)
  //                              defined in degrees (not radiants!)
  // num_neighbors              - number of neighboring images with which each image is matched
  // epipolar_overlap           - minimum overlap of a line segment with the epipolar beam of another segment,
  //                              to be considered a potential match (in [0,1])
  // kNN                        - k-nearest-neighbor matching
  //                              if > 0  -> keep only the k matches with the highest epipolar overlap (per image)
  //                              if <= 0 -> keep all matches that fulfill the epipolar_overlap
  // const_regularization_depth - if positive (and sigma_position is in "meters"), this depth is where
  //                              an uncertainty of 'sigma_position' is allowed (e.g. use 5.0 when you want to
  //                              initialize sigma_p 5 meters in front of the camera)
  line3D.matchImages(sigma_position, sigma_angle, num_neighbors, epipolar_overlap, kNN, const_regularization_depth);

  ALICEVISION_LOG_INFO("Reconstruct 3D lines");
  // void reconstruct3Dlines(...): reconstruct a line-based 3D model (after matching)
  // -------------------------------------
  // PARAMETERS:
  // -------------------------------------
  // visibility_t      - minimum number of different cameras from which clustered 2D segments must originate,
  //                     such that the resulting 3D line is considered to be valid
  // perform_diffusion - perform Replicator Dynamics Diffusion [Donoser, BMVC'13] before
  //                     segment clustering
  // collinearity_t    - threshold (in pixels) for segments from one image to be considered potentially collinear
  //                     if <= 0 -> collinearity not considered (default)
  // use_CERES         - 3D lines are optimized (bundled) using the Ceres-Solver (recommended!)
  // max_iter_CERES    - maximum number of iterations for Ceres
  line3D.reconstruct3Dlines(visibility_t, perform_diffusion, collinearity_t, use_CERES, max_iter_CERES);

  ALICEVISION_LOG_INFO("Retrieve reconstructed lines");
  std::vector<L3DPP::FinalLine3D> result;
  // void get3Dlines(...): returns the current 3D model
  // -------------------------------------
  // PARAMETERS:
  // -------------------------------------
  // result - list of reconstructed 3D lines (see "segment3D.h")
  line3D.get3Dlines(result);
  ALICEVISION_LOG_INFO("Number of 3D lines: " << result.size());

  ALICEVISION_LOG_INFO("Export OBJ: " << outputMeshFilepath);
  line3D.saveResultAsOBJ(outputMeshFilepath);

  IndexT maxLandmarkId = 0;

  for(auto& landmark: sfmData.getLandmarks())
  {
      maxLandmarkId = std::max(landmark.first, maxLandmarkId);
  }
  std::size_t nbSegments = 0;

  std::vector<double> nbVisibilities;

  for(const L3DPP::FinalLine3D& finalLine: result)
  {
      const L3DPP::LineCluster3D& line = finalLine.underlyingCluster_;
      std::set<IndexT> visibilities;
      //std::vector<IndexT> visibilities;
      std::vector<float> segLengths;
      for(const auto& seg2d: *line.residuals())
      {
          // visibilities.push_back(seg2d.camID());
          visibilities.insert(seg2d.camID());

          Eigen::Vector4f s = line3D.getSegmentCoords2D(seg2d);
          ALICEVISION_LOG_INFO("s : " << s);
          Eigen::Vector2f a(s(0), s(1));
          Eigen::Vector2f b(s(2), s(3));
          ALICEVISION_LOG_INFO("(b-a).norm() : " << (b-a).norm());
          segLengths.push_back((b-a).norm());
      }
      float thirdLargestSeg = 0.0f;
      {
          int index = std::min(std::size_t(3), segLengths.size());
          std::partial_sort(segLengths.begin(), segLengths.begin()+index, segLengths.end(), std::greater<int>());
          thirdLargestSeg = segLengths[index];
      }
      // BoxStats<float> lengthsStats(segLengths.begin(), segLengths.end());

      ALICEVISION_LOG_INFO("thirdLargestSeg : " << thirdLargestSeg);
      nbVisibilities.push_back(visibilities.size());

      int nbSteps = std::round(thirdLargestSeg / pixelStep);
      ALICEVISION_LOG_INFO("nbSteps : " << nbSteps);

      nbSteps = std::max(2, nbSteps);
      for(const L3DPP::Segment3D& segment: finalLine.collinear3Dsegments_)
      {
          Eigen::Vector3d v = segment.P2() - segment.P1();
          for(int i = 0; i < nbSteps; ++i)
          {
              aliceVision::sfmData::Landmark& landmark = sfmData.getLandmarks()[++maxLandmarkId];
              landmark.descType = feature::EImageDescriberType::UNKNOWN;
              landmark.X = segment.P1() + v * (i / double(nbSteps-1));
              for(IndexT v: visibilities)
                landmark.observations[v];
          }
          /*
          {
              aliceVision::sfmData::Landmark& landmark = sfmData.getLandmarks()[++maxLandmarkId];
              landmark.descType = feature::EImageDescriberType::UNKNOWN;
              landmark.X = segment.P1();
              for(IndexT v: visibilities)
                landmark.observations[v];
          }
          {
              aliceVision::sfmData::Landmark& landmark = sfmData.getLandmarks()[++maxLandmarkId];
              landmark.descType = feature::EImageDescriberType::UNKNOWN;
              landmark.X = segment.P2();
              for(IndexT v: visibilities)
                landmark.observations[v];
          }*/
          ++nbSegments;
      }
  }

  BoxStats<double> nbVisibilitiesPerLine(nbVisibilities.begin(), nbVisibilities.end());

  ALICEVISION_LOG_INFO("Nb visibilities per line:\n" << nbVisibilitiesPerLine);
  /*
  for(const L3DPP::FinalLine3D& finalLine: result)
  {
      const L3DPP::LineCluster3D& line = finalLine.underlyingCluster_;
      //for(const L3DPP::LineCluster3D& line: line.underlyingCluster_)
      {
          std::vector<IndexT> visibilities;
          for(const auto& seg2d: *line.residuals())
          {
              visibilities.push_back(seg2d.camID());
          }
          int nbSteps = 100;
          Eigen::Vector3d v = line.seg3D().P2() - line.seg3D().P1();
          for(int i = 0; i < nbSteps; ++i)
          {
              aliceVision::sfmData::Landmark& landmark = sfmData.getLandmarks()[++maxLandmarkId];
              landmark.descType = feature::EImageDescriberType::UNKNOWN;
              landmark.X = line.seg3D().P1() + v * (i / double(nbSteps-1));
              for(IndexT v: visibilities)
                landmark.observations[v];
          }

//          {
//              aliceVision::sfmData::Landmark& landmark = sfmData.getLandmarks()[++maxLandmarkId];
//              landmark.descType = feature::EImageDescriberType::UNKNOWN;
//              landmark.X = line.seg3D().P1();
//              for(IndexT v: visibilities)
//                landmark.observations[v];
//          }
//          {
//              aliceVision::sfmData::Landmark& landmark = sfmData.getLandmarks()[++maxLandmarkId];
//              landmark.descType = feature::EImageDescriberType::UNKNOWN;
//              landmark.X = line.seg3D().P2();
//              for(IndexT v: visibilities)
//                landmark.observations[v];
//          }
          ++nbSegments;
      }
  }
  */

  ALICEVISION_LOG_INFO("Number of 3D segments: " << nbSegments);

  ALICEVISION_LOG_INFO("Export Point Cloud: " << outputEdgePointpath);
  // export the SfMData scene in the expected format
  if(!sfmDataIO::Save(sfmData, outputEdgePointpath, sfmDataIO::ESfMData::ALL))
  {
    ALICEVISION_LOG_ERROR("An error occured while trying to save '" << outputEdgePointpath << "'");
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
