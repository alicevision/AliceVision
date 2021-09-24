// This file is part of the AliceVision project.
// Copyright (c) 2018 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/sfmDataIO/AlembicExporter.hpp>
#include <aliceVision/sfmDataIO/viewIO.hpp>
#include <aliceVision/image/all.hpp>
#include <aliceVision/utils/regexFilter.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/progress.hpp>

#include <cstdlib>
#include <limits>
#include <string>
#include <regex>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 2
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;
namespace fs = boost::filesystem;



oiio::ROI computeRod(const camera::IntrinsicBase* intrinsic, bool correctPrincipalPoint)

{
    std::vector<Vec2> pointToBeChecked;
    pointToBeChecked.push_back(Vec2(0, 0));
    pointToBeChecked.push_back(Vec2(intrinsic->w() - 1, 0));
    pointToBeChecked.push_back(Vec2(0, intrinsic->h() - 1));
    pointToBeChecked.push_back(Vec2(intrinsic->w() - 1, intrinsic->h() - 1));
    const Vec2 center(intrinsic->w() * 0.5, intrinsic->h() * 0.5);
    Vec2 ppCorrection(0, 0);
    if(camera::EINTRINSIC::VALID_PINHOLE & intrinsic->getType())
    {
        const camera::Pinhole* pinholePtr = dynamic_cast<const camera::Pinhole*>(intrinsic);
        ppCorrection = pinholePtr->getPrincipalPoint() - center;
    }
    const Vec2 opticalCenter = center + ppCorrection;
    pointToBeChecked.push_back(Vec2(opticalCenter[0], 0));
    pointToBeChecked.push_back(Vec2(opticalCenter[0], intrinsic->h() - 1));
    pointToBeChecked.push_back(Vec2(0, opticalCenter[1]));
    pointToBeChecked.push_back(Vec2(intrinsic->w() - 1, opticalCenter[1]));

    std::vector<Vec2> maxDistortionVector;
    for(const Vec2& n: pointToBeChecked)
    {
      // Undistort pixel without principal point correction
      const Vec2 n_undist = intrinsic->get_ud_pixel(n);
      maxDistortionVector.push_back(n_undist);
    }

    std::sort(std::begin(maxDistortionVector), std::end(maxDistortionVector),
              [](Vec2 a, Vec2 b) { return a[0] > b[0]; });
    const int xRoiMax = std::round(maxDistortionVector.front()[0]);
    const int xRoiMin = std::round(maxDistortionVector.back()[0]);
    std::sort(std::begin(maxDistortionVector), std::end(maxDistortionVector),
              [](Vec2 a, Vec2 b) { return a[1] > b[1]; });
    const int yRoiMax = std::round(maxDistortionVector.front()[1]);
    const int yRoiMin = std::round(maxDistortionVector.back()[1]);

    oiio::ROI rod(xRoiMin, xRoiMax + 1,
                  yRoiMin, yRoiMax + 1);

    if(correctPrincipalPoint)
    {
      rod.xbegin -= ppCorrection(0);
      rod.xend -= ppCorrection(0);
      rod.ybegin -= ppCorrection(1);
      rod.yend -= ppCorrection(1);
    }
    return rod;
}

oiio::ROI convertRodToRoi(const camera::IntrinsicBase* intrinsic, const oiio::ROI& rod)
{
    const int xOffset = rod.xbegin;
    const int yOffset = rod.ybegin; // (intrinsic->h() - rod.yend);
    const oiio::ROI roi(-xOffset, intrinsic->w() - xOffset,
                        -yOffset, intrinsic->h() - yOffset);

    ALICEVISION_LOG_DEBUG("roi:" << roi.xbegin << ";" << roi.xend << ";" << roi.ybegin << ";" << roi.yend);
    return roi;
}

int aliceVision_main(int argc, char** argv)
{
  // command-line parameters

  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  std::string sfmDataFilename;
  std::string outFolder;

  // user optional parameters
  bool undistortedImages = false;
  bool exportUVMaps = false;
  bool exportFullROD = false;
  bool correctPrincipalPoint = true;
  std::map<IndexT, oiio::ROI> roiForIntrinsic;
  std::string viewFilter;
  std::string sfmDataFilterFilepath;
  std::string outImageFileTypeName = image::EImageFileType_enumToString(image::EImageFileType::JPEG);
  std::string outMapFileTypeName = image::EImageFileType_enumToString(image::EImageFileType::EXR);

  po::options_description allParams("AliceVision exportAnimatedCamera");

  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
      "SfMData file containing a complete SfM.")
    ("output,o", po::value<std::string>(&outFolder)->required(),
      "Output folder.");

  po::options_description optionalParams("Optional parameters");
  optionalParams.add_options()
    ("exportUndistortedImages", po::value<bool>(&undistortedImages)->default_value(undistortedImages),
      "Export undistorted images for the animated camera(s).\n"
      "If false, animated camera(s) exported with original frame paths.")
    ("exportFullROD", po::value<bool>(&exportFullROD)->default_value(exportFullROD),
      "Export undistorted images with the full Region of Definition (RoD). Only supported by the EXR image file format.")
    ("exportUVMaps", po::value<bool>(&exportUVMaps)->default_value(exportUVMaps),
      "Export UV Maps in exr format to apply distort/undistort transformations in a compositing software.")
    ("correctPrincipalPoint", po::value<bool>(&correctPrincipalPoint)->default_value(correctPrincipalPoint),
      "apply an offset to correct the position of the principal point")
    ("viewFilter", po::value<std::string>(&viewFilter)->default_value(viewFilter),
      "Select the cameras to export using an expression based on the image filepath. Export all cameras if empty.")
    ("sfmDataFilter", po::value<std::string>(&sfmDataFilterFilepath)->default_value(sfmDataFilterFilepath),
      "Filter out cameras from the export if they are part of this SfMData. Export all cameras if empty.")
    ("undistortedImageType", po::value<std::string>(&outImageFileTypeName)->default_value(outImageFileTypeName),
      image::EImageFileType_informations().c_str());

  po::options_description logParams("Log parameters");
  logParams.add_options()
    ("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
      "verbosity level (fatal,  error, warning, info, debug, trace).");

  allParams.add(requiredParams).add(optionalParams).add(logParams);
  ALICEVISION_LOG_DEBUG("UVmap: " + std::to_string(exportUVMaps));

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

  // set output file type
  const image::EImageFileType outputFileType = image::EImageFileType_stringToEnum(outImageFileTypeName);
  const image::EImageFileType outputMapFileType = image::EImageFileType_stringToEnum(outMapFileTypeName);

  // set verbose level
  system::Logger::get()->setLogLevel(verboseLevel);

  if(exportFullROD && outputFileType != image::EImageFileType::EXR)
  {
    ALICEVISION_LOG_ERROR("Export full RoD (Region Of Definition) is only possible in EXR file format and not in '" << outputFileType << "'.");
    return EXIT_FAILURE;
  }

  // load SfMData files
  sfmData::SfMData sfmData;
  if(!sfmDataIO::Load(sfmData, sfmDataFilename, sfmDataIO::ESfMData::ALL))
  {
    ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFilename << "' cannot be read.");
    return EXIT_FAILURE;
  }

  if(sfmData.getViews().empty())
  {
    ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFilename << "' is empty.");
    return EXIT_FAILURE;
  }

  sfmData::SfMData sfmDataFilter;
  if(!sfmDataFilterFilepath.empty())
  {
      if(!sfmDataIO::Load(sfmDataFilter, sfmDataFilterFilepath, sfmDataIO::ESfMData::VIEWS))
      {
        ALICEVISION_LOG_ERROR("The input filter SfMData file '" << sfmDataFilterFilepath << "' cannot be read.");
        return EXIT_FAILURE;
      }
  }
  system::Timer timer;

  // Decide the views and instrinsics to export
  sfmData::SfMData sfmDataExport;
  for(auto& viewPair : sfmData.getViews())
  {
    sfmData::View& view = *(viewPair.second);

    // regex filter
    if(!viewFilter.empty())
    {
        // Skip the view if it does not match the expression filter
        const std::regex regexFilter = utils::filterToRegex(viewFilter);
        if(!std::regex_match(view.getImagePath(), regexFilter))
            continue;
    }

    // sfmData filter
    if(!sfmDataFilterFilepath.empty())
    {
        // Skip the view if it exist in the sfmDataFilter
        if(sfmDataFilter.getViews().find(view.getViewId()) != sfmDataFilter.getViews().end())
            continue;
    }

    sfmDataExport.getViews().emplace(view.getViewId(), viewPair.second);

    // Export intrinsics with at least one view with a valid pose
    if(sfmData.isPoseAndIntrinsicDefined(&view))
    {
        // std::map::emplace does nothing if the key already exist
        sfmDataExport.getIntrinsics().emplace(view.getIntrinsicId(), sfmData.getIntrinsics().at(view.getIntrinsicId()));
    }
  }

  const fs::path undistortedImagesFolderPath = fs::path(outFolder) / "undistort";
  const bool writeUndistordedResult = undistortedImages || exportUVMaps;

  if(writeUndistordedResult && !fs::exists(undistortedImagesFolderPath))
    fs::create_directory(undistortedImagesFolderPath);

  std::map<std::string, std::map<std::size_t, IndexT>> videoViewPerFrame;
  std::map<std::string, std::vector<std::pair<std::size_t, IndexT>> > dslrViewPerKey;

  // export distortion map / one image per intrinsic
  if(exportUVMaps)
  {
      for(const auto& intrinsicPair : sfmDataExport.getIntrinsics())
      {
          const camera::IntrinsicBase& intrinsic = *(intrinsicPair.second);
          image::Image<image::RGBfColor> image_dist;
          // Init image as black (no distortion)
          image_dist.resize(int(intrinsic.w()), int(intrinsic.h()), true, image::FBLACK);

          // Compute UV vertors for distortion
          const Vec2 center(intrinsic.w() * 0.5, intrinsic.h() * 0.5);
          Vec2 ppCorrection(0.0, 0.0);

          if((camera::EINTRINSIC::VALID_PINHOLE & intrinsic.getType()) && correctPrincipalPoint)// correct principal point
          {
              const camera::Pinhole* pinholePtr = dynamic_cast<const camera::Pinhole*>(intrinsicPair.second.get());
              ppCorrection = pinholePtr->getPrincipalPoint() - center;
          }
          ALICEVISION_LOG_DEBUG("ppCorrection:" + std::to_string(ppCorrection[0]) + ";" +std::to_string(ppCorrection[1]));

          // UV Map: Undistort
          {
              // flip and normalize as UVMap
    #pragma omp parallel for
              for(int y = 0; y < int(intrinsic.h()); ++y)
              {
                  for(int x = 0; x < int(intrinsic.w()); ++x)
                  {
                      const Vec2 undisto_pix(x, y);
                      // compute coordinates with distortion
                      const Vec2 disto_pix = intrinsic.get_d_pixel(undisto_pix) + ppCorrection;

                      image_dist(y, x).r() = float((disto_pix[0]) / (intrinsic.w() - 1));
                      image_dist(y, x).g() = float((intrinsic.h() - 1 - disto_pix[1]) / (intrinsic.h() - 1));
                  }
              }

              const std::string dstImage =
                  (undistortedImagesFolderPath / (std::to_string(intrinsicPair.first) + "_UVMap_Undistort." +
                                                  image::EImageFileType_enumToString(outputMapFileType))).string();
              image::writeImage(dstImage, image_dist, image::EImageColorSpace::AUTO);
          }

          // UV Map: Distort
          {
              // flip and normalize as UVMap
    #pragma omp parallel for
              for(int y = 0; y < int(intrinsic.h()); ++y)
              {
                  for(int x = 0; x < int(intrinsic.w()); ++x)
                  {
                      const Vec2 disto_pix(x, y);
                      // compute coordinates without distortion
                      const Vec2 undisto_pix = intrinsic.get_ud_pixel(disto_pix) - ppCorrection;

                      image_dist(y, x).r() = float((undisto_pix[0]) / (intrinsic.w() - 1));
                      image_dist(y, x).g() = float((intrinsic.h() - 1 - undisto_pix[1]) / (intrinsic.h() - 1));
                  }
              }

              const std::string dstImage =
                  (undistortedImagesFolderPath / (std::to_string(intrinsicPair.first) + "_UVMap_Distort." +
                                                  image::EImageFileType_enumToString(outputMapFileType))).string();
              image::writeImage(dstImage, image_dist, image::EImageColorSpace::AUTO);
          }
      }
  }

  ALICEVISION_LOG_INFO("Build animated camera(s)...");

  image::Image<image::RGBfColor> image, image_ud;
  boost::progress_display progressBar(sfmDataExport.getViews().size());

  for(const auto& viewPair : sfmDataExport.getViews())
  {
    const sfmData::View& view = *(viewPair.second);

    ++progressBar;

    const std::string imagePathStem = fs::path(viewPair.second->getImagePath()).stem().string();

    // undistort camera images
    if(undistortedImages)
    {
      sfmData::Intrinsics::const_iterator iterIntrinsic = sfmDataExport.getIntrinsics().find(view.getIntrinsicId());
      const std::string dstImage = (undistortedImagesFolderPath / (std::to_string(view.getIntrinsicId()) + "_" + imagePathStem + "." + image::EImageFileType_enumToString(outputFileType))).string();
      const camera::IntrinsicBase * cam = iterIntrinsic->second.get();

      image::readImage(view.getImagePath(), image, image::EImageColorSpace::LINEAR);
      oiio::ParamValueList metadata = image::readImageMetadata(view.getImagePath());

      if(cam->isValid() && cam->hasDistortion())
      {
        // undistort the image and save it
        if(exportFullROD)
        {
            // build a ROI
            const IndexT key = view.getIntrinsicId();
            oiio::ROI rod;
            const camera::IntrinsicBase &intrinsic = (*cam);
            if(roiForIntrinsic.find(key) == roiForIntrinsic.end())
            {
                rod = computeRod(cam, correctPrincipalPoint);
                roiForIntrinsic[key] = rod;
            }
            else
            {
                rod = roiForIntrinsic[key];
            }

            ALICEVISION_LOG_DEBUG("rod:" + std::to_string(rod.xbegin) + ";" + std::to_string(rod.xend) + ";" +
                                  std::to_string(rod.ybegin) + ";" + std::to_string(rod.yend));
            camera::UndistortImage(image, cam, image_ud, image::FBLACK, correctPrincipalPoint, rod);
            const oiio::ROI roi = convertRodToRoi(cam, rod);
            writeImage(dstImage, image_ud, image::EImageColorSpace::AUTO, oiio::ParamValueList(), roi);
        }
        else
        {
            camera::UndistortImage(image, cam, image_ud, image::FBLACK, correctPrincipalPoint);
            image::writeImage(dstImage, image_ud, image::EImageColorSpace::AUTO, metadata);
        }
      }
      else // (no distortion)
      {
        // copy the image since there is no distortion
        image::writeImage(dstImage, image, image::EImageColorSpace::AUTO, metadata);
      }
    }

    // pose and intrinsic defined
    // Note: we use "sfmData" and not "sfmDataExport" to have access to poses
    if(!sfmData.isPoseAndIntrinsicDefined(&view))
      continue;

    std::string cameraName =  view.getMetadataMake() + "_" + view.getMetadataModel();
    IndexT frameN = 0;
    bool isSequence = false;

    if(view.isPartOfRig())
      cameraName += std::string("_") + std::to_string(view.getSubPoseId());

    {
      std::string prefix;
      std::string suffix;

      if(sfmDataIO::extractNumberFromFileStem(imagePathStem, frameN, prefix, suffix))
      {
        if(prefix.empty() && suffix.empty())
            cameraName = std::string("Undefined") + "_" + cameraName;
        else
            cameraName = prefix + "frame" + suffix + "_" + cameraName;

        isSequence = true;
      }
    }

    ALICEVISION_LOG_TRACE("imagePathStem: " << imagePathStem << ", frameN: " << frameN << ", isSequence: " << isSequence << ", cameraName: " << cameraName);

    if(isSequence) // video
    {
        const std::size_t frame = frameN;
        videoViewPerFrame[cameraName][frame] = view.getViewId();
    }
    else if(view.hasMetadataDateTimeOriginal()) // picture
    {
        const std::size_t key = view.getMetadataDateTimestamp();

        dslrViewPerKey[cameraName].push_back({key, view.getViewId()});
    }
    else // no time or sequence information
    {
        dslrViewPerKey[cameraName].push_back({0, view.getViewId()});
    }
  }

  // print results
  {
    std::stringstream ss;

    ss << "Camera(s) found:" << std::endl << "\t- # video camera(s): " << videoViewPerFrame.size() << std::endl;

    for(const auto& camera : videoViewPerFrame)
      ss << "\t    - " << camera.first << " | " << camera.second.size() << " frame(s)" << std::endl;

    ss << "\t- # dslr camera(s): " << dslrViewPerKey.size() << std::endl;

    for(const auto& camera : dslrViewPerKey)
      ss << "\t    - " << camera.first << " | " << camera.second.size() << " image(s)" << std::endl;

    ss << "\t- # Used camera intrinsics: " << sfmDataExport.getIntrinsics().size() << std::endl;

    for(const auto& intrinsicIt : sfmDataExport.getIntrinsics())
    {
      const auto intrinsic = intrinsicIt.second;
      ss << "\t    - "
         << intrinsicIt.first << " | "
         << intrinsic->w() << "x" << intrinsic->h()
         << " " << intrinsic->serialNumber()
         << std::endl;
    }

    ALICEVISION_LOG_INFO(ss.str());
  }

  ALICEVISION_LOG_INFO("Export animated camera(s)...");

  sfmDataIO::AlembicExporter exporter((fs::path(outFolder) / "camera.abc").string());

  for(const auto& cameraViews : videoViewPerFrame)
  {
    const std::map<std::size_t, IndexT>& frameToView = cameraViews.second;
    const std::size_t firstFrame = cameraViews.second.begin()->first;

    exporter.initAnimatedCamera(cameraViews.first, firstFrame);

    for(std::size_t frame = firstFrame; frame <= frameToView.rbegin()->first; ++frame)
    {
      const auto findFrameIt = frameToView.find(frame);

      if(findFrameIt != frameToView.end())
      {
        const IndexT viewId = findFrameIt->second;

        // Note: we use "sfmData" and not "sfmDataExport" to have access to poses

        const auto findViewIt = sfmData.getViews().find(viewId);
        assert(findViewIt != sfmData.getViews().end());

        ALICEVISION_LOG_DEBUG("[" + cameraViews.first +"][video] Keyframe added");
        const IndexT intrinsicId = findViewIt->second->getIntrinsicId();
        const camera::Pinhole* cam = dynamic_cast<camera::Pinhole*>(sfmData.getIntrinsicPtr(intrinsicId));
        const sfmData::CameraPose pose = sfmData.getPose(*findViewIt->second);
        const std::string& imagePath = findViewIt->second->getImagePath();
        const std::string undistortedImagePath = (undistortedImagesFolderPath / (std::to_string(intrinsicId) + "_" + fs::path(imagePath).stem().string() + "." + image::EImageFileType_enumToString(outputFileType))).string();

        exporter.addCameraKeyframe(pose.getTransform(), cam, (undistortedImages) ? undistortedImagePath : imagePath, viewId, intrinsicId);
      }
      else
      {
        exporter.jumpKeyframe(std::to_string(frame));
      }
    }
  }

  for(auto& cameraViews : dslrViewPerKey)
  {
    exporter.initAnimatedCamera(cameraViews.first);
    std::sort(cameraViews.second.begin(), cameraViews.second.end());

    for(const auto& cameraView : cameraViews.second)
    {
        ALICEVISION_LOG_DEBUG("[" + cameraViews.first +"][dslr] Keyframe added");
        const sfmData::View& view = *(sfmData.getViews().at(cameraView.second));
        const camera::Pinhole* cam = dynamic_cast<camera::Pinhole*>(sfmData.getIntrinsicPtr(view.getIntrinsicId()));
        const sfmData::CameraPose pose = sfmData.getPose(view);
        const std::string& imagePath = view.getImagePath();
        const std::string undistortedImagePath = (undistortedImagesFolderPath / (std::to_string(view.getIntrinsicId()) + "_" + fs::path(imagePath).stem().string() + "." + image::EImageFileType_enumToString(outputFileType))).string();

        exporter.addCameraKeyframe(pose.getTransform(), cam, (undistortedImages) ? undistortedImagePath : imagePath, view.getViewId(), view.getIntrinsicId());
    }
  }

  ALICEVISION_LOG_INFO("Task done in (s): " + std::to_string(timer.elapsed()));
  return EXIT_SUCCESS;
}
