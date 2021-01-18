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


void writeImageNew(const std::string& path, image::Image<image::RGBfColor>& imageIn,
                   image::EImageColorSpace imageColorSpace, const oiio::ParamValueList& metadata, const oiio::ROI& roi)
{

    oiio::ImageSpec imageSpec(imageIn.Width(), imageIn.Height(), 3, oiio::TypeDesc::FLOAT);
    imageSpec.extra_attribs = metadata; // add custom metadata
    imageSpec.set_roi_full(roi);
    const oiio::ImageBuf imgBuf = oiio::ImageBuf(imageSpec, imageIn.data()); // original image buffer
    imgBuf.write(path);
}

int aliceVision_main(int argc, char** argv)
{
  // command-line parameters

  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  std::string sfmDataFilename;
  std::string outFolder;
  std::string outImageFileTypeName = image::EImageFileType_enumToString(image::EImageFileType::JPEG);
  std::string outMapFileTypeName = image::EImageFileType_enumToString(image::EImageFileType::EXR);
  bool undistortedImages = false;
  bool exportUVMaps = false;
  std::map<std::string, oiio::ROI> roiForIntrinsic;

  // user optional parameters

  std::string viewFilter;
  

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
    ("exportUVMaps", po::value<bool>(&exportUVMaps)->default_value(exportUVMaps),
      "Export UV Maps for Nuke in exr format ")
    ("viewFilter", po::value<std::string>(&viewFilter)->default_value(viewFilter),
      "Path to the output SfMData file (with only views and poses).")
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

  // set verbose level
  system::Logger::get()->setLogLevel(verboseLevel);

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
  system::Timer timer;

  // set output file type
  image::EImageFileType outputFileType = image::EImageFileType_stringToEnum(outImageFileTypeName);
  image::EImageFileType outputMapFileType = image::EImageFileType_stringToEnum(outMapFileTypeName);

  const fs::path undistortedImagesFolderPath = fs::path(outFolder) / "undistort";
  bool writeUndistordedResult = undistortedImages || exportUVMaps;

  if(writeUndistordedResult && !fs::exists(undistortedImagesFolderPath))
    fs::create_directory(undistortedImagesFolderPath);

  std::map<std::string, std::map<std::size_t, IndexT>> videoViewPerFrame;
  std::map<std::string, std::vector<std::pair<std::size_t, IndexT>> > dslrViewPerKey;

  // export distortion map / one image per intrinsic
  if(exportUVMaps)
  {
      for(const auto& intrinsicPair : sfmData.getIntrinsics())
      {
          const camera::IntrinsicBase& intrinsic = *(intrinsicPair.second);
          image::Image<image::RGBfColor> image_dist;
          image_dist.resize(int(intrinsic.w()), int(intrinsic.h()), true, image::FBLACK);

          if(!intrinsic.hasDistortion()) // no distortion, perform a direct copy
          {
              // nothing to do image is black
          }
          else // There is distortion
          {
              const Vec2 center(intrinsic.w() * 0.5, intrinsic.h() * 0.5);
              Vec2 ppCorrection(0.0, 0.0);

              if(camera::EINTRINSIC::VALID_PINHOLE & intrinsic.getType())
              {
                  const camera::Pinhole* pinholePtr = dynamic_cast<const camera::Pinhole*>(intrinsicPair.second.get());
                  ppCorrection = pinholePtr->getPrincipalPoint() - center;
              }

              // flip and normalize for Nuke
#pragma omp parallel for
              for(int j = 0; j < int(intrinsic.h()); ++j)
                  for(int i = 0; i < int(intrinsic.w()); ++i)
                  {
                      const Vec2 undisto_pix(i, j);
                      // compute coordinates with distortion
                      const Vec2 disto_pix = intrinsic.get_d_pixel(undisto_pix) + ppCorrection;

                      if(disto_pix[0] >= 0 && disto_pix[0] < intrinsic.w() && disto_pix[1] >= 0 &&
                         disto_pix[1] < intrinsic.h())
                      {
                          image_dist(j, i).r() = (disto_pix[0]) / intrinsic.w();
                          image_dist(j, i).g() = (intrinsic.h() - disto_pix[1]) / intrinsic.h();
                      }
                      else
                      {
                          image_dist(j, i).r() = -1;
                          image_dist(j, i).g() = -1;
                      }
                  }

              if(exportUVMaps)
              {
                  const std::string dstImage =
                      (undistortedImagesFolderPath / ("Distortion_UVMap_" + std::to_string(intrinsicPair.first) + "." +
                                                      image::EImageFileType_enumToString(outputMapFileType))).string();
                  image::writeImage(dstImage, image_dist, image::EImageColorSpace::NO_CONVERSION);
              }
          }
      }
  }
  
  
      

  ALICEVISION_LOG_INFO("Build animated camera(s)...");

  image::Image<image::RGBfColor> image, image_ud;
  boost::progress_display progressBar(sfmData.getViews().size());

  for(const auto& viewPair : sfmData.getViews())
  {
    const sfmData::View& view = *(viewPair.second);

    ++progressBar;

    // regex filter
    if(!viewFilter.empty())
    {
        const std::regex regexFilter = utils::filterToRegex(viewFilter);
        if(!std::regex_match(view.getImagePath(), regexFilter))
            continue;
    }

    const std::string imagePathStem = fs::path(viewPair.second->getImagePath()).stem().string();

    // undistort camera images
    if(undistortedImages)
    {
      sfmData::Intrinsics::const_iterator iterIntrinsic = sfmData.getIntrinsics().find(view.getIntrinsicId());
      const std::string dstImage = (undistortedImagesFolderPath / (std::to_string(view.getIntrinsicId()) + "_" + imagePathStem + "." + image::EImageFileType_enumToString(outputFileType))).string();
      const camera::IntrinsicBase * cam = iterIntrinsic->second.get();

      image::readImage(view.getImagePath(), image, image::EImageColorSpace::NO_CONVERSION);
      oiio::ParamValueList metadata = image::readImageMetadata(view.getImagePath());

      if(cam->isValid() && cam->hasDistortion())
      {
        // undistort the image and save it
        camera::UndistortImage(image, cam, image_ud, image::FBLACK, true); // correct principal point
        if(outputFileType == image::EImageFileType::EXR)
        {
            // build a ROI
            std::string key = std::to_string(view.getIntrinsicId());
            oiio::ROI roi;
            if(roiForIntrinsic.find(key) == roiForIntrinsic.end())
            {
                const camera::IntrinsicBase& intrinsic = *(cam);
                int xRoiMin = intrinsic.w() - 1;
                int xRoiMax = 0;
                int yRoiMin = intrinsic.h() - 1;
                int yRoiMax = 0;
#pragma omp parallel for
                for(int j = 0; j < int(intrinsic.h()); ++j)
                    for(int i = 0; i < int(intrinsic.w()); ++i)
                    {
                        // for ROD
                        const Vec2 disto_pix_in(i, j);
                        // compute coordinates
                        const Vec2 undisto_pix = intrinsic.get_ud_pixel(disto_pix_in);
                        if(undisto_pix[0] < xRoiMin)
                            xRoiMin = int(undisto_pix[0]);
                        else
                        {
                            if(undisto_pix[0] > xRoiMax)
                                xRoiMax = int(undisto_pix[0]);
                        }
                        if(undisto_pix[1] < yRoiMin)
                            yRoiMin = int(undisto_pix[1]);
                        else
                        {
                            if(undisto_pix[1] > yRoiMax)
                                yRoiMax = int(undisto_pix[1]);
                        }
                    }

                oiio::ROI roi(xRoiMin, xRoiMax, yRoiMin, yRoiMax); // roi for current intrinsic,add it to map
                ALICEVISION_LOG_DEBUG("roi:" + std::to_string(xRoiMin) + ";" + std::to_string(xRoiMax) + ";" +
                                      std::to_string(yRoiMin) + ";" + std::to_string(yRoiMax));
                roiForIntrinsic[key] = roi;
            }
            else
            {
                roi = roiForIntrinsic[key];
            }
            writeImageNew(dstImage, image_ud, image::EImageColorSpace::NO_CONVERSION, metadata, roi);
        }
        else
        {
            //image::writeImage(dstImage, image_ud, image::EImageColorSpace::NO_CONVERSION);
            image::writeImage(dstImage, image_ud, image::EImageColorSpace::NO_CONVERSION, metadata); // To be checked: use of metadata
        }   
      }
      else // (no distortion)
      {
        // copy the image since there is no distortion
        //image::writeImage(dstImage, image, image::EImageColorSpace::NO_CONVERSION);
        image::writeImage(dstImage, image, image::EImageColorSpace::NO_CONVERSION, metadata);
      }
    }

    // pose and intrinsic defined
    if(!sfmData.isPoseAndIntrinsicDefined(&view))
      continue;

    std::string cameraName =  view.getMetadataMake() + "_" + view.getMetadataModel();
    std::size_t frameN = 0;
    bool isSequence = false;

    if(view.isPartOfRig())
      cameraName += std::string("_") + std::to_string(view.getSubPoseId());

    // check if the image is in a sequence
    // regexFrame: ^(.*\D)?([0-9]+)([\-_\.].*[[:alpha:]].*)?$
    std::regex regexFrame("^(.*\\D)?"    // the optional prefix which end with a non digit character
                      "([0-9]+)"         // the sequence frame number
                      "([\\-_\\.]"       // the suffix start with a separator
                      ".*[[:alpha:]].*"  // at least one letter in the suffix
                      ")?$"              // suffix is optional
                      );

    std::smatch matches;
    if(std::regex_search(imagePathStem, matches, regexFrame))
    {
       const std::string prefix = matches[1];
       const std::string suffix = matches[3];
       frameN = std::stoi(matches[2]);

       if(prefix.empty() && suffix.empty())
         cameraName = std::string("Undefined") + "_" + cameraName;
       else
         cameraName = prefix + "frame" + suffix + "_" + cameraName;

       isSequence = true;
    }

    if(view.hasMetadataDateTimeOriginal()) // picture
    {
        const std::size_t key = view.getMetadataDateTimestamp();

      dslrViewPerKey[cameraName].push_back({key, view.getViewId()});
    }
    else if(isSequence) // video
    {
        const std::size_t frame = frameN;
        videoViewPerFrame[cameraName][frame] = view.getViewId();
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

        const auto findViewIt = sfmData.getViews().find(viewId);
        if(findViewIt != sfmData.getViews().end())
        {
            ALICEVISION_LOG_DEBUG("[" + cameraViews.first +"][video] Keyframe added");
            const IndexT intrinsicId = findViewIt->second->getIntrinsicId();
            const camera::Pinhole* cam = dynamic_cast<camera::Pinhole*>(sfmData.getIntrinsicPtr(intrinsicId));
            const sfmData::CameraPose pose = sfmData.getPose(*findViewIt->second);
            const std::string& imagePath = findViewIt->second->getImagePath();
            const std::string undistortedImagePath = (undistortedImagesFolderPath / (std::to_string(intrinsicId) + "_" + fs::path(imagePath).stem().string() + "." + image::EImageFileType_enumToString(outputFileType))).string();

            exporter.addCameraKeyframe(pose.getTransform(), cam, (undistortedImages) ? undistortedImagePath : imagePath, viewId, intrinsicId);
            continue;
        }
      }
      exporter.jumpKeyframe(std::to_string(frame));
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
