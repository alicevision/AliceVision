// This file is part of the AliceVision project.
// Copyright (c) 2018 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/sfmDataIO/AlembicExporter.hpp>
#include <aliceVision/image/all.hpp>

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

int main(int argc, char** argv)
{
  // command-line parameters

  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  std::string sfmDataFilename;
  std::string outFolder;
  std::string outImageFileTypeName = image::EImageFileType_enumToString(image::EImageFileType::JPEG);
  bool undistortedImages = true;

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
    ("viewFilter", po::value<std::string>(&viewFilter)->default_value(viewFilter),
      "Path to the output SfMData file (with only views and poses).")
    ("undistortedImageType", po::value<std::string>(&outImageFileTypeName)->default_value(outImageFileTypeName),
      image::EImageFileType_informations().c_str());

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
  std::regex regexFilter;

  if(!viewFilter.empty())
  {
    std::string filterToRegex = viewFilter;
    filterToRegex = std::regex_replace(filterToRegex, std::regex("\\*"), std::string("(.*)"));
    filterToRegex = std::regex_replace(filterToRegex, std::regex("\\?"), std::string("(.)"));
    filterToRegex = std::regex_replace(filterToRegex, std::regex("\\@"), std::string("[0-9]+")); // one @ correspond to one or more digits
    filterToRegex = std::regex_replace(filterToRegex, std::regex("\\#"), std::string("[0-9]"));  // each # in pattern correspond to a digit
    regexFilter = std::regex(filterToRegex);
  }

  // set output file type
  image::EImageFileType outputFileType = image::EImageFileType_stringToEnum(outImageFileTypeName);

  const fs::path undistortedImagesFolderPath = fs::path(outFolder) / "undistort";

  if(undistortedImages && !fs::exists(undistortedImagesFolderPath))
    fs::create_directory(undistortedImagesFolderPath);

  std::map<std::string, std::map<std::size_t, IndexT>> videoViewPerFrame;
  std::map<std::string, std::vector<std::pair<std::size_t, IndexT>> > dslrViewPerKey;

  ALICEVISION_LOG_INFO("Build animated camera(s)...");

  image::Image<image::RGBfColor> image, image_ud;
  boost::progress_display progressBar(sfmData.getViews().size());

  for(const auto& viewPair : sfmData.getViews())
  {
    const sfmData::View& view = *(viewPair.second);

    ++progressBar;

    // regex filter
    if(!viewFilter.empty() &&
       !std::regex_match(view.getImagePath(), regexFilter))
      continue;

    const std::string imagePathStem = fs::path(viewPair.second->getImagePath()).stem().string();

    // undistort camera images
    if(undistortedImages)
    {
      sfmData::Intrinsics::const_iterator iterIntrinsic = sfmData.getIntrinsics().find(view.getIntrinsicId());
      const std::string dstImage = (undistortedImagesFolderPath / (std::to_string(view.getIntrinsicId()) + "_" + imagePathStem + "." + image::EImageFileType_enumToString(outputFileType))).string();
      const camera::IntrinsicBase * cam = iterIntrinsic->second.get();

      image::readImage(view.getImagePath(), image, image::EImageColorSpace::LINEAR);

      if(cam->isValid() && cam->have_disto())
      {
        // undistort the image and save it
        camera::UndistortImage(image, cam, image_ud, image::FBLACK, true); // correct principal point
        image::writeImage(dstImage, image_ud, image::EImageColorSpace::LINEAR);
      }
      else // (no distortion)
      {
        // copy the image since there is no distortion
        image::writeImage(dstImage, image, image::EImageColorSpace::LINEAR);
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

    std::string dateTimeMetadata = view.getMetadataOrEmpty("Exif:DateTimeOriginal");

    if(!dateTimeMetadata.empty()) // picture
    {
      dateTimeMetadata.erase(std::remove_if(dateTimeMetadata.begin(),dateTimeMetadata.end(), ::isspace), dateTimeMetadata.end());
      dateTimeMetadata.erase(std::remove_if(dateTimeMetadata.begin(),dateTimeMetadata.end(), ::ispunct), dateTimeMetadata.end());

      std::size_t key = std::numeric_limits<unsigned char>::max();

      if(!dateTimeMetadata.empty())
          key = std::stoul(dateTimeMetadata);

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
