// This file is part of the AliceVision project.
// Copyright (c) 2015 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/jsonIO.hpp>
#include <aliceVision/sfmDataIO/viewIO.hpp>
#include <aliceVision/sensorDB/parseDatabase.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/image/io.cpp>

#include <boost/atomic/atomic_ref.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/regex.hpp>

#include <iostream>
#include <fstream>
#include <sstream>
#include <memory>
#include <string>
#include <vector>
#include <cstdlib>
#include <stdexcept>


// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 2
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;
using namespace aliceVision::sfmDataIO;

namespace po = boost::program_options;
namespace fs = boost::filesystem;


/**
 * @brief Recursively list all files from a folder with a specific extension
 * @param[in] folderOrFile A file or foder path
 * @param[in] extensions An extensions filter
 * @param[out] outFiles A list of output image paths
 * @return true if folderOrFile have been load successfully
 */
bool listFiles(const std::string& folderOrFile,
               const std::vector<std::string>& extensions,
               std::vector<std::string>& resources)
{
  if(fs::is_regular_file(folderOrFile))
  {
    std::string fileExtension = fs::extension(folderOrFile);
    std::transform(fileExtension.begin(), fileExtension.end(), fileExtension.begin(), ::tolower);
    for(const std::string& extension: extensions)
    {
      if(fileExtension == extension)
      {
        resources.push_back(folderOrFile);
        return true;
      }
    }
  }
  else if(fs::is_directory(folderOrFile))
  {
    // list all files of the folder
    std::vector<std::string> allFiles;

    fs::directory_iterator endItr;
    for(fs::directory_iterator itr(folderOrFile); itr != endItr; ++itr)
      allFiles.push_back(itr->path().string());

    bool hasFile = false;
    for(const std::string& filePath: allFiles)
    {
      if(listFiles(filePath, extensions, resources))
        hasFile = true;
    }
    return hasFile;
  }
  ALICEVISION_LOG_ERROR("'" << folderOrFile << "' is not a valid folder or file path.");
  return false;
}

/**
 * @brief Create the description of an input image dataset for AliceVision toolsuite
 * - Export a SfMData file with View & Intrinsic data
 */
int aliceVision_main(int argc, char **argv)
{
  // command-line parameters

  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  std::string sfmFilePath;
  std::string imageFolder;
  std::string sensorDatabasePath;
  std::string outputFilePath;

  // user optional parameters
  std::string defaultCameraModelName;
  std::string allowedCameraModelsStr = "pinhole,radial1,radial3,brown,fisheye4,fisheye1";

  double defaultFocalLength = -1.0;
  double defaultFieldOfView = -1.0;
  double defaultFocalRatio = 1.0;
  double defaultOffsetX = 0.0;
  double defaultOffsetY = 0.0;

  EGroupCameraFallback groupCameraFallback = EGroupCameraFallback::FOLDER;
  EViewIdMethod viewIdMethod = EViewIdMethod::METADATA;
  std::string viewIdRegex = ".*?(\\d+)";

  bool allowSingleView = false;
  bool useInternalWhiteBalance = true;

  po::options_description allParams("AliceVision cameraInit");

  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmFilePath)->default_value(sfmFilePath),
      "A SfMData file (*.sfm) [if specified, --imageFolder cannot be used].")
    ("imageFolder", po::value<std::string>(&imageFolder)->default_value(imageFolder),
      "Input images folder [if specified, --input cannot be used].")
    ("output,o", po::value<std::string>(&outputFilePath)->default_value("cameraInit.sfm"),
      "Output file path for the new SfMData file");

  po::options_description optionalParams("Optional parameters");
  optionalParams.add_options()
    ("sensorDatabase,s", po::value<std::string>(&sensorDatabasePath)->default_value(""),
      "Camera sensor width database path.")
    ("defaultFocalLength", po::value<double>(&defaultFocalLength)->default_value(defaultFocalLength),
      "Focal length in mm. (or '-1' to unset)")
    ("defaultFieldOfView", po::value<double>(&defaultFieldOfView)->default_value(defaultFieldOfView),
      "Empirical value for the field of view in degree. (or '-1' to unset)")
    ("defaultFocalRatio", po::value<double>(&defaultFocalRatio)->default_value(defaultFocalRatio),
      "Ratio between the pixel X size on the sensor and the Y size.")
    ("defaultOffsetX", po::value<double>(&defaultOffsetX)->default_value(defaultOffsetX),
      "default offset from the principal point X coordinate")
    ("defaultOffsetY", po::value<double>(&defaultOffsetY)->default_value(defaultOffsetY),
      "default offset from the principal point Y coordinate")
    ("defaultCameraModel", po::value<std::string>(&defaultCameraModelName)->default_value(defaultCameraModelName),
      "Default camera model type (pinhole, radial1, radial3, brown, fisheye4, fisheye1).")
    ("allowedCameraModels", po::value<std::string>(&allowedCameraModelsStr)->default_value(allowedCameraModelsStr),
      "Permitted model type (pinhole, radial1, radial3, brown, fisheye4, fisheye1).")
    ("groupCameraFallback", po::value<EGroupCameraFallback>(&groupCameraFallback)->default_value(groupCameraFallback),
      std::string("When there is no serial number in the image metadata, we cannot know if the images come from the same camera. "
      "This is problematic for grouping images sharing the same internal camera settings and we have to decide on a fallback strategy:\n"
      " * " + EGroupCameraFallback_enumToString(EGroupCameraFallback::GLOBAL) + ": all images may come from a single device (make/model/focal will still be a differentiator).\n"
      " * " + EGroupCameraFallback_enumToString(EGroupCameraFallback::FOLDER) + ": different folders will be considered as different devices\n"
      " * " + EGroupCameraFallback_enumToString(EGroupCameraFallback::IMAGE) + ": consider that each image has different internal camera parameters").c_str())
    ("viewIdMethod", po::value<EViewIdMethod>(&viewIdMethod)->default_value(viewIdMethod),
      std::string("Allows to choose the way the viewID is generated:\n"
      " * " + EViewIdMethod_enumToString(EViewIdMethod::METADATA) + ": Generate viewId from image metadata.\n"
      " * " + EViewIdMethod_enumToString(EViewIdMethod::FILENAME) + ": Generate viewId from file names using regex.") .c_str())
    ("viewIdRegex", po::value<std::string>(&viewIdRegex)->default_value(viewIdRegex),
      "Regex used to catch number used as viewId in filename.")
    ("useInternalWhiteBalance", po::value<bool>(&useInternalWhiteBalance)->default_value(useInternalWhiteBalance),
      "Apply the white balance included in the image metadata (Only for raw images)")
    ("allowSingleView", po::value<bool>(&allowSingleView)->default_value(allowSingleView),
      "Allow the program to process a single view.\n"
      "Warning: if a single view is process, the output file can't be use in many other programs.");

  po::options_description logParams("Log parameters");
  logParams.add_options()
    ("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
      "verbosity level (fatal, error, warning, info, debug, trace).");

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

  // set user camera model
  camera::EINTRINSIC defaultCameraModel = camera::EINTRINSIC::UNKNOWN;
  if(!defaultCameraModelName.empty())
      defaultCameraModel = camera::EINTRINSIC_stringToEnum(defaultCameraModelName);

  // check user choose at least one input option
  if(imageFolder.empty() && sfmFilePath.empty())
  {
    ALICEVISION_LOG_ERROR("Program need -i or --imageFolder option" << std::endl << "No input images.");
    return EXIT_FAILURE;
  }

  // check user don't choose both input options
  if(!imageFolder.empty() && !sfmFilePath.empty())
  {
    ALICEVISION_LOG_ERROR("Cannot combine -i and --imageFolder options");
    return EXIT_FAILURE;
  }

  // check input folder
  if(!imageFolder.empty() && !fs::exists(imageFolder) && !fs::is_directory(imageFolder))
  {
    ALICEVISION_LOG_ERROR("The input folder doesn't exist");
    return EXIT_FAILURE;
  }

  // check sfm file
  if(!sfmFilePath.empty() && !fs::exists(sfmFilePath) && !fs::is_regular_file(sfmFilePath))
  {
    ALICEVISION_LOG_ERROR("The input sfm file doesn't exist");
    return EXIT_FAILURE;
  }

  // check output  string
  if(outputFilePath.empty())
  {
    ALICEVISION_LOG_ERROR("Invalid output");
    return EXIT_FAILURE;
  }

  // ensure output folder exists
  {
    const std::string outputFolderPart = fs::path(outputFilePath).parent_path().string();

    if(!outputFolderPart.empty() && !fs::exists(outputFolderPart))
    {
      if(!fs::create_directory(outputFolderPart))
      {
        ALICEVISION_LOG_ERROR("Cannot create output folder");
        return EXIT_FAILURE;
      }
    }
  }

  if(defaultFocalLength > 0 && defaultFieldOfView > 0)
  {
    ALICEVISION_LOG_ERROR("Cannot combine --defaultFocalLength --defaultFieldOfView options");
    return EXIT_FAILURE;
  }

  if (defaultFocalRatio <= 0.0)
  {
      ALICEVISION_LOG_ERROR("Focal Ratio needs to be a positive value: " << defaultFocalRatio);
      return EXIT_FAILURE;
  }

  // check sensor database
  std::vector<sensorDB::Datasheet> sensorDatabase;
  if (sensorDatabasePath.empty())
  {
      const auto root = image::getAliceVisionRoot();
      if (root.empty())
      {
          ALICEVISION_LOG_WARNING("ALICEVISION_ROOT is not defined, default sensor database cannot be accessed.");
      }
      else
      {
          sensorDatabasePath = root + "/share/aliceVision/cameraSensors.db";
      }
  }

  if(!sensorDatabasePath.empty() && !sensorDB::parseDatabase(sensorDatabasePath, sensorDatabase))
  {
      ALICEVISION_LOG_ERROR("Invalid input database '" << sensorDatabasePath << "', please specify a valid file.");
      return EXIT_FAILURE;
  }

  camera::EINTRINSIC allowedCameraModels = camera::EINTRINSIC_parseStringToBitmask(allowedCameraModelsStr);

  // use current time as seed for random generator for intrinsic Id without metadata
  std::srand(std::time(0));

  std::vector<std::string> noMetadataImagePaths; // imagePaths
  IntrinsicsFromFocal35mmMap intrinsicsSetFromFocal35mm;
  std::vector<std::string> missingDeviceUID;
  UnknownSensorsMap unknownSensors;
  UnsureSensorsMap unsureSensors;
  std::map<IndexT, std::map<int, std::size_t>> detectedRigs; // key rigId, value (subPoseId, nbPose)

  sfmData::SfMData sfmData;

  // number of views with an initialized intrinsic
  std::size_t completeViewCount = 0;

  // load known informations
  if(imageFolder.empty())
  {
    // fill SfMData from the JSON file
    loadJSON(sfmData, sfmFilePath, ESfMData(VIEWS|INTRINSICS|EXTRINSICS), true, viewIdMethod, viewIdRegex);
  }
  else
  {
    // fill SfMData with the images in the input folder
    sfmData::Views& views = sfmData.getViews();
    std::vector<std::string> imagePaths;

    if(listFiles(imageFolder, image::getSupportedExtensions(), imagePaths))
    {
      std::vector<sfmData::View> incompleteViews(imagePaths.size());

      #pragma omp parallel for
      for(int i = 0; i < incompleteViews.size(); ++i)
      {
        sfmData::View& view = incompleteViews.at(i);
        view.setImagePath(imagePaths.at(i));
        updateIncompleteView(view, viewIdMethod, viewIdRegex);
      }

      for(const auto& view : incompleteViews)
        views.emplace(view.getViewId(), std::make_shared<sfmData::View>(view));
    }
    else {
      return EXIT_FAILURE;
    }
  }

  if(sfmData.getViews().empty())
  {
    ALICEVISION_LOG_ERROR("Can't find views in input.");
    return EXIT_FAILURE;
  }

  // create missing intrinsics
  auto viewPairItBegin = sfmData.getViews().begin();

  boost::regex extractComposedNumberRegex("\\d+(?:[\\-\\:\\_\\.]\\d+)*");
  boost::regex extractNumberRegex("\\d+");

  std::map<IndexT, std::vector<IndexT>> poseGroups;

  #pragma omp parallel for
  for(int i = 0; i < sfmData.getViews().size(); ++i)
  {
    sfmData::View& view = *(std::next(viewPairItBegin,i)->second);

    // try to detect rig structure in the input folder
    const fs::path parentPath = fs::path(view.getImagePath()).parent_path();
    if(boost::starts_with(parentPath.parent_path().stem().string(), "rig"))
    {
      try
      {
        IndexT subPoseId;
        std::string prefix;
        std::string suffix;
        if(!sfmDataIO::extractNumberFromFileStem(parentPath.stem().string(), subPoseId, prefix, suffix))
        {
          ALICEVISION_THROW_ERROR("Cannot find sub-pose id from image path: " << parentPath);
        }

        std::hash<std::string> hash; // TODO use boost::hash_combine
        view.setRigAndSubPoseId(hash(parentPath.parent_path().string()), subPoseId);

        #pragma omp critical
        detectedRigs[view.getRigId()][view.getSubPoseId()]++;
      }
      catch(std::exception& e)
      {
        ALICEVISION_LOG_WARNING("Invalid rig structure for view: " << view.getImagePath() << std::endl << e.what() << std::endl << "Used as single image.");
      }
    }

    // try to detect image sequence
    {
        IndexT frameId;
        std::string prefix;
        std::string suffix;
        if(sfmDataIO::extractNumberFromFileStem(fs::path(view.getImagePath()).stem().string(), frameId, prefix, suffix))
        {
          view.setFrameId(frameId);
        }
    }
    
    if(boost::algorithm::starts_with(parentPath.stem().string(), "ps_") ||
       boost::algorithm::starts_with(parentPath.stem().string(), "hdr_"))
    {
        std::hash<std::string> hash;
        IndexT tmpPoseID = hash(parentPath.string()); // use a temporary pose Id to group the images

#pragma omp critical
        {
            poseGroups[tmpPoseID].push_back(view.getViewId());
        }
    }

    bool hasDefinedIntrinsic = false;
    #pragma omp critical
    {
        hasDefinedIntrinsic = sfmDataIO::viewHasDefinedIntrinsic(sfmData, view);
    }
    if (hasDefinedIntrinsic)
    {
        // don't need to build a new intrinsic
        boost::atomic_ref<std::size_t>(completeViewCount)++;
        continue;
    }

    UnknownSensorsMap viewUnknownSensors;
    UnsureSensorsMap viewUnsureSensors;
    std::vector<std::string> viewMissingDeviceUID;
    std::vector<std::string> viewNoMetadataImagePaths;
    IntrinsicsFromFocal35mmMap viewIntrinsicsSetFromFocal35mm;

    auto intrinsicBase = buildViewIntrinsic(view, sensorDatabase,
                                            defaultFocalLength, defaultFieldOfView,
                                            defaultFocalRatio, defaultOffsetX, defaultOffsetY,
                                            defaultCameraModel, allowedCameraModels,
                                            groupCameraFallback,
                                            viewUnknownSensors, viewUnsureSensors,
                                            viewMissingDeviceUID, viewNoMetadataImagePaths,
                                            viewIntrinsicsSetFromFocal35mm);

    if (intrinsicBase && intrinsicBase->isValid())
    {
        // the view intrinsic is initialized
        boost::atomic_ref<std::size_t>(completeViewCount)++;
    }

    #pragma omp critical
    {
      sfmData.getIntrinsics().emplace(view.getIntrinsicId(), intrinsicBase);
      unknownSensors.insert(viewUnknownSensors.begin(), viewUnknownSensors.end());
      unsureSensors.insert(viewUnsureSensors.begin(), viewUnsureSensors.end());
      missingDeviceUID.insert(missingDeviceUID.end(),
                              viewMissingDeviceUID.begin(), viewMissingDeviceUID.end());
      noMetadataImagePaths.insert(noMetadataImagePaths.end(),
                                  viewNoMetadataImagePaths.begin(), viewNoMetadataImagePaths.end());
      intrinsicsSetFromFocal35mm.insert(viewIntrinsicsSetFromFocal35mm.begin(),
                                        viewIntrinsicsSetFromFocal35mm.end());
    }
  }

  // create detected rigs structures
  if(!detectedRigs.empty())
  {
    for(const auto& subPosesPerRigId : detectedRigs)
    {
      const IndexT rigId = subPosesPerRigId.first;
      const std::size_t nbSubPose = subPosesPerRigId.second.size();
      const std::size_t nbPoses = subPosesPerRigId.second.begin()->second;

      for(const auto& nbPosePerSubPoseId : subPosesPerRigId.second)
      {
        // check subPoseId
        if(nbPosePerSubPoseId.first >= nbSubPose)
        {
            ALICEVISION_LOG_ERROR("Wrong subPoseId in rig structure: it should be an index not a random number (subPoseId: " << nbPosePerSubPoseId.first << ", number of subposes: " << nbSubPose << ").");
            return EXIT_FAILURE;
        }
        if(nbPosePerSubPoseId.first < 0)
        {
          ALICEVISION_LOG_ERROR("Wrong subPoseId in rig structure: cannot contain negative value: " << nbPosePerSubPoseId.first);
          return EXIT_FAILURE;
        }

        // check nbPoses
        if(nbPosePerSubPoseId.second != nbPoses)
        {
          ALICEVISION_LOG_WARNING("Wrong number of poses per subPose in detected rig structure (" << nbPosePerSubPoseId.second << " != " << nbPoses << ").");
        }
      }

      sfmData.getRigs().emplace(rigId, sfmData::Rig(nbSubPose));
    }
  }

  // Update poseId for detected multi-exposure or multi-lighting images (multiple shots with the same camera pose)
  if(!poseGroups.empty())
  {
      for(const auto& poseGroup : poseGroups)
      {
          // Sort views of the poseGroup per timestamps
          std::vector<std::pair<int64_t, IndexT>> sortedViews;
          for(const IndexT vId : poseGroup.second)
          {
              int64_t t = sfmData.getView(vId).getMetadataDateTimestamp();
              sortedViews.push_back(std::make_pair(t, vId));
          }
          std::sort(sortedViews.begin(), sortedViews.end());

          // Get the view which was taken at the middle of the sequence
          int median = sortedViews.size() / 2;
          IndexT middleViewId = sortedViews[median].second;

          for(const auto it : sortedViews)
          {
              const IndexT vId = it.second;
              // Update poseId with middle view id
              sfmData.getView(vId).setPoseId(middleViewId);
          }
      }
  }

  if(!noMetadataImagePaths.empty())
  {
    std::stringstream ss;
    ss << "No metadata in image(s):\n";
    for(const auto& imagePath : noMetadataImagePaths)
      ss << "\t- '" << imagePath << "'\n";
    ALICEVISION_LOG_DEBUG(ss.str());
  }

  if(!missingDeviceUID.empty())
  {
      ALICEVISION_LOG_WARNING(
                  "Some image(s) have no serial number to identify the camera/lens device.\n"
                  "This makes it impossible to correctly group the images by device if you have used multiple identical (same model) camera devices.\n"
                  "The reconstruction will assume that only one device has been used, "
                  "so if 2 images share the same focal length approximation they will share the same internal camera parameters.\n"
                  << missingDeviceUID.size() << " image(s) are concerned.");
      ALICEVISION_LOG_DEBUG("The following images are concerned:\n");
      ALICEVISION_LOG_DEBUG(boost::algorithm::join(missingDeviceUID, "\n"));
  }

  if(!unsureSensors.empty())
  {
    ALICEVISION_LOG_WARNING("The camera found in the database is slightly different for image(s):");
    for(const auto& unsureSensor : unsureSensors)
      ALICEVISION_LOG_WARNING("image: '" << fs::path(unsureSensor.second.first).filename().string() << "'" << std::endl
                        << "\t- image camera brand: " << unsureSensor.first.first <<  std::endl
                        << "\t- image camera model: " << unsureSensor.first.second <<  std::endl
                        << "\t- database camera brand: " << unsureSensor.second.second._brand <<  std::endl
                        << "\t- database camera model: " << unsureSensor.second.second._model << std::endl
                        << "\t- database camera sensor width: " << unsureSensor.second.second._sensorWidth  << " mm");
    ALICEVISION_LOG_WARNING("Please check and correct camera model(s) name in the database." << std::endl);
  }

  if(!unknownSensors.empty())
  {
    std::stringstream ss;
    ss << "Sensor width doesn't exist in the database for image(s):\n";
    for(const auto& unknownSensor : unknownSensors)
    {
      ss << "\t- camera brand: " << unknownSensor.first.first << "\n"
         << "\t- camera model: " << unknownSensor.first.second << "\n"
         << "\t   - image: " << fs::path(unknownSensor.second).filename().string() << "\n";
    }
    ss << "Please add camera model(s) and sensor width(s) in the database.";

    ALICEVISION_LOG_WARNING(ss.str());
  }

  if(!intrinsicsSetFromFocal35mm.empty())
  {
    std::stringstream ss;
    ss << "Intrinsic(s) initialized from 'FocalLengthIn35mmFilm' exif metadata in image(s):\n";
    for(const auto& intrinsicSetFromFocal35mm : intrinsicsSetFromFocal35mm)
    {
      ss << "\t- image: " << fs::path(intrinsicSetFromFocal35mm.first).filename().string() << "\n"
         << "\t   - sensor width: " << intrinsicSetFromFocal35mm.second.first  << "\n"
         << "\t   - focal length: " << intrinsicSetFromFocal35mm.second.second << "\n";
    }
    ALICEVISION_LOG_DEBUG(ss.str());
  }

  if(completeViewCount < 1 || (completeViewCount < 2 && !allowSingleView))
  {
    ALICEVISION_LOG_ERROR("At least " << std::string(allowSingleView ? "one image" : "two images") << " should have an initialized intrinsic." << std::endl
                          << "Check your input images metadata (brand, model, focal length, ...), more should be set and correct." << std::endl);
    return EXIT_FAILURE;
  }

  // Add the white balance option to the image metadata
  for (auto vitem : sfmData.getViews())
  {
    if (vitem.second) 
    {
      vitem.second->addMetadata("AliceVision:useWhiteBalance", (useInternalWhiteBalance)?"1":"0");
    }
  }
  
  // store SfMData views & intrinsic data
  if(!Save(sfmData, outputFilePath, ESfMData(VIEWS|INTRINSICS|EXTRINSICS)))
  {
    return EXIT_FAILURE;
  }

  // print report
  ALICEVISION_LOG_INFO("CameraInit report:"
                   << "\n\t- # views listed: " << sfmData.getViews().size()
                   << "\n\t   - # views with an initialized intrinsic listed: " << completeViewCount
                   << "\n\t   - # views without metadata (with a default intrinsic): " <<  noMetadataImagePaths.size()
                   << "\n\t- # intrinsics listed: " << sfmData.getIntrinsics().size());

  return EXIT_SUCCESS;
}
