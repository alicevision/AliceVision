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
#include <aliceVision/system/cmdline.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

#include <iostream>
#include <fstream>
#include <sstream>
#include <memory>
#include <string>
#include <vector>
#include <cstdlib>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 2
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;
namespace fs = boost::filesystem;

/**
 * @brief Check that Kmatrix is a string like "f;0;ppx;0;f;ppy;0;0;1"
 * @param[in] Kmatrix
 * @param[out] focal
 * @param[out] ppx
 * @param[out] ppy
 * @return true if the string is correct
 */
bool checkIntrinsicStringValidity(const std::string& Kmatrix,
                                  double& focal,
                                  double& ppx,
                                  double& ppy)
{
  std::vector<std::string> vec_str;
  boost::split(vec_str, Kmatrix, boost::is_any_of(";"));
  if (vec_str.size() != 9)
  {
    ALICEVISION_LOG_ERROR("In K matrix string, missing ';' character");
    return false;
  }

  // Check that all K matrix value are valid numbers
  for (size_t i = 0; i < vec_str.size(); ++i)
  {
    double readvalue = 0.0;
    std::stringstream ss;
    ss.str(vec_str[i]);
    if(!(ss >> readvalue))
    {
      ALICEVISION_LOG_ERROR("In K matrix string, used an invalid not a number character");
      return false;
    }
    if (i==0) focal = readvalue;
    if (i==2) ppx = readvalue;
    if (i==5) ppy = readvalue;
  }
  return true;
}

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

enum class EGroupCameraFallback {
    GLOBAL,
    FOLDER,
    IMAGE
};

inline std::string EGroupCameraFallback_enumToString(EGroupCameraFallback strategy)
{
  switch(strategy)
  {
    case EGroupCameraFallback::GLOBAL:
      return "global";
    case EGroupCameraFallback::FOLDER:
      return "folder";
    case EGroupCameraFallback::IMAGE:
      return "image";
  }
  throw std::out_of_range("Invalid GroupCameraFallback type Enum: " + std::to_string(int(strategy)));
}

inline EGroupCameraFallback EGroupCameraFallback_stringToEnum(const std::string& strategy)
{
  if(strategy == "global")
    return EGroupCameraFallback::GLOBAL;
  if(strategy == "folder")
    return EGroupCameraFallback::FOLDER;
  if(strategy == "image")
    return EGroupCameraFallback::IMAGE;
  throw std::out_of_range("Invalid GroupCameraFallback type string " + strategy);
}

inline std::ostream& operator<<(std::ostream& os, EGroupCameraFallback s)
{
    return os << EGroupCameraFallback_enumToString(s);
}

inline std::istream& operator>>(std::istream& in, EGroupCameraFallback& s)
{
    std::string token;
    in >> token;
    s = EGroupCameraFallback_stringToEnum(token);
    return in;
}


/**
 * @brief Create the description of an input image dataset for AliceVision toolsuite
 * - Export a SfMData file with View & Intrinsic data
 */
int main(int argc, char **argv)
{
  // command-line parameters

  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  std::string sfmFilePath;
  std::string imageFolder;
  std::string sensorDatabasePath;
  std::string outputFilePath;

  // user optional parameters

  std::string defaultIntrinsicKMatrix;
  std::string defaultCameraModelName;
  double defaultFocalLengthPixel = -1.0;
  double defaultFieldOfView = -1.0;
  EGroupCameraFallback groupCameraFallback = EGroupCameraFallback::FOLDER;

  bool allowSingleView = false;

  po::options_description allParams("AliceVision cameraInit");

  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmFilePath)->default_value(sfmFilePath),
      "A SfMData file (*.sfm) [if specified, --imageFolder cannot be used].")
    ("imageFolder", po::value<std::string>(&imageFolder)->default_value(imageFolder),
      "Input images folder [if specified, --input cannot be used].")
    ("sensorDatabase,s", po::value<std::string>(&sensorDatabasePath)->required(),
      "Camera sensor width database path.")
    ("output,o", po::value<std::string>(&outputFilePath)->default_value("cameraInit.sfm"),
      "Output file path for the new SfMData file");

  po::options_description optionalParams("Optional parameters");
  optionalParams.add_options()
    ("defaultFocalLengthPix", po::value<double>(&defaultFocalLengthPixel)->default_value(defaultFocalLengthPixel),
      "Focal length in pixels. (or '-1' to unset)")
    ("defaultFieldOfView", po::value<double>(&defaultFieldOfView)->default_value(defaultFieldOfView),
      "Empirical value for the field of view in degree. (or '-1' to unset)")
    ("defaultIntrinsic", po::value<std::string>(&defaultIntrinsicKMatrix)->default_value(defaultIntrinsicKMatrix),
      "Intrinsics Kmatrix \"f;0;ppx;0;f;ppy;0;0;1\".")
    ("defaultCameraModel", po::value<std::string>(&defaultCameraModelName)->default_value(defaultCameraModelName),
      "Camera model type (pinhole, radial1, radial3, brown, fisheye4, fisheye1).")
    ("groupCameraFallback", po::value<EGroupCameraFallback>(&groupCameraFallback)->default_value(groupCameraFallback),
      std::string("When there is no serial number in the image metadata, we cannot know if the images come from the same camera. "
      "This is problematic for grouping images sharing the same internal camera settings and we have to decide on a fallback strategy:\n"
      " * " + EGroupCameraFallback_enumToString(EGroupCameraFallback::GLOBAL) + ": all images may come from a single device (make/model/focal will still be a differentiator).\n"
      " * " + EGroupCameraFallback_enumToString(EGroupCameraFallback::FOLDER) + ": different folders will be considered as different devices\n"
      " * " + EGroupCameraFallback_enumToString(EGroupCameraFallback::IMAGE) + ": consider that each image has different internal camera parameters").c_str())
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
  camera::EINTRINSIC defaultCameraModel = camera::EINTRINSIC::PINHOLE_CAMERA_START;
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

  // check user don't combine intrinsic options
  if(!defaultIntrinsicKMatrix.empty() && defaultFocalLengthPixel > 0)
  {
    ALICEVISION_LOG_ERROR("Cannot combine --defaultIntrinsic --defaultFocalLengthPix options");
    return EXIT_FAILURE;
  }

  if(!defaultIntrinsicKMatrix.empty() && defaultFieldOfView > 0)
  {
    ALICEVISION_LOG_ERROR("Cannot combine --defaultIntrinsic --defaultFieldOfView options");
    return EXIT_FAILURE;
  }

  if(defaultFocalLengthPixel > 0 && defaultFieldOfView > 0)
  {
    ALICEVISION_LOG_ERROR("Cannot combine --defaultFocalLengthPix --defaultFieldOfView options");
    return EXIT_FAILURE;
  }

  // read K matrix if valid
  double defaultPPx = -1.0;
  double defaultPPy = -1.0;

  if(!defaultIntrinsicKMatrix.empty() && !checkIntrinsicStringValidity(defaultIntrinsicKMatrix, defaultFocalLengthPixel, defaultPPx, defaultPPy))
  {
    ALICEVISION_LOG_ERROR("--defaultIntrinsic Invalid K matrix input");
    return EXIT_FAILURE;
  }

  // check sensor database
  std::vector<sensorDB::Datasheet> sensorDatabase;
  if(!sensorDatabasePath.empty())
  {
    if(!sensorDB::parseDatabase(sensorDatabasePath, sensorDatabase))
    {
      ALICEVISION_LOG_ERROR("Invalid input database '" << sensorDatabasePath << "', please specify a valid file.");
      return EXIT_FAILURE;
    }
  }

  // use current time as seed for random generator for intrinsic Id without metadata
  std::srand(std::time(0));

  std::vector<std::string> noMetadataImagePaths; // imagePaths
  std::map<std::string, std::pair<double, double>> intrinsicsSetFromFocal35mm; // key imagePath value (sensor width, focal length)
  std::vector<std::string> missingDeviceUID;
  std::map<std::pair<std::string, std::string>, std::string> unknownSensors; // key (make,model), value (first imagePath)
  std::map<std::pair<std::string, std::string>, std::pair<std::string, aliceVision::sensorDB::Datasheet>> unsureSensors; // key (make,model), value (first imagePath,datasheet)
  std::map<IndexT, std::map<int, std::size_t>> detectedRigs; // key rigId, value (subPoseId, nbPose)

  sfmData::SfMData sfmData;

  // number of views with an initialized intrinsic
  std::size_t completeViewCount = 0;

  // load known informations
  if(imageFolder.empty())
  {
    // fill SfMData from the JSON file
    sfmDataIO::loadJSON(sfmData, sfmFilePath, sfmDataIO::ESfMData(sfmDataIO::VIEWS|sfmDataIO::INTRINSICS|sfmDataIO::EXTRINSICS), true);
  }
  else
  {
    // fill SfMData with the images in the input folder
    sfmData::Views& views = sfmData.getViews();
    std::vector<std::string> imagePaths;

    if(listFiles(imageFolder, {".jpg", ".jpeg", ".tif", ".tiff", ".exr"},  imagePaths))
    {
      std::vector<sfmData::View> incompleteViews(imagePaths.size());

      #pragma omp parallel for
      for(int i = 0; i < incompleteViews.size(); ++i)
      {
        sfmData::View& view = incompleteViews.at(i);
        view.setImagePath(imagePaths.at(i));
        sfmDataIO::updateIncompleteView(view);
      }

      for(const auto& view : incompleteViews)
        views.emplace(view.getViewId(), std::make_shared<sfmData::View>(view));
    }
    else
      return EXIT_FAILURE;
  }

  if(sfmData.getViews().empty())
  {
    ALICEVISION_LOG_ERROR("Can't find views in input.");
    return EXIT_FAILURE;
  }

  // create missing intrinsics
  auto viewPairItBegin = sfmData.getViews().begin();

  #pragma omp parallel for
  for(int i = 0; i < sfmData.getViews().size(); ++i)
  {
    sfmData::View& view = *(std::next(viewPairItBegin,i)->second);

    // try to detect rig structure in the input folder
    const fs::path parentPath = fs::path(view.getImagePath()).parent_path();
    if(parentPath.parent_path().stem() == "rig")
    {
      try
      {
        const int frameId = std::stoi(fs::path(view.getImagePath()).stem().string());
        const int subPoseId = std::stoi(parentPath.stem().string());
        std::hash<std::string> hash; // TODO use boost::hash_combine
        view.setRigAndSubPoseId(hash(parentPath.parent_path().string()), subPoseId);
        view.setFrameId(static_cast<IndexT>(frameId));

        #pragma omp critical
        detectedRigs[view.getRigId()][view.getSubPoseId()]++;
      }
      catch(std::exception& e)
      {
        ALICEVISION_LOG_WARNING("Invalid rig structure for view: " << view.getImagePath() << std::endl << "Used as single image.");
      }
    }

    IndexT intrinsicId = view.getIntrinsicId();
    double sensorWidth = -1;
    enum class ESensorWidthSource {
        FROM_DB,
        FROM_METADATA_ESTIMATION,
        UNKNOWN
    } sensorWidthSource = ESensorWidthSource::UNKNOWN;

    double focalLengthmm = view.getMetadataFocalLength();
    const std::string& make = view.getMetadataMake();
    const std::string& model = view.getMetadataModel();
    const bool hasCameraMetadata = (!make.empty() || !model.empty());
    const bool hasFocalIn35mmMetadata = view.hasDigitMetadata("Exif:FocalLengthIn35mmFilm");
    const double focalIn35mm = hasFocalIn35mmMetadata ? std::stod(view.getMetadata("Exif:FocalLengthIn35mmFilm")) : -1.0;
    const double imageRatio = static_cast<double>(view.getWidth()) / static_cast<double>(view.getHeight());
    const double diag24x36 = std::sqrt(36.0 * 36.0 + 24.0 * 24.0);
    camera::EIntrinsicInitMode intrinsicInitMode = camera::EIntrinsicInitMode::UNKNOWN;

    // check if the view intrinsic is already defined
    if(intrinsicId != UndefinedIndexT)
    {
      camera::IntrinsicBase* intrinsicBase = sfmData.getIntrinsicPtr(view.getIntrinsicId());
      camera::Pinhole* intrinsic = dynamic_cast<camera::Pinhole*>(intrinsicBase);
      if(intrinsic != nullptr)
      {
        if(intrinsic->getFocalLengthPix() > 0)
        {
          // the view intrinsic is initialized
          #pragma omp atomic
          ++completeViewCount;

          // don't need to build a new intrinsic
          continue;
        }
      }
    }

    // try to find in the sensor width in the database
    if(hasCameraMetadata)
    {
      sensorDB::Datasheet datasheet;
      if(sensorDB::getInfo(make, model, sensorDatabase, datasheet))
      {
        // sensor is in the database
        ALICEVISION_LOG_TRACE("Sensor width found in database: " << std::endl
                              << "\t- brand: " << make << std::endl
                              << "\t- model: " << model << std::endl
                              << "\t- sensor width: " << datasheet._sensorSize << " mm");

        if(datasheet._model != model) // the camera model in database is slightly different
          unsureSensors.emplace(std::make_pair(make, model), std::make_pair(view.getImagePath(), datasheet)); // will throw a warning message

        sensorWidth = datasheet._sensorSize;
        sensorWidthSource = ESensorWidthSource::FROM_DB;

        if(focalLengthmm > 0.0)
          intrinsicInitMode = camera::EIntrinsicInitMode::ESTIMATED;
      }
    }

    // try to find / compute with 'FocalLengthIn35mmFilm' metadata
    if(hasFocalIn35mmMetadata)
    {
      if(sensorWidth == -1.0)
      {
        const double invRatio = 1.0 / imageRatio;

        if(focalLengthmm > 0.0)
        {
          // no sensorWidth but valid focalLength and valid focalLengthIn35mm, so deduce sensorWith approximation
          const double sensorDiag = (focalLengthmm * diag24x36) / focalIn35mm; // 43.3 is the diagonal of 35mm film
          sensorWidth = sensorDiag * std::sqrt(1.0 / (1.0 + invRatio * invRatio));
          sensorWidthSource = ESensorWidthSource::FROM_METADATA_ESTIMATION;
        }
        else
        {
          // no sensorWidth and no focalLength but valid focalLengthIn35mm, so consider sensorWith as 35mm
          sensorWidth = diag24x36 * std::sqrt(1.0 / (1.0 + invRatio * invRatio));
          focalLengthmm = sensorWidth * (focalIn35mm ) / 36.0;
          sensorWidthSource = ESensorWidthSource::UNKNOWN;
        }

        intrinsicsSetFromFocal35mm.emplace(view.getImagePath(), std::make_pair(sensorWidth, focalLengthmm));
        intrinsicInitMode = camera::EIntrinsicInitMode::ESTIMATED;
      }
      else if(sensorWidth > 0 && focalLengthmm <= 0)
      {
        // valid sensorWidth and valid focalLengthIn35mm but no focalLength, so convert focalLengthIn35mm to the actual width of the sensor
        const double sensorDiag = std::sqrt(std::pow(sensorWidth, 2) +  std::pow(sensorWidth / imageRatio,2));
        focalLengthmm = (sensorDiag * focalIn35mm) / diag24x36;

        intrinsicsSetFromFocal35mm.emplace(view.getImagePath(), std::make_pair(sensorWidth, focalLengthmm));
        intrinsicInitMode = camera::EIntrinsicInitMode::ESTIMATED;
      }
    }

    // error handling
    if(sensorWidth == -1.0)
    {
  #pragma omp critical
      if(hasCameraMetadata)
      {
        // sensor is not in the database
        unknownSensors.emplace(std::make_pair(make, model), view.getImagePath()); // will throw a warning at the end
      }
      else
      {
        // no metadata 'Make' and 'Model' can't find sensor width
        noMetadataImagePaths.emplace_back(view.getImagePath()); // will throw a warning message at the end
      }
    }
    else
    {
      // we have a valid sensorWidth information, so se store it into the metadata (where it would have been nice to have it in the first place)
      if(sensorWidthSource == ESensorWidthSource::FROM_DB)
        view.addMetadata("AliceVision:SensorWidth", std::to_string(sensorWidth));
      else if(sensorWidthSource == ESensorWidthSource::FROM_METADATA_ESTIMATION)
        view.addMetadata("AliceVision:SensorWidthEstimation", std::to_string(sensorWidth));
      // else it is just a guess, so there is no need to put it into the metadata.
    }

    // build intrinsic
    std::shared_ptr<camera::IntrinsicBase> intrinsicBase = sfmDataIO::getViewIntrinsic(view, focalLengthmm, sensorWidth, defaultFocalLengthPixel, defaultFieldOfView, defaultCameraModel, defaultPPx, defaultPPy);
    camera::Pinhole* intrinsic = dynamic_cast<camera::Pinhole*>(intrinsicBase.get());

    // set initialization mode
    intrinsic->setInitializationMode(intrinsicInitMode);

    if(intrinsic && intrinsic->getFocalLengthPix() > 0)
    {
      // the view intrinsic is initialized
      #pragma omp atomic
      ++completeViewCount;
    }

    // Create serial number if not already filled
    if(intrinsic->serialNumber().empty())
    {
      // Create custom serial number
      const std::string& bodySerialNumber = view.getMetadataBodySerialNumber();
      const std::string& lensSerialNumber = view.getMetadataLensSerialNumber();

      if(!bodySerialNumber.empty() || !lensSerialNumber.empty())
      {
        // We can identify the device based on a unique ID.
        intrinsic->setSerialNumber(bodySerialNumber + "_" + lensSerialNumber);
      }
      else
      {
        // We have no way to identify a camera device correctly.
#pragma omp critical
        {
          missingDeviceUID.emplace_back(view.getImagePath()); // will throw a warning message at the end
        }

        // To avoid stopping the process, we fallback to a solution selected by the user:
        if(groupCameraFallback == EGroupCameraFallback::FOLDER)
        {
          // when we don't have a serial number, the folder will become part of the device ID.
          // This means that 2 images in different folder will NOT share intrinsics.
          intrinsic->setSerialNumber(fs::path(view.getImagePath()).parent_path().string());
        }
        else if(groupCameraFallback == EGroupCameraFallback::IMAGE)
        {
          // if no serial number, each view will get its own camera intrinsic parameters.
          intrinsic->setSerialNumber(view.getImagePath());
        }
        else if(groupCameraFallback == EGroupCameraFallback::GLOBAL)
        {
          // if no serial number, images with the same make/model/focal or no make/model/focal
          // will be considered as a single group of camera intrinsics.
        }

        if(!make.empty() || !model.empty())
        {
          // We have no correct way to identify a camera device, we fallback on the camera make/model.
          // If you use multiple identical devices, they will be fused together incorrectly.
          intrinsic->setSerialNumber(intrinsic->serialNumber() + "_" + make + "_" + model);
        }

        if(view.isPartOfRig())
        {
          // when we have no unique camera identifier, so for rig images, we ensure that each camera of the rig have different serial numbers.
          intrinsic->setSerialNumber(intrinsic->serialNumber() + "_rig_" + std::to_string(view.getRigId()) + "_" + std::to_string(view.getSubPoseId()));
        }
      }

      // If we have not managed to initialize the focal length, we need to use the
      if(intrinsic->getFocalLengthPix() <= 0 && focalLengthmm > 0)
      {
        intrinsic->setSerialNumber(intrinsic->serialNumber() + "_FocalLengthMM_" + std::to_string(focalLengthmm));
      }
    }

    // create intrinsic id
    // group camera that share common properties (leads to more faster & stable BA).
    if(intrinsicId == UndefinedIndexT)
    {
      intrinsicId = intrinsic->hashValue();
    }

    #pragma omp critical
    {
      view.setIntrinsicId(intrinsicId);
      sfmData.getIntrinsics().emplace(intrinsicId, intrinsicBase);
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
        if((nbPosePerSubPoseId.first >= nbSubPose) || (nbPosePerSubPoseId.first < 0))
        {
          ALICEVISION_LOG_ERROR("Wrong subPoseId in detected rig structure.");
          return EXIT_FAILURE;
        }

        // check nbPoses
        if(nbPosePerSubPoseId.second != nbPoses)
        {
          ALICEVISION_LOG_ERROR("Wrong number of poses per subPose in detected rig structure (" << nbPosePerSubPoseId.second << " != " << nbPoses << ").");
          return EXIT_FAILURE;
        }
      }

      sfmData.getRigs().emplace(rigId, sfmData::Rig(nbSubPose));
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
                        << "\t- database camera sensor size: " << unsureSensor.second.second._sensorSize << " mm");
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

  // store SfMData views & intrinsic data
  if(!sfmDataIO::Save(sfmData, outputFilePath, sfmDataIO::ESfMData(sfmDataIO::VIEWS|sfmDataIO::INTRINSICS|sfmDataIO::EXTRINSICS)))
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
