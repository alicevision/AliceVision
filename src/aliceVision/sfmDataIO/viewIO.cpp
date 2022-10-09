// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "viewIO.hpp"

#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/sensorDB/parseDatabase.hpp>
#include <aliceVision/sfmData/uid.hpp>
#include <aliceVision/camera/camera.hpp>
#include <aliceVision/image/io.hpp>
#include "aliceVision/utils/filesIO.hpp"

#include <stdexcept>
#include <regex>

namespace fs = boost::filesystem;

namespace aliceVision {
namespace sfmDataIO {

void updateIncompleteView(sfmData::View& view, EViewIdMethod viewIdMethod, const std::string& viewIdRegex)
{
  // check if the view is complete
  if(view.getViewId() != UndefinedIndexT &&
     view.getIntrinsicId() != UndefinedIndexT &&
     view.getPoseId() == view.getViewId() &&
     view.getHeight() > 0 &&
     view.getWidth() >  0)
    return;

  int width, height;
  const auto metadata = image::readImageMetadata(view.getImagePath(), width, height);

  view.setWidth(width);
  view.setHeight(height);

  // reset metadata
  if(view.getMetadata().empty())
    view.setMetadata(image::getMapFromMetadata(metadata));

  // Reset viewId
  if(view.getViewId() == UndefinedIndexT)
  {
    if(viewIdMethod == EViewIdMethod::FILENAME)
    {
      std::regex re;
      try
      {
        re = viewIdRegex;
      }
      catch(const std::regex_error& e)
      {
        throw std::invalid_argument("Invalid regex conversion, your regexfilename '" + viewIdRegex + "' may be invalid.");
      }

      // Get view image filename without extension
      const std::string filename = boost::filesystem::path(view.getImagePath()).stem().string();

      std::smatch match;
      std::regex_search(filename, match, re);
      if(match.size() == 2)
      {
          try
          {
            const IndexT id(std::stoul(match.str(1)));
            view.setViewId(id);
          }
          catch(std::invalid_argument& e)
          {
            ALICEVISION_LOG_ERROR("ViewId captured in the filename '" << filename << "' can't be converted to a number. "
                                  "The regex '" << viewIdRegex << "' is probably incorrect.");
            throw;
          }
      }
      else
      {
        ALICEVISION_LOG_ERROR("The Regex '" << viewIdRegex << "' must match a unique number in the filename " << filename << "' to be used as viewId.");
        throw std::invalid_argument("The Regex '" + viewIdRegex + "' must match a unique number in the filename " + filename + "' to be used as viewId.");
      }
    }
    else
    {
      // Use metadata
      view.setViewId(sfmData::computeViewUID(view));
    }
  }

  if(view.getPoseId() == UndefinedIndexT)
  {
    // check if the rig poseId id is defined
    if(view.isPartOfRig())
    {
      ALICEVISION_LOG_ERROR("Error: Can't find poseId for'" << fs::path(view.getImagePath()).filename().string() << "' marked as part of a rig." << std::endl);
      throw std::invalid_argument("Error: Can't find poseId for'" + fs::path(view.getImagePath()).filename().string() + "' marked as part of a rig.");
    }
    else
      view.setPoseId(view.getViewId());
  }
  else if((!view.isPartOfRig()) && (view.getPoseId() != view.getViewId()))
  {
    ALICEVISION_LOG_WARNING("PoseId and viewId are different for image '" << fs::path(view.getImagePath()).filename().string() << "'." << std::endl);
  }
}

std::shared_ptr<camera::IntrinsicBase> getViewIntrinsic(
                    const sfmData::View& view, double mmFocalLength, double sensorWidth,
                    double defaultFocalLength, double defaultFieldOfView, 
                    double defaultFocalRatio, double defaultOffsetX, double defaultOffsetY,
                    camera::EINTRINSIC defaultIntrinsicType,
                    camera::EINTRINSIC allowedEintrinsics)
{
  // can't combine defaultFocalLengthPx and defaultFieldOfView
  assert(defaultFocalLength < 0 || defaultFieldOfView < 0);

  // get view informations
  const std::string& cameraBrand = view.getMetadataMake();
  const std::string& cameraModel = view.getMetadataModel();
  const std::string& bodySerialNumber = view.getMetadataBodySerialNumber();
  const std::string& lensSerialNumber = view.getMetadataLensSerialNumber();

  double focalLength{-1.0};
  bool hasFocalLengthInput = false;

  if (sensorWidth < 0)
  {
    ALICEVISION_LOG_WARNING("Sensor size is unknown");
    ALICEVISION_LOG_WARNING("Use default sensor size (36 mm)");
    sensorWidth = 36.0;
  }
  
  if(defaultFocalLength > 0.0)
  {
    focalLength = defaultFocalLength;
  }

  if(defaultFieldOfView > 0.0)
  {
    const double focalRatio = 0.5 / std::tan(0.5 * degreeToRadian(defaultFieldOfView));
    focalLength = focalRatio * sensorWidth;
  }

  camera::EINTRINSIC intrinsicType = defaultIntrinsicType;

  bool isResized = false;

  if(view.hasMetadata({"Exif:PixelXDimension", "PixelXDimension"}) && view.hasMetadata({"Exif:PixelYDimension", "PixelYDimension"})) // has dimension metadata
  {
    // check if the image is resized
    int exifWidth = std::stoi(view.getMetadata({"Exif:PixelXDimension", "PixelXDimension"}));
    int exifHeight = std::stoi(view.getMetadata({"Exif:PixelYDimension", "PixelXDimension"}));

    // if metadata is rotated
    if(exifWidth == view.getHeight() && exifHeight == view.getWidth())
      std::swap(exifWidth, exifHeight);


    if(exifWidth > 0 && exifHeight > 0 &&
       (exifWidth != view.getWidth() || exifHeight != view.getHeight()))
    {
      ALICEVISION_LOG_WARNING("Resized image detected: " << fs::path(view.getImagePath()).filename().string() << std::endl
                          << "\t- real image size: " <<  view.getWidth() << "x" <<  view.getHeight() << std::endl
                          << "\t- image size from exif metadata is: " << exifWidth << "x" << exifHeight << std::endl);
      isResized = true;
    }
  }

  // handle case where focal length (mm) is unset or false
  if(mmFocalLength <= 0.0)
  {
    ALICEVISION_LOG_WARNING("Image '" << fs::path(view.getImagePath()).filename().string() << "' focal length (in mm) metadata is missing." << std::endl
                             << "Can't compute focal length, use default." << std::endl);
  }
  else
  {
    // Retrieve the focal from the metadata in mm and convert to pixel.
    focalLength = mmFocalLength;
    hasFocalLengthInput = true;
  }

  double focalLengthIn35mm = 36.0 * focalLength;
  double pxFocalLength = (focalLength / sensorWidth) * std::max(view.getWidth(), view.getHeight());

  bool hasFisheyeCompatibleParameters = ((focalLengthIn35mm > 0.0 && focalLengthIn35mm < 18.0) || (defaultFieldOfView > 100.0));
  bool checkPossiblePinhole = (allowedEintrinsics & camera::EINTRINSIC::PINHOLE_CAMERA_FISHEYE) && hasFisheyeCompatibleParameters;

  // choose intrinsic type
  if(cameraBrand == "Custom")
  {
    intrinsicType = camera::EINTRINSIC_stringToEnum(cameraModel);
  }
  else if(checkPossiblePinhole)
  {
    // If the focal lens is short, the fisheye model should fit better.
    intrinsicType = camera::EINTRINSIC::PINHOLE_CAMERA_FISHEYE;
  }
  else if(intrinsicType == camera::EINTRINSIC::UNKNOWN)
  {
    // Choose a default camera model if no default type
    static const std::initializer_list<camera::EINTRINSIC> intrinsicsPriorities = {
        camera::EINTRINSIC::PINHOLE_CAMERA_RADIAL3,
        camera::EINTRINSIC::PINHOLE_CAMERA_BROWN,
        camera::EINTRINSIC::PINHOLE_CAMERA_RADIAL1,
        camera::EINTRINSIC::PINHOLE_CAMERA_FISHEYE,
        camera::EINTRINSIC::PINHOLE_CAMERA_FISHEYE1,
        camera::EINTRINSIC::PINHOLE_CAMERA
    };

    for(const auto& e : intrinsicsPriorities)
    {
        if(allowedEintrinsics & e)
        {
            intrinsicType = e;
            break;
        }
    }

    // If still unassigned
    if (intrinsicType == camera::EINTRINSIC::UNKNOWN)
    {
        throw std::invalid_argument("No intrinsic type can be attributed.");
    }
  }

  // create the desired intrinsic
  std::shared_ptr<camera::IntrinsicBase> intrinsic = camera::createIntrinsic(intrinsicType, view.getWidth(), view.getHeight(), pxFocalLength, pxFocalLength, 0, 0);
  if(hasFocalLengthInput)
  {
    std::shared_ptr<camera::IntrinsicsScaleOffset> intrinsicScaleOffset = std::dynamic_pointer_cast<camera::IntrinsicsScaleOffset>(intrinsic);
    
    if (intrinsicScaleOffset)
    {
      intrinsicScaleOffset->setInitialScale({pxFocalLength, (pxFocalLength > 0)?pxFocalLength / defaultFocalRatio : -1});
      intrinsicScaleOffset->setOffset({defaultOffsetX, defaultOffsetY});
    }
  }

  // initialize distortion parameters
  switch(intrinsicType)
  {
    case camera::EINTRINSIC::PINHOLE_CAMERA_FISHEYE:
    {
      if(cameraBrand == "GoPro")
        intrinsic->updateFromParams({pxFocalLength, pxFocalLength, 0, 0, 0.0524, 0.0094, -0.0037, -0.0004});
      break;
    }
    case camera::EINTRINSIC::PINHOLE_CAMERA_FISHEYE1:
    {
      if(cameraBrand == "GoPro")
        intrinsic->updateFromParams({pxFocalLength, pxFocalLength, 0, 0, 1.04});
      break;
    }
    default: break;
  }

  // create serial number
  intrinsic->setSerialNumber(bodySerialNumber + lensSerialNumber);

  return intrinsic;
}

std::vector<std::string> viewPathsFromFolders(const sfmData::View& view, const std::vector<std::string>& folders)
{
    return utils::getFilesPathsFromFolders(folders, [&view](const boost::filesystem::path& path) {
        const boost::filesystem::path stem = path.stem();
        return (stem == std::to_string(view.getViewId()) || stem == fs::path(view.getImagePath()).stem());
    });
}

bool extractNumberFromFileStem(const std::string& imagePathStem, IndexT& number, std::string& prefix, std::string& suffix)
{
    // check if the image stem contains a number
    // regexFrame: ^(.*\D)?([0-9]+)([\-_\.].*[[:alpha:]].*)?$
    std::regex regexFrame("^(.*\\D)?"       // the optional prefix which ends with a non digit character
                          "([0-9]+)"        // the number
                          "([\\-_\\.]"      // the suffix starts with a separator
                          ".*[[:alpha:]].*" // at least one letter in the suffix
                          ")?$"             // suffix is optional
    );

    std::smatch matches;
    const bool containsNumber = std::regex_search(imagePathStem, matches, regexFrame);

    if(containsNumber)
    {
        prefix = matches[1];
        suffix = matches[3];
        number = static_cast<IndexT>(std::stoi(matches[2]));
    }

    return containsNumber;
}

bool viewHasDefinedIntrinsic(const sfmData::SfMData& sfmData, const sfmData::View& view)
{
    auto intrinsicId = view.getIntrinsicId();
    if (intrinsicId == UndefinedIndexT)
        return false;


    auto* intrinsicBase = sfmData.getIntrinsicPtr(view.getIntrinsicId());
    auto* intrinsic = dynamic_cast<const camera::Pinhole*>(intrinsicBase);
    if (intrinsic == nullptr)
        return false;

    if (intrinsic->getFocalLengthPixX() <= 0)
        return false;

    return true;
}


std::string EGroupCameraFallback_enumToString(EGroupCameraFallback strategy)
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

EGroupCameraFallback EGroupCameraFallback_stringToEnum(const std::string& strategy)
{
    if (strategy == "global")
        return EGroupCameraFallback::GLOBAL;
    if (strategy == "folder")
        return EGroupCameraFallback::FOLDER;
    if (strategy == "image")
        return EGroupCameraFallback::IMAGE;
    throw std::out_of_range("Invalid GroupCameraFallback type string " + strategy);
}

std::ostream& operator<<(std::ostream& os, EGroupCameraFallback s)
{
    return os << EGroupCameraFallback_enumToString(s);
}

std::istream& operator>>(std::istream& in, EGroupCameraFallback& s)
{
    std::string token;
    in >> token;
    s = EGroupCameraFallback_stringToEnum(token);
    return in;
}

void BuildViewIntrinsicsReport::merge(const BuildViewIntrinsicsReport& other)
{
    unknownSensors.insert(other.unknownSensors.begin(), other.unknownSensors.end());
    unsureSensors.insert(other.unsureSensors.begin(), other.unsureSensors.end());
    missingDeviceUID.insert(missingDeviceUID.end(),
                            other.missingDeviceUID.begin(), other.missingDeviceUID.end());
    noMetadataImagePaths.insert(noMetadataImagePaths.end(),
                                other.noMetadataImagePaths.begin(), other.noMetadataImagePaths.end());
    intrinsicsSetFromFocal35mm.insert(other.intrinsicsSetFromFocal35mm.begin(),
                                      other.intrinsicsSetFromFocal35mm.end());
}

void BuildViewIntrinsicsReport::reportToLog()
{
    if (!noMetadataImagePaths.empty())
    {
        std::stringstream ss;
        ss << "No metadata in image(s):\n";
        for(const auto& imagePath : noMetadataImagePaths)
            ss << "\t- '" << imagePath << "'\n";
        ALICEVISION_LOG_DEBUG(ss.str());
    }

    if (!missingDeviceUID.empty())
    {
        ALICEVISION_LOG_WARNING(
            "Some image(s) have no serial number to identify the camera/lens device.\n"
            "This makes it impossible to correctly group the images by device if you have used \n"
            "multiple identical (same model) camera devices. The reconstruction will assume that \n"
            "only one device has been used, so if 2 images share the same focal length \n"
            "approximation they will share the same internal camera parameters.\n"
            << missingDeviceUID.size() << " image(s) are concerned.");
        ALICEVISION_LOG_DEBUG("The following images are concerned:\n");
        ALICEVISION_LOG_DEBUG(boost::algorithm::join(missingDeviceUID, "\n"));
    }

    if (!unsureSensors.empty())
    {
        ALICEVISION_LOG_WARNING("The camera found in the database is slightly different for image(s):");
        for (const auto& unsureSensor : unsureSensors)
            ALICEVISION_LOG_WARNING("image: '" << fs::path(unsureSensor.second.first).filename().string() << "'\n"
                          << "\t- image camera brand: " << unsureSensor.first.first << "\n"
                          << "\t- image camera model: " << unsureSensor.first.second << "\n"
                          << "\t- database camera brand: " << unsureSensor.second.second._brand << "\n"
                          << "\t- database camera model: " << unsureSensor.second.second._model << "\n"
                          << "\t- database camera sensor width: " << unsureSensor.second.second._sensorWidth  << " mm");
        ALICEVISION_LOG_WARNING("Please check and correct camera model(s) name in the database.\n");
    }

    if (!unknownSensors.empty())
    {
        std::stringstream ss;
        ss << "Sensor width doesn't exist in the database for image(s):\n";
        for (const auto& unknownSensor : unknownSensors)
        {
            ss << "\t- camera brand: " << unknownSensor.first.first << "\n"
               << "\t- camera model: " << unknownSensor.first.second << "\n"
               << "\t   - image: " << fs::path(unknownSensor.second).filename().string() << "\n";
        }
        ss << "Please add camera model(s) and sensor width(s) in the database.";

        ALICEVISION_LOG_WARNING(ss.str());
    }

    if (!intrinsicsSetFromFocal35mm.empty())
    {
        std::stringstream ss;
        ss << "Intrinsic(s) initialized from 'FocalLengthIn35mmFilm' exif metadata in image(s):\n";
        for (const auto& intrinsicSetFromFocal35mm : intrinsicsSetFromFocal35mm)
        {
            ss << "\t- image: " << fs::path(intrinsicSetFromFocal35mm.first).filename().string() << "\n"
               << "\t   - sensor width: " << intrinsicSetFromFocal35mm.second.first  << "\n"
               << "\t   - focal length: " << intrinsicSetFromFocal35mm.second.second << "\n";
        }
        ALICEVISION_LOG_DEBUG(ss.str());
    }
}

std::shared_ptr<camera::IntrinsicBase>
    buildViewIntrinsic(sfmData::View& view,
                       const std::vector<sensorDB::Datasheet>& sensorDatabase,
                       double defaultFocalLength, double defaultFieldOfView,
                       double defaultFocalRatio, double defaultOffsetX, double defaultOffsetY,
                       camera::EINTRINSIC defaultCameraModel,
                       camera::EINTRINSIC allowedCameraModels,
                       EGroupCameraFallback groupCameraFallback,
                       BuildViewIntrinsicsReport& report)
{
    IndexT intrinsicId = view.getIntrinsicId();
    double sensorWidth = -1;
    double sensorHeight = -1;
    enum class ESensorWidthSource {
        FROM_DB,
        FROM_METADATA_ESTIMATION,
        UNKNOWN
    } sensorWidthSource = ESensorWidthSource::UNKNOWN;

    double focalLengthmm = view.getMetadataFocalLength();
    const std::string& make = view.getMetadataMake();
    const std::string& model = view.getMetadataModel();
    const bool hasCameraMetadata = (!make.empty() || !model.empty());
    const bool hasFocalIn35mmMetadata = view.hasDigitMetadata({"Exif:FocalLengthIn35mmFilm",
                                                               "FocalLengthIn35mmFilm"});
    const double focalIn35mm = hasFocalIn35mmMetadata
            ? view.getDoubleMetadata({"Exif:FocalLengthIn35mmFilm", "FocalLengthIn35mmFilm"})
            : -1.0;

    const double imageRatio = static_cast<double>(view.getWidth()) / static_cast<double>(view.getHeight());
    const double diag24x36 = std::sqrt(36.0 * 36.0 + 24.0 * 24.0);
    camera::EIntrinsicInitMode intrinsicInitMode = camera::EIntrinsicInitMode::UNKNOWN;

    // try to find in the sensor width in the database
    if (hasCameraMetadata)
    {
        sensorDB::Datasheet datasheet;
        if (sensorDB::getInfo(make, model, sensorDatabase, datasheet))
        {
            // sensor is in the database
            ALICEVISION_LOG_TRACE("Sensor width found in database: " << std::endl
                                  << "\t- brand: " << make << std::endl
                                  << "\t- model: " << model << std::endl
                                  << "\t- sensor width: " << datasheet._sensorWidth << " mm");

            if (datasheet._model != model)
            {
                // the camera model in database is slightly different
                report.unsureSensors.emplace(std::make_pair(make, model),
                                             std::make_pair(view.getImagePath(), datasheet));
            }

            sensorWidth = datasheet._sensorWidth;
            sensorWidthSource = ESensorWidthSource::FROM_DB;

            if (focalLengthmm > 0.0)
            {
                intrinsicInitMode = camera::EIntrinsicInitMode::ESTIMATED;
            }
        }
    }

    // try to find / compute with 'FocalLengthIn35mmFilm' metadata
    if (hasFocalIn35mmMetadata)
    {
        if (sensorWidth == -1.0)
        {
            const double invRatio = 1.0 / imageRatio;

            if (focalLengthmm > 0.0)
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

            report.intrinsicsSetFromFocal35mm.emplace(view.getImagePath(),
                                                      std::make_pair(sensorWidth, focalLengthmm));
            intrinsicInitMode = camera::EIntrinsicInitMode::ESTIMATED;
        }
        else if (sensorWidth > 0 && focalLengthmm <= 0)
        {
            // valid sensorWidth and valid focalLengthIn35mm but no focalLength, so convert
            // focalLengthIn35mm to the actual width of the sensor
            const double sensorDiag = std::sqrt(std::pow(sensorWidth, 2) +
                                                std::pow(sensorWidth / imageRatio, 2));

            focalLengthmm = (sensorDiag * focalIn35mm) / diag24x36;

            report.intrinsicsSetFromFocal35mm.emplace(view.getImagePath(),
                                                      std::make_pair(sensorWidth, focalLengthmm));
            intrinsicInitMode = camera::EIntrinsicInitMode::ESTIMATED;
        }
    }

    // error handling
    if (sensorWidth == -1.0)
    {
        if (hasCameraMetadata)
        {
            // Sensor is not in the database.
            report.unknownSensors.emplace(std::make_pair(make, model), view.getImagePath());
        }
        else
        {
            // No metadata 'Make' and 'Model' can't find sensor width.
            report.noMetadataImagePaths.emplace_back(view.getImagePath());
        }
    }
    else
    {
        // We have a valid sensorWidth information, so se store it into the metadata (where it would have been nice to have it in the first place)
        if (sensorWidthSource == ESensorWidthSource::FROM_DB)
        {
            view.addMetadata("AliceVision:SensorWidth", std::to_string(sensorWidth));
        }
        else if (sensorWidthSource == ESensorWidthSource::FROM_METADATA_ESTIMATION)
        {
            view.addMetadata("AliceVision:SensorWidthEstimation", std::to_string(sensorWidth));
        }
    }

    if (sensorWidth < 0)
    {
      ALICEVISION_LOG_WARNING("Sensor size is unknown");
      ALICEVISION_LOG_WARNING("Use default sensor size (36 mm)");
      sensorWidth = 36.0;
    }

    // build intrinsic
    auto intrinsicBase = getViewIntrinsic(view, focalLengthmm, sensorWidth, defaultFocalLength,
                                          defaultFieldOfView, defaultFocalRatio,
                                          defaultOffsetX, defaultOffsetY,
                                          defaultCameraModel, allowedCameraModels);
    auto intrinsic = std::dynamic_pointer_cast<camera::IntrinsicsScaleOffset>(intrinsicBase);

    // set initialization mode
    intrinsic->setInitializationMode(intrinsicInitMode);

    // Set sensor size
    if (sensorHeight > 0.0)
    {
        intrinsicBase->setSensorWidth(sensorWidth);
        intrinsicBase->setSensorHeight(sensorHeight);
    }
    else
    {
        if (imageRatio > 1.0)
        {
            intrinsicBase->setSensorWidth(sensorWidth);
            intrinsicBase->setSensorHeight(sensorWidth / imageRatio);
        }
        else
        {
            intrinsicBase->setSensorWidth(sensorWidth);
            intrinsicBase->setSensorHeight(sensorWidth * imageRatio);
        }
    }

    // Create serial number if not already filled
    if (intrinsic->serialNumber().empty())
    {
        // Create custom serial number
        const std::string& bodySerialNumber = view.getMetadataBodySerialNumber();
        const std::string& lensSerialNumber = view.getMetadataLensSerialNumber();

        if (!bodySerialNumber.empty() || !lensSerialNumber.empty())
        {
            // We can identify the device based on a unique ID.
            intrinsic->setSerialNumber(bodySerialNumber + "_" + lensSerialNumber);
        }
        else
        {
            // We have no way to identify a camera device correctly.
            report.missingDeviceUID.emplace_back(view.getImagePath());

            // To avoid stopping the process, we fallback to a solution selected by the user:
            if (groupCameraFallback == EGroupCameraFallback::FOLDER)
            {
                // when we don't have a serial number, the folder will become part of the device ID.
                // This means that 2 images in different folder will NOT share intrinsics.
                intrinsic->setSerialNumber(fs::path(view.getImagePath()).parent_path().string());
            }
            else if (groupCameraFallback == EGroupCameraFallback::IMAGE)
            {
                // If no serial number, each view will get its own camera intrinsic parameters.
                intrinsic->setSerialNumber(view.getImagePath());
            }
            else if (groupCameraFallback == EGroupCameraFallback::GLOBAL)
            {
                // if no serial number, images with the same make/model/focal or no make/model/focal
                // will be considered as a single group of camera intrinsics.
            }

            if (!make.empty() || !model.empty())
            {
                // We have no correct way to identify a camera device, we fallback on the camera
                // make/model. If you use multiple identical devices, they will be fused together
                // incorrectly.
                intrinsic->setSerialNumber(intrinsic->serialNumber() + "_" + make + "_" + model);
            }

            if (view.isPartOfRig())
            {
                // when we have no unique camera identifier, so for rig images, we ensure that each
                // camera of the rig have different serial numbers.
                intrinsic->setSerialNumber(intrinsic->serialNumber() + "_rig_" + std::to_string(view.getRigId()) + "_" + std::to_string(view.getSubPoseId()));
            }
        }

        // If we have not managed to initialize the focal length, we need to use the focalLength in mm
        if (intrinsic->getScale()(0) <= 0 && focalLengthmm > 0)
        {
            intrinsic->setSerialNumber(intrinsic->serialNumber() + "_FocalLengthMM_" +
                                       std::to_string(focalLengthmm));
        }
    }

    // create intrinsic id
    // group camera that share common properties (leads to more faster & stable BA).
    if(intrinsicId == UndefinedIndexT)
    {
      intrinsicId = intrinsic->hashValue();
    }
    view.setIntrinsicId(intrinsicId);
    return intrinsicBase;
}

} // namespace sfmDataIO
} // namespace aliceVision
