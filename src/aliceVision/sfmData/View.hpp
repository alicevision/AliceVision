// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/types.hpp>

#include <regex>
#include <string>
#include <utility>
#include <aliceVision/numeric/numeric.hpp>

namespace aliceVision {
namespace sfmData {

/**
 * @brief EXIF Orientation to names
 */
enum class EEXIFOrientation
{
  NONE = 1
  , REVERSED = 2
  , UPSIDEDOWN = 3
  , UPSIDEDOWN_REVERSED = 4
  , LEFT_REVERSED = 5
  , LEFT = 6  
  , RIGHT_REVERSED = 7
  , RIGHT = 8
  , UNKNOWN = -1
};

struct GPSExifTags
{
    static std::string latitude();
    static std::string latitudeRef();
    static std::string longitude();
    static std::string longitudeRef();
    static std::string altitude();
    static std::string altitudeRef();
    static std::vector<std::string> all();
};


/**
 * @brief A view define an image by a string and unique indexes for
 * the view, the camera intrinsic, the pose and the subpose if the camera is part of a rig
 */
class View
{
public:

  /**
   * @brief View Constructor
   * @param[in] imagePath The image path on disk
   * @param[in] viewId The view id (use unique index)
   * @param[in] intrinsicId The intrinsic id
   * @param[in] poseId The pose id (or the rig pose id)
   * @param[in] width The image width
   * @param[in] height The image height
   * @param[in] rigId The rig id (or undefined)
   * @param[in] subPoseId The sub-pose id (or undefined)
   * @param[in] metadata The image metadata
   */
  View(const std::string& imagePath = "",
       IndexT viewId = UndefinedIndexT,
       IndexT intrinsicId = UndefinedIndexT,
       IndexT poseId = UndefinedIndexT,
       std::size_t width = 0,
       std::size_t height = 0,
       IndexT rigId = UndefinedIndexT,
       IndexT subPoseId = UndefinedIndexT,
       const std::map<std::string, std::string>& metadata = std::map<std::string, std::string>())
    : _imagePath(imagePath)
    , _width(width)
    , _height(height)
    , _viewId(viewId)
    , _intrinsicId(intrinsicId)
    , _poseId(poseId)
    , _rigId(rigId)
    , _subPoseId(subPoseId)
    , _metadata(metadata)
  {}

  bool operator==(const View& other) const
  {
    // image paths can be different
    return _viewId == other._viewId &&
           _intrinsicId == other._intrinsicId &&
           _poseId == other._poseId &&
           _width == other._width &&
           _height == other._height &&
           _rigId == other._rigId &&
           _subPoseId == other._subPoseId;
  }

  inline bool operator!=(const View& other) const { return !(*this == other); }

  /**
   * @brief Get view image path
   * @return image path
   */
  const std::string& getImagePath() const
  {
    return _imagePath;
  }

  /**
   * @brief Get view image width
   * @return image width
   */
  std::size_t getWidth() const
  {
    return _width;
  }

  /**
   * @brief Get view image height
   * @return image height
   */
  std::size_t getHeight() const
  {
    return _height;
  }

  /**
   * @brief Get view image height
   * @return image height
   */
  std::pair<std::size_t, std::size_t> getImgSize() const
  {
    return {_width, _height};
  }

  /**
   * @brief Get the view id
   * @return view id
   */
  IndexT getViewId() const
  {
    return _viewId;
  }

  /**
   * @brief Get the intrinsic id
   * @return intrinsic id
   */
  IndexT getIntrinsicId() const
  {
    return _intrinsicId;
  }

  /**
   * @brief Get the pose id
   * @return pose id
   */
  IndexT getPoseId() const
  {
    return _poseId;
  }

  /**
   * @brief Get the rig id
   * @return rig id or undefined
   */
  IndexT getRigId() const
  {
    return _rigId;
  }

  /**
   * @brief Get the sub-pose id
   * @return sup-pose id or undefined
   */
  IndexT getSubPoseId() const
  {
    return _subPoseId;
  }

  /**
   * @brief Get the frame id
   * @return frame id
   */
  IndexT getFrameId() const
  {
    return _frameId;
  }

  /**
   * @brief Get the resection id
   * @return resection id
   */
  IndexT getResectionId() const
  {
    return _resectionId;
  }

  /**
   * @brief Return if true or false the view is part of a rig
   * @return true if the view is part of a rig
   */
  bool isPartOfRig() const
  {
    return _rigId != UndefinedIndexT;
  }

  /**
   * @brief If the view is part of a camera rig, the camera can be a sub-pose of the rig pose but can also be temporarily solved independently.
   * @return true if the view is not part of a rig.
   *         true if the view is part of a rig and the camera is solved separately.
   *         false if the view is part of a rig and the camera is solved as a sub-pose of the rig pose.
   */
  bool isPoseIndependant() const
  {
    return (!isPartOfRig() || _isPoseIndependent);
  }

  /**
   * @brief Get the Camera Exposure Setting value.
   * For the same scene, this value is linearly proportional to the amount of light captured by the camera according to
   * the shooting parameters (shutter speed, f-number, iso).
   */
  float getCameraExposureSetting(const float referenceISO = 100.0f, const float referenceFNumber = 1.0f) const;

  /**
   * @brief Get the Exposure Value. EV is a number that represents a combination of a camera's shutter speed and
   * f-number, such that all combinations that yield the same exposure have the same EV.
   * It progresses in a linear sequence as camera exposure is changed in power-of-2 steps.
   */
  float getEv() const;

  /**
   * @brief Get an iterator on the map of metadata from a given name.
   */
  std::map<std::string, std::string>::const_iterator findMetadataIterator(const std::string& name) const;

  /**
   * @brief Return true if the given metadata name exists
   * @param[in] names List of possible names for the metadata
   * @return true if the corresponding metadata value exists
   */
  bool hasMetadata(const std::vector<std::string>& names) const;

  /**
   * @brief Return true if the metadata for longitude and latitude exist.
   * It checks that all the tags from GPSExifTags exists
   * @return true if GPS data is available
   */
  bool hasGpsMetadata() const;

  /**
   * @brief Return true if the given metadata name exists and is a digit
   * @param[in] names List of possible names for the metadata
   * @param[in] isPositive true if the metadata must be positive
   * @return true if the corresponding metadata value exists
   */
  bool hasDigitMetadata(const std::vector<std::string>& names, bool isPositive = true) const;

  /**
   * @brief Get the metadata value as a string
   * @param[in] names List of possible names for the metadata
   * @return the metadata value as a string or an empty string if it does not exist
   */
  const std::string& getMetadata(const std::vector<std::string>& names) const;

  /**
   * @brief Read a floating point value from a string. It support an integer, a floating point value or a fraction.
   * @param[in] str string with the number to evaluate
   * @return the extracted floating point value or -1.0 if it fails to convert the string
   */
  double readRealNumber(const std::string& str) const;

  /**
   * @brief Get the metadata value as a double
   * @param[in] names List of possible names for the metadata
   * @return the metadata value as a double or -1.0 if it does not exist
   */
  double getDoubleMetadata(const std::vector<std::string>& names) const;

  /**
   * @brief Get the metadata value as an integer
   * @param[in] names List of possible names for the metadata
   * @return the metadata value as an integer or -1 if it does not exist
   */
  int getIntMetadata(const std::vector<std::string>& names) const;

  /**
   * @brief Get the corresponding "Make" metadata value
   * @return the metadata value string or "" if no corresponding value
   */
  const std::string& getMetadataMake() const
  {
    return getMetadata({"Make", "cameraMake", "camera make"});
  }

  /**
   * @brief Get the corresponding "Model" metadata value
   * @return the metadata value string or "" if no corresponding value
   */
  const std::string& getMetadataModel() const
  {
      return getMetadata({"Model", "cameraModel", "cameraModelName", "camera model", "camera model name"});
  }

  /**
   * @brief Get the corresponding "BodySerialNumber" metadata value
   * @return the metadata value string or "" if no corresponding value
   */
  const std::string& getMetadataBodySerialNumber() const
  {
    return getMetadata({"Exif:BodySerialNumber", "cameraSerialNumber", "SerialNumber", "Serial Number"});
  }

  /**
   * @brief Get the corresponding "LensSerialNumber" metadata value
   * @return the metadata value string or "" if no corresponding value
   */
  const std::string& getMetadataLensSerialNumber() const
  {
    return getMetadata({"Exif:LensSerialNumber", "lensSerialNumber", "lens serial number"});
  }

  /**
   * @brief Get the corresponding "FocalLength" metadata value
   * @return the metadata value float or -1 if no corresponding value
   */
  double getMetadataFocalLength() const
  {
    return getDoubleMetadata({"Exif:FocalLength", "focalLength", "focal length"});
  }

  /**
   * @brief Get the corresponding "ExposureTime" (shutter) metadata value
   * @return the metadata value float or -1 if no corresponding value
   */
  double getMetadataShutter() const
  {
      return getDoubleMetadata({"ExposureTime", "Shutter Speed Value"});
  }

  /**
   * @brief Get the corresponding "FNumber" (relative aperture) metadata value
   * @return the metadata value float or -1 if no corresponding value
   */
  double getMetadataFNumber() const
  {
      if(hasDigitMetadata({"FNumber"}))
      {
          return getDoubleMetadata({"FNumber"});
      }
      if (hasDigitMetadata({"ApertureValue", "Aperture Value"}))
      {
          const double aperture = getDoubleMetadata({"ApertureValue", "Aperture Value"});
          // fnumber = 2^(aperture/2)
          return std::pow(2.0, aperture / 2.0);
      }
      return -1;
  }

  /**
     * @brief Get the corresponding "PhotographicSensitivity" (ISO) metadata value
     * @return the metadata value int or -1 if no corresponding value
     */
  double getMetadataISO() const
  {
    return getDoubleMetadata({"Exif:PhotographicSensitivity", "PhotographicSensitivity", "Photographic Sensitivity", "ISO"});
  }

  /**
   * @brief Get the corresponding "Orientation" metadata value
   * @return the enum EEXIFOrientation
   */
  EEXIFOrientation getMetadataOrientation() const
  {
    const int orientation = getIntMetadata({"Exif:Orientation", "Orientation"});
    if(orientation < 0)
      return EEXIFOrientation::UNKNOWN;
    return static_cast<EEXIFOrientation>(orientation);
  }

  /**
   * @brief Get the gps position in the absolute cartesian reference system.
   * @return The position x, y, z as a three dimensional vector.
   */
  Vec3 getGpsPositionFromMetadata() const;

  /**
   * @brief Get the gps position in the WGS84 reference system.
   * @param[out] lat the latitude
   * @param[out] lon the longitude
   * @param[out] alt the altitude
   */
  void getGpsPositionWGS84FromMetadata(double& lat, double& lon, double& alt) const;

  /**
   * @brief Get the gps position in the WGS84 reference system as a vector.
   * @return A three dimensional vector with latitude, logitude and altitude.
   */
  Vec3 getGpsPositionWGS84FromMetadata() const;

  const bool getApplyWhiteBalance() const 
  {
    if (getIntMetadata({"AliceVision:useWhiteBalance"}) == 0)
    {
      return false;
    }
    
    return true;
  }

  const bool hasMetadataDateTimeOriginal() const
  {
      return hasMetadata(
          {"Exif:DateTimeOriginal", "DateTimeOriginal", "DateTime", "Date Time", "Create Date", "ctime"});
  }

  const std::string& getMetadataDateTimeOriginal() const
  {
    return getMetadata({"Exif:DateTimeOriginal", "DateTimeOriginal", "DateTime", "Date Time", "Create Date", "ctime"});
  }

  int64_t getMetadataDateTimestamp() const {

    std::smatch sm;
    std::string dtstring = getMetadataDateTimeOriginal();
    std::regex regex("([\\d]+):([\\d]+):([\\d]+) ([\\d]+):([\\d]+):([\\d]+)");
    
    if (!std::regex_match(dtstring, sm, regex)) {
      return -1;
    }
    
    int64_t year = std::stoi(sm[1]);
    int64_t month = std::stoi(sm[2]);
    int64_t day = std::stoi(sm[3]);
    int64_t hour = std::stoi(sm[4]);
    int64_t minutes = std::stoi(sm[5]);
    int64_t seconds = std::stoi(sm[6]);
    int64_t timecode = ((((((((year * 12) + month) * 31) + day) * 24) + hour) * 60 + minutes) * 60) + seconds;

    return timecode;
  }

  /**
   * @brief Get the view metadata structure
   * @return the view metadata
   */
  const std::map<std::string, std::string>& getMetadata() const
  {
    return _metadata;
  }

  /**
   * @brief Set the given view image path
   * @param[in] imagePath The given view image path
   */
  void setImagePath(const std::string& imagePath)
  {
    _imagePath = imagePath;
  }

  /**
   * @brief  Set the given view image width
   * @param[in] width The given view image width
   */
  void setWidth(std::size_t width)
  {
    _width = width;
  }

  /**
   * @brief  Set the given view image height
   * @param[in] height The given view image height
   */
  void setHeight(std::size_t height)
  {
    _height = height;
  }

  /**
   * @brief Set the given view id
   * @param[in] viewId The given view id
   */
  void setViewId(IndexT viewId)
  {
    _viewId = viewId;
  }

  /**
   * @brief Set the given intrinsic id
   * @param[in] intrinsicId The given intrinsic id
   */
  void setIntrinsicId(IndexT intrinsicId)
  {
    _intrinsicId = intrinsicId;
  }

  /**
   * @brief Set the given pose id
   * @param[in] poseId The given pose id
   */
  void setPoseId(IndexT poseId)
  {
    _poseId = poseId;
  }

  /**
   * @brief setIndependantPose
   * @param independant
   */
  void setIndependantPose(bool independent)
  {
      _isPoseIndependent = independent;
  }

  /**
   * @brief Set the given rig id and the given sub-pose id
   * @param[in] rigId The given rig id
   * @param[in] subPoseId The given sub-pose id
   */
  void setRigAndSubPoseId(IndexT rigId, IndexT subPoseId)
  {
    _rigId = rigId;
    _subPoseId = subPoseId;
  }


  /**
   * @brief Set the given frame id
   * @param[in] frame The given frame id
   */
  void setFrameId(IndexT frameId)
  {
    _frameId = frameId;
  }

  /**
   * @brief Set the given resection id
   * @param[in] resectionId The given resection id
   */
  void setResectionId(IndexT resectionId)
  {
    _resectionId = resectionId;
  }

  /**
   * @brief Set view metadata
   * @param[in] metadata The metadata map
   */
  void setMetadata(const std::map<std::string, std::string>& metadata)
  {
    _metadata = metadata;
  }

  /**
   * @brief Add view metadata
   * @param[in] key The metadata key
   * @param[in] value The metadata value
   */
  void addMetadata(const std::string& key, const std::string& value)
  {
    _metadata[key] = value;
  }

private:

  /// image path on disk
  std::string _imagePath;
  /// image width
  std::size_t _width;
  /// image height
  std::size_t _height;
  /// view id
  IndexT _viewId;
  /// intrinsics id
  IndexT _intrinsicId;
  /// either the pose of the rig or the pose of the camera if there's no rig
  IndexT _poseId;
  /// corresponding rig id or undefined
  IndexT _rigId;
  /// corresponding sub-pose id or undefined
  IndexT _subPoseId;
  /// corresponding frame id for synchronized views
  IndexT _frameId = UndefinedIndexT;
  /// resection id
  IndexT _resectionId = UndefinedIndexT;
  /// pose independent of other view(s)
  bool _isPoseIndependent = true;
  /// map for metadata
  std::map<std::string, std::string> _metadata;
};

} // namespace sfmData
} // namespace aliceVision
