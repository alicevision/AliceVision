// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/types.hpp>

#include <aliceVision/image/dcp.hpp>
#include <aliceVision/lensCorrectionProfile/lcp.hpp>
#include <aliceVision/sfmData/ExposureSetting.hpp>
#include <aliceVision/sfmData/exif.hpp>
#include <aliceVision/sensorDB/Datasheet.hpp>
#include <aliceVision/camera/IntrinsicInitMode.hpp>

#include <regex>

namespace aliceVision {
namespace sfmData {

class ImageInfo
{
  public:
    /**
     * @brief Image Constructor
     * @param[in] imagePath The image path on disk
     * @param[in] width The image width
     * @param[in] height The image height
     * @param[in] metadata The image metadata
     */
    ImageInfo(const std::string& imagePath = "",
              std::size_t width = 0,
              std::size_t height = 0,
              const std::map<std::string, std::string>& metadata = std::map<std::string, std::string>())
      : _imagePath(imagePath),
        _width(width),
        _height(height),
        _metadata(metadata)
    {}

    bool operator==(const ImageInfo& other) const
    {
        // image paths can be different
        return _width == other._width && _height == other._height;
    }

    inline bool operator!=(const ImageInfo& other) const { return !(*this == other); }

    /**
     * @brief Get view image path
     * @return image path
     */
    const std::string& getImagePath() const { return _imagePath; }

    /**
     * @brief Get view image width
     * @return image width
     */
    std::size_t getWidth() const { return _width; }

    /**
     * @brief Get view image height
     * @return image height
     */
    std::size_t getHeight() const { return _height; }

    /**
     * @brief Get view image size
     * @return image size
     */
    std::pair<std::size_t, std::size_t> getImgSize() const { return {_width, _height}; }

    /**
     * @brief Get the Camera Exposure Setting value.
     * For the same scene, this value is linearly proportional to the amount of light captured by the camera according to
     * the shooting parameters (shutter speed, f-number, iso).
     */
    ExposureSetting getCameraExposureSetting() const { return ExposureSetting(getMetadataShutter(), getMetadataFNumber(), getMetadataISO()); }

    /**
     * @brief Get the Exposure Value. EV is a number that represents a combination of a camera's shutter speed and
     * f-number, such that all combinations that yield the same exposure have the same EV.
     * It progresses in a linear sequence as camera exposure is changed in power-of-2 steps.
     */
    double getEv() const;

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
     * @brief Get the metadata value as a double
     * @param[in] names List of possible names for the metadata
     * @param[in] val Data to be set with the metadata value
     * @return true if the metadata is found or false if it does not exist
     */
    bool getDoubleMetadata(const std::vector<std::string>& names, double& val) const;

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
    const std::string& getMetadataMake() const { return getMetadata({"Make", "cameraMake", "camera make"}); }

    /**
     * @brief Get the corresponding "Model" metadata value
     * @return the metadata value string or "" if no corresponding value
     */
    const std::string& getMetadataModel() const
    {
        return getMetadata({"Model", "cameraModel", "cameraModelName", "CameraModel", "camera model", "camera_model", "camera model name"});
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
     * @brief Get the corresponding "LensModel" metadata value
     * @return the metadata value string or "" if no corresponding value
     */
    const std::string& getMetadataLensModel() const { return getMetadata({"Exif:LensModel", "lensModel", "lens model"}); }

    /**
     * @brief Get the corresponding "LensID" metadata value
     * @return the metadata value -1 if no corresponding value
     */
    int getMetadataLensID() const { return getIntMetadata({"Exif:LensID", "lensID", "lensType"}); }

    /**
     * @brief Get the corresponding "LensSerialNumber" metadata value
     * @return the metadata value string or "" if no corresponding value
     */
    const std::string& getMetadataLensSerialNumber() const
    {
        return getMetadata({"Exif:LensSerialNumber", "lensSerialNumber", "lens serial number", "lens_serial_number"});
    }

    /**
     * @brief Get the corresponding "FocalLength" metadata value
     * @return the metadata value float or -1 if no corresponding value
     */
    double getMetadataFocalLength() const
    {
        double focalLength = getDoubleMetadata({"Exif:FocalLength", "focalLength", "focal length", "lens_focal_length"});

        if (focalLength == -1)
        {
            // Sony metadata: the focal length is provided in meters
            focalLength = getDoubleMetadata({"LensZoomActualFocalLength"});
            if (focalLength != -1)
            {
                focalLength *= 1000;
            }
        }

        // 32767 = (2^15 - 1) - Maximum of a signed short: means there is no available focal length
        // 4294967295 = (2^32 - 1) - Maximum of a signed integer: means there is no available focal length
        //   -> might be truncated and/or rounded up to 4.29497e+06
        else if (focalLength == USHRT_MAX || focalLength == SHRT_MAX || focalLength == UINT_MAX || focalLength == INT_MAX || focalLength == 4294970)
        {
            focalLength = -1;
        }

        // Might be available and more precise (especially if the focal length was initially retrieved from a string)
        double nominativeFocalLength = getDoubleMetadata({"AxialNominalFocalLength"});
        if (nominativeFocalLength != -1)
        {
            focalLength = nominativeFocalLength;
        }

        return focalLength;
    }

    /**
     * @brief Get the corresponding "ExposureTime" (shutter) metadata value
     * @return the metadata value float or -1 if no corresponding value
     */
    double getMetadataShutter() const { return getDoubleMetadata({"ExposureTime", "Shutter Speed Value"}); }

    /**
     * @brief Get the corresponding "FNumber" (relative aperture) metadata value
     * @return the metadata value float or -1 if no corresponding value
     */
    double getMetadataFNumber() const
    {
        if (hasDigitMetadata({"FNumber"}))
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
        if (orientation < 0)
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

    const std::string& getColorProfileFileName() const { return getMetadata({"AliceVision:DCP:colorProfileFileName"}); }

    const std::string& getRawColorInterpretation() const { return getMetadata({"AliceVision:rawColorInterpretation"}); }

    const std::vector<int> getCameraMultiplicators() const
    {
        const std::string cam_mul = getMetadata({"raw:cam_mul"});
        std::vector<int> v_mult;

        size_t last = 0;
        size_t next = 0;
        while ((next = cam_mul.find(" ", last)) != std::string::npos)
        {
            v_mult.push_back(std::stoi(cam_mul.substr(last, next - last)));
            last = next + 1;
        }
        v_mult.push_back(std::stoi(cam_mul.substr(last)));

        return v_mult;
    }

    const bool getVignettingParams(std::vector<float>& v_vignParam) const
    {
        v_vignParam.clear();
        bool valid = true;
        double value;

        valid = valid && getDoubleMetadata({"AliceVision:VignParamFocX"}, value);
        v_vignParam.push_back(static_cast<float>(value));
        valid = valid && getDoubleMetadata({"AliceVision:VignParamFocY"}, value);
        v_vignParam.push_back(static_cast<float>(value));
        valid = valid && getDoubleMetadata({"AliceVision:VignParamCenterX"}, value);
        v_vignParam.push_back(static_cast<float>(value));
        valid = valid && getDoubleMetadata({"AliceVision:VignParamCenterY"}, value);
        v_vignParam.push_back(static_cast<float>(value));
        valid = valid && getDoubleMetadata({"AliceVision:VignParam1"}, value);
        v_vignParam.push_back(static_cast<float>(value));
        valid = valid && getDoubleMetadata({"AliceVision:VignParam2"}, value);
        v_vignParam.push_back(static_cast<float>(value));
        valid = valid && getDoubleMetadata({"AliceVision:VignParam3"}, value);
        v_vignParam.push_back(static_cast<float>(value));
        return valid;
    }

    const bool getChromaticAberrationParams(std::vector<float>& v_caGParam, std::vector<float>& v_caBGParam, std::vector<float>& v_caRGParam) const
    {
        v_caGParam.clear();
        v_caBGParam.clear();
        v_caRGParam.clear();
        bool valid = true;
        double value;

        valid = valid && getDoubleMetadata({"AliceVision:CAGreenFocX"}, value);
        v_caGParam.push_back(static_cast<float>(value));
        valid = valid && getDoubleMetadata({"AliceVision:CAGreenFocY"}, value);
        v_caGParam.push_back(static_cast<float>(value));
        valid = valid && getDoubleMetadata({"AliceVision:CAGreenCenterX"}, value);
        v_caGParam.push_back(static_cast<float>(value));
        valid = valid && getDoubleMetadata({"AliceVision:CAGreenCenterY"}, value);
        v_caGParam.push_back(static_cast<float>(value));
        valid = valid && getDoubleMetadata({"AliceVision:CAGreenParam1"}, value);
        v_caGParam.push_back(static_cast<float>(value));
        valid = valid && getDoubleMetadata({"AliceVision:CAGreenParam2"}, value);
        v_caGParam.push_back(static_cast<float>(value));
        valid = valid && getDoubleMetadata({"AliceVision:CAGreenParam3"}, value);
        v_caGParam.push_back(static_cast<float>(value));
        valid = valid && getDoubleMetadata({"AliceVision:CABlueGreenFocX"}, value);
        v_caBGParam.push_back(static_cast<float>(value));
        valid = valid && getDoubleMetadata({"AliceVision:CABlueGreenFocY"}, value);
        v_caBGParam.push_back(static_cast<float>(value));
        valid = valid && getDoubleMetadata({"AliceVision:CABlueGreenCenterX"}, value);
        v_caBGParam.push_back(static_cast<float>(value));
        valid = valid && getDoubleMetadata({"AliceVision:CABlueGreenCenterY"}, value);
        v_caBGParam.push_back(static_cast<float>(value));
        valid = valid && getDoubleMetadata({"AliceVision:CABlueGreenParam1"}, value);
        v_caBGParam.push_back(static_cast<float>(value));
        valid = valid && getDoubleMetadata({"AliceVision:CABlueGreenParam2"}, value);
        v_caBGParam.push_back(static_cast<float>(value));
        valid = valid && getDoubleMetadata({"AliceVision:CABlueGreenParam3"}, value);
        v_caBGParam.push_back(static_cast<float>(value));
        valid = valid && getDoubleMetadata({"AliceVision:CABlueGreenScaleFactor"}, value);
        v_caBGParam.push_back(static_cast<float>(value));
        valid = valid && getDoubleMetadata({"AliceVision:CARedGreenFocX"}, value);
        v_caRGParam.push_back(static_cast<float>(value));
        valid = valid && getDoubleMetadata({"AliceVision:CARedGreenFocY"}, value);
        v_caRGParam.push_back(static_cast<float>(value));
        valid = valid && getDoubleMetadata({"AliceVision:CARedGreenCenterX"}, value);
        v_caRGParam.push_back(static_cast<float>(value));
        valid = valid && getDoubleMetadata({"AliceVision:CARedGreenCenterY"}, value);
        v_caRGParam.push_back(static_cast<float>(value));
        valid = valid && getDoubleMetadata({"AliceVision:CARedGreenParam1"}, value);
        v_caRGParam.push_back(static_cast<float>(value));
        valid = valid && getDoubleMetadata({"AliceVision:CARedGreenParam2"}, value);
        v_caRGParam.push_back(static_cast<float>(value));
        valid = valid && getDoubleMetadata({"AliceVision:CARedGreenParam3"}, value);
        v_caRGParam.push_back(static_cast<float>(value));
        valid = valid && getDoubleMetadata({"AliceVision:CARedGreenScaleFactor"}, value);
        v_caRGParam.push_back(static_cast<float>(value));

        return valid;
    }

    const bool hasMetadataDateTimeOriginal() const
    {
        return hasMetadata({"Exif:DateTimeOriginal", "DateTimeOriginal", "DateTime", "Date Time", "Create Date", "ctime"});
    }

    const std::string& getMetadataDateTimeOriginal() const
    {
        return getMetadata({"Exif:DateTimeOriginal", "DateTimeOriginal", "DateTime", "Date Time", "Create Date", "ctime"});
    }

    int64_t getMetadataDateTimestamp() const
    {
        std::smatch sm;
        std::string dtstring = getMetadataDateTimeOriginal();
        std::regex regex("([\\d]+):([\\d]+):([\\d]+) ([\\d]+):([\\d]+):([\\d]+)");

        if (!std::regex_match(dtstring, sm, regex))
        {
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

    double getSensorWidth() const { return getDoubleMetadata({"AliceVision:SensorWidth"}); }

    double getSensorHeight() const { return getDoubleMetadata({"AliceVision:SensorHeight"}); }

    /**
     * @brief Get the view metadata structure
     * @return the view metadata
     */
    const std::map<std::string, std::string>& getMetadata() const { return _metadata; }

    /**
     * @brief Set the given view image path
     * @param[in] imagePath The given view image path
     */
    void setImagePath(const std::string& imagePath) { _imagePath = imagePath; }

    /**
     * @brief  Set the given view image width
     * @param[in] width The given view image width
     */
    void setWidth(std::size_t width) { _width = width; }

    /**
     * @brief  Set the given view image height
     * @param[in] height The given view image height
     */
    void setHeight(std::size_t height) { _height = height; }

    /**
     * @brief Set view metadata
     * @param[in] metadata The metadata map
     */
    void setMetadata(const std::map<std::string, std::string>& metadata) { _metadata = metadata; }

    /**
     * @brief Add view metadata
     * @param[in] key The metadata key
     * @param[in] value The metadata value
     */
    void addMetadata(const std::string& key, const std::string& value) { _metadata[key] = value; }

    /**
     * @brief Add DCP info in metadata
     * @param[in] dcpProf The DCP color profile
     */
    void addDCPMetadata(image::DCPProfile& dcpProf)
    {
        addMetadata("AliceVision:DCP:colorProfileFileName", dcpProf.info.filename);

        addMetadata("AliceVision:DCP:Temp1", std::to_string(dcpProf.info.temperature_1));
        addMetadata("AliceVision:DCP:Temp2", std::to_string(dcpProf.info.temperature_2));

        const int colorMatrixNumber =
          (dcpProf.info.has_color_matrix_1 && dcpProf.info.has_color_matrix_2) ? 2 : (dcpProf.info.has_color_matrix_1 ? 1 : 0);
        addMetadata("AliceVision:DCP:ColorMatrixNumber", std::to_string(colorMatrixNumber));

        const int forwardMatrixNumber =
          (dcpProf.info.has_forward_matrix_1 && dcpProf.info.has_forward_matrix_2) ? 2 : (dcpProf.info.has_forward_matrix_1 ? 1 : 0);
        addMetadata("AliceVision:DCP:ForwardMatrixNumber", std::to_string(forwardMatrixNumber));

        const int calibMatrixNumber =
          (dcpProf.info.has_camera_calibration_1 && dcpProf.info.has_camera_calibration_2) ? 2 : (dcpProf.info.has_camera_calibration_1 ? 1 : 0);
        addMetadata("AliceVision:DCP:CameraCalibrationMatrixNumber", std::to_string(calibMatrixNumber));

        std::vector<std::string> v_strColorMatrix;
        dcpProf.getMatricesAsStrings("color", v_strColorMatrix);
        for (int k = 0; k < v_strColorMatrix.size(); k++)
        {
            addMetadata("AliceVision:DCP:ColorMat" + std::to_string(k + 1), v_strColorMatrix[k]);
        }

        std::vector<std::string> v_strForwardMatrix;
        dcpProf.getMatricesAsStrings("forward", v_strForwardMatrix);
        for (int k = 0; k < v_strForwardMatrix.size(); k++)
        {
            addMetadata("AliceVision:DCP:ForwardMat" + std::to_string(k + 1), v_strForwardMatrix[k]);
        }

        std::vector<std::string> v_strCalibMatrix;
        dcpProf.getMatricesAsStrings("calib", v_strCalibMatrix);
        for (int k = 0; k < v_strCalibMatrix.size(); k++)
        {
            addMetadata("AliceVision:DCP:CameraCalibrationMat" + std::to_string(k + 1), v_strCalibMatrix[k]);
        }
    }

    /**
     * @brief Add vignetting model parameters in metadata
     * @param[in] The lens data extracted from a LCP file
     */
    void addVignettingMetadata(LensParam& lensParam)
    {
        addMetadata("AliceVision:VignParamFocX", std::to_string(lensParam.vignParams.FocalLengthX));
        addMetadata("AliceVision:VignParamFocY", std::to_string(lensParam.vignParams.FocalLengthY));
        addMetadata("AliceVision:VignParamCenterX", std::to_string(lensParam.vignParams.ImageXCenter));
        addMetadata("AliceVision:VignParamCenterY", std::to_string(lensParam.vignParams.ImageYCenter));
        addMetadata("AliceVision:VignParam1", std::to_string(lensParam.vignParams.VignetteModelParam1));
        addMetadata("AliceVision:VignParam2", std::to_string(lensParam.vignParams.VignetteModelParam2));
        addMetadata("AliceVision:VignParam3", std::to_string(lensParam.vignParams.VignetteModelParam3));
    }

    /**
     * @brief Add chromatic model parameters in metadata
     * @param[in] The lens data extracted from a LCP file
     */
    void addChromaticMetadata(LensParam& lensParam)
    {
        addMetadata("AliceVision:CAGreenFocX", std::to_string(lensParam.ChromaticGreenParams.FocalLengthX));
        addMetadata("AliceVision:CAGreenFocY", std::to_string(lensParam.ChromaticGreenParams.FocalLengthY));
        addMetadata("AliceVision:CAGreenCenterX", std::to_string(lensParam.ChromaticGreenParams.ImageXCenter));
        addMetadata("AliceVision:CAGreenCenterY", std::to_string(lensParam.ChromaticGreenParams.ImageYCenter));
        addMetadata("AliceVision:CAGreenParam1", std::to_string(lensParam.ChromaticGreenParams.RadialDistortParam1));
        addMetadata("AliceVision:CAGreenParam2", std::to_string(lensParam.ChromaticGreenParams.RadialDistortParam2));
        addMetadata("AliceVision:CAGreenParam3", std::to_string(lensParam.ChromaticGreenParams.RadialDistortParam3));
        addMetadata("AliceVision:CABlueGreenFocX", std::to_string(lensParam.ChromaticBlueGreenParams.FocalLengthX));
        addMetadata("AliceVision:CABlueGreenFocY", std::to_string(lensParam.ChromaticBlueGreenParams.FocalLengthY));
        addMetadata("AliceVision:CABlueGreenCenterX", std::to_string(lensParam.ChromaticBlueGreenParams.ImageXCenter));
        addMetadata("AliceVision:CABlueGreenCenterY", std::to_string(lensParam.ChromaticBlueGreenParams.ImageYCenter));
        addMetadata("AliceVision:CABlueGreenParam1", std::to_string(lensParam.ChromaticBlueGreenParams.RadialDistortParam1));
        addMetadata("AliceVision:CABlueGreenParam2", std::to_string(lensParam.ChromaticBlueGreenParams.RadialDistortParam2));
        addMetadata("AliceVision:CABlueGreenParam3", std::to_string(lensParam.ChromaticBlueGreenParams.RadialDistortParam3));
        addMetadata("AliceVision:CABlueGreenScaleFactor", std::to_string(lensParam.ChromaticBlueGreenParams.ScaleFactor));
        addMetadata("AliceVision:CARedGreenFocX", std::to_string(lensParam.ChromaticRedGreenParams.FocalLengthX));
        addMetadata("AliceVision:CARedGreenFocY", std::to_string(lensParam.ChromaticRedGreenParams.FocalLengthY));
        addMetadata("AliceVision:CARedGreenCenterX", std::to_string(lensParam.ChromaticRedGreenParams.ImageXCenter));
        addMetadata("AliceVision:CARedGreenCenterY", std::to_string(lensParam.ChromaticRedGreenParams.ImageYCenter));
        addMetadata("AliceVision:CARedGreenParam1", std::to_string(lensParam.ChromaticRedGreenParams.RadialDistortParam1));
        addMetadata("AliceVision:CARedGreenParam2", std::to_string(lensParam.ChromaticRedGreenParams.RadialDistortParam2));
        addMetadata("AliceVision:CARedGreenParam3", std::to_string(lensParam.ChromaticRedGreenParams.RadialDistortParam3));
        addMetadata("AliceVision:CARedGreenScaleFactor", std::to_string(lensParam.ChromaticRedGreenParams.ScaleFactor));
    }

    /**
     * @brief Get sensor size by combining info in metadata and in sensor database
     * @param[in] sensorDatabase The sensor database
     * @param[out] sensorWidth The sensor width
     * @param[out] sensorHeight The sensor height
     * @param[out] focalLengthmm The focal length
     * @param[out] intrinsicInitMode The intrinsic init mode
     * @param[in] verbose Enable verbosity
     * @return An Error or Warning code: 1 - Unknown sensor, 2 - No metadata, 3 - Unsure sensor, 4 - Computation from 35mm Focal
     */
    int getSensorSize(const std::vector<sensorDB::Datasheet>& sensorDatabase,
                      double& sensorWidth,
                      double& sensorHeight,
                      double& focalLengthmm,
                      camera::EInitMode& intrinsicInitMode,
                      bool verbose = false);

  private:
    /// image path on disk
    std::string _imagePath;
    /// image width
    std::size_t _width;
    /// image height
    std::size_t _height;
    /// map for metadata
    std::map<std::string, std::string> _metadata;
};

}  // namespace sfmData
}  // namespace aliceVision
