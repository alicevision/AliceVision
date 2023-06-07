// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "View.hpp"

#include <boost/algorithm/string.hpp>

#include <string>
#include <utility>
#include <regex>

#include <iostream>
#include <aliceVision/numeric/gps.hpp>
#include <aliceVision/sensorDB/parseDatabase.hpp>
#include <aliceVision/sfmDataIO/viewIO.hpp>

namespace aliceVision {
namespace sfmData {

double View::getEv() const
{
    return std::log2(1.0 / getCameraExposureSetting().getExposure());
}

std::map<std::string, std::string>::const_iterator View::findMetadataIterator(const std::string& name) const
{
    auto it = _metadata.find(name);
    if(it != _metadata.end())
        return it;
    std::string nameLower = name;
    boost::algorithm::to_lower(nameLower);
    for(auto mIt = _metadata.begin(); mIt != _metadata.end(); ++mIt)
    {
        std::string key = mIt->first;
        boost::algorithm::to_lower(key);
        if(key.size() > name.size())
        {
            auto delimiterIt = key.find_last_of("/:");
            if(delimiterIt != std::string::npos)
                key = key.substr(delimiterIt + 1);
        }
        if(key == nameLower)
        {
            return mIt;
        }
    }
    return _metadata.end();
}

bool View::hasMetadata(const std::vector<std::string>& names) const
{
    return std::any_of(names.cbegin(), names.cend(),
                       [this](const std::string& name){
                           return (findMetadataIterator(name) != _metadata.end());
                       });
}

bool View::hasDigitMetadata(const std::vector<std::string>& names, bool isPositive) const
{
    bool hasDigitMetadata = false;
    double value = -1.0;

    for(const std::string& name : names)
    {
        const auto it = findMetadataIterator(name);

        if(it == _metadata.end() || it->second.empty())
        {
            continue;
        }

        try
        {
            value = std::stod(it->second);
            hasDigitMetadata = true;
            break;
        }
        catch(std::exception&)
        {
            value = -1.0;
        }
    }
    return hasDigitMetadata && (!isPositive || (value > 0));
}

const std::string& View::getMetadata(const std::vector<std::string>& names) const
{
    static const std::string emptyString;
    for(const std::string& name : names)
    {
        const auto it = findMetadataIterator(name);
        if(it != _metadata.end())
            return it->second;
    }
    return emptyString;
}

double View::readRealNumber(const std::string& str) const
{
    try
    {
        std::smatch m;
        std::regex pattern_frac("([0-9]+)\\/([0-9]+)");

        if(!std::regex_search(str, m, pattern_frac))
        {
            return std::stod(str);
        }

        const int num = std::stoi(m[1].str());
        const int denum = std::stoi(m[2].str());

        if(denum != 0)
            return double(num) / double(denum);
        else
            return 0.0;
    }
    catch(std::exception&)
    {
        return -1.0;
    }
}

double View::getDoubleMetadata(const std::vector<std::string>& names) const
{
    const std::string value = getMetadata(names);
    if (value.empty())
        return -1.0;
    return readRealNumber(value);
}

bool View::getDoubleMetadata(const std::vector<std::string>& names, double& val) const
{
    const std::string value = getMetadata(names);
    if (value.empty())
        return false;
    val = readRealNumber(value);
    return true;
}

int View::getIntMetadata(const std::vector<std::string>& names) const
{
    const std::string value = getMetadata(names);
    if(value.empty())
        return -1;
    try
    {
        return std::stoi(value);
    }
    catch(std::exception&)
    {
        return -1;
    }
}

bool View::hasGpsMetadata() const
{
    const auto tags = GPSExifTags::all();
    return std::all_of(tags.cbegin(), tags.cend(), [this](const std::string& t){ return hasMetadata({t}); });
}

Vec3 View::getGpsPositionFromMetadata() const
{
    const auto gps = getGpsPositionWGS84FromMetadata();
    return WGS84ToCartesian(gps);
}

void View::getGpsPositionWGS84FromMetadata(double& lat, double& lon, double& alt) const
{
    const auto& meta = getMetadata();
    const auto gpsLat = meta.at(GPSExifTags::latitude());
    const auto gpsLatRef = meta.at(GPSExifTags::latitudeRef());
    lat = parseGPSFromString(gpsLat, gpsLatRef);

    const auto gpsLon = meta.at(GPSExifTags::longitude());
    const auto gpsLonRef = meta.at(GPSExifTags::longitudeRef());
    lon = parseGPSFromString(gpsLon, gpsLonRef);

    const auto gpsAlt = meta.at(GPSExifTags::altitude());
    const auto gpsAltRef = meta.at(GPSExifTags::altitudeRef());
    alt = parseAltitudeFromString(gpsAlt, gpsAltRef);
}

Vec3 View::getGpsPositionWGS84FromMetadata() const
{
    double lat{0};
    double lon{0};
    double alt{0};
    getGpsPositionWGS84FromMetadata(lat, lon, alt);

    return {lat, lon, alt};
}

int View::getSensorSize(const std::vector<sensorDB::Datasheet>& sensorDatabase, double& sensorWidth, double& sensorHeight,
                        double& focalLengthmm, camera::EInitMode& intrinsicInitMode, bool verbose)
{
    int errCode = 0;

    enum class ESensorWidthSource
    {
        FROM_DB,
        FROM_METADATA_ESTIMATION,
        UNKNOWN
    } sensorWidthSource = ESensorWidthSource::UNKNOWN;

    const std::string& make = getMetadataMake();
    const std::string& model = getMetadataModel();
    focalLengthmm = getMetadataFocalLength();
    const bool hasCameraMetadata = (!make.empty() || !model.empty());

    if (hasCameraMetadata)
    {
        intrinsicInitMode = camera::EInitMode::UNKNOWN;
        sensorDB::Datasheet datasheet;
        if (sensorDB::getInfo(make, model, sensorDatabase, datasheet))
        {
            if (verbose)
            {
                // sensor is in the database
                ALICEVISION_LOG_TRACE("Sensor width found in sensor database: " << std::endl
                                                                                << "\t- brand: " << make << std::endl
                                                                                << "\t- model: " << model << std::endl
                                                                                << "\t- sensor width: " << datasheet._sensorWidth << " mm");
            }

            if ((datasheet._model != model) && (datasheet._model != make + " " + model))
            {
                // the camera model in sensor database is slightly different
                errCode = 3;

                if (verbose)
                {
                    ALICEVISION_LOG_WARNING("The camera found in the sensor database is slightly different for image " << getImagePath());
                    ALICEVISION_LOG_WARNING("\t- image camera brand: " << make << std::endl
                                                                       << "\t- image camera model: " << model << std::endl
                                                                       << "\t- sensor database camera brand: " << datasheet._brand << std::endl
                                                                       << "\t- sensor database camera model: " << datasheet._model << std::endl
                                                                       << "\t- sensor database camera sensor width: " << datasheet._sensorWidth << " mm");
                    ALICEVISION_LOG_WARNING("Please check and correct camera model(s) name in the sensor database." << std::endl);
                }
            }

            sensorWidth = datasheet._sensorWidth;
            sensorWidthSource = ESensorWidthSource::FROM_DB;

            if (focalLengthmm > 0.0)
            {
                intrinsicInitMode = camera::EInitMode::ESTIMATED;
            }
        }
    }

    // try to find / compute with 'FocalLengthIn35mmFilm' metadata
    const bool hasFocalIn35mmMetadata = hasDigitMetadata({"Exif:FocalLengthIn35mmFilm", "FocalLengthIn35mmFilm"});
    if (hasFocalIn35mmMetadata)
    {
        const double imageRatio = static_cast<double>(getWidth()) / static_cast<double>(getHeight());
        const double diag24x36 = std::sqrt(36.0 * 36.0 + 24.0 * 24.0);
        const double focalIn35mm = hasFocalIn35mmMetadata ? getDoubleMetadata({"Exif:FocalLengthIn35mmFilm", "FocalLengthIn35mmFilm"}) : -1.0;

        if (sensorWidth == -1.0)
        {
            const double invRatio = 1.0 / imageRatio;

            if (focalLengthmm > 0.0)
            {
                // no sensorWidth but valid focalLength and valid focalLengthIn35mm, so deduce
                // sensorWith approximation
                const double sensorDiag = (focalLengthmm * diag24x36) / focalIn35mm; // 43.3 is the diagonal of 35mm film
                sensorWidth = sensorDiag * std::sqrt(1.0 / (1.0 + invRatio * invRatio));
                sensorWidthSource = ESensorWidthSource::FROM_METADATA_ESTIMATION;
            }
            else
            {
                // no sensorWidth and no focalLength but valid focalLengthIn35mm, so consider sensorWith
                // as 35mm
                sensorWidth = diag24x36 * std::sqrt(1.0 / (1.0 + invRatio * invRatio));
                focalLengthmm = sensorWidth * (focalIn35mm) / 36.0;
                sensorWidthSource = ESensorWidthSource::UNKNOWN;
            }
            errCode = 4;

            if (verbose)
            {
                std::stringstream ss;
                ss << "Intrinsic(s) initialized from 'FocalLengthIn35mmFilm' exif metadata in image " << getImagePath() << "\n";
                ss << "\t- sensor width: " << sensorWidth << "\n";
                ss << "\t- focal length: " << focalLengthmm << "\n";
                ALICEVISION_LOG_DEBUG(ss.str());
            }

            sensorHeight = (imageRatio > 1.0) ? sensorWidth / imageRatio : sensorWidth * imageRatio;

            intrinsicInitMode = camera::EInitMode::ESTIMATED;
        }
        else if (sensorWidth > 0 && focalLengthmm <= 0)
        {
            // valid sensorWidth and valid focalLengthIn35mm but no focalLength, so convert
            // focalLengthIn35mm to the actual width of the sensor
            const double sensorDiag = std::sqrt(std::pow(sensorWidth, 2) + std::pow(sensorWidth / imageRatio, 2));
            focalLengthmm = (sensorDiag * focalIn35mm) / diag24x36;

            errCode = 4;

            if (verbose)
            {
                std::stringstream ss;
                ss << "Intrinsic(s) initialized from 'FocalLengthIn35mmFilm' exif metadata in image " << getImagePath() << "\n";
                ss << "\t- sensor width: " << sensorWidth << "\n";
                ss << "\t- focal length: " << focalLengthmm << "\n";
                ALICEVISION_LOG_DEBUG(ss.str());
            }

            intrinsicInitMode = camera::EInitMode::ESTIMATED;
        }
    }

    // error handling
    if (sensorWidth == -1.0)
    {
        if (hasCameraMetadata)
        {
            // sensor is not in the database
            errCode = 1;
            if (verbose)
            {
                std::stringstream ss;
                ss << "Sensor width doesn't exist in the sensor database for image " << getImagePath() << "\n";
                ss << "\t- camera brand: " << make << "\n";
                ss << "\t- camera model: " << model << "\n";
                ss << "Please add camera model and sensor width in the database.";
                ALICEVISION_LOG_WARNING(ss.str());
            }
        }
        else
        {
            // no metadata 'Make' and 'Model' can't find sensor width
            errCode = 2;
            if (verbose)
            {
                std::stringstream ss;
                ss << "No metadata in image " << getImagePath() << "\n";
                ALICEVISION_LOG_DEBUG(ss.str());
            }
        }
    }
    else
    {
        // we have a valid sensorWidth information, so we store it into the metadata (where it would
        // have been nice to have it in the first place)
        if (sensorWidthSource == ESensorWidthSource::FROM_DB)
        {
            addMetadata("AliceVision:SensorWidth", std::to_string(sensorWidth));
        }
        else if (sensorWidthSource == ESensorWidthSource::FROM_METADATA_ESTIMATION)
        {
            addMetadata("AliceVision:SensorWidthEstimation", std::to_string(sensorWidth));
        }
    }

    if (sensorWidth < 0)
    {
        if (verbose)
        {
            ALICEVISION_LOG_WARNING("Sensor size is unknown");
            ALICEVISION_LOG_WARNING("Use default sensor size (24x36 mm)");
        }
        sensorWidth = 36.0;
        sensorHeight = 24.0;
    }

    return errCode;
}

std::string GPSExifTags::latitude()
{
    return "GPS:Latitude";
}
std::string GPSExifTags::latitudeRef()
{
    return "GPS:LatitudeRef";
}
std::string GPSExifTags::longitude()
{
    return "GPS:Longitude";
}
std::string GPSExifTags::longitudeRef()
{
    return "GPS:LongitudeRef";
}
std::vector<std::string> GPSExifTags::all()
{
    return {GPSExifTags::latitude(), GPSExifTags::latitudeRef(), GPSExifTags::longitude(), GPSExifTags::longitudeRef(), GPSExifTags::altitude(), GPSExifTags::altitudeRef()};
}
std::string GPSExifTags::altitude()
{
    return "GPS:Altitude";
}
std::string GPSExifTags::altitudeRef()
{
    return "GPS:AltitudeRef";
}

} // namespace sfmData
} // namespace aliceVision
