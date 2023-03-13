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
