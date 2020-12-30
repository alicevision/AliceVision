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


namespace aliceVision {
namespace sfmData {

float computeCameraExposureSetting(float shutter, float fnumber, float iso, float refFNumber, float refISO)
{
    if(iso < 1e-6f)
    {
        iso = refISO;
    }
    const float isoExp = iso / refISO;
    const float fnumberExp = std::pow(fnumber / refFNumber, 2);
    const float cameraExposureSetting = (isoExp * shutter) / fnumberExp;
    return cameraExposureSetting;
}

float View::getCameraExposureSetting() const
{
    const float shutter = getMetadataShutter();
    const float fnumber = getMetadataFNumber();
    if(shutter <= 0.0f || fnumber <= 0.0f)
        return -1.f;

    const float iso = getMetadataISO();
    const float cameraExposureSetting = computeCameraExposureSetting(shutter, fnumber, iso);

    return cameraExposureSetting;
}

float View::getEv() const
{
    return std::log2(1.f / getCameraExposureSetting());
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
    for(const std::string& name : names)
    {
        const auto it = findMetadataIterator(name);
        if(it != _metadata.end())
            return true;
    }
    return false;
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
    if(value.empty())
        return -1.0;
    return readRealNumber(value);
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

} // namespace sfmData
} // namespace aliceVision
