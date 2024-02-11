// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/types.hpp>
#include <aliceVision/numeric/numeric.hpp>

namespace aliceVision {
namespace sfmData {

class ExposureSetting
{
  public:
    ExposureSetting() {}

    ExposureSetting(double shutter, double fnumber, double iso)
      : _shutter(shutter),
        _fnumber(fnumber),
        _iso(iso)
    {}

    double _shutter{-1.0};
    double _fnumber{-1.0};
    double _iso{-1.0};

    bool hasShutter() const { return _shutter > 0.0 && std::isnormal(_shutter); }
    bool hasFNumber() const { return _fnumber > 0.0 && std::isnormal(_fnumber); }
    bool hasISO() const { return _iso > 0.0 && std::isnormal(_iso); }

    bool isFullyDefined() const { return hasShutter() && hasFNumber() && hasISO(); }

    bool isPartiallyDefined() const { return hasShutter() || hasFNumber(); }

    double getExposure(const double referenceISO = 100.0, const double referenceFNumber = 1.0) const
    {
        const bool validShutter = hasShutter();
        const bool validFNumber = hasFNumber();

        if (!validShutter && !validFNumber)
            return -1.0;

        const bool validRefFNumber = referenceFNumber > 0.0 && std::isnormal(referenceFNumber);

        double shutter = _shutter;
        if (!validShutter)
        {
            shutter = 1.0 / 200.0;
        }
        double fnumber = _fnumber;
        // Usually we should get a valid shutter speed, but we could have invalid fnumber.
        // For instance, if there is a connection problem between the lens and the camera, all lens related option like fnumber could be invalid.
        // In this particular case, the exposure should rely only on the shutter speed.
        if (!validFNumber)
        {
            if (validRefFNumber)
                fnumber = referenceFNumber;
            else
                fnumber = 2.0;
        }
        double lReferenceFNumber = referenceFNumber;
        if (!validRefFNumber)
        {
            lReferenceFNumber = fnumber;
        }

        const double iso = _iso;
        /*
        iso = qLt / aperture^2
        isoratio = iso2 / iso1 = (qLt / aperture2^2) / (qLt / aperture1^2)
        isoratio = aperture1^2 / aperture2^2
        aperture2^2 = aperture1^2 / isoratio
        aperture2^2 = (aperture1^2 / (iso2 / iso1))
        aperture2^2 = (iso1 / iso2)
        aperture2 = sqrt(iso1 / iso2)
        */
        double iso2Aperture = 1.0;
        if (iso > 1e-6 && referenceISO > 1e-6)
        {
            // Need to have both iso and reference iso to use it
            iso2Aperture = std::sqrt(iso / referenceISO);
        }

        /*
        aperture = f / diameter
        aperture2 / aperture1 = diameter1 / diameter2
        (aperture2 / aperture1)^2 = (area1 / pi) / (area2 / pi)
        area2 = (aperture1 / aperture2)^2
        */
        double newFnumber = fnumber * iso2Aperture;
        double expIncrease = (lReferenceFNumber / newFnumber) * (lReferenceFNumber / newFnumber);

        // If the aperture was more important for this image, this means that it received less light than with a default aperture
        // This means also that if we want to simulate that all the image have the same aperture, we have to increase virtually th
        // light received as if the aperture was smaller. So we increase the exposure time

        // If the iso is larger than the default value, this means that it recevied more light than with a default iso
        // This means also that if we want to simulate that all the image have the same iso, we have to decrease virtually th
        // light received as if the iso was smaller. So we decrease the exposure time or equivalent, increase the aperture value

        // Checks
        // iso 20, f/2 = 2500
        // iso 40, f/2.8 = 2500

        return shutter * expIncrease;
    }
    bool operator<(const ExposureSetting& other) const { return getExposure() < other.getExposure(); }
    bool operator==(const ExposureSetting& other) const { return getExposure() == other.getExposure(); }
};

inline std::ostream& operator<<(std::ostream& os, const ExposureSetting& s)
{
    os << "shutter: " << s._shutter << ", fnumber: " << s._fnumber << ", iso: " << s._iso;
    return os;
}

inline bool hasComparableExposures(const std::vector<ExposureSetting>& exposuresSetting)
{
    if (exposuresSetting.size() < 2)
        return false;

    const bool hasShutter = exposuresSetting.front().hasShutter();
    const bool hasFNumber = exposuresSetting.front().hasFNumber();
    const bool hasISO = exposuresSetting.front().hasISO();
    for (std::size_t i = 1; i < exposuresSetting.size(); ++i)
    {
        const ExposureSetting& s = exposuresSetting[i];
        if (hasShutter != s.hasShutter())
            return false;
        if (hasFNumber != s.hasFNumber())
            return false;
        if (hasISO != s.hasISO())
            return false;
    }
    return true;
}

inline std::vector<double> getExposures(const std::vector<ExposureSetting>& exposuresSetting)
{
    std::vector<double> output;
    output.reserve(exposuresSetting.size());
    for (const ExposureSetting& exp : exposuresSetting)
    {
        output.push_back(exp.getExposure());
    }
    return output;
}

}  // namespace sfmData
}  // namespace aliceVision
