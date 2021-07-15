// This file is part of the AliceVision project.
// Copyright (c) 2021 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "gps.hpp"
#include <cmath>
#include <boost/algorithm/string.hpp>
#include <aliceVision/system/Logger.hpp>

namespace aliceVision
{

Vec3 WGS84ToCartesian(const Vec3& gps)
{

    // equatorial radius WGS84 major axis
    const static double equRadius = 6378137.0;
    const static double flattening = 1.0 / 298.257222101;

    const static double sqrEccentricity = flattening * (2.0 - flattening);

    const double lat = degreeToRadian(gps(0));
    const double lon = degreeToRadian(gps(1));
    const double alt = gps(2);

    const double sinLat = std::sin(lat);
    const double cosLat = std::cos(lat);
    const double sinLon = std::sin(lon);
    const double cosLon = std::cos(lon);

    // Normalized radius
    const double normRadius = equRadius / std::sqrt(1.0 - sqrEccentricity * sinLat * sinLat);

    return {(normRadius + alt) * cosLat * cosLon,
            (normRadius + alt) * cosLat * sinLon,
            (normRadius * (1.0 - sqrEccentricity) + alt) * sinLat};
}

double parseAltitudeFromString(const std::string& alt, const std::string& altRef)
{
    static const std::array<std::string, 2> allowedRef = {"0", "1"};
    if(!std::any_of(allowedRef.cbegin(), allowedRef.cend(), [altRef](const std::string& s){return altRef == s;}))
    {
        const auto message = "Unexpected value for gps reference: " + altRef;
        ALICEVISION_LOG_ERROR(message);
        throw std::invalid_argument(message);
    }
    const double ref = std::stod(altRef);
    const double altitude = std::stod(alt);

    return ref > 0.0 ? -altitude : altitude;
}

//WGS84
double parseGPSFromString(const std::string& gpsDegrees, const std::string& gpsRef)
{
    static const std::array<std::string, 4> allowedRef = {"W", "E", "N", "S"};
    if(!std::any_of(allowedRef.cbegin(), allowedRef.cend(), [gpsRef](const std::string& s){return gpsRef == s;}))
    {
        const auto message = "Unexpected value for gps reference: " + gpsRef;
        ALICEVISION_LOG_ERROR(message);
        throw std::invalid_argument(message);
    }
    std::vector<std::string> result;
    // parse the comma separated values
    boost::split(result, gpsDegrees, boost::is_any_of(","));

    double gpsCoord{0};

    for(std::size_t i = 0; i < result.size(); ++i)
    {
        const auto d = std::stod(result[i]);
        gpsCoord += d * std::pow(60., -static_cast<int>(i));
    }

    return (gpsRef == "S" || gpsRef == "W") ? -gpsCoord : gpsCoord;
}

} // namespace aliceVision
