// This file is part of the AliceVision project.
// Copyright (c) 2021 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "numeric.hpp"

namespace aliceVision {

/**
 * @brief Convert the position expressed in WGS84 reference frame (latitude, longitude, altitude) to the cartesian
 * coordinates x, y, z.
 * @param[in] gps A three dimensional vector containing the WGS84 coordinates latitude, longitude, altitude
 * @return A three dimensional vector containing the x, y and z coordinates.
 */
Vec3 WGS84ToCartesian(const Vec3& gps);

/**
 * @brief Convert the string representation of the altitude in gps representation to a numeric value.
 * @param[in] alt String containing the altitude.
 * @param[in] altRef String containing the reference for the altitude, if 1 the altitude will be interpreted as below
 * the sea level.
 * (https://www.awaresystems.be/imaging/tiff/tifftags/privateifd/gps/gpsaltituderef.html)
 * @return the altitude in meters over the sea level.
 * @throws std::invalid_argument() if the data is not valid.
 */
double parseAltitudeFromString(const std::string& alt, const std::string& altRef);

/**
 * @brief Convert the string representation of the gps position (latitude or longitude) to a numeric value.
 * @param[in] gpsDegrees The string representation of the latitude or longitude in degrees.
 * @param[in] gpsRef A string giving the reference for the latitude or longitude (i.e. "N" or "S" for latitude,
 * "E" or "W" for longitude). https://www.awaresystems.be/imaging/tiff/tifftags/privateifd/gps/gpslatituderef.html
 * @return The decimal degree representation of the position.
 * @throws std::invalid_argument() if the data is not valid.
 */
double parseGPSFromString(const std::string& gpsDegrees, const std::string& gpsRef);

}