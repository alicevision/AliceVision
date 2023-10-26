// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "exif.hpp"
#include <string>

namespace aliceVision {
namespace sfmData {

std::string GPSExifTags::latitude() { return "GPS:Latitude"; }

std::string GPSExifTags::latitudeRef() { return "GPS:LatitudeRef"; }

std::string GPSExifTags::longitude() { return "GPS:Longitude"; }

std::string GPSExifTags::longitudeRef() { return "GPS:LongitudeRef"; }

std::vector<std::string> GPSExifTags::all()
{
    return {GPSExifTags::latitude(),
            GPSExifTags::latitudeRef(),
            GPSExifTags::longitude(),
            GPSExifTags::longitudeRef(),
            GPSExifTags::altitude(),
            GPSExifTags::altitudeRef()};
}

std::string GPSExifTags::altitude() { return "GPS:Altitude"; }

std::string GPSExifTags::altitudeRef() { return "GPS:AltitudeRef"; }

}  // namespace sfmData
}  // namespace aliceVision
