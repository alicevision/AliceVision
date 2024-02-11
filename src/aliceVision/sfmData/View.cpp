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
namespace sfmData {}  // namespace sfmData
}  // namespace aliceVision
