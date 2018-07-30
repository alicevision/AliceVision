// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <string>

namespace aliceVision {

namespace sfmData {
class SfMData;
} // namespace sfmData

namespace sfm {

/**
 * @brief generate a report for the structure from motion
 * @param sfmData The input sfmData
 * @param htmlFilename The filename of the HTML report
 * @return true if ok
 */
bool generateSfMReport(const sfmData::SfMData& sfmData, const std::string& htmlFilename);

} // namespace sfm
} // namespace aliceVision
