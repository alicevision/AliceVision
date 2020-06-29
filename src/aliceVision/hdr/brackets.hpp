// This file is part of the AliceVision project.
// Copyright (c) 2019 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/sfmData/SfMData.hpp>

namespace aliceVision {
namespace hdr {

/**
 * Estimate brackets information from sfm data
 * If an error occur, like an invalid sfm data, return false
 */
bool estimateBracketsFromSfmData(std::vector<std::vector<std::shared_ptr<sfmData::View>>> & groups, std::vector<std::shared_ptr<sfmData::View>> & targetViews, const sfmData::SfMData & sfmData, size_t countBrackets);

}
}