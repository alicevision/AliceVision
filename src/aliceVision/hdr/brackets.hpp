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
 * @brief Estimate brackets information from sfm data
 * @param[out] groups: estimated groups
 * @param[in] sfmData
 * @param[in] countBrackets: number of brackets
 * @return false if an error occur, like an invalid sfm data
 */
bool estimateBracketsFromSfmData(std::vector<std::vector<std::shared_ptr<sfmData::View>>> & groups, const sfmData::SfMData& sfmData, size_t countBrackets);

/**
 * @brief Select the target views (used for instance to define the expoure)
 * @param[out] targetViews: estimated target views
 * @param[in] groups: groups of Views corresponding to multi-bracketing. Warning: Needs be sorted by exposure time.
 * @param[in] offsetRefBracketIndex: 0 mean center bracket and you can choose +N/-N to select the reference bracket
 */
void selectTargetViews(std::vector<std::shared_ptr<sfmData::View>> & out_targetViews, const std::vector<std::vector<std::shared_ptr<sfmData::View>>>& groups, int offsetRefBracketIndex);

}
}
