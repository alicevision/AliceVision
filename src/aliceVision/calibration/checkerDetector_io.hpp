// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/calibration/checkerDetector.hpp>

#include <boost/json.hpp>

namespace aliceVision {
namespace calibration {

/**
 * @brief Deserialize JSON object to checkerboard corner.
 */
CheckerDetector::CheckerBoardCorner tag_invoke(boost::json::value_to_tag<CheckerDetector::CheckerBoardCorner>, boost::json::value const& jv);

/**
 * @brief Serialize checkerboard corner to JSON object.
 */
void tag_invoke(const boost::json::value_from_tag&, boost::json::value& jv, CheckerDetector::CheckerBoardCorner const& t);

/**
 * @brief Deserialize JSON object to checkerboard detector.
 */
CheckerDetector tag_invoke(boost::json::value_to_tag<CheckerDetector>, boost::json::value const& jv);

/**
 * @brief Serialize checkerboard detector to JSON object.
 */
void tag_invoke(const boost::json::value_from_tag&, boost::json::value& jv, CheckerDetector const& t);

}  // namespace calibration
}  // namespace aliceVision
