// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/track/Track.hpp>

#include <boost/json.hpp>

namespace aliceVision {
namespace track {

/**
 * @brief Serialize track to JSON object.
 */
void tag_invoke(const boost::json::value_from_tag&, boost::json::value& jv, aliceVision::track::Track const& input);

} // namespace track
} // namespace aliceVision
