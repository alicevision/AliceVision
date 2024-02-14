// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <boost/json.hpp>

namespace aliceVision {
namespace fuseCut {

struct Input
{
    std::string sfmPath;
    std::string subMeshPath;
};

using InputSet = std::vector<Input>;

/**
 * @brief Deserialize JSON object to Input.
 */
Input tag_invoke(boost::json::value_to_tag<Input>, boost::json::value const& jv);

/**
 * @brief Serialize Input to JSON object.
 */
void tag_invoke(const boost::json::value_from_tag&, boost::json::value& jv, Input const& t);

}  // namespace fuseCut
}  // namespace aliceVision
