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

template<class T>
stl::flat_map<size_t, T> flat_map_value_to(const boost::json::value& jv)
{
    stl::flat_map<size_t, T> ret;
    
    const boost::json::array obj = jv.as_array();

    for (const auto & item: obj)
    {
        const boost::json::array inner = item.as_array();
        ret.insert({boost::json::value_to<std::size_t>(inner[0]), boost::json::value_to<T>(inner[1])});
    }

    return ret;
}

/**
 * @brief Serialize track to JSON object.
 */
void tag_invoke(const boost::json::value_from_tag&, boost::json::value& jv, aliceVision::track::Track const& input);

/**
 * @brief Deserialize track from JSON object.
 */
aliceVision::track::Track tag_invoke(boost::json::value_to_tag<aliceVision::track::Track>, boost::json::value const& jv);

} // namespace track
} // namespace aliceVision
