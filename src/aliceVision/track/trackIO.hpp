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

/**
 * @brief Deserialize track from JSON object.
 */
aliceVision::track::Track tag_invoke(boost::json::value_to_tag<aliceVision::track::Track>, boost::json::value const& jv);

/**
 * @brief load tracks from file
 * @param mapTracks the result container
 * @param filename the input file path
 * @return false if the load failed
*/
bool loadTracks(TracksMap& mapTracks, const std::string& filename);

/**
 * @brief save tracks to file
 * @param mapTracks the input container
 * @param filename the input file path
 * @return false if the save failed
*/
bool saveTracks(const TracksMap& mapTracks, const std::string& filename);

}  // namespace track
}  // namespace aliceVision
