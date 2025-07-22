// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "trackIO.hpp"
#include <aliceVision/dataio/json.hpp>
#include <fstream>

namespace aliceVision {
namespace track {

void tag_invoke(const boost::json::value_from_tag&, boost::json::value& jv, aliceVision::track::TrackItem const& input)
{
    jv = {
      {"featureId", boost::json::value_from(input.featureId)},
      {"coords", boost::json::value_from(input.coords)},
      {"scale", boost::json::value_from(input.scale)},
      {"depth", boost::json::value_from(input.depth)},
    };
}

aliceVision::track::TrackItem tag_invoke(boost::json::value_to_tag<aliceVision::track::TrackItem>, boost::json::value const& jv)
{
    const boost::json::object& obj = jv.as_object();

    aliceVision::track::TrackItem ret;
    ret.featureId = boost::json::value_to<std::size_t>(obj.at("featureId"));
    ret.coords = boost::json::value_to<Vec2>(obj.at("coords"));
    ret.scale = boost::json::value_to<double>(obj.at("scale"));
    ret.depth = boost::json::value_to<double>(obj.at("depth"));

    return ret;
}

void tag_invoke(const boost::json::value_from_tag&, boost::json::value& jv, aliceVision::track::Track const& input)
{
    jv = {{"descType", EImageDescriberType_enumToString(input.descType)}, {"featPerView", boost::json::value_from(input.featPerView)}};
}

aliceVision::track::Track tag_invoke(boost::json::value_to_tag<aliceVision::track::Track>, boost::json::value const& jv)
{
    const boost::json::object& obj = jv.as_object();

    aliceVision::track::Track ret;
    ret.descType = feature::EImageDescriberType_stringToEnum(boost::json::value_to<std::string>(obj.at("descType")));
    ret.featPerView = map_value_to<std::size_t, track::TrackItem>(obj.at("featPerView"));

    return ret;
}

bool loadTracks(TracksMap& mapTracks, const std::string& filename)
{
    std::ifstream tracksFile(filename);
    if (tracksFile.is_open() == false)
    {
        return false;
    }

    std::stringstream buffer;
    buffer << tracksFile.rdbuf();

    // Parse json
    boost::json::value jv = boost::json::parse(buffer.str());
    mapTracks = track::TracksMap(map_value_to<std::size_t, track::Track>(jv));

    return true;
}

bool saveTracks(const TracksMap& mapTracks, const std::string& filename)
{
    std::ofstream tracksFile(filename);
    if (tracksFile.is_open() == false)
    {
        return false;
    } 

    boost::json::value jv = boost::json::value_from(mapTracks);

    tracksFile << boost::json::serialize(jv);
    tracksFile.close();

    return true;
}

}  // namespace track
}  // namespace aliceVision
