// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "trackIO.hpp"

namespace aliceVision {
namespace track {

void tag_invoke(const boost::json::value_from_tag&, boost::json::value& jv, aliceVision::track::Track const& input)
{
    jv = {
    	{"descType", EImageDescriberType_enumToString(input.descType)} ,
        {"featPerView", boost::json::value_from(input.featPerView)}
    };
}

} // namespace track
} // namespace aliceVision
