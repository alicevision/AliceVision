// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.


#include "InputSet.hpp"

#include <iostream>

namespace aliceVision {
namespace fuseCut {

Input tag_invoke(boost::json::value_to_tag<Input>, boost::json::value const& jv)
{
    const boost::json::object& obj = jv.as_object();

    Input ret;
    ret.path = boost::json::value_to<std::string>(obj.at("path"));

    return ret;
}

void tag_invoke(const boost::json::value_from_tag&, boost::json::value& jv, Input const& t)
{
    jv = {{"path", t.path},
          };
}

}
}