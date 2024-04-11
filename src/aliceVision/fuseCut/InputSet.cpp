// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "InputSet.hpp"
#include <iostream>

namespace Eigen {

template<typename T, int M, int N>
Eigen::Matrix<T, M, N> tag_invoke(boost::json::value_to_tag<Eigen::Matrix<T, M, N>>, boost::json::value const& jv)
{
    Eigen::Matrix<T, M, N> ret;

    std::vector<T> buf = boost::json::value_to<std::vector<T>>(jv);

    int pos = 0;
    for (int i = 0; i < M; i++)
    {
        for (int j = 0; j < N; j++)
        {
            ret(i, j) = buf[pos];
            pos++;
        }
    }

    return ret;
}

template<typename T, int M, int N>
void tag_invoke(const boost::json::value_from_tag&, boost::json::value& jv, Eigen::Matrix<T, M, N> const& input)
{
    std::vector<T> buf;

    for (int i = 0; i < M; i++)
    {
        for (int j = 0; j < N; j++)
        {
            buf.push_back(input(i, j));
        }
    }

    jv = boost::json::value_from<std::vector<T>>(std::move(buf));
}

}  // namespace Eigen

namespace aliceVision {
namespace fuseCut {

Input tag_invoke(boost::json::value_to_tag<Input>, boost::json::value const& jv)
{
    const boost::json::object& obj = jv.as_object();

    Input ret;
    ret.sfmPath = boost::json::value_to<std::string>(obj.at("sfmPath"));
    ret.subMeshPath = boost::json::value_to<std::string>(obj.at("subMeshPath"));
    ret.bbMin = boost::json::value_to<Eigen::Vector3d>(obj.at("bbMin"));
    ret.bbMax = boost::json::value_to<Eigen::Vector3d>(obj.at("bbMax"));

    return ret;
}

void tag_invoke(const boost::json::value_from_tag&, boost::json::value& jv, Input const& t)
{
    jv = {
      {"sfmPath", t.sfmPath},
      {"subMeshPath", t.subMeshPath},
      {"bbMin", boost::json::value_from(t.bbMin)},
      {"bbMax", boost::json::value_from(t.bbMax)},
    };
}

}  // namespace fuseCut
}  // namespace aliceVision