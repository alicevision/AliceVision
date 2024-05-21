// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <vector>
#include <iostream>
#include <boost/json.hpp>
#include <Eigen/Dense>

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

namespace aliceVision
{

std::vector<boost::json::value> readJsons(std::istream& is, boost::json::error_code& ec);

}