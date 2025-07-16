// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <vector>
#include <iostream>
#include <boost/json.hpp>
#include <Eigen/Dense>

#include <aliceVision/stl/FlatMap.hpp>
#include <map>
#include <unordered_map>

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

template<class K, class T>
stl::flat_map<K, T> flat_map_value_to(const boost::json::value& jv)
{
    stl::flat_map<K, T> ret;

    const boost::json::array obj = jv.as_array();

    for (const auto& item : obj)
    {
        const boost::json::array inner = item.as_array();
        ret.insert({boost::json::value_to<K>(inner[0]), boost::json::value_to<T>(inner[1])});
    }

    return ret;
}

template<class K, class T>
std::unordered_map<K, T> unordered_map_value_to(const boost::json::value& jv)
{
    std::unordered_map<K, T> ret;

    const boost::json::array obj = jv.as_array();

    for (const auto& item : obj)
    {
        const boost::json::array inner = item.as_array();
        ret.insert({boost::json::value_to<K>(inner[0]), boost::json::value_to<T>(inner[1])});
    }

    return ret;
}


template<class K, class T>
std::map<K, T> map_value_to(const boost::json::value& jv)
{
    std::map<K, T> ret;

    const boost::json::array obj = jv.as_array();

    for (const auto& item : obj)
    {
        const boost::json::array inner = item.as_array();
        ret.insert({boost::json::value_to<K>(inner[0]), boost::json::value_to<T>(inner[1])});
    }

    return ret;
}

std::vector<boost::json::value> readJsons(std::istream& is, boost::system::error_code& ec);

}