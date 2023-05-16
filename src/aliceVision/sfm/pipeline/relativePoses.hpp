// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/types.hpp>
#include <aliceVision/numeric/numeric.hpp>
#include <boost/json.hpp>
#include <aliceVision/sfm/liealgebra.hpp>

namespace Eigen
{
    template <typename T, int M, int N>
    Eigen::Matrix<T, M, N> tag_invoke(boost::json::value_to_tag<Eigen::Matrix<T, M, N>>, boost::json::value const& jv)
    {
        Eigen::Matrix<T, M, N> ret;
        
        std::vector<T> buf = boost::json::value_to<std::vector<T>>(jv);

        int pos = 0;
        for (int i = 0; i < M; i ++)
        {
            for (int j = 0; j < N; j++)
            {
                ret(i, j) = buf[pos];
                pos++;
            }
        }

        return ret;
    }

    template <typename T, int M, int N>
    void tag_invoke(const boost::json::value_from_tag&, boost::json::value& jv, Eigen::Matrix<T, M, N> const& input)
    {
        std::vector<T> buf;

        for (int i = 0; i < M; i ++)
        {
            for (int j = 0; j < N; j++)
            {
                buf.push_back(input(i, j));
            }
        }


        jv = boost::json::value_from<std::vector<T>>(std::move(buf));
    }

}

namespace aliceVision
{
namespace sfm
{

struct ReconstructedPair
{
    IndexT reference;
    IndexT next;
    Mat3 R;
    Vec3 t;
};


void tag_invoke(const boost::json::value_from_tag&, boost::json::value& jv, sfm::ReconstructedPair const& input)
{
    jv = {
        {"reference", input.reference}, 
        {"next", input.next}, 
        {"R", boost::json::value_from(SO3::logm(input.R))}, 
        {"t", boost::json::value_from(input.t)}
    };
}

ReconstructedPair tag_invoke(boost::json::value_to_tag<ReconstructedPair>, boost::json::value const& jv)
{
    const boost::json::object& obj = jv.as_object();

    ReconstructedPair ret;

    ret.reference = boost::json::value_to<IndexT>(obj.at("reference"));
    ret.next = boost::json::value_to<IndexT>(obj.at("next"));
    ret.R = SO3::expm(boost::json::value_to<Vec3>(obj.at("R")));
    ret.t = boost::json::value_to<Vec3>(obj.at("t"));

    return ret;
}

}
}