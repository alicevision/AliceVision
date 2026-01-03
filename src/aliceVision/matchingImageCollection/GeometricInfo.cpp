// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/matchingImageCollection/GeometricInfo.hpp>
#include <aliceVision/dataio/json.hpp>
#include <fstream>

namespace aliceVision {
namespace matchingImageCollection {

void tag_invoke(const boost::json::value_from_tag&, boost::json::value& jv, aliceVision::matchingImageCollection::PairGeometricInfo const& input)
{
    // ATM, all model are 3x3
    Eigen::Matrix3d M = input.model;

    jv = {
      {"type", EGeometricFilterType_enumToString(input.type)},
      {"model", boost::json::value_from(M)},
      {"threshold", boost::json::value_from(input.threshold)},
      {"inliers", boost::json::value_from(input.inliers)},
    };
}

aliceVision::matchingImageCollection::PairGeometricInfo tag_invoke(boost::json::value_to_tag<aliceVision::matchingImageCollection::PairGeometricInfo>,
                                                                   boost::json::value const& jv)
{
    const boost::json::object& obj = jv.as_object();

    aliceVision::matchingImageCollection::PairGeometricInfo ret;
    ret.type = EGeometricFilterType_stringToEnum(boost::json::value_to<std::string>(obj.at("type")));
    ret.inliers = boost::json::value_to<size_t>(obj.at("inliers"));
    ret.threshold = boost::json::value_to<double>(obj.at("threshold"));
    ret.model = boost::json::value_to<Eigen::Matrix3d>(obj.at("model"));

    return ret;
}

bool saveGeometricInfos(const std::string& path, const PairwiseGeometricInfo& infos)
{
    std::ofstream giFile(path);
    if (giFile.is_open() == false)
    {
        return false;
    }

    boost::json::value jv = boost::json::value_from(infos);

    giFile << boost::json::serialize(jv);
    giFile.close();

    return true;
}

bool loadGeometricInfos(const std::string& path, PairwiseGeometricInfo& infos)
{
    std::ifstream giFile(path);
    if (giFile.is_open() == false)
    {
        return false;
    }

    std::stringstream buffer;
    buffer << giFile.rdbuf();
    giFile.close();

    // Parse json
    boost::json::value jv = boost::json::parse(buffer.str());
    infos = PairwiseGeometricInfo(map_value_to<aliceVision::Pair, PairGeometricInfo>(jv));

    return true;
}

}  // namespace matchingImageCollection
}  // namespace aliceVision