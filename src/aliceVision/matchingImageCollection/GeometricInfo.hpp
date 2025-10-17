// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <Eigen/Dense>
#include <aliceVision/matchingImageCollection/GeometricFilterType.hpp>
#include <aliceVision/types.hpp>
#include <map>
#include <boost/json.hpp>

namespace aliceVision {
namespace matchingImageCollection {

struct PairGeometricInfo
{
    Eigen::MatrixXd model;
    size_t inliers;
    double threshold;
    EGeometricFilterType type;
};

typedef std::map<Pair, PairGeometricInfo> PairwiseGeometricInfo;

/**
 * @brief Serialize PairGeometricInfo to JSON object.
 */
void tag_invoke(const boost::json::value_from_tag&, boost::json::value& jv, aliceVision::matchingImageCollection::PairGeometricInfo const& input);

/**
 * @brief save Geometric infos to a file
 * @param path the path to the output file
 * @param infos the PairwiseGeometricInfo to save
 * @return false on failure
 */
bool saveGeometricInfos(const std::string& path, const PairwiseGeometricInfo& infos);

/**
 * @brief load Geometric infos to a file
 * @param path the path to the input file
 * @param infos the PairwiseGeometricInfo to load
 * @return false on failure
 */
bool loadGeometricInfos(const std::string& path, PairwiseGeometricInfo& infos);

}  // namespace matchingImageCollection
}  // namespace aliceVision