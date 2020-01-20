// This file is part of the AliceVision project.
// Copyright (c) 2019 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/image/all.hpp>
#include <aliceVision/numeric/numeric.hpp>

namespace aliceVision {
namespace hdr {


struct ImageSamples
{
    std::vector<image::Rgb<double>> colors;
    int camId = 0;
    double exposure = 0.0;
};


void extractSamples(
    std::vector<std::vector<ImageSamples>>& out_samples,
    const std::vector<std::vector<std::string>>& imagePathsGroups,
    const std::vector< std::vector<float> >& cameraExposures,
    int nbPoints,
    int calibrationDownscale,
    bool fisheye);


} // namespace hdr
} // namespace aliceVision
