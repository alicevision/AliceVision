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


struct PixelDescription {
    float exposure;
    image::Rgb<float> mean;
    image::Rgb<float> variance;
};

struct ImageSample
{
    size_t x = 0;
    size_t y = 0;
    std::vector<PixelDescription> descriptions;

    ImageSample() = default;
    ImageSample(int val) {};
};

bool extractSamples(std::vector<ImageSample>& out_samples, const std::vector<std::string> & imagePaths, const std::vector<float>& times, const size_t channelQuantization);
bool extractSamplesGroups(std::vector<std::vector<ImageSample>> & out_samples, const std::vector<std::vector<std::string>> & imagePaths, const std::vector<std::vector<float>>& times, const size_t channelQuantization);

} // namespace hdr
} // namespace aliceVision
