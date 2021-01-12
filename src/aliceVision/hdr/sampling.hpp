// This file is part of the AliceVision project.
// Copyright (c) 2019 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/image/all.hpp>
#include <aliceVision/numeric/numeric.hpp>
#include <set>

namespace aliceVision {
namespace hdr {

struct UniqueDescriptor
{
    float exposure;
    int channel;
    int quantizedValue;

    bool operator<(const UniqueDescriptor &o) const;
};

struct PixelDescription
{
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
};

std::ostream & operator<<(std::ostream& os, const ImageSample & s);
std::ostream & operator<<(std::ostream& os, const PixelDescription & p);
std::istream & operator>>(std::istream& os, ImageSample & s);
std::istream & operator>>(std::istream& os, PixelDescription & p);


class Sampling
{
public:
    struct Coordinates
    {
        unsigned int imageIndex;
        unsigned int sampleIndex;
    };
    struct Params
    {
        int blockSize = 256;
        int radius = 5;
        size_t maxCountSample = 200;
    };

    using MapSampleRefList = std::map<UniqueDescriptor, std::vector<Coordinates>>;

public:
    void analyzeSource(std::vector<ImageSample> & samples, int channelQuantization, int imageIndex);
    void filter(size_t maxTotalPoints);
    void extractUsefulSamples(std::vector<ImageSample> & out_samples, const std::vector<ImageSample> & samples, int imageIndex) const;
    
    static bool extractSamplesFromImages(std::vector<ImageSample>& out_samples, const std::vector<std::string> & imagePaths, const std::vector<float>& times, const size_t imageWidth, const size_t imageHeight, const size_t channelQuantization, const image::EImageColorSpace & colorspace, bool applyWhiteBalance, const Params params);

private:
    MapSampleRefList _positions;
};

} // namespace hdr
} // namespace aliceVision
