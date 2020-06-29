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

    bool operator<(const UniqueDescriptor &o )  const
    {
        if (exposure < o.exposure)
            return true;
        if (exposure == o.exposure && channel < o.channel)
            return true;
        if (exposure == o.exposure && channel == o.channel && quantizedValue < o.quantizedValue)
            return true;

        return false;
    }
};

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
};

std::ostream & operator<<(std::ostream& os, const ImageSample & s);
std::ostream & operator<<(std::ostream& os, const PixelDescription & p);
std::istream & operator>>(std::istream& os, ImageSample & s);
std::istream & operator>>(std::istream& os, PixelDescription & p);

bool extractSamples(std::vector<ImageSample>& out_samples, const std::vector<std::string> & imagePaths, const std::vector<float>& times, const size_t channelQuantization);
bool extractSamplesGroups(std::vector<std::vector<ImageSample>> & out_samples, const std::vector<std::vector<std::string>> & imagePaths, const std::vector<std::vector<float>>& times, const size_t channelQuantization);

class Sampling {
public:
    struct Coordinates {
        unsigned int image_index;
        unsigned int sample_index;
    };

    using MapSampleRefList = std::map<UniqueDescriptor, std::vector<Coordinates>>;

public:

    void analyzeSource(std::vector<ImageSample> & samples, int channelQuantization, int image_index) {

        for (int sample_index = 0; sample_index < samples.size(); sample_index++) {

            ImageSample & sample = samples[sample_index];

            for (auto & desc : sample.descriptions) {
                UniqueDescriptor udesc;
                udesc.exposure = desc.exposure;
                
                for (int channel = 0; channel < 3; channel++) {

                    udesc.channel = channel;                    
                    udesc.quantizedValue = int(std::round(desc.mean(channel)  * (channelQuantization - 1)));
                    if (udesc.quantizedValue < 0 || udesc.quantizedValue >= channelQuantization) {
                        continue;
                    }

                    Coordinates c;
                    c.image_index = image_index;
                    c.sample_index = sample_index;

                    _positions[udesc].push_back(c);
                }
            }
        }

        for (auto & item : _positions) {

            if (item.second.size() > 200) {

                /*Shuffle and ignore the exceeding samples*/
                std::random_shuffle(item.second.begin(), item.second.end());
                item.second.resize(500);
            }
        }
    }

    void filter(size_t max_total_points) {

        size_t limit_per_group = 510;
        size_t total_points = max_total_points + 1;

        while (total_points > max_total_points) {

            limit_per_group = limit_per_group - 10;

            total_points = 0;
            for (auto & item : _positions) {

                if (item.second.size() > limit_per_group) {
                    /*Shuffle and ignore the exceeding samples*/
                    std::random_shuffle(item.second.begin(), item.second.end());
                    item.second.resize(limit_per_group);
                }

                total_points += item.second.size();
            }
        }
    }

    void extractSamples(std::vector<ImageSample> & out_samples, std::vector<ImageSample> & samples, int image_index) {

        std::set<unsigned int> unique_indices;

        for (auto & item : _positions) {

            for (auto & pos : item.second) {
                
                if (pos.image_index == image_index) {

                    unique_indices.insert(pos.sample_index);
                }
            }
        }

        for (auto & index : unique_indices) {

            if (samples[index].descriptions.size() > 0) {

                out_samples.push_back(samples[index]);
                samples[index].descriptions.clear();
            }
        }
    }

private:
    MapSampleRefList _positions;
};

} // namespace hdr
} // namespace aliceVision
