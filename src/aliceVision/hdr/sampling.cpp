// This file is part of the AliceVision project.
// Copyright (c) 2019 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "sampling.hpp"

#include <aliceVision/alicevision_omp.hpp>
#include <aliceVision/system/Logger.hpp>

#include <OpenImageIO/imagebufalgo.h>


namespace aliceVision {
namespace hdr {

using namespace aliceVision::image;

struct Descriptor {
    float exposure;
    int channel;
    int quantizedValue;

    bool operator<(const Descriptor &o )  const {

        if (exposure < o.exposure) return true;
        if (exposure == o.exposure && channel < o.channel) return true;
        if (exposure == o.exposure && channel == o.channel && quantizedValue < o.quantizedValue) return true;

        return false;
    }
};

void integral(image::Image<image::RGBfColor> & dest, image::Image<image::RGBfColor> & source) {

    /*
    A B C 
    D E F 
    G H I
    J K L
    = 
    A            A+B                A+B+C
    A+D          A+B+D+E            A+B+C+D+E+F
    A+D+G        A+B+D+E+G+H        A+B+C+D+E+F+G+H+I
    A+D+G+J      A+B+D+E+G+H+J+K    A+B+C+D+E+F+G+H+I+J+K+L
    */

    dest.resize(source.Width(), source.Height());

    dest(0, 0) = source(0, 0);
    for (int j = 1; j < source.Width(); j++) {
        dest(0, j) = dest(0, j - 1) + source(0, j);
    }

    for (int i = 1; i < source.Height(); i++) {

        dest(i, 0) = dest(i - 1, 0) + source(i, 0);

        for (int j = 1; j < source.Width(); j++) {

            dest(i, j) = dest(i, j - 1) - dest(i - 1, j - 1) + dest(i - 1, j) + source(i, j);
        }
    }
}

void square(image::Image<image::RGBfColor> & dest, image::Image<image::RGBfColor> & source) {

    dest.resize(source.Width(), source.Height());

    for (int i = 0; i < source.Height(); i++) {

        for (int j = 0; j < source.Width(); j++) {

            dest(i, j).r() = source(i, j).r() * source(i, j).r();
            dest(i, j).g() = source(i, j).g() * source(i, j).g();
            dest(i, j).g() = source(i, j).b() * source(i, j).b();
        }
    }
}



bool extractSamples(std::vector<ImageSample>& out_samples, const std::vector<std::string> & imagePaths, const std::vector<float>& times, const size_t channelQuantization)
{
    const int radius = 5;
    const int diameter = (radius * 2) + 1;
    const float area = float(diameter * diameter);

    /* For all brackets, For each pixel, compute image sample */
    image::Image<ImageSample> samples;
    for (unsigned int idBracket = 0; idBracket < imagePaths.size(); idBracket++)
    {   
        const float exposure = times[idBracket];

        /**
         * Load image
        */
        Image<RGBfColor> img, imgSquare;
        Image<RGBfColor> imgIntegral, imgIntegralSquare;
        readImage(imagePaths[idBracket], img, EImageColorSpace::LINEAR);
        
        if (idBracket == 0) {
            samples.resize(img.Width(), img.Height());
        }
        
        /**
        * Stats for deviation
        */
        square(imgSquare, img);
        integral(imgIntegral, img);
        integral(imgIntegralSquare, imgSquare);

        for (int i = radius; i < img.Height() - radius; i++)  {
            for (int j = radius; j < img.Width() - radius; j++)  {

                image::RGBfColor S1 = imgIntegral(i + radius, j + radius) + imgIntegral(i - radius, j - radius) - imgIntegral(i + radius, j - radius) - imgIntegral(i - radius, j + radius);
                image::RGBfColor S2 = imgIntegralSquare(i + radius, j + radius) + imgIntegralSquare(i - radius, j - radius) - imgIntegralSquare(i + radius, j - radius) - imgIntegralSquare(i - radius, j + radius);
                
                PixelDescription pd;
                
                pd.exposure = exposure;
                pd.mean.r() = S1.r() / area;
                pd.mean.g() = S1.g() / area;
                pd.mean.b() = S1.b() / area;
                pd.variance.r() = (S2.r() - (S1.r()*S1.r()) / area) / area;
                pd.variance.g() = (S2.g() - (S1.g()*S1.g()) / area) / area;
                pd.variance.b() = (S2.b() - (S1.b()*S1.b()) / area) / area;

                samples(i, j).x = j;
                samples(i, j).y = i;
                samples(i, j).descriptions.push_back(pd);
            }
        }            
    }

    if (samples.Width() == 0) {
        /*Why ? just to be sure*/
        return false;
    }

    /*Create samples image*/
    for (int i = radius; i < samples.Height() - radius; i++)  {
        for (int j = radius; j < samples.Width() - radius; j++)  {
            
            ImageSample & sample = samples(i, j);
            if (sample.descriptions.size() < 2) {
                continue;
            }

            int last_ok = 0;

            /*
            Make sure we don't have a patch with high variance on any bracket
            */
            bool valid = true;
            for (int k = 0; k < sample.descriptions.size(); k++) {
                
                if (sample.descriptions[k].variance.r() > 0.05) {
                    valid = false;
                    break;
                }

                if (sample.descriptions[k].variance.g() > 0.05) {
                    valid = false;
                    break;
                }

                if (sample.descriptions[k].variance.b() > 0.05) {
                    valid = false;
                    break;
                }
            }

            if (!valid) {
                sample.descriptions.clear();
                continue;
            }

            /* Makes sure the curve is monotonic */
            int lastvalid = 0;
            for (int k = 1; k < sample.descriptions.size(); k++) {
                bool valid = false;

                if (sample.descriptions[k].mean.r() > 1.0) {
                    continue;
                }

                if (sample.descriptions[k].mean.g() > 1.0) {
                    continue;
                }

                if (sample.descriptions[k].mean.b() > 1.0) {
                    continue;
                }

                if (sample.descriptions[k].mean.r() > 1.004 * sample.descriptions[k - 1].mean.r()) {
                    valid = true;
                }

                if (sample.descriptions[k].mean.g() > 1.004 * sample.descriptions[k - 1].mean.g()) {
                    valid = true;
                }

                if (sample.descriptions[k].mean.b() > 1.004 * sample.descriptions[k - 1].mean.b()) {
                    valid = true;
                }

                if (sample.descriptions[k].mean.r() < sample.descriptions[k - 1].mean.r()) {
                    valid = false;
                }

                if (sample.descriptions[k].mean.g() < sample.descriptions[k - 1].mean.g()) {
                    valid = false;
                }

                if (sample.descriptions[k].mean.b() < sample.descriptions[k - 1].mean.b()) {
                    valid = false;
                }

                if (sample.descriptions[k - 1].mean.norm() > 0.1) {
                    
                    /*Check that both colors are similars*/
                    float n1 = sample.descriptions[k - 1].mean.norm();
                    float n2 = sample.descriptions[k].mean.norm();
                    float dot = sample.descriptions[k - 1].mean.dot(sample.descriptions[k].mean);
                    float cosa = dot / (n1*n2);
                    if (cosa < 0.95) {
                        valid = false;
                    }
                }

                if (valid) {
                    lastvalid = k;
                }
                else {
                    break;
                }
            }

            if (lastvalid == 0) {
                sample.descriptions.clear();
            }
            else if (lastvalid < (sample.descriptions.size() - 1)) {
                sample.descriptions.resize(lastvalid + 1);
            }
        }
    }


    using Coordinates = std::pair<int, int>;
    using CoordinatesList = std::vector<Coordinates>;
    using Counters = std::map<Descriptor, CoordinatesList>;
    Counters counters;

    for (int i = radius; i < samples.Height() - radius; i++)  {
        for (int j = radius; j < samples.Width() - radius; j++)  {

            ImageSample & sample = samples(i, j);
            Descriptor desc;

            for (int k = 0; k < sample.descriptions.size(); k++) { 
                
                desc.exposure = sample.descriptions[k].exposure;

                for (int channel = 0; channel < 3; channel++) {

                    desc.channel = channel;
                    
                    /* Get quantized value */
                    desc.quantizedValue = int(std::round(sample.descriptions[k].mean(channel)  * (channelQuantization - 1)));
                    if (desc.quantizedValue < 0 || desc.quantizedValue >= channelQuantization) {
                        continue;
                    }
                    
                    Coordinates coordinates = std::make_pair(sample.x, sample.y);
                    counters[desc].push_back(coordinates);
                }
            }
        }
    }

    const size_t maxCountSample = 300;
    Descriptor desc;
    for (unsigned int idBracket = 0; idBracket < imagePaths.size(); idBracket++) {

        desc.exposure = times[idBracket];

        for (int channel = 0; channel < 3; channel++) {

            desc.channel = channel;

            for (int k = 0; k < 1024; k++) {

                desc.quantizedValue = k;

                if (counters.find(desc) == counters.end()) {
                    continue;
                }

                if (counters[desc].size() > maxCountSample) {

                    /*Shuffle and ignore the exceeding samples*/
                    std::random_shuffle(counters[desc].begin(), counters[desc].end());
                    counters[desc].resize(maxCountSample);
                }

                for (int l = 0; l < counters[desc].size(); l++) {
                    Coordinates coords = counters[desc][l];
                    
                    if (samples(coords.second, coords.first).descriptions.size() > 0) {
                        out_samples.push_back(samples(coords.second, coords.first));
                        samples(coords.second, coords.first).descriptions.clear();
                    }
                }
            }
        }
    }

    return true;
}

bool extractSamplesGroups(std::vector<std::vector<ImageSample>> & out_samples, const std::vector<std::vector<std::string>> & imagePaths, const std::vector<std::vector<float>>& times, const size_t channelQuantization) {

    std::vector<std::vector<ImageSample>> nonFilteredSamples;
    out_samples.resize(imagePaths.size());

    using SampleRef = std::pair<int, int>;
    using SampleRefList = std::vector<SampleRef>;
    using MapSampleRefList = std::map<Descriptor, SampleRefList>;

    MapSampleRefList mapSampleRefList;

    for (int idGroup = 0; idGroup < imagePaths.size(); idGroup++) {
        
        std::vector<ImageSample> groupSamples;
        if (!extractSamples(groupSamples, imagePaths[idGroup], times[idGroup], channelQuantization)) {
            return false;
        }

        nonFilteredSamples.push_back(groupSamples);
    }

    for (int idGroup = 0; idGroup < imagePaths.size(); idGroup++) {
        
        std::vector<ImageSample> & groupSamples = nonFilteredSamples[idGroup];

        for (int idSample = 0; idSample < groupSamples.size(); idSample++) {

            SampleRef s;
            s.first = idGroup;
            s.second = idSample;
            
            const ImageSample & sample = groupSamples[idSample];

            for (int idDesc = 0; idDesc < sample.descriptions.size(); idDesc++) {
                
                Descriptor desc;
                desc.exposure = sample.descriptions[idDesc].exposure;

                for (int channel = 0; channel < 3; channel++) {
                    
                    desc.channel = channel;
                    
                    /* Get quantized value */
                    desc.quantizedValue = int(std::round(sample.descriptions[idDesc].mean(channel)  * (channelQuantization - 1)));
                    if (desc.quantizedValue < 0 || desc.quantizedValue >= channelQuantization) {
                        continue;
                    }

                    mapSampleRefList[desc].push_back(s);
                }
            }
        }
    }

    const size_t maxCountSample = 200;
    for (auto & list : mapSampleRefList) {

        if (list.second.size() > maxCountSample) {
             /*Shuffle and ignore the exceeding samples*/
            std::random_shuffle(list.second.begin(), list.second.end());
            list.second.resize(maxCountSample);
        }

        for (auto & item : list.second) {
            
            if (nonFilteredSamples[item.first][item.second].descriptions.size() > 0) {
                out_samples[item.first].push_back(nonFilteredSamples[item.first][item.second]);
                nonFilteredSamples[item.first][item.second].descriptions.clear();
            }
        }
    }

    return true;
}


} // namespace hdr
} // namespace aliceVision
