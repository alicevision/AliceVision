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

bool UniqueDescriptor::operator<(const UniqueDescriptor &o ) const
{
    if (exposure < o.exposure)
        return true;
    if (exposure == o.exposure && channel < o.channel)
        return true;
    if (exposure == o.exposure && channel == o.channel && quantizedValue < o.quantizedValue)
        return true;

    return false;
}

std::ostream & operator<<(std::ostream& os, const ImageSample & s)
{
    os.write((const char*)&s.x, sizeof(s.x));
    os.write((const char*)&s.y, sizeof(s.y));

    std::size_t size = s.descriptions.size();
    os.write((const char*)&size, sizeof(size));

    for (int i = 0; i  < s.descriptions.size(); ++i)
    {
        os << s.descriptions[i];
    }

    return os;
}

std::istream & operator>>(std::istream& is, ImageSample & s)
{
    std::size_t size;

    is.read((char *)&s.x, sizeof(s.x));
    is.read((char *)&s.y, sizeof(s.y));
    is.read((char *)&size, sizeof(size));
    s.descriptions.resize(size);

    for (int i = 0; i  < size; ++i)
    {
        is >> s.descriptions[i];
    }

    return is;
}

std::ostream & operator<<(std::ostream& os, const PixelDescription & p)
{
    os.write((const char *)&p.exposure, sizeof(p.exposure));
    os.write((const char *)&p.mean.r(), sizeof(p.mean.r()));
    os.write((const char *)&p.mean.g(), sizeof(p.mean.g()));
    os.write((const char *)&p.mean.b(), sizeof(p.mean.b()));
    os.write((const char *)&p.variance.r(), sizeof(p.variance.r()));
    os.write((const char *)&p.variance.g(), sizeof(p.variance.g()));
    os.write((const char *)&p.variance.b(), sizeof(p.variance.b()));

    return os;
}

std::istream & operator>>(std::istream& is, PixelDescription & p)
{
    is.read((char *)&p.exposure, sizeof(p.exposure));
    is.read((char *)&p.mean.r(), sizeof(p.mean.r()));
    is.read((char *)&p.mean.g(), sizeof(p.mean.g()));
    is.read((char *)&p.mean.b(), sizeof(p.mean.b()));
    is.read((char *)&p.variance.r(), sizeof(p.variance.r()));
    is.read((char *)&p.variance.g(), sizeof(p.variance.g()));
    is.read((char *)&p.variance.b(), sizeof(p.variance.b()));

    return is;
}

void integral(image::Image<image::Rgb<double>> & dest, const Eigen::Matrix<image::RGBfColor, Eigen::Dynamic, Eigen::Dynamic> & source)
{
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

    dest.resize(source.cols(), source.rows());

    dest(0, 0).r() = source(0, 0).r();
    dest(0, 0).g() = source(0, 0).g();
    dest(0, 0).b() = source(0, 0).b();

    for (int j = 1; j < source.cols(); ++j)
    {
        dest(0, j).r() = dest(0, j - 1).r() + double(source(0, j).r());
        dest(0, j).g() = dest(0, j - 1).g() + double(source(0, j).g());
        dest(0, j).b() = dest(0, j - 1).b() + double(source(0, j).b());
    }

    for (int i = 1; i < source.rows(); ++i)
    {
        dest(i, 0).r() = dest(i - 1, 0).r() + double(source(i, 0).r());
        dest(i, 0).g() = dest(i - 1, 0).g() + double(source(i, 0).g());
        dest(i, 0).b() = dest(i - 1, 0).b() + double(source(i, 0).b());

        for (int j = 1; j < source.cols(); ++j)
        {
            dest(i, j).r() = dest(i, j - 1).r() - dest(i - 1, j - 1).r() + dest(i - 1, j).r() + double(source(i, j).r());
            dest(i, j).g() = dest(i, j - 1).g() - dest(i - 1, j - 1).g() + dest(i - 1, j).g() + double(source(i, j).g());
            dest(i, j).b() = dest(i, j - 1).b() - dest(i - 1, j - 1).b() + dest(i - 1, j).b() + double(source(i, j).b());
        }
    }
}

void square(image::Image<image::RGBfColor> & dest, const Eigen::Matrix<image::RGBfColor, Eigen::Dynamic, Eigen::Dynamic> & source)
{
    dest.resize(source.cols(), source.rows());

    for (int i = 0; i < source.rows(); i++)
    {
        for (int j = 0; j < source.cols(); j++)
        {
            dest(i, j).r() = source(i, j).r() * source(i, j).r();
            dest(i, j).g() = source(i, j).g() * source(i, j).g();
            dest(i, j).b() = source(i, j).b() * source(i, j).b();
        }
    }
}

bool Sampling::extractSamplesFromImages(std::vector<ImageSample>& out_samples, const std::vector<std::string> & imagePaths, const std::vector<float>& times, const size_t imageWidth, const size_t imageHeight, const size_t channelQuantization, const EImageColorSpace & colorspace, const Sampling::Params params)
{
    const int radiusp1 = params.radius + 1;
    const int diameter = (params.radius * 2) + 1;
    const float area = float(diameter * diameter);

    std::vector<std::pair<int, int>> vec_blocks;
    for(int cy = 0; cy < imageHeight; cy += params.blockSize - diameter)
    {
        for(int cx = 0; cx < imageWidth; cx += params.blockSize - diameter)
        {
            vec_blocks.push_back(std::make_pair(cx, cy));
        }
    }

    Image<RGBfColor> img;

    // For all brackets, For each pixel, compute image sample
    image::Image<ImageSample> samples(imageWidth, imageHeight, true);
    for (unsigned int idBracket = 0; idBracket < imagePaths.size(); ++idBracket)
    {
        const float exposure = times[idBracket];

        // Load image
        readImage(imagePaths[idBracket], img, colorspace);

        if(img.Width() != imageWidth ||
           img.Height() != imageHeight)
        {
            std::stringstream ss;
            ss << "Failed to extract samples, the images with multi-bracketing do not have the same image resolution.\n"
               << " Current image resolution is: " << img.Width() << "x" << img.Height()
               << ", instead of: " << imageWidth<< "x" << imageHeight << ".\n"
               << "Current image path is: " << imagePaths[idBracket];
            throw std::runtime_error(ss.str());
        }

        #pragma omp parallel for
        for (int idx = 0; idx < vec_blocks.size(); ++idx)
        {
            int cx = vec_blocks[idx].first;
            int cy = vec_blocks[idx].second;

            int blockWidth = ((img.Width() - cx) > params.blockSize) ? params.blockSize : img.Width() - cx;
            int blockHeight = ((img.Height() - cy) > params.blockSize) ? params.blockSize : img.Height() - cy;

            auto blockInput = img.block(cy, cx, blockHeight, blockWidth);
            auto blockOutput = samples.block(cy, cx, blockHeight, blockWidth);

            // Stats for deviation
            Image<Rgb<double>> imgIntegral, imgIntegralSquare; 
            Image<RGBfColor> imgSquare;

            square(imgSquare, blockInput);
            integral(imgIntegral, blockInput);
            integral(imgIntegralSquare, imgSquare);

            for(int i = radiusp1; i < imgIntegral.Height() - params.radius; ++i)
            {
                for(int j = radiusp1; j < imgIntegral.Width() - params.radius; ++j)
                {
                    image::Rgb<double> S1 = imgIntegral(i + params.radius, j + params.radius) + imgIntegral(i - radiusp1, j - radiusp1) - imgIntegral(i + params.radius, j - radiusp1) - imgIntegral(i - radiusp1, j + params.radius);
                    image::Rgb<double> S2 = imgIntegralSquare(i + params.radius, j + params.radius) + imgIntegralSquare(i - radiusp1, j - radiusp1) - imgIntegralSquare(i + params.radius, j - radiusp1) - imgIntegralSquare(i - radiusp1, j + params.radius);
                    
                    PixelDescription pd;
                    
                    pd.exposure = exposure;
                    pd.mean.r() = blockInput(i,j).r(); 
                    pd.mean.g() = blockInput(i,j).g(); 
                    pd.mean.b() = blockInput(i,j).b();
                    pd.variance.r() = (S2.r() - (S1.r()*S1.r()) / area) / area;
                    pd.variance.g() = (S2.g() - (S1.g()*S1.g()) / area) / area;
                    pd.variance.b() = (S2.b() - (S1.b()*S1.b()) / area) / area;

                    blockOutput(i, j).x = cx + j;
                    blockOutput(i, j).y = cy + i;
                    blockOutput(i, j).descriptions.push_back(pd);
                }
            }
        }
    }

    if (samples.Width() == 0)
    {
        // Why? just to be sure
        return false;
    }

    // Create samples image
    #pragma omp parallel for
    for (int i = params.radius; i < samples.Height() - params.radius; ++i)
    {
        for (int j = params.radius; j < samples.Width() - params.radius; ++j)
        {
            ImageSample & sample = samples(i, j);
            if (sample.descriptions.size() < 2)
            {
                continue;
            }

            int last_ok = 0;

            // Make sure we don't have a patch with high variance on any bracket.
            // If the variance is too high somewhere, ignore the whole coordinate samples
            bool valid = true;
            for (int k = 0; k < sample.descriptions.size(); ++k)
            {
                if (sample.descriptions[k].variance.r() > 0.05 ||
                    sample.descriptions[k].variance.g() > 0.05 ||
                    sample.descriptions[k].variance.b() > 0.05)
                {
                    valid = false;
                    break;
                }
            }

            if (!valid)
            {
                sample.descriptions.clear();
                continue;
            }

            // Makes sure the curve is monotonic
            int firstvalid = -1;
            int lastvalid = 0;
            for (std::size_t k = 1; k < sample.descriptions.size(); ++k)
            {
                bool valid = false;

                if (sample.descriptions[k].mean.r() > 0.99f ||
                    sample.descriptions[k].mean.g() > 0.99f ||
                    sample.descriptions[k].mean.b() > 0.99f)
                {
                    continue;
                }

                if (sample.descriptions[k].mean.r() > 1.004f * sample.descriptions[k - 1].mean.r() ||
                    sample.descriptions[k].mean.g() > 1.004f * sample.descriptions[k - 1].mean.g() ||
                    sample.descriptions[k].mean.b() > 1.004f * sample.descriptions[k - 1].mean.b())
                {
                    valid = true;
                }

                if (sample.descriptions[k].mean.r() < sample.descriptions[k - 1].mean.r() ||
                    sample.descriptions[k].mean.g() < sample.descriptions[k - 1].mean.g() ||
                    sample.descriptions[k].mean.b() < sample.descriptions[k - 1].mean.b())
                {
                    valid = false;
                }

                if (sample.descriptions[k - 1].mean.norm() > 0.1f)
                {
                    // Check that both colors are similars
                    const float n1 = sample.descriptions[k - 1].mean.norm();
                    const float n2 = sample.descriptions[k].mean.norm();
                    const float dot = sample.descriptions[k - 1].mean.dot(sample.descriptions[k].mean);
                    const float cosa = dot / (n1*n2);
                    if (cosa < 0.95f)
                    {
                        valid = false;
                    }
                }

                if (valid)
                {
                    if (firstvalid < 0)
                    {
                        firstvalid = int(k) - 1;
                    }
                    lastvalid = int(k);
                }
                else
                {
                    if (lastvalid != 0)
                    {
                        break;
                    }
                }
            }

            if (lastvalid == 0 || firstvalid < 0)
            {
                sample.descriptions.clear();
                continue;
            }

            if (firstvalid > 0 || lastvalid < int(sample.descriptions.size()) - 1)
            {
                std::vector<PixelDescription> replace;
                for (int pos = firstvalid; pos <= lastvalid; ++pos)
                {
                    replace.push_back(sample.descriptions[pos]);
                }
                sample.descriptions = replace;
            }
        }
    }

    // Get a counter for all unique descriptors
    using Coordinates = std::pair<int, int>;
    using CoordinatesList = std::vector<Coordinates>;
    using Counters = std::map<UniqueDescriptor, CoordinatesList>;

    Counters counters;
    {
        std::vector<Counters> counters_vec(omp_get_max_threads());

        #pragma omp parallel for
        for (int i = params.radius; i < samples.Height() - params.radius; ++i)
        {
            Counters & counters_thread = counters_vec[omp_get_thread_num()];

            for (int j = params.radius; j < samples.Width() - params.radius; ++j)
            {
                ImageSample & sample = samples(i, j);
                UniqueDescriptor desc;

                for (int k = 0; k < sample.descriptions.size(); ++k)
                {
                    desc.exposure = sample.descriptions[k].exposure;

                    for (int channel = 0; channel < 3; ++channel)
                    {
                        desc.channel = channel;
                        // Get quantized value
                        desc.quantizedValue = int(std::round(sample.descriptions[k].mean(channel)  * (channelQuantization - 1)));
                        if (desc.quantizedValue < 0 || desc.quantizedValue >= channelQuantization)
                        {
                            continue;
                        }                        
                        Coordinates coordinates = std::make_pair(sample.x, sample.y);
                        counters_thread[desc].push_back(coordinates);
                    }
                }
            }
        }

        for (int i = 0; i < omp_get_max_threads(); ++i)
        {
            for (auto & item : counters_vec[i])
            {
                auto found = counters.find(item.first);
                if (found != counters.end())
                {
                    found->second.insert(found->second.end(), item.second.begin(), item.second.end());
                }
                else
                {
                    counters[item.first] = item.second;
                }
            }
        }
    }

    for (auto & item : counters)
    {
        if (item.second.size() > params.maxCountSample)
        {
            // Shuffle and ignore the exceeding samples
            std::random_shuffle(item.second.begin(), item.second.end());
            item.second.resize(params.maxCountSample);
        }

        for (std::size_t l = 0; l < item.second.size(); ++l)
        {
            Coordinates coords = item.second[l];

            if (!samples(coords.second, coords.first).descriptions.empty())
            {
                out_samples.push_back(samples(coords.second, coords.first));
                samples(coords.second, coords.first).descriptions.clear();
            }
        }
    }

    return true;
}

void Sampling::analyzeSource(std::vector<ImageSample> & samples, int channelQuantization, int imageIndex)
{
    for (std::size_t sampleIndex = 0; sampleIndex < samples.size(); ++sampleIndex)
    {
        ImageSample & sample = samples[sampleIndex];

        for (auto & desc : sample.descriptions)
        {
            UniqueDescriptor udesc;
            udesc.exposure = desc.exposure;
            
            for (int channel = 0; channel < 3; ++channel)
            {
                udesc.channel = channel;
                udesc.quantizedValue = int(std::round(desc.mean(channel)  * (channelQuantization - 1)));
                if (udesc.quantizedValue < 0 || udesc.quantizedValue >= channelQuantization)
                {
                    continue;
                }

                Coordinates c;
                c.imageIndex = imageIndex;
                c.sampleIndex = sampleIndex;

                _positions[udesc].push_back(c);
            }
        }
    }

    for (auto & item : _positions)
    {
        if (item.second.size() > 500)
        {
            // Shuffle and ignore the exceeding samples
            std::random_shuffle(item.second.begin(), item.second.end());
            item.second.resize(500);
        }
    }
}

void Sampling::filter(size_t maxTotalPoints)
{
    size_t limitPerGroup = 510;
    size_t total_points = maxTotalPoints + 1;

    while (total_points > maxTotalPoints)
    {
        limitPerGroup = limitPerGroup - 10;

        total_points = 0;
        for (auto & item : _positions)
        {
            if (item.second.size() > limitPerGroup)
            {
                // Shuffle and ignore the exceeding samples
                std::random_shuffle(item.second.begin(), item.second.end());
                item.second.resize(limitPerGroup);
            }

            total_points += item.second.size();
        }
    }
}

void Sampling::extractUsefulSamples(std::vector<ImageSample> & out_samples, std::vector<ImageSample> & samples, int imageIndex)
{
    std::set<unsigned int> unique_indices;

    for (auto & item : _positions)
    {
        for (auto & pos : item.second)
        {
            if (pos.imageIndex == imageIndex)
            {
                unique_indices.insert(pos.sampleIndex);
            }
        }
    }

    for (auto & index : unique_indices)
    {
        if (samples[index].descriptions.size() > 0)
        {
            out_samples.push_back(samples[index]);
            samples[index].descriptions.clear();
        }
    }
}

} // namespace hdr
} // namespace aliceVision
