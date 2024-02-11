#pragma once
#include <aliceVision/image/all.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/utils/filesIO.hpp>

#include "rgbCurve.hpp"
#include "sampling.hpp"

#include <filesystem>
#include <random>
#include <array>

namespace aliceVision {
namespace hdr {
namespace test {

bool extractSamplesGroups(std::vector<std::vector<ImageSample>>& out_samples,
                          const std::vector<std::vector<std::string>>& imagePaths,
                          const std::vector<std::vector<double>>& times,
                          const size_t channelQuantization)
{
    std::vector<std::vector<ImageSample>> nonFilteredSamples;
    out_samples.resize(imagePaths.size());

    using SampleRef = std::pair<int, int>;
    using SampleRefList = std::vector<SampleRef>;
    using MapSampleRefList = std::map<UniqueDescriptor, SampleRefList>;

    MapSampleRefList mapSampleRefList;

    for (int idGroup = 0; idGroup < imagePaths.size(); idGroup++)
    {
        int width = 0;
        int height = 0;
        image::readImageSize(imagePaths[idGroup][0], width, height);

        image::ImageReadOptions imgReadOptions;
        imgReadOptions.rawColorInterpretation = image::ERawColorInterpretation::LibRawWhiteBalancing;
        imgReadOptions.workingColorSpace = image::EImageColorSpace::LINEAR;

        std::vector<IndexT> viewIds;
        for (size_t id = 0; id < imagePaths[idGroup].size(); ++id)
        {
            viewIds.push_back(id);
        }

        std::vector<ImageSample> groupSamples;
        if (!Sampling::extractSamplesFromImages(
              groupSamples, imagePaths[idGroup], viewIds, times[idGroup], width, height, channelQuantization, imgReadOptions, Sampling::Params{}))
        {
            return false;
        }

        nonFilteredSamples.push_back(groupSamples);
    }

    for (int idGroup = 0; idGroup < imagePaths.size(); idGroup++)
    {
        std::vector<ImageSample>& groupSamples = nonFilteredSamples[idGroup];

        for (int idSample = 0; idSample < groupSamples.size(); idSample++)
        {
            SampleRef s;
            s.first = idGroup;
            s.second = idSample;

            const ImageSample& sample = groupSamples[idSample];

            for (int idDesc = 0; idDesc < sample.descriptions.size(); idDesc++)
            {
                UniqueDescriptor desc;
                desc.exposure = sample.descriptions[idDesc].exposure;

                for (int channel = 0; channel < 3; channel++)
                {
                    desc.channel = channel;

                    /* Get quantized value */
                    desc.quantizedValue = int(std::round(sample.descriptions[idDesc].mean(channel) * (channelQuantization - 1)));
                    if (desc.quantizedValue < 0 || desc.quantizedValue >= channelQuantization)
                    {
                        continue;
                    }

                    mapSampleRefList[desc].push_back(s);
                }
            }
        }
    }

    const size_t maxCountSample = 100;
    std::random_device rd;
    std::mt19937 gen(rd());
    for (auto& list : mapSampleRefList)
    {
        if (list.second.size() > maxCountSample)
        {
            /*Shuffle and ignore the exceeding samples*/
            std::shuffle(list.second.begin(), list.second.end(), gen);
            list.second.resize(maxCountSample);
        }

        for (auto& item : list.second)
        {
            if (nonFilteredSamples[item.first][item.second].descriptions.size() > 0)
            {
                out_samples[item.first].push_back(nonFilteredSamples[item.first][item.second]);
                nonFilteredSamples[item.first][item.second].descriptions.clear();
            }
        }
    }

    return true;
}

bool buildBrackets(std::vector<std::string>& paths, std::vector<double>& times, const rgbCurve& gt_response)
{
    /* Exposure time for each bracket */
    times = {0.05f, 0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f, 0.7f, 0.8f, 0.9f, 1.0f, 1.1f};

    std::default_random_engine generator;
    std::uniform_real_distribution<float> distribution(0.0f, 1.0f);

    const size_t size_region = 11;
    const size_t regions_count_per_row = 64;
    const size_t regions_count_per_col = 64;

    /* Generate a random image made of flat blocks (to pass variancetest )*/
    image::Image<image::RGBfColor> img(512, 512, true, image::RGBfColor(0.0f));
    int val = 0;
    for (int i = 0; i < img.height(); i++)
    {
        int y = i / size_region;

        for (int j = 0; j < img.width(); j++)
        {
            int x = j / size_region;
            int val = y * regions_count_per_col + x;
            val = val % 4096;

            float r = float(val) / 1023.0;
            float g = float(val) / 1023.0;
            float b = float(val) / 1023.0;

            img(i, j) = image::RGBfColor(r, g, b);
        }
    }

    for (double time : times)
    {
        image::Image<image::RGBfColor> img_bracket(img.width(), img.height());
        for (int i = 0; i < img.height(); i++)
        {
            for (int j = 0; j < img.width(); j++)
            {
                image::RGBfColor color = img(i, j);

                for (int k = 0; k < 3; k++)
                {
                    float radiance = color[k];
                    float radiance_dt = radiance * time;
                    float val = gt_response(radiance_dt, k);
                    val = std::min(1.0f, val);
                    img_bracket(i, j)[k] = val;
                }
            }
        }

        std::filesystem::path temp = std::filesystem::temp_directory_path();
        temp /= utils::generateUniqueFilename();
        temp += ".exr";

        ALICEVISION_LOG_INFO("writing to " << temp.string());

        image::writeImage(temp.string(), img_bracket, image::ImageWriteOptions().toColorSpace(image::EImageColorSpace::LINEAR));
        paths.push_back(temp.string());
    }

    return true;
}

}  // namespace test
}  // namespace hdr
}  // namespace aliceVision
