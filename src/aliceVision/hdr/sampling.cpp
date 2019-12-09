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

void extractSamples(
    std::vector<std::vector<ImageSamples>>& out_samples,
    const std::vector<std::vector<std::string>>& imagePathsGroups,
    const std::vector< std::vector<float> >& cameraExposures,
    int nbPoints,
    int calibrationDownscale,
    bool fisheye
    )
{
    const int nbGroups = imagePathsGroups.size();
    out_samples.resize(nbGroups);

    int averallNbImages = 0;
    for (const auto& imgPaths : imagePathsGroups)
    {
        averallNbImages += imgPaths.size();
    }
    const int samplesPerImage = nbPoints / averallNbImages;

    ALICEVISION_LOG_TRACE("samplesPerImage: " << samplesPerImage);

    #pragma omp parallel for num_threads(3)
    for (int g = 0; g<nbGroups; ++g)
    {
        std::vector<ImageSamples>& out_hdrSamples = out_samples[g];

        const std::vector<std::string > &imagePaths = imagePathsGroups[g];
        out_hdrSamples.resize(imagePaths.size());

        const std::vector<float>& exposures = cameraExposures[g];

        for (unsigned int i = 0; i < imagePaths.size(); ++i)
        {
            out_hdrSamples[i].exposure = exposures[i];
            out_hdrSamples[i].colors.reserve(samplesPerImage);
            std::vector<Rgb<double>>& colors = out_hdrSamples[i].colors;

            Image<RGBfColor> img;
            readImage(imagePaths[i], img, EImageColorSpace::LINEAR);
            if (calibrationDownscale != 1.0f)
            {
                unsigned int w = img.Width();
                unsigned int h = img.Height();
                unsigned int nw = (unsigned int)(floor(float(w) / calibrationDownscale));
                unsigned int nh = (unsigned int)(floor(float(h) / calibrationDownscale));

                image::Image<image::RGBfColor> rescaled(nw, nh);

                oiio::ImageSpec imageSpecResized(nw, nh, 3, oiio::TypeDesc::FLOAT);
                oiio::ImageSpec imageSpecOrigin(w, h, 3, oiio::TypeDesc::FLOAT);
                oiio::ImageBuf bufferOrigin(imageSpecOrigin, img.data());
                oiio::ImageBuf bufferResized(imageSpecResized, rescaled.data());
                oiio::ImageBufAlgo::resample(bufferResized, bufferOrigin);

                const oiio::ImageBuf inBuf(oiio::ImageSpec(w, h, 3, oiio::TypeDesc::FLOAT), img.data());
                oiio::ImageBuf outBuf(oiio::ImageSpec(nw, nh, 3, oiio::TypeDesc::FLOAT), rescaled.data());

                oiio::ImageBufAlgo::resize(outBuf, inBuf);
                img.swap(rescaled);
            }

            const std::size_t width = img.Width();
            const std::size_t height = img.Height();

            const std::size_t minSize = std::min(width, height) * 0.97;
            const Vec2i center(width / 2, height / 2);

            const int xMin = std::ceil(center(0) - minSize / 2);
            const int yMin = std::ceil(center(1) - minSize / 2);
            const int xMax = std::floor(center(0) + minSize / 2);
            const int yMax = std::floor(center(1) + minSize / 2);
            const std::size_t maxDist2 = pow(minSize * 0.5, 2);

            const int step = std::ceil(minSize / sqrt(samplesPerImage));

            // extract samples
            for (int y = yMin; y <= yMax - step; y += step)
            {
                for (int x = xMin; x <= xMax - step; x += step)
                {
                    if (fisheye)
                    {
                        std::size_t dist2 = pow(center(0) - x, 2) + pow(center(1) - y, 2);
                        if (dist2 > maxDist2)
                            continue;
                    }
                    RGBfColor& c = img(y, x);
                    colors.push_back(Rgb<double>(c(0), c(1), c(2)));
                }
            }
        }
    }
}



} // namespace hdr
} // namespace aliceVision
