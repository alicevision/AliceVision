// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/image/all.hpp>

#include <string>
#include <sstream>

#define BOOST_TEST_MODULE ImageResampling

#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>

using namespace aliceVision;
using namespace aliceVision::image;

BOOST_AUTO_TEST_CASE(Resampling_SampleSamePosition)
{
    Image<unsigned char> image;
    std::string pngFilename = std::string(THIS_SOURCE_DIR) + "/image_test/lena.png";
    ALICEVISION_LOG_DEBUG(pngFilename);
    BOOST_CHECK_NO_THROW(readImage(pngFilename, image, image::EImageColorSpace::NO_CONVERSION));

    // Build sampling grid
    std::vector<std::pair<float, float>> samplingGrid;
    for (int i = 0; i < image.height(); ++i)
    {
        for (int j = 0; j < image.width(); ++j)
        {
            samplingGrid.push_back(std::make_pair(i, j));
        }
    }

    // Resample image
    Sampler2d<SamplerLinear> sampler;

    Image<unsigned char> imageOut;

    genericResample(image, samplingGrid, image.width(), image.height(), sampler, imageOut);

    std::string outFilename = ("test_resample_same.png");
    BOOST_CHECK_NO_THROW(writeImage(outFilename, imageOut, image::ImageWriteOptions().toColorSpace(image::EImageColorSpace::NO_CONVERSION)));
}

// Iterative image rotations
// Allow to check if the sampling function have some signal loss.
template<typename SamplerT, typename ImageT>
void imageRotation(const ImageT& imageIn, const SamplerT& sampler, const std::string& samplerString)
{
    bool bOk = true;
    ImageT image = imageIn;

    const int nbRot = 6;
    const float delta = (2.0 * 3.141592) / nbRot;

    const float middleX = image.width() / 2.0;
    const float middleY = image.height() / 2.0;

    // Rotate image then set starting image as source
    for (int idRot = 0; idRot < nbRot; ++idRot)
    {
        // angle of rotation (negative because it's inverse transformation)
        const float curAngle = delta;

        const float cs = cosf(-curAngle);
        const float ss = sinf(-curAngle);

        std::vector<std::pair<float, float>> samplingGrid;
        // Compute sampling grid
        for (int i = 0; i < image.height(); ++i)
        {
            for (int j = 0; j < image.width(); ++j)
            {
                // Compute rotation of pixel (i,j) around center of image

                // Center pixel
                const float dx = static_cast<float>(j) - middleX;
                const float dy = static_cast<float>(i) - middleY;

                const float rotatedX = cs * dx - ss * dy;
                const float rotatedY = ss * dx + cs * dy;

                // Get back to original center
                const float curX = rotatedX + middleX;
                const float curY = rotatedY + middleY;

                samplingGrid.push_back(std::make_pair(curY, curX));
            }
        }

        // Sample input image
        ImageT imageOut;

        genericResample(image, samplingGrid, image.width(), image.height(), sampler, imageOut);

        std::stringstream str;
        str << "test_resample_" << samplerString << "_rotate_" << idRot << ".png";
        writeImage(str.str(), imageOut, image::ImageWriteOptions().toColorSpace(image::EImageColorSpace::NO_CONVERSION));
        image = imageOut;
    }
}

BOOST_AUTO_TEST_CASE(Resampling_SampleRotate)
{
    Image<RGBColor> image;

    std::string pngFilename = std::string(THIS_SOURCE_DIR) + "/image_test/lena.png";
    ALICEVISION_LOG_DEBUG(pngFilename);
    BOOST_CHECK_NO_THROW(readImage(pngFilename, image, image::EImageColorSpace::NO_CONVERSION));

    BOOST_CHECK_NO_THROW(imageRotation(image, Sampler2d<SamplerNearest>(), "SamplerNearest"));
    BOOST_CHECK_NO_THROW(imageRotation(image, Sampler2d<SamplerLinear>(), "SamplerLinear"));
    BOOST_CHECK_NO_THROW(imageRotation(image, Sampler2d<SamplerCubic>(), "SamplerCubic"));
    BOOST_CHECK_NO_THROW(imageRotation(image, Sampler2d<SamplerSpline16>(), "SamplerSpline16"));
    BOOST_CHECK_NO_THROW(imageRotation(image, Sampler2d<SamplerSpline64>(), "SamplerSpline64"));
}
