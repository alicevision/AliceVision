// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/feature/sift/SIFT.hpp>
#include <aliceVision/feature/regionsFactory.hpp>
#include <aliceVision/alicevision_omp.hpp>

#include <random>

#define BOOST_TEST_MODULE sift

#include <boost/test/unit_test.hpp>

using namespace aliceVision;
using namespace aliceVision::feature;

namespace {

/// Build a smoothed noise image: textured enough to give many keypoints with several orientations
image::Image<float> makeTexturedImage(int size)
{
    std::mt19937 generator(42);
    std::uniform_real_distribution<float> distribution(0.f, 1.f);

    image::Image<float> noise(size, size);
    for (int y = 0; y < size; ++y)
        for (int x = 0; x < size; ++x)
            noise(y, x) = distribution(generator);

    // Box blur so the structures span a few pixels
    const int radius = 2;
    image::Image<float> img(size, size, true, 0.f);
    for (int y = radius; y < size - radius; ++y)
    {
        for (int x = radius; x < size - radius; ++x)
        {
            float sum = 0.f;
            for (int dy = -radius; dy <= radius; ++dy)
                for (int dx = -radius; dx <= radius; ++dx)
                    sum += noise(y + dy, x + dx);
            img(y, x) = sum / ((2 * radius + 1) * (2 * radius + 1));
        }
    }
    return img;
}

std::unique_ptr<Regions> extractWithThreads(const image::Image<float>& img, int nbThreads)
{
    omp_set_num_threads(nbThreads);
    std::unique_ptr<Regions> regions;
    SiftParams params;
    BOOST_REQUIRE(extractSIFT<unsigned char>(img, regions, params, true, nullptr));
    return regions;
}

}  // namespace

/// The features and descriptors must not depend on the number of threads, nor on their scheduling
BOOST_AUTO_TEST_CASE(sift_outputDoesNotDependOnThreads)
{
    VLFeatInstance::initialize();

    const image::Image<float> img = makeTexturedImage(256);

    const std::unique_ptr<Regions> reference = extractWithThreads(img, 1);
    const auto& referenceRegions = dynamic_cast<const SIFT_Regions&>(*reference);
    BOOST_REQUIRE_GT(referenceRegions.RegionCount(), 100);

    for (int run = 0; run < 3; ++run)
    {
        const std::unique_ptr<Regions> other = extractWithThreads(img, 4);
        const auto& otherRegions = dynamic_cast<const SIFT_Regions&>(*other);

        BOOST_TEST_CONTEXT("run " << run)
        {
            BOOST_REQUIRE_EQUAL(otherRegions.RegionCount(), referenceRegions.RegionCount());
            for (std::size_t i = 0; i < referenceRegions.RegionCount(); ++i)
            {
                const PointFeature& f1 = referenceRegions.Features()[i];
                const PointFeature& f2 = otherRegions.Features()[i];
                BOOST_CHECK_EQUAL(f1.x(), f2.x());
                BOOST_CHECK_EQUAL(f1.y(), f2.y());
                BOOST_CHECK_EQUAL(f1.scale(), f2.scale());
                BOOST_CHECK_EQUAL(f1.orientation(), f2.orientation());
                BOOST_CHECK(referenceRegions.Descriptors()[i] == otherRegions.Descriptors()[i]);
            }
        }
    }

    VLFeatInstance::destroy();
}
