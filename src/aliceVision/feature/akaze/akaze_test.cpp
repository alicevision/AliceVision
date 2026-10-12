// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/feature/akaze/AKAZE.hpp>

#include <cmath>
#include <vector>

#define BOOST_TEST_MODULE akaze

#include <boost/test/unit_test.hpp>

using namespace aliceVision;
using namespace aliceVision::feature;

namespace {

/// Build an image made of Gaussian blobs of various sizes, so that keypoints are found on several octaves
image::Image<float> makeBlobImage(int size)
{
    image::Image<float> img(size, size, true, 0.f);
    const float sigmas[] = {2.f, 3.f, 5.f, 8.f};
    const int nbBlobsPerRow = 4;
    const float step = static_cast<float>(size) / nbBlobsPerRow;

    for (int by = 0; by < nbBlobsPerRow; ++by)
    {
        for (int bx = 0; bx < nbBlobsPerRow; ++bx)
        {
            const float cx = (bx + 0.5f) * step;
            const float cy = (by + 0.5f) * step;
            const float s = sigmas[(bx + by) % 4];
            for (int y = 0; y < size; ++y)
            {
                for (int x = 0; x < size; ++x)
                {
                    const float d2 = (x - cx) * (x - cx) + (y - cy) * (y - cy);
                    img(y, x) += std::exp(-d2 / (2.f * s * s));
                }
            }
        }
    }
    return img;
}

}  // namespace

// The keypoints detected on slice (p, q) must reference the slice they were detected on.
// Use a configuration where the number of octaves differs from the number of slices per octave
// so that a wrong slice index cannot go unnoticed.
BOOST_AUTO_TEST_CASE(AKAZE_featureDetection_sliceIndex)
{
    AKAZEOptions options;
    options.nbOctaves = 2;
    options.nbSlicePerOctave = 3;
    options.threshold = 0.0001f;
    options.maxTotalKeypoints = 0;

    const image::Image<float> img = makeBlobImage(256);

    AKAZE akaze(img, options);
    akaze.computeScaleSpace();

    std::vector<AKAZEKeypoint> keypoints;
    akaze.featureDetection(keypoints);

    BOOST_REQUIRE(!keypoints.empty());

    const std::vector<AKAZE::TEvolution>& slices = akaze.getSlices();
    BOOST_REQUIRE_EQUAL(slices.size(), static_cast<std::size_t>(options.nbOctaves * options.nbSlicePerOctave));

    std::size_t nbKeypointsUpperOctaves = 0;
    for (const AKAZEKeypoint& kp : keypoints)
    {
        BOOST_REQUIRE_LT(kp.class_id, slices.size());
        BOOST_CHECK_EQUAL(kp.class_id / options.nbSlicePerOctave, kp.octave);

        if (kp.octave > 0)
            ++nbKeypointsUpperOctaves;

        // retrieve the pixel the keypoint was detected on, in its slice resolution
        const float ratio = static_cast<float>(1 << kp.octave);
        const int ix = static_cast<int>(std::lround((kp.x - 0.5f * (ratio - 1.f)) / ratio));
        const int iy = static_cast<int>(std::lround((kp.y - 0.5f * (ratio - 1.f)) / ratio));

        const image::Image<float>& Lhess = slices[kp.class_id].Lhess;
        BOOST_REQUIRE(ix >= 0 && ix < Lhess.width() && iy >= 0 && iy < Lhess.height());

        // the keypoint response is the Hessian determinant value of the slice it was detected on
        BOOST_CHECK_CLOSE(kp.response, std::abs(Lhess(iy, ix)), 1e-4);
    }

    // the test is only meaningful if keypoints are found beyond the first octave
    BOOST_CHECK_GT(nbKeypointsUpperOctaves, 0);
}
