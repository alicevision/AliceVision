// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/camera/cameraUndistortImage.hpp>
#include <aliceVision/camera/camera_test_common.hpp>

#define BOOST_TEST_MODULE cameraUndistortImage

#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>
#include <aliceVision/unitTest.hpp>

using namespace aliceVision;
using namespace aliceVision::camera;

// The input images store their own pixel coordinates: bilinear sampling of a linear function is exact,
// so each output pixel tells which input position it was sampled from.

namespace {

const int width = 320;
const int height = 240;
const float fill = -1.0f;

void makeCoordinateImages(image::Image<float>& xImage, image::Image<float>& yImage)
{
    xImage.resize(width, height);
    yImage.resize(width, height);
    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            xImage(y, x) = static_cast<float>(x);
            yImage(y, x) = static_cast<float>(y);
        }
    }
}

std::vector<std::shared_ptr<Pinhole>> makeDistortedCameras()
{
    std::vector<std::shared_ptr<Pinhole>> cameras;
    cameras.push_back(std::make_shared<Pinhole>(width, height, 300.0, 300.0, 4.0, -3.0, std::make_shared<DistortionRadialK3>(-0.18, 0.19, -0.025)));

    auto undistortion = createUndistortion(EUNDISTORTION::UNDISTORTION_3DERADIAL4, width, height);
    undistortion->setParameters(test::undistortionTestParameters(EUNDISTORTION::UNDISTORTION_3DERADIAL4));
    cameras.push_back(std::make_shared<Pinhole>(width, height, 300.0, 300.0, 4.0, -3.0, nullptr, undistortion));

    return cameras;
}

/// Where the output pixel (x, y) was sampled from, or false if it was filled.
bool sampledFrom(const image::Image<float>& xOut, const image::Image<float>& yOut, int x, int y, Vec2& source)
{
    if (xOut(y, x) == fill)
    {
        return false;
    }
    source = Vec2(xOut(y, x), yOut(y, x));
    return true;
}

}  // namespace

BOOST_AUTO_TEST_CASE(cameraUndistortImage_noDistortionCopies)
{
    image::Image<float> xImage, yImage;
    makeCoordinateImages(xImage, yImage);

    const Pinhole camera(width, height, 300.0, 300.0, 4.0, -3.0);
    image::Image<float> out;
    UndistortImage(xImage, &camera, out, fill);

    BOOST_REQUIRE_EQUAL(out.width(), width);
    BOOST_REQUIRE_EQUAL(out.height(), height);
    BOOST_CHECK(out == xImage);
}

BOOST_AUTO_TEST_CASE(cameraUndistortImage_samplesDistortedPixel)
{
    image::Image<float> xImage, yImage;
    makeCoordinateImages(xImage, yImage);

    for (const auto& camera : makeDistortedCameras())
    {
        image::Image<float> xOut, yOut;
        UndistortImage(xImage, camera.get(), xOut, fill);
        UndistortImage(yImage, camera.get(), yOut, fill);

        BOOST_REQUIRE_EQUAL(xOut.width(), width);
        BOOST_REQUIRE_EQUAL(xOut.height(), height);

        int sampled = 0;
        int filled = 0;
        double maxError = 0.0;
        double maxRoundTripError = 0.0;
        for (int y = 0; y < height; ++y)
        {
            for (int x = 0; x < width; ++x)
            {
                const Vec2 expected = camera->getDistortedPixel(Vec2(x, y));
                Vec2 source;
                if (!sampledFrom(xOut, yOut, x, y, source))
                {
                    // Only pixels mapping outside of the input image are filled.
                    BOOST_CHECK(expected(0) < 0.0 || expected(1) < 0.0 || expected(0) > width - 1 || expected(1) > height - 1);
                    ++filled;
                    continue;
                }
                ++sampled;

                // The image domain test truncates the position to integers, so positions in (-1, 0) and
                // (size - 1, size) are also sampled, with a clamped neighbor: only the interior is exact.
                if (expected(0) < 0.0 || expected(1) < 0.0 || expected(0) > width - 1 || expected(1) > height - 1)
                {
                    continue;
                }
                maxError = std::max(maxError, (source - expected).norm());
                maxRoundTripError = std::max(maxRoundTripError, (camera->getUndistortedPixel(source) - Vec2(x, y)).norm());
            }
        }

        BOOST_CHECK_GT(sampled, width * height / 2);
        BOOST_CHECK_SMALL(maxError, 1e-3);
        BOOST_CHECK_SMALL(maxRoundTripError, 1e-2);
    }
}

// With the principal point correction, the output is centered on the principal point.
BOOST_AUTO_TEST_CASE(cameraUndistortImage_correctPrincipalPoint)
{
    image::Image<float> xImage, yImage;
    makeCoordinateImages(xImage, yImage);

    const auto camera = makeDistortedCameras().front();
    const Vec2 correction = camera->getPrincipalPoint() - Vec2(width * 0.5, height * 0.5);

    image::Image<float> xOut, yOut;
    UndistortImage(xImage, camera.get(), xOut, fill, true);
    UndistortImage(yImage, camera.get(), yOut, fill, true);

    for (int y = 20; y < height - 20; y += 20)
    {
        for (int x = 20; x < width - 20; x += 20)
        {
            Vec2 source;
            BOOST_REQUIRE(sampledFrom(xOut, yOut, x, y, source));
            const Vec2 expected = camera->getDistortedPixel(Vec2(x, y) + correction);
            BOOST_CHECK_SMALL((source - expected).norm(), 1e-3);
        }
    }
}

BOOST_AUTO_TEST_CASE(cameraUndistortImage_roi)
{
    image::Image<float> xImage, yImage;
    makeCoordinateImages(xImage, yImage);

    const auto camera = makeDistortedCameras().front();

    image::Image<float> xFull, yFull;
    UndistortImage(xImage, camera.get(), xFull, fill);
    UndistortImage(yImage, camera.get(), yFull, fill);

    const oiio::ROI roi(100, 300, 50, 150);
    image::Image<float> xRoi, yRoi;
    UndistortImage(xImage, camera.get(), xRoi, fill, false, roi);
    UndistortImage(yImage, camera.get(), yRoi, fill, false, roi);

    BOOST_REQUIRE_EQUAL(xRoi.width(), 200);
    BOOST_REQUIRE_EQUAL(xRoi.height(), 100);
    for (int y = 0; y < xRoi.height(); ++y)
    {
        for (int x = 0; x < xRoi.width(); ++x)
        {
            BOOST_CHECK_EQUAL(xRoi(y, x), xFull(y + 50, x + 100));
            BOOST_CHECK_EQUAL(yRoi(y, x), yFull(y + 50, x + 100));
        }
    }
}

// Undistorting to a camera without distortion and the same calibration is the single camera version.
BOOST_AUTO_TEST_CASE(cameraUndistortImage_toOtherIntrinsic)
{
    image::Image<float> xImage, yImage;
    makeCoordinateImages(xImage, yImage);

    for (const auto& camera : makeDistortedCameras())
    {
        const Pinhole output(width, height, camera->getScale()(0), camera->getScale()(1), camera->getOffset()(0), camera->getOffset()(1));

        image::Image<float> expected, out;
        UndistortImage(xImage, camera.get(), expected, fill);
        UndistortImage(xImage, camera.get(), &output, nullptr, out, fill);

        BOOST_REQUIRE_EQUAL(out.width(), expected.width());
        BOOST_REQUIRE_EQUAL(out.height(), expected.height());
        BOOST_CHECK_SMALL((Eigen::MatrixXf(out) - Eigen::MatrixXf(expected)).cwiseAbs().maxCoeff(), 1e-3f);

        // A larger output image keeps the same mapping around the principal point.
        const Pinhole larger(width + 40, height + 20, camera->getScale()(0), camera->getScale()(1), camera->getOffset()(0), camera->getOffset()(1));
        image::Image<float> largerOut;
        UndistortImage(xImage, camera.get(), &larger, nullptr, largerOut, fill);
        BOOST_CHECK_EQUAL(largerOut.width(), width + 40);
        BOOST_CHECK_EQUAL(largerOut.height(), height + 20);
        BOOST_CHECK_SMALL(largerOut(height / 2 + 10, width / 2 + 20) - expected(height / 2, width / 2), 1e-3f);
    }
}
