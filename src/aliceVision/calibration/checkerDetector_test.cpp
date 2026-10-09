// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/calibration/checkerDetector.hpp>

#define BOOST_TEST_MODULE checkerDetector
#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>
#include <aliceVision/unitTest.hpp>

#include <cmath>
#include <limits>
#include <vector>

using namespace aliceVision;
using namespace aliceVision::calibration;

namespace {

/**
 * @brief Synthetic checkerboard seen through a sensor with a given pixel aspect ratio.
 *
 * The board is defined in physical (square) coordinates.
 * An anamorphic sensor squeezes the width: physical point (X, Y) lands at sensor pixel (X / pa, Y).
 */
struct SyntheticBoard
{
    int sensorWidth;
    int sensorHeight;
    double pixelAspectRatio;
    int squaresX;
    int squaresY;
    double squareSize;  // physical pixels
    double angle;       // radians
    Vec2 center;        // physical coordinates

    Vec2 boardToPhysical(const Vec2& b) const
    {
        const double c = std::cos(angle);
        const double s = std::sin(angle);
        const Vec2 local((b.x() - squaresX * 0.5) * squareSize, (b.y() - squaresY * 0.5) * squareSize);
        return center + Vec2(c * local.x() - s * local.y(), s * local.x() + c * local.y());
    }

    Vec2 physicalToBoard(const Vec2& p) const
    {
        const double c = std::cos(angle);
        const double s = std::sin(angle);
        const Vec2 d = p - center;
        const Vec2 local(c * d.x() + s * d.y(), -s * d.x() + c * d.y());
        return Vec2(local.x() / squareSize + squaresX * 0.5, local.y() / squareSize + squaresY * 0.5);
    }

    double intensity(const Vec2& physical) const
    {
        const Vec2 b = physicalToBoard(physical);
        if (b.x() < 0.0 || b.y() < 0.0 || b.x() >= squaresX || b.y() >= squaresY)
        {
            return 1.0;
        }
        const int parity = (static_cast<int>(std::floor(b.x())) + static_cast<int>(std::floor(b.y()))) % 2;
        return parity == 0 ? 0.0 : 1.0;
    }

    /// Area-sampled sensor image (pixel (x, y) covers [x, x+1] x [y, y+1]).
    image::Image<image::RGBColor> render() const
    {
        const int samples = 8;
        image::Image<image::RGBColor> img(sensorWidth, sensorHeight);
        for (int y = 0; y < sensorHeight; ++y)
        {
            for (int x = 0; x < sensorWidth; ++x)
            {
                double sum = 0.0;
                for (int j = 0; j < samples; ++j)
                {
                    for (int i = 0; i < samples; ++i)
                    {
                        const double sx = x + (i + 0.5) / samples;
                        const double sy = y + (j + 0.5) / samples;
                        sum += intensity(Vec2(sx * pixelAspectRatio, sy));
                    }
                }
                const unsigned char v = static_cast<unsigned char>(std::lround(255.0 * sum / (samples * samples)));
                img(y, x) = image::RGBColor(v);
            }
        }
        return img;
    }

    /// Inner corners in sensor coordinates.
    std::vector<Vec2> expectedSensorCorners() const
    {
        std::vector<Vec2> corners;
        for (int j = 1; j < squaresY; ++j)
        {
            for (int i = 1; i < squaresX; ++i)
            {
                const Vec2 p = boardToPhysical(Vec2(i, j));
                corners.emplace_back(p.x() / pixelAspectRatio, p.y());
            }
        }
        return corners;
    }
};

struct CornerErrors
{
    Vec2 maxAbsOffset = Vec2::Zero();
    Vec2 meanOffset = Vec2::Zero();
};

/// Match each expected corner to the closest corner used by a detected board.
CornerErrors compareCorners(const CheckerDetector& detector, const std::vector<Vec2>& expected)
{
    const std::vector<CheckerDetector::CheckerBoardCorner> corners = detector.getCorners();

    std::vector<Vec2> boardCorners;
    for (const auto& board : detector.getBoards())
    {
        for (int i = 0; i < board.rows(); ++i)
        {
            for (int j = 0; j < board.cols(); ++j)
            {
                if (board(i, j) != UndefinedIndexT)
                {
                    boardCorners.push_back(corners[board(i, j)].center);
                }
            }
        }
    }

    CornerErrors errors;
    for (const Vec2& e : expected)
    {
        double best = std::numeric_limits<double>::max();
        Vec2 bestOffset = Vec2::Zero();
        for (const Vec2& c : boardCorners)
        {
            const double d = (c - e).norm();
            if (d < best)
            {
                best = d;
                bestOffset = c - e;
            }
        }
        errors.maxAbsOffset = errors.maxAbsOffset.cwiseMax(bestOffset.cwiseAbs());
        errors.meanOffset += bestOffset;
    }
    errors.meanOffset /= static_cast<double>(expected.size());
    return errors;
}

std::size_t countBoardCorners(const CheckerDetector::CheckerBoard& board)
{
    std::size_t count = 0;
    for (int i = 0; i < board.rows(); ++i)
    {
        for (int j = 0; j < board.cols(); ++j)
        {
            if (board(i, j) != UndefinedIndexT)
            {
                ++count;
            }
        }
    }
    return count;
}

constexpr std::size_t maxLevels = 2;
constexpr std::size_t minConsensus = 5;
// Per-corner precision of the detector, in the (square pixels) image used for detection
constexpr double maxCornerError = 0.6;
// Systematic offset over all corners, in sensor pixels
constexpr double maxCornerBias = 0.1;

/**
 * @param detectionToSensor scale from the image used for detection to the sensor image, per axis
 */
void checkDetection(const SyntheticBoard& synth, const CheckerDetector& detector, const Vec2& detectionToSensor)
{
    BOOST_REQUIRE_EQUAL(detector.getBoards().size(), 1);
    const std::size_t innerCorners = static_cast<std::size_t>((synth.squaresX - 1) * (synth.squaresY - 1));
    BOOST_CHECK_EQUAL(countBoardCorners(detector.getBoards()[0]), innerCorners);

    const CornerErrors errors = compareCorners(detector, synth.expectedSensorCorners());
    BOOST_TEST_MESSAGE("pa=" << synth.pixelAspectRatio << " max offset=" << errors.maxAbsOffset.transpose()
                              << " mean offset=" << errors.meanOffset.transpose());
    BOOST_CHECK_LT(errors.maxAbsOffset.x(), maxCornerError * detectionToSensor.x());
    BOOST_CHECK_LT(errors.maxAbsOffset.y(), maxCornerError * detectionToSensor.y());
    BOOST_CHECK_SMALL(errors.meanOffset.x(), maxCornerBias);
    BOOST_CHECK_SMALL(errors.meanOffset.y(), maxCornerBias);
}

}  // namespace

// Test summary:
// - Render a synthetic 9x7 checkerboard (40px squares, rotated 5 degrees) on a 640x480 image with square pixels
// - Run CheckerDetector::process
// - Check that exactly one board is found with all 8x6 inner corners
// - Check each corner against its analytic position (max error per axis, mean bias)
BOOST_AUTO_TEST_CASE(checkerDetector_process_squarePixels)
{
    const SyntheticBoard synth{640, 480, 1.0, 9, 7, 40.0, 5.0 * M_PI / 180.0, Vec2(320.0, 240.0)};

    CheckerDetector detector;
    BOOST_REQUIRE(detector.process(synth.render(), maxLevels, minConsensus, false, false, false));

    checkDetection(synth, detector, Vec2(1.0, 1.0));
}

// Test summary:
// - Same board and image as checkerDetector_process_squarePixels
// - Run CheckerDetector::detectCheckerboard with a pixel aspect ratio of 1 (no resize)
// - Check that the result matches the analytic corners as with process
BOOST_AUTO_TEST_CASE(checkerDetector_detectCheckerboard_squarePixels)
{
    const SyntheticBoard synth{640, 480, 1.0, 9, 7, 40.0, 5.0 * M_PI / 180.0, Vec2(320.0, 240.0)};

    CheckerDetector detector;
    BOOST_REQUIRE(detector.detectCheckerboard(synth.render(), 1.0, false, maxLevels, minConsensus, false, false, false));

    checkDetection(synth, detector, Vec2(1.0, 1.0));
}

// Test summary:
// - Render a physical 1280x480 scene (8x6 checkerboard, 56px squares) squeezed onto a 640x480 sensor (pixel aspect ratio 2)
// - Run CheckerDetector::detectCheckerboard with the pixel aspect ratio 2
// - Check that corners are returned in sensor coordinates (x = X / pa, y = Y)
//   - per axis error is allowed to scale with the resize ratio, mean bias must stay below 0.1 pixel
BOOST_AUTO_TEST_CASE(checkerDetector_detectCheckerboard_anamorphic2)
{
    // Physical scene 1280x480 squeezed onto a 640x480 sensor
    const SyntheticBoard synth{640, 480, 2.0, 8, 6, 56.0, 5.0 * M_PI / 180.0, Vec2(640.0, 240.0)};

    CheckerDetector detector;
    BOOST_REQUIRE(detector.detectCheckerboard(synth.render(), 2.0, false, maxLevels, minConsensus, false, false, false));

    checkDetection(synth, detector, Vec2(1.0, 2.0));
}

// Test summary:
// - Render a physical ~851x480 scene (9x7 checkerboard, 44px squares) squeezed onto a 640x480 sensor (pixel aspect ratio 1.33)
// - Run CheckerDetector::detectCheckerboard with the pixel aspect ratio 1.33
// - The resized height (480 / 1.33 = 360.9) is rounded to an integer by the resize
// - Check that corners are mapped back with the actual resize ratio: no drift along y (mean bias below 0.1 pixel)
BOOST_AUTO_TEST_CASE(checkerDetector_detectCheckerboard_anamorphic133)
{
    // Physical scene ~851x480 squeezed onto a 640x480 sensor
    const SyntheticBoard synth{640, 480, 1.33, 9, 7, 44.0, 5.0 * M_PI / 180.0, Vec2(425.6, 240.0)};

    CheckerDetector detector;
    BOOST_REQUIRE(detector.detectCheckerboard(synth.render(), 1.33, false, maxLevels, minConsensus, false, false, false));

    checkDetection(synth, detector, Vec2(1.0, 480.0 / 360.0));
}

// Test summary:
// - Render a small 9x7 checkerboard (20px squares) on a 320x240 image with square pixels
// - Run CheckerDetector::detectCheckerboard with doubleSize (detection on the 2x upscaled image)
// - Check that corners are mapped back to the source image coordinates
BOOST_AUTO_TEST_CASE(checkerDetector_detectCheckerboard_doubleSize)
{
    // Small squares, detection is done on the upscaled image
    const SyntheticBoard synth{320, 240, 1.0, 9, 7, 20.0, 5.0 * M_PI / 180.0, Vec2(160.0, 120.0)};

    CheckerDetector detector;
    BOOST_REQUIRE(detector.detectCheckerboard(synth.render(), 1.0, true, maxLevels, minConsensus, false, false, false));

    checkDetection(synth, detector, Vec2(0.5, 0.5));
}

// Test summary:
// - Run CheckerDetector::process on a uniform gray image
// - Check that no checkerboard is detected
BOOST_AUTO_TEST_CASE(checkerDetector_process_emptyImage_noBoard)
{
    const image::Image<image::RGBColor> blank(320, 240, true, image::RGBColor(128));

    CheckerDetector detector;
    detector.process(blank, maxLevels, minConsensus, false, false, false);

    BOOST_CHECK(detector.getBoards().empty());
}
