// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/calibration/distortionEstimationLine.hpp>
#include <aliceVision/calibration/distortionEstimation_test_common.hpp>
#include <aliceVision/camera/UndistortionRadial.hpp>
#include <aliceVision/camera/camera.hpp>

#define BOOST_TEST_MODULE distortionEstimationLine
#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>
#include <aliceVision/unitTest.hpp>

#include <cmath>
#include <memory>
#include <vector>

using namespace aliceVision;
using namespace aliceVision::calibration;
using namespace aliceVision::calibration::test;

namespace {

/**
 * @brief Grid of slightly rotated horizontal and vertical physical lines, distorted by a radial sensor.
 * Line parameters are initialized from the distorted end points only.
 */
std::vector<LineWithPoints> radialLines(const RadialSensor& sensor)
{
    const double rotation = 3.0 * M_PI / 180.0;
    const Vec2 u(std::cos(rotation), std::sin(rotation));
    const Vec2 v(-u.y(), u.x());

    std::vector<LineWithPoints> result;
    for (int direction = 0; direction < 2; ++direction)
    {
        const Vec2 along = direction == 0 ? u : v;
        const Vec2 across = direction == 0 ? v : u;

        for (double offset = -0.6; offset <= 0.6001; offset += 0.1)
        {
            LineWithPoints line;
            for (double t = -1.2; t <= 1.2001; t += 0.02)
            {
                // Straight line in normalized undistorted desqueezed coordinates
                const Vec2 p = sensor.distort(sensor.unnormalize(across * offset + along * t));
                if (sensor.inside(p))
                {
                    line.points.push_back({p, 0.0});
                }
            }

            if (line.points.size() < 10)
            {
                continue;
            }

            // Initial line from the distorted end points, in desqueezed pixel coordinates (y / pa)
            const double pa = sensor.pixelAspectRatio;
            const Vec2 a(line.points.front().center.x(), line.points.front().center.y() / pa);
            const Vec2 b(line.points.back().center.x(), line.points.back().center.y() / pa);
            const Vec2 dir = (b - a).normalized();
            const Vec2 normal(-dir.y(), dir.x());
            line.angle = std::atan2(normal.y(), normal.x());
            line.dist = normal.dot(a);

            result.push_back(line);
        }
    }
    return result;
}


/**
 * @brief Ground truth parameters for each undistortion model, initial values and locks used by distortionCalibration.
 */
struct ModelCase
{
    camera::EUNDISTORTION type;
    std::vector<double> truth;
    std::vector<double> initial;
    std::vector<bool> locks;
    // Parameters acting as a linear map on the undistorted image keep lines straight: lines cannot observe them
    std::vector<bool> unobservable;
};

const std::vector<ModelCase> allModels = {
  {camera::EUNDISTORTION::UNDISTORTION_RADIALK3, {-0.12, 0.025, -0.004}, {0.0, 0.0, 0.0}, {false, false, false}, {false, false, false}},
  {camera::EUNDISTORTION::UNDISTORTION_3DECLASSICLD,
   {-0.08, 1.02, 0.01, -0.015, 0.01},
   {0.0, 1.0, 0.0, 0.0, 0.0},
   {false, false, false, false, false},
   {false, false, false, false, false}},
  {camera::EUNDISTORTION::UNDISTORTION_3DERADIAL4,
   {-0.08, 0.002, -0.003, 0.01, 0.001, -0.001, 0.3, 0.02},
   {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
   {false, false, false, false, false, false, false, false},
   // Cylindric direction and bending
   {false, false, false, false, false, false, true, true}},
  // Squeeze cannot be observed from lines: locked to 1
  {camera::EUNDISTORTION::UNDISTORTION_3DEANAMORPHIC4,
   {-0.08, -0.07, 0.01, -0.01, 0.01, 0.012, 0.002, -0.002, 0.001, 0.001, 0.01, 1.0, 1.0},
   {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0},
   {false, false, false, false, false, false, false, false, false, false, false, true, true},
   {false, false, false, false, false, false, false, false, false, false, false, false, false}},
};


/**
 * @brief Grid of slightly rotated straight lines in undistorted sensor coordinates, distorted by a ground truth model.
 * Line parameters are initialized from the distorted end points only.
 */
std::vector<LineWithPoints> distortedLines(const camera::Undistortion& truth)
{
    const double width = truth.getSize().x();
    const double height = truth.getSize().y();
    const double pa = truth.getPixelAspectRatio();
    const Vec2 center(width / 2.0, height / 2.0);

    const double rotation = 3.0 * M_PI / 180.0;
    const Vec2 u(std::cos(rotation), std::sin(rotation));
    const Vec2 v(-u.y(), u.x());

    std::vector<LineWithPoints> result;
    for (int direction = 0; direction < 2; ++direction)
    {
        const Vec2 along = direction == 0 ? u : v;
        const Vec2 across = direction == 0 ? v : u;

        for (double offset = -0.45; offset <= 0.4501; offset += 0.075)
        {
            LineWithPoints line;
            for (double t = -0.7; t <= 0.7001; t += 0.01)
            {
                const Vec2 p = truth.inverse(center + (across * offset + along * t) * width);
                if (p.x() >= 0.0 && p.y() >= 0.0 && p.x() < width && p.y() < height)
                {
                    line.points.push_back({p, 0.0});
                }
            }

            if (line.points.size() < 10)
            {
                continue;
            }

            const Vec2 a(line.points.front().center.x(), line.points.front().center.y() / pa);
            const Vec2 b(line.points.back().center.x(), line.points.back().center.y() / pa);
            const Vec2 dir = (b - a).normalized();
            const Vec2 normal(-dir.y(), dir.x());
            line.angle = std::atan2(normal.y(), normal.x());
            line.dist = normal.dot(a);

            result.push_back(line);
        }
    }
    return result;
}

void checkRecovery(const RadialSensor& sensor)
{
    std::vector<LineWithPoints> lines = radialLines(sensor);
    BOOST_REQUIRE_GT(lines.size(), 10);

    std::shared_ptr<camera::Undistortion> undistortion = sensor.makeUndistortion();

    Statistics statistics;
    BOOST_REQUIRE(estimate(undistortion, statistics, lines, true, false, noLock));

    const std::vector<double>& params = undistortion->getParameters();
    BOOST_TEST_MESSAGE("pa=" << sensor.pixelAspectRatio << " desqueezed=" << sensor.isDesqueezed << " max residual=" << statistics.max << " k=" << params[0]
                             << " " << params[1] << " " << params[2]);
    BOOST_CHECK_SMALL(statistics.max, 1e-3);
    BOOST_CHECK_SMALL(params[0] - sensor.k1, 1e-4);
    BOOST_CHECK_SMALL(params[1] - sensor.k2, 1e-4);
    BOOST_CHECK_SMALL(params[2] - sensor.k3, 1e-4);
}

}  // namespace

// Test summary:
// - Generate a grid of physical straight lines (distorted points only) on a 1600x1200 sensor, square pixels
//   - the ground truth is a radial k1/k2/k3 distortion written independently of camera::Undistortion
//     (radial only on purpose: see RadialSensor in distortionEstimation_test_common.hpp)
//   - the sensor squeezes the width by the pixel aspect ratio: physical (X, Y) lands at sensor pixel (X / pa, Y)
// - Estimate an UndistortionRadialK3 starting from zero parameters (distortion center locked)
// - Check that the residual is negligible and that k1, k2, k3 are recovered
BOOST_AUTO_TEST_CASE(distortionEstimationLine_recover_squarePixels)
{
    checkRecovery(makeSensor(1.0, false));
}

// Test summary:
// - Generate a grid of physical straight lines (distorted points only) on a 1600x1200 sensor, pixel aspect ratio 1.33
//   - the ground truth is a radial k1/k2/k3 distortion written independently of camera::Undistortion
//     (radial only on purpose: see RadialSensor in distortionEstimation_test_common.hpp)
//   - the sensor squeezes the width by the pixel aspect ratio: physical (X, Y) lands at sensor pixel (X / pa, Y)
// - Estimate an UndistortionRadialK3 starting from zero parameters (distortion center locked)
// - Check that the residual is negligible and that k1, k2, k3 are recovered
BOOST_AUTO_TEST_CASE(distortionEstimationLine_recover_anamorphic133)
{
    checkRecovery(makeSensor(1.33, false));
}

// Test summary:
// - Generate a grid of physical straight lines (distorted points only) on a 1600x1200 sensor, pixel aspect ratio 2
//   - the ground truth is a radial k1/k2/k3 distortion written independently of camera::Undistortion
//     (radial only on purpose: see RadialSensor in distortionEstimation_test_common.hpp)
//   - the sensor squeezes the width by the pixel aspect ratio: physical (X, Y) lands at sensor pixel (X / pa, Y)
// - Estimate an UndistortionRadialK3 starting from zero parameters (distortion center locked)
// - Check that the residual is negligible and that k1, k2, k3 are recovered
BOOST_AUTO_TEST_CASE(distortionEstimationLine_recover_anamorphic2)
{
    checkRecovery(makeSensor(2.0, false));
}

// Test summary:
// - Generate a grid of physical straight lines (distorted points only) on a 1600x1200 sensor, pixel aspect ratio 2 on an image already desqueezed (square pixels)
//   - the ground truth is a radial k1/k2/k3 distortion written independently of camera::Undistortion
//     (radial only on purpose: see RadialSensor in distortionEstimation_test_common.hpp)
//   - the sensor squeezes the width by the pixel aspect ratio: physical (X, Y) lands at sensor pixel (X / pa, Y)
// - Estimate an UndistortionRadialK3 flagged as desqueezed starting from zero parameters (distortion center locked)
// - Check that the residual is negligible and that k1, k2, k3 are recovered
BOOST_AUTO_TEST_CASE(distortionEstimationLine_recover_anamorphic2_desqueezed)
{
    checkRecovery(makeSensor(2.0, true));
}

// Test summary:
// - Generate distorted lines on a sensor with a pixel aspect ratio of 2
// - Estimate an UndistortionRadialK3 assuming square pixels
// - Check that the lines cannot be straightened (residual > 1 pixel): the pixel aspect ratio is required
BOOST_AUTO_TEST_CASE(distortionEstimationLine_wrongPixelAspectRatio_cannotStraighten)
{
    // Lines bent on a 2x anamorphic sensor cannot be straightened by a radial model on square pixels
    const RadialSensor sensor = makeSensor(2.0, false);
    std::vector<LineWithPoints> lines = radialLines(sensor);

    auto undistortion = std::make_shared<camera::UndistortionRadialK3>(1600, 1200);

    Statistics statistics;
    BOOST_REQUIRE(estimate(undistortion, statistics, lines, true, false, noLock));

    BOOST_TEST_MESSAGE("max residual with wrong pa=" << statistics.max);
    BOOST_CHECK_GT(statistics.max, 1.0);
}

// Test summary:
// - Generate distorted lines on a sensor with a pixel aspect ratio of 2
// - Estimate with k2 locked to a wrong value (0.05), the distortion center locked to (3, -2) and line angles locked
// - Check that k2, the center and every line angle are exactly unchanged while k1 is estimated
BOOST_AUTO_TEST_CASE(distortionEstimationLine_lockedParameters_unchanged)
{
    const RadialSensor sensor = makeSensor(2.0, false);
    std::vector<LineWithPoints> lines = radialLines(sensor);

    std::shared_ptr<camera::Undistortion> undistortion = sensor.makeUndistortion();
    undistortion->setParameters({0.0, 0.05, 0.0});
    undistortion->setOffset(Vec2(3.0, -2.0));

    std::vector<double> anglesBefore;
    for (const auto& l : lines)
    {
        anglesBefore.push_back(l.angle);
    }

    Statistics statistics;
    BOOST_REQUIRE(estimate(undistortion, statistics, lines, true, true, {false, true, false}));

    BOOST_CHECK_EQUAL(undistortion->getParameters()[1], 0.05);
    BOOST_CHECK_NE(undistortion->getParameters()[0], 0.0);
    BOOST_CHECK_EQUAL(undistortion->getOffset().x(), 3.0);
    BOOST_CHECK_EQUAL(undistortion->getOffset().y(), -2.0);
    for (std::size_t i = 0; i < lines.size(); ++i)
    {
        BOOST_CHECK_EQUAL(lines[i].angle, anglesBefore[i]);
    }
}

// Test summary:
// - Check that estimation fails on: a null undistortion, an empty set of lines,
//   a lock vector whose size does not match the number of parameters
BOOST_AUTO_TEST_CASE(distortionEstimationLine_invalidInput_fails)
{
    const RadialSensor sensor = makeSensor(1.0, false);
    std::shared_ptr<camera::Undistortion> undistortion = sensor.makeUndistortion();
    std::vector<LineWithPoints> lines = radialLines(sensor);
    std::vector<LineWithPoints> noLines;
    Statistics statistics;

    BOOST_CHECK(!estimate(nullptr, statistics, lines, true, false, noLock));
    BOOST_CHECK(!estimate(undistortion, statistics, noLines, true, false, noLock));
    BOOST_CHECK(!estimate(undistortion, statistics, lines, true, false, {false, false}));
}

// Test summary:
// - Generate straight lines distorted with the inverse of a ground truth undistortion model
//   - for each undistortion model: radialk3, 3declassicld, 3deradial4, 3deanamorphic4
//   - for each sensor: pixel aspect ratio 1, 1.33, 2, and 1.33, 2 on already desqueezed images
// - Estimate the same model starting from the initial values and locks used by distortionCalibration
//   (3deanamorphic4 squeeze is locked as it cannot be observed from lines)
// - Check that the lines are straightened (negligible residual) and that the parameters are recovered,
//   except 3deradial4 cylindric direction and bending which keep lines straight and cannot be observed
BOOST_AUTO_TEST_CASE(distortionEstimationLine_recover_allModels)
{
    for (const ModelCase& model : allModels)
    {
        for (const SensorCase& sensor : allSensors)
        {
            BOOST_TEST_CONTEXT("model=" << camera::EUNDISTORTION_enumToString(model.type) << " pa=" << sensor.pixelAspectRatio
                                         << " desqueezed=" << sensor.isDesqueezed)
            {
                const std::shared_ptr<camera::Undistortion> truth = makeModel(model.type, model.truth, sensor);
                std::vector<LineWithPoints> lines = distortedLines(*truth);
                BOOST_REQUIRE_GT(lines.size(), 10);

                std::shared_ptr<camera::Undistortion> undistortion = makeModel(model.type, model.initial, sensor);

                Statistics statistics;
                BOOST_REQUIRE(estimate(undistortion, statistics, lines, true, false, model.locks));

                const std::vector<double>& params = undistortion->getParameters();
                std::stringstream ss;
                for (std::size_t i = 0; i < params.size(); ++i)
                {
                    ss << " " << params[i] - model.truth[i];
                }
                BOOST_TEST_MESSAGE(camera::EUNDISTORTION_enumToString(model.type) << " pa=" << sensor.pixelAspectRatio << " desqueezed=" << sensor.isDesqueezed << " max residual=" << statistics.max
                                                                                  << " param errors:" << ss.str());
                BOOST_CHECK_SMALL(statistics.max, 1e-3);

                for (std::size_t i = 0; i < params.size(); ++i)
                {
                    if (model.unobservable[i])
                    {
                        continue;
                    }
                    BOOST_TEST_CONTEXT("param " << i)
                    {
                        BOOST_CHECK_SMALL(params[i] - model.truth[i], 1e-4);
                    }
                }
            }
        }
    }
}
