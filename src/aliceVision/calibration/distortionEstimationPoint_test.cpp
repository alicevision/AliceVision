// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/calibration/distortionEstimationPoint.hpp>
#include <aliceVision/calibration/distortionEstimation_test_common.hpp>
#include <aliceVision/camera/UndistortionRadial.hpp>
#include <aliceVision/camera/camera.hpp>

#define BOOST_TEST_MODULE distortionEstimationPoint
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

/// Distortion center offset of the radial sensors.
const Vec2 centerOffset(12.0, -8.0);

/// Point pairs on a 40 pixels grid of distorted points.
std::vector<PointPair> pointPairs(const RadialSensor& sensor)
{
    std::vector<PointPair> pairs;
    for (double y = 20.0; y < sensor.height; y += 40.0)
    {
        for (double x = 20.0; x < sensor.width; x += 40.0)
        {
            PointPair pp;
            pp.distortedPoint = Vec2(x, y);
            pp.undistortedPoint = sensor.undistort(pp.distortedPoint);
            pp.scale = 0.0;
            pairs.push_back(pp);
        }
    }
    return pairs;
}


/// Ground truth parameters for each undistortion model, and the initial values used by distortionCalibration.
struct ModelCase
{
    camera::EUNDISTORTION type;
    std::vector<double> truth;
    std::vector<double> initial;
};

const std::vector<ModelCase> allModels = {
  {camera::EUNDISTORTION::UNDISTORTION_RADIALK3, {-0.12, 0.025, -0.004}, {0.0, 0.0, 0.0}},
  {camera::EUNDISTORTION::UNDISTORTION_3DECLASSICLD, {-0.08, 1.02, 0.01, -0.015, 0.01}, {0.0, 1.0, 0.0, 0.0, 0.0}},
  {camera::EUNDISTORTION::UNDISTORTION_3DERADIAL4,
   {-0.08, 0.002, -0.003, 0.01, 0.001, -0.001, 0.3, 0.02},
   {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
  {camera::EUNDISTORTION::UNDISTORTION_3DEANAMORPHIC4,
   {-0.08, -0.07, 0.01, -0.01, 0.01, 0.012, 0.002, -0.002, 0.001, 0.001, 0.01, 1.01, 0.99},
   {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0}},
};


void checkRecovery(const RadialSensor& sensor)
{
    const std::vector<PointPair> pairs = pointPairs(sensor);
    std::shared_ptr<camera::Undistortion> undistortion = sensor.makeUndistortion();

    Statistics statistics;
    BOOST_REQUIRE(estimate(undistortion, statistics, pairs, false, noLock));

    BOOST_TEST_MESSAGE("pa=" << sensor.pixelAspectRatio << " desqueezed=" << sensor.isDesqueezed << " max residual=" << statistics.max);
    BOOST_CHECK_SMALL(statistics.max, 1e-3);

    const std::vector<double>& params = undistortion->getParameters();
    BOOST_CHECK_SMALL(params[0] - sensor.k1, 1e-5);
    BOOST_CHECK_SMALL(params[1] - sensor.k2, 1e-5);
    BOOST_CHECK_SMALL(params[2] - sensor.k3, 1e-5);
    BOOST_CHECK_SMALL(undistortion->getOffset().x() - sensor.offset.x(), 1e-3);
    BOOST_CHECK_SMALL(undistortion->getOffset().y() - sensor.offset.y(), 1e-3);
}

}  // namespace

// Test summary:
// - Generate a grid of point pairs (distorted / undistorted) on a 1600x1200 sensor, square pixels
//   - the ground truth is a radial k1/k2/k3 distortion written independently of camera::Undistortion
//     (radial only on purpose: see RadialSensor in distortionEstimation_test_common.hpp)
//   - the sensor squeezes the width by the pixel aspect ratio: physical (X, Y) lands at sensor pixel (X / pa, Y)
// - Estimate an UndistortionRadialK3 starting from zero parameters
// - Check that the residual is negligible and that k1, k2, k3 and the distortion center offset are recovered
BOOST_AUTO_TEST_CASE(distortionEstimationPoint_recover_squarePixels)
{
    checkRecovery(makeSensor(1.0, false, centerOffset));
}

// Test summary:
// - Generate a grid of point pairs (distorted / undistorted) on a 1600x1200 sensor, pixel aspect ratio 1.33
//   - the ground truth is a radial k1/k2/k3 distortion written independently of camera::Undistortion
//     (radial only on purpose: see RadialSensor in distortionEstimation_test_common.hpp)
//   - the sensor squeezes the width by the pixel aspect ratio: physical (X, Y) lands at sensor pixel (X / pa, Y)
// - Estimate an UndistortionRadialK3 starting from zero parameters
// - Check that the residual is negligible and that k1, k2, k3 and the distortion center offset are recovered
BOOST_AUTO_TEST_CASE(distortionEstimationPoint_recover_anamorphic133)
{
    checkRecovery(makeSensor(1.33, false, centerOffset));
}

// Test summary:
// - Generate a grid of point pairs (distorted / undistorted) on a 1600x1200 sensor, pixel aspect ratio 2
//   - the ground truth is a radial k1/k2/k3 distortion written independently of camera::Undistortion
//     (radial only on purpose: see RadialSensor in distortionEstimation_test_common.hpp)
//   - the sensor squeezes the width by the pixel aspect ratio: physical (X, Y) lands at sensor pixel (X / pa, Y)
// - Estimate an UndistortionRadialK3 starting from zero parameters
// - Check that the residual is negligible and that k1, k2, k3 and the distortion center offset are recovered
BOOST_AUTO_TEST_CASE(distortionEstimationPoint_recover_anamorphic2)
{
    checkRecovery(makeSensor(2.0, false, centerOffset));
}

// Test summary:
// - Generate a grid of point pairs (distorted / undistorted) on a 1600x1200 sensor, pixel aspect ratio 2 on an image already desqueezed (square pixels)
//   - the ground truth is a radial k1/k2/k3 distortion written independently of camera::Undistortion
//     (radial only on purpose: see RadialSensor in distortionEstimation_test_common.hpp)
//   - the sensor squeezes the width by the pixel aspect ratio: physical (X, Y) lands at sensor pixel (X / pa, Y)
// - Estimate an UndistortionRadialK3 flagged as desqueezed starting from zero parameters
// - Check that the residual is negligible and that k1, k2, k3 and the distortion center offset are recovered
BOOST_AUTO_TEST_CASE(distortionEstimationPoint_recover_anamorphic2_desqueezed)
{
    checkRecovery(makeSensor(2.0, true, centerOffset));
}

// Test summary:
// - Generate point pairs on a sensor with a pixel aspect ratio of 2
// - Estimate an UndistortionRadialK3 assuming square pixels
// - Check that the residual stays large (> 1 pixel): the pixel aspect ratio is required to explain the data
BOOST_AUTO_TEST_CASE(distortionEstimationPoint_wrongPixelAspectRatio_cannotFit)
{
    // Distortion of a 2x anamorphic sensor cannot be explained by a radial model on square pixels
    const RadialSensor sensor = makeSensor(2.0, false, centerOffset);
    const std::vector<PointPair> pairs = pointPairs(sensor);

    auto undistortion = std::make_shared<camera::UndistortionRadialK3>(1600, 1200);

    Statistics statistics;
    BOOST_REQUIRE(estimate(undistortion, statistics, pairs, false, noLock));

    BOOST_TEST_MESSAGE("max residual with wrong pa=" << statistics.max);
    BOOST_CHECK_GT(statistics.max, 1.0);
}

// Test summary:
// - Generate point pairs on a sensor with a pixel aspect ratio of 2
// - Estimate with k2 locked to a wrong value (0.05) and the distortion center locked
// - Check that k2 and the center are exactly unchanged while k1 is estimated
BOOST_AUTO_TEST_CASE(distortionEstimationPoint_lockedParameters_unchanged)
{
    const RadialSensor sensor = makeSensor(2.0, false, centerOffset);
    const std::vector<PointPair> pairs = pointPairs(sensor);

    std::shared_ptr<camera::Undistortion> undistortion = sensor.makeUndistortion();
    undistortion->setParameters({0.0, 0.05, 0.0});

    Statistics statistics;
    BOOST_REQUIRE(estimate(undistortion, statistics, pairs, true, {false, true, false}));

    BOOST_CHECK_EQUAL(undistortion->getParameters()[1], 0.05);
    BOOST_CHECK_NE(undistortion->getParameters()[0], 0.0);
    BOOST_CHECK_EQUAL(undistortion->getOffset().x(), 0.0);
    BOOST_CHECK_EQUAL(undistortion->getOffset().y(), 0.0);
}

// Test summary:
// - Check that estimation fails on: a null undistortion, an empty set of point pairs,
//   a lock vector whose size does not match the number of parameters
BOOST_AUTO_TEST_CASE(distortionEstimationPoint_invalidInput_fails)
{
    const RadialSensor sensor = makeSensor(1.0, false, centerOffset);
    std::shared_ptr<camera::Undistortion> undistortion = sensor.makeUndistortion();
    Statistics statistics;

    BOOST_CHECK(!estimate(nullptr, statistics, pointPairs(sensor), false, noLock));
    BOOST_CHECK(!estimate(undistortion, statistics, {}, false, noLock));
    BOOST_CHECK(!estimate(undistortion, statistics, pointPairs(sensor), false, {false, false}));
}

// Test summary:
// - Generate a grid of point pairs with a ground truth undistortion model (with a center offset)
//   - for each undistortion model: radialk3, 3declassicld, 3deradial4, 3deanamorphic4
//   - for each sensor: pixel aspect ratio 1, 1.33, 2, and 1.33, 2 on already desqueezed images
// - Estimate the same model starting from the initial values used by distortionCalibration, nothing locked
// - Check that the residual is negligible and that all parameters and the center offset are recovered
BOOST_AUTO_TEST_CASE(distortionEstimationPoint_recover_allModels)
{
    for (const ModelCase& model : allModels)
    {
        for (const SensorCase& sensor : allSensors)
        {
            BOOST_TEST_CONTEXT("model=" << camera::EUNDISTORTION_enumToString(model.type) << " pa=" << sensor.pixelAspectRatio
                                         << " desqueezed=" << sensor.isDesqueezed)
            {
                std::shared_ptr<camera::Undistortion> truth = makeModel(model.type, model.truth, sensor);
                truth->setOffset(Vec2(12.0, -8.0));

                std::vector<PointPair> pairs;
                for (double y = 20.0; y < 1200.0; y += 40.0)
                {
                    for (double x = 20.0; x < 1600.0; x += 40.0)
                    {
                        PointPair pp;
                        pp.distortedPoint = Vec2(x, y);
                        pp.undistortedPoint = truth->undistort(pp.distortedPoint);
                        pp.scale = 0.0;
                        pairs.push_back(pp);
                    }
                }

                std::shared_ptr<camera::Undistortion> undistortion = makeModel(model.type, model.initial, sensor);

                Statistics statistics;
                BOOST_REQUIRE(estimate(undistortion, statistics, pairs, false, std::vector<bool>(model.truth.size(), false)));

                BOOST_TEST_MESSAGE(camera::EUNDISTORTION_enumToString(model.type) << " pa=" << sensor.pixelAspectRatio << " desqueezed=" << sensor.isDesqueezed << " max residual=" << statistics.max);
                BOOST_CHECK_SMALL(statistics.max, 1e-3);

                const std::vector<double>& params = undistortion->getParameters();
                for (std::size_t i = 0; i < params.size(); ++i)
                {
                    BOOST_TEST_CONTEXT("param " << i)
                    {
                        BOOST_CHECK_SMALL(params[i] - model.truth[i], 1e-4);
                    }
                }
                BOOST_CHECK_SMALL(undistortion->getOffset().x() - 12.0, 1e-2);
                BOOST_CHECK_SMALL(undistortion->getOffset().y() + 8.0, 1e-2);
            }
        }
    }
}
