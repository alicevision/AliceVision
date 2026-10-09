// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/calibration/distortionEstimationGeometry.hpp>
#include <aliceVision/calibration/distortionEstimation_test_common.hpp>
#include <aliceVision/camera/UndistortionRadial.hpp>
#include <aliceVision/camera/camera.hpp>

#define BOOST_TEST_MODULE distortionEstimationGeometry
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

/// Focal length (physical pixels) so that the board covers most of the sensor.
double boardFocal(const RadialSensor& sensor)
{
    return 0.8 * std::min(sensor.width * sensor.squeeze(), sensor.height);
}

/**
 * @brief Point pairs of a 1x1 board (coordinates in [-0.5, 0.5]) seen from a given pose through a radial sensor.
 * The board is projected with an isotropic pinhole in physical pixels, then squeezed in x on the sensor.
 */
std::vector<PointPair> boardView(const RadialSensor& sensor, const Vec3& rotationAxisAngle, const Vec3& t)
{
    const Mat3 R = Eigen::AngleAxisd(rotationAxisAngle.norm(), rotationAxisAngle.normalized()).toRotationMatrix();
    const double focal = boardFocal(sensor);

    std::vector<PointPair> pairs;
    for (int j = 0; j <= 20; ++j)
    {
        for (int i = 0; i <= 20; ++i)
        {
            const Vec3 board(i / 20.0 - 0.5, j / 20.0 - 0.5, 0.0);
            const Vec3 c = R * board + t;
            const Vec2 physical = focal * c.head<2>() / c.z();
            const Vec2 undistorted(physical.x() / sensor.squeeze() + sensor.width / 2.0, physical.y() + sensor.height / 2.0);

            PointPair pp;
            pp.undistortedPoint = board.head<2>();
            pp.distortedPoint = sensor.distort(undistorted);
            pp.scale = 0.0;
            pairs.push_back(pp);
        }
    }
    return pairs;
}


/**
 * @brief Ground truth parameters for each undistortion model, initial values, locks and sharing as used by distortionCalibration.
 */
struct ModelCase
{
    camera::EUNDISTORTION type;
    std::vector<double> truth;
    std::vector<double> initial;
    std::vector<bool> locks;
    std::vector<bool> shared;
    // Allowed error per parameter (INFINITY for parameters the board poses can compensate)
    std::vector<double> tolerances;
};

const std::vector<ModelCase> allModels = {
  {camera::EUNDISTORTION::UNDISTORTION_RADIALK3, {-0.12, 0.025, -0.004}, {0.0, 0.0, 0.0}, {false, false, false}, {true, true, true}, {1e-4, 1e-4, 1e-4}},
  {camera::EUNDISTORTION::UNDISTORTION_3DECLASSICLD,
   {-0.08, 1.02, 0.01, -0.015, 0.01},
   {0.0, 1.0, 0.0, 0.0, 0.0},
   {false, false, false, false, false},
   {true, true, true, true, true},
   {1e-4, 1e-4, 1e-4, 1e-4, 1e-4}},
  {camera::EUNDISTORTION::UNDISTORTION_3DERADIAL4,
   {-0.08, 0.002, -0.003, 0.01, 0.001, -0.001, 0.3, 0.02},
   {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
   {false, false, false, false, false, false, false, false},
   {true, true, true, true, true, true, true, true},
   // The cylindric linear map is mostly compensated by the board poses: direction and bending are not checked
   {1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, INFINITY, INFINITY}},
  // Squeeze x is estimated per view and squeeze y is locked (global scale is given by the views)
  {camera::EUNDISTORTION::UNDISTORTION_3DEANAMORPHIC4,
   {-0.08, -0.07, 0.01, -0.01, 0.01, 0.012, 0.002, -0.002, 0.001, 0.001, 0.01, 1.01, 1.0},
   {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0},
   {false, false, false, false, false, false, false, false, false, false, false, false, true},
   {true, true, true, true, true, true, true, true, true, true, true, false, true},
   // A per view squeeze x is compensated by the view pose (more pose parameters than a planar homography): not checked
   {1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, INFINITY, 1e-4}},
};


/**
 * @brief Point pairs of a 1x1 board seen from a given pose, distorted by a ground truth model.
 * The board is projected with an isotropic pinhole in physical pixels, then squeezed in x on the sensor
 * (unless the image is already desqueezed).
 */
std::vector<PointPair> distortedView(const camera::Undistortion& truth, const Vec3& rotationAxisAngle, const Vec3& t)
{
    const Mat3 R = Eigen::AngleAxisd(rotationAxisAngle.norm(), rotationAxisAngle.normalized()).toRotationMatrix();
    // A desqueezed image has square pixels
    const double pa = truth.isDesqueezed() ? 1.0 : truth.getPixelAspectRatio();
    const Vec2 size = truth.getSize();
    const double focal = 0.8 * std::min(size.x() * pa, size.y());

    std::vector<PointPair> pairs;
    for (int j = 0; j <= 20; ++j)
    {
        for (int i = 0; i <= 20; ++i)
        {
            const Vec3 board(i / 20.0 - 0.5, j / 20.0 - 0.5, 0.0);
            const Vec3 c = R * board + t;
            const Vec2 physical = focal * c.head<2>() / c.z();
            const Vec2 sensor(physical.x() / pa + size.x() / 2.0, physical.y() + size.y() / 2.0);

            PointPair pp;
            pp.undistortedPoint = board.head<2>();
            pp.distortedPoint = truth.inverse(sensor);
            pp.scale = 0.0;
            pairs.push_back(pp);
        }
    }
    return pairs;
}

const std::vector<Vec3> rotations = {Vec3(0.05, -0.08, 0.02), Vec3(-0.1, 0.03, -0.05), Vec3(0.02, 0.12, 0.1)};
const std::vector<Vec3> translations = {Vec3(0.02, -0.01, 1.0), Vec3(-0.03, 0.02, 1.05), Vec3(0.01, 0.03, 0.95)};

void checkRecovery(const RadialSensor& sensor)
{
    DistortionEstimationGeometry estimator({true, true, true});

    std::vector<std::shared_ptr<camera::Undistortion>> undistortions;
    for (std::size_t i = 0; i < rotations.size(); ++i)
    {
        undistortions.push_back(sensor.makeUndistortion());
        estimator.addView(undistortions.back(), boardView(sensor, rotations[i], translations[i]));
    }

    Statistics statistics;
    BOOST_REQUIRE(estimator.compute(statistics, true, noLock));

    BOOST_TEST_MESSAGE("pa=" << sensor.pixelAspectRatio << " desqueezed=" << sensor.isDesqueezed << " max residual=" << statistics.max);
    BOOST_CHECK_SMALL(statistics.max, 1e-3);

    for (const auto& undistortion : undistortions)
    {
        const std::vector<double>& params = undistortion->getParameters();
        BOOST_CHECK_SMALL(params[0] - sensor.k1, 1e-4);
        BOOST_CHECK_SMALL(params[1] - sensor.k2, 1e-4);
        BOOST_CHECK_SMALL(params[2] - sensor.k3, 1e-4);
    }
}

}  // namespace

// Test summary:
// - Generate 3 views of a planar calibration board (different poses) on a 1600x1200 sensor, square pixels
//   - the ground truth is a radial k1/k2/k3 distortion written independently of camera::Undistortion
//     (radial only on purpose: see RadialSensor in distortionEstimation_test_common.hpp)
//   - the sensor squeezes the width by the pixel aspect ratio: physical (X, Y) lands at sensor pixel (X / pa, Y)
// - Estimate an UndistortionRadialK3 starting from zero parameters (distortion center locked)
// - Check that the residual is negligible and that k1, k2, k3 are recovered
BOOST_AUTO_TEST_CASE(distortionEstimationGeometry_recover_squarePixels)
{
    checkRecovery(makeSensor(1.0, false));
}

// Test summary:
// - Generate 3 views of a planar calibration board (different poses) on a 1600x1200 sensor, pixel aspect ratio 1.33
//   - the ground truth is a radial k1/k2/k3 distortion written independently of camera::Undistortion
//     (radial only on purpose: see RadialSensor in distortionEstimation_test_common.hpp)
//   - the sensor squeezes the width by the pixel aspect ratio: physical (X, Y) lands at sensor pixel (X / pa, Y)
// - Estimate an UndistortionRadialK3 starting from zero parameters (distortion center locked)
// - Check that the residual is negligible and that k1, k2, k3 are recovered
BOOST_AUTO_TEST_CASE(distortionEstimationGeometry_recover_anamorphic133)
{
    checkRecovery(makeSensor(1.33, false));
}

// Test summary:
// - Generate 3 views of a planar calibration board (different poses) on a 1600x1200 sensor, pixel aspect ratio 2
//   - the ground truth is a radial k1/k2/k3 distortion written independently of camera::Undistortion
//     (radial only on purpose: see RadialSensor in distortionEstimation_test_common.hpp)
//   - the sensor squeezes the width by the pixel aspect ratio: physical (X, Y) lands at sensor pixel (X / pa, Y)
// - Estimate an UndistortionRadialK3 starting from zero parameters (distortion center locked)
// - Check that the residual is negligible and that k1, k2, k3 are recovered
BOOST_AUTO_TEST_CASE(distortionEstimationGeometry_recover_anamorphic2)
{
    checkRecovery(makeSensor(2.0, false));
}

// Test summary:
// - Generate 3 views of a planar calibration board (different poses) on a 1600x1200 sensor, pixel aspect ratio 2 on an image already desqueezed (square pixels)
//   - the ground truth is a radial k1/k2/k3 distortion written independently of camera::Undistortion
//     (radial only on purpose: see RadialSensor in distortionEstimation_test_common.hpp)
//   - the sensor squeezes the width by the pixel aspect ratio: physical (X, Y) lands at sensor pixel (X / pa, Y)
// - Estimate an UndistortionRadialK3 flagged as desqueezed starting from zero parameters (distortion center locked)
// - Check that the residual is negligible and that k1, k2, k3 are recovered
BOOST_AUTO_TEST_CASE(distortionEstimationGeometry_recover_anamorphic2_desqueezed)
{
    checkRecovery(makeSensor(2.0, true));
}

// Test summary:
// - Generate 3 board views on a sensor with a pixel aspect ratio of 2
// - Estimate an UndistortionRadialK3 assuming square pixels
// - Check that the residual stays large (> 1 pixel): the pixel aspect ratio is required to explain the data
BOOST_AUTO_TEST_CASE(distortionEstimationGeometry_wrongPixelAspectRatio_cannotFit)
{
    // A 2x anamorphic sensor cannot be explained by a model on square pixels
    const RadialSensor sensor = makeSensor(2.0, false);
    DistortionEstimationGeometry estimator({true, true, true});
    for (std::size_t i = 0; i < rotations.size(); ++i)
    {
        estimator.addView(std::make_shared<camera::UndistortionRadialK3>(1600, 1200), boardView(sensor, rotations[i], translations[i]));
    }

    Statistics statistics;
    BOOST_REQUIRE(estimator.compute(statistics, true, noLock));

    BOOST_TEST_MESSAGE("max residual with wrong pa=" << statistics.max);
    BOOST_CHECK_GT(statistics.max, 1.0);
}

// Test summary:
// - Generate 3 board views with a pixel aspect ratio of 2, each view with a different k1 (-0.10, -0.12, -0.14)
//   to simulate lens breathing, k2 and k3 being the same for all views
// - Estimate with k1 per view and k2, k3 shared
// - Check that the residual is negligible, that each view recovers its own k1 and that k2, k3 are recovered
BOOST_AUTO_TEST_CASE(distortionEstimationGeometry_perViewParameter_breathing)
{
    // k1 changes per view (e.g. focus breathing), k2 and k3 are shared
    const std::vector<double> k1PerView = {-0.10, -0.12, -0.14};

    DistortionEstimationGeometry estimator({false, true, true});

    std::vector<std::shared_ptr<camera::Undistortion>> undistortions;
    for (std::size_t i = 0; i < k1PerView.size(); ++i)
    {
        RadialSensor sensor = makeSensor(2.0, false);
        sensor.k1 = k1PerView[i];
        undistortions.push_back(sensor.makeUndistortion());
        estimator.addView(undistortions.back(), boardView(sensor, rotations[i], translations[i]));
    }

    Statistics statistics;
    BOOST_REQUIRE(estimator.compute(statistics, true, noLock));

    BOOST_TEST_MESSAGE("breathing max residual=" << statistics.max);
    BOOST_CHECK_SMALL(statistics.max, 1e-3);
    for (std::size_t i = 0; i < k1PerView.size(); ++i)
    {
        const std::vector<double>& params = undistortions[i]->getParameters();
        BOOST_CHECK_SMALL(params[0] - k1PerView[i], 1e-4);
        BOOST_CHECK_SMALL(params[1] - 0.025, 1e-4);
        BOOST_CHECK_SMALL(params[2] - (-0.004), 1e-4);
    }
}

// Test summary:
// - Check that computation fails with no view,
//   and with a lock vector whose size does not match the number of parameters
BOOST_AUTO_TEST_CASE(distortionEstimationGeometry_invalidInput_fails)
{
    Statistics statistics;

    DistortionEstimationGeometry empty({true, true, true});
    BOOST_CHECK(!empty.compute(statistics, true, noLock));

    const RadialSensor sensor = makeSensor(1.0, false);
    DistortionEstimationGeometry estimator({true, true, true});
    estimator.addView(sensor.makeUndistortion(), boardView(sensor, rotations[0], translations[0]));
    BOOST_CHECK(!estimator.compute(statistics, true, {false, false}));
}

// Test summary:
// - Generate 3 board views distorted with the inverse of a ground truth undistortion model
//   - for each undistortion model: radialk3, 3declassicld, 3deradial4, 3deanamorphic4
//   - for each sensor: pixel aspect ratio 1, 1.33, 2, and 1.33, 2 on already desqueezed images
// - Estimate the same model starting from the initial values used by distortionCalibration, all parameters shared
//   (3deanamorphic4 as in distortionCalibration: squeeze x per view, squeeze y locked)
// - Check that the residual is negligible and that the parameters are recovered,
//   except the parameters the board poses can compensate (3deradial4 cylindric direction and bending, 3deanamorphic4 squeeze x)
BOOST_AUTO_TEST_CASE(distortionEstimationGeometry_recover_allModels)
{
    for (const ModelCase& model : allModels)
    {
        for (const SensorCase& sensor : allSensors)
        {
            BOOST_TEST_CONTEXT("model=" << camera::EUNDISTORTION_enumToString(model.type) << " pa=" << sensor.pixelAspectRatio
                                         << " desqueezed=" << sensor.isDesqueezed)
            {
                const std::shared_ptr<camera::Undistortion> truth = makeModel(model.type, model.truth, sensor);

                DistortionEstimationGeometry estimator(model.shared);
                std::vector<std::shared_ptr<camera::Undistortion>> undistortions;
                for (std::size_t i = 0; i < rotations.size(); ++i)
                {
                    undistortions.push_back(makeModel(model.type, model.initial, sensor));
                    estimator.addView(undistortions.back(), distortedView(*truth, rotations[i], translations[i]));
                }

                Statistics statistics;
                BOOST_REQUIRE(estimator.compute(statistics, true, model.locks));

                std::stringstream ss;
                for (std::size_t i = 0; i < model.truth.size(); ++i)
                {
                    ss << " " << undistortions[0]->getParameters()[i] - model.truth[i];
                }
                BOOST_TEST_MESSAGE(camera::EUNDISTORTION_enumToString(model.type) << " pa=" << sensor.pixelAspectRatio << " desqueezed=" << sensor.isDesqueezed << " max residual=" << statistics.max
                                                                                  << " param errors:" << ss.str());
                BOOST_CHECK_SMALL(statistics.max, 1e-3);

                for (const auto& undistortion : undistortions)
                {
                    const std::vector<double>& params = undistortion->getParameters();
                    for (std::size_t i = 0; i < params.size(); ++i)
                    {
                        BOOST_TEST_CONTEXT("param " << i)
                        {
                            BOOST_CHECK_LE(std::abs(params[i] - model.truth[i]), model.tolerances[i]);
                        }
                    }
                }
            }
        }
    }
}

