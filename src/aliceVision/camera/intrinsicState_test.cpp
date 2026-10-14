// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/camera/camera.hpp>
#include <aliceVision/camera/camera_test_common.hpp>
#include <aliceVision/geometry/Pose3.hpp>
#include <aliceVision/version.hpp>

#define BOOST_TEST_MODULE intrinsicState

#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>
#include <aliceVision/unitTest.hpp>

#include <functional>

using namespace aliceVision;
using namespace aliceVision::camera;
using namespace aliceVision::camera::test;

namespace {

std::shared_ptr<IntrinsicScaleOffsetDisto> cloneIntrinsic(const IntrinsicScaleOffsetDisto& intrinsic)
{ return std::shared_ptr<IntrinsicScaleOffsetDisto>(dynamic_cast<IntrinsicScaleOffsetDisto*>(intrinsic.clone())); }

/// Change every distortion and undistortion parameter of the camera.
void perturbDistortion(IntrinsicScaleOffsetDisto& intrinsic)
{
    if (intrinsic.getDistortion())
    {
        std::vector<double> params = intrinsic.getDistortionParams();
        for (double& p : params)
        {
            p += 0.01;
        }
        intrinsic.setDistortionParams(params);
    }
    if (intrinsic.getUndistortion())
    {
        std::vector<double> params = intrinsic.getUndistortion()->getParameters();
        for (double& p : params)
        {
            p += 0.01;
        }
        intrinsic.getUndistortion()->setParameters(params);
    }
}

}  // namespace

//-----------------
// Copy
//-----------------

BOOST_AUTO_TEST_CASE(intrinsicState_clone)
{
    for (const TestCamera& tc : makeTestCameras())
    {
        BOOST_TEST_CONTEXT(tc.name)
        {
            std::shared_ptr<IntrinsicScaleOffsetDisto> copy = cloneIntrinsic(*tc.intrinsic);

            BOOST_REQUIRE(copy);
            BOOST_CHECK(copy.get() != tc.intrinsic.get());
            BOOST_CHECK(*copy == *tc.intrinsic);
            BOOST_CHECK_EQUAL(copy->getType(), tc.intrinsic->getType());
            BOOST_CHECK_EQUAL(copy->hashValue(), tc.intrinsic->hashValue());

            // The copy owns its (un)distortion: changing it leaves the original untouched.
            if (copy->hasDistortion())
            {
                BOOST_CHECK(copy->getDistortion() != tc.intrinsic->getDistortion() || !copy->getDistortion());
                BOOST_CHECK(copy->getUndistortion() != tc.intrinsic->getUndistortion() || !copy->getUndistortion());
                perturbDistortion(*copy);
                BOOST_CHECK(*copy != *tc.intrinsic);
            }
        }
    }
}

namespace {

// After a.assign(b), changing b's distortion must not change a.
// Returns true if a kept its own copy of the (un)distortion.
bool assignIsDeep(const TestCamera& tc)
{
    std::shared_ptr<IntrinsicScaleOffsetDisto> source = cloneIntrinsic(*tc.intrinsic);
    std::shared_ptr<IntrinsicScaleOffsetDisto> target =
      makeTestCamera(tc.intrinsicType, EDISTORTION::DISTORTION_NONE, EUNDISTORTION::UNDISTORTION_NONE).intrinsic;

    target->assign(*source);
    BOOST_CHECK_MESSAGE(*target == *source, tc.name << ": assign does not copy");

    perturbDistortion(*source);
    return *target == *tc.intrinsic;
}

}  // namespace

BOOST_AUTO_TEST_CASE(intrinsicState_assign)
{
    for (const TestCamera& tc : makeTestCameras())
    {
        BOOST_CHECK_MESSAGE(assignIsDeep(tc), tc.name << ": assigned intrinsic shares its distortion with the source");
    }
}

//-----------------
// Comparison
//-----------------

BOOST_AUTO_TEST_CASE(intrinsicState_equality)
{
    using Mutation = std::function<void(IntrinsicScaleOffsetDisto&)>;
    const std::vector<std::pair<std::string, Mutation>> mutations = {
      {"width", [](IntrinsicScaleOffsetDisto& i) { i.setWidth(i.w() + 2); }},
      {"height", [](IntrinsicScaleOffsetDisto& i) { i.setHeight(i.h() + 2); }},
      {"sensor width", [](IntrinsicScaleOffsetDisto& i) { i.setSensorWidth(i.sensorWidth() + 1.0); }},
      {"sensor height", [](IntrinsicScaleOffsetDisto& i) { i.setSensorHeight(i.sensorHeight() + 1.0); }},
      {"serial number", [](IntrinsicScaleOffsetDisto& i) { i.setSerialNumber("other"); }},
      {"initialization mode", [](IntrinsicScaleOffsetDisto& i) { i.setInitializationMode(EInitMode::CALIBRATED); }},
      {"scale", [](IntrinsicScaleOffsetDisto& i) { i.setScale(i.getScale() * 1.1); }},
      {"offset", [](IntrinsicScaleOffsetDisto& i) { i.setOffset(i.getOffset() + Vec2(3.0, 0.0)); }},
      {"initial scale", [](IntrinsicScaleOffsetDisto& i) { i.setInitialScale(Vec2(100.0, 100.0)); }},
      {"ratio locked", [](IntrinsicScaleOffsetDisto& i) { i.setRatioLocked(!i.isRatioLocked()); }},
      {"offset locked", [](IntrinsicScaleOffsetDisto& i) { i.setOffsetLocked(!i.isOffsetLocked()); }},
      {"scale locked", [](IntrinsicScaleOffsetDisto& i) { i.setScaleLocked(!i.isScaleLocked()); }},
      {"distortion initialization mode", [](IntrinsicScaleOffsetDisto& i) { i.setDistortionInitializationMode(EInitMode::ESTIMATED); }},
    };

    for (const TestCamera& tc : makeTestCameras())
    {
        BOOST_TEST_CONTEXT(tc.name)
        {
            const IntrinsicScaleOffsetDisto& reference = *tc.intrinsic;
            BOOST_CHECK(reference == reference);
            BOOST_CHECK(!(reference != reference));

            for (const auto& mutation : mutations)
            {
                std::shared_ptr<IntrinsicScaleOffsetDisto> other = cloneIntrinsic(reference);
                mutation.second(*other);
                BOOST_CHECK_MESSAGE(*other != reference, "change of " << mutation.first << " not detected");
                BOOST_CHECK_MESSAGE(reference != *other, "change of " << mutation.first << " not detected (reversed)");
            }

            if (reference.hasDistortion())
            {
                // Distortion parameters
                std::shared_ptr<IntrinsicScaleOffsetDisto> other = cloneIntrinsic(reference);
                perturbDistortion(*other);
                BOOST_CHECK(*other != reference);
                BOOST_CHECK(other->equalTo(reference, true));
                BOOST_CHECK(!other->equalTo(reference, false));

                // Presence of the distortion
                std::shared_ptr<IntrinsicScaleOffsetDisto> without = cloneIntrinsic(reference);
                without->setDistortionObject(nullptr);
                without->setUndistortionObject(nullptr);
                BOOST_CHECK(*without != reference);
                BOOST_CHECK(reference != *without);
                BOOST_CHECK(without->equalTo(reference, true));
            }
        }
    }
}

BOOST_AUTO_TEST_CASE(intrinsicState_equalityAcrossTypes)
{
    // Same size, focal and offset, different models.
    const Pinhole pinhole(1000, 1000, 500.0, 500.0, 0.0, 0.0);
    const Equidistant equidistant(1000, 1000, 500.0, 0.0, 0.0);
    const Equirectangular equirectangular(1000, 1000, 500.0, 500.0, 0.0, 0.0);

    // Compared as IntrinsicBase: in C++20, the reversed operator== of Equidistant makes mixed-type comparisons ambiguous.
    const IntrinsicBase& pinholeBase = pinhole;
    const IntrinsicBase& equidistantBase = equidistant;
    const IntrinsicBase& equirectangularBase = equirectangular;
    BOOST_CHECK(pinholeBase != equidistantBase);
    BOOST_CHECK(equidistantBase != pinholeBase);
    BOOST_CHECK(pinholeBase != equirectangularBase);
    BOOST_CHECK(equidistantBase != equirectangularBase);

    // Same model, different distortion types.
    const Pinhole k1(1000, 1000, 500.0, 500.0, 0.0, 0.0, std::make_shared<DistortionRadialK1>(0.1));
    const Pinhole fisheye1(1000, 1000, 500.0, 500.0, 0.0, 0.0, std::make_shared<DistortionFisheye1>(0.1));
    BOOST_CHECK(k1 != pinhole);
    BOOST_CHECK(pinhole != k1);
    // Distortion::operator== only compares the parameter values, not the model.
    BOOST_CHECK(k1 == fisheye1);
}

BOOST_AUTO_TEST_CASE(intrinsicState_equalityEquidistantCircle)
{
    const Equidistant reference(1920, 1080, 450.0, 0.0, 0.0);

    Equidistant otherRadius(reference);
    otherRadius.setCircleRadius(reference.getCircleRadius() + 10.0);
    BOOST_CHECK(otherRadius != reference);

    Equidistant otherCenter(reference);
    otherCenter.setCircleCenterX(reference.getCircleCenterX() + 10.0);
    BOOST_CHECK(otherCenter != reference);

    BOOST_CHECK(Equidistant(reference) == reference);
}

BOOST_AUTO_TEST_CASE(intrinsicState_hash)
{
    for (const TestCamera& tc : makeTestCameras())
    {
        BOOST_TEST_CONTEXT(tc.name)
        {
            const IntrinsicScaleOffsetDisto& reference = *tc.intrinsic;
            BOOST_CHECK_EQUAL(cloneIntrinsic(reference)->hashValue(), reference.hashValue());

            std::shared_ptr<IntrinsicScaleOffsetDisto> other = cloneIntrinsic(reference);
            other->setScale(reference.getScale() * 1.1);
            BOOST_CHECK_NE(other->hashValue(), reference.hashValue());

            other = cloneIntrinsic(reference);
            other->setOffset(reference.getOffset() + Vec2(1.0, 0.0));
            BOOST_CHECK_NE(other->hashValue(), reference.hashValue());

            other = cloneIntrinsic(reference);
            other->setWidth(reference.w() + 2);
            BOOST_CHECK_NE(other->hashValue(), reference.hashValue());

            other = cloneIntrinsic(reference);
            other->setSerialNumber("abc");
            BOOST_CHECK_NE(other->hashValue(), reference.hashValue());
        }
    }
}

//-----------------
// Parameters
//-----------------

BOOST_AUTO_TEST_CASE(intrinsicState_parameters)
{
    for (const TestCamera& tc : makeTestCameras())
    {
        BOOST_TEST_CONTEXT(tc.name)
        {
            std::shared_ptr<IntrinsicScaleOffsetDisto> intrinsic = cloneIntrinsic(*tc.intrinsic);

            // scale x, scale y, offset x, offset y
            const std::vector<double> params = intrinsic->getParameters();
            BOOST_REQUIRE_EQUAL(params.size(), 4);
            BOOST_CHECK_EQUAL(intrinsic->getParametersSize(), 4);
            BOOST_CHECK_EQUAL(params[0], intrinsic->getScale()(0));
            BOOST_CHECK_EQUAL(params[1], intrinsic->getScale()(1));
            BOOST_CHECK_EQUAL(params[2], intrinsic->getOffset()(0));
            BOOST_CHECK_EQUAL(params[3], intrinsic->getOffset()(1));

            const std::vector<double> updated = {params[0] + 10.0, params[1] + 20.0, params[2] + 1.0, params[3] - 2.0};
            BOOST_CHECK(intrinsic->updateFromParams(updated));
            BOOST_CHECK(intrinsic->getParameters() == updated);

            // Too few parameters: rejected, nothing changes.
            BOOST_CHECK(!intrinsic->updateFromParams({1.0, 2.0, 3.0}));
            BOOST_CHECK(intrinsic->getParameters() == updated);

            // Intrinsic parameters followed by the distortion parameters update both.
            if (intrinsic->getDistortionParamsSize() > 0)
            {
                std::vector<double> distortion = intrinsic->getDistortionParams();
                for (double& d : distortion)
                {
                    d *= 0.5;
                }
                std::vector<double> all = params;
                all.insert(all.end(), distortion.begin(), distortion.end());

                BOOST_CHECK(intrinsic->updateFromParams(all));
                BOOST_CHECK(intrinsic->getParameters() == params);
                BOOST_CHECK(intrinsic->getDistortionParams() == distortion);
            }
        }
    }
}

BOOST_AUTO_TEST_CASE(intrinsicState_distortionParams)
{
    for (const TestCamera& tc : makeTestCameras())
    {
        BOOST_TEST_CONTEXT(tc.name)
        {
            std::shared_ptr<IntrinsicScaleOffsetDisto> intrinsic = cloneIntrinsic(*tc.intrinsic);
            const std::size_t count = intrinsic->getDistortionParamsSize();

            BOOST_CHECK_EQUAL(count, distortionTestParameters(getDistortionType(*intrinsic)).size());
            BOOST_CHECK_EQUAL(intrinsic->getDistortionParams().size(), count);

            // Wrong count
            BOOST_CHECK_THROW(intrinsic->setDistortionParams(std::vector<double>(count + 1, 0.0)), std::runtime_error);
            BOOST_CHECK_THROW(intrinsic->setDistortionParamsFn(count + 1, [](std::size_t) { return 0.0; }), std::runtime_error);

            // Right count
            BOOST_CHECK_NO_THROW(intrinsic->setDistortionParamsFn(count, [](std::size_t i) { return 0.001 * (i + 1); }));
            for (std::size_t i = 0; i < count; ++i)
            {
                BOOST_CHECK_CLOSE(intrinsic->getDistortionParams()[i], 0.001 * (i + 1), 1e-9);
            }

            const std::vector<double> values(count, 0.002);
            BOOST_CHECK_NO_THROW(intrinsic->setDistortionParams(values));
            BOOST_CHECK(intrinsic->getDistortionParams() == values);
        }
    }
}

BOOST_AUTO_TEST_CASE(intrinsicState_importFromParams)
{
    const unsigned int w = 1000;
    const unsigned int h = 800;

    // Before 1.2.0: a single focal length, and the principal point in absolute pixel coordinates.
    {
        Pinhole intrinsic(w, h, 1.0, 1.0, 0.0, 0.0, std::make_shared<DistortionRadialK1>());
        BOOST_CHECK(intrinsic.importFromParams({900.0, 510.0, 395.0, 0.05}, Version(1, 1, 0)));
        EXPECT_MATRIX_NEAR(intrinsic.getScale(), Vec2(900.0, 900.0), 1e-12);
        EXPECT_MATRIX_NEAR(intrinsic.getOffset(), Vec2(10.0, -5.0), 1e-12);
        EXPECT_MATRIX_NEAR(intrinsic.getPrincipalPoint(), Vec2(510.0, 395.0), 1e-12);
        BOOST_CHECK_EQUAL(intrinsic.getDistortionParams()[0], 0.05);
    }

    // 1.2.0: focal x and y, principal point still absolute.
    {
        Pinhole intrinsic(w, h, 1.0, 1.0, 0.0, 0.0);
        BOOST_CHECK(intrinsic.importFromParams({900.0, 950.0, 510.0, 395.0}, Version(1, 2, 0)));
        EXPECT_MATRIX_NEAR(intrinsic.getScale(), Vec2(900.0, 950.0), 1e-12);
        EXPECT_MATRIX_NEAR(intrinsic.getOffset(), Vec2(10.0, -5.0), 1e-12);
    }

    // Since 1.2.1: offset relative to the image center.
    {
        Pinhole intrinsic(w, h, 1.0, 1.0, 0.0, 0.0);
        BOOST_CHECK(intrinsic.importFromParams({900.0, 950.0, 10.0, -5.0}, Version(1, 2, 1)));
        EXPECT_MATRIX_NEAR(intrinsic.getScale(), Vec2(900.0, 950.0), 1e-12);
        EXPECT_MATRIX_NEAR(intrinsic.getOffset(), Vec2(10.0, -5.0), 1e-12);

        BOOST_CHECK(!intrinsic.importFromParams({900.0, 950.0}, Version(1, 2, 1)));
    }
}

//-----------------
// Lock and estimator state
//-----------------

BOOST_AUTO_TEST_CASE(intrinsicState_lock)
{
    Pinhole intrinsic(1000, 800, 900.0, 900.0, 0.0, 0.0);

    BOOST_CHECK(!intrinsic.isLocked());
    intrinsic.initializeState();
    BOOST_CHECK(intrinsic.getState() == EEstimatorParameterState::REFINED);

    intrinsic.lock();
    BOOST_CHECK(intrinsic.isLocked());
    intrinsic.initializeState();
    BOOST_CHECK(intrinsic.getState() == EEstimatorParameterState::CONSTANT);

    intrinsic.unlock();
    BOOST_CHECK(!intrinsic.isLocked());

    intrinsic.setState(EEstimatorParameterState::IGNORED);
    BOOST_CHECK(intrinsic.getState() == EEstimatorParameterState::IGNORED);
}

//-----------------
// Focal length in millimeters
//-----------------

BOOST_AUTO_TEST_CASE(intrinsicState_focalLength)
{
    Pinhole intrinsic(6000, 4000, 1.0, 1.0, 0.0, 0.0);
    intrinsic.setSensorWidth(36.0);
    intrinsic.setSensorHeight(24.0);

    intrinsic.setFocalLength(50.0, 1.0);
    BOOST_CHECK_CLOSE(intrinsic.getScale()(0), 50.0 * 6000.0 / 36.0, 1e-9);
    BOOST_CHECK_CLOSE(intrinsic.getFocalLength(), 50.0, 1e-9);
    BOOST_CHECK_CLOSE(intrinsic.getPixelAspectRatio(), 1.0, 1e-9);

    // The focal length ignores the pixel aspect ratio, which stretches x.
    intrinsic.setFocalLength(50.0, 2.0);
    BOOST_CHECK_CLOSE(intrinsic.getFocalLength(), 50.0, 1e-9);
    BOOST_CHECK_CLOSE(intrinsic.getPixelAspectRatio(), 2.0, 1e-9);
    BOOST_CHECK_CLOSE(intrinsic.getScale()(1), 50.0 * 6000.0 / 36.0, 1e-9);
    BOOST_CHECK_CLOSE(intrinsic.getScale()(0), 25.0 * 6000.0 / 36.0, 1e-9);

    // Compatibility mode: the focal length is the one along x.
    intrinsic.setFocalLength(50.0, 2.0, true);
    BOOST_CHECK_CLOSE(intrinsic.getScale()(0), 50.0 * 6000.0 / 36.0, 1e-9);
    BOOST_CHECK_CLOSE(intrinsic.getPixelAspectRatio(), 2.0, 1e-9);

    // Initial focal length
    BOOST_CHECK_EQUAL(intrinsic.getInitialFocalLength(), -1.0);
    intrinsic.setInitialFocalLength(35.0, 1.0);
    intrinsic.setFocalLength(35.0, 1.0);
    BOOST_CHECK_CLOSE(intrinsic.getInitialFocalLength(), 35.0, 1e-9);
    EXPECT_MATRIX_NEAR(intrinsic.getInitialScale(), intrinsic.getScale(), 1e-9);
    intrinsic.setInitialFocalLength(-1.0, 1.0);
    BOOST_CHECK_EQUAL(intrinsic.getInitialFocalLength(), -1.0);
}

// Millimeters are converted with the largest image side, so that portrait images give the same focal.
BOOST_AUTO_TEST_CASE(intrinsicState_focalLengthPortrait)
{
    Pinhole intrinsic(4000, 6000, 1.0, 1.0, 0.0, 0.0);
    intrinsic.setSensorWidth(36.0);
    intrinsic.setSensorHeight(24.0);

    intrinsic.setFocalLength(50.0, 1.0);
    BOOST_CHECK_CLOSE(intrinsic.getScale()(0), 50.0 * 6000.0 / 36.0, 1e-9);
    BOOST_CHECK_CLOSE(intrinsic.getFocalLength(), 50.0, 1e-9);

    intrinsic.setFocalLength(50.0, 2.0, true);
    BOOST_CHECK_CLOSE(intrinsic.getScale()(0), 50.0 * 6000.0 / 36.0, 1e-9);

    intrinsic.setInitialFocalLength(50.0, 1.0);
    intrinsic.setFocalLength(50.0, 1.0);
    BOOST_CHECK_CLOSE(intrinsic.getInitialFocalLength(), 50.0, 1e-9);
}

//-----------------
// Pinhole specifics
//-----------------

BOOST_AUTO_TEST_CASE(intrinsicState_pinholeK)
{
    Mat3 K;
    K << 1500.0, 0.0, 970.0, 0.0, 1520.0, 532.0, 0.0, 0.0, 1.0;

    Pinhole intrinsic(1920, 1080, 1.0, 1.0, 0.0, 0.0);
    intrinsic.setK(K);
    EXPECT_MATRIX_NEAR(intrinsic.K(), K, 1e-12);
    // The offset is relative to the image center.
    EXPECT_MATRIX_NEAR(intrinsic.getOffset(), Vec2(10.0, -8.0), 1e-12);
    EXPECT_MATRIX_NEAR(intrinsic.getPrincipalPoint(), Vec2(970.0, 532.0), 1e-12);

    intrinsic.setK(1000.0, 1100.0, 900.0, 500.0);
    Mat3 expected;
    expected << 1000.0, 0.0, 900.0, 0.0, 1100.0, 500.0, 0.0, 0.0, 1.0;
    EXPECT_MATRIX_NEAR(intrinsic.K(), expected, 1e-12);
}

BOOST_AUTO_TEST_CASE(intrinsicState_pinholeProjectiveEquivalent)
{
    makeRandomOperationsReproducible();

    const TestCamera tc = makeTestCamera(EINTRINSIC::PINHOLE_CAMERA, EDISTORTION::DISTORTION_RADIALK3, EUNDISTORTION::UNDISTORTION_NONE);
    const Pinhole& pinhole = dynamic_cast<const Pinhole&>(*tc.intrinsic);

    for (int i = 0; i < 10; ++i)
    {
        const geometry::Pose3 pose = geometry::randomPose();
        const Mat34 P = pinhole.getProjectiveEquivalent(pose);

        const Vec2 pix = samplePixels(pinhole)[i];
        const Vec3 X = pinhole.backprojectTransform(pix, false, pose, 4.0);

        const Vec3 x = P * X.homogeneous();
        EXPECT_MATRIX_NEAR(Vec2(x.hnormalized()), pinhole.transformProject(pose, X.homogeneous(), false), 1e-6);
    }
}

//-----------------
// Validity, visibility, field of view
//-----------------

BOOST_AUTO_TEST_CASE(intrinsicState_isValid)
{
    BOOST_CHECK(Pinhole(1920, 1080, 1500.0, 1500.0, 0.0, 0.0).isValid());
    BOOST_CHECK(!Pinhole(0, 1080, 1500.0, 1500.0, 0.0, 0.0).isValid());
    BOOST_CHECK(!Pinhole(1920, 0, 1500.0, 1500.0, 0.0, 0.0).isValid());
    BOOST_CHECK(!Equidistant(0, 0, 450.0, 0.0, 0.0).isValid());

    BOOST_CHECK(Equirectangular(2000, 1000, 300.0, 300.0, 0.0, 0.0).isValid());
    BOOST_CHECK(!Equirectangular(2000, 1000, 0.0, 300.0, 0.0, 0.0).isValid());
    BOOST_CHECK(!Equirectangular(0, 1000, 300.0, 300.0, 0.0, 0.0).isValid());
}

BOOST_AUTO_TEST_CASE(intrinsicState_isVisiblePixel)
{
    const Pinhole intrinsic(100, 50, 100.0, 100.0, 0.0, 0.0);

    BOOST_CHECK(intrinsic.isVisible(Vec2(0.0, 0.0)));
    BOOST_CHECK(intrinsic.isVisible(Vec2(99.5, 49.5)));
    BOOST_CHECK(!intrinsic.isVisible(Vec2(-0.1, 10.0)));
    BOOST_CHECK(!intrinsic.isVisible(Vec2(10.0, -0.1)));
    BOOST_CHECK(!intrinsic.isVisible(Vec2(100.0, 10.0)));
    BOOST_CHECK(!intrinsic.isVisible(Vec2(10.0, 50.0)));

    BOOST_CHECK(intrinsic.isVisible(Vec2f(99.5f, 49.5f)));
    BOOST_CHECK(!intrinsic.isVisible(Vec2f(100.0f, 10.0f)));
}

BOOST_AUTO_TEST_CASE(intrinsicState_isVisibleRay)
{
    const Pinhole pinhole(1000, 1000, 500.0, 500.0, 0.0, 0.0);
    BOOST_CHECK(pinhole.isVisibleRay(Vec3(0.0, 0.0, 1.0)));
    BOOST_CHECK(pinhole.isVisibleRay(Vec3(0.9, 0.9, 1.0)));
    BOOST_CHECK(!pinhole.isVisibleRay(Vec3(1.1, 0.0, 1.0)));
    BOOST_CHECK(!pinhole.isVisibleRay(Vec3(0.0, -1.1, 1.0)));
    BOOST_CHECK(!pinhole.isVisibleRay(Vec3(0.0, 0.0, -1.0)));
    BOOST_CHECK(!pinhole.isVisibleRay(Vec3(1.0, 0.0, 0.0)));

    // 36x24mm sensor, 1000px: the field of view is 24 / (f * 36 / 1000) radians.
    const Equidistant equidistant(1000, 1000, 200.0, 0.0, 0.0);
    const double halfFov = 0.5 * equidistant.getHorizontalFov();
    BOOST_CHECK(equidistant.isVisibleRay(Vec3(0.0, 0.0, 1.0)));
    BOOST_CHECK(equidistant.isVisibleRay(Vec3(std::sin(0.9 * halfFov), 0.0, std::cos(0.9 * halfFov))));
    BOOST_CHECK(!equidistant.isVisibleRay(Vec3(std::sin(1.1 * halfFov), 0.0, std::cos(1.1 * halfFov))));
    BOOST_CHECK(!equidistant.isVisibleRay(Vec3(0.0, 0.0, -1.0)));

    const Equirectangular equirectangular(2000, 1000, 2000.0 / (2.0 * M_PI), 1000.0 / M_PI, 0.0, 0.0);
    BOOST_CHECK(equirectangular.isVisibleRay(Vec3(0.0, 0.0, -1.0)));
    BOOST_CHECK(equirectangular.isVisibleRay(Vec3(0.0, 1.0, 0.0)));
}

BOOST_AUTO_TEST_CASE(intrinsicState_fieldOfView)
{
    // Pinhole: 2 atan(w / 2f), whatever the sensor size.
    Pinhole pinhole(1920, 1080, 1500.0, 1520.0, 0.0, 0.0);
    BOOST_CHECK_CLOSE(pinhole.getHorizontalFov(), 2.0 * std::atan(1920.0 / (2.0 * 1500.0)), 1e-9);
    BOOST_CHECK_CLOSE(pinhole.getVerticalFov(), 2.0 * std::atan(1080.0 / (2.0 * 1520.0)), 1e-9);
    pinhole.setSensorWidth(23.5);
    pinhole.setSensorHeight(15.6);
    BOOST_CHECK_CLOSE(pinhole.getHorizontalFov(), 2.0 * std::atan(1920.0 / (2.0 * 1500.0)), 1e-9);
    BOOST_CHECK_CLOSE(pinhole.getVerticalFov(), 2.0 * std::atan(1080.0 / (2.0 * 1520.0)), 1e-9);

    // Equidistant: the smallest sensor side over the focal length in millimeters.
    Equidistant equidistant(1920, 1080, 450.0, 0.0, 0.0);
    const double fmm = 450.0 * 36.0 / 1920.0;
    BOOST_CHECK_CLOSE(equidistant.getHorizontalFov(), 24.0 / fmm, 1e-9);
    BOOST_CHECK_CLOSE(equidistant.getVerticalFov(), 24.0 / fmm, 1e-9);

    const Equirectangular equirectangular(2000, 1000, 300.0, 300.0, 0.0, 0.0);
    BOOST_CHECK_CLOSE(equirectangular.getHorizontalFov(), 2.0 * M_PI, 1e-9);
    BOOST_CHECK_CLOSE(equirectangular.getVerticalFov(), M_PI, 1e-9);
}

BOOST_AUTO_TEST_CASE(intrinsicState_imagePlaneToCameraPlaneError)
{
    BOOST_CHECK_CLOSE(Pinhole(1920, 1080, 1500.0, 1520.0, 0.0, 0.0).imagePlaneToCameraPlaneError(3.0), 3.0 / 1500.0, 1e-9);
    BOOST_CHECK_CLOSE(Equidistant(1920, 1080, 450.0, 0.0, 0.0).imagePlaneToCameraPlaneError(3.0), 3.0 / 450.0, 1e-9);
    BOOST_CHECK_EQUAL(Equirectangular(2000, 1000, 300.0, 300.0, 0.0, 0.0).imagePlaneToCameraPlaneError(3.0), 0.0);
}

BOOST_AUTO_TEST_CASE(intrinsicState_maximalDistortion)
{
    const Pinhole noDistortion(1920, 1080, 1500.0, 1500.0, 0.0, 0.0);
    BOOST_CHECK_CLOSE(noDistortion.getMaximalDistortion(0.0, 0.8), 0.8, 1e-5);

    auto distortion = std::make_shared<DistortionRadialK3>(-0.18, 0.19, -0.025);
    const Pinhole withDistortion(1920, 1080, 1500.0, 1500.0, 0.0, 0.0, distortion);
    BOOST_CHECK_CLOSE(withDistortion.getMaximalDistortion(0.0, 0.8), distortion->getUndistortedRadius(0.8), 1e-5);
}

//-----------------
// Types
//-----------------

BOOST_AUTO_TEST_CASE(intrinsicState_typesAndCasts)
{
    std::shared_ptr<IntrinsicBase> pinhole = std::make_shared<Pinhole>(1920, 1080, 1500.0, 1500.0, 0.0, 0.0);
    std::shared_ptr<IntrinsicBase> equidistant = std::make_shared<Equidistant>(1920, 1080, 450.0, 0.0, 0.0);
    std::shared_ptr<IntrinsicBase> equirectangular = std::make_shared<Equirectangular>(2000, 1000, 300.0, 300.0, 0.0, 0.0);

    BOOST_CHECK_EQUAL(pinhole->getTypeStr(), "pinhole");
    BOOST_CHECK_EQUAL(equidistant->getTypeStr(), "equidistant");
    BOOST_CHECK_EQUAL(equirectangular->getTypeStr(), "equirectangular");

    BOOST_CHECK(Pinhole::cast(pinhole));
    BOOST_CHECK(!Pinhole::cast(equidistant));
    BOOST_CHECK(Equidistant::cast(equidistant));
    BOOST_CHECK(!Equidistant::cast(equirectangular));
    BOOST_CHECK(Equirectangular::cast(equirectangular));
    BOOST_CHECK(!Equirectangular::cast(pinhole));

    for (const auto& intrinsic : {pinhole, equidistant, equirectangular})
    {
        BOOST_CHECK(IntrinsicScaleOffset::cast(intrinsic));
        BOOST_CHECK(IntrinsicScaleOffsetDisto::cast(intrinsic));
        BOOST_CHECK(!intrinsic->hasDistortion());
    }
}

BOOST_AUTO_TEST_CASE(intrinsicState_equidistantCircle)
{
    // Default: the largest circle centered in the image.
    Equidistant intrinsic(1920, 1080, 450.0, 0.0, 0.0);
    BOOST_CHECK_EQUAL(intrinsic.getCircleRadius(), 540.0);
    EXPECT_MATRIX_NEAR(intrinsic.getCircleCenter(), Vec2(960.0, 540.0), 1e-12);

    // A null radius also means the default one.
    BOOST_CHECK_EQUAL(Equidistant(1920, 1080, 450.0, 0.0, 0.0, 0.0).getCircleRadius(), 540.0);
    BOOST_CHECK_EQUAL(Equidistant(1920, 1080, 450.0, 0.0, 0.0, 500.0).getCircleRadius(), 500.0);

    intrinsic.setCircleRadius(400.0);
    intrinsic.setCircleCenterX(950.0);
    intrinsic.setCircleCenterY(530.0);
    BOOST_CHECK_EQUAL(intrinsic.getCircleRadius(), 400.0);
    EXPECT_MATRIX_NEAR(intrinsic.getCircleCenter(), Vec2(950.0, 530.0), 1e-12);

    // The circle maps the camera plane to pixels.
    const Vec2 expected = intrinsic.getPrincipalPoint() + Vec2(400.0, 0.0);
    EXPECT_MATRIX_NEAR(intrinsic.cam2ima(Vec2(1.0, 0.0)), expected, 1e-12);
}
