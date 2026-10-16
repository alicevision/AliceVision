// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/camera/camera.hpp>
#include <aliceVision/camera/camera_test_common.hpp>
#include <aliceVision/camera/IntrinsicInitMode.hpp>
#include <aliceVision/geometry/Pose3.hpp>

#define BOOST_TEST_MODULE cameraCommon

#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>
#include <aliceVision/unitTest.hpp>

#include <sstream>

using namespace aliceVision;
using namespace aliceVision::camera;
using namespace aliceVision::camera::test;

//-----------------
// Enums <-> strings
//-----------------

BOOST_AUTO_TEST_CASE(cameraCommon_intrinsicEnum)
{
    for (EINTRINSIC type : allIntrinsicTypes())
    {
        const std::string name = EINTRINSIC_enumToString(type);
        BOOST_CHECK_EQUAL(EINTRINSIC_stringToEnum(name), type);

        std::string upper = name;
        std::transform(upper.begin(), upper.end(), upper.begin(), ::toupper);
        BOOST_CHECK_EQUAL(EINTRINSIC_stringToEnum(upper), type);

        std::stringstream out;
        out << type;
        BOOST_CHECK_EQUAL(out.str(), name);

        std::stringstream in(name);
        EINTRINSIC parsed = EINTRINSIC::UNKNOWN;
        in >> parsed;
        BOOST_CHECK_EQUAL(parsed, type);
    }

    BOOST_CHECK_EQUAL(EINTRINSIC_enumToString(EINTRINSIC::PINHOLE_CAMERA), "pinhole");
    BOOST_CHECK_EQUAL(EINTRINSIC_enumToString(EINTRINSIC::EQUIDISTANT_CAMERA), "equidistant");
    BOOST_CHECK_EQUAL(EINTRINSIC_enumToString(EINTRINSIC::EQUIRECTANGULAR_CAMERA), "equirectangular");

    BOOST_CHECK_THROW(EINTRINSIC_enumToString(EINTRINSIC::UNKNOWN), std::out_of_range);
    BOOST_CHECK_THROW(EINTRINSIC_stringToEnum("unknown"), std::out_of_range);
    BOOST_CHECK_THROW(EINTRINSIC_stringToEnum(""), std::out_of_range);
}

BOOST_AUTO_TEST_CASE(cameraCommon_intrinsicTypePredicates)
{
    BOOST_CHECK(isPinhole(EINTRINSIC::PINHOLE_CAMERA));
    BOOST_CHECK(!isPinhole(EINTRINSIC::EQUIDISTANT_CAMERA));
    BOOST_CHECK(!isPinhole(EINTRINSIC::EQUIRECTANGULAR_CAMERA));
    BOOST_CHECK(!isPinhole(EINTRINSIC::UNKNOWN));

    BOOST_CHECK(isEquidistant(EINTRINSIC::EQUIDISTANT_CAMERA));
    BOOST_CHECK(!isEquidistant(EINTRINSIC::PINHOLE_CAMERA));

    BOOST_CHECK(isEquirectangular(EINTRINSIC::EQUIRECTANGULAR_CAMERA));
    BOOST_CHECK(!isEquirectangular(EINTRINSIC::PINHOLE_CAMERA));

    for (EINTRINSIC type : allIntrinsicTypes())
    {
        BOOST_CHECK(isValid(type));
    }
    BOOST_CHECK(!isValid(EINTRINSIC::UNKNOWN));
}

BOOST_AUTO_TEST_CASE(cameraCommon_distortionEnum)
{
    for (EDISTORTION type : allDistortionTypes())
    {
        const std::string name = EDISTORTION_enumToString(type);
        BOOST_CHECK_EQUAL(EDISTORTION_stringToEnum(name), type);

        std::string upper = name;
        std::transform(upper.begin(), upper.end(), upper.begin(), ::toupper);
        BOOST_CHECK_EQUAL(EDISTORTION_stringToEnum(upper), type);
    }

    BOOST_CHECK_THROW(EDISTORTION_stringToEnum("radial"), std::out_of_range);
    BOOST_CHECK_THROW(EDISTORTION_stringToEnum(""), std::out_of_range);
}

BOOST_AUTO_TEST_CASE(cameraCommon_undistortionEnum)
{
    for (EUNDISTORTION type : allUndistortionTypes())
    {
        const std::string name = EUNDISTORTION_enumToString(type);
        BOOST_CHECK_EQUAL(EUNDISTORTION_stringToEnum(name), type);

        std::string upper = name;
        std::transform(upper.begin(), upper.end(), upper.begin(), ::toupper);
        BOOST_CHECK_EQUAL(EUNDISTORTION_stringToEnum(upper), type);
    }

    BOOST_CHECK_THROW(EUNDISTORTION_stringToEnum("3de"), std::out_of_range);
    BOOST_CHECK_THROW(EUNDISTORTION_stringToEnum(""), std::out_of_range);
}

BOOST_AUTO_TEST_CASE(cameraCommon_initModeEnum)
{
    for (EInitMode mode : {EInitMode::NONE, EInitMode::CALIBRATED, EInitMode::ESTIMATED, EInitMode::UNKNOWN})
    {
        const std::string name = EInitMode_enumToString(mode);
        BOOST_CHECK(EInitMode_stringToEnum(name) == mode);

        std::string upper = name;
        std::transform(upper.begin(), upper.end(), upper.begin(), ::toupper);
        BOOST_CHECK(EInitMode_stringToEnum(upper) == mode);

        std::stringstream out;
        out << mode;
        BOOST_CHECK_EQUAL(out.str(), name);

        std::stringstream in(name);
        EInitMode parsed = EInitMode::NONE;
        in >> parsed;
        BOOST_CHECK(parsed == mode);
    }

    BOOST_CHECK_THROW(EInitMode_stringToEnum("calibration"), std::out_of_range);
    BOOST_CHECK_THROW(EInitMode_enumToString(static_cast<EInitMode>(42)), std::out_of_range);
}

// Camera model names written by older versions of AliceVision.
BOOST_AUTO_TEST_CASE(cameraCommon_compatibilityStringToEnums)
{
    struct Legacy
    {
        std::string name;
        EINTRINSIC intrinsic;
        EDISTORTION distortion;
        EUNDISTORTION undistortion;
    };
    const std::vector<Legacy> legacy = {
      {"pinhole", EINTRINSIC::PINHOLE_CAMERA, EDISTORTION::DISTORTION_NONE, EUNDISTORTION::UNDISTORTION_NONE},
      {"radial1", EINTRINSIC::PINHOLE_CAMERA, EDISTORTION::DISTORTION_RADIALK1, EUNDISTORTION::UNDISTORTION_NONE},
      {"radial3", EINTRINSIC::PINHOLE_CAMERA, EDISTORTION::DISTORTION_RADIALK3, EUNDISTORTION::UNDISTORTION_NONE},
      {"brown", EINTRINSIC::PINHOLE_CAMERA, EDISTORTION::DISTORTION_BROWN, EUNDISTORTION::UNDISTORTION_NONE},
      {"fisheye4", EINTRINSIC::PINHOLE_CAMERA, EDISTORTION::DISTORTION_FISHEYE, EUNDISTORTION::UNDISTORTION_NONE},
      {"fisheye1", EINTRINSIC::PINHOLE_CAMERA, EDISTORTION::DISTORTION_FISHEYE1, EUNDISTORTION::UNDISTORTION_NONE},
      {"3deanamorphic4", EINTRINSIC::PINHOLE_CAMERA, EDISTORTION::DISTORTION_NONE, EUNDISTORTION::UNDISTORTION_3DEANAMORPHIC4},
      {"equidistant", EINTRINSIC::EQUIDISTANT_CAMERA, EDISTORTION::DISTORTION_NONE, EUNDISTORTION::UNDISTORTION_NONE},
      {"equidistant_r3", EINTRINSIC::EQUIDISTANT_CAMERA, EDISTORTION::DISTORTION_RADIALK3PT, EUNDISTORTION::UNDISTORTION_NONE},
      {"RADIAL3", EINTRINSIC::PINHOLE_CAMERA, EDISTORTION::DISTORTION_RADIALK3, EUNDISTORTION::UNDISTORTION_NONE},
    };

    for (const Legacy& l : legacy)
    {
        BOOST_TEST_CONTEXT(l.name)
        {
            EINTRINSIC intrinsic = EINTRINSIC::UNKNOWN;
            EDISTORTION distortion = EDISTORTION::DISTORTION_FISHEYE;
            EUNDISTORTION undistortion = EUNDISTORTION::UNDISTORTION_3DERADIAL4;
            compatibilityStringToEnums(l.name, intrinsic, distortion, undistortion);

            BOOST_CHECK_EQUAL(intrinsic, l.intrinsic);
            BOOST_CHECK_EQUAL(distortion, l.distortion);
            BOOST_CHECK_EQUAL(undistortion, l.undistortion);
        }
    }

    EINTRINSIC intrinsic;
    EDISTORTION distortion;
    EUNDISTORTION undistortion;
    BOOST_CHECK_THROW(compatibilityStringToEnums("radial2", intrinsic, distortion, undistortion), std::out_of_range);
}

//-----------------
// Factories
//-----------------

BOOST_AUTO_TEST_CASE(cameraCommon_createDistortion)
{
    BOOST_CHECK(!createDistortion(EDISTORTION::DISTORTION_NONE));

    for (EDISTORTION type : allDistortionTypes())
    {
        if (type == EDISTORTION::DISTORTION_NONE)
        {
            continue;
        }

        BOOST_TEST_CONTEXT(EDISTORTION_enumToString(type))
        {
            const std::vector<double> params = distortionTestParameters(type);

            std::shared_ptr<Distortion> distortion = createDistortion(type);
            BOOST_REQUIRE(distortion);
            BOOST_CHECK_EQUAL(distortion->getType(), type);
            BOOST_CHECK_EQUAL(distortion->getDistortionParametersCount(), params.size());

            // Default parameters: null distortion, except for the fisheye model which always maps r to atan(r).
            if (type != EDISTORTION::DISTORTION_FISHEYE)
            {
                EXPECT_MATRIX_NEAR(distortion->addDistortion(Vec2(0.3, -0.2)), Vec2(0.3, -0.2), 1e-12);
            }

            // Clone keeps type and parameters.
            distortion->setParameters(params);
            std::unique_ptr<Distortion> copy(distortion->clone());
            BOOST_CHECK_EQUAL(copy->getType(), type);
            BOOST_CHECK(*copy == *distortion);

            // setParameters ignores a wrong number of parameters.
            distortion->setParameters(std::vector<double>(params.size() + 1, 0.5));
            BOOST_CHECK(distortion->getParameters() == params);

            // Locking
            BOOST_CHECK(!distortion->isLocked());
            distortion->setLocked(true);
            BOOST_CHECK(distortion->isLocked());
        }
    }

    // Explicit parameters
    std::shared_ptr<Distortion> k3 = createDistortion(EDISTORTION::DISTORTION_RADIALK3, {0.1, 0.2, 0.3});
    BOOST_CHECK(k3->getParameters() == std::vector<double>({0.1, 0.2, 0.3}));
    BOOST_CHECK_THROW(createDistortion(EDISTORTION::DISTORTION_RADIALK3, {0.1, 0.2}), std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(cameraCommon_createUndistortion)
{
    BOOST_CHECK(!createUndistortion(EUNDISTORTION::UNDISTORTION_NONE, 1920, 1080));

    for (EUNDISTORTION type : allUndistortionTypes())
    {
        if (type == EUNDISTORTION::UNDISTORTION_NONE)
        {
            continue;
        }

        BOOST_TEST_CONTEXT(EUNDISTORTION_enumToString(type))
        {
            const std::vector<double> params = undistortionTestParameters(type);

            std::shared_ptr<Undistortion> undistortion = createUndistortion(type, 1920, 1080);
            BOOST_REQUIRE(undistortion);
            BOOST_CHECK_EQUAL(undistortion->getType(), type);
            BOOST_CHECK_EQUAL(undistortion->getUndistortionParametersCount(), params.size());
            EXPECT_MATRIX_NEAR(undistortion->getSize(), Vec2(1920, 1080), 1e-12);

            // Default parameters: null distortion.
            EXPECT_MATRIX_NEAR(undistortion->undistort(Vec2(100.0, 200.0)), Vec2(100.0, 200.0), 1e-9);

            undistortion->setParameters(params);
            std::unique_ptr<Undistortion> copy(undistortion->clone());
            BOOST_CHECK_EQUAL(copy->getType(), type);
            BOOST_CHECK(*copy == *undistortion);

            undistortion->setParameters(std::vector<double>(params.size() + 1, 0.5));
            BOOST_CHECK(undistortion->getParameters() == params);
        }
    }

    std::shared_ptr<Undistortion> k3 = createUndistortion(EUNDISTORTION::UNDISTORTION_RADIALK3, 1920, 1080, {0.1, 0.2, 0.3});
    BOOST_CHECK(k3->getParameters() == std::vector<double>({0.1, 0.2, 0.3}));
    BOOST_CHECK_THROW(createUndistortion(EUNDISTORTION::UNDISTORTION_RADIALK3, 1920, 1080, {0.1}), std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(cameraCommon_createIntrinsic)
{
    for (EINTRINSIC intrinsicType : allIntrinsicTypes())
    {
        for (EDISTORTION distortionType : allDistortionTypes())
        {
            for (EUNDISTORTION undistortionType : allUndistortionTypes())
            {
                BOOST_TEST_CONTEXT(EINTRINSIC_enumToString(intrinsicType)
                                   << "/" << EDISTORTION_enumToString(distortionType) << "/" << EUNDISTORTION_enumToString(undistortionType))
                {
                    std::shared_ptr<IntrinsicBase> intrinsic =
                      createIntrinsic(intrinsicType, distortionType, undistortionType, 1920, 1080, 1500.0, 1520.0, 10.0, -5.0);
                    BOOST_REQUIRE(intrinsic);
                    BOOST_CHECK_EQUAL(intrinsic->getType(), intrinsicType);
                    BOOST_CHECK_EQUAL(intrinsic->w(), 1920);
                    BOOST_CHECK_EQUAL(intrinsic->h(), 1080);

                    auto isod = IntrinsicScaleOffsetDisto::cast(intrinsic);
                    BOOST_REQUIRE(isod);
                    EXPECT_MATRIX_NEAR(isod->getOffset(), Vec2(10.0, -5.0), 1e-12);

                    if (isEquirectangular(intrinsicType))
                    {
                        // Equirectangular cameras have no distortion.
                        BOOST_CHECK(!isod->getDistortion());
                        BOOST_CHECK(!isod->getUndistortion());
                        EXPECT_MATRIX_NEAR(isod->getScale(), Vec2(1500.0, 1520.0), 1e-12);
                    }
                    else if (isEquidistant(intrinsicType))
                    {
                        // A single focal length, and no undistortion.
                        BOOST_CHECK_EQUAL(getDistortionType(*intrinsic), distortionType);
                        BOOST_CHECK(!isod->getUndistortion());
                        EXPECT_MATRIX_NEAR(isod->getScale(), Vec2(1500.0, 1500.0), 1e-12);
                    }
                    else
                    {
                        BOOST_CHECK_EQUAL(getDistortionType(*intrinsic), distortionType);
                        const EUNDISTORTION created = isod->getUndistortion() ? isod->getUndistortion()->getType() : EUNDISTORTION::UNDISTORTION_NONE;
                        BOOST_CHECK_EQUAL(created, undistortionType);
                        EXPECT_MATRIX_NEAR(isod->getScale(), Vec2(1500.0, 1520.0), 1e-12);
                    }
                }
            }
        }
    }

    BOOST_CHECK(!createIntrinsic(EINTRINSIC::UNKNOWN, EDISTORTION::DISTORTION_NONE, EUNDISTORTION::UNDISTORTION_NONE));
}

BOOST_AUTO_TEST_CASE(cameraCommon_createPinholeAndEquidistant)
{
    std::shared_ptr<Pinhole> pinhole =
      createPinhole(EDISTORTION::DISTORTION_RADIALK3, EUNDISTORTION::UNDISTORTION_NONE, 1920, 1080, 1500.0, 1520.0, 10.0, -5.0, {0.1, 0.2, 0.3});
    BOOST_REQUIRE(pinhole);
    BOOST_CHECK(pinhole->getDistortionParams() == std::vector<double>({0.1, 0.2, 0.3}));
    BOOST_CHECK(!pinhole->getUndistortion());
    EXPECT_MATRIX_NEAR(pinhole->getScale(), Vec2(1500.0, 1520.0), 1e-12);
    EXPECT_MATRIX_NEAR(pinhole->getOffset(), Vec2(10.0, -5.0), 1e-12);

    std::shared_ptr<Pinhole> pinhole3de =
      createPinhole(EDISTORTION::DISTORTION_NONE, EUNDISTORTION::UNDISTORTION_RADIALK3, 1920, 1080, 1500.0, 1520.0, 0.0, 0.0, {0.1, 0.2, 0.3});
    BOOST_REQUIRE(pinhole3de->getUndistortion());
    BOOST_CHECK(pinhole3de->getUndistortion()->getParameters() == std::vector<double>({0.1, 0.2, 0.3}));
    BOOST_CHECK(!pinhole3de->getDistortion());

    std::shared_ptr<Equidistant> equidistant = createEquidistant(EDISTORTION::DISTORTION_RADIALK3PT, 1920, 1080, 450.0, 10.0, -5.0, {0.1, 0.2, 0.3});
    BOOST_REQUIRE(equidistant);
    BOOST_CHECK(equidistant->getDistortionParams() == std::vector<double>({0.1, 0.2, 0.3}));
    EXPECT_MATRIX_NEAR(equidistant->getScale(), Vec2(450.0, 450.0), 1e-12);
    EXPECT_MATRIX_NEAR(equidistant->getOffset(), Vec2(10.0, -5.0), 1e-12);
}

BOOST_AUTO_TEST_CASE(cameraCommon_getDistortionType)
{
    for (const TestCamera& tc : makeTestCameras())
    {
        BOOST_CHECK_EQUAL(getDistortionType(*tc.intrinsic), tc.distortionType);
    }
}

//-----------------
// Rays
//-----------------

BOOST_AUTO_TEST_CASE(cameraCommon_angleBetweenRays)
{
    BOOST_CHECK_CLOSE(angleBetweenRays(Vec3(1.0, 0.0, 0.0), Vec3(0.0, 2.0, 0.0)), 90.0, 1e-9);
    BOOST_CHECK_CLOSE(angleBetweenRays(Vec3(1.0, 0.0, 0.0), Vec3(1.0, 1.0, 0.0)), 45.0, 1e-9);
    // The result is clamped away from 0 and 180 degrees.
    BOOST_CHECK_SMALL(angleBetweenRays(Vec3(0.0, 0.0, 3.0), Vec3(0.0, 0.0, 1.0)), 0.01);
    BOOST_CHECK_SMALL(180.0 - angleBetweenRays(Vec3(0.0, 0.0, 1.0), Vec3(0.0, 0.0, -1.0)), 0.01);

    // From two camera centers to a point.
    const geometry::Pose3 pose1(Mat3::Identity(), Vec3(-1.0, 0.0, 0.0));
    const geometry::Pose3 pose2(Mat3::Identity(), Vec3(1.0, 0.0, 0.0));
    BOOST_CHECK_CLOSE(angleBetweenRays(pose1, pose2, Vec3(0.0, 0.0, 1.0)), 90.0, 1e-9);
}

BOOST_AUTO_TEST_CASE(cameraCommon_angleBetweenObservations)
{
    makeRandomOperationsReproducible();

    for (const TestCamera& tc : makeTestCameras())
    {
        const IntrinsicBase& cam = *tc.intrinsic;

        BOOST_TEST_CONTEXT(tc.name)
        {
            const geometry::Pose3 pose1 = geometry::randomPose();
            const geometry::Pose3 pose2(pose1.rotation(), pose1.center() + Vec3(0.5, 0.1, -0.2));

            const Vec2 pix1 = samplePixels(cam)[10];
            const Vec3 X = cam.backprojectTransform(pix1, true, pose1, 4.0);
            const Vec2 pix2 = cam.transformProject(pose2, X.homogeneous(), true);

            // The world ray through a pixel points from the camera center to the 3D point.
            const Vec3 ray = applyIntrinsicExtrinsic(pose1, &cam, pix1);
            EXPECT_MATRIX_NEAR(ray, (X - pose1.center()).normalized(), 1e-6);

            const double expected = angleBetweenRays(pose1, pose2, X);
            BOOST_CHECK_CLOSE(angleBetweenRays(pose1, &cam, pose2, &cam, pix1, pix2), expected, 1e-3);
        }
    }
}
