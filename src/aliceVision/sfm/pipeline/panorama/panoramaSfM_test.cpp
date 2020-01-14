// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/feature/imageDescriberCommon.hpp>
#include <aliceVision/sfm/utils/statistics.hpp>
#include <aliceVision/sfm/utils/syntheticScene.hpp>
#include <aliceVision/sfm/sfm.hpp>

#include <cmath>
#include <cstdio>
#include <iostream>

#define BOOST_TEST_MODULE PANORAMA_SFM
#include <boost/test/included/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <aliceVision/unitTest.hpp>

using namespace aliceVision;
using namespace aliceVision::camera;
using namespace aliceVision::geometry;
using namespace aliceVision::sfm;
using namespace aliceVision::sfmData;

// Test summary:
// - Create a rotation matrix between two views
// - Create two matrices of calibration for two views


BOOST_AUTO_TEST_CASE(PANORAMA_SFM)
{
    // rotation between the two views
    const Mat3 rotation = aliceVision::rotationXYZ(0.01, -0.001, -0.2);
    ALICEVISION_LOG_INFO("Ground truth rotation:\n" << rotation);

    // some random 3D points on the unit sphere
    const Mat3X pts3d = Mat3X::Random(3, 20).colwise().normalized();
    ALICEVISION_LOG_INFO("points 3d:\n" << pts3d);

    // calibration 1
    Mat3 K1;
    K1 << 1200, 0, 960,
         0, 1200, 540,
         0, 0, 1;
    ALICEVISION_LOG_INFO("K1:\n" << K1);

    // calibration 2
    Mat3 K2;
    K2 << 1100, 0, 960,
           0, 1100, 540,
           0, 0, 1;
    ALICEVISION_LOG_INFO("K2:\n" << K2);

    // 2d points on each side as projection of the 3D points
    const Mat2X pts1 = (K1 * pts3d).colwise().hnormalized();
    //        ALICEVISION_LOG_INFO("pts1:\n" << pts1);
    const Mat2X pts2 = (K2 * rotation *  pts3d).colwise().hnormalized();
    //        ALICEVISION_LOG_INFO("pts2:\n" << pts2);

    // test the uncalibrated version, just pass the 2d points and compute R

    const double epsilon = 1e-4;

    // Relative Rotation from H
    {
        RelativeRotationInfo rotationInfo{};
    ALICEVISION_LOG_INFO("\n\n###########################################\nUncalibrated from H");
        robustRelativeRotation_fromH(K1, K2, pts1, pts2,
                              std::make_pair<std::size_t, std::size_t>(1920, 1080),
                              std::make_pair<std::size_t, std::size_t>(1920, 1080),
                              rotationInfo);
        ALICEVISION_LOG_INFO("rotation.inverse() * rotationInfo._relativeRotation:\n" << rotation.inverse() * rotationInfo._relativeRotation);

        EXPECT_MATRIX_NEAR(rotation, rotationInfo._relativeRotation, epsilon);


        // test the calibrated version, compute normalized points for each view (by multiplying by the inverse of K) and estimate H and R
    ALICEVISION_LOG_INFO("\n\n###########################################\nCalibrated from H");
        robustRelativeRotation_fromH(Mat3::Identity(), Mat3::Identity(),
                              (K1.inverse()*pts1.colwise().homogeneous()).colwise().hnormalized(),
                              (K2.inverse()*pts2.colwise().homogeneous()).colwise().hnormalized(),
                              std::make_pair<std::size_t, std::size_t>(1920, 1080),
                              std::make_pair<std::size_t, std::size_t>(1920, 1080),
                              rotationInfo);
        ALICEVISION_LOG_INFO("rotation.inverse() * rotationInfo._relativeRotation:\n" << rotation.inverse() * rotationInfo._relativeRotation);

        EXPECT_MATRIX_NEAR(rotation, rotationInfo._relativeRotation, epsilon);
    }

  /*
    // Relative Rotation from E
    {
    RelativePoseInfo rotationInfo{};
    ALICEVISION_LOG_INFO("\n\n###########################################\nUncalibrated from E");
        robustRelativeRotation_fromE(K1, K2, pts1, pts2,
                              std::make_pair<std::size_t, std::size_t>(1920, 1080),
                              std::make_pair<std::size_t, std::size_t>(1920, 1080),
                              rotationInfo);
    ALICEVISION_LOG_INFO("rotation.inverse() * rotationInfo._relativeRotation:\n" << rotation.inverse() * rotationInfo.relativePose.rotation());

    EXPECT_MATRIX_NEAR(rotation, rotationInfo.relativePose.rotation(), epsilon);



        // test the calibrated version, compute normalized points for each view (by multiplying by the inverse of K) and estimate H and R
    ALICEVISION_LOG_INFO("\n\n###########################################\nCalibrated from E");
        robustRelativeRotation_fromE(Mat3::Identity(), Mat3::Identity(),
                              (K1.inverse()*pts1.colwise().homogeneous()).colwise().hnormalized(),
                              (K2.inverse()*pts2.colwise().homogeneous()).colwise().hnormalized(),
                              std::make_pair<std::size_t, std::size_t>(1920, 1080),
                              std::make_pair<std::size_t, std::size_t>(1920, 1080),
                              rotationInfo);
    ALICEVISION_LOG_INFO("rotation.inverse() * rotationInfo._relativeRotation:\n" << rotation.inverse() * rotationInfo.relativePose.rotation());

    EXPECT_MATRIX_NEAR(rotation, rotationInfo.relativePose.rotation(), epsilon);
    }
  */
}
