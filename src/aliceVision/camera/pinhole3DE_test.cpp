// This file is part of the AliceVision project.
// Copyright (c) 2021 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/camera/camera.hpp>

#define BOOST_TEST_MODULE pinholeRadial

#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>
#include <aliceVision/unitTest.hpp>

using namespace aliceVision;
using namespace aliceVision::camera;

//-----------------
// Test summary:
//-----------------
// - Create a Pinhole3DERadial4 camera
// - Generate random point inside the image domain
// - Add and remove distortion and assert we found back the generated point
// - Check the last point in the camera & image domain
// - Assert that the tested distortion is not null (in order to ensure validity of the test)
//-----------------
BOOST_AUTO_TEST_CASE(cameraPinhole3DE_disto_undisto_Radial4)
{
  const Pinhole3DERadial4 cam(1000, 1000, 1000, 1000, 0, 0,
    -0.4839495643487452,
    1.0301284234642258,
    0.014928332802185664,
    -0.0007797104872758904,
    -0.038994206396183909,
    8.0474385001183646e-05);

  const double epsilon = 1e-4;
  for(int i = 0; i < 10; ++i)
  {
    // generate random point inside the image domain (last random to avoid 0,0)
    const Vec2 ptImage_gt = (Vec2::Random() * 800./2.) + Vec2(500,500) + Vec2::Random();

    const Vec2 ptCamera = cam.ima2cam(ptImage_gt);
    // Check that adding and removing distortion allow to recover the provided point
    EXPECT_MATRIX_NEAR( ptCamera, cam.removeDistortion(cam.addDistortion(ptCamera)), epsilon);
    EXPECT_MATRIX_NEAR( ptImage_gt, cam.cam2ima(cam.removeDistortion(cam.addDistortion(ptCamera))), epsilon);

    // Assert that distortion field is not null and it has moved the initial provided point
    BOOST_CHECK(! (cam.addDistortion(ptCamera) == cam.removeDistortion(cam.addDistortion(ptCamera))) ) ;

    // Check projection / back-projection
    const double depth_gt = std::abs(Vec2::Random()(0)) * 100.0;
    const geometry::Pose3 pose(geometry::randomPose());

    const Vec3 pt3d = cam.backproject(ptImage_gt, true, pose, depth_gt);
    const Vec2 pt2d_proj = cam.project(pose, pt3d.homogeneous(), true);

    EXPECT_MATRIX_NEAR(ptImage_gt, pt2d_proj, epsilon);
  }
}

//-----------------
// Test summary:
//-----------------
// - Create a Pinhole3DEClassicLD camera
// - Generate random point inside the image domain
// - Add and remove distortion and assert we found back the generated point
// - Check the last point in the camera & image domain
// - Assert that the tested distortion is not null (in order to ensure validity of the test)
//-----------------
BOOST_AUTO_TEST_CASE(cameraPinhole3DE_disto_undisto_ClassicLD)
{
  const Pinhole3DEClassicLD cam(1000, 1000, 1000, 1000, 0, 0,
    -0.34768564335290314,
    1.5809150001711287,
    -0.17204522667665839,
    -0.15541950225726325,
    1.1240093674337683);

  const double epsilon = 1e-4;
  for(int i = 0; i < 10; ++i)
  {
    // generate random point inside the image domain (last random to avoid 0,0)
    const Vec2 ptImage_gt = (Vec2::Random() * 800./2.) + Vec2(500,500) + Vec2::Random();

    const Vec2 ptCamera = cam.ima2cam(ptImage_gt);
    // Check that adding and removing distortion allow to recover the provided point
    EXPECT_MATRIX_NEAR( ptCamera, cam.removeDistortion(cam.addDistortion(ptCamera)), epsilon);
    EXPECT_MATRIX_NEAR( ptImage_gt, cam.cam2ima(cam.removeDistortion(cam.addDistortion(ptCamera))), epsilon);

    // Assert that distortion field is not null and it has moved the initial provided point
    BOOST_CHECK(! (cam.addDistortion(ptCamera) == cam.removeDistortion(cam.addDistortion(ptCamera))) ) ;

    // Check projection / back-projection
    const double depth_gt = std::abs(Vec2::Random()(0)) * 100.0;
    const geometry::Pose3 pose(geometry::randomPose());

    const Vec3 pt3d = cam.backproject(ptImage_gt, true, pose, depth_gt);
    const Vec2 pt2d_proj = cam.project(pose, pt3d.homogeneous(), true);

    EXPECT_MATRIX_NEAR(ptImage_gt, pt2d_proj, epsilon);
  }
}

