// This file is part of the AliceVision project.
// Copyright (c) 2020 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/camera/camera.hpp>
#include <aliceVision/calibration/distortionEstimation.hpp>

#define BOOST_TEST_MODULE distortioncalibration

#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>
#include <aliceVision/unitTest.hpp>

using namespace aliceVision;

//-----------------
// Test summary:
//-----------------
// Generate a set of distorted points
// Estimate the distortion
// Undistort the points
// Checks that these points are close to the original points
//-----------------
BOOST_AUTO_TEST_CASE(distortionCalibration_calibrate_classicld)
{
  const camera::Pinhole3DEClassicLD cam(1000, 1000, 1000, 1000, 0, 0, -0.34768564335290314, 1.5809150001711287, -0.17204522667665839, -0.15541950225726325, 1.1240093674337683);

  //Create points
  std::vector<calibration::PointPair> pts;
  for (int i = 0; i < 1000; i+=10)
  {
    for (int j = 0; j < 1000; j+=10)
    {
      const Vec2 pt(j, i);
      const Vec2 cpt = cam.ima2cam(pt);
      const Vec2 upt = cam.addDistortion(cpt);
      const Vec2 distortedPoint = cam.cam2ima(upt);

      calibration::PointPair pp;
      pp.distortedPoint = distortedPoint;
      pp.undistortedPoint = pt;
      pts.push_back(pp);
    }
  }


  //Calibrate
  calibration::Statistics st;
  std::shared_ptr<camera::Pinhole> estimatedCam = std::make_shared<camera::Pinhole3DEClassicLD>(1000, 1000, 1000, 1000, 0, 0, 0, M_PI_2, 0, 0, 0);

  std::vector<bool> lockedDistortions = {false, false, false, false, false};
  BOOST_CHECK(calibration::estimate(estimatedCam, st, pts, true, false, lockedDistortions));

  for (const calibration::PointPair & pair : pts)
  {
    const Vec2 cpt = estimatedCam->ima2cam(pair.distortedPoint);
    const Vec2 upt = estimatedCam->removeDistortion(cpt);
    const Vec2 undistortedPoint = estimatedCam->cam2ima(upt);

    const double residual = (undistortedPoint - pair.undistortedPoint).norm();

    BOOST_CHECK_SMALL(residual, 1e-2);
  }
}

//-----------------
// Test summary:
//-----------------
// Generate a set of distorted points
// Estimate the distortion
// Undistort the points
// Checks that these points are close to the original points
//-----------------
BOOST_AUTO_TEST_CASE(distortionCalibration_calibrate_radial4)
{
  const camera::Pinhole3DERadial4 cam(1000, 1000, 1000, 1000, 0, 0, -0.4839495643487452, 1.0301284234642258, 0.014928332802185664, -0.0007797104872758904, -0.038994206396183909, 8.0474385001183646e-05);

  //Create points
  std::vector<calibration::PointPair> pts;
  for (int i = 0; i < 1000; i+=10)
  {
    for (int j = 0; j < 1000; j+=10)
    {
      const Vec2 pt(j, i);
      const Vec2 cpt = cam.ima2cam(pt);
      const Vec2 upt = cam.addDistortion(cpt);
      const Vec2 distortedPoint = cam.cam2ima(upt);

      calibration::PointPair pp;
      pp.distortedPoint = distortedPoint;
      pp.undistortedPoint = pt;
      pts.push_back(pp);
    }
  }


  //Calibrate
  calibration::Statistics st;
  std::shared_ptr<camera::Pinhole> estimatedCam = std::make_shared<camera::Pinhole3DERadial4>(1000, 1000, 1000, 1000, 0, 0, 0, 0, 0, 0, 0);

  std::vector<bool> lockedDistortions = {false, false, false, false, false, false};
  BOOST_CHECK(calibration::estimate(estimatedCam, st, pts, true, false, lockedDistortions));

  for (const calibration::PointPair & pair : pts)
  {
    const Vec2 cpt = estimatedCam->ima2cam(pair.distortedPoint);
    const Vec2 upt = estimatedCam->removeDistortion(cpt);
    const Vec2 undistortedPoint = estimatedCam->cam2ima(upt);

    const double residual = (undistortedPoint - pair.undistortedPoint).norm();

    BOOST_CHECK_SMALL(residual, 1e-2);
  }
}


//-----------------
// Test summary:
//-----------------
// Generate a set of distorted points on lines
// Estimate the distortion
// Undistort the points
// Checks that these points are close to the original points
// !!!! Note that for convergence reasons, this calibration is reversed (calibrates undistortion)
//-----------------
BOOST_AUTO_TEST_CASE(distortionCalibration_calibrate_lines_classicld)
{
  const camera::Pinhole3DEClassicLD cam(1000, 1000, 1000, 1000, 0, 0, -0.34768564335290314, 1.5809150001711287, -0.17204522667665839, -0.15541950225726325, 1.1240093674337683);

  //Create points
  std::vector<calibration::PointPair> pts;
  std::vector<calibration::LineWithPoints> lines;
  for (int i = 0; i < 1000; i+=10)
  {
    calibration::LineWithPoints line;
    line.angle = M_PI_4;
    line.dist = 0;
    line.horizontal = true;

    for (int j = 0; j < 1000; j+=10)
    {
      const Vec2 pt(j, i);
      const Vec2 cpt = cam.ima2cam(pt);
      const Vec2 upt = cam.removeDistortion(cpt);
      const Vec2 distortedPoint = cam.cam2ima(upt);

      calibration::PointPair pp;
      pp.distortedPoint = distortedPoint;
      pp.undistortedPoint = pt;
      pts.push_back(pp);

      line.points.push_back(distortedPoint);
    }

    lines.push_back(line);
  }

  for (int j = 0; j < 1000; j+=10)
  {
    calibration::LineWithPoints line;
    line.angle = M_PI_4;
    line.dist = 0;
    line.horizontal = false;

    for (int i = 0; i < 1000; i+=10)
    {
      const Vec2 pt(j, i);
      const Vec2 cpt = cam.ima2cam(pt);
      const Vec2 upt = cam.removeDistortion(cpt);
      const Vec2 distortedPoint = cam.cam2ima(upt);

      line.points.push_back(distortedPoint);
    }

    lines.push_back(line);
  }


  //Calibrate
  calibration::Statistics st;
  std::shared_ptr<camera::Pinhole> estimatedCam = std::make_shared<camera::Pinhole3DEClassicLD>(1000, 1000, 1000, 1000, 0, 0, 0, M_PI_2, 0, 0, 0);

  {
    std::vector<bool> lockedDistortions = {true, true, true, true, true};
    BOOST_CHECK(calibration::estimate(estimatedCam, st, lines, true, true, lockedDistortions));
  }
  {
    std::vector<bool> lockedDistortions = {false, true, true, true, true};
    BOOST_CHECK(calibration::estimate(estimatedCam, st, lines, true, false, lockedDistortions));
  }
  {
    std::vector<bool> lockedDistortions = {false, false, false, false, false};
    BOOST_CHECK(calibration::estimate(estimatedCam, st, lines, true, false, lockedDistortions));
  }

  for (const calibration::PointPair & pair : pts)
  {
    const Vec2 cpt = estimatedCam->ima2cam(pair.distortedPoint);
    const Vec2 upt = estimatedCam->addDistortion(cpt);
    const Vec2 undistortedPoint = estimatedCam->cam2ima(upt);

    const double residual = (undistortedPoint - pair.undistortedPoint).norm();

    BOOST_CHECK_SMALL(residual, 1e-2);
  }
}

//-----------------
// Test summary:
//-----------------
// Generate a set of distorted points on lines
// Estimate the distortion
// Undistort the points
// Checks that these points are close to the original points
// !!!! Note that for convergence reasons, this calibration is reversed (calibrates undistortion)
//-----------------
BOOST_AUTO_TEST_CASE(distortionCalibration_calibrate_lines_radial4)
{
  const camera::Pinhole3DERadial4 cam(1000, 1000, 1000, 1000, 0, 0, -0.4839495643487452, 1.0301284234642258, 0.014928332802185664, -0.0007797104872758904, -0.038994206396183909, 8.0474385001183646e-05);

  //Create points
  std::vector<calibration::PointPair> pts;
  std::vector<calibration::LineWithPoints> lines;
  for (int i = 0; i < 1000; i+=10)
  {
    calibration::LineWithPoints line;
    line.angle = M_PI_4;
    line.dist = 0;
    line.horizontal = true;

    for (int j = 0; j < 1000; j+=10)
    {
      const Vec2 pt(j, i);
      const Vec2 cpt = cam.ima2cam(pt);
      const Vec2 upt = cam.removeDistortion(cpt);
      const Vec2 distortedPoint = cam.cam2ima(upt);

      calibration::PointPair pp;
      pp.distortedPoint = distortedPoint;
      pp.undistortedPoint = pt;
      pts.push_back(pp);

      line.points.push_back(distortedPoint);
    }

    lines.push_back(line);
  }

  for (int j = 0; j < 1000; j+=10)
  {
    calibration::LineWithPoints line;
    line.angle = M_PI_4;
    line.dist = 0;
    line.horizontal = false;

    for (int i = 0; i < 1000; i+=10)
    {
      const Vec2 pt(j, i);
      const Vec2 cpt = cam.ima2cam(pt);
      const Vec2 upt = cam.removeDistortion(cpt);
      const Vec2 distortedPoint = cam.cam2ima(upt);

      line.points.push_back(distortedPoint);
    }

    lines.push_back(line);
  }


  //Calibrate
  calibration::Statistics st;
  std::shared_ptr<camera::Pinhole> estimatedCam = std::make_shared<camera::Pinhole3DERadial4>(1000, 1000, 1000, 1000, 0, 0, 0, 0, 0, 0, 0, 0);

  {
    std::vector<bool> lockedDistortions = {true, true, true, true, true, true};
    BOOST_CHECK(calibration::estimate(estimatedCam, st, lines, true, true, lockedDistortions));
  }
  {
    std::vector<bool> lockedDistortions = {false, true, true, true, true, true};
    BOOST_CHECK(calibration::estimate(estimatedCam, st, lines, true, false, lockedDistortions));
  }
  {
    std::vector<bool> lockedDistortions = {false, false, false, false, false, false};
    BOOST_CHECK(calibration::estimate(estimatedCam, st, lines, true, false, lockedDistortions));
  }

  for (const calibration::PointPair & pair : pts)
  {
    const Vec2 cpt = estimatedCam->ima2cam(pair.distortedPoint);
    const Vec2 upt = estimatedCam->addDistortion(cpt);
    const Vec2 undistortedPoint = estimatedCam->cam2ima(upt);

    const double residual = (undistortedPoint - pair.undistortedPoint).norm();

    BOOST_CHECK_SMALL(residual, 1e-2);
  }
}

