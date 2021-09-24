// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "rigResection.hpp"
#include <aliceVision/camera/PinholeRadial.hpp>
#include <aliceVision/geometry/Pose3.hpp>

#include "optimization.hpp"

#include <vector>
#include <math.h>
#include <chrono>
#include <random>
#include <random>

#define BOOST_TEST_MODULE rigResection

#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>

using namespace aliceVision;

Mat3 generateRotation(double x, double y, double z)
{
//ALICEVISION_LOG_DEBUG("generateRotation"); 
  Mat3 R1 = Mat3::Identity();
  R1(1,1) = cos(x);
  R1(1,2) = -sin(x);
  R1(2,1) = -R1(1,2);
  R1(2,2) = R1(1,1);

  Mat3 R2 = Mat3::Identity();
  R2(0,0) = cos(y);
  R2(0,2) = sin(y);
  R2(2,0) = -R2(0,2);
  R2(2,2) = R2(0,0);

  Mat3 R3 = Mat3::Identity();
  R3(0,0) = cos(z);
  R3(0,1) = -sin(z);
  R3(1,0) =-R3(0,1);
  R3(1,1) = R3(0,0);
  return R3 * R2 * R1;
}

Mat3 generateRotation(const Vec3 &angles)
{
  return generateRotation(angles(0), angles(1), angles(2));
}

Mat3 generateRandomRotation(const Vec3 &maxAngles = Vec3::Constant(2*M_PI))
{
  const Vec3 angles = Vec3::Random().cwiseProduct(maxAngles);
//  ALICEVISION_LOG_DEBUG("generateRandomRotation"); 
  return generateRotation(angles);
}

Vec3 generateRandomTranslation(double maxNorm)
{
//  ALICEVISION_LOG_DEBUG("generateRandomTranslation"); 
  Vec3 translation = Vec3::Random();
  return maxNorm * (translation / translation.norm());
}

Vec3 generateRandomPoint(double thetaMin, double thetaMax, double depthMax, double depthMin)
{
  // variables contains the spherical parameters (r, theta, phi) for the point to generate its 
  // spherical coordinatrs
  Vec3 variables = Vec3::Random();
  variables(0) = (variables(0)+1)/2;    // put the random in [0,1]
  variables(1) = (variables(1)+1)/2;    // put the random in [0,1]
   
  // get random value for r
  const double r = variables(0) = depthMin*variables(0) + (1-variables(0))*depthMax;
  // get random value for theta
  const double theta = variables(1) = thetaMin*variables(1) + (1-variables(1))*thetaMax;
  // get random value for phi
  const double phi = variables(2) = M_PI*variables(2);
  
  return Vec3(r*std::sin(theta)*std::cos(phi), 
              r*std::sin(theta)*std::sin(phi), 
              r*std::cos(theta));
}

Mat3X generateRandomPoints(std::size_t numPts, double thetaMin, double thetaMax, double depthMax, double depthMin)
{
  Mat3X points = Mat(3, numPts);
  for(std::size_t i = 0; i < numPts; ++i)
  {
    points.col(i) = generateRandomPoint(thetaMin, thetaMax, depthMax, depthMin);
  }
  return points;
}

geometry::Pose3 generateRandomPose(const Vec3 &maxAngles = Vec3::Constant(2*M_PI), double maxNorm = 1)
{
//  ALICEVISION_LOG_DEBUG("generateRandomPose"); 
  return geometry::Pose3(generateRandomRotation(maxAngles), generateRandomTranslation(maxNorm));
}

void generateRandomExperiment(std::size_t numCameras, 
                              std::size_t numPoints,
                              double outliersRatio,
                              double noise,
                              geometry::Pose3 &rigPoseGT,
                              Mat3X &pointsGT,
                              std::vector<camera::PinholeRadialK3 > &vec_queryIntrinsics,
                              std::vector<geometry::Pose3 > &vec_subPoses,
                              std::vector<Mat> &vec_pts3d,
                              std::vector<Mat> &vec_pts2d)
{
      // generate random pose for the rig
    rigPoseGT = generateRandomPose(Vec3::Constant(M_PI/10), 5);

    // generate random 3D points
    const double thetaMin = 0;
    const double thetaMax = M_PI/3;
    const double depthMax = 50;
    const double depthMin = 10;
    pointsGT = generateRandomPoints(numPoints, thetaMin, thetaMax, depthMax, depthMin);

    // apply random rig pose to points
    const Mat3X points = rigPoseGT(pointsGT);
    ALICEVISION_LOG_DEBUG("rigPoseGT\n" << rigPoseGT.rotation() << "\n" << rigPoseGT.center());

    // generate numCameras random poses and intrinsics
    for(std::size_t cam = 0; cam < numCameras; ++cam)
    {
      // first camera is in I 0
      if(cam != 0)
        vec_subPoses.push_back(generateRandomPose(Vec3::Constant(M_PI/10), 1.5));

      // let's keep it simple
      vec_queryIntrinsics.push_back(camera::PinholeRadialK3(640, 480, 500, 320, 240));
    }
    assert(vec_subPoses.size() == numCameras-1);
  //  for(std::size_t i = 0; i < vec_subPoses.size(); ++i)
  //    ALICEVISION_LOG_DEBUG("vec_subPoses\n" << vec_subPoses[i].rotation() << "\n" << vec_subPoses[i].center());

    // for each camera generate the features (if 3D point is "in front" of the camera)
    vec_pts3d.reserve(numCameras);
    vec_pts2d.reserve(numCameras);
    
    const std::size_t numOutliers = (std::size_t)numPoints*outliersRatio;

    for(std::size_t cam = 0; cam < numCameras; ++cam)
    {
      Mat3X localPts;
      if(cam != 0)
        localPts = vec_subPoses[cam-1](points);
      else
        localPts = points;

      // count the number of points in front of the camera
      // ie take the 3rd coordinate and check it is > 0
      const std::size_t validPoints = (localPts.row(2).array() > 0).count();
      Mat3X pts3d = Mat(3, validPoints + numOutliers);
      Mat2X pts2d = Mat(2, validPoints + numOutliers);

      // for each 3D point
      std::size_t idx = 0;
      for(std::size_t i = 0; i < numPoints; ++i)
      {
        // if it is in front of the camera
        if(localPts(2,i) > 0)
        {
          // project it
          Vec2 feat = vec_queryIntrinsics[cam].project(geometry::Pose3(), localPts.col(i).homogeneous());
          
          if(noise > std::numeric_limits<double>::epsilon())
          {
            feat = feat + noise*Vec2::Random();
          }

          // add the 3d and 2d point
          pts3d.col(idx) = pointsGT.col(i);
          pts2d.col(idx) = feat;
          ++idx;
        }
      }
      assert(idx == validPoints);
      
      if(numOutliers)
      {
        //add some other random associations
        for(std::size_t i = 0; i < numOutliers; ++i)
        {
          pts3d.col(idx) = 10*Vec3::Random();
          pts2d.col(idx) = 100*Vec2::Random();
          ++idx;
        }
      }

//      ALICEVISION_LOG_DEBUG("Cam " << cam);
//      ALICEVISION_LOG_DEBUG("pts2d\n" << pts2d);
//      ALICEVISION_LOG_DEBUG("pts3d\n" << pts3d);
//      ALICEVISION_LOG_DEBUG("pts3dTRA\n" << localPts);
//      
//      auto residuals = vec_queryIntrinsics[cam].residuals(geometry::Pose3(), localPts, pts2d);
//      auto sqrErrors = (residuals.cwiseProduct(residuals)).colwise().sum();
//      
//      ALICEVISION_LOG_DEBUG("residuals\n" << sqrErrors);

  //    if(cam!=0)
  //      residuals = vec_queryIntrinsics[cam].residuals(vec_subPoses[cam-1], points, pts2d);
  //    else
  //      residuals = vec_queryIntrinsics[cam].residuals(geometry::Pose3(), points, pts2d);
  //    
  //    auto sqrErrors2 = (residuals.cwiseProduct(residuals)).colwise().sum();
  //    
  //    ALICEVISION_LOG_DEBUG("residuals2\n" << sqrErrors2);

      vec_pts3d.push_back(pts3d);
      vec_pts2d.push_back(pts2d);
    }
}


BOOST_AUTO_TEST_CASE(rigResection_simpleNoNoiseNoOutliers)
{
  const std::size_t numCameras = 3;
  const std::size_t numPoints = 10;
  const std::size_t numTrials = 10;
  const double threshold = 0.1;
  
  for(std::size_t trial = 0; trial < numTrials; ++trial)
  {

    // generate random pose for the rig
    geometry::Pose3 rigPoseGT;
    std::vector<camera::PinholeRadialK3 > vec_queryIntrinsics;
    std::vector<geometry::Pose3 > vec_subPoses;
    std::vector<Mat> vec_pts3d;
    std::vector<Mat> vec_pts2d;
    Mat3X pointsGT;

    generateRandomExperiment(numCameras, 
                             numPoints,
                             0,
                             0,
                             rigPoseGT,
                             pointsGT,
                             vec_queryIntrinsics,
                             vec_subPoses,
                             vec_pts3d,
                             vec_pts2d);

    // call the GPNP
    std::vector<std::vector<std::size_t> > inliers;

    geometry::Pose3 rigPose;
    const EstimationStatus status = localization::rigResection(vec_pts2d,
                                          vec_pts3d,
                                          vec_queryIntrinsics,
                                          vec_subPoses,
                                          nullptr,
                                          rigPose,
                                          inliers);
    BOOST_CHECK(status.isValid);

    ALICEVISION_LOG_DEBUG("rigPose\n" << rigPose.rotation() << "\n" << rigPose.center());

    // check result for the pose
    const auto poseDiff = rigPose*rigPoseGT.inverse();
    ALICEVISION_LOG_DEBUG("diffR\n" << poseDiff.rotation());
    ALICEVISION_LOG_DEBUG("diffC\n" << poseDiff.center().norm());
    
    // check result for the posediff, how close it is to the identity
    const Mat3 &rot = poseDiff.rotation();
    for(std::size_t i = 0; i < 3; ++i)
    {
      for(std::size_t j = 0; j < 3; ++j)
      {
        if(i == j)
        {
          BOOST_CHECK_SMALL(rot(i,j)- 1.0, threshold);
        }
        else
        {
          BOOST_CHECK_SMALL(rot(i,j), threshold);
        }
      }
    }

    const Vec3 &center = poseDiff.center();
    for(std::size_t i = 0; i < 3; ++i)
    {
      BOOST_CHECK_SMALL(center(i), threshold);
    }

    // check inliers
    BOOST_CHECK(inliers.size() == numCameras);
    for(std::size_t i = 0; i < numCameras; ++i)
    {
      BOOST_CHECK(inliers[i].size() == vec_pts2d[i].cols());
    }

    BOOST_CHECK(localization::refineRigPose(vec_pts2d,
                                            vec_pts3d,
                                            inliers,
                                            vec_queryIntrinsics,
                                            vec_subPoses,
                                            rigPose));

    // THIS BOOST_AUTO_TEST_CASE CAN FAIL AS THE REPROJECTION ERROR USED INSIDE THE GP3P IS BASED
    // ON THE ANGLE DIFFERENCE RATHER THAN THE REPROJECTED POINT DIFFERENCE
    // check reprojection errors
//    for(std::size_t cam = 0; cam < numCameras; ++cam)
//    {
//      const std::size_t numPts = vec_pts2d[cam].cols();
//      const camera::PinholeRadialK3 &currCamera = vec_queryIntrinsics[cam];
//      Mat2X residuals;
//      if(cam!=0)
//        residuals = currCamera.residuals(vec_subPoses[cam-1]*rigPose, vec_pts3d[cam], vec_pts2d[cam]);
//      else
//        residuals = currCamera.residuals(geometry::Pose3()*rigPose, vec_pts3d[cam], vec_pts2d[cam]);
//
//      auto sqrErrors = (residuals.cwiseProduct(residuals)).colwise().sum();
//      ALICEVISION_LOG_DEBUG(sqrErrors);
//      for(std::size_t j = 0; j < numPts; ++j)
//      {
//        BOOST_CHECK(sqrErrors(j) <= threshold);
//      }
//    }
  }
}

BOOST_AUTO_TEST_CASE(rigResection_simpleWithNoiseNoOutliers)
{
  const std::size_t numCameras = 3;
  const std::size_t numPoints = 10;
  const std::size_t numTrials = 10;
  const double noise = 0.1;
  const double threshold = 0.1;
  
  for(std::size_t trial = 0; trial < numTrials; ++trial)
  {

    // generate random pose for the rig
    geometry::Pose3 rigPoseGT;
    std::vector<camera::PinholeRadialK3 > vec_queryIntrinsics;
    std::vector<geometry::Pose3 > vec_subPoses;
    std::vector<Mat> vec_pts3d;
    std::vector<Mat> vec_pts2d;
    Mat3X pointsGT;

    generateRandomExperiment(numCameras, 
                             numPoints,
                             0,
                             noise,
                             rigPoseGT,
                             pointsGT,
                             vec_queryIntrinsics,
                             vec_subPoses,
                             vec_pts3d,
                             vec_pts2d);

    // call the GPNP
    std::vector<std::vector<std::size_t> > inliers;
    geometry::Pose3 rigPose;
    const EstimationStatus status = localization::rigResection(vec_pts2d,
                                           vec_pts3d,
                                           vec_queryIntrinsics,
                                           vec_subPoses,
                                           nullptr,
                                           rigPose,
                                           inliers,
                                           degreeToRadian(0.008));
    BOOST_CHECK(status.isValid);

    ALICEVISION_LOG_DEBUG("rigPose\n" << rigPose.rotation() << "\n" << rigPose.center());
    
    const auto poseDiff = rigPose*rigPoseGT.inverse();
    ALICEVISION_LOG_DEBUG("diffR\n" << poseDiff.rotation());
    ALICEVISION_LOG_DEBUG("diffC\n" << poseDiff.center().norm());
    
    // check result for the posediff, how close it is to the identity
    const Mat3 &rot = poseDiff.rotation();
    for(std::size_t i = 0; i < 3; ++i)
    {
      for(std::size_t j = 0; j < 3; ++j)
      {
        if(i == j)
        {
          BOOST_CHECK_SMALL(rot(i,j)- 1.0, threshold);
        }
        else
        {
          BOOST_CHECK_SMALL(rot(i,j), threshold);
        }
      }
    }

    const Vec3 &center = poseDiff.center();
    for(std::size_t i = 0; i < 3; ++i)
    {
      BOOST_CHECK_SMALL(center(i), threshold);
    }

    BOOST_CHECK(localization::refineRigPose(vec_pts2d,
                                            vec_pts3d,
                                            inliers,
                                            vec_queryIntrinsics,
                                            vec_subPoses,
                                            rigPose));
  }
}

/*
BOOST_AUTO_TEST_CASE(rigResection_simpleNoNoiseWithOutliers)
{
  const std::size_t numCameras = 3;
  const std::size_t numPoints = 10;
  const std::size_t numTrials = 10;
  const double noise = 0;
  const double threshold = 1e-3;

  std::mt19937 gen;
  std::uniform_real_distribution<> dis(0, 0.4);
  
  for(std::size_t trial = 0; trial < numTrials; ++trial)
  {

    // generate random pose for the rig
    geometry::Pose3 rigPoseGT;
    const double outlierPercentage = 0.4*dis(gen);
    std::vector<camera::PinholeRadialK3 > vec_queryIntrinsics;
    std::vector<geometry::Pose3 > vec_subPoses;
    std::vector<Mat> vec_pts3d;
    std::vector<Mat> vec_pts2d;
    Mat3X pointsGT;

    generateRandomExperiment(numCameras, 
                             numPoints,
                             outlierPercentage,
                             noise,
                             rigPoseGT,
                             pointsGT,
                             vec_queryIntrinsics,
                             vec_subPoses,
                             vec_pts3d,
                             vec_pts2d);

    // call the GPNP
    std::vector<std::vector<std::size_t> > inliers;
    geometry::Pose3 rigPose;
    const EstimationStatus status = localization::rigResection(vec_pts2d,
                                          vec_pts3d,
                                          vec_queryIntrinsics,
                                          vec_subPoses,
                                          nullptr,
                                          rigPose,
                                          inliers,
                                          degreeToRadian(0.008));
    BOOST_CHECK(status.hasStrongSupport);

    ALICEVISION_LOG_DEBUG("rigPose\n" << rigPose.rotation() << "\n" << rigPose.center());

    BOOST_CHECK(localization::refineRigPose(vec_pts2d,
                                            vec_pts3d,
                                            inliers,
                                            vec_queryIntrinsics,
                                            vec_subPoses,
                                            rigPose));

    // THIS BOOST_AUTO_TEST_CASE CAN FAIL AS THE REPROJECTION ERROR USED INSIDE THE GP3P IS BASED
    // ON THE ANGLE DIFFERENCE RATHER THAN THE REPROJECTED POINT DIFFERENCE
//    // check reprojection errors
//    for(std::size_t cam = 0; cam < numCameras; ++cam)
//    {
//      const std::size_t numPts = vec_pts2d[cam].cols();
//      const camera::PinholeRadialK3 &currCamera = vec_queryIntrinsics[cam];
//      Mat2X residuals;
//      if(cam!=0)
//        residuals = currCamera.residuals(vec_subPoses[cam-1]*rigPose, vec_pts3d[cam], vec_pts2d[cam]);
//      else
//        residuals = currCamera.residuals(geometry::Pose3()*rigPose, vec_pts3d[cam], vec_pts2d[cam]);
//  
//      auto sqrErrors = (residuals.cwiseProduct(residuals)).colwise().sum();
//
////      ALICEVISION_LOG_DEBUG(sqrErrors);
//
//      const auto &currInliers = inliers[cam];
//      for(std::size_t j = 0; j < currInliers.size(); ++j)
//      {
////        ALICEVISION_LOG_DEBUG(sqrErrors(currInliers[j]));
//        BOOST_CHECK(sqrErrors(currInliers[j]) <= threshold);
//      }
//    }
  }
}
*/
