// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/geometry/rigidTransformation3D.hpp>
#include <aliceVision/robustEstimation/ACRansac.hpp>

#include <iostream>

#define BOOST_TEST_MODULE rigidTransformation3D

#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>

using namespace aliceVision;
using namespace aliceVision::geometry;
using namespace std;

BOOST_AUTO_TEST_CASE(SRT_precision_Experiment_ScaleOnly)
{

  const std::size_t nbPoints = 10;
  Mat x1 = Mat::Random(3, nbPoints);
  Mat x2 = x1;

  const double scale = 2;
  Mat3 rot = Mat3::Identity();
  Vec3 t(0, 0, 0);

  for(std::size_t i = 0; i < nbPoints; ++i)
  {
    Vec3 pt = x1.col(i);
    x2.col(i) = (scale * rot * pt + t);
  }

  // Compute the Similarity transform
  double Sc = 0;
  Mat3 Rc;
  Vec3 tc;
  FindRTS(x1, x2, Sc, tc, Rc);
  Refine_RTS(x1, x2, Sc, tc, Rc);

  ALICEVISION_LOG_DEBUG(
          "Scale " << Sc << "\n" <<
          "Rot \n" << Rc << "\n" <<
          "t " << tc.transpose());
}

BOOST_AUTO_TEST_CASE(SRT_precision_Experiment_ScaleAndRot)
{

  const std::size_t nbPoints = 10;
  Mat x1 = Mat::Random(3, nbPoints);
  Mat x2 = x1;

  const double scale = 2;
  Mat3 rot = (Eigen::AngleAxis<double>(.2, Vec3::UnitX())
          * Eigen::AngleAxis<double>(.3, Vec3::UnitY())
          * Eigen::AngleAxis<double>(.6, Vec3::UnitZ())).toRotationMatrix();
  Vec3 t(0, 0, 0);

  for(std::size_t i = 0; i < nbPoints; ++i)
  {
    Vec3 pt = x1.col(i);
    x2.col(i) = (scale * rot * pt + t);
  }

  // Compute the Similarity transform
  double Sc = 0;
  Mat3 Rc;
  Vec3 tc;
  FindRTS(x1, x2, Sc, tc, Rc);
  Refine_RTS(x1, x2, Sc, tc, Rc);

  ALICEVISION_LOG_DEBUG(
          "Scale " << Sc << "\n" <<
          "Rot \n" << Rc << "\n" <<
          "t " << tc.transpose());

  ALICEVISION_LOG_DEBUG("GT\n" <<
          "Scale " << scale << "\n" <<
          "Rot \n" << rot << "\n" <<
          "t " << t.transpose());
}

BOOST_AUTO_TEST_CASE(SRT_precision_Experiment_ScaleRotTranslation)
{

  const std::size_t nbPoints = 10;
  Mat x1 = Mat::Random(3, nbPoints);
  Mat x2 = x1;

  const double scale = 2;
  Mat3 rot = (Eigen::AngleAxis<double>(.2, Vec3::UnitX())
          * Eigen::AngleAxis<double>(.3, Vec3::UnitY())
          * Eigen::AngleAxis<double>(.6, Vec3::UnitZ())).toRotationMatrix();
  Vec3 t(0.5, -0.3, .38);

  for(std::size_t i = 0; i < nbPoints; ++i)
  {
    Vec3 pt = x1.col(i);
    x2.col(i) = (scale * rot * pt + t);
  }

  // Compute the Similarity transform
  double Sc = 0;
  Mat3 Rc;
  Vec3 tc;
  FindRTS(x1, x2, Sc, tc, Rc);
  Refine_RTS(x1, x2, Sc, tc, Rc);

  ALICEVISION_LOG_DEBUG(
          "Scale " << Sc << "\n" <<
          "Rot \n" << Rc << "\n" <<
          "t " << tc.transpose());

  ALICEVISION_LOG_DEBUG("GT\n" <<
          "Scale " << scale << "\n" <<
          "Rot \n" << rot << "\n" <<
          "t " << t.transpose());
}

BOOST_AUTO_TEST_CASE(SRT_precision_ACRANSAC_noNoise)
{
  std::mt19937 randomNumberGenerator;
  const std::size_t nbPoints = 100;
  Mat x1 = Mat::Random(3, nbPoints);
  Mat x2 = x1;

  const double scale = 2;
  Mat3 rot = (Eigen::AngleAxis<double>(.2, Vec3::UnitX())
          * Eigen::AngleAxis<double>(.3, Vec3::UnitY())
          * Eigen::AngleAxis<double>(.6, Vec3::UnitZ())).toRotationMatrix();
  Vec3 t(0.5, -0.3, .38);

  for(std::size_t i = 0; i < nbPoints; ++i)
  {
    const Vec3 &pt = x1.col(i);
    x2.col(i) = (scale * rot * pt + t);
  }

  // Compute the Similarity transform
  double Sc;
  Mat3 Rc;
  Vec3 tc;

  std::vector<std::size_t> inliers;
  const bool result = ACRansac_FindRTS(x1, x2, randomNumberGenerator, Sc, tc, Rc, inliers, true);

  BOOST_CHECK(result);
  BOOST_CHECK(inliers.size() == nbPoints);

  ALICEVISION_LOG_DEBUG(
          "Scale " << Sc << "\n" <<
          "Rot \n" << Rc << "\n" <<
          "t " << tc.transpose());

  ALICEVISION_LOG_DEBUG("GT\n" <<
          "Scale " << scale << "\n" <<
          "Rot \n" << rot << "\n" <<
          "t " << t.transpose());

  BOOST_CHECK_SMALL(scale - Sc, 1e-9);
  
  Mat4 RTS;
  composeRTS(Sc, tc, Rc, RTS);

  robustEstimation::MatrixModel<Mat4> modelRTS(RTS);
  geometry::RTSSquaredResidualError errorEstimator;

  for(std::size_t i = 0; i < nbPoints; ++i)
  {
    const double error = errorEstimator.error(modelRTS, x1.col(i), x2.col(i));
    BOOST_CHECK_SMALL(error, 1e-9);
  }
}

BOOST_AUTO_TEST_CASE(SRT_precision_ACRANSAC_noiseByShuffling)
{
  std::mt19937 randomNumberGenerator;
  // it generates some points x1, it only generates the corresponding 
  // transformed points x2 for nbPoints-nbShuffles of them while the rest
  // are again taken randomly in order to generate outliers
  const std::size_t nbPoints = 100;
  const std::size_t nbNoisy = 50;

  Mat x1 = Mat::Random(3, nbPoints);
  Mat x2 = Mat::Random(3, nbPoints);

  const double scale = 2;
  Mat3 rot = (Eigen::AngleAxis<double>(.2, Vec3::UnitX())
          * Eigen::AngleAxis<double>(.3, Vec3::UnitY())
          * Eigen::AngleAxis<double>(.6, Vec3::UnitZ())).toRotationMatrix();
  Vec3 t(0.5, -0.3, .38);

  for(std::size_t i = 0; i < nbPoints - nbNoisy; ++i)
  {
    const Vec3 &pt = x1.col(i);
    x2.col(i) = (scale * rot * pt + t);
  }

  // Compute the Similarity transform
  double Sc;
  Mat3 Rc;
  Vec3 tc;

  std::vector<std::size_t> inliers;
  const bool result = ACRansac_FindRTS(x1, x2, randomNumberGenerator, Sc, tc, Rc, inliers, true);

  ALICEVISION_LOG_DEBUG(
          "Scale " << Sc << "\n" <<
          "Rot \n" << Rc << "\n" <<
          "t " << tc.transpose());

  ALICEVISION_LOG_DEBUG("GT\n" <<
          "Scale " << scale << "\n" <<
          "Rot \n" << rot << "\n" <<
          "t " << t.transpose());
  
  BOOST_CHECK(result);
  // all the points must be inliers (no noise)
  const std::size_t nbInliers = inliers.size();
  BOOST_CHECK(nbInliers == nbPoints - nbNoisy);

  Mat inliers1 = Mat3X(3, nbInliers);
  Mat inliers2 = Mat3X(3, nbInliers);

  for(std::size_t i = 0; i < nbInliers; ++i)
  {
    inliers1.col(i) = x1.col(inliers[i]);
    inliers2.col(i) = x2.col(inliers[i]);
  }

  // check scale
  BOOST_CHECK_SMALL(scale - Sc, 1e-9);
  
  Mat4 RTS;
  composeRTS(Sc, tc, Rc, RTS);

  robustEstimation::MatrixModel<Mat4> modelRTS(RTS);
  geometry::RTSSquaredResidualError errorEstimator;

  // check the residuals for the inliers
  for(std::size_t i = 0; i < nbInliers; ++i)
  {
    const double error = errorEstimator.error(modelRTS, inliers1.col(i), inliers2.col(i));
    BOOST_CHECK_SMALL(error, 1e-9);
  }
}
