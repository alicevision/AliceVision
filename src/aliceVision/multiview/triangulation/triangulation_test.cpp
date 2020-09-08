// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// Copyright (c) 2010 libmv contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/multiview/triangulation/Triangulation.hpp>
#include <aliceVision/multiview/NViewDataSet.hpp>

#define BOOST_TEST_MODULE Triangulation

#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>

#include <vector>

using namespace aliceVision;

BOOST_AUTO_TEST_CASE(TriangulateNView_FiveViews)
{
  const int nviews = 5;
  const int npoints = 6;
  const NViewDataSet d = NRealisticCamerasRing(nviews, npoints);

  // Collect P matrices together.
  std::vector<Mat34> Ps(nviews);
  for (int j = 0; j < nviews; ++j)
  {
    Ps[j] = d.P(j);
  }

  for (int i = 0; i < npoints; ++i)
  {
    // Collect the image of point i in each frame.
    Mat2X xs(2, nviews);
    for (int j = 0; j < nviews; ++j)
    {
      xs.col(j) = d._x[j].col(i);
    }
    Vec4 X;
    multiview::TriangulateNView(xs, Ps, &X);

    // Check reprojection error. Should be nearly zero.
    for (int j = 0; j < nviews; ++j)
    {
      Vec3 x_reprojected = Ps[j]*X;
      x_reprojected /= x_reprojected(2);
      const double error = (x_reprojected.head(2) - xs.col(j)).norm();
      BOOST_CHECK_SMALL(error, 1e-9);
    }
  }
}

BOOST_AUTO_TEST_CASE(TriangulateNViewAlgebraic_FiveViews) {
  const int nviews = 5;
  const int npoints = 6;
  const NViewDataSet d = NRealisticCamerasRing(nviews, npoints);

  // Collect P matrices together.
  std::vector<Mat34> Ps(nviews);
  for (int j = 0; j < nviews; ++j)
  {
    Ps[j] = d.P(j);
  }

  for (int i = 0; i < npoints; ++i)
  {
    // Collect the image of point i in each frame.
    Mat2X xs(2, nviews);
    for (int j = 0; j < nviews; ++j)
    {
      xs.col(j) = d._x[j].col(i);
    }
    Vec4 X;
    multiview::TriangulateNViewAlgebraic(xs, Ps, &X);

    // Check reprojection error. Should be nearly zero.
    for (int j = 0; j < nviews; ++j)
    {
      Vec3 x_reprojected = Ps[j]*X;
      x_reprojected /= x_reprojected(2);
      const double error = (x_reprojected.head<2>() - xs.col(j)).norm();
      BOOST_CHECK_SMALL(error, 1e-9);
    }
  }
}

// test DLT triangulation using weights, it generates some random projection
// matrices, a random 3D points and its corresponding 2d image points. Some of these
// points are considered as outliers. Inliers are assigned a max weight, outliers
// a zero weight. Note: this is just an algebric test, ie points and projection
// matrices have no physical meaning (eg no notion of point in front of the camera
// is considered).
BOOST_AUTO_TEST_CASE(Triangulate_NViewAlgebraic_WithWeights)
{
  const std::size_t nbViews = 20;
  const std::size_t nbOutliers = 8;
  
  // Collect random P matrices together.
  std::vector<Mat34> Ps(nbViews);
  for(std::size_t j = 0; j < nbViews; ++j)
  {
    Ps[j] = Mat34::Random();
  }
  
  // generate a random 3D point
  Vec4 pt3d(Vec3::Random().homogeneous());
  
  // project the 3D point and prepare weights
  const double w = 1e8;
  std::vector<double> weights(nbViews, w);
  Mat2X pt2d(2, nbViews);
  for(std::size_t j = 0; j < nbViews; ++j)
  {
    if(j < nbViews - nbOutliers)
    {
      // project the 3D point
      pt2d.col(j) = (Ps[j] * pt3d).hnormalized();
    }
    else
    {
      // for the outliers just set them to some random value
      pt2d.col(j) = Vec2::Random();
      // set the weight to 0 for the outliers
      weights[j] = 0.0;
    }
  }

  Vec4 X;
  multiview::TriangulateNViewAlgebraic(pt2d, Ps, &X, &weights);

  // Check the reprojection error is nearly zero for inliers.
  for (std::size_t j = 0; j < nbViews - nbOutliers; ++j)
  {
    const Vec2 x_reprojected = (Ps[j] * X).hnormalized();
    const double error = (x_reprojected - pt2d.col(j)).norm();
    BOOST_CHECK_SMALL(error, 1e-9);
  }
}

BOOST_AUTO_TEST_CASE(Triangulate_NViewIterative_FiveViews)
{
  const int nviews = 5;
  const int npoints = 6;
  const NViewDataSet d = NRealisticCamerasRing(nviews, npoints);

  for(int i = 0; i < npoints; ++i)
  {

    multiview::Triangulation triangulationObj;
    for (int j = 0; j < nviews; ++j)
      triangulationObj.add(d.P(j), d._x[j].col(i));

    const Vec3 X = triangulationObj.compute();
    // Check reprojection error. Should be nearly zero.
    BOOST_CHECK_SMALL(triangulationObj.error(X), 1e-9);
    for (int j = 0; j < nviews; ++j)
    {
      Vec3 x_reprojected = d.P(j) * Vec4(X(0), X(1), X(2), 1.0);
      x_reprojected /= x_reprojected(2);
      const double error = (x_reprojected.head<2>() - d._x[j].col(i)).norm();
      BOOST_CHECK_SMALL(error, 1e-9);
    }
  }
}

//// Test triangulation as algebric problem, it generates some random projection
//// matrices, a random 3D points and its corresponding 2d image points. Some of these
//// points are considered as outliers. Inliers are assigned a max weight, outliers
//// a zero weight. Note: this is just an algebric test, ie points and projection
//// matrices have no physical meaning (eg no notion of point in front of the camera
//// is considered).
BOOST_AUTO_TEST_CASE(Triangulate_NViewIterative_LORANSAC)
{
  std::mt19937 randomNumberGenerator;
  
  const std::size_t numTrials = 100;
  for(std::size_t trial = 0; trial < numTrials; ++ trial)
  {
    const std::size_t nbViews = 20;
    const std::size_t nbOutliers = 8;
    const std::size_t nbInliers = nbViews - nbOutliers;

    // Collect random P matrices together.
    std::vector<Mat34> Ps(nbViews);
    for(std::size_t j = 0; j < nbViews; ++j)
    {
      Ps[j] = Mat34::Random();
    }

    // generate a random 3D point
    Vec4 pt3d(Vec3::Random().homogeneous());

    // project the 3D point and prepare weights
    Mat2X pt2d(2, nbViews);
    for(std::size_t j = 0; j < nbViews; ++j)
    {
      if(j < nbViews - nbOutliers)
      {
        // project the 3D point
        pt2d.col(j) = (Ps[j] * pt3d).hnormalized();
      }
      else
      {
        // for the outliers just set them to some random value
        pt2d.col(j) = Vec2::Random();
        // set the weight to 0 for the outliers
      }
    }

    std::vector<std::size_t> inliers;
    Vec4 X;
    double const threshold = 0.01; // modify the default value: 4 pixels is too much in this configuration.
    multiview::TriangulateNViewLORANSAC(pt2d, Ps, randomNumberGenerator, &X, &inliers, threshold);
    
    // check inliers are correct
    BOOST_CHECK_EQUAL(inliers.size(), nbInliers);

    // Check the reprojection error is nearly zero for inliers.
    for (std::size_t j = 0; j < nbInliers; ++j)
    {
      const Vec2 x_reprojected = (Ps[j] * X).hnormalized();
      const double error = (x_reprojected - pt2d.col(j)).norm();
//      EXPECT_NEAR(error, 0.0, 1e-4);
      BOOST_CHECK_SMALL(error, 1e-5);
    }
  }
}

