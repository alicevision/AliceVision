
// Copyright (c) 2010 libmv authors.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to
// deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.


// Copyright (c) 2012, 2013 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "openMVG/multiview/triangulation_nview.hpp"
#include "openMVG/multiview/test_data_sets.hpp"
#include "testing/testing.h"

#include <vector>

using namespace openMVG;

TEST(Triangulate_NView, FiveViews)
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
    TriangulateNView(xs, Ps, &X);

    // Check reprojection error. Should be nearly zero.
    for (int j = 0; j < nviews; ++j)
    {
      Vec3 x_reprojected = Ps[j]*X;
      x_reprojected /= x_reprojected(2);
      const double error = (x_reprojected.head(2) - xs.col(j)).norm();
      EXPECT_NEAR(error, 0.0, 1e-9);
    }
  }
}

TEST(Triangulate_NViewAlgebraic, FiveViews) {
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
    TriangulateNViewAlgebraic(xs, Ps, &X);

    // Check reprojection error. Should be nearly zero.
    for (int j = 0; j < nviews; ++j)
    {
      Vec3 x_reprojected = Ps[j]*X;
      x_reprojected /= x_reprojected(2);
      const double error = (x_reprojected.head<2>() - xs.col(j)).norm();
      EXPECT_NEAR(error, 0.0, 1e-9);
    }
  }
}

// test DLT triangulation using weights, it generates some random projection
// matrices, a random 3D points and its corresponding 2d image points. Some of these
// points are considered as outliers. Inliers are assigned a max weight, outliers
// a zero weight. Note: this is just an algebric test, ie points and projection
// matrices have no physical meaning (eg no notion of point in front of the camera
// is considered).
TEST(Triangulate_NViewAlgebraic, WithWeights)
{
  const std::size_t numviews = 20;
  const std::size_t outliers = 8;
  
  // Collect random P matrices together.
  std::vector<Mat34> Ps(numviews);
  for(std::size_t j = 0; j < numviews; ++j)
  {
    Ps[j] = Mat34::Random();
  }
  
  // generate a random 3D point
  Vec4 pt3d(Vec3::Random().homogeneous());
  
  // project the 3D point and prepare weights
  const double w = 1e8;
  std::vector<double> weights(numviews, w);
  Mat2X pt2d(2, numviews);
  for(std::size_t j = 0; j < numviews; ++j)
  {
    if(j < numviews - outliers)
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
  TriangulateNViewAlgebraic(pt2d, Ps, &X, &weights);

  // Check the reprojection error is nearly zero for inliers.
  for (std::size_t j = 0; j < numviews - outliers; ++j)
  {
    const Vec2 x_reprojected = (Ps[j] * X).hnormalized();
    const double error = (x_reprojected - pt2d.col(j)).norm();
    EXPECT_NEAR(error, 0.0, 1e-9);
  }
}

TEST(Triangulate_NViewIterative, FiveViews)
{
  const int nviews = 5;
  const int npoints = 6;
  const NViewDataSet d = NRealisticCamerasRing(nviews, npoints);

  for (int i = 0; i < npoints; ++i)
  {

    Triangulation triangulationObj;
    for (int j = 0; j < nviews; ++j)
      triangulationObj.add(d.P(j), d._x[j].col(i));

    const Vec3 X = triangulationObj.compute();
    // Check reprojection error. Should be nearly zero.
    EXPECT_NEAR(triangulationObj.error(X), 0.0, 1e-9);
    for (int j = 0; j < nviews; ++j)
    {
      Vec3 x_reprojected = d.P(j) * Vec4(X(0), X(1), X(2), 1.0);
      x_reprojected /= x_reprojected(2);
      const double error = (x_reprojected.head<2>() - d._x[j].col(i)).norm();
      EXPECT_NEAR(error, 0.0, 1e-9);
    }
  }
}


/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
