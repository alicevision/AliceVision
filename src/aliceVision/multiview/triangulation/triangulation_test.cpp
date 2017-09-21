// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#include "aliceVision/multiview/triangulation/Triangulation.hpp"
#include "aliceVision/multiview/NViewDataSet.hpp"
#include "testing/testing.h"

#include <vector>

using namespace aliceVision;

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
