// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "aliceVision/multiview/triangulation/Triangulation.hpp"
#include "aliceVision/multiview/NViewDataSet.hpp"

#define BOOST_TEST_MODULE Triangulation
#include <boost/test/included/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>

#include <vector>

using namespace aliceVision;

BOOST_AUTO_TEST_CASE(Triangulate_NView_FiveViews)
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
      BOOST_CHECK_SMALL(error, 1e-9);
    }
  }
}

BOOST_AUTO_TEST_CASE(Triangulate_NViewAlgebraic_FiveViews) {
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
      BOOST_CHECK_SMALL(error, 1e-9);
    }
  }
}

BOOST_AUTO_TEST_CASE(Triangulate_NViewIterative_FiveViews)
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
