// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "aliceVision/geometry/HalfPlane.hpp"
#include "aliceVision/geometry/Frustum.hpp"

#include "aliceVision/multiview/NViewDataSet.hpp"
#include "aliceVision/multiview/projection.hpp"

#include <iostream>

#define BOOST_TEST_MODULE frustumIntersection
#include <boost/test/included/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>

using namespace aliceVision;
using namespace aliceVision::geometry;
using namespace aliceVision::geometry::halfPlane;
using namespace std;

//--
// Camera frustum intersection unit test
//--

BOOST_AUTO_TEST_CASE(intersection)
{
  const int focal = 1000;
  const int principal_Point = 500;
  //-- Setup a circular camera rig or "cardioid".
  const int iNviews = 4;
  const int iNbPoints = 6;
  const NViewDataSet d =
    NRealisticCamerasRing(
    iNviews, iNbPoints,
    NViewDatasetConfigurator(focal, focal, principal_Point, principal_Point, 5, 0));

  // Test with infinite Frustum for each camera
  {
    std::vector<Frustum> vec_frustum;
    for (int i=0; i < iNviews; ++i)
    {
      vec_frustum.push_back(
        Frustum(principal_Point*2, principal_Point*2, d._K[i], d._R[i], d._C[i]));
      BOOST_CHECK(vec_frustum[i].isInfinite());
    }

    // Check that frustums have an overlap
    for (int i = 0; i < iNviews; ++i)
      for (int j = 0; j < iNviews; ++j)
        BOOST_CHECK(vec_frustum[i].intersect(vec_frustum[j]));
  }

  // Test with truncated frustum
  {
    // Build frustum with near and far plane defined by min/max depth per camera
    std::vector<Frustum> vec_frustum;
    for (int i=0; i < iNviews; ++i)
    {
      double minDepth = std::numeric_limits<double>::max();
      double maxDepth = std::numeric_limits<double>::min();
      for (int j=0; j < iNbPoints; ++j)
      {
        const double depth = Depth(d._R[i], d._t[i], d._X.col(j));
        if (depth < minDepth)
          minDepth = depth;
        if (depth > maxDepth)
          maxDepth = depth;
      }
      vec_frustum.push_back(
        Frustum(principal_Point*2, principal_Point*2,
          d._K[i], d._R[i], d._C[i], minDepth, maxDepth));
      BOOST_CHECK(vec_frustum[i].isTruncated());
    }

    // Check that frustums have an overlap
    for (int i = 0; i < iNviews; ++i)
      for (int j = 0; j < iNviews; ++j)
        BOOST_CHECK(vec_frustum[i].intersect(vec_frustum[j]));
  }
}

BOOST_AUTO_TEST_CASE(empty_intersection)
{
  // Create infinite frustum that do not share any space
  //--
  // 4 cameras on a circle that look to the same direction
  // Apply a 180° rotation to the rotation matrix in order to make the cameras
  //  don't share any visual hull
  //--

  const int focal = 1000;
  const int principal_Point = 500;
  const int iNviews = 4;
  const int iNbPoints = 6;
  const NViewDataSet d =
    NRealisticCamerasRing(
    iNviews, iNbPoints,
    NViewDatasetConfigurator(focal, focal, principal_Point, principal_Point, 5, 0));

  // Test with infinite Frustum for each camera
  {
    std::vector<Frustum> vec_frustum;
    for (int i=0; i < iNviews; ++i)
    {
      const Mat3 flipMatrix = RotationAroundY(degreeToRadian(180.0));
      vec_frustum.push_back(
        Frustum(principal_Point*2, principal_Point*2, d._K[i], d._R[i]*flipMatrix, d._C[i]));
      BOOST_CHECK(vec_frustum[i].isInfinite());
    }

    // Test if the frustum have an overlap
    for (int i=0; i < iNviews; ++i)
    {
      for (int j=0; j < iNviews; ++j)
      {
        if (i == j) // Same frustum (intersection must exist)
        {
          BOOST_CHECK(vec_frustum[i].intersect(vec_frustum[j]));
        }
        else // different frustum
        {
          BOOST_CHECK(!vec_frustum[i].intersect(vec_frustum[j]));
        }
      }
    }
  }

  // Test but with truncated frustum
  {
    // Build frustum with near and far plane defined by min/max depth per camera
    std::vector<Frustum> vec_frustum;
    for (int i=0; i < iNviews; ++i)
    {
      double minDepth = std::numeric_limits<double>::max();
      double maxDepth = std::numeric_limits<double>::min();
      for (int j=0; j < iNbPoints; ++j)
      {
        const double depth = Depth(d._R[i], d._t[i], d._X.col(j));
        if (depth < minDepth)
          minDepth = depth;
        if (depth > maxDepth)
          maxDepth = depth;
      }
      const Mat3 flipMatrix = RotationAroundY(degreeToRadian(180.0));
      vec_frustum.push_back(
        Frustum(principal_Point*2, principal_Point*2,
          d._K[i], d._R[i]*flipMatrix, d._C[i], minDepth, maxDepth));
      BOOST_CHECK(vec_frustum[i].isTruncated());
    }

    // Test if the frustum have an overlap
    for (int i=0; i < iNviews; ++i)
    {
      for (int j=0; j < iNviews; ++j)
      {
        if (i == j) // Same frustum (intersection must exist)
        {
          BOOST_CHECK(vec_frustum[i].intersect(vec_frustum[j]));
        }
        else // different frustum
        {
          BOOST_CHECK(!vec_frustum[i].intersect(vec_frustum[j]));
        }
      }
    }
  }
}
