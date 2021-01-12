// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "aliceVision/geometry/HalfPlane.hpp"
#include "aliceVision/geometry/Frustum.hpp"

#include "aliceVision/multiview/NViewDataSet.hpp"
#include "aliceVision/numeric/projection.hpp"

#include <iostream>

#define BOOST_TEST_MODULE frustumIntersection

#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>

using namespace aliceVision;
using namespace aliceVision::geometry;
using namespace aliceVision::geometry::halfPlane;


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

  //Test with partially truncated frustum
  {
    //Build frustum with near and far plane defined by a different value
    std::vector<Frustum> vec_frustum;
    for (int i=0; i < iNviews; ++i)
    {
      vec_frustum.push_back(
        Frustum(principal_Point*2, principal_Point*2,
                d._K[i], d._R[i], d._C[i], 0.1, -1.0));
      BOOST_CHECK(vec_frustum[i].isPartiallyTruncated());
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

void createPanoramaScene(double coeff, NViewDataSet& d, const int iNviews, const int iNbPoints, const int principal_Point)
{
    // Create a panorama scene
    //--
    // Create a panorama scene with a field of view
    //  depending on a coefficient and the number of views
    // 1 camera looks to iNviews different directions
    //--

    double field_of_view = (360.0*coeff)/iNviews;
    double focalRatio = 0.5/(tan(0.5*degreeToRadian(field_of_view)));
    double focalLengthPix=focalRatio*2*principal_Point;
    NViewDatasetConfigurator config(focalLengthPix, focalLengthPix, principal_Point, principal_Point, 1, 0);
    d._n = iNviews;
    d._K.resize(iNviews);
    d._R.resize(iNviews);
    d._t.resize(iNviews);
    d._C.resize(iNviews);

    for (size_t i = 0; i < iNviews; ++i)
    {
      Vec3 camera_center(0., 0., 0.);
      const double theta = i * 2 * M_PI / iNviews;
      d._C[i] = camera_center;
      // Circle
      Vec3 lookdir(sin(theta), 0.0, cos(theta)); // Y axis UP

      d._K[i] << config._fx,           0, config._cx,
                          0,  config._fy, config._cy,
                          0,           0,          1;
      d._R[i] = LookAt(lookdir);  // Y axis UP
      d._t[i] = -d._R[i] * camera_center; // [t]=[-RC] Cf HZ.
    }

}

BOOST_AUTO_TEST_CASE(panorama_intersection)
{
  // Create partially truncated frustum
  //--
  // 1 camera looks to 4 different directions on a circle
  // Create a panorama scene with a field of view
  //  more than 90° for each view which means no overlap
  //--

  const int principal_Point = 500;
  // Setup a panorama camera rig: cameras rotations around a nodal point
  const int iNviews = 4;
  const int iNbPoints = 6;
  NViewDataSet d;
  double coeff = 1.2; // overlap coefficient: more than 1 means overlap
  // create panorama scene
  createPanoramaScene(coeff, d, iNviews, iNbPoints, principal_Point);

  //Test with partially truncated frustum
  {
    //Build frustum with near and far plane defined by a different value
    std::vector<Frustum> vec_frustum;
    for (int i=0; i < iNviews; ++i)
    {
      vec_frustum.push_back(
        Frustum(principal_Point*2, principal_Point*2,
                d._K[i], d._R[i], d._t[i], 0.1, -1.0));
      BOOST_CHECK(vec_frustum[i].isPartiallyTruncated());
    }

    //Check that there is overlap between all frustums
    for (int i = 0; i < iNviews; ++i)
    {
      int j = (i+1) % iNviews;
      BOOST_CHECK(vec_frustum[i].intersect(vec_frustum[j]));

      int k = (i-1+iNviews) % iNviews;
      BOOST_CHECK(vec_frustum[i].intersect(vec_frustum[k]));
    }
  }
}

BOOST_AUTO_TEST_CASE(panorama_without_intersection)
{
  // Create partially truncated frustum
  //--
  // 1 camera looks to 4 different directions on a circle
  // Create a panorama scene with a field of view
  //  less than 90° for each view which means no overlap
  //--

  const int principal_Point = 500;
  // Setup a panorama camera rig: cameras rotations around a nodal point
  const int iNviews = 4;
  const int iNbPoints = 6;
  NViewDataSet d;
  double coeff = 0.8;  // overlap coefficient: less than 1 means no overlap

  //create panorama scene
  createPanoramaScene(coeff, d, iNviews, iNbPoints, principal_Point);

  //Test with partially truncated frustum
  {
    //Build frustum with near and far plane defined by a different value
    std::vector<Frustum> vec_frustum;
    for (int i=0; i < iNviews; ++i)
    {
      vec_frustum.push_back(
        Frustum(principal_Point*2, principal_Point*2,
                d._K[i], d._R[i], d._t[i], 0.1, -1.0));
      BOOST_CHECK(vec_frustum[i].isPartiallyTruncated());
    }

    //Check that there is no overlap between all frustums
    for (int i = 0; i < iNviews; ++i)
    {
      for (int j = 0; j < iNviews; ++j)
      {
        if(i == j)
            continue;

        BOOST_CHECK(!vec_frustum[i].intersect(vec_frustum[j]));
      }
    }
  }
}

