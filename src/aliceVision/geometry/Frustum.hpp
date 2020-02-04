// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#ifndef ALICEVISION_GEOMETRY_FRUSTUM_HPP_
#define ALICEVISION_GEOMETRY_FRUSTUM_HPP_

#include "aliceVision/geometry/HalfPlane.hpp"

namespace aliceVision {
namespace geometry {

using namespace aliceVision::geometry::halfPlane;

/// Define a camera Frustum:
///  - infinite Frustum (4 Half Spaces) (a pyramid)
///  - truncated Frustum (6 Half Spaces) (a truncated pyramid)
///  - This structure is used for testing frustum intersection (see if two cam can share visual content)
struct Frustum
{
  Vec3 cones[5]; // camera centre and the 4 points that define the image plane
  Half_planes planes; // Define infinite frustum planes + 2 optional Near and Far Half Space
  double z_near = -1., z_far = -1.;
  std::vector<Vec3> points;

  Frustum() {}

  // Build a frustum from the image size, camera intrinsic and pose
  Frustum(const int w, const int h, const Mat3 & K, const Mat3 & R, const Vec3 & C, const double zNear = -1.0, const double zFar = -1.0)
      : z_near(zNear)
      , z_far(zFar)
  {
    // supporting point are the points defined by the truncated cone
    const Mat3 Kinv = K.inverse();
    const Mat3 Rt = R.transpose();

    // Definition of the frustum with the supporting points
    cones[0] = C;
    cones[1] = Rt * ((Kinv * Vec3(0,0,1.0))) + C;
    cones[2] = Rt * ((Kinv * Vec3(w,0,1.0))) + C;
    cones[3] = Rt * ((Kinv * Vec3(w,h,1.0))) + C;
    cones[4] = Rt * ((Kinv * Vec3(0,h,1.0))) + C;

    // Definition of the supporting planes
    planes.push_back( Half_plane_p(cones[0], cones[4], cones[1]) );
    planes.push_back( Half_plane_p(cones[0], cones[1], cones[2]) );
    planes.push_back( Half_plane_p(cones[0], cones[2], cones[3]) );
    planes.push_back( Half_plane_p(cones[0], cones[3], cones[4]) );

    // Add Znear and ZFar half plane using the cam looking direction
    const Vec3 camLookDirection_n = R.row(2).normalized();

    if(zNear > 0)
    {
        const double d_near = - zNear - camLookDirection_n.dot(C);
        planes.push_back( Half_plane(camLookDirection_n, d_near) );

        points.push_back( Rt * (z_near * (Kinv * Vec3(0,0,1.0))) + C);
        points.push_back( Rt * (z_near * (Kinv * Vec3(w,0,1.0))) + C);
        points.push_back( Rt * (z_near * (Kinv * Vec3(w,h,1.0))) + C);
        points.push_back( Rt * (z_near * (Kinv * Vec3(0,h,1.0))) + C);
    }
    else
    {
        points.push_back(cones[0]);
    }
    if(zFar > 0)
    {
        const double d_Far = zFar + camLookDirection_n.dot(C);
        planes.push_back( Half_plane(-camLookDirection_n, d_Far) );

        points.push_back( Rt * (z_far * (Kinv * Vec3(0,0,1.0))) + C);
        points.push_back( Rt * (z_far * (Kinv * Vec3(w,0,1.0))) + C);
        points.push_back( Rt * (z_far * (Kinv * Vec3(w,h,1.0))) + C);
        points.push_back( Rt * (z_far * (Kinv * Vec3(0,h,1.0))) + C);
    }
    else
    {
        points.push_back(cones[1]);
        points.push_back(cones[2]);
        points.push_back(cones[3]);
        points.push_back(cones[4]);
    }
  }

  /// Test if two frustums intersect or not
  bool intersect(const Frustum & f) const
  {
    // Concatenate the Half Planes and see if an intersection exists
    std::vector<Half_plane> vec_planes(planes.size() + f.planes.size());
    std::copy(&planes[0], &planes[0]+planes.size(), &vec_planes[0]);
    std::copy(&f.planes[0], &f.planes[0]+f.planes.size(), &vec_planes[planes.size()]);

    return halfPlane::isNotEmpty(vec_planes);
  }

  /// Return true if the Frustum is an infinite one
  bool isInfinite() const
  {
    return planes.size() == 4;
  }

  /// Return true if the Frustum is truncated
  bool isTruncated() const
  {
    return planes.size() == 6;
  }

  bool isPartiallyTruncated() const
  {
    return planes.size() == 5;
  }

  // Return the supporting frustum points (5 for the infinite, 8 for the truncated)
  const std::vector<Vec3> & frustum_points() const
  {
    return points;
  }

}; // struct Frustum

} // namespace geometry
} // namespace aliceVision

#endif // ALICEVISION_GEOMETRY_FRUSTUM_HPP_
