// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "aliceVision/linearProgramming/OSIXSolver.hpp"

#include <Eigen/Geometry>
#include <Eigen/StdVector>

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Hyperplane<double,3>)

namespace aliceVision {
namespace geometry {
namespace halfPlane {

/// Define the Half_plane equation (abcd coefficients)
typedef Eigen::Hyperplane<double,3> Half_plane;
/// Define a collection of Half_plane
typedef std::vector<Half_plane> Half_planes;

// Define a plane passing through the points (p, q, r).
// The plane is oriented such that p, q and r are oriented in a positive sense (that is counterclockwise).
inline Half_plane Half_plane_p(const Vec3 & p, const Vec3 & q, const Vec3 & r)
{
  const Vec3 abc = (p-r).cross(q-r);
  const double d = - abc.dot(r);
  Half_plane hp;
  hp.coeffs() << abc(0),abc(1),abc(2),d;
  return hp;
}

// [1] Paper: Finding the intersection of n half-spaces in time O(n log n).
// Author: F.P. Preparata, D.E. Muller
// Published in: Theoretical Computer Science, Volume 8, Issue 1, Pages 45-55
// Year: 1979
// More: ISSN 0304-3975, http://dx.doi.org/10.1016/0304-3975(79)90055-0.

/// Return true if the half_planes define a not empty volume (an intersection exists)
bool isNotEmpty(const Half_planes & hplanes);

} // namespace geometry
} // namespace aliceVision
} // namespace halfPlane
