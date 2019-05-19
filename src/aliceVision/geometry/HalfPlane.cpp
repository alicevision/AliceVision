// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "HalfPlane.hpp"
#include "aliceVision/linearProgramming/OSIXSolver.hpp"

namespace aliceVision {
namespace geometry {
namespace halfPlane {

/// Return true if the half_planes define a not empty volume (an intersection exists)
bool isNotEmpty(const Half_planes & hplanes)
{
  // Check if it exists a point on all positive side of the half plane thanks to a Linear Program formulation [1].
  // => If a point exists: there is a common subspace defined and so intersections.
  // The LP formulation consists in set the Half_plane as constraint and check if a point can fit the equations.

  using namespace aliceVision;
  using namespace aliceVision::linearProgramming;

  LPConstraints cstraint;
  {
    cstraint._nbParams = 3; // {X,Y,Z}
    cstraint._vec_bounds.resize(cstraint._nbParams);
    std::fill(cstraint._vec_bounds.begin(),cstraint._vec_bounds.end(),
      std::make_pair((double)-1e+30, (double)1e+30)); // [X,Y,Z] => -inf, +inf
    cstraint._bminimize = true;

    // Configure constraints
    const size_t nbConstraints = hplanes.size();
    cstraint._constraintMat = Mat(nbConstraints,3);
    cstraint._vec_sign.resize(nbConstraints);
    cstraint._Cst_objective = Vec(nbConstraints);

    // Fill the constrains (half-space equations)
    for (unsigned char i= 0; i < hplanes.size(); ++i)
    {
      const Vec & half_plane_coeff = hplanes[i].coeffs();
      // add the half plane equation to the system
      cstraint._constraintMat.row(i) =
        Vec3(half_plane_coeff(0),
          half_plane_coeff(1),
          half_plane_coeff(2));
      cstraint._vec_sign[i] = LPConstraints::LP_GREATER_OR_EQUAL;
      cstraint._Cst_objective(i) = - half_plane_coeff(3);
    }
  }

  // Solve in order to see if a point exists within the half spaces positive side?
  OSI_CISolverWrapper solver(cstraint._nbParams);
  solver.setup(cstraint);
  const bool bIntersect = solver.solve(); // Status of the solver tell if there is an intersection or not
  return bIntersect;
}

} // namespace halfPlane
} // namespace geometry
} // namespace aliceVision
