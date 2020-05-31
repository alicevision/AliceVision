// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <iostream>
#include <vector>

#include "aliceVision/multiview/NViewDataSet.hpp"
#include "aliceVision/numeric/numeric.hpp"
#include <aliceVision/config.hpp>

#include "aliceVision/numeric/projection.hpp"

#include "aliceVision/linearProgramming/ISolver.hpp"
#include "aliceVision/linearProgramming/OSIXSolver.hpp"
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_MOSEK)
#include "aliceVision/linearProgramming/MOSEKSolver.hpp"
#endif

#include "aliceVision/linearProgramming/bisectionLP.hpp"
#include "aliceVision/linearProgramming/lInfinityCV/triangulation.hpp"

#define BOOST_TEST_MODULE lInfinityCVTriangulation

#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>

using namespace aliceVision;
using namespace linearProgramming;
using namespace lInfinityCV;

BOOST_AUTO_TEST_CASE(lInfinityCV_Triangulation_OSICLPSOLVER) {

  NViewDataSet d = NRealisticCamerasRing(6, 10,
    NViewDatasetConfigurator(1,1,0,0,5,0)); // Suppose a camera with Unit matrix as K

  std::vector<Mat34> vec_Pi;

  d.exportToPLY("test_Before_Infinity_Triangulation_OSICLP.ply");
  //-- Test triangulation of all the point
  NViewDataSet d2 = d;
  d2._X.fill(0); //Set _Xi of dataset 2 to 0 to be sure of new data computation

  for (int i = 0; i < d._n; ++i)
    vec_Pi.push_back(d.P(i));

  for (int k = 0; k < d._x[0].cols(); ++k)
  {
    Mat2X x_ij;
    x_ij.resize(2,d._n);
    for (int i = 0; i < d._n; ++i)
      x_ij.col(i) = d._x[i].col(k);

    std::vector<double> vec_solution(3);

    OSI_CISolverWrapper wrapperOSICLPSolver(3);
    Triangulation_L1_ConstraintBuilder cstBuilder(vec_Pi, x_ij);
    // Use bisection in order to find the global optimum and so find the
    //  best triangulated point under the L_infinity norm
    BOOST_CHECK(
      (BisectionLP<Triangulation_L1_ConstraintBuilder,LPConstraints>(
      wrapperOSICLPSolver,
      cstBuilder,
      &vec_solution,
      1.0,
      0.0))
    );

    Vec3 XSolution(vec_solution[0], vec_solution[1], vec_solution[2]);
    d2._X.col(k) = XSolution.transpose();

    // Compute residuals L2 from estimated parameter values :
    const Vec3 & X = XSolution;
    Vec2 x1, xsum(0.0,0.0);
    for (int i = 0; i < d2._n; ++i) {
      x1 = project(d2.P(i), X);
      xsum += Vec2((x1-x_ij.col(i)).array().pow(2));
    }
    double dResidual2D = (xsum.array().sqrt().sum());

    // Residual LInfinity between GT 3D point and found one
    double dResidual3D = DistanceLInfinity(XSolution, Vec3(d._X.col(k)));

    // Check that 2D re-projection and 3D point are near to GT.
    BOOST_CHECK_SMALL(dResidual2D, 1e-5);
    BOOST_CHECK_SMALL(dResidual3D, 1e-5);
  }
  d2.exportToPLY("test_After_Infinity_Triangulation_OSICLP.ply");
}

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_MOSEK)
BOOST_AUTO_TEST_CASE(computervision_Triangulation_MOSEK) {

  NViewDataSet d = NRealisticCamerasRing(6, 10,
    NViewDatasetConfigurator(1,1,0,0,5,0)); // Suppose a camera with Unit matrix as K

  std::vector<Mat34> vec_Pi;

  d.exportToPLY("test_Before_Infinity_Triangulation_MOSEK.ply");
  //-- Test triangulation of all the point
  NViewDataSet d2 = d;
  d2._X.fill(0); //Set _Xi of dataset 2 to 0 to be sure of new data computation

  for (int i = 0; i < d._n; ++i)
    vec_Pi.push_back(d.P(i));

  for (int k = 0; k < d._x[0].cols(); ++k)
  {
    Mat2X x_ij;
    x_ij.resize(2,d._n);
    for (int i = 0; i < d._n; ++i)
      x_ij.col(i) = d._x[i].col(k);

    std::vector<double> vec_solution(3);

    MOSEKSolver wrapperLpSolve(3);
    Triangulation_L1_ConstraintBuilder cstBuilder(vec_Pi, x_ij);
    // Use bisection in order to find the global optimum and so find the
    //  best triangulated point under the L_infinity norm
    BOOST_CHECK(
      (BisectionLP<Triangulation_L1_ConstraintBuilder,LPConstraints>(
      wrapperLpSolve,
      cstBuilder,
      &vec_solution,
      1.0,
      0.0))
    );

    Vec3 XSolution(vec_solution[0], vec_solution[1], vec_solution[2]);
    d2._X.col(k) = XSolution.transpose();

    // Compute residuals L2 from estimated parameter values :
    const Vec3 & X = XSolution;
    Vec2 x1, xsum(0.0,0.0);
    for (int i = 0; i < d2._n; ++i) {
      x1 = project(d2.P(i), X);
      xsum += Vec2((x1-x_ij.col(i)).array().pow(2));
    }
    double dResidual2D = (xsum.array().sqrt().sum());

    // Residual LInfinity between GT 3D point and found one
    double dResidual3D = DistanceLInfinity(XSolution, Vec3(d._X.col(k)));

    // Check that 2D re-projection and 3D point are near to GT.
    BOOST_CHECK_SMALL(dResidual2D, 1e-5);
    BOOST_CHECK_SMALL(dResidual3D, 1e-5);
  }
  d2.exportToPLY("test_After_Infinity_Triangulation_MOSEK.ply");
}
#endif // ALICEVISION_HAVE_MOSEK
