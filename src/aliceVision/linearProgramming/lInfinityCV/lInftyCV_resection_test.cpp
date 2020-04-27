// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <iostream>
#include <vector>

#include "aliceVision/multiview/NViewDataSet.hpp"
#include "aliceVision/numeric/projection.hpp"
#include <aliceVision/config.hpp>

#include "aliceVision/linearProgramming/ISolver.hpp"
#include "aliceVision/linearProgramming/OSIXSolver.hpp"
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_MOSEK)
#include "aliceVision/linearProgramming/MOSEKSolver.hpp"
#endif // ALICEVISION_HAVE_MOSEK
#include "aliceVision/linearProgramming/bisectionLP.hpp"
#include "aliceVision/linearProgramming/lInfinityCV/resection.hpp"


#define BOOST_TEST_MODULE ResectionLInfinity

#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>
#include <aliceVision/unitTest.hpp>

using namespace aliceVision;

using namespace linearProgramming;
using namespace lInfinityCV;

void translate(const Mat3X & X, const Vec3 & vecTranslation,
  Mat3X * XPoints)  {
    XPoints->resize(X.rows(), X.cols());
    for (size_t i=0; i<(size_t)X.cols(); ++i) {
      XPoints->col(i) = X.col(i) + vecTranslation;
    }
}

BOOST_AUTO_TEST_CASE(Resection_L_Infinity_OSICLP) {

  const int nViews = 3;
  const int nbPoints = 10;
  const NViewDataSet d = NRealisticCamerasRing(nViews, nbPoints,
    NViewDatasetConfigurator(1,1,0,0,5,0)); // Suppose a camera with Unit matrix as K

  d.exportToPLY("test_Before_Infinity.ply");
  //-- Modify a dataset (set to 0 and parse new value) (Assert good values)
  NViewDataSet d2 = d;

  const int nResectionCameraIndex = 2;
  //-- Set to 0 the future computed data to be sure of computation results :
  d2._R[nResectionCameraIndex] = Mat3::Zero();
  d2._t[nResectionCameraIndex] = Vec3::Zero();

  // Solve the problem and check that fitted value are good enough
  {
    std::vector<double> vec_solution(11);

    //-- Translate 3D point in order to have X0 = (0,0,0,1).
    Vec3 vecTranslation = - d2._X.col(0);
    Mat4 translationMatrix = Mat4::Identity();
    translationMatrix << 1, 0, 0, vecTranslation(0),
                         0, 1, 0, vecTranslation(1),
                         0, 0, 1, vecTranslation(2),
                         0, 0, 0, 1;
    Mat3X XPoints;
    translate(d2._X, vecTranslation, &XPoints);

    OSI_CISolverWrapper wrapperOSICLPSolver(vec_solution.size());
    Resection_L1_ConstraintBuilder cstBuilder(d2._x[nResectionCameraIndex], XPoints);
    BOOST_CHECK(
      (BisectionLP<Resection_L1_ConstraintBuilder, LPConstraintsSparse>(
      wrapperOSICLPSolver,
      cstBuilder,
      &vec_solution,
      1.0,
      0.0))
    );

    // Move computed value to dataset for residual estimation.
    Mat34 P;
    P << vec_solution[0], vec_solution[1], vec_solution[2], vec_solution[3],
         vec_solution[4], vec_solution[5], vec_solution[6], vec_solution[7],
         vec_solution[8], vec_solution[9], vec_solution[10], 1.0;
    P = P * translationMatrix;

    // Check that Projection matrix is near to the GT :
    Mat34 GT_ProjectionMatrix = d.P(nResectionCameraIndex).array()
                                / d.P(nResectionCameraIndex).norm();
    Mat34 COMPUTED_ProjectionMatrix = P.array() / P.norm();
    EXPECT_MATRIX_NEAR(GT_ProjectionMatrix, COMPUTED_ProjectionMatrix, 1e-4);
  }
  d2.exportToPLY("test_After_Infinity.ply");
}

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_MOSEK)
BOOST_AUTO_TEST_CASE(Resection_L_Infinity_MOSEK) {

  const int nViews = 3;
  const int nbPoints = 10;
  const NViewDataSet d = NRealisticCamerasRing(nViews, nbPoints,
    NViewDatasetConfigurator(1,1,0,0,5,0)); // Suppose a camera with Unit matrix as K

  d.exportToPLY("test_Before_Infinity.ply");
  //-- Modify a dataset (set to 0 and parse new value) (Assert good values)
  NViewDataSet d2 = d;

  const int nResectionCameraIndex = 2;
  //-- Set to 0 the future computed data to be sure of computation results :
  d2._R[nResectionCameraIndex] = Mat3::Zero();
  d2._t[nResectionCameraIndex] = Vec3::Zero();

  // Solve the problem and check that fitted value are good enough
  {
    std::vector<double> vec_solution(11);

    //-- Translate 3D point in order to have X0 = (0,0,0,1).
    Vec3 vecTranslation = - d2._X.col(0);
    Mat4 translationMatrix = Mat4::Identity();
    translationMatrix << 1, 0, 0, vecTranslation(0),
                         0, 1, 0, vecTranslation(1),
                         0, 0, 1, vecTranslation(2),
                         0, 0, 0, 1;
    Mat3X XPoints;
    translate(d2._X, vecTranslation, &XPoints);

    MOSEKSolver wrapperMosek(vec_solution.size());
    Resection_L1_ConstraintBuilder cstBuilder(d2._x[nResectionCameraIndex], XPoints);
    BOOST_CHECK(
      (BisectionLP<Resection_L1_ConstraintBuilder, LPConstraintsSparse>(
      wrapperMosek,
      cstBuilder,
      &vec_solution,
      1.0,
      0.0))
    );

    // Move computed value to dataset for residual estimation.
    Mat34 P;
    P << vec_solution[0], vec_solution[1], vec_solution[2], vec_solution[3],
         vec_solution[4], vec_solution[5], vec_solution[6], vec_solution[7],
         vec_solution[8], vec_solution[9], vec_solution[10], 1.0;
    P = P * translationMatrix;

    // Check that Projection matrix is near to the GT :
    Mat34 GT_ProjectionMatrix = d.P(nResectionCameraIndex).array()
                                / d.P(nResectionCameraIndex).norm();
    Mat34 COMPUTED_ProjectionMatrix = P.array() / P.norm();
    EXPECT_MATRIX_NEAR(GT_ProjectionMatrix, COMPUTED_ProjectionMatrix, 1e-4);
  }
  d2.exportToPLY("test_After_Infinity.ply");
}
#endif // #if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_MOSEK)
