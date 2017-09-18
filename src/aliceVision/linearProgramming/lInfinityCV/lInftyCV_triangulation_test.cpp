// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#include <iostream>
#include <vector>

#include "aliceVision/multiview/test_data_sets.hpp"
#include "aliceVision/numeric/numeric.h"
#include <aliceVision/config.hpp>
#include "testing/testing.h"

#include "aliceVision/multiview/projection.hpp"

#include "aliceVision/linearProgramming/ISolver.hpp"
#include "aliceVision/linearProgramming/OSIXSolver.hpp"
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_MOSEK)
#include "aliceVision/linearProgramming/MOSEKSolver.hpp"
#endif

#include "aliceVision/linearProgramming/bisectionLP.hpp"
#include "aliceVision/linearProgramming/lInfinityCV/triangulation.hpp"

using namespace aliceVision;
using namespace linearProgramming;
using namespace lInfinityCV;

TEST(lInfinityCV, Triangulation_OSICLPSOLVER) {

  NViewDataSet d = NRealisticCamerasRing(6, 10,
    nViewDatasetConfigurator(1,1,0,0,5,0)); // Suppose a camera with Unit matrix as K

  std::vector<Mat34> vec_Pi;

  d.ExportToPLY("test_Before_Infinity_Triangulation_OSICLP.ply");
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
    EXPECT_TRUE(
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
      x1 = Project(d2.P(i), X);
      xsum += Vec2((x1-x_ij.col(i)).array().pow(2));
    }
    double dResidual2D = (xsum.array().sqrt().sum());

    // Residual LInfinity between GT 3D point and found one
    double dResidual3D = DistanceLInfinity(XSolution, Vec3(d._X.col(k)));

    // Check that 2D re-projection and 3D point are near to GT.
    EXPECT_NEAR(0.0, dResidual2D, 1e-5);
    EXPECT_NEAR(0.0, dResidual3D, 1e-5);
  }
  d2.ExportToPLY("test_After_Infinity_Triangulation_OSICLP.ply");
}

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_MOSEK)
TEST(computervision, Triangulation_MOSEK) {

  NViewDataSet d = NRealisticCamerasRing(6, 10,
    nViewDatasetConfigurator(1,1,0,0,5,0)); // Suppose a camera with Unit matrix as K

  std::vector<Mat34> vec_Pi;

  d.ExportToPLY("test_Before_Infinity_Triangulation_MOSEK.ply");
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
    EXPECT_TRUE(
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
      x1 = Project(d2.P(i), X);
      xsum += Vec2((x1-x_ij.col(i)).array().pow(2));
    }
    double dResidual2D = (xsum.array().sqrt().sum());

    // Residual LInfinity between GT 3D point and found one
    double dResidual3D = DistanceLInfinity(XSolution, Vec3(d._X.col(k)));

    // Check that 2D re-projection and 3D point are near to GT.
    EXPECT_NEAR(0.0, dResidual2D, 1e-5);
    EXPECT_NEAR(0.0, dResidual3D, 1e-5);
  }
  d2.ExportToPLY("test_After_Infinity_Triangulation_MOSEK.ply");
}
#endif // ALICEVISION_HAVE_MOSEK

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
