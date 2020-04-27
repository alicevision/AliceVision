// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "aliceVision/linearProgramming/ISolver.hpp"
#include "aliceVision/linearProgramming/OSIXSolver.hpp"
#include "aliceVision/linearProgramming/lInfinityCV/global_translations_fromTij.hpp"

#include "aliceVision/multiview/translationAveraging/translationAveragingTest.hpp"

#define BOOST_TEST_MODULE translation_averaging_globalTi_from_tijs

#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>
#include <aliceVision/unitTest.hpp>

using namespace aliceVision;
using namespace aliceVision::linearProgramming;
using namespace lInfinityCV;
using namespace std;

BOOST_AUTO_TEST_CASE(translation_averaging_globalTi_from_tijs) {

  const int focal = 1000;
  const int principal_Point = 500;
  //-- Setup a circular camera rig or "cardiod".
  const int iNviews = 12;
  const int iNbPoints = 6;

  const bool bCardiod = true;
  const bool bRelative_Translation_PerTriplet = true;
  std::vector<aliceVision::translationAveraging::relativeInfo > vec_relative_estimates;

  const NViewDataSet d =
    Setup_RelativeTranslations_AndNviewDataset
    (
      vec_relative_estimates,
      focal, principal_Point, iNviews, iNbPoints,
      bCardiod, bRelative_Translation_PerTriplet
    );

  d.exportToPLY("global_translations_from_Tij_GT.ply");
  visibleCamPosToSVGSurface(d._C, "global_translations_from_Tij_GT.svg");

  //-- Compute the global translations from the translation heading directions
  //-   with the L_infinity optimization
  // 3*NCam*[X,Y,Z] ; Ncam*[Lambda], [gamma]
  std::vector<double> vec_solution(iNviews*3 + vec_relative_estimates.size() + 1);

  //- a. Setup the LP solver,
  //- b. Setup the constraints generator (for the dedicated L_inf problem),
  //- c. Build constraints and solve the problem,
  //- d. Get back the estimated parameters.

  //- a. Setup the LP solver,
  OSI_CISolverWrapper solverLP(vec_solution.size());

  //- b. Setup the constraints generator (for the dedicated L_inf problem),
  Tifromtij_ConstraintBuilder cstBuilder(vec_relative_estimates);

  //- c. Build constraints and solve the problem (Setup constraints and solver)
  LPConstraintsSparse constraint;
  cstBuilder.Build(constraint);
  solverLP.setup(constraint);
  //-- Solving
  BOOST_CHECK(solverLP.solve()); // the linear program must have a solution

  //- d. Get back the estimated parameters.
  solverLP.getSolution(vec_solution);
  const double gamma = vec_solution[vec_solution.size()-1];

  //--
  //-- Unit test checking about the found solution
  //--
  BOOST_CHECK_SMALL(gamma, 1e-6); // Gamma must be 0, no noise, perfect data have been sent

  ALICEVISION_LOG_DEBUG("Found solution with gamma = " << gamma);

  //-- Get back computed camera translations
  std::vector<double> vec_camTranslation(iNviews*3,0);
  std::copy(&vec_solution[0], &vec_solution[iNviews*3], &vec_camTranslation[0]);

  //-- Get back computed lambda factors
  std::vector<double> vec_camRelLambdas(&vec_solution[iNviews*3], &vec_solution[iNviews*3 + vec_relative_estimates.size()]);

  // Check validity of the camera centers:
  // Check the direction since solution if found up to a scale
  for (size_t i = 0; i < iNviews; ++i)
  {
    const Vec3 t(vec_camTranslation[i*3], vec_camTranslation[i*3+1], vec_camTranslation[i*3+2]);
    const Mat3 & Ri = d._R[i];
    const Vec3 C_computed = - Ri.transpose() * t;

    const Vec3 C_GT = d._C[i] - d._C[0];

    //-- Check that found camera position is equal to GT value
    if (i==0)  {
      EXPECT_MATRIX_NEAR(C_computed, C_GT, 1e-6);
    }
    else  {
     BOOST_CHECK_SMALL(DistanceLInfinity(C_computed.normalized(), C_GT.normalized()), 1e-6);
    }
  }
}
