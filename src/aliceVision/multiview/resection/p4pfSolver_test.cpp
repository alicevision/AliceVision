// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#define BOOST_TEST_MODULE P4PfSolver

#include <aliceVision/multiview/resection/ResectionKernel.hpp>
#include <aliceVision/multiview/resection/P4PfSolver.hpp>
#include <aliceVision/multiview/NViewDataSet.hpp>


#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>

#include <vector>


using namespace aliceVision;
using namespace aliceVision::multiview;

bool isEqual(const resection::P4PfModel first, const resection::P4PfModel second)
{
  double eps = 1e-3;
  return ((first._R - second._R).norm() < first._R.maxCoeff() * eps &&
          (first._t - second._t).norm() < first._t.maxCoeff() * eps &&
          abs(first._f - second._f) < first._f * eps);
}

BOOST_AUTO_TEST_CASE(Resection_P4Pf_AssignmentWithOneResult)
{
  // INPUT DATA
  Mat pt2D_1 = Mat(2, 4);
  pt2D_1 << -493.1500, 1051.9100, 176.9500, -1621.9800,
          -878.4550, -984.7530, -381.4300, -543.3450;
  Mat pt3D_1 = Mat(3, 4);
  pt3D_1 << 2.7518, 2.2375, 1.1940, 2.5778,
          0.1336, -0.3709, 0.2048, -0.9147,
          -0.5491, -2.0511, 1.1480, -1.6151;

  // EXPECTED RESULT
  Mat R_1 = Mat(3, 3);
  R_1 << -0.97189, 0.05884, -0.22797, -0.02068, -0.98586, -0.16631, -0.23454, -0.15692, 0.95936;
  Vec3 t_1;
  t_1 << 2.00322, -1.27420, 2.92685;
  resection::P4PfModel sol_1(R_1, t_1, 887.17549);

  // PROCESS THE RESECTION P4Pf
  std::vector<resection::P4PfModel> models_1;
  resection::P4PfSolver solver;
  solver.solve(pt2D_1, pt3D_1, models_1);

  bool pass = true;
  if(!(models_1.size() == 1 && isEqual(models_1.at(0), sol_1)))
    pass = false;
  BOOST_CHECK(pass);
}

BOOST_AUTO_TEST_CASE(Resection_P4Pf_AssignmentWithMoreResults)
{
  // DATA
  Mat pt2D_2 = Mat(2, 4);
  pt2D_2 << 774.88000, -772.31000, -1661.63300, -1836.57300,
          -534.74500, -554.09400, -585.53300, -430.03000;
  Mat pt3D_2 = Mat(3, 4);
  pt3D_2 << 2.01852, 1.00709, 0.74051, 0.61962,
          0.02133, 0.30770, 0.16656, 0.11249,
          -1.68077, 0.81502, 1.21056, 1.22624;

  // RESULTS
  Mat R_21 = Mat(3, 3);
  R_21 << 0.74908, 0.58601, -0.30898, 0.65890, -0.61061, 0.43933, 0.06879, -0.53268, -0.84352;
  Mat R_22 = Mat(3, 3);
  R_22 << 0.06352, -0.56461, -0.82291, -0.97260, 0.14975, -0.17781, 0.22362, 0.81166, -0.53963;
  Mat R_23 = Mat(3, 3);
  R_23 << 0.02362, -0.60298, -0.79741, -0.96400, 0.19758, -0.17796, 0.26486, 0.77290, -0.57661;
  Vec3 t_21;
  t_21 << -1.17794, -1.17674, 3.57853;
  Vec3 t_22;
  t_22 << 0.08257, 0.57753, 1.04335;
  Vec3 t_23;
  t_23 << 0.16029, 0.58720, 1.07571;
  resection::P4PfModel sol_21(R_21, t_21, 4571.95746);
  resection::P4PfModel sol_22(R_22, t_22, 1193.30606);
  resection::P4PfModel sol_23(R_23, t_23, 1315.17564);

  // PROCESS
  std::vector<resection::P4PfModel> models_2;
  resection::P4PfSolver solver;
  solver.solve(pt2D_2, pt3D_2, models_2);

  bool pass = true;
  if(!(models_2.size() == 3
          && isEqual(models_2.at(0), sol_21)
          && isEqual(models_2.at(1), sol_22)
          && isEqual(models_2.at(2), sol_23)))
    pass = false;
  BOOST_CHECK(pass);
}

BOOST_AUTO_TEST_CASE(Resection_P4Pf_AssignmentWithNoResults)
{
  // DATA
  Mat pt2D_3 = Mat(2, 4);
  pt2D_3 << 774.88000, -570.41000, -1881.86960, 1529.54000,
          -534.74500, -834.63100, -167.32000, -1203.28000;
  Mat pt3D_3 = Mat(3, 4);
  pt3D_3 << 2.01852, 1.28149, 0.55264, 2.29633,
          0.02133, 0.26101, 0.14578, -1.80998,
          -1.68077, 0.70813, 1.22217, -1.76850;

  // PROCESS
  std::vector<resection::P4PfModel> models_3;
  resection::P4PfSolver solver;
  solver.solve(pt2D_3, pt3D_3, models_3);

  bool pass = true;
  if(!models_3.empty())
    pass = false;
  BOOST_CHECK(pass);
}
