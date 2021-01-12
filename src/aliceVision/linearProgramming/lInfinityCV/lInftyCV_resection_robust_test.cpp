// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "aliceVision/multiview/NViewDataSet.hpp"

#include "aliceVision/linearProgramming/lInfinityCV/resection_kernel.hpp"
#include "aliceVision/robustEstimation/maxConsensus.hpp"
#include "aliceVision/robustEstimation/ScoreEvaluator.hpp"
#include "aliceVision/numeric/projection.hpp"

#include <iostream>
#include <vector>

#define BOOST_TEST_MODULE ResectionLInfinityRobust

#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>

using namespace aliceVision;
using namespace aliceVision::robustEstimation;

BOOST_AUTO_TEST_CASE(Resection_L_Infinity_Robust_OutlierFree) {
  std::mt19937 randomNumberGenerator;
  const int nViews = 3;
  const int nbPoints = 10;
  const NViewDataSet d = NRealisticCamerasRing(nViews, nbPoints,
    NViewDatasetConfigurator(1,1,0,0,5,0)); // Suppose a camera with Unit matrix as K

  //-- Modify a dataset (set to 0 and parse new value) (Assert good values)
  NViewDataSet d2 = d;

  const int nResectionCameraIndex = 2;
  //-- Set to 0 the future computed data to be sure of computation results :
  d2._R[nResectionCameraIndex] = Mat3::Zero();
  d2._t[nResectionCameraIndex] = Vec3::Zero();

  // Solve the problem and check that fitted value are good enough
  {
    typedef  lInfinityCV::kernel::l1PoseResectionKernel KernelType;
    const Mat& pt2D = d2._x[nResectionCameraIndex];
    const Mat& pt3D = d2._X;

    KernelType kernel(pt2D, pt3D);
    ScoreEvaluator<KernelType> scorer(2*Square(0.6));
    robustEstimation::Mat34Model P = maxConsensus(kernel, scorer, randomNumberGenerator, nullptr, 128);

    // Check that Projection matrix is near to the GT :
    Mat34 GT_ProjectionMatrix = d.P(nResectionCameraIndex).array()
                                / d.P(nResectionCameraIndex).norm();
    Mat34 COMPUTED_ProjectionMatrix = P.getMatrix().array() / P.getMatrix().norm();

    // Extract K[R|t]
    Mat3 R,K;
    Vec3 t;
    KRt_from_P(P.getMatrix(), &K, &R, &t);

    d2._R[nResectionCameraIndex] = R;
    d2._t[nResectionCameraIndex] = t;

    //CHeck matrix to GT, and residual
    BOOST_CHECK_SMALL(FrobeniusDistance(GT_ProjectionMatrix, COMPUTED_ProjectionMatrix), 1e-1 );
    BOOST_CHECK_SMALL(reprojectionErrorRMSE(pt2D, pt3D.colwise().homogeneous(), COMPUTED_ProjectionMatrix), 1e-1);
  }
}

BOOST_AUTO_TEST_CASE(Resection_L_Infinity_Robust_OneOutlier)
{
  std::mt19937 randomNumberGenerator;

  const int nViews = 3;
  const int nbPoints = 20;
  const NViewDataSet d = NRealisticCamerasRing(nViews, nbPoints,
    NViewDatasetConfigurator(1,1,0,0,5,0)); // Suppose a camera with Unit matrix as K

  d.exportToPLY("test_Before_Infinity.ply");
  //-- Modify a dataset (set to 0 and parse new value) (Assert good values)
  NViewDataSet d2 = d;

  const int nResectionCameraIndex = 2;
  //-- Set to 0 the future computed data to be sure of computation results :
  d2._R[nResectionCameraIndex] = Mat3::Zero();
  d2._t[nResectionCameraIndex] = Vec3::Zero();

  // Set 20% of the 3D point as outlier
  const int nbOutlier = nbPoints*0.2;
  for (int i=0; i < nbOutlier; ++i)
  {
    d2._X.col(i)(0) += 120.0;
    d2._X.col(i)(1) += -60.0;
    d2._X.col(i)(2) += 80.0;
  }

  // Solve the problem and check that fitted value are good enough
  {
    typedef  lInfinityCV::kernel::l1PoseResectionKernel KernelType;
    const Mat & pt2D = d2._x[nResectionCameraIndex];
    const Mat & pt3D = d2._X;
    KernelType kernel(pt2D, pt3D);
    ScoreEvaluator<KernelType> scorer(Square(0.1)); //Highly intolerant for the test
    robustEstimation::Mat34Model P = maxConsensus(kernel, scorer, randomNumberGenerator, nullptr, 128);

    // Check that Projection matrix is near to the GT :
    Mat34 GT_ProjectionMatrix = d.P(nResectionCameraIndex).array()
      / d.P(nResectionCameraIndex).norm();
    Mat34 estimatedProjectionMatrix = P.getMatrix().array() / P.getMatrix().norm();

    // Extract K[R|t]
    Mat3 R,K;
    Vec3 t;
    KRt_from_P(P.getMatrix(), &K, &R, &t);

    d2._R[nResectionCameraIndex] = R;
    d2._t[nResectionCameraIndex] = t;

    //CHeck matrix to GT, and residual
    BOOST_CHECK_SMALL(FrobeniusDistance(GT_ProjectionMatrix, estimatedProjectionMatrix), 1e-1 );
    BOOST_CHECK_SMALL(reprojectionErrorRMSE(pt2D, pt3D.colwise().homogeneous(), estimatedProjectionMatrix), 0.75);
  }
  d2.exportToPLY("test_After_Infinity.ply");
}
