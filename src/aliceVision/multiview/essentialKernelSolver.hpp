// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// Copyright (c) 2010 libmv contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/multiview/twoViewKernel.hpp>
#include <aliceVision/multiview/essential.hpp>
#include <aliceVision/multiview/fundamentalKernelSolver.hpp>

#include <vector>

namespace aliceVision {
namespace essential {
namespace kernel {

using namespace std;

/**
 * Eight-point algorithm for solving for the essential matrix from normalized
 * image coordinates of point correspondences.
 * See page 294 in HZ Result 11.1.
 *
 */
struct EightPointRelativePoseSolver {
  enum { MINIMUM_SAMPLES = 8 };
  enum { MAX_MODELS = 1 };
  static void Solve(const Mat &x1, const Mat &x2, vector<Mat3> *E);
};

/**
 * Five-point algorithm to solve the Essential matrix from 5 points
 * correspondences. It solves the relative pose problem.
 * Input point must be normalized one.
 */
struct FivePointSolver {
  enum { MINIMUM_SAMPLES = 5 };
  enum { MAX_MODELS = 10 };
  static void Solve(const Mat &x1, const Mat &x2, vector<Mat3> *E);
};

//-- Generic Solver for the 5pt Essential Matrix Estimation.
//-- Need a new Class that inherit of twoView::kernel::kernel.
//    Error must be overwrite in order to compute F from E and K's.
//-- Fitting must normalize image values to camera values.
template<typename SolverArg,
  typename ErrorArg,
  typename ModelArg = Mat3>
class EssentialKernel :
   public twoView::kernel::Kernel<SolverArg,ErrorArg, ModelArg>
{
public:
  EssentialKernel(const Mat &x1, const Mat &x2,
                  const Mat3 &K1, const Mat3 &K2):
  twoView::kernel::Kernel<SolverArg,ErrorArg, ModelArg>(x1,x2),
                                                         K1_(K1), K2_(K2) {}
  void Fit(const vector<size_t> &samples, vector<ModelArg> *models) const {
    const Mat x1 = ExtractColumns(this->x1_, samples);
    const Mat x2 = ExtractColumns(this->x2_, samples);

    assert(2 == x1.rows());
    assert(SolverArg::MINIMUM_SAMPLES <= x1.cols());
    assert(x1.rows() == x2.rows());
    assert(x1.cols() == x2.cols());

    // Normalize the data (image coords to camera coords).
    const Mat3 K1Inverse = K1_.inverse();
    const Mat3 K2Inverse = K2_.inverse();
    Mat x1_normalized, x2_normalized;
    ApplyTransformationToPoints(x1, K1Inverse, &x1_normalized);
    ApplyTransformationToPoints(x2, K2Inverse, &x2_normalized);
    SolverArg::Solve(x1_normalized, x2_normalized, models);
  }

  double Error(size_t sample, const ModelArg &model) const {
    Mat3 F;
    FundamentalFromEssential(model, K1_, K2_, &F);
    return ErrorArg::Error(F, this->x1_.col(sample), this->x2_.col(sample));
  }
protected:
  Mat3 K1_, K2_; // The two camera calibrated camera matrix
};

//-- Solver kernel for the 8pt Essential Matrix Estimation
typedef essential::kernel::EssentialKernel<EightPointRelativePoseSolver,
  fundamental::kernel::SampsonError, Mat3>  EightPointKernel;


//-- Solver kernel for the 5pt Essential Matrix Estimation
typedef essential::kernel::EssentialKernel<FivePointSolver,
  fundamental::kernel::SampsonError, Mat3>  FivePointKernel;


}  // namespace kernel
}  // namespace essential
}  // namespace aliceVision
