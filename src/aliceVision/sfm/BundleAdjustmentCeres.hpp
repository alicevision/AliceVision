// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "aliceVision/sfm/SfMData.hpp"
#include "aliceVision/sfm/BundleAdjustment.hpp"
#include "aliceVision/sfm/ResidualErrorFunctor.hpp"
#include "ceres/ceres.h"

namespace aliceVision {
namespace sfm {

/// Create the appropriate cost functor according the provided input camera intrinsic model
ceres::CostFunction * createCostFunctionFromIntrinsics(camera::IntrinsicBase * intrinsic, const Vec2 & observation);
ceres::CostFunction * createRigCostFunctionFromIntrinsics(camera::IntrinsicBase * intrinsic, const Vec2 & observation);

class BundleAdjustmentCeres : public BundleAdjustment
{
  public:
  struct BA_options
  {
    bool _bVerbose;
    unsigned int _nbThreads;
    bool _bCeres_Summary;
    ceres::LinearSolverType _linear_solver_type;
    ceres::PreconditionerType _preconditioner_type;
    ceres::SparseLinearAlgebraLibraryType _sparse_linear_algebra_library_type;

    BA_options(const bool bVerbose = true, bool bmultithreaded = true);
    void setDenseBA();
    void setSparseBA();
  };
  private:
    BA_options _aliceVision_options;

  public:
  BundleAdjustmentCeres(BundleAdjustmentCeres::BA_options options = BA_options());

  /**
   * @see BundleAdjustment::Adjust
   */
  bool Adjust(
    SfMData & sfm_data,
    BA_Refine refineOptions = BA_REFINE_ALL);
};

} // namespace sfm
} // namespace aliceVision
