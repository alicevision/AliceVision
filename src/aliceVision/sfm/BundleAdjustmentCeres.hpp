// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#pragma once

#include "aliceVision/sfm/SfMData.hpp"
#include "aliceVision/sfm/BundleAdjustment.hpp"
#include "aliceVision/sfm/ResidualErrorFunctor.hpp"
#include "ceres/ceres.h"

namespace aliceVision {
namespace sfm {

/// Create the appropriate cost functor according the provided input camera intrinsic model
ceres::CostFunction * IntrinsicsToCostFunction(
  camera::IntrinsicBase * intrinsic,
  const Vec2 & observation);

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
