// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/types.hpp>
#include <aliceVision/sfm/BundleAdjustment.hpp>
#include <aliceVision/sfm/ResidualErrorFunctor.hpp>

#include <ceres/ceres.h>

namespace aliceVision {

namespace sfmData {
class SfMData;
} // namespace sfmData

namespace sfm {

/// Create the appropriate cost functor according the provided input camera intrinsic model
ceres::CostFunction* createCostFunctionFromIntrinsics(camera::IntrinsicBase* intrinsic, const Vec2& observation);
ceres::CostFunction* createRigCostFunctionFromIntrinsics(camera::IntrinsicBase* intrinsic, const Vec2& observation);

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

    BA_options(const bool verbose = true, bool multithreaded = true);

    void setDenseBA();
    void setSparseBA();
  };
private:
    BA_options _aliceVision_options;
    // Data wrapper for refinement:
    HashMap<IndexT, std::vector<double> > map_poses;
    // Setup rig sub-poses
    HashMap<IndexT, HashMap<IndexT, std::vector<double>>> map_subposes;
    std::vector<double*> parameterBlocks;
    HashMap<IndexT, std::vector<double> > map_intrinsics;

public:

  BundleAdjustmentCeres(BundleAdjustmentCeres::BA_options options = BA_options());
  void createProblem(sfmData::SfMData& sfmData, ERefineOptions refineOptions, ceres::Problem& problem);
  void createJacobian(sfmData::SfMData& sfmData, ERefineOptions refineOptions, ceres::CRSMatrix& jacobian);

  /**
   * @see BundleAdjustment::Adjust
   */
  bool adjust(sfmData::SfMData& sfmData, ERefineOptions refineOptions = REFINE_ALL);
};

} // namespace sfm
} // namespace aliceVision
