// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#ifndef OPENMVG_SFM_DATA_BA_CERES_HPP
#define OPENMVG_SFM_DATA_BA_CERES_HPP

#include "aliceVision/sfm/sfm_data.hpp"
#include "aliceVision/sfm/sfm_data_BA.hpp"
#include "aliceVision/sfm/sfm_data_BA_ceres_camera_functor.hpp"
#include "ceres/ceres.h"

namespace aliceVision {
namespace sfm {

/// Create the appropriate cost functor according the provided input camera intrinsic model
ceres::CostFunction * IntrinsicsToCostFunction(
  cameras::IntrinsicBase * intrinsic,
  const Vec2 & observation);

class Bundle_Adjustment_Ceres : public Bundle_Adjustment
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
  Bundle_Adjustment_Ceres(Bundle_Adjustment_Ceres::BA_options options = BA_options());

  /**
   * @see Bundle_Adjustment::Adjust
   */
  bool Adjust(
    SfM_Data & sfm_data,
    BA_Refine refineOptions = BA_REFINE_ALL);
};

} // namespace sfm
} // namespace aliceVision

#endif // OPENMVG_SFM_DATA_BA_CERES_HPP
