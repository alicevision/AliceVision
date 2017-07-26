// Copyright (c) 2015 Pierre Moulon.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef OPENMVG_SFM_DATA_BA_CERES_HPP
#define OPENMVG_SFM_DATA_BA_CERES_HPP

#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_BA.hpp"
#include "openMVG/sfm/sfm_data_BA_ceres_camera_functor.hpp"
#include "ceres/ceres.h"

namespace openMVG {
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
    bool useParametersOrdering; 
    bool useLocalBA;
    ceres::LinearSolverType _linear_solver_type;
    ceres::PreconditionerType _preconditioner_type;
    ceres::SparseLinearAlgebraLibraryType _sparse_linear_algebra_library_type;

    BA_options(const bool bVerbose = true, bool bmultithreaded = true);
    void setDenseBA();
    void setSparseBA();
    void enableParametersOrdering() {useParametersOrdering = true;}
    void disableParametersOrdering() {useParametersOrdering = false;}
    void enableLocalBA() {useLocalBA = true;}
    void disableLocalBA() {useLocalBA = false;}
    bool isLocalBAEnabled() {return useLocalBA;}
  };
  private:
    BA_options _openMVG_options;

  public:
  Bundle_Adjustment_Ceres(Bundle_Adjustment_Ceres::BA_options options = BA_options());

  /**
   * @see Bundle_Adjustment::Adjust
   */
  bool Adjust(
    SfM_Data & sfm_data,
    BA_Refine refineOptions = BA_REFINE_ALL);

  // Used for Local BA strategy: 
  enum LocalBAState{ refined, constant, ignored };
  std::map<IndexT, int> map_viewId_distanceToRecentCameras;
  std::map<IndexT, int> map_poseId_distanceToRecentCameras;
  std::map<IndexT, LocalBAState> map_poseId_BAState;
  std::map<IndexT, LocalBAState> map_intrinsicId_BAState;
  std::map<IndexT, LocalBAState> map_landmarkId_BAState;
  
  LocalBAState getPoseBAState(const IndexT poseId) {return map_poseId_BAState.find(poseId)->second;}
  LocalBAState getIntrinsicsBAState(const IndexT intrinsicId) {return map_intrinsicId_BAState.find(intrinsicId)->second;}
  LocalBAState getLandmarkBAState(const IndexT landmarkId) {return map_landmarkId_BAState.find(landmarkId)->second;}
    
  void setMapDistancePerViewId(const std::map<IndexT, int>& map) {map_viewId_distanceToRecentCameras = map;}
  void setMapDistancePerPoseId(const std::map<IndexT, int>& map) {map_poseId_distanceToRecentCameras = map;}
  void applyRefinementRules(const SfM_Data & sfm_data, const IndexT strategyId=0, const std::size_t distanceLimit=1);

  /**
   * @see Bundle_Adjustment::AdjustPartialReconstruction
   * @brief Ajust parameters according to the reconstruction graph or refine everything
   * if graph is empty. 
   */
  bool adjustPartialReconstruction(SfM_Data & sfm_data, BAStats &baStats);

  private:

  void setSolverOptions(ceres::Solver::Options& solver_options);

  Hash_Map<IndexT, std::vector<double>> addPosesToCeresProblem(
    const Poses & poses, 
    ceres::Problem & problem, 
    ceres::Solver::Options &options,
    BAStats& baStats);
    
  Hash_Map<IndexT, std::vector<double>> addIntrinsicsToCeresProblem(
    const SfM_Data & sfm_data, 
    ceres::Problem & problem,
    ceres::Solver::Options& options,
    BAStats& baStats);
  
  bool solveBA(
    ceres::Problem & problem, 
    ceres::Solver::Options &options, 
    ceres::Solver::Summary &summary);

  void updateCameraPoses(
    const Hash_Map<IndexT, std::vector<double>> & map_poses,
    Poses & poses);
    
  void updateCameraIntrinsics(
    const Hash_Map<IndexT, std::vector<double>> & map_intrinsics,
    Intrinsics & intrinsics);

};

} // namespace sfm
} // namespace openMVG

#endif // OPENMVG_SFM_DATA_BA_CERES_HPP
