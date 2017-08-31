// Copyright (c) 2015 Pierre Moulon.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef OPENMVG_SFM_DATA_BA_LOCAL_CERES_HPP
#define OPENMVG_SFM_DATA_BA_LOCAL_CERES_HPP

//#include "openMVG/sfm/sfm_data.hpp"
//#include "openMVG/sfm/sfm_data_BA.hpp"
#include "openMVG/sfm/sfm_data_BA_ceres.hpp"
//#include "openMVG/sfm/sfm_data_BA_ceres_camera_functor.hpp"
//#include "ceres/ceres.h"

namespace openMVG {
namespace sfm {

class Local_Bundle_Adjustment_Ceres : public Bundle_Adjustment_Ceres
{
  public:
  struct LocalBA_options : public Bundle_Adjustment_Ceres::BA_options
  {
  
    LocalBA_options(const bool bVerbose = true, bool bmultithreaded = true) 
      : Bundle_Adjustment_Ceres::BA_options(bVerbose, bmultithreaded)
    {
      useParametersOrdering = false;
      useLocalBA = false;
    }

    bool useParametersOrdering; 
    void enableParametersOrdering() {useParametersOrdering = true;}
    void disableParametersOrdering() {useParametersOrdering = false;}
    bool isParameterOrderingEnabled() {return useParametersOrdering;}
    
    bool useLocalBA;
    void enableLocalBA() {useLocalBA = true;}
    void disableLocalBA() {useLocalBA = false;}
    bool isLocalBAEnabled() {return useLocalBA;}
  };
  
  private:
    LocalBA_options LBA_openMVG_options;
    LocalBA_stats LBA_stats;

  public:
  Local_Bundle_Adjustment_Ceres(Local_Bundle_Adjustment_Ceres::LocalBA_options options = LocalBA_options());

  /**
   * @see Bundle_Adjustment::AdjustPartialReconstruction
   * @brief Ajust parameters according to the reconstruction graph or refine everything
   * if graph is empty. 
   */
  bool Adjust(SfM_Data & sfm_data);

  // Used for Local BA strategy: 
  enum LocalBAState{ refined, constant, ignored };
  std::map<IndexT, std::size_t> map_viewId_distanceToRecentCameras;
  std::map<IndexT, std::size_t> map_poseId_distanceToRecentCameras;
  std::map<IndexT, LocalBAState> map_poseId_BAState;
  std::map<IndexT, LocalBAState> map_intrinsicId_BAState;
  std::map<IndexT, LocalBAState> map_landmarkId_BAState;
  
  LocalBAState getPoseBAState(const IndexT poseId)            {return map_poseId_BAState.find(poseId)->second;}
  LocalBAState getIntrinsicsBAState(const IndexT intrinsicId) {return map_intrinsicId_BAState.find(intrinsicId)->second;}
  LocalBAState getLandmarkBAState(const IndexT landmarkId)    {return map_landmarkId_BAState.find(landmarkId)->second;}
    
  void setMapDistancePerViewId(const std::map<IndexT, std::size_t>& map) {map_viewId_distanceToRecentCameras = map;}
  void setMapDistancePerPoseId(const std::map<IndexT, std::size_t>& map) {map_poseId_distanceToRecentCameras = map;}
  void applyRefinementRules(const SfM_Data & sfm_data, const IndexT strategyId=0, const std::size_t distanceLimit=1);
  
  void setBAStatisticsContainer(LocalBA_stats& baStats) {LBA_stats = baStats;}

  /// Export statistics about bundle adjustment in a TXT file ("BaStats.txt")
  /// The contents of the file have been writen such that it is easy to handle it with
  /// a Python script or any spreadsheets (e.g. by copy/past the full content to LibreOffice) 
  bool exportStatistics(const std::string& path);
  
  private:

  void setSolverOptions(ceres::Solver::Options& solver_options);

  Hash_Map<IndexT, std::vector<double>> addPosesToCeresProblem(const Poses & poses, 
    ceres::Problem & problem);
    
  Hash_Map<IndexT, std::vector<double>> addIntrinsicsToCeresProblem(
    const SfM_Data & sfm_data, 
    ceres::Problem & problem);
  
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

#endif // OPENMVG_SFM_DATA_BA_LOCAL_CERES_HPP
