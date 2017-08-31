// Copyright (c) 2015 Pierre Moulon.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef OPENMVG_SFM_DATA_BA_LOCAL_CERES_HPP
#define OPENMVG_SFM_DATA_BA_LOCAL_CERES_HPP

#include "openMVG/sfm/sfm_data_BA_ceres.hpp"
#include "openMVG/tracks/tracks.hpp"

//#include "lemon/list_graph.h"


namespace openMVG {
namespace sfm {

/**
 * @brief 
 * 
 */
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
  
  Local_Bundle_Adjustment_Ceres(Local_Bundle_Adjustment_Ceres::LocalBA_options options = LocalBA_options());

  /**
   * @see Bundle_Adjustment::AdjustPartialReconstruction
   * @brief Ajust parameters according to the reconstruction graph or refine everything
   * if graph is empty. 
   */
  bool Adjust(SfM_Data & sfm_data);

  void computeDistancesMaps(
    const SfM_Data& sfm_data, 
    const std::set<IndexT>& newViewIds,
    const openMVG::tracks::TracksPerView& map_tracksPerView);
  
//  void setMapDistancePerViewId(const std::map<IndexT, std::size_t>& map) {map_viewId_distanceToRecentCameras = map;}
//  void setMapDistancePerPoseId(const std::map<IndexT, std::size_t>& map) {map_poseId_distanceToRecentCameras = map;}
  void applyRefinementRules(const SfM_Data & sfm_data, const IndexT strategyId=0, const std::size_t distanceLimit=1);
  
  void setBAStatisticsContainer(LocalBA_stats& baStats) {LBA_statistics = baStats;}

  /// Export statistics about bundle adjustment in a TXT file ("BaStats.txt")
  /// The contents of the file have been writen such that it is easy to handle it with
  /// a Python script or any spreadsheets (e.g. by copy/past the full content to LibreOffice) 
  bool exportStatistics(const std::string& path);
  
private:

  // Used for Local BA strategy: 
  // Local BA data
  LocalBA_options LBA_openMVG_options;
  LocalBA_stats LBA_statistics;
  
  lemon::ListGraph _reconstructionGraph;
  lemon::ListGraph::NodeMap<IndexT> _nodeMap; // <node, viewId>
  std::map<IndexT, lemon::ListGraph::Node> _invNodeMap; // <viewId, node>
  
  
  std::map<IndexT, std::size_t> map_viewId_distance;
  std::map<IndexT, std::size_t> map_poseId_distance;
  
  enum LocalBAState{ refined, constant, ignored };
  
  std::map<IndexT, LocalBAState> map_poseId_LBAState;
  std::map<IndexT, LocalBAState> map_intrinsicId_LBAState;
  std::map<IndexT, LocalBAState> map_landmarkId_LBAState;
  
  
  LocalBAState getPoseState(const IndexT poseId)            {return map_poseId_LBAState.find(poseId)->second;}
  LocalBAState getIntrinsicsState(const IndexT intrinsicId) {return map_intrinsicId_LBAState.find(intrinsicId)->second;}
  LocalBAState getLandmarkState(const IndexT landmarkId)    {return map_landmarkId_LBAState.find(landmarkId)->second;}
    
  void updateDistancesGraph(const SfM_Data& sfm_data, 
    const tracks::TracksPerView& map_tracksPerView,
    const std::set<IndexT>& newViewIds);

  void setSolverOptions(ceres::Solver::Options& solver_options);

  Hash_Map<IndexT, std::vector<double>> addPosesToCeresProblem(
    const Poses & poses, 
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
