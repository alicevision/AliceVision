// Copyright (c) 2015 Pierre Moulon.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef OPENMVG_SFM_DATA_BA_LOCAL_CERES_HPP
#define OPENMVG_SFM_DATA_BA_LOCAL_CERES_HPP

#include "openMVG/sfm/sfm_data_BA_ceres.hpp"
#include "openMVG/sfm/sfm_data_localBA.hpp"
#include "openMVG/tracks/tracks.hpp"
#include "lemon/bfs.h"

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
      _useParametersOrdering = false;
      _useLocalBA = false;
    }

    bool _useParametersOrdering; 
    void enableParametersOrdering() {_useParametersOrdering = true;}
    void disableParametersOrdering() {_useParametersOrdering = false;}
    bool isParameterOrderingEnabled() {return _useParametersOrdering;}
    
    bool _useLocalBA;
    void enableLocalBA() {_useLocalBA = true;}
    void disableLocalBA() {_useLocalBA = false;}
    bool isLocalBAEnabled() {return _useLocalBA;}
  };
  
  Local_Bundle_Adjustment_Ceres(Local_Bundle_Adjustment_Ceres::LocalBA_options options = LocalBA_options());

  /**
   * @see Bundle_Adjustment::AdjustPartialReconstruction
   * @brief Ajust parameters according to the reconstruction graph or refine everything
   * if graph is empty. 
   */
  bool Adjust(SfM_Data & sfm_data);
  bool AdjustNoChanges(const SfM_Data & sfm_data2);

  
  /// @brief Complete the graph '_reconstructionGraph' with new poses
  void updateGraph(
    const SfM_Data& sfm_data, 
    const tracks::TracksPerView& map_tracksPerView,
    const std::set<IndexT>& newViewIds, 
    std::map<IndexT, lemon::ListGraph::Node>& map_viewId_node, 
    lemon::ListGraph& graph_poses);
    
  /// \brief Add the newly resected views 'newViewsIds' into a graph (nodes: cameras, egdes: matching)
  /// and compute the intragraph-distance between these new cameras and all the others.
  void computeDistancesMaps(
    const lemon::ListGraph &graph_poses, 
    const std::map<IndexT, lemon::ListGraph::Node> &map_viewId_node, 
    const SfM_Data& sfm_data, 
    const std::set<IndexT>& newViewIds, 
    std::map<IndexT, int> &outMapPoseIdDistance);

//  /// \brief  A 'LocalBAStrategy' defined the state (refined, constant or ignored) of each parameter 
//  /// of the reconstruction (landmarks, poses & intrinsics) in the BA solver according to the 
//  /// distances graph 'reconstructionGraph'.
//  /// Each strategy is explicitly coded in the 'computeStatesMaps()' method.
//  enum LocalBAStrategy { 
//    strategy_1, ///< 
//    strategy_2, ///< 
//    strategy_3, ///< 
//    strategy_4, ///< 
//    none        ///< Everything is refined (= no Local BA)
//  };
  
//  /// \brief Define the state of each parameter (landmarks, poses & intrinsics) according to the 
//  /// distance graph and the wished Local BA strategy 'LocalBAStrategy'.
//  void computeStatesMaps(const SfM_Data & sfm_data, 
//    const LocalBAStrategy& strategy, 
//    const std::size_t distanceLimit, 
//    const std::set<IndexT>& newReconstructedViewIds );

  void computeStatesMaps_strategy1(const SfM_Data & sfm_data,  const std::size_t distanceLimit);

  void computeStatesMaps_strategy2(const SfM_Data & sfm_data, const std::size_t distanceLimit);

  void computeStatesMaps_strategy3(const SfM_Data & sfm_data, const std::set<IndexT> &newReconstructedViewIds);

  void computeStatesMaps_strategy4(const SfM_Data & sfm_data, LocalBA_Data& lba_data, const std::set<IndexT> &newReconstructedViewIds);

  
  void setBAStatisticsContainer(LocalBA_stats& baStats) {_LBA_statistics = baStats;}

  /// \brief Export statistics about bundle adjustment in a TXT file ("BaStats.txt")
  /// The contents of the file have been writen such that it is easy to handle it with
  /// a Python script or any spreadsheets (e.g. by copy/past the full content to LibreOffice) 
  bool exportStatistics(const std::string& path, const SfM_Data &sfm_data);
  
private:

  // Used for Local BA approach: 
  LocalBA_options _LBA_openMVG_options;
  LocalBA_stats _LBA_statistics;
  
  // Used to generated & handle the distance graph
//  lemon::ListGraph _reconstructionGraph;
//  lemon::ListGraph::NodeMap<IndexT> _map_node_viewId; // <node, viewId>
//  std::map<IndexT, lemon::ListGraph::Node> _map_viewId_node; // <viewId, node>
  
  // Store the graph-distances to the new poses/views. 
  // If the view/pose is not connected to the new poses/views, its distance is -1.
  std::map<IndexT, int> _map_viewId_distance;
  std::map<IndexT, int> _map_poseId_distance;
  
  // Define the state of the all parameter of the reconstruction (structure, poses, intrinsics) in the BA:
  enum LocalBAState { 
    refined,  //< will be adjuted by the BA solver
    constant, //< will be set as constant in the sover
    ignored   //< will not be set into the BA solver
  };
  
  // Store the LocalBAState of each parameter (structure, poses, intrinsics) :
  std::map<IndexT, LocalBAState> _map_poseId_LBAState;
  std::map<IndexT, LocalBAState> _map_intrinsicId_LBAState;
  std::map<IndexT, LocalBAState> _map_landmarkId_LBAState;
  
  // Get back the 'LocalBAState' for a specific parameter :
  LocalBAState getPoseState(const IndexT poseId)            {return _map_poseId_LBAState.find(poseId)->second;}
  LocalBAState getIntrinsicsState(const IndexT intrinsicId) {return _map_intrinsicId_LBAState.find(intrinsicId)->second;}
  LocalBAState getLandmarkState(const IndexT landmarkId)    {return _map_landmarkId_LBAState.find(landmarkId)->second;}


  void setSolverOptions(ceres::Solver::Options& solver_options);

  // Create a parameter block for each pose according to the Ceres format: [Rx, Ry, Rz, tx, ty, tz]
  Hash_Map<IndexT, std::vector<double>> addPosesToCeresProblem(
    const Poses & poses, 
    ceres::Problem & problem);
    
  // Create a parameter block for each intrinsic according to the Ceres format
  Hash_Map<IndexT, std::vector<double>> addIntrinsicsToCeresProblem(
    const SfM_Data & sfm_data, 
    ceres::Problem & problem);
  
  // Run the Ceres solver
  bool solveBA(
    ceres::Problem & problem, 
    ceres::Solver::Options &options, 
    ceres::Solver::Summary &summary);

  // Update camera poses with refined data
  void updateCameraPoses(
    const Hash_Map<IndexT, std::vector<double>> & map_poses,
    Poses & poses);
    
  // Update camera intrinsics with refined data
  void updateCameraIntrinsics(
    const Hash_Map<IndexT, std::vector<double>> & map_intrinsics,
    Intrinsics & intrinsics);
};

} // namespace sfm
} // namespace openMVG

#endif // OPENMVG_SFM_DATA_BA_LOCAL_CERES_HPP
