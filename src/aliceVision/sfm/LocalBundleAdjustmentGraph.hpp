// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/types.hpp>
#include <aliceVision/track/TracksBuilder.hpp>
#include <aliceVision/sfm/BundleAdjustment.hpp>

#include <lemon/list_graph.h>


namespace aliceVision {

namespace sfmData {
class SfMData;
} // namespace sfmData

namespace sfm {

/**
 * @brief The LocalBundleAdjustmentGraph class
 * Contains all the data needed to apply a Local Bundle Adjustment.
 */
class LocalBundleAdjustmentGraph
{
  
public:

  explicit LocalBundleAdjustmentGraph(const sfmData::SfMData& sfmData);

  /**
   * @brief Return the number of posed views for each graph-distance
   * @return map<distance, numViews>
   */
  std::map<int, std::size_t> getDistancesHistogram() const;
    
  /**
   * @brief Return the BundleAdjustment::EParameterState for a specific pose.
   * @param[in] poseId The given pose Id
   * @return BundleAdjustment::EParameterState
   */
  inline BundleAdjustment::EParameterState getPoseState(const IndexT poseId) const
  {
    return _statePerPoseId.at(poseId);
  }
 
  /**
   * @brief Return the BundleAdjustment::EParameterState for a specific intrinsic.
   * @param[in] intrinsicId The given intrinsic Id
   * @return BundleAdjustment::EParameterState
   */
  inline BundleAdjustment::EParameterState getIntrinsicState(const IndexT intrinsicId) const
  {
    return _statePerIntrinsicId.at(intrinsicId);
  }

  /**
   * @brief Return the BundleAdjustment::EParameterState for a specific landmark.
   * @param[in] landmarkId The given landmark Id
   * @return BundleAdjustment::EParameterState
   */
  inline BundleAdjustment::EParameterState getLandmarkState(const IndexT landmarkId) const
  {
    return _statePerLandmarkId.at(landmarkId);
  }

  /**
   * @brief Return the number of poses with the given BundleAdjustment::EParameterState.
   * @param[in] state The given BundleAdjustment::EParameterState.
   * @return number of poses with the given BundleAdjustment::EParameterState.
   */
  inline std::size_t getNbPosesPerState(BundleAdjustment::EParameterState state) const
  {
    std::size_t nb = 0;
    for(const auto& poseStatePair : _statePerPoseId)
      if(poseStatePair.second == state)
        ++nb;
    return nb;
  }

  /**
   * @brief Return the number of intrinsics with the given BundleAdjustment::EParameterState.
   * @param[in] state The given BundleAdjustment::EParameterState.
   * @return number of intrinsics with the given BundleAdjustment::EParameterState.
   */
  inline std::size_t getNbIntrinsicsPerState(BundleAdjustment::EParameterState state) const
  {
    std::size_t nb = 0;
    for(const auto& intrinsicStatePair : _statePerIntrinsicId)
      if(intrinsicStatePair.second == state)
        ++nb;
    return nb;
  }

  /**
   * @brief Return the number of landmarks with the given BundleAdjustment::EParameterState.
   * @param[in] state The given BundleAdjustment::EParameterState.
   * @return number of landmarks with the given BundleAdjustment::EParameterState.
   */
  inline std::size_t getNbLandmarksPerState(BundleAdjustment::EParameterState state) const
  {
    std::size_t nb = 0;
    for(const auto& landmarkStatePair : _statePerLandmarkId)
      if(landmarkStatePair.second == state)
        ++nb;
    return nb;
  }

  /**
   * @brief Get the graph-distance limit setting the Active region
   * @return graph-distance limit setting
   */
  inline std::size_t getGraphDistanceLimit() const
  {
    return _graphDistanceLimit;
  }

  /**
   * @brief Set the graph-distance limit setting the Active region
   * @param[in] graph-distance limit setting
   */
  inline void setGraphDistanceLimit(const std::size_t limit)
  {
    _graphDistanceLimit = limit;
  }
    
  /**
   * @brief Set every parameters of the BA problem to Refine: the Local BA becomes a classic BA.
   * @param[in] sfmData contains all the information about the reconstruction
   */
  void setAllParametersToRefine(const sfmData::SfMData& sfmData);
 
  /**
   * @brief Save all the intrinsics to the memory to retain the evolution of each focal length during the reconstruction.
   * @param[in] sfmData contains all the information about the reconstruction notably current focal lengths
   */
  void saveIntrinsicsToHistory(const sfmData::SfMData& sfmData);
  
  /**
   * @brief Export the history of each intrinsics. It create a file \a K_<intrinsic_index>.csv in \c folder.
   * @param [in] folder The folder where you want to save the intrinsic history file
   * @param [in] filename  The filename of the intrinsic history file
   */
  void exportIntrinsicsHistory(const std::string& folder, const std::string& filename);

  /**
   * @brief Remove specific views from the LocalBA graph. 
   * @details Delete the nodes corresponding to those views and all their incident edges.
   * @param[in] sfmData contains all the information about the reconstruction
   * @param[in] removedViewsId Set of views index to remove
   * @return true if the number of removed node is equal to the size of \c removedViewsId
   */
  bool removeViews(const sfmData::SfMData& sfmData, const std::set<IndexT>& removedViewsId);

  /**
   * @brief Complete the graph with the newly resected views or all the posed views if the graph is empty.
   * @param[in] sfmData contains all the information about the reconstruction
   * @param[in] map_tracksPerView A map giving the tracks for each view
   * @param[in] newReconstructedViews The list of the newly resected views
   * @param[in] kMinNbOfMatches The min. number of shared matches to create an edge between two views (nodes)
   */
  void updateGraphWithNewViews(const sfmData::SfMData& sfmData,
      const track::TracksPerView& map_tracksPerView, 
      const std::set<IndexT>& newImageIndex,
      const std::size_t kMinNbOfMatches = 50);
  
  /**
   * @brief Compute the intragraph-distance between all the nodes of the graph (posed views) and the newly resected views.
   * @details The graph-distances are computed using a Breadth-first Search (BFS) method.
   * @param[in] sfmData contains all the information about the reconstruction, notably the posed views
   * @param[in] newReconstructedViews The list of the newly resected views used (used as source in the BFS algorithm)
   */
  void computeGraphDistances(const sfmData::SfMData& sfmData, const std::set<IndexT>& newReconstructedViews);
  
  /**
   * @brief  Use the graph-distances of each posed view to set each parameter of the problem (poses, intrinsics, landmarks):
   *     - as Refined (will be refined during the adjustment)
   *     - as Constant (will be set as constant in the adjustment)
   *     - as Ignored (won't be used during th adjustment).
   * @details Here is the algorithm assignating a state at each parameter:
   *     \b D = the distance of the 'active' region (\c kLimitDistance)
   *     \b W = the range (in term of num. of poses) on which to study the focal length variations (kWindowSize)
   *     \b L = the max. percentage of the variations of the focal length to consider it as constant (kStdevPercentage)
   *      - a Pose is set to:
   *        - \a Refined <=> dist <= D
   *        - \a Constant <=> dist == D+1
   *        - \a Ignored <=> else (dist = -1 U [D+2; +inf.[)
   *      - an Intrinsic is set to:
   *        - \a Refined by default
   *        - \a Constant <=> its focal lenght is considered as stable in its W last saved values according to all of its values.
   *     - a Landmarks is set to:
   *        - \a Ignored by default
   *        - \a Refined <=> its connected to a refined camera
   * @param[in] sfmData contains all the information about the reconstruction
   */
  void convertDistancesToStates(const sfmData::SfMData& sfmData);

  /**
   * @brief Update rigs edges.
   * @param[in] sfmData contains all the information about the reconstruction
   * @return
   */
  std::size_t updateRigEdgesToTheGraph(const sfmData::SfMData& sfmData);

  /**
   * @brief Count and return the number of nodes in the underlying lemon graph.
   * @return The number of nodes in the graph.
   */
  unsigned int countNodes() const;

  /**
   * @brief Count and return the number of edges in the underlying lemon graph.
   * @return The number of edges in the graph.
   */
  unsigned int countEdges() const;

private:
  
  /**
   * @brief Return the distance between a specific pose and the new posed views.
   * @param[in] poseId is the index of the poseId
   * @return Return \c -1 if the pose is not connected to any new posed view.
   */
  int getPoseDistance(const IndexT poseId) const;

  /**
   * @brief Return the distance between a specific view and the new posed views.
   * @param[in] viewId is the index of the view
   * @return Return \c -1 if the view is not connected to any new posed view.
   */
  int getViewDistance(const IndexT viewId) const;

  /**
   * @brief Return the state for a given distance
   * @param[in] distance between two views
   * @return BundleAdjustment::EParameterState
   */
  BundleAdjustment::EParameterState getStateFromDistance(int distance) const;

  /**
   * @brief Draw the current graph in the given directory.
   * @details The file is name \a graph_<numOfNodes>_<nameComplement>.
   * Node format: [<viewId>: D<distance> K<intrinsics>].
   * Node color: red (D=0), green (D=1), blue (D=2) or black (D>2 or D=-1)
   * Edge color: black (classic) or red (due to the intrinsic edges)
   * @param[in] sfmData contains all the information about the reconstruction
   * @param[in] dir
   * @param[in] nameComplement
   */
  void drawGraph(const sfmData::SfMData& sfmData, const std::string& folder, const std::string& nameComplement = "");
  
  /**
   * @brief Compute, for each camera the variation of the last \a windowSize values of the focal length.
   *  If the focal lenght variations are considered as enought constant the function updates \a _mapFocalIsConstant.
   * @details Pipeline:
   *    \b H: the history of all the focal length for a given intrinsic
   *    \b S: the subpart of H including the last \a wondowSize values only.
   *    \b sigma = stddev(S)
   *    \b sigma_normalized = sigma / (max(H) - min(H))
   *    \c if sigma_normalized < \a stdevPercentageLimit \c then the limit is reached.
   * @param[in] windowSize Compute the variation on the \a windowSize parameters
   * @param[in] stdevPercentageLimit The limit is reached when the standard deviation of the \a windowSize values is less than \a stdevPecentageLimit % of the range of all the values.
   */
  void checkFocalLengthsConsistency(const std::size_t windowSize, const double stdevPercentageLimit);
  
  /**
   * @brief Count the number of shared landmarks between all the new views and each already resected cameras.
   * @param[in] sfmData contains all the information about the reconstruction
   * @param[in] map_tracksPerView A map giving the tracks for each view
   * @param[in] newViewsId A set with the views index that we want to count matches with resected cameras.
   * @return A map giving the number of matches for each images pair.
   */
  static std::vector<Pair> getNewEdges(const sfmData::SfMData& sfmData,
      const track::TracksPerView& map_tracksPerView,
      const std::set<IndexT>& newViewsId,
      const std::size_t minNbOfMatches,
      const std::size_t minNbOfEdgesPerView);
  
  /**
   * @brief Return the state of the focal length (constant or not) for a specific intrinsic.
   * @param intrinsicId To update the focal lengths states, use \c LocalBundleAdjustmentGraph::checkFocalLengthsConsistency()
   * @return true if the focal length is considered as Constant
   */
  inline bool isFocalLengthConstant(const IndexT intrinsicId) const
  {
    return _mapFocalIsConstant.at(intrinsicId);
  }
    
  /**
   * @brief Compute the standard deviation.
   * @param[in] The vector of values
   * @return The standard deviation
   */
  template<typename T> 
  static double standardDeviation(const std::vector<T>& data);

  /**
   * @brief Add an edge in the graph when 2 views share a same intrinsic not considered as Constant
   * @param[in] sfmData contains all the information about the reconstruction
   * @param[in] newReconstructedViews
   * @return
   */
  std::size_t addIntrinsicEdgesToTheGraph(const sfmData::SfMData& sfmData, const std::set<IndexT>& newReconstructedViews);
  
  /**
   * @brief Remove all the edges added by the \c addIntrinsicEdgesToTheGraph function related to .
   * @param[in] intrinsicId
   */
  void removeIntrinsicEdgesFromTheGraph(IndexT intrinsicId);

  // Distances data
  // - Local BA needs to know the distance of all the old posed views to the new resected views.
  // - The bundle adjustment will be processed on the closest poses only.

  /// A graph where nodes are poses and an edge exists when 2 poses shared at least 'kMinNbOfMatches' matches.
  lemon::ListGraph _graph; 
  /// The graph-distance limit setting the Active region (default value: 1)
  std::size_t _graphDistanceLimit = 1;
  /// Associates each view (indexed by its viewId) to its corresponding node in the graph.
  std::map<IndexT, lemon::ListGraph::Node> _nodePerViewId;
  /// Associates each node (in the graph) to its corresponding view.
  std::map<lemon::ListGraph::Node, IndexT> _viewIdPerNode;
  /// Store the graph-distances from the new views (0: is a new view, -1: is not connected to the new views)
  std::map<IndexT, int> _distancePerViewId;
  /// Store the graph-distances from the new poses (0: is a new pose, -1: is not connected to the new poses)
  std::map<IndexT, int> _distancePerPoseId;
  /// Store the \c EParameterState of each pose in the scene.
  std::map<IndexT, BundleAdjustment::EParameterState> _statePerPoseId;
  /// Store the \c EParameterState of each intrinsic in the scene.
  std::map<IndexT, BundleAdjustment::EParameterState> _statePerIntrinsicId;
  /// Store the \c EParameterState of each landmark in the scene.
  std::map<IndexT, BundleAdjustment::EParameterState> _statePerLandmarkId;
  
  // Intrinsics data
  // - Local BA needs to know the evolution of all the intrinsics parameters.
  // - When camera parameters are enought refined (no variation) they are set to constant in the BA.

  struct IntrinsicHistory
  {
    std::size_t nbPoses = 0;
    double focalLength = 0.0;
    bool isConstant = false;
  };

  /**
   * @brief Save the progression for all the intrinsics parameters
   * @details <IntrinsicId, std::vector<std::pair<NumOfPosesCamerasWithThisIntrinsic, FocalLengthHistory>
   *       K1:
   *        0 1200
   *        1 1250
   *        ...
   *       K2:
   *        ...
   */
  using IntrinsicsHistory = std::map<IndexT, std::vector<IntrinsicHistory>>;
  
  /// Backup of the intrinsics focal length values
  IntrinsicsHistory _intrinsicsHistory;

  /**
   * @brief Indicates, for each intrinsic, if its focallength has been concidered as constant.
   * <IntrinsicId, isConsideredAsConstant>
   */
  std::map<IndexT, bool> _mapFocalIsConstant;

  /**
   * @brief Store the Lemon index of the edges added for the intrinsic links "the intrinsic-edges"
   * <IntrinsicId, [edgeId]>
   */
  std::map<IndexT, std::vector<int>> _intrinsicEdgesId;

  /**
   * @brief Store the Lemon index of the edges added for the rig links "the intrinsic-edges"
   * <rigId, [edgeId]>
   */
  std::map<IndexT, std::vector<int>> _rigEdgesId;
};

} // namespace sfm
} // namespace aliceVision
