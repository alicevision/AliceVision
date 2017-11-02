// Copyright (c) 2015 Pierre Moulon.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef OPENMVG_SFM_DATA_LOCALBA_HPP
#define OPENMVG_SFM_DATA_LOCALBA_HPP

#include "openMVG/types.hpp"
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/tracks/tracks.hpp"
#include "lemon/list_graph.h"
#include "lemon/bfs.h"
#include "openMVG/stl/stlMap.hpp"
#include "openMVG/system/timer.hpp"


namespace openMVG {
namespace sfm {

/// Contains all the data needed to apply a Local Bundle Adjustment.
class LocalBA_Data
{
  
public:

  /// Defines the state of the all parameter of the reconstruction (structure, poses, intrinsics) in the BA:
  enum EState { 
    refined,  ///< will be adjuted by the BA solver
    constant, ///< will be set as constant in the sover
    ignored   ///< will not be set into the BA solver
  };
 
  // -- Constructor
  
  LocalBA_Data(const SfM_Data& sfm_data);
  
  // -- Getters
  
  /// @brief Return the distance between a specific pose and the new posed views.
  /// @param[in] poseId is the index of the poseId
  /// @details Return \c -1 if the pose is not connected to any new posed view.
  int getPoseDistance(const IndexT poseId) const;
  
  /// @brief Return the distance between a specific view and the new posed views.
  /// @param[in] viewId is the index of the view
  /// @details Return \c -1 if the view is not connected to any new posed view.
  int getViewDistance(const IndexT viewId) const;
  
  /// Return the number of posed views for each graph-distance.
  std::map<int, std::size_t> getDistancesHistogram() const;
    
    
  /// Return the \c EState for a specific pose.
  EState getPoseState(const IndexT poseId) const           {return _mapLBAStatePerPoseId.at(poseId);}
 
  /// Return the \c EState for a specific intrinsic.
  EState getIntrinsicState(const IndexT intrinsicId) const {return _mapLBAStatePerIntrinsicId.at(intrinsicId);}

  /// Return the \c EState for a specific landmark.
  EState getLandmarkState(const IndexT landmarkId) const   {return _mapLBAStatePerLandmarkId.at(landmarkId);}

  
  /// Return the number of refined poses.
  std::size_t getNumOfRefinedPoses() const        {return getNumberOf(LocalBA_Data::EParameter::pose, LocalBA_Data::EState::refined);}
 
  /// Return the number of constant poses.
  std::size_t getNumOfConstantPoses() const       {return getNumberOf(LocalBA_Data::EParameter::pose, LocalBA_Data::EState::constant);}

  /// Return the number of ignored poses.
  std::size_t getNumOfIgnoredPoses() const        {return getNumberOf(LocalBA_Data::EParameter::pose, LocalBA_Data::EState::ignored);}

  /// Return the number of refined landmarks.
  std::size_t getNumOfRefinedLandmarks() const    {return getNumberOf(LocalBA_Data::EParameter::landmark, LocalBA_Data::EState::refined);}
 
  /// Return the number of constant landmarks.
  std::size_t getNumOfConstantLandmarks() const   {return getNumberOf(LocalBA_Data::EParameter::landmark, LocalBA_Data::EState::constant);}
 
  /// Return the number of ignored landmarks.
  std::size_t getNumOfIgnoredLandmarks() const    {return getNumberOf(LocalBA_Data::EParameter::landmark, LocalBA_Data::EState::ignored);}
 
  /// Return the number of refined intrinsics.
  std::size_t getNumOfRefinedIntrinsics() const   {return getNumberOf(LocalBA_Data::EParameter::intrinsic, LocalBA_Data::EState::refined);}
 
  /// Return the number of constant intrinsics.
  std::size_t getNumOfConstantIntrinsics() const  {return getNumberOf(LocalBA_Data::EParameter::intrinsic, LocalBA_Data::EState::constant);}

  /// Return the number of ignored intrinsics.
  std::size_t getNumOfIgnoredIntrinsics() const   {return getNumberOf(LocalBA_Data::EParameter::intrinsic, LocalBA_Data::EState::ignored);}
  
  // -- Methods
  
  /// @brief Save all the focal lengths to the memory to retain the evolution of each focal length during the reconstruction.
  /// @param[in] sfm_data contains all the information about the reconstruction, notably current focal lengths
  void saveFocalLengths(const SfM_Data& sfm_data);
  
  /// @brief Export the history of each focal length. It create a file \a K<intrinsic_index>.txt in \c folder.
  /// @param[in] folder The folder in which the \a K*.txt files are saved.
  void exportFocalLengths(const std::string& folder);
  
  /// @brief Complete the graph with the newly resected views or all the posed views if the graph is empty.
  /// @param[in] sfm_data 
  /// @param[in] map_tracksPerView A map giving the tracks for each view
  /// @param[in] newReconstructedViews The list of the newly resected views
  /// @param[in] kMinNbOfMatches The min. number of shared matches to create an edge between two views (nodes)
  void updateGraphWithNewViews(const SfM_Data& sfm_data, 
      const tracks::TracksPerView& map_tracksPerView, 
      const std::set<IndexT> &newReconstructedViews, 
      const std::size_t kMinNbOfMatches = 50);
  
  /// @brief Remove some views to the graph. It delete the node and all the incident edges for each removed view.
  /// @param[in] removedViewsId Set of views index to remove
  /// @return true if the number of removed node is equal to the size of \c removedViewsId
  bool removeViewsToTheGraph(const std::set<IndexT>& removedViewsId);
  
  /// @brief Compute the intragraph-distance between all the nodes of the graph (posed views) and the newly resected
  /// views.
  /// @details The graph-distances are computed using a Breadth-first Search (BFS) method.
  /// @param[in] sfm_data contains all the information about the reconstruction, notably the posed views
  /// @param[in] newReconstructedViews The list of the newly resected views used (used as source in the BFS algorithm)
  void computeGraphDistances(const SfM_Data& sfm_data, const std::set<IndexT> &newReconstructedViews);
  
  /// @brief Use the graph-distances of each posed view to set each parameter of the problem (poses, intrinsics, landmarks)
  /// as Refined (will be refined during the adjustment), Constant (will be set as constant in the adjustment) 
  /// or Ignored (won't be used during th adjustment).
  /// @details Here is the algorithm assignating a state at each parameter : 
  ///     \b D = the distance of the 'active' region (\c kLimitDistance)
  ///     \b W = the range (in term of num. of poses) on which to study the focal length variations (kWindowSize)
  ///     \b L = the max. percentage of the variations of the focal length to consider it as constant (kStdevPercentage)
  ///     - a Pose is set to:
  ///       - \a Refined <=> dist <= D
  ///       - \a Constant <=> dist == D+1
  ///       - \a Ignored <=> else (dist = -1 U [D+2; +inf.[)
  ///     - an Intrinsic is set to:
  ///       - \a Refined by default
  ///       - \a Constant <=> its focal lenght is considered as stable in its W last saved values
  ///         according to all of its values.                       
  ///     - a Landmarks is set to:
  ///       - \a Ignored by default
  ///       - \a Refined <=> its connected to a refined camera
  /// @param[in] sfm_data
  /// @param[in] kLimitDistance the distance of the active region
  void convertDistancesToLBAStates(const SfM_Data & sfm_data, const std::size_t kLimitDistance);
  
  /// @brief Add an edge in the graph when 2 views share a same intrinsic not considered as Constant
  /// @details (no longer used)
  std::size_t addIntrinsicEdgesToTheGraph(const SfM_Data& sfm_data, const std::set<IndexT> &newReconstructedViews);
  
  /// @brief Remove all the edges added by the \c addIntrinsicEdgesToTheGraph function.
  void removeIntrinsicEdgesToTheGraph();

  /// @brief Draw the current graph in the given directory. 
  /// @details The file is name \a graph_<numOfNodes>_<nameComplement>. 
  /// Node format: [<viewId>: D<distance> K<intrinsics>].
  /// Node color: red (D=0), green (D=1), blue (D=2) or black (D>2 or D=-1)
  /// Edge color: black (classic) or red (due to the intrinsic edges)
  /// @param[in] sfm_data 
  /// @param[in] dir 
  /// @param[in] nameComplement 
  void drawGraph(const SfM_Data &sfm_data, const std::string& dir, const std::string& nameComplement = "");
  
private:
   
  /// Defines all the types of parameter adjusted during bundle adjustment.
  enum EParameter { 
    pose,       ///< The pose
    intrinsic,  ///< The intrinsic
    landmark    ///< The landmark
  };
  
  /// Return the number of parameters \c EParameter being in the \c EState state.
  std::size_t getNumberOf(EParameter param, EState state) const {return _parametersCounter.at(std::make_pair(param, state));}
  
  /// @brief Compute, for each camera the variation of the last \a windowSize values of the focal length.
  /// If the focal lenght variations are considered as enought constant the function updates \a _mapFocalIsConstant.
  /// @details Pipeline:
  ///   \b H: the history of all the focal length for a given intrinsic
  ///   \b S: the subpart of H including the last \a wondowSize values only.
  ///   \b sigma = stddev(S)  
  ///   \b sigma_normalized = sigma / (max(H) - min(H))
  ///   \c if sigma_normalized < \a stdevPercentageLimit \c then the limit is reached.
  /// @param[in] windowSize Compute the variation on the \a windowSize parameters
  /// @param[in] stdevPercentageLimit The limit is reached when the standard deviation of the \a windowSize values is less than \a stdevPecentageLimit % of the range of all the values.
  void checkFocalLengthsConsistency(const std::size_t windowSize, const double stdevPercentageLimit);
  
  /// @brief Count the number of shared landmarks between all the new views and each already resected cameras.
  /// @param[in] sfm_data
  /// @param[in] map_tracksPerView
  /// @param[in] newViewsId A set with the views index that we want to count matches with resected cameras. 
  /// @return A map giving the number of matches for each images pair.
  static std::map<Pair, std::size_t> countSharedLandmarksPerImagesPair(
      const SfM_Data& sfm_data,
      const tracks::TracksPerView& map_tracksPerView,
      const std::set<IndexT>& newViewsId);
  
  /// @brief Return the state of the focal length (constant or not) for a specific intrinsic.
  /// @details To update the focal lengths states, use \c LocalBA_Data::checkFocalLengthsConsistency()
  /// @return true if the focal length is considered as Constant
  bool isFocalLengthConstant(const IndexT intrinsicId) const { return _mapFocalIsConstant.at(intrinsicId);}
  
  /// Return the last value of the focal length for a specific intrinsic.
  double getLastFocalLength(const IndexT intrinsicId) const {return _focalLengthsHistory.at(intrinsicId).back().second;}
  
  /// @brief standardDeviation Compute the standard deviation.
  /// @param[in] The vector of values
  /// @return The standard deviation
  template<typename T> 
  static double standardDeviation(const std::vector<T>& data);
  
  // ------------------------
  // - Distances data -
  // Local BA needs to know the distance of all the old posed views to the new resected views.
  // The bundle adjustment will be processed on the closest poses only.
  // ------------------------
  
  /// A graph where nodes are poses and an edge exists when 2 poses shared at least 'kMinNbOfMatches' matches.
  lemon::ListGraph _graph; 
  
  /// Associates each view (indexed by its viewId) to its corresponding node in the graph.
  std::map<IndexT, lemon::ListGraph::Node> _mapNodePerViewId;
  /// Associates each node (in the graph) to its corresponding view.
  std::map<lemon::ListGraph::Node, IndexT> _mapViewIdPerNode;
    
  /// Store the graph-distances from the new views (0: is a new view, -1: is not connected to the new views)
  std::map<IndexT, int> _mapDistancePerViewId;
  /// Store the graph-distances from the new poses (0: is a new pose, -1: is not connected to the new poses)
  std::map<IndexT, int> _mapDistancePerPoseId;
  
  /// Store the \c EState of each pose in the scene.
  std::map<IndexT, EState> _mapLBAStatePerPoseId;
  /// Store the \c EState of each intrinsic in the scene.
  std::map<IndexT, EState> _mapLBAStatePerIntrinsicId;
  /// Store the \c EState of each landmark in the scene.
  std::map<IndexT, EState> _mapLBAStatePerLandmarkId;
  
  /// Store the number of parameter \c EParameter in a specific state \c EState
  std::map<std::pair<EParameter, EState>, int> _parametersCounter;
  
  // ------------------------
  // - Intrinsics data -
  // Local BA needs to know the evolution of all the intrinsics parameters.
  // When camera parameters are enought reffined (no variation) they are set to constant in the BA.
  // ------------------------
  
  /// Save the progression for all the intrinsics parameters
  /// <IntrinsicId, std::vector<std::pair<NumOfPosesCamerasWithThisIntrinsic, FocalLengthHistory>
  /// K1:
  ///   0 1200 
  ///   1 1250
  ///   ...
  /// K2:
  ///   ... 
  using IntrinsicsHistory = std::map<IndexT, std::vector<std::pair<std::size_t, double>>>;
  
  /// Backup of the intrinsics focal length values
  IntrinsicsHistory _focalLengthsHistory; 
  
  /// Store, for each parameter of each intrinsic, the BA's index from which it has been concidered as constant.
  /// <IntrinsicId, isConsideredAsConstant>
  std::map<IndexT, bool> _mapFocalIsConstant; 
  
  /// @brief Store the Lemon index of the edges added thanks to the intrinsics "the intrinsic-edges" 
  /// @details (no longer used)
  std::set<int> _intrinsicEdgesId; 
};


//------------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------
//                                                 LocalBA_statistics      
//------------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------

/// Contain all the information about a Bundle Adjustment loop
struct LocalBA_statistics
{
  LocalBA_statistics(
      const std::set<IndexT>& newlyResectedViewsId = std::set<IndexT>(),
      const std::map<int, std::size_t>& distancesHistogram = std::map<int, std::size_t>()) 
  {
    _newViewsId = newlyResectedViewsId;
    _numCamerasPerDistance = distancesHistogram;
  }
  
  
  // Parameters returned by Ceres:
  double _time = 0.0;                          // spent time to solve the BA (s)
  std::size_t _numSuccessfullIterations = 0;   // number of successfull iterations
  std::size_t _numUnsuccessfullIterations = 0; // number of unsuccessfull iterations
  
  std::size_t _numResidualBlocks = 0;          // num. of resiudal block in the Ceres problem
  
  double _RMSEinitial = 0.0; // sqrt(initial_cost / num_residuals)
  double _RMSEfinal = 0.0;   // sqrt(final_cost / num_residuals)
  
  // Parameters specifically used by Local BA:
  std::size_t _numRefinedPoses = 0;           // number of refined poses among all the estimated views          
  std::size_t _numConstantPoses = 0;          // number of poses set constant in the BA solver
  std::size_t _numIgnoredPoses = 0;           // number of not added poses to the BA solver
  std::size_t _numRefinedIntrinsics = 0;      // num. of refined intrinsics
  std::size_t _numConstantIntrinsics = 0;     // num. of intrinsics set constant in the BA solver
  std::size_t _numIgnoredIntrinsics = 0;      // num. of not added intrinsicsto the BA solver
  std::size_t _numRefinedLandmarks = 0;       // num. of refined landmarks
  std::size_t _numConstantLandmarks = 0;      // num. of landmarks set constant in the BA solver
  std::size_t _numIgnoredLandmarks = 0;       // num. of not added landmarks to the BA solver
  
  std::map<int, std::size_t> _numCamerasPerDistance; // distribution of the cameras for each graph distance
  
  std::set<IndexT> _newViewsId;  // index of the new views added (newly resected)
};



} // namespace sfm
} // namespace openMVG

#endif // OPENMVG_SFM_DATA_LOCAL_HPP
