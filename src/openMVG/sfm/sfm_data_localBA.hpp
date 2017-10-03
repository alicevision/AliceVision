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

namespace openMVG {
namespace sfm {


/// Intrinsic parameters 
/// <NbOfPosesWithACommonIntrinsic, IntrinsicParameters>
using IntrinsicParams = std::pair<std::size_t, std::vector<double>>;

/// Save the progression for all the intrinsics parameters
/// <IntrinsicId, IntrinsicsParametersHistory>
using IntrinicsHistory = std::map<IndexT, std::vector<IntrinsicParams>>;

/// Contains all the data needed to apply a Local Bundle Adjustment.
class LocalBA_Data
{
  
public:
  
  // -- Constructor
  
  LocalBA_Data(const SfM_Data& sfm_data);
  
  // -- Getters
  
  int getPoseDistance(const IndexT poseId) const;
  
  int getViewDistance(const IndexT viewId) const;
  
  std::set<IndexT> getNewViewsId() const {return set_newViewsId;}
  
  // -- Setters
  
  void setNewViewsId(const std::set<IndexT>& newPosesId) {set_newViewsId = newPosesId;}
  
  // -- Methods
  
  /// @brief addIntrinsicsToHistory Add the current intrinsics of the reconstruction in the intrinsics history.
  /// @param[in] sfm_data Contains all the information about the reconstruction, notably current intrinsics
  void addIntrinsicsToHistory(const SfM_Data& sfm_data);
  
  /// @brief EIntrinsicParameter
  enum EIntrinsicParameter{Focal, Cx, Cy};
  
  /// @brief checkParameterLimits Compute, for each camera/intrinsic, the variation of the last \a windowSize values of the \a parameter.
  /// If it consideres the variation of \a parameter as enought constant it updates \a intrinsicsLimitIds.
  /// @details Pipeline:
  /// \b H: the history of all the concidered parameter
  /// \b S: the subpart of H including the last \a wondowSize values only.
  /// \b sigma = stddev(S)  
  /// \b sigma_normalized = sigma / (max(H) - min(H))
  /// \c if sigma_normalized < \a stdevPercentageLimit \c then the limit is reached.
  /// @param[in] parameter The \a EIntrinsicParameter you want to check.
  /// @param[in] windowSize Compute the variation on the \a windowSize parameter
  /// @param[in] stdevPercentageLimit The limit is reached when the standard deviation of the \a windowSize values is less than \a stdevPecentageLimit % of the range of all the values.
  void checkParameterLimits(
      const EIntrinsicParameter parameter, 
      const std::size_t windowSize, 
      const double stdevPercentageLimit);
  
  /// @brief checkAllParametersLimits Run \a checkParameterLimits() for each \a EIntrinsicParameter.
  void checkAllParametersLimits(const std::size_t kWindowSize, const double kStdDevPercentage);
  
  /// @brief isLimitReached Giving an intrinsic index and the wished parameter, is return \c true if the limit has alerady been reached, else \c false.
  /// @param[in] intrinsicId The intrinsic index.
  /// @param[in] parameter The \a EIntrinsicParameter to observe.
  /// @return true if the limit is reached, else false
  bool isLimitReached(const IndexT intrinsicId, const EIntrinsicParameter parameter) const { return intrinsicsLimitIds.at(intrinsicId).at(parameter) != 0;}
  
  /// @brief exportIntrinsicsHistory Save the history of each intrinsic. It create a file \b K<intrinsic_index>.txt in \a folder.
  /// @param[in] folder The folder in which the \b K*.txt are saved.
  void exportIntrinsicsHistory(const std::string& folder);
  
  /// @brief updateGraph Complete the graph with the newly resected views \a set_newViewsId, or all the posed views if the graph is empty.
  /// @param[in] sfm_data 
  /// @param[in] map_tracksPerView A map giving the tracks for each view
  void updateGraph(
      const SfM_Data& sfm_data, 
      const tracks::TracksPerView& map_tracksPerView);
  
  /// @brief removeViewsToTheGraph Remove some views to the graph. It delete the node and all the incident arcs for each removed view.
  /// @param[in] removedViewsId Set of views index to remove
  /// @return true if the number of removed node is equal to the size of \c removedViewsId
  bool removeViewsToTheGraph(const std::set<IndexT>& removedViewsId);
  
  /// @brief computeDistancesMaps Add the newly resected views 'newViewsIds' into a graph (nodes: cameras, egdes: matching)
  /// and compute the intragraph-distance between these new cameras and all the others.
  void computeDistancesMaps(const SfM_Data& sfm_data);
  
private:
  
  std::set<IndexT> selectViewsToAddToTheGraph(const SfM_Data& sfm_data);
  
  std::map<Pair, std::size_t> countMatchesPerImagesPair(
      const SfM_Data& sfm_data,
      const tracks::TracksPerView& map_tracksPerView,
      const std::set<IndexT>& addedViewsId);
 
  std::vector<IndexT> getIntrinsicLimitIds(const IndexT intrinsicId) const {return intrinsicsLimitIds.at(intrinsicId);}
  
  std::vector<double> getLastIntrinsicParameters(const IndexT intrinsicId) const {return intrinsicsHistory.at(intrinsicId).back().second;}
  
  template<typename T> 
  double standardDeviation(const std::vector<T>& data);
  
  // ------------------------
  // - Distances data -
  // Local BA needs to know the distance of all the old posed views to the new resected views.
  // The bundle adjustment will be processed on the closest poses only.
  // ------------------------
  
  // Ensure a minimum number of landmarks in common to consider 2 views as connected in the graph.
  static std::size_t const kMinNbOfMatches = 100;
  
  // A graph where nodes are poses and an edge exists when 2 poses shared at least 'kMinNbOfMatches' matches.
  lemon::ListGraph graph_poses; 
  
  // A map associating each view index at its node in the graph 'graph_poses'.
  std::map<IndexT, lemon::ListGraph::Node> map_viewId_node;
  
  // Contains all the last resected cameras
  std::set<IndexT> set_newViewsId; 
  
  // Store the graph-distances to the new poses/views. 
  // If the view/pose is not connected to the new poses/views, its distance is -1.
  std::map<IndexT, int> map_viewId_distance;
  std::map<IndexT, int> map_poseId_distance;
  
  // ------------------------
  // - Intrinsics data -
  // Local BA needs to know the evolution of all the intrinsics parameters.
  // When camera parameters are enought reffined (no variation) they are set to constant in the BA.
  // ------------------------
  
  // Backup of the intrinsics parameters
  IntrinicsHistory intrinsicsHistory; 
  
  // Store, for each parameter of each intrinsic, the BA's index from which it has been concidered as constant.
  // <IntrinsicIndex, <F_limitId, CX_limitId, CY_limitId>>
  std::map<IndexT, std::vector<IndexT>> intrinsicsLimitIds; 
  
};

} // namespace sfm
} // namespace openMVG

#endif // OPENMVG_SFM_DATA_LOCAL_HPP
