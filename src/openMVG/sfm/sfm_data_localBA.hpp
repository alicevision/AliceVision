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

class LocalBA_Data
{
public:
  lemon::ListGraph graph_poses; 
  
  std::map<IndexT, lemon::ListGraph::Node> map_viewId_node;
  
  // Store the graph-distances to the new poses/views. 
  // If the view/pose is not connected to the new poses/views, its distance is -1.
  std::map<IndexT, int> map_viewId_distance;
  std::map<IndexT, int> map_poseId_distance;
  
  IntrinicsHistory intrinsicsHistory; // Backup of the intrinsics parameters
  
  std::map<IndexT, std::vector<IndexT>> intrinsicsLimitIds; // <IntrinsicIndex, <F_limitId, CX_limitId, CY_limitId>>
  
public:
  
  LocalBA_Data() {;}
  LocalBA_Data(const SfM_Data& sfm_data);
  
  enum IntrinsicParameter{Focal, Cx, Cy};
  
  void addIntrinsicsToHistory(const SfM_Data& sfm_data);
  
  void computeParameterLimits(const IntrinsicParameter &parameter, const std::size_t kWindowSize, const double kStdDevPercentage);

  void computeAllParametersLimits(const std::size_t kWindowSize, const double kStdDevPercentage);
  
  std::vector<IndexT> getIntrinsicLimitIds(const IndexT intrinsicId) const {return intrinsicsLimitIds.at(intrinsicId);}
  
  std::vector<double> getLastIntrinsicParameters(const IndexT intrinsicId) const {return intrinsicsHistory.at(intrinsicId).back().second;}
  
  bool isLimitReached(const IndexT intrinsicId, IntrinsicParameter parameter) const { return intrinsicsLimitIds.at(intrinsicId).at(parameter) != 0;}
  
  void exportIntrinsicsHistory(const std::string& folder);
  
  
  
    /// @brief Complete the graph '_reconstructionGraph' with new poses
  void updateGraph(const SfM_Data& sfm_data, 
    const tracks::TracksPerView& map_tracksPerView,
    const std::set<IndexT>& newViewIds);
    
  /// \brief Add the newly resected views 'newViewsIds' into a graph (nodes: cameras, egdes: matching)
  /// and compute the intragraph-distance between these new cameras and all the others.
  void computeDistancesMaps(const SfM_Data& sfm_data, 
    const std::set<IndexT>& newViewIds);
  
private:
  /// Normalize data as: 
  /// normalizedData[i] = (data[i] - min(data)) / (max(data) - min(data)) 
  template<typename T> 
  std::vector<T> normalize(const std::vector<T>& data);
  
  template<typename T> 
  double standardDeviation(const std::vector<T>& data);
};



} // namespace sfm
} // namespace openMVG

#endif // OPENMVG_SFM_DATA_LOCAL_HPP
