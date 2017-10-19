// Copyright (c) 2015 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.


#include "openMVG/sfm/sfm_data_localBA.hpp"

namespace openMVG {
namespace sfm {

// -- Constructor
LocalBA_Data::LocalBA_Data(const SfM_Data& sfm_data)
{
  for (const auto& it : sfm_data.intrinsics)
  {
    _intrinsicsHistory[it.first];
    _intrinsicsHistory.at(it.first).push_back(std::make_pair(0, sfm_data.GetIntrinsicPtr(it.first)->getParams().at(0)));
    _mapIntrinsicIsConstant[it.first];
    _mapIntrinsicIsConstant.at(it.first) = false; 
  }
}

// -- Getters
int LocalBA_Data::getPoseDistance(const IndexT poseId) const
{
  if (_mapDistancePerPoseId.find(poseId) == _mapDistancePerPoseId.end())
  {
    OPENMVG_LOG_DEBUG("The pose #" << poseId << " does not exist in the '_mapDistancePerPoseId':\n"
                      << _mapDistancePerPoseId);
    return -1;
  }
  return _mapDistancePerPoseId.at(poseId);
}

int LocalBA_Data::getViewDistance(const IndexT viewId) const
{
  if (_mapDistancePerViewId.find(viewId) == _mapDistancePerViewId.end())
  {
    OPENMVG_LOG_DEBUG("The view #" << viewId << " does not exist in the '_mapDistancePerViewId':\n"
                      << _mapDistancePerViewId);
    return -1;
  }
  return _mapDistancePerViewId.at(viewId);
}

// -- Methods 
std::set<IndexT> LocalBA_Data::selectViewsToAddToTheGraph(const SfM_Data& sfm_data)
{
  std::set<IndexT> addedViewsId;
  
  // Identify the view we need to add to the graph:
  if (_graph.maxNodeId() == -1) // empty graph 
  {
    OPENMVG_LOG_INFO("'map_viewId_node' is empty: first local BA.");
    for (auto & it : sfm_data.GetViews())
    {
      if (sfm_data.IsPoseAndIntrinsicDefined(it.first))
          addedViewsId.insert(it.first);
    }
  }
  else // not empty graph 
  {
    for (const IndexT viewId : _newViewsId)
    {
      auto it = map_viewId_node.find(viewId);
      if (it == map_viewId_node.end()) // the view does not already have an associated node
        addedViewsId.insert(viewId);
    }
  }
  
  if (addedViewsId.empty())
  {
    OPENMVG_LOG_DEBUG("No views is add to the graph.");
  }
  return addedViewsId;
}

std::map<Pair, std::size_t> LocalBA_Data::countSharedLandmarksPerImagesPair(
    const SfM_Data& sfm_data,
    const tracks::TracksPerView& map_tracksPerView,
    const std::set<IndexT>& newViewsId)
{
  // An edge is created between 2 views when they share at least 'L' landmarks (by default L=100).
  // At first, we need to count the number of shared landmarks between all the new views 
  // and each already resected cameras (already in the graph)
  std::map<Pair, std::size_t> map_imagesPair_nbSharedLandmarks;
  
  // Get landmarks id. of all the reconstructed 3D points (: landmarks)
  // TODO: avoid copy and use boost::transform_iterator
  std::set<IndexT> landmarkIds;
  std::transform(sfm_data.GetLandmarks().begin(), sfm_data.GetLandmarks().end(),
                 std::inserter(landmarkIds, landmarkIds.begin()),
                 stl::RetrieveKey());
  
  for(const auto& viewId: newViewsId)
  {
    // Get all the tracks of the new added view
    const openMVG::tracks::TrackIdSet& newView_trackIds = map_tracksPerView.at(viewId);
    
    // Keep the reconstructed tracks (with an associated landmark)
    std::vector<IndexT> newView_landmarks; // all landmarks (already reconstructed) visible from the new view
    
    newView_landmarks.reserve(newView_trackIds.size());
    std::set_intersection(newView_trackIds.begin(), newView_trackIds.end(),
                          landmarkIds.begin(), landmarkIds.end(),
                          std::back_inserter(newView_landmarks));
    
    // Retrieve the common track Ids
    for(auto landmark: newView_landmarks)
    {
      for(auto viewObs: sfm_data.structure.at(landmark).observations)
      {
        if (viewObs.first == viewId) continue; // do not compare an observation with itself
        
        // Increment the number of common landmarks between the new view and the already 
        // reconstructed cameras (observations).
        // format: pair<min_viewid, max_viewid>
        auto viewPair = std::make_pair(std::min(viewId, viewObs.first), std::max(viewId, viewObs.first));
        auto it = map_imagesPair_nbSharedLandmarks.find(viewPair);
        if(it == map_imagesPair_nbSharedLandmarks.end())  // the first common landmark
          map_imagesPair_nbSharedLandmarks[viewPair] = 1;
        else
          it->second++;
      }
    }
  }
  return map_imagesPair_nbSharedLandmarks;
}

void LocalBA_Data::updateGraphWithNewViews(
    const SfM_Data& sfm_data, 
    const tracks::TracksPerView& map_tracksPerView)
{
  OPENMVG_LOG_INFO("Updating the distances graph with newly resected views...");
  
  std::set<IndexT> addedViewsId = selectViewsToAddToTheGraph(sfm_data);
  
  std::map<Pair, std::size_t> map_imagesPair_nbSharedLandmarks 
      = countSharedLandmarksPerImagesPair(sfm_data, map_tracksPerView, addedViewsId);
  
  // -- Add nodes to the graph
  for (auto& viewId : addedViewsId)
  {
    lemon::ListGraph::Node newNode = _graph.addNode();
    map_viewId_node[viewId] = newNode;  
  }
  
  // -- Add edges to the graph
  std::size_t numEdges = 0;
  for(auto& it: map_imagesPair_nbSharedLandmarks)
  {
    if(it.second > _kMinNbOfMatches) // ensure a minimum number of landmarks in common to consider the link
    {
      _graph.addEdge(map_viewId_node.at(it.first.first), map_viewId_node.at(it.first.second));
      numEdges++;
    }
  }
  OPENMVG_LOG_INFO("The distances graph has been completed with " 
  << addedViewsId.size() << " nodes & " << numEdges << " edges.");
}

bool LocalBA_Data::removeViewsToTheGraph(const std::set<IndexT>& removedViewsId)
{
  std::size_t numRemovedNode = 0;
  for (const IndexT& viewId : removedViewsId)
  {
    auto it = map_viewId_node.find(viewId);
    if (it != map_viewId_node.end())
    {
      // this function erase a node with its incident arcs
      _graph.erase(it->second);
      map_viewId_node.erase(it);
      numRemovedNode++;
      OPENMVG_LOG_INFO("The view #" << viewId << " has been successfully removed to the distance graph.");
    }
    else 
      OPENMVG_LOG_DEBUG("The removed view #" << viewId << " does not exist in the 'map_viewId_node'.");
  }
  
  return numRemovedNode == removedViewsId.size();
}

void LocalBA_Data::computeDistancesMaps(const SfM_Data& sfm_data)
{ 
  OPENMVG_LOG_INFO("Computing distance maps...");
  _mapDistancePerViewId.clear();
  _mapDistancePerPoseId.clear();
  
  // Setup Breadth First Search using Lemon
  lemon::Bfs<lemon::ListGraph> bfs(_graph);
  bfs.init();
  
  // Add source views for the bfs visit of the _reconstructionGraph
  for(const IndexT viewId: _newViewsId)
  {
    auto it = map_viewId_node.find(viewId);
    if (it == map_viewId_node.end())
      OPENMVG_LOG_DEBUG("The removed view #" << viewId << " does not exist in the 'map_viewId_node'.");
    bfs.addSource(it->second);
  }
  bfs.start();
  
  // Handle bfs results (distances)
  for(auto it: map_viewId_node) // each node in the graph
  {
    auto& node = it.second;
    int d = -1; 
    
    if (bfs.reached(node))
      d = bfs.dist(node);
    
    _mapDistancePerViewId[it.first] = d;
  }
  
  // Re-mapping: from <ViewId, distance> to <PoseId, distance>:
  for(auto it: _mapDistancePerViewId)
  {
    // Get the poseId of the camera no. viewId
    IndexT idPose = sfm_data.GetViews().at(it.first)->getPoseId(); // PoseId of a resected camera
    
    auto poseIt = _mapDistancePerPoseId.find(idPose);
    // If multiple views share the same pose
    if(poseIt != _mapDistancePerPoseId.end())
      poseIt->second = std::min(poseIt->second, it.second);
    else
      _mapDistancePerPoseId[idPose] = it.second;
  } 
  
  // (Optionnal) Display result: viewId -> distance to recent cameras
  OPENMVG_LOG_INFO("-- Distribution ViewId(Distance): ");
  for (auto & itVMap: _mapDistancePerViewId)
  {
    std::cout << itVMap.first << "(" << itVMap.second << ") ";
  }
  std::cout << "\n";
}

void LocalBA_Data::convertDistancesToLBAStates(const SfM_Data & sfm_data)
{
  // reset the maps
  _mapLBAStatePerPoseId.clear();
  _mapLBAStatePerIntrinsicId.clear();
  _mapLBAStatePerLandmarkId.clear();
  
  // ----------------------------------------------------
  //  D = distanceLimit
  //  L = percentageLimit
  //  W = windowSize
  //  - cameras:
  //    - dist <= D: refined
  //    - dist == D+1: constant
  //    - else ignored
  //  - intrinsic:
  //    All the parameters of each intrinic are saved.        
  //    All the intrinsics are set to Refined by default.     
  //    An intrinsic is set to contant when its focal lenght  
  //    is considered as stable in its W last saved values
  //    according to all of its values.                       
  //  - landmarks:
  //    - connected to a refined camera: refined
  //    - else ignored
  // ----------------------------------------------------
  const std::size_t kDistanceLimit = 1; // graph distance
  const std::size_t kWindowSize = 25;   // nb of the last value in which compute the variation
  const double kStdevPercentage = 1.0;  // limit percentage of the Std deviation according to the range of all the parameters (e.i. focal)
  
  // -- Poses
  for (Poses::const_iterator itPose = sfm_data.GetPoses().begin(); itPose != sfm_data.GetPoses().end(); ++itPose)
  {
    const IndexT poseId = itPose->first;
    int dist = getPoseDistance(poseId);
    if (dist >= 0 && dist <= kDistanceLimit) 
      _mapLBAStatePerPoseId[poseId] = ELocalBAState::refined;
    else if (dist == kDistanceLimit + 1)
      _mapLBAStatePerPoseId[poseId] = ELocalBAState::constant;
    else // dist < 0 (not connected to the node) or > D + 1
      _mapLBAStatePerPoseId[poseId] = ELocalBAState::ignored;
  }
  
  // -- Instrinsics
  checkIntrinsicsConsistency(kWindowSize, kStdevPercentage); 
  
  for(const auto& itIntrinsic: sfm_data.GetIntrinsics())
  {
    if (isIntrinsicConstant(itIntrinsic.first))
      _mapLBAStatePerIntrinsicId[itIntrinsic.first] = ELocalBAState::constant;
    else
      _mapLBAStatePerIntrinsicId[itIntrinsic.first] = ELocalBAState::refined;
  }
  
  // -- Landmarks
  for(const auto& itLandmark: sfm_data.structure)
  {
    const IndexT landmarkId = itLandmark.first;
    const Observations & observations = itLandmark.second.observations;
    
    _mapLBAStatePerLandmarkId[landmarkId] = ELocalBAState::ignored;
    
    for(const auto& observationIt: observations)
    {
      int dist = getViewDistance(observationIt.first);
      if(dist >= 0 && dist <= kDistanceLimit)
      {
        _mapLBAStatePerLandmarkId[landmarkId] = ELocalBAState::refined;
        break;
      }
    }
  }
}

std::size_t LocalBA_Data::getNumberOfConstantAndRefinedCameras()
{
  std::size_t num = 0;
  for (auto it : _mapLBAStatePerPoseId)
  {
    if (it.second == LocalBA_Data::ELocalBAState::refined || it.second == LocalBA_Data::ELocalBAState::constant)
      num++;
  }
  return num;
}

void LocalBA_Data::addIntrinsicsToHistory(const SfM_Data& sfm_data)
{
  // Count the number of poses for each intrinsic
  std::map<IndexT, std::size_t> map_intrinsicId_usageNum;
  for (const auto& itView : sfm_data.GetViews())
  {
    const View * view = itView.second.get();
    
    if (sfm_data.IsPoseAndIntrinsicDefined(view))
    {
      auto itIntr = map_intrinsicId_usageNum.find(view->getIntrinsicId());
      if (itIntr == map_intrinsicId_usageNum.end())
        map_intrinsicId_usageNum[view->getIntrinsicId()] = 1;
      else
        map_intrinsicId_usageNum[view->getIntrinsicId()]++;
    }
  }
  
  // Complete the intrinsics history withe the current focal lengths
  for (auto& it : sfm_data.intrinsics)
  {
    _intrinsicsHistory.at(it.first).push_back(
          std::make_pair(map_intrinsicId_usageNum[it.first],
          sfm_data.GetIntrinsicPtr(it.first)->getParams().at(0))
        );
  }
}

void LocalBA_Data::checkIntrinsicsConsistency(const std::size_t windowSize, const double stdevPercentageLimit)
{
  OPENMVG_LOG_INFO("Checking, for each camera, if the focal length is stable...");
  
  for (auto& elt : _intrinsicsHistory)
  {
    IndexT idIntr = elt.first;
    
    // Do not compute the variation, if the intrinsic has already be regarded as constant.
    if (isIntrinsicConstant(idIntr))
      continue;
    
    // Get the full history of intrinsic parameters
    std::vector<std::size_t> allNumPosesVec;
    std::vector<double> allValuesVec; 
    
    for (const auto& pair_uses_params : _intrinsicsHistory.at(idIntr))
    {
      allNumPosesVec.push_back(pair_uses_params.first);
      allValuesVec.push_back(pair_uses_params.second);
    }
    
    // Clean 'intrinsicsHistorical':
    //  [4 5 5 7 8 6 9]
    // - detect duplicates -> [4 (5) 5 7 8 6 9]
    // - detecting removed cameras -> [4 5 (7 8) 6 9]
    std::vector<std::size_t> filteredNumPosesVec(allNumPosesVec);
    std::vector<double> filteredValuesVec(allValuesVec);
    
    std::size_t numPosesEndWindow = allNumPosesVec.back();
    
    for (int id = filteredNumPosesVec.size()-2; id > 0; --id)
    {
      if (filteredNumPosesVec.size() < 2)
        break;
      
      if (filteredNumPosesVec.at(id) >= filteredNumPosesVec.at(id+1))
      {
        filteredNumPosesVec.erase(filteredNumPosesVec.begin()+id);
        filteredValuesVec.erase(filteredValuesVec.begin()+id);
      }
    }
    
    // Detect limit according to 'kWindowSize':
    if (numPosesEndWindow < windowSize)
      continue;
    
    IndexT idStartWindow = 0;
    for (int id = filteredNumPosesVec.size()-2; id > 0; --id)
    {
      if (numPosesEndWindow - filteredNumPosesVec.at(id) >= windowSize)
      {
        idStartWindow = id;
        break;
      }
    }
    
    // Compute the standard deviation for each parameter, between [idLimit; end()]
    std::vector<double> subValuesVec (filteredValuesVec.begin()+idStartWindow, filteredValuesVec.end());
    double stdev = standardDeviation(subValuesVec);
    
    // Normalize stdev (: divide by the range of the values)
    double minVal = *std::min_element(filteredValuesVec.begin(), filteredValuesVec.end());
    double maxVal = *std::max_element(filteredValuesVec.begin(), filteredValuesVec.end());
    double normStdev = stdev / (maxVal - minVal);
    
    // Check if the normed standard deviation is < stdevPercentageLimit
    if (normStdev*100.0 <= stdevPercentageLimit)
    {
      _mapIntrinsicIsConstant.at(idIntr) = true;
      
      OPENMVG_LOG_INFO("The intrinsic #" << idIntr << " is considered to be stable.\n" 
                       << "- minimum focal = " << minVal << "\n"
                       << "- maximal focal = " << minVal << "\n"
                       << "- std. dev. (normalized) = " << normStdev << "\n"
                       << "- current focal = " << filteredValuesVec.back() << " (= constant) \n");
    }
    else
      _mapIntrinsicIsConstant.at(idIntr) = false;
  }
}

template<typename T> 
double LocalBA_Data::standardDeviation(const std::vector<T>& data) 
{ 
  double mean = std::accumulate(data.begin(), data.end(), 0.0) / data.size();
  std::vector<double> diff(data.size());
  std::transform(data.begin(), data.end(), diff.begin(), [mean](double x) { return x - mean; });
  double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
  return std::sqrt(sq_sum / data.size());
}  

void LocalBA_Data::exportIntrinsicsHistory(const std::string& folder)
{
  OPENMVG_LOG_INFO("Exporting intrinsics history...");
  for (auto& itIntr : _intrinsicsHistory)
  {
    IndexT idIntr = itIntr.first;
    
    std::string filename = folder + "K" + std::to_string(idIntr) + ".txt";
    std::ofstream os;
    os.open(filename, std::ios::app);
    os.seekp(0, std::ios::end); //put the cursor at the end
    
    
    if (_intrinsicsHistory.at(idIntr).size() == 1) // 'intrinsicsHistory' contains EXIF data only
    {
      // -- HEADER
      if (os.tellp() == 0) // 'tellp' return the cursor's position
      {
        std::vector<std::string> header;
        header.push_back("#poses");
        header.push_back("f"); 
        header.push_back("isConstant"); 
        for (std::string & head : header)
          os << head << "\t";
        os << "\n"; 
      }
      
      // -- EXIF DATA
      os << 0 << "\t";
      os << getLastFocalLength(idIntr) << "\t";
      os << isIntrinsicConstant(idIntr) << "\t";
      os << "\n";
    }
    else // Write the last intrinsics
    {
      // -- DATA
      os << _intrinsicsHistory.at(idIntr).back().first << "\t";
      os << _intrinsicsHistory.at(idIntr).back().second << "\t";
      os << isIntrinsicConstant(idIntr) << "\t";
      os << "\n";
    }
    os.close();
  }
}

} // namespace sfm
} // namespace openMVG
