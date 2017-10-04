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
    intrinsicsHistory[it.first];
    intrinsicsHistory.at(it.first).push_back(std::make_pair(0, sfm_data.GetIntrinsicPtr(it.first)->getParams()));
    intrinsicsLimitIds[it.first];
    intrinsicsLimitIds.at(it.first) = std::vector<IndexT> (3, 0); 
  }
}

// -- Getters
int LocalBA_Data::getPoseDistance(const IndexT poseId) const
{
  if (map_poseId_distance.find(poseId) == map_poseId_distance.end())
  {
    OPENMVG_LOG_DEBUG("The pose #" << poseId << " does not exist in the 'map_poseId_distance':\n"
                      << map_poseId_distance);
    return -1;
  }
  return map_poseId_distance.at(poseId);
}

int LocalBA_Data::getViewDistance(const IndexT viewId) const
{
  if (map_viewId_distance.find(viewId) == map_viewId_distance.end())
  {
    OPENMVG_LOG_DEBUG("The view #" << viewId << " does not exist in the 'map_viewId_distance':\n"
                      << map_viewId_distance);
    return -1;
  }
  return map_viewId_distance.at(viewId);
}

// -- Methods 
std::set<IndexT> LocalBA_Data::selectViewsToAddToTheGraph(const SfM_Data& sfm_data)
{
  std::set<IndexT> addedViewsId;
  
  // Identify the view we need to add to the graph:
  if (graph_poses.maxNodeId() == -1) // empty graph 
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
    for (const IndexT viewId : set_newViewsId)
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

std::map<Pair, std::size_t> LocalBA_Data::countMatchesWithCamerasOfTheReconstruction(
    const SfM_Data& sfm_data,
    const tracks::TracksPerView& map_tracksPerView,
    const std::set<IndexT>& newViewsId)
{
  // An edge is created between 2 views when they share at least 'L' landmarks (by default L=100).
  // At first, we need to count the number of shared landmarks between all the new views 
  // and each already resected cameras (already in the graph)
  std::map<Pair, std::size_t> map_imagesPair_nbSharedLandmarks;
  
  // Get landmarks id. of all the reconstructed 3D points (: landmarks)
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

void LocalBA_Data::updateGraph(
    const SfM_Data& sfm_data, 
    const tracks::TracksPerView& map_tracksPerView)
{
  OPENMVG_LOG_INFO("Updating the distances graph with newly resected views...");
  
  std::set<IndexT> addedViewsId = selectViewsToAddToTheGraph(sfm_data);
  
  std::map<Pair, std::size_t> map_imagesPair_nbSharedLandmarks 
      = countMatchesWithCamerasOfTheReconstruction(sfm_data, map_tracksPerView, addedViewsId);
  
  // -- Add nodes to the graph
  for (auto& viewId : addedViewsId)
  {
    lemon::ListGraph::Node newNode = graph_poses.addNode();
    map_viewId_node[viewId] = newNode;  
  }
  
  // -- Add edges to the graph
  std::size_t numEdges = 0;
  for(auto& it: map_imagesPair_nbSharedLandmarks)
  {
    if(it.second > kMinNbOfMatches) // ensure a minimum number of landmarks in common to consider the link
    {
      graph_poses.addEdge(map_viewId_node.at(it.first.first), map_viewId_node.at(it.first.second));
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
      graph_poses.erase(it->second);
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
  map_viewId_distance.clear();
  map_poseId_distance.clear();
  
  // Setup Breadth First Search using Lemon
  lemon::Bfs<lemon::ListGraph> bfs(graph_poses);
  bfs.init();
  
  // Add source views for the bfs visit of the _reconstructionGraph
  for(const IndexT viewId: set_newViewsId)
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
    
    map_viewId_distance[it.first] = d;
  }
  
  // Re-mapping: from <ViewId, distance> to <PoseId, distance>:
  for(auto it: map_viewId_distance)
  {
    // Get the poseId of the camera no. viewId
    IndexT idPose = sfm_data.GetViews().at(it.first)->getPoseId(); // PoseId of a resected camera
    
    auto poseIt = map_poseId_distance.find(idPose);
    // If multiple views share the same pose
    if(poseIt != map_poseId_distance.end())
      poseIt->second = std::min(poseIt->second, it.second);
    else
      map_poseId_distance[idPose] = it.second;
  } 
  
  // (Optionnal) Display result: viewId -> distance to recent cameras
  OPENMVG_LOG_INFO("-- Distribution ViewId(Distance): ");
  for (auto & itVMap: map_viewId_distance)
  {
    std::cout << itVMap.first << "(" << itVMap.second << ") ";
  }
  std::cout << "\n";
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
  
  // Complete the intrinsics history
  for (auto& it : sfm_data.intrinsics)
  {
    intrinsicsHistory.at(it.first).push_back(
          std::make_pair(map_intrinsicId_usageNum[it.first],
          sfm_data.GetIntrinsicPtr(it.first)->getParams())
        );
  }
}

void LocalBA_Data::checkParameterLimits(const EIntrinsicParameter parameter, const std::size_t windowSize, const double stdevPercentageLimit)
{
  std::string paramName;
  if (parameter == EIntrinsicParameter::Focal) paramName = "Focal";
  if (parameter == EIntrinsicParameter::Cx) paramName = "Principal Point X";
  if (parameter == EIntrinsicParameter::Cy) paramName = "Principal Point Y";
  OPENMVG_LOG_INFO("Checking, for each camera, if the " << paramName << " is stable...");
  
  for (auto& elt : intrinsicsHistory)
  {
    IndexT idIntr = elt.first;
    
    // Do not compute limits if there are already reached for each parameter
    if (intrinsicsLimitIds.at(idIntr).at(parameter) != 0)
      continue;
    
    // Get the full history of intrinsic parameters
    std::vector<std::size_t> allNumPosesVec;
    std::vector<double> allValuesVec; 
    
    for (const auto& pair_uses_params : intrinsicsHistory.at(idIntr))
    {
      allNumPosesVec.push_back(pair_uses_params.first);
      allValuesVec.push_back(pair_uses_params.second.at(parameter));
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
    if (normStdev*100.0 <= stdevPercentageLimit && intrinsicsLimitIds.at(idIntr).at(parameter) == 0)
    {
      intrinsicsLimitIds.at(idIntr).at(parameter) = numPosesEndWindow;
      
      OPENMVG_LOG_INFO("The " << paramName << " is considered to be stable.\n" 
                       << "- minimum value = " << minVal << "\n"
                       << "- maximal value = " << minVal << "\n"
                       << "- std. dev. (normalized) = " << normStdev << "\n"
                       << "- current value = " << filteredValuesVec.back() << " (= constant) \n");
    }
  }
}

void LocalBA_Data::checkAllParametersLimits(const std::size_t kWindowSize, const double kStdDevPercentage)
{
  checkParameterLimits(EIntrinsicParameter::Focal, kWindowSize, kStdDevPercentage); 
  checkParameterLimits(EIntrinsicParameter::Cx, kWindowSize, kStdDevPercentage); 
  checkParameterLimits(EIntrinsicParameter::Cy, kWindowSize, kStdDevPercentage); 
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
  for (auto& itIntr : intrinsicsHistory)
  {
    IndexT idIntr = itIntr.first;
    
    std::string filename = folder + "K" + std::to_string(idIntr) + ".txt";
    std::ofstream os;
    os.open(filename, std::ios::app);
    os.seekp(0, std::ios::end); //put the cursor at the end
    
    
    if (intrinsicsHistory.at(idIntr).size() == 1) // 'intrinsicsHistory' contains EXIF data only
    {
      // -- HEADER
      if (os.tellp() == 0) // 'tellp' return the cursor's position
      {
        std::vector<std::string> header;
        header.push_back("#poses");
        header.push_back("f"); 
        header.push_back("ppx"); 
        header.push_back("ppy"); 
        header.push_back("d1"); 
        header.push_back("d2"); 
        header.push_back("d3"); 
        header.push_back("f_limit"); 
        header.push_back("ppx_limit"); 
        header.push_back("ppy_limit");  
        for (std::string & head : header)
          os << head << "\t";
        os << "\n"; 
      }
      
      // -- EXIF DATA
      os << 0 << "\t";
      os << getLastIntrinsicParameters(idIntr).at(0) << "\t";
      os << getLastIntrinsicParameters(idIntr).at(1) << "\t";
      os << getLastIntrinsicParameters(idIntr).at(2) << "\t";
      os << getLastIntrinsicParameters(idIntr).at(3) << "\t";
      os << getLastIntrinsicParameters(idIntr).at(4) << "\t";
      os << getLastIntrinsicParameters(idIntr).at(5) << "\t";
      os << getIntrinsicLimitIds(idIntr).at(0) << "\t";
      os << getIntrinsicLimitIds(idIntr).at(1) << "\t";
      os << getIntrinsicLimitIds(idIntr).at(2) << "\t";
      os << "\n";
    }
    else // Write the last intrinsics
    {
      // -- DATA
      IntrinsicParams lastParams = intrinsicsHistory.at(idIntr).back();
      os << lastParams.first << "\t";
      os << lastParams.second.at(0) << "\t";
      os << lastParams.second.at(1) << "\t";
      os << lastParams.second.at(2) << "\t";
      os << lastParams.second.at(3) << "\t";
      os << lastParams.second.at(4) << "\t";
      os << lastParams.second.at(5) << "\t";
      os << getIntrinsicLimitIds(idIntr).at(0) << "\t";
      os << getIntrinsicLimitIds(idIntr).at(1) << "\t";
      os << getIntrinsicLimitIds(idIntr).at(2) << "\t";
      os << "\n";
    }
    os.close();
  }
}

} // namespace sfm
} // namespace openMVG
