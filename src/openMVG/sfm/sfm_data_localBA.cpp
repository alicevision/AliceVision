
// Copyright (c) 2015 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.


#include "openMVG/sfm/sfm_data_localBA.hpp"

namespace openMVG {
namespace sfm {

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

void LocalBA_Data::updateGraph(
    const SfM_Data& sfm_data, 
    const tracks::TracksPerView& map_tracksPerView, 
    const std::set<IndexT>& newViewIds)
{
  std::cout << "in: updateDistancesgrpah" << std::endl;
  std::cout << "newViewIds.size() = " << newViewIds.size() << std::endl;
  std::set<IndexT> viewIdsAddedToTheGraph;
  
  // -- Add nodes (= views)
  // Identify the view we need to add to the graph:
  if (graph_poses.maxNodeId() == -1) 
  {
    std::cout << "map_viewId_node est vide ! (premier LBA théoriquement)" << std::endl;
    for (auto & it : sfm_data.GetPoses())
      viewIdsAddedToTheGraph.insert(it.first);
  }
  else 
  {
    std::cout << "map_viewId_node non vide ! (tjs sauf lors du du premier LBA théoriquement)" << std::endl;
    for (const IndexT viewId : newViewIds)
    {
      auto it = map_viewId_node.find(viewId);
      if (it == map_viewId_node.end()) // the view doesn't already have an associated node
        viewIdsAddedToTheGraph.insert(viewId);
    }
  }
  
  if (viewIdsAddedToTheGraph.empty())
    return;
  
  std::cout << "graph_poses.maxNodeId() = " << graph_poses.maxNodeId() << std::endl;
  std::cout << "viewIdsAddedToTheGraph.size() = " << viewIdsAddedToTheGraph.size() << std::endl;
  
  // Add the views as nodes to the graph:
  //  std::cout << "nouveaux noeuds ajoutés au graph (viewIds) : " << std::endl;
  std::cout << "adding node to the graph..." << std::endl;
  for (auto& viewId : viewIdsAddedToTheGraph)
  {
    lemon::ListGraph::Node newNode = graph_poses.addNode();
    map_viewId_node[viewId] = newNode;  
  }
  
  // -- Add edge.
  
  // An edge is created between 2 views when they share at least 'L' landmarks (by default L=100).
  // At first, we need to count the number of shared landmarks between all the new views 
  // and each already resected cameras (already in the graph)
  std::map<Pair, std::size_t> map_imagesPair_nbSharedLandmarks;
  
  // Get landmarks id. of all the reconstructed 3D points (: landmarks)
  std::set<IndexT> landmarkIds;
  std::transform(sfm_data.GetLandmarks().begin(), sfm_data.GetLandmarks().end(),
                 std::inserter(landmarkIds, landmarkIds.begin()),
                 stl::RetrieveKey());
  
  std::cout << "createing map_imagesPair_nbSharedLandmarks..." << std::endl;
  for(const auto& viewId: viewIdsAddedToTheGraph)
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
    
    std::map<Pair, std::size_t> map_imagesPair_nbSharedLandmarks;
    
    std::set<IndexT> landmarkIds;
    std::transform(sfm_data.GetLandmarks().begin(), sfm_data.GetLandmarks().end(),
                   std::inserter(landmarkIds, landmarkIds.begin()),
                   stl::RetrieveKey());
    
    std::cout << "createing map_imagesPair_nbSharedLandmarks..." << std::endl;
    for(const auto& viewId: viewIdsAddedToTheGraph)
    {
      const openMVG::tracks::TrackIdSet& newView_trackIds = map_tracksPerView.at(viewId);
      
      // Retrieve the common track Ids
      std::vector<IndexT> newView_landmarks; // all landmarks (already reconstructed) visible from the new view
      
      newView_landmarks.reserve(newView_trackIds.size());
      
      std::set_intersection(newView_trackIds.begin(), newView_trackIds.end(),
                            landmarkIds.begin(), landmarkIds.end(),
                            std::back_inserter(newView_landmarks));
      
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
    
    // add edges in the graph
    std::cout << "adding Edges to the graph..." << std::endl;
    
    for(auto& it: map_imagesPair_nbSharedLandmarks)
    {
      std::size_t L = 100; // typically: 100
      if(it.second > L) // ensure a minimum number of landmarks in common to consider the link
      {
        graph_poses.addEdge(map_viewId_node.at(it.first.first), map_viewId_node.at(it.first.second));
      }
    }
  }
  std::cout << "in: updateDistancesgrpah: done" << std::endl;
  
}

void LocalBA_Data::computeDistancesMaps(
    const SfM_Data& sfm_data, 
    const std::set<IndexT>& newViewIds)
{  
  std::cout << "\nvvv ------------------------ vvv" << std::endl;
  std::cout << "in: computeDistancesMaps" << std::endl;
  std::cout << "newViewIds.size() = " << newViewIds.size() << std::endl;
  map_viewId_distance.clear();
  map_poseId_distance.clear();
  
  // Setup Breadth First Search using Lemon
  lemon::Bfs<lemon::ListGraph> bfs(graph_poses);
  bfs.init();
  
  // Add source views for the bfs visit of the _reconstructionGraph
  std::cout << "... adding sources to the BFS " << std::endl;
  std::cout << "nb de sources : " << newViewIds.size() << std::endl;
  std::cout << "taille de la map_viewId_node = " << map_viewId_node.size() << std::endl;
  for(const IndexT viewId: newViewIds)
  {
    std::cout << "ajout d'un noeud avec la vue #" << viewId << std::endl;
    auto it = map_viewId_node.find(viewId);
    if (it == map_viewId_node.end())
      std::cout << "ERROR!!! La vue #" << viewId << " pas trouvée dans map_viewId_node!" << std::endl;
    bfs.addSource(it->second);
  }
  std::cout << "bfs.emptyQueue() = " << bfs.emptyQueue() << " - doit être false (0)" << std::endl;
  std::cout << "bfs.start... " << std::endl;
  bfs.start();
  
  // Handle bfs results (distances)
  std::cout << " Handle bfs results" << std::endl;
  for(auto it: map_viewId_node) // each node in the graph
  {
    auto& node = it.second;
    //    int d = bfs.dist(node);
    
    int d = -1; 
    if (bfs.reached(node))
      d = bfs.dist(node);
    
    //    int d = bfs.dist(node);
    //    if (bfs.reached(node)) // check if the node is connected to a source node
    //      std::cout << "vue #" << it.first << ", dist:" << d << std::endl;
    //    else 
    //      std::cout << "vue #" << it.first << ", dist:" << d << " (not reached) "<< std::endl;
    
    map_viewId_distance[it.first] = d;
    
  }
  
  std::cout << "_map_viewId_distance.size() = " << map_viewId_distance.size() << std::endl;
  
  // Re-mapping: from <ViewId, distance> to <PoseId, distance>:
  std::cout << "Re-mapping: from <ViewId, distance> to <PoseId, distance>" << std::endl;
  for(auto it: map_viewId_distance)
  {
    // Get the poseId of the camera no. viewId
    IndexT idPose = sfm_data.GetViews().at(it.first)->id_pose; // PoseId of a resected camera
    
    auto poseIt = map_poseId_distance.find(idPose);
    // If multiple views share the same pose
    if(poseIt != map_poseId_distance.end())
      poseIt->second = std::min(poseIt->second, it.second);
    else
      map_poseId_distance[idPose] = it.second;
  } 
  
  // Display result: viewId -> distance to recent cameras
  {    
    OPENMVG_LOG_INFO("-- View distances map: ");
    for (auto & itVMap: map_viewId_distance)
    {
      OPENMVG_LOG_INFO( itVMap.first << " -> " << itVMap.second);
    }
    
    //    OPENMVG_LOG_INFO("-- Pose distances map: ");
    //    for (auto & itPMap: _map_poseId_distance)
    //    {
    //      OPENMVG_LOG_INFO( itPMap.first << " -> " << itPMap.second);
    //    }
  }
  
  
  std::cout << "in: computeDistancesMaps (done)" << std::endl;
  std::cout << "^^^ ------------------------ ^^^\n" << std::endl;
  
}



void LocalBA_Data::addIntrinsicsToHistory(const SfM_Data& sfm_data)
{
  std::map<IndexT, std::size_t> map_intrId_numPoses = sfm_data.GetIntrinsicsUsage();
  
  for (auto& it : sfm_data.intrinsics)
  {
    intrinsicsHistory.at(it.first).push_back(
          std::make_pair(map_intrId_numPoses[it.first],
          sfm_data.GetIntrinsicPtr(it.first)->getParams())
        );
  }
}

// focal: parameterId = 0
// Cx: parameterId = 1
// Cy: parameterId = 2
void LocalBA_Data::computeParameterLimits(const IntrinsicParameter& parameter, const std::size_t kWindowSize, const double kStdDevPercentage)
{
  std::cout << "Updating parameter #" << parameter << std::endl;
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
    
    std::cout << "- Clean duplicated & removed cameras..." << std::endl;
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
    
    /* Display info */
    std::cout << "-- K #" << idIntr << std::endl;
    std::cout << "allNumPosesVec = " << allNumPosesVec << std::endl;
    std::cout << "allValuesVec = " << allValuesVec << std::endl;
    std::cout << "filteredValuesVec = " << filteredValuesVec << std::endl;
    
    // Detect limit according to 'kWindowSize':
    if (numPosesEndWindow < kWindowSize)
      continue;
    
    IndexT idStartWindow = 0;
    for (int id = filteredNumPosesVec.size()-2; id > 0; --id)
    {
      if (numPosesEndWindow - filteredNumPosesVec.at(id) >= kWindowSize)
      {
        idStartWindow = id;
        break;
      }
    }

    std::size_t numPosesStartWindow = filteredNumPosesVec.at(idStartWindow);
    
    // Compute the standard deviation for each parameter, between [idLimit; end()]
    std::cout << "- Subpart vector..." << std::endl;
    std::vector<double> subNumPosesVec (filteredNumPosesVec.begin()+idStartWindow, filteredNumPosesVec.end());
    std::vector<double> subValuesVec (filteredValuesVec.begin()+idStartWindow, filteredValuesVec.end());
    std::cout << "- Compute stdev..." << std::endl;
    double stdev = standardDeviation(subValuesVec);
    
    // Normalize stdev (: divide by the range of the values)
    double minVal = *std::min_element(filteredValuesVec.begin(), filteredValuesVec.end());
    double maxVal = *std::max_element(filteredValuesVec.begin(), filteredValuesVec.end());
    double normStdev = stdev / (maxVal - minVal);
    
    /* Display info */
    std::cout << "filtredNumPosesVec = " << filteredNumPosesVec << std::endl;
    std::cout << "idStartWindow = " << idStartWindow << std::endl;
    std::cout << "numPosesStartWindow = " << numPosesStartWindow << std::endl;
    std::cout << "numPosesEndWindow = " << numPosesEndWindow << std::endl;
    std::cout << "subNumPosesVec = " << subNumPosesVec << std::endl;
    std::cout << "normStdev = " << normStdev << std::endl;
    
    if (normStdev*100.0 <= kStdDevPercentage && intrinsicsLimitIds.at(idIntr).at(parameter) == 0)
    {
      intrinsicsLimitIds.at(idIntr).at(parameter) = numPosesEndWindow;    
    }
  }
}


//// focal: parameterId = 0
//// Cx: parameterId = 1
//// Cy: parameterId = 2
//void LocalBA_Data::computeParameterLimits(const IntrinsicParameter& parameter, const std::size_t kWindowSize, const double kStdDevPercentage)
//{
//  std::cout << "Updating parameter #" << parameter << std::endl;
//  for (auto& elt : intrinsicsHistory)
//  {
//    IndexT idIntr = elt.first;
    
//    // Do not compute limits if there are already reached for each parameter
//    if (intrinsicsLimitIds.at(idIntr).at(parameter) != 0)
//      continue;
    
    
//    // Get the full history of intrinsic parameters
//    std::vector<std::size_t> allNumPosesVec;
//    std::vector<double> allValuesVec; 
    
//    for (const auto& pair_uses_params : intrinsicsHistory.at(idIntr))
//    {
//      allNumPosesVec.push_back(pair_uses_params.first);
//      allValuesVec.push_back(pair_uses_params.second.at(parameter));
//    }
    
//    std::cout << "- Clean duplicated & removed cameras..." << std::endl;
//    // Clean 'intrinsicsHistorical':
//    //  [4 5 5 7 8 6 9]
//    // - detect duplicates -> [4 (5) 5 7 8 6 9]
//    // - detecting removed cameras -> [4 5 (7 8) 6 9]
//    std::vector<std::size_t> filteredNumPosesVec(allNumPosesVec);
//    std::vector<double> filteredValuesVec(allValuesVec);
    
//    std::size_t numPosesEndWindow = allNumPosesVec.back();
    
//    for (int id = filteredNumPosesVec.size()-2; id > 0; --id)
//    {
//      if (filteredNumPosesVec.size() < 2)
//        break;
      
//      if (filteredNumPosesVec.at(id) >= filteredNumPosesVec.at(id+1))
//      {
//        filteredNumPosesVec.erase(filteredNumPosesVec.begin()+id);
//        filteredValuesVec.erase(filteredValuesVec.begin()+id);
//      }
//    }
    
//    /* Display info */
//    std::cout << "-- K #" << idIntr << std::endl;
//    std::cout << "allNumPosesVec = " << allNumPosesVec << std::endl;
//    std::cout << "allValuesVec = " << allValuesVec << std::endl;
//    std::cout << "filteredValuesVec = " << filteredValuesVec << std::endl;
    
//    // Detect limit according to 'kWindowSize':
//    if (numPosesEndWindow < kWindowSize)
//      continue;
    
//    IndexT idStartWindow = 0;
//    for (int id = filteredNumPosesVec.size()-2; id > 0; --id)
//    {
//      if (numPosesEndWindow - filteredNumPosesVec.at(id) >= kWindowSize)
//      {
//        idStartWindow = id;
//        break;
//      }
//    }
    
//    std::size_t numPosesStartWindow = filteredNumPosesVec.at(idStartWindow);
    
//    // Normalize parameters historical:
//    // The normalization need to be done on the all historical
//    std::cout << "- Normalize..." << std::endl;
//    std::vector<double> normalizedValuesVec = normalize(filteredValuesVec);
//    // Compute the standard deviation for each parameter, between [idLimit; end()]
//    std::cout << "- Subpart vector..." << std::endl;
//    std::vector<double> subNumPosesVec (filteredNumPosesVec.begin()+idStartWindow, filteredNumPosesVec.end());
//    std::vector<double> subNormValuesVec (normalizedValuesVec.begin()+idStartWindow, normalizedValuesVec.end());
//    std::cout << "- Compute stdev..." << std::endl;
//    double stdevSubValues = standardDeviation(subNormValuesVec);
    
//    /* Display info */
//    std::cout << "filtredNumPosesVec = " << filteredNumPosesVec << std::endl;
//    std::cout << "idStartWindow = " << idStartWindow << std::endl;
//    std::cout << "numPosesStartWindow = " << numPosesStartWindow << std::endl;
//    std::cout << "numPosesEndWindow = " << numPosesEndWindow << std::endl;
//    std::cout << "subNumPosesVec = " << subNumPosesVec << std::endl;
//    std::cout << "normalizedValuesVec = " << normalizedValuesVec << std::endl;
//    std::cout << "subNormValuesVec = " << subNormValuesVec << std::endl;
//    std::cout << "stdevSubValues = " << stdevSubValues << std::endl;
    
//    if (stdevSubValues*100.0 <= kStdDevPercentage && intrinsicsLimitIds.at(idIntr).at(parameter) == 0)
//    {
//      intrinsicsLimitIds.at(idIntr).at(parameter) = numPosesEndWindow;    
//    }
//  }
//}

void LocalBA_Data::computeAllParametersLimits(const std::size_t kWindowSize, const double kStdDevPercentage)
{
  computeParameterLimits(IntrinsicParameter::Focal, kWindowSize, kStdDevPercentage); 
  computeParameterLimits(IntrinsicParameter::Cx, kWindowSize, kStdDevPercentage); 
  computeParameterLimits(IntrinsicParameter::Cy, kWindowSize, kStdDevPercentage); 
}

template<typename T> 
std::vector<T> LocalBA_Data::normalize(const std::vector<T>& data) 
{ 
  std::vector<T> normalizedData;
  normalizedData.reserve(data.size());
  T minVal = *std::min_element(data.begin(), data.end());
  T maxVal = *std::max_element(data.begin(), data.end());
  for (auto const& val : data)
    normalizedData.push_back((val - minVal)/(maxVal - minVal));
  return normalizedData;
}  

template<typename T> 
double LocalBA_Data::standardDeviation(const std::vector<T>& data) 
{ 
  double sum = std::accumulate(data.begin(), data.end(), 0.0);
  double mean = sum / data.size();
  std::vector<double> diff(data.size());
  std::transform(data.begin(), data.end(), diff.begin(), [mean](double x) { return x - mean; });
  double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
  return std::sqrt(sq_sum / data.size());
}  


void LocalBA_Data::exportIntrinsicsHistory(const std::string& folder)
{
  std::cout << "Writting intrinsics..." << std::endl;
  for (auto& itIntr : intrinsicsHistory)
  {
    IndexT idIntr = itIntr.first;
    
    std::string filename = folder + "/K" + std::to_string(idIntr) + ".txt";
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
  std::cout << "Writting intrinsics... done" << std::endl;
}

} // namespace sfm
} // namespace openMVG
