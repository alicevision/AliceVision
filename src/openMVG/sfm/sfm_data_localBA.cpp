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
    _focalLengthsHistory[it.first];
    _focalLengthsHistory.at(it.first).push_back(std::make_pair(0, sfm_data.GetIntrinsicPtr(it.first)->getParams().at(0)));
    _mapFocalIsConstant[it.first];
    _mapFocalIsConstant.at(it.first) = false; 
    
    resetParametersCounter();
  }
}

std::map<int, std::size_t> LocalBA_Data::getDistancesHistogram() const
{
  std::map<int, std::size_t> hist;
  
  for (const auto& x : _mapDistancePerViewId)
  {
    if (hist.find(x.second) == hist.end())
      hist[x.second] = 1;
    else
      hist.at(x.second)++;
  }
  
  return hist;
}

void LocalBA_Data::setAllParametersToRefine(const SfM_Data& sfm_data)
{
  _mapDistancePerViewId.clear();
  _mapDistancePerPoseId.clear();
  _mapLBAStatePerPoseId.clear();
  _mapLBAStatePerIntrinsicId.clear();
  _mapLBAStatePerLandmarkId.clear();
  resetParametersCounter();
  
  // -- Poses
  for (Poses::const_iterator itPose = sfm_data.GetPoses().begin(); itPose != sfm_data.GetPoses().end(); ++itPose)
  {
    _mapLBAStatePerPoseId[itPose->first] = EState::refined;
    _parametersCounter.at(std::make_pair(EParameter::pose, EState::refined))++;
  }
  // -- Instrinsics
  for(const auto& itIntrinsic: sfm_data.GetIntrinsics())
  {
      _mapLBAStatePerIntrinsicId[itIntrinsic.first] = EState::refined;
      _parametersCounter.at(std::make_pair(EParameter::intrinsic, EState::refined))++;
  }
  // -- Landmarks
  for(const auto& itLandmark: sfm_data.structure)
  {
    _mapLBAStatePerLandmarkId[itLandmark.first] = EState::refined;
    _parametersCounter.at(std::make_pair(EParameter::landmark, EState::refined))++;
  }  
}

void LocalBA_Data::updateParametersState(
    const SfM_Data& sfm_data, 
    const tracks::TracksPerView& map_tracksPerView, 
    const std::set<IndexT> &newReconstructedViews, 
    const std::size_t kMinNbOfMatches,
    const std::size_t kLimitDistance)
{
  // ----------------
  // Steps:
  // 1. Add the new views to the graph (1 node per new view & 1 edge connecting to views sharing matches)
  // 2. Compute the graph-distances from the new views to all the other posed views.
  // 3. Convert each graph-distances to the corresponding LBA state (refined, constant, ignored) 
  //    for every parameters included in the ceres problem (poses, landmarks, intrinsics)
  // ----------------
  updateGraphWithNewViews(sfm_data, map_tracksPerView, newReconstructedViews, kMinNbOfMatches);
  
  computeGraphDistances(sfm_data, newReconstructedViews);
  
  convertDistancesToLBAStates(sfm_data, kLimitDistance); 
}

void LocalBA_Data::saveFocalLengths(const SfM_Data& sfm_data)
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
  
  // Complete the intrinsics history with the current focal lengths
  for (const auto& x : sfm_data.intrinsics)
  {
    _focalLengthsHistory.at(x.first).push_back(
          std::make_pair(map_intrinsicId_usageNum[x.first],
          sfm_data.GetIntrinsicPtr(x.first)->getParams().at(0))
        );
  }
}

void LocalBA_Data::exportFocalLengths(const std::string& folder)
{
  OPENMVG_LOG_INFO("Exporting focal lengths history...");
  for (const auto& x : _focalLengthsHistory)
  {
    IndexT idIntr = x.first;
    
    std::string filename = stlplus::folder_append_separator(folder) + "K" + std::to_string(idIntr) + ".txt";
    std::ofstream os;
    os.open(filename, std::ios::app);
    os.seekp(0, std::ios::end); //put the cursor at the end
    
    if (_focalLengthsHistory.at(idIntr).size() == 1) // 'intrinsicsHistory' contains EXIF data only
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
      os << isFocalLengthConstant(idIntr) << "\t";
      os << "\n";
    }
    else // Write the last intrinsics
    {
      // -- DATA
      os << _focalLengthsHistory.at(idIntr).back().first << "\t";
      os << _focalLengthsHistory.at(idIntr).back().second << "\t";
      os << isFocalLengthConstant(idIntr) << "\t";
      os << "\n";
    }
    os.close();
  }
}

bool LocalBA_Data::removeViewsToTheGraph(const std::set<IndexT>& removedViewsId)
{
  std::size_t numRemovedNode = 0;
  for (const IndexT& viewId : removedViewsId)
  {
    auto it = _mapNodePerViewId.find(viewId);
    if (it != _mapNodePerViewId.end())
    {
      _graph.erase(it->second); // this function erase a node with its incident arcs
      _mapNodePerViewId.erase(it);
      _mapViewIdPerNode.erase(it->second);
      
      numRemovedNode++;
      OPENMVG_LOG_INFO("The view #" << viewId << " has been successfully removed to the distance graph.");
    }
    else 
      OPENMVG_LOG_DEBUG("The removed view #" << viewId << " does not exist in the '_mapNodePerViewId'.");
  }
  
  return numRemovedNode == removedViewsId.size();
}

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

void LocalBA_Data::resetParametersCounter()
{
  _parametersCounter.clear();
  _parametersCounter[std::make_pair(EParameter::pose, EState::refined)] = 0;
  _parametersCounter[std::make_pair(EParameter::pose, EState::constant)] = 0;
  _parametersCounter[std::make_pair(EParameter::pose, EState::ignored)] = 0;
  _parametersCounter[std::make_pair(EParameter::intrinsic, EState::refined)] = 0;
  _parametersCounter[std::make_pair(EParameter::intrinsic, EState::constant)] = 0;
  _parametersCounter[std::make_pair(EParameter::intrinsic, EState::ignored)] = 0;
  _parametersCounter[std::make_pair(EParameter::landmark, EState::refined)] = 0;
  _parametersCounter[std::make_pair(EParameter::landmark, EState::constant)] = 0;
  _parametersCounter[std::make_pair(EParameter::landmark, EState::ignored)] = 0;
}

void LocalBA_Data::updateGraphWithNewViews(
    const SfM_Data& sfm_data, 
    const tracks::TracksPerView& map_tracksPerView,
    const std::set<IndexT>& newReconstructedViews,
    const std::size_t kMinNbOfMatches)
{
  // -----------
  // Identify the views we need to add to the graph:
  // - this is the first Local BA: the graph is still empty, so add all the posed views of the scene
  // - else: add the newly posed views only.
  // Add the posed views to the graph:
  // - each node represents the posed views
  // - each edge links 2 views if they share at least 'kMinNbOfMatches' matches
  // -----------
  
  OPENMVG_LOG_INFO("Updating the distances graph with newly resected views...");
  
  // Identify the views we need to add to the graph:
  std::set<IndexT> addedViewsId;
  
  if (_graph.maxNodeId() == -1) // the graph is empty: add all the posed views
  {
    OPENMVG_LOG_INFO("|- The graph is empty: all posed views will be added.");
    for (const auto & x : sfm_data.GetViews())
    {
      if (sfm_data.IsPoseAndIntrinsicDefined(x.first))
        addedViewsId.insert(x.first);
    }
  }
  else // the graph is not empty
    addedViewsId = newReconstructedViews;
  
  std::map<Pair, std::size_t> map_imagesPair_nbSharedLandmarks 
      = countSharedLandmarksPerImagesPair(sfm_data, map_tracksPerView, addedViewsId);
  
  // -- Add nodes to the graph
  std::vector<lemon::ListGraph::Node> newNodes;
  for (const auto& viewId : addedViewsId)
  {
    lemon::ListGraph::Node newNode = _graph.addNode();
    newNodes.push_back(newNode);
    _mapNodePerViewId[viewId] = newNode;  
    _mapViewIdPerNode[newNode] = viewId;
  }
  
  // -- Add edges to the graph
  std::size_t numEdges = 0;
  
  for(const auto& x: map_imagesPair_nbSharedLandmarks)
  {
    if(x.second > kMinNbOfMatches) // ensure a minimum number of landmarks in common to consider the link
    {
      _graph.addEdge(_mapNodePerViewId.at(x.first.first), _mapNodePerViewId.at(x.first.second));
      numEdges++;
    }
  }
  
  OPENMVG_LOG_INFO("|- The distances graph has been completed with " 
                   << addedViewsId.size() << " nodes & " << numEdges << " edges.");
  OPENMVG_LOG_INFO("|- It contains " << _graph.maxNodeId() << " nodes & " << _graph.maxEdgeId() << " edges");                   
}

void LocalBA_Data::computeGraphDistances(const SfM_Data& sfm_data, const std::set<IndexT>& newReconstructedViews)
{ 
  OPENMVG_LOG_INFO("Computing graph-distances...");
  // reset the maps
  _mapDistancePerViewId.clear();
  _mapDistancePerPoseId.clear();
  
  // -- Setup Breadth First Search using Lemon
  lemon::Bfs<lemon::ListGraph> bfs(_graph);
  bfs.init();
  
  // -- Add source views for the bfs visit of the _graph
  for(const IndexT viewId: newReconstructedViews)
  {
    auto it = _mapNodePerViewId.find(viewId);
    if (it == _mapNodePerViewId.end())
      OPENMVG_LOG_DEBUG("The removed view #" << viewId << " does not exist in the '_mapNodePerViewId'.");
    bfs.addSource(it->second);
  }
  bfs.start();
  
  // -- Handle bfs results (distances)
  for(const auto& x : _mapNodePerViewId) // each node in the graph
  {
    auto& node = x.second;
    int d = -1; 
    
    if (bfs.reached(node))
    {
      d = bfs.dist(node);
      // dist(): "If node v is not reached from the root(s), then the return value of this function is undefined."
      // This is why the distance is previously set to -1.
    }
    _mapDistancePerViewId[x.first] = d;
  }
  
  // -- Re-mapping from <ViewId, distance> to <PoseId, distance>:
  for(auto x: _mapDistancePerViewId)
  {
    // Get the poseId of the camera no. viewId
    IndexT idPose = sfm_data.GetViews().at(x.first)->getPoseId(); // PoseId of a resected camera
    
    auto poseIt = _mapDistancePerPoseId.find(idPose);
    // If multiple views share the same pose
    if(poseIt != _mapDistancePerPoseId.end())
      poseIt->second = std::min(poseIt->second, x.second);
    else
      _mapDistancePerPoseId[idPose] = x.second;
  } 
}

void LocalBA_Data::convertDistancesToLBAStates(const SfM_Data & sfm_data, const std::size_t kLimitDistance)
{
  // reset the maps
  _mapLBAStatePerPoseId.clear();
  _mapLBAStatePerIntrinsicId.clear();
  _mapLBAStatePerLandmarkId.clear();
  resetParametersCounter();
  
  const std::size_t kWindowSize = 25;   // nb of the last value in which compute the variation
  const double kStdevPercentage = 1.0;  // limit percentage of the Std deviation according to the range of all the parameters (e.i. focal)
  
  // ----------------------------------------------------
  //  D = the distance of the 'active' region (kLimitDistance)
  //  W = the range (in term of num. of poses) on which to study the focal length variations (kWindowSize)
  //  L = the max. percentage of the variations of the focal length to consider it as cosntant (kStdevPercentage)
  //  - A posed views is setas to:
  //    - Refined <=> dist <= D
  //    - Constant <=> dist == D+1
  //    - Ignored <=> else (dist = -1 U [D+2; +inf.[)
  //  - An intrinsic is set to:
  //    - Refined by default
  //    - Constant <=> its focal lenght is considered as stable in its W last saved values
  //    according to all of its values.                       
  //  - A landmarks is set to:
  //    - Ignored by default
  //    - Refined <=> its connected to a refined camera
  // ----------------------------------------------------
  // -- Poses
  for (Poses::const_iterator itPose = sfm_data.GetPoses().begin(); itPose != sfm_data.GetPoses().end(); ++itPose)
  {
    const IndexT poseId = itPose->first;
    int dist = getPoseDistance(poseId);
    if (dist >= 0 && dist <= kLimitDistance) // [0; D]
    {
      _mapLBAStatePerPoseId[poseId] = EState::refined;
      _parametersCounter.at(std::make_pair(EParameter::pose, EState::refined))++;
    }
    else if (dist == kLimitDistance + 1)  // {D+1}
    {
      _mapLBAStatePerPoseId[poseId] = EState::constant;
      _parametersCounter.at(std::make_pair(EParameter::pose, EState::constant))++;
    }
    else // [-inf; 0[ U [D+2; +inf.[  (-1: not connected to the new views)
    {
      _mapLBAStatePerPoseId[poseId] = EState::ignored;
      _parametersCounter.at(std::make_pair(EParameter::pose, EState::ignored))++;
    }
  }
  
  // -- Instrinsics
  checkFocalLengthsConsistency(kWindowSize, kStdevPercentage); 
  
  for(const auto& itIntrinsic: sfm_data.GetIntrinsics())
  {
    if (isFocalLengthConstant(itIntrinsic.first))
    {
      _mapLBAStatePerIntrinsicId[itIntrinsic.first] = EState::constant;
      _parametersCounter.at(std::make_pair(EParameter::intrinsic, EState::constant))++;
    }
    else
    {
      _mapLBAStatePerIntrinsicId[itIntrinsic.first] = EState::refined;
      _parametersCounter.at(std::make_pair(EParameter::intrinsic, EState::refined))++;
    }
  }
  
  // -- Landmarks
  for(const auto& itLandmark: sfm_data.structure)
  {
    const IndexT landmarkId = itLandmark.first;
    const Observations & observations = itLandmark.second.observations;
    
    _mapLBAStatePerLandmarkId[landmarkId] = EState::ignored;
    _parametersCounter.at(std::make_pair(EParameter::landmark, EState::ignored))++;
    
    
    for(const auto& observationIt: observations)
    {
      int dist = getViewDistance(observationIt.first);
      if(dist >= 0 && dist <= kLimitDistance) // [0; D]
      {
        _mapLBAStatePerLandmarkId[landmarkId] = EState::refined;
        _parametersCounter.at(std::make_pair(EParameter::landmark, EState::refined))++;
        _parametersCounter.at(std::make_pair(EParameter::landmark, EState::ignored))--;
        break;
      }
    }
  }
}

std::map<Pair, std::size_t> LocalBA_Data::countSharedLandmarksPerImagesPair(
    const SfM_Data& sfm_data,
    const tracks::TracksPerView& map_tracksPerView,
    const std::set<IndexT>& newViewsId)
{
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
    for(const auto& landmarkId: newView_landmarks)
    {
      for(const auto& observations: sfm_data.structure.at(landmarkId).observations)
      {
        if (observations.first == viewId) continue; // do not compare an observation with itself
        
        // Increment the number of common landmarks between the new view and the already 
        // reconstructed cameras (observations).
        // format: pair<min_viewid, max_viewid>
        auto viewPair = std::make_pair(std::min(viewId, observations.first), std::max(viewId, observations.first));
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

void LocalBA_Data::checkFocalLengthsConsistency(const std::size_t windowSize, const double stdevPercentageLimit)
{
  OPENMVG_LOG_INFO("Checking, for each camera, if the focal length is stable...");
  
  for (const auto& x : _focalLengthsHistory)
  {
    IndexT idIntr = x.first;
    
    // Do not compute the variation, if the intrinsic has already be regarded as constant.
    if (isFocalLengthConstant(idIntr))
      continue;
    
    // Get the full history of intrinsic parameters
    std::vector<std::size_t> allNumPosesVec;
    std::vector<double> allValuesVec; 
    
    for (const auto& pair_uses_params : _focalLengthsHistory.at(idIntr))
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
      _mapFocalIsConstant.at(idIntr) = true;
      
      OPENMVG_LOG_INFO("The intrinsic #" << idIntr << " is considered to be stable.\n" 
                       << "- minimum focal = " << minVal << "\n"
                       << "- maximal focal = " << minVal << "\n"
                       << "- std. dev. (normalized) = " << normStdev << "\n"
                       << "- current focal = " << filteredValuesVec.back() << " (= constant) \n");
    }
    else
      _mapFocalIsConstant.at(idIntr) = false;
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



void LocalBA_Data::drawGraph(const SfM_Data& sfm_data, const std::string& dir, const std::string& nameComplement)
{
  if (!stlplus::folder_exists(dir))
    stlplus::folder_create(dir);
  
  std::stringstream dotStream;
  dotStream << "digraph lemon_dot_example {" << "\n";
  
  // -- Node
  dotStream << "  node [ shape=ellipse, penwidth=5.0, fontname=Helvetica, fontsize=40 ];" << "\n";
  for(lemon::ListGraph::NodeIt n(_graph); n!=lemon::INVALID; ++n)
  {
    IndexT viewId = _mapViewIdPerNode[n];
    int viewDist = _mapDistancePerViewId[viewId];
    
    std::string color = ", color=";
    if (viewDist == 0) color += "red";
    else if (viewDist == 1 ) color += "green";
    else if (viewDist == 2 ) color += "blue";
    else color += "black";
    dotStream << "  n" << _graph.id(n)
              << " [ label=\"" << viewId << ": D" << viewDist << " K" << sfm_data.GetViews().at(viewId)->getIntrinsicId() << "\"" << color << "]; " << "\n";
  }
  
  // -- Edge
  dotStream << "  edge [ shape=ellipse, fontname=Helvetica, fontsize=5, color=black ];" << "\n";
  for(lemon::ListGraph::EdgeIt e(_graph); e!=lemon::INVALID; ++e)
  {
    dotStream << "  n" << _graph.id(_graph.u(e)) << " -> " << " n" << _graph.id(_graph.v(e));
    if (_intrinsicEdgesId.find(_graph.id(e)) != _intrinsicEdgesId.end())
      dotStream << " [color=red]\n";
    else
      dotStream << "\n";
  }
  dotStream << "}" << "\n";
  
  const std::string dotFilepath = stlplus::create_filespec(dir, "/graph_" + std::to_string(_mapViewIdPerNode.size())  + "_" + nameComplement + ".dot");
  std::ofstream dotFile;
  dotFile.open(dotFilepath);
  dotFile.write(dotStream.str().c_str(), dotStream.str().length());
  dotFile.close();
  
  OPENMVG_LOG_INFO("The graph '"<< dir << "/graph_" << std::to_string(_mapViewIdPerNode.size()) << "_" << nameComplement << ".dot' has been saved.");
}

std::size_t LocalBA_Data::addIntrinsicEdgesToTheGraph(const SfM_Data& sfm_data, const std::set<IndexT>& newReconstructedViews)
{
  std::size_t numAddedEdges = 0;
  
  // TODO: maintain a map<IntrinsicId, vector<ViewId>>
  for (IndexT newViewId : newReconstructedViews) // for each new view
  {
    IndexT newViewIntrinsicId = sfm_data.GetViews().at(newViewId)->getIntrinsicId();
    
    if (isFocalLengthConstant(newViewIntrinsicId)) // do not add edges for a consisitent intrinsic
      continue;
    
    for (const auto& x : _mapNodePerViewId) // for each view in the graph
    {
      if (newViewId == x.first)  // do not compare a view with itself
        continue; 
      
      IndexT oldViewIntrinsicId = sfm_data.GetViews().at(x.first)->getIntrinsicId();
      
      if (oldViewIntrinsicId == newViewIntrinsicId)
      {
        IndexT minId = std::min(x.first, newViewId);
        IndexT maxId = std::max(x.first, newViewId);
        
        lemon::ListGraph::Edge edge = _graph.addEdge(_mapNodePerViewId[minId], _mapNodePerViewId[maxId]);
        _intrinsicEdgesId.insert(_graph.id(edge));
        numAddedEdges++;
        std::cout << "added intrinicsn edge: " << minId << " - " << maxId << " (#" << _graph.id(edge) << ")" << std::endl;
      }
    }
  }
  return numAddedEdges;
}

void LocalBA_Data::removeIntrinsicEdgesToTheGraph()
{
  for(int edgeId : _intrinsicEdgesId)
  {
    _graph.erase(_graph.fromId(edgeId, lemon::ListGraph::Edge()));
  }
  _intrinsicEdgesId.clear();
}

} // namespace sfm
} // namespace openMVG
