// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "LocalBundleAdjustmentData.hpp"
#include <aliceVision/stl/stl.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <boost/filesystem.hpp>
#include <lemon/bfs.h>

#include <fstream>

namespace fs = boost::filesystem;

namespace aliceVision {
namespace sfm {

LocalBundleAdjustmentData::LocalBundleAdjustmentData(const sfmData::SfMData& sfm_data)
{
  for (const auto& it : sfm_data.intrinsics)
  {
    _focalLengthsHistory[it.first];
    _focalLengthsHistory.at(it.first).push_back(std::make_pair(0, sfm_data.getIntrinsicPtr(it.first)->getParams().at(0)));
    _mapFocalIsConstant[it.first];
    _mapFocalIsConstant.at(it.first) = false; 
    
    resetParametersCounter();
  }
}

std::map<int, std::size_t> LocalBundleAdjustmentData::getDistancesHistogram() const
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

void LocalBundleAdjustmentData::setAllParametersToRefine(const sfmData::SfMData& sfm_data)
{
  _mapDistancePerViewId.clear();
  _mapDistancePerPoseId.clear();
  _mapLBAStatePerPoseId.clear();
  _mapLBAStatePerIntrinsicId.clear();
  _mapLBAStatePerLandmarkId.clear();
  resetParametersCounter();
  
  // -- Poses
  for(sfmData::Poses::const_iterator itPose = sfm_data.getPoses().begin(); itPose != sfm_data.getPoses().end(); ++itPose)
  {
    _mapLBAStatePerPoseId[itPose->first] = EState::refined;
    _parametersCounter.at(std::make_pair(EParameter::pose, EState::refined))++;
  }
  // -- Instrinsics
  for(const auto& itIntrinsic: sfm_data.getIntrinsics())
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

void LocalBundleAdjustmentData::saveFocallengthsToHistory(const sfmData::SfMData& sfm_data)
{
  // Count the number of poses for each intrinsic
  std::map<IndexT, std::size_t> map_intrinsicId_usageNum;
  for (const auto& itView : sfm_data.getViews())
  {
    const sfmData::View * view = itView.second.get();
    
    if (sfm_data.isPoseAndIntrinsicDefined(view))
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
          sfm_data.getIntrinsicPtr(x.first)->getParams().at(0))
        );
  }
}

void LocalBundleAdjustmentData::exportFocalLengths(const std::string& folder)
{
  ALICEVISION_LOG_DEBUG("Exporting focal lengths history...");
  for (const auto& x : _focalLengthsHistory)
  {
    IndexT idIntr = x.first;
    std::string filename = (fs::path(folder) / ("K" + std::to_string(idIntr) + std::string(".txt"))).string();
    
    bool isNewFile = !fs::exists(filename);
    
    std::ofstream os;
    os.open(filename, std::ios::app);
    os.seekp(0, std::ios::end); //put the cursor at the end
    
    if (isNewFile) // Print Header + EXIF data .
    {
      // -- HEADER
      if (os.tellp() == std::streampos(0)) // 'tellp' return the cursor's position
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
      os << _focalLengthsHistory.at(idIntr).at(0).second << "\t"; // first focallength = EXIF
      os << isFocalLengthConstant(idIntr) << "\t";
      os << "\n";
    }
    
    // -- DATA
    if (_focalLengthsHistory.at(idIntr).back().first != 0) // print EXIF once only
    {
      os << _focalLengthsHistory.at(idIntr).back().first << "\t"; // num. of posed views with this intrinsic
      os << _focalLengthsHistory.at(idIntr).back().second << "\t"; // last focallength value 
      os << isFocalLengthConstant(idIntr) << "\t";
      os << "\n"; 
    }
    os.close();
  }
}

bool LocalBundleAdjustmentData::removeViewsToTheGraph(const std::set<IndexT>& removedViewsId)
{
  std::size_t numRemovedNode = 0;
  for (const IndexT& viewId : removedViewsId)
  {
    auto it = _mapNodePerViewId.find(viewId);
    if (it != _mapNodePerViewId.end())
    {
      _graph.erase(it->second); // this function erase a node with its incident arcs
      _mapNodePerViewId.erase(it->first);
      _mapViewIdPerNode.erase(it->second);

      numRemovedNode++;
      ALICEVISION_LOG_DEBUG("The view #" << viewId << " has been successfully removed to the distance graph.");
    }
    else
      ALICEVISION_LOG_WARNING("The removed view #" << viewId << " does not exist in the graph.");
  }

  return numRemovedNode == removedViewsId.size();
}

int LocalBundleAdjustmentData::getPoseDistance(const IndexT poseId) const
{
  if (_mapDistancePerPoseId.find(poseId) == _mapDistancePerPoseId.end())
  {
    ALICEVISION_LOG_DEBUG("The pose #" << poseId << " does not exist in the '_mapDistancePerPoseId' (map size: " << _mapDistancePerPoseId.size() << ") \n");
    return -1;
  }
  return _mapDistancePerPoseId.at(poseId);
}

int LocalBundleAdjustmentData::getViewDistance(const IndexT viewId) const
{
  if (_mapDistancePerViewId.find(viewId) == _mapDistancePerViewId.end())
  {
    ALICEVISION_LOG_DEBUG("Cannot get the graph-distance of the view #" << viewId << ": does not exist in the '_mapDistancePerViewId':\n");
    return -1;
  }
  return _mapDistancePerViewId.at(viewId);
}

void LocalBundleAdjustmentData::resetParametersCounter()
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

void LocalBundleAdjustmentData::updateGraphWithNewViews(
    const sfmData::SfMData& sfm_data,
    const track::TracksPerView& map_tracksPerView,
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
  
  ALICEVISION_LOG_DEBUG("Updating the distances graph with newly resected views...");
  // Identify the views we need to add to the graph:
  std::set<IndexT> addedViewsId;
  
  if (_graph.maxNodeId() + 1 == 0) // the graph is empty: add all the poses of the scene 
  {
    ALICEVISION_LOG_DEBUG("|- The graph is empty: initial pair & new view(s) added.");
    for (const auto & x : sfm_data.getViews())
    {
      if (sfm_data.isPoseAndIntrinsicDefined(x.first))
        addedViewsId.insert(x.first);
    }
  }
  else // the graph is not empty
    addedViewsId = newReconstructedViews;
  
  // --------------------------  
  // -- Add nodes to the graph
  // --------------------------  
  std::size_t nbAddedNodes = 0;
  for (const IndexT& viewId : addedViewsId)
  {
    // Check if the node does not already exist in the graph
    // It happens when multiple local BA are run successively, with no new reconstructed views.
    if (_mapNodePerViewId.find(viewId) != _mapNodePerViewId.end())
    {
      ALICEVISION_LOG_DEBUG("Cannot add the view #" << viewId << " to the graph: already exists in the graph.");
      continue;
    }

    // Check if the node corresponds to a posed views
    if (!sfm_data.isPoseAndIntrinsicDefined(viewId))
    {
      ALICEVISION_LOG_WARNING("Cannot add the view #" << viewId << " to the graph: its pose & intrinsic are not defined.");
      continue;
    }
     
    lemon::ListGraph::Node newNode = _graph.addNode();
    _mapNodePerViewId[viewId] = newNode;  
    _mapViewIdPerNode[newNode] = viewId;
    ++nbAddedNodes;
  }
  
  // Check consistency between the map/graph & the scene   
  if (_mapNodePerViewId.size() != sfm_data.getPoses().size())
    ALICEVISION_LOG_WARNING("The number of poses in the map (summarizing the graph content) "
                            "and in the scene is different (" << _mapNodePerViewId.size() << " vs. " << sfm_data.getPoses().size() << ")");

  // -------------------------- 
  // -- Add edges to the graph
  // -------------------------- 
  std::size_t numAddedEdges = 0;
  if (!addedViewsId.empty())
  {
    // Count the nb of common landmarks between the new views and the all the reconstructed views of the scene
    std::map<Pair, std::size_t> nbSharedLandmarksPerImagesPair = countSharedLandmarksPerImagesPair(sfm_data, map_tracksPerView, addedViewsId);
    
    for(const auto& x: nbSharedLandmarksPerImagesPair)
    {
      if(x.second > kMinNbOfMatches) // ensure a minimum number of landmarks in common to consider the link
      {
        _graph.addEdge(_mapNodePerViewId.at(x.first.first), _mapNodePerViewId.at(x.first.second));
        numAddedEdges++;
      }
    }
  }
  
  ALICEVISION_LOG_DEBUG("|- The distances graph has been completed with " << nbAddedNodes<< " nodes & " << numAddedEdges << " edges.");
  ALICEVISION_LOG_DEBUG("|- It contains " << _graph.maxNodeId() + 1 << " nodes & " << _graph.maxEdgeId() + 1 << " edges");                   
}

void LocalBundleAdjustmentData::computeGraphDistances(const sfmData::SfMData& sfm_data, const std::set<IndexT>& newReconstructedViews)
{ 
  ALICEVISION_LOG_DEBUG("Computing graph-distances...");
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
      ALICEVISION_LOG_WARNING("The reconstructed view #" << viewId << " cannot be added as source for the BFS: does not exist in the graph.");
    else
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
    IndexT idPose = sfm_data.getViews().at(x.first)->getPoseId(); // PoseId of a resected camera
    
    auto poseIt = _mapDistancePerPoseId.find(idPose);
    // If multiple views share the same pose
    if(poseIt != _mapDistancePerPoseId.end())
      poseIt->second = std::min(poseIt->second, x.second);
    else
      _mapDistancePerPoseId[idPose] = x.second;
  } 
}

void LocalBundleAdjustmentData::convertDistancesToLBAStates(const sfmData::SfMData & sfm_data)
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
  for(sfmData::Poses::const_iterator itPose = sfm_data.getPoses().begin(); itPose != sfm_data.getPoses().end(); ++itPose)
  {
    const IndexT poseId = itPose->first;
    int dist = getPoseDistance(poseId);
    if (dist >= 0 && dist <= _graphDistanceLimit) // [0; D]
    {
      _mapLBAStatePerPoseId[poseId] = EState::refined;
      _parametersCounter.at(std::make_pair(EParameter::pose, EState::refined))++;
    }
    else if (dist == _graphDistanceLimit + 1)  // {D+1}
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
  
  for(const auto& itIntrinsic: sfm_data.getIntrinsics())
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
    const sfmData::Observations & observations = itLandmark.second.observations;
    
    _mapLBAStatePerLandmarkId[landmarkId] = EState::ignored;
    _parametersCounter.at(std::make_pair(EParameter::landmark, EState::ignored))++;
    
    
    for(const auto& observationIt: observations)
    {
      int dist = getViewDistance(observationIt.first);
      if(dist >= 0 && dist <= _graphDistanceLimit) // [0; D]
      {
        _mapLBAStatePerLandmarkId[landmarkId] = EState::refined;
        _parametersCounter.at(std::make_pair(EParameter::landmark, EState::refined))++;
        _parametersCounter.at(std::make_pair(EParameter::landmark, EState::ignored))--;
        break;
      }
    }
  }
}

std::map<Pair, std::size_t> LocalBundleAdjustmentData::countSharedLandmarksPerImagesPair(
    const sfmData::SfMData& sfm_data,
    const track::TracksPerView& map_tracksPerView,
    const std::set<IndexT>& newViewsId)
{
  std::map<Pair, std::size_t> map_imagesPair_nbSharedLandmarks;
  
  // Get landmarks id. of all the reconstructed 3D points (: landmarks)
  // TODO: avoid copy and use boost::transform_iterator
  std::set<IndexT> landmarkIds;
  std::transform(sfm_data.getLandmarks().begin(), sfm_data.getLandmarks().end(),
                 std::inserter(landmarkIds, landmarkIds.begin()),
                 stl::RetrieveKey());
  
  for(const auto& viewId: newViewsId)
  {
    // Get all the tracks of the new added view
    const aliceVision::track::TrackIdSet& newView_trackIds = map_tracksPerView.at(viewId);
    
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

void LocalBundleAdjustmentData::checkFocalLengthsConsistency(const std::size_t windowSize, const double stdevPercentageLimit)
{
  ALICEVISION_LOG_DEBUG("Checking, for each camera, if the focal length is stable...");
  std::size_t numOfConstFocal = 0;
  for (const auto& x : _focalLengthsHistory)
  {
    IndexT idIntr = x.first;
    
    // Do not compute the variation, if the intrinsic has already be regarded as constant.
    if (isFocalLengthConstant(idIntr))
    {
      numOfConstFocal++;
      continue;
    }
    
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
      numOfConstFocal++;
      ALICEVISION_LOG_DEBUG("|- The intrinsic #" << idIntr << " is now considered to be stable.\n");
    }
    else
      _mapFocalIsConstant.at(idIntr) = false;
  }
  ALICEVISION_LOG_DEBUG("|- " << numOfConstFocal << "/" << _mapFocalIsConstant.size() << " intrinsics with a stable focal.");
}

template<typename T> 
double LocalBundleAdjustmentData::standardDeviation(const std::vector<T>& data) 
{ 
  double mean = std::accumulate(data.begin(), data.end(), 0.0) / data.size();
  std::vector<double> diff(data.size());
  std::transform(data.begin(), data.end(), diff.begin(), [mean](double x) { return x - mean; });
  double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
  return std::sqrt(sq_sum / data.size());
}  



void LocalBundleAdjustmentData::drawGraph(const sfmData::SfMData& sfm_data, const std::string& dir, const std::string& nameComplement)
{
  if (!fs::exists(dir))
    fs::create_directory(dir);
  
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
              << " [ label=\"" << viewId << ": D" << viewDist << " K" << sfm_data.getViews().at(viewId)->getIntrinsicId() << "\"" << color << "]; " << "\n";
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
  
  const std::string dotFilepath = (fs::path(dir) / ("graph_" + std::to_string(_mapViewIdPerNode.size())  + "_" + nameComplement + ".dot")).string();
  std::ofstream dotFile;
  dotFile.open(dotFilepath);
  dotFile.write(dotStream.str().c_str(), dotStream.str().length());
  dotFile.close();
  
  ALICEVISION_LOG_DEBUG("The graph '"<< dotFilepath << "' has been saved.");
}

std::size_t LocalBundleAdjustmentData::addIntrinsicEdgesToTheGraph(const sfmData::SfMData& sfm_data, const std::set<IndexT>& newReconstructedViews)
{
  std::size_t numAddedEdges = 0;
  
  // TODO: maintain a map<IntrinsicId, vector<ViewId>>
  for (IndexT newViewId : newReconstructedViews) // for each new view
  {
    IndexT newViewIntrinsicId = sfm_data.getViews().at(newViewId)->getIntrinsicId();
    
    if (isFocalLengthConstant(newViewIntrinsicId)) // do not add edges for a consisitent intrinsic
      continue;
    
    for (const auto& x : _mapNodePerViewId) // for each view in the graph
    {
      if (newViewId == x.first)  // do not compare a view with itself
        continue; 
      
      IndexT oldViewIntrinsicId = sfm_data.getViews().at(x.first)->getIntrinsicId();
      
      if (oldViewIntrinsicId == newViewIntrinsicId)
      {
        IndexT minId = std::min(x.first, newViewId);
        IndexT maxId = std::max(x.first, newViewId);
        
        lemon::ListGraph::Edge edge = _graph.addEdge(_mapNodePerViewId[minId], _mapNodePerViewId[maxId]);
        _intrinsicEdgesId.insert(_graph.id(edge));
        numAddedEdges++;
      }
    }
  }
  return numAddedEdges;
}

void LocalBundleAdjustmentData::removeIntrinsicEdgesToTheGraph()
{
  for(int edgeId : _intrinsicEdgesId)
  {
    _graph.erase(_graph.fromId(edgeId, lemon::ListGraph::Edge()));
  }
  _intrinsicEdgesId.clear();
}

} // namespace sfm
} // namespace aliceVision
