// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "LocalBundleAdjustmentGraph.hpp"
#include <aliceVision/stl/stl.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <boost/filesystem.hpp>

#include <lemon/bfs.h>

#include <fstream>
#include <algorithm>

namespace fs = boost::filesystem;

namespace aliceVision {
namespace sfm {

LocalBundleAdjustmentGraph::LocalBundleAdjustmentGraph(const sfmData::SfMData& sfmData)
{
  for(const auto& it : sfmData.getIntrinsics())
  {
    const IndexT intrinsicId = it.first;
    const auto intrinsicPtr = it.second;

    IntrinsicHistory intrinsicHistory;
    intrinsicHistory.nbPoses = 0;
    intrinsicHistory.focalLength = intrinsicPtr->getParams().at(0);
    intrinsicHistory.isConstant = intrinsicPtr->isLocked();

    _intrinsicsHistory[intrinsicId].push_back(intrinsicHistory);
    _mapFocalIsConstant[intrinsicId] = intrinsicPtr->isLocked();
   }
}

std::map<int, std::size_t> LocalBundleAdjustmentGraph::getDistancesHistogram() const
{
  std::map<int, std::size_t> histogram;

  for(const auto& x : _distancePerViewId)
  {
    if(histogram.find(x.second) == histogram.end())
      histogram[x.second] = 1;
    else
      histogram.at(x.second)++;
  }
  return histogram;
}

void LocalBundleAdjustmentGraph::setAllParametersToRefine(const sfmData::SfMData& sfmData)
{
  _distancePerViewId.clear();
  _distancePerPoseId.clear();
  _statePerPoseId.clear();
  _statePerIntrinsicId.clear();
  _statePerLandmarkId.clear();
  
  // poses
  for(sfmData::Poses::const_iterator itPose = sfmData.getPoses().begin(); itPose != sfmData.getPoses().end(); ++itPose)
    _statePerPoseId[itPose->first] = BundleAdjustment::EParameterState::REFINED;

  // instrinsics
  for(const auto& itIntrinsic: sfmData.getIntrinsics())
    _statePerIntrinsicId[itIntrinsic.first] = BundleAdjustment::EParameterState::REFINED;

  // landmarks
  for(const auto& itLandmark: sfmData.structure)
    _statePerLandmarkId[itLandmark.first] = BundleAdjustment::EParameterState::REFINED;
}

void LocalBundleAdjustmentGraph::saveIntrinsicsToHistory(const sfmData::SfMData& sfmData)
{
  // count the number of poses for each intrinsic
  std::map<IndexT, std::size_t> intrinsicUsage;

  for(const auto& itView : sfmData.getViews())
  {
    const sfmData::View * view = itView.second.get();
    
    if(sfmData.isPoseAndIntrinsicDefined(view))
    {
      auto itIntr = intrinsicUsage.find(view->getIntrinsicId());
      if(itIntr == intrinsicUsage.end())
        intrinsicUsage[view->getIntrinsicId()] = 1;
      else
        intrinsicUsage[view->getIntrinsicId()]++;
    }
  }
  
  // complete the intrinsics history with the current focal lengths
  for(const auto& it : sfmData.getIntrinsics())
  {
    const IndexT intrinsicId = it.first;
    const auto intrinsicPtr = it.second;

    IntrinsicHistory intrinsicHistory;
    intrinsicHistory.nbPoses = intrinsicUsage[intrinsicId];
    intrinsicHistory.focalLength = intrinsicPtr->getParams().at(0);
    intrinsicHistory.isConstant = isFocalLengthConstant(intrinsicId);

    _intrinsicsHistory.at(intrinsicId).push_back(intrinsicHistory);
  }
}

void LocalBundleAdjustmentGraph::exportIntrinsicsHistory(const std::string& folder, const std::string& filename)
{
  ALICEVISION_LOG_DEBUG("Exporting intrinsics history...");
  std::ofstream os;
  os.open((fs::path(folder) / filename).string(), std::ios::app);
  os.seekp(0, std::ios::end); // put the cursor at the end

  for(const auto& intrinsicHistoryPair : _intrinsicsHistory)
    os << intrinsicHistoryPair.first << ";;;;";
  os << "\n";

  for(const auto& intrinsicHistoryPair : _intrinsicsHistory)
    os << "#poses;f;isConstant;;";
  os << "\n";

  const std::size_t nbIterations = _intrinsicsHistory.begin()->second.size();

  for(std::size_t i = 0; i < nbIterations; ++i)
  {
    for(const auto& intrinsicHistoryPair : _intrinsicsHistory)
    {
      const IntrinsicHistory& intrinsicsHistory = intrinsicHistoryPair.second.at(i);
      os << intrinsicsHistory.nbPoses << ";"<< intrinsicsHistory.focalLength << ";" << intrinsicsHistory.isConstant << ";;";
    }
    os << "\n";
  }
  os.close();
}

bool LocalBundleAdjustmentGraph::removeViews(const sfmData::SfMData& sfmData, const std::set<IndexT>& removedViewsId)
{
  std::size_t numRemovedNode = 0;
  std::map<IndexT, std::vector<int>> removedEdgesByIntrinsic;

  for(const IndexT& viewId : removedViewsId)
  {
    const auto it = _nodePerViewId.find(viewId);
    if(it == _nodePerViewId.end())
    {
      ALICEVISION_LOG_WARNING("The view id: " << viewId << " does not exist in the graph, cannot remove it.");
      continue;
    }

    // keep track of node incident edges that are going to be removed
    // in order to update _intrinsicEdgesId accordingly
    { 
      const IndexT intrinsicId = sfmData.getView(viewId).getIntrinsicId();
      const auto intrinsicIt = _intrinsicEdgesId.find(intrinsicId);
      if(intrinsicIt != _intrinsicEdgesId.end())
      {
        // store incident edge ids before removal
        for(lemon::ListGraph::IncEdgeIt e(_graph, it->second); e != lemon::INVALID; ++e)
        {
          removedEdgesByIntrinsic[intrinsicId].push_back(_graph.id(lemon::ListGraph::Edge(e)));
        }
      }
    }
    
    _graph.erase(it->second); // this function erase a node with its incident arcs
    _viewIdPerNode.erase(it->second);
    _nodePerViewId.erase(it->first); // warning: invalidates the iterator "it", so it can not be used after this line

    ++numRemovedNode;
    ALICEVISION_LOG_DEBUG("The view #" << viewId << " has been successfully removed to the distance graph.");
  }

  // remove erased edges from _intrinsicsEdgesId
  for(auto& edgesIt : removedEdgesByIntrinsic)
  {
    const IndexT intrinsicId =  edgesIt.first;
    std::vector<int>& edgeIds = _intrinsicEdgesId[intrinsicId];
    std::vector<int>& removedEdges = edgesIt.second;

    std::vector<int> newEdgeIds;
    // sort before using set_difference
    std::sort(edgeIds.begin(), edgeIds.end());
    std::sort(removedEdges.begin(), removedEdges.end());

    std::set_difference(
      edgeIds.begin(), edgeIds.end(), 
      removedEdges.begin(), removedEdges.end(), 
      std::back_inserter(newEdgeIds)
    );
    std::swap(edgeIds, newEdgeIds);

    if(edgeIds.empty())
    {
      _intrinsicEdgesId.erase(intrinsicId);
    }
  }
  return numRemovedNode == removedViewsId.size();
}

int LocalBundleAdjustmentGraph::getPoseDistance(const IndexT poseId) const
{
  if(_distancePerPoseId.find(poseId) == _distancePerPoseId.end())
  {
    ALICEVISION_LOG_DEBUG("The pose #" << poseId << " does not exist in the '_mapDistancePerPoseId' (map size: " << _distancePerPoseId.size() << ") \n");
    return -1;
  }
  return _distancePerPoseId.at(poseId);
}

int LocalBundleAdjustmentGraph::getViewDistance(const IndexT viewId) const
{
  if(_distancePerViewId.find(viewId) == _distancePerViewId.end())
  {
    ALICEVISION_LOG_DEBUG("Cannot get the graph-distance of the view #" << viewId << ": does not exist in the '_mapDistancePerViewId':\n");
    return -1;
  }
  return _distancePerViewId.at(viewId);
}

BundleAdjustment::EParameterState LocalBundleAdjustmentGraph::getStateFromDistance(int distance) const
{
  if(distance >= 0 && distance <= _graphDistanceLimit) // [0; D]
    return BundleAdjustment::EParameterState::REFINED;
  else if(distance == _graphDistanceLimit + 1)  // {D+1}
    return BundleAdjustment::EParameterState::CONSTANT;

  // [-inf; 0[ U [D+2; +inf.[  (-1: not connected to the new views)
  return BundleAdjustment::EParameterState::IGNORED;
}

void LocalBundleAdjustmentGraph::updateGraphWithNewViews(
    const sfmData::SfMData& sfmData,
    const track::TracksPerView& map_tracksPerView,
    const std::set<IndexT>& newReconstructedViews,
    const std::size_t minNbOfMatches)
{
  // identify the views we need to add to the graph:
  // - this is the first Local BA: the graph is still empty, so add all the posed views of the scene
  // - else: add the newly posed views only.
  // add the posed views to the graph:
  // - each node represents the posed views
  // - each edge links 2 views if they share at least 'kMinNbOfMatches' matches
  
  ALICEVISION_LOG_DEBUG("Updating the distances graph with newly resected views...");

  // identify the views we need to add to the graph:
  std::set<IndexT> addedViewsId;
  
  if(_graph.maxNodeId() + 1 == 0) // the graph is empty: add all the poses of the scene
  {
    ALICEVISION_LOG_DEBUG("The graph is empty: initial pair & new view(s) added.");
    for(const auto & x : sfmData.getViews())
    {
      if(sfmData.isPoseAndIntrinsicDefined(x.first))
        addedViewsId.insert(x.first);
    }
  }
  else // the graph is not empty
    addedViewsId = newReconstructedViews;
  
  // add nodes to the graph
  std::size_t nbAddedNodes = 0;
  for(const IndexT& viewId : addedViewsId)
  {
    // check if the node does not already exist in the graph
    // it happens when multiple local BA are run successively, with no new reconstructed views.
    if(_nodePerViewId.find(viewId) != _nodePerViewId.end())
    {
      ALICEVISION_LOG_DEBUG("Cannot add the view id: " << viewId << " to the graph, already exists in the graph.");
      continue;
    }

    // check if the node corresponds to a posed views
    if(!sfmData.isPoseAndIntrinsicDefined(viewId))
    {
      ALICEVISION_LOG_WARNING("Cannot add the view id: " << viewId << " to the graph, its pose & intrinsic are not defined.");
      continue;
    }
     
    lemon::ListGraph::Node newNode = _graph.addNode();
    _nodePerViewId[viewId] = newNode;
    _viewIdPerNode[newNode] = viewId;
    ++nbAddedNodes;
  }

  // add edges to the graph
  std::size_t numAddedEdges = 0;
  if(!addedViewsId.empty())
  {
    // each new view need to be connected to the graph
    // we create the 'minNbOfEdgesPerView' best edges and all the other with more than 'minNbOfMatches' shared landmarks
    const std::size_t minNbOfEdgesPerView = 10;
    std::vector<Pair> newEdges = getNewEdges(sfmData, map_tracksPerView, addedViewsId, minNbOfMatches, minNbOfEdgesPerView);
    numAddedEdges = newEdges.size();

    for(const Pair& edge: newEdges)
      _graph.addEdge(_nodePerViewId.at(edge.first), _nodePerViewId.at(edge.second));

    numAddedEdges += addIntrinsicEdgesToTheGraph(sfmData, addedViewsId);
  }
  
  ALICEVISION_LOG_DEBUG("The distances graph has been completed with " << nbAddedNodes<< " nodes & " << numAddedEdges << " edges.");
  ALICEVISION_LOG_DEBUG("It contains " << _graph.maxNodeId() + 1 << " nodes & " << _graph.maxEdgeId() + 1 << " edges");
}

void LocalBundleAdjustmentGraph::computeGraphDistances(const sfmData::SfMData& sfmData, const std::set<IndexT>& newReconstructedViews)
{ 
  ALICEVISION_LOG_DEBUG("Computing graph-distances...");

  // reset the maps
  _distancePerViewId.clear();
  _distancePerPoseId.clear();
  
  // setup Breadth First Search using Lemon
  lemon::Bfs<lemon::ListGraph> bfs(_graph);
  bfs.init();
  
  // add source views for the bfs visit of the _graph
  for(const IndexT viewId: newReconstructedViews)
  {
    auto it = _nodePerViewId.find(viewId);
    if(it == _nodePerViewId.end())
      ALICEVISION_LOG_WARNING("The reconstructed view #" << viewId << " cannot be added as source for the BFS: does not exist in the graph.");
    else
      bfs.addSource(it->second);
  }
  bfs.start();
  
  // handle bfs results (distances)
  for(const auto& x : _nodePerViewId) // each node in the graph
  {
    auto& node = x.second;
    int d = -1; 
    
    if(bfs.reached(node))
    {
      d = bfs.dist(node);
      // dist(): "If node v is not reached from the root(s), then the return value of this function is undefined."
      // this is why the distance is previously set to -1.
    }
    _distancePerViewId[x.first] = d;
  }
  
  // re-mapping from <ViewId, distance> to <PoseId, distance>:
  for(auto x: _distancePerViewId)
  {
    // get the poseId of the camera no. viewId
    const IndexT idPose = sfmData.getViews().at(x.first)->getPoseId(); // PoseId of a resected camera
    
    auto poseIt = _distancePerPoseId.find(idPose);
    // if multiple views share the same pose
    if(poseIt != _distancePerPoseId.end())
      poseIt->second = std::min(poseIt->second, x.second);
    else
      _distancePerPoseId[idPose] = x.second;
  } 
}

void LocalBundleAdjustmentGraph::convertDistancesToStates(const sfmData::SfMData& sfmData)
{
  // reset the maps
  _statePerPoseId.clear();
  _statePerIntrinsicId.clear();
  _statePerLandmarkId.clear();
  
  const std::size_t kWindowSize = 25;   //< nb of the last value in which compute the variation
  const double kStdevPercentage = 1.0;  //< limit percentage of the Std deviation according to the range of all the parameters (e.i. focal)
  
  //  D = the distance of the 'active' region (kLimitDistance)
  //  W = the range (in term of num. of poses) on which to study the focal length variations (kWindowSize)
  //  L = the max. percentage of the variations of the focal length to consider it as cosntant (kStdevPercentage)
  //  - a posed views is setas to:
  //    - Refined <=> dist <= D
  //    - Constant <=> dist == D+1
  //    - Ignored <=> else (dist = -1 U [D+2; +inf.[)
  //  - an intrinsic is set to:
  //    - Refined by default
  //    - Constant <=> its focal lenght is considered as stable in its W last saved values
  //    according to all of its values.                       
  //  - a landmarks is set to:
  //    - Ignored by default
  //    - Refined <=> its connected to a refined camera

  // poses
  for(const auto& posePair : sfmData.getPoses())
  {
    const IndexT poseId = posePair.first;
    const int distance = getPoseDistance(poseId);
    const BundleAdjustment::EParameterState state = getStateFromDistance(distance);

    _statePerPoseId[poseId] = state;
  }
  
  // instrinsics
  checkFocalLengthsConsistency(kWindowSize, kStdevPercentage); 
  
  for(const auto& itIntrinsic: sfmData.getIntrinsics())
  {
    if(isFocalLengthConstant(itIntrinsic.first))
      _statePerIntrinsicId[itIntrinsic.first] = BundleAdjustment::EParameterState::CONSTANT;
    else
      _statePerIntrinsicId[itIntrinsic.first] = BundleAdjustment::EParameterState::REFINED;
  }
  
  // landmarks
  for(const auto& itLandmark: sfmData.structure)
  {
    const IndexT landmarkId = itLandmark.first;
    const sfmData::Observations& observations = itLandmark.second.observations;
    
    assert(observations.size() >= 2);

    std::array<bool, 3> states = {false, false, false};
    for(const auto& observationIt: observations)
    {
      const int distance = getViewDistance(observationIt.first);
      const BundleAdjustment::EParameterState viewState = getStateFromDistance(distance);
      states.at(static_cast<std::size_t>(viewState)) = true;
    }

    // in the general case, a landmark can NOT have observations from refined AND ignored cameras.
    // in pratice, there is a minimal number of common points to declare the connection between images.
    // so we can have some points that are not declared in the graph of cameras connections.
    // for these particular cases, we can have landmarks with refined AND ignored cameras.
    // in this particular case, we prefer to ignore the landmark to avoid wrong/unconstraint refinements.

    if(!states.at(static_cast<std::size_t>(BundleAdjustment::EParameterState::REFINED)) ||
        states.at(static_cast<std::size_t>(BundleAdjustment::EParameterState::IGNORED)))
      _statePerLandmarkId[landmarkId] = BundleAdjustment::EParameterState::IGNORED;
    else
      _statePerLandmarkId[landmarkId] = BundleAdjustment::EParameterState::REFINED;
  }
}

std::vector<Pair> LocalBundleAdjustmentGraph::getNewEdges(
    const sfmData::SfMData& sfmData,
    const track::TracksPerView& tracksPerView,
    const std::set<IndexT>& newViewsId,
    const std::size_t minNbOfMatches,
    const std::size_t minNbOfEdgesPerView)
{
  std::vector<Pair> newEdges;
  
  // get landmarks id. of all the reconstructed 3D points (: landmarks)
  // TODO: avoid copy and use boost::transform_iterator
  std::set<IndexT> landmarkIds;
  std::transform(sfmData.getLandmarks().begin(), sfmData.getLandmarks().end(),
                 std::inserter(landmarkIds, landmarkIds.begin()),
                 stl::RetrieveKey());
  
  for(IndexT viewId: newViewsId)
  {
    std::map<IndexT, std::size_t> sharedLandmarksPerView;

    // get all the tracks of the new added view
    const aliceVision::track::TrackIdSet& newViewTrackIds = tracksPerView.at(viewId);
    
    // keep the reconstructed tracks (with an associated landmark)
    std::vector<IndexT> newViewLandmarks; // all landmarks (already reconstructed) visible from the new view
    
    newViewLandmarks.reserve(newViewTrackIds.size());
    std::set_intersection(newViewTrackIds.begin(), newViewTrackIds.end(),
                          landmarkIds.begin(), landmarkIds.end(),
                          std::back_inserter(newViewLandmarks));
    
    // retrieve the common track Ids
    for(IndexT landmarkId: newViewLandmarks)
    {
      for(const auto& observations: sfmData.getLandmarks().at(landmarkId).observations)
      {
        if(observations.first == viewId)
          continue; // do not compare an observation with itself
        
        // increment the number of common landmarks between the new view and the already
        // reconstructed cameras (observations).
        auto it = sharedLandmarksPerView.find(observations.first);
        if(it == sharedLandmarksPerView.end())  // the first common landmark
          sharedLandmarksPerView[observations.first] = 1;
        else
          ++it->second;
      }
    }

    using ViewNbLandmarks = std::pair<IndexT, std::size_t>;

    std::vector<ViewNbLandmarks> sharedLandmarksPerViewSorted;
    sharedLandmarksPerViewSorted.reserve(sharedLandmarksPerView.size());
    for(const auto& sharedLandmarkPair: sharedLandmarksPerView)
      sharedLandmarksPerViewSorted.push_back(sharedLandmarkPair);

    std::sort(sharedLandmarksPerViewSorted.begin(), sharedLandmarksPerViewSorted.end(), [](const ViewNbLandmarks& a, const ViewNbLandmarks& b){ return (a.second > b.second); });

    std::size_t nbEdgesPerView = 0;
    for(const ViewNbLandmarks& sharedLandmarkPair : sharedLandmarksPerViewSorted)
    {
      if(nbEdgesPerView >= minNbOfEdgesPerView &&
         sharedLandmarkPair.second < minNbOfMatches)
        break;

      // edges format: pair<min_viewid, max_viewid>
      newEdges.emplace_back(std::min(viewId, sharedLandmarkPair.first), std::max(viewId, sharedLandmarkPair.first));
      ++nbEdgesPerView;
    }
  }
  return newEdges;
}

void LocalBundleAdjustmentGraph::checkFocalLengthsConsistency(const std::size_t windowSize, const double stdevPercentageLimit)
{
  ALICEVISION_LOG_DEBUG("Checking, for each camera, if the focal length is stable...");
  std::size_t numOfConstFocal = 0;

  for(const auto& intrinsicEntry : _intrinsicsHistory)
  {
    const IndexT idIntrinsic = intrinsicEntry.first;
    
    // do not compute the variation, if the intrinsic has already be set as constant.
    if(isFocalLengthConstant(idIntrinsic))
    {
      numOfConstFocal++;
      continue;
    }
    
    // get the full history of intrinsic parameters
    std::vector<std::size_t> allNbPoses;
    std::vector<double> allFocalLengths;
    
    for(const IntrinsicHistory& intrinsicHistory : intrinsicEntry.second)
    {
      allNbPoses.push_back(intrinsicHistory.nbPoses);
      allFocalLengths.push_back(intrinsicHistory.focalLength);
    }
    
    // clean 'intrinsicsHistorical':
    //  [4 5 5 7 8 6 9]
    // - detect duplicates -> [4 (5) 5 7 8 6 9]
    // - detecting removed cameras -> [4 5 (7 8) 6 9]
    std::vector<std::size_t> filteredNbPoses(allNbPoses);
    std::vector<double> filteredFocalLengths(allFocalLengths);
    
    std::size_t numPosesEndWindow = allNbPoses.back();
    
    for(int id = filteredNbPoses.size()-2; id > 0; --id)
    {
      if(filteredNbPoses.size() < 2)
        break;
      
      if(filteredNbPoses.at(id) >= filteredNbPoses.at(id+1))
      {
        filteredNbPoses.erase(filteredNbPoses.begin()+id);
        filteredFocalLengths.erase(filteredFocalLengths.begin()+id);
      }
    }
    
    // detect limit according to 'kWindowSize':
    if(numPosesEndWindow < windowSize)
      continue;
    
    IndexT idStartWindow = 0;
    for(int id = filteredNbPoses.size()-2; id > 0; --id)
    {
      if(numPosesEndWindow - filteredNbPoses.at(id) >= windowSize)
      {
        idStartWindow = id;
        break;
      }
    }
    
    // compute the standard deviation for each parameter, between [idLimit; end()]
    std::vector<double> subValuesVec (filteredFocalLengths.begin()+idStartWindow, filteredFocalLengths.end());
    double stdev = standardDeviation(subValuesVec);
    
    // normalize stdev (: divide by the range of the values)
    double minVal = *std::min_element(filteredFocalLengths.begin(), filteredFocalLengths.end());
    double maxVal = *std::max_element(filteredFocalLengths.begin(), filteredFocalLengths.end());
    double normStdev = stdev / (maxVal - minVal);
    
    // check if the normed standard deviation is < stdevPercentageLimit
    if(normStdev * 100.0 <= stdevPercentageLimit)
    {
      _mapFocalIsConstant.at(idIntrinsic) = true;
      removeIntrinsicEdgesFromTheGraph(idIntrinsic);
      numOfConstFocal++;
      ALICEVISION_LOG_DEBUG("The intrinsic #" << idIntrinsic << " is now considered to be stable.\n");
    }
  }
  ALICEVISION_LOG_DEBUG(numOfConstFocal << "/" << _mapFocalIsConstant.size() << " intrinsics with a stable focal.");
}

template<typename T> 
double LocalBundleAdjustmentGraph::standardDeviation(const std::vector<T>& data)
{ 
  const double mean = std::accumulate(data.begin(), data.end(), 0.0) / data.size();
  std::vector<double> diff(data.size());
  std::transform(data.begin(), data.end(), diff.begin(), [mean](double x) { return x - mean; });
  const double sqSum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
  return std::sqrt(sqSum / data.size());
}  



void LocalBundleAdjustmentGraph::drawGraph(const sfmData::SfMData& sfmData, const std::string& folder, const std::string& nameComplement)
{
  if(!fs::exists(folder))
    fs::create_directory(folder);
  
  std::stringstream dotStream;
  dotStream << "digraph lemon_dot_example {" << "\n";
  
  // node
  dotStream << "  node [ shape=ellipse, penwidth=5.0, fontname=Helvetica, fontsize=40 ];" << "\n";
  for(lemon::ListGraph::NodeIt n(_graph); n!=lemon::INVALID; ++n)
  {
    const IndexT viewId = _viewIdPerNode[n];
    const int viewDist = _distancePerViewId[viewId];
    
    std::string color = ", color=";
    if(viewDist == 0) color += "red";
    else if(viewDist == 1 ) color += "green";
    else if(viewDist == 2 ) color += "blue";
    else color += "black";
    dotStream << "  n" << _graph.id(n)
              << " [ label=\"" << viewId << ": D" << viewDist << " K" << sfmData.getViews().at(viewId)->getIntrinsicId() << "\"" << color << "]; " << "\n";
  }
  
  // edge
  dotStream << "  edge [ shape=ellipse, fontname=Helvetica, fontsize=5, color=black ];" << "\n";
  for(lemon::ListGraph::EdgeIt e(_graph); e!=lemon::INVALID; ++e)
  {
    dotStream << "  n" << _graph.id(_graph.u(e)) << " -> " << " n" << _graph.id(_graph.v(e));
    if(_intrinsicEdgesId.find(static_cast<IndexT>(_graph.id(e))) != _intrinsicEdgesId.end())
      dotStream << " [color=red]\n";
    else
      dotStream << "\n";
  }
  dotStream << "}" << "\n";
  
  const std::string dotFilepath = (fs::path(folder) / ("graph_" + std::to_string(_viewIdPerNode.size())  + "_" + nameComplement + ".dot")).string();
  std::ofstream dotFile;
  dotFile.open(dotFilepath);
  dotFile.write(dotStream.str().c_str(), dotStream.str().length());
  dotFile.close();
  
  ALICEVISION_LOG_DEBUG("The graph '"<< dotFilepath << "' has been saved.");
}

std::size_t LocalBundleAdjustmentGraph::addIntrinsicEdgesToTheGraph(const sfmData::SfMData& sfmData, const std::set<IndexT>& newReconstructedViews)
{
  std::map<Pair, IndexT> newIntrinsicEdges;

  for(IndexT newViewId : newReconstructedViews) // for each new view
  {
    IndexT newViewIntrinsicId = sfmData.getViews().at(newViewId)->getIntrinsicId();
    
    if(isFocalLengthConstant(newViewIntrinsicId)) // do not add edges for a constant intrinsic
      continue;

    // for each reconstructed view in the graph
    // warning: at this point, "_nodePerViewId" already contains the "newReconstructedViews"
    for(const auto& x : _nodePerViewId)
    {
      const auto& otherViewId = x.first;

      // if the new view share the same intrinsic than previously reconstructed views
      // note: do not compare a view with itself (must be tested since "_nodePerViewId" contains "newReconstructedViews")
      if(newViewId != otherViewId
         && newViewIntrinsicId == sfmData.getViews().at(otherViewId)->getIntrinsicId())
      {
        // register a new intrinsic edge between those views
        newIntrinsicEdges[std::minmax(otherViewId, newViewId)] = newViewIntrinsicId;
      }
    }
  }

  // create registered intrinsic edges in lemon graph 
  // and update _intrinsicEdgesId accordingly
  for(const auto& newEdge : newIntrinsicEdges)
  {
    const lemon::ListGraph::Edge edge = _graph.addEdge(_nodePerViewId[newEdge.first.first], _nodePerViewId[newEdge.first.second]);
    _intrinsicEdgesId[newEdge.second].push_back(_graph.id(edge));
  }
  return newIntrinsicEdges.size();
}

void LocalBundleAdjustmentGraph::removeIntrinsicEdgesFromTheGraph(IndexT intrinsicId)
{
  if(_intrinsicEdgesId.count(intrinsicId) == 0)
    return;
  for(const int edgeId : _intrinsicEdgesId.at(intrinsicId))
  {
    const auto& edge = _graph.edgeFromId(edgeId);
    assert(_graph.valid(edge));
    _graph.erase(edge);
  }
  _intrinsicEdgesId.erase(intrinsicId);
}


std::size_t LocalBundleAdjustmentGraph::updateRigEdgesToTheGraph(const sfmData::SfMData& sfmData)
{
  std::size_t numAddedEdges = 0;

  // remove all rig edges
  for(auto& edgesPerRid: _rigEdgesId)
  {
    for(const int edgeId : edgesPerRid.second)
    {
      const auto& edge = _graph.edgeFromId(edgeId);
      assert(_graph.valid(edge));
      _graph.erase(edge);
    }
  }
  _rigEdgesId.clear();

  // recreate rig edges
  std::map<IndexT, std::vector<IndexT>> viewIdsPerRig;

  for(const auto& viewNode : _nodePerViewId) // for each reconstructed view in the graph
  {
    const sfmData::View& view = sfmData.getView(viewNode.first);
    if(view.isPoseIndependant())
      continue;
    viewIdsPerRig[view.getRigId()].push_back(viewNode.first);
  }

  for(auto& it : viewIdsPerRig)
    std::sort(it.second.begin(), it.second.end());

  for(const auto& it : viewIdsPerRig)
  {
    // if(sfmData.getRig(rigId).isLocked()) // TODO
    //   continue;
    const IndexT rigId = it.first;
    const std::vector<IndexT>& views = it.second;
    for(int i = 0; i < views.size(); ++i)
    {
      for(int j = i; j < views.size(); ++j)
      {
        lemon::ListGraph::Edge edge = _graph.addEdge(_nodePerViewId[views[i]], _nodePerViewId[views[j]]);
        _rigEdgesId[rigId].push_back(_graph.id(edge));
        numAddedEdges++;
      }
    }
  }

  return numAddedEdges;
}

unsigned int LocalBundleAdjustmentGraph::countNodes() const
{
  unsigned int count = 0;
  for(lemon::ListGraph::NodeIt n(_graph); n != lemon::INVALID; ++n)
    ++count;
  return count;
}

unsigned int LocalBundleAdjustmentGraph::countEdges() const
{
  unsigned int count = 0;
  for(lemon::ListGraph::EdgeIt e(_graph); e != lemon::INVALID; ++e)
    ++count;
  return count;
}

} // namespace sfm
} // namespace aliceVision
