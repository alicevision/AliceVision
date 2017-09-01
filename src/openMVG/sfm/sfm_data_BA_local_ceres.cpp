// Copyright (c) 2015 Pierre Moulon.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "openMVG/sfm/sfm_data_BA_local_ceres.hpp"
#include <openMVG/config.hpp>
#include <openMVG/openmvg_omp.hpp>

#include "openMVG/stl/stlMap.hpp"
#include "ceres/rotation.h"
#include "lemon/bfs.h"
//#include <lemon/core.h>

//#include <lemon/bits/map_extender.h>
//#include <lemon/bits/default_map.h>

//#include <lemon/concept_check.h>
//#include <lemon/concepts/maps.h>
//#include "lemon/lgf_writer.h"
//#include "lemon/lgf_reader.h"
//#include "lemon/bits/graph_extender.h"

namespace openMVG {
namespace sfm {

using namespace openMVG::cameras;
using namespace openMVG::geometry;

Local_Bundle_Adjustment_Ceres::Local_Bundle_Adjustment_Ceres(
  Local_Bundle_Adjustment_Ceres::LocalBA_options options)
  : _LBA_openMVG_options(options),
  _map_node_viewId(_reconstructionGraph) 
{}

bool Local_Bundle_Adjustment_Ceres::Adjust(SfM_Data& sfm_data)
{

  //----------
  // Add camera parameters
  // - intrinsics
  // - poses [R|t]
  
  // Create residuals for each observation in the bundle adjustment problem. The
  // parameters for cameras and points are added automatically.
  //----------
  
  ceres::Solver::Options solver_options;
  setSolverOptions(solver_options);
  if (_LBA_openMVG_options.isParameterOrderingEnabled()) 
    solver_options.linear_solver_ordering.reset(new ceres::ParameterBlockOrdering);

  ceres::Problem problem;
  
  // Data wrapper for refinement:
  Hash_Map<IndexT, std::vector<double> > map_posesBlocks;
  Hash_Map<IndexT, std::vector<double> > map_intrinsicsBlocks;

  // Add Poses data to the Ceres problem as Parameter Blocks (do not take care of Local BA strategy)
  map_posesBlocks = addPosesToCeresProblem(sfm_data.poses, problem);
  
  // Add Intrinsics data to the Ceres problem as Parameter Blocks (do not take care of Local BA strategy)
  map_intrinsicsBlocks = addIntrinsicsToCeresProblem(sfm_data, problem);
     
  // Count the number of Refined, Constant & Ignored parameters
  if (_LBA_openMVG_options.isLocalBAEnabled())
  {
    for (auto it : map_intrinsicsBlocks)
    {
      IndexT intrinsicId = it.first;
      if (getIntrinsicsState(intrinsicId) == LocalBAState::refined)  ++_LBA_statistics.numRefinedIntrinsics;
      if (getIntrinsicsState(intrinsicId) == LocalBAState::constant) ++_LBA_statistics.numConstantIntrinsics;
      if (getIntrinsicsState(intrinsicId) == LocalBAState::ignored)  ++_LBA_statistics.numIgnoredIntrinsics;
    }
    for (auto it : map_posesBlocks)
    {
      IndexT poseId = it.first;
      if (getPoseState(poseId) == LocalBAState::refined)  ++_LBA_statistics.numRefinedPoses;
      if (getPoseState(poseId) == LocalBAState::constant) ++_LBA_statistics.numConstantPoses;
      if (getPoseState(poseId) == LocalBAState::ignored)  ++_LBA_statistics.numIgnoredPoses;
    }
  }   
  else
  {
    _LBA_statistics.numRefinedPoses = map_posesBlocks.size();
    _LBA_statistics.numRefinedIntrinsics = map_intrinsicsBlocks.size();
  }
  
  // Set a LossFunction to be less penalized by false measurements
  //  - set it to NULL if you don't want use a lossFunction.
  ceres::LossFunction * p_LossFunction = new ceres::HuberLoss(Square(4.0));
  // TODO: make the LOSS function and the parameter an option

  // For all visibility add reprojections errors:
  for(auto& landmarkIt: sfm_data.structure)
  {             
    IndexT landmarkId = landmarkIt.first;
    
    // Count the number of Refined, Constant & Ignored landmarks
    if (getLandmarkState(landmarkId) == LocalBAState::refined)  ++_LBA_statistics.numRefinedLandmarks;
    if (getLandmarkState(landmarkId) == LocalBAState::constant) ++_LBA_statistics.numConstantLandmarks;
    if (getLandmarkState(landmarkId) == LocalBAState::ignored)  ++_LBA_statistics.numIgnoredLandmarks;
            
    const Observations & observations = landmarkIt.second.observations;
    // Iterate over 2D observation associated to the 3D landmark
    for (const auto& observationIt: observations)
    {
      // Build the residual block corresponding to the track observation:
      const View * view = sfm_data.views.at(observationIt.first).get();
      IndexT intrinsicId = view->id_intrinsic;
      IndexT poseId = view->id_pose;
      
      // Do not create a residual block if the pose, the intrinsic or the landmark 
      // have been set as Ignored by the Local BA strategy
      if (_LBA_openMVG_options.isLocalBAEnabled())
      {
        if (getPoseState(poseId) == LocalBAState::ignored 
          || getIntrinsicsState(intrinsicId) == LocalBAState::ignored 
          || getLandmarkState(landmarkId) == LocalBAState::ignored)
        {
          continue;
        }
      }
 
      // Each Residual block takes a point and a camera as input and outputs a 2
      // dimensional residual. Internally, the cost function stores the observed
      // image location and compares the reprojection against the observation.
      ceres::CostFunction* cost_function =
          IntrinsicsToCostFunction(sfm_data.intrinsics[intrinsicId].get(), observationIt.second.x);
      
      if (cost_function)
      {
        // Needed parameters to create a residual block (K, pose & landmark)
        double* intrinsicBlock = &map_intrinsicsBlocks[intrinsicId][0];
        double* poseBlock = &map_posesBlocks[poseId][0];
        double* landmarkBlock = landmarkIt.second.X.data();

        // Apply a specific parameter ordering: 
        if (_LBA_openMVG_options.isParameterOrderingEnabled()) 
        {
          solver_options.linear_solver_ordering->AddElementToGroup(landmarkBlock, 0);
          solver_options.linear_solver_ordering->AddElementToGroup(intrinsicBlock, 1);
          solver_options.linear_solver_ordering->AddElementToGroup(poseBlock, 2);
        }
        // Set to constant parameters previoously set as Constant by the Local BA strategy
        if (_LBA_openMVG_options.isLocalBAEnabled())
        {
          if (getIntrinsicsState(intrinsicId) == LocalBAState::constant) problem.SetParameterBlockConstant(intrinsicBlock);        
          if (getPoseState(poseId) == LocalBAState::constant)            problem.SetParameterBlockConstant(poseBlock);
          if (getLandmarkState(landmarkId) == LocalBAState::constant)    problem.SetParameterBlockConstant(landmarkBlock);
        } 
        // Create a residual block:
        problem.AddResidualBlock(cost_function,
                                 p_LossFunction,
                                 intrinsicBlock,
                                 poseBlock,
                                 landmarkBlock); //Do we need to copy 3D point to avoid false motion, if failure ?
      }
    }
  }
    
  ceres::Solver::Summary summary;
  if (!solveBA(problem, solver_options, summary))
    return false;
  
  // Solution is usable
  if (_LBA_openMVG_options._bVerbose)
  {
    // Display statistics about the minimization
    OPENMVG_LOG_DEBUG(
      "Bundle Adjustment statistics (approximated RMSE):\n"
      " #views: " << sfm_data.views.size() << "\n"
      " #poses: " << sfm_data.poses.size() << "\n"
      " #intrinsics: " << sfm_data.intrinsics.size() << "\n"
      " #tracks: " << sfm_data.structure.size() << "\n"
      " #residuals: " << summary.num_residuals << "\n"
      " Initial RMSE: " << std::sqrt( summary.initial_cost / summary.num_residuals) << "\n"
      " Final RMSE: " << std::sqrt( summary.final_cost / summary.num_residuals) << "\n"
      " Time (s): " << summary.total_time_in_seconds << "\n"
      );
  }

  if (_LBA_openMVG_options._bVerbose && _LBA_openMVG_options.isParameterOrderingEnabled())
  {
  // Display statistics about "parameter ordering"
    OPENMVG_LOG_DEBUG(
      "Parameter ordering statistics:\n"
      " (group 0 (landmarks)): " << solver_options.linear_solver_ordering->GroupSize(0) << "\n"
      " (group 1 (intrinsics)): " << solver_options.linear_solver_ordering->GroupSize(1) << "\n"
      " (group 2 (poses)): " << solver_options.linear_solver_ordering->GroupSize(2) << "\n"
      );
  }
  
  // Add statitics about the BA loop:
  _LBA_statistics.time = summary.total_time_in_seconds;
  _LBA_statistics.numSuccessfullIterations = summary.num_successful_steps;
  _LBA_statistics.numUnsuccessfullIterations = summary.num_unsuccessful_steps;
  _LBA_statistics.numResidualBlocks = summary.num_residuals;
  _LBA_statistics.RMSEinitial = std::sqrt( summary.initial_cost / summary.num_residuals);
  _LBA_statistics.RMSEfinal = std::sqrt( summary.final_cost / summary.num_residuals);
  
  if (_LBA_openMVG_options._bVerbose && _LBA_openMVG_options.isLocalBAEnabled())
  {
    // Generate the histogram <distance, NbOfPoses>
    for (auto it: _map_poseId_distance) // distanceToRecentPoses: <poseId, distance>
    {
      auto itHisto = _LBA_statistics.map_distance_numCameras.find(it.second);
      if(itHisto != _LBA_statistics.map_distance_numCameras.end())
        ++_LBA_statistics.map_distance_numCameras.at(it.second);
      else // first pose with this specific distance
          _LBA_statistics.map_distance_numCameras[it.second] = 1;
    }
    
    // Display statistics about the Local BA
    OPENMVG_LOG_DEBUG(
      "Local BA statistics:\n"
      " #poses: " << _LBA_statistics.numRefinedPoses << " refined, " 
        << _LBA_statistics.numConstantPoses << " constant, "
        << _LBA_statistics.numIgnoredPoses << " ignored.\n"
      " #intrinsics: " << _LBA_statistics.numRefinedIntrinsics << " refined, " 
        << _LBA_statistics.numConstantIntrinsics << " constant, "
        << _LBA_statistics.numIgnoredIntrinsics<< " ignored.\n"   
      " #landmarks: " << _LBA_statistics.numRefinedLandmarks << " refined, " 
        << _LBA_statistics.numConstantLandmarks << " constant, "
        << _LBA_statistics.numIgnoredLandmarks << " ignored.\n"
    );
  }

  // Update camera poses with refined data
  updateCameraPoses(map_posesBlocks, sfm_data.poses);

  // Update camera intrinsics with refined data
  updateCameraIntrinsics(map_intrinsicsBlocks, sfm_data.intrinsics);

  return true;
}

void Local_Bundle_Adjustment_Ceres::updateDistancesGraph(
  const SfM_Data& sfm_data, 
  const tracks::TracksPerView& map_tracksPerView, 
  const std::set<IndexT>& newViewIds)
{
  std::set<IndexT> viewIdsAddedToTheGraph;
  
  // -- Add nodes (= views)
  // Identify the view we need to add to the graph:
  if (_map_viewId_node.empty()) // is the fisrt Local BA
  {
    for (auto & it : sfm_data.GetPoses())
      viewIdsAddedToTheGraph.insert(it.first);
  }
  else // not the first Local BA
    viewIdsAddedToTheGraph = newViewIds; 
  
  // Add the views as nodes to the graph:
  for (auto& viewId : viewIdsAddedToTheGraph)
  {
    lemon::ListGraph::Node newNode = _reconstructionGraph.addNode();
    _map_node_viewId.set(newNode, viewId);
    _map_viewId_node[viewId] = newNode;  
  }
      
  // -- Add edge.
  // An edge is created between 2 views when they share at least 'L' landmarks (by default L=100).
  // At first, we need to count the number of shared landmarks between all the new views 
  // and each already resected cameras (already in the graph)
  std::map<Pair, std::size_t> map_nbSharedLandmarksPerImagesPair;
  
  std::set<IndexT> landmarkIds;
  std::transform(sfm_data.GetLandmarks().begin(), sfm_data.GetLandmarks().end(),
    std::inserter(landmarkIds, landmarkIds.begin()),
    stl::RetrieveKey());

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
        auto it = map_nbSharedLandmarksPerImagesPair.find(viewPair);
        if(it == map_nbSharedLandmarksPerImagesPair.end())  // the first common landmark
          map_nbSharedLandmarksPerImagesPair[viewPair] = 1;
        else
          it->second++;
      }
    }
  }
  // add edges in the graph
  for(auto& it: map_nbSharedLandmarksPerImagesPair)
  {
    std::size_t L = 100; // typically: 100
    if(it.second > L) // ensure a minimum number of landmarks in common to consider the link
    {
      _reconstructionGraph.addEdge(_map_viewId_node.at(it.first.first), _map_viewId_node.at(it.first.second));
    }
  }
}

void Local_Bundle_Adjustment_Ceres::computeDistancesMaps(
  const SfM_Data& sfm_data, 
  const std::set<IndexT>& newViewIds,
  const tracks::TracksPerView& map_tracksPerView)
{  
  _map_viewId_distance.clear();
  _map_poseId_distance.clear();
  
  // Update the 'reconstructionGraph' using the recently added cameras
  updateDistancesGraph(sfm_data, map_tracksPerView, newViewIds);

  // Setup Breadth First Search using Lemon
  lemon::Bfs<lemon::ListGraph> bfs(_reconstructionGraph);
  bfs.init();

  // Add source views for the bfs visit of the _reconstructionGraph
  for(const IndexT viewId: newViewIds)
  {
    bfs.addSource(_map_viewId_node.find(viewId)->second);
  }
  bfs.start();

  // Handle bfs results (distances)
  for(auto it: _map_viewId_node)
  {
    auto& node = it.second;
    int d = bfs.dist(node);
    _map_viewId_distance[it.first] = d;
  }

  // Re-mapping: from <ViewId, distance> to <PoseId, distance>:
  for(auto it: _map_viewId_distance)
  {
    // Get the poseId of the camera no. viewId
    IndexT idPose = sfm_data.GetViews().at(it.first)->id_pose; // PoseId of a resected camera

    auto poseIt = _map_poseId_distance.find(idPose);
    // If multiple views share the same pose
    if(poseIt != _map_poseId_distance.end())
      poseIt->second = std::min(poseIt->second, it.second);
    else
      _map_poseId_distance[idPose] = it.second;
  } 

  // Display result: viewId -> distance to recent cameras
  {    
    OPENMVG_LOG_INFO("-- View distance map: ");
    for (auto & itMap: _map_viewId_distance)
    {
      OPENMVG_LOG_INFO( itMap.first << " -> " << itMap.second);
    }
  }
}

void Local_Bundle_Adjustment_Ceres::computeStatesMaps(const SfM_Data & sfm_data, const LocalBAStrategy& strategy, const std::size_t distanceLimit)
{
  // reset the maps
  _map_poseId_LBAState.clear();
  _map_intrinsicId_LBAState.clear();
  _map_landmarkId_LBAState.clear();

  switch(strategy)
  {
    case LocalBAStrategy::strategy_1:
    {
      // ----------------------------------------------------
      // -- Strategy 1 : (2017.07.14)
      //  D = 1
      //  - cameras:
      //    - dist <= D: refined
      //    - else fixed
      //  - all intrinsics refined
      //  - landmarks:
      //    - connected to a refined camera: refined
      //    - else ignored
      // ----------------------------------------------------
      // -- Poses
      for (Poses::const_iterator itPose = sfm_data.poses.begin(); itPose != sfm_data.poses.end(); ++itPose)
      {
        const IndexT poseId = itPose->first;
        int dist = _map_poseId_distance.at(poseId);
        if (dist <= distanceLimit) // 0 or 1
          _map_poseId_LBAState[poseId] = LocalBAState::refined;
        else
          _map_poseId_LBAState[poseId] = LocalBAState::constant;
      }

      // -- Instrinsics
      for(const auto& itIntrinsic: sfm_data.GetIntrinsics())
      {
        const IndexT intrinsicId = itIntrinsic.first;
        _map_intrinsicId_LBAState[intrinsicId] = LocalBAState::refined;
      }

      // -- Landmarks
      for(const auto& itLandmark: sfm_data.structure)
      {
        const IndexT landmarkId = itLandmark.first;
        const Observations & observations = itLandmark.second.observations;

        _map_landmarkId_LBAState[landmarkId] = LocalBAState::ignored;

        for(const auto& observationIt: observations)
        {
          int dist = _map_viewId_distance.at(observationIt.first);
          if(dist <= distanceLimit)
          {
            _map_landmarkId_LBAState[landmarkId] = LocalBAState::refined;
            continue;
          }
        }
      }
    }
    break;

    case LocalBAStrategy::strategy_2 :
    {
      // ----------------------------------------------------
      // -- Strategy 2 : (2017.07.19)
      //  D = 1
      //  - cameras:
      //    - dist <= D: refined
      //    - dist == D+1: fixed
      //    - else ignored
      //  - all intrinsics refined
      //  - landmarks:
      //    - connected to a refined camera: refined
      //    - else ignored
      // ----------------------------------------------------

      // -- Poses
      for (Poses::const_iterator itPose = sfm_data.poses.begin(); itPose != sfm_data.poses.end(); ++itPose)
      {
        const IndexT poseId = itPose->first;
        int dist = _map_poseId_distance.at(poseId);
        if (dist <= distanceLimit) // 0 or 1
          _map_poseId_LBAState[poseId] = LocalBAState::refined;
        else if (dist == distanceLimit + 1)
          _map_poseId_LBAState[poseId] = LocalBAState::constant;
        else
          _map_poseId_LBAState[poseId] = LocalBAState::ignored;
      }

      // -- Instrinsics
      for(const auto& itIntrinsic: sfm_data.GetIntrinsics())
      {
        const IndexT intrinsicId = itIntrinsic.first;
        _map_intrinsicId_LBAState[intrinsicId] = LocalBAState::refined;
      }

      // -- Landmarks
      for(const auto& itLandmark: sfm_data.structure)
      {
        const IndexT landmarkId = itLandmark.first;
        const Observations & observations = itLandmark.second.observations;

        _map_landmarkId_LBAState[landmarkId] = LocalBAState::ignored;

        for(const auto& observationIt: observations)
        {
          int dist = _map_viewId_distance.at(observationIt.first);
          if(dist <= distanceLimit)
          {
            _map_landmarkId_LBAState[landmarkId] = LocalBAState::refined;
            continue;
          }
        }
      }
    }
    break;
      
    default:
    {
      // ----------------------------------------------------
      // -- All parameters are refined = NO LOCAL BA
      // ----------------------------------------------------
      // Poses
      for (Poses::const_iterator itPose = sfm_data.poses.begin(); itPose != sfm_data.poses.end(); ++itPose)
      {
        const IndexT poseId = itPose->first;
        _map_poseId_LBAState[poseId] = LocalBAState::refined;
      }
      // Instrinsics
      for(const auto& itIntrinsic: sfm_data.GetIntrinsics())
      {
        const IndexT intrinsicId = itIntrinsic.first;
        _map_intrinsicId_LBAState[intrinsicId] = LocalBAState::refined;
      }
      // Landmarks
      for(const auto& itLandmark: sfm_data.structure)
      {
        const IndexT landmarkId = itLandmark.first;
        _map_landmarkId_LBAState[landmarkId] = LocalBAState::refined;
      }
    }
    break;
  }
}

Hash_Map<IndexT, std::vector<double> > Local_Bundle_Adjustment_Ceres::addPosesToCeresProblem(
  const Poses & poses,
  ceres::Problem & problem)
{
  // Data wrapper for refinement:
  Hash_Map<IndexT, std::vector<double> > map_poses;
  
  // Setup Poses data 
  for (Poses::const_iterator itPose = poses.begin(); itPose != poses.end(); ++itPose)
  {
    const IndexT poseId = itPose->first;
       
    const Pose3 & pose = itPose->second;
    const Mat3 R = pose.rotation();
    const Vec3 t = pose.translation();
    
    double angleAxis[3];
    ceres::RotationMatrixToAngleAxis((const double*)R.data(), angleAxis);
    map_poses[poseId].reserve(6); //angleAxis + translation
    map_poses[poseId].push_back(angleAxis[0]);
    map_poses[poseId].push_back(angleAxis[1]);
    map_poses[poseId].push_back(angleAxis[2]);
    map_poses[poseId].push_back(t(0));
    map_poses[poseId].push_back(t(1));
    map_poses[poseId].push_back(t(2));
    
    double * parameter_block = &map_poses[poseId][0];
    problem.AddParameterBlock(parameter_block, 6);
  }
  return map_poses;
}

Hash_Map<IndexT, std::vector<double>> Local_Bundle_Adjustment_Ceres::addIntrinsicsToCeresProblem(
  const SfM_Data & sfm_data,
  ceres::Problem & problem)
{
  Hash_Map<IndexT, std::size_t> intrinsicsUsage;
  
  // Setup Intrinsics data 
  // Count how many posed views use each intrinsic
  for(const auto& itView: sfm_data.GetViews())
  {
    const View* view = itView.second.get();
    if (sfm_data.IsPoseAndIntrinsicDefined(view))
    {
      if(intrinsicsUsage.find(view->id_intrinsic) == intrinsicsUsage.end())
        intrinsicsUsage[view->id_intrinsic] = 1;
      else
        ++intrinsicsUsage[view->id_intrinsic];
    }
    else
    {
      if(intrinsicsUsage.find(view->id_intrinsic) == intrinsicsUsage.end())
        intrinsicsUsage[view->id_intrinsic] = 0;
    }
  }
  
  Hash_Map<IndexT, std::vector<double>> map_intrinsics;
  for(const auto& itIntrinsic: sfm_data.GetIntrinsics())
  {
    const IndexT intrinsicIds = itIntrinsic.first;
    
    // Do not refine an intrinsic does not used by any reconstructed view
    if(intrinsicsUsage[intrinsicIds] == 0)
      continue;

    assert(isValid(itIntrinsic.second->getType()));
    map_intrinsics[intrinsicIds] = itIntrinsic.second->getParams();
    
    double * parameter_block = &map_intrinsics[intrinsicIds][0];
    problem.AddParameterBlock(parameter_block, map_intrinsics[intrinsicIds].size());
    
    // Refine the focal length
    if(itIntrinsic.second->initialFocalLengthPix() > 0)
    {
      // If we have an initial guess, we only authorize a margin around this value.
      assert(map_intrinsics[intrinsicIds].size() >= 1);
      const unsigned int maxFocalErr = 0.2 * std::max(itIntrinsic.second->w(), itIntrinsic.second->h());
      problem.SetParameterLowerBound(parameter_block, 0, (double)itIntrinsic.second->initialFocalLengthPix() - maxFocalErr);
      problem.SetParameterUpperBound(parameter_block, 0, (double)itIntrinsic.second->initialFocalLengthPix() + maxFocalErr);
    }
    else // no initial guess
    {
      // We don't have an initial guess, but we assume that we use
      // a converging lens, so the focal length should be positive.
      problem.SetParameterLowerBound(parameter_block, 0, 0.0);
    }
    
    const std::size_t minImagesForOpticalCenter = 3;
    
    // Optical center
    // Refine optical center within 10% of the image size.
    assert(map_intrinsics[intrinsicIds].size() >= 3);
    
    const double opticalCenterMinPercent = 0.45;
    const double opticalCenterMaxPercent = 0.55;
    
    // Add bounds to the principal point
    problem.SetParameterLowerBound(parameter_block, 1, opticalCenterMinPercent * itIntrinsic.second->w());
    problem.SetParameterUpperBound(parameter_block, 1, opticalCenterMaxPercent * itIntrinsic.second->w());
    
    problem.SetParameterLowerBound(parameter_block, 2, opticalCenterMinPercent * itIntrinsic.second->h());
    problem.SetParameterUpperBound(parameter_block, 2, opticalCenterMaxPercent * itIntrinsic.second->h());
  }
  return map_intrinsics;
} 

/// Transfert the BA options from OpenMVG to Ceres
void Local_Bundle_Adjustment_Ceres::setSolverOptions(ceres::Solver::Options& solver_options)
{
  solver_options.preconditioner_type = _LBA_openMVG_options._preconditioner_type;
  solver_options.linear_solver_type = _LBA_openMVG_options._linear_solver_type;
  solver_options.sparse_linear_algebra_library_type = _LBA_openMVG_options._sparse_linear_algebra_library_type;
  solver_options.minimizer_progress_to_stdout = _LBA_openMVG_options._bVerbose;
  solver_options.logging_type = ceres::SILENT;
  solver_options.num_threads = _LBA_openMVG_options._nbThreads;
  solver_options.num_linear_solver_threads = _LBA_openMVG_options._nbThreads;
}

bool Local_Bundle_Adjustment_Ceres::solveBA(
  ceres::Problem& problem, 
  ceres::Solver::Options& options, 
  ceres::Solver::Summary& summary)
{
  // Configure a BA engine and run it
  // Solve BA
  
  ceres::Solve(options, &problem, &summary);
  if (_LBA_openMVG_options._bCeres_Summary)
    OPENMVG_LOG_DEBUG(summary.FullReport());
  
  // If no error, get back refined parameters
  if (!summary.IsSolutionUsable())
  {
    OPENMVG_LOG_WARNING("Bundle Adjustment failed.");
    return false;
  }
  return true;
}

void Local_Bundle_Adjustment_Ceres::updateCameraPoses(
  const Hash_Map<IndexT, std::vector<double>> & map_poses,
  Poses & poses)
{
  for (Poses::iterator itPose = poses.begin();
         itPose != poses.end(); ++itPose)
  {
    const IndexT poseId = itPose->first;
    
    // Do not update a camera pose set as Ignored or Constant in the Local BA strategy
    if (_LBA_openMVG_options.isLocalBAEnabled() )
    {
      if (getPoseState(poseId) == ignored) 
        continue;
      if (getPoseState(poseId) == constant) 
        continue;
    }

    Mat3 R_refined;
    ceres::AngleAxisToRotationMatrix(&map_poses.at(poseId)[0], R_refined.data());
    Vec3 t_refined(map_poses.at(poseId)[3], map_poses.at(poseId)[4], map_poses.at(poseId)[5]);
    // Update the pose
    Pose3 & pose = itPose->second;
    pose = Pose3(R_refined, -R_refined.transpose() * t_refined);
  }
}
  
void Local_Bundle_Adjustment_Ceres::updateCameraIntrinsics(
  const Hash_Map<IndexT, std::vector<double>> & map_intrinsics,
  Intrinsics & intrinsics)
{
  for (const auto& intrinsicsV: map_intrinsics)
  {
    const IndexT intrinsicId = intrinsicsV.first;
    
    // Do not update an camera intrinsic set as Ignored or Constant in the Local BA strategy
    if (_LBA_openMVG_options.isLocalBAEnabled() )
    {
      if (getIntrinsicsState(intrinsicId) == ignored) 
        continue;
      if (getIntrinsicsState(intrinsicId) == constant) 
        continue;
    }
    
    intrinsics[intrinsicId]->updateFromParams(intrinsicsV.second);
  }
}

bool Local_Bundle_Adjustment_Ceres::exportStatistics(const std::string& path)
{
  std::string filename = stlplus::folder_append_separator(path)+"BaStats.txt";
  std::ofstream os;
  os.open(filename, std::ios::app);
  os.seekp(0, std::ios::end); //put the cursor at the end
  if (!os.is_open())
  {
    OPENMVG_LOG_DEBUG("Unable to open the Bundle adjustment stat file '" << filename << "'.");
    return false;
  }
    
  if (os.tellp() == 0) // 'tellp' return the cursor's position
  {
    // If the file does't exist: add a header.
    std::vector<std::string> header;
    header.push_back("Time/BA(s)");
    header.push_back("RefinedPose"); header.push_back("ConstPose");  header.push_back("IgnoredPose");
    header.push_back("RefinedPts");  header.push_back("ConstPts");   header.push_back("IgnoredPts");
    header.push_back("RefinedK");    header.push_back("ConstK");     header.push_back("IgnoredK");
    header.push_back("ResidualBlocks");
    header.push_back("SuccessIter"); header.push_back("BadIter");
    header.push_back("InitRMSE"); header.push_back("FinalRMSE");
    header.push_back("dAR=0"); header.push_back("dAR=1"); header.push_back("dAR=2");
    header.push_back("dAR=3"); header.push_back("dAR=4"); header.push_back("dAR=5");   
    header.push_back("dAR=6"); header.push_back("dAR=7"); header.push_back("dAR=8"); 
    header.push_back("dAR=9"); header.push_back("dAR=10+");
    header.push_back("New Views");
    
    for (std::string & head : header)
      os << head << "\t";
    os << "\n"; 
  }
  
  // Add the '_LBA_statistics' contents:
  // Compute the number of poses with a distanceToRecenteCameras > 10
  std::size_t posesWthDistUpperThanTen = 0;
  if (_LBA_statistics.map_distance_numCameras.size() >= 10)
  {
    for (int i=10; i<_LBA_statistics.map_distance_numCameras.size(); ++i)
    {
      posesWthDistUpperThanTen += _LBA_statistics.map_distance_numCameras.at(i);
    }
  }
  
  os << _LBA_statistics.time << "\t"
        
     << _LBA_statistics.numRefinedPoses << "\t"
     << _LBA_statistics.numConstantPoses << "\t"
     << _LBA_statistics.numIgnoredPoses << "\t"
     << _LBA_statistics.numRefinedLandmarks << "\t"
     << _LBA_statistics.numConstantLandmarks << "\t"
     << _LBA_statistics.numIgnoredLandmarks << "\t"
     << _LBA_statistics.numRefinedIntrinsics << "\t"
     << _LBA_statistics.numConstantIntrinsics << "\t"
     << _LBA_statistics.numIgnoredIntrinsics << "\t"
        
     << _LBA_statistics.numResidualBlocks << "\t"
     << _LBA_statistics.numSuccessfullIterations << "\t"
     << _LBA_statistics.numUnsuccessfullIterations << "\t"
        
     << _LBA_statistics.RMSEinitial << "\t"
     << _LBA_statistics.RMSEfinal << "\t"
        
     << _LBA_statistics.map_distance_numCameras[0] << "\t"
     << _LBA_statistics.map_distance_numCameras[1] << "\t"
     << _LBA_statistics.map_distance_numCameras[2] << "\t"
     << _LBA_statistics.map_distance_numCameras[3] << "\t"
     << _LBA_statistics.map_distance_numCameras[4] << "\t"
     << _LBA_statistics.map_distance_numCameras[5] << "\t"
     << _LBA_statistics.map_distance_numCameras[6] << "\t"
     << _LBA_statistics.map_distance_numCameras[7] << "\t"
     << _LBA_statistics.map_distance_numCameras[8] << "\t"
     << _LBA_statistics.map_distance_numCameras[9] << "\t"
     << posesWthDistUpperThanTen << "\t";
  
  for (const IndexT id : _LBA_statistics.newViewsId)
  {
    os << id << "\t";
  }
  os << "\n";
  os.close();
  
  return true;
}

} // namespace sfm
} // namespace openMVG

