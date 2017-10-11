// Copyright (c) 2015 Pierre Moulon.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "openMVG/sfm/sfm_data_BA_local_ceres.hpp"
#include "openMVG/sfm/pipelines/sequential/sequential_SfM.hpp"
#include <openMVG/config.hpp>
#include <openMVG/openmvg_omp.hpp>

#include "openMVG/stl/stlMap.hpp"
#include "ceres/rotation.h"
#include "lemon/bfs.h"
#include <fstream>

//#include "openMVG/tracks/tracks.hpp"

namespace openMVG {
namespace sfm {

using namespace openMVG::cameras;
using namespace openMVG::geometry;

Local_Bundle_Adjustment_Ceres::Local_Bundle_Adjustment_Ceres(Local_Bundle_Adjustment_Ceres::LocalBA_options options)
  : 
    _LBA_openMVG_options(options)
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
  map_posesBlocks = addPosesToCeresProblem(sfm_data.GetPoses(), problem);
  
  // Add Intrinsics data to the Ceres problem as Parameter Blocks (do not take care of Local BA strategy)
  map_intrinsicsBlocks = addIntrinsicsToCeresProblem(sfm_data, problem);
  
  // Count the number of Refined, Constant & Ignored parameters
  if (_LBA_openMVG_options.isLocalBAEnabled())
  {
    for (auto it : map_intrinsicsBlocks)
    {
      IndexT intrinsicId = it.first;
      if (getIntrinsicsState(intrinsicId) == ELocalBAState::refined)  ++_LBA_statistics.numRefinedIntrinsics;
      if (getIntrinsicsState(intrinsicId) == ELocalBAState::constant) ++_LBA_statistics.numConstantIntrinsics;
      if (getIntrinsicsState(intrinsicId) == ELocalBAState::ignored)  ++_LBA_statistics.numIgnoredIntrinsics;
    }
    for (auto it : map_posesBlocks)
    {
      IndexT poseId = it.first;
      if (getPoseState(poseId) == ELocalBAState::refined)  ++_LBA_statistics.numRefinedPoses;
      if (getPoseState(poseId) == ELocalBAState::constant) ++_LBA_statistics.numConstantPoses;
      if (getPoseState(poseId) == ELocalBAState::ignored)  ++_LBA_statistics.numIgnoredPoses;
    }
  }   
  else
  {
    _LBA_statistics.numRefinedPoses = map_posesBlocks.size();
    _LBA_statistics.numRefinedIntrinsics = map_intrinsicsBlocks.size();
  }
  
  if (_LBA_statistics.numConstantPoses == 0 && _LBA_statistics.numIgnoredPoses != 0)
  {
    OPENMVG_LOG_WARNING("Local bundle adjustment not executed: There is no pose set to Constant in the solver.");
    // It happens when the added pose(s) is(are) not connected to the rest of the graph.
    return false;
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
    if (getLandmarkState(landmarkId) == ELocalBAState::refined)  ++_LBA_statistics.numRefinedLandmarks;
    if (getLandmarkState(landmarkId) == ELocalBAState::constant) ++_LBA_statistics.numConstantLandmarks;
    if (getLandmarkState(landmarkId) == ELocalBAState::ignored)  ++_LBA_statistics.numIgnoredLandmarks;
    
    const Observations & observations = landmarkIt.second.observations;
    // Iterate over 2D observation associated to the 3D landmark
    for (const auto& observationIt: observations)
    {
      // Build the residual block corresponding to the track observation:
      const View * view = sfm_data.views.at(observationIt.first).get();
      IndexT intrinsicId = view->getIntrinsicId();
      IndexT poseId = view->getPoseId();
      
      // Do not create a residual block if the pose, the intrinsic or the landmark 
      // have been set as Ignored by the Local BA strategy
      if (_LBA_openMVG_options.isLocalBAEnabled())
      {
        if (getPoseState(poseId) == ELocalBAState::ignored 
            || getIntrinsicsState(intrinsicId) == ELocalBAState::ignored 
            || getLandmarkState(landmarkId) == ELocalBAState::ignored)
        {
          continue;
        }
      }
      
      // Each Residual block takes a point and a camera as input and outputs a 2
      // dimensional residual. Internally, the cost function stores the observed
      // image location and compares the reprojection against the observation.
      ceres::CostFunction* cost_function = 
          createCostFunctionFromIntrinsics(sfm_data.intrinsics[intrinsicId].get(), observationIt.second.x);
      
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
          if (getIntrinsicsState(intrinsicId) == ELocalBAState::constant) problem.SetParameterBlockConstant(intrinsicBlock);        
          if (getPoseState(poseId) == ELocalBAState::constant)            problem.SetParameterBlockConstant(poseBlock);
          if (getLandmarkState(landmarkId) == ELocalBAState::constant)    problem.SetParameterBlockConstant(landmarkBlock);
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
  
  OPENMVG_LOG_DEBUG("Solving ceres problem...");
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
                                                  " #poses: " << sfm_data.GetPoses().size() << "\n"
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
          "Parameter ordering statistics:"
          << "\n (group 0 (landmarks)): " << solver_options.linear_solver_ordering->GroupSize(0) 
          << "\n (group 1 (intrinsics)): " << solver_options.linear_solver_ordering->GroupSize(1) 
          << "\n (group 2 (poses)): " << solver_options.linear_solver_ordering->GroupSize(2) << "\n"
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
  updateCameraPoses(map_posesBlocks, sfm_data.GetPoses());
  
  // Update camera intrinsics with refined data
  updateCameraIntrinsics(map_intrinsicsBlocks, sfm_data.intrinsics);
  
  return true;
}

bool Local_Bundle_Adjustment_Ceres::Adjust(const SfM_Data& const_sfm_data)
{
  SfM_Data sfm_data;
  sfm_data._poses = const_sfm_data._poses;
  sfm_data.structure = const_sfm_data.structure;
  // clone views
  for (const auto& it : const_sfm_data.views)
    sfm_data.views[it.first] = std::make_shared<View>(*(it.second));
  sfm_data.intrinsics = const_sfm_data.GetIntrinsics();
  
  return Adjust(sfm_data);
}

void Local_Bundle_Adjustment_Ceres::computeStatesMaps_strategy0(const SfM_Data & sfm_data)
{
  // reset the maps
  _map_poseId_LBAState.clear();
  _map_intrinsicId_LBAState.clear();
  _map_landmarkId_LBAState.clear();
  
  // ----------------------------------------------------
  // -- Strategy 0 : EVERYTHING IS REFINED (No Local BA)
  // ----------------------------------------------------
  
  // -- Poses
  for (Poses::const_iterator itPose = sfm_data.GetPoses().begin(); itPose != sfm_data.GetPoses().end(); ++itPose)
  {
    const IndexT poseId = itPose->first;
    _map_poseId_LBAState[poseId] = ELocalBAState::refined;
  }
  
  // -- Instrinsics
  for(const auto& itIntrinsic: sfm_data.GetIntrinsics())
  {
    const IndexT intrinsicId = itIntrinsic.first;
    _map_intrinsicId_LBAState[intrinsicId] = ELocalBAState::refined;
  }
  
  // -- Landmarks
  for(const auto& itLandmark: sfm_data.structure)
  {
    const IndexT landmarkId = itLandmark.first;
    _map_landmarkId_LBAState[landmarkId] = ELocalBAState::refined;
  }
}

void Local_Bundle_Adjustment_Ceres::computeStatesMaps_strategy1(const SfM_Data & sfm_data,  const std::shared_ptr<LocalBA_Data> localBA_data)
{
  // reset the maps
  _map_poseId_LBAState.clear();
  _map_intrinsicId_LBAState.clear();
  _map_landmarkId_LBAState.clear();
  
  // ----------------------------------------------------
  // -- Strategy 1 : (2017.07.14)
  //  D = distanceLimit
  //  - cameras:
  //    - dist <= D: refined
  //    - else constant
  //  - all intrinsics refined
  //  - landmarks:
  //    - connected to a refined camera: refined
  //    - else ignored
  // ----------------------------------------------------
  const std::size_t kDistanceLimit = 1;
  
  // -- Poses
  for (Poses::const_iterator itPose = sfm_data.GetPoses().begin(); itPose != sfm_data.GetPoses().end(); ++itPose)
  {
    const IndexT poseId = itPose->first;
    int dist = localBA_data->getPoseDistance(poseId);
    if (dist <= kDistanceLimit) // 0 or 1
      _map_poseId_LBAState[poseId] = ELocalBAState::refined;
    else
      _map_poseId_LBAState[poseId] = ELocalBAState::constant;
  }
  
  // -- Instrinsics
  for(const auto& itIntrinsic: sfm_data.GetIntrinsics())
  {
    const IndexT intrinsicId = itIntrinsic.first;
    _map_intrinsicId_LBAState[intrinsicId] = ELocalBAState::refined;
  }
  
  // -- Landmarks
  for(const auto& itLandmark: sfm_data.structure)
  {
    const IndexT landmarkId = itLandmark.first;
    const Observations & observations = itLandmark.second.observations;
    
    _map_landmarkId_LBAState[landmarkId] = ELocalBAState::ignored;
    
    for(const auto& observationIt: observations)
    {
      int dist = localBA_data->getViewDistance(observationIt.first);
      if(dist <= kDistanceLimit)
      {
        _map_landmarkId_LBAState[landmarkId] = ELocalBAState::refined;
        continue;
      }
    }
  }
}

void Local_Bundle_Adjustment_Ceres::computeStatesMaps_strategy2(const SfM_Data & sfm_data, const std::shared_ptr<LocalBA_Data> localBA_data)
{
  // reset the maps
  _map_poseId_LBAState.clear();
  _map_intrinsicId_LBAState.clear();
  _map_landmarkId_LBAState.clear();
  
  // ----------------------------------------------------
  // -- Strategy 2 : (2017.07.19)
  //  D = distanceLimit
  //  - cameras:
  //    - dist <= D: refined
  //    - dist == D+1: fixed      [*NEW*]
  //    - else ignored            [*NEW*]
  //  - all intrinsics refined
  //  - landmarks:
  //    - connected to a refined camera: refined
  //    - else ignored
  // ----------------------------------------------------
  const std::size_t kDistanceLimit = 1;
  
  // -- Poses
  for (Poses::const_iterator itPose = sfm_data.GetPoses().begin(); itPose != sfm_data.GetPoses().end(); ++itPose)
  {
    const IndexT poseId = itPose->first;
    int dist = localBA_data->getPoseDistance(poseId);
    if (dist <= kDistanceLimit) // 0 or 1
      _map_poseId_LBAState[poseId] = ELocalBAState::refined;
    else if (dist == kDistanceLimit + 1) // 2
      _map_poseId_LBAState[poseId] = ELocalBAState::constant;
    else // +2
      _map_poseId_LBAState[poseId] = ELocalBAState::ignored;
  }
  
  // -- Instrinsics
  for(const auto& itIntrinsic: sfm_data.GetIntrinsics())
  {
    const IndexT intrinsicId = itIntrinsic.first;
    _map_intrinsicId_LBAState[intrinsicId] = ELocalBAState::refined;
  }
  
  // -- Landmarks
  for(const auto& itLandmark: sfm_data.structure)
  {
    const IndexT landmarkId = itLandmark.first;
    const Observations & observations = itLandmark.second.observations;
    
    _map_landmarkId_LBAState[landmarkId] = ELocalBAState::ignored;
    
    for(const auto& observationIt: observations)
    {
      int dist = localBA_data->getViewDistance(observationIt.first);
      if(dist <= kDistanceLimit)
      {
        _map_landmarkId_LBAState[landmarkId] = ELocalBAState::refined;
        continue;
      }
    }
  }
}

void Local_Bundle_Adjustment_Ceres::computeStatesMaps_strategy3(
    const SfM_Data & sfm_data, 
    const std::shared_ptr<LocalBA_Data> localBA_data)
{
  // reset the maps
  _map_poseId_LBAState.clear();
  _map_intrinsicId_LBAState.clear();
  _map_landmarkId_LBAState.clear();
  
  
  // ----------------------------------------------------
  // -- Strategy 3 : (2017.08.05)
  //  D = distanceLimit
  //  N = sharingLimit        [*NEW*]
  //  - cameras:
  //    - dist <= D: refined
  //    - dist == D+1: fixed
  //    - else ignored
  //  - intrinsic:            
  //    - used by more than N already resected cameras: constant  [*NEW*]
  //    - else refined        [*NEW*]
  //  - landmarks:
  //    - connected to a refined camera: refined
  //    - else ignored
  // ----------------------------------------------------
  const std::size_t kDistanceLimit = 1;
  const std::size_t kIntrinsicSharingLimit = 30;
  
  // -- Poses
  for (Poses::const_iterator itPose = sfm_data.GetPoses().begin(); itPose != sfm_data.GetPoses().end(); ++itPose)
  {
    const IndexT poseId = itPose->first;
    int dist = localBA_data->getPoseDistance(poseId);
    if (dist >= 0 && dist <= kDistanceLimit) 
      _map_poseId_LBAState[poseId] = ELocalBAState::refined;
    else if (dist == kDistanceLimit + 1)
      _map_poseId_LBAState[poseId] = ELocalBAState::constant;
    else // dist < 0 (not connected to the node) or > D + 1
      _map_poseId_LBAState[poseId] = ELocalBAState::ignored;
  }
  
  // -- Instrinsics
  
  // transform the 'newReconstructedViewIds' set to 'newReconstructedPoseIds'
  std::set<IndexT> newPoseIds;
  for (auto& viewId : localBA_data->getNewViewsId())
  {
    const View * view = sfm_data.views.at(viewId).get();
    newPoseIds.insert(view->getPoseId());
  }
  
  // count the number of usage of each intrinsic among the aready resected poses
  std::map<IndexT, std::size_t> map_intrinsicId_usageNum;
  
  for (const auto& itView : sfm_data.views)
  {
    const View * view = itView.second.get();
    
    if (sfm_data.IsPoseAndIntrinsicDefined(view))
    {
      auto itPose = newPoseIds.find(view->getPoseId());
      if (itPose == newPoseIds.end()) // not a newly resected view/pose
      {
        auto itIntr = map_intrinsicId_usageNum.find(view->getIntrinsicId());
        if (itIntr == map_intrinsicId_usageNum.end())
          map_intrinsicId_usageNum[view->getIntrinsicId()] = 1;
        else
          map_intrinsicId_usageNum[view->getIntrinsicId()]++;
      }
    }
  }
  
  // 
  for(const auto& itIntrinsic: sfm_data.GetIntrinsics())
  {
    const IndexT intrinsicId = itIntrinsic.first;
    if (map_intrinsicId_usageNum[intrinsicId] >= kIntrinsicSharingLimit)
      _map_intrinsicId_LBAState[intrinsicId] = ELocalBAState::constant;
    else
      _map_intrinsicId_LBAState[intrinsicId] = ELocalBAState::refined;
  }
  
  // -- Landmarks
  for(const auto& itLandmark: sfm_data.structure)
  {
    const IndexT landmarkId = itLandmark.first;
    const Observations & observations = itLandmark.second.observations;
    
    _map_landmarkId_LBAState[landmarkId] = ELocalBAState::ignored;
    
    for(const auto& observationIt: observations)
    {
      int dist = localBA_data->getViewDistance(observationIt.first);
      if(dist <= kDistanceLimit)
      {
        _map_landmarkId_LBAState[landmarkId] = ELocalBAState::refined;
        continue;
      }
    }
  }
}

void Local_Bundle_Adjustment_Ceres::computeStatesMaps_strategy4(
    const SfM_Data & sfm_data, 
    std::shared_ptr<LocalBA_Data> localBA_data)
{
  // reset the maps
  _map_poseId_LBAState.clear();
  _map_intrinsicId_LBAState.clear();
  _map_landmarkId_LBAState.clear();
  
  // ----------------------------------------------------
  // -- Strategy 4 : (2017.09.25)
  //  D = distanceLimit
  //  L = percentageLimit
  //  W = windowSize
  //  - cameras:
  //    - dist <= D: refined
  //    - dist == D+1: constant
  //    - else ignored
  //  - intrinsic:
  //    All the parameters of each intrinic are saved.        [*NEW*]
  //    All the intrinsics are set to Refined by default.     [*NEW*]
  //    An intrinsic is set to contant when its focal lenght  [*NEW*]
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
    int dist = localBA_data->getPoseDistance(poseId);
    if (dist >= 0 && dist <= kDistanceLimit) 
      _map_poseId_LBAState[poseId] = ELocalBAState::refined;
    else if (dist == kDistanceLimit + 1)
      _map_poseId_LBAState[poseId] = ELocalBAState::constant;
    else // dist < 0 (not connected to the node) or > D + 1
      _map_poseId_LBAState[poseId] = ELocalBAState::ignored;
  }
  
  // -- Instrinsics
  localBA_data->checkParameterLimits(LocalBA_Data::EIntrinsicParameter::Focal, kWindowSize, kStdevPercentage); 
  
  for(const auto& itIntrinsic: sfm_data.GetIntrinsics())
  {
    if (localBA_data->isLimitReached(itIntrinsic.first, LocalBA_Data::EIntrinsicParameter::Focal))
      _map_intrinsicId_LBAState[itIntrinsic.first] = ELocalBAState::constant;
    else
      _map_intrinsicId_LBAState[itIntrinsic.first] = ELocalBAState::refined;
  }
  
  // -- Landmarks
  for(const auto& itLandmark: sfm_data.structure)
  {
    const IndexT landmarkId = itLandmark.first;
    const Observations & observations = itLandmark.second.observations;
    
    _map_landmarkId_LBAState[landmarkId] = ELocalBAState::ignored;
    
    for(const auto& observationIt: observations)
    {
      int dist = localBA_data->getViewDistance(observationIt.first);
      if(dist <= kDistanceLimit)
      {
        _map_landmarkId_LBAState[landmarkId] = ELocalBAState::refined;
        continue;
      }
    }
  }
}

std::size_t Local_Bundle_Adjustment_Ceres::getNumberOfCamerasInTheSolver()
{
  std::size_t num = 0;
  for (auto it : _map_poseId_LBAState)
  {
    if (it.second == ELocalBAState::refined || it.second == ELocalBAState::constant)
      num++;
  }
  return num;
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
      if(intrinsicsUsage.find(view->getIntrinsicId()) == intrinsicsUsage.end())
        intrinsicsUsage[view->getIntrinsicId()] = 1;
      else
        ++intrinsicsUsage[view->getIntrinsicId()];
    }
    else
    {
      if(intrinsicsUsage.find(view->getIntrinsicId()) == intrinsicsUsage.end())
        intrinsicsUsage[view->getIntrinsicId()] = 0;
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
    header.push_back("dAR=-1");
    header.push_back("dAR=0"); header.push_back("dAR=1"); header.push_back("dAR=2");
    header.push_back("dAR=3"); header.push_back("dAR=4"); header.push_back("dAR=5");   
    header.push_back("dAR=6"); header.push_back("dAR=7"); header.push_back("dAR=8"); 
    header.push_back("dAR=9"); header.push_back("dAR=10+");
    header.
        push_back("New Views");
    
    for (std::string & head : header)
      os << head << "\t";
    os << "\n"; 
  }
  
  // Add the '_LBA_statistics' contents:
  // Compute the number of poses with a distanceToRecenteCameras > 10
  // remind: distances range is {-1; 10+} so 11th element is the dist. 10
  std::size_t posesWthDistUpperThanTen = 0;
  
  for (const auto& it : _LBA_statistics.map_distance_numCameras)
  {
    if (it.first >= 10)
      posesWthDistUpperThanTen += it.second;
  }
  //  if (_LBA_statistics.map_distance_numCameras.size() > 11) 
  //  {
  //    for (int i=11; i<_LBA_statistics.map_distance_numCameras.size(); ++i)
  //    {
  //      std::cout << "dist " << i << " - ";
  //      std::cout << _LBA_statistics.map_distance_numCameras.at(i) << std::endl;
  //      posesWthDistUpperThanTen += _LBA_statistics.map_distance_numCameras.at(i);
  //    }
  //  }
  
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
        
     << _LBA_statistics.map_distance_numCameras[-1] << "\t"
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

