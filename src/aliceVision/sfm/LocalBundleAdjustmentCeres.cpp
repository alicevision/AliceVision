// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "LocalBundleAdjustmentCeres.hpp"

#include <boost/filesystem.hpp>

#include <fstream>

namespace fs = boost::filesystem;

namespace aliceVision {
namespace sfm {

using namespace aliceVision::camera;
using namespace aliceVision::geometry;

void LocalBundleAdjustmentCeres::LocalBA_statistics::show()
{
  int numCamDistEqOne = 0;
  if (_numCamerasPerDistance.find(1) != _numCamerasPerDistance.end())
    numCamDistEqOne = _numCamerasPerDistance.at(1);
  
  int numCamDistUpperOne = 0;
  for (const auto & camdistIt : _numCamerasPerDistance)
  {
    if (camdistIt.first > 1)
      numCamDistUpperOne += camdistIt.second;
  }
  
  ALICEVISION_LOG_DEBUG("\n----- Local BA Ceres statistics ------\n" 
                        << "|- adjustment duration: " << _time << " s \n"
                        << "|- graph-distances distribution: \n"
                        << "  * not connected: " << _numCamerasPerDistance[-1] << " cameras \n"
                        << "  * D = 0: " << _numCamerasPerDistance[0] << " cameras \n"
                        << "  * D = 1: " << numCamDistEqOne << " cameras \n"
                        << "  * D > 1: " << numCamDistUpperOne << " cameras \n"
                        << "|- poses: \t" 
                        << _numRefinedPoses << " refined, \t"
                        << _numConstantPoses << " constant, \t"
                        << _numIgnoredPoses << " ignored \n" 
                        << "|- landmarks: \t" 
                        << _numRefinedLandmarks << " refined, \t"
                        << _numConstantLandmarks << " constant, \t"
                        << _numIgnoredLandmarks<< " ignored \n"
                        << "|- intrinsics: \t" 
                        << _numRefinedIntrinsics << " refined, \t"
                        << _numConstantIntrinsics << " constant, \t"
                        << _numIgnoredIntrinsics << " ignored \n"
                        << "|- #residual blocks = " << _numResidualBlocks << "\n"
                        << "|- #successful iterations = " << _numSuccessfullIterations<< "\n"
                        << "|- #unsuccessful iterations = " << _numUnsuccessfullIterations<< "\n"
                        << "|- initial RMSE = " << _RMSEinitial << "\n"
                        << "|- final RMSE = " << _RMSEfinal << "\n"
                        << "---------------------------------------");
}

LocalBundleAdjustmentCeres::LocalBundleAdjustmentCeres(const LocalBundleAdjustmentData& localBA_data,
                                                       const LocalBundleAdjustmentCeres::LocalBA_options& options,
                                                       const std::set<IndexT>& newReconstructedViews)
  : 
    _LBAOptions(options),
    _LBAStatistics(newReconstructedViews, localBA_data.getDistancesHistogram())
{}

bool LocalBundleAdjustmentCeres::Adjust(SfMData& sfm_data, const LocalBundleAdjustmentData& localBA_data)
{
  //----------
  // Steps:
  // 1. Generate parameter blocks (using poses & intrinsics)
  // 2. Create residuals for each observation seen by a view with a non-"ignored" pose and intrinsic.
  // 3. Solve the minimization.
  // 4. Store statisics
  // 5. Update the scene with the new poses & intrinsics (set to Refine)  
  //----------
  
  ceres::Solver::Options solver_options;
  setSolverOptions(solver_options);
  
  if (_LBAOptions.isParameterOrderingEnabled()) 
    solver_options.linear_solver_ordering.reset(new ceres::ParameterBlockOrdering);
  
  ceres::Problem problem;
  
  // Data wrapper for refinement:
  std::map<IndexT, std::vector<double> > map_posesBlocks;
  std::map<IndexT, std::vector<double> > map_intrinsicsBlocks;
  
  // 1. Generate parameter block with all the poses & intrinsics
  // Add Poses data to the Ceres problem as Parameter Blocks (do not take care of Local BA strategy)
  map_posesBlocks = addPosesToCeresProblem(sfm_data.getPoses(), problem);
  
  // Add Intrinsics data to the Ceres problem as Parameter Blocks (do not take care of Local BA strategy)
  map_intrinsicsBlocks = addIntrinsicsToCeresProblem(sfm_data, problem);
  
  // 2. Create residuals for each observation in the bundle adjustment problem seen by a view with a non-"ignored" pose and intrinsic.
  // Set a LossFunction to be less penalized by false measurements
  //  - set it to NULL if you don't want use a lossFunction.
  ceres::LossFunction * p_LossFunction = new ceres::HuberLoss(Square(4.0));
  // TODO: make the LOSS function and the parameter an option
  
  // For all visibility add reprojections errors:
  for(auto& landmarkIt: sfm_data.structure)
  {             
    IndexT landmarkId = landmarkIt.first;
    
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
      if (_LBAOptions.isLocalBAEnabled())
      {
        if (localBA_data.getPosestate(poseId) == LocalBundleAdjustmentData::EState::ignored 
            || localBA_data.getIntrinsicstate(intrinsicId) == LocalBundleAdjustmentData::EState::ignored 
            || localBA_data.getLandmarkState(landmarkId) == LocalBundleAdjustmentData::EState::ignored)
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
        if (_LBAOptions.isParameterOrderingEnabled()) 
        {
          solver_options.linear_solver_ordering->AddElementToGroup(landmarkBlock, 0);
          solver_options.linear_solver_ordering->AddElementToGroup(poseBlock, 1);
          solver_options.linear_solver_ordering->AddElementToGroup(intrinsicBlock, 2);
        }
        // Set to constant parameters previoously set as Constant by the Local BA strategy
        if (_LBAOptions.isLocalBAEnabled())
        {
          if (localBA_data.getIntrinsicstate(intrinsicId) == LocalBundleAdjustmentData::EState::constant)  problem.SetParameterBlockConstant(intrinsicBlock);        
          if (localBA_data.getPosestate(poseId) == LocalBundleAdjustmentData::EState::constant)            problem.SetParameterBlockConstant(poseBlock);
          if (localBA_data.getLandmarkState(landmarkId) == LocalBundleAdjustmentData::EState::constant)    problem.SetParameterBlockConstant(landmarkBlock);
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
  
  // 3. Solve the minimization.
  ceres::Solver::Summary summary;
  if (!solveBA(problem, solver_options, summary))
    return false;
  
  // 4. Store statisics
  // Solution is usable
  if (_LBAOptions.isLocalBAEnabled())
  {
    _LBAStatistics._numRefinedPoses       = localBA_data.getNumOfRefinedPoses();
    _LBAStatistics._numConstantPoses      = localBA_data.getNumOfConstantPoses();
    _LBAStatistics._numIgnoredPoses       = localBA_data.getNumOfIgnoredPoses();
    _LBAStatistics._numRefinedIntrinsics  = localBA_data.getNumOfRefinedIntrinsics();
    _LBAStatistics._numConstantIntrinsics = localBA_data.getNumOfConstantIntrinsics();
    _LBAStatistics._numIgnoredIntrinsics  = localBA_data.getNumOfIgnoredIntrinsics();
    _LBAStatistics._numRefinedLandmarks   = localBA_data.getNumOfRefinedLandmarks();
    _LBAStatistics._numConstantLandmarks  = localBA_data.getNumOfConstantLandmarks();
    _LBAStatistics._numIgnoredLandmarks   = localBA_data.getNumOfIgnoredLandmarks();
  }   
  else
  {
    // All the parameters are considered as Refined in a classic BA
    _LBAStatistics._numRefinedPoses = map_posesBlocks.size();
    _LBAStatistics._numRefinedIntrinsics = map_intrinsicsBlocks.size();
    _LBAStatistics._numRefinedLandmarks = sfm_data.structure.size();
  }
  
  // Add statitics about the BA loop:
  _LBAStatistics._time = summary.total_time_in_seconds;
  _LBAStatistics._numSuccessfullIterations = summary.num_successful_steps;
  _LBAStatistics._numUnsuccessfullIterations = summary.num_unsuccessful_steps;
  _LBAStatistics._numResidualBlocks = summary.num_residuals;
  _LBAStatistics._RMSEinitial = std::sqrt( summary.initial_cost / summary.num_residuals);
  _LBAStatistics._RMSEfinal = std::sqrt( summary.final_cost / summary.num_residuals);
  
  _LBAStatistics.show();
  
  // 5. Update the scene with the new poses & intrinsics (set to Refine)  
  // Update camera poses with refined data
  updateCameraPoses(map_posesBlocks, localBA_data, sfm_data.getPoses());
  
  // Update camera intrinsics with refined data
  updateCameraIntrinsics(map_intrinsicsBlocks, localBA_data, sfm_data.intrinsics);
  
  return true;
}

bool LocalBundleAdjustmentCeres::exportStatistics(const std::string& dir, const std::string& filename)
{
  std::ofstream os;
  os.open((fs::path(dir) / filename).string(), std::ios::app);
  if (!os.is_open())
  {
    ALICEVISION_LOG_DEBUG("Unable to open the Bundle adjustment stat file '" << filename << "'.");
    return false;
  }
  os.seekp(0, std::ios::end); //put the cursor at the end
  
  if (os.tellp() == std::streampos(0)) // 'tellp' return the cursor's position
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
    header.push_back("New Views");
    
    for (std::string & head : header)
      os << head << "\t";
    os << "\n"; 
  }
  
  // Add the '_LBA_statistics' contents:
  // Compute the number of poses with a distanceToRecenteCameras > 10
  // remind: distances range is {-1; 10+} so 11th element is the dist. 10
  std::size_t posesWithDistUpperThanTen = 0;
  
  for (const auto& it : _LBAStatistics._numCamerasPerDistance)
  {
    if (it.first >= 10)
      posesWithDistUpperThanTen += it.second;
  }
  
  os << _LBAStatistics._time << "\t"
        
     << _LBAStatistics._numRefinedPoses << "\t"
     << _LBAStatistics._numConstantPoses << "\t"
     << _LBAStatistics._numIgnoredPoses << "\t"
     << _LBAStatistics._numRefinedLandmarks << "\t"
     << _LBAStatistics._numConstantLandmarks << "\t"
     << _LBAStatistics._numIgnoredLandmarks << "\t"
     << _LBAStatistics._numRefinedIntrinsics << "\t"
     << _LBAStatistics._numConstantIntrinsics << "\t"
     << _LBAStatistics._numIgnoredIntrinsics << "\t"
        
     << _LBAStatistics._numResidualBlocks << "\t"
     << _LBAStatistics._numSuccessfullIterations << "\t"
     << _LBAStatistics._numUnsuccessfullIterations << "\t"
        
     << _LBAStatistics._RMSEinitial << "\t"
     << _LBAStatistics._RMSEfinal << "\t"
        
     << _LBAStatistics._numCamerasPerDistance[-1] << "\t"
     << _LBAStatistics._numCamerasPerDistance[0] << "\t"
     << _LBAStatistics._numCamerasPerDistance[1] << "\t"
     << _LBAStatistics._numCamerasPerDistance[2] << "\t"
     << _LBAStatistics._numCamerasPerDistance[3] << "\t"
     << _LBAStatistics._numCamerasPerDistance[4] << "\t"
     << _LBAStatistics._numCamerasPerDistance[5] << "\t"
     << _LBAStatistics._numCamerasPerDistance[6] << "\t"
     << _LBAStatistics._numCamerasPerDistance[7] << "\t"
     << _LBAStatistics._numCamerasPerDistance[8] << "\t"
     << _LBAStatistics._numCamerasPerDistance[9] << "\t"
     << posesWithDistUpperThanTen << "\t";
  
  for (const IndexT id : _LBAStatistics._newViewsId)
  {
    os << id << "\t";
  }
  os << "\n";
  os.close();
  
  return true;
}

std::map<IndexT, std::vector<double> > LocalBundleAdjustmentCeres::addPosesToCeresProblem(
    const Poses & poses,
    ceres::Problem & problem)
{
  // Data wrapper for refinement:
  std::map<IndexT, std::vector<double> > map_poses;
  
  // Setup Poses data 
  for (Poses::const_iterator itPose = poses.begin(); itPose != poses.end(); ++itPose)
  {
    const IndexT poseId = itPose->first;
    
    const CameraPose& cameraPose = itPose->second;
    const Mat3& R = cameraPose.getTransform().rotation();
    const Vec3& t = cameraPose.getTransform().translation();
    
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

    if(cameraPose.isLocked()) //set the whole parameter block as constant.
      problem.SetParameterBlockConstant(parameter_block);
  }
  return map_poses;
}

std::map<IndexT, std::vector<double>> LocalBundleAdjustmentCeres::addIntrinsicsToCeresProblem(
    const SfMData & sfm_data,
    ceres::Problem & problem)
{
  std::map<IndexT, std::size_t> intrinsicsUsage;
  
  // Setup Intrinsics data 
  // Count the number of reconstructed views per intrinsic
  for(const auto& itView: sfm_data.getViews())
  {
    const View* view = itView.second.get();
    if (sfm_data.isPoseAndIntrinsicDefined(view))
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
  
  std::map<IndexT, std::vector<double>> map_intrinsics;
  for(const auto& itIntrinsic: sfm_data.getIntrinsics())
  {
    const IndexT intrinsicIds = itIntrinsic.first;
    
    // Do not refine an intrinsic does not used by any reconstructed view
    if(intrinsicsUsage[intrinsicIds] == 0)
      continue;
    
    assert(isValid(itIntrinsic.second->getType()));
    map_intrinsics[intrinsicIds] = itIntrinsic.second->getParams();
    
    double * parameter_block = &map_intrinsics[intrinsicIds][0];
    problem.AddParameterBlock(parameter_block, map_intrinsics[intrinsicIds].size());

    if(itIntrinsic.second->isLocked())
    {
      //set the whole parameter block as constant.
      problem.SetParameterBlockConstant(parameter_block);
      continue;
    }

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

/// Set BA options to Ceres
void LocalBundleAdjustmentCeres::setSolverOptions(ceres::Solver::Options& solver_options)
{
  solver_options.preconditioner_type = _LBAOptions._preconditioner_type;
  solver_options.linear_solver_type = _LBAOptions._linear_solver_type;
  solver_options.sparse_linear_algebra_library_type = _LBAOptions._sparse_linear_algebra_library_type;
  solver_options.minimizer_progress_to_stdout = _LBAOptions._bVerbose;
  solver_options.logging_type = ceres::SILENT;
  solver_options.num_threads = _LBAOptions._nbThreads;
  solver_options.num_linear_solver_threads = _LBAOptions._nbThreads;
}

bool LocalBundleAdjustmentCeres::solveBA(
    ceres::Problem& problem, 
    ceres::Solver::Options& options, 
    ceres::Solver::Summary& summary)
{
  if (_LBAOptions._linear_solver_type == ceres::LinearSolverType::DENSE_SCHUR)
    ALICEVISION_LOG_DEBUG("Solving ceres problem (linear solver type: DENSE_SCHUR)...");
  else if (_LBAOptions._linear_solver_type == ceres::LinearSolverType::SPARSE_SCHUR)
    ALICEVISION_LOG_DEBUG("Solving ceres problem (linear solver type: SPARSE_SCHUR)...");
  else if (_LBAOptions._linear_solver_type == ceres::LinearSolverType::ITERATIVE_SCHUR)
    ALICEVISION_LOG_DEBUG("Solving ceres problem (linear solver type: ITERATIVE_SCHUR)...");
  else
    ALICEVISION_LOG_DEBUG("Solving ceres problem...");
  
  // Configure a BA engine and run it
  // Solve BA
  
  ceres::Solve(options, &problem, &summary);
  if (_LBAOptions._bCeres_Summary)
    ALICEVISION_LOG_DEBUG(summary.FullReport());
  
  // If no error, get back refined parameters
  if (!summary.IsSolutionUsable())
  {
    ALICEVISION_LOG_WARNING("Bundle Adjustment failed.");
    return false;
  }
  return true;
}

void LocalBundleAdjustmentCeres::updateCameraPoses(
    const std::map<IndexT, std::vector<double>> & map_poseblocks,
    const LocalBundleAdjustmentData& localBA_data,
    Poses & poses)
{
  for (Poses::iterator itPose = poses.begin();
       itPose != poses.end(); ++itPose)
  {
    const IndexT poseId = itPose->first;
    
    // Do not update a camera pose set as Ignored or Constant in the Local BA strategy
    if (_LBAOptions.isLocalBAEnabled() )
    {
      if (localBA_data.getPosestate(poseId) == LocalBundleAdjustmentData::EState::ignored) 
        continue;
      if (localBA_data.getPosestate(poseId) == LocalBundleAdjustmentData::EState::constant) 
        continue;
    }
    
    Mat3 R_refined;
    ceres::AngleAxisToRotationMatrix(&map_poseblocks.at(poseId)[0], R_refined.data());
    Vec3 t_refined(map_poseblocks.at(poseId)[3], map_poseblocks.at(poseId)[4], map_poseblocks.at(poseId)[5]);

    // Update the pose
    itPose->second.setTransform(Pose3(R_refined, -R_refined.transpose() * t_refined));
  }
}

void LocalBundleAdjustmentCeres::updateCameraIntrinsics(
    const std::map<IndexT, std::vector<double>> & map_intrinsicblocks,
    const LocalBundleAdjustmentData& localBA_data, 
    Intrinsics & intrinsics)
{
  for (const auto& intrinsicsV: map_intrinsicblocks)
  {
    const IndexT intrinsicId = intrinsicsV.first;
    
    // Do not update an camera intrinsic set as Ignored or Constant in the Local BA strategy
    if (_LBAOptions.isLocalBAEnabled() )
    {
      if (localBA_data.getIntrinsicstate(intrinsicId) == LocalBundleAdjustmentData::EState::ignored) 
        continue;
      if (localBA_data.getIntrinsicstate(intrinsicId) == LocalBundleAdjustmentData::EState::constant) 
        continue;
    }
    
    intrinsics[intrinsicId]->updateFromParams(intrinsicsV.second);
  }
}

} // namespace sfm
} // namespace aliceVision

