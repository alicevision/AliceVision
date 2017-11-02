// Copyright (c) 2015 Pierre Moulon.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "openMVG/sfm/sfm_data_localBA_ceres.hpp"
#include "openMVG/sfm/pipelines/sequential/sequential_SfM.hpp"
#include <openMVG/config.hpp>
#include <openMVG/openmvg_omp.hpp>

#include "openMVG/stl/stlMap.hpp"
#include "ceres/rotation.h"
#include "lemon/bfs.h"
#include <fstream>

namespace openMVG {
namespace sfm {

using namespace openMVG::cameras;
using namespace openMVG::geometry;

Local_Bundle_Adjustment_Ceres::Local_Bundle_Adjustment_Ceres(
  const Local_Bundle_Adjustment_Ceres::LocalBA_options& options, 
  const LocalBA_Data& localBA_data,
  const std::set<IndexT>& newReconstructedViews)
  : 
    _LBAOptions(options),
    _LBAStatistics(newReconstructedViews, localBA_data.getDistancesHistogram())
{}

bool Local_Bundle_Adjustment_Ceres::Adjust(SfM_Data& sfm_data, const LocalBA_Data& localBA_data)
{
  //----------
  // Steps:
  // 1. Generate parameter block with all the poses & intrinsics
  // 2. Create residuals for each observation in the bundle adjustment problem seen by a view with a non-"ignored" pose and intrinsic.
  // 3. Solve the minimization.
  // 4. Store statisics
  // 5. Update the scene with the new poses, landmarks & intrinsics (set to Refine)  
  //----------
  
  ceres::Solver::Options solver_options;
  setSolverOptions(solver_options);
  if (_LBAOptions.isParameterOrderingEnabled()) 
    solver_options.linear_solver_ordering.reset(new ceres::ParameterBlockOrdering);
  
  ceres::Problem problem;
  
  // Data wrapper for refinement:
  Hash_Map<IndexT, std::vector<double> > map_posesBlocks;
  Hash_Map<IndexT, std::vector<double> > map_intrinsicsBlocks;
  
  // 1. Generate parameter block with all the poses & intrinsics
  // Add Poses data to the Ceres problem as Parameter Blocks (do not take care of Local BA strategy)
  map_posesBlocks = addPosesToCeresProblem(sfm_data.GetPoses(), problem);
  
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
        if (localBA_data.getPoseState(poseId) == LocalBA_Data::EState::ignored 
            || localBA_data.getIntrinsicState(intrinsicId) == LocalBA_Data::EState::ignored 
            || localBA_data.getLandmarkState(landmarkId) == LocalBA_Data::EState::ignored)
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
          if (localBA_data.getIntrinsicState(intrinsicId) == LocalBA_Data::EState::constant)  problem.SetParameterBlockConstant(intrinsicBlock);        
          if (localBA_data.getPoseState(poseId) == LocalBA_Data::EState::constant)            problem.SetParameterBlockConstant(poseBlock);
          if (localBA_data.getLandmarkState(landmarkId) == LocalBA_Data::EState::constant)    problem.SetParameterBlockConstant(landmarkBlock);
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
  OPENMVG_LOG_DEBUG("Solving ceres problem...");
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
  
  if (_LBAOptions._bVerbose)
  {
    // Display statistics about the minimization
    OPENMVG_LOG_DEBUG(
          "Bundle Adjustment statistics (approximated RMSE):\n"
          " #views: " << sfm_data.views.size() << "\n"
          << " #poses: " << sfm_data.GetPoses().size() << "\n"
          << " #intrinsics: " << sfm_data.intrinsics.size() << "\n"
                                                               " #tracks: " << sfm_data.structure.size() << "\n"
          << " #residuals: " << summary.num_residuals << "\n"
          << " Initial RMSE: " << std::sqrt( summary.initial_cost / summary.num_residuals) << "\n"
          << " Final RMSE: " << std::sqrt( summary.final_cost / summary.num_residuals) << "\n"
          << " Time (s): " << summary.total_time_in_seconds << "\n"
          );
    
    // Display statistics about the Local BA
    OPENMVG_LOG_DEBUG(
          "Local BA statistics:\n"
          << " #poses: " << _LBAStatistics._numRefinedPoses << " refined, " 
          << _LBAStatistics._numConstantPoses << " constant, "
          << _LBAStatistics._numIgnoredPoses << " ignored.\n"
          << " #intrinsics: " << _LBAStatistics._numRefinedIntrinsics << " refined, " 
          << _LBAStatistics._numConstantIntrinsics << " constant, "
          << _LBAStatistics._numIgnoredIntrinsics<< " ignored.\n"   
          << " #landmarks: " << _LBAStatistics._numRefinedLandmarks << " refined, " 
          << _LBAStatistics._numConstantLandmarks << " constant, "
          << _LBAStatistics._numIgnoredLandmarks << " ignored.\n"
          );
    
    if (_LBAOptions.isParameterOrderingEnabled())
    {
      // Display statistics about "parameter ordering"
      OPENMVG_LOG_DEBUG(
            "Parameter ordering statistics:"
            << "\n (group 0 (landmarks)): " << solver_options.linear_solver_ordering->GroupSize(0) 
            << "\n (group 1 (intrinsics)): " << solver_options.linear_solver_ordering->GroupSize(1) 
            << "\n (group 2 (poses)): " << solver_options.linear_solver_ordering->GroupSize(2) << "\n"
            );
    }
  }
  
  // 5. Update the scene with the new poses, landmarks & intrinsics (set to Refine)  
  // Update camera poses with refined data
  updateCameraPoses(map_posesBlocks, localBA_data, sfm_data.GetPoses());

  // Update camera intrinsics with refined data
  updateCameraIntrinsics(map_intrinsicsBlocks, localBA_data, sfm_data.intrinsics);
  
  return true;
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
  solver_options.preconditioner_type = _LBAOptions._preconditioner_type;
  solver_options.linear_solver_type = _LBAOptions._linear_solver_type;
  solver_options.sparse_linear_algebra_library_type = _LBAOptions._sparse_linear_algebra_library_type;
  solver_options.minimizer_progress_to_stdout = _LBAOptions._bVerbose;
  solver_options.logging_type = ceres::SILENT;
  solver_options.num_threads = _LBAOptions._nbThreads;
  solver_options.num_linear_solver_threads = _LBAOptions._nbThreads;
}

bool Local_Bundle_Adjustment_Ceres::solveBA(
    ceres::Problem& problem, 
    ceres::Solver::Options& options, 
    ceres::Solver::Summary& summary)
{
  // Configure a BA engine and run it
  // Solve BA
  
  ceres::Solve(options, &problem, &summary);
  if (_LBAOptions._bCeres_Summary)
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
    const Hash_Map<IndexT, std::vector<double>> & map_poseblocks,
    const LocalBA_Data& localBA_data,
    Poses & poses)
{
  for (Poses::iterator itPose = poses.begin();
       itPose != poses.end(); ++itPose)
  {
    const IndexT poseId = itPose->first;
    
    // Do not update a camera pose set as Ignored or Constant in the Local BA strategy
    if (_LBAOptions.isLocalBAEnabled() )
    {
      if (localBA_data.getPoseState(poseId) == LocalBA_Data::EState::ignored) 
        continue;
      if (localBA_data.getPoseState(poseId) == LocalBA_Data::EState::constant) 
        continue;
    }
    
    Mat3 R_refined;
    ceres::AngleAxisToRotationMatrix(&map_poseblocks.at(poseId)[0], R_refined.data());
    Vec3 t_refined(map_poseblocks.at(poseId)[3], map_poseblocks.at(poseId)[4], map_poseblocks.at(poseId)[5]);
    // Update the pose
    Pose3 & pose = itPose->second;
    pose = Pose3(R_refined, -R_refined.transpose() * t_refined);
  }
}

void Local_Bundle_Adjustment_Ceres::updateCameraIntrinsics(
    const Hash_Map<IndexT, std::vector<double>> & map_intrinsicblocks,
    const LocalBA_Data& localBA_data, 
    Intrinsics & intrinsics)
{
  for (const auto& intrinsicsV: map_intrinsicblocks)
  {
    const IndexT intrinsicId = intrinsicsV.first;
    
    // Do not update an camera intrinsic set as Ignored or Constant in the Local BA strategy
    if (_LBAOptions.isLocalBAEnabled() )
    {
      if (localBA_data.getIntrinsicState(intrinsicId) == LocalBA_Data::EState::ignored) 
        continue;
      if (localBA_data.getIntrinsicState(intrinsicId) == LocalBA_Data::EState::constant) 
        continue;
    }
    
    intrinsics[intrinsicId]->updateFromParams(intrinsicsV.second);
  }
}

bool Local_Bundle_Adjustment_Ceres::exportStatistics(
  const std::string& path, 
  const std::set<IndexT>& newReconstructedViews,
  const std::size_t& kMinNbOfMatches, 
  const std::size_t kLimitDistance)
{
  showStatistics();
  
  std::string filename = stlplus::folder_append_separator(path)+"BaStats_M" + std::to_string(kMinNbOfMatches) + "_D" + std::to_string(kLimitDistance) + ".txt";
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
  
  for (const auto& it : _LBAStatistics._numCamerasPerDistance)
  {
    if (it.first >= 10)
      posesWthDistUpperThanTen += it.second;
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
     << posesWthDistUpperThanTen << "\t";
  
  for (const IndexT id : newReconstructedViews)
  {
    os << id << "\t";
  }
  os << "\n";
  os.close();
  
  return true;
}

void Local_Bundle_Adjustment_Ceres::showStatistics()
{
  std::cout << "\n----- Local BA Ceres statistics ------" << std::endl;
  std::cout << "|- adjutment duration: " << _LBAStatistics._numRefinedPoses << " ms" << std::endl;
  std::cout << "|- poses: " 
            << _LBAStatistics._numRefinedPoses << " refined, "
            << _LBAStatistics._numConstantPoses << " constant, "
            << _LBAStatistics._numIgnoredPoses << " ignored" << std::endl;  
  std::cout << "|- landmarks: " 
            << _LBAStatistics._numRefinedLandmarks << " refined, "
            << _LBAStatistics._numConstantLandmarks << " constant, "
            << _LBAStatistics._numIgnoredLandmarks<< " ignored" << std::endl;
  std::cout << "|- intrinsics: " 
            << _LBAStatistics._numRefinedIntrinsics << " refined, "
            << _LBAStatistics._numConstantIntrinsics << " constant, "
            << _LBAStatistics._numIgnoredIntrinsics << " ignored" << std::endl;
  std::cout << "|- #residual blocks = " << _LBAStatistics._numResidualBlocks << std::endl;
  std::cout << "|- #successful iterations = " << _LBAStatistics._numSuccessfullIterations << std::endl;
  std::cout << "|- #unsuccessful iterations = " << _LBAStatistics._numUnsuccessfullIterations << std::endl;
  std::cout << "|- initial RMSE = " << _LBAStatistics._RMSEinitial << std::endl;
  std::cout << "|- final RMSE = " << _LBAStatistics._RMSEfinal<< std::endl;
  std::cout << "|- graph-distances: " << std::endl;
  for (int i = -1; i < 10; i++)
  {
    std::cout << "D(" << i << ") : " << _LBAStatistics._numCamerasPerDistance[i] << std::endl;
  }
  std::cout << "---------------------------------------\n" << std::endl;
}



} // namespace sfm
} // namespace openMVG

