// Copyright (c) 2015 Pierre Moulon.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "openMVG/sfm/sfm_data_BA_ceres.hpp"
#include <openMVG/config.hpp>
#include <openMVG/openmvg_omp.hpp>

#include "ceres/rotation.h"

namespace openMVG {
namespace sfm {

using namespace openMVG::cameras;
using namespace openMVG::geometry;

/// Create the appropriate cost functor according the provided input camera intrinsic model
ceres::CostFunction * IntrinsicsToCostFunction(IntrinsicBase * intrinsic, const Vec2 & observation)
{
  switch(intrinsic->getType())
  {
    case PINHOLE_CAMERA:
      return new ceres::AutoDiffCostFunction<ResidualErrorFunctor_Pinhole_Intrinsic, 2, 3, 6, 3>(
        new ResidualErrorFunctor_Pinhole_Intrinsic(observation.data()));
    break;
    case PINHOLE_CAMERA_RADIAL1:
      return new ceres::AutoDiffCostFunction<ResidualErrorFunctor_Pinhole_Intrinsic_Radial_K1, 2, 4, 6, 3>(
        new ResidualErrorFunctor_Pinhole_Intrinsic_Radial_K1(observation.data()));
    break;
    case PINHOLE_CAMERA_RADIAL3:
      return new ceres::AutoDiffCostFunction<ResidualErrorFunctor_Pinhole_Intrinsic_Radial_K3, 2, 6, 6, 3>(
        new ResidualErrorFunctor_Pinhole_Intrinsic_Radial_K3(observation.data()));
    break;
    case PINHOLE_CAMERA_BROWN:
      return new ceres::AutoDiffCostFunction<ResidualErrorFunctor_Pinhole_Intrinsic_Brown_T2, 2, 8, 6, 3>(
        new ResidualErrorFunctor_Pinhole_Intrinsic_Brown_T2(observation.data()));
    break;
    case PINHOLE_CAMERA_FISHEYE:
      return new ceres::AutoDiffCostFunction<ResidualErrorFunctor_Pinhole_Intrinsic_Fisheye, 2, 7, 6, 3>(
              new ResidualErrorFunctor_Pinhole_Intrinsic_Fisheye(observation.data()));
    case PINHOLE_CAMERA_FISHEYE1:
      return new ceres::AutoDiffCostFunction<ResidualErrorFunctor_Pinhole_Intrinsic_Fisheye1, 2, 4, 6, 3>(
              new ResidualErrorFunctor_Pinhole_Intrinsic_Fisheye1(observation.data()));
    default:
      return nullptr;
  }
}


Bundle_Adjustment_Ceres::BA_options::BA_options(const bool bVerbose, bool bmultithreaded)
  :_bVerbose(bVerbose),
   _nbThreads(1)
{
  // set number of threads, 1 if openMP is not enabled
  _nbThreads = omp_get_max_threads();

  if (!bmultithreaded)
    _nbThreads = 1;

  _bCeres_Summary = false;
  useParametersOrdering = false;
  useLocalBA = false;
  
  // Use dense BA by default
  setDenseBA();
}

void Bundle_Adjustment_Ceres::BA_options::setDenseBA()
{
  // Default configuration use a DENSE representation
  _preconditioner_type = ceres::JACOBI;
  _linear_solver_type = ceres::DENSE_SCHUR;
    OPENMVG_LOG_DEBUG("Bundle_Adjustment_Ceres: DENSE_SCHUR");
}

void Bundle_Adjustment_Ceres::BA_options::setSparseBA()
{
  _preconditioner_type = ceres::JACOBI;
  // If Sparse linear solver are available
  // Descending priority order by efficiency (SUITE_SPARSE > CX_SPARSE > EIGEN_SPARSE)
  if (ceres::IsSparseLinearAlgebraLibraryTypeAvailable(ceres::SUITE_SPARSE))
  {
    _sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
    _linear_solver_type = ceres::SPARSE_SCHUR;
    OPENMVG_LOG_DEBUG("Bundle_Adjustment_Ceres: SPARSE_SCHUR, SUITE_SPARSE");
  }
  else if (ceres::IsSparseLinearAlgebraLibraryTypeAvailable(ceres::CX_SPARSE))
  {
    _sparse_linear_algebra_library_type = ceres::CX_SPARSE;
    _linear_solver_type = ceres::SPARSE_SCHUR;
    OPENMVG_LOG_DEBUG("Bundle_Adjustment_Ceres: SPARSE_SCHUR, CX_SPARSE");
  }
  else if (ceres::IsSparseLinearAlgebraLibraryTypeAvailable(ceres::EIGEN_SPARSE))
  {
    _sparse_linear_algebra_library_type = ceres::EIGEN_SPARSE;
    _linear_solver_type = ceres::SPARSE_SCHUR;
    OPENMVG_LOG_DEBUG("Bundle_Adjustment_Ceres: SPARSE_SCHUR, EIGEN_SPARSE");
  }
  else
  {
    _linear_solver_type = ceres::DENSE_SCHUR;
    OPENMVG_LOG_WARNING("Bundle_Adjustment_Ceres: no sparse BA available, fallback to dense BA.");
  }
}

Bundle_Adjustment_Ceres::Bundle_Adjustment_Ceres(
  Bundle_Adjustment_Ceres::BA_options options)
  : _openMVG_options(options)
{}

bool Bundle_Adjustment_Ceres::Adjust(
  SfM_Data & sfm_data,     // the SfM scene to refine
  BA_Refine refineOptions)
{
  // Ensure we are not using incompatible options:
  //  - BA_REFINE_INTRINSICS_OPTICALCENTER_ALWAYS and BA_REFINE_INTRINSICS_OPTICALCENTER_IF_ENOUGH_DATA cannot be used at the same time
  assert(!((refineOptions & BA_REFINE_INTRINSICS_OPTICALCENTER_ALWAYS) && (refineOptions & BA_REFINE_INTRINSICS_OPTICALCENTER_IF_ENOUGH_DATA)));
  
  //----------
  // Add camera parameters
  // - intrinsics
  // - poses [R|t]

  // Create residuals for each observation in the bundle adjustment problem. The
  // parameters for cameras and points are added automatically.
  //----------

  ceres::Problem problem;

  // Data wrapper for refinement:
  Hash_Map<IndexT, std::vector<double> > map_poses;
  
  // Setup Poses data & subparametrization
  for (Poses::const_iterator itPose = sfm_data.poses.begin(); itPose != sfm_data.poses.end(); ++itPose)
  {
    const IndexT indexPose = itPose->first;

    const Pose3 & pose = itPose->second;
    const Mat3 R = pose.rotation();
    const Vec3 t = pose.translation();

    double angleAxis[3];
    ceres::RotationMatrixToAngleAxis((const double*)R.data(), angleAxis);
    map_poses[indexPose].reserve(6); //angleAxis + translation
    map_poses[indexPose].push_back(angleAxis[0]);
    map_poses[indexPose].push_back(angleAxis[1]);
    map_poses[indexPose].push_back(angleAxis[2]);
    map_poses[indexPose].push_back(t(0));
    map_poses[indexPose].push_back(t(1));
    map_poses[indexPose].push_back(t(2));

    double * parameter_block = &map_poses[indexPose][0];
    problem.AddParameterBlock(parameter_block, 6);
    // Keep the camera extrinsics constants
    if (!(refineOptions & BA_REFINE_TRANSLATION) && !(refineOptions & BA_REFINE_ROTATION))
    {
      //set the whole parameter block as constant for best performance.
      problem.SetParameterBlockConstant(parameter_block);
    }
    else
    {
      // Subset parametrization
      std::vector<int> vec_constant_extrinsic;
      // Don't refine rotations (if BA_REFINE_ROTATION is not specified)
      if(!(refineOptions & BA_REFINE_ROTATION))
      {
        vec_constant_extrinsic.push_back(0);
        vec_constant_extrinsic.push_back(1);
        vec_constant_extrinsic.push_back(2);
      }
      // Don't refine translations (if BA_REFINE_TRANSLATION is not specified)
      if(!(refineOptions & BA_REFINE_TRANSLATION))
      {
        vec_constant_extrinsic.push_back(3);
        vec_constant_extrinsic.push_back(4);
        vec_constant_extrinsic.push_back(5);
      }
      if (!vec_constant_extrinsic.empty())
      {
        ceres::SubsetParameterization *subset_parameterization =
          new ceres::SubsetParameterization(6, vec_constant_extrinsic);
        problem.SetParameterization(parameter_block, subset_parameterization);
      }
    }
  }

  Hash_Map<IndexT, std::size_t> intrinsicsUsage;

  // Setup Intrinsics data & subparametrization
  const bool refineIntrinsicsOpticalCenter = (refineOptions & BA_REFINE_INTRINSICS_OPTICALCENTER_ALWAYS) || (refineOptions & BA_REFINE_INTRINSICS_OPTICALCENTER_IF_ENOUGH_DATA);
  const bool refineIntrinsics = (refineOptions & BA_REFINE_INTRINSICS_FOCAL) ||
                                (refineOptions & BA_REFINE_INTRINSICS_DISTORTION) ||
                                refineIntrinsicsOpticalCenter;
  for(const auto& itView: sfm_data.GetViews())
  {
    const View* v = itView.second.get();
    if (sfm_data.IsPoseAndIntrinsicDefined(v))
    {
      if(intrinsicsUsage.find(v->id_intrinsic) == intrinsicsUsage.end())
        intrinsicsUsage[v->id_intrinsic] = 1;
      else
        ++intrinsicsUsage[v->id_intrinsic];
    }
    else
    {
      if(intrinsicsUsage.find(v->id_intrinsic) == intrinsicsUsage.end())
        intrinsicsUsage[v->id_intrinsic] = 0;
    }
  }

  Hash_Map<IndexT, std::vector<double> > map_intrinsics;
  // Setup Intrinsics data & subparametrization
  for(const auto& itIntrinsic: sfm_data.GetIntrinsics())
  {
    const IndexT idIntrinsics = itIntrinsic.first;
    if(intrinsicsUsage[idIntrinsics] == 0)
    {
      continue;
    }
    assert(isValid(itIntrinsic.second->getType()));
    map_intrinsics[idIntrinsics] = itIntrinsic.second->getParams();

    double * parameter_block = &map_intrinsics[idIntrinsics][0];
    problem.AddParameterBlock(parameter_block, map_intrinsics[idIntrinsics].size());
    if (!refineIntrinsics)
    {
      // Nothing to refine in the intrinsics,
      // so set the whole parameter block as constant with better performances.
      problem.SetParameterBlockConstant(parameter_block);
    }
    else
    {
      std::vector<int> vec_constant_params;
      // Focal length
      if(refineOptions & BA_REFINE_INTRINSICS_FOCAL)
      {
        // Refine the focal length
        if(itIntrinsic.second->initialFocalLengthPix() > 0)
        {
          // If we have an initial guess, we only authorize a margin around this value.
          assert(map_intrinsics[idIntrinsics].size() >= 1);
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
      }
      else
      {
        // Set focal length as constant
        vec_constant_params.push_back(0);
      }

      const std::size_t minImagesForOpticalCenter = 3;

      // Optical center
      if((refineOptions & BA_REFINE_INTRINSICS_OPTICALCENTER_ALWAYS) ||
         ((refineOptions & BA_REFINE_INTRINSICS_OPTICALCENTER_IF_ENOUGH_DATA) && intrinsicsUsage[idIntrinsics] > minImagesForOpticalCenter)
         )
      {
        // Refine optical center within 10% of the image size.
        assert(map_intrinsics[idIntrinsics].size() >= 3);
        
        const double opticalCenterMinPercent = 0.45;
        const double opticalCenterMaxPercent = 0.55;
        
        // Add bounds to the principal point
        problem.SetParameterLowerBound(parameter_block, 1, opticalCenterMinPercent * itIntrinsic.second->w());
        problem.SetParameterUpperBound(parameter_block, 1, opticalCenterMaxPercent * itIntrinsic.second->w());

        problem.SetParameterLowerBound(parameter_block, 2, opticalCenterMinPercent * itIntrinsic.second->h());
        problem.SetParameterUpperBound(parameter_block, 2, opticalCenterMaxPercent * itIntrinsic.second->h());
      }
      else
      {
        // Don't refine the optical center
        vec_constant_params.push_back(1);
        vec_constant_params.push_back(2);
      }

      // Lens distortion
      if(!(refineOptions & BA_REFINE_INTRINSICS_DISTORTION))
      {
        for(std::size_t i = 3; i < map_intrinsics[idIntrinsics].size(); ++i)
        {
          vec_constant_params.push_back(i);
        }
      }

      if(!vec_constant_params.empty())
      {
        ceres::SubsetParameterization *subset_parameterization =
          new ceres::SubsetParameterization(map_intrinsics[idIntrinsics].size(), vec_constant_params);
        problem.SetParameterization(parameter_block, subset_parameterization);
      }
    }
  }

  // Set a LossFunction to be less penalized by false measurements
  //  - set it to NULL if you don't want use a lossFunction.
  ceres::LossFunction * p_LossFunction = new ceres::HuberLoss(Square(4.0));
  // TODO: make the LOSS function and the parameter an option

  // For all visibility add reprojections errors:
  for(auto& landmarkIt: sfm_data.structure)
  {
    const Observations & observations = landmarkIt.second.observations;
    // Iterate over 2D observation associated to the 3D landmark
    for (const auto& observationIt: observations)
    {
      // Build the residual block corresponding to the track observation:
      const View * view = sfm_data.views.at(observationIt.first).get();

      // Each Residual block takes a point and a camera as input and outputs a 2
      // dimensional residual. Internally, the cost function stores the observed
      // image location and compares the reprojection against the observation.
      ceres::CostFunction* cost_function =
        IntrinsicsToCostFunction(sfm_data.intrinsics[view->id_intrinsic].get(), observationIt.second.x);

      if (cost_function)
        problem.AddResidualBlock(cost_function,
          p_LossFunction,
          &map_intrinsics[view->id_intrinsic][0],
          &map_poses[view->id_pose][0],
          landmarkIt.second.X.data()); //Do we need to copy 3D point to avoid false motion, if failure ?
    }
    if (!(refineOptions & BA_REFINE_STRUCTURE))
      problem.SetParameterBlockConstant( landmarkIt.second.X.data());
  }

  // Configure a BA engine and run it
  //  Make Ceres automatically detect the bundle structure.
  ceres::Solver::Options options;
  options.preconditioner_type = _openMVG_options._preconditioner_type;
  options.linear_solver_type = _openMVG_options._linear_solver_type;
  options.sparse_linear_algebra_library_type = _openMVG_options._sparse_linear_algebra_library_type;
  options.minimizer_progress_to_stdout = _openMVG_options._bVerbose;
  options.logging_type = ceres::SILENT;
  options.num_threads = _openMVG_options._nbThreads;
  options.num_linear_solver_threads = _openMVG_options._nbThreads;

  // Solve BA
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  if (_openMVG_options._bCeres_Summary)
    OPENMVG_LOG_DEBUG(summary.FullReport());

  // If no error, get back refined parameters
  if (!summary.IsSolutionUsable())
  {
    OPENMVG_LOG_WARNING("Bundle Adjustment failed.");
    return false;
  }

  // Solution is usable
  if (_openMVG_options._bVerbose)
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

  // Update camera poses with refined data
  if ((refineOptions & BA_REFINE_ROTATION) || (refineOptions & BA_REFINE_TRANSLATION))
  {
    for (Poses::iterator itPose = sfm_data.poses.begin();
      itPose != sfm_data.poses.end(); ++itPose)
    {
      const IndexT indexPose = itPose->first;

      Mat3 R_refined;
      ceres::AngleAxisToRotationMatrix(&map_poses[indexPose][0], R_refined.data());
      Vec3 t_refined(map_poses[indexPose][3], map_poses[indexPose][4], map_poses[indexPose][5]);
      // Update the pose
      Pose3 & pose = itPose->second;
      pose = Pose3(R_refined, -R_refined.transpose() * t_refined);
    }
  }

  // Update camera intrinsics with refined data
  if (refineIntrinsics)
  {
    for (const auto& intrinsicsV: map_intrinsics)
    {
      sfm_data.intrinsics[intrinsicsV.first]->updateFromParams(intrinsicsV.second);
    }
  }
  return true;
}

void Bundle_Adjustment_Ceres::applyRefinementRules(const SfM_Data & sfm_data, const IndexT strategyId, const std::size_t distanceLimit)
{
  // reset the maps
  map_poseId_BAState.clear();
  map_intrinsicId_BAState.clear();
  map_landmarkId_BAState.clear();

  switch(strategyId)
  {
    case 1:
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
        int dist = map_poseId_distanceToRecentCameras.at(poseId);
        if (dist <= distanceLimit) // 0 or 1
          map_poseId_BAState[poseId] = LocalBAState::refined;
        else
          map_poseId_BAState[poseId] = LocalBAState::constant;
      }

      // -- Instrinsics
      for(const auto& itIntrinsic: sfm_data.GetIntrinsics())
      {
        const IndexT intrinsicId = itIntrinsic.first;
        map_intrinsicId_BAState[intrinsicId] = LocalBAState::refined;
      }

      // -- Landmarks
      for(const auto& itLandmark: sfm_data.structure)
      {
        const IndexT landmarkId = itLandmark.first;
        const Observations & observations = itLandmark.second.observations;

        map_landmarkId_BAState[landmarkId] = LocalBAState::ignored;

        for(const auto& observationIt: observations)
        {
          int dist = map_viewId_distanceToRecentCameras.at(observationIt.first);
          if(dist <= distanceLimit)
          {
            map_landmarkId_BAState[landmarkId] = LocalBAState::refined;
            continue;
          }
        }
      }
    }
    break;

    case 2 :
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
        int dist = map_poseId_distanceToRecentCameras.at(poseId);
        if (dist <= distanceLimit) // 0 or 1
          map_poseId_BAState[poseId] = LocalBAState::refined;
        else if (dist == distanceLimit + 1)
          map_poseId_BAState[poseId] = LocalBAState::constant;
        else
          map_poseId_BAState[poseId] = LocalBAState::ignored;
      }

      // -- Instrinsics
      for(const auto& itIntrinsic: sfm_data.GetIntrinsics())
      {
        const IndexT intrinsicId = itIntrinsic.first;
        map_intrinsicId_BAState[intrinsicId] = LocalBAState::refined;
      }

      // -- Landmarks
      for(const auto& itLandmark: sfm_data.structure)
      {
        const IndexT landmarkId = itLandmark.first;
        const Observations & observations = itLandmark.second.observations;

        map_landmarkId_BAState[landmarkId] = LocalBAState::ignored;

        for(const auto& observationIt: observations)
        {
          int dist = map_viewId_distanceToRecentCameras.at(observationIt.first);
          if(dist <= distanceLimit)
          {
            map_landmarkId_BAState[landmarkId] = LocalBAState::refined;
            continue;
          }
        }
      }
    }
    break;
    
    case 3 :
    {
      // ----------------------------------------------------
      // -- Strategy 3 (DEBUG) 
      // ----------------------------------------------------

      // -- Poses
      for (Poses::const_iterator itPose = sfm_data.poses.begin(); itPose != sfm_data.poses.end(); ++itPose)
      {
        const IndexT poseId = itPose->first;
        int dist = map_poseId_distanceToRecentCameras.at(poseId);
        if (dist < distanceLimit) // 0 
          map_poseId_BAState[poseId] = LocalBAState::refined;
        else if (dist == distanceLimit ) // 1 
          map_poseId_BAState[poseId] = LocalBAState::constant;
        else // 2 
          map_poseId_BAState[poseId] = LocalBAState::ignored;
      }

      // -- Instrinsics
      for(const auto& itIntrinsic: sfm_data.GetIntrinsics())
      {
        const IndexT intrinsicId = itIntrinsic.first;
        map_intrinsicId_BAState[intrinsicId] = LocalBAState::refined;
      }

      // -- Landmarks
      for(const auto& itLandmark: sfm_data.structure)
      {
        const IndexT landmarkId = itLandmark.first;
        const Observations & observations = itLandmark.second.observations;

        map_landmarkId_BAState[landmarkId] = LocalBAState::ignored;

        for(const auto& observationIt: observations)
        {
          int dist = map_viewId_distanceToRecentCameras.at(observationIt.first);
          if(dist < distanceLimit)
          {
            map_landmarkId_BAState[landmarkId] = LocalBAState::refined;
            continue;
          }
        }
      }
    }
    break;
    
    case 4 :
    {
      // ----------------------------------------------------
      // -- Strategy 4 (DEBUG) : Nothing ignored
      // ----------------------------------------------------

      // -- Poses
      for (Poses::const_iterator itPose = sfm_data.poses.begin(); itPose != sfm_data.poses.end(); ++itPose)
      {
        const IndexT poseId = itPose->first;
        int dist = map_poseId_distanceToRecentCameras.at(poseId);
        if (dist <= distanceLimit) // 0 or 1
          map_poseId_BAState[poseId] = LocalBAState::refined;
        else // > 1 
          map_poseId_BAState[poseId] = LocalBAState::constant;
      }

      // -- Instrinsics
      for(const auto& itIntrinsic: sfm_data.GetIntrinsics())
      {
        const IndexT intrinsicId = itIntrinsic.first;
        map_intrinsicId_BAState[intrinsicId] = LocalBAState::refined;
      }

      // -- Landmarks
      for(const auto& itLandmark: sfm_data.structure)
      {
        const IndexT landmarkId = itLandmark.first;
        const Observations & observations = itLandmark.second.observations;

        map_landmarkId_BAState[landmarkId] = LocalBAState::constant;

        for(const auto& observationIt: observations)
        {
          int dist = map_viewId_distanceToRecentCameras.at(observationIt.first);
          if(dist <= distanceLimit)
          {
            map_landmarkId_BAState[landmarkId] = LocalBAState::refined;
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
        map_poseId_BAState[poseId] = LocalBAState::refined;
      }
      // Instrinsics
      for(const auto& itIntrinsic: sfm_data.GetIntrinsics())
      {
        const IndexT intrinsicId = itIntrinsic.first;
        map_intrinsicId_BAState[intrinsicId] = LocalBAState::refined;
      }
      // Landmarks
      for(const auto& itLandmark: sfm_data.structure)
      {
        const IndexT landmarkId = itLandmark.first;
        map_landmarkId_BAState[landmarkId] = LocalBAState::refined;
      }
    }
    break;
  }
}


bool Bundle_Adjustment_Ceres::adjustPartialReconstruction(SfM_Data& sfm_data, BAStats& baStats)
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
  ceres::Problem problem;
  
  // Data wrapper for refinement:
  Hash_Map<IndexT, std::vector<double> > map_posesBlocks;
  Hash_Map<IndexT, std::vector<double> > map_intrinsicsBlocks;

  // Add Poses data to the Ceres problem as Parameter Blocks (do not take care about Local BA strategy)
  map_posesBlocks = addPosesToCeresProblem(sfm_data.poses, problem, solver_options, baStats);
  
  // Add Poses data to the Ceres problem as Parameter Blocks (do not take care about Local BA strategy)
  map_intrinsicsBlocks = addIntrinsicsToCeresProblem(sfm_data, problem, solver_options, baStats);
     
  // Count the number of Refined, Constant & Ignored parameters
  if (_openMVG_options.isLocalBAEnabled())
  {
    for (auto it : map_intrinsicsBlocks)
    {
      IndexT intrinsicId = it.first;
      if (getIntrinsicsBAState(intrinsicId) == refined)  ++baStats.numRefinedIntrinsics;
      if (getIntrinsicsBAState(intrinsicId) == constant) ++baStats.numConstantIntrinsics;
      if (getIntrinsicsBAState(intrinsicId) == ignored)  ++baStats.numIgnoredIntrinsics;
    }
    for (auto it : map_posesBlocks)
    {
      IndexT poseId = it.first;
      if (getPoseBAState(poseId) == refined)  ++baStats.numRefinedPoses;
      if (getPoseBAState(poseId) == constant) ++baStats.numConstantPoses;
      if (getPoseBAState(poseId) == ignored)  ++baStats.numIgnoredPoses;
    }
  }   
  else
  {
    baStats.numRefinedPoses = map_posesBlocks.size();
    baStats.numRefinedIntrinsics = map_intrinsicsBlocks.size();
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
    if (getLandmarkBAState(landmarkId) == refined)  ++baStats.numRefinedLandmarks;
    if (getLandmarkBAState(landmarkId) == constant) ++baStats.numConstantLandmarks;
    if (getLandmarkBAState(landmarkId) == ignored)  ++baStats.numIgnoredLandmarks;
            
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
      if (_openMVG_options.isLocalBAEnabled())
      {
        if (getPoseBAState(poseId) == ignored 
          || getIntrinsicsBAState(intrinsicId) == ignored 
          || getLandmarkBAState(landmarkId) == ignored)
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
          
        // Create a residual block:
        problem.AddResidualBlock(cost_function,
                                 p_LossFunction,
                                 intrinsicBlock,
                                 poseBlock,
                                 landmarkBlock); //Do we need to copy 3D point to avoid false motion, if failure ?
        
        // Set to constant parameters previoously set as Constant by the Local BA strategy
        if (_openMVG_options.isLocalBAEnabled())
        {
          if (getIntrinsicsBAState(intrinsicId) == constant) problem.SetParameterBlockConstant(intrinsicBlock);        
          if (getPoseBAState(poseId) == constant)            problem.SetParameterBlockConstant(poseBlock);
          if (getLandmarkBAState(landmarkId) == constant)    problem.SetParameterBlockConstant(landmarkBlock);
        } 
        
        // Apply a specific parameter ordering: 
        if (_openMVG_options.useParametersOrdering) 
        {
          ceres::ParameterBlockOrdering* linear_solver_ordering = new ceres::ParameterBlockOrdering;
          linear_solver_ordering->AddElementToGroup(landmarkBlock, 0);
          linear_solver_ordering->AddElementToGroup(intrinsicBlock, 1);
          linear_solver_ordering->AddElementToGroup(poseBlock, 2);
          solver_options.linear_solver_ordering.reset(linear_solver_ordering);
        }
      }
    }
  }
 
  ceres::Solver::Summary summary;
  if (!solveBA(problem, solver_options, summary))
    return false;
      
  // Solution is usable
  if (_openMVG_options._bVerbose)
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
  
  // Add statitics about the BA loop:
  baStats.time = summary.total_time_in_seconds;
  baStats.numSuccessfullIterations = summary.num_successful_steps;
  baStats.numUnsuccessfullIterations = summary.num_unsuccessful_steps;
  baStats.RMSEinitial = std::sqrt( summary.initial_cost / summary.num_residuals);
  baStats.RMSEfinal = std::sqrt( summary.final_cost / summary.num_residuals);
  
   if (_openMVG_options._bVerbose && _openMVG_options.isLocalBAEnabled())
  {
    // Generate the histogram <distance, NbOfPoses>
    for (auto it: map_poseId_distanceToRecentCameras) // distanceToRecentPoses: <poseId, distance>
    {
      auto itHisto = baStats.map_distance_numCameras.find(it.second);
      if(itHisto != baStats.map_distance_numCameras.end())
        ++baStats.map_distance_numCameras.at(it.second);
      else // first pose with this specific distance
          baStats.map_distance_numCameras[it.second] = 1;
    }
//    // [Optionnal] Display the histogram:
//    std::cout << "Histrogramme des distances :" << std::endl;
//    for (auto it: baStats.map_distance_numCameras)
//    {
//      std::cout << '[' << it.first << "] " << it.second << " poses" << std::endl;
//    }
    
    // Display statistics about the Local BA
    OPENMVG_LOG_DEBUG(
      "Local BA statistics:\n"
      " #poses: " << baStats.numRefinedPoses << " refined, " 
        << baStats.numConstantPoses << " constant, "
        << baStats.numIgnoredPoses << " ignored.\n"
      " #intrinsics: " << baStats.numRefinedIntrinsics << " refined, " 
        << baStats.numConstantIntrinsics << " constant, "
        << baStats.numIgnoredIntrinsics<< " ignored.\n"   
      " #landmarks: " << baStats.numRefinedLandmarks << " refined, " 
        << baStats.numConstantLandmarks << " constant, "
        << baStats.numIgnoredLandmarks << " ignored.\n"
    );
  }
  
  // Update camera poses with refined data
  updateCameraPoses(map_posesBlocks, sfm_data.poses);

  // Update camera intrinsics with refined data
  updateCameraIntrinsics(map_intrinsicsBlocks, sfm_data.intrinsics);

  return true;
}

Hash_Map<IndexT, std::vector<double> > Bundle_Adjustment_Ceres::addPosesToCeresProblem(
  const Poses & poses,
  ceres::Problem & problem,
  ceres::Solver::Options& options,
  BAStats& baStats)
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

Hash_Map<IndexT, std::vector<double>> Bundle_Adjustment_Ceres::addIntrinsicsToCeresProblem(const SfM_Data & sfm_data,
  ceres::Problem & problem, 
  ceres::Solver::Options &options,
  BAStats& baStats)
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
void Bundle_Adjustment_Ceres::setSolverOptions(ceres::Solver::Options& solver_options)
{
  solver_options.preconditioner_type = _openMVG_options._preconditioner_type;
  solver_options.linear_solver_type = _openMVG_options._linear_solver_type;
  solver_options.sparse_linear_algebra_library_type = _openMVG_options._sparse_linear_algebra_library_type;
  solver_options.minimizer_progress_to_stdout = _openMVG_options._bVerbose;
  solver_options.logging_type = ceres::SILENT;
  solver_options.num_threads = _openMVG_options._nbThreads;
  solver_options.num_linear_solver_threads = _openMVG_options._nbThreads;
}

bool Bundle_Adjustment_Ceres::solveBA(
  ceres::Problem& problem, 
  ceres::Solver::Options& options, 
  ceres::Solver::Summary& summary)
{
  // Configure a BA engine and run it
  // Solve BA
  
  ceres::Solve(options, &problem, &summary);
  if (_openMVG_options._bCeres_Summary)
    OPENMVG_LOG_DEBUG(summary.FullReport());
  
  // If no error, get back refined parameters
  if (!summary.IsSolutionUsable())
  {
    OPENMVG_LOG_WARNING("Bundle Adjustment failed.");
    return false;
  }
  return true;
}

void Bundle_Adjustment_Ceres::updateCameraPoses(
  const Hash_Map<IndexT, std::vector<double>> & map_poses,
  Poses & poses)
{
  for (Poses::iterator itPose = poses.begin();
         itPose != poses.end(); ++itPose)
  {
    const IndexT poseId = itPose->first;
    
    // Do not update a camera pose set as Ignored or Constant in the Local BA strategy
    if (_openMVG_options.isLocalBAEnabled() )
    {
      if (getPoseBAState(poseId) == ignored) 
        continue;
      if (getPoseBAState(poseId) == constant) 
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
  
void Bundle_Adjustment_Ceres::updateCameraIntrinsics(
  const Hash_Map<IndexT, std::vector<double>> & map_intrinsics,
  Intrinsics & intrinsics)
{
  for (const auto& intrinsicsV: map_intrinsics)
  {
    const IndexT intrinsicId = intrinsicsV.first;
    
    // Do not update an camera intrinsic set as Ignored or Constant in the Local BA strategy
    if (_openMVG_options.isLocalBAEnabled() )
    {
      if (getIntrinsicsBAState(intrinsicId) == ignored) 
        continue;
      if (getIntrinsicsBAState(intrinsicId) == constant) 
        continue;
    }
    
    intrinsics[intrinsicId]->updateFromParams(intrinsicsV.second);
  }
}

} // namespace sfm
} // namespace openMVG

