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
ceres::CostFunction * createCostFunctionFromIntrinsics(IntrinsicBase * intrinsic, const Vec2 & observation)
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
      throw std::logic_error("Unrecognized intrinsic type in BA.");
  }
}

/// Create the appropriate cost functor according the provided input rig camera intrinsic model
ceres::CostFunction * createRigCostFunctionFromIntrinsics(IntrinsicBase * intrinsic, const Vec2 & observation)
{
  switch(intrinsic->getType())
  {
    case PINHOLE_CAMERA:
      return new ceres::AutoDiffCostFunction<ResidualErrorFunctor_Pinhole_Intrinsic, 2, 3, 6, 6, 3>(
        new ResidualErrorFunctor_Pinhole_Intrinsic(observation.data()));
    break;
    case PINHOLE_CAMERA_RADIAL1:
      return new ceres::AutoDiffCostFunction<ResidualErrorFunctor_Pinhole_Intrinsic_Radial_K1, 2, 4, 6, 6, 3>(
        new ResidualErrorFunctor_Pinhole_Intrinsic_Radial_K1(observation.data()));
    break;
    case PINHOLE_CAMERA_RADIAL3:
      return new ceres::AutoDiffCostFunction<ResidualErrorFunctor_Pinhole_Intrinsic_Radial_K3, 2, 6, 6, 6, 3>(
        new ResidualErrorFunctor_Pinhole_Intrinsic_Radial_K3(observation.data()));
    break;
    case PINHOLE_CAMERA_BROWN:
      return new ceres::AutoDiffCostFunction<ResidualErrorFunctor_Pinhole_Intrinsic_Brown_T2, 2, 8, 6, 6, 3>(
        new ResidualErrorFunctor_Pinhole_Intrinsic_Brown_T2(observation.data()));
    break;
    case PINHOLE_CAMERA_FISHEYE:
      return new ceres::AutoDiffCostFunction<ResidualErrorFunctor_Pinhole_Intrinsic_Fisheye, 2, 7, 6, 6, 3>(
        new ResidualErrorFunctor_Pinhole_Intrinsic_Fisheye(observation.data()));
    case PINHOLE_CAMERA_FISHEYE1:
      return new ceres::AutoDiffCostFunction<ResidualErrorFunctor_Pinhole_Intrinsic_Fisheye1, 2, 4, 6, 6, 3>(
        new ResidualErrorFunctor_Pinhole_Intrinsic_Fisheye1(observation.data()));
    default:
      throw std::logic_error("Unrecognized intrinsic type in BA.");
  }
}


void addPose(ceres::Problem& problem,
             BA_Refine refineOptions,
             const Pose3 & pose,
             std::vector<double>& out_poseParams)
{
  const Mat3 R = pose.rotation();
  const Vec3 t = pose.translation();

  double angleAxis[3];
  ceres::RotationMatrixToAngleAxis((const double*)R.data(), angleAxis);
  out_poseParams.reserve(6); //angleAxis + translation
  out_poseParams.push_back(angleAxis[0]);
  out_poseParams.push_back(angleAxis[1]);
  out_poseParams.push_back(angleAxis[2]);
  out_poseParams.push_back(t(0));
  out_poseParams.push_back(t(1));
  out_poseParams.push_back(t(2));

  double * parameter_block = &out_poseParams[0];
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

Bundle_Adjustment_Ceres::BA_options::BA_options(const bool bVerbose, bool bmultithreaded)
  :_bVerbose(bVerbose)
{
  // set number of threads, 1 if openMP is not enabled
  _nbThreads = omp_get_max_threads();

  if (!bmultithreaded)
    _nbThreads = 1;

  _bCeres_Summary = false;
  
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
  for (Poses::const_iterator itPose = sfm_data.GetPoses().begin(); itPose != sfm_data.GetPoses().end(); ++itPose)
  {
    const IndexT indexPose = itPose->first;
    const Pose3& pose = itPose->second;

    OPENMVG_LOG_DEBUG("[BA] declare pose: " << indexPose);
    addPose(problem, refineOptions, pose, map_poses[indexPose]);
  }

  // Setup rig sub-poses
  Hash_Map<IndexT, Hash_Map<IndexT, std::vector<double>>> map_subposes;
  for(const auto& rigIt : sfm_data.getRigs())
  {
    const IndexT rigId = rigIt.first;
    const Rig& rig = rigIt.second;
    const std::size_t nbSubPoses = rig.getNbSubPoses();

    for(std::size_t subPoseId = 0 ; subPoseId < nbSubPoses; ++subPoseId)
    {
      const RigSubPose& rigSubPose = rig.getSubPose(subPoseId);

      if(rigSubPose.status == ERigSubPoseStatus::UNINITIALIZED)
        continue;

      OPENMVG_LOG_DEBUG("[BA] declare RIG: " << rigId << ", " << subPoseId);
      addPose(problem, refineOptions, rigSubPose.pose, map_subposes[rigId][subPoseId]);
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
        OPENMVG_LOG_DEBUG("[BA] declare intrinsic: " << idIntrinsics);
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

      if(view->isPartOfRig())
      {
        ceres::CostFunction* costFunction = createRigCostFunctionFromIntrinsics(sfm_data.intrinsics[view->id_intrinsic].get(), observationIt.second.x);

        const Rig& rig = sfm_data.getRig(*view);
        const RigSubPose& rigSubPose = rig.getSubPose(view->getSubPoseId());
        assert(rigSubPose.status != ERigSubPoseStatus::UNINITIALIZED);

        double* subpose_ptr = &map_subposes.at(view->getRigId()).at(view->getSubPoseId())[0];

        OPENMVG_LOG_DEBUG("[BA] declare landmark: " << landmarkIt.first << ", RIG observation: " << view->getRigId() << ", " << view->getSubPoseId());
        problem.AddResidualBlock(
          costFunction,
          p_LossFunction,
          &map_intrinsics[view->id_intrinsic][0],
          &map_poses[view->id_pose][0],
          subpose_ptr, // subpose of the cameras rig
          landmarkIt.second.X.data()); //Do we need to copy 3D point to avoid false motion, if failure ?
      }
      else
      {
        ceres::CostFunction* costFunction = createCostFunctionFromIntrinsics(sfm_data.intrinsics[view->id_intrinsic].get(), observationIt.second.x);

        problem.AddResidualBlock(
          costFunction,
          p_LossFunction,
          &map_intrinsics[view->id_intrinsic][0],
          &map_poses[view->id_pose][0],
          landmarkIt.second.X.data()); //Do we need to copy 3D point to avoid false motion, if failure ?
      }
    }
    if (!(refineOptions & BA_REFINE_STRUCTURE))
      problem.SetParameterBlockConstant(landmarkIt.second.X.data());
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
      " #poses: " << sfm_data.GetPoses().size() << "\n"
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
    for (Poses::iterator itPose = sfm_data.GetPoses().begin();
      itPose != sfm_data.GetPoses().end(); ++itPose)
    {
      const IndexT indexPose = itPose->first;

      Mat3 R_refined;
      ceres::AngleAxisToRotationMatrix(&map_poses[indexPose][0], R_refined.data());
      Vec3 t_refined(map_poses[indexPose][3], map_poses[indexPose][4], map_poses[indexPose][5]);
      // Update the pose
      Pose3 & pose = itPose->second;
      pose = poseFromRT(R_refined, t_refined);
    }

    for(const auto& rigIt : map_subposes)
    {
      Rig& rig = sfm_data.getRigs().at(rigIt.first);

      for(const auto& subPoseit : rigIt.second)
      {
        RigSubPose& subpose = rig.getSubPose(subPoseit.first);

        Mat3 R_refined;
        ceres::AngleAxisToRotationMatrix(&subPoseit.second[0], R_refined.data());
        Vec3 t_refined(subPoseit.second[3], subPoseit.second[4], subPoseit.second[5]);

        // Update the pose
        subpose.pose = poseFromRT(R_refined, t_refined);
      }
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

} // namespace sfm
} // namespace openMVG

