// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Rig.hpp"
#include "ResidualError.hpp"
#include <aliceVision/sfm/BundleAdjustmentCeres.hpp>
#include <aliceVision/system/Logger.hpp>

#include <ceres/rotation.h>

#include <fstream>
#include <exception>

#ifdef VISUAL_DEBUG_MODE
#include <opencv2/opencv.hpp>
#endif

namespace aliceVision {
namespace rig {

Rig::~Rig()
{
}

const std::size_t Rig::getRelativePosesSize() const 
{ 
  return _vRelativePoses.size(); 
}

const geometry::Pose3& Rig::getRelativePose(std::size_t i) const 
{ 
  return _vRelativePoses.at(i-1); 
}

const std::vector<geometry::Pose3>& Rig::getRelativePoses() const 
{ 
  return _vRelativePoses; 
}

const geometry::Pose3 & Rig::getPose(std::size_t i) const 
{ 
  return _vPoses.at(i); 
}

const std::size_t Rig::getPosesSize( ) const 
{ 
  return _vPoses.size(); 
}

const std::vector<geometry::Pose3> & Rig::getPoses( ) const 
{ 
  return _vPoses; 
}

void Rig::setTrackingResult(
        std::vector<localization::LocalizationResult> vLocalizationResults,
        std::size_t i)
{
  _vLocalizationResults.emplace(i, vLocalizationResults);
}

bool Rig::initializeCalibration()
{
  if(_isInitialized)
  {
    ALICEVISION_LOG_DEBUG("The rig is already initialized");
    return _isInitialized;
  }
  // check that there are cameras
  assert(!_vLocalizationResults.empty());
  
  // make all the cameras have the same number of localizationResults (equal to the shortest clip)
  const std::size_t nCams = _vLocalizationResults.size();
  {
    std::size_t shortestSeqLength = std::numeric_limits<std::size_t>::max();
    for (const auto& v: _vLocalizationResults)
      shortestSeqLength = std::min(shortestSeqLength, v.second.size());
    for (auto& v: _vLocalizationResults)
      v.second.resize(shortestSeqLength);
  
    if(shortestSeqLength == 0)
    {
        ALICEVISION_LOG_DEBUG("The calibration results are empty!");
        return false;
    }
  }
  
  // Tracker of the main cameras
  std::vector<localization::LocalizationResult> & resMainCamera = _vLocalizationResults[0];
  
  // Clear all relative poses
  _vRelativePoses.clear();
  _vRelativePoses.reserve(nCams-1);
  
  // Loop over all witness cameras
  for(std::size_t iLocalizer=1 ; iLocalizer < nCams ; ++iLocalizer)
  {
    // Perform the pose averaging over all relative pose between the main camera
    // (index 0) and the witness camera (index i)
    std::vector<localization::LocalizationResult> & resWitnessCamera = _vLocalizationResults[iLocalizer];
    
    // vRelativePoses will store all the relative poses overall frames where both
    // the pose computation of the main camera and witness camera succeed
    std::vector<geometry::Pose3> vRelativePoses;
    vRelativePoses.reserve(resWitnessCamera.size());
    
    for(std::size_t iView=0 ; iView < resWitnessCamera.size() ; ++iView )
    {
      // Check that both pose computations succeeded 
      if ( resMainCamera[iView].isValid() && resWitnessCamera[iView].isValid() )
      {
        // q0 ~ [poseMain] Q
        // q1 ~ [relative][poseMain] Q = [poseWitness]Q
        // [relative] = [poseWitness]*inv([poseMain])
        vRelativePoses.push_back(computeRelativePose(resMainCamera[iView].getPose(), 
                                                     resWitnessCamera[iView].getPose()));
      }
    }
    if(vRelativePoses.empty())
    {
      ALICEVISION_CERR("Unable to find candidate poses between the main camera and "
              "the witness camera " << iLocalizer << "\nAborting...");
      return false;
    }
    geometry::Pose3 optimalRelativePose;
    findBestRelativePose(vRelativePoses, iLocalizer, optimalRelativePose );
  
    //poseAveraging(vRelativePoses, averageRelativePose);
    _vRelativePoses.push_back(optimalRelativePose);
  }
  
  // Update all poses in all localization results
  for(std::size_t iRelativePose = 0 ; iRelativePose < _vRelativePoses.size() ; ++iRelativePose )
  {
    std::size_t iRes = iRelativePose+1;
    for(std::size_t iView = 0 ; iView < _vLocalizationResults[iRes].size() ; ++iView )
    {
      if(  _vLocalizationResults[0][iView].isValid() )
      {
        const geometry::Pose3 & relativePose = _vRelativePoses[iRelativePose];
        
        const geometry::Pose3 poseWitnessCamera = poseFromMainToWitness(_vLocalizationResults[0][iView].getPose(), relativePose);
        _vLocalizationResults[iRes][iView].setPose(poseWitnessCamera);
        
      }
    }
  }
  _isInitialized = true;
  return _isInitialized;
}

// From a set of relative pose, find the optimal one for a given tracker iTraker which
// minimize the reprojection errors over all images
void Rig::findBestRelativePose(
        const std::vector<geometry::Pose3> & vPoses,
        std::size_t iLocalizer,
        geometry::Pose3 & result )
{
  assert(iLocalizer < _vLocalizationResults.size());
  assert(_vLocalizationResults.size() > 0);
  
  const std::vector<localization::LocalizationResult> & resMainCamera = _vLocalizationResults[0];
  const std::vector<localization::LocalizationResult> & resWitnessCamera = _vLocalizationResults[iLocalizer];
  
  double minReprojError = std::numeric_limits<double>::max();
  std::size_t iMin = 0;
  
  assert(vPoses.size() > 0);
  
  for(std::size_t i=0 ; i < vPoses.size() ; ++i)
  {
    const geometry::Pose3 & relativePose = vPoses[i];

    double error = 0;
    for(std::size_t j=0 ; j < resWitnessCamera.size() ; ++j )
    {
      // Check that both pose computations succeeded 
      if ( ( resMainCamera[j].isValid() ) && ( resWitnessCamera[j].isValid() ) )
      {
        // [poseWitness] = [relativePose]*[poseMainCamera]
        const geometry::Pose3 poseWitnessCamera = poseFromMainToWitness(resMainCamera[j].getPose(), relativePose);
        error += reprojectionError(resWitnessCamera[j], poseWitnessCamera);
      }
    }
    if ( error < minReprojError )
    {
      iMin = i;
      minReprojError = error;
    }           
  }
  result = vPoses[iMin];
  
  displayRelativePoseReprojection(geometry::Pose3(aliceVision::Mat3::Identity(), aliceVision::Vec3::Zero()), 0);
  displayRelativePoseReprojection(result, iLocalizer);
}

// Display reprojection error based on a relative pose
void Rig::displayRelativePoseReprojection(const geometry::Pose3 & relativePose, std::size_t iLocalizer)
{
#ifdef VISUAL_DEBUG_MODE
  std::vector<localization::LocalizationResult> & mainLocalizerResults = _vLocalizationResults[0];
  std::vector<localization::LocalizationResult> & witnessLocalizerResults = _vLocalizationResults[iLocalizer];

  // Set the marker size
  std::size_t semiWidth = 3.0;
  std::size_t marginUndisto = 200;
          
  for(int iView=0 ; iView < witnessLocalizerResults.size() ; ++iView )
  {
        // Check that both pose computations succeed 
    if ( witnessLocalizerResults[iView].isValid() )
    {
      const std::size_t width = witnessLocalizerResults[iView].getIntrinsics()._w;
      const std::size_t height = witnessLocalizerResults[iView].getIntrinsics()._h;

      // Window to display reprojection errors
      cv::Mat imgRes(height+2*marginUndisto, width+2*marginUndisto, CV_8UC3);
      imgRes = cv::Scalar(255,255,255);
      cvNamedWindow("Reprojection", CV_WINDOW_NORMAL);
      cv::moveWindow("Reprojection",0,0);
      cv::resizeWindow("Reprojection", width/2, height/2);
      cv::imshow("Reprojection", imgRes);
      
      const geometry::Pose3 poseWitnessCamera = poseFromMainToWitness(mainLocalizerResults[iView].getPose(), relativePose);
        
      const aliceVision::Mat points2D = witnessLocalizerResults[iView].getPt2D();
      const aliceVision::Mat points3D = witnessLocalizerResults[iView].getPt3D();
      
      for(const IndexT iInlier : witnessLocalizerResults[iView].getInliers())
      {
        // Reprojections
        const Vec3 & point3D = points3D.col(iInlier);
        // Its reprojection
        Vec2 itsReprojection = witnessLocalizerResults[iView].getIntrinsics().project(poseWitnessCamera, point3D);
        // Its associated observation location
        const Vec2 & point2D = points2D.col(iInlier);
 
        // Display reprojections and observations
        cv::rectangle(imgRes, 
                cvPoint(marginUndisto+point2D(0)-semiWidth,marginUndisto+point2D(1)-semiWidth),
                cvPoint(marginUndisto+point2D(0)+semiWidth,marginUndisto+point2D(1)+semiWidth),
                cv::Scalar(0,255,0));
        
        cv::rectangle(imgRes,
                cvPoint(marginUndisto+itsReprojection(0)-semiWidth,marginUndisto+itsReprojection(1)-semiWidth),
                cvPoint(marginUndisto+itsReprojection(0)+semiWidth,marginUndisto+itsReprojection(1)+semiWidth),
                cv::Scalar(255,0,0));
      }
      cv::imshow("Reprojection", imgRes);
      cvpause();
    }
  }
#endif
}

bool Rig::optimizeCalibration()
{
  if(!_isInitialized)
  {
    ALICEVISION_LOG_DEBUG("The rig is yet initialized");
    return _isInitialized;
  }
  
  assert(_vRelativePoses.size() > 0);
  assert(_vLocalizationResults.size() > 0);
  
  ceres::Problem problem;

  // Add relative pose as a parameter block over all witness cameras.
  std::vector<std::vector<double> > vRelativePoses;
  vRelativePoses.reserve(_vRelativePoses.size());
  for(std::size_t iRelativePose = 0 ; iRelativePose < _vRelativePoses.size() ; ++iRelativePose )
  {
    geometry::Pose3 & pose = _vRelativePoses[iRelativePose];
    
    const aliceVision::Mat3 & R = pose.rotation();
    const aliceVision::Vec3 & t = pose.translation();

    double angleAxis[3];
    ceres::RotationMatrixToAngleAxis((const double*)R.data(), angleAxis);
    
    std::vector<double> relativePose;
    relativePose.reserve(6); //angleAxis + translation
    relativePose.push_back(angleAxis[0]);
    relativePose.push_back(angleAxis[1]);
    relativePose.push_back(angleAxis[2]);
    relativePose.push_back(t(0));
    relativePose.push_back(t(1));
    relativePose.push_back(t(2));

    vRelativePoses.push_back(relativePose);
  }
  for(std::vector<double> &pose : vRelativePoses)
  {
    double * parameter_block = &pose[0];
    assert(parameter_block && "parameter_block is null in vRelativePoses");
    problem.AddParameterBlock(parameter_block, 6);
  }
  
  // Add rig pose (i.e. main camera pose) as a parameter block over all views (i.e. timestamps).
  std::map<std::size_t, std::vector<double> > vMainPoses;
  for(std::size_t iView = 0 ; iView < _vLocalizationResults[0].size() ; ++iView )
  {
    if(!_vLocalizationResults[0][iView].isValid())
      continue;
    
    const geometry::Pose3 & pose = _vLocalizationResults[0][iView].getPose();
    const aliceVision::Mat3 & R = pose.rotation();
    const aliceVision::Vec3 & t = pose.translation();

    double angleAxis[3];
    ceres::RotationMatrixToAngleAxis((const double*)R.data(), angleAxis);

    std::vector<double> mainPose;
    mainPose.reserve(6); //angleAxis + translation
    mainPose.push_back(angleAxis[0]);
    mainPose.push_back(angleAxis[1]);
    mainPose.push_back(angleAxis[2]);
    mainPose.push_back(t(0));
    mainPose.push_back(t(1));
    mainPose.push_back(t(2));

    vMainPoses.emplace(iView, mainPose);
  }

  for(auto &elem : vMainPoses)
  {
    auto pose = elem.second;
    double * parameter_block = &pose[0];
    assert(parameter_block && "parameter_block is null in vMainPoses");
    problem.AddParameterBlock(parameter_block, 6);
  }

// The following code can be used if the intrinsics have to be refined in the bundle adjustment
  
//for (Intrinsics::const_iterator itIntrinsic = sfm_data.intrinsics.begin();
//                      itIntrinsic != sfm_data.intrinsics.end(); ++itIntrinsic)
//{
//  for (int iLocalizer = 0 ; iLocalizer < _vLocalizationResults.size() ; ++iLocalizer )
//  {
//    
//    for (int iView = 0 ; iView < _vLocalizationResults[iLocalizer].size() ; ++iView )
//    {
//      const localization::LocalizationResult & localizationResult = _vLocalizationResults[iLocalizer][iView];
//      
//      if ( localizationResult.isValid() )
//      {
//        std::vector<double> intrinsics;
//
//        switch (localizationResult.getIntrinsics().getType())
//        {
//          case PINHOLE_CAMERA:
//          {
//            std::vector<double> vec_params = localizationResult.getIntrinsics().getParams();
//            intrinsics.reserve(3);
//            intrinsics.push_back(vec_params[0]);
//            intrinsics.push_back(vec_params[1]);
//            intrinsics.push_back(vec_params[2]);
//
//            double * parameter_block = &intrinsics[0];
//            problem.AddParameterBlock(parameter_block, 3);
//            if (true) //(!bRefineIntrinsics)
//            {
//              //set the whole parameter block as constant for best performance.
//              problem.SetParameterBlockConstant(parameter_block);
//            }
//          }
//          case PINHOLE_CAMERA_RADIAL3
//          {
//            std::vector<double> vec_params = localizationResult.getIntrinsics().getParams();
//            intrinsics.reserve(6);
//            intrinsics.push_back(vec_params[0]);
//            intrinsics.push_back(vec_params[1]);
//            intrinsics.push_back(vec_params[2]);
//            intrinsics.push_back(vec_params[3]);
//            intrinsics.push_back(vec_params[4]);
//            intrinsics.push_back(vec_params[5]);
//
//            double * parameter_block = &intrinsics[0];
//            problem.AddParameterBlock(parameter_block, 6);
//            if (true) //(!bRefineIntrinsics)
//            {
//              //set the whole parameter block as constant for best performance.
//              problem.SetParameterBlockConstant(parameter_block);
//            }
//          }
//        }
//      }
//    }
//  }

  // Set a LossFunction to be less penalized by false measurements
  //  - set it to NULL if you don't want use a lossFunction.
  ceres::LossFunction * p_LossFunction = nullptr;//new ceres::HuberLoss(Square(4.0));
  // todo: make the LOSS function and the parameter an option

  // For all visibility add reprojections errors:
  for(std::size_t iLocalizer = 0 ; iLocalizer < _vLocalizationResults.size() ; ++iLocalizer)
  {
    const std::vector<localization::LocalizationResult> & currentResult = _vLocalizationResults[iLocalizer];
    
    for(std::size_t iView = 0 ; iView < currentResult.size() ; ++iView)
    {
      // if the current localization is not valid skip it
      if(!currentResult[iView].isValid())
        continue;
      
      // if it is not the main camera and the same view was not localized for the 
      // main camera, skip it
      if( (iLocalizer != 0) && (!_vLocalizationResults[0][iView].isValid()) )
        continue;
      
      // Get the inliers 3D points
      const Mat & points3D = currentResult[iView].getPt3D();
      // Get their image locations (also referred as observations)
      const Mat & points2D = currentResult[iView].getPt2D();
      
      // Add a residual block for all inliers
      for(const IndexT iPoint : currentResult[iView].getInliers() )
      {
        // Each Residual block takes a point and a camera as input and outputs a 2
        // dimensional residual. Internally, the cost function stores the observations
        // and the 3D point and compares the reprojection against the observation.
        ceres::CostFunction* cost_function;

        // Add a residual block for the main camera
        if ( iLocalizer == 0 )
        {
          // Vector-2 residual, pose of the rig parameterized by 6 parameters
          cost_function = new ceres::AutoDiffCostFunction<ResidualErrorMainCameraFunctor, 2, 6>(
          new ResidualErrorMainCameraFunctor(currentResult[iView].getIntrinsics(), points2D.col(iPoint), points3D.col(iPoint) ));
            
          if (cost_function)
          {
            assert(vMainPoses.find(iView) != vMainPoses.end());
            problem.AddResidualBlock( cost_function,
                                      p_LossFunction,
                                      &vMainPoses[iView][0]);
          }else
          {
            ALICEVISION_CERR("Fail in adding residual block for the main camera");
          }

        }else
        // Add a residual block for a secondary camera
        {
            // Vector-2 residual, pose of the rig parameterized by 6 parameters
            //                  + relative pose of the secondary camera parameterized by 6 parameters
            
            cost_function = new ceres::AutoDiffCostFunction<ResidualErrorSecondaryCameraFunctor, 2, 6, 6>(
          new ResidualErrorSecondaryCameraFunctor(currentResult[iView].getIntrinsics(), points2D.col(iPoint), points3D.col(iPoint)));
          
          if (cost_function)
          {
            assert(vMainPoses.find(iView) != vMainPoses.end());
            assert(iLocalizer-1 < vRelativePoses.size());
            problem.AddResidualBlock( cost_function,
                                      p_LossFunction,
                                      &vMainPoses[iView][0],
                                      &vRelativePoses[iLocalizer-1][0]);
          }else
          {
            ALICEVISION_CERR("Fail in adding residual block for a secondary camera");
          }
        }
      }
    }
  }

  // Configure a BA engine and run it
  // todo: Set the most appropriate options
  aliceVision::sfm::BundleAdjustmentCeres::CeresOptions aliceVision_options(true);
  
  ceres::Solver::Options options;
  
  options.preconditioner_type = aliceVision_options.preconditionerType;
  options.linear_solver_type = aliceVision_options.linearSolverType;
  options.sparse_linear_algebra_library_type = aliceVision_options.sparseLinearAlgebraLibraryType;
  options.minimizer_progress_to_stdout = aliceVision_options.verbose;
  options.logging_type = ceres::SILENT;
  options.num_threads = 1;//aliceVision_options._nbThreads;
#if CERES_VERSION_MAJOR < 2
  options.num_linear_solver_threads = 1;//aliceVision_options._nbThreads;
#endif
  
  // Solve BA
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  
  if (aliceVision_options.summary)
    ALICEVISION_LOG_DEBUG(summary.FullReport());

  // If no error, get back refined parameters
  if (!summary.IsSolutionUsable())
  {
    if (aliceVision_options.verbose)
      ALICEVISION_LOG_DEBUG("Bundle Adjustment failed.");
    return false;
  }

  if (aliceVision_options.verbose)
  {
    // Display statistics about the minimization
    ALICEVISION_LOG_DEBUG("Bundle Adjustment statistics (approximated RMSE):");
    ALICEVISION_LOG_DEBUG(" #localizers: " << _vLocalizationResults.size());
    ALICEVISION_LOG_DEBUG(" #views: " << _vLocalizationResults[0].size());
    ALICEVISION_LOG_DEBUG(" #residuals: " << summary.num_residuals);
    ALICEVISION_LOG_DEBUG(" Initial RMSE: " << std::sqrt( summary.initial_cost / summary.num_residuals));
    ALICEVISION_LOG_DEBUG(" Final RMSE: " << std::sqrt( summary.final_cost / summary.num_residuals));
  }

  // Update relative pose after optimization
  for(std::size_t iRelativePose = 0 ; iRelativePose < _vRelativePoses.size() ; ++iRelativePose)
  {
    aliceVision::Mat3 R_refined;
    std::vector<double> vPose;
    vPose.reserve(6);
    ceres::AngleAxisToRotationMatrix(&vRelativePoses[iRelativePose][0], R_refined.data());
    aliceVision::Vec3 t_refined(vRelativePoses[iRelativePose][3], vRelativePoses[iRelativePose][4], vRelativePoses[iRelativePose][5]);
    // Update the pose
    geometry::Pose3 & pose = _vRelativePoses[iRelativePose];
    pose = geometry::Pose3(R_refined, -R_refined.transpose() * t_refined);
  }
    
    // Update the main camera pose after optimization
  for(std::size_t iView = 0 ; iView < _vLocalizationResults[0].size() ; ++iView)
  {
    if( _vLocalizationResults[0][iView].isValid() )
    {
      aliceVision::Mat3 R_refined;
      std::vector<double> vPose;
      vPose.reserve(6);
      ceres::AngleAxisToRotationMatrix(&vMainPoses[iView][0], R_refined.data());
      aliceVision::Vec3 t_refined(vMainPoses[iView][3], vMainPoses[iView][4], vMainPoses[iView][5]);
      // Push the optimized pose
      geometry::Pose3 pose = geometry::Pose3(R_refined, -R_refined.transpose() * t_refined);
      _vLocalizationResults[0][iView].setPose(pose);
      _vPoses.push_back(pose);
    }
  }
    
  displayRelativePoseReprojection(geometry::Pose3(aliceVision::Mat3::Identity(), aliceVision::Vec3::Zero()), 0);
    
    // Update all poses over all witness cameras after optimization
  for(std::size_t iRelativePose = 0; iRelativePose < _vRelativePoses.size(); ++iRelativePose)
  {
    const std::size_t iLocalizer = iRelativePose+1;
    // Loop over all views
    for(std::size_t iView = 0; iView < _vLocalizationResults[iLocalizer].size(); ++iView)
    {
      // If the localization has succeeded then if the witness camera localization succeeded 
      // then update the pose else continue.
      if( _vLocalizationResults[0][iView].isValid() && _vLocalizationResults[iLocalizer][iView].isValid() )
      {
        // Retrieve the witness camera pose from the main camera one.
        const geometry::Pose3 poseWitnessCamera = poseFromMainToWitness(_vLocalizationResults[0][iView].getPose(), _vRelativePoses[iRelativePose]);
        _vLocalizationResults[iLocalizer][iView].setPose(poseWitnessCamera);
      }
    }
    displayRelativePoseReprojection(_vRelativePoses[iRelativePose], iLocalizer);
  }
  // Possibility to update the intrinsics here

  // Update camera intrinsics with refined data
  //    for (Intrinsics::iterator itIntrinsic = sfm_data.intrinsics.begin();
  //      itIntrinsic != sfm_data.intrinsics.end(); ++itIntrinsic)
  //    {
  //      const IndexT indexCam = itIntrinsic->first;
  //
  //      const std::vector<double> & vec_params = map_intrinsics[indexCam];
  //      itIntrinsic->second.get()->updateFromParams(vec_params);
  //    }
  return true;
}


bool Rig::saveCalibration(const std::string &filename)
{
  return saveRigCalibration(filename, _vRelativePoses);
}


// it returns poseWitnessCamera*inv(poseMainCamera)
geometry::Pose3 computeRelativePose(const geometry::Pose3 &poseMainCamera, const geometry::Pose3 &poseWitnessCamera)
{
  const aliceVision::Mat3 & R1 = poseMainCamera.rotation();
  const aliceVision::Vec3 & t1 = poseMainCamera.translation();
  const aliceVision::Mat3 & R2 = poseWitnessCamera.rotation();
  const aliceVision::Vec3 & t2 = poseWitnessCamera.translation();

  const aliceVision::Mat3 R12 = R2 * R1.transpose();
  const aliceVision::Vec3 t12 = t2 - R12 * t1;

  return geometry::Pose3( R12 , -R12.transpose()*t12 );
}
        

// it returns relativePose*poseMainCamera
geometry::Pose3 poseFromMainToWitness(const geometry::Pose3 &poseMainCamera, const geometry::Pose3 &relativePose)
{
  const aliceVision::Mat3 & R1 = poseMainCamera.rotation();
  const aliceVision::Vec3 & t1 = poseMainCamera.translation();

  const aliceVision::Mat3 & R12 = relativePose.rotation();
  const aliceVision::Vec3 & t12 = relativePose.translation();

  const aliceVision::Mat3 R2 = R12 * R1;
  const aliceVision::Vec3 t2 = R12 * t1 + t12 ;
  
  return geometry::Pose3( R2 , -R2.transpose() * t2 );
}

double reprojectionError(const localization::LocalizationResult & localizationResult, const geometry::Pose3 & pose)
{
  double residual = 0;
  
  for(const IndexT iInliers : localizationResult.getInliers())
  {
    // Inlier 3D point
    const Vec3 & point3D = localizationResult.getPt3D().col(iInliers);
    // Its reprojection
    const Vec2 itsReprojection = localizationResult.getIntrinsics().project(pose, point3D.homogeneous());
    // Its associated observation location
    const Vec2 & point2D = localizationResult.getPt2D().col(iInliers);
    // Residual
    residual += (point2D(0) - itsReprojection(0))*(point2D(0) - itsReprojection(0));
    residual += (point2D(1) - itsReprojection(1))*(point2D(1) - itsReprojection(1));
  }
  return residual;
}

void cvpause(){
#ifdef VISUAL_DEBUG_MODE
  int keyboard;
  while( !(char) keyboard ){ // ASCII code for 'CR'
    keyboard = cv::waitKey( 0 );
  }

  if ( (char) keyboard == 'q' )
  {
    ALICEVISION_CERR("The program has been manually stopped");
    std::exit(0);
  }
#endif
}

bool loadRigCalibration(const std::string &filename, std::vector<geometry::Pose3> &subposes)
{
  std::ifstream fs(filename, std::ios::in);
  if(!fs.is_open())
  {
    ALICEVISION_CERR("Unable to load the calibration file " << filename);
    throw std::invalid_argument("Unable to load the calibration file "+filename);
  }  
  
  // first read the number of cameras subposes stores
  std::size_t numCameras = 0;
  fs >> numCameras;
  ALICEVISION_LOG_DEBUG("Found " << numCameras << " cameras");
  subposes.reserve(numCameras);
  
  for(std::size_t cam = 0; cam < numCameras; ++cam)
  {
    // load the rotation part
    Mat3 rot;
    for(std::size_t i = 0; i < 3; ++i)
      for(std::size_t j = 0; j < 3; ++j)
        fs >> rot(i,j);
    
    // load the center
    Vec3 center;
    fs >> center(0);
    fs >> center(1);
    fs >> center(2);
    
    // add the pose in the vector
    subposes.emplace_back(rot, center);
  }
  
  bool isOk = fs.good();
  fs.close();
  return isOk;
}

//numCam
//R[0][0] // first camera rotation
//R[0][1]
//...
//C[0] // first camera center
//C[1]
//C[2]
//R[0][0] // second camera rotation
//...
bool saveRigCalibration(const std::string &filename, const std::vector<geometry::Pose3> &subposes)
{
  std::ofstream fs(filename, std::ios::out);
  if(!fs.is_open())
  {
    ALICEVISION_CERR("Unable to create the calibration file " << filename);
    throw std::invalid_argument("Unable to create the calibration file "+filename);
  }
  fs << subposes.size() << std::endl;
  
  for(const geometry::Pose3 & p : subposes)
  {
    // write the rotatiop part
    const Mat3 &rot = p.rotation();
    for(std::size_t i = 0; i < 3; ++i)
      for(std::size_t j = 0; j < 3; ++j)
        fs << rot(i,j) << std::endl;
    
    // write the center
    const Vec3 &center = p.center();
    fs << center(0) << std::endl;
    fs << center(1) << std::endl;
    fs << center(2) << std::endl;
  }
  const bool isOk = fs.good();
  fs.close();
  return isOk;
}

} // namespace rig
} // namespace aliceVision
