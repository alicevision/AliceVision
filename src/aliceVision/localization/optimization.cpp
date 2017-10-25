// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "optimization.hpp"
#include "aliceVision/sfm/sfmDataIO.hpp"
#include <aliceVision/sfm/BundleAdjustmentCeres.hpp>
#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/rig/ResidualError.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/Timer.hpp>


#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/density.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/sum.hpp>

namespace aliceVision{
namespace localization{

bool refineSequence(std::vector<LocalizationResult> & vec_localizationResult,
                    bool allTheSameIntrinsics /*= true*/,
                    bool b_refine_intrinsic /*= true*/,
                    bool b_no_distortion /*= false*/,
                    bool b_refine_pose /*= true*/,
                    bool b_refine_structure /*= false*/,
                    const std::string outputFilename /*= ""*/,
                    std::size_t minPointVisibility /*=0*/)
{
  
  const std::size_t numViews = vec_localizationResult.size();
  assert(numViews > 0 );
   
  // the id for the instrinsic group
  IndexT intrinsicID = 0;
    
  // Setup a tiny SfM scene with the corresponding 2D-3D data
  sfm::SfMData tinyScene;
  
  // if we have only one camera just set the intrinsics group once for all
  if(allTheSameIntrinsics)
  {
    // find the first valid localization result and use its intrinsics
    std::size_t intrinsicIndex = 0;
    for(std::size_t viewID = 0; viewID < numViews; ++viewID, ++intrinsicIndex)
    {
      if(vec_localizationResult[viewID].isValid())
        break;
    }
    // it may be the case that all the localization result are invalid...
    if(!vec_localizationResult[intrinsicIndex].isValid())
    {
      ALICEVISION_CERR("Apparently all the vec_localizationResult are invalid! Aborting...");
      return false;
    }
    
    ALICEVISION_CERR("allTheSameIntrinsics mode: using the intrinsics of the " << intrinsicIndex << " result");
    
    camera::PinholeRadialK3* currIntrinsics = &vec_localizationResult[intrinsicIndex].getIntrinsics();
    
    if(b_no_distortion)
    {
      // no distortion refinement
      ALICEVISION_LOG_DEBUG("Optical distortion won't be considered");
      // just add a simple pinhole camera with the same K as the input camera
      Vec2 pp = currIntrinsics->principal_point();
      tinyScene.intrinsics[intrinsicID] = std::make_shared<camera::Pinhole>(currIntrinsics->_w, currIntrinsics->_h, currIntrinsics->focal(), pp(0), pp(1));
    }
    else
    {
      // intrinsic (the shared_ptr does not take the ownership, will not release the input pointer)
      tinyScene.intrinsics[intrinsicID] = std::shared_ptr<camera::PinholeRadialK3>(currIntrinsics, [](camera::PinholeRadialK3*){});
      ALICEVISION_LOG_DEBUG("Type of intrinsics " <<tinyScene.intrinsics[0].get()->getType());
    }
  }
  
  {
    // just debugging -- print reprojection errors -- this block can be safely removed/commented out
    for(size_t viewID = 0; viewID < numViews; ++viewID)
    {
      const LocalizationResult &currResult = vec_localizationResult[viewID];
      if(!currResult.isValid())
      {
        continue;
      }
      
      const Mat2X residuals = currResult.computeInliersResiduals();
      
      const auto sqrErrors = (residuals.cwiseProduct(residuals)).colwise().sum();
      ALICEVISION_LOG_DEBUG("View " << viewID << " RMSE = " << std::sqrt(sqrErrors.mean()) 
              << " min = " << std::sqrt(sqrErrors.minCoeff()) 
              << " mean = " << std::sqrt(sqrErrors.mean())
              << " max = " << std::sqrt(sqrErrors.maxCoeff())
              << " threshold = " << currResult.getMaxReprojectionError());
    }
  }
  
  for(size_t viewID = 0; viewID < numViews; ++viewID)
  {
    LocalizationResult &currResult = vec_localizationResult[viewID];
    // skip invalid poses
    if(!currResult.isValid())
    {
      ALICEVISION_LOG_DEBUG("\n*****\nskipping invalid View " << viewID);
      continue;
    }
    
//    ALICEVISION_LOG_DEBUG("\n*****\nView " << viewID);
    // view
    std::shared_ptr<sfm::View> view = std::make_shared<sfm::View>("",viewID, intrinsicID, viewID);
    tinyScene.views.insert( std::make_pair(viewID, view));
    // pose
    tinyScene.setPose(*view, currResult.getPose());

    
    if(!allTheSameIntrinsics)
    {
      camera::PinholeRadialK3* currIntrinsics = &currResult.getIntrinsics();
       // intrinsic (the shared_ptr does not take the ownership, will not release the input pointer)
      tinyScene.intrinsics[intrinsicID] = std::shared_ptr<camera::PinholeRadialK3>(currIntrinsics, [](camera::PinholeRadialK3*){});
      ++intrinsicID;
    }
    
    // structure data (2D-3D correspondences)
    const std::vector<IndMatch3D2D> &matches = currResult.getIndMatch3D2D();
    
    for(const size_t idx : currResult.getInliers() )
    {
      // the idx should be in the size range of the data
      assert(idx < currResult.getPt3D().cols());
      const IndMatch3D2D& match = matches[idx];
//      ALICEVISION_LOG_DEBUG("inlier " << idx << " is land " << landmarkID << " and feat " << featID);
      // get the corresponding feature
      const Vec2 &feature = currResult.getPt2D().col(idx);
      // check if the point exists already
      if(tinyScene.structure.count(match.landmarkId))
      {
        sfm::Landmark& landmark = tinyScene.structure.at(match.landmarkId);
        assert(landmark.descType == match.descType);
        // normally there should be no other features already associated to this
        // 3D point in this view
        if(landmark.observations.count(viewID) != 0)
        {
          // this is weird but it could happen when two features are really close to each other (?)
          ALICEVISION_LOG_DEBUG("Point 3D " << match.landmarkId << " has multiple features "
                  << "in the same view " << viewID << ", current size of obs: " 
                  << landmark.observations.size() );
          ALICEVISION_LOG_DEBUG("its associated features are: ");
          for(std::size_t i = 0; i <  matches.size(); ++i)
          {
            auto const &p = matches[i];
            if(p.landmarkId == match.landmarkId)
            {
              assert(p.descType == match.descType);
              const Vec2 &fff = currResult.getPt2D().col(i);
              ALICEVISION_LOG_DEBUG("\tfeatID " << feature::EImageDescriberType_enumToString(p.descType) << " " << p.featId << " " << fff.transpose());
            }
          }
          continue;
        }

        // the 3D point exists already, add the observation
        landmark.observations[viewID] =  sfm::Observation(feature, match.featId);
      }
      else
      {
        // create a new 3D point
        sfm::Landmark newLandmark;
        newLandmark.descType = match.descType;
        newLandmark.X = currResult.getPt3D().col(idx);
        newLandmark.observations[viewID] = sfm::Observation(feature, match.featId);
        tinyScene.structure[match.landmarkId] = std::move(newLandmark);
      }
    }
  }

//  {
//    ALICEVISION_LOG_DEBUG("Number of 3D-2D associations before filtering " << tinyScene.structure.size());
//    sfm::Landmarks &landmarks = tinyScene.structure;
//    for(sfm::Landmarks::iterator it = landmarks.begin(), ite = landmarks.end(); it != ite;)
//    {
//      if(it->second.observations.size() < 5)
//      {
//         it = landmarks.erase(it);
//      }
//      else
//        ++it;
//    }
//  }
  
  {
    // just debugging some stats -- this block can be safely removed/commented out
    
    ALICEVISION_LOG_DEBUG("Number of 3D-2D associations " << tinyScene.structure.size());
    
    std::size_t maxObs = 0;
    for(const auto landmark : tinyScene.GetLandmarks() )
    {
      if(landmark.second.observations.size() > maxObs)
        maxObs = landmark.second.observations.size();
    }
    namespace bacc = boost::accumulators;
    bacc::accumulator_set<std::size_t, bacc::stats<bacc::tag::mean, bacc::tag::min, bacc::tag::max, bacc::tag::sum > > stats;
    std::vector<std::size_t> hist(maxObs+1, 0);
    for(const auto landmark : tinyScene.GetLandmarks() )
    {
      const std::size_t nobs = landmark.second.observations.size();
      assert(nobs < hist.size());
      stats(nobs);
      hist[nobs]++;
    }
    ALICEVISION_LOG_DEBUG("Min number of observations per point:   " << bacc::min(stats) );
    ALICEVISION_LOG_DEBUG("Mean number of observations per point:   " << bacc::mean(stats) );
    ALICEVISION_LOG_DEBUG("Max number of observations per point:   " << bacc::max(stats) );
    
    std::size_t cumulative = 0;
    const std::size_t num3DPoints = tinyScene.structure.size();
    for(std::size_t i = 0; i < hist.size(); i++ ) 
    {
      ALICEVISION_LOG_DEBUG("Points with " << i << " observations: " << hist[i] 
              << " (cumulative in %: " << 100*(num3DPoints-cumulative)/float(num3DPoints) << ")"); 
      cumulative += hist[i];
    }

    // just debugging stuff
    if(allTheSameIntrinsics)
    {
      std::vector<double> params = tinyScene.intrinsics[0].get()->getParams();
      ALICEVISION_LOG_DEBUG("K before bundle: " << params[0] << " " << params[1] << " "<< params[2]);
      if(params.size() == 6)
        ALICEVISION_LOG_DEBUG("Distortion before bundle: " << params[3] << " " << params[4] << " "<< params[5]);
    }
  }

  // filter out the 3D points having too few observations.
  if(minPointVisibility > 0)
  {
    auto &landmarks = tinyScene.structure;
    auto iter = landmarks.begin();
    auto endIter = landmarks.end();

    for(; iter != endIter;)
    {
      if(iter->second.observations.size() < minPointVisibility)
      {
        iter = landmarks.erase(iter);
      }
      else
      {
        ++iter;
      }
    }
  }
  
  if(!outputFilename.empty())
  {
    const std::string outfile = outputFilename+".BEFORE.json";
    if(!sfm::Save(tinyScene, outfile, sfm::ESfMData::ALL))
      ALICEVISION_CERR("Could not save " << outfile);
  }

  sfm::BundleAdjustmentCeres bundle_adjustment_obj;
  sfm::BA_Refine refineOptions = sfm::BA_REFINE_NONE;
  if(b_refine_pose)
    refineOptions |= sfm::BA_REFINE_ROTATION | sfm::BA_REFINE_TRANSLATION;
  if(b_refine_intrinsic)
    refineOptions |= sfm::BA_REFINE_INTRINSICS_ALL;
  if(b_refine_structure)
    refineOptions |= sfm::BA_REFINE_STRUCTURE;

  const bool b_BA_Status = bundle_adjustment_obj.Adjust(tinyScene, refineOptions);
  if(b_BA_Status)
  {
    // get back the results and update the localization result with the refined pose
    for(const auto &pose : tinyScene.GetPoses())
    {
      const IndexT idPose = pose.first;
      vec_localizationResult[idPose].setPose(pose.second);
    }

    if(!outputFilename.empty())
    {
      const std::string outfile = outputFilename+".AFTER.json";
      if(!sfm::Save(tinyScene, outfile, sfm::ESfMData::ALL))
        ALICEVISION_CERR("Could not save " << outfile);
    }
  }
  
  if(allTheSameIntrinsics)
  {
    // if we used the same intrinsics for all the localization results we need to
    // update the intrinsics of each localization result
    
    // get its optimized parameters
    std::vector<double> params = tinyScene.intrinsics[0].get()->getParams();
    ALICEVISION_LOG_DEBUG("Type of intrinsics " <<tinyScene.intrinsics[0].get()->getType());
    if(params.size() == 3)
    {
      // this means that the b_no_distortion has been passed
      // set distortion to 0
      params.push_back(0);
      params.push_back(0);
      params.push_back(0);
    }
    assert(params.size() == 6);
    ALICEVISION_LOG_DEBUG("K after bundle: " << params[0] << " " << params[1] << " "<< params[2]);
    ALICEVISION_LOG_DEBUG("Distortion after bundle " << params[3] << " " << params[4] << " "<< params[5]);

    // update the intrinsics of the each localization result
    for(size_t viewID = 0; viewID < numViews; ++viewID)
    {
      LocalizationResult &currResult = vec_localizationResult[viewID];
      if(!currResult.isValid())
      {
        continue;
      }
      currResult.updateIntrinsics(params);
      
      // just debugging -- print reprojection errors
      const Mat2X residuals = currResult.computeInliersResiduals();
      
      const auto sqrErrors = (residuals.cwiseProduct(residuals)).colwise().sum();
      ALICEVISION_LOG_DEBUG("View " << viewID << " RMSE = " << std::sqrt(sqrErrors.mean()) 
              << " min = " << std::sqrt(sqrErrors.minCoeff()) 
              << " mean = " << std::sqrt(sqrErrors.mean())
              << " max = " << std::sqrt(sqrErrors.maxCoeff())
              << " threshold = " << currResult.getMaxReprojectionError());
    }
    
  }
  
  return b_BA_Status;
}

bool refineRigPose(const std::vector<geometry::Pose3 > &vec_subPoses,
                   const std::vector<localization::LocalizationResult> vec_localizationResults,
                   geometry::Pose3 & rigPose)
{
  const std::size_t numCameras = vec_localizationResults.size();
  assert(vec_subPoses.size() == numCameras - 1);
  
  ceres::Problem problem;
  
  const aliceVision::Mat3 & R = rigPose.rotation();
  const aliceVision::Vec3 & t = rigPose.translation();

  double mainPose[6];
  ceres::RotationMatrixToAngleAxis((const double*)R.data(), mainPose);

  mainPose[3] = t(0);
  mainPose[4] = t(1);
  mainPose[5] = t(2);
  problem.AddParameterBlock(mainPose, 6);


  // Set a LossFunction to be less penalized by false measurements
  //  - set it to NULL if you don't want use a lossFunction.
  ceres::LossFunction * p_LossFunction = nullptr;//new ceres::HuberLoss(Square(4.0));
  // todo: make the LOSS function and the parameter an option

  // For all visibility add reprojections errors:
  for(std::size_t iLocalizer = 0; iLocalizer < numCameras; ++iLocalizer)
  {
    const localization::LocalizationResult & localizationResult = vec_localizationResults[iLocalizer];

    if(!localizationResult.isValid())
    {
      ALICEVISION_LOG_DEBUG("Skipping camera " << iLocalizer << " as it has not been localized");
      continue;
    }
    // Get the inliers 3D points
    const Mat & points3D = localizationResult.getPt3D();
    // Get their image locations (also referred as observations)
    const Mat & points2D = localizationResult.getPt2D();

    // Add a residual block for all inliers
    for(const IndexT iPoint : localizationResult.getInliers())
    {
      assert(iPoint < points2D.cols());
      assert(iPoint < points3D.cols());
      
      // Each Residual block takes a point and a camera as input and outputs a 2
      // dimensional residual. Internally, the cost function stores the observations
      // and the 3D point and compares the reprojection against the observation.
      ceres::CostFunction* cost_function;

      // Vector-2 residual, pose of the rig parameterized by 6 parameters
      //                  + relative pose of the secondary camera parameterized by 6 parameters
      
      geometry::Pose3 subPose;
      // if it is not the main camera (whose subpose is the identity)
      if(iLocalizer != 0)
      {
        subPose = vec_subPoses[iLocalizer - 1];
      }

      cost_function = new ceres::AutoDiffCostFunction<rig::ResidualErrorSecondaryCameraFixedRelativeFunctor, 2, 6>(
              new rig::ResidualErrorSecondaryCameraFixedRelativeFunctor(localizationResult.getIntrinsics(),
                                                                   points2D.col(iPoint),
                                                                   points3D.col(iPoint),
                                                                   subPose));

      if(cost_function)
      {
        problem.AddResidualBlock(cost_function,
                                 p_LossFunction,
                                 mainPose);
      }
      else
      {
        ALICEVISION_CERR("Fail in adding residual block for the " << iLocalizer 
                << " camera while adding point id " << iPoint);
      }
    }
  }

  // Configure a BA engine and run it
  // todo: Set the most appropriate options
  aliceVision::sfm::BundleAdjustmentCeres::BA_options aliceVision_options; // Set all
  // the options field in our owm struct - unnecessary dependancy to aliceVision here
  
  ceres::Solver::Options options;
  
  options.preconditioner_type = aliceVision_options._preconditioner_type;
  options.linear_solver_type = aliceVision_options._linear_solver_type;
  options.sparse_linear_algebra_library_type = aliceVision_options._sparse_linear_algebra_library_type;
  options.minimizer_progress_to_stdout = aliceVision_options._bVerbose;
  options.logging_type = ceres::SILENT;
  options.num_threads = 1;//aliceVision_options._nbThreads;
  options.num_linear_solver_threads = 1;//aliceVision_options._nbThreads;
  
  // Solve BA
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  
  if (aliceVision_options._bCeres_Summary)
    ALICEVISION_LOG_DEBUG(summary.FullReport());

  // If no error, get back refined parameters
  if (!summary.IsSolutionUsable())
  {
    if (aliceVision_options._bVerbose)
      ALICEVISION_CERR("Bundle Adjustment failed.");
    return false;
  }

  if(aliceVision_options._bVerbose)
  {
    // Display statistics about the minimization
    ALICEVISION_LOG_DEBUG("Bundle Adjustment statistics (approximated RMSE");
    ALICEVISION_LOG_DEBUG(" #localizers: " << vec_localizationResults.size());
    ALICEVISION_LOG_DEBUG(" #residuals: " << summary.num_residuals);
    ALICEVISION_LOG_DEBUG(" Initial RMSE: " << std::sqrt(summary.initial_cost / summary.num_residuals));
    ALICEVISION_LOG_DEBUG(" Final RMSE: " << std::sqrt(summary.final_cost / summary.num_residuals));
  }

  // update the rigPose 
  aliceVision::Mat3 R_refined;
  ceres::AngleAxisToRotationMatrix(mainPose, R_refined.data());
  aliceVision::Vec3 t_refined(mainPose[3], mainPose[4], mainPose[5]);
  // Push the optimized pose
  rigPose = geometry::Pose3(R_refined, -R_refined.transpose() * t_refined);

//  displayRelativePoseReprojection(geometry::Pose3(aliceVision::Mat3::Identity(), aliceVision::Vec3::Zero()), 0);

  // @todo do we want to update pose inside the LocalizationResults 

  return true;
}

bool refineRigPose(const std::vector<Mat> &pts2d,
                   const std::vector<Mat> &pts3d,
                   const std::vector<std::vector<std::size_t> > &inliers,
                   const std::vector<camera::PinholeRadialK3 > &vec_queryIntrinsics,
                   const std::vector<geometry::Pose3 > &vec_subPoses,
                   geometry::Pose3 &rigPose)
{
  const size_t numCameras = pts2d.size();
  assert(pts3d.size() == numCameras);
  assert(vec_queryIntrinsics.size() == numCameras);
  assert(inliers.size() == numCameras);
  assert(vec_subPoses.size() == numCameras - 1);
  
  ceres::Problem problem;
  
  const aliceVision::Mat3 & R = rigPose.rotation();
  const aliceVision::Vec3 & t = rigPose.translation();

  double mainPose[6];
  ceres::RotationMatrixToAngleAxis((const double*)R.data(), mainPose);

  mainPose[3] = t(0);
  mainPose[4] = t(1);
  mainPose[5] = t(2);
  problem.AddParameterBlock(mainPose, 6);


  // Set a LossFunction to be less penalized by false measurements
  //  - set it to NULL if you don't want use a lossFunction.
  ceres::LossFunction * p_LossFunction = nullptr;//new ceres::HuberLoss(Square(4.0));
  // todo: make the LOSS function and the parameter an option

  // For all visibility add reprojections errors:
  for(size_t cam = 0; cam < numCameras; ++cam)
  {

    // Get the inliers 3D points
    const Mat & points3D = pts3d[cam];
    assert(points3D.rows() == 3);
    // Get their image locations (also referred as observations)
    const Mat & points2D = pts2d[cam];
    assert(points2D.rows() == 2);

    if(inliers[cam].empty())
    {
      ALICEVISION_LOG_DEBUG("Skipping cam " << cam << " as it has no inliers");
      continue;
    }
    // Add a residual block for all inliers
    for(const IndexT iPoint : inliers[cam])
    {
      assert(iPoint < points2D.cols());
      assert(iPoint < points3D.cols());
      
      // Each Residual block takes a point and a camera as input and outputs a 2
      // dimensional residual. Internally, the cost function stores the observations
      // and the 3D point and compares the reprojection against the observation.
      ceres::CostFunction* cost_function;

      // Vector-2 residual, pose of the rig parameterized by 6 parameters
      //                  + relative pose of the secondary camera parameterized by 6 parameters
      
      geometry::Pose3 subPose;
      // if it is not the main camera (whose subpose is the identity)
      if(cam != 0)
      {
        subPose = vec_subPoses[cam - 1];
      }

      cost_function = new ceres::AutoDiffCostFunction<rig::ResidualErrorSecondaryCameraFixedRelativeFunctor, 2, 6>(
              new rig::ResidualErrorSecondaryCameraFixedRelativeFunctor(vec_queryIntrinsics[cam],
                                                                   points2D.col(iPoint),
                                                                   points3D.col(iPoint),
                                                                   subPose));

      if(cost_function)
      {
        problem.AddResidualBlock(cost_function,
                                 p_LossFunction,
                                 mainPose);
      }
      else
      {
        ALICEVISION_CERR("Fail in adding residual block for the " << cam 
                << " camera while adding point id " << iPoint);
      }
    }
  }

  // Configure a BA engine and run it
  // todo: Set the most appropriate options
  aliceVision::sfm::BundleAdjustmentCeres::BA_options aliceVision_options; // Set all
  // the options field in our owm struct - unnecessary dependancy to aliceVision here
  
  ceres::Solver::Options options;
  
  options.preconditioner_type = aliceVision_options._preconditioner_type;
  options.linear_solver_type = aliceVision_options._linear_solver_type;
  options.sparse_linear_algebra_library_type = aliceVision_options._sparse_linear_algebra_library_type;
  options.minimizer_progress_to_stdout = true;
  //options.logging_type = ceres::SILENT;
  options.num_threads = 1;//aliceVision_options._nbThreads;
  options.num_linear_solver_threads = 1;//aliceVision_options._nbThreads;
  
  // Solve BA
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  
  if (aliceVision_options._bCeres_Summary)
    ALICEVISION_LOG_DEBUG(summary.FullReport());

  // If no error, get back refined parameters
  if (!summary.IsSolutionUsable())
  {
    if (aliceVision_options._bVerbose)
      ALICEVISION_LOG_DEBUG("Bundle Adjustment failed.");
    return false;
  }

  if(aliceVision_options._bVerbose)
  {
    // Display statistics about the minimization
    ALICEVISION_LOG_DEBUG(
            "Bundle Adjustment statistics (approximated RMSE):\n"
            " #cameras: " << numCameras << "\n"
            " #residuals: " << summary.num_residuals << "\n"
            " Initial RMSE: " << std::sqrt(summary.initial_cost / summary.num_residuals) << "\n"
            " Final RMSE: " << std::sqrt(summary.final_cost / summary.num_residuals)
           );
  }

  // update the rigPose 
  aliceVision::Mat3 R_refined;
  ceres::AngleAxisToRotationMatrix(mainPose, R_refined.data());
  aliceVision::Vec3 t_refined(mainPose[3], mainPose[4], mainPose[5]);
  // Push the optimized pose
  rigPose = geometry::Pose3(R_refined, -R_refined.transpose() * t_refined);

//  displayRelativePoseReprojection(geometry::Pose3(aliceVision::Mat3::Identity(), aliceVision::Vec3::Zero()), 0);

  // @todo do we want to update pose inside the LocalizationResults 

  return true;
}

// rmse, min, max
std::tuple<double, double, double> computeStatistics(const Mat &pts2D, 
                                                     const Mat &pts3D,
                                                     const camera::PinholeRadialK3 &currCamera,
                                                     const std::vector<std::size_t> &currInliers,
                                                     const geometry::Pose3 &subPoses,
                                                     const geometry::Pose3 &rigPose)
{
  if(currInliers.empty())
    return std::make_tuple(0., 0., 0.);
  
  const std::size_t numPts = pts2D.cols();
  Mat2X residuals = currCamera.residuals(subPoses*rigPose, pts3D, pts2D);

  Vec sqrErrors = (residuals.cwiseProduct(residuals)).colwise().sum();

  //      ALICEVISION_LOG_DEBUG("Camera " << camID << " all reprojection errors:");
  //      ALICEVISION_LOG_DEBUG(sqrErrors);
  //
  //      ALICEVISION_LOG_DEBUG("Camera " << camID << " inliers reprojection errors:");

  double rmse = 0;
  double rmseMin = std::numeric_limits<double>::max();
  double rmseMax = 0;
  for(std::size_t j = 0; j < currInliers.size(); ++j)
  {
    // ALICEVISION_LOG_DEBUG(sqrErrors(currInliers[j]));
    const double err = sqrErrors(currInliers[j]);
    rmse += err;
    if(err > rmseMax)
      rmseMax = err;
    if(err < rmseMin)
      rmseMin = err;
  }
  
  return std::make_tuple(std::sqrt(rmse / currInliers.size()), std::sqrt(rmseMin), std::sqrt(rmseMax));
}

void printRigRMSEStats(const std::vector<Mat> &vec_pts2D,
                       const std::vector<Mat> &vec_pts3D,
                       const std::vector<camera::PinholeRadialK3 > &vec_queryIntrinsics,
                       const std::vector<geometry::Pose3 > &vec_subPoses,
                       const geometry::Pose3 &rigPose,
                       const std::vector<std::vector<std::size_t> > &vec_inliers)
{
  const std::size_t numCams = vec_pts2D.size();
  assert(numCams == vec_pts3D.size());
  assert(numCams == vec_queryIntrinsics.size());
  assert(numCams == vec_subPoses.size() + 1);
  
  // compute the reprojection error for inliers (just debugging purposes)
  double totalRMSE = 0;
  std::size_t totalInliers = 0;
  for(std::size_t camID = 0; camID < numCams; ++camID)
  { 
    if(!vec_inliers[camID].empty())
    {
      const auto& stats = computeStatistics(vec_pts2D[camID],
                                            vec_pts3D[camID],
                                            vec_queryIntrinsics[camID],
                                            vec_inliers[camID],
                                            (camID != 0 ) ? vec_subPoses[camID-1] : geometry::Pose3(),
                                            rigPose);
      ALICEVISION_LOG_DEBUG("\nCam #" << camID 
              << " RMSE inliers: " << std::get<0>(stats)
              << " min: " << std::get<1>(stats)
              << " max: " << std::get<2>(stats));        

    totalRMSE += Square(std::get<0>(stats))*vec_inliers[camID].size();
    totalInliers += vec_inliers[camID].size();
    }
  }
  ALICEVISION_LOG_DEBUG("Overall RMSE: " << std::sqrt(totalRMSE/totalInliers));
}

std::pair<double, bool> computeInliers(const std::vector<Mat> &vec_pts2d,
                                       const std::vector<Mat> &vec_pts3d,
                                       const std::vector<camera::PinholeRadialK3 > &vec_queryIntrinsics,
                                       const std::vector<geometry::Pose3 > &vec_subPoses,
                                       double maxReprojectionError,
                                       const geometry::Pose3 &rigPose,
                                       std::vector<std::vector<std::size_t> > &vec_inliers)
{
  const std::size_t numCams = vec_pts2d.size();
  assert(numCams == vec_pts3d.size());
  assert(numCams == vec_queryIntrinsics.size());
  assert(numCams == vec_subPoses.size() + 1);
  
  const double squareThreshold = Square(maxReprojectionError);
  
  double rmse = 0;
  std::size_t numInliers = 0;
  // number of inlier removed
  std::size_t numAdded = 0;
  // number of point that were not inliers and now they are
  std::size_t numRemoved = 0;
  
  std::vector<std::vector<std::size_t> > vec_newInliers(numCams);

  // compute the reprojection error for inliers (just debugging purposes)
  for(std::size_t camID = 0; camID < numCams; ++camID)
  {
    const std::size_t numPts = vec_pts2d[camID].cols();
    
    const camera::PinholeRadialK3 &currCamera = vec_queryIntrinsics[camID];
    
    Mat2X residuals;
    if(camID != 0)
      residuals = currCamera.residuals(vec_subPoses[camID - 1] * rigPose, vec_pts3d[camID], vec_pts2d[camID]);
    else
      residuals = currCamera.residuals(geometry::Pose3() * rigPose, vec_pts3d[camID], vec_pts2d[camID]);

    Vec sqrErrors = (residuals.cwiseProduct(residuals)).colwise().sum();

    auto &currInliers = vec_newInliers[camID];
    const auto &oldInliers = vec_inliers[camID];
    currInliers.reserve(numPts);
    
    for(std::size_t i = 0; i < numPts; ++i)
    {
      // check whether the current point was an inlier
      const auto occ = std::count(oldInliers.begin(), oldInliers.end(), i);
      assert(occ == 0 || occ == 1);
      
      if(sqrErrors(i) < squareThreshold)
      {
        currInliers.push_back(i);
        
        // if it was not an inlier mark it as added one
        if(occ == 0)
          ++numAdded;
        
        rmse += sqrErrors(i);
        ++numInliers;
      }
      else
      {
         // if it was an inlier mark it as removed one
         if(occ == 1)
          ++numRemoved;       
      }
    }
  }
  ALICEVISION_LOG_DEBUG("Removed " << numRemoved << " inliers, added new " << numAdded << " point");
  
  // swap
  vec_inliers.swap(vec_newInliers);
  return std::make_pair(std::sqrt(rmse/numInliers), (numRemoved > 0 || numAdded > 0) );
}


bool iterativeRefineRigPose(const std::vector<Mat> &pts2d,
                            const std::vector<Mat> &pts3d,
                            const std::vector<camera::PinholeRadialK3 > &vec_queryIntrinsics,
                            const std::vector<geometry::Pose3 > &vec_subPoses,
                            double maxReprojectionError,
                            std::size_t minNumPoints,
                            std::vector<std::vector<std::size_t> > &vec_inliers,
                            geometry::Pose3 &rigPose,
                            std::size_t maxIterationNumber)

{
  geometry::Pose3 optimalPose = rigPose;
  std::size_t iterationNumber = 1;
  bool hasChanged = false;
  
  do
  {
    ALICEVISION_LOG_DEBUG("[poseEstimation]\tIteration " << iterationNumber);
    const bool refineOk = refineRigPose(pts2d,
                                        pts3d,
                                        vec_inliers,
                                        vec_queryIntrinsics,
                                        vec_subPoses,
                                        optimalPose);
    if(!refineOk)
    {
      ALICEVISION_LOG_DEBUG("[poseEstimation]\tIterative refine rig pose failed");
      return false;
    }
    
    // recompute the inliers and the RMSE
    const auto& result = computeInliers(pts2d,
                                        pts3d,
                                        vec_queryIntrinsics,
                                        vec_subPoses,
                                        maxReprojectionError,
                                        optimalPose,
                                        vec_inliers);
    // if there have been changes in the inlier sets
    hasChanged = result.second;
    
    // if not enough inliers stop?
    std::size_t numInliers = 0;
    for(const auto& inliers : vec_inliers)
      numInliers += inliers.size();
    
    if(numInliers <= minNumPoints)
    {
      ALICEVISION_LOG_DEBUG("[poseEstimation]\tIterative refine rig pose has reached the minimum number of points");
      return false;
    }
    
    ++iterationNumber;
    if(iterationNumber > maxIterationNumber)
      ALICEVISION_LOG_DEBUG("Terminating refine because the max number of iterations has been reached");
  }
  while(hasChanged && iterationNumber <= maxIterationNumber);

  rigPose = optimalPose;

  return true;
}


} //namespace localization
} //namespace aliceVision

