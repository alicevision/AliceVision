// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/numeric/projection.hpp>
#include <aliceVision/robustEstimation/LORansac.hpp>
#include <aliceVision/robustEstimation/conditioning.hpp>
#include <aliceVision/robustEstimation/ScoreEvaluator.hpp>
#include <aliceVision/robustEstimation/randSampling.hpp>
#include <aliceVision/multiview/resection/ResectionKernel.hpp>
#include <aliceVision/multiview/resection/P3PSolver.hpp>
#include <aliceVision/multiview/ResectionKernel.hpp>
#include <aliceVision/multiview/Unnormalizer.hpp>
#include <aliceVision/camera/camera.hpp>
#include <aliceVision/sfm/sfm.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/geometry/Pose3.hpp>
#include <aliceVision/numeric/numeric.hpp>

#include <vector>
#include <random>
#include <algorithm>

#define BOOST_TEST_MODULE ResectionLORansac

#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>

using namespace aliceVision;

bool refinePoseAsItShouldbe(const Mat & pt3D,
                            const Mat & pt2D,
                            const std::vector<std::size_t> & vec_inliers,
                            camera::IntrinsicBase * intrinsics,
                            geometry::Pose3& pose,
                            bool b_refine_pose,
                            bool b_refine_intrinsic)
{
  using namespace sfm;
  using namespace sfmData;

  // Setup a tiny SfM scene with the corresponding 2D-3D data
  SfMData sfm_data;
  // view
  std::shared_ptr<View> view = std::make_shared<View>("", 0, 0, 0);
  sfm_data.views.emplace(0, view);
  // pose
  sfm_data.setPose(*view, CameraPose(pose));
  // intrinsic (the shared_ptr does not take the ownership, will not release the input pointer)
  sfm_data.intrinsics[0] = std::shared_ptr<camera::IntrinsicBase>(intrinsics, [](camera::IntrinsicBase*)
  {
  });
  // structure data (2D-3D correspondences)
  const double unknownScale = 0.0;
  for(size_t i = 0; i < vec_inliers.size(); ++i)
  {
    const size_t idx = vec_inliers[i];
    Landmark landmark;
    landmark.X = pt3D.col(idx);
    landmark.observations[0] = Observation(pt2D.col(idx), UndefinedIndexT, unknownScale);
    sfm_data.structure[i] = std::move(landmark);
  }

  BundleAdjustmentCeres bundle_adjustment_obj;
  BundleAdjustment::ERefineOptions refineOptions = BundleAdjustment::REFINE_NONE;
  if(b_refine_pose)
    refineOptions |= sfm::BundleAdjustment::REFINE_ROTATION | sfm::BundleAdjustment::REFINE_TRANSLATION;
  if(b_refine_intrinsic)
    refineOptions |= sfm::BundleAdjustment::REFINE_INTRINSICS_ALL;
  const bool b_BA_Status = bundle_adjustment_obj.adjust(sfm_data, refineOptions);
  if(b_BA_Status)
  {
    pose = sfm_data.getPose(*view).getTransform();
  }
  return b_BA_Status;
}

// test LORansac repetability over the same test case
BOOST_AUTO_TEST_CASE(P3P_Ransac_noisyFromImagePoints)
{
  std::mt19937 randomNumberGenerator;

  // camera and image parameters
  const std::size_t WIDTH = 1600;
  const std::size_t HEIGHT = 1200;
  const std::size_t FOCAL = 2000;
  const std::size_t PPX = WIDTH / 2;
  const std::size_t PPY = HEIGHT / 2;
  
  // simulation parameters
  // number of trials to run
  const std::size_t NUMTRIALS = 100;
  // create outliers
  const bool withOutliers = true;
  const double OUTLIERSRATIO = .15;
  assert(OUTLIERSRATIO <= 1.0 && OUTLIERSRATIO >= .0);
  // parameters for generating 3D points
  // 3D points are generating selecting a point on the image and then picking a
  // points on its associated projection ray with a distance in [MINDIST, MAXDIST]
  const double MINDIST = 15;
  const double MAXDIST = 35;
  assert(MINDIST <= MAXDIST);
  // number of points to generate
  const std::size_t nbPoints = 50;
  // noise level in pixels
  const double gaussianNoiseLevel = 4.0;
  // tolerance errors for test to pass
  const double maxAngularError = 0.1;
  const double maxBaselineError = 0.01;
  
  std::vector<std::size_t> vec_outliers;

  std::mt19937 gen;
  std::uniform_real_distribution<double> realDist(0, 1.0);

  // generate a random RT transform to apply to the 3D points
  const Mat3 Rgt = rotationXYZ(M_PI_2 * realDist(gen), M_PI_2 * realDist(gen), M_PI_2 * realDist(gen));
  const Vec3 Tgt = MINDIST * Vec3(realDist(gen), realDist(gen), realDist(gen));
  Mat3 Kgt;
  Kgt << FOCAL, 0, PPX,
          0, FOCAL, PPY,
          0, 0, 1;
  const Vec3 Cgt = -Rgt.transpose() * Tgt;

  // draw some random points in the camera image
  Mat pts2D = Mat(2, nbPoints);
  Mat pts3D = Mat(3, nbPoints);
  Mat pts3DGt = Mat(3, nbPoints);

  for(std::size_t i = 0; i < nbPoints; ++i)
  {
    // for each point add a 3D point lying on the projection ray and having a 
    // distance [MINDIST, MAXDIST]
    pts2D.col(i) = Vec2(WIDTH * realDist(gen), HEIGHT * realDist(gen));
    Vec3 direction;
    direction << ((pts2D.col(i) - Vec2(PPX, PPY)) / FOCAL), 1;
    direction /= direction.norm();
    direction *= (MINDIST + (MAXDIST - MINDIST) * realDist(gen));
    pts3DGt.col(i) = direction;
    // multiply by the inverse of the pose
    pts3D.col(i) = Rgt.transpose() * direction + -Rgt.transpose() * Tgt;
  }

  // add some gaussian noise to the 2d points
  for(std::size_t i = 0; i < nbPoints; ++i)
  {
    const double theta = realDist(gen)*2 * M_PI;
    const double radius = gaussianNoiseLevel * realDist(gen);
    pts2D.col(i) += radius * Vec2(std::cos(theta), std::sin(theta));
  }
  
  if(withOutliers)
  {
    // take a random sample to be used as outliers
    const std::size_t NUMOUTLIERS = std::size_t(OUTLIERSRATIO*nbPoints);
    vec_outliers.resize(NUMOUTLIERS);
    std::iota(vec_outliers.begin(), vec_outliers.end(), 0);
    std::sort(vec_outliers.begin(), vec_outliers.end());
    
    // add outliers
    for(const auto &idx : vec_outliers)
    {
      std::size_t iter = 0;
      Vec2 pt = Vec2(WIDTH * realDist(gen), HEIGHT * realDist(gen));
      while( (pt-pts2D.col(idx)).norm() < 15*gaussianNoiseLevel)
      {
        pt = Vec2(WIDTH * realDist(gen), HEIGHT * realDist(gen));
        ++iter;
        // safeguard against infinite loops
        assert(iter > 1000 && "Unable to generate a random point, iterations excedeed!");
      }
      pts2D.col(idx) = pt;
    }
  }

  for(std::size_t trial = 0; trial < NUMTRIALS; ++trial)
  {
    ALICEVISION_LOG_DEBUG("Trial #" << trial);
    typedef multiview::resection::P3PSolver SolverType;
    typedef multiview::resection::Resection6PSolver SolverLSType;
  
    typedef aliceVision::multiview::ResectionKernel_K<SolverType,
                                                      multiview::resection::ProjectionDistanceSquaredError,
                                                      multiview::UnnormalizerResection,
                                                      robustEstimation::Mat34Model,
                                                      SolverLSType> KernelType;

    // this is just to simplify and use image plane coordinates instead of camera
    // (pixel) coordinates
    Mat pts2Dnorm;
    robustEstimation::applyTransformationToPoints(pts2D, Kgt.inverse(), &pts2Dnorm);
    KernelType kernel(pts2Dnorm, pts3D, Mat3::Identity());

    std::vector<std::size_t> inliers;
    const double threshold = 2*gaussianNoiseLevel;
    const double normalizedThreshold = Square(threshold / FOCAL);
    robustEstimation::ScoreEvaluator<KernelType> scorer(normalizedThreshold);
    robustEstimation::Mat34Model model = robustEstimation::LO_RANSAC(kernel, scorer, randomNumberGenerator, &inliers);
    Mat34 Pest = model.getMatrix();
    const std::size_t numInliersFound = inliers.size();
    const std::size_t numInliersExpected = nbPoints-vec_outliers.size();
    
    BOOST_CHECK(numInliersFound > kernel.getMinimumNbRequiredSamples()  *2.5);
    
    Mat3 Rest;
    Mat3 Kest;
    Vec3 Test;
    KRt_from_P(Pest, &Kest, &Rest, &Test);

    ALICEVISION_LOG_DEBUG("Est: Pest:\n" << Pest
            << "\nRest:\n" << Rest
            << "\nKest:\n" << Kest
            << "\ntest:\n" << Test);
    
    ALICEVISION_LOG_DEBUG("Solution found with " << numInliersFound << " inliers");
    ALICEVISION_LOG_DEBUG("Expected number of inliers " << numInliersExpected);

    BOOST_CHECK_EQUAL(numInliersFound, numInliersExpected);
    
    const double angError = radianToDegree(getRotationMagnitude(Rgt * Rest.transpose()));
    ALICEVISION_LOG_DEBUG("Angular error: " << angError);
    ALICEVISION_LOG_DEBUG("Baseline error: " << (Test - Tgt).squaredNorm());

    geometry::Pose3 pose = geometry::poseFromRT(Rest, Test);
    refinePoseAsItShouldbe(pts3D,
                           pts2Dnorm,
                           inliers,
                           new camera::Pinhole(0, 0, 1, 1, 0, 0),
                           pose,
                           true,
                           false );

    const double angErrorRef = radianToDegree(getRotationMagnitude(Rgt * pose.rotation().transpose()));
    const double baselineErrorRef = (Tgt - pose.translation()).squaredNorm();
    ALICEVISION_LOG_DEBUG("Final angular error #"<<trial<<" : " << angErrorRef);
    ALICEVISION_LOG_DEBUG("Final baseline error #"<<trial<<" : " << baselineErrorRef);
    
    BOOST_CHECK_SMALL(angErrorRef, maxAngularError);
    BOOST_CHECK_SMALL(baselineErrorRef, maxBaselineError);

    ALICEVISION_LOG_DEBUG("Refined pose:\n"
                << "\nEst: Rest:\n" << pose.rotation()
                << "\nCest:\n" << pose.center()
                << "\nTest:\n" << pose.translation());
    
    if(withOutliers)
    {
      // test if inliers found and outliers GT have a empty intersection
      std::vector<std::size_t> inters(nbPoints);
      std::sort(inliers.begin(), inliers.end());
      auto it = std::set_intersection(inliers.begin(), inliers.end(),
                                      vec_outliers.begin(), vec_outliers.end(),
                                      inters.begin());
      inters.resize(it-inters.begin());
      if(inters.size() > 0)
        ALICEVISION_LOG_WARNING("******* there are " << inters.size() << " outliers considered as inliers");
      BOOST_CHECK_EQUAL(inters.size(), 0);
    }
  }
}
