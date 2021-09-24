// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/feature/imageDescriberCommon.hpp>
#include <aliceVision/sfm/utils/statistics.hpp>
#include <aliceVision/sfm/sfm.hpp>
#include <aliceVision/sfm/pipeline/panorama/ReconstructionEngine_panorama.hpp>

#include <cmath>
#include <cstdio>
#include <iostream>

#define BOOST_TEST_MODULE PANORAMA_SFM

#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>
#include <aliceVision/unitTest.hpp>

using namespace aliceVision;

// Test summary:
// - Create a rotation matrix between two views
// - Create two matrices of calibration for two views

void test_panorama(std::shared_ptr<camera::IntrinsicBase> & intrinsic_gt, std::shared_ptr<camera::IntrinsicBase> & intrinsic_noisy, double ratio_inliers) {

  sfmData::SfMData sfmdata;
  sfmdata.getIntrinsics()[0] = intrinsic_noisy;

  /*Create cameras */
  std::vector<geometry::Pose3> poses_gt;
  size_t count = 0;
  for (double latitude = - M_PI_2; latitude <= M_PI_2; latitude += 0.8) {
    for (double longitude = - M_PI; longitude <= M_PI; longitude += 0.5) {
      
      Eigen::Matrix3d matLongitude = Eigen::AngleAxisd(longitude, Vec3(0,1,0)).toRotationMatrix();
      Eigen::Matrix3d matLatitude = Eigen::AngleAxisd(latitude, Vec3(1,0,0)).toRotationMatrix();
      Eigen::Matrix3d R = matLongitude * matLatitude;
      geometry::Pose3 pose(R, Vec3(0,0,0));
      poses_gt.push_back(pose);
      
      std::shared_ptr<sfmData::View> v = std::make_shared<sfmData::View>("fakeimg.png", count, 0, count, 1920, 1080);
      sfmdata.getViews()[count] = v;
      count++;
    }
  }


  feature::FeaturesPerView fpv;
  matching::PairwiseMatches matches;

  std::mt19937 mt;
  std::uniform_real_distribution<double> dist(0.0, 1.0);
  std::normal_distribution<double> noise(0.0, 100.0);
  
  for (double latitude = - M_PI_2; latitude <= M_PI_2; latitude += 0.02) {
    for (double longitude = - M_PI; longitude <= M_PI; longitude += 0.02) {
      
      const double Px = cos(latitude) * sin(longitude);
      const double Py = sin(latitude);
      const double Pz = cos(latitude) * cos(longitude);

      Vec3 pt(Px, Py, Pz);

      std::vector<std::pair<size_t, size_t>> observations;

      /*Project this point on all cameras*/
      for (int idPose = 0; idPose < poses_gt.size(); idPose++) {

        geometry::Pose3 pose = poses_gt[idPose];

        Vec3 ray = pose(pt);
        if (!intrinsic_gt->isVisibleRay(ray)) {
          continue;
        }

        Vec2 im = intrinsic_gt->project(pose, pt.homogeneous(), true);
        if (!intrinsic_gt->isVisible(im)) {
          continue;
        }

        /*If it is visible, store a fake feature */
        feature::MapFeaturesPerView & map = fpv.getData();
        feature::PointFeatures & pfs = map[idPose][feature::EImageDescriberType::UNKNOWN];
        
        /*Also store the index of this feature for this view*/
        observations.push_back(std::make_pair(idPose, pfs.size()));

        double dice = dist(mt);
        if (dice < ratio_inliers) {
          //Outlier: generate large error
          im(0) += noise(mt);
          im(1) += noise(mt);
        }

        feature::PointFeature pf(im(0), im(1), 1.0, 0.0);
        pfs.push_back(pf);
      }

      for (size_t idObs1 = 0; idObs1 < observations.size(); idObs1++) {
        for (size_t idObs2 = idObs1 + 1; idObs2 < observations.size(); idObs2++) {

          size_t idPose1 = observations[idObs1].first;
          size_t idPose2 = observations[idObs2].first;

          Pair p = std::make_pair(idPose1, idPose2);
          
          matching::IndMatch match(observations[idObs1].second, observations[idObs2].second);

          matching::IndMatches & vec_matches = matches[p][feature::EImageDescriberType::UNKNOWN];
          vec_matches.push_back(match);
        }
      }
    }
  } 
  sfm::ReconstructionEngine_panorama::Params params;
  params.eRelativeRotationMethod = sfm::RELATIVE_ROTATION_FROM_R;

  sfm::ReconstructionEngine_panorama pano(sfmdata, params, "");
  pano.SetFeaturesProvider(&fpv);
  pano.SetMatchesProvider(&matches);

  if (!pano.process()) {
    BOOST_TEST_FAIL("Panorama processing failed");
    return;
  }

  if (!pano.Adjust()) {
    BOOST_TEST_FAIL("Panorama adjustment failed");
    return;
  }

  sfmdata = pano.getSfMData();

  geometry::Pose3 zRw_gt = poses_gt[0];
  geometry::Pose3 zRw_est = sfmdata.getPoses()[0].getTransform();

  for (int idPose = 0; idPose < poses_gt.size(); idPose++) {
    geometry::Pose3 cRw_gt = poses_gt[idPose];
    geometry::Pose3 cRw_est = sfmdata.getPoses()[idPose].getTransform();

    Eigen::Matrix3d cRz_gt = cRw_gt.rotation() * zRw_gt.rotation().transpose();
    Eigen::Matrix3d cRz_est = cRw_est.rotation() * zRw_est.rotation().transpose();

    Eigen::Matrix3d delta = cRz_gt * cRz_est.transpose();

    Eigen::AngleAxisd aa;
    aa.fromRotationMatrix(delta);
    BOOST_CHECK_LT(aa.angle(), 1e-2);
  }
}


BOOST_AUTO_TEST_CASE(PANORAMA_SFM_RADIAL3)
{
  std::shared_ptr<camera::IntrinsicBase> intrinsic_gt = camera::createIntrinsic(camera::EINTRINSIC::PINHOLE_CAMERA_RADIAL3, 1920, 1080, 1357.0, 1357.0, 0, 0);
  std::shared_ptr<camera::IntrinsicBase> intrinsic_est = camera::createIntrinsic(camera::EINTRINSIC::PINHOLE_CAMERA_RADIAL3, 1920, 1080, 1200.0, 1200.0, 40, -20);

  test_panorama(intrinsic_gt, intrinsic_est, 0.0);
}

BOOST_AUTO_TEST_CASE(PANORAMA_SFM_RADIAL3_OUTLIERS)
{
  std::shared_ptr<camera::IntrinsicBase> intrinsic_gt = camera::createIntrinsic(camera::EINTRINSIC::PINHOLE_CAMERA_RADIAL3, 1920, 1080, 1357.0, 1357.0, 0, 0);
  std::shared_ptr<camera::IntrinsicBase> intrinsic_est = camera::createIntrinsic(camera::EINTRINSIC::PINHOLE_CAMERA_RADIAL3, 1920, 1080, 1000.0, 1000.0, 40, -20);

  test_panorama(intrinsic_gt, intrinsic_est, 0.3);
}


BOOST_AUTO_TEST_CASE(PANORAMA_SFM_EQUIDISTANT)
{
  std::shared_ptr<camera::IntrinsicBase> intrinsic_gt = camera::createIntrinsic(camera::EINTRINSIC::EQUIDISTANT_CAMERA_RADIAL3, 1920, 1080, 1357.0, 1357.0, 0, 0);
  std::shared_ptr<camera::IntrinsicBase> intrinsic_est = camera::createIntrinsic(camera::EINTRINSIC::EQUIDISTANT_CAMERA_RADIAL3, 1920, 1080, 1200.0, 1200.0, 40, -20);

  test_panorama(intrinsic_gt, intrinsic_est, 0.0);
}

BOOST_AUTO_TEST_CASE(PANORAMA_SFM_EQUIDISTANT_OUTLIERS)
{
  std::shared_ptr<camera::IntrinsicBase> intrinsic_gt = camera::createIntrinsic(camera::EINTRINSIC::EQUIDISTANT_CAMERA_RADIAL3, 1920, 1080, 1357.0, 1357.0, 0, 0);
  std::shared_ptr<camera::IntrinsicBase> intrinsic_est = camera::createIntrinsic(camera::EINTRINSIC::EQUIDISTANT_CAMERA_RADIAL3, 1920, 1080, 1000.0, 1000.0, 40, -20);

  test_panorama(intrinsic_gt, intrinsic_est, 0.3);
}
