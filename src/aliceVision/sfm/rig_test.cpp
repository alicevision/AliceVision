// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "SfMData.hpp"

#include <memory>

#define BOOST_TEST_MODULE sfmRig
#include <boost/test/included/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>

using namespace aliceVision;
using namespace aliceVision::sfm;

BOOST_AUTO_TEST_CASE(rig_initialization)
{
  static constexpr IndexT rigId = 0;
  static constexpr std::size_t nbSubPoses = 5;

  SfMData sfmData;

  sfmData.intrinsics.emplace(0, std::make_shared<camera::Pinhole>(1000, 1000, 36.0, std::rand()%10000, std::rand()%10000));
  sfmData.getRigs().emplace(rigId, Rig(nbSubPoses));

  std::vector<View> rigViews;

  for(std::size_t subPoseId = 0; subPoseId < nbSubPoses; ++subPoseId)
  {
    rigViews.emplace_back("", subPoseId, 0, 0, 10, 10, rigId, subPoseId);
  }

  for(const View& view : rigViews)
  {
    sfmData.views.emplace(view.getViewId(), std::make_shared<View>(view));
  }

  const Mat3 r = Mat3::Random();
  const Vec3 c = Vec3::Random();
  const geometry::Pose3 firstPose(r, c);
  const IndexT firstPoseId = std::rand() % nbSubPoses;

  const Rig& rig = sfmData.getRig(rigViews.front());

  // rig uninitialized
  BOOST_CHECK(!rig.isInitialized());

  sfmData.setPose(rigViews.at(firstPoseId), CameraPose(firstPose));

  // setPose done, rig initialized
  BOOST_CHECK(rig.isInitialized());

  // Check rig sub-poses
  for(std::size_t subPoseId = 0; subPoseId < nbSubPoses; ++subPoseId)
  {
    const RigSubPose& subPose = rig.getSubPose(subPoseId);

    if(subPoseId == firstPoseId)
    {
      // first sub-pose should be initialized
      BOOST_CHECK(subPose.status != ERigSubPoseStatus::UNINITIALIZED);
    }
    else
    {
      // other sub-poses are uninitialized
      BOOST_CHECK(subPose.status == ERigSubPoseStatus::UNINITIALIZED);
    }

    // all sub-poses should be at identity
    BOOST_CHECK(subPose.pose == geometry::Pose3());
  }

  // Check rig pose
  for(std::size_t subPoseId = 0; subPoseId < nbSubPoses; ++subPoseId)
  {
    const View& view = rigViews.at(subPoseId);
    BOOST_CHECK(sfmData.getPose(view).getTransform() == firstPose);
  }
}

BOOST_AUTO_TEST_CASE(rig_setPose)
{
  static constexpr IndexT rigId = 0;
  static constexpr std::size_t nbSubPoses = 5;
  static constexpr std::size_t nbPoses = 2;

  SfMData sfmData;

  sfmData.intrinsics.emplace(0, std::make_shared<camera::Pinhole>(1000, 1000, 36.0, std::rand()%10000, std::rand()%10000));
  sfmData.getRigs().emplace(rigId, Rig(nbSubPoses));

  std::vector<View> rigViews;

  for(std::size_t poseId = 0; poseId < nbPoses; ++poseId)
  {
    for(std::size_t subPoseId = 0; subPoseId < nbSubPoses; ++subPoseId)
    {
      const IndexT viewId = poseId * nbSubPoses + subPoseId;
      rigViews.emplace_back("", viewId, 0, poseId, 10, 10, rigId, subPoseId);
    }
  }

  for(const View& view : rigViews)
  {
    sfmData.views.emplace(view.getViewId(), std::make_shared<View>(view));
  }

  const Rig& rig = sfmData.getRig(rigViews.front());

  for(std::size_t poseId = 0; poseId < nbPoses; ++poseId)
  {
    for(std::size_t subPoseId = 0; subPoseId < nbSubPoses; ++subPoseId)
    {
      const IndexT viewId = poseId * nbSubPoses + subPoseId;
      const View& view = *(sfmData.views.at(viewId));
      const RigSubPose& subPose = rig.getSubPose(subPoseId);

      if(subPoseId == 0)
      {
        // first sub-pose, rig pose is unknown
        BOOST_CHECK(!sfmData.existsPose(view));

        if(poseId == 0)
        {
          // first rig pose, first sub-pose, sub-pose uninitialized
          BOOST_CHECK(subPose.status == ERigSubPoseStatus::UNINITIALIZED);
        }
        else
        {
          // not first rig pose, first sub-pose, sub-pose initialized
          BOOST_CHECK(subPose.status != ERigSubPoseStatus::UNINITIALIZED);
        }

        const Mat3 r = Mat3::Random();
        const Vec3 c = Vec3::Random();
        const geometry::Pose3 firstPose = geometry::Pose3(r, c);

        sfmData.setPose(view, CameraPose(firstPose));

        // setPose done, sub-pose must be initialized
        BOOST_CHECK(subPose.status != ERigSubPoseStatus::UNINITIALIZED);

        // setPose done, rig initialized
        BOOST_CHECK(sfmData.existsPose(view));
        BOOST_CHECK(sfmData.getPose(view).getTransform() == firstPose);
      }
      else
      {

        // rig pose should be initialized
        BOOST_CHECK(sfmData.existsPose(view));

        if(poseId == 0) //other poses are redundant
        {
          const Mat3 r = Mat3::Random();
          const Vec3 c = Vec3::Random();
          const geometry::Pose3 pose = geometry::Pose3(r, c);

          // first rig pose, sub-pose must be uninitialized
          BOOST_CHECK(subPose.status == ERigSubPoseStatus::UNINITIALIZED);

          sfmData.setPose(view, CameraPose(pose));

          // setPose done, sub-pose must be initialized
          BOOST_CHECK(subPose.status != ERigSubPoseStatus::UNINITIALIZED);
        }
      }
    }
  }
}

BOOST_AUTO_TEST_CASE(rig_getPose)
{
  static constexpr IndexT rigId = 0;
  static constexpr std::size_t nbSubPoses = 5;
  static constexpr std::size_t nbPoses = 2;

  SfMData sfmData;

  sfmData.intrinsics.emplace(0, std::make_shared<camera::Pinhole>(1000, 1000, 36.0, std::rand()%10000, std::rand()%10000));
  sfmData.getRigs().emplace(rigId, Rig(nbSubPoses));

  std::vector<View> rigViews;

  for(std::size_t poseId = 0; poseId < nbPoses; ++poseId)
  {
    for(std::size_t subPoseId = 0; subPoseId < nbSubPoses; ++subPoseId)
    {
      const IndexT viewId = poseId * nbSubPoses + subPoseId;
      rigViews.emplace_back("", viewId, 0, poseId, 10, 10, rigId, subPoseId);
    }
  }

  for(const View& view : rigViews)
  {
    sfmData.views.emplace(view.getViewId(), std::make_shared<View>(view));
  }

  const Rig& rig = sfmData.getRig(rigViews.front());

  for(std::size_t poseId = 0; poseId < nbPoses; ++poseId)
  {
    for(std::size_t subPoseId = 0; subPoseId < nbSubPoses; ++subPoseId)
    {
      const IndexT viewId = poseId * nbSubPoses + subPoseId;
      const View& view = *(sfmData.views.at(viewId));
      const RigSubPose& subPose = rig.getSubPose(subPoseId);

      if(subPoseId == 0)
      {
        const Mat3 r = Mat3::Random();
        const Vec3 c = Vec3::Random();
        const geometry::Pose3 firstPose = geometry::Pose3(r, c);

        sfmData.setPose(view, CameraPose(firstPose));

        // setPose done, rig initialized
        if(poseId == 0)
        {
          // the rig pose is the first pose
          BOOST_CHECK(sfmData.getPose(view).getTransform() == firstPose);
        }
        else
        {
          // the rig pose is the sub-pose inverse multiply by the rig pose
          BOOST_CHECK(sfmData.getPose(view).getTransform() == (subPose.pose.inverse() * firstPose));
        }

      }
      else
      {
        const geometry::Pose3& rigPose = sfmData.getPose(view).getTransform();

        if(poseId == 0) //other poses are redundant
        {
          const Mat3 r = Mat3::Random();
          const Vec3 c = Vec3::Random();
          const geometry::Pose3 absolutePose = geometry::Pose3(r, c);

          sfmData.setPose(view, CameraPose(absolutePose));

          // the view sub-pose is the absolute pose multiply by the rig pose inverse
          BOOST_CHECK(subPose.pose == (absolutePose * rigPose.inverse()));
        }

        // the view absolute pose is the sub-pose multiply by the rig pose
        BOOST_CHECK(sfmData.getPose(view).getTransform() == (subPose.pose * sfmData.getAbsolutePose(view.getPoseId()).getTransform()));
      }
    }
  }
}
