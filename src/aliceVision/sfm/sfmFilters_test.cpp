// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfm/sfmFilters.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/camera/Pinhole.hpp>

#define BOOST_TEST_MODULE sfmFilters
#include <boost/test/unit_test.hpp>

using namespace aliceVision;
using namespace aliceVision::sfmData;

BOOST_AUTO_TEST_CASE(eraseUnstablePoses_detachedRigViewsGetUnusedPoseIds)
{
    SfMData scene;
    scene.getIntrinsics().emplace(0, std::make_shared<camera::Pinhole>(100, 100, 100.0, 100.0, 0.0, 0.0));

    // Views 0 and 1 share the rig pose 100, which is not observed and will be removed
    const IndexT rigPoseId = 100;
    scene.getViews().emplace(0, std::make_shared<View>("", 0, 0, rigPoseId, 100, 100, 0, 0));
    scene.getViews().emplace(1, std::make_shared<View>("", 1, 0, rigPoseId, 100, 100, 0, 1));
    scene.getPoses().assign(rigPoseId, CameraPose(geometry::Pose3()));

    // View 2 has the pose id 0 (same value as the view id of a rig view) and survives
    scene.getViews().emplace(2, std::make_shared<View>("", 2, 0, 0, 100, 100));
    scene.getPoses().assign(0, CameraPose(geometry::Pose3()));

    Landmark landmark(Vec3(0.0, 0.0, 1.0));
    landmark.getObservations().emplace(2, Observation(Vec2::Zero(), 0, 1.0));
    scene.getLandmarks()[0] = landmark;

    std::set<IndexT> removedViewsId;
    BOOST_CHECK(sfm::eraseUnstablePoses(scene, 1, &removedViewsId));

    BOOST_CHECK(removedViewsId == std::set<IndexT>({0, 1}));
    BOOST_CHECK(scene.getPoses().size() == 1);
    BOOST_CHECK(scene.getPoses().count(0) == 1);

    const IndexT poseId0 = scene.getView(0).getPoseId();
    const IndexT poseId1 = scene.getView(1).getPoseId();
    BOOST_CHECK(poseId0 != UndefinedIndexT);
    BOOST_CHECK(poseId1 != UndefinedIndexT);
    BOOST_CHECK(poseId0 != poseId1);
    BOOST_CHECK(poseId0 != 0);
    BOOST_CHECK(poseId1 != 0);
    BOOST_CHECK(!scene.isPoseAndIntrinsicDefined(IndexT(0)));
    BOOST_CHECK(!scene.isPoseAndIntrinsicDefined(IndexT(1)));
    BOOST_CHECK(scene.getView(0).isPoseIndependant());
    BOOST_CHECK(scene.getView(1).isPoseIndependant());
}
