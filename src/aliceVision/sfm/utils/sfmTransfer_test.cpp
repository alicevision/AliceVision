// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfm/utils/sfmTransfer.hpp>
#include <aliceVision/camera/Pinhole.hpp>

#define BOOST_TEST_MODULE sfmTransfer
#include <boost/test/unit_test.hpp>

using namespace aliceVision;
using namespace aliceVision::sfmData;

namespace {

const IndexT rigId = 0;

geometry::Pose3 makePose(double angle, const Vec3& center)
{
    return geometry::Pose3(Eigen::AngleAxisd(angle, Vec3::UnitZ()).toRotationMatrix(), center);
}

void addIntrinsic(SfMData& scene, IndexT intrinsicId, double focal)
{
    scene.getIntrinsics().emplace(intrinsicId, std::make_shared<camera::Pinhole>(100, 100, focal, focal, 0.0, 0.0));
}

void addPose(SfMData& scene, IndexT poseId, const geometry::Pose3& pose) { scene.getPoses().assign(poseId, CameraPose(pose)); }

// View which is not part of a rig, with its own pose ID
View& addView(SfMData& scene, IndexT viewId, IndexT intrinsicId, IndexT poseId)
{
    scene.getViews().emplace(viewId, std::make_shared<View>("", viewId, intrinsicId, poseId, 100, 100));
    return scene.getView(viewId);
}

// View part of the rig, either rig-dependent (pose of its rig frame) or with an independent pose (its view ID)
View& addRigView(SfMData& scene, IndexT viewId, IndexT intrinsicId, IndexT subPoseId, IndexT frameId, bool independent)
{
    const IndexT poseId = independent ? viewId : getRigPoseId(rigId, frameId);
    scene.getViews().emplace(viewId, std::make_shared<View>("", viewId, intrinsicId, poseId, 100, 100, rigId, subPoseId));
    View& view = scene.getView(viewId);
    view.setFrameId(frameId);
    view.setIndependantPose(independent);
    return view;
}

// Reference poses
const geometry::Pose3 S0 = makePose(0.1, Vec3(0.1, 0.0, 0.0));  // sub-pose 0
const geometry::Pose3 S1 = makePose(0.2, Vec3(-0.1, 0.0, 0.0));  // sub-pose 1
const geometry::Pose3 F1 = makePose(0.3, Vec3(1.0, 2.0, 3.0));  // rig pose of frame 1
const geometry::Pose3 F2 = makePose(0.4, Vec3(4.0, 5.0, 6.0));  // rig pose of frame 2
const geometry::Pose3 P200 = makePose(0.5, Vec3(7.0, 8.0, 9.0));  // pose of the view out of the rig
const geometry::Pose3 P300 = makePose(0.6, Vec3(-1.0, -2.0, -3.0));  // independent pose of a view of the rig

// Destination poses
const geometry::Pose3 Dold = makePose(-0.1, Vec3(0.0, 0.0, -5.0));
const geometry::Pose3 Dkeep = makePose(-0.2, Vec3(0.0, 0.3, 0.0));

/**
 * Reference scene, all views using intrinsic 7:
 *  - rig with 2 initialized sub-poses (S0, S1), frames 1 (F1) and 2 (F2)
 *  - view 100: sub-pose 0, frame 1 / view 101: sub-pose 1, frame 1 / view 102: sub-pose 0, frame 2
 *  - view 200: out of the rig, pose P200
 *  - view 300: in the rig (sub-pose 0, frame 3) but with an independent pose P300
 *  - view 400: in the rig with an uninitialized sub-pose, so not reconstructed
 */
SfMData makeReference()
{
    SfMData ref;
    addIntrinsic(ref, 7, 1000.0);

    ref.getRigs().emplace(rigId, Rig(3));
    ref.getRigs().at(rigId).setSubPose(0, RigSubPose(S0, ERigSubPoseStatus::ESTIMATED));
    ref.getRigs().at(rigId).setSubPose(1, RigSubPose(S1, ERigSubPoseStatus::CONSTANT));

    addPose(ref, getRigPoseId(rigId, 1), F1);
    addPose(ref, getRigPoseId(rigId, 2), F2);
    addRigView(ref, 100, 7, 0, 1, false).setResectionId(4);
    addRigView(ref, 101, 7, 1, 1, false);
    addRigView(ref, 102, 7, 0, 2, false);

    addPose(ref, 200, P200);
    addView(ref, 200, 7, 200);

    addPose(ref, 300, P300);
    addRigView(ref, 300, 7, 0, 3, true);

    addRigView(ref, 400, 7, 2, 1, false);

    return ref;
}

/// Destination scene with an uninitialized rig of 2 sub-poses, except sub-pose 1 set to Dkeep
SfMData makeDestinationWithRig()
{
    SfMData dst;
    addIntrinsic(dst, 1, 500.0);
    dst.getRigs().emplace(rigId, Rig(2));
    dst.getRigs().at(rigId).setSubPose(1, RigSubPose(Dkeep, ERigSubPoseStatus::ESTIMATED));
    return dst;
}

bool samePose(const SfMData& dst, IndexT dstViewId, const SfMData& ref, IndexT refViewId)
{
    return dst.getPose(dst.getView(dstViewId)).getTransform() == ref.getPose(ref.getView(refViewId)).getTransform();
}

}  // namespace

BOOST_AUTO_TEST_CASE(transferPoses_rigIsKept)
{
    const SfMData ref = makeReference();
    SfMData dst = makeDestinationWithRig();

    // Independent destination view of the rig, matched with a rig-dependent reference view
    addPose(dst, 10, Dold);
    addRigView(dst, 10, 1, 0, 5, true);

    sfm::transferMatchingPoses(dst, ref, {{10, 100}});

    const View& view = dst.getView(10);
    BOOST_CHECK_EQUAL(view.getPoseId(), getRigPoseId(rigId, 5));
    BOOST_CHECK(!view.isPoseIndependant());
    BOOST_CHECK_EQUAL(view.getResectionId(), 4);
    BOOST_CHECK(dst.getAbsolutePose(view.getPoseId()).getTransform() == F1);

    // Only the sub-pose of the matched view is copied
    const Rig& rig = dst.getRigs().at(rigId);
    BOOST_CHECK_EQUAL(rig.getNbSubPoses(), 2);
    BOOST_CHECK(rig.getSubPose(0) == RigSubPose(S0, ERigSubPoseStatus::ESTIMATED));
    BOOST_CHECK(rig.getSubPose(1) == RigSubPose(Dkeep, ERigSubPoseStatus::ESTIMATED));

    BOOST_CHECK(samePose(dst, 10, ref, 100));
}

BOOST_AUTO_TEST_CASE(transferPoses_rigFrameWithSeveralViews)
{
    const SfMData ref = makeReference();
    SfMData dst = makeDestinationWithRig();

    addPose(dst, getRigPoseId(rigId, 5), Dold);
    addRigView(dst, 10, 1, 0, 5, false);
    addRigView(dst, 11, 1, 1, 5, false);

    sfm::transferMatchingPoses(dst, ref, {{10, 100}, {11, 101}});

    BOOST_CHECK(dst.getAbsolutePose(getRigPoseId(rigId, 5)).getTransform() == F1);
    BOOST_CHECK(dst.getRigs().at(rigId).getSubPose(0) == RigSubPose(S0, ERigSubPoseStatus::ESTIMATED));
    BOOST_CHECK(dst.getRigs().at(rigId).getSubPose(1) == RigSubPose(S1, ERigSubPoseStatus::CONSTANT));
    BOOST_CHECK(samePose(dst, 10, ref, 100));
    BOOST_CHECK(samePose(dst, 11, ref, 101));
}

BOOST_AUTO_TEST_CASE(transferPoses_rigSubPosesAreExtended)
{
    const SfMData ref = makeReference();
    SfMData dst;
    addIntrinsic(dst, 1, 500.0);
    dst.getRigs().emplace(rigId, Rig(1));

    // Sub-pose 1 does not exist yet in the destination rig
    addRigView(dst, 11, 1, 1, 5, true);
    addPose(dst, 11, Dold);

    sfm::transferMatchingPoses(dst, ref, {{11, 101}});

    const Rig& rig = dst.getRigs().at(rigId);
    BOOST_CHECK_EQUAL(rig.getNbSubPoses(), 2);
    BOOST_CHECK(rig.getSubPose(0).status == ERigSubPoseStatus::UNINITIALIZED);
    BOOST_CHECK(rig.getSubPose(1) == RigSubPose(S1, ERigSubPoseStatus::CONSTANT));
    BOOST_CHECK(samePose(dst, 11, ref, 101));
}

BOOST_AUTO_TEST_CASE(transferPoses_rigFrameFromDifferentReferenceFrames)
{
    const SfMData ref = makeReference();
    SfMData dst = makeDestinationWithRig();

    // Both views of destination frame 5 are matched with views of reference frames 1 and 2
    addPose(dst, getRigPoseId(rigId, 5), Dold);
    addRigView(dst, 10, 1, 0, 5, false);
    addRigView(dst, 11, 1, 1, 5, false);

    sfm::transferMatchingPoses(dst, ref, {{10, 100}, {11, 102}});

    // The last transferred frame pose is kept
    BOOST_CHECK(dst.getAbsolutePose(getRigPoseId(rigId, 5)).getTransform() == F2);
    BOOST_CHECK(dst.getRigs().at(rigId).getSubPose(1) == RigSubPose(S0, ERigSubPoseStatus::ESTIMATED));
    BOOST_CHECK(samePose(dst, 11, ref, 102));
}

BOOST_AUTO_TEST_CASE(transferPoses_missingDestinationRig)
{
    const SfMData ref = makeReference();
    SfMData dst;
    addIntrinsic(dst, 1, 500.0);

    // Rig-dependent destination view whose rig does not exist: it gets an independent pose
    addRigView(dst, 10, 1, 0, 5, false);

    sfm::transferMatchingPoses(dst, ref, {{10, 100}});

    const View& view = dst.getView(10);
    BOOST_CHECK_EQUAL(view.getPoseId(), 10);
    BOOST_CHECK(view.isPoseIndependant());
    BOOST_CHECK(dst.getRigs().empty());
    // The reference pose is the composition of the rig pose and the sub-pose
    BOOST_CHECK(samePose(dst, 10, ref, 100));
    BOOST_CHECK(dst.getAbsolutePose(10).getTransform() == S0 * F1);
}

BOOST_AUTO_TEST_CASE(transferPoses_independentReferenceViewOfRig)
{
    const SfMData ref = makeReference();
    SfMData dst = makeDestinationWithRig();

    // Rig-dependent destination view matched with a reference view with an independent pose
    addPose(dst, getRigPoseId(rigId, 5), Dold);
    addRigView(dst, 10, 1, 0, 5, false);

    sfm::transferMatchingPoses(dst, ref, {{10, 300}});

    // The destination view is detached from its rig, the rig is left untouched
    const View& view = dst.getView(10);
    BOOST_CHECK_EQUAL(view.getPoseId(), 10);
    BOOST_CHECK(view.isPoseIndependant());
    BOOST_CHECK(dst.getAbsolutePose(10).getTransform() == P300);
    BOOST_CHECK(dst.getAbsolutePose(getRigPoseId(rigId, 5)).getTransform() == Dold);
    BOOST_CHECK(dst.getRigs().at(rigId).getSubPose(0).status == ERigSubPoseStatus::UNINITIALIZED);
}

BOOST_AUTO_TEST_CASE(transferPoses_independentDestinationViewOutOfRig)
{
    const SfMData ref = makeReference();
    SfMData dst = makeDestinationWithRig();

    // Destination views out of the rig, matched with a rig-dependent view and a view out of the rig
    addPose(dst, 20, Dold);
    addView(dst, 20, 1, 20);
    addPose(dst, 21, Dold);
    addView(dst, 21, 1, 21);

    sfm::transferMatchingPoses(dst, ref, {{20, 100}, {21, 200}});

    BOOST_CHECK_EQUAL(dst.getView(20).getPoseId(), 20);
    BOOST_CHECK_EQUAL(dst.getView(21).getPoseId(), 21);
    BOOST_CHECK(dst.getAbsolutePose(20).getTransform() == S0 * F1);
    BOOST_CHECK(dst.getAbsolutePose(21).getTransform() == P200);
    BOOST_CHECK(dst.getRigs().at(rigId).getSubPose(0).status == ERigSubPoseStatus::UNINITIALIZED);
}

BOOST_AUTO_TEST_CASE(transferPoses_unreconstructedReferenceIsSkipped)
{
    SfMData ref = makeReference();
    SfMData dst = makeDestinationWithRig();

    // Reference view 400 has an uninitialized sub-pose, view 500 has no pose
    addView(ref, 500, 7, 500);

    addPose(dst, 10, Dold);
    addRigView(dst, 10, 1, 0, 5, true);
    addPose(dst, 20, Dold);
    addView(dst, 20, 1, 20);

    sfm::transferMatchingPoses(dst, ref, {{10, 400}, {20, 500}});

    BOOST_CHECK_EQUAL(dst.getView(10).getPoseId(), 10);
    BOOST_CHECK(dst.getView(10).isPoseIndependant());
    BOOST_CHECK(dst.getAbsolutePose(10).getTransform() == Dold);
    BOOST_CHECK(dst.getAbsolutePose(20).getTransform() == Dold);
    BOOST_CHECK(dst.getRigs().at(rigId).getSubPose(0).status == ERigSubPoseStatus::UNINITIALIZED);
}

BOOST_AUTO_TEST_CASE(transferIntrinsics_overwriteAndSplit)
{
    SfMData ref = makeReference();
    addIntrinsic(ref, 8, 2000.0);
    addPose(ref, 201, P200);
    addView(ref, 201, 8, 201);

    // Destination views 0, 1 and 2 share intrinsic 1, only 0 and 1 are matched, with different reference intrinsics
    SfMData dst;
    addIntrinsic(dst, 1, 500.0);
    addView(dst, 0, 1, 0);
    addView(dst, 1, 1, 1);
    addView(dst, 2, 1, 2);

    sfm::transferMatchingIntrinsics(dst, ref, {{0, 200}, {1, 201}});

    BOOST_CHECK_EQUAL(dst.getIntrinsics().size(), 2);
    BOOST_CHECK_EQUAL(dst.getView(0).getIntrinsicId(), 1);
    BOOST_CHECK_EQUAL(dst.getView(1).getIntrinsicId(), 2);
    BOOST_CHECK_EQUAL(dst.getView(2).getIntrinsicId(), 1);
    BOOST_CHECK(*dst.getIntrinsics().at(1) == *ref.getIntrinsics().at(7));
    BOOST_CHECK(*dst.getIntrinsics().at(2) == *ref.getIntrinsics().at(8));
}

BOOST_AUTO_TEST_CASE(transferIntrinsics_neverMerged)
{
    const SfMData ref = makeReference();

    // Destination intrinsics 1 and 2 are matched with the same reference intrinsic 7
    SfMData dst;
    addIntrinsic(dst, 1, 500.0);
    addIntrinsic(dst, 2, 600.0);
    addView(dst, 0, 1, 0);
    addView(dst, 1, 2, 1);
    // Destination view with a missing intrinsic
    addView(dst, 2, 9, 2);

    sfm::transferMatchingIntrinsics(dst, ref, {{0, 100}, {1, 101}, {2, 200}});

    BOOST_CHECK_EQUAL(dst.getIntrinsics().size(), 3);
    BOOST_CHECK_EQUAL(dst.getView(0).getIntrinsicId(), 1);
    BOOST_CHECK_EQUAL(dst.getView(1).getIntrinsicId(), 2);
    BOOST_CHECK_EQUAL(dst.getView(2).getIntrinsicId(), 3);
    for (IndexT intrinsicId : {1, 2, 3})
    {
        BOOST_CHECK(*dst.getIntrinsics().at(intrinsicId) == *ref.getIntrinsics().at(7));
    }
}

BOOST_AUTO_TEST_CASE(transferLandmarks_reindexed)
{
    SfMData ref = makeReference();
    Landmark seen(Vec3(1.0, 2.0, 3.0));
    seen.getObservations().emplace(100, Observation(Vec2(10.0, 20.0), 1, 1.0));
    seen.getObservations().emplace(102, Observation(Vec2(30.0, 40.0), 2, 1.0));
    ref.getLandmarks().emplace(1, seen);
    Landmark unseen(Vec3(4.0, 5.0, 6.0));
    unseen.getObservations().emplace(102, Observation(Vec2(50.0, 60.0), 3, 1.0));
    ref.getLandmarks().emplace(2, unseen);

    SfMData dst;
    dst.getLandmarks().emplace(5, Landmark(Vec3(0.0, 0.0, 0.0)));

    sfm::transferMatchingLandmarks(dst, ref, {{10, 100}});

    // Destination landmarks are replaced, and only observations in matched views are kept
    BOOST_REQUIRE_EQUAL(dst.getLandmarks().size(), 1);
    const Landmark& landmark = dst.getLandmarks().at(1);
    BOOST_CHECK_EQUAL(landmark.getObservations().size(), 1);
    BOOST_CHECK_EQUAL(landmark.getObservations().at(10).getFeatureId(), 1);
}

BOOST_AUTO_TEST_CASE(transferSurveyPoints_perView)
{
    SfMData ref = makeReference();
    ref.getSurveyPoints()[100] = {SurveyPoint(Vec3(1.0, 2.0, 3.0), Vec2(4.0, 5.0), Vec2::Zero())};

    SfMData dst;
    dst.getSurveyPoints()[10] = {SurveyPoint(), SurveyPoint()};
    dst.getSurveyPoints()[11] = {SurveyPoint()};

    sfm::transferMatchingSurveyPoints(dst, ref, {{10, 100}, {11, 101}});

    // View 10 gets the reference survey points, view 11 keeps its own as reference view 101 has none
    BOOST_REQUIRE_EQUAL(dst.getSurveyPoints().at(10).size(), 1);
    BOOST_CHECK(dst.getSurveyPoints().at(10)[0].survey == Vec2(4.0, 5.0));
    BOOST_CHECK_EQUAL(dst.getSurveyPoints().at(11).size(), 1);
}
