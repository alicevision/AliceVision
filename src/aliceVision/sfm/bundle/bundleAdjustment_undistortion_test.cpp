// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfm/sfm.hpp>
#include <aliceVision/camera/camera_test_common.hpp>
#include <aliceVision/geometry/lie.hpp>
#include <aliceVision/multiview/NViewDataSet.hpp>

#define BOOST_TEST_MODULE bundleAdjustmentUndistortion

#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>

using namespace aliceVision;
using namespace aliceVision::camera;
using namespace aliceVision::sfm;
using namespace aliceVision::sfmData;

namespace {

/**
 * @brief Synthetic scene seen by a ring of cameras sharing one intrinsic, with exact observations
 *        computed with the full camera model (undistortion included).
 */
SfMData makeScene(const std::shared_ptr<IntrinsicScaleOffsetDisto>& intrinsic, int nviews, int npoints)
{
    const NViewDatasetConfigurator config(1500, 1500, 960, 540, 1.5, 0.01);
    const NViewDataSet d = NRealisticCamerasRing(nviews, npoints, config);

    SfMData sfmData;
    sfmData.getIntrinsics().emplace(0, intrinsic);

    for (int i = 0; i < nviews; ++i)
    {
        sfmData.getViews().emplace(i, std::make_shared<View>("", i, 0, i, intrinsic->w(), intrinsic->h()));
        sfmData.setPose(*sfmData.getViews().at(i), CameraPose(geometry::Pose3(d._R[i], d._C[i])));
    }

    for (int j = 0; j < npoints; ++j)
    {
        Landmark landmark;
        landmark.setX(d._X.col(j));
        for (int i = 0; i < nviews; ++i)
        {
            const geometry::Pose3 pose(d._R[i], d._C[i]);
            const Vec2 pix = intrinsic->transformProject(pose, d._X.col(j).homogeneous(), true);
            landmark.getObservations()[i] = Observation(pix, j, 1.0);
        }
        sfmData.getLandmarks()[j] = landmark;
    }

    return sfmData;
}

/// Root mean square of the raw pixel reprojection errors, with the full camera model.
double reprojectionRMSE(const SfMData& sfmData)
{
    double sum = 0.0;
    int count = 0;
    for (const auto& [landmarkId, landmark] : sfmData.getLandmarks())
    {
        for (const auto& [viewId, observation] : landmark.getObservations())
        {
            const View& view = sfmData.getView(viewId);
            const IntrinsicBase& intrinsic = sfmData.getIntrinsic(view.getIntrinsicId());
            const Vec2 projected = intrinsic.transformProject(sfmData.getPose(view).getTransform(), landmark.getX().homogeneous(), true);
            sum += (projected - observation.getCoordinates()).squaredNorm();
            ++count;
        }
    }
    return std::sqrt(sum / count);
}

/// Move the poses and the points away from the solution.
void perturb(SfMData& sfmData)
{
    makeRandomOperationsReproducible();
    for (auto& [poseId, pose] : sfmData.getPoses())
    {
        const geometry::Pose3& transform = pose->getTransform();
        const Mat3 R = transform.rotation() * SO3::expm(0.01 * Vec3::Random());
        pose->setTransform(geometry::Pose3(R, transform.center() + 0.01 * Vec3::Random()));
    }
    for (auto& [landmarkId, landmark] : sfmData.getLandmarks())
    {
        landmark.setX(landmark.getX() + 0.01 * Vec3::Random());
    }
}

}  // namespace

// With a fixed 3DE undistortion, the bundle adjustment converges back to exact observations.
BOOST_AUTO_TEST_CASE(bundleAdjustmentUndistortion_converges)
{
    for (EUNDISTORTION undistortion : test::allUndistortionTypes())
    {
        if (undistortion == EUNDISTORTION::UNDISTORTION_NONE)
        {
            continue;
        }

        BOOST_TEST_CONTEXT(EUNDISTORTION_enumToString(undistortion))
        {
            const test::TestCamera tc = test::makeTestCamera(EINTRINSIC::PINHOLE_CAMERA, EDISTORTION::DISTORTION_NONE, undistortion);
            SfMData sfmData = makeScene(tc.intrinsic, 6, 60);
            perturb(sfmData);
            BOOST_CHECK_GT(reprojectionRMSE(sfmData), 1.0);

            BundleAdjustmentCeres bundleAdjustment;
            BOOST_REQUIRE(bundleAdjustment.adjust(
              sfmData, BundleAdjustment::REFINE_ROTATION | BundleAdjustment::REFINE_CENTER | BundleAdjustment::REFINE_STRUCTURE));

            BOOST_CHECK_SMALL(reprojectionRMSE(sfmData), 1e-4);
        }
    }
}

// With both objects, the distortion on top of the fixed undistortion is recovered.
BOOST_AUTO_TEST_CASE(bundleAdjustmentUndistortion_refineDistortion)
{
    const test::TestCamera tc =
      test::makeTestCamera(EINTRINSIC::PINHOLE_CAMERA, EDISTORTION::DISTORTION_RADIALK3, EUNDISTORTION::UNDISTORTION_3DERADIAL4);
    const std::vector<double> groundTruth = tc.intrinsic->getDistortionParams();

    SfMData sfmData = makeScene(tc.intrinsic, 6, 60);
    perturb(sfmData);

    std::vector<double> start = groundTruth;
    for (double& p : start)
    {
        p += 0.02;
    }
    tc.intrinsic->setDistortionParams(start);
    BOOST_REQUIRE(tc.intrinsic->getDistortionInitializationMode() != EInitMode::CALIBRATED);

    BundleAdjustmentCeres bundleAdjustment;
    BOOST_REQUIRE(bundleAdjustment.adjust(sfmData,
                                          BundleAdjustment::REFINE_ROTATION | BundleAdjustment::REFINE_CENTER | BundleAdjustment::REFINE_STRUCTURE |
                                            BundleAdjustment::REFINE_INTRINSICS_DISTORTION));

    BOOST_CHECK_SMALL(reprojectionRMSE(sfmData), 1e-4);
    const std::vector<double> refined = tc.intrinsic->getDistortionParams();
    for (std::size_t i = 0; i < groundTruth.size(); ++i)
    {
        BOOST_CHECK_SMALL(refined[i] - groundTruth[i], 1e-3);
    }
}
