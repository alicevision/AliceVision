// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/camera/camera.hpp>
#include <aliceVision/camera/camera_test_common.hpp>
#include <aliceVision/geometry/Pose3.hpp>

#define BOOST_TEST_MODULE intrinsicProjection

#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>
#include <aliceVision/unitTest.hpp>

using namespace aliceVision;
using namespace aliceVision::camera;
using namespace aliceVision::camera::test;

// Every test below runs on all the camera/distortion combinations of makeTestCameras(),
// and reports the largest error over a grid of pixels for each combination.

BOOST_AUTO_TEST_CASE(intrinsicProjection_cameraList)
{
    // 7 distortions for pinhole, 4 undistortions for pinhole, 6 x 4 pairs for pinhole,
    // 7 distortions for equidistant, 1 equirectangular.
    const std::vector<TestCamera> cameras = makeTestCameras();
    BOOST_CHECK_EQUAL(cameras.size(), 43);

    for (const TestCamera& tc : cameras)
    {
        BOOST_TEST_CONTEXT(tc.name)
        {
            BOOST_REQUIRE(tc.intrinsic);
            BOOST_CHECK_EQUAL(tc.intrinsic->getType(), tc.intrinsicType);
            BOOST_CHECK_EQUAL(getDistortionType(*tc.intrinsic), tc.distortionType);
            const EUNDISTORTION undistortionType =
              tc.intrinsic->getUndistortion() ? tc.intrinsic->getUndistortion()->getType() : EUNDISTORTION::UNDISTORTION_NONE;
            BOOST_CHECK_EQUAL(undistortionType, tc.undistortionType);
            BOOST_CHECK(tc.intrinsic->isValid());
            BOOST_CHECK_EQUAL(tc.intrinsic->hasDistortion(),
                              tc.distortionType != EDISTORTION::DISTORTION_NONE || tc.undistortionType != EUNDISTORTION::UNDISTORTION_NONE);
        }
    }
}

BOOST_AUTO_TEST_CASE(intrinsicProjection_cam2imaIma2cam)
{
    for (const TestCamera& tc : makeTestCameras())
    {
        const IntrinsicBase& cam = *tc.intrinsic;

        double maxError = 0.0;
        for (const Vec2& pix : samplePixels(cam))
        {
            maxError = std::max(maxError, (cam.cam2ima(cam.ima2cam(pix)) - pix).norm());
        }
        BOOST_CHECK_MESSAGE(maxError < 1e-9, tc.name << ": cam2ima(ima2cam(x)) != x, max error " << maxError);
    }
}

// Points are taken undistorted in the camera plane, distorted, then undistorted again.
BOOST_AUTO_TEST_CASE(intrinsicProjection_addRemoveDistortion)
{
    for (const TestCamera& tc : makeTestCameras())
    {
        const IntrinsicBase& cam = *tc.intrinsic;

        double maxError = 0.0;
        double maxDisplacement = 0.0;
        for (const Vec2& pix : samplePixels(cam))
        {
            const Vec2 undistorted = cam.ima2cam(pix);
            const Vec2 distorted = cam.addDistortion(undistorted);

            maxError = std::max(maxError, (cam.removeDistortion(distorted) - undistorted).norm());
            maxDisplacement = std::max(maxDisplacement, (distorted - undistorted).norm());
        }

        BOOST_CHECK_MESSAGE(maxError < 1e-6, tc.name << ": removeDistortion(addDistortion(p)) != p, max error " << maxError);

        // The distortion must actually move points, otherwise the round trip above proves nothing.
        if (cam.hasDistortion())
        {
            BOOST_CHECK_MESSAGE(maxDisplacement > 1e-3, tc.name << ": distortion does not move points");
        }
        else
        {
            BOOST_CHECK_MESSAGE(maxDisplacement == 0.0, tc.name << ": camera without distortion moves points");
        }
    }
}

BOOST_AUTO_TEST_CASE(intrinsicProjection_distortedUndistortedPixel)
{
    for (const TestCamera& tc : makeTestCameras())
    {
        const IntrinsicBase& cam = *tc.intrinsic;

        double maxErrorDistortFirst = 0.0;
        double maxErrorUndistortFirst = 0.0;
        for (const Vec2& pix : samplePixels(cam))
        {
            maxErrorDistortFirst = std::max(maxErrorDistortFirst, (cam.getUndistortedPixel(cam.getDistortedPixel(pix)) - pix).norm());
            maxErrorUndistortFirst = std::max(maxErrorUndistortFirst, (cam.getDistortedPixel(cam.getUndistortedPixel(pix)) - pix).norm());

            // The pixel functions must be the camera-plane functions expressed in pixels.
            const Vec2 expected = cam.cam2ima(cam.addDistortion(cam.ima2cam(pix)));
            BOOST_CHECK_SMALL((cam.getDistortedPixel(pix) - expected).norm(), 1e-9);
        }

        BOOST_CHECK_MESSAGE(maxErrorDistortFirst < 1e-3,
                            tc.name << ": getUndistortedPixel(getDistortedPixel(x)) != x, max error " << maxErrorDistortFirst);
        BOOST_CHECK_MESSAGE(maxErrorUndistortFirst < 1e-3,
                            tc.name << ": getDistortedPixel(getUndistortedPixel(x)) != x, max error " << maxErrorUndistortFirst);
    }
}

// Back-project a pixel at a known depth through a random pose, then project it again.
BOOST_AUTO_TEST_CASE(intrinsicProjection_backprojectProject)
{
    makeRandomOperationsReproducible();

    for (const TestCamera& tc : makeTestCameras())
    {
        const IntrinsicBase& cam = *tc.intrinsic;

        double maxError = 0.0;
        double maxErrorNoDistortion = 0.0;
        double maxDepthError = 0.0;
        for (const Vec2& pix : samplePixels(cam))
        {
            const geometry::Pose3 pose = geometry::randomPose();
            const double depth = 2.0 + 10.0 * std::abs(Vec2::Random()(0));

            const Vec3 X = cam.backprojectTransform(pix, true, pose, depth);
            maxError = std::max(maxError, (cam.transformProject(pose, X.homogeneous(), true) - pix).norm());
            maxDepthError = std::max(maxDepthError, std::abs((pose(X)).norm() - depth));

            const Vec3 Xnd = cam.backprojectTransform(pix, false, pose, depth);
            maxErrorNoDistortion = std::max(maxErrorNoDistortion, (cam.transformProject(pose, Xnd.homogeneous(), false) - pix).norm());
        }

        BOOST_CHECK_MESSAGE(maxError < 1e-3, tc.name << ": project(backproject(x)) != x, max error " << maxError);
        BOOST_CHECK_MESSAGE(maxErrorNoDistortion < 1e-6,
                            tc.name << ": project(backproject(x)) != x without distortion, max error " << maxErrorNoDistortion);
        BOOST_CHECK_MESSAGE(maxDepthError < 1e-6, tc.name << ": back-projected point not at the requested distance, max error " << maxDepthError);
    }
}

BOOST_AUTO_TEST_CASE(intrinsicProjection_projectMatchesTransformProject)
{
    makeRandomOperationsReproducible();

    for (const TestCamera& tc : makeTestCameras())
    {
        const IntrinsicBase& cam = *tc.intrinsic;

        double maxError = 0.0;
        double maxErrorEigen = 0.0;
        for (const Vec2& pix : samplePixels(cam))
        {
            const Vec4 X = (3.0 * cam.backProjectUnit(pix)).homogeneous();
            maxError = std::max(maxError, (cam.project(X, true) - cam.transformProject(Eigen::Matrix4d::Identity(), X, true)).norm());
            maxError = std::max(maxError, (cam.project(X, false) - cam.transformProject(Eigen::Matrix4d::Identity(), X, false)).norm());

            // The Pose3 overload must agree with the matrix one.
            const geometry::Pose3 pose = geometry::randomPose();
            const Vec3 Xc = X.head<3>();
            const Vec3 Yw = pose.inverse()(Xc);
            const Vec4 Y = Yw.homogeneous();
            maxErrorEigen =
              std::max(maxErrorEigen, (cam.transformProject(pose, Y, true) - cam.transformProject(pose.getHomogeneous(), Y, true)).norm());
        }

        BOOST_CHECK_MESSAGE(maxError < 1e-9, tc.name << ": project != transformProject(identity), max error " << maxError);
        BOOST_CHECK_MESSAGE(maxErrorEigen < 1e-9, tc.name << ": transformProject(Pose3) != transformProject(Matrix4d), max error " << maxErrorEigen);
    }
}

BOOST_AUTO_TEST_CASE(intrinsicProjection_unitSphere)
{
    for (const TestCamera& tc : makeTestCameras())
    {
        const IntrinsicBase& cam = *tc.intrinsic;

        double maxNormError = 0.0;
        double maxProjectionError = 0.0;
        for (const Vec2& pix : samplePixels(cam))
        {
            const Vec3 ray = cam.backProjectUnit(pix);
            maxNormError = std::max(maxNormError, std::abs(ray.norm() - 1.0));
            maxNormError = std::max(maxNormError, std::abs(cam.toUnitSphere(cam.ima2cam(pix)).norm() - 1.0));

            // Any point along the ray projects back to the pixel.
            maxProjectionError = std::max(maxProjectionError, (cam.project((0.5 * ray).homogeneous(), true) - pix).norm());
            maxProjectionError = std::max(maxProjectionError, (cam.project((25.0 * ray).homogeneous(), true) - pix).norm());
        }

        BOOST_CHECK_MESSAGE(maxNormError < 1e-9, tc.name << ": rays are not unit vectors, max error " << maxNormError);
        BOOST_CHECK_MESSAGE(maxProjectionError < 1e-3, tc.name << ": project(backProjectUnit(x)) != x, max error " << maxProjectionError);
    }
}

// residual() compares the observation and the projection in undistorted pixel space.
BOOST_AUTO_TEST_CASE(intrinsicProjection_residual)
{
    makeRandomOperationsReproducible();

    for (const TestCamera& tc : makeTestCameras())
    {
        const IntrinsicBase& cam = *tc.intrinsic;
        const std::vector<Vec2> pixels = samplePixels(cam);
        const geometry::Pose3 pose = geometry::randomPose();
        const Vec2 delta(0.5, -0.25);

        Mat3X X(3, pixels.size());
        Mat2X x(2, pixels.size());
        for (std::size_t i = 0; i < pixels.size(); ++i)
        {
            X.col(i) = cam.backprojectTransform(pixels[i], true, pose, 5.0);
            x.col(i) = pixels[i];
        }

        const Mat2X residuals = cam.residuals(pose, X, x);
        BOOST_REQUIRE_EQUAL(residuals.cols(), static_cast<Eigen::Index>(pixels.size()));

        double maxError = 0.0;
        for (std::size_t i = 0; i < pixels.size(); ++i)
        {
            const Vec4 Xh = X.col(i).homogeneous();

            // The exact observation of a point has no residual.
            const Vec2 residual = cam.residual(pose, Xh, x.col(i));
            maxError = std::max(maxError, residual.norm());
            maxError = std::max(maxError, (residuals.col(i) - residual).norm());

            // Without distortion, the residual is observation - projection.
            const Vec2 undistortedObservation = cam.transformProject(pose, Xh, false) + delta;
            maxError = std::max(maxError, (cam.residual(pose, Xh, undistortedObservation, false) - delta).norm());
        }

        BOOST_CHECK_MESSAGE(maxError < 1e-3, tc.name << ": wrong residual, max error " << maxError);
    }
}

namespace {

// Rescaling the camera by 1/2 must divide every pixel coordinate by 2.
// Returns the largest error over the sample pixels.
double rescaleError(const IntrinsicBase& cam, float factorW, float factorH)
{
    std::shared_ptr<IntrinsicBase> rescaled(cam.clone());
    rescaled->rescale(factorW, factorH);

    BOOST_CHECK_EQUAL(rescaled->w(), static_cast<unsigned int>(cam.w() * factorW));
    BOOST_CHECK_EQUAL(rescaled->h(), static_cast<unsigned int>(cam.h() * factorH));

    double maxError = 0.0;
    for (const Vec2& pix : samplePixels(cam))
    {
        const Vec4 X = (4.0 * cam.backProjectUnit(pix)).homogeneous();
        maxError = std::max(maxError, (rescaled->project(X, true) - pix.cwiseProduct(Vec2(factorW, factorH))).norm());
    }
    return maxError;
}

}  // namespace

// Rescaling the camera must scale every pixel coordinate by the same factors.
BOOST_AUTO_TEST_CASE(intrinsicProjection_rescale)
{
    for (const TestCamera& tc : makeTestCameras())
    {
        const double maxError = rescaleError(*tc.intrinsic, 0.5f, 0.5f);
        BOOST_CHECK_MESSAGE(maxError < 1e-3,
                            tc.name << ": camera rescaled by 0.5 does not project at half the pixel coordinates, max error " << maxError);

        // 1920x1080 to 240x135: the undistortion center of an odd size is not an integer.
        const double maxErrorOdd = rescaleError(*tc.intrinsic, 0.125f, 0.125f);
        BOOST_CHECK_MESSAGE(maxErrorOdd < 1e-3,
                            tc.name << ": camera rescaled by 1/8 (odd height) does not project at the rescaled pixel coordinates, max error "
                                    << maxErrorOdd);

        // Non uniform: the pixel aspect ratio changes. The anamorphic model also uses the pixel aspect
        // ratio as its lens squeeze, so only desqueezing is exact for it (see intrinsicProjection_rescaleDesqueeze).
        // The image circle of equidistant cameras cannot become an ellipse.
        if (tc.undistortionType != EUNDISTORTION::UNDISTORTION_3DEANAMORPHIC4 && !isEquidistant(tc.intrinsicType))
        {
            const double maxErrorNonUniform = rescaleError(*tc.intrinsic, 0.5f, 0.25f);
            BOOST_CHECK_MESSAGE(maxErrorNonUniform < 1e-3,
                                tc.name << ": camera rescaled by (0.5, 0.25) does not project at the rescaled pixel coordinates, max error "
                                        << maxErrorNonUniform);
        }
    }
}

// Desqueezing an anamorphic image (as imageProcessing does): x is stretched by the pixel aspect ratio.
BOOST_AUTO_TEST_CASE(intrinsicProjection_rescaleDesqueeze)
{
    for (const TestCamera& tc : makeTestCameras())
    {
        if (tc.undistortionType == EUNDISTORTION::UNDISTORTION_NONE)
        {
            continue;
        }

        std::shared_ptr<IntrinsicScaleOffsetDisto> squeezed(dynamic_cast<IntrinsicScaleOffsetDisto*>(tc.intrinsic->clone()));
        squeezed->getUndistortion()->setPixelAspectRatio(2.0);
        BOOST_REQUIRE(!squeezed->getUndistortion()->isDesqueezed());

        const double maxError = rescaleError(*squeezed, 1.0f, 0.5f);
        BOOST_CHECK_MESSAGE(maxError < 1e-3,
                            tc.name << ": desqueezed camera does not project at the rescaled pixel coordinates, max error " << maxError);

        std::shared_ptr<IntrinsicScaleOffsetDisto> desqueezed(dynamic_cast<IntrinsicScaleOffsetDisto*>(squeezed->clone()));
        desqueezed->rescale(1.0f, 0.5f);
        BOOST_CHECK(desqueezed->getUndistortion()->isDesqueezed());
        BOOST_CHECK_EQUAL(desqueezed->getUndistortion()->getPixelAspectRatio(), 2.0);
    }
}

// Rays from pixels inside the image are visible.
BOOST_AUTO_TEST_CASE(intrinsicProjection_visibility)
{
    for (const TestCamera& tc : makeTestCameras())
    {
        const IntrinsicBase& cam = *tc.intrinsic;

        int notVisible = 0;
        for (const Vec2& pix : samplePixels(cam))
        {
            BOOST_CHECK(cam.isVisible(pix));
            BOOST_CHECK(cam.isVisible(Vec2f(pix.cast<float>())));
            if (!cam.isVisibleRay(cam.backProjectUnit(pix)))
            {
                ++notVisible;
            }
        }
        BOOST_CHECK_MESSAGE(notVisible == 0, tc.name << ": " << notVisible << " rays from inside the image are reported not visible");

        if (!isEquirectangular(cam.getType()))
        {
            // Straight behind the camera
            BOOST_CHECK_MESSAGE(!cam.isVisibleRay(Vec3(0.0, 0.0, -1.0)), tc.name << ": ray behind the camera reported visible");
        }
    }
}

// With both objects, the undistortion is applied on the raw image and the distortion on top of it.
BOOST_AUTO_TEST_CASE(intrinsicProjection_composition)
{
    for (const TestCamera& tc : makeTestCameras())
    {
        const IntrinsicScaleOffsetDisto& cam = *tc.intrinsic;
        if (!cam.getDistortion() || !cam.getUndistortion())
        {
            continue;
        }

        const Distortion& D = *cam.getDistortion();
        const Undistortion& U = *cam.getUndistortion();

        double maxError = 0.0;
        for (const Vec2& pix : samplePixels(cam))
        {
            const Vec2 p = cam.ima2cam(pix);

            const Vec2 removed = D.removeDistortion(cam.ima2cam(U.undistort(cam.cam2ima(p))));
            maxError = std::max(maxError, (cam.removeDistortion(p) - removed).norm());

            const Vec2 added = cam.ima2cam(U.inverse(cam.cam2ima(D.addDistortion(p))));
            maxError = std::max(maxError, (cam.addDistortion(p) - added).norm());
        }

        BOOST_CHECK_MESSAGE(maxError < 1e-12, tc.name << ": distortion and undistortion are not composed, max error " << maxError);
    }
}
