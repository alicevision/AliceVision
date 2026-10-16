// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/camera/camera.hpp>
#include <aliceVision/camera/camera_test_common.hpp>
#include <aliceVision/geometry/Pose3.hpp>

#define BOOST_TEST_MODULE intrinsicDerivatives

#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>
#include <aliceVision/unitTest.hpp>

using namespace aliceVision;
using namespace aliceVision::camera;
using namespace aliceVision::camera::test;

// The analytic Jacobians of the intrinsics are fed to Ceres by the bundle adjustment cost
// functions (sfm/bundle/costfunctions/intrinsicsProject.hpp and intrinsicsLift.hpp).
// Each of them is compared to a central finite difference of the function it differentiates,
// on every camera/distortion combination of makeTestCameras().

namespace {

enum class Check
{
    TransformProjectWrtPoint3,
    TransformProjectWrtParams,
    TransformProjectWrtDistortion,
    BackProjectUnitWrtParams,
    BackProjectUnitWrtDistortion,
    ToUnitSphereWrtPoint,
    Cam2ImaIma2Cam,
};

std::string toString(Check check)
{
    switch (check)
    {
        case Check::TransformProjectWrtPoint3:
            return "getDerivativeTransformProjectWrtPoint3";
        case Check::TransformProjectWrtParams:
            return "getDerivativeTransformProjectWrtParams";
        case Check::TransformProjectWrtDistortion:
            return "getDerivativeTransformProjectWrtDistortion";
        case Check::BackProjectUnitWrtParams:
            return "getDerivativeBackProjectUnitWrtParams";
        case Check::BackProjectUnitWrtDistortion:
            return "getDerivativeBackProjectUnitWrtDistortion";
        case Check::ToUnitSphereWrtPoint:
            return "getDerivativetoUnitSphereWrtPoint";
        case Check::Cam2ImaIma2Cam:
            return "cam2ima/ima2cam derivatives";
    }
    return "";
}

/// Relative difference, so that the tolerance does not depend on the magnitude of the Jacobian.
/// Non-finite Jacobians are reported as an infinite error: a NaN would be ignored by std::max.
double relativeError(const Eigen::MatrixXd& analytic, const Eigen::MatrixXd& numeric)
{
    if (!analytic.allFinite() || !numeric.allFinite())
    {
        return std::numeric_limits<double>::infinity();
    }
    const double scale = std::max(1.0, numeric.size() ? numeric.cwiseAbs().maxCoeff() : 0.0);
    return maxAbsDiff(analytic, numeric) / scale;
}

/// 3D points seen by the camera, expressed in world coordinates, with the pose observing them.
struct Observation
{
    Eigen::Matrix4d T;
    Vec3 X;
    Vec2 pix;
};

std::vector<Observation> makeObservations(const IntrinsicBase& cam)
{
    makeRandomOperationsReproducible();

    std::vector<Observation> observations;
    for (const Vec2& pix : samplePixels(cam, 5))
    {
        const geometry::Pose3 pose = geometry::randomPose();
        Observation obs;
        obs.T = pose.getHomogeneous();
        obs.X = cam.backprojectTransform(pix, true, pose, 3.0 + pix(0) / cam.w());
        obs.pix = pix;
        observations.push_back(obs);
    }
    return observations;
}

Eigen::Matrix<double, 3, 2> derivativeToUnitSphereWrtPoint(const IntrinsicBase& cam, const Vec2& pt)
{
    if (const Pinhole* pinhole = dynamic_cast<const Pinhole*>(&cam))
    {
        return pinhole->getDerivativetoUnitSphereWrtPoint(pt);
    }
    if (const Equidistant* equidistant = dynamic_cast<const Equidistant*>(&cam))
    {
        return equidistant->getDerivativetoUnitSphereWrtPoint(pt);
    }
    return dynamic_cast<const Equirectangular&>(cam).getDerivativetoUnitSphereWrtPoint(pt);
}

/**
 * Numerical Jacobians of backProjectUnitDistortionOnly(pix) = toUnitSphere(q),
 * with q = D^-1(ima2cam(pix)) and D the Distortion object.
 *
 * D^-1 is solved iteratively for most distortion models, so differencing it directly only reaches
 * the accuracy of the solver. Its derivatives are taken from the forward model instead, which is exact:
 * dq/dtheta = (dD/dq)^-1 * (dima2cam/dtheta - dD/dtheta).
 */
Vec2 addDistortionOnly(const IntrinsicScaleOffsetDisto& cam, const Vec2& p)
{ return cam.getDistortion() ? cam.getDistortion()->addDistortion(p) : p; }

Vec2 removeDistortionOnly(const IntrinsicScaleOffsetDisto& cam, const Vec2& p)
{ return cam.getDistortion() ? cam.getDistortion()->removeDistortion(p) : p; }

Eigen::MatrixXd backProjectUnitWrtParams(const IntrinsicScaleOffsetDisto& cam, const Vec2& pix)
{
    std::shared_ptr<IntrinsicBase> copy(cam.clone());
    const Eigen::VectorXd params = toVector(cam.getParameters());

    const Vec2 q = removeDistortionOnly(cam, cam.ima2cam(pix));

    const auto add = [&](const Eigen::VectorXd& x) -> Eigen::VectorXd { return addDistortionOnly(cam, Vec2(x)); };
    const auto sphereWrtPoint = [&](const Eigen::VectorXd& x) -> Eigen::VectorXd { return cam.toUnitSphere(Vec2(x)); };
    const auto ima2camWrtParams = [&](const Eigen::VectorXd& p) -> Eigen::VectorXd {
        copy->updateFromParams(toStdVector(p));
        return copy->ima2cam(pix);
    };
    const auto sphereWrtParams = [&](const Eigen::VectorXd& p) -> Eigen::VectorXd {
        copy->updateFromParams(toStdVector(p));
        return copy->toUnitSphere(q);
    };

    const Eigen::MatrixXd dqdtheta = numericalJacobian(add, q).inverse() * numericalJacobian(ima2camWrtParams, params);
    return numericalJacobian(sphereWrtParams, params) + numericalJacobian(sphereWrtPoint, q) * dqdtheta;
}

Eigen::MatrixXd backProjectUnitWrtDistortion(const IntrinsicScaleOffsetDisto& cam, const Vec2& pix)
{
    std::shared_ptr<IntrinsicScaleOffsetDisto> copy(dynamic_cast<IntrinsicScaleOffsetDisto*>(cam.clone()));
    const Eigen::VectorXd params = toVector(cam.getDistortionParams());

    const Vec2 q = removeDistortionOnly(cam, cam.ima2cam(pix));

    const auto add = [&](const Eigen::VectorXd& x) -> Eigen::VectorXd { return addDistortionOnly(cam, Vec2(x)); };
    const auto sphereWrtPoint = [&](const Eigen::VectorXd& x) -> Eigen::VectorXd { return cam.toUnitSphere(Vec2(x)); };
    const auto addWrtDistortion = [&](const Eigen::VectorXd& p) -> Eigen::VectorXd {
        copy->setDistortionParamsFn(p.size(), [&](std::size_t i) { return p(i); });
        return addDistortionOnly(*copy, q);
    };

    const Eigen::MatrixXd dqdtheta = -numericalJacobian(add, q).inverse() * numericalJacobian(addWrtDistortion, params);
    return numericalJacobian(sphereWrtPoint, q) * dqdtheta;
}

/// Largest relative error between the analytic and numerical Jacobians, over all the observations.
double checkError(const TestCamera& tc, Check check)
{
    const IntrinsicScaleOffsetDisto& cam = *tc.intrinsic;
    const std::size_t distortionSize = cam.getDistortionParamsSize();

    double maxError = 0.0;
    for (const Observation& obs : makeObservations(cam))
    {
        const Vec4 Xh = obs.X.homogeneous();

        switch (check)
        {
            case Check::TransformProjectWrtPoint3:
            {
                const auto f = [&](const Eigen::VectorXd& X) -> Eigen::VectorXd {
                    return cam.transformProjectDistortionOnly(obs.T, Vec3(X).homogeneous());
                };
                maxError = std::max(maxError, relativeError(cam.getDerivativeTransformProjectWrtPoint3(obs.T, Xh), numericalJacobian(f, obs.X)));
                break;
            }
            case Check::TransformProjectWrtParams:
            {
                std::shared_ptr<IntrinsicScaleOffsetDisto> copy(dynamic_cast<IntrinsicScaleOffsetDisto*>(cam.clone()));
                const auto f = [&](const Eigen::VectorXd& p) -> Eigen::VectorXd {
                    copy->updateFromParams(toStdVector(p));
                    return copy->transformProjectDistortionOnly(obs.T, Xh);
                };
                maxError =
                  std::max(maxError,
                           relativeError(cam.getDerivativeTransformProjectWrtParams(obs.T, Xh), numericalJacobian(f, toVector(cam.getParameters()))));
                break;
            }
            case Check::TransformProjectWrtDistortion:
            {
                std::shared_ptr<IntrinsicScaleOffsetDisto> copy(dynamic_cast<IntrinsicScaleOffsetDisto*>(cam.clone()));
                const auto f = [&](const Eigen::VectorXd& p) -> Eigen::VectorXd {
                    copy->setDistortionParamsFn(p.size(), [&](std::size_t i) { return p(i); });
                    return copy->transformProjectDistortionOnly(obs.T, Xh);
                };
                const Eigen::MatrixXd analytic = cam.getDerivativeTransformProjectWrtDistortion(obs.T, Xh);
                maxError = std::max(maxError, relativeError(analytic, numericalJacobian(f, toVector(cam.getDistortionParams()))));
                break;
            }
            case Check::BackProjectUnitWrtParams:
            {
                maxError =
                  std::max(maxError, relativeError(cam.getDerivativeBackProjectUnitWrtParams(obs.pix), backProjectUnitWrtParams(cam, obs.pix)));
                break;
            }
            case Check::BackProjectUnitWrtDistortion:
            {
                maxError = std::max(
                  maxError, relativeError(cam.getDerivativeBackProjectUnitWrtDistortion(obs.pix), backProjectUnitWrtDistortion(cam, obs.pix)));
                break;
            }
            case Check::ToUnitSphereWrtPoint:
            {
                const Vec2 pt = removeDistortionOnly(cam, cam.ima2cam(obs.pix));
                const auto f = [&](const Eigen::VectorXd& p) -> Eigen::VectorXd { return cam.toUnitSphere(Vec2(p)); };
                maxError = std::max(maxError, relativeError(derivativeToUnitSphereWrtPoint(cam, pt), numericalJacobian(f, pt)));
                break;
            }
            case Check::Cam2ImaIma2Cam:
            {
                const Vec2 pt = cam.ima2cam(obs.pix);
                const auto cam2ima = [&](const Eigen::VectorXd& p) -> Eigen::VectorXd { return cam.cam2ima(Vec2(p)); };
                const auto ima2cam = [&](const Eigen::VectorXd& p) -> Eigen::VectorXd { return cam.ima2cam(Vec2(p)); };
                maxError = std::max(maxError, relativeError(cam.getDerivativeCam2ImaWrtPoint(), numericalJacobian(cam2ima, pt)));
                maxError = std::max(maxError, relativeError(cam.getDerivativeIma2CamWrtPoint(), numericalJacobian(ima2cam, obs.pix)));

                // With respect to the principal point (the offset).
                std::shared_ptr<IntrinsicScaleOffsetDisto> copy(dynamic_cast<IntrinsicScaleOffsetDisto*>(cam.clone()));
                const auto cam2imaOffset = [&](const Eigen::VectorXd& o) -> Eigen::VectorXd {
                    copy->setOffset(Vec2(o));
                    return copy->cam2ima(pt);
                };
                const auto ima2camOffset = [&](const Eigen::VectorXd& o) -> Eigen::VectorXd {
                    copy->setOffset(Vec2(o));
                    return copy->ima2cam(obs.pix);
                };
                maxError =
                  std::max(maxError, relativeError(cam.getDerivativeCam2ImaWrtPrincipalPoint(), numericalJacobian(cam2imaOffset, cam.getOffset())));
                maxError =
                  std::max(maxError, relativeError(cam.getDerivativeIma2CamWrtPrincipalPoint(), numericalJacobian(ima2camOffset, cam.getOffset())));
                copy->setOffset(cam.getOffset());

                // With respect to the scale. Equidistant cameras map the camera plane to pixels with the
                // image circle radius, not the scale, and do not use these two functions.
                if (!isEquidistant(cam.getType()))
                {
                    const auto cam2imaScale = [&](const Eigen::VectorXd& s) -> Eigen::VectorXd {
                        copy->setScale(Vec2(s));
                        return copy->cam2ima(pt);
                    };
                    const auto ima2camScale = [&](const Eigen::VectorXd& s) -> Eigen::VectorXd {
                        copy->setScale(Vec2(s));
                        return copy->ima2cam(obs.pix);
                    };
                    maxError =
                      std::max(maxError, relativeError(cam.getDerivativeCam2ImaWrtScale(pt), numericalJacobian(cam2imaScale, cam.getScale())));
                    maxError =
                      std::max(maxError, relativeError(cam.getDerivativeIma2CamWrtScale(obs.pix), numericalJacobian(ima2camScale, cam.getScale())));
                }
                break;
            }
        }
    }

    return maxError;
}

/// Whether the check makes sense for this camera.
bool isApplicable(const TestCamera& tc, Check check)
{
    switch (check)
    {
        case Check::TransformProjectWrtDistortion:
        case Check::BackProjectUnitWrtDistortion:
            return tc.intrinsic->getDistortionParamsSize() > 0;
        default:
            return true;
    }
}

const double tolerance = 1e-5;

void checkCamera(const TestCamera& tc, Check check)
{
    double error = 0.0;
    try
    {
        error = checkError(tc, check);
    }
    catch (const std::exception& e)
    {
        BOOST_ERROR(tc.name << ": " << toString(check) << " throws: " << e.what());
        return;
    }
    BOOST_CHECK_MESSAGE(error < tolerance, tc.name << ": " << toString(check) << " does not match finite differences, relative error " << error);
}

void runCheck(Check check)
{
    for (const TestCamera& tc : makeTestCameras())
    {
        if (isApplicable(tc, check))
        {
            checkCamera(tc, check);
        }
    }
}

}  // namespace

BOOST_AUTO_TEST_CASE(intrinsicDerivatives_transformProjectWrtPoint3) { runCheck(Check::TransformProjectWrtPoint3); }

BOOST_AUTO_TEST_CASE(intrinsicDerivatives_transformProjectWrtParams) { runCheck(Check::TransformProjectWrtParams); }

BOOST_AUTO_TEST_CASE(intrinsicDerivatives_transformProjectWrtDistortion) { runCheck(Check::TransformProjectWrtDistortion); }

BOOST_AUTO_TEST_CASE(intrinsicDerivatives_backProjectUnitWrtParams) { runCheck(Check::BackProjectUnitWrtParams); }

BOOST_AUTO_TEST_CASE(intrinsicDerivatives_backProjectUnitWrtDistortion) { runCheck(Check::BackProjectUnitWrtDistortion); }

BOOST_AUTO_TEST_CASE(intrinsicDerivatives_toUnitSphereWrtPoint) { runCheck(Check::ToUnitSphereWrtPoint); }

BOOST_AUTO_TEST_CASE(intrinsicDerivatives_cam2imaIma2cam) { runCheck(Check::Cam2ImaIma2Cam); }

BOOST_AUTO_TEST_CASE(intrinsicDerivatives_jacobianSizes)
{
    for (const TestCamera& tc : makeTestCameras())
    {
        const IntrinsicScaleOffsetDisto& cam = *tc.intrinsic;
        const Observation obs = makeObservations(cam).front();
        const Vec4 Xh = obs.X.homogeneous();

        BOOST_TEST_CONTEXT(tc.name)
        {
            BOOST_CHECK_EQUAL(cam.getDerivativeTransformProjectWrtParams(obs.T, Xh).cols(), cam.getParametersSize());
            BOOST_CHECK_EQUAL(cam.getDerivativeBackProjectUnitWrtParams(obs.pix).cols(), cam.getParametersSize());

            // Without distortion parameters, pinhole and equidistant multiply by an empty matrix: only called when there are some.
            if (cam.getDistortionParamsSize() > 0 || isEquirectangular(tc.intrinsicType))
            {
                BOOST_CHECK_EQUAL(cam.getDerivativeTransformProjectWrtDistortion(obs.T, Xh).cols(), cam.getDistortionParamsSize());
                BOOST_CHECK_EQUAL(cam.getDerivativeBackProjectUnitWrtDistortion(obs.pix).cols(), cam.getDistortionParamsSize());
            }
        }
    }
}

// Without Undistortion, the undistorted space functions are the camera functions.
BOOST_AUTO_TEST_CASE(intrinsicDerivatives_undistortedSpaceWithoutUndistortion)
{
    for (const TestCamera& tc : makeTestCameras())
    {
        const IntrinsicScaleOffsetDisto& cam = *tc.intrinsic;
        if (cam.getUndistortion())
        {
            continue;
        }

        for (const Observation& obs : makeObservations(cam))
        {
            const Vec4 Xh = obs.X.homogeneous();
            BOOST_CHECK_SMALL((cam.transformProjectDistortionOnly(obs.T, Xh) - cam.transformProject(obs.T, Xh, true)).norm(), 1e-12);
            BOOST_CHECK_SMALL((cam.backProjectUnitDistortionOnly(obs.pix) - cam.backProjectUnit(obs.pix)).norm(), 1e-12);
        }
    }
}

// With an Undistortion, the camera is the undistorted space camera applied on undistorted pixels.
BOOST_AUTO_TEST_CASE(intrinsicDerivatives_undistortedSpaceWithUndistortion)
{
    for (const TestCamera& tc : makeTestCameras())
    {
        const IntrinsicScaleOffsetDisto& cam = *tc.intrinsic;
        if (!cam.getUndistortion())
        {
            continue;
        }

        BOOST_TEST_CONTEXT(tc.name)
        {
            for (const Observation& obs : makeObservations(cam))
            {
                const Vec4 Xh = obs.X.homogeneous();
                const Vec2 projected = cam.getUndistortion()->inverse(cam.transformProjectDistortionOnly(obs.T, Xh));
                BOOST_CHECK_SMALL((projected - cam.transformProject(obs.T, Xh, true)).norm(), 1e-6);

                const Vec2 undistortedPixel = cam.getUndistortion()->undistort(obs.pix);
                BOOST_CHECK_SMALL((cam.backProjectUnitDistortionOnly(undistortedPixel) - cam.backProjectUnit(obs.pix)).norm(), 1e-9);
            }
        }
    }
}
