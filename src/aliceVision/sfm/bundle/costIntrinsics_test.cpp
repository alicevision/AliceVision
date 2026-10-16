// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfm/bundle/costfunctions/intrinsicsLift.hpp>
#include <aliceVision/sfm/bundle/costfunctions/intrinsicsProject.hpp>
#include <aliceVision/camera/camera_test_common.hpp>

#define BOOST_TEST_MODULE costIntrinsics

#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>
#include <aliceVision/unitTest.hpp>

using namespace aliceVision;
using namespace aliceVision::camera;
using namespace aliceVision::camera::test;
using namespace aliceVision::sfm;

namespace {

// A distortion, an undistortion, both, none.
// The distortion is Fisheye1, whose removeDistortion is in closed form: the other models solve it
// iteratively, which makes finite differences of Evaluate (lift, exact observation) only as accurate
// as the solver. Their Jacobians are checked against exact references in the camera tests.
std::vector<TestCamera> makeCameras()
{
    return {makeTestCamera(EINTRINSIC::PINHOLE_CAMERA, EDISTORTION::DISTORTION_FISHEYE1, EUNDISTORTION::UNDISTORTION_NONE),
            makeTestCamera(EINTRINSIC::PINHOLE_CAMERA, EDISTORTION::DISTORTION_NONE, EUNDISTORTION::UNDISTORTION_3DERADIAL4),
            makeTestCamera(EINTRINSIC::PINHOLE_CAMERA, EDISTORTION::DISTORTION_FISHEYE1, EUNDISTORTION::UNDISTORTION_3DERADIAL4),
            makeTestCamera(EINTRINSIC::PINHOLE_CAMERA, EDISTORTION::DISTORTION_NONE, EUNDISTORTION::UNDISTORTION_NONE)};
}

/// The parameter blocks of a cost function, which reads the intrinsic object and the point block.
struct Blocks
{
    std::vector<double> intrinsic;
    std::vector<double> distortion;
    std::vector<double> point;
};

Blocks makeBlocks(const IntrinsicScaleOffsetDisto& cam, const Vec3& point)
{
    Blocks blocks;
    blocks.intrinsic = cam.getParameters();
    blocks.distortion = cam.getDistortion() ? cam.getDistortionParams() : std::vector<double>{0.0};
    blocks.point = {point(0), point(1), point(2)};
    return blocks;
}

/// Evaluate the cost function, with the intrinsic object updated from the blocks as the bundle adjustment does.
Eigen::VectorXd evaluate(const ceres::CostFunction& cost,
                         IntrinsicScaleOffsetDisto& cam,
                         const Blocks& blocks,
                         std::vector<Eigen::MatrixXd>* jacobians = nullptr)
{
    cam.updateFromParams(blocks.intrinsic);
    if (cam.getDistortion())
    {
        cam.setDistortionParams(blocks.distortion);
    }

    std::vector<const double*> parameters = {blocks.intrinsic.data(), blocks.distortion.data(), blocks.point.data()};
    const std::vector<int32_t>& sizes = cost.parameter_block_sizes();
    parameters.resize(sizes.size());

    Eigen::VectorXd residuals(cost.num_residuals());
    if (!jacobians)
    {
        cost.Evaluate(parameters.data(), residuals.data(), nullptr);
        return residuals;
    }

    std::vector<std::vector<double>> storage(sizes.size());
    std::vector<double*> jacobianPtrs(sizes.size());
    for (std::size_t i = 0; i < sizes.size(); ++i)
    {
        storage[i].assign(cost.num_residuals() * sizes[i], 0.0);
        jacobianPtrs[i] = storage[i].data();
    }
    cost.Evaluate(parameters.data(), residuals.data(), jacobianPtrs.data());

    jacobians->clear();
    for (std::size_t i = 0; i < sizes.size(); ++i)
    {
        jacobians->push_back(
          Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(storage[i].data(), cost.num_residuals(), sizes[i]));
    }
    return residuals;
}

/// Central finite difference of the cost function with respect to one parameter block.
Eigen::MatrixXd numericalJacobian(const ceres::CostFunction& cost, IntrinsicScaleOffsetDisto& cam, const Blocks& blocks, int block)
{
    const double h = 1e-6;
    std::vector<double> Blocks::* members[] = {&Blocks::intrinsic, &Blocks::distortion, &Blocks::point};
    const std::size_t size = (blocks.*members[block]).size();

    Eigen::MatrixXd J(cost.num_residuals(), size);
    for (std::size_t i = 0; i < size; ++i)
    {
        Blocks plus = blocks;
        Blocks minus = blocks;
        (plus.*members[block])[i] += h;
        (minus.*members[block])[i] -= h;
        J.col(i) = (evaluate(cost, cam, plus) - evaluate(cost, cam, minus)) / (2.0 * h);
    }
    evaluate(cost, cam, blocks);
    return J;
}

double relativeError(const Eigen::MatrixXd& analytic, const Eigen::MatrixXd& numeric)
{ return maxAbsDiff(analytic, numeric) / std::max(1.0, numeric.cwiseAbs().maxCoeff()); }

}  // namespace

BOOST_AUTO_TEST_CASE(costIntrinsics_projectJacobians)
{
    for (const TestCamera& tc : makeCameras())
    {
        BOOST_TEST_CONTEXT(tc.name)
        {
            std::shared_ptr<IntrinsicScaleOffsetDisto> cam(dynamic_cast<IntrinsicScaleOffsetDisto*>(tc.intrinsic->clone()));
            for (const Vec2& pix : samplePixels(*cam, 5))
            {
                const Vec3 point = 4.0 * cam->backProjectUnit(pix);
                // Observation slightly off the projection, with a feature scale.
                const sfmData::Observation observation(pix + Vec2(0.7, -0.4), 0, 2.0);
                const CostIntrinsicsProject cost(observation, cam);

                const Blocks blocks = makeBlocks(*cam, point);
                std::vector<Eigen::MatrixXd> analytic;
                evaluate(cost, *cam, blocks, &analytic);

                BOOST_CHECK_SMALL(relativeError(analytic[0], numericalJacobian(cost, *cam, blocks, 0)), 1e-5);
                if (cam->getDistortion())
                {
                    BOOST_CHECK_SMALL(relativeError(analytic[1], numericalJacobian(cost, *cam, blocks, 1)), 1e-5);
                }
                BOOST_CHECK_SMALL(relativeError(analytic[2], numericalJacobian(cost, *cam, blocks, 2)), 1e-5);
            }
        }
    }
}

BOOST_AUTO_TEST_CASE(costIntrinsics_liftJacobians)
{
    for (const TestCamera& tc : makeCameras())
    {
        BOOST_TEST_CONTEXT(tc.name)
        {
            std::shared_ptr<IntrinsicScaleOffsetDisto> cam(dynamic_cast<IntrinsicScaleOffsetDisto*>(tc.intrinsic->clone()));
            for (const Vec2& pix : samplePixels(*cam, 5))
            {
                const CostIntrinsicsLift cost(pix, cam);

                // The lift has no point block.
                const Blocks blocks = makeBlocks(*cam, Vec3::Zero());
                std::vector<Eigen::MatrixXd> analytic;
                const Eigen::VectorXd ray = evaluate(cost, *cam, blocks, &analytic);

                // The lifted ray is the camera back-projection of the raw pixel.
                BOOST_CHECK_SMALL((Vec3(ray) - cam->backProjectUnit(pix)).norm(), 1e-9);

                BOOST_CHECK_SMALL(relativeError(analytic[0], numericalJacobian(cost, *cam, blocks, 0)), 1e-5);
                if (cam->getDistortion())
                {
                    BOOST_CHECK_SMALL(relativeError(analytic[1], numericalJacobian(cost, *cam, blocks, 1)), 1e-5);
                }
            }
        }
    }
}

// The residual approximates the raw pixel error, whatever the camera.
BOOST_AUTO_TEST_CASE(costIntrinsics_projectResidualInPixels)
{
    for (const TestCamera& tc : makeCameras())
    {
        BOOST_TEST_CONTEXT(tc.name)
        {
            std::shared_ptr<IntrinsicScaleOffsetDisto> cam(dynamic_cast<IntrinsicScaleOffsetDisto*>(tc.intrinsic->clone()));
            const std::vector<Vec2> pixels = samplePixels(*cam, 5);

            // Including the image center and a pixel near the border.
            std::vector<Vec2> tested = pixels;
            tested.push_back(Vec2(cam->w() * 0.5, cam->h() * 0.5));
            tested.push_back(Vec2(cam->w() * 0.97, cam->h() * 0.95));

            for (const Vec2& pix : tested)
            {
                const Vec3 point = 4.0 * cam->backProjectUnit(pix);
                const Blocks blocks = makeBlocks(*cam, point);

                const CostIntrinsicsProject exact(sfmData::Observation(pix, 0, 1.0), cam);
                const Eigen::VectorXd zero = evaluate(exact, *cam, blocks);
                BOOST_CHECK(zero.allFinite());
                BOOST_CHECK_SMALL(zero.norm(), 1e-6);

                // residual = projection - observation, divided by the feature scale.
                const Vec2 d(0.3, -0.2);
                const CostIntrinsicsProject offset(sfmData::Observation(pix + d, 0, 2.0), cam);
                const Eigen::VectorXd residual = evaluate(offset, *cam, blocks);
                BOOST_CHECK_SMALL((Vec2(residual) + d / 2.0).norm(), 1e-3);
            }
        }
    }
}

// An observation where the undistortion folds (singular Jacobian) must not make the residual
// non-finite or huge: Ceres would fail the whole solve, or the observation would dominate the cost.
BOOST_AUTO_TEST_CASE(costIntrinsics_projectDegenerateUndistortion)
{
    const TestCamera tc = makeTestCamera(EINTRINSIC::PINHOLE_CAMERA, EDISTORTION::DISTORTION_NONE, EUNDISTORTION::UNDISTORTION_RADIALK3);
    std::shared_ptr<IntrinsicScaleOffsetDisto> cam(dynamic_cast<IntrinsicScaleOffsetDisto*>(tc.intrinsic->clone()));

    // r_u = r (1 + k1 r^2) folds where 1 + 3 k1 r^2 = 0: at normalized radius 0.8 for k1 = -1 / (3 * 0.64).
    std::shared_ptr<Undistortion> undistortion = cam->getUndistortion();
    undistortion->setParameters({-1.0 / (3.0 * 0.64), 0.0, 0.0});
    const Vec2 fold = undistortion->getCenter() + Vec2(0.8 * undistortion->getDiagonal(), 0.0);
    BOOST_REQUIRE_SMALL(undistortion->getDerivativeUndistortWrtPoint(fold).determinant(), 1e-9);

    const Vec3 point = 4.0 * cam->backProjectUnit(Vec2(cam->w() * 0.5 + 10.0, cam->h() * 0.5));
    const CostIntrinsicsProject cost(sfmData::Observation(fold, 0, 1.0), cam);

    const Blocks blocks = makeBlocks(*cam, point);
    std::vector<Eigen::MatrixXd> jacobians;
    const Eigen::VectorXd residual = evaluate(cost, *cam, blocks, &jacobians);

    BOOST_CHECK(residual.allFinite());
    // The residual stays of the order of the raw pixel error, instead of being amplified by the fold.
    const double rawError = (cam->transformProject(Eigen::Matrix4d::Identity(), point.homogeneous(), true) - fold).norm();
    BOOST_CHECK_LT(residual.norm(), 10.0 * rawError);
    for (const Eigen::MatrixXd& J : jacobians)
    {
        BOOST_CHECK(J.allFinite());
    }
}
