// This file is part of the AliceVision project.
// Copyright (c) 2020 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/camera/Distortion.hpp>
#include <aliceVision/camera/DistortionBrown.hpp>
#include <aliceVision/camera/DistortionFisheye.hpp>
#include <aliceVision/camera/DistortionFisheye1.hpp>
#include <aliceVision/camera/DistortionRadial.hpp>

#define BOOST_TEST_MODULE distortion
#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>
#include <aliceVision/unitTest.hpp>

#include <memory>
#include <vector>

using namespace aliceVision;
using namespace aliceVision::camera;

//-----------------
BOOST_AUTO_TEST_CASE(distortion_distort_undistort)
{
    makeRandomOperationsReproducible();

    std::array<std::unique_ptr<Distortion>, 6> distortionsModels;
    distortionsModels[0].reset(new DistortionBrown(-0.25349, 0.11868, -0.00028, 0.00005, 0.0000001));
    distortionsModels[1].reset(new DistortionFisheye(0.02, -0.03, 0.1, -0.2));
    distortionsModels[2].reset(new DistortionFisheye1(0.02));
    distortionsModels[3].reset(new DistortionRadialK1(0.02));
    distortionsModels[4].reset(new DistortionRadialK3(-1.8061369278146561e-01, 1.8759742680633607e-01, -2.5341468279930644e-02));
    distortionsModels[5].reset(new DistortionRadialK3PT(-1.8061369278146561e-01, 1.8759742680633607e-01, -2.5341468279930644e-02));

    const double epsilon = 1e-4;
    const std::size_t numPts{1000};
    for (std::size_t i = 0; i < numPts; ++i)
    {
        // random point in [-lim, lim]x[-lim, lim]
        const double lim{0.8};
        const Vec2 ptImage = lim * Vec2::Random();

        for (const auto& model : distortionsModels)
        {
            const auto distorted = model->addDistortion(ptImage);
            const auto undistorted = model->removeDistortion(distorted);

            // distortion actually happened
            BOOST_CHECK(!(distorted == ptImage));

            EXPECT_MATRIX_CLOSE_FRACTION(ptImage, undistorted, epsilon);
        }
    }
}

namespace {

std::vector<std::unique_ptr<Distortion>> makeDistortionModels()
{
    std::vector<std::unique_ptr<Distortion>> models;
    models.emplace_back(new DistortionBrown(-0.25349, 0.11868, -0.00028, 0.00005, 0.0000001));
    models.emplace_back(new DistortionFisheye(0.02, -0.03, 0.1, -0.2));
    models.emplace_back(new DistortionFisheye1(0.05));
    models.emplace_back(new DistortionRadialK1(0.02));
    models.emplace_back(new DistortionRadialK3(-1.8061369278146561e-01, 1.8759742680633607e-01, -2.5341468279930644e-02));
    models.emplace_back(new DistortionRadialK3PT(-1.8061369278146561e-01, 1.8759742680633607e-01, -2.5341468279930644e-02));
    return models;
}

// Central finite difference of a point map with respect to each distortion parameter.
template<typename F>
Eigen::MatrixXd numericalDerivativeWrtDisto(Distortion& distortion, const Vec2& p, F map)
{
    const std::vector<double> params = distortion.getParameters();
    const double h = 1e-6;

    Eigen::MatrixXd J(2, params.size());
    for (std::size_t i = 0; i < params.size(); ++i)
    {
        std::vector<double> plus = params;
        std::vector<double> minus = params;
        plus[i] += h;
        minus[i] -= h;

        distortion.setParameters(plus);
        const Vec2 a = map(distortion, p);
        distortion.setParameters(minus);
        const Vec2 b = map(distortion, p);

        J.col(i) = (a - b) / (2.0 * h);
    }
    distortion.setParameters(params);

    return J;
}

// Central finite difference of a point map with respect to the point.
template<typename F>
Eigen::Matrix2d numericalDerivativeWrtPt(const Distortion& distortion, const Vec2& p, F map)
{
    const double h = 1e-6;

    Eigen::Matrix2d J;
    for (int i = 0; i < 2; ++i)
    {
        Vec2 plus = p;
        Vec2 minus = p;
        plus(i) += h;
        minus(i) -= h;

        J.col(i) = (map(distortion, plus) - map(distortion, minus)) / (2.0 * h);
    }

    return J;
}

const auto addMap = [](const Distortion& d, const Vec2& p) { return d.addDistortion(p); };

}  // namespace

// The analytic Jacobians are fed straight to Ceres by the pinhole camera's derivative
// functions, so each of them must match a numerical derivative of the map it differentiates.
BOOST_AUTO_TEST_CASE(distortion_addDerivatives)
{
    const std::vector<Vec2> points = {Vec2(0.3, -0.2), Vec2(-0.15, 0.42), Vec2(0.55, 0.1)};

    auto models = makeDistortionModels();
    for (auto& model : models)
    {
        for (const Vec2& p : points)
        {
            BOOST_TEST_CONTEXT("model type " << static_cast<int>(model->getType()) << ", point " << p.transpose())
            {
                EXPECT_MATRIX_NEAR(model->getDerivativeAddDistoWrtDisto(p), numericalDerivativeWrtDisto(*model, p, addMap), 1e-5);
                EXPECT_MATRIX_NEAR(model->getDerivativeAddDistoWrtPt(p), numericalDerivativeWrtPt(*model, p, addMap), 1e-5);
            }
        }
    }
}

// removeDistortion is solved iteratively for most models, so differencing it directly only
// reaches the accuracy of the solver. The Jacobians of the inverse are instead checked
// against the ones implied by differencing addDistortion, which is exact:
// with p = add(q, theta), dq/dp = (dadd/dq)^-1 and dq/dtheta = -(dadd/dq)^-1 * dadd/dtheta.
BOOST_AUTO_TEST_CASE(distortion_removeDerivatives)
{
    const std::vector<Vec2> undistortedPoints = {Vec2(0.3, -0.2), Vec2(-0.15, 0.42), Vec2(0.55, 0.1)};

    auto models = makeDistortionModels();
    for (auto& model : models)
    {
        // DistortionBrown does not implement the inverse Jacobians (they throw).
        if (model->getType() == EDISTORTION::DISTORTION_BROWN)
        {
            continue;
        }

        for (const Vec2& q : undistortedPoints)
        {
            BOOST_TEST_CONTEXT("model type " << static_cast<int>(model->getType()) << ", undistorted point " << q.transpose())
            {
                const Vec2 p = model->addDistortion(q);
                const Eigen::Matrix2d dRemoveDp = numericalDerivativeWrtPt(*model, q, addMap).inverse();
                const Eigen::MatrixXd dRemoveDtheta = -dRemoveDp * numericalDerivativeWrtDisto(*model, q, addMap);

                EXPECT_MATRIX_NEAR(model->getDerivativeRemoveDistoWrtPt(p), dRemoveDp, 1e-5);

                // The radial and fisheye models' getDerivativeRemoveDistoWrtDisto keep the undistorted
                // radius fixed, which is a first-order approximation of this Jacobian, so it is only
                // compared for the model that computes it exactly.
                if (model->getType() == EDISTORTION::DISTORTION_FISHEYE1)
                {
                    EXPECT_MATRIX_NEAR(model->getDerivativeRemoveDistoWrtDisto(p), dRemoveDtheta, 1e-5);
                }
            }
        }
    }
}
