// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/camera/Undistortion.hpp>
#include <aliceVision/camera/Undistortion3DEA4.hpp>
#include <aliceVision/camera/Undistortion3DEClassicLD.hpp>
#include <aliceVision/camera/Undistortion3DERadial4.hpp>
#include <aliceVision/camera/UndistortionRadial.hpp>

#define BOOST_TEST_MODULE undistortion
#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>
#include <aliceVision/unitTest.hpp>

#include <memory>
#include <vector>

using namespace aliceVision;
using namespace aliceVision::camera;

namespace {

// Central finite difference of undistortNormalized with respect to each parameter.
Eigen::Matrix<double, 2, Eigen::Dynamic> numericalDerivativeWrtParameters(Undistortion& undistortion, const Vec2& pt)
{
    const std::vector<double> params = undistortion.getParameters();
    const double h = 1e-6;

    Eigen::Matrix<double, 2, Eigen::Dynamic> J(2, params.size());
    for (std::size_t i = 0; i < params.size(); ++i)
    {
        std::vector<double> plus = params;
        std::vector<double> minus = params;
        plus[i] += h;
        minus[i] -= h;

        undistortion.setParameters(plus);
        const Vec2 a = undistortion.undistortNormalized(pt);
        undistortion.setParameters(minus);
        const Vec2 b = undistortion.undistortNormalized(pt);

        J.col(i) = (a - b) / (2.0 * h);
    }
    undistortion.setParameters(params);

    return J;
}

}  // namespace

BOOST_AUTO_TEST_CASE(undistortion_derivativeWrtParameters)
{
    std::vector<std::unique_ptr<Undistortion>> models;
    models.emplace_back(new UndistortionRadialK3(1920, 1080));
    models[0]->setParameters({0.1, -0.05, 0.02});
    models.emplace_back(new Undistortion3DEClassicLD(1920, 1080));
    models[1]->setParameters({0.1, 1.1, 0.02, -0.03, 0.04});
    models.emplace_back(new Undistortion3DEAnamorphic4(1920, 1080));
    models[2]->setParameters({0.1, 0.02, -0.03, 0.04, 0.01, -0.02, 0.03, 0.005, -0.006, 0.007, 0.2, 1.1, 0.9});
    models.emplace_back(new Undistortion3DERadial4(1920, 1080));
    models[3]->setParameters({0.1, 0.01, -0.02, 0.03, 0.004, -0.005, 0.3, 0.05});

    const std::vector<Vec2> points = {Vec2(0.3, -0.2), Vec2(-0.25, 0.1), Vec2(0.05, 0.4)};

    for (auto& model : models)
    {
        for (const Vec2& pt : points)
        {
            const Eigen::Matrix<double, 2, Eigen::Dynamic> analytic = model->getDerivativeUndistortNormalizedwrtParameters(pt);
            const Eigen::Matrix<double, 2, Eigen::Dynamic> numeric = numericalDerivativeWrtParameters(*model, pt);

            EXPECT_MATRIX_NEAR(analytic, numeric, 1e-6);
        }
    }
}
