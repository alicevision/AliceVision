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
#include <aliceVision/camera/camera_test_common.hpp>

#define BOOST_TEST_MODULE undistortion
#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>
#include <aliceVision/unitTest.hpp>

#include <memory>
#include <vector>

using namespace aliceVision;
using namespace aliceVision::camera;
using namespace aliceVision::camera::test;

namespace {

/// One instance of every undistortion model, with non-trivial parameters.
std::vector<std::shared_ptr<Undistortion>> makeModels(int width = 1920, int height = 1080)
{
    std::vector<std::shared_ptr<Undistortion>> models;
    for (EUNDISTORTION type : allUndistortionTypes())
    {
        if (type == EUNDISTORTION::UNDISTORTION_NONE)
        {
            continue;
        }
        std::shared_ptr<Undistortion> model = createUndistortion(type, width, height);
        model->setParameters(undistortionTestParameters(type));
        models.push_back(model);
    }
    return models;
}

/// Grid of pixels covering the central 80% of the image.
std::vector<Vec2> pixelGrid(const Undistortion& undistortion)
{
    std::vector<Vec2> pixels;
    const Vec2 size = undistortion.getSize();
    for (int j = 0; j <= 6; ++j)
    {
        for (int i = 0; i <= 6; ++i)
        {
            pixels.push_back(Vec2((0.1 + 0.8 * i / 6.0) * size(0) + 0.37, (0.1 + 0.8 * j / 6.0) * size(1) - 0.21));
        }
    }
    return pixels;
}

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
    const std::vector<Vec2> points = {Vec2(0.3, -0.2), Vec2(-0.25, 0.1), Vec2(0.05, 0.4)};

    for (auto& model : makeModels())
    {
        for (const Vec2& pt : points)
        {
            const Eigen::Matrix<double, 2, Eigen::Dynamic> analytic = model->getDerivativeUndistortNormalizedwrtParameters(pt);
            const Eigen::Matrix<double, 2, Eigen::Dynamic> numeric = numericalDerivativeWrtParameters(*model, pt);

            EXPECT_MATRIX_NEAR(analytic, numeric, 1e-6);
        }
    }
}

BOOST_AUTO_TEST_CASE(undistortion_derivativeWrtPoint)
{
    const std::vector<Vec2> points = {Vec2(0.3, -0.2), Vec2(-0.25, 0.1), Vec2(0.05, 0.4)};

    for (auto& model : makeModels())
    {
        BOOST_TEST_CONTEXT(EUNDISTORTION_enumToString(model->getType()))
        {
            const auto f = [&](const Eigen::VectorXd& p) -> Eigen::VectorXd { return model->undistortNormalized(Vec2(p)); };
            for (const Vec2& pt : points)
            {
                const Eigen::Matrix2d analytic = model->getDerivativeUndistortNormalizedwrtPoint(pt);
                const Eigen::MatrixXd numeric = numericalJacobian(f, pt);
                EXPECT_MATRIX_NEAR(analytic, numeric, 1e-6);
            }
        }
    }
}

// The pixel level derivatives wrap the normalized ones with the image center, offset and diagonal.
BOOST_AUTO_TEST_CASE(undistortion_pixelDerivatives)
{
    for (auto& model : makeModels())
    {
        BOOST_TEST_CONTEXT(EUNDISTORTION_enumToString(model->getType()))
        {
            model->setOffset(Vec2(12.0, -7.0));
            model->setPixelAspectRatio(1.2);

            const Eigen::VectorXd params = toVector(model->getParameters());
            const auto wrtParams = [&](const Eigen::VectorXd& p, const Vec2& pix) -> Eigen::VectorXd {
                model->setParameters(toStdVector(p));
                const Vec2 result = model->undistort(pix);
                model->setParameters(toStdVector(params));
                return result;
            };
            const Vec2 offset = model->getOffset();
            const auto wrtOffset = [&](const Eigen::VectorXd& o, const Vec2& pix) -> Eigen::VectorXd {
                model->setOffset(Vec2(o));
                const Vec2 result = model->undistort(pix);
                model->setOffset(offset);
                return result;
            };

            for (const Vec2& pix : pixelGrid(*model))
            {
                const Eigen::MatrixXd numericParams = numericalJacobian([&](const Eigen::VectorXd& p) { return wrtParams(p, pix); }, params);
                const Eigen::MatrixXd analyticParams = model->getDerivativeUndistortWrtParameters(pix);
                BOOST_CHECK_SMALL(maxAbsDiff(analyticParams, numericParams) / std::max(1.0, numericParams.cwiseAbs().maxCoeff()), 1e-6);

                const Eigen::MatrixXd numericOffset = numericalJacobian([&](const Eigen::VectorXd& o) { return wrtOffset(o, pix); }, offset, 1e-4);
                const Eigen::MatrixXd analyticOffset = model->getDerivativeUndistortWrtOffset(pix);
                BOOST_CHECK_SMALL(maxAbsDiff(analyticOffset, numericOffset), 1e-6);
            }
        }
    }
}

// inverse() adds the distortion back: undistort(inverse(p)) = p and inverse(undistort(p)) = p.
BOOST_AUTO_TEST_CASE(undistortion_undistortInverse)
{
    struct Setup
    {
        std::string name;
        Vec2 offset;
        double pixelAspectRatio;
        bool desqueezed;
    };
    const std::vector<Setup> setups = {
      {"centered", Vec2(0.0, 0.0), 1.0, false},
      {"offset", Vec2(15.0, -9.0), 1.0, false},
      {"squeezed", Vec2(5.0, 3.0), 2.0, false},
      {"desqueezed", Vec2(5.0, 3.0), 2.0, true},
    };

    for (const Setup& setup : setups)
    {
        for (auto& model : makeModels())
        {
            BOOST_TEST_CONTEXT(EUNDISTORTION_enumToString(model->getType()) << " " << setup.name)
            {
                model->setPixelAspectRatio(setup.pixelAspectRatio);
                model->setDesqueezed(setup.desqueezed);
                model->setOffset(setup.offset);

                double maxError = 0.0;
                double maxDisplacement = 0.0;
                for (const Vec2& pix : pixelGrid(*model))
                {
                    const Vec2 undistorted = model->undistort(pix);
                    maxDisplacement = std::max(maxDisplacement, (undistorted - pix).norm());
                    maxError = std::max(maxError, (model->inverse(undistorted) - pix).norm());
                    maxError = std::max(maxError, (model->undistort(model->inverse(pix)) - pix).norm());
                }

                BOOST_CHECK_SMALL(maxError, 1e-3);
                BOOST_CHECK_GT(maxDisplacement, 1.0);

                // The distortion center is not moved.
                EXPECT_MATRIX_NEAR(model->undistort(model->getCenter()), model->getCenter(), 1e-6);
            }
        }
    }
}

BOOST_AUTO_TEST_CASE(undistortion_geometry)
{
    UndistortionRadialK3 model(1920, 1080);

    EXPECT_MATRIX_NEAR(model.getSize(), Vec2(1920, 1080), 1e-12);
    EXPECT_MATRIX_NEAR(model.getCenter(), Vec2(960, 540), 1e-12);
    BOOST_CHECK_CLOSE(model.getDiagonal(), 0.5 * std::hypot(1920.0, 1080.0), 1e-9);
    BOOST_CHECK_EQUAL(model.getPixelAspectRatio(), 1.0);
    BOOST_CHECK(!model.isDesqueezed());

    // The offset moves the distortion center.
    model.setOffset(Vec2(10.0, -4.0));
    EXPECT_MATRIX_NEAR(model.getOffset(), Vec2(10.0, -4.0), 1e-12);
    EXPECT_MATRIX_NEAR(model.getCenter(), Vec2(970, 536), 1e-12);
    const Vec2 scaledOffset = Vec2(10.0, -4.0) / model.getDiagonal();
    EXPECT_MATRIX_NEAR(model.getScaledOffset(), scaledOffset, 1e-12);

    // A squeezed image is normalized as if it was desqueezed in y.
    model.setPixelAspectRatio(2.0);
    BOOST_CHECK_CLOSE(model.getDiagonal(), 0.5 * std::hypot(1920.0, 540.0), 1e-9);
    model.setDesqueezed(true);
    BOOST_CHECK(model.isDesqueezed());
    BOOST_CHECK_CLOSE(model.getDiagonal(), 0.5 * std::hypot(1920.0, 1080.0), 1e-9);
    model.setDesqueezed(false);

    model.setSize(1000, 500);
    EXPECT_MATRIX_NEAR(model.getSize(), Vec2(1000, 500), 1e-12);
    BOOST_CHECK_CLOSE(model.getDiagonal(), 0.5 * std::hypot(1000.0, 250.0), 1e-9);

    // An explicit diagonal, used for plates with a different size than the lens grid.
    model.setDiagonal(800.0);
    BOOST_CHECK_EQUAL(model.getDiagonal(), 800.0);

    BOOST_CHECK(!model.isLocked());
    model.setLocked(true);
    BOOST_CHECK(model.isLocked());
}

// Default parameters give no distortion.
BOOST_AUTO_TEST_CASE(undistortion_identity)
{
    for (EUNDISTORTION type : allUndistortionTypes())
    {
        if (type == EUNDISTORTION::UNDISTORTION_NONE)
        {
            continue;
        }

        BOOST_TEST_CONTEXT(EUNDISTORTION_enumToString(type))
        {
            std::shared_ptr<Undistortion> model = createUndistortion(type, 1920, 1080);
            for (const Vec2& pix : pixelGrid(*model))
            {
                EXPECT_MATRIX_NEAR(model->undistort(pix), pix, 1e-9);
                EXPECT_MATRIX_NEAR(model->inverse(pix), pix, 1e-6);
            }
        }
    }
}

// The pixel level point Jacobian wraps the normalized one with the center, offset and diagonal.
BOOST_AUTO_TEST_CASE(undistortion_derivativeWrtPixel)
{
    struct Setup
    {
        std::string name;
        Vec2 offset;
        double pixelAspectRatio;
        bool desqueezed;
    };
    const std::vector<Setup> setups = {
      {"centered", Vec2(0.0, 0.0), 1.0, false},
      {"offset", Vec2(15.0, -9.0), 1.0, false},
      {"squeezed", Vec2(5.0, 3.0), 2.0, false},
      {"desqueezed", Vec2(5.0, 3.0), 2.0, true},
    };

    for (const Setup& setup : setups)
    {
        for (auto& model : makeModels())
        {
            BOOST_TEST_CONTEXT(EUNDISTORTION_enumToString(model->getType()) << " " << setup.name)
            {
                model->setPixelAspectRatio(setup.pixelAspectRatio);
                model->setDesqueezed(setup.desqueezed);
                model->setOffset(setup.offset);

                const auto f = [&](const Eigen::VectorXd& p) -> Eigen::VectorXd { return model->undistort(Vec2(p)); };
                for (const Vec2& pix : pixelGrid(*model))
                {
                    const Eigen::MatrixXd numeric = numericalJacobian(f, pix, 1e-4);
                    BOOST_CHECK_SMALL(maxAbsDiff(model->getDerivativeUndistortWrtPoint(pix), numeric), 1e-6);
                }

                // Default parameters: identity.
                std::shared_ptr<Undistortion> identity = createUndistortion(model->getType(), 1920, 1080);
                EXPECT_MATRIX_NEAR(identity->getDerivativeUndistortWrtPoint(Vec2(300.0, 700.0)), Eigen::Matrix2d::Identity(), 1e-9);
            }
        }
    }
}
