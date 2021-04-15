// This file is part of the AliceVision project.
// Copyright (c) 2021 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/multiview/NViewDataSet.hpp>

#include <aliceVision/multiview/resection/Resection6PSolver.hpp>

#define BOOST_TEST_MODULE ResectionKernel

#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>
#include <aliceVision/unitTest.hpp>
#include <aliceVision/numeric/projection.hpp>

#include <vector>
#include <utility>

using namespace aliceVision;
using namespace aliceVision::multiview;

BOOST_AUTO_TEST_CASE(Resection6PSolver)
{

    const std::size_t nbViews{3};
    const std::size_t nbPoints{30};
    // Suppose a camera with Unit matrix as K
    const NViewDataSet d = NRealisticCamerasRing(
        nbViews, nbPoints, NViewDatasetConfigurator(1, 1, 0, 0, 5, 0));

    // Solve the problem and check that fitted value are good enough
    for(std::size_t camIndex = 0; camIndex < nbViews; ++camIndex)
    {
        const Mat pts2d = d._x[camIndex];
        const Mat pts3d = d._X;

        resection::Resection6PSolver kernel;

        std::vector<robustEstimation::Mat34Model> projMats;

        kernel.solve(pts2d, pts3d, projMats);

        BOOST_CHECK_EQUAL(1, projMats.size());

        // Check that Projection matrix is near to the GT :
        Mat34 gtProjMat = d.P(camIndex).array() / d.P(camIndex).norm();
        Mat34 estProjMat = projMats.at(0).getMatrix().array() / projMats.at(0).getMatrix().norm();
        EXPECT_MATRIX_NEAR(gtProjMat, estProjMat, 1e-8);

        for(Mat::Index i = 0; i < pts2d.cols(); ++i)
        {
            const auto error = (project(estProjMat, Vec3(pts3d.col(i))) - pts2d.col(i)).norm();
            BOOST_CHECK_SMALL(error, 1e-8);
        }
    }
}

BOOST_AUTO_TEST_CASE(Resection6PSolver_weights)
{

    const std::size_t nbViews{3};
    const std::size_t nbPoints{30};
    // Suppose a camera with Unit matrix as K
    const NViewDataSet d = NRealisticCamerasRing(
        nbViews, nbPoints, NViewDatasetConfigurator(1, 1, 0, 0, 5, 0));

    const std::vector<double> weights(nbPoints, 1.);

    // Solve the problem and check that fitted value are good enough
    for(std::size_t camIndex = 0; camIndex < nbViews; ++camIndex)
    {
        const Mat pts2d = d._x[camIndex];
        const Mat pts3d = d._X;

        resection::Resection6PSolver kernel;

        std::vector<robustEstimation::Mat34Model> projMats;

        kernel.solve(pts2d, pts3d, projMats, weights);

        BOOST_CHECK_EQUAL(1, projMats.size());

        // Check that Projection matrix is near to the GT :
        Mat34 gtProjMat = d.P(camIndex).array() / d.P(camIndex).norm();
        Mat34 estProjMat = projMats.at(0).getMatrix().array() / projMats.at(0).getMatrix().norm();
        EXPECT_MATRIX_NEAR(gtProjMat, estProjMat, 1e-8);

        for(Mat::Index i = 0; i < pts2d.cols(); ++i)
        {
            const auto error = (project(estProjMat, Vec3(pts3d.col(i))) - pts2d.col(i)).norm();
            BOOST_CHECK_SMALL(error, 1e-8);
        }
    }
}

using VectorOfPair = std::vector<std::pair<Mat::Index, Mat::Index>>;

bool isIndexInVector(const VectorOfPair& vec, VectorOfPair::value_type::first_type val)
{
    for(const auto v : vec)
    {
        if(val == v.first || val == v.second)
        {
            return true;
        }
    }
    return false;
}

BOOST_AUTO_TEST_CASE(Resection6PSolver_weights_outliers)
{
    // test the DLT solution using weights and outliers
    // outliers will be given a weight of 0 to robustly estimate the pose

    const std::size_t nbViews{3};
    const std::size_t nbPoints{30};
    // Suppose a camera with Unit matrix as K
    const NViewDataSet d = NRealisticCamerasRing(
        nbViews, nbPoints, NViewDatasetConfigurator(1, 1, 0, 0, 5, 0));

    std::vector<double> weights(nbPoints, 1.);

    Mat pts3d = d._X;

    // generate some outliers, simply swap the position of some 3d points
    const VectorOfPair idx2swap{ {3, 10}, {15, 23} };

    for(const auto idx : idx2swap)
    {
        // swap the relevant columns
        pts3d.col(idx.first).swap(pts3d.col(idx.second));
        // set the weights to 0
        weights[idx.first] = .0;
        weights[idx.second] = .0;
    }

    // Solve the problem and check that fitted value are good enough
    for(std::size_t camIndex = 0; camIndex < nbViews; ++camIndex)
    {
        const Mat pts2d = d._x[camIndex];

        resection::Resection6PSolver kernel;

        std::vector<robustEstimation::Mat34Model> projMats;

        kernel.solve(pts2d, pts3d, projMats, weights);

        BOOST_CHECK_EQUAL(1, projMats.size());

        // Check that Projection matrix is near to the GT :
        Mat34 gtProjMat = d.P(camIndex).array() / d.P(camIndex).norm();
        Mat34 estProjMat = projMats.at(0).getMatrix().array() / projMats.at(0).getMatrix().norm();
        EXPECT_MATRIX_NEAR(gtProjMat, estProjMat, 1e-8);

        for(Mat::Index i = 0; i < pts2d.cols(); ++i)
        {
            const auto error = (project(estProjMat, Vec3(pts3d.col(i))) - pts2d.col(i)).norm();
            if(isIndexInVector(idx2swap, i))
            {
                // if it is an outlier
                BOOST_CHECK_GT(error, 1e-3);
            }
            else
            {
                BOOST_CHECK_SMALL(error, 1e-8);
            }
        }

        // test that using equal weights it fails
        std::vector<double> weightsEquals(nbPoints, 1.);
        projMats.clear();
        kernel.solve(pts2d, pts3d, projMats, weightsEquals);

        BOOST_CHECK(projMats.empty());
    }
}

BOOST_AUTO_TEST_CASE(Resection6PSolver_weights_only6)
{
    // test the DLT solution using 6 points and some less reliable points
    // for which the weight is set to 0.5

    const std::size_t nbViews{3};
    const std::size_t nbPoints{6};
    // Suppose a camera with Unit matrix as K
    const NViewDataSet d = NRealisticCamerasRing(
        nbViews, nbPoints, NViewDatasetConfigurator(1, 1, 0, 0, 5, 0));

    std::vector<double> weights(nbPoints, 1.);

    Mat pts3d = d._X;

    // chose a couple of points with a weigth of 0.5
    weights[1] = .5;
    weights[2] = .5;


    // Solve the problem and check that fitted value are good enough
    for(std::size_t camIndex = 0; camIndex < nbViews; ++camIndex)
    {
        const Mat pts2d = d._x[camIndex];

        resection::Resection6PSolver kernel;

        std::vector<robustEstimation::Mat34Model> projMats;

        kernel.solve(pts2d, pts3d, projMats, weights);

        BOOST_CHECK_EQUAL(1, projMats.size());

        // Check that Projection matrix is near to the GT :
        Mat34 gtProjMat = d.P(camIndex).array() / d.P(camIndex).norm();
        Mat34 estProjMat = projMats.at(0).getMatrix().array() / projMats.at(0).getMatrix().norm();
        EXPECT_MATRIX_NEAR(gtProjMat, estProjMat, 1e-8);

        for(Mat::Index i = 0; i < pts2d.cols(); ++i)
        {
            const auto error = (project(estProjMat, Vec3(pts3d.col(i))) - pts2d.col(i)).norm();
            // given that all the associations 2d-3d are "perfect", even the ones with a smaller
            // weight should have a perfect error
            BOOST_CHECK_SMALL(error, 1e-8);
        }
    }
}
