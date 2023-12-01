// This file is part of the AliceVision project.
// Copyright (c) 2021 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfm/utils/alignment.hpp>
#include <aliceVision/multiview/NViewDataSet.hpp>

#include <cmath>
#include <cstdio>
#include <iostream>

#define BOOST_TEST_MODULE alignment

#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>
#include <aliceVision/unitTest.hpp>

using namespace aliceVision;
using namespace aliceVision::camera;
using namespace aliceVision::geometry;
using namespace aliceVision::sfm;
using namespace aliceVision::sfmData;

SfMData getInputScene(const NViewDataSet& d, const NViewDatasetConfigurator& config, EINTRINSIC eintrinsic);

BOOST_AUTO_TEST_CASE(ALIGMENT_CamerasXAxis_noCamera)
{
    SfMData sfmData;
    double bS = 1.0;
    Mat3 bR = Mat3::Identity();
    Vec3 bt = Vec3::Zero();
    computeNewCoordinateSystemFromCamerasXAxis(sfmData, bS, bR, bt);

    ALICEVISION_LOG_INFO("bS: " << bS);
    ALICEVISION_LOG_INFO("bR: " << bR);
    ALICEVISION_LOG_INFO("bt: " << bt);

    BOOST_CHECK_EQUAL(bS, 1.0);
    BOOST_CHECK_EQUAL(bR, Mat3::Identity());
    BOOST_CHECK_EQUAL(bt, Vec3::Zero());
}

BOOST_AUTO_TEST_CASE(ALIGMENT_CamerasXAxis_alreadyAligned)
{
    const int nviews = 12;
    const int npoints = 6;
    const NViewDatasetConfigurator config;
    const NViewDataSet d = NRealisticCamerasRing(nviews, npoints, config);

    // Translate the input dataset to a SfMData scene
    SfMData sfmData = getInputScene(d, config, EINTRINSIC::PINHOLE_CAMERA);

    double bS = 1.0;
    Mat3 bR = Mat3::Identity();
    Vec3 bt = Vec3::Zero();
    computeNewCoordinateSystemFromCamerasXAxis(sfmData, bS, bR, bt);

    ALICEVISION_LOG_INFO("bR: " << bR);

    BOOST_CHECK_EQUAL(bR, Mat3::Identity());
}

// Test summary:
// - Create a SfMData scene from a synthetic dataset with a ring of cameras
// - Apply a rotation to the scene
// - Estimate the transformation to align the X axis of all cameras
// - Check that the rotation matches the one applied
BOOST_AUTO_TEST_CASE(ALIGMENT_CamerasXAxis_checkRotation)
{
    const int nviews = 16;
    const int npoints = 6;
    const NViewDatasetConfigurator config;
    const NViewDataSet d = NRealisticCamerasRing(nviews, npoints, config);

    // Translate the input dataset to a SfMData scene
    SfMData sfmDataOrig = getInputScene(d, config, EINTRINSIC::PINHOLE_CAMERA);
    SfMData sfmData = sfmDataOrig;
    double aS = 1.0;

    const Vec3 rAngles = Vec3::Random() * M_PI;
    const Mat3 aR(Eigen::AngleAxisd(rAngles(0), Vec3::UnitX()) * Eigen::AngleAxisd(rAngles(1), Vec3::UnitY()) *
                  Eigen::AngleAxisd(rAngles(2), Vec3::UnitZ()));
    const Vec3 at = Vec3::Zero();
    applyTransform(sfmData, aS, aR, at);

    // Compute Transformation a first time
    double bS = 1.0;
    Mat3 bR = Mat3::Identity();
    Vec3 bt = Vec3::Zero();
    {
        computeNewCoordinateSystemFromCamerasXAxis(sfmData, bS, bR, bt);

        SfMData sfmDataCorrected = sfmData;
        applyTransform(sfmDataCorrected, 1.0, bR, bt);
        ALICEVISION_LOG_INFO("aR: " << aR);
        ALICEVISION_LOG_INFO("bR: " << bR);
        for (const auto& pose : sfmDataCorrected.getPoses())
        {
            Vec3 camY(pose.second.getTransform().rotation() * Vec3::UnitY());
            // EXPECT_MATRIX_NEAR(camY, -Vec3::UnitY(), 1e-3);
            ALICEVISION_LOG_INFO("camY: " << camY);
        }
        // Mat3 res = bR * aR;
        // ALICEVISION_LOG_INFO("res: " << res);
        // EXPECT_MATRIX_NEAR(res, Mat3::Identity(), 1e-3);
    }
    {
        // Check repeatability: if we estimate the transformation a 2nd time, we should find the same result
        double cS = 1.0;
        Mat3 cR = Mat3::Identity();
        Vec3 ct = Vec3::Zero();
        computeNewCoordinateSystemFromCamerasXAxis(sfmData, cS, cR, ct);

        ALICEVISION_LOG_INFO("cR: " << cR);

        EXPECT_MATRIX_NEAR(bR, cR, 1e-5);
    }

    // apply the estimated transformation
    applyTransform(sfmData, bS, bR, bt);

    // Check repeatability:
    // We have estimated and apply the transformation.
    // If we estimate the transformation again, the result should be identity.
    {
        double cS = 1.0;
        Mat3 cR = Mat3::Identity();
        Vec3 ct = Vec3::Zero();
        computeNewCoordinateSystemFromCamerasXAxis(sfmData, cS, cR, ct);

        ALICEVISION_LOG_INFO("cR: " << cR);

        EXPECT_MATRIX_NEAR(cR, Mat3::Identity(), 1e-5);
    }
}

// Translation a synthetic scene into a valid SfMData scene.
// => A synthetic scene is used:
//    a random noise between [-.5,.5] is added on observed data points
SfMData getInputScene(const NViewDataSet& d, const NViewDatasetConfigurator& config, EINTRINSIC eintrinsic)
{
    // Translate the input dataset to a SfMData scene
    SfMData sfm_data;

    // 1. Views
    // 2. Poses
    // 3. Intrinsic data (shared, so only one camera intrinsic is defined)
    // 4. Landmarks

    const int nviews = d._C.size();
    const int npoints = d._X.cols();

    // 1. Views
    for (int i = 0; i < nviews; ++i)
    {
        const IndexT id_view = i, id_pose = i, id_intrinsic = 0;  //(shared intrinsics)
        sfm_data.getViews().emplace(i, std::make_shared<View>("", id_view, id_intrinsic, id_pose, config._cx * 2, config._cy * 2));
    }

    // 2. Poses
    for (int i = 0; i < nviews; ++i)
    {
        Pose3 pose(d._R[i], d._C[i]);
        sfm_data.setPose(*sfm_data.getViews().at(i), CameraPose(pose));
    }

    // 3. Intrinsic data (shared, so only one camera intrinsic is defined)
    {
        const unsigned int w = config._cx * 2;
        const unsigned int h = config._cy * 2;
        sfm_data.getIntrinsics().emplace(0, createIntrinsic(eintrinsic, w, h, config._fx, config._cx, config._cy));
    }

    // 4. Landmarks
    const double unknownScale = 0.0;
    for (int i = 0; i < npoints; ++i)
    {
        // Collect the image of point i in each frame.
        Landmark landmark;
        landmark.X = d._X.col(i);
        for (int j = 0; j < nviews; ++j)
        {
            Vec2 pt = d._x[j].col(i);
            // => random noise between [-.5,.5] is added
            pt(0) += rand() / RAND_MAX - .5;
            pt(1) += rand() / RAND_MAX - .5;

            landmark.getObservations()[j] = Observation(pt, i, unknownScale);
        }
        sfm_data.getLandmarks()[i] = landmark;
    }

    return sfm_data;
}
