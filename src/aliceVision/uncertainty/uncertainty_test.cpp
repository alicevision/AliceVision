// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#define BOOST_TEST_MODULE uncertainty

#include <boost/test/unit_test.hpp>

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sceneSample.hpp>
#include <aliceVision/sfm/bundle/BundleAdjustmentCeres.hpp>
#include <aliceVision/sfm/utils/gauge.hpp>
#include <aliceVision/uncertainty/uncertainty.hpp>

#include <Eigen/Sparse>

using namespace aliceVision;
using namespace aliceVision::sfmDataIO;

namespace
{

/// Run the full uncertainty pipeline on a pre-populated SfMData.
/// Returns true if computeUncertainty succeeded.
bool runUncertainty(sfmData::SfMData& sfmData)
{
    std::array<IndexT, 3> sample;
    if (!sfm::selectTripletForGaugeRemoval(sfmData, sample))
    {
        return false;
    }

    std::array<bool, 3> previouslyLocked;
    for (int i = 0; i < 3; i++)
    {
        previouslyLocked[i] = sfmData.getLandmarks().at(sample[i]).isLocked();
        sfmData.getLandmarks().at(sample[i]).setLocked(true);
    }

    std::map<IndexT, size_t> poseToPosition;
    std::map<IndexT, size_t> intrinsicsToPosition;
    std::map<IndexT, size_t> distortionToPosition;
    std::map<IndexT, size_t> landmarkToPosition;
    ceres::CRSMatrix jacobian;
    sfm::BundleAdjustmentCeres bundleAdjustmentObj;
    bundleAdjustmentObj.createJacobian(sfmData, jacobian, poseToPosition, intrinsicsToPosition, distortionToPosition, landmarkToPosition);

    Eigen::Map<const Eigen::SparseMatrix<double, Eigen::RowMajor, int>> J_eigen(
        jacobian.num_rows,
        jacobian.num_cols,
        static_cast<int>(jacobian.values.size()),
        jacobian.rows.data(),
        jacobian.cols.data(),
        jacobian.values.data()
    );

    Eigen::MatrixXd covarianceCameras;
    const bool res = uncertainty::computeUncertainty(covarianceCameras, J_eigen, landmarkToPosition.size());

    for (int i = 0; i < 3; i++)
    {
        sfmData.getLandmarks().at(sample[i]).setLocked(previouslyLocked[i]);
    }

    return res;
}

} // anonymous namespace

BOOST_AUTO_TEST_CASE(Uncertainty_SceneSample_Sphere)
{
    ALICEVISION_LOG_INFO("Uncertainty_SceneSample_Sphere");
    sfmData::SfMData sfmData;
    generateSphereScene(sfmData, 1000, 20);
    BOOST_CHECK(runUncertainty(sfmData));
}

BOOST_AUTO_TEST_CASE(Uncertainty_SceneSample_Planar)
{
    ALICEVISION_LOG_INFO("Uncertainty_SceneSample_Planar");
    sfmData::SfMData sfmData;
    generatePlanarScene(sfmData, 30, 60);
    BOOST_CHECK(runUncertainty(sfmData));
}

BOOST_AUTO_TEST_CASE(Uncertainty_SceneSample_CollinearCameras)
{
    ALICEVISION_LOG_INFO("Uncertainty_SceneSample_CollinearCameras");
    sfmData::SfMData sfmData;
    generateCollinearCamerasScene(sfmData, 500, 20);
    BOOST_CHECK(runUncertainty(sfmData));
}

BOOST_AUTO_TEST_CASE(Uncertainty_SceneSample_ForwardMotion)
{
    ALICEVISION_LOG_INFO("Uncertainty_SceneSample_ForwardMotion");
    sfmData::SfMData sfmData;
    generateForwardMotionScene(sfmData, 500, 30);
    BOOST_CHECK(runUncertainty(sfmData));
}

BOOST_AUTO_TEST_CASE(Uncertainty_SceneSample_NarrowBaseline)
{
    ALICEVISION_LOG_INFO("Uncertainty_SceneSample_NarrowBaseline");
    sfmData::SfMData sfmData;
    generateNarrowBaselineScene(sfmData, 1000, 240);
    BOOST_CHECK(runUncertainty(sfmData));
}

BOOST_AUTO_TEST_CASE(Uncertainty_SceneSample_UnevenCoverage)
{
    ALICEVISION_LOG_INFO("Uncertainty_SceneSample_UnevenCoverage");
    sfmData::SfMData sfmData;
    generateUnevenCoverageScene(sfmData, 200, 800, 240);
    BOOST_CHECK(runUncertainty(sfmData));
}

BOOST_AUTO_TEST_CASE(Uncertainty_SceneSample_PureRotation)
{
    ALICEVISION_LOG_INFO("Uncertainty_SceneSample_PureRotation");
    sfmData::SfMData sfmData;
    generatePureRotationScene(sfmData, 1000, 240);
    BOOST_CHECK(runUncertainty(sfmData));
}

BOOST_AUTO_TEST_CASE(Uncertainty_SceneSample_Sparse)
{
    ALICEVISION_LOG_INFO("Uncertainty_SceneSample_Sparse");
    sfmData::SfMData sfmData;
    generateSparseCameraObservationsScene(sfmData, 100, 60, 5);
    BOOST_CHECK(runUncertainty(sfmData));
}

BOOST_AUTO_TEST_CASE(Uncertainty_SceneSample_ClusteredObservations)
{
    ALICEVISION_LOG_INFO("Uncertainty_SceneSample_ClusteredObservations");
    sfmData::SfMData sfmData;
    generateClusteredObservationsScene(sfmData, 200, 2);
    BOOST_CHECK(runUncertainty(sfmData));
}

