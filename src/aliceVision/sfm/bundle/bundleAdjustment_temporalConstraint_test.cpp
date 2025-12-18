// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#define BOOST_TEST_MODULE bundleAdjustment_temporalConstraint

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfm/bundle/BundleAdjustmentCeres.hpp>
#include <aliceVision/sfmDataIO/sceneSample.hpp>
#include <aliceVision/sfm/utils/poseNoise.hpp>
#include <aliceVision/track/tracksUtils.hpp>

#include <aliceVision/sfm/utils/statistics.hpp>
#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>

using namespace aliceVision;

void computeTemporalSmoothness(const sfmData::SfMData& sfmData, double& diff0Mean, double& diff1Mean, double& diff2Mean,
                               double& angleDiff0Mean, double& angleDiff1Mean, double& angleDiff2Mean)
{
    using namespace Eigen;
    using namespace Eigen::indexing;

    const int viewCount = sfmData.getViews().size();

    MatrixXd viewCenters(3, viewCount);

    int frameIdx = 0;
    for (auto& [_, pose] : sfmData.getPoses().valueRange())
        viewCenters.col(frameIdx++) = pose.getTransform().center();

    Vector3d viewsCenter = viewCenters.rowwise().mean();

    double radiusMean = (viewCenters.colwise() - viewsCenter).colwise().norm().mean();

    diff0Mean = std::sqrt((viewCenters(all,seq(1, last)) - viewCenters(all,seq(0, last-1))).array().pow(2).mean());
    diff0Mean = diff0Mean / radiusMean;

    diff1Mean = std::sqrt((2. * viewCenters(all,seq(1, last-1))
                              - viewCenters(all,seq(0, last-2))
                              - viewCenters(all,seq(2, last))).array().pow(2).mean());
    diff1Mean = diff1Mean / radiusMean;

    diff2Mean = std::sqrt((3. * viewCenters(all,seq(1, last-2))
                         - 3. * viewCenters(all,seq(2, last-1))
                              + viewCenters(all,seq(3, last))
                              - viewCenters(all,seq(0, last-3))).array().pow(2).mean());
    diff2Mean = diff2Mean / radiusMean;

    MatrixXd angleDiff(3, viewCount-1);

    frameIdx = -1;
    for (auto& [_, pose] : sfmData.getPoses().valueRange())
    {
        Quaterniond prevRot, currRot;
        prevRot = currRot;
        currRot = Quaterniond(pose.getTransform().rotation());
        if (frameIdx == -1)
        {
            frameIdx++;
            continue;
        }
        AngleAxisd aa(currRot * prevRot.conjugate());
        angleDiff.col(frameIdx++) = aa.angle() * aa.axis();
    }

    angleDiff0Mean = std::sqrt(angleDiff.array().pow(2).mean());

    angleDiff1Mean = std::sqrt((angleDiff(all,seq(1, last)) - angleDiff(all,seq(0, last-1))).array().pow(2).mean());

    angleDiff2Mean = std::sqrt((2. * angleDiff(all,seq(1, last-1))
                                   - angleDiff(all,seq(0, last-2))
                                   - angleDiff(all,seq(2, last))).array().pow(2).mean());
}

BOOST_AUTO_TEST_CASE(BA_temporalConstraint)
{
    sfmData::SfMData sfmData;
    sfmDataIO::generateSphereScene(sfmData, 100, 240);

    track::TracksMap mapTracks;
    track::simulateTracks(sfmData, 200., 0., 0., false, mapTracks);
    for (auto& [landmarkId, landmark] : sfmData.getLandmarks())
    for (auto& [viewId, obs] : mapTracks[landmarkId].featPerView)
        landmark.getObservations().at(viewId).setCoordinates(obs.coords);

    double diff0MeanClean, diff1MeanClean, diff2MeanClean;
    double angleDiff0MeanClean, angleDiff1MeanClean, angleDiff2MeanClean;
    computeTemporalSmoothness(sfmData, diff0MeanClean, diff1MeanClean, diff2MeanClean,
                              angleDiff0MeanClean, angleDiff1MeanClean, angleDiff2MeanClean);

    sfm::addPoseNoise(sfmData, 0.3, 0.05);

    double diff0MeanNoise, diff1MeanNoise, diff2MeanNoise;
    double angleDiff0MeanNoise, angleDiff1MeanNoise, angleDiff2MeanNoise;
    computeTemporalSmoothness(sfmData, diff0MeanNoise, diff1MeanNoise, diff2MeanNoise,
                              angleDiff0MeanNoise, angleDiff1MeanNoise, angleDiff2MeanNoise);

    sfm::BundleAdjustmentCeres::CeresOptions options;
    sfm::BundleAdjustment::ERefineOptions refineOptions;
    refineOptions |= sfm::BundleAdjustment::REFINE_ROTATION;
    refineOptions |= sfm::BundleAdjustment::REFINE_STRUCTURE;
    refineOptions |= sfm::BundleAdjustment::REFINE_TRANSLATION;
    refineOptions |= sfm::BundleAdjustment::REFINE_INTRINSICS_ALL;
    options.summary = false;

    // Bundle adjustment without temporal constraint
    double rmseBeforeBA = sfm::RMSE(sfmData);
    sfm::BundleAdjustmentCeres BA(options);
    BA.adjust(sfmData, refineOptions);
    double rmseAfterBA_noTC = sfm::RMSE(sfmData);
    BOOST_CHECK_LT(rmseAfterBA_noTC, .5 * rmseBeforeBA);

    double diff0MeanNoTC, diff1MeanNoTC, diff2MeanNoTC;
    double angleDiff0MeanNoTC, angleDiff1MeanNoTC, angleDiff2MeanNoTC;
    computeTemporalSmoothness(sfmData, diff0MeanNoTC, diff1MeanNoTC, diff2MeanNoTC,
                              angleDiff0MeanNoTC, angleDiff1MeanNoTC, angleDiff2MeanNoTC);

    BOOST_CHECK_LT(angleDiff0MeanNoTC, angleDiff0MeanNoise);
    BOOST_CHECK_LT(angleDiff1MeanNoTC, angleDiff1MeanNoise);
    BOOST_CHECK_LT(angleDiff2MeanNoTC, angleDiff2MeanNoise);

    // Add the temporal constraint
    refineOptions |= sfm::BundleAdjustment::REFINE_TEMPORAL_SMOOTHNESS_CONSTRAINT;
    options.temporalConstraintParams.positionWeight = 10.;
    options.temporalConstraintParams.orientationWeight = 0.;
    BA = sfm::BundleAdjustmentCeres(options);

    // Bundle adjustment with temporal constraint on positions
    BA.adjust(sfmData, refineOptions);
    double rmseAfterBA_TC = sfm::RMSE(sfmData);
    BOOST_CHECK_LT(rmseAfterBA_TC, 1.2 * rmseAfterBA_noTC);

    double diff0MeanTCP, diff1MeanTCP, diff2MeanTCP;
    double angleDiff0MeanTCP, angleDiff1MeanTCP, angleDiff2MeanTCP;
    computeTemporalSmoothness(sfmData, diff0MeanTCP, diff1MeanTCP, diff2MeanTCP,
                              angleDiff0MeanTCP, angleDiff1MeanTCP, angleDiff2MeanTCP);

    BOOST_CHECK_LT(diff0MeanTCP, .75 * diff0MeanNoTC);
    BOOST_CHECK_LT(diff1MeanTCP, .1 * diff1MeanNoTC);
    BOOST_CHECK_LT(diff2MeanTCP, .1 * diff2MeanNoTC);
    BOOST_CHECK_LT(angleDiff0MeanTCP, .75 * angleDiff0MeanNoTC);
    BOOST_CHECK_LT(angleDiff1MeanTCP, .75 * angleDiff1MeanNoTC);
    BOOST_CHECK_LT(angleDiff2MeanTCP, .75 * angleDiff2MeanNoTC);

    BOOST_CHECK_LT(diff0MeanTCP, 2. * diff0MeanClean);
    BOOST_CHECK_LT(angleDiff0MeanTCP, 2. * angleDiff0MeanClean);

    options.temporalConstraintParams.positionWeight = 0.;
    options.temporalConstraintParams.orientationWeight = 10.;
    BA = sfm::BundleAdjustmentCeres(options);

    // Bundle adjustment with temporal constraint on orientations
    BA.adjust(sfmData, refineOptions);
    rmseAfterBA_TC = sfm::RMSE(sfmData);
    BOOST_CHECK_LT(rmseAfterBA_TC, 1.2 * rmseAfterBA_noTC);

    double diff0MeanTCO, diff1MeanTCO, diff2MeanTCO;
    double angleDiff0MeanTCO, angleDiff1MeanTCO, angleDiff2MeanTCO;
    computeTemporalSmoothness(sfmData, diff0MeanTCO, diff1MeanTCO, diff2MeanTCO,
                              angleDiff0MeanTCO, angleDiff1MeanTCO, angleDiff2MeanTCO);

    BOOST_CHECK_LT(diff0MeanTCO, .75 * diff0MeanNoTC);
    BOOST_CHECK_LT(diff1MeanTCO, .75 * diff1MeanNoTC);
    BOOST_CHECK_LT(diff2MeanTCO, .75 * diff2MeanNoTC);
    BOOST_CHECK_LT(angleDiff0MeanTCO, .75 * angleDiff0MeanNoTC);
    BOOST_CHECK_LT(angleDiff1MeanTCO, .5 * angleDiff1MeanNoTC);
    BOOST_CHECK_LT(angleDiff2MeanTCO, .5 * angleDiff2MeanNoTC);

    BOOST_CHECK_LT(diff0MeanTCO, 2. * diff0MeanClean);
    BOOST_CHECK_LT(angleDiff0MeanTCO, 1.5 * angleDiff0MeanClean);

    BOOST_CHECK_LT(diff1MeanTCP, .5 * diff1MeanTCO);
    BOOST_CHECK_LT(diff2MeanTCP, .5 * diff2MeanTCO);
    BOOST_CHECK_LT(angleDiff0MeanTCO, angleDiff0MeanTCP);
    BOOST_CHECK_LT(angleDiff1MeanTCO, .75 * angleDiff1MeanTCP);
    BOOST_CHECK_LT(angleDiff2MeanTCO, .5 * angleDiff2MeanTCP);
}
