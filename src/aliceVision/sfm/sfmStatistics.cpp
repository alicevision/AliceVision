// This file is part of the AliceVision project.
// Copyright (c) 2020 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "sfmStatistics.hpp"

#include <aliceVision/sfm/pipeline/localization/SfMLocalizer.hpp>

#include <aliceVision/sfm/pipeline/regionsIO.hpp>
#include <aliceVision/sfm/pipeline/pairwiseMatchesIO.hpp>
#include <aliceVision/track/TracksBuilder.hpp>

namespace aliceVision {
namespace sfm {

void computeResidualsHistogram(const sfmData::SfMData& sfmData,
                               BoxStats<double>& outStats,
                               utils::Histogram<double>* outHistogram,
                               const std::set<IndexT>& specificViews)
{
    {
        // Init output params
        outStats = BoxStats<double>();
        if (outHistogram)
        {
            *outHistogram = utils::Histogram<double>();
        }
    }
    if (sfmData.getLandmarks().empty())
        return;

    // Collect residuals for each observation
    std::vector<double> vecResiduals;
    vecResiduals.reserve(sfmData.getLandmarks().size());

    for (const auto& track : sfmData.getLandmarks())
    {
        const aliceVision::sfmData::Observations& observations = track.second.getObservations();
        for (const auto& obs : observations)
        {
            if (!specificViews.empty())
            {
                if (specificViews.count(obs.first) == 0)
                    continue;
            }
            const sfmData::View& view = sfmData.getView(obs.first);
            const aliceVision::geometry::Pose3 pose = sfmData.getPose(view).getTransform();
            const std::shared_ptr<aliceVision::camera::IntrinsicBase> intrinsic = sfmData.getIntrinsics().find(view.getIntrinsicId())->second;
            const Vec2 residual = intrinsic->residual(pose, track.second.X.homogeneous(), obs.second.getCoordinates());
            vecResiduals.push_back(residual.norm());
            // ALICEVISION_LOG_INFO("[AliceVision] sfmtstatistics::computeResidualsHistogram track: " << track.first << ", residual: " <<
            // residual.norm());
        }
    }

    // ALICEVISION_LOG_INFO("[AliceVision] sfmtstatistics::computeResidualsHistogram vecResiduals.size(): " << vec_residuals.size());

    if (vecResiduals.empty())
        return;

    outStats = BoxStats<double>(vecResiduals.begin(), vecResiduals.end());

    if (outHistogram)
    {
        *outHistogram = utils::Histogram<double>(0.0, std::ceil(outStats.max), std::ceil(outStats.max) * 2);
        outHistogram->Add(vecResiduals.begin(), vecResiduals.end());
    }
}

void computeObservationsLengthsHistogram(const sfmData::SfMData& sfmData,
                                         BoxStats<double>& outStats,
                                         int& overallNbObservations,
                                         utils::Histogram<double>* outHistogram,
                                         const std::set<IndexT>& specificViews)
{
    {
        // Init output params
        outStats = BoxStats<double>();
        if (outHistogram)
        {
            *outHistogram = utils::Histogram<double>();
        }
    }
    if (sfmData.getLandmarks().empty())
        return;

    // Collect tracks size: number of 2D observations per 3D points
    std::vector<int> nbObservations;
    nbObservations.reserve(sfmData.getLandmarks().size());

    for (const auto& landmark : sfmData.getLandmarks())
    {
        const aliceVision::sfmData::Observations& observations = landmark.second.getObservations();
        if (!specificViews.empty())
        {
            int nbObsSpecificViews = 0;
            for (const auto& obs : observations)
            {
                if (specificViews.count(obs.first) == 1)
                {
                    ++nbObsSpecificViews;
                }
            }
            if (nbObsSpecificViews > 0)
            {
                nbObservations.push_back(observations.size());
            }
        }
        else
        {
            nbObservations.push_back(observations.size());
        }
        overallNbObservations += observations.size();
    }

    if (nbObservations.empty())
        return;

    outStats = BoxStats<double>(nbObservations.begin(), nbObservations.end());

    if (outHistogram)
    {
        *outHistogram = utils::Histogram<double>(outStats.min, outStats.max + 1, outStats.max - outStats.min + 1);
        outHistogram->Add(nbObservations.begin(), nbObservations.end());
    }
}

void computeLandmarksPerViewHistogram(const sfmData::SfMData& sfmData, BoxStats<double>& outStats, utils::Histogram<double>* outHistogram)
{
    {
        // Init output params
        outStats = BoxStats<double>();
        if (outHistogram)
        {
            *outHistogram = utils::Histogram<double>();
        }
    }
    if (sfmData.getLandmarks().empty())
        return;

    std::map<IndexT, int> nbLandmarksPerView;

    for (const auto& landmark : sfmData.getLandmarks())
    {
        for (const auto& obsIt : landmark.second.getObservations())
        {
            const auto& viewId = obsIt.first;
            auto it = nbLandmarksPerView.find(viewId);
            if (it != nbLandmarksPerView.end())
                ++(it->second);
            else
                it->second = 1;
        }
    }
    if (nbLandmarksPerView.empty())
        return;

    // Collect tracks size: number of 2D observations per 3D points
    std::vector<int> nbLandmarksPerViewVec;
    for (const auto& it : nbLandmarksPerView)
        nbLandmarksPerViewVec.push_back(it.second);

    outStats = BoxStats<double>(nbLandmarksPerViewVec.begin(), nbLandmarksPerViewVec.end());

    if (outHistogram)
    {
        //*outHistogram = Histogram<double>(0, sfmData.getViews().size(), sfmData.getViews().size());
        *outHistogram = utils::Histogram<double>(outStats.min, (outStats.max + 1), 10);
        outHistogram->Add(nbLandmarksPerViewVec.begin(), nbLandmarksPerViewVec.end());
    }
}

void computeLandmarksPerView(const sfmData::SfMData& sfmData, std::vector<int>& outNbLandmarksPerView)
{
    {
        outNbLandmarksPerView.clear();
    }
    if (sfmData.getLandmarks().empty())
        return;

    std::map<IndexT, int> nbLandmarksPerView;

    for (const auto& landmark : sfmData.getLandmarks())
    {
        const aliceVision::sfmData::Observations& observations = landmark.second.getObservations();
        for (const auto& obs : observations)
        {
            auto it = nbLandmarksPerView.find(obs.first);
            if (it != nbLandmarksPerView.end())
                ++(it->second);
            else
                nbLandmarksPerView[obs.first] = 1;
        }
    }
    if (nbLandmarksPerView.empty())
        return;

    outNbLandmarksPerView.reserve(nbLandmarksPerView.size());
    for (const auto& viewIt : sfmData.getViews())
    {
        const auto it = nbLandmarksPerView.find(viewIt.first);
        int nbLandmarks = 0;
        if (it != nbLandmarksPerView.end())
            nbLandmarks = it->second;
        outNbLandmarksPerView.push_back(nbLandmarks);
    }
}

void computeFeatMatchPerView(const sfmData::SfMData& sfmData, std::vector<std::size_t>& outFeatPerView, std::vector<std::size_t>& outMatchPerView)
{
    const auto descTypesTmp = sfmData.getLandmarkDescTypes();
    const std::vector<feature::EImageDescriberType> describerTypes(descTypesTmp.begin(), descTypesTmp.end());

    // features reading
    feature::FeaturesPerView featuresPerView;
    if (!sfm::loadFeaturesPerView(featuresPerView, sfmData, sfmData.getFeaturesFolders(), describerTypes))
    {
        ALICEVISION_LOG_ERROR("Invalid features.");
        return;
    }

    // matches reading
    matching::PairwiseMatches pairwiseMatches;
    if (!sfm::loadPairwiseMatches(pairwiseMatches, sfmData, sfmData.getMatchesFolders(), describerTypes, 0, 0, true))
    {
        ALICEVISION_LOG_ERROR("Unable to load matches.");
        return;
    }

    track::TracksMap tracks;
    track::TracksPerView tracksPerView;

    track::TracksBuilder tracksBuilder;
    tracksBuilder.build(pairwiseMatches);
    tracksBuilder.exportToSTL(tracks);

    // Init tracksPerView to have an entry in the map for each view (even if there is no track at all)
    for (const auto& viewIt : sfmData.getViews())
    {
        // create an entry in the map
        tracksPerView[viewIt.first];
    }
    track::computeTracksPerView(tracks, tracksPerView);

    outFeatPerView.reserve(sfmData.getViews().size());
    outMatchPerView.reserve(sfmData.getViews().size());
    for (const auto& viewIt : sfmData.getViews())
    {
        const auto viewId = viewIt.first;

        if (featuresPerView.viewExist(viewId))
            outFeatPerView.push_back(featuresPerView.getNbFeatures(viewId));
        else
            outFeatPerView.push_back(0);

        if (tracksPerView.count(viewId))
            outMatchPerView.push_back(tracksPerView.at(viewId).size());
        else
            outMatchPerView.push_back(0);
    }
}

void computeScaleHistogram(const sfmData::SfMData& sfmData,
                           BoxStats<double>& outStats,
                           utils::Histogram<double>* outHistogram,
                           const std::set<IndexT>& specificViews)
{
    {
        // Init output params
        outStats = BoxStats<double>();
        if (outHistogram)
        {
            *outHistogram = utils::Histogram<double>();
        }
    }
    if (sfmData.getLandmarks().empty())
        return;

    // Collect tracks size: number of 2D observations per 3D points
    std::vector<double> vecScaleObservations;
    vecScaleObservations.reserve(sfmData.getLandmarks().size());
    for (const auto& landmark : sfmData.getLandmarks())
    {
        const aliceVision::sfmData::Observations& observations = landmark.second.getObservations();

        for (const auto& obs : observations)
        {
            if (!specificViews.empty())
            {
                if (specificViews.count(obs.first) == 0)
                    continue;
            }
            vecScaleObservations.push_back(obs.second.getScale());
        }
    }

    if (vecScaleObservations.empty())
        return;

    outStats = BoxStats<double>(vecScaleObservations.begin(), vecScaleObservations.end());

    if (outHistogram)
    {
        size_t maxValue = std::ceil(outStats.max);
        *outHistogram = utils::Histogram<double>(0.0, double(maxValue), maxValue + 1);
        outHistogram->Add(vecScaleObservations.begin(), vecScaleObservations.end());
    }
}

void computeResidualsPerView(const sfmData::SfMData& sfmData,
                             int& nbViews,
                             std::vector<double>& nbResidualsPerViewMin,
                             std::vector<double>& nbResidualsPerViewMax,
                             std::vector<double>& nbResidualsPerViewMean,
                             std::vector<double>& nbResidualsPerViewMedian,
                             std::vector<double>& nbResidualsPerViewFirstQuartile,
                             std::vector<double>& nbResidualsPerViewThirdQuartile)
{
    if (sfmData.getLandmarks().empty())
        return;

    nbViews = sfmData.getViews().size();
    nbResidualsPerViewMin.resize(nbViews);
    nbResidualsPerViewMax.resize(nbViews);
    nbResidualsPerViewMean.resize(nbViews);
    nbResidualsPerViewMedian.resize(nbViews);
    nbResidualsPerViewFirstQuartile.resize(nbViews);
    nbResidualsPerViewThirdQuartile.resize(nbViews);

    // Collect residuals (number of residuals per 3D points) of all landmarks visible in each view
    std::map<IndexT, std::vector<double>> residualsPerView;

    for (const auto& landmark : sfmData.getLandmarks())
    {
        const aliceVision::sfmData::Observations& observations = landmark.second.getObservations();
        for (const auto& obs : observations)
        {
            const sfmData::View& view = sfmData.getView(obs.first);
            const aliceVision::geometry::Pose3 pose = sfmData.getPose(view).getTransform();
            const std::shared_ptr<aliceVision::camera::IntrinsicBase> intrinsic = sfmData.getIntrinsics().find(view.getIntrinsicId())->second;
            const Vec2 residual = intrinsic->residual(pose, landmark.second.X.homogeneous(), obs.second.getCoordinates());
            residualsPerView[obs.first].push_back(residual.norm());
        }
    }

    std::vector<IndexT> viewKeys;
    for (const auto& v : sfmData.getViews())
        viewKeys.push_back(v.first);

#pragma omp parallel for
    for (int viewIdx = 0; viewIdx < nbViews; ++viewIdx)
    {
        const IndexT viewId = viewKeys[viewIdx];

        const auto it = residualsPerView.find(viewId);
        if (it == residualsPerView.end())
            continue;
        const std::vector<double>& residuals = it->second;
        BoxStats<double> residualStats(residuals.begin(), residuals.end());
        utils::Histogram<double> residual_histogram =
          utils::Histogram<double>(residualStats.min, residualStats.max + 1, residualStats.max - residualStats.min + 1);
        residual_histogram.Add(residuals.begin(), residuals.end());

        nbResidualsPerViewMin[viewIdx] = residualStats.min;
        nbResidualsPerViewMax[viewIdx] = residualStats.max;
        nbResidualsPerViewMean[viewIdx] = residualStats.mean;
        nbResidualsPerViewMedian[viewIdx] = residualStats.median;
        nbResidualsPerViewFirstQuartile[viewIdx] = residualStats.firstQuartile;
        nbResidualsPerViewThirdQuartile[viewIdx] = residualStats.thirdQuartile;
    }
}

void computeObservationsLengthsPerView(const sfmData::SfMData& sfmData,
                                       int& nbViews,
                                       std::vector<double>& nbObservationsLengthsPerViewMin,
                                       std::vector<double>& nbObservationsLengthsPerViewMax,
                                       std::vector<double>& nbObservationsLengthsPerViewMean,
                                       std::vector<double>& nbObservationsLengthsPerViewMedian,
                                       std::vector<double>& nbObservationsLengthsPerViewFirstQuartile,
                                       std::vector<double>& nbObservationsLengthsPerViewThirdQuartile)
{
    if (sfmData.getLandmarks().empty())
        return;

    nbViews = sfmData.getViews().size();
    nbObservationsLengthsPerViewMin.resize(nbViews);
    nbObservationsLengthsPerViewMax.resize(nbViews);
    nbObservationsLengthsPerViewMean.resize(nbViews);
    nbObservationsLengthsPerViewMedian.resize(nbViews);
    nbObservationsLengthsPerViewFirstQuartile.resize(nbViews);
    nbObservationsLengthsPerViewThirdQuartile.resize(nbViews);

    // Collect observations length (number of 2D observations per 3D points) of all landmarks visible in each view
    std::map<IndexT, std::vector<int>> observationLengthsPerView;

    for (const auto& landmark : sfmData.getLandmarks())
    {
        const aliceVision::sfmData::Observations& observations = landmark.second.getObservations();
        for (const auto& obs : observations)
        {
            observationLengthsPerView[obs.first].push_back(observations.size());
        }
    }

    std::vector<IndexT> viewKeys;
    for (const auto& v : sfmData.getViews())
        viewKeys.push_back(v.first);

#pragma omp parallel for
    for (int viewIdx = 0; viewIdx < nbViews; ++viewIdx)
    {
        const IndexT viewId = viewKeys[viewIdx];
        const std::vector<int>& nbObservations = observationLengthsPerView[viewId];
        BoxStats<double> observationsLengthsStats(nbObservations.begin(), nbObservations.end());
        utils::Histogram<double> observationsLengthsHistogram(
          observationsLengthsStats.min, observationsLengthsStats.max + 1, observationsLengthsStats.max - observationsLengthsStats.min + 1);
        observationsLengthsHistogram.Add(nbObservations.begin(), nbObservations.end());

        nbObservationsLengthsPerViewMin[viewIdx] = observationsLengthsStats.min;
        nbObservationsLengthsPerViewMax[viewIdx] = observationsLengthsStats.max;
        nbObservationsLengthsPerViewMean[viewIdx] = observationsLengthsStats.mean;
        nbObservationsLengthsPerViewMedian[viewIdx] = observationsLengthsStats.median;
        nbObservationsLengthsPerViewFirstQuartile[viewIdx] = observationsLengthsStats.firstQuartile;
        nbObservationsLengthsPerViewThirdQuartile[viewIdx] = observationsLengthsStats.thirdQuartile;
    }
}

}  // namespace sfm
}  // namespace aliceVision
