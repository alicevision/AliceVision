// This file is part of the AliceVision project.
// Copyright (c) 2020 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "sfmStatistics.hpp"

#include <aliceVision/sfm/pipeline/localization/SfMLocalizer.hpp>


namespace aliceVision {
namespace sfm {

void computeResidualsHistogram(const sfmData::SfMData& sfmData, MinMaxMeanMedian<double>& out_stats, Histogram<double>* out_histogram, const std::set<IndexT>& specificViews)
{
  if (sfmData.getLandmarks().empty())
    return;

  // Collect residuals for each observation
  std::vector<double> vec_residuals;
  vec_residuals.reserve(sfmData.structure.size());

  for(const auto &track : sfmData.getLandmarks())
  {
    const aliceVision::sfmData::Observations & observations = track.second.observations;
    for(const auto& obs: observations)
    {
      if(!specificViews.empty())
      {
          if(specificViews.count(obs.first) == 0)
              continue;
      }
      const sfmData::View& view = sfmData.getView(obs.first);
      const aliceVision::geometry::Pose3 pose = sfmData.getPose(view).getTransform();
      const std::shared_ptr<aliceVision::camera::IntrinsicBase> intrinsic = sfmData.getIntrinsics().find(view.getIntrinsicId())->second;
      const Vec2 residual = intrinsic->residual(pose, track.second.X, obs.second.x);
      vec_residuals.push_back( fabs(residual(0)) );
      vec_residuals.push_back( fabs(residual(1)) );
    }
  }

  assert(!vec_residuals.empty());

  out_stats = MinMaxMeanMedian<double>(vec_residuals.begin(), vec_residuals.end());

  if (out_histogram)
  {
    *out_histogram = Histogram<double>(0.0, std::ceil(out_stats.max), std::ceil(out_stats.max)*2);
    out_histogram->Add(vec_residuals.begin(), vec_residuals.end());
  }
}


void computeObservationsLengthsHistogram(const sfmData::SfMData& sfmData, MinMaxMeanMedian<double>& out_stats, int& overallNbObservations, Histogram<double>* histo, const std::set<IndexT>& specificViews)
{
  if (sfmData.getLandmarks().empty())
    return;

  // Collect tracks size: number of 2D observations per 3D points
  std::vector<int> nbObservations;
  nbObservations.reserve(sfmData.getLandmarks().size());

  for(const auto& landmark : sfmData.getLandmarks())
  {
    const aliceVision::sfmData::Observations & observations = landmark.second.observations;
    if (!specificViews.empty())
    {
        int nbObsSpecificViews = 0;
        for(const auto& obs: observations)
        {
            if(specificViews.count(obs.first) == 1)
            {
                ++nbObsSpecificViews;
            }
        }
        if(nbObsSpecificViews > 0)
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

  assert(!nbObservations.empty());

  out_stats = MinMaxMeanMedian<double>(nbObservations.begin(), nbObservations.end());

  if (histo)
  {
    *histo = Histogram<double>(out_stats.min, out_stats.max + 1, out_stats.max - out_stats.min + 1);
    histo->Add(nbObservations.begin(), nbObservations.end());
  }
}

void computeLandmarksPerViewHistogram(const sfmData::SfMData& sfmData, MinMaxMeanMedian<double>& out_stats, track::TracksPerView tracksPerView, Histogram<double>* histo)
{
  if(sfmData.getLandmarks().empty())
    return;

  // Collect tracks size: number of 2D observations per 3D points
  std::vector<int> nbLandmarksPerView;
  nbLandmarksPerView.reserve(sfmData.getViews().size());

  std::set<std::size_t> landmarksId;
  std::transform(sfmData.getLandmarks().begin(), sfmData.getLandmarks().end(),
    std::inserter(landmarksId, landmarksId.begin()),
    stl::RetrieveKey());

  for(const auto &viewIt : sfmData.getViews())
  {
    const sfmData::View & view = *viewIt.second;
    if(!sfmData.isPoseAndIntrinsicDefined(view.getViewId()))
      continue;

    aliceVision::track::TrackIdSet viewLandmarksIds;
    {
      ALICEVISION_LOG_INFO("[AliceVision] sfmStatistics::computeLandmarksPerViewHistogram view.getViewId(): " << view.getViewId());
      ALICEVISION_LOG_INFO("[AliceVision] sfmStatistics::computeLandmarksPerViewHistogram tracksPerView.at(view.getViewId()): " << tracksPerView.at(view.getViewId()));

      const aliceVision::track::TrackIdSet& viewTracksIds = tracksPerView.at(view.getViewId());
      // Get the ids of the already reconstructed tracks
      std::set_intersection(viewTracksIds.begin(), viewTracksIds.end(),
        landmarksId.begin(), landmarksId.end(),
        std::inserter(viewLandmarksIds, viewLandmarksIds.begin()));
    }
    nbLandmarksPerView.push_back(viewLandmarksIds.size());
  }

  out_stats = MinMaxMeanMedian<double>(nbLandmarksPerView.begin(), nbLandmarksPerView.end());

  if (histo)
  {
    *histo = Histogram<double>(0, sfmData.getViews().size(), sfmData.getViews().size());
    histo->Add(nbLandmarksPerView.begin(), nbLandmarksPerView.end());
  }
}

void computeScaleHistogram(const sfmData::SfMData& sfmData, MinMaxMeanMedian<double>& out_stats, Histogram<double>* histo, const std::set<IndexT>& specificViews)
{
    if(sfmData.getLandmarks().empty())
      return;

    // Collect tracks size: number of 2D observations per 3D points
    std::vector<double> vec_scaleObservations;
    vec_scaleObservations.reserve(sfmData.getLandmarks().size());
    for(const auto& landmark: sfmData.getLandmarks())
    {
        const aliceVision::sfmData::Observations & observations = landmark.second.observations;

        for(const auto& obs: observations)
        {
            if(!specificViews.empty())
            {
                if(specificViews.count(obs.first) == 0)
                    continue;
            }
            double scaleObs = obs.second.scale;
            vec_scaleObservations.push_back(scaleObs);
        }
    }

    assert(!vec_scaleObservations.empty());

    out_stats = MinMaxMeanMedian<double>(vec_scaleObservations.begin(), vec_scaleObservations.end());

    if (histo)
    {
      *histo = Histogram<double>(0.0, std::ceil(out_stats.max), std::ceil(out_stats.max));
      histo->Add(vec_scaleObservations.begin(), vec_scaleObservations.end());
    }
}

void computeResidualsPerView(const sfmData::SfMData& sfmData, int& nbViews, std::vector<double>& nbResidualsPerViewMin,
                                      std::vector<double>& nbResidualsPerViewMax, std::vector<double>& nbResidualsPerViewMean,
                                      std::vector<double>& nbResidualsPerViewMedian)
{
    if(sfmData.getLandmarks().empty())
      return;

    for(const auto &viewIt : sfmData.getViews())
    {
        MinMaxMeanMedian<double> residualStats;
        Histogram<double> residual_histogram = Histogram<double>();
        computeResidualsHistogram(sfmData, residualStats, &residual_histogram, {viewIt.first});
        nbResidualsPerViewMin.push_back(residualStats.min);
        nbResidualsPerViewMax.push_back(residualStats.max);
        nbResidualsPerViewMean.push_back(residualStats.mean);
        nbResidualsPerViewMedian.push_back(residualStats.median);
        nbViews++;
    }

    assert(!nbResidualsPerViewMin.empty() && !nbResidualsPerViewMax.empty() && !nbResidualsPerViewMean.empty() && !nbResidualsPerViewMedian.empty());

}

void computePointsValidatedPerView(const sfmData::SfMData& sfmData, int& nbViews, std::vector<double> nbPointsValidatedPerView)
{
    if(sfmData.getLandmarks().empty())
      return;

    sfm::ImageLocalizerMatchData matchData;
    for(const auto &viewIt : sfmData.getViews())
    {
        double resultValidated = matchData.vec_inliers.size();
        ALICEVISION_LOG_INFO("[AliceVision] sfmstatistics computePointsValidated resultValidated: " << resultValidated);
        nbPointsValidatedPerView.push_back(resultValidated);
        nbViews++;
    }

    assert(!nbPointsValidatedPerView.empty());
}

}
}
