// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "sfmTriangulation.hpp"
#include <aliceVision/multiview/triangulation/Triangulation.hpp>
#include <aliceVision/robustEstimation/randSampling.hpp>
#include <aliceVision/config.hpp>

#include <boost/progress.hpp>

#include <deque>
#include <memory>

namespace aliceVision {
namespace sfm {

using namespace aliceVision::geometry;
using namespace aliceVision::camera;

StructureComputation_basis::StructureComputation_basis(bool verbose)
  : _bConsoleVerbose(verbose)
{}

StructureComputation_blind::StructureComputation_blind(bool verbose)
  : StructureComputation_basis(verbose)
{}

void StructureComputation_blind::triangulate(sfmData::SfMData& sfmData, std::mt19937 & randomNumberGenerator) const
{
  std::deque<IndexT> rejectedId;
  std::unique_ptr<boost::progress_display> my_progress_bar;
  if (_bConsoleVerbose)
    my_progress_bar.reset( new boost::progress_display(
    sfmData.structure.size(),
    std::cout,
    "Blind triangulation progress:\n" ));
  #pragma omp parallel
  for(sfmData::Landmarks::iterator iterTracks = sfmData.structure.begin();
    iterTracks != sfmData.structure.end();
    ++iterTracks)
  {
    #pragma omp single nowait
    {
      if (_bConsoleVerbose)
      {
        #pragma omp critical
        ++(*my_progress_bar);
      }
      // Triangulate each landmark
      multiview::Triangulation trianObj;
      const sfmData::Observations & observations = iterTracks->second.observations;
      for(const auto& itObs : observations)
      {
        const sfmData::View * view = sfmData.views.at(itObs.first).get();
        if (sfmData.isPoseAndIntrinsicDefined(view))
        {
          std::shared_ptr<IntrinsicBase> cam = sfmData.getIntrinsics().at(view->getIntrinsicId());
          std::shared_ptr<camera::Pinhole> pinHoleCam = std::dynamic_pointer_cast<camera::Pinhole>(cam);
          if (!pinHoleCam) {
            ALICEVISION_LOG_ERROR("Camera is not pinhole in triangulate");
            continue;
          }

          const Pose3 pose = sfmData.getPose(*view).getTransform();
          trianObj.add(
            pinHoleCam->getProjectiveEquivalent(pose),
            cam->get_ud_pixel(itObs.second.x));
        }
      }
      if (trianObj.size() < 2)
      {
        #pragma omp critical
        {
          rejectedId.push_front(iterTracks->first);
        }
      }
      else
      {
        // Compute the 3D point
        const Vec3 X = trianObj.compute();
        if (trianObj.minDepth() > 0) // Keep the point only if it have a positive depth
        {
          iterTracks->second.X = X;
        }
        else
        {
          #pragma omp critical
          {
            rejectedId.push_front(iterTracks->first);
          }
        }
      }
    }
  }
  // Erase the unsuccessful triangulated tracks
  for (auto& it : rejectedId)
  {
    sfmData.structure.erase(it);
  }
}

StructureComputation_robust::StructureComputation_robust(bool verbose)
  : StructureComputation_basis(verbose)
{}

void StructureComputation_robust::triangulate(sfmData::SfMData& sfmData, std::mt19937 & randomNumberGenerator) const
{
  robust_triangulation(sfmData, randomNumberGenerator);
}

/// Robust triangulation of track data contained in the structure
/// All observations must have View with valid Intrinsic and Pose data
/// Invalid landmark are removed.
void StructureComputation_robust::robust_triangulation(sfmData::SfMData& sfmData, std::mt19937 & randomNumberGenerator) const
{
  std::deque<IndexT> rejectedId;
  std::unique_ptr<boost::progress_display> my_progress_bar;
  if(_bConsoleVerbose)
    my_progress_bar.reset( new boost::progress_display(
    sfmData.structure.size(),
    std::cout,
    "Robust triangulation progress:\n" ));
  #pragma omp parallel
  for(sfmData::Landmarks::iterator iterTracks = sfmData.structure.begin();
    iterTracks != sfmData.structure.end();
    ++iterTracks)
  {
    #pragma omp single nowait
    {
      if (_bConsoleVerbose)
      {
        #pragma omp critical
        ++(*my_progress_bar);
      }
      Vec3 X;
      if (robust_triangulation(sfmData, iterTracks->second.observations, randomNumberGenerator, X)) {
        iterTracks->second.X = X;
      }
      else {
        iterTracks->second.X = Vec3::Zero();
        #pragma omp critical
        {
          rejectedId.push_front(iterTracks->first);
        }
      }
    }
  }
  // Erase the unsuccessful triangulated tracks
  for(auto& it : rejectedId)
  {
    sfmData.structure.erase(it);
  }
}

/// Robustly try to estimate the best 3D point using a ransac Scheme
/// A point must be seen in at least 3 views
/// Return true for a successful triangulation
bool StructureComputation_robust::robust_triangulation(const sfmData::SfMData& sfmData,
                                                       const sfmData::Observations& observations,
                                                       std::mt19937 & randomNumberGenerator,
                                                       Vec3& X,
                                                       const IndexT min_required_inliers,
                                                       const IndexT min_sample_index) const
{
  if (observations.size() < 3)
  {
    return false;
  }

  const double dThresholdPixel = 4.0; // TODO: make this parameter customizable

  const IndexT nbIter = observations.size(); // TODO: automatic computation of the number of iterations?

  // - Ransac variables
  Vec3 best_model;
  std::set<IndexT> best_inlier_set;
  double best_error = std::numeric_limits<double>::max();

  // - Ransac loop
  for(IndexT i = 0; i < nbIter; ++i)
  {
    std::set<IndexT> samples;
    robustEstimation::uniformSample(randomNumberGenerator, std::min(std::size_t(min_sample_index), observations.size()), observations.size(), samples);

    // Hypothesis generation.
    const Vec3 current_model = track_sample_triangulation(sfmData, observations, samples);

    // Test validity of the hypothesis
    // - chierality (for the samples)
    // - residual error

    // Chierality (Check the point is in front of the sampled cameras)
    bool bChierality = true;

    for(auto& it : samples)
    {
      sfmData::Observations::const_iterator itObs = observations.begin();
      std::advance(itObs, it);
      const sfmData::View * view = sfmData.views.at(itObs->first).get();
      const IntrinsicBase * cam = sfmData.getIntrinsics().at(view->getIntrinsicId()).get();
      const Pose3 pose = sfmData.getPose(*view).getTransform();
      const double z = pose.depth(current_model); // TODO: cam->depth(pose(X));
      bChierality &= z > 0;
    }

    if (!bChierality)
      continue;

    std::set<IndexT> inlier_set;
    double current_error = 0.0;
    
    // Classification as inlier/outlier according pixel residual errors.
    for (const auto& itObs : observations)
    {
      const sfmData::View * view = sfmData.views.at(itObs.first).get();
      const IntrinsicBase * intrinsic = sfmData.getIntrinsics().at(view->getIntrinsicId()).get();
      const Pose3 pose = sfmData.getPose(*view).getTransform();
      const Vec2 residual = intrinsic->residual(pose, current_model.homogeneous(), itObs.second.x);
      const double residual_d = residual.norm();

      if (residual_d < dThresholdPixel)
      {
        inlier_set.insert(itObs.first);
        current_error += residual_d;
      }
      else
      {
        current_error += dThresholdPixel;
      }
    }
    // Does the hypothesis is the best one we have seen and have sufficient inliers.
    if (current_error < best_error && inlier_set.size() >= min_required_inliers)
    {
      X = best_model = current_model;
      best_inlier_set = inlier_set;
      best_error = current_error;
    }
  }
  return !best_inlier_set.empty();
}


/// Triangulate a given track from a selection of observations
Vec3 StructureComputation_robust::track_sample_triangulation(const sfmData::SfMData& sfmData,
                                                             const sfmData::Observations& observations,
                                                             const std::set<IndexT>& samples) const
{
  multiview::Triangulation trianObj;
  for (const IndexT idx : samples)
  {
    assert(idx < observations.size());
    sfmData::Observations::const_iterator itObs = observations.begin();
    std::advance(itObs, idx);
    const sfmData::View * view = sfmData.views.at(itObs->first).get();

    std::shared_ptr<camera::IntrinsicBase> cam = sfmData.getIntrinsics().at(view->getIntrinsicId());
    std::shared_ptr<camera::Pinhole> camPinHole = std::dynamic_pointer_cast<camera::Pinhole>(cam);
    if (!camPinHole) {
      ALICEVISION_LOG_ERROR("Camera is not pinhole in filter");
      return Vec3();
    }

    const Pose3 pose = sfmData.getPose(*view).getTransform();
    trianObj.add(
      camPinHole->getProjectiveEquivalent(pose),
      cam->get_ud_pixel(itObs->second.x));
  }
  return trianObj.compute();
}

} // namespace sfm
} // namespace aliceVision
