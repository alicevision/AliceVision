// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#include "aliceVision/sfm/sfm_data_triangulation.hpp"

#include "aliceVision/multiview/triangulation_nview.hpp"
#include "aliceVision/robustEstimation/randSampling.hpp"
#include <aliceVision/config.hpp>

#include "dependencies/progress/progress.hpp"

#include <deque>
#include <memory>

namespace aliceVision {
namespace sfm {

using namespace aliceVision::geometry;
using namespace aliceVision::camera;

SfM_Data_Structure_Computation_Basis::SfM_Data_Structure_Computation_Basis(bool bConsoleVerbose)
  :_bConsoleVerbose(bConsoleVerbose)
{
}

SfM_Data_Structure_Computation_Blind::SfM_Data_Structure_Computation_Blind(bool bConsoleVerbose)
  :SfM_Data_Structure_Computation_Basis(bConsoleVerbose)
{
}

void SfM_Data_Structure_Computation_Blind::triangulate(SfM_Data & sfm_data) const
{
  std::deque<IndexT> rejectedId;
  std::unique_ptr<C_Progress_display> my_progress_bar;
  if (_bConsoleVerbose)
    my_progress_bar.reset( new C_Progress_display(
    sfm_data.structure.size(),
    std::cout,
    "Blind triangulation progress:\n" ));
  #pragma omp parallel
  for(Landmarks::iterator iterTracks = sfm_data.structure.begin();
    iterTracks != sfm_data.structure.end();
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
      Triangulation trianObj;
      const Observations & observations = iterTracks->second.observations;
      for(const auto& itObs : observations)
      {
        const View * view = sfm_data.views.at(itObs.first).get();
        if (sfm_data.IsPoseAndIntrinsicDefined(view))
        {
          const IntrinsicBase * cam = sfm_data.GetIntrinsics().at(view->getIntrinsicId()).get();
          const Pose3 pose = sfm_data.getPose(*view);
          trianObj.add(
            cam->get_projective_equivalent(pose),
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
    sfm_data.structure.erase(it);
  }
}

SfM_Data_Structure_Computation_Robust::SfM_Data_Structure_Computation_Robust(bool bConsoleVerbose)
  :SfM_Data_Structure_Computation_Basis(bConsoleVerbose)
{
}

void SfM_Data_Structure_Computation_Robust::triangulate(SfM_Data & sfm_data) const
{
  robust_triangulation(sfm_data);
}

/// Robust triangulation of track data contained in the structure
/// All observations must have View with valid Intrinsic and Pose data
/// Invalid landmark are removed.
void SfM_Data_Structure_Computation_Robust::robust_triangulation(SfM_Data & sfm_data) const
{
  std::deque<IndexT> rejectedId;
  std::unique_ptr<C_Progress_display> my_progress_bar;
  if (_bConsoleVerbose)
    my_progress_bar.reset( new C_Progress_display(
    sfm_data.structure.size(),
    std::cout,
    "Robust triangulation progress:\n" ));
  #pragma omp parallel
  for(Landmarks::iterator iterTracks = sfm_data.structure.begin();
    iterTracks != sfm_data.structure.end();
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
      if (robust_triangulation(sfm_data, iterTracks->second.observations, X)) {
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
  for (auto& it : rejectedId)
  {
    sfm_data.structure.erase(it);
  }
}

/// Robustly try to estimate the best 3D point using a ransac Scheme
/// A point must be seen in at least 3 views
/// Return true for a successful triangulation
bool SfM_Data_Structure_Computation_Robust::robust_triangulation(
  const SfM_Data & sfm_data,
  const Observations & observations,
  Vec3 & X,
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
  for (IndexT i = 0; i < nbIter; ++i)
  {
    std::set<IndexT> samples;
    robustEstimation::UniformSample(std::min(std::size_t(min_sample_index), observations.size()), observations.size(), samples);

    // Hypothesis generation.
    const Vec3 current_model = track_sample_triangulation(sfm_data, observations, samples);

    // Test validity of the hypothesis
    // - chierality (for the samples)
    // - residual error

    // Chierality (Check the point is in front of the sampled cameras)
    bool bChierality = true;

    for (auto& it : samples)
    {
      Observations::const_iterator itObs = observations.begin();
      std::advance(itObs, it);
      const View * view = sfm_data.views.at(itObs->first).get();
      const IntrinsicBase * cam = sfm_data.GetIntrinsics().at(view->getIntrinsicId()).get();
      const Pose3 pose = sfm_data.getPose(*view);
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
      const View * view = sfm_data.views.at(itObs.first).get();
      const IntrinsicBase * intrinsic = sfm_data.GetIntrinsics().at(view->getIntrinsicId()).get();
      const Pose3 pose = sfm_data.getPose(*view);
      const Vec2 residual = intrinsic->residual(pose, current_model, itObs.second.x);
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
Vec3 SfM_Data_Structure_Computation_Robust::track_sample_triangulation(
  const SfM_Data & sfm_data,
  const Observations & observations,
  const std::set<IndexT> & samples) const
{
  Triangulation trianObj;
  for (const IndexT idx : samples)
  {
    assert(idx < observations.size());
    Observations::const_iterator itObs = observations.begin();
    std::advance(itObs, idx);
    const View * view = sfm_data.views.at(itObs->first).get();
    const IntrinsicBase * cam = sfm_data.GetIntrinsics().at(view->getIntrinsicId()).get();
    const Pose3 pose = sfm_data.getPose(*view);
    trianObj.add(
      cam->get_projective_equivalent(pose),
      cam->get_ud_pixel(itObs->second.x));
  }
  return trianObj.compute();
}

} // namespace sfm
} // namespace aliceVision
