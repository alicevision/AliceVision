// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "statistics.hpp"

#include <aliceVision/sfm/SfMData.hpp>

namespace aliceVision {
namespace sfm {

double RMSE(const SfMData& sfmData)
{
  // Compute residuals for each observation
  std::vector<double> vec;
  for(Landmarks::const_iterator iterTracks = sfmData.getLandmarks().begin();
      iterTracks != sfmData.getLandmarks().end();
      ++iterTracks)
  {
    const Observations & obs = iterTracks->second.observations;
    for(Observations::const_iterator itObs = obs.begin();
      itObs != obs.end(); ++itObs)
    {
      const View * view = sfmData.getViews().find(itObs->first)->second.get();
      const geometry::Pose3 pose = sfmData.getPose(*view).getTransform();
      const std::shared_ptr<camera::IntrinsicBase> intrinsic = sfmData.getIntrinsics().at(view->getIntrinsicId());
      const Vec2 residual = intrinsic->residual(pose, iterTracks->second.X, itObs->second.x);
      //ALICEVISION_LOG_DEBUG(residual);
      vec.push_back( residual(0) );
      vec.push_back( residual(1) );
    }
  }
  const Eigen::Map<Eigen::RowVectorXd> residuals(&vec[0], vec.size());
  const double RMSE = std::sqrt(residuals.squaredNorm() / vec.size());
  return RMSE;
}

} // namespace sfm
} // namespace aliceVision
