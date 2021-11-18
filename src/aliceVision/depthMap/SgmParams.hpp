// This file is part of the AliceVision project.
// Copyright (c) 2021 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <string>

namespace aliceVision {

// MultiViewParams forward declaration
namespace mvsUtils { class MultiViewParams; } 

namespace depthMap {

/**
 * @brief Semi Global Matching Parameters
 */
struct SgmParams
{
  // user parameters

  int scale = -1;
  int stepXY = -1;
  int stepZ = -1;
  int wsh = 4;
  int maxTCams = 10;
  int maxDepths = 3000;
  int maxDepthsPerTc = 1500;
  int maxSideXY = 700;
  double gammaC = 5.5;
  double gammaP = 8.0;
  double p1 = 10;
  double p2Weighting = 100.0;
  std::string filteringAxes = "YX";
  bool useSfmSeeds = true;
  bool exportIntermediateResults = false;

  // constant parameters
  
  const bool prematchinMinMaxDepthDontUseSeeds = false;
  const float prematchingMaxDepthScale = 1.5f;
  const float prematchingMinCamDist = 0.0f;
  const float prematchingMaxCamDist = 15.0f;

  const int rcTcDepthsHalfLimit = 2048;
  const int rcDepthsCompStep = 6;
  const double seedsRangeInflate = 0.2;
  const double seedsRangePercentile = 0.999;
  const bool doSgmOptimizeVolume = true;
  const bool interpolateRetrieveBestDepth = false;
  const bool saveDepthsToSweepTxtFile = false;
};

void computeScaleStepSgmParams(const mvsUtils::MultiViewParams& mp, SgmParams& sgmParams);

} // namespace depthMap
} // namespace aliceVision
