// This file is part of the AliceVision project.
// Copyright (c) 2021 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <string>

namespace aliceVision {
namespace depthMap {

/**
 * @brief Semi Global Matching Parameters
 */
struct SgmParams
{
  // user parameters

  int scale = 2;
  int stepXY = 2;
  int stepZ = -1;
  int wsh = 4;
  int maxDepths = 1500;
  int maxTCamsPerTile = 4;
  double seedsRangeInflate = 0.2;
  double depthThicknessInflate = 0.0;
  double maxSimilarity = 1.0;
  double gammaC = 5.5;
  double gammaP = 8.0;
  double p1 = 10;
  double p2Weighting = 100.0;
  std::string filteringAxes = "YX";
  bool useSfmSeeds = true;
  bool depthListPerTile = false;
  bool useConsistentScale = false;
  bool useCustomPatchPattern = false;

  // intermediate results export parameters

  bool exportIntermediateDepthSimMaps = false;
  bool exportIntermediateNormalMaps = false;
  bool exportIntermediateVolumes = false;
  bool exportIntermediateCrossVolumes = false;
  bool exportIntermediateTopographicCutVolumes = false;
  bool exportIntermediateVolume9pCsv = false;
  const bool exportDepthsTxtFiles = false;

  // constant parameters

  const bool updateUninitializedSim = true; // should always be true, false for debug purposes
  const float prematchingMaxDepthScale = 1.5f;
  const double seedsRangePercentile = 0.999;
  const bool doSgmOptimizeVolume = true;
  
};

} // namespace depthMap
} // namespace aliceVision
