// This file is part of the AliceVision project.
// Copyright (c) 2021 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

namespace aliceVision {
namespace depthMap {

/**
 * @brief Refine Parameters
 */
struct RefineParams
{
  // user parameters

  int scale = 1;
  int stepXY = 1;
  int wsh = 3;
  int halfNbDepths = 15;
  int nbSubsamples = 10;
  int maxTCamsPerTile = 4;
  int optimizationNbIterations = 100;
  double sigma = 15.0;
  double gammaC = 15.5;
  double gammaP = 8.0;
  bool interpolateMiddleDepth = false;
  bool useConsistentScale = false;
  bool useCustomPatchPattern = false;
  bool useRefineFuse = true;
  bool useColorOptimization = true;

  // intermediate results export parameters

  bool exportIntermediateDepthSimMaps = false;
  bool exportIntermediateNormalMaps = false;
  bool exportIntermediateCrossVolumes = false;
  bool exportIntermediateTopographicCutVolumes = false;
  bool exportIntermediateVolume9pCsv = false;

  // constant parameters

  const bool useSgmNormalMap = false; // for experimentation purposes
};

} // namespace depthMap
} // namespace aliceVision
