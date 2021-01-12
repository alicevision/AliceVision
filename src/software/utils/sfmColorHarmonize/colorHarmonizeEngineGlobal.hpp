// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// Copyright (c) 2013 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/feature/feature.hpp>
#include <aliceVision/feature/RegionsPerView.hpp>
#include <aliceVision/track/TracksBuilder.hpp>

#include <memory>

namespace aliceVision {

enum EHistogramSelectionMethod
{
    eHistogramHarmonizeFullFrame     = 0,
    eHistogramHarmonizeMatchedPoints = 1,
    eHistogramHarmonizeVLDSegment    = 2,
};

/**
 * @brief The ColorHarmonizationEngineGlobal class
 *
 * [1] "Global Multiple-View Color Consistency."
 * Authors: Pierre MOULON, Bruno DUISIT and Pascal MONASSE
 * Date: November 2013
 * Conference: CVMP
 */
class ColorHarmonizationEngineGlobal
{
public:
  ColorHarmonizationEngineGlobal(
    const std::string& sfmDataFilename,
    const std::vector<std::string>& featuresFolders,
    const std::vector<std::string>& matchesFolders,
    const std::string& outputDirectory,
    const std::vector<feature::EImageDescriberType>& descTypes,
    int selectionMethod = -1,
    int imgRef = -1);

  ~ColorHarmonizationEngineGlobal();

  virtual bool Process();

private:

  EHistogramSelectionMethod _selectionMethod;
  int _imgRef;

  // Input data

  feature::RegionsPerView _regionsPerView;
  /// considered images
  std::vector<std::string> _fileNames;
  /// size of each image
  std::vector<std::pair<size_t, size_t>> _imageSize;
  /// pairwise geometric matches
  aliceVision::matching::PairwiseMatches _pairwiseMatches;
  /// describer type use for color harmonizations
  std::vector<feature::EImageDescriberType> _descTypes;
  /// path to the Sfm Scene
  std::string _sfmDataFilename;
  /// path to matches
  std::vector<std::string> _matchesFolders;
  /// path to features
  std::vector<std::string> _featuresFolders;
  /// output path where outputs will be stored
  std::string _outputDirectory;

  /// Clean graph
  bool CleanGraph();

  /// Read input data (point correspondences)
  bool ReadInputData();
};

} // namespace aliceVision
