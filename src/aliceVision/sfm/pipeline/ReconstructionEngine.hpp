// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmData/colorize.hpp>

#include <string>
#include <random>

namespace aliceVision {
namespace sfm {

void retrieveMarkersId(sfmData::SfMData& sfmData);


/**
 * @brief Basic Reconstruction Engine.
 * Process Function handle the reconstruction.
 */
class ReconstructionEngine
{
public:

  /**
   * @brief ReconstructionEngine Constructor
   * @param[in] sfmData The input SfMData of the scene
   * @param[in] outFolder The folder where outputs will be stored
   */
  ReconstructionEngine(const sfmData::SfMData& sfmData, const std::string& outFolder)
    : _outputFolder(outFolder)
    , _sfmData(sfmData)
  {}

  virtual ~ReconstructionEngine() {}

  /**
   * @brief Reconstruction process
   * @return true if the scene is reconstructed
   */
  virtual bool process() = 0;

  /**
   * @brief Get the scene SfMData
   * @return SfMData
   */
  inline const sfmData::SfMData& getSfMData() const
  {
    return _sfmData;
  }

  /**
   * @brief Get the scene SfMData
   * @return SfMData
   */
  inline sfmData::SfMData& getSfMData()
  {
    return _sfmData;
  }

  /**
   * @brief Colorization of the reconstructed scene
   * @return true if ok
   */
  inline void colorize()
  {
    sfmData::colorizeTracks(_sfmData);
  }

  void retrieveMarkersId()
  {
      aliceVision::sfm::retrieveMarkersId(_sfmData);
  }

  void initRandomSeed(int seed)
  {
      _randomNumberGenerator.seed(seed == -1 ? std::random_device()() : seed);
  }

protected:
  /// Output folder where outputs will be stored
  std::string _outputFolder;
  /// Internal SfMData
  sfmData::SfMData _sfmData;
  //Random engine
  std::mt19937 _randomNumberGenerator;
};


} // namespace sfm
} // namespace aliceVision
