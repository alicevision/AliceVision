// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2014 openMVG contributors.
// Copyright (c) 2013 "Robot Vision Group, NLPR, CASIA", Zhenhua Wang, Bin Fan and Fuchao Wu.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/image/all.hpp>
#include <aliceVision/feature/PointFeature.hpp>

#include <map>
#include <vector>

namespace aliceVision {
namespace feature{

/**
 * @brief Local Intensity Order Pattern
 *
 * This code has been adapted from
 * "Robot Vision Group, NLPR, CASIA", Zhenhua Wang, Bin Fan and Fuchao Wu.
 *
 * [1] "Local Intensity Order Pattern for Feature Description"
 * Authors: Zhenhua Wang, Bin Fan and Fuchao Wu
 * Date: 2011, ICCV, IEEE International Conference on Computer Vision
 */

class DescriptorExtractor_LIOP
{
private:
  std::map<int, unsigned char> m_LiopPatternMap;
  std::vector<int> m_LiopPosWeight;

  static const int _maxRegionNum = 10;
  static const int _maxPixelNum  = 1681;
  static const int _maxSampleNum = 10;
  static const int _liopNum = 4;
  static const int _regionNum = 6;
public:

  DescriptorExtractor_LIOP();

  void extract(
    const image::Image<float> & I,
    const PointFeature & feat,
    float desc[144]);

  void CreateLIOP_GOrder(
    const image::Image<float> & outPatch,
    const image::Image<unsigned char> & flagPatch,
    const int inRadius,
    float desc[144]) const;

  void GeneratePatternMap(
    std::map<int,unsigned char> & pattern_map,
    std::vector<int> & pos_weight,
    unsigned char n);
};

} // namespace LIO} // namespace feature
} // namespace aliceVision
