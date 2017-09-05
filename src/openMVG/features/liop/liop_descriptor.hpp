// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#pragma once

#include <openMVG/image/image.hpp>
#include <openMVG/features/feature.hpp>

#include <map>
#include <vector>

namespace openMVG {
namespace features{
namespace LIOP    {

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

class Liop_Descriptor_Extractor
{
private:
  std::map<int, unsigned char> m_LiopPatternMap;
  std::vector<int> m_LiopPosWeight;
public:

  Liop_Descriptor_Extractor();

  void extract(
    const image::Image<unsigned char> & I,
    const SIOPointFeature & feat,
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

} // namespace LIOP
} // namespace features
} // namespace openMVG
