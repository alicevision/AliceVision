// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#pragma once

#include "openMVG/config.hpp"
#include "openMVG/types.hpp"
#include "openMVG/features/regions.hpp"
#include "openMVG/features/descriptor.hpp"
#include "openMVG/features/ImageDescriberCommon.hpp"

#include <vector>
#include <map>
#include <memory>

namespace openMVG {
namespace localization {


struct ReconstructedRegionsMapping
{
  std::vector<IndexT> _associated3dPoint;
  std::map<IndexT, IndexT> _mapFullToLocal;
};


inline std::unique_ptr<features::Regions> createFilteredRegions(const features::Regions& regions, const std::vector<features::FeatureInImage>& featuresInImage, ReconstructedRegionsMapping& out_mapping)
{
  return regions.createFilteredRegions(featuresInImage, out_mapping._associated3dPoint, out_mapping._mapFullToLocal);
}

using ReconstructedRegionsMappingPerDesc = std::map<features::EImageDescriberType, ReconstructedRegionsMapping>;

using ReconstructedRegionsMappingPerView = std::map<IndexT, ReconstructedRegionsMappingPerDesc>;


} // namespace features
} // namespace openMVG
