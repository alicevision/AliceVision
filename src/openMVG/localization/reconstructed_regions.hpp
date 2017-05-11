
// Copyright (c) 2015 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#pragma once

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
