
// Copyright (c) 2015 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.
#pragma once

#include <openMVG/types.hpp>
#include <openMVG/sfm/sfm_data.hpp>
#include <openMVG/features/regions.hpp>
#include <openMVG/features/image_describer.hpp>
#include <openMVG/features/ImageDescriberCommon.hpp>
#include "third_party/progress/progress.hpp"

#include <memory>

namespace openMVG {
namespace sfm {

/**
 * @brief Container for all Regions (Features and Descriptors) for each View.
 */
class RegionsPerView
{
public:
  
  const Hash_Map<IndexT, std::unique_ptr<features::Regions> >& getData() const
  {
    return _data;
  }
  
  const features::Regions& getFirstViewRegions() const
  {
    return *_data.begin()->second.get();
  }
  
  const features::Regions& getRegions(IndexT viewId) const
  {
    return *_data.at(viewId).get();
  }
  
  bool viewExist(IndexT viewId) const
  {
    return _data.count(viewId) > 0;
  }
  
  bool isEmpty() const
  {
    return _data.empty();
  }
  
  void addRegions(IndexT viewId, std::unique_ptr<features::Regions> regionsPtr)
  {
    _data[viewId] = std::move(regionsPtr);
  }

  void clearDescriptors()
  {
    for(auto& it: _data)
    {
      it.second->clearDescriptors();
    }
  }
  
private:
  
  /// Regions per ViewId of the considered SfM_Data container
  Hash_Map<IndexT, std::unique_ptr<features::Regions> > _data;
};

/**
 * Load Regions (Features & Descriptors) for each view of the provided SfM_Data container.
 * 
 * @param regionsPerView
 * @param sfmData
 * @param storageDirectory
 * @param imageDescriberType
 * @param filter: to load Regions only for a sub-set of the views contained in the sfmData
 * @return 
 */
bool loadRegionsPerView(RegionsPerView& regionsPerView,
            const SfM_Data& sfmData,
            const std::string& storageDirectory,
            features::EImageDescriberType imageDescriberType,
            const std::set<IndexT>& filter = std::set<IndexT>());


} // namespace sfm
} // namespace openMVG


