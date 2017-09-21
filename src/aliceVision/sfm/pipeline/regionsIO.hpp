// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#pragma once

#include <aliceVision/types.hpp>
#include <aliceVision/sfm/SfMData.hpp>

#include <aliceVision/feature/ImageDescriber.hpp>
#include <aliceVision/feature/imageDescriberCommon.hpp>
#include <aliceVision/feature/RegionsPerView.hpp>
#include <aliceVision/feature/FeaturesPerView.hpp>

#include <memory>

namespace aliceVision {
namespace sfm {

/**
 * Load Regions (Features & Descriptors) for one view.
 *
 * @param folder
 * @param id_view
 * @param imageDescriber
 *
 * @return Loaded Regions
 */
std::unique_ptr<feature::Regions> loadRegions(const std::string& folder, IndexT id_view, const feature::ImageDescriber& imageDescriber);

/**
 * Load Regions (Features & Descriptors) for each view of the provided SfMData container.
 * 
 * @param regionsPerView
 * @param sfmData
 * @param storageDirectory
 * @param imageDescriberType
 * @param filter: to load Regions only for a sub-set of the views contained in the sfmData
 * @return true if the regions are correctlty loaded
 */
bool loadRegionsPerView(feature::RegionsPerView& regionsPerView,
            const SfMData& sfmData,
            const std::string& storageDirectory,
            const std::vector<feature::EImageDescriberType>& imageDescriberTypes,
            const std::set<IndexT>& filter = std::set<IndexT>());


/**
 * @brief loadFeaturesPerView
 * @param featuresPerView
 * @param sfmData
 * @param folder
 * @param imageDescriberType
 * @return true if the features are correctlty loaded
 */
bool loadFeaturesPerView(feature::FeaturesPerView& featuresPerView,
                    const SfMData& sfmData,
                    const std::string& folder,
                    const std::vector<feature::EImageDescriberType>& imageDescriberTypes);


} // namespace sfm
} // namespace aliceVision


