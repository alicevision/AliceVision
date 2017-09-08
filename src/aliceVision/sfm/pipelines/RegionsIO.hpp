// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#pragma once

#include <aliceVision/types.hpp>
#include <aliceVision/sfm/sfm_data.hpp>

#include <aliceVision/features/image_describer.hpp>
#include <aliceVision/features/ImageDescriberCommon.hpp>
#include <aliceVision/features/RegionsPerView.hpp>
#include <aliceVision/features/FeaturesPerView.hpp>

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
std::unique_ptr<features::Regions> loadRegions(const std::string& folder, IndexT id_view, const features::Image_describer& imageDescriber);

/**
 * Load Regions (Features & Descriptors) for each view of the provided SfM_Data container.
 * 
 * @param regionsPerView
 * @param sfmData
 * @param storageDirectory
 * @param imageDescriberType
 * @param filter: to load Regions only for a sub-set of the views contained in the sfmData
 * @return true if the regions are correctlty loaded
 */
bool loadRegionsPerView(features::RegionsPerView& regionsPerView,
            const SfM_Data& sfmData,
            const std::string& storageDirectory,
            const std::vector<features::EImageDescriberType>& imageDescriberTypes,
            const std::set<IndexT>& filter = std::set<IndexT>());


/**
 * @brief loadFeaturesPerView
 * @param featuresPerView
 * @param sfmData
 * @param folder
 * @param imageDescriberType
 * @return true if the features are correctlty loaded
 */
bool loadFeaturesPerView(features::FeaturesPerView& featuresPerView,
                    const SfM_Data& sfmData,
                    const std::string& folder,
                    const std::vector<features::EImageDescriberType>& imageDescriberTypes);


} // namespace sfm
} // namespace aliceVision


