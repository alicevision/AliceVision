#pragma once

#include <openMVG/types.hpp>
#include <openMVG/sfm/sfm_data.hpp>

#include <openMVG/features/image_describer.hpp>
#include <openMVG/features/ImageDescriberCommon.hpp>
#include <openMVG/features/RegionsPerView.hpp>
#include <openMVG/features/FeaturesPerView.hpp>

#include <memory>

namespace openMVG {
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
 * @return 
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
 * @return
 */
bool loadFeaturesPerView(features::FeaturesPerView& featuresPerView,
                    const SfM_Data& sfmData,
                    const std::string& folder,
                    const std::vector<features::EImageDescriberType>& imageDescriberTypes);


} // namespace sfm
} // namespace openMVG


