// This file is part of the AliceVision project.
// Copyright (c) 2018 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "SIFT.hpp"

namespace aliceVision {
namespace feature {

int VLFeatInstance::nbInstances = 0;

void SiftParams::setPreset(ConfigurationPreset preset)
{
    switch(preset.descPreset)
    {
        case EImageDescriberPreset::LOW:
        {
            _maxTotalKeypoints = 1000;
            _peakThreshold = 0.02f;
            break;
        }
        case EImageDescriberPreset::MEDIUM:
        {
            _maxTotalKeypoints = 5000;
            _peakThreshold = 0.01f;
            break;
        }
        case EImageDescriberPreset::NORMAL:
        {
            _maxTotalKeypoints = 10000;
            _peakThreshold = 0.005f;
            break;
        }
        case EImageDescriberPreset::HIGH:
        {
            _maxTotalKeypoints = 50000;
            _peakThreshold = 0.001f;
            break;
        }
        case EImageDescriberPreset::ULTRA:
        {
            _maxTotalKeypoints = 100000;
            _peakThreshold = 0.0001f;
            break;
        }
        default:
            throw std::out_of_range("Invalid image describer preset enum");
    }
    switch(preset.quality)
    {
        case EFeatureQuality::LOW:
        {
            _firstOctave = 2;
            _numScales = 3;
            break;
        }
        case EFeatureQuality::MEDIUM:
        {
            _firstOctave = 1;
            _numScales = 3;
            break;
        }
        case EFeatureQuality::NORMAL:
        {
            _firstOctave = 0;
            _numScales = 3;
            break;
        }
        case EFeatureQuality::HIGH:
        {
            _firstOctave = 0;
            _numScales = 6;
            break;
        }
        case EFeatureQuality::ULTRA:
        {
            _firstOctave = -1;
            _numScales = 6;
            break;
        }
    }
    if(!preset.gridFiltering)
    {
        _gridSize = 0;
    }
    _contrastFiltering = preset.contrastFiltering;
}


std::size_t getMemoryConsumptionVLFeat(std::size_t width, std::size_t height, const SiftParams& params)
{
  double scaleFactor = 1.0;
  if(params._firstOctave > 0)
      scaleFactor = 1.0 / std::pow(2.0, params._firstOctave);
  else if(params._firstOctave < 0)
      scaleFactor = std::pow(2.0, std::abs(params._firstOctave));
  const std::size_t fullImgSize = width * height * scaleFactor * scaleFactor;

  std::size_t pyramidMemoryConsuption = 0;
  double downscale = 1.0;
  for(int octave = 0; octave < params._numOctaves; ++octave)
  {
    pyramidMemoryConsuption += fullImgSize / (downscale*downscale);
    downscale *= 2.0;
  }
  pyramidMemoryConsuption *= params._numScales * sizeof(float);

  const int nbTempPyramids = 4; // Gaussian + DOG + Gradiant + orientation (Note: DOG use 1 layer less, but this is ignored here)
  return fullImgSize * 4 * sizeof(float) + // input RGBA image
         nbTempPyramids * pyramidMemoryConsuption + // pyramids
         (params._maxTotalKeypoints * 128 * sizeof(float)); // output keypoints
}

void VLFeatInstance::initialize()
{
  assert(nbInstances >= 0);
  if(nbInstances <= 0)
    vl_constructor();
  ++nbInstances;
}

void VLFeatInstance::destroy()
{
  assert(nbInstances > 0);
  --nbInstances;
  if(nbInstances <= 0)
    vl_destructor();
}

template <typename T>
bool extractSIFT(const image::Image<float>& image, std::unique_ptr<Regions>& regions, const SiftParams& params,
                 bool orientation, const image::Image<unsigned char>* mask)
{
    const int w = image.Width(), h = image.Height();
    const int numOctaves = params._numOctaves + (params._firstOctave < 0 ? 1 : 0); // if first octave is -a, add one octave
    VlSiftFilt* filt = vl_sift_new(w, h, numOctaves, params._numScales, params._firstOctave);
    if(params._edgeThreshold >= 0)
        vl_sift_set_edge_thresh(filt, params._edgeThreshold);

    switch(params._contrastFiltering)
    {
        case EFeatureConstrastFiltering::Static:
        {
            ALICEVISION_LOG_TRACE("SIFT constrastTreshold Static: " << params._peakThreshold);
            if(params._peakThreshold >= 0)
            {
                vl_sift_set_peak_thresh(filt, params._peakThreshold / params._numScales);
            }
            break;
        }
        case EFeatureConstrastFiltering::AdaptiveToMedianVariance:
        {
            const float medianOfGradiants = computeAutomaticContrastFactor(image, 0.5);
            const float relativePeakThreshold = 0.02f;
            const float dynPeakTreshold = relativePeakThreshold * medianOfGradiants;
            ALICEVISION_LOG_TRACE("SIFT relativePeakThreshold * medianOfGradiants: \n"
                                  << " - relativePeakThreshold: " << relativePeakThreshold << "\n"
                                  << " - medianOfGradiants: " << medianOfGradiants << "\n"
                                  << " - peakTreshold: " << dynPeakTreshold);
            vl_sift_set_peak_thresh(filt, dynPeakTreshold / params._numScales);
            break;
        }
        case EFeatureConstrastFiltering::NoFiltering:
        {
            ALICEVISION_LOG_TRACE("SIFT constrastTreshold NoFiltering.");
            break;
        }
        case EFeatureConstrastFiltering::GridSortOctaves:
        {
            ALICEVISION_LOG_TRACE("SIFT constrastTreshold GridSortOctaves.");
            break;
        }
        case EFeatureConstrastFiltering::GridSort:
        {
            ALICEVISION_LOG_TRACE("SIFT constrastTreshold GridSort.");
            break;
        }
        case EFeatureConstrastFiltering::GridSortScaleSteps:
        case EFeatureConstrastFiltering::GridSortOctaveSteps:
        {
            break;
        }
    }

    Descriptor<vl_sift_pix, 128> vlFeatDescriptor;
    Descriptor<T, 128> descriptor;

    // Process SIFT computation
    vl_sift_process_first_octave(filt, image.data());

    using SIFT_Region_T = ScalarRegions<T, 128>;
    SIFT_Region_T* regionsCasted = new SIFT_Region_T();
    regions.reset(regionsCasted);

    // Build alias to cached data
    // reserve some memory for faster keypoint saving
    const std::size_t reserveSize = (params._gridSize && params._maxTotalKeypoints) ? params._maxTotalKeypoints : 2000;
    regionsCasted->Features().reserve(reserveSize);
    regionsCasted->Descriptors().reserve(reserveSize);
    std::vector<float> featuresPeakValue;
    featuresPeakValue.reserve(reserveSize);

    // TODO: params._numOctaves
    size_t maxOctaveKeypoints = params._maxTotalKeypoints;

    while(true)
    {
        vl_sift_detect(filt);

        VlSiftKeypoint const* keys = vl_sift_get_keypoints(filt);
        const int nkeys = vl_sift_get_nkeypoints(filt);

        std::vector<IndexT> filteredKeypointsIndex;

        // TODO: should we reduce maxOctaveKeypoints per octave?

        // grid filtering at the octave level
        if(params._gridSize && params._maxTotalKeypoints &&
           (params._contrastFiltering == EFeatureConstrastFiltering::GridSort ||
            params._contrastFiltering == EFeatureConstrastFiltering::GridSortScaleSteps ||
            params._contrastFiltering == EFeatureConstrastFiltering::GridSortOctaves))
        {
            // Only filter features if we have more features than the maxTotalKeypoints
            if(nkeys > maxOctaveKeypoints)
            {
                // Sorting the extracted features according to their dog value (peak threshold)
                std::vector<std::size_t> keysIndexSort(nkeys);
                std::iota(keysIndexSort.begin(), keysIndexSort.end(), 0);

                if(params._contrastFiltering == EFeatureConstrastFiltering::GridSortScaleSteps)
                {
                    std::sort(keysIndexSort.begin(), keysIndexSort.end(), [&](std::size_t a, std::size_t b) {
                        const int scaleA = int(log2(keys[a].sigma) * 3.0f); // 3 scale steps per octave
                        const int scaleB = int(log2(keys[b].sigma) * 3.0f);
                        if(scaleA == scaleB)
                        {
                            // sort by peak value, when we are in the same scale
                            return keys[a].peak_value > keys[b].peak_value;
                        }
                        return scaleA > scaleB;
                    });
                }
                else if(params._contrastFiltering == EFeatureConstrastFiltering::GridSortOctaveSteps)
                {
                    std::sort(keysIndexSort.begin(), keysIndexSort.end(), [&](std::size_t a, std::size_t b) {
                        const int scaleA = int(log2(keys[a].sigma)); // 3 scale steps per octave
                        const int scaleB = int(log2(keys[b].sigma));
                        if(scaleA == scaleB)
                        {
                            // sort by peak value, when we are in the same scale
                            return keys[a].peak_value > keys[b].peak_value;
                        }
                        return scaleA > scaleB;
                    });
                }
                else if(params._contrastFiltering == EFeatureConstrastFiltering::GridSort)
                {
                    std::sort(keysIndexSort.begin(), keysIndexSort.end(), [&](std::size_t a, std::size_t b) {
                        return keys[a].sigma * keys[a].peak_value > keys[b].sigma * keys[b].peak_value;
                    });
                }
                else // GridSortOctaves
                {
                    // sort from largest peaks to smallest ones
                    std::sort(keysIndexSort.begin(), keysIndexSort.end(),
                              [&](std::size_t a, std::size_t b) { return keys[a].peak_value > keys[b].peak_value; });
                }

                std::vector<IndexT> rejected_indexes;
                filteredKeypointsIndex.reserve(maxOctaveKeypoints);
                rejected_indexes.reserve(nkeys);

                const std::size_t sizeMat = params._gridSize * params._gridSize;
                std::vector<std::size_t> countFeatPerCell(sizeMat, 0);
                for(int idx = 0; idx < sizeMat; ++idx)
                {
                    countFeatPerCell[idx] = 0;
                }
                const std::size_t keypointsPerCell = params._maxTotalKeypoints / sizeMat;
                const double regionWidth = w / double(params._gridSize);
                const double regionHeight = h / double(params._gridSize);

                for(IndexT ii = 0; ii < nkeys; ++ii)
                {
                    const IndexT i = keysIndexSort[ii]; // use sorted keypoints
                    const auto& keypoint = keys[i];

                    const std::size_t cellX = std::min(std::size_t(keypoint.x / regionWidth), params._gridSize);
                    const std::size_t cellY = std::min(std::size_t(keypoint.y / regionHeight), params._gridSize);

                    std::size_t& count = countFeatPerCell[cellX * params._gridSize + cellY];
                    ++count;

                    if(count < keypointsPerCell)
                        filteredKeypointsIndex.push_back(i);
                    else
                        rejected_indexes.push_back(i);
                }
                // If we don't have enough features (less than maxTotalKeypoints) after the grid filtering (empty
                // regions in the grid for example). We add the best other ones, without repartition constraint.
                if(filteredKeypointsIndex.size() < params._maxTotalKeypoints && !rejected_indexes.empty())
                {
                    const std::size_t remainingElements =
                        std::min(rejected_indexes.size(), params._maxTotalKeypoints - filteredKeypointsIndex.size());
                    ALICEVISION_LOG_TRACE("Octave Grid filtering -- Copy remaining points: " << remainingElements);
                    filteredKeypointsIndex.insert(filteredKeypointsIndex.end(), rejected_indexes.begin(),
                                            rejected_indexes.begin() + remainingElements);
                }

                ALICEVISION_LOG_TRACE("Octave SIFT keypoints:\n"
                                      << " * detected: " << nkeys << "\n"
                                      << " * max octave keypoints: " << maxOctaveKeypoints << "\n"
                                      << " * after grid filtering: " << filteredKeypointsIndex.size());
            }
        }

        if(filteredKeypointsIndex.empty())
        {
            ALICEVISION_LOG_TRACE("Octave SIFT nb keypoints:\n" << nkeys << " (no grid filtering)");
            filteredKeypointsIndex.resize(nkeys);
            std::iota(filteredKeypointsIndex.begin(), filteredKeypointsIndex.end(), 0);
        }

        // Update gradient before launching parallel extraction
        vl_sift_update_gradient(filt);

#pragma omp parallel for private(vlFeatDescriptor, descriptor)
        for(int ii = 0; ii < filteredKeypointsIndex.size(); ++ii)
        {
            const int i = filteredKeypointsIndex[ii];

            // Feature masking
            if(mask)
            {
                const image::Image<unsigned char>& maskIma = *mask;
                if(maskIma(keys[i].y, keys[i].x) > 0)
                    continue;
            }

            double angles[4] = {0.0, 0.0, 0.0, 0.0};
            int nangles = 1; // by default (1 upright feature)
            if(orientation)
            { // compute from 1 to 4 orientations
                nangles = vl_sift_calc_keypoint_orientations(filt, angles, keys + i);
            }

            for(int q = 0; q < nangles; ++q)
            {
                vl_sift_calc_keypoint_descriptor(filt, &vlFeatDescriptor[0], keys + i, angles[q]);
                const PointFeature fp(keys[i].x, keys[i].y, keys[i].sigma, static_cast<float>(angles[q]));

                convertSIFT<T>(&vlFeatDescriptor[0], descriptor, params._rootSift);

#pragma omp critical
                {
                    regionsCasted->Descriptors().push_back(descriptor);
                    regionsCasted->Features().push_back(fp);
                    featuresPeakValue.push_back(keys[i].peak_value);
                }
            }
        }

        if(vl_sift_process_next_octave(filt))
            break; // Last octave
    }
    vl_sift_delete(filt);

    const auto& features = regionsCasted->Features();
    const auto& descriptors = regionsCasted->Descriptors();
    assert(features.size() == descriptors.size());

    // Sorting the extracted features according to their scale
    {
        std::vector<std::size_t> indexSort(features.size());
        std::iota(indexSort.begin(), indexSort.end(), 0);
        if(params._contrastFiltering == EFeatureConstrastFiltering::GridSortScaleSteps)
        {
            std::sort(indexSort.begin(), indexSort.end(), [&](std::size_t a, std::size_t b) {
                const int scaleA = int(log2(features[a].scale()) * 3.0f); // 3 scale steps per octave
                const int scaleB = int(log2(features[b].scale()) * 3.0f);
                if(scaleA == scaleB)
                {
                    return featuresPeakValue[a] > featuresPeakValue[b];
                }
                return scaleA > scaleB;
            });
        }
        else if(params._contrastFiltering == EFeatureConstrastFiltering::GridSortOctaveSteps)
        {
            std::sort(indexSort.begin(), indexSort.end(), [&](std::size_t a, std::size_t b) {
                const int scaleA = int(log2(features[a].scale())); // 3 scale steps per octave
                const int scaleB = int(log2(features[b].scale()));
                if(scaleA == scaleB)
                {
                    return featuresPeakValue[a] > featuresPeakValue[b];
                }
                return scaleA > scaleB;
            });
        }
        else if(params._contrastFiltering == EFeatureConstrastFiltering::GridSort)
        {
            std::sort(indexSort.begin(), indexSort.end(), [&](std::size_t a, std::size_t b) {
                return features[a].scale() * featuresPeakValue[a] > features[b].scale() * featuresPeakValue[b];
            });
        }
        else
        {
            // sort from largest scales to smallest ones
            std::sort(indexSort.begin(), indexSort.end(), [&](std::size_t a, std::size_t b) {
                return features[a].scale() > features[b].scale();
            });
        }

        std::vector<PointFeature> sortedFeatures(features.size());
        std::vector<typename SIFT_Region_T::DescriptorT> sortedDescriptors(features.size());
        for(std::size_t i : indexSort)
        {
            sortedFeatures[i] = features[indexSort[i]];
            sortedDescriptors[i] = descriptors[indexSort[i]];
        }
        regionsCasted->Features().swap(sortedFeatures);
        regionsCasted->Descriptors().swap(sortedDescriptors);
    }

    // Grid filtering of the keypoints to ensure a global repartition
    if(params._gridSize && params._maxTotalKeypoints)
    {
        // Only filter features if we have more features than the maxTotalKeypoints
        if(features.size() > params._maxTotalKeypoints)
        {
            std::vector<IndexT> filtered_indexes;
            std::vector<IndexT> rejected_indexes;
            filtered_indexes.reserve(std::min(features.size(), params._maxTotalKeypoints));
            rejected_indexes.reserve(features.size());

            const std::size_t sizeMat = params._gridSize * params._gridSize;
            std::vector<std::size_t> countFeatPerCell(sizeMat, 0);
            for(int Indice = 0; Indice < sizeMat; Indice++)
            {
                countFeatPerCell[Indice] = 0;
            }
            const std::size_t keypointsPerCell = params._maxTotalKeypoints / sizeMat;
            const double regionWidth = w / double(params._gridSize);
            const double regionHeight = h / double(params._gridSize);

            for(IndexT i = 0; i < features.size(); ++i)
            {
                const auto& keypoint = features.at(i);

                const std::size_t cellX = std::min(std::size_t(keypoint.x() / regionWidth), params._gridSize);
                const std::size_t cellY = std::min(std::size_t(keypoint.y() / regionHeight), params._gridSize);

                std::size_t& count = countFeatPerCell[cellX * params._gridSize + cellY];
                ++count;

                if(count < keypointsPerCell)
                    filtered_indexes.push_back(i);
                else
                    rejected_indexes.push_back(i);
            }
            // If we don't have enough features (less than maxTotalKeypoints) after the grid filtering (empty regions in
            // the grid for example). We add the best other ones, without repartition constraint.
            if(filtered_indexes.size() < params._maxTotalKeypoints)
            {
                const std::size_t remainingElements =
                    std::min(rejected_indexes.size(), params._maxTotalKeypoints - filtered_indexes.size());
                ALICEVISION_LOG_TRACE("Grid filtering -- Copy remaining points: " << remainingElements);
                filtered_indexes.insert(filtered_indexes.end(), rejected_indexes.begin(),
                                        rejected_indexes.begin() + remainingElements);
            }

            std::vector<PointFeature> filtered_features(filtered_indexes.size());
            std::vector<typename SIFT_Region_T::DescriptorT> filtered_descriptors(filtered_indexes.size());
            for(IndexT i = 0; i < filtered_indexes.size(); ++i)
            {
                filtered_features[i] = features[filtered_indexes[i]];
                filtered_descriptors[i] = descriptors[filtered_indexes[i]];
            }

            ALICEVISION_LOG_TRACE("SIFT Features: before: " << features.size()
                                                           << ", after grid filtering: " << filtered_features.size());

            regionsCasted->Features().swap(filtered_features);
            regionsCasted->Descriptors().swap(filtered_descriptors);
        }
    }
    ALICEVISION_LOG_TRACE("SIFT Features: " << features.size() << " (max: " << params._maxTotalKeypoints << ").");
    assert(features.size() == descriptors.size());

    return true;
}


template bool extractSIFT<float>(const image::Image<float>& image, std::unique_ptr<Regions>& regions, const SiftParams& params,
                 bool orientation, const image::Image<unsigned char>* mask);

template bool extractSIFT<unsigned char>(const image::Image<float>& image, std::unique_ptr<Regions>& regions,
                                 const SiftParams& params, bool orientation, const image::Image<unsigned char>* mask);

} //namespace feature
} //namespace aliceVision
