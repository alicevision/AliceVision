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
            _maxTotalKeypoints = 5000;
            _peakThreshold = 0.02f;
            break;
        }
        case EImageDescriberPreset::MEDIUM:
        {
            _maxTotalKeypoints = 10000;
            _peakThreshold = 0.01f;
            break;
        }
        case EImageDescriberPreset::NORMAL:
        {
            _maxTotalKeypoints = 20000;
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
    if(preset.maxNbFeatures > 0)
    {
        _maxTotalKeypoints = preset.maxNbFeatures;
    }
    _firstOctave = 0;
    _numScales = 3;
    switch(preset.quality)
    {
        case EFeatureQuality::LOW:
        {
            // Work on lower image resolution
            _firstOctave = 1;
            break;
        }
        case EFeatureQuality::MEDIUM:
        {
            break;
        }
        case EFeatureQuality::NORMAL:
        {
            break;
        }
        case EFeatureQuality::HIGH:
        {
            // Increase scale space sampling to improve feature repeatability.
            // "An analysis of scale-space sampling in SIFT, Ives Rey-Otero, Jean-Michel Morel, Mauricio Delbracio
            // https://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.703.9294&rep=rep1&type=pdf
            _numScales = 6;
            break;
        }
        case EFeatureQuality::ULTRA:
        {
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

  // if image resolution is low, increase resolution for extraction
  const int firstOctave = params.getImageFirstOctave(width, height);
  if(firstOctave > 0)
      scaleFactor = 1.0 / std::pow(2.0, firstOctave);
  else if(firstOctave < 0)
      scaleFactor = std::pow(2.0, std::abs(firstOctave));
  const std::size_t fullImgSize = width * height * scaleFactor * scaleFactor;

  const int numOctaves = std::max(int(std::floor(std::log2(std::min(width, height))) - firstOctave - 3), 1);

  std::size_t pyramidMemoryConsuption = 0;
  double downscale = 1.0;
  for(int octave = 0; octave < numOctaves; ++octave)
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
    const int numOctaves = -1; // auto
    // if image resolution is low, increase resolution for extraction
    const int firstOctave = params.getImageFirstOctave(w, h);
    VlSiftFilt* filt = vl_sift_new(w, h, numOctaves, params._numScales, firstOctave);
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
        case EFeatureConstrastFiltering::GridSortOctaves:
        case EFeatureConstrastFiltering::GridSort:
        case EFeatureConstrastFiltering::GridSortScaleSteps:
        case EFeatureConstrastFiltering::GridSortOctaveSteps:
        case EFeatureConstrastFiltering::NonExtremaFiltering:
        {
            ALICEVISION_LOG_TRACE("SIFT constrastTreshold: " << EFeatureConstrastFiltering_enumToString(params._contrastFiltering));
            break;
        }
    }

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
        else if(params._maxTotalKeypoints &&
                params._contrastFiltering == EFeatureConstrastFiltering::NonExtremaFiltering)
        {
            std::vector<float> radiusMaxima(nkeys, std::numeric_limits<float>::max());
            for(IndexT i = 0; i < nkeys; ++i)
            {
                const auto& keypointI = keys[i];
                for(IndexT j = 0; j < nkeys; ++j)
                {
                    const auto& keypointJ = keys[j];
                    if(keypointJ.peak_value > keypointI.peak_value)
                    {
                        const float dx = (keypointJ.x - keypointI.x);
                        const float dy = (keypointJ.y - keypointI.y);
                        const float radius = dx * dx + dy * dy;
                        if(radius < radiusMaxima[i])
                            radiusMaxima[i] = radius;
                    }
                }
            }
            filteredKeypointsIndex.resize(nkeys);
            std::iota(filteredKeypointsIndex.begin(), filteredKeypointsIndex.end(), 0);
            const std::size_t maxKeypoints = std::min(params._maxTotalKeypoints, std::size_t(nkeys));
            std::partial_sort(filteredKeypointsIndex.begin(),
                              filteredKeypointsIndex.begin() + maxKeypoints,
                              filteredKeypointsIndex.end(), [&](int a, int b) {
                                  return radiusMaxima[a] * keys[a].sigma > radiusMaxima[b] * keys[b].sigma;
                              });
            filteredKeypointsIndex.resize(maxKeypoints);
        }

        if(filteredKeypointsIndex.empty())
        {
            ALICEVISION_LOG_TRACE("Octave SIFT nb keypoints:\n" << nkeys << " (no grid filtering)");
            filteredKeypointsIndex.resize(nkeys);
            std::iota(filteredKeypointsIndex.begin(), filteredKeypointsIndex.end(), 0);
        }

        // Update gradient before launching parallel extraction
        vl_sift_update_gradient(filt);

        // Feature masking
        if(mask)
        {
            std::vector<IndexT> newFilteredKeypointsIndex;
            const image::Image<unsigned char>& maskIma = *mask;

            for(int ii = 0; ii < filteredKeypointsIndex.size(); ++ii)
            {
                const int i = filteredKeypointsIndex[ii];
                if(maskIma(keys[i].y, keys[i].x) > 0)
                    continue;
                newFilteredKeypointsIndex.push_back(i);
            }
            filteredKeypointsIndex.swap(newFilteredKeypointsIndex);
        }

        std::vector<std::vector<double>> anglesPerKeypoint(filteredKeypointsIndex.size());

#pragma omp parallel for
        for(int ii = 0; ii < filteredKeypointsIndex.size(); ++ii)
        {
            const int i = filteredKeypointsIndex[ii];

            double angles[4] = {0.0, 0.0, 0.0, 0.0};
            int nangles = 1; // by default (1 upright feature)
            if(orientation)
            { // compute from 1 to 4 orientations
                nangles = vl_sift_calc_keypoint_orientations(filt, angles, keys + i);
            }

            Descriptor<vl_sift_pix, 128> vlFeatDescriptor;
            Descriptor<T, 128> descriptor;

            for(int q = 0; q < nangles; ++q)
            {
                const PointFeature fp(keys[i].x, keys[i].y, keys[i].sigma, static_cast<float>(angles[q]));

                vl_sift_calc_keypoint_descriptor(filt, &vlFeatDescriptor[0], keys + i, angles[q]);
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

    assert(regionsCasted->Features().size() == regionsCasted->Descriptors().size());

    // Sorting the extracted features according to their scale
    {
        const auto& features = regionsCasted->Features();
        const auto& descriptors = regionsCasted->Descriptors();

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
                const int scaleA = int(log2(features[a].scale())); // 1 scale steps per octave
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
        std::vector<float> sortedFeaturesPeakValue(features.size());
        for(std::size_t i : indexSort)
        {
            sortedFeatures[i] = features[indexSort[i]];
            sortedDescriptors[i] = descriptors[indexSort[i]];
            sortedFeaturesPeakValue[i] = featuresPeakValue[indexSort[i]];
        }
        regionsCasted->Features().swap(sortedFeatures);
        regionsCasted->Descriptors().swap(sortedDescriptors);
        featuresPeakValue.swap(sortedFeaturesPeakValue);
    }

    if(params._maxTotalKeypoints && params._contrastFiltering == EFeatureConstrastFiltering::NonExtremaFiltering)
    {
        const auto& features = regionsCasted->Features();
        const auto& descriptors = regionsCasted->Descriptors();

        // Only filter features if we have more features than the maxTotalKeypoints
        if(features.size() > params._maxTotalKeypoints)
        {
            std::vector<float> radiusMaxima(features.size(), std::numeric_limits<float>::max());
            for(IndexT i = 0; i < features.size(); ++i)
            {
                const auto& keypointI = features[i];
                for(IndexT j = 0; j < features.size(); ++j)
                {
                    const auto& keypointJ = features[j];
                    if(featuresPeakValue[j] > featuresPeakValue[i])
                    {
                        const float dx = (keypointJ.x() - keypointI.x());
                        const float dy = (keypointJ.y() - keypointI.y());
                        const float radius = dx * dx + dy * dy;
                        if(radius < radiusMaxima[i])
                            radiusMaxima[i] = radius;
                    }
                }
            }
            std::vector<IndexT> indexSort(features.size());
            std::iota(indexSort.begin(), indexSort.end(), 0);
            std::partial_sort(indexSort.begin(),
                              indexSort.begin() + std::min(params._maxTotalKeypoints, features.size()), indexSort.end(),
                              [&](int a, int b) {
                                  return radiusMaxima[a] * features[a].scale() > radiusMaxima[b] * features[b].scale();
                              });
            indexSort.resize(std::min(params._maxTotalKeypoints, features.size()));

            std::vector<PointFeature> filteredFeatures(indexSort.size());
            std::vector<typename SIFT_Region_T::DescriptorT> filteredDescriptors(indexSort.size());
            for(IndexT i = 0; i < indexSort.size(); ++i)
            {
                filteredFeatures[i] = features[indexSort[i]];
                filteredDescriptors[i] = descriptors[indexSort[i]];
            }
            ALICEVISION_LOG_TRACE("SIFT Features: before: " << features.size()
                                                            << ", after grid filtering: " << filteredFeatures.size());
            regionsCasted->Features().swap(filteredFeatures);
            regionsCasted->Descriptors().swap(filteredDescriptors);
        }
    }
    // Grid filtering of the keypoints to ensure a global repartition
    else if(params._gridSize && params._maxTotalKeypoints)
    {
        const auto& features = regionsCasted->Features();
        const auto& descriptors = regionsCasted->Descriptors();
        // Only filter features if we have more features than the maxTotalKeypoints
        if(features.size() > params._maxTotalKeypoints)
        {
            std::vector<IndexT> filteredIndexes;
            std::vector<IndexT> rejectedIndexes;
            filteredIndexes.reserve(std::min(features.size(), params._maxTotalKeypoints));
            rejectedIndexes.reserve(features.size());

            const std::size_t sizeMat = params._gridSize * params._gridSize;
            std::vector<std::size_t> countFeatPerCell(sizeMat, 0);
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
                    filteredIndexes.push_back(i);
                else
                    rejectedIndexes.push_back(i);
            }
            // If we do not have enough features (less than maxTotalKeypoints) after the grid filtering (empty regions in
            // the grid for example). We add the best other ones, without repartition constraint.
            if(filteredIndexes.size() < params._maxTotalKeypoints)
            {
                const std::size_t remainingElements =
                    std::min(rejectedIndexes.size(), params._maxTotalKeypoints - filteredIndexes.size());
                ALICEVISION_LOG_TRACE("Grid filtering -- Copy remaining points: " << remainingElements);
                filteredIndexes.insert(filteredIndexes.end(), rejectedIndexes.begin(),
                                        rejectedIndexes.begin() + remainingElements);
            }

            std::vector<PointFeature> filteredFeatures(filteredIndexes.size());
            std::vector<typename SIFT_Region_T::DescriptorT> filteredDescriptors(filteredIndexes.size());
            for(IndexT i = 0; i < filteredIndexes.size(); ++i)
            {
                filteredFeatures[i] = features[filteredIndexes[i]];
                filteredDescriptors[i] = descriptors[filteredIndexes[i]];
            }

            ALICEVISION_LOG_TRACE("SIFT Features: before: " << features.size()
                                                           << ", after grid filtering: " << filteredFeatures.size());

            regionsCasted->Features().swap(filteredFeatures);
            regionsCasted->Descriptors().swap(filteredDescriptors);
        }
    }
    ALICEVISION_LOG_TRACE("SIFT Features: " << regionsCasted->Features().size()
                                            << " (max: " << params._maxTotalKeypoints << ").");
    assert(regionsCasted->Features().size() == regionsCasted->Descriptors().size());

    return true;
}


template bool extractSIFT<float>(const image::Image<float>& image, std::unique_ptr<Regions>& regions, const SiftParams& params,
                 bool orientation, const image::Image<unsigned char>* mask);

template bool extractSIFT<unsigned char>(const image::Image<float>& image, std::unique_ptr<Regions>& regions,
                                 const SiftParams& params, bool orientation, const image::Image<unsigned char>* mask);

} //namespace feature
} //namespace aliceVision
