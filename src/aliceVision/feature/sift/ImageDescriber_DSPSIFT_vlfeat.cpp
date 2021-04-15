// This file is part of the AliceVision project.
// Copyright (c) 2020 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "ImageDescriber_DSPSIFT_vlfeat.hpp"

#include <aliceVision/feature/Descriptor.hpp>
#include <aliceVision/feature/ImageDescriber.hpp>
#include <aliceVision/feature/regionsFactory.hpp>
#include <aliceVision/feature/sift/SIFT.hpp>

extern "C" {
#include <nonFree/sift/vl/covdet.h>
#include <nonFree/sift/vl/sift.h>
}

#include <iostream>
#include <numeric>

namespace aliceVision {
namespace feature {


void DspSiftParams::setPreset(ConfigurationPreset preset)
{
    SiftParams::setPreset(preset);

    domainSizePooling = true;
    estimateAffineShape = false;

    // Parameters selection in the original paper:
    // "Improvements are observed as soon as more than one scale is used,
    // with diminishing return: Performance decreases with domain size pooling radius exceeding sigma/2.
    // Although the more samples the merrier, three size samples are sufficient to outperform ordinary SIFT,
    // and improvement beyond 10 samples is minimal. Additional samples do not further increase the mean average
    // precision, but incur more computational cost."
    switch(preset.quality)
    {
        case EFeatureQuality::LOW:
        {
            domainSizePooling = false;
            dspNumScales = 1;
            break;
        }
        case EFeatureQuality::MEDIUM:
        {
            dspNumScales = 3;
            break;
        }
        case EFeatureQuality::NORMAL:
        {
            dspNumScales = 6;
            break;
        }
        case EFeatureQuality::HIGH:
        {
            dspNumScales = 9;
            break;
        }
        case EFeatureQuality::ULTRA:
        {
            dspNumScales = 10;
            break;
        }
    }
}

template <typename T>
bool extractDSPSIFT(const image::Image<float>& image, std::unique_ptr<Regions>& regions, const DspSiftParams& params,
                    bool orientation, const image::Image<unsigned char>* mask)
{
    const int w = image.Width(), h = image.Height();
    // Setup covariant SIFT detector.
    std::unique_ptr<VlCovDet, void (*)(VlCovDet*)> covdet(vl_covdet_new(VL_COVDET_METHOD_DOG), &vl_covdet_delete);

    // if image resolution is low, increase resolution for extraction
    const int firstOctave = params.getImageFirstOctave(w, h);
    // keep numOctaves=auto (-1)
    vl_covdet_set_first_octave(covdet.get(), firstOctave);
    vl_covdet_set_octave_resolution(covdet.get(), params._numScales);
    vl_covdet_set_edge_threshold(covdet.get(), params._edgeThreshold);
    // vl_covdet_set_non_extrema_suppression_threshold(covdet.get(), 0.0f); // vl_covdet default is 0.5

    switch(params._contrastFiltering)
    {
        case EFeatureConstrastFiltering::Static:
        {
            ALICEVISION_LOG_TRACE("SIFT constrastTreshold Static: " << params._peakThreshold);
            if(params._peakThreshold >= 0)
            {
                vl_covdet_set_peak_threshold(covdet.get(), params._peakThreshold / params._numScales);
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
            vl_covdet_set_peak_threshold(covdet.get(), dynPeakTreshold / params._numScales);
            break;
        }
        case EFeatureConstrastFiltering::NoFiltering:
        case EFeatureConstrastFiltering::GridSortOctaves:
        case EFeatureConstrastFiltering::GridSortOctaveSteps:
        case EFeatureConstrastFiltering::GridSort:
        case EFeatureConstrastFiltering::GridSortScaleSteps:
        case EFeatureConstrastFiltering::NonExtremaFiltering:
        {
            vl_covdet_set_peak_threshold(covdet.get(), 0.0);
            ALICEVISION_LOG_TRACE("DSP-SIFT constrastTreshold: " << EFeatureConstrastFiltering_enumToString(params._contrastFiltering));
            break;
        }
    }

    vl_covdet_put_image(covdet.get(), image.data(), image.Width(), image.Height());

    vl_covdet_detect(covdet.get());

    const double kBoundaryMargin = 2.0;
    vl_covdet_drop_features_outside(covdet.get(), kBoundaryMargin);

    if(orientation)
    {
        VlCovDetBuffer internalBuffer;
        vl_covdetbuffer_init(&internalBuffer);
        if(params.estimateAffineShape)
        {
            vl_covdet_extract_affine_shape(covdet.get(), &internalBuffer);
        }

        vl_covdet_extract_orientations(covdet.get(), &internalBuffer);

        vl_covdetbuffer_clear(&internalBuffer);
    }

    VlCovDet const* plopD = covdet.get();

    int nbFeatures = vl_covdet_get_num_features(covdet.get());
    VlCovDetFeature* features = vl_covdet_get_features(covdet.get());

    using SIFT_Region_T = ScalarRegions<T, 128>;
    SIFT_Region_T* regionsCasted = new SIFT_Region_T();
    regions.reset(regionsCasted);

    // Build alias to cached data
    // reserve some memory for faster keypoint saving
    const std::size_t reserveSize = (params._gridSize && params._maxTotalKeypoints) ? params._maxTotalKeypoints : nbFeatures;
    regionsCasted->Features().reserve(reserveSize);

    std::vector<float> featuresPeakValue(nbFeatures, 0.0f);

    std::vector<IndexT> indexSort(nbFeatures);
    // Sorting the extracted features according to their scale
    {
        std::iota(indexSort.begin(), indexSort.end(), 0);
        if(params._contrastFiltering == EFeatureConstrastFiltering::GridSortScaleSteps)
        {
            std::sort(indexSort.begin(), indexSort.end(), [&](std::size_t a, std::size_t b) {
                const int scaleA = int(log2(features[a].sigma) * 3.0f); // 3 scale steps per octave
                const int scaleB = int(log2(features[b].sigma) * 3.0f);
                if(scaleA == scaleB)
                {
                    return features[a].peakScore > features[b].peakScore;
                }
                return scaleA > scaleB;
            });
        }
        else if(params._contrastFiltering == EFeatureConstrastFiltering::GridSortOctaveSteps)
        {
            std::sort(indexSort.begin(), indexSort.end(), [&](std::size_t a, std::size_t b) {
                const int scaleA = int(log2(features[a].sigma)); // 3 scale steps per octave
                const int scaleB = int(log2(features[b].sigma));
                if(scaleA == scaleB)
                {
                    return features[a].peakScore > features[b].peakScore;
                }
                return scaleA > scaleB;
            });
        }
        else if(params._contrastFiltering == EFeatureConstrastFiltering::GridSort)
        {
            std::sort(indexSort.begin(), indexSort.end(), [&](std::size_t a, std::size_t b) {
                return features[a].sigma * features[a].peakScore > features[b].sigma * features[b].peakScore;
            });
        }
        else
        {
            // sort from largest scales to smallest ones
            std::sort(indexSort.begin(), indexSort.end(),
                      [&](std::size_t a, std::size_t b) { return features[a].sigma > features[b].sigma; });
        }
    }

    if(params._maxTotalKeypoints && params._contrastFiltering == EFeatureConstrastFiltering::NonExtremaFiltering)
    {
        // Only filter features if we have more features than the maxTotalKeypoints
        if(indexSort.size() > params._maxTotalKeypoints)
        {
            std::vector<float> radiusMaxima(indexSort.size(), std::numeric_limits<float>::max());
            for(IndexT ii = 0; ii < indexSort.size(); ++ii)
            {
                const IndexT i = indexSort[ii];
                const auto& keypointI = features[i];
                for(IndexT jj = 0; jj < indexSort.size(); ++jj)
                {
                    const IndexT j = indexSort[jj];
                    const auto& keypointJ = features[j];
                    if(featuresPeakValue[j] > featuresPeakValue[i])
                    {
                        const float dx = (keypointJ.frame.x - keypointI.frame.x);
                        const float dy = (keypointJ.frame.y - keypointI.frame.y);
                        const float radius = dx * dx + dy * dy;
                        if(radius < radiusMaxima[i])
                            radiusMaxima[i] = radius;
                    }
                }
            }

            std::size_t maxNbKeypoints = std::min(params._maxTotalKeypoints, indexSort.size());
            std::partial_sort(indexSort.begin(), indexSort.begin() + maxNbKeypoints,
                              indexSort.end(),
                              [&](int a, int b) { return radiusMaxima[a] > radiusMaxima[b]; });
            indexSort.resize(maxNbKeypoints);

            ALICEVISION_LOG_TRACE(
                "DSPSIFT Features: before: " << nbFeatures
                << ", after grid filtering: " << indexSort.size());
        }
    }
    // Grid filtering of the keypoints to ensure a global repartition
    else if(params._gridSize && params._maxTotalKeypoints && (params._contrastFiltering != EFeatureConstrastFiltering::Static))
    {
        // Only filter features if we have more features than the maxTotalKeypoints
        if(nbFeatures > params._maxTotalKeypoints)
        {
            std::vector<IndexT> filteredIndexes;
            std::vector<IndexT> rejectedIndexes;
            filteredIndexes.reserve(std::min(std::size_t(nbFeatures), params._maxTotalKeypoints));
            rejectedIndexes.reserve(nbFeatures);

            const std::size_t sizeMat = params._gridSize * params._gridSize;
            std::vector<std::size_t> countFeatPerCell(sizeMat, 0);
            const std::size_t keypointsPerCell = params._maxTotalKeypoints / sizeMat;
            const double regionWidth = image.Width() / double(params._gridSize);
            const double regionHeight = image.Height() / double(params._gridSize);

            for(IndexT ii = 0; ii < indexSort.size(); ++ii)
            {
                const IndexT i = indexSort[ii];
                const auto& keypoint = features[i];

                const std::size_t cellX = std::min(std::size_t(keypoint.frame.x / regionWidth), params._gridSize);
                const std::size_t cellY = std::min(std::size_t(keypoint.frame.y / regionHeight), params._gridSize);

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
            indexSort.swap(filteredIndexes);
            ALICEVISION_LOG_TRACE("SIFT Features: before: " << nbFeatures
                                                            << ", after grid filtering: " << filteredIndexes.size());
        }
    }
    else if(params._maxTotalKeypoints)
    {
        // Retrieve extracted features
        if(indexSort.size() > params._maxTotalKeypoints)
            indexSort.resize(params._maxTotalKeypoints);
    }

    // Compute the descriptors for the detected keypoints.
    {
        regionsCasted->Features().resize(indexSort.size());
        regionsCasted->Descriptors().resize(indexSort.size());

        std::unique_ptr<VlSiftFilt, void (*)(VlSiftFilt*)> sift(vl_sift_new(16, 16, 1, params._numScales, 0),
                                                                &vl_sift_delete);

        // All constant parameters
        const size_t kPatchResolution = 15;
        const size_t kPatchSide = 2 * kPatchResolution + 1;
        const double kPatchRelativeExtent = 7.5;
        const double kPatchRelativeSmoothing = 1;
        const double kPatchStep = kPatchRelativeExtent / kPatchResolution;
        const double kSigma = kPatchRelativeExtent / (3.0 * (4 + 1) / 2) / kPatchStep;
        // DSP parameters
        float dspMinScale = 1;
        float dspScaleStep = 0;
        int dspNumScales = 1;
        if(params.domainSizePooling)
        {
            dspMinScale = params.dspMinScale;
            dspScaleStep = (params.dspMaxScale - params.dspMinScale) / params.dspNumScales;
            dspNumScales = params.dspNumScales;
        }

#pragma omp parallel
        {
            VlCovDetBuffer internalBuffer;
            vl_covdetbuffer_init(&internalBuffer);
            Eigen::Matrix<float, Eigen::Dynamic, 128, Eigen::RowMajor> descriptorsOverDspScales(dspNumScales, 128);
            std::vector<float> patch(kPatchSide * kPatchSide);
            std::vector<float> patchXY(2 * kPatchSide * kPatchSide);

#pragma omp for
            for(int oIndex = 0; oIndex < indexSort.size(); ++oIndex)
            {
                const int iIndex = indexSort[oIndex];
                const auto& inFeat = features[iIndex];

                for(int s = 0; s < dspNumScales; ++s)
                {
                    const double dspScale = dspMinScale + s * dspScaleStep;

                    VlFrameOrientedEllipse frameAtScale = inFeat.frame;
                    frameAtScale.a11 *= dspScale;
                    frameAtScale.a12 *= dspScale;
                    frameAtScale.a21 *= dspScale;
                    frameAtScale.a22 *= dspScale;

                    vl_covdet_extract_patch_for_frame(covdet.get(), patch.data(), kPatchResolution, &internalBuffer,
                                                      kPatchRelativeExtent, kPatchRelativeSmoothing, frameAtScale);

                    vl_imgradient_polar_f(patchXY.data(), patchXY.data() + 1, 2, 2 * kPatchSide, patch.data(),
                                          kPatchSide, kPatchSide, kPatchSide);

                    vl_sift_calc_raw_descriptor(sift.get(), patchXY.data(), descriptorsOverDspScales.row(s).data(),
                                                kPatchSide, kPatchSide, kPatchResolution, kPatchResolution, kSigma, /*angle0=*/0);
                }

                Eigen::Matrix<float, 1, 128> descriptor;
                if(dspNumScales > 1)
                {
                    descriptor = descriptorsOverDspScales.colwise().mean();
                }
                else
                {
                    descriptor = descriptorsOverDspScales;
                }

                regionsCasted->Features()[oIndex] = PointFeature(inFeat.frame.x, inFeat.frame.y, inFeat.sigma, inFeat.orientationScore);
                Descriptor<T, 128>& outDescriptor = regionsCasted->Descriptors()[oIndex];
                convertSIFT<T>(descriptor.data(), outDescriptor, params._rootSift);
            }

            vl_covdetbuffer_clear(&internalBuffer);
        }
    }

    return true;
}

template bool extractDSPSIFT<float>(const image::Image<float>& image, std::unique_ptr<Regions>& regions,
                                    const DspSiftParams& params, bool orientation, const image::Image<unsigned char>* mask);

template bool extractDSPSIFT<unsigned char>(const image::Image<float>& image, std::unique_ptr<Regions>& regions,
                                            const DspSiftParams& params, bool orientation,
                                            const image::Image<unsigned char>* mask);


} // namespace feature
} // namespace aliceVision
