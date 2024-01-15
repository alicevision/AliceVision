// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "FeatureExtractor.hpp"
#include <aliceVision/image/io.hpp>
#include <aliceVision/system/MemoryInfo.hpp>
#include <aliceVision/alicevision_omp.hpp>

#include <filesystem>
#include <iomanip>

namespace fs = std::filesystem;

namespace aliceVision {
namespace featureEngine {

FeatureExtractorViewJob::FeatureExtractorViewJob(const sfmData::View& view, const std::string& outputFolder)
  : _view(view),
    _outputBasename(fs::path(fs::path(outputFolder) / fs::path(std::to_string(view.getViewId()))).string())
{}

FeatureExtractorViewJob::~FeatureExtractorViewJob() = default;

void FeatureExtractorViewJob::setImageDescribers(const std::vector<std::shared_ptr<feature::ImageDescriber>>& imageDescribers)
{
    for (std::size_t i = 0; i < imageDescribers.size(); ++i)
    {
        const std::shared_ptr<feature::ImageDescriber>& imageDescriber = imageDescribers.at(i);
        feature::EImageDescriberType imageDescriberType = imageDescriber->getDescriberType();

        if (fs::exists(getFeaturesPath(imageDescriberType)) && fs::exists(getDescriptorPath(imageDescriberType)))
        {
            continue;
        }

        _memoryConsuption += imageDescriber->getMemoryConsumption(_view.getImage().getWidth(), _view.getImage().getHeight());

        if (imageDescriber->useCuda())
            _gpuImageDescriberIndexes.push_back(i);
        else
            _cpuImageDescriberIndexes.push_back(i);
    }
}

FeatureExtractor::FeatureExtractor(const sfmData::SfMData& sfmData)
  : _sfmData(sfmData)
{}

FeatureExtractor::~FeatureExtractor() = default;

void FeatureExtractor::process(const HardwareContext& hContext, const image::EImageColorSpace workingColorSpace)
{
    size_t maxAvailableMemory = hContext.getUserMaxMemoryAvailable();
    unsigned int maxAvailableCores = hContext.getMaxThreads();

    // iteration on each view in the range in order
    // to prepare viewJob stack
    sfmData::Views::const_iterator itViewBegin = _sfmData.getViews().begin();
    sfmData::Views::const_iterator itViewEnd = _sfmData.getViews().end();

    if (_rangeStart != -1)
    {
        std::advance(itViewBegin, _rangeStart);
        itViewEnd = itViewBegin;
        std::advance(itViewEnd, _rangeSize);
    }

    std::size_t jobMaxMemoryConsuption = 0;

    std::vector<FeatureExtractorViewJob> cpuJobs;
    std::vector<FeatureExtractorViewJob> gpuJobs;

    for (auto it = itViewBegin; it != itViewEnd; ++it)
    {
        const sfmData::View& view = *(it->second.get());
        FeatureExtractorViewJob viewJob(view, _outputFolder);

        viewJob.setImageDescribers(_imageDescribers);
        jobMaxMemoryConsuption = std::max(jobMaxMemoryConsuption, viewJob.memoryConsuption());

        if (viewJob.useCPU())
            cpuJobs.push_back(viewJob);

        if (viewJob.useGPU())
            gpuJobs.push_back(viewJob);
    }

    if (!cpuJobs.empty())
    {
        system::MemoryInfo memoryInformation = system::getMemoryInfo();

        // Put an upper bound with user specified memory
        size_t maxMemory = std::min(memoryInformation.availableRam, maxAvailableMemory);
        size_t maxTotalMemory = std::min(memoryInformation.totalRam, maxAvailableMemory);

        ALICEVISION_LOG_INFO("Job max memory consumption for one image: " << jobMaxMemoryConsuption / (1024 * 1024) << " MB");
        ALICEVISION_LOG_INFO("Memory information: " << std::endl << memoryInformation);

        if (jobMaxMemoryConsuption == 0)
            throw std::runtime_error("Cannot compute feature extraction job max memory consumption.");

        // How many buffers can fit in 90% of the available RAM?
        // This is used to estimate how many jobs can be computed in parallel without SWAP.
        const std::size_t memoryImageCapacity = std::size_t((0.9 * maxMemory) / jobMaxMemoryConsuption);

        std::size_t nbThreads = std::max(std::size_t(1), memoryImageCapacity);
        ALICEVISION_LOG_INFO("Max number of threads regarding memory usage: " << nbThreads);
        const double oneGB = 1024.0 * 1024.0 * 1024.0;
        if (jobMaxMemoryConsuption > maxMemory)
        {
            ALICEVISION_LOG_WARNING("The amount of RAM available is critical to extract features.");
            if (jobMaxMemoryConsuption <= maxTotalMemory)
            {
                ALICEVISION_LOG_WARNING("But the total amount of RAM is enough to extract features, "
                                        << "so you should close other running applications.");
                ALICEVISION_LOG_WARNING(" => " << std::size_t(std::round((double(maxTotalMemory - maxMemory) / oneGB)))
                                               << " GB are used by other applications for a total RAM capacity of "
                                               << std::size_t(std::round(double(maxTotalMemory) / oneGB)) << " GB.");
            }
        }
        else
        {
            if (maxMemory < 0.5 * maxTotalMemory)
            {
                ALICEVISION_LOG_WARNING("More than half of the RAM is used by other applications. It would be more efficient to close them.");
                ALICEVISION_LOG_WARNING(" => " << std::size_t(std::round(double(maxTotalMemory - maxMemory) / oneGB))
                                               << " GB are used by other applications for a total RAM capacity of "
                                               << std::size_t(std::round(double(maxTotalMemory) / oneGB)) << " GB.");
            }
        }

        if (maxMemory == 0)
        {
            ALICEVISION_LOG_WARNING("Cannot find available system memory, this can be due to OS limitation.\n"
                                    "Use only one thread for CPU feature extraction.");
            nbThreads = 1;
        }

        // nbThreads should not be higher than the available cores
        nbThreads = std::min(static_cast<std::size_t>(maxAvailableCores), nbThreads);

        // nbThreads should not be higher than the number of jobs
        nbThreads = std::min(cpuJobs.size(), nbThreads);

        ALICEVISION_LOG_INFO("# threads for extraction: " << nbThreads);
        omp_set_nested(1);

#pragma omp parallel for num_threads(nbThreads)
        for (int i = 0; i < cpuJobs.size(); ++i)
            computeViewJob(cpuJobs.at(i), false, workingColorSpace);
    }

    if (!gpuJobs.empty())
    {
        for (const auto& job : gpuJobs)
            computeViewJob(job, true, workingColorSpace);
    }
}

void FeatureExtractor::computeViewJob(const FeatureExtractorViewJob& job, bool useGPU, const image::EImageColorSpace workingColorSpace)
{
    image::Image<float> imageGrayFloat;
    image::Image<unsigned char> imageGrayUChar;
    image::Image<unsigned char> mask;

    

    image::readImage(job.view().getImage().getImagePath(), imageGrayFloat, workingColorSpace);

    double pixelRatio = 1.0;
    job.view().getImage().getDoubleMetadata({"PixelAspectRatio"}, pixelRatio);

    if (pixelRatio != 1.0)
    {
        // Resample input image in order to work with square pixels
        const int w = imageGrayFloat.width();
        const int h = imageGrayFloat.height();

        const int nw = static_cast<int>(static_cast<double>(w) * pixelRatio);
        const int nh = h;

        image::Image<float> resizedInput;
        imageAlgo::resizeImage(nw, nh, imageGrayFloat, resizedInput);
        imageGrayFloat.swap(resizedInput);
    }

    if (!_masksFolder.empty() && fs::exists(_masksFolder))
    {
        const auto masksFolder = fs::path(_masksFolder);
        const auto idMaskPath = masksFolder / fs::path(std::to_string(job.view().getViewId())).replace_extension(_maskExtension);
        const auto nameMaskPath = masksFolder / fs::path(job.view().getImage().getImagePath()).filename().replace_extension(_maskExtension);

        if (fs::exists(idMaskPath))
        {
            image::readImage(idMaskPath.string(), mask, image::EImageColorSpace::LINEAR);
        }
        else if (fs::exists(nameMaskPath))
        {
            image::readImage(nameMaskPath.string(), mask, image::EImageColorSpace::LINEAR);
        }
    }

    for (const auto& imageDescriberIndex : job.imageDescriberIndexes(useGPU))
    {
        const auto& imageDescriber = _imageDescribers.at(imageDescriberIndex);
        const feature::EImageDescriberType imageDescriberType = imageDescriber->getDescriberType();
        const std::string imageDescriberTypeName = feature::EImageDescriberType_enumToString(imageDescriberType);

        // Compute features and descriptors and export them to files
        ALICEVISION_LOG_INFO("Extracting " << imageDescriberTypeName << " features from view '" << job.view().getImage().getImagePath() << "' "
                                           << (useGPU ? "[gpu]" : "[cpu]"));

        std::unique_ptr<feature::Regions> regions;
        if (imageDescriber->useFloatImage())
        {
            // image buffer use float image, use the read buffer
            imageDescriber->describe(imageGrayFloat, regions);
        }
        else
        {
            // image buffer can't use float image
            if (imageGrayUChar.width() == 0)  // the first time, convert the float buffer to uchar
                imageGrayUChar = (imageGrayFloat.getMat() * 255.f).cast<unsigned char>();
            imageDescriber->describe(imageGrayUChar, regions);
        }

        if (pixelRatio != 1.0)
        {
            // Re-position point features on input image
            for (auto& feat : regions->Features())
            {
                feat.x() /= pixelRatio;
            }
        }

        //Get Fisheye mask
        bool isFisheye = false;
        Vec2 circleCenter = Vec2::Zero();
        double circleRadius = std::numeric_limits<double>::max();
        auto & intrinsics = _sfmData.getIntrinsics();
        if (job.view().getIntrinsicId() != UndefinedIndexT)
        {
            const auto & intrinsic = intrinsics.at(job.view().getIntrinsicId());
            const auto & equidistant = std::dynamic_pointer_cast<const camera::Equidistant>(intrinsic);

            //If the current view comes from a fisheye optic
            if (equidistant)
            {
                //Make sure the circle radius has been initialized before
                if (equidistant->getCircleRadius() > 1.0)
                {
                    circleCenter = equidistant->getCircleCenter();
                    circleRadius = equidistant->getCircleRadius();
                    isFisheye = true;
                }
            }
        }

        //On mask or fisheye circle availability
        if (mask.height() > 0 || isFisheye); 
        {
            std::vector<feature::FeatureInImage> selectedIndices;
            for (size_t i = 0, n = regions->RegionCount(); i != n; ++i)
            {
                const Vec2 position = regions->GetRegionPosition(i);
                const int x = int(position.x());
                const int y = int(position.y());


                bool masked = false;

                //Check if point lies inside potential fisheye circle
                if ((position - circleCenter).norm() > circleRadius)
                {
                    masked = true;
                }
                //Check if the optional image mask tells us to ignore this coordinate
                else if (x < mask.width() && y < mask.height())
                {
                    if ((mask(y, x) == 0 && !_maskInvert) || (mask(y, x) != 0 && _maskInvert))
                    {
                        masked = true;
                    }
                }

                if (!masked)
                {
                    selectedIndices.push_back({IndexT(i), 0});
                }
            }

            std::vector<IndexT> out_associated3dPoint;
            std::map<IndexT, IndexT> out_mapFullToLocal;
            regions = regions->createFilteredRegions(selectedIndices, out_associated3dPoint, out_mapFullToLocal);
        }

        imageDescriber->Save(regions.get(), job.getFeaturesPath(imageDescriberType), job.getDescriptorPath(imageDescriberType));
        ALICEVISION_LOG_INFO(std::left << std::setw(6) << " " << regions->RegionCount() << " " << imageDescriberTypeName
                                       << " features extracted from view '" << job.view().getImage().getImagePath() << "'");
    }
}

}  // namespace featureEngine
}  // namespace aliceVision
