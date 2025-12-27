// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Refine.hpp"

#include <aliceVision/alicevision_omp.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/mvsData/Point2d.hpp>
#include <aliceVision/mvsData/Point3d.hpp>
#include <aliceVision/mvsUtils/fileIO.hpp>
#include <aliceVision/depthMap_sycl/depthMapUtils.hpp>
#include <aliceVision/depthMap_sycl/volumeIO.hpp>
#include <aliceVision/depthMap_sycl/sycl/DeviceCache.hpp>
#include <aliceVision/depthMap_sycl/sycl/planeSweeping/deviceDepthSimilarityMap.hpp>
#include <aliceVision/depthMap_sycl/sycl/planeSweeping/deviceSimilarityVolume.hpp>

namespace aliceVision {
namespace depthMap {

    Refine::Refine(const mvsUtils::MultiViewParams& mp, const mvsUtils::TileParams& tileParams, const RefineParams& refineParams, bool& allocationSuccess, sycl::queue queue)
  : _mp(mp),
    _tileParams(tileParams),
    _refineParams(refineParams),
    _sgmDepthPixSizeMap_dmp(queue),
    _refinedDepthSimMap_dmp(queue),
    _optimizedDepthSimMap_dmp(queue),
    _sgmNormalMap_dmp(queue),
    _normalMap_dmp(queue),
    _volumeRefineSim_dmp(queue),
    _optTmpDepthMap_dmp(queue),
    _optImgVariance_dmp(queue),
    _queue(queue)
{
    // get tile maximum dimensions
    const int downscale = _refineParams.scale * _refineParams.stepXY;
    const int maxTileWidth = divideRoundUp(tileParams.bufferWidth, downscale);
    const int maxTileHeight = divideRoundUp(tileParams.bufferHeight, downscale);

    // compute depth/sim map maximum dimensions
    const SyclSize<2> depthSimMapDim(maxTileWidth, maxTileHeight);

    // allocate depth/sim maps in device memory
    allocationSuccess &= _sgmDepthPixSizeMap_dmp.allocate(depthSimMapDim);
    allocationSuccess &= _refinedDepthSimMap_dmp.allocate(depthSimMapDim);
    allocationSuccess &= _optimizedDepthSimMap_dmp.allocate(depthSimMapDim);

    // allocate SGM upscaled normal map in device memory
    if (_refineParams.useSgmNormalMap)
        allocationSuccess &= _sgmNormalMap_dmp.allocate(depthSimMapDim);

    // allocate normal map in device memory
    if (_refineParams.exportIntermediateNormalMaps)
        allocationSuccess &= _normalMap_dmp.allocate(depthSimMapDim);

    // compute volume maximum dimensions
    const int nbDepthsToRefine = _refineParams.halfNbDepths * 2 + 1;
    const SyclSize<3> volDim(maxTileWidth, maxTileHeight, nbDepthsToRefine);

    // allocate refine volume in device memory
    allocationSuccess &= _volumeRefineSim_dmp.allocate(volDim);

    // allocate depth/sim map optimization buffers
    if (_refineParams.useColorOptimization)
    {
        allocationSuccess &= _optTmpDepthMap_dmp.allocate(depthSimMapDim);
        allocationSuccess &= _optImgVariance_dmp.allocate(depthSimMapDim);
    }
}

double Refine::getDeviceMemoryConsumption() const
{
    size_t bytes = 0;

    bytes += _sgmDepthPixSizeMap_dmp.getBytesPadded();
    bytes += _refinedDepthSimMap_dmp.getBytesPadded();
    bytes += _optimizedDepthSimMap_dmp.getBytesPadded();
    bytes += _sgmNormalMap_dmp.getBytesPadded();
    bytes += _normalMap_dmp.getBytesPadded();
    bytes += _volumeRefineSim_dmp.getBytesPadded();
    bytes += _optTmpDepthMap_dmp.getBytesPadded();
    bytes += _optImgVariance_dmp.getBytesPadded();

    return (double(bytes) / (1024.0 * 1024.0));
}

double Refine::getDeviceMemoryConsumptionUnpadded() const
{
    size_t bytes = 0;

    bytes += _sgmDepthPixSizeMap_dmp.getBytesUnpadded();
    bytes += _refinedDepthSimMap_dmp.getBytesUnpadded();
    bytes += _optimizedDepthSimMap_dmp.getBytesUnpadded();
    bytes += _sgmNormalMap_dmp.getBytesUnpadded();
    bytes += _normalMap_dmp.getBytesUnpadded();
    bytes += _volumeRefineSim_dmp.getBytesUnpadded();
    bytes += _optTmpDepthMap_dmp.getBytesUnpadded();
    bytes += _optImgVariance_dmp.getBytesUnpadded();

    return (double(bytes) / (1024.0 * 1024.0));
}

sycl::event Refine::refineRc(const Tile& tile,
                             const SyclDeviceMemoryPitched<sycl::float2, 2>& in_sgmDepthThicknessMap_dmp,
                             const SyclDeviceMemoryPitched<sycl::float3, 2>& in_sgmNormalMap_dmp,
                             sycl::event prerequisite)
{
    const IndexT viewId = _mp.getViewId(tile.rc);

    ALICEVISION_LOG_INFO(tile << "Refine depth/sim map of view id: " << viewId << ", rc: " << tile.rc << " (" << (tile.rc + 1) << " / " << _mp.ncams
                              << ").");

    // compute upscaled SGM depth/pixSize map
    // compute upscaled SGM normal map
    sycl::event upscaledDepthPixSizeMap;
    {
        // downscale the region of interest
        const ROI downscaledRoi = downscaleROI(tile.roi, _refineParams.scale * _refineParams.stepXY);

        // get device cache instance
        DeviceCache& deviceCache = DeviceCache::getInstance();

        // get R device mipmap image from cache
        const DeviceMipmapImage& rcDeviceMipmapImage = deviceCache.requestMipmapImage(tile.rc, _mp, _queue);

        // compute upscaled SGM depth/pixSize map
        // - upscale SGM depth/thickness map
        // - filter masked pixels (alpha)
        // - compute pixSize from SGM thickness
        upscaledDepthPixSizeMap = sycl_computeSgmUpscaledDepthPixSizeMap(_sgmDepthPixSizeMap_dmp, in_sgmDepthThicknessMap_dmp, tile.rc, _mp, rcDeviceMipmapImage, _refineParams, downscaledRoi, _queue, prerequisite);

        // export intermediate depth/pixSize map (if requested by user)
        if (_refineParams.exportIntermediateDepthSimMaps)
            upscaledDepthPixSizeMap.wait();
            writeDepthPixSizeMap(tile.rc, _mp, _tileParams, tile.roi, _sgmDepthPixSizeMap_dmp, _queue, _refineParams.scale, _refineParams.stepXY, "sgmUpscaled");

        // upscale SGM normal map (if needed)
        if (_refineParams.useSgmNormalMap && in_sgmNormalMap_dmp.getBuffer() != nullptr)
        {
             upscaledDepthPixSizeMap = sycl_normalMapUpscale(_sgmNormalMap_dmp, in_sgmNormalMap_dmp, downscaledRoi, _queue, upscaledDepthPixSizeMap);
        }
    }

    // refine and fuse depth/sim map
    sycl::event depthMap;
    if (_refineParams.useRefineFuse)
    {
        // refine and fuse with volume strategy
        depthMap = refineAndFuseDepthSimMap(tile, upscaledDepthPixSizeMap);
    }
    else
    {
        ALICEVISION_LOG_INFO(tile << "Refine and fuse depth/sim map volume disabled.");
        depthMap = sycl_depthSimMapCopyDepthOnly(_refinedDepthSimMap_dmp, _sgmDepthPixSizeMap_dmp, 1.0f, _queue, upscaledDepthPixSizeMap);
    }

    // export intermediate depth/sim map (if requested by user)
    if (_refineParams.exportIntermediateDepthSimMaps)
    {
        depthMap.wait();
        writeDepthSimMap(tile.rc, _mp, _tileParams, tile.roi, _refinedDepthSimMap_dmp, _queue, _refineParams.scale, _refineParams.stepXY, "refinedFused");
    }

    // export intermediate normal map (if requested by user)
    if (_refineParams.exportIntermediateNormalMaps)
    {
        computeAndWriteNormalMap(tile, _refinedDepthSimMap_dmp, depthMap, "refinedFused");
    }

    // optimize depth/sim map
    if (_refineParams.useColorOptimization && _refineParams.optimizationNbIterations > 0)
    {
        depthMap = optimizeDepthSimMap(tile, depthMap);
    }
    else
    {
        ALICEVISION_LOG_INFO(tile << "Color optimize depth/sim map disabled.");
        depthMap = _optimizedDepthSimMap_dmp.copyFrom(_refinedDepthSimMap_dmp, depthMap);
    }

    // export intermediate normal map (if requested by user)
    if (_refineParams.exportIntermediateNormalMaps)
        computeAndWriteNormalMap(tile, _optimizedDepthSimMap_dmp, depthMap);

    ALICEVISION_LOG_INFO(tile << "Refine depth/sim map done.");

    return depthMap;
}

sycl::event Refine::refineAndFuseDepthSimMap(const Tile& tile, sycl::event prerequisite)
{
    ALICEVISION_LOG_INFO(tile << "Refine and fuse depth/sim map volume.");

    // downscale the region of interest
    const ROI downscaledRoi = downscaleROI(tile.roi, _refineParams.scale * _refineParams.stepXY);

    // get the depth range
    const Range depthRange(0, _volumeRefineSim_dmp.getSize().z());

    // initialize the similarity volume at 0
    // each tc filtered and inverted similarity value will be summed in this volume
    sycl_volumeInitialize(_volumeRefineSim_dmp, TSimRefine(0.f), _queue, prerequisite);

    // get device cache instance
    DeviceCache& deviceCache = DeviceCache::getInstance();

    // get R device mipmap image from cache
    const DeviceMipmapImage& rcDeviceMipmapImage = deviceCache.requestMipmapImage(tile.rc, _mp, _queue);

    // compute for each RcTc each similarity value for each depth to refine
    // sum the inverted / filtered similarity value, best value is the HIGHEST
    std::vector<sycl::event> rcTcComputed(tile.refineTCams.size());
    for (std::size_t tci = 0; tci < tile.refineTCams.size(); ++tci)
    {
        const int tc = tile.refineTCams.at(tci);

        // get T device mipmap image from cache
        const DeviceMipmapImage& tcDeviceMipmapImage = deviceCache.requestMipmapImage(tc, _mp, _queue);

        ALICEVISION_LOG_DEBUG(tile << "Refine similarity volume:" << std::endl
                                   << "\t- rc: " << tile.rc << std::endl
                                   << "\t- tc: " << tc << " (" << (tci + 1) << "/" << tile.refineTCams.size() << ")" << std::endl
                                   << "\t- tile range x: [" << downscaledRoi.x.begin << " - " << downscaledRoi.x.end << "]" << std::endl
                                   << "\t- tile range y: [" << downscaledRoi.y.begin << " - " << downscaledRoi.y.end << "]" << std::endl);

        rcTcComputed.at(tci) = sycl_volumeRefineSimilarity(
                                    _volumeRefineSim_dmp,
                                    _sgmDepthPixSizeMap_dmp,
                                    (_refineParams.useSgmNormalMap) ? &_sgmNormalMap_dmp : nullptr,
                                    tile.rc,
                                    tc,
                                    _mp,
                                    rcDeviceMipmapImage,
                                    tcDeviceMipmapImage,
                                    _refineParams,
                                    depthRange,
                                    downscaledRoi,
                                    _queue,
                                    prerequisite);
    }

    // export intermediate volume information (if requested by user)
    if (!_refineParams.exportIntermediateCrossVolumes && !_refineParams.exportIntermediateVolume9pCsv)
    {
        for (sycl::event& event : rcTcComputed)
            event.wait();
        exportVolumeInformation(tile, "afterRefine");
    }

    // retrieve the best depth/sim in the volume
    // compute sub-pixel sample using a sliding gaussian
    sycl::event bestDepth = sycl_volumeRefineBestDepth(_refinedDepthSimMap_dmp, _sgmDepthPixSizeMap_dmp, _volumeRefineSim_dmp, _refineParams, downscaledRoi, _queue, rcTcComputed);

    ALICEVISION_LOG_INFO(tile << "Refine and fuse depth/sim map volume dispatched.");

    return bestDepth;
}

sycl::event Refine::optimizeDepthSimMap(const Tile& tile, sycl::event prerequisite)
{
    ALICEVISION_LOG_INFO(tile << "Color optimize depth/sim map.");

    // downscale the region of interest
    const ROI downscaledRoi = downscaleROI(tile.roi, _refineParams.scale * _refineParams.stepXY);

    // get R device camera from cache
    DeviceCache& deviceCache = DeviceCache::getInstance();

    // get R device mipmap image from cache
    const DeviceMipmapImage& rcDeviceMipmapImage = deviceCache.requestMipmapImage(tile.rc, _mp, _queue);

    sycl::event gradientDescent = sycl_depthSimMapOptimizeGradientDescent(
                                            _optimizedDepthSimMap_dmp,  // output depth/sim map optimized
                                            _optImgVariance_dmp,        // image variance buffer pre-allocate
                                            _optTmpDepthMap_dmp,        // temporary depth map buffer pre-allocate
                                            _sgmDepthPixSizeMap_dmp,    // input SGM upscaled depth/pixSize map
                                            _refinedDepthSimMap_dmp,    // input refined and fused depth/sim map
                                            tile.rc,
                                            _mp,
                                            rcDeviceMipmapImage,
                                            _refineParams,
                                            downscaledRoi,
                                            _queue,
                                            prerequisite);

    ALICEVISION_LOG_INFO(tile << "Color optimize depth/sim map done.");

    return gradientDescent;
}

void Refine::computeAndWriteNormalMap(const Tile& tile, const SyclDeviceMemoryPitched<sycl::float2, 2>& in_depthSimMap_dmp, sycl::event prerequisite, const std::string& name)
{
    // downscale the region of interest
    const ROI downscaledRoi = downscaleROI(tile.roi, _refineParams.scale * _refineParams.stepXY);

    ALICEVISION_LOG_INFO(tile << "Refine compute normal map of view id: " << _mp.getViewId(tile.rc) << ", rc: " << tile.rc << " (" << (tile.rc + 1)
                              << " / " << _mp.ncams << ").");

    sycl_depthSimMapComputeNormal(_normalMap_dmp, in_depthSimMap_dmp, tile.rc, _mp, _refineParams.stepXY, downscaledRoi, _queue, prerequisite).wait();

    writeNormalMap(tile.rc, _mp, _tileParams, tile.roi, _normalMap_dmp, _queue, _refineParams.scale, _refineParams.stepXY, name);
}

void Refine::exportVolumeInformation(const Tile& tile, const std::string& name) const
{
    if (!_refineParams.exportIntermediateCrossVolumes && !_refineParams.exportIntermediateVolume9pCsv)
    {
        // nothing to do
        return;
    }

    // get tile begin indexes (default no tile)
    int tileBeginX = -1;
    int tileBeginY = -1;

    if (tile.nbTiles > 1)
    {
        tileBeginX = tile.roi.x.begin;
        tileBeginY = tile.roi.y.begin;
    }

    // copy device similarity volume to host memory
    SyclHostMemoryHeap<TSimRefine, 3> volumeSim_hmh(_volumeRefineSim_dmp.getSize(), _queue);
    volumeSim_hmh.copyFrom(_volumeRefineSim_dmp, _queue, sycl::event()).wait();

    // copy device SGM upscale depth/sim map to host memory
    SyclHostMemoryHeap<sycl::float2, 2> depthPixSizeMapSgmUpscale_hmh(_sgmDepthPixSizeMap_dmp.getSize(), _queue);
    depthPixSizeMapSgmUpscale_hmh.copyFrom(_sgmDepthPixSizeMap_dmp, _queue, sycl::event()).wait();

    if (_refineParams.exportIntermediateCrossVolumes)
    {
        ALICEVISION_LOG_INFO(tile << "Export similarity volume cross (" << name << ").");

        const std::string volumeCrossPath = getFileNameFromIndex(_mp, tile.rc, mvsUtils::EFileType::volumeCross, "_" + name, tileBeginX, tileBeginY);

        exportSimilarityVolumeCross(volumeSim_hmh, depthPixSizeMapSgmUpscale_hmh, _mp, tile.rc, _refineParams, volumeCrossPath, tile.roi);

        ALICEVISION_LOG_INFO(tile << "Export similarity volume cross (" << name << ") done.");
    }

    if (_refineParams.exportIntermediateTopographicCutVolumes)
    {
        ALICEVISION_LOG_INFO(tile << "Export similarity volume topographic cut (" << name << ").");

        const std::string volumeCutPath =
          getFileNameFromIndex(_mp, tile.rc, mvsUtils::EFileType::volumeTopographicCut, "_" + name, tileBeginX, tileBeginY);

        exportSimilarityVolumeTopographicCut(volumeSim_hmh, depthPixSizeMapSgmUpscale_hmh, _mp, tile.rc, _refineParams, volumeCutPath, tile.roi);

        ALICEVISION_LOG_INFO(tile << "Export similarity volume topographic cut (" << name << ") done.");
    }

    if (_refineParams.exportIntermediateVolume9pCsv)
    {
        ALICEVISION_LOG_INFO(tile << "Export similarity volume 9 points CSV (" << name << ").");

        const std::string stats9Path = getFileNameFromIndex(_mp, tile.rc, mvsUtils::EFileType::stats9p, "_refine", tileBeginX, tileBeginY);

        exportSimilaritySamplesCSV(volumeSim_hmh, name, _refineParams, stats9Path, tile.roi);

        ALICEVISION_LOG_INFO(tile << "Export similarity volume 9 points CSV (" << name << ") done.");
    }
}

}  // namespace depthMap
}  // namespace aliceVision
