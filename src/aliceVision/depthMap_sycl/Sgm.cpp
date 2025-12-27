// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Sgm.hpp"

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/mvsUtils/fileIO.hpp>
#include <aliceVision/depthMap_sycl/depthMapUtils.hpp>
#include <aliceVision/depthMap_sycl/volumeIO.hpp>
#include <aliceVision/depthMap_sycl/sycl/DeviceCache.hpp>
#include <aliceVision/depthMap_sycl/sycl/planeSweeping/deviceDepthSimilarityMap.hpp>
#include <aliceVision/depthMap_sycl/sycl/planeSweeping/deviceSimilarityVolume.hpp>

#include <iostream>
#include <sstream>

namespace aliceVision {
namespace depthMap {

Sgm::Sgm(const mvsUtils::MultiViewParams& mp,
         const mvsUtils::TileParams& tileParams,
         const SgmParams& sgmParams,
         bool computeDepthSimMap,
         bool computeNormalMap,
         bool& allocationSuccess,
         sycl::queue queue)
  : _mp(mp),
    _tileParams(tileParams),
    _sgmParams(sgmParams),
    _computeDepthSimMap(computeDepthSimMap || sgmParams.exportIntermediateDepthSimMaps),
    _computeNormalMap(computeNormalMap || sgmParams.exportIntermediateNormalMaps),
    _depths_dmp(queue),
    _depthThicknessMap_dmp(queue),
    _depthSimMap_dmp(queue),
    _normalMap_dmp(queue),
    _volumeBestSim_dmp(queue),
    _volumeSecBestSim_dmp(queue),
    _volumeSliceAccA_dmp(queue),
    _volumeSliceAccB_dmp(queue),
    _volumeAxisAcc_dmp(queue),
    _queue(queue)
{
    // get tile maximum dimensions
    const int downscale = _sgmParams.scale * _sgmParams.stepXY;
    const int maxTileWidth = divideRoundUp(tileParams.bufferWidth, downscale);
    const int maxTileHeight = divideRoundUp(tileParams.bufferHeight, downscale);

    // compute map maximum dimensions
    const SyclSize<2> mapDim(maxTileWidth, maxTileHeight);

    // allocate depth list in device memory
    {
        const SyclSize<2> depthsDim(_sgmParams.maxDepths, 1);

        allocationSuccess &= _depths_hmh.allocate(depthsDim);
        allocationSuccess &= _depths_dmp.allocate(depthsDim);
    }

    // allocate depth thickness map in device memory
    allocationSuccess &= _depthThicknessMap_dmp.allocate(mapDim);

    // allocate depth/sim map in device memory
    if (_computeDepthSimMap)
        allocationSuccess &= _depthSimMap_dmp.allocate(mapDim);

    // allocate normal map in device memory
    if (_computeNormalMap)
        allocationSuccess &= _normalMap_dmp.allocate(mapDim);

    // allocate similarity volumes in device memory
    {
        const SyclSize<3> volDim(maxTileWidth, maxTileHeight, _sgmParams.maxDepths);

        allocationSuccess &= _volumeBestSim_dmp.allocate(volDim);
        allocationSuccess &= _volumeSecBestSim_dmp.allocate(volDim);
    }

    // allocate similarity volume optimization buffers
    if (sgmParams.doSgmOptimizeVolume)
    {
        const size_t maxTileSide = std::max(maxTileWidth, maxTileHeight);

        allocationSuccess &= _volumeSliceAccA_dmp.allocate(SyclSize<2>(maxTileSide, _sgmParams.maxDepths));
        allocationSuccess &= _volumeSliceAccB_dmp.allocate(SyclSize<2>(maxTileSide, _sgmParams.maxDepths));
        allocationSuccess &= _volumeAxisAcc_dmp.allocate(SyclSize<2>(maxTileSide, 1));
    }
}

double Sgm::getDeviceMemoryConsumption() const
{
    size_t bytes = 0;

    bytes += _depths_dmp.getBytesPadded();
    bytes += _depthThicknessMap_dmp.getBytesPadded();
    bytes += _depthSimMap_dmp.getBytesPadded();
    bytes += _normalMap_dmp.getBytesPadded();
    bytes += _volumeBestSim_dmp.getBytesPadded();
    bytes += _volumeSecBestSim_dmp.getBytesPadded();
    bytes += _volumeSliceAccA_dmp.getBytesPadded();
    bytes += _volumeSliceAccB_dmp.getBytesPadded();
    bytes += _volumeAxisAcc_dmp.getBytesPadded();

    return (double(bytes) / (1024.0 * 1024.0));
}

double Sgm::getDeviceMemoryConsumptionUnpadded() const
{
    size_t bytes = 0;

    bytes += _depths_dmp.getBytesUnpadded();
    bytes += _depthThicknessMap_dmp.getBytesUnpadded();
    bytes += _depthSimMap_dmp.getBytesUnpadded();
    bytes += _normalMap_dmp.getBytesUnpadded();
    bytes += _volumeBestSim_dmp.getBytesUnpadded();
    bytes += _volumeSecBestSim_dmp.getBytesUnpadded();
    bytes += _volumeSliceAccA_dmp.getBytesUnpadded();
    bytes += _volumeSliceAccB_dmp.getBytesUnpadded();
    bytes += _volumeAxisAcc_dmp.getBytesUnpadded();

    return (double(bytes) / (1024.0 * 1024.0));
}

sycl::event Sgm::sgmRc(const Tile& tile, const SgmDepthList& tileDepthList, sycl::event prerequisite)
{
    const IndexT viewId = _mp.getViewId(tile.rc);

    ALICEVISION_LOG_INFO(tile << "SGM depth/thickness map of view id: " << viewId << ", rc: " << tile.rc << " (" << (tile.rc + 1) << " / "
                              << _mp.ncams << ").");

    // check SGM depth list and T cameras
    if (tile.sgmTCams.empty() || tileDepthList.getDepths().empty())
        ALICEVISION_THROW_ERROR(tile << "Cannot compute Semi-Global Matching, no depths or no T cameras (viewId: " << viewId << ").");

    // copy rc depth data in page-locked host memory
    for (int i = 0; i < tileDepthList.getDepths().size(); ++i)
        _depths_hmh(i, 0) = tileDepthList.getDepths()[i];

    // copy rc depth data in device memory
    sycl::event copyDepthData = _depths_dmp.copyFrom(_depths_hmh, prerequisite);

    // compute best sim and second best sim volumes
    sycl::event computeVolume = computeSimilarityVolumes(tile, tileDepthList, copyDepthData);

    // export intermediate volume information (if requested by user)
    exportVolumeInformation(tile, tileDepthList, _volumeSecBestSim_dmp, "beforeFiltering", computeVolume);

    // this is here for experimental purposes
    // to show how SGGC work on non optimized depthmaps
    // it must equals to true in normal case
    sycl::event optimizeVolume;
    if (_sgmParams.doSgmOptimizeVolume)
    {
        optimizeVolume = optimizeSimilarityVolume(tile, tileDepthList, computeVolume);
    }
    else
    {
        // best sim volume is normally reuse to put optimized similarity
        optimizeVolume = _volumeBestSim_dmp.copyFrom(_volumeSecBestSim_dmp, computeVolume);
    }

    // export intermediate volume information (if requested by user)
    exportVolumeInformation(tile, tileDepthList, _volumeBestSim_dmp, "afterFiltering", optimizeVolume);

    // retrieve best depth
    sycl::event retrieveDepth = retrieveBestDepth(tile, tileDepthList, optimizeVolume);

    // export intermediate depth/sim map (if requested by user)
    if (_sgmParams.exportIntermediateDepthSimMaps)
    {
        retrieveDepth.wait();
        writeDepthSimMap(tile.rc, _mp, _tileParams, tile.roi, _depthSimMap_dmp, _queue, _sgmParams.scale, _sgmParams.stepXY, "sgm");
    }

    // compute normal map from depth/sim map if needed
    sycl::event finished = retrieveDepth;
    if (_computeNormalMap)
    {
        // downscale the region of interest
        const ROI downscaledRoi = downscaleROI(tile.roi, _sgmParams.scale * _sgmParams.stepXY);

        ALICEVISION_LOG_INFO(tile << "SGM compute normal map of view id: " << viewId << ", rc: " << tile.rc << " (" << (tile.rc + 1) << " / "
                                  << _mp.ncams << ").");
        finished = sycl_depthSimMapComputeNormal(_normalMap_dmp, _depthSimMap_dmp, tile.rc, _mp, _sgmParams.stepXY, downscaledRoi, _queue, retrieveDepth);

        // export intermediate normal map (if requested by user)
        if (_sgmParams.exportIntermediateNormalMaps)
        {
            finished.wait();
            writeNormalMap(tile.rc, _mp, _tileParams, tile.roi, _normalMap_dmp, _queue, _sgmParams.scale, _sgmParams.stepXY, "sgm");
        }
    }

    ALICEVISION_LOG_INFO(tile << "SGM depth/thickness map done.");

    return finished;
}

sycl::event Sgm::smoothThicknessMap(const Tile& tile, const RefineParams& refineParams, sycl::event prerequisite)
{
    ALICEVISION_LOG_INFO(tile << "SGM Smooth thickness map.");

    // downscale the region of interest
    const ROI downscaledRoi = downscaleROI(tile.roi, _sgmParams.scale * _sgmParams.stepXY);

    // in-place result thickness map smoothing with adjacent pixels
    sycl::event compute = sycl_depthThicknessSmoothThickness(_depthThicknessMap_dmp, _sgmParams, refineParams, downscaledRoi, _queue, prerequisite);

    ALICEVISION_LOG_INFO(tile << "SGM Smooth thickness map done.");

    return compute;
}

sycl::event Sgm::computeSimilarityVolumes(const Tile& tile, const SgmDepthList& tileDepthList, sycl::event prerequisite)
{
    ALICEVISION_LOG_INFO(tile << "SGM Compute similarity volume.");

    // downscale the region of interest
    const ROI downscaledRoi = downscaleROI(tile.roi, _sgmParams.scale * _sgmParams.stepXY);

    // initialize the two similarity volumes at 255
    sycl::event bestSimInit = sycl_volumeInitialize(_volumeBestSim_dmp, 255.f, _queue, prerequisite);
    sycl::event secBestSimInit = sycl_volumeInitialize(_volumeSecBestSim_dmp, 255.f, _queue, prerequisite);

    // get device cache instance
    DeviceCache& deviceCache = DeviceCache::getInstance();

    // get R device mipmap image from cache
    const DeviceMipmapImage& rcDeviceMipmapImage = deviceCache.requestMipmapImage(tile.rc, _mp, _queue);

    // compute similarity volume per Rc Tc
    std::vector<sycl::event> rcTcComputed(tile.refineTCams.size());
    for (std::size_t tci = 0; tci < tile.sgmTCams.size(); ++tci)
    {
        const int tc = tile.sgmTCams.at(tci);

        const int firstDepth = tileDepthList.getDepthsTcLimits()[tci].x;
        const int lastDepth = firstDepth + tileDepthList.getDepthsTcLimits()[tci].y;

        const Range tcDepthRange(firstDepth, lastDepth);

        // get T device mipmap image from cache
        const DeviceMipmapImage& tcDeviceMipmapImage = deviceCache.requestMipmapImage(tc, _mp, _queue);

        ALICEVISION_LOG_DEBUG(tile << "Compute similarity volume:" << std::endl
                                   << "\t- rc: " << tile.rc << std::endl
                                   << "\t- tc: " << tc << " (" << (tci + 1) << "/" << tile.sgmTCams.size() << ")" << std::endl
                                   << "\t- tc first depth: " << firstDepth << std::endl
                                   << "\t- tc last depth: " << lastDepth << std::endl
                                   << "\t- tile range x: [" << downscaledRoi.x.begin << " - " << downscaledRoi.x.end << "]" << std::endl
                                   << "\t- tile range y: [" << downscaledRoi.y.begin << " - " << downscaledRoi.y.end << "]" << std::endl);

        rcTcComputed.at(tci) = sycl_volumeComputeSimilarity(
                                     _volumeBestSim_dmp,
                                     _volumeSecBestSim_dmp,
                                     _depths_dmp,
                                     tile.rc,
                                     tc,
                                     _mp,
                                     rcDeviceMipmapImage,
                                     tcDeviceMipmapImage,
                                     _sgmParams,
                                     tcDepthRange,
                                     downscaledRoi,
                                     _queue,
                                     { bestSimInit, secBestSimInit });
    }

    // update second best uninitialized similarity volume values with first best similarity volume values
    // - allows to avoid the particular case with a single tc (second best volume has no valid similarity values)
    // - useful if a tc alone contributes to the calculation of a subpart of the similarity volume
    sycl::event finished;
    if (_sgmParams.updateUninitializedSim)  // should always be true, false for debug purposes
    {
        ALICEVISION_LOG_DEBUG(tile << "SGM Update uninitialized similarity volume values from best similarity volume.");

        finished = sycl_volumeUpdateUninitializedSimilarity(_volumeBestSim_dmp, _volumeSecBestSim_dmp, _queue, rcTcComputed);
    }
    else // ensure debug still works correctly, at performance cost
    {
        for (sycl::event e : rcTcComputed)
            e.wait();
        finished = sycl::event();
    }

    ALICEVISION_LOG_INFO(tile << "SGM Compute similarity volume done.");

    return finished;
}

sycl::event Sgm::optimizeSimilarityVolume(const Tile& tile, const SgmDepthList& tileDepthList, sycl::event prerequisite)
{
    ALICEVISION_LOG_INFO(tile << "SGM Optimizing volume (filtering axes: " << _sgmParams.filteringAxes << ").");

    // downscale the region of interest
    const ROI downscaledRoi = downscaleROI(tile.roi, _sgmParams.scale * _sgmParams.stepXY);

    // get R device mipmap image from cache
    DeviceCache& deviceCache = DeviceCache::getInstance();
    const DeviceMipmapImage& rcDeviceMipmapImage = deviceCache.requestMipmapImage(tile.rc, _mp, _queue);

    sycl::event finished = sycl_volumeOptimize(
                        _volumeBestSim_dmp,     // output volume (reuse best sim to put optimized similarity)
                        _volumeSliceAccA_dmp,   // slice A accumulation buffer pre-allocate
                        _volumeSliceAccB_dmp,   // slice B accumulation buffer pre-allocate
                        _volumeAxisAcc_dmp,     // axis accumulation buffer pre-allocate
                        _volumeSecBestSim_dmp,  // input volume
                        rcDeviceMipmapImage,
                        _sgmParams,
                        tileDepthList.getDepths().size(),
                        downscaledRoi,
                        _queue,
                        prerequisite);

    ALICEVISION_LOG_INFO(tile << "SGM Optimizing volume done.");

    return finished;
}

sycl::event Sgm::retrieveBestDepth(const Tile& tile, const SgmDepthList& tileDepthList, sycl::event prerequisite)
{
    ALICEVISION_LOG_INFO(tile << "SGM Retrieve best depth in volume.");

    // downscale the region of interest
    const ROI downscaledRoi = downscaleROI(tile.roi, _sgmParams.scale * _sgmParams.stepXY);

    // get depth range
    const Range depthRange(0, tileDepthList.getDepths().size());

    sycl::event finished = sycl_volumeRetrieveBestDepth(
                                 _depthThicknessMap_dmp,  // output depth thickness map
                                 _depthSimMap_dmp,        // output depth/sim map (or empty)
                                 _depths_dmp,             // rc depth
                                 _volumeBestSim_dmp,      // second best sim volume optimized in best sim volume
                                 tile.rc,
                                 _mp,
                                 _sgmParams,
                                 depthRange,
                                 downscaledRoi,
                                 _queue,
                                 prerequisite);

    ALICEVISION_LOG_INFO(tile << "SGM Retrieve best depth in volume done.");

    return finished;
}

void Sgm::exportVolumeInformation(const Tile& tile,
                                  const SgmDepthList& tileDepthList,
                                  const SyclDeviceMemoryPitched<TSim, 3>& in_volume_dmp,
                                  const std::string& name,
                                  sycl::event prerequisite) const
{
    if (!_sgmParams.exportIntermediateVolumes && !_sgmParams.exportIntermediateCrossVolumes && !_sgmParams.exportIntermediateVolume9pCsv)
    {
        // nothing to do
        return;
    }

    // get file tile begin indexes (default is single tile)
    int tileBeginX = -1;
    int tileBeginY = -1;

    if (tile.nbTiles > 1)
    {
        tileBeginX = tile.roi.x.begin;
        tileBeginY = tile.roi.y.begin;
    }

    // copy device similarity volume to host memory
    SyclHostMemoryHeap<TSim, 3> volumeSim_hmh(in_volume_dmp.getSize(), _queue);
    volumeSim_hmh.copyFrom(in_volume_dmp, _queue, prerequisite).wait();

    if (_sgmParams.exportIntermediateVolumes)
    {
        ALICEVISION_LOG_INFO(tile << "Export similarity volume (" << name << ").");

        const std::string volumePath = getFileNameFromIndex(_mp, tile.rc, mvsUtils::EFileType::volume, "_" + name, tileBeginX, tileBeginY);

        exportSimilarityVolume(volumeSim_hmh, tileDepthList.getDepths(), _mp, tile.rc, _sgmParams, volumePath, tile.roi);

        ALICEVISION_LOG_INFO(tile << "Export similarity volume (" << name << ") done.");
    }

    if (_sgmParams.exportIntermediateCrossVolumes)
    {
        ALICEVISION_LOG_INFO(tile << "Export similarity volume cross (" << name << ").");

        const std::string volumeCrossPath = getFileNameFromIndex(_mp, tile.rc, mvsUtils::EFileType::volumeCross, "_" + name, tileBeginX, tileBeginY);

        exportSimilarityVolumeCross(volumeSim_hmh, tileDepthList.getDepths(), _mp, tile.rc, _sgmParams, volumeCrossPath, tile.roi);

        ALICEVISION_LOG_INFO(tile << "Export similarity volume cross (" << name << ") done.");
    }

    if (_sgmParams.exportIntermediateTopographicCutVolumes)
    {
        ALICEVISION_LOG_INFO(tile << "Export similarity volume topographic cut (" << name << ").");

        const std::string volumeCutPath =
          getFileNameFromIndex(_mp, tile.rc, mvsUtils::EFileType::volumeTopographicCut, "_" + name, tileBeginX, tileBeginY);

        exportSimilarityVolumeTopographicCut(volumeSim_hmh, tileDepthList.getDepths(), _mp, tile.rc, _sgmParams, volumeCutPath, tile.roi);

        ALICEVISION_LOG_INFO(tile << "Export similarity volume topographic cut (" << name << ") done.");
    }

    if (_sgmParams.exportIntermediateVolume9pCsv)
    {
        ALICEVISION_LOG_INFO(tile << "Export similarity volume 9 points CSV (" << name << ").");

        const std::string stats9Path = getFileNameFromIndex(_mp, tile.rc, mvsUtils::EFileType::stats9p, "_sgm", tileBeginX, tileBeginY);

        exportSimilaritySamplesCSV(volumeSim_hmh, tileDepthList.getDepths(), name, _sgmParams, stats9Path, tile.roi);

        ALICEVISION_LOG_INFO(tile << "Export similarity volume 9 points CSV (" << name << ") done.");
    }
}

}  // namespace depthMap
}  // namespace aliceVision
