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
#include <aliceVision/depthMap/depthMapUtils.hpp>
#include <aliceVision/depthMap/volumeIO.hpp>
#include <aliceVision/depthMap/cuda/host/DeviceCache.hpp>
#include <aliceVision/depthMap/cuda/planeSweeping/deviceDepthSimilarityMap.hpp>
#include <aliceVision/depthMap/cuda/planeSweeping/deviceSimilarityVolume.hpp>

namespace aliceVision {
namespace depthMap {

Refine::Refine(const mvsUtils::MultiViewParams& mp,
               const mvsUtils::TileParams& tileParams, 
               const RefineParams& refineParams, 
               cudaStream_t stream)
    : _mp(mp)
    , _tileParams(tileParams)
    , _refineParams(refineParams)
    , _stream(stream)
{
    // get tile maximum dimensions
    const int downscale = _refineParams.scale * _refineParams.stepXY;
    const int maxTileWidth  = std::ceil(tileParams.width  / float(downscale));
    const int maxTileHeight = std::ceil(tileParams.height / float(downscale));

    // compute depth/sim map maximum dimensions
    const CudaSize<2> depthSimMapDim(maxTileWidth, maxTileHeight);

    // allocate depth/sim maps in device memory
    _sgmDepthPixSizeMap_dmp.allocate(depthSimMapDim);
    _refinedDepthSimMap_dmp.allocate(depthSimMapDim);
    _optimizedDepthSimMap_dmp.allocate(depthSimMapDim);

    // allocate normal map in device memory
    if(refineParams.useNormalMap)
      _normalMap_dmp.allocate(depthSimMapDim);

    // compute volume maximum dimensions
    const CudaSize<3> volDim(maxTileWidth, maxTileHeight, _refineParams.nDepthsToRefine);

    // allocate refine volume in device memory
    _volumeRefineSim_dmp.allocate(volDim);

    // allocate depth/sim map optimization buffers
    if(_refineParams.doRefineOptimization)
    {
        _optImgVariance_dmp.allocate(depthSimMapDim);
        _optTmpDepthMap_dmp.allocate(depthSimMapDim);
    }
}

double Refine::getDeviceMemoryConsumption() const
{
    size_t bytes = 0;

    bytes += _sgmDepthPixSizeMap_dmp.getBytesPadded();
    bytes += _refinedDepthSimMap_dmp.getBytesPadded();
    bytes += _optimizedDepthSimMap_dmp.getBytesPadded();
    bytes += _normalMap_dmp.getBytesPadded();
    bytes += _volumeRefineSim_dmp.getBytesPadded();

    if(_refineParams.doRefineOptimization)
    {
        bytes += _optImgVariance_dmp.getBytesPadded();
        bytes += _optTmpDepthMap_dmp.getBytesPadded();
    }

    return (double(bytes) / (1024.0 * 1024.0));
}

double Refine::getDeviceMemoryConsumptionUnpadded() const
{
    size_t bytes = 0;

    bytes += _sgmDepthPixSizeMap_dmp.getBytesUnpadded();
    bytes += _refinedDepthSimMap_dmp.getBytesUnpadded();
    bytes += _optimizedDepthSimMap_dmp.getBytesUnpadded();
    bytes += _normalMap_dmp.getBytesUnpadded();
    bytes += _volumeRefineSim_dmp.getBytesUnpadded();

    if(_refineParams.doRefineOptimization)
    {
        bytes += _optImgVariance_dmp.getBytesUnpadded();
        bytes += _optTmpDepthMap_dmp.getBytesUnpadded();
    }

    return (double(bytes) / (1024.0 * 1024.0));
}

void Refine::refineRc(const Tile& tile, const CudaDeviceMemoryPitched<float2, 2>& in_sgmDepthSimMap_dmp)
{
    const IndexT viewId = _mp.getViewId(tile.rc);

    ALICEVISION_LOG_INFO(tile << "Refine depth/sim map of view id: " << viewId << ", rc: " << tile.rc << " (" << (tile.rc + 1) << " / " << _mp.ncams << ").");

    // compute upscaled SGM depth/pixSize map
    {
        // downscale the region of interest
        const ROI downscaledRoi = downscaleROI(tile.roi, _refineParams.scale * _refineParams.stepXY);

        // get R device camera from cache
        DeviceCache& deviceCache = DeviceCache::getInstance();
        const DeviceCamera& rcDeviceCamera = deviceCache.requestCamera(tile.rc, _refineParams.scale, _mp);

        // upscale SGM depth/sim map
        cuda_depthSimMapUpscale(_sgmDepthPixSizeMap_dmp, in_sgmDepthSimMap_dmp, _stream);

        if(_refineParams.exportIntermediateResults)
          writeDepthSimMap(tile.rc, _mp, _tileParams, tile.roi, _sgmDepthPixSizeMap_dmp, _refineParams.scale, _refineParams.stepXY, "_sgmUpscaled");

        // compute pixSize to replace similarity (this is usefull for depth/sim map optimization)
        cuda_depthSimMapComputePixSize(_sgmDepthPixSizeMap_dmp, rcDeviceCamera, _refineParams, downscaledRoi, _stream);

        // compute normal map from upscaled depth/pixSize map if needed
        if(_refineParams.useNormalMap)
        {
            cuda_depthSimMapComputeNormal(_normalMap_dmp, _sgmDepthPixSizeMap_dmp, rcDeviceCamera, downscaledRoi, _stream);
            writeDeviceImage(_normalMap_dmp, getFileNameFromIndex(_mp, tile.rc, mvsUtils::EFileType::depthMap, _refineParams.scale, "Normal", tile.roi.x.begin, tile.roi.y.begin));
        }
    }

    // refine and fuse depth/sim map
    if(_refineParams.doRefineFuse)
    {
        // refine and fuse with volume strategy
        refineAndFuseDepthSimMap(tile);

    }
    else
    {
        cuda_depthSimMapCopyDepthOnly(_refinedDepthSimMap_dmp, _sgmDepthPixSizeMap_dmp, 1.0f, _stream);
    }

    if(_refineParams.exportIntermediateResults)
      writeDepthSimMap(tile.rc, _mp, _tileParams, tile.roi, _refinedDepthSimMap_dmp, _refineParams.scale, _refineParams.stepXY, "_refinedFused");

    // optimize depth/sim map
    if(_refineParams.doRefineOptimization && _refineParams.optimizationNbIters > 0)
    {
        optimizeDepthSimMap(tile);
    }
    else
    {
        _optimizedDepthSimMap_dmp.copyFrom(_refinedDepthSimMap_dmp, _stream);
    }

    ALICEVISION_LOG_INFO(tile << "Refine depth/sim map done.");
}

void Refine::refineAndFuseDepthSimMap(const Tile& tile)
{
    ALICEVISION_LOG_INFO(tile << "Refine and fuse depth/sim map volume.");

    // downscale the region of interest
    const ROI downscaledRoi = downscaleROI(tile.roi, _refineParams.scale * _refineParams.stepXY);

    // get the depth range
    const Range depthRange(0, _volumeRefineSim_dmp.getSize().z());

    // initialize the similarity volume at 0
    // each tc filtered and inverted similarity value will be summed in this volume
    cuda_volumeInitialize(_volumeRefineSim_dmp, 0.f, _stream);

    // get device cache instance
    DeviceCache& deviceCache = DeviceCache::getInstance();

    // get R device camera from cache
    const DeviceCamera& rcDeviceCamera = deviceCache.requestCamera(tile.rc, _refineParams.scale, _mp);

    // compute for each RcTc each similarity value for each depth to refine
    // sum the inverted / filtered similarity value, best value is the HIGHEST
    for(std::size_t tci = 0; tci < tile.refineTCams.size(); ++tci)
    {
        const int tc = tile.refineTCams.at(tci);

        // get T device camera from cache
        const DeviceCamera& tcDeviceCamera = deviceCache.requestCamera(tc, _refineParams.scale, _mp);

        ALICEVISION_LOG_DEBUG(tile << "Refine similarity volume:" << std::endl
                                   << "\t- rc: " << tile.rc << std::endl
                                   << "\t- tc: " << tc << " (" << (tci + 1) << "/" << tile.refineTCams.size() << ")" << std::endl
                                   << "\t- rc camera device id: " << rcDeviceCamera.getDeviceCamId() << std::endl
                                   << "\t- tc camera device id: " << tcDeviceCamera.getDeviceCamId() << std::endl
                                   << "\t- tile range x: [" << downscaledRoi.x.begin << " - " << downscaledRoi.x.end << "]" << std::endl
                                   << "\t- tile range y: [" << downscaledRoi.y.begin << " - " << downscaledRoi.y.end << "]" << std::endl);

        cuda_volumeRefineSimilarity(_volumeRefineSim_dmp, 
                                    _sgmDepthPixSizeMap_dmp,
                                    (_refineParams.useNormalMap) ? &_normalMap_dmp : nullptr,
                                    rcDeviceCamera, 
                                    tcDeviceCamera,
                                    _refineParams, 
                                    depthRange,
                                    downscaledRoi, 
                                    _stream);
    }

    if(_refineParams.exportIntermediateResults)
        exportVolumeInformation(tile, "afterRefine");

    // retrieve the best depth/sim in the volume
    // compute sub-pixel sample using a sliding gaussian 
    cuda_volumeRefineBestDepth(_refinedDepthSimMap_dmp, 
                                _sgmDepthPixSizeMap_dmp, 
                                _volumeRefineSim_dmp,
                                rcDeviceCamera, 
                                _refineParams,
                                downscaledRoi, 
                                _stream);
    
    ALICEVISION_LOG_INFO(tile << "Refine and fuse depth/sim map volume done.");
}

void Refine::optimizeDepthSimMap(const Tile& tile)
{
    ALICEVISION_LOG_INFO(tile << "Optimize depth/sim map.");

    // downscale the region of interest
    const ROI downscaledRoi = downscaleROI(tile.roi, _refineParams.scale * _refineParams.stepXY);
    
    // get R device camera from cache
    DeviceCache& deviceCache = DeviceCache::getInstance();
    const DeviceCamera& rcDeviceCamera = deviceCache.requestCamera(tile.rc, _refineParams.scale, _mp);

    cuda_depthSimMapOptimizeGradientDescent(_optimizedDepthSimMap_dmp, // output depth/sim map optimized
                                            _optImgVariance_dmp,       // image variance buffer pre-allocate
                                            _optTmpDepthMap_dmp,       // temporary depth map buffer pre-allocate
                                            _sgmDepthPixSizeMap_dmp,   // input SGM upscaled depth/pixSize map
                                            _refinedDepthSimMap_dmp,   // input refined and fused depth/sim map
                                            rcDeviceCamera,
                                            _refineParams,
                                            downscaledRoi,
                                            _stream);

    ALICEVISION_LOG_INFO(tile << "Optimize depth/sim map done.");
}

void Refine::exportVolumeInformation(const Tile& tile, const std::string& name) const
{
    // get tile begin indexes (default no tile)
    int tileBeginX = -1;
    int tileBeginY = -1;

    if(tile.nbTiles > 1)
    {
        tileBeginX = tile.roi.x.begin;
        tileBeginY = tile.roi.y.begin;
    }

    CudaHostMemoryHeap<TSimRefine, 3> volumeSim_hmh(_volumeRefineSim_dmp.getSize());
    volumeSim_hmh.copyFrom(_volumeRefineSim_dmp);

    CudaHostMemoryHeap<float2, 2> depthPixSizeMapSgmUpscale_hmh(_sgmDepthPixSizeMap_dmp.getSize());
    depthPixSizeMapSgmUpscale_hmh.copyFrom(_sgmDepthPixSizeMap_dmp);

    const std::string volumeCrossPath = getFileNameFromIndex(_mp, tile.rc, mvsUtils::EFileType::volumeCross, _refineParams.scale, "_" + name, tileBeginX, tileBeginY);
    const std::string stats9Path = getFileNameFromIndex(_mp, tile.rc, mvsUtils::EFileType::stats9p, _refineParams.scale, "_refine", tileBeginX, tileBeginY);

    exportSimilarityVolumeCross(volumeSim_hmh, depthPixSizeMapSgmUpscale_hmh, _mp, tile.rc, _refineParams, volumeCrossPath, tile.roi);
    exportSimilaritySamplesCSV(volumeSim_hmh, tile.rc, name, stats9Path);
}

} // namespace depthMap
} // namespace aliceVision
