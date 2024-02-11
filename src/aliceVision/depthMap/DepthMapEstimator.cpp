// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "DepthMapEstimator.hpp"

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/mvsUtils/fileIO.hpp>
#include <aliceVision/mvsUtils/mapIO.hpp>
#include <aliceVision/mvsUtils/MultiViewParams.hpp>
#include <aliceVision/depthMap/depthMapUtils.hpp>
#include <aliceVision/depthMap/DepthMapParams.hpp>
#include <aliceVision/depthMap/SgmDepthList.hpp>
#include <aliceVision/depthMap/Sgm.hpp>
#include <aliceVision/depthMap/Refine.hpp>
#include <aliceVision/depthMap/cuda/host/utils.hpp>
#include <aliceVision/depthMap/cuda/host/patchPattern.hpp>
#include <aliceVision/depthMap/cuda/host/DeviceCache.hpp>
#include <aliceVision/depthMap/cuda/host/DeviceStreamManager.hpp>
#include <aliceVision/depthMap/cuda/planeSweeping/deviceDepthSimilarityMap.hpp>

namespace aliceVision {
namespace depthMap {

DepthMapEstimator::DepthMapEstimator(const mvsUtils::MultiViewParams& mp,
                                     const mvsUtils::TileParams& tileParams,
                                     const DepthMapParams& depthMapParams,
                                     const SgmParams& sgmParams,
                                     const RefineParams& refineParams)
  : _mp(mp),
    _tileParams(tileParams),
    _depthMapParams(depthMapParams),
    _sgmParams(sgmParams),
    _refineParams(refineParams)
{
    // compute maximum downscale (scaleStep)
    const int maxDownscale = std::max(_sgmParams.scale * _sgmParams.stepXY, _refineParams.scale * _refineParams.stepXY);

    // compute tile ROI list
    getTileRoiList(_tileParams, _mp.getMaxImageWidth(), _mp.getMaxImageHeight(), maxDownscale, _tileRoiList);

    // log tiling information and ROI list
    logTileRoiList(_tileParams, _mp.getMaxImageWidth(), _mp.getMaxImageHeight(), maxDownscale, _tileRoiList);

    // log SGM downscale & stepXY
    ALICEVISION_LOG_INFO("SGM parameters:" << std::endl << "\t- scale: " << _sgmParams.scale << std::endl << "\t- stepXY: " << _sgmParams.stepXY);

    // log Refine downscale & stepXY
    ALICEVISION_LOG_INFO("Refine parameters:" << std::endl
                                              << "\t- scale: " << _refineParams.scale << std::endl
                                              << "\t- stepXY: " << _refineParams.stepXY);
}

int DepthMapEstimator::getNbSimultaneousTiles() const
{
    const int nbTilesPerCamera = _tileRoiList.size();

    // mipmap image cost
    // mipmap image should not exceed (1.5 * max_width) * max_height
    // TODO: special case first mipmap level != 1
    const double mipmapCostMB =
      ((_mp.getMaxImageWidth() * 1.5) * _mp.getMaxImageHeight() * sizeof(CudaRGBA)) / (1024.0 * 1024.0);  // process downscale apply

    // cameras cost per R camera computation
    // Rc mipmap + Tcs mipmaps
    const double rcCamsCostMB = mipmapCostMB + _depthMapParams.maxTCams * mipmapCostMB;

    // number of camera parameters in device constant memory
    // (Rc + Tcs) * 2 (SGM + Refine downscale) + 1 (SGM needs downscale 1)
    // note: special case SGM downsccale = Refine downscale not handle
    const int rcNbCameraParams = (_depthMapParams.useRefine) ? 2 : 1;
    const int rcCamParams = (1 /* rc */ + _depthMapParams.maxTCams) * rcNbCameraParams + ((_refineParams.scale > 1) ? 1 : 0);

    // single tile SGM cost
    double sgmTileCostMB = 0.0;
    double sgmTileCostUnpaddedMB = 0.0;

    {
        const bool sgmComputeDepthSimMap = !_depthMapParams.useRefine;
        const bool sgmComputeNormalMap = _refineParams.useSgmNormalMap;

        Sgm sgm(_mp, _tileParams, _sgmParams, sgmComputeDepthSimMap, sgmComputeNormalMap, 0 /*stream*/);
        sgmTileCostMB = sgm.getDeviceMemoryConsumption();
        sgmTileCostUnpaddedMB = sgm.getDeviceMemoryConsumptionUnpadded();
    }

    // single tile Refine cost
    double refineTileCostMB = 0.0;
    double refineTileCostUnpaddedMB = 0.0;

    if (_depthMapParams.useRefine)
    {
        Refine refine(_mp, _tileParams, _refineParams, 0 /*stream*/);
        refineTileCostMB = refine.getDeviceMemoryConsumption();
        refineTileCostUnpaddedMB = refine.getDeviceMemoryConsumptionUnpadded();
    }

    // tile computation cost
    // SGM tile cost + Refine tile cost
    const double tileCostMB = sgmTileCostMB + refineTileCostMB;
    const double tileCostUnpaddedMB = sgmTileCostUnpaddedMB + refineTileCostUnpaddedMB;

    // min/max cost of an R camera computation
    // min cost for a single tile computation
    // max cost for all tiles computation
    const double rcMinCostMB = rcCamsCostMB + tileCostMB;
    const double rcMaxCostMB = rcCamsCostMB + nbTilesPerCamera * tileCostMB;

    // available device memory
    double deviceMemoryMB;
    {
        double availableMB, usedMB, totalMB;
        getDeviceMemoryInfo(availableMB, usedMB, totalMB);
        deviceMemoryMB = availableMB * 0.8;  // available memory margin
    }

    // number of full R camera computation that can be done simultaneously
    int nbSimultaneousFullRc = static_cast<int>(deviceMemoryMB / rcMaxCostMB);

    // try to add a part of an R camera computation
    int nbRemainingTiles = 0;
    {
        const double remainingMemoryMB = deviceMemoryMB - (nbSimultaneousFullRc * rcMaxCostMB);
        nbRemainingTiles = static_cast<int>(std::max(0.0, remainingMemoryMB - rcCamsCostMB) / tileCostMB);
    }

    // check that we do not need more constant camera parameters than the ones in device constant memory
    if (ALICEVISION_DEVICE_MAX_CONSTANT_CAMERA_PARAM_SETS < ((nbSimultaneousFullRc + ((nbRemainingTiles > 0) ? 1 : 0)) * rcCamParams))
    {
        const int previousNbSimultaneousFullRc = nbSimultaneousFullRc;
        nbSimultaneousFullRc = static_cast<int>(ALICEVISION_DEVICE_MAX_CONSTANT_CAMERA_PARAM_SETS / rcCamParams);

        ALICEVISION_LOG_INFO("DepthMapEstimator::getNbSimultaneousTiles(): limit the number of simultaneous RC due to "
                             << "the max constant memory for camera params from " << previousNbSimultaneousFullRc << " to " << nbSimultaneousFullRc);
        nbRemainingTiles = 0;
    }

    // compute number of simultaneous tiles
    const int out_nbSimultaneousTiles = nbSimultaneousFullRc * nbTilesPerCamera + nbRemainingTiles;

    // log memory information
    ALICEVISION_LOG_INFO("Device memory:" << std::endl
                                          << "\t- available: " << deviceMemoryMB << " MB" << std::endl
                                          << "\t- requirement for the first tile: " << rcMinCostMB << " MB" << std::endl
                                          << "\t- # computation buffers per tile: " << tileCostMB << " MB"
                                          << " (Sgm: " << sgmTileCostMB << " MB"
                                          << ", Refine: " << refineTileCostMB << " MB)" << std::endl
                                          << "\t- # input images (R + " << _depthMapParams.maxTCams << " Ts): " << rcCamsCostMB
                                          << " MB (single mipmap image size: " << mipmapCostMB << " MB)");

    ALICEVISION_LOG_DEBUG("Theoretical device memory cost for a tile without padding: " << tileCostUnpaddedMB << " MB"
                                                                                        << " (Sgm: " << sgmTileCostUnpaddedMB << " MB"
                                                                                        << ", Refine: " << refineTileCostUnpaddedMB << " MB)");

    ALICEVISION_LOG_INFO("Parallelization:" << std::endl
                                            << "\t- # tiles per image: " << nbTilesPerCamera << std::endl
                                            << "\t- # simultaneous depth maps computation: "
                                            << ((nbRemainingTiles < 1) ? nbSimultaneousFullRc : (nbSimultaneousFullRc + 1)) << std::endl
                                            << "\t- # simultaneous tiles computation: " << out_nbSimultaneousTiles);

    // check at least one single tile computation
    if (rcCamParams > ALICEVISION_DEVICE_MAX_CONSTANT_CAMERA_PARAM_SETS || out_nbSimultaneousTiles < 1)
    {
        ALICEVISION_THROW_ERROR("Not enough GPU memory to compute a single tile.");
    }

    return out_nbSimultaneousTiles;
}

void DepthMapEstimator::getTilesList(const std::vector<int>& cams, std::vector<Tile>& tiles) const
{
    const int nbTilesPerCamera = _tileRoiList.size();

    // tiles list should be empty
    assert(tiles.empty());

    // reserve memory
    tiles.reserve(cams.size() * nbTilesPerCamera);

    for (int rc : cams)
    {
        // get R camera Tcs list
        const std::vector<int> tCams = _mp.findNearestCamsFromLandmarks(rc, _depthMapParams.maxTCams).getDataWritable();

        // get R camera ROI
        const ROI rcImageRoi(Range(0, _mp.getWidth(rc)), Range(0, _mp.getHeight(rc)));

        for (std::size_t i = 0; i < nbTilesPerCamera; ++i)
        {
            Tile t;

            t.id = i;
            t.nbTiles = nbTilesPerCamera;
            t.rc = rc;
            t.roi = intersect(_tileRoiList.at(i), rcImageRoi);

            if (t.roi.isEmpty())
            {
                // do nothing, this ROI cannot intersect the R camera ROI.
            }
            else if (_depthMapParams.chooseTCamsPerTile)
            {
                // find nearest T cameras per tile
                t.sgmTCams = _mp.findTileNearestCams(rc, _sgmParams.maxTCamsPerTile, tCams, t.roi);

                if (_depthMapParams.useRefine)
                    t.refineTCams = _mp.findTileNearestCams(rc, _refineParams.maxTCamsPerTile, tCams, t.roi);
            }
            else
            {
                // use previously selected T cameras from the entire image
                t.sgmTCams = tCams;
                t.refineTCams = tCams;
            }

            tiles.push_back(t);
        }
    }
}

void DepthMapEstimator::compute(int cudaDeviceId, const std::vector<int>& cams)
{
    // set the device to use for GPU executions
    // the CUDA runtime API is thread-safe, it maintains per-thread state about the current device
    setCudaDeviceId(cudaDeviceId);

    // initialize RAM image cache
    // note: maybe move it as class member in order to share it across multiple GPUs
    mvsUtils::ImagesCache<image::Image<image::RGBAfColor>> ic(_mp, image::EImageColorSpace::LINEAR);

    // build tile list order by R camera
    std::vector<Tile> tiles;
    getTilesList(cams, tiles);

    // get maximum number of simultaneous tiles
    // for now, we use one CUDA stream per tile (SGM + Refine)
    const int nbStreams = std::min(getNbSimultaneousTiles(), static_cast<int>(tiles.size()));
    DeviceStreamManager deviceStreamManager(nbStreams);

    // constants
    const bool hasRcSameDownscale = (_sgmParams.scale == _refineParams.scale);  // we only need one camera params per image
    const bool hasRcWithoutDownscale =
      _sgmParams.scale == 1 || (_depthMapParams.useRefine && _refineParams.scale == 1);  // we need R camera params SGM (downscale = 1)
    const int nbCameraParamsPerSgm =
      (1 + _depthMapParams.maxTCams) + (hasRcWithoutDownscale ? 0 : 1);  // number of Sgm camera parameters per R camera
    const int nbCameraParamsPerRefine =
      (_depthMapParams.useRefine && !hasRcSameDownscale) ? (1 + _depthMapParams.maxTCams) : 0;  // number of Refine camera parameters per R camera

    // build device cache
    const int nbTilesPerCamera = static_cast<int>(_tileRoiList.size());

    int nbRcPerBatch = divideRoundUp(nbStreams, nbTilesPerCamera);  // number of R cameras in the same batch
    if (nbRcPerBatch * (nbCameraParamsPerSgm + nbCameraParamsPerRefine) > ALICEVISION_DEVICE_MAX_CONSTANT_CAMERA_PARAM_SETS)
    {
        int previousNbRcPerBatch = nbRcPerBatch;
        nbRcPerBatch = ALICEVISION_DEVICE_MAX_CONSTANT_CAMERA_PARAM_SETS / (nbCameraParamsPerSgm + nbCameraParamsPerRefine);
        ALICEVISION_LOG_INFO("DepthMapEstimator::compute(): limit the number of simultaneous RC due to the max constant"
                             << " memory for camera params from " << previousNbRcPerBatch << " to " << nbRcPerBatch);
    }

    const int nbCamerasParamsPerBatch =
      nbRcPerBatch * (nbCameraParamsPerSgm + nbCameraParamsPerRefine);                 // number of camera parameters in the same batch
    const int nbTilesPerBatch = nbRcPerBatch * nbTilesPerCamera;                       // number of tiles in the same batch
    const int nbMipmapImagesPerBatch = nbRcPerBatch * (1 + _depthMapParams.maxTCams);  // number of camera mipmap image in the same batch

    DeviceCache& deviceCache = DeviceCache::getInstance();
    deviceCache.build(nbMipmapImagesPerBatch, nbCamerasParamsPerBatch);

    // build custom patch pattern in CUDA constant memory
    if (_sgmParams.useCustomPatchPattern || _refineParams.useCustomPatchPattern)
        buildCustomPatchPattern(_depthMapParams.customPatchPattern);

    // allocate Sgm and Refine per stream in device memory
    std::vector<Sgm> sgmPerStream;
    std::vector<Refine> refinePerStream;

    sgmPerStream.reserve(nbStreams);
    refinePerStream.reserve(_depthMapParams.useRefine ? nbStreams : 0);

    // initialize Sgm and Refine objects
    {
        const bool sgmComputeDepthSimMap = !_depthMapParams.useRefine;
        const bool sgmComputeNormalMap = _refineParams.useSgmNormalMap;

        // initialize Sgm objects
        for (int i = 0; i < nbStreams; ++i)
            sgmPerStream.emplace_back(_mp, _tileParams, _sgmParams, sgmComputeDepthSimMap, sgmComputeNormalMap, deviceStreamManager.getStream(i));

        // initialize Refine objects
        if (_depthMapParams.useRefine)
            for (int i = 0; i < nbStreams; ++i)
                refinePerStream.emplace_back(_mp, _tileParams, _refineParams, deviceStreamManager.getStream(i));
    }

    // allocate final deth/similarity map tile list in host memory
    std::vector<std::vector<CudaHostMemoryHeap<float2, 2>>> depthSimMapTilePerCam(nbRcPerBatch);
    std::vector<std::vector<std::pair<float, float>>> depthMinMaxTilePerCam(nbRcPerBatch);

    for (int i = 0; i < nbRcPerBatch; ++i)
    {
        auto& depthSimMapTiles = depthSimMapTilePerCam.at(i);
        auto& depthMinMaxTiles = depthMinMaxTilePerCam.at(i);

        depthSimMapTiles.resize(nbTilesPerCamera);
        depthMinMaxTiles.resize(nbTilesPerCamera);

        for (int j = 0; j < nbTilesPerCamera; ++j)
        {
            if (_depthMapParams.useRefine)
                depthSimMapTiles.at(j).allocate(refinePerStream.front().getDeviceDepthSimMap().getSize());
            else  // final depth/similarity map is SGM only
                depthSimMapTiles.at(j).allocate(sgmPerStream.front().getDeviceDepthSimMap().getSize());
        }
    }

    // log device memory information
    logDeviceMemoryInfo();

    // compute number of batches
    const int nbBatches = divideRoundUp(static_cast<int>(tiles.size()), nbTilesPerBatch);
    const int minMipmapDownscale = std::min(_refineParams.scale, _sgmParams.scale);
    const int maxMipmapDownscale = std::max(_refineParams.scale, _sgmParams.scale) * std::pow(2, 6);  // we add 6 downscale levels

    // compute each batch of R cameras
    for (int b = 0; b < nbBatches; ++b)
    {
        // find first/last tile to compute
        const int firstTileIndex = b * nbTilesPerBatch;
        const int lastTileIndex = std::min((b + 1) * nbTilesPerBatch, static_cast<int>(tiles.size()));

        // load tile R and corresponding T cameras in device cache
        for (int i = firstTileIndex; i < lastTileIndex; ++i)
        {
            const Tile& tile = tiles.at(i);

            // add Sgm R camera to Device cache
            deviceCache.addMipmapImage(tile.rc, minMipmapDownscale, maxMipmapDownscale, ic, _mp);
            deviceCache.addCameraParams(tile.rc, _sgmParams.scale, _mp);

            // add Sgm T cameras to Device cache
            for (const int tc : tile.sgmTCams)
            {
                deviceCache.addMipmapImage(tc, minMipmapDownscale, maxMipmapDownscale, ic, _mp);
                deviceCache.addCameraParams(tc, _sgmParams.scale, _mp);
            }

            if (_depthMapParams.useRefine)
            {
                // add Refine R camera to Device cache
                deviceCache.addCameraParams(tile.rc, _refineParams.scale, _mp);

                // add Refine T cameras to Device cache
                for (const int tc : tile.refineTCams)
                {
                    deviceCache.addMipmapImage(tc, minMipmapDownscale, maxMipmapDownscale, ic, _mp);
                    deviceCache.addCameraParams(tc, _refineParams.scale, _mp);
                }
            }

            if (!hasRcWithoutDownscale)
            {
                // add SGM R camera at scale 1 to Device cache.
                // R camera parameters at scale 1 are required for SGM retrieve best depth
                deviceCache.addCameraParams(tile.rc, 1, _mp);
            }
        }

        // wait for camera loading in device cache
        cudaDeviceSynchronize();

        // compute each batch tile
        for (int i = firstTileIndex; i < lastTileIndex; ++i)
        {
            Tile& tile = tiles.at(i);
            const int batchCamIndex = tile.rc % nbRcPerBatch;
            const int streamIndex = tile.id % nbStreams;

            // do not compute empty ROI
            // some images in the dataset may be smaller than others
            if (tile.roi.isEmpty())
                continue;

            // get tile result depth/similarity map in host memory
            CudaHostMemoryHeap<float2, 2>& tileDepthSimMap_hmh = depthSimMapTilePerCam.at(batchCamIndex).at(tile.id);

            // check T cameras
            if (tile.sgmTCams.empty() || (_depthMapParams.useRefine && tile.refineTCams.empty()))  // no T camera found
            {
                resetDepthSimMap(tileDepthSimMap_hmh);
                continue;
            }

            // build tile SGM depth list
            SgmDepthList sgmDepthList(_mp, _sgmParams, tile);

            // compute the R camera depth list
            sgmDepthList.computeListRc();

            // check number of depths
            if (sgmDepthList.getDepths().empty())  // no depth found
            {
                resetDepthSimMap(tileDepthSimMap_hmh);
                depthMinMaxTilePerCam.at(batchCamIndex).at(tile.id) = {0.f, 0.f};
                continue;
            }

            // remove T cameras with no depth found.
            sgmDepthList.removeTcWithNoDepth(tile);

            // store min/max depth
            depthMinMaxTilePerCam.at(batchCamIndex).at(tile.id) = sgmDepthList.getMinMaxDepths();

            // log debug camera / depth information
            sgmDepthList.logRcTcDepthInformation();

            // check if starting and stopping depth are valid
            sgmDepthList.checkStartingAndStoppingDepth();

            // compute Semi-Global Matching
            Sgm& sgm = sgmPerStream.at(streamIndex);
            sgm.sgmRc(tile, sgmDepthList);

            if (_depthMapParams.useRefine)
            {
                // smooth SGM thickness map
                // in order to be a proper Refine input parameter
                sgm.smoothThicknessMap(tile, _refineParams);

                // compute Refine
                Refine& refine = refinePerStream.at(streamIndex);
                refine.refineRc(tile, sgm.getDeviceDepthThicknessMap(), sgm.getDeviceNormalMap());

                // copy Refine depth/similarity map from device to host
                tileDepthSimMap_hmh.copyFrom(refine.getDeviceDepthSimMap(), deviceStreamManager.getStream(streamIndex));
            }
            else
            {
                // copy Sgm depth/similarity map from device to host
                tileDepthSimMap_hmh.copyFrom(sgm.getDeviceDepthSimMap(), deviceStreamManager.getStream(streamIndex));
            }
        }

        // wait for tiles batch computation
        cudaDeviceSynchronize();

        // find first and last tile R camera
        const int firstRc = tiles.at(firstTileIndex).rc;
        int lastRc = tiles.at(lastTileIndex - 1).rc;

        // check if last tile depth map is finished
        if (lastTileIndex < tiles.size() && (tiles.at(lastTileIndex).rc == lastRc))
            --lastRc;

        // write depth/sim map result
        for (int c = firstRc; c <= lastRc; ++c)
        {
            const int batchCamIndex = c % nbRcPerBatch;

            if (_depthMapParams.useRefine)
                writeDepthSimMapFromTileList(
                  c, _mp, _tileParams, _tileRoiList, depthSimMapTilePerCam.at(batchCamIndex), _refineParams.scale, _refineParams.stepXY);
            else
                writeDepthSimMapFromTileList(
                  c, _mp, _tileParams, _tileRoiList, depthSimMapTilePerCam.at(batchCamIndex), _sgmParams.scale, _sgmParams.stepXY);

            if (_depthMapParams.exportTilePattern)
                exportDepthSimMapTilePatternObj(c, _mp, _tileRoiList, depthMinMaxTilePerCam.at(batchCamIndex));
        }
    }

    // merge intermediate results tiles if needed and desired
    if (tiles.size() > cams.size())
    {
        // merge tiles if needed and desired
        for (int rc : cams)
        {
            if (_sgmParams.exportIntermediateDepthSimMaps)
            {
                mergeDepthSimMapTiles(rc, _mp, _sgmParams.scale, _sgmParams.stepXY, "sgm");
            }

            if (_sgmParams.exportIntermediateNormalMaps)
            {
                mergeNormalMapTiles(rc, _mp, _sgmParams.scale, _sgmParams.stepXY, "sgm");
            }

            if (_depthMapParams.useRefine)
            {
                if (_refineParams.exportIntermediateDepthSimMaps)
                {
                    mergeDepthPixSizeMapTiles(rc, _mp, _refineParams.scale, _refineParams.stepXY, "sgmUpscaled");
                    mergeDepthSimMapTiles(rc, _mp, _refineParams.scale, _refineParams.stepXY, "refinedFused");
                }

                if (_refineParams.exportIntermediateNormalMaps)
                {
                    mergeNormalMapTiles(rc, _mp, _refineParams.scale, _refineParams.stepXY, "refinedFused");
                    mergeNormalMapTiles(rc, _mp, _refineParams.scale, _refineParams.stepXY);
                }
            }
        }
    }

    // some objects countains CUDA objects
    // this objects should be destroyed before the end of the program (i.e. the end of the CUDA context)
    DeviceCache::getInstance().clear();
    sgmPerStream.clear();
    refinePerStream.clear();
}

}  // namespace depthMap
}  // namespace aliceVision
