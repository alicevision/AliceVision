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
#include <aliceVision/depthMap_sycl/depthMapUtils.hpp>
#include <aliceVision/depthMap_sycl/DepthMapParams.hpp>
#include <aliceVision/depthMap_sycl/SgmDepthList.hpp>
#include <aliceVision/depthMap_sycl/Sgm.hpp>
#include <aliceVision/depthMap_sycl/Refine.hpp>
#include <aliceVision/depthMap_sycl/sycl/DeviceCache.hpp>
#include <aliceVision/depthMap_sycl/sycl/PatchPattern.hpp>

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
    _refineParams(refineParams),
    _ic(mp, image::EImageColorSpace::LINEAR) // share host cache between devices
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

void DepthMapEstimator::getTilesList(const std::vector<int>& cams, std::vector<Tile>& tiles)
{
    const int nbTilesPerCamera = _tileRoiList.size();

    // tiles list should be empty
    assert(tiles.empty());

    // reserve memory
    tiles.reserve(cams.size() * nbTilesPerCamera);

    for (int rc : cams)
    {
        // "warm up" cpu cache
        _ic.refreshImage_async(rc);
        // get R camera Tcs list
        const std::vector<int> tCams = _mp.findNearestCamsFromLandmarks(rc, _depthMapParams.maxTCams).getDataWritable();
        _ic.refreshImages_async(tCams);

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

void DepthMapEstimator::compute(sycl::queue& queue, const std::vector<int>& cams)
{
    // Used for debug messages
    const auto deviceName = queue.get_device().get_info<sycl::info::device::name>();

    // build tile list order by R camera
    ALICEVISION_LOG_DEBUG(deviceName << ": building tile metadata in memory");
    std::vector<Tile> tiles;
    getTilesList(cams, tiles);
    ALICEVISION_LOG_DEBUG(deviceName << ": need to process " << tiles.size() << " tiles");

    // constants
    //const bool hasRcSameDownscale = (_sgmParams.scale == _refineParams.scale);  // we only need one camera params per image
    //const bool hasRcWithoutDownscale =
    //  _sgmParams.scale == 1 || (_depthMapParams.useRefine && _refineParams.scale == 1);  // we need R camera params SGM (downscale = 1)
    //const int nbCameraParamsPerSgm =
    //  (1 + _depthMapParams.maxTCams) + (hasRcWithoutDownscale ? 0 : 1);  // number of Sgm camera parameters per R camera
    //const int nbCameraParamsPerRefine =
    //  (_depthMapParams.useRefine && !hasRcSameDownscale) ? (1 + _depthMapParams.maxTCams) : 0;  // number of Refine camera parameters per R camera
    const int nbTilesPerCamera = static_cast<int>(_tileRoiList.size());

    const int nbTiles = tiles.size();                           // number of tiles
    const int firstTileRc = tiles.at(0).rc;
    const int nbRc = divideRoundUp(nbTiles, nbTilesPerCamera);  // number of R cameras in the same batch

    //const int nbCamerasParams = nbRc * (nbCameraParamsPerSgm + nbCameraParamsPerRefine);  // number of camera parameters
    //const int nbMipmapImages = nbRc * (1 + _depthMapParams.maxTCams);                     // number of camera mipmap image in the same batch
    const int minMipmapDownscale = std::min(_refineParams.scale, _sgmParams.scale);
    const int maxMipmapDownscale = std::max(_refineParams.scale, _sgmParams.scale) * std::pow(2, 6);  // we add 6 downscale levels

    // Get device cache
    DeviceCache& deviceCache = DeviceCache::getInstance();

    // Build custom patch pattern in global instance
    if (_sgmParams.useCustomPatchPattern || _refineParams.useCustomPatchPattern)
        buildCustomPatchPattern(_depthMapParams.customPatchPattern);

    // porting note: we are replacing CUDA constant memory with SYCL specialization constants

    // containers to keep track of sgm and refine objects in device memory
    std::unordered_map<uint, Sgm> sgmPerTile;
    std::unordered_map<uint, Refine> refinePerTile;
	std::vector<uint> freeCompObj;
    sgmPerTile.reserve(nbTiles);
    if (_depthMapParams.useRefine) refinePerTile.reserve(nbTiles);
    freeCompObj.reserve(nbTiles);

    // allocate final deth/similarity map tile list in host memory
    ALICEVISION_LOG_DEBUG(deviceName <<": Allocating final deth/similarity map tile list in host memory");
    std::vector<std::vector<SyclHostMemoryHeap<sycl::float2, 2>>> depthSimMapTilePerCam(nbRc);
    std::vector<std::vector<std::pair<float, float>>> depthMinMaxTilePerCam(nbRc);

    for (int i = 0; i < nbRc; ++i)
    {
        auto& depthSimMapTiles = depthSimMapTilePerCam.at(i);
        auto& depthMinMaxTiles = depthMinMaxTilePerCam.at(i);

        depthSimMapTiles.reserve(nbTilesPerCamera);
        depthMinMaxTiles.resize(nbTilesPerCamera);

        for (int j = 0; j < nbTilesPerCamera; ++j)
        {
            auto& ref = depthSimMapTiles.emplace_back(queue);
            if (_depthMapParams.useRefine)
                ref.allocate(SyclSize<2>(divideRoundUp(_tileParams.bufferWidth, _refineParams.scale * _refineParams.stepXY), divideRoundUp(_tileParams.bufferHeight, _refineParams.scale * _refineParams.stepXY)));
            else  // final depth/similarity map is SGM only
                ref.allocate(SyclSize<2>(divideRoundUp(_tileParams.bufferWidth, _sgmParams.scale * _sgmParams.stepXY), divideRoundUp(_tileParams.bufferHeight, _sgmParams.scale * _sgmParams.stepXY)));
        }
    }

    // events for waiting on individual tiles or entire images
    std::vector<std::vector<sycl::event>> tileStatuses(nbRc);
    for (int i = 0; i < nbRc; ++i)
    {
        tileStatuses.at(i).resize(nbTilesPerCamera);
    }

    // progress counters
    uint tilesDispatched = 0;
    uint tilesFinished = 0;

    // switch between dispatch and collection until we finish
    ALICEVISION_LOG_DEBUG(deviceName << ": begining dispatch-execute-writeback loop (i.e. main computation)");
    while (tilesFinished < nbTiles)
    {
        ALICEVISION_LOG_TRACE(deviceName <<": have dispatched " << tilesDispatched << " tiles, of which finished " << tilesFinished);
        // dispatch as many tiles as possible with current memory
        while (tilesDispatched < nbTiles)
        {
            ALICEVISION_LOG_TRACE(deviceName << ": trying to dispatch tile " << tilesDispatched << " out of " << tiles.size() - 1);
            Tile& tile = tiles.at(tilesDispatched);
            const int tileIndex = tile.rc - firstTileRc;
            ALICEVISION_LOG_TRACE(deviceName << ": tile has index " << tileIndex << ", rc " << tile.rc << " and id " << tile.id);

            // event for dependency ordering
            sycl::event tileProcessed{};

            // do not compute empty ROI
            // some images in the dataset may be smaller than others
            if (tile.roi.isEmpty())
            {
                ALICEVISION_LOG_TRACE(deviceName << ": skipping empty tile");
                tilesDispatched++;
                continue;
            }

            // get tile result depth/similarity map in host memory
            SyclHostMemoryHeap<sycl::float2, 2>& tileDepthSimMap_hmh = depthSimMapTilePerCam.at(tileIndex).at(tile.id);

            // check T cameras
            if (tile.sgmTCams.empty() || (_depthMapParams.useRefine && tile.refineTCams.empty()))  // no T camera found
            {
                ALICEVISION_LOG_TRACE(deviceName << ": skipping tile with no T camera");
                resetDepthSimMap(tileDepthSimMap_hmh);
                tilesDispatched++;
                continue;
            }

            // track if we run out of memory
            bool success = true;

            // add Sgm R camera to Device cache
            deviceCache.addMipmapImage(tile.rc, minMipmapDownscale, maxMipmapDownscale, _ic, _mp, success, queue);

            // add Sgm T cameras to Device cache
            for (const int tc : tile.sgmTCams)
            {
                deviceCache.addMipmapImage(tc, minMipmapDownscale, maxMipmapDownscale, _ic, _mp, success, queue);
            }

            // Check if we need new objects
            auto sgmIt = sgmPerTile.begin();
            auto refineIt = refinePerTile.begin();
            uint objectIndex;
            bool createNewObjects = freeCompObj.empty();
            if (!createNewObjects)
            {
                objectIndex = freeCompObj.back();
                freeCompObj.pop_back();
            }

            // allocate sgm object if needed
            if (createNewObjects)
            {
                // need a new compute object
                sgmIt = sgmPerTile.emplace(std::piecewise_construct,
                                           std::forward_as_tuple(tilesDispatched),
                                           std::forward_as_tuple(_mp, _tileParams, _sgmParams, !_depthMapParams.useRefine, _refineParams.useSgmNormalMap, success, queue)).first;
            }
            else
            {
                auto node = sgmPerTile.extract(objectIndex);
                node.key() = tilesDispatched;
                sgmIt = sgmPerTile.insert(std::move(node)).position;
            }

            if (_depthMapParams.useRefine)
            {
                // add Refine T cameras to Device cache
                for (const int tc : tile.refineTCams)
                {
                    deviceCache.addMipmapImage(tc, minMipmapDownscale, maxMipmapDownscale, _ic, _mp, success, queue);
                }

                // allocate refine object
                if (createNewObjects)
                {
                    refineIt = refinePerTile.emplace(std::piecewise_construct,
                                                     std::forward_as_tuple(tilesDispatched),
                                                     std::forward_as_tuple(_mp, _tileParams, _refineParams, success, queue)).first;
                }
                else
                {
                    auto node = refinePerTile.extract(objectIndex);
                    node.key() = tilesDispatched;
                    refineIt = refinePerTile.insert(std::move(node)).position;
                }
            }

            if (!success)
            {
                if (tilesDispatched == 0) ALICEVISION_THROW_ERROR(deviceName << ": Not enough device memory to compute a single tile!");
                ALICEVISION_LOG_TRACE(deviceName << ": Pausing allocation at tile " << tilesDispatched << " due to memory limits.");
                // Delete stale compute objects
                sgmPerTile.erase(tilesDispatched);
                if(_depthMapParams.useRefine) refinePerTile.erase(tilesDispatched);
                break;
            }

            ALICEVISION_LOG_TRACE(deviceName << ": begining computation of tile " << tilesDispatched << " after having finished allocation");

            // build tile SGM depth list
            SgmDepthList sgmDepthList(_mp, _sgmParams, tile);

            // compute the R camera depth list
            sgmDepthList.computeListRc();

            // check number of depths
            if (sgmDepthList.getDepths().empty())  // no depth found
            {
                resetDepthSimMap(tileDepthSimMap_hmh);
                depthMinMaxTilePerCam.at(tileIndex).at(tile.id) = {0.f, 0.f};
                tilesDispatched++;
                continue;
            }

            // remove T cameras with no depth found.
            sgmDepthList.removeTcWithNoDepth(tile);

            // store min/max depth
            depthMinMaxTilePerCam.at(tileIndex).at(tile.id) = sgmDepthList.getMinMaxDepths();

            // log debug camera / depth information
            sgmDepthList.logRcTcDepthInformation();

            // check if starting and stopping depth are valid
            sgmDepthList.checkStartingAndStoppingDepth();

            // compute Semi-Global Matching
            Sgm& sgm = sgmIt->second;
            ALICEVISION_LOG_TRACE(deviceName << ": begining semi-global matching of tile " << tilesDispatched);
            sycl::event rcComputed = sgm.sgmRc(tile, sgmDepthList, sycl::event());

            if (_depthMapParams.useRefine)
            {
                // smooth SGM thickness map
                // in order to be a proper Refine input parameter
                sycl::event smoothComputed = sgm.smoothThicknessMap(tile, _refineParams, rcComputed);

                // compute Refine
                Refine& refine = refineIt->second;
                sycl::event refineComputed = refine.refineRc(tile, sgm.getDeviceDepthThicknessMap(), sgm.getDeviceNormalMap(), smoothComputed);

                // copy Refine depth/similarity map from device to host
                sycl::event writeback = tileDepthSimMap_hmh.copyFrom(refine.getDeviceDepthSimMap(), refineComputed);

                // set appropriate event for waiting on the tile
                tileStatuses.at(tileIndex).at(tile.id) = writeback;
            }
            else
            {
                // copy Sgm depth/similarity map from device to host
                sycl::event writeback = tileDepthSimMap_hmh.copyFrom(sgm.getDeviceDepthSimMap(), rcComputed);

                // set appropriate event for waiting on the tile
                tileStatuses.at(tileIndex).at(tile.id) = writeback;
            }

            tilesDispatched++;

            const Tile& running = tiles.at(tilesFinished);
            if(tileStatuses.at(running.rc - firstTileRc).at(running.id).get_info<sycl::info::event::command_execution_status>() == sycl::info::event_command_status::complete) // If we have work that is finished, free that memory
                break;
        }

        ALICEVISION_LOG_TRACE(deviceName << ": begining readback of tile " << tilesFinished);
        // read back a tile, free memory where possible
        const Tile& readback = tiles.at(tilesFinished);
        const int readbackIndex = readback.rc - firstTileRc;

        // wait on computation
        tileStatuses.at(readbackIndex).at(readback.id).wait();

        if (sgmPerTile.contains(tilesFinished)) // Don't liberate objects that don't exist
        {
            if(freeCompObj.size() < nbTiles - tilesFinished)
            {
                // add tile intex to vector of compute objects that are free to use
                freeCompObj.emplace_back(tilesFinished);
            }
            else
            {
                // free objects
                sgmPerTile.erase(tilesFinished);
                if(_depthMapParams.useRefine) refinePerTile.erase(tilesFinished);
            }
        }

        ALICEVISION_LOG_TRACE(deviceName << ": freeing images for tile " << tilesFinished);

        // free Sgm R camera
        deviceCache.freeMipmapImage(readback.rc, queue);

        // free Sgm T cameras to Device cache
        for (const int tc : readback.sgmTCams)
        {
            deviceCache.freeMipmapImage(tc, queue);
        }

        if (_depthMapParams.useRefine)
        {
            // add Refine T cameras to Device cache
            for (const int tc : readback.refineTCams)
            {
                deviceCache.freeMipmapImage(tc, queue);
            }
        }

        ALICEVISION_LOG_TRACE(deviceName << ": tile " << tilesFinished << " finished");

        tilesFinished++;
    }

    // find first and last tile R camera
    const int firstRc = tiles.at(0).rc;
    int lastRc = tiles.at(tiles.size() - 1).rc;

    // write depth/sim map result
#pragma omp parallel for
    for (int c = firstRc; c <= lastRc; ++c)
    {
        const int batchCamIndex = c % nbRc;

        if (_depthMapParams.useRefine)
            writeDepthSimMapFromTileList(
              c, _mp, _tileParams, _tileRoiList, depthSimMapTilePerCam.at(batchCamIndex), _refineParams.scale, _refineParams.stepXY);
        else
            writeDepthSimMapFromTileList(
              c, _mp, _tileParams, _tileRoiList, depthSimMapTilePerCam.at(batchCamIndex), _sgmParams.scale, _sgmParams.stepXY);

        if (_depthMapParams.exportTilePattern)
            exportDepthSimMapTilePatternObj(c, _mp, _tileRoiList, depthMinMaxTilePerCam.at(batchCamIndex));
    }

    // merge intermediate results tiles if needed and desired
    if (tiles.size() > cams.size())
    {
        ALICEVISION_LOG_DEBUG(deviceName << ": merging tiles into images");
        // merge tiles if needed and desired
#pragma omp parallel for
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

    // some objects contains SYCL objects, which should be freed to avoid memory leaks
    ALICEVISION_LOG_DEBUG(deviceName << ": cleaning up");
    DeviceCache::getInstance().clear(queue);
}

}  // namespace depthMap
}  // namespace aliceVision
