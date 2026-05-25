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
#include <aliceVision/depthMapCommon/DepthMapParams.hpp>
#include <aliceVision/depthMap_sycl/SgmDepthList.hpp>
#include <aliceVision/depthMap_sycl/sycl/PatchPattern.hpp>

#define ALICEVISION_DEPTHMAP_MAX_ALLOC_RETRIES 8

namespace aliceVision {
namespace depthMap_sycl {

DepthMapEstimator::DepthMapEstimator(const mvsUtils::MultiViewParams& mp,
                                     const mvsUtils::TileParams& tileParams,
                                     const depthMapCommon::DepthMapParams& depthMapParams,
                                     const depthMapCommon::SgmParams& sgmParams,
                                     const depthMapCommon::RefineParams& refineParams)
  : _mp(mp),
    _tileParams(tileParams),
    _depthMapParams(depthMapParams),
    _sgmParams(sgmParams),
    _refineParams(refineParams),
    _ic(mp, image::EImageColorSpace::LINEAR), // share host cache between devices
    _deviceCache() // construct device cache instance
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

DepthMapEstimator::ComputeObject& DepthMapEstimator::ComputeObjectBuffer::getComputeObject(
                                const mvsUtils::MultiViewParams& mp,
                                const mvsUtils::TileParams& tileParams,
                                const depthMapCommon::SgmParams& sgmParams,
                                const depthMapCommon::RefineParams& refineParams,
                                const bool constructRefine,
                                const bool computeDepthSimMap,
                                const bool computeNormalMap,
                                const sycl::device& device)
{
    // keep track of object with least users, in case (due to memory pressure) we need to return an object already in use
    ComputeObject* leastUserObj = nullptr;

    // try to use existing objects
    containerLock.lock_shared();
    for(auto& obj : objects)
    {
        if(obj.lock.try_lock()) { containerLock.unlock_shared(); obj.users++; return obj; }
        else if(leastUserObj == nullptr || obj.users < leastUserObj->users) leastUserObj = &obj;
    }
    containerLock.unlock_shared();

    // try to construct new object
    {
        std::unique_lock lkg(containerLock);
        ComputeObject* returnObj = nullptr;
        bool allocationSuccess = true;

        sycl::queue queue = constructQueue(device);

        if(constructRefine) returnObj = &objects.emplace_back(
            queue,
            std::forward_as_tuple(mp, tileParams, sgmParams, computeDepthSimMap, computeNormalMap, allocationSuccess, queue),
            std::forward_as_tuple(std::in_place, mp, tileParams, refineParams, allocationSuccess, queue)
        );
        else returnObj = &objects.emplace_back(
            queue,
            std::forward_as_tuple(mp, tileParams, sgmParams, computeDepthSimMap, computeNormalMap, allocationSuccess, queue),
            std::forward_as_tuple(std::nullopt)
        );

        if(allocationSuccess) { returnObj->users++; returnObj->lock.lock(); return *returnObj; }
        else
        {
            // delete the stale object
            objects.pop_back();
            if(objects.size() == 0)
            {
                // failure state: can't allocate a single set of compute objects
                const auto deviceName = queue.get_device().get_info<sycl::info::device::name>();
                ALICEVISION_THROW_ERROR(deviceName << ": Not enough memory to compute a single tile!");
            }
        }
    }

    // use existing object
    leastUserObj->users++;
    leastUserObj->lock.lock();
    return *leastUserObj;
}

void DepthMapEstimator::getTiles(const int cam, std::vector<Tile>& tiles)
{
    const int nbTilesPerCamera = _tileRoiList.size();
    assert(tiles.size() == nbTilesPerCamera);

    // get R camera Tcs list
    const std::vector<int> tCams = _mp.findNearestCamsFromLandmarks(cam, _depthMapParams.maxTCams).getDataWritable();

    // get R camera ROI
    const ROI rcImageRoi(Range(0, _mp.getWidth(cam)), Range(0, _mp.getHeight(cam)));

    for (std::size_t i = 0; i < nbTilesPerCamera; ++i)
    {
        Tile& t = tiles.at(i);

        t.id = i;
        t.nbTiles = nbTilesPerCamera;
        t.rc = cam;
        t.roi = intersect(_tileRoiList.at(i), rcImageRoi);

        if (t.roi.isEmpty())
        {
            // do nothing, this ROI cannot intersect the R camera ROI.
        }
        else if (_depthMapParams.chooseTCamsPerTile)
        {
            // find nearest T cameras per tile
            t.sgmTCams = _mp.findTileNearestCams(cam, _sgmParams.maxTCamsPerTile, tCams, t.roi);

            if (_depthMapParams.useRefine)
                t.refineTCams = _mp.findTileNearestCams(cam, _refineParams.maxTCamsPerTile, tCams, t.roi);
        }
        else
        {
            // use previously selected T cameras from the entire image
            t.sgmTCams = tCams;
            t.refineTCams = tCams;
        }
    }
}

void DepthMapEstimator::compute(const sycl::device& device, const std::vector<int>& cams)
{
    // Used for debug messages
    const auto deviceName = getDeviceName(device);

    // constants
    //const bool hasRcSameDownscale = (_sgmParams.scale == _refineParams.scale);  // we only need one camera params per image
    //const bool hasRcWithoutDownscale =
    //  _sgmParams.scale == 1 || (_depthMapParams.useRefine && _refineParams.scale == 1);  // we need R camera params SGM (downscale = 1)
    //const int nbCameraParamsPerSgm =
    //  (1 + _depthMapParams.maxTCams) + (hasRcWithoutDownscale ? 0 : 1);  // number of Sgm camera parameters per R camera
    //const int nbCameraParamsPerRefine =
    //  (_depthMapParams.useRefine && !hasRcSameDownscale) ? (1 + _depthMapParams.maxTCams) : 0;  // number of Refine camera parameters per R camera
    const int nbTiles = _tileRoiList.size();

    //const int nbCamerasParams = nbRc * (nbCameraParamsPerSgm + nbCameraParamsPerRefine);  // number of camera parameters
    //const int nbMipmapImages = nbRc * (1 + _depthMapParams.maxTCams);                     // number of camera mipmap image in the same batch
    const int minMipmapDownscale = std::min(_refineParams.scale, _sgmParams.scale);
    const int maxMipmapDownscale = std::max(_refineParams.scale, _sgmParams.scale) * std::pow(2, 6);  // we add 6 downscale levels

    // Build custom patch pattern in global instance
    if (_sgmParams.useCustomPatchPattern || _refineParams.useCustomPatchPattern)
        buildCustomPatchPattern(_depthMapParams.customPatchPattern);

    // porting note: we are replacing CUDA constant memory with SYCL specialization constants

    // Sgm and refine objects for computation
    ComputeObjectBuffer& objBuffer = _objectBuffers.getOrConstruct(device);

    // allocate final deth/similarity map tile list in host memory
    ALICEVISION_LOG_DEBUG(deviceName <<": Allocating final depth/similarity map tile list in host memory");
    std::vector<SyclHostMemoryHeap<sycl::float2, 2>> depthSimMapPerTile{};
    std::vector<std::pair<float, float>> depthMinMaxPerTile(nbTiles);

    {
        // dummy queue for allocation
        sycl::queue queue = constructQueue(device);

        depthSimMapPerTile.reserve(nbTiles);
        for (int j = 0; j < nbTiles; ++j)
        {
            auto& ref = depthSimMapPerTile.emplace_back(queue);
            if (_depthMapParams.useRefine)
                ref.allocate(SyclSize<2>(divideRoundUp(_tileParams.bufferWidth, _refineParams.scale * _refineParams.stepXY), divideRoundUp(_tileParams.bufferHeight, _refineParams.scale * _refineParams.stepXY)));
            else  // final depth/similarity map is SGM only
                ref.allocate(SyclSize<2>(divideRoundUp(_tileParams.bufferWidth, _sgmParams.scale * _sgmParams.stepXY), divideRoundUp(_tileParams.bufferHeight, _sgmParams.scale * _sgmParams.stepXY)));
        }
    }

    // locks to protect above memory from being overwritten until it is flushed
    std::vector<std::mutex> bufferPerTileLocks(nbTiles);

    // allocate and compute tile metadata list
    std::vector<std::vector<Tile>> tilesPerCamera(cams.size());
    for (int i = 0; i < cams.size(); i++)
    {
        const int cam = cams.at(i);
        std::vector<Tile>& tiles = tilesPerCamera.at(i);
        ALICEVISION_LOG_TRACE(deviceName << ": constructing tile information for camera id " << cam);
        tiles.resize(nbTiles);
        getTiles(cam, tiles);
    }

    // keep track of state of last camera
    std::optional<std::future<void>> asyncObject;

    // keep trak of state of tiles
    std::vector<std::shared_future<void>> tileStatuses(nbTiles);

    // dump for futures (their destructor blocks for task to finish)
    std::list<std::future<void>> futures;

    // iterate through cameras
    ALICEVISION_LOG_DEBUG(deviceName << ": begining computation");
    for(int i = 0; i < cams.size(); i++)
    {
        const int& cam = cams.at(i);
        std::vector<Tile>& tiles = tilesPerCamera.at(i);

        ALICEVISION_LOG_TRACE(deviceName << ": computing camera id " << cam);
        // dispatch as many tiles as possible with current memory
        for(Tile& tile : tiles) tileStatuses.at(tile.id) = std::async(std::launch::async, [&, tileStatuses] ()
        {
            if(tileStatuses.at(tile.id).valid()) tileStatuses.at(tile.id).wait(); // finish tile of previous camera before continueing
            ALICEVISION_LOG_TRACE(deviceName << ": computing tile of camera id " << tile.rc << " with tile id " << tile.id);

            // do not compute empty ROI
            // some images in the dataset may be smaller than others
            if (tile.roi.isEmpty())
            {
                ALICEVISION_LOG_TRACE(deviceName << ": skipping empty tile");
                return;
            }

            // get tile result depth/similarity map in host memory
            SyclHostMemoryHeap<sycl::float2, 2>& tileDepthSimMap_hmh = depthSimMapPerTile.at(tile.id);

            // check T cameras
            if (tile.sgmTCams.empty() || (_depthMapParams.useRefine && tile.refineTCams.empty()))  // no T camera found
            {
                ALICEVISION_LOG_TRACE(deviceName << ": skipping tile with no T camera");
                bufferPerTileLocks.at(tile.id).lock(); // make sure we have flushed memory
                resetDepthSimMap(tileDepthSimMap_hmh);
                return;
            }

            // get compute objects for computing this tile
            ComputeObject& compObj = objBuffer.getComputeObject(_mp, _tileParams, _sgmParams, _refineParams, _depthMapParams.useRefine, !_depthMapParams.useRefine, _refineParams.useSgmNormalMap, device);
            sycl::queue& queue = compObj.queue;
            Sgm& sgm = compObj.sgm;
            std::optional<Refine>& refineOpt = compObj.refineOpt;

            // Track how many times we try to allocate images
            int retryAlloc = ALICEVISION_DEPTHMAP_MAX_ALLOC_RETRIES;
            // add Sgm R camera to Device cache
            while(retryAlloc > 0)
            {
                // track if we run out of memory
                bool success = true;
                _deviceCache.addMipmapImage(tile.rc, minMipmapDownscale, maxMipmapDownscale, _ic, _mp, success, queue);

                // add Sgm T cameras to Device cache
                for (const int tc : tile.sgmTCams)
                {
                    _deviceCache.addMipmapImage(tc, minMipmapDownscale, maxMipmapDownscale, _ic, _mp, success, queue);
                }

                if (_depthMapParams.useRefine)
                {
                    // add Refine T cameras to Device cache
                    for (const int tc : tile.refineTCams)
                    {
                        _deviceCache.addMipmapImage(tc, minMipmapDownscale, maxMipmapDownscale, _ic, _mp, success, queue);
                    }
                }

                if(!success)
                {
                    ALICEVISION_LOG_DEBUG(deviceName << ": very low on memory. Trying to reclaim");
                    // wait on a previous tile to finish computation
                    tileStatuses.at((tile.id - retryAlloc) % nbTiles).wait();

                    retryAlloc--;
                    if (retryAlloc <= 0) ALICEVISION_THROW_ERROR(deviceName << ": Not enough device memory to hold all camera views for a single tile");
                }
                else break;
            }

            ALICEVISION_LOG_TRACE(deviceName << ": begining computation of tile " << tile.id << " after having finished allocation");

            // build tile SGM depth list
            SgmDepthList sgmDepthList(_mp, _sgmParams, tile);

            // compute the R camera depth list
            sgmDepthList.computeListRc();

            // check number of depths
            if (sgmDepthList.getDepths().empty())  // no depth found
            {
                bufferPerTileLocks.at(tile.id).lock(); // make sure we have flushed memory
                resetDepthSimMap(tileDepthSimMap_hmh);
                depthMinMaxPerTile.at(tile.id) = {0.f, 0.f};
                return;
            }

            // remove T cameras with no depth found.
            sgmDepthList.removeTcWithNoDepth(tile);

            // store min/max depth
            depthMinMaxPerTile.at(tile.id) = sgmDepthList.getMinMaxDepths();

            // log debug camera / depth information
            sgmDepthList.logRcTcDepthInformation();

            // check if starting and stopping depth are valid
            sgmDepthList.checkStartingAndStoppingDepth();

            // compute Semi-Global Matching
            sgm.sgmRc(tile, sgmDepthList, _deviceCache, sycl::event());

            if (_depthMapParams.useRefine)
            {
                // smooth SGM thickness map
                // in order to be a proper Refine input parameter
                sgm.smoothThicknessMap(tile, _refineParams, sycl::event());

                // compute Refine
                Refine& refine = refineOpt.value();
                refine.refineRc(tile, sgm.getDeviceDepthThicknessMap(), sgm.getDeviceNormalMap(), _deviceCache, sycl::event());

                // copy Refine depth/similarity map from device to host
                bufferPerTileLocks.at(tile.id).lock(); // make sure we have flushed memory
                tileDepthSimMap_hmh.copyFrom(refine.getDeviceDepthSimMap(), queue, sycl::event());
            }
            else
            {
                // copy Sgm depth/similarity map from device to host
                bufferPerTileLocks.at(tile.id).lock(); // make sure we have flushed memory
                tileDepthSimMap_hmh.copyFrom(sgm.getDeviceDepthSimMap(), queue, sycl::event());
            }

            ALICEVISION_LOG_TRACE(deviceName << ": freeing compute objects for tile " << tile.id);
            compObj.release(); // let another tile be computed with this set of objects

            // wait on computation before exiting
            ALICEVISION_LOG_TRACE(deviceName << ": waiting for computation of tile " << tile.id);
            queue.wait();

            // free up cache when finished
            futures.emplace_back(std::async(std::launch::async, [&] ()
            {
                // free Sgm R camera
                _deviceCache.freeMipmapImage(tile.rc, device);

                ALICEVISION_LOG_TRACE(deviceName << ": freeing images for tile " << tile.id);

                // free Sgm T cameras to Device cache
                for (const int tc : tile.sgmTCams)
                {
                    _deviceCache.freeMipmapImage(tc, device);
                }

                if (_depthMapParams.useRefine)
                {
                    // add Refine T cameras to Device cache
                    for (const int tc : tile.refineTCams)
                    {
                        _deviceCache.freeMipmapImage(tc, device);
                    }
                }
            }));
        });

        // only have at most two cameras computing at the same time
        if (asyncObject.has_value())
        {
            ALICEVISION_LOG_TRACE(deviceName << ": holding for completion of previous camera");
            if(asyncObject.value().valid()) asyncObject.value().get();
        }

        // asynchronously write images to disk
        asyncObject.emplace(std::async(std::launch::async, [&, tileStatuses] ()
        {
            if (_depthMapParams.useRefine)
                writeDepthSimMapFromTileList(cam, _mp, _tileParams, _tileRoiList,
                                             depthSimMapPerTile, bufferPerTileLocks, tileStatuses,
                                             _refineParams.scale, _refineParams.stepXY);
            else
                writeDepthSimMapFromTileList(cam, _mp, _tileParams, _tileRoiList,
                                             depthSimMapPerTile, bufferPerTileLocks, tileStatuses,
                                             _sgmParams.scale, _sgmParams.stepXY);

            if (_depthMapParams.exportTilePattern)
                exportDepthSimMapTilePatternObj(cam, _mp, _tileRoiList, depthMinMaxPerTile);
        }));
    }

    // wait for computation to finish
    if(asyncObject.value().valid()) asyncObject.value().get();
    ALICEVISION_LOG_TRACE(deviceName << ": finished waiting on computation");

    // merge intermediate results tiles if needed and desired
    if (nbTiles > 1 &&
        (_sgmParams.exportIntermediateDepthSimMaps || _sgmParams.exportIntermediateNormalMaps ||
         (_depthMapParams.useRefine &&
          (_refineParams.exportIntermediateDepthSimMaps || _refineParams.exportIntermediateNormalMaps)
          )))
    {
        ALICEVISION_LOG_DEBUG(deviceName << ": merging tiles into images");
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
}

}  // namespace depthMap_sycl
}  // namespace aliceVision
