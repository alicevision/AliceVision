// This file is part of the AliceVision project.
// Copyright (c) 2021 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "depthMap.hpp"

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/mvsUtils/fileIO.hpp>
#include <aliceVision/mvsUtils/depthSimMapIO.hpp>
#include <aliceVision/mvsUtils/MultiViewParams.hpp>
#include <aliceVision/depthMap/depthMapUtils.hpp>
#include <aliceVision/depthMap/DepthMapParams.hpp>
#include <aliceVision/depthMap/SgmDepthList.hpp>
#include <aliceVision/depthMap/Sgm.hpp>
#include <aliceVision/depthMap/Refine.hpp>
#include <aliceVision/depthMap/cuda/host/utils.hpp>
#include <aliceVision/depthMap/cuda/host/DeviceCache.hpp>
#include <aliceVision/depthMap/cuda/host/DeviceStreamManager.hpp>
#include <aliceVision/depthMap/cuda/normalMapping/DeviceNormalMapper.hpp>
#include <aliceVision/depthMap/cuda/normalMapping/deviceNormalMap.hpp>

#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

namespace aliceVision {
namespace depthMap {

int computeDownscale(const mvsUtils::MultiViewParams& mp, int scale, int maxWidth, int maxHeight)
{
    const int maxImageWidth = mp.getMaxImageWidth() / scale;
    const int maxImageHeight = mp.getMaxImageHeight() / scale;

    int downscale = 1;
    int downscaleWidth = mp.getMaxImageWidth() / scale;
    int downscaleHeight = mp.getMaxImageHeight() / scale;

    while((downscaleWidth > maxWidth) || (downscaleHeight > maxHeight))
    {
        downscale++;
        downscaleWidth = maxImageWidth / downscale;
        downscaleHeight = maxImageHeight / downscale;
    }

    return downscale;
}

bool computeScaleStepSgmParams(const mvsUtils::MultiViewParams& mp, SgmParams& sgmParams)
{
    if(sgmParams.scale != -1 && sgmParams.stepXY != -1)
      return false;

    const int fileScale = 1; // input images scale (should be one)
    const int maxSideXY = 700 / mp.getProcessDownscale(); // max side in order to fit in device memory
    const int maxImageW = mp.getMaxImageWidth();
    const int maxImageH = mp.getMaxImageHeight();

    int maxW = maxSideXY;
    int maxH = maxSideXY * 0.8;

    if(maxImageW < maxImageH)
        std::swap(maxW, maxH);

    if(sgmParams.scale == -1)
    {
        // compute the number of scales that will be used in the plane sweeping.
        // the highest scale should have a resolution close to 700x550 (or less).
        const int scaleTmp = computeDownscale(mp, fileScale, maxW, maxH);
        sgmParams.scale = std::min(2, scaleTmp);
    }

    if(sgmParams.stepXY == -1)
    {
        sgmParams.stepXY = computeDownscale(mp, fileScale * sgmParams.scale, maxW, maxH);
    }

    return true;
}

void updateDepthMapParamsForSingleTileComputation(const mvsUtils::MultiViewParams& mp, bool autoSgmScaleStep, DepthMapParams& depthMapParams)
{
    if(!depthMapParams.autoAdjustSmallImage)
    {
      // cannot adjust depth map parameters
      return;
    }

    // update SGM maxTCamsPerTile
    if(depthMapParams.sgmParams.maxTCamsPerTile < depthMapParams.maxTCams)
    {
      ALICEVISION_LOG_WARNING("Single tile computation, override SGM maximum number of T cameras per tile (before: "
                              << depthMapParams.sgmParams.maxTCamsPerTile << ", now: " << depthMapParams.maxTCams << ").");
      depthMapParams.sgmParams.maxTCamsPerTile = depthMapParams.maxTCams;
    }

    // update Refine maxTCamsPerTile
    if(depthMapParams.refineParams.maxTCamsPerTile < depthMapParams.maxTCams)
    {
      ALICEVISION_LOG_WARNING("Single tile computation, override Refine maximum number of T cameras per tile (before: "
                              << depthMapParams.refineParams.maxTCamsPerTile << ", now: " << depthMapParams.maxTCams << ").");
      depthMapParams.refineParams.maxTCamsPerTile = depthMapParams.maxTCams;
    }

    const int maxSgmBufferWidth  = divideRoundUp(mp.getMaxImageWidth() , depthMapParams.sgmParams.scale * depthMapParams.sgmParams.stepXY);
    const int maxSgmBufferHeight = divideRoundUp(mp.getMaxImageHeight(), depthMapParams.sgmParams.scale * depthMapParams.sgmParams.stepXY);

    // update SGM step XY
    if(!autoSgmScaleStep && // user define SGM scale & stepXY
       (depthMapParams.sgmParams.stepXY == 2) && // default stepXY
       (maxSgmBufferWidth  < depthMapParams.tileParams.bufferWidth  * 0.5) &&
       (maxSgmBufferHeight < depthMapParams.tileParams.bufferHeight * 0.5))
    {
      ALICEVISION_LOG_WARNING("Single tile computation, override SGM step XY (before: " << depthMapParams.sgmParams.stepXY  << ", now: 1).");
      depthMapParams.sgmParams.stepXY = 1;
    }
}

int getNbStreams(const mvsUtils::MultiViewParams& mp, const DepthMapParams& depthMapParams, int nbTilesPerCamera)
{
    const int maxImageSize = mp.getMaxImageWidth() * mp.getMaxImageHeight(); // process downscale apply

    const double sgmFrameCostMB = ((maxImageSize / depthMapParams.sgmParams.scale) * sizeof(CudaRGBA)) / (1024.0 * 1024.0); // SGM RGBA
    const double refineFrameCostMB = ((maxImageSize / depthMapParams.refineParams.scale) * sizeof(CudaRGBA)) / (1024.0 * 1024.0); // Refine RGBA
    const double cameraFrameCostMB = sgmFrameCostMB + (depthMapParams.useRefine ? refineFrameCostMB : 0.0); // SGM + Refine single frame cost

    double sgmTileCostMB = 0.0;
    double sgmTileCostUnpaddedMB = 0.0;
    {
      Sgm sgm(mp, depthMapParams.tileParams, depthMapParams.sgmParams, 0 /*stream*/);
      sgmTileCostMB = sgm.getDeviceMemoryConsumption();
      sgmTileCostUnpaddedMB = sgm.getDeviceMemoryConsumptionUnpadded();
    }

    double refineTileCostMB = 0.0;
    double refineTileCostUnpaddedMB = 0.0;

    if(depthMapParams.useRefine)
    {
      Refine refine(mp, depthMapParams.tileParams, depthMapParams.refineParams, 0 /*stream*/);
      refineTileCostMB = refine.getDeviceMemoryConsumption();
      refineTileCostUnpaddedMB = refine.getDeviceMemoryConsumptionUnpadded();
    }

    const double tileCostMB = sgmTileCostMB + refineTileCostMB;
    const double tileCostUnpaddedMB = sgmTileCostUnpaddedMB + refineTileCostUnpaddedMB;

    const double rcCamsCost = cameraFrameCostMB + depthMapParams.maxTCams * cameraFrameCostMB;
    const double rcMinCostMB = rcCamsCost + tileCostMB;
    const double rcMaxCostMB = rcCamsCost + nbTilesPerCamera * tileCostMB;
    const int rcCamParams = (1 + depthMapParams.maxTCams) * 2; // number of camera parameters in device constant memory

    double deviceMemoryMB;
    {
        double availableMB, usedMB, totalMB;
        getDeviceMemoryInfo(availableMB, usedMB, totalMB);
        deviceMemoryMB = availableMB * 0.8; // available memory margin
    }

    int nbAllowedSimultaneousRc = int(deviceMemoryMB / rcMaxCostMB);
    int nbRemainingTiles = 0;

    {
        const double remainingMemoryMB = deviceMemoryMB - (nbAllowedSimultaneousRc * rcMaxCostMB);
        nbRemainingTiles = int(std::max(0.0, remainingMemoryMB - rcCamsCost) / tileCostMB);
    }

    // check that we do not need more constant camera parameters than the ones in device constant memory
    if(ALICEVISION_DEVICE_MAX_CONSTANT_CAMERA_PARAM_SETS < (nbAllowedSimultaneousRc * rcCamParams))
    {
      nbAllowedSimultaneousRc = int(ALICEVISION_DEVICE_MAX_CONSTANT_CAMERA_PARAM_SETS / rcCamParams);
      nbRemainingTiles = 0;
    }

    const int out_nbAllowedStreams = nbAllowedSimultaneousRc * nbTilesPerCamera + nbRemainingTiles;

    ALICEVISION_LOG_INFO("Device memory:" << std::endl
                         << "\t- available: " << deviceMemoryMB << " MB" << std::endl
                         << "\t- requirement for the first tile: " << rcMinCostMB << " MB" << std::endl
                         << "\t- # computation buffers per tile: " << tileCostMB << " MB" << " (Sgm: " << sgmTileCostMB << " MB" << ", Refine: " << refineTileCostMB << " MB)" << std::endl
                         << "\t- # input images (R + " << depthMapParams.maxTCams << " Ts): " << rcCamsCost << " MB (single multi-res image size: " << cameraFrameCostMB << " MB)");

    ALICEVISION_LOG_DEBUG( "Theoretical device memory cost for a tile without padding: " << tileCostUnpaddedMB << " MB" << " (Sgm: " << sgmTileCostUnpaddedMB << " MB" << ", Refine: " << refineTileCostUnpaddedMB << " MB)");

    ALICEVISION_LOG_INFO("Parallelization:" << std::endl
                         << "\t- # tiles per image: " << nbTilesPerCamera << std::endl
                         << "\t- # simultaneous depth maps computation: " << ((nbRemainingTiles < 1) ? nbAllowedSimultaneousRc : (nbAllowedSimultaneousRc + 1)) << std::endl
                         << "\t- # streams: " << out_nbAllowedStreams);

    if(out_nbAllowedStreams < 1 || rcCamParams > ALICEVISION_DEVICE_MAX_CONSTANT_CAMERA_PARAM_SETS)
        ALICEVISION_THROW_ERROR("Not enough GPU memory to compute a single tile.");

    return out_nbAllowedStreams;
}

void getDepthMapParams(const mvsUtils::MultiViewParams& mp, DepthMapParams& depthMapParams)
{
    // get tile user parameters from MultiViewParams property_tree

    auto& tileParams = depthMapParams.tileParams;
    tileParams.bufferWidth = mp.userParams.get<int>("tile.bufferWidth", tileParams.bufferWidth);
    tileParams.bufferHeight = mp.userParams.get<int>("tile.bufferHeight", tileParams.bufferHeight);
    tileParams.padding = mp.userParams.get<int>("tile.padding", tileParams.padding);

    // get SGM user parameters from MultiViewParams property_tree

    auto& sgmParams = depthMapParams.sgmParams;
    sgmParams.scale = mp.userParams.get<int>("sgm.scale", sgmParams.scale);
    sgmParams.stepXY = mp.userParams.get<int>("sgm.stepXY", sgmParams.stepXY);
    sgmParams.stepZ = mp.userParams.get<int>("sgm.stepZ", sgmParams.stepZ);
    sgmParams.wsh = mp.userParams.get<int>("sgm.wsh", sgmParams.wsh);
    sgmParams.maxDepths = mp.userParams.get<int>("sgm.maxDepths", sgmParams.maxDepths);
    sgmParams.maxTCamsPerTile = mp.userParams.get<int>("sgm.maxTCamsPerTile", sgmParams.maxTCamsPerTile);
    sgmParams.seedsRangeInflate = mp.userParams.get<double>("sgm.seedsRangeInflate", sgmParams.seedsRangeInflate);
    sgmParams.gammaC = mp.userParams.get<double>("sgm.gammaC", sgmParams.gammaC);
    sgmParams.gammaP = mp.userParams.get<double>("sgm.gammaP", sgmParams.gammaP);
    sgmParams.p1 = mp.userParams.get<double>("sgm.p1", sgmParams.p1);
    sgmParams.p2Weighting = mp.userParams.get<double>("sgm.p2Weighting", sgmParams.p2Weighting);
    sgmParams.filteringAxes = mp.userParams.get<std::string>("sgm.filteringAxes", sgmParams.filteringAxes);
    sgmParams.useSfmSeeds = mp.userParams.get<bool>("sgm.useSfmSeeds", sgmParams.useSfmSeeds);
    sgmParams.chooseDepthListPerTile = mp.userParams.get<bool>("sgm.chooseDepthListPerTile", sgmParams.chooseDepthListPerTile);
    sgmParams.exportIntermediateDepthSimMaps = mp.userParams.get<bool>("sgm.exportIntermediateDepthSimMaps", sgmParams.exportIntermediateDepthSimMaps);
    sgmParams.exportIntermediateVolumes = mp.userParams.get<bool>("sgm.exportIntermediateVolumes", sgmParams.exportIntermediateVolumes);
    sgmParams.exportIntermediateCrossVolumes = mp.userParams.get<bool>("sgm.exportIntermediateCrossVolumes", sgmParams.exportIntermediateCrossVolumes);
    sgmParams.exportIntermediateVolume9pCsv = mp.userParams.get<bool>("sgm.exportIntermediateVolume9pCsv", sgmParams.exportIntermediateVolume9pCsv);

    // get Refine user parameters from MultiViewParams property_tree

    auto& refineParams = depthMapParams.refineParams;
    refineParams.scale = mp.userParams.get<int>("refine.scale", refineParams.scale);
    refineParams.stepXY = mp.userParams.get<int>("refine.stepXY", refineParams.stepXY);
    refineParams.wsh = mp.userParams.get<int>("refine.wsh", refineParams.wsh);
    refineParams.nDepthsToRefine = mp.userParams.get<int>("refine.nDepthsToRefine", refineParams.nDepthsToRefine);
    refineParams.nSamplesHalf = mp.userParams.get<int>("refine.nSamplesHalf", refineParams.nSamplesHalf);
    refineParams.optimizationNbIters = mp.userParams.get<int>("refine.optimizationNbIters", refineParams.optimizationNbIters);
    refineParams.maxTCamsPerTile = mp.userParams.get<int>("refine.maxTCamsPerTile", refineParams.maxTCamsPerTile);
    refineParams.sigma = mp.userParams.get<double>("refine.sigma", refineParams.sigma);
    refineParams.gammaC = mp.userParams.get<double>("refine.gammaC", refineParams.gammaC);
    refineParams.gammaP = mp.userParams.get<double>("refine.gammaP", refineParams.gammaP);
    refineParams.doRefineFuse = mp.userParams.get<bool>("refine.doRefineFuse", refineParams.doRefineFuse);
    refineParams.doRefineOptimization = mp.userParams.get<bool>("refine.doRefineOptimization", refineParams.doRefineOptimization);
    refineParams.exportIntermediateDepthSimMaps = mp.userParams.get<bool>("refine.exportIntermediateDepthSimMaps", refineParams.exportIntermediateDepthSimMaps);
    refineParams.exportIntermediateCrossVolumes = mp.userParams.get<bool>("refine.exportIntermediateCrossVolumes", refineParams.exportIntermediateCrossVolumes);
    refineParams.exportIntermediateVolume9pCsv = mp.userParams.get<bool>("refine.exportIntermediateVolume9pCsv", refineParams.exportIntermediateVolume9pCsv);

    // get workflow user parameters from MultiViewParams property_tree

    depthMapParams.maxTCams = mp.userParams.get<int>("depthMap.maxTCams", depthMapParams.maxTCams);
    depthMapParams.useRefine = mp.userParams.get<bool>("depthMap.useRefine", depthMapParams.useRefine);
    depthMapParams.chooseTCamsPerTile = mp.userParams.get<bool>("depthMap.chooseTCamsPerTile", depthMapParams.chooseTCamsPerTile);
    depthMapParams.exportTilePattern = mp.userParams.get<bool>("depthMap.exportTilePattern", depthMapParams.exportTilePattern);
    depthMapParams.autoAdjustSmallImage = mp.userParams.get<bool>("depthMap.autoAdjustSmallImage", depthMapParams.autoAdjustSmallImage);
}

void estimateAndRefineDepthMaps(int cudaDeviceId, mvsUtils::MultiViewParams& mp, const std::vector<int>& cams)
{
    // set the device to use for GPU executions
    // the CUDA runtime API is thread-safe, it maintains per-thread state about the current device 
    setCudaDeviceId(cudaDeviceId);

    // initialize RAM image cache
    mvsUtils::ImagesCache<image::Image<image::RGBAfColor>> ic(mp, image::EImageColorSpace::LINEAR);

    // get user parameters from MultiViewParams property_tree
    DepthMapParams depthMapParams;
    getDepthMapParams(mp, depthMapParams);

    // compute SGM scale and step (set to -1)
    const bool autoSgmScaleStep = computeScaleStepSgmParams(mp, depthMapParams.sgmParams);

    // single tile case, update parameters
    if(hasOnlyOneTile(depthMapParams.tileParams, mp.getMaxImageWidth(), mp.getMaxImageHeight()))
      updateDepthMapParamsForSingleTileComputation(mp, autoSgmScaleStep, depthMapParams);

    // compute the maximum downscale factor
    const int maxDownscale = std::max(depthMapParams.sgmParams.scale * depthMapParams.sgmParams.stepXY,
                                      depthMapParams.refineParams.scale * depthMapParams.refineParams.stepXY);

    if(depthMapParams.tileParams.padding % maxDownscale != 0)
    {
      const int padding = divideRoundUp(depthMapParams.tileParams.padding, maxDownscale) * maxDownscale;
      ALICEVISION_LOG_WARNING("Override tiling padding parameter (before: " << depthMapParams.tileParams.padding << ", now: " << padding << ").");
      depthMapParams.tileParams.padding = padding;
    }

    // compute tile ROI list
    std::vector<ROI> tileRoiList;
    getTileRoiList(depthMapParams.tileParams, mp.getMaxImageWidth(), mp.getMaxImageHeight(), maxDownscale, tileRoiList);
    const int nbTilesPerCamera = tileRoiList.size();

    // log tiling information and ROI list
    logTileRoiList(depthMapParams.tileParams, mp.getMaxImageWidth(), mp.getMaxImageHeight(), maxDownscale, tileRoiList);

    // log SGM downscale & stepXY
    ALICEVISION_LOG_INFO("SGM parameters:" << std::endl
                         << "\t- scale: " << depthMapParams.sgmParams.scale << std::endl
                         << "\t- stepXY: " << depthMapParams.sgmParams.stepXY);

    // log Refine downscale & stepXY
    ALICEVISION_LOG_INFO("Refine parameters:" << std::endl
                         << "\t- scale: " << depthMapParams.refineParams.scale << std::endl
                         << "\t- stepXY: " << depthMapParams.refineParams.stepXY);

    // get maximum number of stream (simultaneous tiles)
    const int nbStreams = getNbStreams(mp, depthMapParams, nbTilesPerCamera);
    DeviceStreamManager deviceStreamManager(nbStreams);

    // build device cache
    const int nbRcPerBatch = divideRoundUp(nbStreams, nbTilesPerCamera);                // number of R cameras in the same batch
    const int nbTilesPerBatch = nbRcPerBatch * nbTilesPerCamera;                        // number of tiles in the same batch
    const bool hasRcWithoutDownscale = depthMapParams.sgmParams.scale == 1 || (depthMapParams.useRefine && depthMapParams.refineParams.scale == 1);
    const int nbCamerasPerSgm = (1 + depthMapParams.maxTCams) + (hasRcWithoutDownscale ? 0 : 1); // number of Sgm cameras per R camera
    const int nbCamerasPerRefine = depthMapParams.useRefine ? (1 + depthMapParams.maxTCams) : 0; // number of Refine cameras per R camera
    const int nbCamerasPerBatch = nbRcPerBatch * (nbCamerasPerSgm + nbCamerasPerRefine);         // number of cameras in the same batch

    DeviceCache& deviceCache = DeviceCache::getInstance();
    deviceCache.buildCache(nbCamerasPerBatch);
    
    // build tile list
    // order by R camera
    std::vector<Tile> tiles;
    tiles.reserve(cams.size() * tileRoiList.size());

    for(int rc : cams)
    {
        // compute T cameras list per R camera
        const std::vector<int> tCams = mp.findNearestCamsFromLandmarks(rc, depthMapParams.maxTCams).getDataWritable();
        const ROI rcImageRoi(Range(0, mp.getWidth(rc)), Range(0, mp.getHeight(rc)));

        for(std::size_t ti = 0;  ti < tileRoiList.size(); ++ti)
        {
            Tile t;

            t.id = ti;
            t.nbTiles = nbTilesPerCamera;
            t.rc = rc;
            t.roi = intersect(tileRoiList.at(ti), rcImageRoi);

            if(t.roi.isEmpty())
            {
              // do nothing, this ROI cannot intersect the R camera ROI.
            }
            else if(depthMapParams.chooseTCamsPerTile)
            {
              // find nearest T cameras per tile
              t.sgmTCams = mp.findTileNearestCams(rc, depthMapParams.sgmParams.maxTCamsPerTile, tCams, t.roi);

              if(depthMapParams.useRefine)
                t.refineTCams = mp.findTileNearestCams(rc, depthMapParams.refineParams.maxTCamsPerTile, tCams, t.roi);
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

    // allocate Sgm and Refine per stream in device memory
    std::vector<Sgm> sgmPerStream;
    std::vector<Refine> refinePerStream;

    sgmPerStream.reserve(nbStreams);
    refinePerStream.reserve(depthMapParams.useRefine ? nbStreams : 0);

    // initialize Sgm objects
    for(int i = 0; i < nbStreams; ++i)
      sgmPerStream.emplace_back(mp, depthMapParams.tileParams, depthMapParams.sgmParams, deviceStreamManager.getStream(i));

    // initialize Refine objects
    if(depthMapParams.useRefine)
      for(int i = 0; i < nbStreams; ++i)
          refinePerStream.emplace_back(mp, depthMapParams.tileParams, depthMapParams.refineParams, deviceStreamManager.getStream(i));

    // allocate final deth/similarity map tile list in host memory
    std::vector<std::vector<CudaHostMemoryHeap<float2, 2>>> depthSimMapTilePerCam(nbRcPerBatch);
    std::vector<std::vector<std::pair<float, float>>> depthMinMaxTilePerCam(nbRcPerBatch);

    for(int i = 0; i < nbRcPerBatch; ++i)
    {
        auto& depthSimMapTiles = depthSimMapTilePerCam.at(i);
        auto& depthMinMaxTiles = depthMinMaxTilePerCam.at(i);

        depthSimMapTiles.resize(nbTilesPerCamera);
        depthMinMaxTiles.resize(nbTilesPerCamera);

        for(int j = 0; j < nbTilesPerCamera; ++j)
        {
          if(depthMapParams.useRefine)
            depthSimMapTiles.at(j).allocate(refinePerStream.front().getDeviceDepthSimMap().getSize());
          else // final depth/similarity map is SGM only
            depthSimMapTiles.at(j).allocate(sgmPerStream.front().getDeviceDepthSimMap().getSize());
        }
    }

    // log device memory information
    logDeviceMemoryInfo();

    // compute number of batches
    const int nbBatches = divideRoundUp(int(tiles.size()), nbTilesPerBatch);

    // compute each batch of R cameras
    for(int b = 0; b < nbBatches; ++b)
    {
        // find first/last tile to compute
        const int firstTileIndex = b * nbTilesPerBatch;
        const int lastTileIndex = std::min((b + 1) * nbTilesPerBatch, int(tiles.size()));
        
        // load tile R and corresponding T cameras in device cache  
        for(int i = firstTileIndex; i < lastTileIndex; ++i)
        {
            const Tile& tile = tiles.at(i);

            // add Sgm R camera to Device cache
            deviceCache.addCamera(tile.rc, depthMapParams.sgmParams.scale, ic, mp);

            // add Sgm T cameras to Device cache
            for(const int tc : tile.sgmTCams)
                deviceCache.addCamera(tc, depthMapParams.sgmParams.scale, ic, mp);

            if(depthMapParams.useRefine)
            {
                // add Refine R camera to Device cache
                deviceCache.addCamera(tile.rc, depthMapParams.refineParams.scale, ic, mp);

                // add Refine T cameras to Device cache
                for(const int tc : tile.refineTCams)
                    deviceCache.addCamera(tc, depthMapParams.refineParams.scale, ic, mp);
            }

            if(depthMapParams.sgmParams.scale != 1 && (!depthMapParams.useRefine || depthMapParams.refineParams.scale != 1))
            {
              // add SGM R camera at scale 1 to Device cache.
              // R camera parameters at scale 1 are required for SGM retrieve best depth
              // TODO: Add only camera parameters to Device cache
              deviceCache.addCamera(tile.rc, 1, ic, mp);
            }
        }

        // wait for camera loading in device cache
        cudaDeviceSynchronize();

        // compute each batch tile
        for(int i = firstTileIndex; i < lastTileIndex; ++i)
        {
            Tile& tile = tiles.at(i);
            const int batchCamIndex = tile.rc % nbRcPerBatch;
            const int streamIndex = tile.id % nbStreams;

            // do not compute empty ROI
            // some images in the dataset may be smaller than others
            if(tile.roi.isEmpty())
                continue;

            // get tile result depth/similarity map in host memory
            CudaHostMemoryHeap<float2, 2>& tileDepthSimMap_hmh = depthSimMapTilePerCam.at(batchCamIndex).at(tile.id);

            // check T cameras
            if(tile.sgmTCams.empty() || (depthMapParams.useRefine && tile.refineTCams.empty())) // no T camera found
            {
                resetDepthSimMap(tileDepthSimMap_hmh);
                continue;
            }

            // build tile SGM depth list
            SgmDepthList sgmDepthList(mp, depthMapParams.sgmParams, tile);

            // compute the R camera depth list
            sgmDepthList.computeListRc();

            // check number of depths
            if(sgmDepthList.getDepths().empty()) // no depth found
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

            // compute Refine
            if(depthMapParams.useRefine)
            {
              Refine& refine = refinePerStream.at(streamIndex);
              refine.refineRc(tile, sgm.getDeviceDepthSimMap(), sgm.getDeviceNormalMap());

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
        if(lastTileIndex < tiles.size() && (tiles.at(lastTileIndex).rc == lastRc))
          --lastRc;

        // write depth/sim map result
        for(int c = firstRc; c <= lastRc; ++c)
        {
          const int batchCamIndex = c % nbRcPerBatch;

          if(depthMapParams.useRefine)
            writeDepthSimMapFromTileList(c, mp, depthMapParams.tileParams, tileRoiList, depthSimMapTilePerCam.at(batchCamIndex), depthMapParams.refineParams.scale, depthMapParams.refineParams.stepXY);
          else
            writeDepthSimMapFromTileList(c, mp, depthMapParams.tileParams, tileRoiList, depthSimMapTilePerCam.at(batchCamIndex), depthMapParams.sgmParams.scale, depthMapParams.sgmParams.stepXY);

          if(depthMapParams.exportTilePattern)
              exportDepthSimMapTilePatternObj(c, mp, tileRoiList, depthMinMaxTilePerCam.at(batchCamIndex));
        }
    }

    // merge intermediate results tiles if needed and desired
    if(tiles.size() > cams.size())
    {
        // merge tiles if needed and desired
        for(int rc : cams)
        {
            if(depthMapParams.sgmParams.exportIntermediateDepthSimMaps)
            {
                mergeDepthSimMapTiles(rc, mp, depthMapParams.sgmParams.scale, depthMapParams.sgmParams.stepXY, "_sgm");
            }

            if(depthMapParams.useRefine && depthMapParams.refineParams.exportIntermediateDepthSimMaps)
            {
                mergeDepthSimMapTiles(rc, mp, depthMapParams.refineParams.scale, depthMapParams.refineParams.stepXY, "_sgmUpscaled");
                mergeDepthSimMapTiles(rc, mp, depthMapParams.refineParams.scale, depthMapParams.refineParams.stepXY, "_refinedFused");
            }
        }
    }

    // some objects countains CUDA objects
    // this objects should be destroyed before the end of the program (i.e. the end of the CUDA context)
    DeviceCache::getInstance().clear();
    sgmPerStream.clear();
    refinePerStream.clear();
}

void computeNormalMaps(int cudaDeviceId, mvsUtils::MultiViewParams& mp, const std::vector<int>& cams)
{
    // set the device to use for GPU executions
    // the CUDA runtime API is thread-safe, it maintains per-thread state about the current device 
    setCudaDeviceId(cudaDeviceId);
    
    const float gammaC = 1.0f;
    const float gammaP = 1.0f;
    const int wsh = 3;

    mvsUtils::ImagesCache<image::Image<image::RGBAfColor>> ic(mp, image::EImageColorSpace::LINEAR);

    DeviceNormalMapper normalMapper;

    for(const int rc : cams)
    {
        const std::string normalMapFilepath = getFileNameFromIndex(mp, rc, mvsUtils::EFileType::normalMap, 0);

        if (!fs::exists(normalMapFilepath))
        {
            const int scale = 1;

            image::Image<float> depthMap;
            readImage(getFileNameFromIndex(mp, rc, mvsUtils::EFileType::depthMap, 0), depthMap, image::EImageColorSpace::NO_CONVERSION);

            image::Image<image::RGBfColor> normalMap(mp.getWidth(rc), mp.getHeight(rc));

            const int w = mp.getWidth(rc) / scale;
            const int h = mp.getHeight(rc) / scale;

            const system::Timer timer;
            ALICEVISION_LOG_INFO("Compute normal map (rc: " << rc << ")");

            // Fill Camera Struct

            fillHostCameraParameters(*(normalMapper.cameraParameters_h), rc, scale, mp);
            normalMapper.loadCameraParameters();
            normalMapper.allocHostMaps(w, h);
            normalMapper.copyDepthMap(depthMap.data(), depthMap.size());

            cuda_computeNormalMap(&normalMapper, w, h, wsh, gammaC, gammaP);

            float3* normalMapPtr = normalMapper.getNormalMapHst();

            constexpr bool q = (sizeof(image::RGBfColor[2]) == sizeof(float3[2]));
            if(q == true)
            {
                memcpy(normalMap.data(), normalMapper.getNormalMapHst(), w * h * sizeof(float3));
            }
            else
            {
                for(int i = 0; i < w * h; i++)
                {
                    normalMap(i).r() = normalMapPtr[i].x;
                    normalMap(i).g() = normalMapPtr[i].y;
                    normalMap(i).b() = normalMapPtr[i].z;
                }
            }

            image::writeImage(normalMapFilepath, normalMap,
                              image::ImageWriteOptions().toColorSpace(image::EImageColorSpace::LINEAR)
                                                        .storageDataType(image::EStorageDataType::Float));

            ALICEVISION_LOG_INFO("Compute normal map (rc: " << rc << ") done in: " << timer.elapsedMs() << " ms.");
        }
    }
}

} // namespace depthMap
} // namespace aliceVision
