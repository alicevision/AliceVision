// This file is part of the AliceVision project.
// Copyright (c) 2021 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "depthMap.hpp"

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/mvsData/imageIO.hpp>
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

void computeScaleStepSgmParams(const mvsUtils::MultiViewParams& mp, SgmParams& sgmParams)
{
    const int fileScale = 1; // input images scale (should be one)
    const int maxSideXY = sgmParams.maxSideXY / mp.getProcessDownscale();
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

    ALICEVISION_LOG_INFO("Computed SGM parameters:" << std::endl
                         << "\t- scale: " << sgmParams.scale << std::endl
                         << "\t- stepXY: " << sgmParams.stepXY);
}

int getNbStreams(const mvsUtils::MultiViewParams& mp, const DepthMapParams& depthMapParams, int nbTilesPerCamera)
{
    const int maxImageSize = mp.getMaxImageWidth() * mp.getMaxImageHeight(); // process downscale apply
    const double cameraFrameCostMB = (((maxImageSize / depthMapParams.sgmParams.scale) + (maxImageSize / depthMapParams.refineParams.scale)) * 16.0) / (1024.0 * 1024.0); // SGM + Refine float4 RGBA

    double sgmTileCostMB;
    double sgmTileCostUnpaddedMB;
    {
      Sgm sgm(mp, depthMapParams.tileParams, depthMapParams.sgmParams, 0 /*stream*/);
      sgmTileCostMB = sgm.getDeviceMemoryConsumption();
      sgmTileCostUnpaddedMB = sgm.getDeviceMemoryConsumptionUnpadded();
    }

    double refineTileCostMB;
    double refineTileCostUnpaddedMB;
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
                         << "\t- requirement per tile: " << rcMinCostMB << " MB" << std::endl
                         << "\t- # computation buffers per tile: " << tileCostMB << " MB" << " (Sgm: " << sgmTileCostMB << " MB" << ", Refine: " << refineTileCostMB << " MB)" << std::endl
                         << "\t- # input images (R + " << depthMapParams.maxTCams << " Ts): " << rcCamsCost << " MB (single multi-res image size: " << cameraFrameCostMB << " MB)");

    ALICEVISION_LOG_DEBUG( "Theoretical device memory cost for a tile without padding: " << tileCostUnpaddedMB << " MB" << " (Sgm: " << sgmTileCostUnpaddedMB << " MB" << ", Refine: " << refineTileCostUnpaddedMB << " MB)");

    ALICEVISION_LOG_INFO("Parallelization:" << std::endl
                         << "\t- # tiles per image: " << nbTilesPerCamera << std::endl
                         << "\t- # simultaneous depth maps computation: " << ((nbRemainingTiles < 1) ? nbAllowedSimultaneousRc : (nbAllowedSimultaneousRc + 1)) << std::endl
                         << "\t- # streams: " << out_nbAllowedStreams);

    if(out_nbAllowedStreams < 1 || rcCamParams > ALICEVISION_DEVICE_MAX_CONSTANT_CAMERA_PARAM_SETS)
        ALICEVISION_THROW_ERROR("Not enough GPU memory to compute a single tile.")

    return out_nbAllowedStreams;
}

void getDepthMapParams(const mvsUtils::MultiViewParams& mp, DepthMapParams& depthMapParams)
{
    // get tile user parameters from MultiViewParams property_tree

    auto& tileParams = depthMapParams.tileParams;
    tileParams.width = mp.userParams.get<int>("tile.width", tileParams.width);
    tileParams.height = mp.userParams.get<int>("tile.height", tileParams.height);
    tileParams.padding = mp.userParams.get<int>("tile.padding", tileParams.padding);

    // get SGM user parameters from MultiViewParams property_tree

    auto& sgmParams = depthMapParams.sgmParams;
    sgmParams.scale = mp.userParams.get<int>("sgm.scale", sgmParams.scale);
    sgmParams.stepXY = mp.userParams.get<int>("sgm.stepXY", sgmParams.stepXY);
    sgmParams.stepZ = mp.userParams.get<int>("sgm.stepZ", sgmParams.stepZ);
    sgmParams.wsh = mp.userParams.get<int>("sgm.wsh", sgmParams.wsh);
    sgmParams.maxDepths = mp.userParams.get<int>("sgm.maxDepths", sgmParams.maxDepths);
    sgmParams.maxDepthsPerTc = mp.userParams.get<int>("sgm.maxDepthsPerTc", sgmParams.maxDepthsPerTc);
    sgmParams.maxSideXY = mp.userParams.get<int>("sgm.maxSideXY", sgmParams.maxSideXY);
    sgmParams.maxTCamsPerTile = mp.userParams.get<int>("sgm.maxTCamsPerTile", sgmParams.maxTCamsPerTile);
    sgmParams.gammaC = mp.userParams.get<double>("sgm.gammaC", sgmParams.gammaC);
    sgmParams.gammaP = mp.userParams.get<double>("sgm.gammaP", sgmParams.gammaP);
    sgmParams.p1 = mp.userParams.get<double>("sgm.p1", sgmParams.p1);
    sgmParams.p2Weighting = mp.userParams.get<double>("sgm.p2Weighting", sgmParams.p2Weighting);
    sgmParams.filteringAxes = mp.userParams.get<std::string>("sgm.filteringAxes", sgmParams.filteringAxes);
    sgmParams.useSfmSeeds = mp.userParams.get<bool>("sgm.useSfmSeeds", sgmParams.useSfmSeeds);
    sgmParams.chooseDepthListPerTile = mp.userParams.get<bool>("sgm.chooseDepthListPerTile", sgmParams.chooseDepthListPerTile);
    sgmParams.exportIntermediateResults = mp.userParams.get<bool>("sgm.exportIntermediateResults", sgmParams.exportIntermediateResults);

    // get Refine user parameters from MultiViewParams property_tree

    auto& refineParams = depthMapParams.refineParams;
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
    refineParams.exportIntermediateResults = mp.userParams.get<bool>("refine.exportIntermediateResults", refineParams.exportIntermediateResults);

    // get workflow user parameters from MultiViewParams property_tree

    depthMapParams.maxTCams = mp.userParams.get<int>("depthMap.maxTCams", depthMapParams.maxTCams);
    depthMapParams.chooseTCamsPerTile = mp.userParams.get<bool>("depthMap.chooseTCamsPerTile", depthMapParams.chooseTCamsPerTile);
}

void estimateAndRefineDepthMaps(int cudaDeviceId, mvsUtils::MultiViewParams& mp, const std::vector<int>& cams)
{
    // set the device to use for GPU executions
    // the CUDA runtime API is thread-safe, it maintains per-thread state about the current device 
    setCudaDeviceId(cudaDeviceId);

    // get user parameters from MultiViewParams property_tree
    DepthMapParams depthMapParams;
    getDepthMapParams(mp, depthMapParams);

    // compute SGM scale and step
    computeScaleStepSgmParams(mp, depthMapParams.sgmParams);

    // initialize RAM image cache
    mvsUtils::ImagesCache<ImageRGBAf> ic(mp, imageIO::EImageColorSpace::LINEAR);

    // compute tile ROI list
    std::vector<ROI> tileRoiList;
    getTileList(depthMapParams.tileParams, mp.getMaxImageWidth(), mp.getMaxImageHeight(), tileRoiList);
    const int nbTilesPerCamera = tileRoiList.size();

    // get maximum number of stream (simultaneous tiles)
    const int nbStreams = getNbStreams(mp, depthMapParams, nbTilesPerCamera);
    DeviceStreamManager deviceStreamManager(nbStreams);

    // build device cache
    const int nbRcPerBatch = std::ceil(nbStreams / float(nbTilesPerCamera));            // number of R cameras in the same batch
    const int nbTilesPerBatch = nbRcPerBatch * nbTilesPerCamera;                        // number of tiles in the same batch
    const int nbCamerasPerBatch = (nbRcPerBatch * (1 + depthMapParams.maxTCams)) * 2;   // number of cameras (refine + sgm downscaled images) in the same batch

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

        for(std::size_t ti = 0;  ti < tileRoiList.size(); ++ti)
        {
            Tile t;

            t.id = ti;
            t.nbTiles = nbTilesPerCamera;
            t.rc = rc;
            t.roi = tileRoiList.at(ti);

            if(depthMapParams.chooseTCamsPerTile)
            {
              // find nearest T cameras per tile
              t.sgmTCams = mp.findTileNearestCams(rc, depthMapParams.sgmParams.maxTCamsPerTile, tCams, t.roi);
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

        if(depthMapParams.sgmParams.exportIntermediateResults || depthMapParams.refineParams.exportIntermediateResults)
            exportDepthSimMapTilePatternObj(rc, mp, tileRoiList);
    }

    // allocate Sgm and Refine per stream in device memory
    std::vector<Sgm> sgmPerStream;
    std::vector<Refine> refinePerStream;

    sgmPerStream.reserve(nbStreams);
    refinePerStream.reserve(nbStreams);

    for(int i = 0; i < nbStreams; ++i)
    {
        sgmPerStream.emplace_back(mp, depthMapParams.tileParams, depthMapParams.sgmParams, deviceStreamManager.getStream(i));
        refinePerStream.emplace_back(mp, depthMapParams.tileParams, depthMapParams.refineParams, deviceStreamManager.getStream(i));
    }

    // allocate final deth/similarity map tile list in host memory
    std::vector<std::vector<CudaHostMemoryHeap<float2, 2>>> depthSimMapTilePerCam(nbRcPerBatch);

    for(int i = 0; i < nbRcPerBatch; ++i)
    {
        auto& depthSimMapTiles = depthSimMapTilePerCam.at(i);
        depthSimMapTiles.resize(nbTilesPerCamera);

        for(int j = 0; j < nbTilesPerCamera; ++j)
          depthSimMapTiles.at(j).allocate(refinePerStream.front().getDeviceDepthSimMap().getSize());
    }


    // log device memory information
    logDeviceMemoryInfo();

    // compute number of batches
    const int nbBatches = std::ceil(tiles.size() / float(nbTilesPerBatch));

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

            deviceCache.addCamera(tile.rc, depthMapParams.sgmParams.scale, ic, mp);
            deviceCache.addCamera(tile.rc, depthMapParams.refineParams.scale, ic, mp);

            for(const int tc : tile.sgmTCams)
                deviceCache.addCamera(tc, depthMapParams.sgmParams.scale, ic, mp);

            for(const int tc : tile.refineTCams)
                deviceCache.addCamera(tc, depthMapParams.refineParams.scale, ic, mp);
        }

        // wait for camera loading in device cache
        cudaDeviceSynchronize();

        // compute each batch tile
        for(int i = firstTileIndex; i < lastTileIndex; ++i)
        {
            Tile& tile = tiles.at(i);
            const int batchCamIndex = tile.rc % nbRcPerBatch;
            const int streamIndex = tile.id % nbStreams;

            // get tile result depth/similarity map in host memory
            CudaHostMemoryHeap<float2, 2>& tileDepthSimMap_hmh = depthSimMapTilePerCam.at(batchCamIndex).at(tile.id);

            // check T cameras
            if(tile.sgmTCams.empty() || tile.refineTCams.empty()) // no T camera found
            {
                resetDepthSimMap(tileDepthSimMap_hmh);
                continue;
            }

            // build tile SGM depth list
            SgmDepthList sgmDepthList(tile);

            // compute the R camera depth list
            sgmDepthList.computeListRc(mp, depthMapParams.sgmParams);

            // check number of depths
            if(sgmDepthList.getDepths().empty()) // no depth found
            {
                resetDepthSimMap(tileDepthSimMap_hmh);
                continue;
            }

            // log debug camera / depth information
            sgmDepthList.logRcTcDepthInformation(mp);

            // check if starting and stopping depth are valid
            sgmDepthList.checkStartingAndStoppingDepth();

            // compute Semi-Global Matching
            Sgm& sgm = sgmPerStream.at(streamIndex);
            sgm.sgmRc(tile, sgmDepthList);

            // compute Refine
            Refine& refine = refinePerStream.at(streamIndex);
            refine.refineRc(tile, sgm.getDeviceDepthSimMap());
            
            // copy depth/similarity map from device to host
            tileDepthSimMap_hmh.copyFrom(refine.getDeviceDepthSimMap(), deviceStreamManager.getStream(streamIndex));
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
          writeDepthSimMapFromTileList(c, mp, depthMapParams.tileParams, tileRoiList, depthSimMapTilePerCam.at(batchCamIndex), depthMapParams.refineParams.scale, depthMapParams.refineParams.stepXY);
        }
    }

    // merge intermediate results tiles if needed and desired
    if(tiles.size() > cams.size())
    {
        // merge tiles if needed and desired
        for(int rc : cams)
        {
            if(depthMapParams.sgmParams.exportIntermediateResults)
            {
                mergeDepthSimMapTiles(rc, mp, depthMapParams.sgmParams.scale, depthMapParams.sgmParams.stepXY, "_sgm");
            }

            if(depthMapParams.refineParams.exportIntermediateResults)
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
    using namespace imageIO;

    // set the device to use for GPU executions
    // the CUDA runtime API is thread-safe, it maintains per-thread state about the current device 
    setCudaDeviceId(cudaDeviceId);
    
    const float gammaC = 1.0f;
    const float gammaP = 1.0f;
    const int wsh = 3;

    mvsUtils::ImagesCache<ImageRGBAf> ic(mp, EImageColorSpace::LINEAR);

    DeviceNormalMapper normalMapper;

    for(const int rc : cams)
    {
        const std::string normalMapFilepath = getFileNameFromIndex(mp, rc, mvsUtils::EFileType::normalMap, 0);

        if (!fs::exists(normalMapFilepath))
        {
            const int scale = 1;
            std::vector<float> depthMap;
            mvsUtils::readDepthMap(rc, mp, depthMap);

            std::vector<ColorRGBf> normalMap;
            normalMap.resize(mp.getWidth(rc) * mp.getHeight(rc));

            const int w = mp.getWidth(rc) / scale;
            const int h = mp.getHeight(rc) / scale;

            const system::Timer timer;
            ALICEVISION_LOG_INFO("Compute normal map (rc: " << rc << ")");

            // Fill Camera Struct

            fillHostCameraParameters(*(normalMapper.cameraParameters_h), rc, scale, mp);
            normalMapper.loadCameraParameters();
            normalMapper.allocHostMaps(w, h);
            normalMapper.copyDepthMap(depthMap);

            cuda_computeNormalMap(&normalMapper, w, h, wsh, gammaC, gammaP);

            float3* normalMapPtr = normalMapper.getNormalMapHst();

            constexpr bool q = (sizeof(ColorRGBf[2]) == sizeof(float3[2]));
            if(q == true)
            {
                memcpy(normalMap.data(), normalMapper.getNormalMapHst(), w * h * sizeof(float3));
            }
            else
            {
                for(int i = 0; i < w * h; i++)
                {
                    normalMap[i].r = normalMapPtr[i].x;
                    normalMap[i].g = normalMapPtr[i].y;
                    normalMap[i].b = normalMapPtr[i].z;
                }
            }

            writeImage(normalMapFilepath, mp.getWidth(rc), mp.getHeight(rc), normalMap, EImageQuality::LOSSLESS, OutputFileColorSpace(EImageColorSpace::NO_CONVERSION));

            ALICEVISION_LOG_INFO("Compute normal map (rc: " << rc << ") done in: " << timer.elapsedMs() << " ms.");
        }
    }
}

} // namespace depthMap
} // namespace aliceVision
