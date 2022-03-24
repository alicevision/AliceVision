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
#include <aliceVision/mvsUtils/TileParams.hpp>
#include <aliceVision/depthMap/depthMapUtils.hpp>
#include <aliceVision/depthMap/Sgm.hpp>
#include <aliceVision/depthMap/SgmParams.hpp>
#include <aliceVision/depthMap/SgmDepthList.hpp>
#include <aliceVision/depthMap/Refine.hpp>
#include <aliceVision/depthMap/RefineParams.hpp>
#include <aliceVision/depthMap/cuda/host/utils.hpp>
#include <aliceVision/depthMap/cuda/host/DeviceCache.hpp>
#include <aliceVision/depthMap/cuda/host/DeviceStreamManager.hpp>
#include <aliceVision/depthMap/cuda/normalMapping/DeviceNormalMapper.hpp>
#include <aliceVision/depthMap/cuda/normalMapping/deviceNormalMap.hpp>

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

int getNbSimultaneousTile(const mvsUtils::MultiViewParams& mp, 
                          const mvsUtils::TileParams& tileParams,
                          const SgmParams& sgmParams, 
                          const RefineParams& refineParams, 
                          int nbTilesPerCamera)
{
    const int maxImageSize = mp.getMaxImageWidth() * mp.getMaxImageHeight(); // process downscale apply
    const int maxTCams = std::max(sgmParams.maxTCams, refineParams.maxTCams);

    const double cameraFrameCostMB = (((maxImageSize / sgmParams.scale) + (maxImageSize / refineParams.scale)) * 16.0) / (1024.0 * 1024.0); // SGM + Refine float4 RGBA
    const double sgmTileCostMB = Sgm(mp, tileParams, sgmParams, 0 /*stream*/).getDeviceMemoryConsumption();
    const double refineTileCostMB = Refine(mp, tileParams, refineParams, 0 /*stream*/).getDeviceMemoryConsumption();
    const double rcCostMB = (cameraFrameCostMB + maxTCams * cameraFrameCostMB) + nbTilesPerCamera * (sgmTileCostMB + refineTileCostMB);
    const int rcCamParams = (1 + maxTCams) * 2; // number of camera parameters in device constant memory

    double deviceMemoryMB;
    {
        double availableMB, usedMB, totalMB;
        getDeviceMemoryInfo(availableMB, usedMB, totalMB);
        deviceMemoryMB = availableMB * 0.8; // available memory margin
    }

    const int nbAllowedSimultaneousRc = std::min(int(deviceMemoryMB / rcCostMB), (ALICEVISION_DEVICE_MAX_CONSTANT_CAMERA_PARAM_SETS / rcCamParams));

    int out_nbAllowedSimultaneousTile;
    if(nbAllowedSimultaneousRc < 1) // an R camera cannot be computed simultaneously
    {
        const double camerasCostMB = (cameraFrameCostMB + maxTCams * cameraFrameCostMB);
        out_nbAllowedSimultaneousTile = std::floor((deviceMemoryMB - camerasCostMB) / (sgmTileCostMB + refineTileCostMB));
    }
    else
    {
        out_nbAllowedSimultaneousTile = nbAllowedSimultaneousRc * nbTilesPerCamera;
    }

    ALICEVISION_LOG_INFO("Device memory cost / stream management:" << std::endl
                         << "\t- device memory available for computation: " << deviceMemoryMB << " MB" << std::endl
                         << "\t- device memory cost for a single camera: " << cameraFrameCostMB << " MB" << std::endl
                         << "\t- device memory cost for a Sgm tile: " << sgmTileCostMB << " MB" << std::endl
                         << "\t- device memory cost for a Refine tile: " << refineTileCostMB << " MB" << std::endl
                         << "\t- maximum device memory cost for a R camera computation: " << rcCostMB << " MB" << std::endl
                         << "\t- # allowed simultaneous R camera computation: " << nbAllowedSimultaneousRc << std::endl
                         << "\t- # allowed simultaneous tile computation: " << out_nbAllowedSimultaneousTile);

    if(out_nbAllowedSimultaneousTile < 1 || rcCamParams > ALICEVISION_DEVICE_MAX_CONSTANT_CAMERA_PARAM_SETS)
        ALICEVISION_THROW_ERROR("Not enough GPU memory to compute a single tile.")

    return out_nbAllowedSimultaneousTile;
}

void getTileParams(const mvsUtils::MultiViewParams& mp, mvsUtils::TileParams& tileParams)
{
    // get tile user parameters from MultiViewParams property_tree

    tileParams.width = mp.userParams.get<int>("tile.width", tileParams.width);
    tileParams.height = mp.userParams.get<int>("tile.height", tileParams.height);
    tileParams.padding = mp.userParams.get<int>("tile.padding", tileParams.padding);
    tileParams.mergeTiles = mp.userParams.get<bool>("tile.mergeTiles", tileParams.mergeTiles);
}

void getSgmParams(const mvsUtils::MultiViewParams& mp, SgmParams& sgmParams) 
{
    // get SGM user parameters from MultiViewParams property_tree

    sgmParams.scale = mp.userParams.get<int>("sgm.scale", sgmParams.scale);
    sgmParams.stepXY = mp.userParams.get<int>("sgm.stepXY", sgmParams.stepXY);
    sgmParams.stepZ = mp.userParams.get<int>("sgm.stepZ", sgmParams.stepZ);
    sgmParams.wsh = mp.userParams.get<int>("sgm.wsh", sgmParams.wsh);
    sgmParams.maxTCams = mp.userParams.get<int>("sgm.maxTCams", sgmParams.maxTCams);
    sgmParams.maxDepths = mp.userParams.get<int>("sgm.maxDepths", sgmParams.maxDepths);
    sgmParams.maxDepthsPerTc = mp.userParams.get<int>("sgm.maxDepthsPerTc", sgmParams.maxDepthsPerTc);
    sgmParams.maxSideXY = mp.userParams.get<int>("sgm.maxSideXY", sgmParams.maxSideXY);
    sgmParams.gammaC = mp.userParams.get<double>("sgm.gammaC", sgmParams.gammaC);
    sgmParams.gammaP = mp.userParams.get<double>("sgm.gammaP", sgmParams.gammaP);
    sgmParams.p1 = mp.userParams.get<double>("sgm.p1", sgmParams.p1);
    sgmParams.p2Weighting = mp.userParams.get<double>("sgm.p2Weighting", sgmParams.p2Weighting);
    sgmParams.filteringAxes = mp.userParams.get<std::string>("sgm.filteringAxes", sgmParams.filteringAxes);
    sgmParams.useSfmSeeds = mp.userParams.get<bool>("sgm.useSfmSeeds", sgmParams.useSfmSeeds);
    sgmParams.exportIntermediateResults = mp.userParams.get<bool>("sgm.exportIntermediateResults", sgmParams.exportIntermediateResults);
}

void getRefineParams(const mvsUtils::MultiViewParams& mp, RefineParams& refineParams) 
{
    // get Refine user parameters from MultiViewParams property_tree

    refineParams.wsh = mp.userParams.get<int>("refine.wsh", refineParams.wsh);
    refineParams.maxTCams = mp.userParams.get<int>("refine.maxTCams", refineParams.maxTCams);
    refineParams.nDepthsToRefine = mp.userParams.get<int>("refine.nDepthsToRefine", refineParams.nDepthsToRefine);
    refineParams.nSamplesHalf = mp.userParams.get<int>("refine.nSamplesHalf", refineParams.nSamplesHalf);
    refineParams.optimizationNbIters = mp.userParams.get<int>("refine.optimizationNbIters", refineParams.optimizationNbIters);
    refineParams.sigma = mp.userParams.get<double>("refine.sigma", refineParams.sigma);
    refineParams.gammaC = mp.userParams.get<double>("refine.gammaC", refineParams.gammaC);
    refineParams.gammaP = mp.userParams.get<double>("refine.gammaP", refineParams.gammaP);
    refineParams.doRefineFuse = mp.userParams.get<bool>("refine.doRefineFuse", refineParams.doRefineFuse);
    refineParams.doRefineOptimization = mp.userParams.get<bool>("refine.doRefineOptimization", refineParams.doRefineOptimization);
    refineParams.exportIntermediateResults = mp.userParams.get<bool>("refine.exportIntermediateResults", refineParams.exportIntermediateResults);
}

void estimateAndRefineDepthMaps(int cudaDeviceId, mvsUtils::MultiViewParams& mp, const std::vector<int>& cams)
{
    // set the device to use for GPU executions
    // the CUDA runtime API is thread-safe, it maintains per-thread state about the current device 
    setCudaDeviceId(cudaDeviceId);

    mvsUtils::TileParams tileParams;
    SgmParams sgmParams;
    RefineParams refineParams;

    // get user parameters from MultiViewParams property_tree
    getTileParams(mp, tileParams);
    getSgmParams(mp, sgmParams);
    getRefineParams(mp, refineParams);

    // compute SGM scale and step
    computeScaleStepSgmParams(mp, sgmParams);

    // initialize RAM image cache
    mvsUtils::ImagesCache<ImageRGBAf> ic(mp, imageIO::EImageColorSpace::LINEAR);

    // compute tile ROI list
    std::vector<ROI> tileRoiList;
    getTileRoiList(tileParams, mp.getMaxImageOriginalWidth(), mp.getMaxImageOriginalHeight(), tileRoiList);

    // get maximum number of stream
    const int nbSimultaneousTile = getNbSimultaneousTile(mp, tileParams, sgmParams, refineParams, tileRoiList.size());
    DeviceStreamManager deviceStreamManager(nbSimultaneousTile);

    // compute max T cameras
    const int maxTCams = std::max(sgmParams.maxTCams, refineParams.maxTCams);

    // build device cache
    const int maxSimultaneousRc = std::ceil(nbSimultaneousTile / float(tileRoiList.size()));
    const int maxSimultaneousCameras = (maxSimultaneousRc * (1 + maxTCams)) * 2; // refine + sgm downscaled images

    DeviceCache& deviceCache = DeviceCache::getInstance();
    deviceCache.buildCache(maxSimultaneousCameras);               
    
    // build tile list
    struct Tile
    {
        int rc;
        std::vector<int> tCams;
        ROI roi;
    };
    std::vector<Tile> tiles;

    tiles.reserve(cams.size() * tileRoiList.size());

    for(int rc : cams)
    {
        // compute T cameras list per R camera
        const std::vector<int> tCams = mp.findNearestCamsFromLandmarks(rc, maxTCams).getDataWritable();

        for(const ROI& roi : tileRoiList)
        {
            Tile t;
            t.rc = rc;
            t.tCams = tCams;
            t.roi = roi;
            tiles.push_back(t);
        }
    }

    // allocate Sgm / Refine and final depth/similarity map per stream in device memory
    std::vector<Sgm> sgmPerStream;
    std::vector<Refine> refinePerStream;
    std::vector<CudaHostMemoryHeap<float2, 2>> depthSimMapPerStream;

    sgmPerStream.reserve(nbSimultaneousTile);
    refinePerStream.reserve(nbSimultaneousTile);
    depthSimMapPerStream.resize(nbSimultaneousTile);

    for(int i = 0; i < nbSimultaneousTile; ++i)
    {
        sgmPerStream.emplace_back(mp, tileParams, sgmParams, deviceStreamManager.getStream(i));
        refinePerStream.emplace_back(mp, tileParams, refineParams, deviceStreamManager.getStream(i));
        depthSimMapPerStream.at(i).allocate(refinePerStream.at(i).getDeviceDepthSimMap().getSize());
    }

    // log device memory information
    logDeviceMemoryInfo();

    // compute number of batches
    const int nbBatches = std::ceil(tiles.size() / float(nbSimultaneousTile));

    // compute each batch of R cameras
    for(int b = 0; b < nbBatches; ++b)
    {
        // find first/last tile to compute
        const int firstTileIndex = b * nbSimultaneousTile;
        const int lastTileIndex = std::min((b + 1) * nbSimultaneousTile, int(tiles.size()));
        
        // load tile R and corresponding T cameras in device cache  
        for(int i = firstTileIndex; i < lastTileIndex; ++i)
        {
            const Tile& tile = tiles.at(i);

            deviceCache.addCamera(tile.rc, sgmParams.scale, ic, mp);
            deviceCache.addCamera(tile.rc, refineParams.scale, ic, mp);

            for(const int tc : tile.tCams)
            {
                deviceCache.addCamera(tc, sgmParams.scale, ic, mp);
                deviceCache.addCamera(tc, refineParams.scale, ic, mp);
            }
        }

        // wait for camera loading in device cache
        cudaDeviceSynchronize();

        // compute each batch tile
        for(int i = firstTileIndex; i < lastTileIndex; ++i)
        {
            const Tile& tile = tiles.at(i);
            const int streamIndex = (i - firstTileIndex);

            if(tile.tCams.empty()) // no T camera found
                continue;

            // build SGM depth list
            SgmDepthList sgmDepthList(tile.rc, tile.tCams, mp, sgmParams, tile.roi);

            // compute the R camera depth list
            sgmDepthList.computeListRc();
            if(sgmDepthList.getDepths().empty()) // no depth found
                continue;

            // log debug camera / depth information
            sgmDepthList.logRcTcDepthInformation();

            // check if starting and stopping depth are valid
            sgmDepthList.checkStartingAndStoppingDepth();

            // compute Semi-Global Matching
            Sgm& sgm = sgmPerStream.at(streamIndex);
            sgm.sgmRc(tile.rc, sgmDepthList, tile.roi);

            // compute Refine
            Refine& refine = refinePerStream.at(streamIndex);
            refine.refineRc(tile.rc, tile.tCams, sgm.getDeviceDepthSimMap(), tile.roi);
            
            // copy depth/similarity map from device to host
            depthSimMapPerStream.at(streamIndex).copyFrom(refine.getDeviceDepthSimMap(), deviceStreamManager.getStream(streamIndex));
        }

        // wait for tiles batch computation
        cudaDeviceSynchronize();
        
        // write tiles depth/sim map
        for(int i = firstTileIndex; i < lastTileIndex; ++i)
        {
           const Tile& tile = tiles.at(i);
           const int streamIndex = (i - firstTileIndex);
           writeDepthSimMap(tile.rc, mp, tileParams, tile.roi, depthSimMapPerStream.at(streamIndex), refineParams.scale, refineParams.stepXY);
        }
    }

    // merge tiles if needed and desired
    if(tileParams.mergeTiles && tiles.size() > cams.size())
    {
        // merge tiles if needed and desired
        for(int rc : cams)
        {
            mergeDepthSimMapTiles(rc, mp, refineParams.scale, refineParams.stepXY);

            if(sgmParams.exportIntermediateResults)
            {
                mergeDepthSimMapTiles(rc, mp, sgmParams.scale, sgmParams.stepXY, "_sgm");
            }

            if(refineParams.exportIntermediateResults)
            {
                mergeDepthSimMapTiles(rc, mp, refineParams.scale, refineParams.stepXY, "_sgmUpscaled");
                mergeDepthSimMapTiles(rc, mp, refineParams.scale, refineParams.stepXY, "_refinedFused");
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

        if(!mvsUtils::FileExists(normalMapFilepath))
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
