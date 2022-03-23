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
    const int out_nbAllowedSimultaneousTile = nbAllowedSimultaneousRc * nbTilesPerCamera;

    ALICEVISION_LOG_INFO("Device memory cost / stream management:" << std::endl
                         << "\t- device memory available for computation: " << deviceMemoryMB << " MB" << std::endl
                         << "\t- device memory cost for a single camera: " << cameraFrameCostMB << " MB" << std::endl
                         << "\t- device memory cost for a Sgm tile: " << sgmTileCostMB << " MB" << std::endl
                         << "\t- device memory cost for a Refine tile: " << refineTileCostMB << " MB" << std::endl
                         << "\t- maximum device memory cost for a R camera computation: " << rcCostMB << " MB" << std::endl
                         << "\t- # allowed simultaneous R camera computation: " << nbAllowedSimultaneousRc << std::endl
                         << "\t- # allowed simultaneous tile computation: " << out_nbAllowedSimultaneousTile);

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
    std::vector<ROI> tiles;
    getTileList(tileParams, mp.getMaxImageOriginalWidth(), mp.getMaxImageOriginalHeight(), tiles);

    // get maximum number of stream
    const int nbSimultaneousTile = getNbSimultaneousTile(mp, tileParams, sgmParams, refineParams, tiles.size());
    DeviceStreamManager deviceStreamManager(nbSimultaneousTile);

    // compute max T cameras
    const int maxTCams = std::max(sgmParams.maxTCams, refineParams.maxTCams);

    // build device cache
    const int maxSimultaneousRc = std::ceil(nbSimultaneousTile / float(tiles.size()));
    const int maxSimultaneousCameras = (maxSimultaneousRc * (1 + maxTCams)) * 2; // refine + sgm downscaled images

    DeviceCache& deviceCache = DeviceCache::getInstance();
    deviceCache.buildCache(maxSimultaneousCameras);               
    
    // allocate Sgm and Refine structures per tile in device memory
    std::vector<Sgm> sgmPerTile;
    std::vector<Refine> refinePerTile;

    sgmPerTile.reserve(nbSimultaneousTile);
    refinePerTile.reserve(nbSimultaneousTile);

    for(int i = 0; i < nbSimultaneousTile; ++i)
    {
        sgmPerTile.emplace_back(mp, tileParams, sgmParams, deviceStreamManager.getStream(i));
        refinePerTile.emplace_back(mp, tileParams, refineParams, deviceStreamManager.getStream(i));
    }

    // log device memory information
    logDeviceMemoryInfo();
    
    // compute T cameras list per R camera
    std::vector<std::vector<int>> tcsPerRc(cams.size()); // <rci, tc ids>
    for(int rci = 0; rci < cams.size(); ++rci)
    {
        const int rc = cams.at(rci);
        tcsPerRc.at(rci) = mp.findNearestCamsFromLandmarks(rc, maxTCams).getDataWritable();
    }

    // compute number of batches
    const int nbBatches = std::ceil(cams.size() / float(maxSimultaneousRc));

    // compute each batch of R cameras
    for(int b = 0; b < nbBatches; ++b)
    {
        // find first/last batch R cameras 
        const int firstRci = b * maxSimultaneousRc;
        const int lastRci = std::min((b + 1) * maxSimultaneousRc, int(cams.size()));
        
        // load R and corresponding T cameras in device cache  
        for(int rci = firstRci; rci < lastRci; ++rci)
        {
            const int rc = cams.at(rci);
            
            deviceCache.addCamera(rc, sgmParams.scale, ic, mp);
            deviceCache.addCamera(rc, refineParams.scale, ic, mp);

            for(const int tc : tcsPerRc.at(rci))
            {
                deviceCache.addCamera(tc, sgmParams.scale, ic, mp);
                deviceCache.addCamera(tc, refineParams.scale, ic, mp);
            }
        }

        // wait for camera loading in device cache
        cudaDeviceSynchronize();

        // compute R cameras
        for(int rci = firstRci; rci < lastRci; ++rci)
        {
            const int rc = cams.at(rci);
            const std::vector<int>& tCams = tcsPerRc.at(rci);

            if(tCams.empty()) // no T camera found
                continue;

            // compute each R camera tile with streams
            for(int t = 0; t < tiles.size(); ++t)
            {
                // get batch tile index
                const int tileIdx = (rci - firstRci) * tiles.size() + t;

                // get tile 2d region of interest
                const ROI& roi = tiles.at(t);

                // build SGM depth list
                SgmDepthList sgmDepthList(rc, tCams, mp, sgmParams, roi);

                // compute the R camera depth list
                sgmDepthList.computeListRc();
                if(sgmDepthList.getDepths().empty()) // no depth found
                    continue;

                // log debug camera / depth information
                sgmDepthList.logRcTcDepthInformation();

                // check if starting and stopping depth are valid
                sgmDepthList.checkStartingAndStoppingDepth();

                // compute Semi-Global Matching
                Sgm& sgm = sgmPerTile.at(tileIdx);
                sgm.sgmRc(rc, sgmDepthList, roi);

                // compute Refine
                Refine& refine = refinePerTile.at(tileIdx);
                refine.refineRc(rc, tCams, sgm.getDeviceDepthSimMap(), roi);
            }
        }

        // wait for R cameras batch computation
        cudaDeviceSynchronize();
        
        // write R cameras depth/sim map
        for(int rci = firstRci; rci < lastRci; ++rci)
        {
            const int rc = cams.at(rci);

            for(int t = 0; t < tiles.size(); ++t)
            {
                // get batch tile index
                const int tileIdx = (rci - firstRci) * tiles.size() + t;

                // get tile 2d region of interest
                const ROI& roi = tiles.at(t);

                // write refine final depth/sim
                writeDepthSimMap(rc, mp, tileParams, roi, refinePerTile.at(tileIdx).getDeviceDepthSimMap(), refineParams.scale, refineParams.stepXY);
            }

            // merge tiles if needed and desired
            if(tileParams.mergeTiles && tiles.size() > 1)
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
    }

    // some objects countains CUDA objects
    // this objects should be destroyed before the end of the program (i.e. the end of the CUDA context)
    DeviceCache::getInstance().clear();
    sgmPerTile.clear();
    refinePerTile.clear();
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
