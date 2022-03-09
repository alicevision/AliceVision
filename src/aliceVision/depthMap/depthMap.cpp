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
#include <aliceVision/mvsUtils/MultiViewParams.hpp>
#include <aliceVision/mvsUtils/TileParams.hpp>
#include <aliceVision/depthMap/Sgm.hpp>
#include <aliceVision/depthMap/SgmParams.hpp>
#include <aliceVision/depthMap/SgmDepthList.hpp>
#include <aliceVision/depthMap/Refine.hpp>
#include <aliceVision/depthMap/RefineParams.hpp>
#include <aliceVision/depthMap/cuda/host/utils.hpp>
#include <aliceVision/depthMap/cuda/host/DeviceCache.hpp>
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

void getTileList(std::vector<ROI>& tileList, const TileParams& tileParams, const mvsUtils::MultiViewParams& mp, int rc)
{
    const int width = mp.getOriginalWidth(rc);
    const int height = mp.getOriginalHeight(rc);

    const int tileWidth  = (tileParams.width  > 0) ? tileParams.width  : (tileParams.height > 0) ? tileParams.height : width;
    const int tileHeight = (tileParams.height > 0) ? tileParams.height : (tileParams.width  > 0) ? tileParams.width  : height;

    if(tileParams.padding >= std::min(tileWidth, tileHeight))
        ALICEVISION_THROW_ERROR("Unable to compute tile list, tile padding size is too large.");

    const int nbTileSideX = 1 + int(std::ceil(float(width  - tileWidth)  / float(tileWidth  - tileParams.padding)));
    const int nbTileSideY = 1 + int(std::ceil(float(height - tileHeight) / float(tileHeight - tileParams.padding)));

    tileList.resize(nbTileSideX * nbTileSideY);

    for(int i = 0; i < nbTileSideX; ++i)
    {
        const int startX = i * (tileWidth - tileParams.padding);
        const int endX = std::min(startX + tileWidth, width);

        for(int j = 0; j < nbTileSideY; ++j)
        {
            const int startY = j * (tileHeight - tileParams.padding);
            const int endY = std::min(startY + tileHeight, height);

            tileList.at(i * nbTileSideY + j) = ROI(startX, endX, startY, endY);
        }
    }
}

void getTileParams(const mvsUtils::MultiViewParams& mp, TileParams& tileParams)
{
    // get tile user parameters from MultiViewParams property_tree

    tileParams.width = mp.userParams.get<int>("tile.width", tileParams.width);
    tileParams.height = mp.userParams.get<int>("tile.height", tileParams.height);
    tileParams.padding = mp.userParams.get<int>("tile.padding", tileParams.padding);
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
    refineParams.nIters = mp.userParams.get<int>("refine.nIters", refineParams.nIters);
    refineParams.sigma = mp.userParams.get<double>("refine.sigma", refineParams.sigma);
    refineParams.gammaC = mp.userParams.get<double>("refine.gammaC", refineParams.gammaC);
    refineParams.gammaP = mp.userParams.get<double>("refine.gammaP", refineParams.gammaP);
    refineParams.useTcOrRcPixSize = mp.userParams.get<bool>("refine.useTcOrRcPixSize", refineParams.useTcOrRcPixSize);
    refineParams.useRefineFuseVolumeStrategy = mp.userParams.get<bool>("refine.useRefineFuseVolumeStrategy", refineParams.useRefineFuseVolumeStrategy);
    refineParams.doRefineFuse = mp.userParams.get<bool>("refine.doRefineFuse", refineParams.doRefineFuse);
    refineParams.doRefineOptimization = mp.userParams.get<bool>("refine.doRefineOptimization", refineParams.doRefineOptimization);
    refineParams.exportIntermediateResults = mp.userParams.get<bool>("refine.exportIntermediateResults", refineParams.exportIntermediateResults);
}

void estimateAndRefineDepthMaps(int cudaDeviceId, mvsUtils::MultiViewParams& mp, const std::vector<int>& cams)
{
    // set the device to use for GPU executions
    // the CUDA runtime API is thread-safe, it maintains per-thread state about the current device 
    setCudaDeviceId(cudaDeviceId);

    TileParams tileParams;
    SgmParams sgmParams;
    RefineParams refineParams;

    // get user parameters from MultiViewParams property_tree
    getTileParams(mp, tileParams);
    getSgmParams(mp, sgmParams);
    getRefineParams(mp, refineParams);

    // compute scale and step
    computeScaleStepSgmParams(mp, sgmParams);

    // load images from files into RAM
    mvsUtils::ImagesCache<ImageRGBAf> ic(mp, imageIO::EImageColorSpace::LINEAR);

    for(const int rc : cams)
    {
        std::vector<ROI> tileList;
        getTileList(tileList, tileParams, mp, rc);

        for(int i = 0; i < tileList.size(); ++i)
        {
            // get correcponding ROI
            const ROI& roi = tileList.at(i);

            Sgm sgm(rc, ic, mp, sgmParams, tileParams, roi, 0 /*stream*/);

            // compute Semi-Global Matching
            sgm.sgmRc();

            Refine refine(rc, ic, mp, refineParams, tileParams, roi, 0 /*stream*/);

            // R camera has no T cameras
            if(refine.getTCams().empty() || sgm.empty())
            {
                ALICEVISION_LOG_INFO("No T cameras for camera rc: " << rc << ", generate default depth and sim maps.");
                continue;
            }

            // compute Refine
            refine.refineRc(sgm.getDepthSimMap());

            // write results
            refine.getDepthSimMap().save();
        }

        // merge tiles if multiple tile
        if(tileList.size() > 1)
        {
            DepthSimMap finalDepthSimMap(rc, mp, refineParams.scale, refineParams.stepXY);
            finalDepthSimMap.loadFromTiles(tileList);
            finalDepthSimMap.save();

            if(sgmParams.exportIntermediateResults)
            {
                DepthSimMap sgmDepthSimMap(rc, mp, sgmParams.scale, sgmParams.stepXY);
                sgmDepthSimMap.loadFromTiles(tileList, "_sgm");
                sgmDepthSimMap.save("_sgm");

                DepthSimMap sgmStep1DepthSimMap(rc, mp, sgmParams.scale, 1);
                sgmStep1DepthSimMap.loadFromTiles(tileList, "_sgmStep1");
                sgmStep1DepthSimMap.save("_sgmStep1");
            }

            if(refineParams.exportIntermediateResults)
            {
                DepthSimMap refineDepthSimMap(rc, mp, refineParams.scale, refineParams.stepXY);
                refineDepthSimMap.loadFromTiles(tileList, "_sgmUpscaled");
                refineDepthSimMap.save("_sgmUpscaled");
                refineDepthSimMap.loadFromTiles(tileList, "_refinedFused");
                refineDepthSimMap.save("_refinedFused");
            }
        }
    }

    // DeviceCache countains CUDA objects 
    // this objects should be destroyed before the end of the program (i.e. the end of the CUDA context)
    DeviceCache::getInstance().clear();
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
            {
                int w = 0;
                int h = 0;
                readImage(getFileNameFromIndex(mp, rc, mvsUtils::EFileType::depthMap, 0), w, h, depthMap, EImageColorSpace::NO_CONVERSION);
            }

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
