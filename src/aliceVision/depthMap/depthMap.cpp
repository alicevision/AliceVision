// This file is part of the AliceVision project.
// Copyright (c) 2021 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "depthMap.hpp"

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/mvsUtils/MultiViewParams.hpp>
#include <aliceVision/mvsUtils/fileIO.hpp>
#include <aliceVision/mvsData/imageIO.hpp>
#include <aliceVision/depthMap/Refine.hpp>
#include <aliceVision/depthMap/RefineParams.hpp>
#include <aliceVision/depthMap/Sgm.hpp>
#include <aliceVision/depthMap/SgmParams.hpp>
#include <aliceVision/depthMap/cuda/PlaneSweepingCuda.hpp>

namespace aliceVision {
namespace depthMap {

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
    refineParams.exportIntermediateResults = mp.userParams.get<bool>("refine.exportIntermediateResults", refineParams.exportIntermediateResults);
}

void estimateAndRefineDepthMaps(int cudaDeviceIndex, mvsUtils::MultiViewParams& mp, const std::vector<int>& cams)
{
    SgmParams sgmParams;
    RefineParams refineParams;

    // get user parameters from MultiViewParams property_tree
    getSgmParams(mp, sgmParams);
    getRefineParams(mp, refineParams);

    // compute scale and step
    computeScaleStepSgmParams(mp, sgmParams);

    // load images from files into RAM
    mvsUtils::ImagesCache<ImageRGBAf> ic(mp, imageIO::EImageColorSpace::LINEAR);

    // load stuff on GPU memory and creates multi-level images and computes gradients
    PlaneSweepingCuda cps(cudaDeviceIndex, ic, mp, sgmParams.scale);

    for(const int rc : cams)
    {
        Sgm sgm(sgmParams, mp, cps, rc);
        Refine refine(refineParams, mp, cps, rc);
        
        // preload sgmTcams async
        {
            const auto startTime = std::chrono::high_resolution_clock::now();
            cps._ic.refreshImages_async(sgm.getTCams().getData());
            ALICEVISION_LOG_INFO("Preload T cameras done in: " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startTime).count() << " ms.");
        }

        sgm.sgmRc();

        // rc has no tcam
        if(refine.getTCams().empty() || sgm.getDepths().empty())
        {
            ALICEVISION_LOG_INFO("No T cameras for camera rc: " << rc << ", generate default depth and sim maps.");
            refine.getDepthSimMap().save(); // generate default depthSimMap
            continue;
        }

        refine.refineRc(sgm.getDepthSimMap());

        // write results
        refine.getDepthSimMap().save();
    }
}

void computeNormalMaps(int cudaDeviceIndex, mvsUtils::MultiViewParams& mp, const std::vector<int>& cams)
{
    using namespace imageIO;
    
    const float gammaC = 1.0f;
    const float gammaP = 1.0f;
    const int wsh = 3;

    mvsUtils::ImagesCache<ImageRGBAf> ic(mp, EImageColorSpace::LINEAR);
    PlaneSweepingCuda cps(cudaDeviceIndex, ic, mp, 1);

    NormalMapping* mapping = cps.createNormalMapping();

    for(const int rc : cams)
    {
        const std::string normalMapFilepath = getFileNameFromIndex(mp, rc, mvsUtils::EFileType::normalMap, 0);

        if(!mvsUtils::FileExists(normalMapFilepath))
        {
            std::vector<float> depthMap;
            int w = 0;
            int h = 0;
            readImage(getFileNameFromIndex(mp, rc, mvsUtils::EFileType::depthMap, 0), w, h, depthMap, EImageColorSpace::NO_CONVERSION);

            std::vector<ColorRGBf> normalMap;
            normalMap.resize(mp.getWidth(rc) * mp.getHeight(rc));

            cps.computeNormalMap(mapping, depthMap, normalMap, rc, 1, gammaC, gammaP, wsh);
            writeImage(normalMapFilepath, mp.getWidth(rc), mp.getHeight(rc), normalMap, EImageQuality::LOSSLESS, OutputFileColorSpace(EImageColorSpace::NO_CONVERSION));
        }
    }
    cps.deleteNormalMapping(mapping);
}

} // namespace depthMap
} // namespace aliceVision
