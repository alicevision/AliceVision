// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Refine.hpp"
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/gpu/gpu.hpp>

#include <aliceVision/depthMap/RefineParams.hpp>
#include <aliceVision/depthMap/cuda/PlaneSweepingCuda.hpp>
#include <aliceVision/mvsData/Point2d.hpp>
#include <aliceVision/mvsData/Point3d.hpp>
#include <aliceVision/mvsUtils/common.hpp>
#include <aliceVision/mvsUtils/fileIO.hpp>
#include <aliceVision/mvsData/imageIO.hpp>
#include <aliceVision/alicevision_omp.hpp>

#include <boost/filesystem.hpp>

namespace aliceVision {
namespace depthMap {

namespace bfs = boost::filesystem;

RefineRc::RefineRc(const RefineParams& refineParams, const mvsUtils::MultiViewParams& mp, PlaneSweepingCuda& cps, int rc)
    : _rc(rc)
    , _mp(mp)
    , _cps(cps)
    , _refineParams(refineParams)
    , _depthSimMap(_rc, _mp, 1, 1)
{
    _tCams = _mp.findNearestCamsFromLandmarks(_rc, _refineParams.maxTCams);
}

RefineRc::~RefineRc()
{}

void RefineRc::getDepthPixSizeMapFromSGM(const DepthSimMap& sgmDepthSimMap, DepthSimMap& out_depthSimMapScale1Step1)
{
    const int w11 = _mp.getWidth(_rc);
    const int h11 = _mp.getHeight(_rc);

    out_depthSimMapScale1Step1.initFromSmaller(sgmDepthSimMap);

    // set sim (y) to pixsize
    for(int y = 0; y < h11; ++y)
    {
        for(int x = 0; x < w11; ++x)
        {
            const Point3d p = _mp.CArr[_rc] + (_mp.iCamArr[_rc] * Point2d(static_cast<float>(x), static_cast<float>(y))).normalize() * out_depthSimMapScale1Step1._dsm[y * w11 + x].depth;
            DepthSim& depthSim = out_depthSimMapScale1Step1._dsm[y * w11 + x];
            if(_refineParams.useTcOrRcPixSize)
                depthSim.sim = _mp.getCamsMinPixelSize(p, _tCams);
            else
                depthSim.sim = _mp.getCamPixelSize(p, _rc);
        }
    }
}

void RefineRc::filterMaskedPixels(DepthSimMap& out_depthSimMap)
{
    mvsUtils::ImagesCache<ImageRGBAf>::ImgSharedPtr img = _cps._ic.getImg_sync(_rc);

    const int h = _mp.getHeight(_rc);
    const int w = _mp.getWidth(_rc);
    for(int y = 0; y < h; ++y)
    {
        for(int x = 0; x < w; ++x)
        {
            const ColorRGBAf& floatRGBA = img->at(x, y);
            if(floatRGBA.a < 0.1f)
            {
                DepthSim& depthSim = out_depthSimMap._dsm[y * w + x];

                depthSim.depth = -2.0;
                depthSim.sim = -1.0;
            }
        }
    }
}

void RefineRc::refineRcTcDepthSimMap(DepthSimMap& depthSimMap, int tc)
{
    const int scale = depthSimMap._scale;
    const int w = _mp.getWidth(_rc) / scale;
    const int h = _mp.getHeight(_rc) / scale;

    if(_mp.verbose)
        ALICEVISION_LOG_DEBUG("refineRcTcDepthSimMap: width: " << w << ", height: " << h);

    long t1 = clock();

    const int nParts = 4;
    const int wPart = w / nParts;
    for(int p = 0; p < nParts; p++)
    {
        int xFrom = p * wPart;
        int wPartAct = std::min(wPart, w - xFrom);
        StaticVector<float> depthMap;
        depthSimMap.getDepthMapStep1XPart(depthMap, xFrom, wPartAct);
        StaticVector<float> simMap;
        depthSimMap.getSimMapStep1XPart(simMap, xFrom, wPartAct);

        _cps.refineRcTcDepthMap(_refineParams.useTcOrRcPixSize, _refineParams.ndepthsToRefine, simMap, depthMap, _rc, tc, scale, _refineParams.wsh, _refineParams.gammaC, _refineParams.gammaP, xFrom,
                                wPartAct);

        for(int yp = 0; yp < h; yp++)
        {
            for(int xp = xFrom; xp < xFrom + wPartAct; xp++)
            {
                const float depth = depthMap[yp * wPartAct + (xp - xFrom)];
                const float sim = simMap[yp * wPartAct + (xp - xFrom)];
                const float oldSim = depthSimMap._dsm[(yp / depthSimMap._step) * depthSimMap._w + (xp / depthSimMap._step)].sim;
                if((depth > 0.0f) && (sim < oldSim))
                {
                    depthSimMap._dsm[(yp / depthSimMap._step) * depthSimMap._w + (xp / depthSimMap._step)] = DepthSim(depth, sim);
                }
            }
        }

        if(_mp.verbose)
            mvsUtils::printfElapsedTime(t1, "refineRcTcDepthSimMap");
    }
}

void RefineRc::refineAndFuseDepthSimMapCUDA(DepthSimMap& out_depthSimMapFused, const DepthSimMap& depthPixSizeMapVis)
{
    auto startTime = std::chrono::high_resolution_clock::now();

    int w11 = _mp.getWidth(_rc);
    int h11 = _mp.getHeight(_rc);

    StaticVector<const DepthSimMap*> dataMaps;
    dataMaps.reserve(_tCams.size() + 1);
    // Put the raw SGM result first:
    dataMaps.push_back(&depthPixSizeMapVis); //!!DO NOT ERASE!!!

    for(int c = 0; c < _tCams.size(); c++)
    {
        int tc = _tCams[c];

        DepthSimMap* depthSimMapC = new DepthSimMap(_rc, _mp, 1, 1);
        depthSimMapC->initJustFromDepthMap(depthPixSizeMapVis, 1.0f);
        refineRcTcDepthSimMap(*depthSimMapC, tc);
        dataMaps.push_back(depthSimMapC);

        if(_refineParams.exportIntermediateResults)
        {
            depthSimMapC->save("_refine_tc_" + std::to_string(tc) + "_" + std::to_string(_mp.getViewId(tc)));
            // depthSimMapC->saveToImage(_mp.getDepthMapsFolder() + "refine_photo_" + std::to_string(_mp.getViewId(_rc)) + "_tc_" + std::to_string(_mp.getViewId(tc)) + ".png", -2.0f);
        }
    }

    // in order to fit into GPU memory
    int nhParts = 4;
    int hPartHeightGlob = h11 / nhParts;
    for(int hPart = 0; hPart < nhParts; hPart++)
    {
        int hPartHeight = std::min(h11, (hPart + 1) * hPartHeightGlob) - hPart * hPartHeightGlob;

        // vector of one depthSimMap tile per Tc
        StaticVector<StaticVector<DepthSim>*> dataMapsHPart;
        dataMapsHPart.reserve(dataMaps.size());
        for(int i = 0; i < dataMaps.size(); i++) // iterate over Tc cameras
        {
            StaticVector<DepthSim>* dataMapHPart = new StaticVector<DepthSim>();
            dataMapHPart->resize(w11 * hPartHeight);
            const StaticVector<DepthSim>& dsm = dataMaps[i]->_dsm;

#pragma omp parallel for
            for(int y = 0; y < hPartHeight; y++)
            {
                for(int x = 0; x < w11; x++)
                {
                    (*dataMapHPart)[y * w11 + x] = dsm[(y + hPart * hPartHeightGlob) * w11 + x];
                }
            }

            dataMapsHPart.push_back(dataMapHPart);
        }

        StaticVector<DepthSim> depthSimMapFusedHPart;
        depthSimMapFusedHPart.resize_with(w11 * hPartHeight, DepthSim(-1.0f, 1.0f));

        _cps.fuseDepthSimMapsGaussianKernelVoting(w11, hPartHeight, depthSimMapFusedHPart, dataMapsHPart, _refineParams.nSamplesHalf, _refineParams.nDepthsToRefine, _refineParams.sigma);

#pragma omp parallel for
        for(int y = 0; y < hPartHeight; y++)
        {
            for(int x = 0; x < w11; x++)
            {
                out_depthSimMapFused._dsm[(y + hPart * hPartHeightGlob) * w11 + x] =
                    depthSimMapFusedHPart[y * w11 + x];
            }
        }

        deleteAllPointers(dataMapsHPart);
    }

    dataMaps[0] = nullptr; // it is input dsmap we dont want to delete it
    for(int c = 1; c < dataMaps.size(); c++)
    {
        delete dataMaps[c];
    }
    ALICEVISION_LOG_INFO("==== refineAndFuseDepthSimMapCUDA done in : " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startTime).count() << "ms.");
}

void RefineRc::optimizeDepthSimMapCUDA(DepthSimMap& out_depthSimMapOptimized, // optimized
                                       const DepthSimMap& depthPixSizeMapVis, // SGM
                                       const DepthSimMap& depthSimMapPhoto) // refined
{
    auto startTime = std::chrono::high_resolution_clock::now();

    if(_refineParams.nIters == 0)
    {
        _depthSimMap.init(depthSimMapPhoto);
        return;
    }

    int h11 = _mp.getHeight(_rc);

    int nParts = 4; // TODO: estimate the amount of VRAM available to decide the tiling
    int hPart = h11 / nParts;
    for(int part = 0; part < nParts; ++part)
    {
        int yFrom = part * hPart;
        int hPartAct = std::min(hPart, h11 - yFrom);
        _cps.optimizeDepthSimMapGradientDescent(out_depthSimMapOptimized._dsm, depthPixSizeMapVis._dsm, depthSimMapPhoto._dsm, _rc, _refineParams.nSamplesHalf, _refineParams.nDepthsToRefine, _refineParams.sigma,
                                                   _refineParams.nIters,
                                                   yFrom, hPartAct);
    }
    ALICEVISION_LOG_INFO("==== optimizeDepthSimMapCUDA done in : " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startTime).count() << "ms.");
}

bool RefineRc::refineRc(const DepthSimMap& depthSimMapToRefine)
{
    const auto startTime = std::chrono::high_resolution_clock::now();
    const IndexT viewId = _mp.getViewId(_rc);

    ALICEVISION_LOG_INFO("Refine depth map of view id: " << viewId << " (rc: " << (_rc + 1) << " / " << _mp.ncams << ")");

    if(_refineParams.checkAlreadyComputed && (mvsUtils::FileExists(getOptSimMapFileName(viewId, 1, 1))))
    {
        ALICEVISION_LOG_INFO("Already computed: " << getOptSimMapFileName(viewId, 1, 1));
        return false;
    }

    long tall = clock();

    DepthSimMap depthPixSizeMapVis(_rc, _mp, 1, 1);
    getDepthPixSizeMapFromSGM(depthSimMapToRefine, depthPixSizeMapVis);
    filterMaskedPixels(depthPixSizeMapVis);

    if(_refineParams.exportIntermediateResults)
    {
        depthPixSizeMapVis.save("_sgmRescaled");
    }

    DepthSimMap depthSimMapPhoto(_rc, _mp, 1, 1);

    if (_refineParams.doRefineFuse)
    {
        refineAndFuseDepthSimMapCUDA(depthSimMapPhoto, depthPixSizeMapVis);
    }
    else
    {
        depthSimMapPhoto.initJustFromDepthMap(depthPixSizeMapVis, 1.0f);
    }

    if(_refineParams.doRefineOpt && _refineParams.nIters != 0)
    {
        if (_refineParams.exportIntermediateResults)
        {
            // depthPixSizeMapVis.saveToImage(_mp.getDepthMapsFolder() + "refine_" + std::to_string(viewId) + "_vis.png", 0.0f);
            // depthSimMapPhoto.saveToImage(_mp.getDepthMapsFolder() + "refine_" + std::to_string(viewId) + "_photo.png", 0.0f);
            depthSimMapPhoto.save("_photo");
            // _depthSimMapOpt.saveToImage(_mp.getDepthMapsFolder() + "refine_" + std::to_string(viewId) + "_opt.png", 0.0f);
        }

        optimizeDepthSimMapCUDA(_depthSimMap, depthPixSizeMapVis, depthSimMapPhoto);
    }
    else
    {
        _depthSimMap.init(depthSimMapPhoto);
    }

    ALICEVISION_LOG_INFO("Refine depth map done in: " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startTime).count() << " ms.");
    return true;
}

std::string RefineRc::getPhotoDepthMapFileName(IndexT viewId, int scale, int step) const
{
    return _mp.getDepthMapsFolder() + std::to_string(viewId) + "_depthMap_scale" + mvsUtils::num2str(scale) + "_step" + mvsUtils::num2str(step) + "_refinePhoto.exr";
}

std::string RefineRc::getPhotoSimMapFileName(IndexT viewId, int scale, int step) const
{
    return _mp.getDepthMapsFolder() + std::to_string(viewId) + "_simMap_scale" + mvsUtils::num2str(scale) + "_step" + mvsUtils::num2str(step) + "_refinePhoto.exr";
}

std::string RefineRc::getOptDepthMapFileName(IndexT viewId, int scale, int step) const
{
    return _mp.getDepthMapsFolder() + std::to_string(viewId) + "_depthMap_scale" + mvsUtils::num2str(scale) + "_step" + mvsUtils::num2str(step) + "_refineOpt.exr";
}

std::string RefineRc::getOptSimMapFileName(IndexT viewId, int scale, int step) const
{
    return _mp.getDepthMapsFolder() + std::to_string(viewId) + "_simMap_scale" + mvsUtils::num2str(scale) + "_step" + mvsUtils::num2str(step) + "_refineOpt.exr";
}

} // namespace depthMap
} // namespace aliceVision
