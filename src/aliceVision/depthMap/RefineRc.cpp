// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "RefineRc.hpp"
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/gpu/gpu.hpp>

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

RefineRc::RefineRc(int rc, int scale, int step, SemiGlobalMatchingParams& sp)
    : SemiGlobalMatchingRc(rc, scale, step, sp)
    , _depthSimMapOpt(_rc, _sp.mp, 1, 1)
{
    const int nbNearestCams = _sp.mp.userParams.get<int>("refineRc.maxTCams", 6);
    _refineTCams  = _sp.mp.findNearestCamsFromLandmarks(rc, nbNearestCams);
    _userTcOrPixSize = _sp.mp.userParams.get<bool>("refineRc.useTcOrRcPixSize", false);
    _nbDepthsToRefine = _sp.mp.userParams.get<int>("refineRc.ndepthsToRefine", 31);
    _refineWsh = _sp.mp.userParams.get<int>("refineRc.wsh", 3);
    _refineNiters = _sp.mp.userParams.get<int>("refineRc.niters", 100);
    _refineNSamplesHalf = _sp.mp.userParams.get<int>("refineRc.nSamplesHalf", 150);
    _refineSigma  = static_cast<float>(_sp.mp.userParams.get<double>("refineRc.sigma", 15.0));
    _refineGammaC = static_cast<float>(_sp.mp.userParams.get<double>("refineRc.gammaC", 15.5));
    _refineGammaP = static_cast<float>(_sp.mp.userParams.get<double>("refineRc.gammaP", 8.0));
}

RefineRc::~RefineRc()
{
}

void RefineRc::preloadSgmTcams_async()
{
    _sp.cps._ic.refreshImages_async(_sgmTCams.getData());
}

void RefineRc::getDepthPixSizeMapFromSGM(DepthSimMap& out_depthSimMapScale1Step1)
{
    const int w11 = _sp.mp.getWidth(_rc);
    const int h11 = _sp.mp.getHeight(_rc);

    out_depthSimMapScale1Step1.initFromSmaller(_sgmDepthSimMap);

    // set sim (y) to pixsize
    for(int y = 0; y < h11; ++y)
    {
        for(int x = 0; x < w11; ++x)
        {
            const Point3d p = _sp.mp.CArr[_rc] + (_sp.mp.iCamArr[_rc] * Point2d(static_cast<float>(x), static_cast<float>(y))).normalize() * out_depthSimMapScale1Step1._dsm[y * w11 + x].depth;
            DepthSim& depthSim = out_depthSimMapScale1Step1._dsm[y * w11 + x];
            if(_userTcOrPixSize)
                depthSim.sim = _sp.mp.getCamsMinPixelSize(p, _refineTCams);
            else
                depthSim.sim = _sp.mp.getCamPixelSize(p, _rc);
        }
    }
}

void RefineRc::filterMaskedPixels(DepthSimMap& out_depthSimMap, int rc)
{
    mvsUtils::ImagesCache<ImageRGBAf>::ImgSharedPtr img = _sp.cps._ic.getImg_sync(rc);

    const int h = _sp.mp.getHeight(rc);
    const int w = _sp.mp.getWidth(rc);
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

void RefineRc::refineAndFuseDepthSimMapCUDA(DepthSimMap& out_depthSimMapFused, const DepthSimMap& depthPixSizeMapVis)
{
    auto startTime = std::chrono::high_resolution_clock::now();

    int w11 = _sp.mp.getWidth(_rc);
    int h11 = _sp.mp.getHeight(_rc);

    StaticVector<const DepthSimMap*> dataMaps;
    dataMaps.reserve(_refineTCams.size() + 1);
    // Put the raw SGM result first:
    dataMaps.push_back(&depthPixSizeMapVis); //!!DO NOT ERASE!!!

    const int scale = 1;
    RcTc prt(_sp.mp, _sp.cps);

    for(int c = 0; c < _refineTCams.size(); c++)
    {
        int tc = _refineTCams[c];

        DepthSimMap* depthSimMapC = new DepthSimMap(_rc, _sp.mp, scale, 1);
        depthSimMapC->initJustFromDepthMap(depthPixSizeMapVis, 1.0f);

        prt.refineRcTcDepthSimMap(_userTcOrPixSize, depthSimMapC, _rc, tc, _nbDepthsToRefine, _refineWsh, _refineGammaC, _refineGammaP);

        dataMaps.push_back(depthSimMapC);

        if (_sp.exportIntermediateResults)
        {
            depthSimMapC->save("_refine_tc_" + std::to_string(tc) + "_" + std::to_string(_sp.mp.getViewId(tc)));
        //     // depthSimMapC->saveToImage(_sp.mp.getDepthMapsFolder() + "refine_photo_" + std::to_string(_sp.mp.getViewId(_rc)) + "_tc_" +
        //     //                           std::to_string(_sp.mp.getViewId(tc)) + ".png", -2.0f);
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

        _sp.cps.fuseDepthSimMapsGaussianKernelVoting(w11, hPartHeight, depthSimMapFusedHPart, dataMapsHPart,
                                                      _refineNSamplesHalf, _nbDepthsToRefine, _refineSigma);

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

    if (_refineNiters == 0)
    {
        _depthSimMapOpt.init(depthSimMapPhoto);
        return;
    }

    int h11 = _sp.mp.getHeight(_rc);

    int nParts = 4; // TODO: estimate the amount of VRAM available to decide the tiling
    int hPart = h11 / nParts;
    for(int part = 0; part < nParts; ++part)
    {
        int yFrom = part * hPart;
        int hPartAct = std::min(hPart, h11 - yFrom);
        _sp.cps.optimizeDepthSimMapGradientDescent(out_depthSimMapOptimized._dsm, depthPixSizeMapVis._dsm, depthSimMapPhoto._dsm,
                                                   _rc, _refineNSamplesHalf,
                                                   _nbDepthsToRefine, _refineSigma, _refineNiters, yFrom, hPartAct);
    }
    ALICEVISION_LOG_INFO("==== optimizeDepthSimMapCUDA done in : " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startTime).count() << "ms.");
}

bool RefineRc::refinerc(bool checkIfExists)
{
    const IndexT viewId = _sp.mp.getViewId(_rc);

    ALICEVISION_LOG_DEBUG("Refine CUDA (rc: " << (_rc + 1) << " / " << _sp.mp.ncams << ")");

    // generate default depthSimMap if rc has no tcam
    if(_refineTCams.empty() || _depths.empty())
    {
        ALICEVISION_LOG_INFO("No T cameras for camera rc=" << _rc);
        return true;
    }

    if(checkIfExists && (mvsUtils::FileExists(_sp.getREFINE_opt_simMapFileName(viewId, 1, 1))))
    {
        ALICEVISION_LOG_INFO("Already computed: " << _sp.getREFINE_opt_simMapFileName(viewId, 1, 1));
        return false;
    }

    long tall = clock();

    DepthSimMap depthPixSizeMapVis(_rc, _sp.mp, 1, 1);
    getDepthPixSizeMapFromSGM(depthPixSizeMapVis);
    filterMaskedPixels(depthPixSizeMapVis, _rc);

    if (_sp.exportIntermediateResults)
    {
        depthPixSizeMapVis.save("_sgmRescaled");
    }

    DepthSimMap depthSimMapPhoto(_rc, _sp.mp, 1, 1);

    if (_sp.doRefineFuse)
    {
        refineAndFuseDepthSimMapCUDA(depthSimMapPhoto, depthPixSizeMapVis);
    }
    else
    {
        depthSimMapPhoto.initJustFromDepthMap(depthPixSizeMapVis, 1.0f);
    }

    if(_sp.doRefineOpt && _refineNiters != 0)
    {
        if (_sp.exportIntermediateResults)
        {
            // depthPixSizeMapVis.saveToImage(_sp.mp.getDepthMapsFolder() + "refine_" + std::to_string(viewId) + "_vis.png", 0.0f);
            // depthSimMapPhoto.saveToImage(_sp.mp.getDepthMapsFolder() + "refine_" + std::to_string(viewId) + "_photo.png", 0.0f);
            depthSimMapPhoto.save("_photo");
            // _depthSimMapOpt.saveToImage(_sp.mp.getDepthMapsFolder() + "refine_" + std::to_string(viewId) + "_opt.png", 0.0f);
        }

        optimizeDepthSimMapCUDA(_depthSimMapOpt, depthPixSizeMapVis, depthSimMapPhoto);
    }
    else
    {
        _depthSimMapOpt.init(depthSimMapPhoto);
    }

    mvsUtils::printfElapsedTime(tall, "Refine CUDA (rc: " + mvsUtils::num2str(_rc) + " / " + mvsUtils::num2str(_sp.mp.ncams) + ")");

    return true;
}

void RefineRc::writeDepthMap()
{
  _depthSimMapOpt.save();
}

void estimateAndRefineDepthMaps(int cudaDeviceIndex, mvsUtils::MultiViewParams& mp, const std::vector<int>& cams)
{
  const int fileScale = 1; // input images scale (should be one)
  int sgmScale = mp.userParams.get<int>("semiGlobalMatching.scale", -1);
  int sgmStepXY = mp.userParams.get<int>("semiGlobalMatching.stepXY", -1);
  const int maxSideXY = mp.userParams.get<int>("semiGlobalMatching.maxSideXY", 700) / mp.getProcessDownscale();
  const int maxImageW = mp.getMaxImageWidth();
  const int maxImageH = mp.getMaxImageHeight();
  int maxSgmW = maxSideXY;
  int maxSgmH = maxSideXY * 0.8;
  if (maxImageW < maxImageH)
      std::swap(maxSgmW, maxSgmH);

  if(sgmScale == -1)
  {
      // compute the number of scales that will be used in the plane sweeping.
      // the highest scale should have a resolution close to 700x550 (or less).
      const int scaleTmp = computeStep(mp, fileScale, maxSgmW, maxSgmH);

      sgmScale = std::min(2, scaleTmp);
  }
  if (sgmStepXY == -1)
  {
      sgmStepXY = computeStep(mp, fileScale * sgmScale, maxSgmW, maxSgmH);
  }

  ALICEVISION_LOG_INFO("Plane sweeping parameters:\n"
      "\t- scale: " << sgmScale << "\n"
      "\t- stepXY: " << sgmStepXY);

  // load images from files into RAM
  mvsUtils::ImagesCache<ImageRGBAf> ic(mp, imageIO::EImageColorSpace::LINEAR);
  // load stuff on GPU memory and creates multi-level images and computes gradients
  PlaneSweepingCuda cps(cudaDeviceIndex, ic, mp, sgmScale);
  // init plane sweeping parameters
  SemiGlobalMatchingParams sp(mp, cps);

  for(const int rc : cams)
  {
      RefineRc sgmRefineRc(rc, sgmScale, sgmStepXY, sp);

      auto startTime = std::chrono::high_resolution_clock::now();
      sgmRefineRc.preloadSgmTcams_async();
      ALICEVISION_LOG_INFO("==== preloadSgmTcams_async done in : " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startTime).count() << "ms.");

      startTime = std::chrono::high_resolution_clock::now();
      ALICEVISION_LOG_INFO("Estimate depth map, view id: " << mp.getViewId(rc));
      sgmRefineRc.sgmrc();
      ALICEVISION_LOG_INFO("==== SGM done in : " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startTime).count() << "ms.");

      startTime = std::chrono::high_resolution_clock::now();
      ALICEVISION_LOG_INFO("Refine depth map, view id: " << mp.getViewId(rc));
      sgmRefineRc.refinerc();
      ALICEVISION_LOG_INFO("==== RefineRC done in : " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startTime).count() << "ms.");

      // write results
      sgmRefineRc.writeDepthMap();
  }
}


void computeNormalMaps(int cudaDeviceIndex, mvsUtils::MultiViewParams& mp, const std::vector<int>& cams)
{
  const float igammaC = 1.0f;
  const float igammaP = 1.0f;
  const int wsh = 3;

  using namespace imageIO;
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
      
      cps.computeNormalMap( mapping, depthMap, normalMap, rc, 1, igammaC, igammaP, wsh);

      writeImage(normalMapFilepath, mp.getWidth(rc), mp.getHeight(rc), normalMap, EImageQuality::LOSSLESS, OutputFileColorSpace(EImageColorSpace::NO_CONVERSION));
    }
  }
  cps.deleteNormalMapping( mapping );
}




} // namespace depthMap
} // namespace aliceVision
