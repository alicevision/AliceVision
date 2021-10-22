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
#include <aliceVision/depthMap/cuda/planeSweeping/plane_sweeping_cuda.hpp>

#include <boost/filesystem.hpp>

#include <nvtx3/nvToolsExt.h>

#include <thread>
#include <chrono>

namespace aliceVision {
namespace depthMap {

namespace bfs = boost::filesystem;

RefineRc::RefineRc(int rc, int scale, int step, SemiGlobalMatchingParams& sp)
    : SemiGlobalMatchingRc(rc, scale, step, sp)
    , _depthSimMapOpt(_rc, _sp.mp, 1, 1)
{
    nvtxRangePush(__func__);
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
    nvtxRangePop();
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
    nvtxRangePush(__func__);
    const int w11 = _sp.mp.getWidth(_rc);
    const int h11 = _sp.mp.getHeight(_rc);

    nvtxRangePush("initFromSmaller");
    out_depthSimMapScale1Step1.initFromSmaller(_sgmDepthSimMap);
    nvtxRangePop();

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

void RefineRc::getDepthPixSizeMapFromSGM(std::vector<CudaDeviceMemoryPitched<float2, 2>>& dataMaps_d,
    cudaStream_t stream)
{
    // Here both scale and step of dataMaps_d buffers are assumed to be one!
    nvtxRangePush(__func__);
    const int w11 = _sp.mp.getWidth(_rc);
    const int h11 = _sp.mp.getHeight(_rc);

    float const ratio = 1.0f / float(_sgmDepthSimMap._scale * _sgmDepthSimMap._step);

    nvtxRangePush("initFromSmaller");
    // Copy _sgmDepthSimMap to device
    cudaMemcpy2DAsync(dataMaps_d[1].getBuffer(), dataMaps_d[1].getPitch(), _sgmDepthSimMap._dsm.getData().data(),
        _sgmDepthSimMap._w * sizeof(DepthSim), _sgmDepthSimMap._w * sizeof(DepthSim), _sgmDepthSimMap._h,
        cudaMemcpyHostToDevice,
        0); // TODO: Add error check
    ps_initFromSmaller(dataMaps_d[0], w11, h11, dataMaps_d[1], _sgmDepthSimMap._w,
        _sgmDepthSimMap._h, ratio, 0, _sp.mp.verbose);
    nvtxRangePop();

    // set sim (y) to pixsize
    if (_userTcOrPixSize)
    {
        for (int i = 0; i< _refineTCams.size(); ++i)
        {
            int tc = _refineTCams[i];
            int rc_idx = _sp.cps.addCam(_rc, 1);
            int tc_idx = _sp.cps.addCam(tc, 1);
            if (i == 0)
                ps_setPixSizesInit(_sp.cps._cams[rc_idx].param_dev.i, dataMaps_d[0], w11, h11,
                    _sp.cps._cams[tc_idx].param_dev.i, stream, _sp.mp.verbose);
            else
                ps_setPixSizesMin(_sp.cps._cams[rc_idx].param_dev.i, dataMaps_d[0], w11, h11,
                    _sp.cps._cams[tc_idx].param_dev.i, stream, _sp.mp.verbose);
        }
    }
    else
    {
        int rc_idx = _sp.cps.addCam(_rc, 1);
        ps_setPixSizesInit(_sp.cps._cams[rc_idx].param_dev.i, dataMaps_d[0], w11, h11,
            _sp.cps._cams[rc_idx].param_dev.i, stream, _sp.mp.verbose);
    }

    nvtxRangePop();
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
    // Scale and step seem to be always one. This is assumed for the optimizations applied.
    // If this is not the case, branch between the original, unoptimized version and the optimized one,
    // depending on the input maps scales and steps.
    assert(out_depthSimMapFused._scale == 1);
    assert(out_depthSimMapFused._step == 1);
    assert(depthPixSizeMapVis._scale == 1);
    assert(depthPixSizeMapVis._step == 1);

    nvtxRangePush(__func__);
    auto startTime = std::chrono::high_resolution_clock::now();

    int w11 = _sp.mp.getWidth(_rc);
    int h11 = _sp.mp.getHeight(_rc);

    RcTc prt(_sp.mp, _sp.cps);

    // Allocate CUDA device depth sim maps
    std::vector<CudaDeviceMemoryPitched<float2, 2>> dataMaps_d;
    dataMaps_d.reserve(_refineTCams.size() + 1);
    std::vector<const float2*, PinnedMemoryAllocator<const float2*>> dataMapsPtrs_h;
    dataMapsPtrs_h.reserve(_refineTCams.size() + 1);
    for(int c = 0; c < _refineTCams.size() + 1; c++)
    {
        dataMaps_d.emplace_back(CudaSize<2>(depthPixSizeMapVis._w, depthPixSizeMapVis._h));
        dataMapsPtrs_h.push_back(dataMaps_d[c].getBuffer());
    }

    CudaDeviceMemory<const float2*> dataMapsPtrs_d(dataMaps_d.size());
    cudaMemcpyAsync(dataMapsPtrs_d.getBuffer(), dataMapsPtrs_h.data(), dataMapsPtrs_h.size() * sizeof(const float2*),
                    cudaMemcpyHostToDevice, 0); // streams[nextStreamIdx]); // TODO: Add error check

    // Copy reference depth sim data
    cudaMemcpy2DAsync(dataMaps_d[0].getBuffer(), dataMaps_d[0].getPitch(), depthPixSizeMapVis._dsm.getData().data(),
                      depthPixSizeMapVis._w * sizeof(DepthSim), depthPixSizeMapVis._w * sizeof(DepthSim),
                      depthPixSizeMapVis._h, cudaMemcpyHostToDevice,
                      0); // streams[nextStreamIdx]); // TODO: Add error check

    const int scale = 1;

    for(int c = 0; c < _refineTCams.size(); c++)
    {
        int tc = _refineTCams[c];
  
        prt.refineRcTcDepthSimMap(_userTcOrPixSize, dataMaps_d[0], dataMaps_d[c + 1], _rc, tc, _nbDepthsToRefine,
                                  _refineWsh, _refineGammaC, _refineGammaP, depthPixSizeMapVis._w,
                                  depthPixSizeMapVis._h, 0); // streams[nextStreamIdx]);

        if(_sp.exportIntermediateResults)
        {
            // TODO
            // depthSimMapC->save("_refine_tc_" + std::to_string(tc) + "_" + std::to_string(_sp.mp.getViewId(tc)));
            //     // depthSimMapC->saveToImage(_sp.mp.getDepthMapsFolder() + "refine_photo_" +
            //     std::to_string(_sp.mp.getViewId(_rc)) + "_tc_" +
            //     //                           std::to_string(_sp.mp.getViewId(tc)) + ".png", -2.0f);
        }
    }

    _sp.cps.fuseDepthSimMapsGaussianKernelVoting(w11, h11, dataMaps_d, dataMapsPtrs_d, _refineNSamplesHalf,
                                                 _nbDepthsToRefine, _refineSigma);

    cudaMemcpy2D(out_depthSimMapFused._dsm.getDataWritable().data(), out_depthSimMapFused._w * sizeof(DepthSim),
                 dataMaps_d[1].getBuffer(), dataMaps_d[1].getPitch(), out_depthSimMapFused._w * sizeof(DepthSim),
                 out_depthSimMapFused._h, cudaMemcpyDeviceToHost); // TODO: Add error check

    nvtxRangePop();
    ALICEVISION_LOG_INFO(
        "==== refineAndFuseDepthSimMapCUDA done in : "
        << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startTime)
               .count()
        << "ms.");
}

void RefineRc::refineAndFuseDepthSimMapCUDA(DepthSimMap& out_depthSimMapFused,
                                            std::vector<CudaDeviceMemoryPitched<float2, 2>>& dataMaps_d,
                                            CudaDeviceMemory<const float2*>& dataMapsPtrs_d)
{
    // Scale and step seem to be always one. This is assumed for the optimizations applied.
    // If this is not the case, branch between the original, unoptimized version and the optimized one,
    // depending on the input maps scales and steps.
    assert(out_depthSimMapFused._scale == 1);
    assert(out_depthSimMapFused._step == 1);
    //assert(depthPixSizeMapVis._scale == 1);
    //assert(depthPixSizeMapVis._step == 1);

    nvtxRangePush(__func__);
    auto startTime = std::chrono::high_resolution_clock::now();

    int w11 = _sp.mp.getWidth(_rc);
    int h11 = _sp.mp.getHeight(_rc);

    RcTc prt(_sp.mp, _sp.cps);

    const int scale = 1;

    for(int c = 0; c < _refineTCams.size(); c++)
    {
        int tc = _refineTCams[c];

        prt.refineRcTcDepthSimMap(_userTcOrPixSize, dataMaps_d[0], dataMaps_d[c + 1], _rc, tc, _nbDepthsToRefine,
                                  _refineWsh, _refineGammaC, _refineGammaP, w11,
                                  h11, 0); // streams[nextStreamIdx]);

        if(_sp.exportIntermediateResults)
        {
            // TODO !
            // depthSimMapC->save("_refine_tc_" + std::to_string(tc) + "_" + std::to_string(_sp.mp.getViewId(tc)));
            //     // depthSimMapC->saveToImage(_sp.mp.getDepthMapsFolder() + "refine_photo_" +
            //     std::to_string(_sp.mp.getViewId(_rc)) + "_tc_" +
            //     //                           std::to_string(_sp.mp.getViewId(tc)) + ".png", -2.0f);
        }
    }

    _sp.cps.fuseDepthSimMapsGaussianKernelVoting(w11, h11, dataMaps_d, dataMapsPtrs_d, _refineNSamplesHalf,
                                                 _nbDepthsToRefine, _refineSigma);

    cudaMemcpy2D(out_depthSimMapFused._dsm.getDataWritable().data(), out_depthSimMapFused._w * sizeof(DepthSim),
                 dataMaps_d[1].getBuffer(), dataMaps_d[1].getPitch(), out_depthSimMapFused._w * sizeof(DepthSim),
                 out_depthSimMapFused._h, cudaMemcpyDeviceToHost); // TODO: Add error check

    nvtxRangePop();
}

void RefineRc::optimizeDepthSimMapCUDA(DepthSimMap& out_depthSimMapOptimized, // optimized
                                       const DepthSimMap& depthPixSizeMapVis, // SGM
                                       const DepthSimMap& depthSimMapPhoto) // refined
{
    nvtxRangePush(__func__);
    auto startTime = std::chrono::high_resolution_clock::now();

    if (_refineNiters == 0)
    {
        _depthSimMapOpt.init(depthSimMapPhoto);
        nvtxRangePop();
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
    nvtxRangePop();
}

bool RefineRc::refinerc(bool checkIfExists)
{
    nvtxRangePush(__func__);
    const IndexT viewId = _sp.mp.getViewId(_rc);

    ALICEVISION_LOG_DEBUG("Refine CUDA (rc: " << (_rc + 1) << " / " << _sp.mp.ncams << ")");

    // generate default depthSimMap if rc has no tcam
    if(_refineTCams.empty() || _depths.empty())
    {
        ALICEVISION_LOG_INFO("No T cameras for camera rc=" << _rc);
        nvtxRangePop();
        return true;
    }

    if(checkIfExists && (mvsUtils::FileExists(_sp.getREFINE_opt_simMapFileName(viewId, 1, 1))))
    {
        ALICEVISION_LOG_INFO("Already computed: " << _sp.getREFINE_opt_simMapFileName(viewId, 1, 1));
        nvtxRangePop();
        return false;
    }

    long tall = clock();

    // Allocate CUDA device depth sim maps
    const int numDeviceBuffers = std::max(_refineTCams.size() + 1, 2);
    std::vector<CudaDeviceMemoryPitched<float2, 2>> dataMaps_d;
    dataMaps_d.reserve(numDeviceBuffers);
    std::vector<const float2*, PinnedMemoryAllocator<const float2*>> dataMapsPtrs_h;
    dataMapsPtrs_h.reserve(numDeviceBuffers);
    for(int c = 0; c < numDeviceBuffers; c++)
    {
        dataMaps_d.emplace_back(CudaSize<2>(_sp.mp.getWidth(_rc), _sp.mp.getHeight(_rc)));
        dataMapsPtrs_h.push_back(dataMaps_d[c].getBuffer());
    }

    CudaDeviceMemory<const float2*> dataMapsPtrs_d(dataMaps_d.size());
    cudaMemcpyAsync(dataMapsPtrs_d.getBuffer(), dataMapsPtrs_h.data(), dataMapsPtrs_h.size() * sizeof(const float2*),
                    cudaMemcpyHostToDevice, 0); // TODO: Add error check

    //// Copy reference depth sim data
    // cudaMemcpy2DAsync(dataMaps_d[0].getBuffer(), dataMaps_d[0].getPitch(), depthPixSizeMapVis._dsm.getData().data(),
    //                  depthPixSizeMapVis._w * sizeof(DepthSim), depthPixSizeMapVis._w * sizeof(DepthSim),
    //                  depthPixSizeMapVis._h, cudaMemcpyHostToDevice,
    //                  0); // streams[nextStreamIdx]); // TODO: Add error check

    //DepthSimMap depthPixSizeMapVis(_rc, _sp.mp, 1, 1);
   // getDepthPixSizeMapFromSGM(depthPixSizeMapVis);

    getDepthPixSizeMapFromSGM(dataMaps_d, 0);
    
    
    if(_sp.exportIntermediateResults)
    {
        //depthPixSizeMapVis.save("_sgmRescaled"); // TODO!
    }

    DepthSimMap depthSimMapPhoto(_rc, _sp.mp, 1, 1);

    if(_sp.doRefineFuse)
    {
       // refineAndFuseDepthSimMapCUDA(depthSimMapPhoto, depthPixSizeMapVis);
        refineAndFuseDepthSimMapCUDA(depthSimMapPhoto, dataMaps_d, dataMapsPtrs_d);
    }
    else
    {
        DepthSimMap depthPixSizeMapVis(_rc, _sp.mp, 1, 1);
        cudaMemcpy2DAsync(depthPixSizeMapVis._dsm.getDataWritable().data(), depthPixSizeMapVis._w * sizeof(DepthSim),
                          dataMaps_d[0].getBuffer(), dataMaps_d[0].getPitch(), depthPixSizeMapVis._w * sizeof(DepthSim),
                          depthPixSizeMapVis._h, cudaMemcpyDeviceToHost, 0); // TODO: Add error check

        depthSimMapPhoto.initJustFromDepthMap(depthPixSizeMapVis, 1.0f);
    }

    if(_sp.doRefineOpt && _refineNiters != 0)
    {
        if(_sp.exportIntermediateResults)
        {
            // depthPixSizeMapVis.saveToImage(_sp.mp.getDepthMapsFolder() + "refine_" + std::to_string(viewId) +
            // "_vis.png", 0.0f); depthSimMapPhoto.saveToImage(_sp.mp.getDepthMapsFolder() + "refine_" +
            // std::to_string(viewId) + "_photo.png", 0.0f);
            depthSimMapPhoto.save("_photo");
            // _depthSimMapOpt.saveToImage(_sp.mp.getDepthMapsFolder() + "refine_" + std::to_string(viewId) +
            // "_opt.png", 0.0f);
        }

        DepthSimMap depthPixSizeMapVis(_rc, _sp.mp, 1, 1);

        cudaMemcpy2DAsync(depthPixSizeMapVis._dsm.getDataWritable().data(), depthPixSizeMapVis._w * sizeof(DepthSim),
                          dataMaps_d[0].getBuffer(), dataMaps_d[0].getPitch(), depthPixSizeMapVis._w * sizeof(DepthSim),
                          depthPixSizeMapVis._h, cudaMemcpyDeviceToHost, 0); // TODO: Add error check
        cudaDeviceSynchronize();
        optimizeDepthSimMapCUDA(_depthSimMapOpt, depthPixSizeMapVis, depthSimMapPhoto);
    }
    else
    {
        _depthSimMapOpt.init(depthSimMapPhoto);
    }

    mvsUtils::printfElapsedTime(tall, "Refine CUDA (rc: " + mvsUtils::num2str(_rc) + " / " + mvsUtils::num2str(_sp.mp.ncams) + ")");

    nvtxRangePop();
    return true;
}

void RefineRc::writeDepthMap()
{
  _depthSimMapOpt.save();
}

void estimateAndRefineDepthMaps(int cudaDeviceIndex, mvsUtils::MultiViewParams& mp, const std::vector<int>& cams)
{
    nvtxRangePush(__func__);
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

  // Preload all data, currently just a hack to get an idea of the performance gain, should be implemented by proper
  // queue that allways precaches the next N images

  nvtxRangePush((__func__ + std::string(": ") + "Preload all images").c_str());
  for(const int rc : cams)
  {
      sp.cps._ic.refreshImage_async(rc);
      const int nbNearestCams = sp.mp.userParams.get<int>("semiGlobalMatching.maxTCams", 10);
      sp.cps._ic.refreshImages_async(sp.mp.findNearestCamsFromLandmarks(rc, nbNearestCams).getData());
  }
  nvtxRangePop();

  for(const int rc : cams)
  {
      nvtxRangePush(("estimateAndRefineDepthMaps: Loop iteration for camera " + std::to_string(rc) + " / " +
                     std::to_string(cams.size()))
                        .c_str());
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
      nvtxRangePop();
  }
  nvtxRangePop();
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
