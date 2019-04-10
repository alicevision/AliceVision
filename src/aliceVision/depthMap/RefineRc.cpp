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
#include <aliceVision/imageIO/image.hpp>
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
    const int zborder = 2;

    StaticVector<IdValue> volumeBestIdVal(_width * _height);

    for(int i = 0; i < _volumeBestIdVal.size(); i++)
    {
        // float sim = depthSimMapFinal->dsm[i].y;
        // sim = std::min(sim,mp->simThr);
        const float sim = _sp.mp.simThr - 0.0001;
        const int id = _volumeBestIdVal[i].id;

        volumeBestIdVal[i] = IdValue(std::max(0, id), sim);
    }

    {
        DepthSimMap depthSimMap(_rc, _sp.mp, _scale, _step);
        _sp.getDepthSimMapFromBestIdVal(depthSimMap, _width, _height, volumeBestIdVal, _scale, _step, _rc, zborder, _depths);
        out_depthSimMapScale1Step1.add11(depthSimMap);
    }

    // set sim (y) to pixsize
    for(int y = 0; y < h11; ++y)
    {
        for(int x = 0; x < w11; ++x)
        {
            const Point3d p = _sp.mp.CArr[_rc] + (_sp.mp.iCamArr[_rc] * Point2d(static_cast<float>(x), static_cast<float>(y))).normalize() * out_depthSimMapScale1Step1._dsm[y * w11 + x].depth;

            if(_userTcOrPixSize)
                out_depthSimMapScale1Step1._dsm[y * w11 + x].sim = _sp.mp.getCamsMinPixelSize(p, _refineTCams);
            else
                out_depthSimMapScale1Step1._dsm[y * w11 + x].sim = _sp.mp.getCamPixelSize(p, _rc);
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

        // if(_sp.exportIntermediateResults)
        //     depthSimMapC->saveToImage(_sp.mp.getDepthMapsFolder() + "refine_photo_" + std::to_string(_sp.mp.getViewId(_rc)) + "_tc_" +
        //                                    std::to_string(_sp.mp.getViewId(tc)) + ".png", -2.0f);
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

void RefineRc::optimizeDepthSimMapCUDA(DepthSimMap& out_depthSimMapOptimized,
                                       const DepthSimMap& depthPixSizeMapVis,
                                       const DepthSimMap& depthSimMapPhoto)
{
    auto startTime = std::chrono::high_resolution_clock::now();

    if (_refineNiters == 0)
    {
        _depthSimMapOpt.add(depthSimMapPhoto);
        return;
    }

    int h11 = _sp.mp.getHeight(_rc);

    StaticVector<const StaticVector<DepthSim>*> dataMaps;
    dataMaps.push_back(&depthPixSizeMapVis._dsm); // link without ownership
    dataMaps.push_back(&depthSimMapPhoto._dsm); // link without ownership

    int nParts = 4;
    int hPart = h11 / nParts;
    for(int part = 0; part < nParts; part++)
    {
        int yFrom = part * hPart;
        int hPartAct = std::min(hPart, h11 - yFrom);
        _sp.cps.optimizeDepthSimMapGradientDescent(out_depthSimMapOptimized._dsm, dataMaps, _rc, _refineNSamplesHalf,
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

    DepthSimMap depthSimMapPhoto(_rc, _sp.mp, 1, 1);

    if (_sp.doRefineFuse)
    {
        refineAndFuseDepthSimMapCUDA(depthSimMapPhoto, depthPixSizeMapVis);
    }
    else
    {
        depthSimMapPhoto.initJustFromDepthMap(depthPixSizeMapVis, 1.0f);
    }

    if(_sp.doRefineOpt)
    {
        optimizeDepthSimMapCUDA(_depthSimMapOpt, depthPixSizeMapVis, depthSimMapPhoto);
    }
    else
    {
        _depthSimMapOpt.add(depthSimMapPhoto);
    }

    if(_sp.exportIntermediateResults)
    {
        // depthPixSizeMapVis.saveToImage(_sp.mp.getDepthMapsFolder() + "refine_" + std::to_string(viewId) + "_vis.png", 0.0f);
        // depthSimMapPhoto.saveToImage(_sp.mp.getDepthMapsFolder() + "refine_" + std::to_string(viewId) + "_photo.png", 0.0f);
        depthSimMapPhoto.save("_photo");
        // _depthSimMapOpt.saveToImage(_sp.mp.getDepthMapsFolder() + "refine_" + std::to_string(viewId) + "_opt.png", 0.0f);
    }

    mvsUtils::printfElapsedTime(tall, "Refine CUDA (rc: " + mvsUtils::num2str(_rc) + " / " + mvsUtils::num2str(_sp.mp.ncams) + ")");

    return true;
}

void RefineRc::writeDepthMap()
{
  _depthSimMapOpt.save();
}

void estimateAndRefineDepthMaps(mvsUtils::MultiViewParams& mp, const std::vector<int>& cams, int nbGPUs)
{
  const int numGpus = listCUDADevices(true);
  const int numCpuThreads = omp_get_num_procs();
  int numThreads = std::min(numGpus, numCpuThreads);

  ALICEVISION_LOG_INFO("# GPU devices: " << numGpus << ", # CPU threads: " << numCpuThreads);

  if(nbGPUs > 0)
      numThreads = nbGPUs;

  if(numThreads == 1)
  {
      // the GPU sorting is determined by an environment variable named CUDA_DEVICE_ORDER
      // possible values: FASTEST_FIRST (default) or PCI_BUS_ID
      const int cudaDeviceNo = 0;
      estimateAndRefineDepthMaps(cudaDeviceNo, mp, cams);
  }
  else
  {
      omp_set_num_threads(numThreads); // create as many CPU threads as there are CUDA devices
#pragma omp parallel
      {
          const int cpuThreadId = omp_get_thread_num();
          const int cudaDeviceNo = cpuThreadId % numThreads;
          const int rcFrom = cudaDeviceNo * (cams.size() / numThreads);
          int rcTo = (cudaDeviceNo + 1) * (cams.size() / numThreads);

          ALICEVISION_LOG_INFO("CPU thread " << cpuThreadId << " / " << numThreads << " uses CUDA device: " << cudaDeviceNo);

          if(cudaDeviceNo == numThreads - 1)
              rcTo = cams.size();

          std::vector<int> subcams;
          subcams.reserve(cams.size());

          for(int rc = rcFrom; rc < rcTo; rc++)
              subcams.push_back(cams[rc]);

          estimateAndRefineDepthMaps(cpuThreadId, mp, subcams);
      }
  }
}

void estimateAndRefineDepthMaps(int cudaDeviceNo, mvsUtils::MultiViewParams& mp, const std::vector<int>& cams)
{
  const int fileScale = 1; // input images scale (should be one)
  int sgmScale = mp.userParams.get<int>("semiGlobalMatching.scale", -1);
  int sgmStep = mp.userParams.get<int>("semiGlobalMatching.step", -1);

  if(sgmScale == -1)
  {
      // compute the number of scales that will be used in the plane sweeping.
      // the highest scale should have a minimum resolution of 700x550.
      const int width = mp.getMaxImageWidth();
      const int height = mp.getMaxImageHeight();
      const int scaleTmp = computeStep(mp, fileScale, (width > height ? 700 : 550), (width > height ? 550 : 700));

      sgmScale = std::min(2, scaleTmp);
      sgmStep = computeStep(mp, fileScale * sgmScale, (width > height ? 700 : 550), (width > height ? 550 : 700));

      ALICEVISION_LOG_INFO("Plane sweeping parameters:\n"
                           "\t- scale: " << sgmScale << "\n"
                           "\t- step: " << sgmStep);
  }

  const int bandType = 0;

  // load images from files into RAM
  mvsUtils::ImagesCache ic(mp, bandType);
  // load stuff on GPU memory and creates multi-level images and computes gradients
  PlaneSweepingCuda cps(cudaDeviceNo, ic, mp, sgmScale);
  // init plane sweeping parameters
  SemiGlobalMatchingParams sp(mp, cps);

  for(const int rc : cams)
  {
      RefineRc sgmRefineRc(rc, sgmScale, sgmStep, sp);

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




void computeNormalMaps(int CUDADeviceNo, mvsUtils::MultiViewParams& mp, const StaticVector<int>& cams)
{
  const float igammaC = 1.0f;
  const float igammaP = 1.0f;
  const int bandType = 0;
  const int wsh = 3;

  mvsUtils::ImagesCache ic(mp, bandType);
  PlaneSweepingCuda cps(CUDADeviceNo, ic, mp, 1);

  for(const int rc : cams)
  {
    const std::string normalMapFilepath = getFileNameFromIndex(mp, rc, mvsUtils::EFileType::normalMap, 0);

    if(!mvsUtils::FileExists(normalMapFilepath))
    {
      StaticVector<float> depthMap;
      int w = 0;
      int h = 0;
      imageIO::readImage(getFileNameFromIndex(mp, rc, mvsUtils::EFileType::depthMap, 0), w, h, depthMap.getDataWritable());

      StaticVector<Color> normalMap;
      normalMap.resize(mp.getWidth(rc) * mp.getHeight(rc));
      
      cps.computeNormalMap(&depthMap, &normalMap, rc, 1, igammaC, igammaP, wsh);

      imageIO::writeImage(normalMapFilepath, mp.getWidth(rc), mp.getHeight(rc), normalMap.getDataWritable(), imageIO::EImageQuality::LOSSLESS);
    }
  }
}

void computeNormalMaps(mvsUtils::MultiViewParams& mp, const StaticVector<int>& cams)
{
  const int nbGPUs = listCUDADevices(true);
  const int nbCPUThreads = omp_get_num_procs();

  ALICEVISION_LOG_INFO("Number of GPU devices: " << nbGPUs << ", number of CPU threads: " << nbCPUThreads);

  const int nbGPUsToUse = mp.userParams.get<int>("refineRc.num_gpus_to_use", 1);
  int nbThreads = std::min(nbGPUs, nbCPUThreads);

  if(nbGPUsToUse > 0)
  {
    nbThreads = nbGPUsToUse;
  }

  if(nbThreads == 1)
  {
    const int CUDADeviceNo = 0;
    computeNormalMaps(CUDADeviceNo, mp, cams);
  }
  else
  {
    omp_set_num_threads(nbThreads); // create as many CPU threads as there are CUDA devices
#pragma omp parallel
    {
      const int cpuThreadId = omp_get_thread_num();
      const int CUDADeviceNo = cpuThreadId % nbThreads;

      ALICEVISION_LOG_INFO("CPU thread " << cpuThreadId << " (of " << nbThreads << ") uses CUDA device: " << CUDADeviceNo);

      const int nbCamsPerThread = (cams.size() / nbThreads);
      const int rcFrom = CUDADeviceNo * nbCamsPerThread;
      int rcTo = (CUDADeviceNo + 1) * nbCamsPerThread;

      if(CUDADeviceNo == nbThreads - 1)
      {
        rcTo = cams.size();
      }

      StaticVector<int> subcams;
      subcams.reserve(cams.size());

      for(int rc = rcFrom; rc < rcTo; ++rc)
      {
        subcams.push_back(cams[rc]);
      }

      computeNormalMaps(CUDADeviceNo, mp, subcams);
    }
  }
}





} // namespace depthMap
} // namespace aliceVision
