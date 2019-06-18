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

RefineRc::RefineRc(int rc, int scale, int step, SemiGlobalMatchingParams* sp)
    : SemiGlobalMatchingRc(rc, scale, step, sp)
{
    const int nbNearestCams = sp->mp->userParams.get<int>("refineRc.maxTCams", 6);
    _refineTCams  = sp->mp->findNearestCamsFromLandmarks(rc, nbNearestCams);
    _userTcOrPixSize = _sp->mp->userParams.get<bool>("refineRc.useTcOrRcPixSize", false);
    _nbDepthsToRefine = _sp->mp->userParams.get<int>("refineRc.ndepthsToRefine", 31);
    _refineWsh = _sp->mp->userParams.get<int>("refineRc.wsh", 3);
    _refineNiters = _sp->mp->userParams.get<int>("refineRc.niters", 100);
    _refineNSamplesHalf = _sp->mp->userParams.get<int>("refineRc.nSamplesHalf", 150);
    _refineSigma  = static_cast<float>(_sp->mp->userParams.get<double>("refineRc.sigma", 15.0));
    _refineGammaC = static_cast<float>(_sp->mp->userParams.get<double>("refineRc.gammaC", 15.5));
    _refineGammaP = static_cast<float>(_sp->mp->userParams.get<double>("refineRc.gammaP", 8.0));
}

RefineRc::~RefineRc()
{
  delete _depthSimMapOpt;
}

void RefineRc::preloadSgmTcams_async()
{
  for(int tc : _sgmTCams.getData())
    _sp->cps._ic.refreshData_async(tc);
}

DepthSimMap* RefineRc::getDepthPixSizeMapFromSGM()
{
    const int w11 = _sp->mp->getWidth(_rc);
    const int h11 = _sp->mp->getHeight(_rc);
    const int zborder = 2;

    StaticVector<IdValue> volumeBestIdVal;
    volumeBestIdVal.reserve(_width * _height);

    for(int i = 0; i < _volumeBestIdVal->size(); i++)
    {
        // float sim = (*depthSimMapFinal->dsm)[i].y;
        // sim = std::min(sim,mp->simThr);
        const float sim = _sp->mp->simThr - 0.0001;
        const int id = (*_volumeBestIdVal)[i].id;

        if(id > 0)
          volumeBestIdVal.push_back(IdValue(id, sim));
        else
          volumeBestIdVal.push_back(IdValue(0, sim));
    }

    DepthSimMap* depthSimMapScale1Step1 = new DepthSimMap(_rc, _sp->mp, 1, 1);
    {
        DepthSimMap* depthSimMap = _sp->getDepthSimMapFromBestIdVal(_width, _height, &volumeBestIdVal, _scale, _step, _rc, zborder, _depths);
        depthSimMapScale1Step1->add11(depthSimMap);
        delete depthSimMap;
    }

    // set sim (y) to pixsize
    for(int y = 0; y < h11; ++y)
    {
        for(int x = 0; x < w11; ++x)
        {
            Point3d p = _sp->mp->CArr[_rc] + (_sp->mp->iCamArr[_rc] * Point2d(static_cast<float>(x), static_cast<float>(y))).normalize() * (*depthSimMapScale1Step1->dsm)[y * w11 + x].depth;

            if(_userTcOrPixSize)
                (*depthSimMapScale1Step1->dsm)[y * w11 + x].sim = _sp->mp->getCamsMinPixelSize(p, _refineTCams);
            else
                (*depthSimMapScale1Step1->dsm)[y * w11 + x].sim = _sp->mp->getCamPixelSize(p, _rc);
        }
    }

    return depthSimMapScale1Step1;
}

DepthSimMap* RefineRc::refineAndFuseDepthSimMapCUDA(DepthSimMap* depthPixSizeMapVis)
{
    int w11 = _sp->mp->getWidth(_rc);
    int h11 = _sp->mp->getHeight(_rc);

    StaticVector<DepthSimMap*>* dataMaps = new StaticVector<DepthSimMap*>();
    dataMaps->reserve(_refineTCams.size() + 1);
    dataMaps->push_back(depthPixSizeMapVis); //!!DO NOT ERASE!!!

    const int scale = 1;

    for(int c = 0; c < _refineTCams.size(); c++)
    {
        int tc = _refineTCams[c];

        DepthSimMap* depthSimMapC = new DepthSimMap(_rc, _sp->mp, scale, 1);
        StaticVector<float>* depthMap = depthPixSizeMapVis->getDepthMap();
        depthSimMapC->initJustFromDepthMap(depthMap, 1.0f);
        delete depthMap;

        _sp->prt->refineRcTcDepthSimMap(_userTcOrPixSize, depthSimMapC, _rc, tc, _nbDepthsToRefine, _refineWsh, _refineGammaC, _refineGammaP,
                                       0.0f);

        dataMaps->push_back(depthSimMapC);

        if(_sp->exportIntermediateResults)
            depthSimMapC->saveToImage(_sp->mp->getDepthMapsFolder() + "refine_photo_" + std::to_string(_sp->mp->getViewId(_rc)) + "_tc_" +
                                           std::to_string(_sp->mp->getViewId(tc)) + ".png", -2.0f);
    }

    // in order to fit into GPU memory
    DepthSimMap* depthSimMapFused = new DepthSimMap(_rc, _sp->mp, scale, 1);

    int nhParts = 4;
    int hPartHeightGlob = h11 / nhParts;
    for(int hPart = 0; hPart < nhParts; hPart++)
    {
        int hPartHeight = std::min(h11, (hPart + 1) * hPartHeightGlob) - hPart * hPartHeightGlob;

        // vector of one depthSimMap tile per Tc
        StaticVector<StaticVector<DepthSim>*>* dataMapsHPart =
            new StaticVector<StaticVector<DepthSim>*>();
        dataMapsHPart->reserve(dataMaps->size());
        for(int i = 0; i < dataMaps->size(); i++) // iterate over Tc cameras
        {
            StaticVector<DepthSim>* dataMapHPart = new StaticVector<DepthSim>();
            dataMapHPart->reserve(w11 * hPartHeight);
            dataMapHPart->resize(w11 * hPartHeight);

#pragma omp parallel for
            for(int y = 0; y < hPartHeight; y++)
            {
                for(int x = 0; x < w11; x++)
                {
                    (*dataMapHPart)[y * w11 + x] = (*(*dataMaps)[i]->dsm)[(y + hPart * hPartHeightGlob) * w11 + x];
                }
            }

            dataMapsHPart->push_back(dataMapHPart);
        }

        StaticVector<DepthSim>* depthSimMapFusedHPart = new StaticVector<DepthSim>();
        depthSimMapFusedHPart->reserve(w11 * hPartHeight);
        depthSimMapFusedHPart->resize_with(w11 * hPartHeight, DepthSim(-1.0f, 1.0f));

        _sp->cps.fuseDepthSimMapsGaussianKernelVoting(w11, hPartHeight, depthSimMapFusedHPart, dataMapsHPart,
                                                      _refineNSamplesHalf, _nbDepthsToRefine, _refineSigma);

#pragma omp parallel for
        for(int y = 0; y < hPartHeight; y++)
        {
            for(int x = 0; x < w11; x++)
            {
                (*depthSimMapFused->dsm)[(y + hPart * hPartHeightGlob) * w11 + x] =
                    (*depthSimMapFusedHPart)[y * w11 + x];
            }
        }

        delete depthSimMapFusedHPart;
        deleteArrayOfArrays<DepthSim>(&dataMapsHPart);
    }

    (*dataMaps)[0] = nullptr; // it is input dsmap we dont want to delete it
    for(int c = 0; c < _refineTCams.size(); c++)
    {
        delete(*dataMaps)[c + 1];
    }
    delete dataMaps;

    return depthSimMapFused;
}

DepthSimMap* RefineRc::optimizeDepthSimMapCUDA(DepthSimMap* depthPixSizeMapVis,
                                                      DepthSimMap* depthSimMapPhoto)
{
    int h11 = _sp->mp->getHeight(_rc);

    StaticVector<DepthSimMap*>* dataMaps = new StaticVector<DepthSimMap*>();
    dataMaps->reserve(2);
    dataMaps->push_back(depthPixSizeMapVis); //!!DO NOT ERASE!!!
    dataMaps->push_back(depthSimMapPhoto);   //!!DO NOT ERASE!!!

    DepthSimMap* depthSimMapOptimized = new DepthSimMap(_rc, _sp->mp, 1, 1);

    {
        StaticVector<StaticVector<DepthSim>*>* dataMapsPtrs = new StaticVector<StaticVector<DepthSim>*>();
        dataMapsPtrs->reserve(dataMaps->size());
        for(int i = 0; i < dataMaps->size(); i++)
        {
            dataMapsPtrs->push_back((*dataMaps)[i]->dsm);
        }

        int nParts = 4;
        int hPart = h11 / nParts;
        for(int part = 0; part < nParts; part++)
        {
            int yFrom = part * hPart;
            int hPartAct = std::min(hPart, h11 - yFrom);
            _sp->cps.optimizeDepthSimMapGradientDescent(depthSimMapOptimized->dsm, dataMapsPtrs, _rc, _refineNSamplesHalf,
                                                        _nbDepthsToRefine, _refineSigma, _refineNiters, yFrom, hPartAct);
        }

        for(int i = 0; i < dataMaps->size(); i++)
        {
            (*dataMapsPtrs)[i] = nullptr;
        }
        delete dataMapsPtrs;
    }

    (*dataMaps)[0] = nullptr; // it is input dsmap we dont want to delete it
    (*dataMaps)[1] = nullptr; // it is input dsmap we dont want to delete it
    delete dataMaps;

    return depthSimMapOptimized;
}

bool RefineRc::refinerc(bool checkIfExists)
{
    const IndexT viewId = _sp->mp->getViewId(_rc);

    if(_sp->mp->verbose)
        ALICEVISION_LOG_DEBUG("Refine CUDA (rc: " << (_rc + 1) << " / " << _sp->mp->ncams << ")");

    // generate default depthSimMap if rc has no tcam
    if(_refineTCams.size() == 0 || _depths.empty())
    {
        _depthSimMapOpt = new DepthSimMap(_rc, _sp->mp, 1, 1);
        return true;
    }

    if(checkIfExists && (mvsUtils::FileExists(_sp->getREFINE_opt_simMapFileName(viewId, 1, 1))))
    {
        ALICEVISION_LOG_INFO("Already computed: " + _sp->getREFINE_opt_simMapFileName(viewId, 1, 1));
        return false;
    }

    long tall = clock();

    DepthSimMap* depthPixSizeMapVis = getDepthPixSizeMapFromSGM();
    DepthSimMap* depthSimMapPhoto = refineAndFuseDepthSimMapCUDA(depthPixSizeMapVis);

    if(_sp->doRefineRc)
    {
        _depthSimMapOpt = optimizeDepthSimMapCUDA(depthPixSizeMapVis, depthSimMapPhoto);
    }
    else
    {
        _depthSimMapOpt = new DepthSimMap(_rc, _sp->mp, 1, 1);
        _depthSimMapOpt->add(depthSimMapPhoto);
    }

    if(_sp->exportIntermediateResults)
    {
      depthPixSizeMapVis->saveToImage(_sp->mp->getDepthMapsFolder() + "refine_" + std::to_string(viewId) + "_vis.png", 0.0f);
      depthSimMapPhoto->saveToImage(_sp->mp->getDepthMapsFolder() + "refine_" + std::to_string(viewId) + "_photo.png", 0.0f);
      depthSimMapPhoto->saveRefine(_rc, _sp->getREFINE_photo_depthMapFileName(viewId, 1, 1), _sp->getREFINE_photo_simMapFileName(viewId, 1, 1));
      _depthSimMapOpt->saveToImage(_sp->mp->getDepthMapsFolder() + "refine_" + std::to_string(viewId) + "_opt.png", 0.0f);
      _depthSimMapOpt->saveRefine(_rc, _sp->getREFINE_opt_depthMapFileName(viewId, 1, 1), _sp->getREFINE_opt_simMapFileName(viewId, 1, 1));
    }

    mvsUtils::printfElapsedTime(tall, "Refine CUDA (rc: " + mvsUtils::num2str(_rc) + " / " + mvsUtils::num2str(_sp->mp->ncams) + ")");

    delete depthPixSizeMapVis;
    delete depthSimMapPhoto;
    return true;
}

void RefineRc::writeDepthMap()
{
  _depthSimMapOpt->save(_rc, _refineTCams);
}

void estimateAndRefineDepthMaps(mvsUtils::MultiViewParams* mp, const std::vector<int>& cams, int nbGPUs)
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

void estimateAndRefineDepthMaps(int cudaDeviceNo, mvsUtils::MultiViewParams* mp, const std::vector<int>& cams)
{
  const int fileScale = 1; // input images scale (should be one)
  int sgmScale = mp->userParams.get<int>("semiGlobalMatching.scale", -1);
  int sgmStep = mp->userParams.get<int>("semiGlobalMatching.step", -1);

  if(sgmScale == -1)
  {
      // compute the number of scales that will be used in the plane sweeping.
      // the highest scale should have a minimum resolution of 700x550.
      const int width = mp->getMaxImageWidth();
      const int height = mp->getMaxImageHeight();
      const int scaleTmp = computeStep(mp, fileScale, (width > height ? 700 : 550), (width > height ? 550 : 700));

      sgmScale = std::min(2, scaleTmp);
      sgmStep = computeStep(mp, fileScale * sgmScale, (width > height ? 700 : 550), (width > height ? 550 : 700));

      ALICEVISION_LOG_INFO("Plane sweeping parameters:\n"
                           "\t- scale: " << sgmScale << "\n"
                           "\t- step: " << sgmStep);
  }

  const int bandType = 0;

  // load images from files into RAM
  mvsUtils::ImagesCache ic(mp, bandType, imageIO::EImageColorSpace::LINEAR);
  // load stuff on GPU memory and creates multi-level images and computes gradients
  PlaneSweepingCuda cps(cudaDeviceNo, ic, mp, sgmScale);
  // init plane sweeping parameters
  SemiGlobalMatchingParams sp(mp, cps);

  for(const int rc : cams)
  {
      RefineRc sgmRefineRc(rc, sgmScale, sgmStep, &sp);

      sgmRefineRc.preloadSgmTcams_async();

      ALICEVISION_LOG_INFO("Estimate depth map, view id: " << mp->getViewId(rc));
      sgmRefineRc.sgmrc();

      ALICEVISION_LOG_INFO("Refine depth map, view id: " << mp->getViewId(rc));
      sgmRefineRc.refinerc();

      // write results
      sgmRefineRc.writeDepthMap();
  }
}




void computeNormalMaps(int CUDADeviceNo, mvsUtils::MultiViewParams* mp, const StaticVector<int>& cams)
{
  const float igammaC = 1.0f;
  const float igammaP = 1.0f;
  const int bandType = 0;
  const int wsh = 3;

  mvsUtils::ImagesCache ic(mp, bandType, imageIO::EImageColorSpace::LINEAR);
  PlaneSweepingCuda cps(CUDADeviceNo, ic, mp, 1);

  for(const int rc : cams)
  {
    const std::string normalMapFilepath = getFileNameFromIndex(mp, rc, mvsUtils::EFileType::normalMap, 0);

    if(!mvsUtils::FileExists(normalMapFilepath))
    {
      StaticVector<float> depthMap;
      int w = 0;
      int h = 0;
      imageIO::readImage(getFileNameFromIndex(mp, rc, mvsUtils::EFileType::depthMap, 0), w, h, depthMap.getDataWritable(), imageIO::EImageColorSpace::NO_CONVERSION);

      StaticVector<Color> normalMap;
      normalMap.resize(mp->getWidth(rc) * mp->getHeight(rc));
      
      cps.computeNormalMap(&depthMap, &normalMap, rc, 1, igammaC, igammaP, wsh);

      imageIO::writeImage(normalMapFilepath, mp->getWidth(rc), mp->getHeight(rc), normalMap.getDataWritable(), imageIO::EImageQuality::LOSSLESS, imageIO::EImageColorSpace::NO_CONVERSION);
    }
  }
}

void computeNormalMaps(mvsUtils::MultiViewParams* mp, const StaticVector<int>& cams)
{
  const int nbGPUs = listCUDADevices(true);
  const int nbCPUThreads = omp_get_num_procs();

  ALICEVISION_LOG_INFO("Number of GPU devices: " << nbGPUs << ", number of CPU threads: " << nbCPUThreads);

  const int nbGPUsToUse = mp->userParams.get<int>("refineRc.num_gpus_to_use", 1);
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
