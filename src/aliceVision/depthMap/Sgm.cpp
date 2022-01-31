// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Sgm.hpp"

#include <aliceVision/alicevision_omp.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/mvsData/OrientedPoint.hpp>
#include <aliceVision/mvsData/Point3d.hpp>
#include <aliceVision/mvsData/geometry.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/depthMap/SgmParams.hpp>
#include <aliceVision/depthMap/volumeIO.hpp>
#include <aliceVision/depthMap/cuda/utils.hpp>
#include <aliceVision/depthMap/cuda/memory.hpp>
#include <aliceVision/depthMap/cuda/DeviceCache.hpp>
#include <aliceVision/depthMap/cuda/planeSweeping/deviceSimilarityVolume.hpp>

#include <iostream>
#include <sstream>

namespace aliceVision {
namespace depthMap {

Sgm::Sgm(const SgmParams& sgmParams, const mvsUtils::MultiViewParams& mp, mvsUtils::ImagesCache<ImageRGBAf>& ic, int rc)
    : _rc(rc)
    , _mp(mp)
    , _ic(ic)
    , _sgmParams(sgmParams)
    , _depthSimMap(_rc, _mp, _sgmParams.scale, _sgmParams.stepXY)
{
    _tCams = _mp.findNearestCamsFromLandmarks(_rc, _sgmParams.maxTCams);
    _depthsTcamsLimits.clear();

    computeDepthsAndResetTCams();
}

bool Sgm::sgmRc()
{
    const system::Timer timer;
    const IndexT viewId = _mp.getViewId(_rc);

    ALICEVISION_LOG_INFO("SGM depth/sim map of view id: " << viewId << ", rc: " << _rc << " (" << (_rc + 1) << " / " << _mp.ncams << ")");

    if(_tCams.empty())
    {
      return false;
    }

    // log debug camera / depth information
    logRcTcDepthInformation();

    // compute volume dimensions
    const int volDimX = _mp.getWidth(_rc) / (_sgmParams.scale * _sgmParams.stepXY);
    const int volDimY = _mp.getHeight(_rc) / (_sgmParams.scale * _sgmParams.stepXY);
    const int volDimZ = _depths.size();

    const CudaSize<3> volDim(volDimX, volDimY, volDimZ);

    // log volumes allocation size / gpu device id
    ALICEVISION_LOG_DEBUG("Allocating 2 volumes (x: " << volDim.x() << ", y: " << volDim.y() << ", z: " << volDim.z() << ") on GPU device " << getCudaDeviceId() << ".");

    // allocate best sim and second best sim volumes
    CudaDeviceMemoryPitched<TSim, 3> volumeBestSim_dmp(volDim);
    CudaDeviceMemoryPitched<TSim, 3> volumeSecBestSim_dmp(volDim);

    // check if starting and stopping depth are valid
    checkStartingAndStoppingDepth();

    // compute best sim and second best sim volumes
    computeSimilarityVolumes(volumeBestSim_dmp, volumeSecBestSim_dmp);

    // particular case with only one tc
    if(_tCams.size() < 2)
    {
        // the second best volume has no valid similarity values
        volumeSecBestSim_dmp.copyFrom(volumeBestSim_dmp);
    }

    if(_sgmParams.exportIntermediateResults)
        exportVolumeInformation(volumeSecBestSim_dmp, "beforeFiltering");

    // reuse best sim to put optimized sim volume
    CudaDeviceMemoryPitched<TSim, 3>& volumeOptimizedSim_dmp = volumeBestSim_dmp;


    // this is here for experimental reason ... to show how SGGC work on non
    // optimized depthmaps ... it must equals to true in normal case
    if(_sgmParams.doSgmOptimizeVolume)                      
    {
        optimizeSimilarityVolume(volumeOptimizedSim_dmp, volumeSecBestSim_dmp);
    }
    else
    {
        volumeOptimizedSim_dmp.copyFrom(volumeSecBestSim_dmp);
    }

    if(_sgmParams.exportIntermediateResults)
        exportVolumeInformation(volumeOptimizedSim_dmp, "afterFiltering");

    // retrieve best depth
    retrieveBestDepth(_depthSimMap, volumeOptimizedSim_dmp);

    if(_sgmParams.exportIntermediateResults)
    {
        _depthSimMap.save("_sgm");
        _depthSimMap.save("_sgmStep1", true);
    }

    ALICEVISION_LOG_INFO("SGM depth/sim map (rc: " << _rc << ") done in: " << timer.elapsedMs() << " ms.");
    return true;
}

void Sgm::logRcTcDepthInformation() const 
{
    std::ostringstream ostr;
    ostr << "Camera / Depth information: " << std::endl
         << "\t- rc camera:" << std::endl  
         << "\t  - id: " << _rc << std::endl
         << "\t  - view id: " << _mp.getViewId(_rc) << std::endl
         << "\t  - depth planes: " << _depths.size() << std::endl
         << "\t  - depths range: [" << _depths[0] << "-" << _depths[_depths.size() - 1] << "]" << std::endl 
         << "\t- tc cameras:" << std::endl;

    for(int c = 0; c < _tCams.size(); c++)
    {
        ostr << "\t  - tc camera (" << (c+1) << "/" << _tCams.size() << "):" << std::endl
             << "\t    - id: " << _tCams[c] << std::endl
             << "\t    - view id: " << _mp.getViewId(_tCams[c]) << std::endl
             << "\t    - depth planes: " << _depthsTcamsLimits[c].y << std::endl
             << "\t    - depths range: [" << _depths[_depthsTcamsLimits[c].x] << "-"
             << _depths[_depthsTcamsLimits[c].x + _depthsTcamsLimits[c].y - 1] << "]" << std::endl
             << "\t    - depth indexes range: [" << _depthsTcamsLimits[c].x << "-" 
             << _depthsTcamsLimits[c].x + _depthsTcamsLimits[c].y << "]" << std::endl;
    }

    ALICEVISION_LOG_DEBUG(ostr.str());
}

void Sgm::checkStartingAndStoppingDepth() const
{
    struct MinOffX
    {
        bool operator()(const Pixel& l, const Pixel& r) const { return (l.x < r.x); }
    };

    struct MinOffXplusY
    {
        bool operator()(const Pixel& l, const Pixel& r) const { return (l.x + l.y < r.x + r.y); }
    };

    {
        const std::vector<Pixel>& depthTcamsLimitsVec = _depthsTcamsLimits.getData();
        const int startingDepth =
            std::min_element(depthTcamsLimitsVec.begin(), depthTcamsLimitsVec.end(), MinOffX())->x;
        const auto depth_it = std::max_element(depthTcamsLimitsVec.begin(), depthTcamsLimitsVec.end(), MinOffXplusY());
        const int stoppingDepth = depth_it->x + depth_it->y;

        // The overall starting depth index should always be zero.
        assert(startingDepth == 0);

        // Usually stoppingDepth should be equal to the total number of depths.
        // But due to sgmMaxDepths and sgmMaxDepthPerTc, we can have more depths
        // than we finally use in all TC cameras.
        assert(_depths.size() >= stoppingDepth);
    }
}

void Sgm::computeSimilarityVolumes(CudaDeviceMemoryPitched<TSim, 3>& out_volBestSim_dmp, CudaDeviceMemoryPitched<TSim, 3>& out_volSecBestSim_dmp) const
{
    const system::Timer timer;

    const CudaSize<3>& volDim = out_volBestSim_dmp.getSize();

    ALICEVISION_LOG_INFO("SGM Compute similarity volume (rc: " << _rc << " x: " << volDim.x() << ", y: " << volDim.y() << ", z: " << volDim.z() << ")");

    // initialize the two similarity volumes at 255
    cuda_volumeInitialize(out_volBestSim_dmp, 255.f, 0 /*stream*/);
    cuda_volumeInitialize(out_volSecBestSim_dmp, 255.f, 0 /*stream*/);

    // load rc & tc images in the CPU ImageCache
    _ic.getImg_sync(_rc);
    for(int tc : _tCams)
    {
        _ic.getImg_sync(tc);
    }

    // copy rc depth data in device memory
    CudaDeviceMemory<float> depths_d(_depths.getData().data(), _depths.size());

    // log memory information
    logDeviceMemoryInfo();

    for(int tci = 0; tci < _tCams.size(); ++tci)
    {
        const system::Timer timerPerTc;

        const int tc = _tCams[tci];

        const int firstDepth = _depthsTcamsLimits[tci].x;
        const int lastDepth = firstDepth + _depthsTcamsLimits[tci].y;

        DeviceCache& deviceCache = DeviceCache::getInstance();

        const DeviceCamera& rcDeviceCamera = deviceCache.requestCamera(_rc, _sgmParams.scale, _ic, _mp, 0);
        const DeviceCamera& tcDeviceCamera = deviceCache.requestCamera( tc, _sgmParams.scale, _ic, _mp, 0);

        const ROI roi(0, out_volBestSim_dmp.getSize().x(), 0, out_volBestSim_dmp.getSize().y(), firstDepth, lastDepth);

        ALICEVISION_LOG_DEBUG("Compute similarity volume:" << std::endl
                              << "\t- rc: " << _rc << std::endl
                              << "\t- tc: " << tc << " (" << tci << "/" << _tCams.size() << ")" << std::endl
                              << "\t- rc camera device id: " << rcDeviceCamera.getDeviceCamId() << std::endl
                              << "\t- tc camera device id: " << tcDeviceCamera.getDeviceCamId() << std::endl
                              << "\t- tc first depth: " << firstDepth << std::endl
                              << "\t- tc last depth: " << lastDepth << std::endl
                              << "\t- rc width: " << rcDeviceCamera.getWidth() << std::endl
                              << "\t- rc height: " << rcDeviceCamera.getHeight() << std::endl
                              << "\t- device similarity volume size: " << out_volBestSim_dmp.getBytesPadded() / (1024.0 * 1024.0) << " MB" << std::endl
                              << "\t- device unpadded similarity volume size: " << out_volBestSim_dmp.getBytesUnpadded() / (1024.0 * 1024.0) << " MB" << std::endl);

        cuda_volumeComputeSimilarity(out_volBestSim_dmp, 
                                     out_volSecBestSim_dmp, 
                                     depths_d, 
                                     rcDeviceCamera, 
                                     tcDeviceCamera,
                                     _sgmParams, 
                                     roi, 
                                     0 /*stream*/);

        ALICEVISION_LOG_DEBUG("Compute similarity volume (with rc: " << _rc << ", tc: " << tc << ") done in: " << timerPerTc.elapsedMs() << " ms.");
    }
    ALICEVISION_LOG_INFO("SGM Compute similarity volume (rc: " << _rc << ") done in: " << timer.elapsedMs() << " ms.");
}

void Sgm::optimizeSimilarityVolume(CudaDeviceMemoryPitched<TSim, 3>& out_volSimOptimized_dmp, const CudaDeviceMemoryPitched<TSim, 3>& in_volSim_dmp) const
{
    const system::Timer timer;

    const CudaSize<3>& volDim = in_volSim_dmp.getSize();

    ALICEVISION_LOG_INFO("SGM Optimizing volume:" << std::endl
                          << "\t- filtering axes: " << _sgmParams.filteringAxes << std::endl
                          << "\t- volume dimensions: (x: " << volDim.x() << ", y: " << volDim.y() << ", z: " << volDim.z() << ")" << std::endl
                          << "\t- device similarity volume size: " << (double(in_volSim_dmp.getBytesPadded()) / (1024.0 * 1024.0)) << " MB" << std::endl);

    DeviceCache& deviceCache = DeviceCache::getInstance();

    const DeviceCamera& rcDeviceCamera = deviceCache.requestCamera(_rc, _sgmParams.scale, _ic, _mp);

    cuda_volumeOptimize(out_volSimOptimized_dmp, 
                        in_volSim_dmp, 
                        rcDeviceCamera, 
                        _sgmParams, 
                        0 /*stream*/);

    ALICEVISION_LOG_INFO("SGM Optimizing volume done in: " << timer.elapsedMs() << " ms.");
}

void Sgm::retrieveBestDepth(DepthSimMap& out_bestDepthSimMap, const CudaDeviceMemoryPitched<TSim, 3>& in_volSim_dmp) const 
{
    const system::Timer timer;

    const CudaSize<3>& volDim = in_volSim_dmp.getSize();
  
    ALICEVISION_LOG_INFO("SGM Retrieve best depth in volume (x: " << volDim.x() << ", y: " << volDim.y() << ", z: " << volDim.z() << ")");

    DeviceCache& deviceCache = DeviceCache::getInstance();

    const DeviceCamera& rcDeviceCamera = deviceCache.requestCamera(_rc, 1, _ic, _mp);

    const CudaSize<2> depthSimDim(volDim.x(), volDim.y());

    CudaDeviceMemory<float> depths_d(_depths.getData().data(), _depths.size());
    CudaDeviceMemoryPitched<float, 2> bestDepth_dmp(depthSimDim);
    CudaDeviceMemoryPitched<float, 2> bestSim_dmp(depthSimDim);

    const ROI roi(0, in_volSim_dmp.getSize().x(), 0, in_volSim_dmp.getSize().y(), 0, in_volSim_dmp.getSize().z());

    cuda_volumeRetrieveBestDepth(bestDepth_dmp,
                                 bestSim_dmp, 
                                 in_volSim_dmp, 
                                 depths_d, 
                                 rcDeviceCamera,
                                 _sgmParams,
                                 roi, 
                                 0 /*stream*/);

    CudaHostMemoryHeap<float, 2> bestDepth_hmh(depthSimDim);
    bestDepth_hmh.copyFrom(bestDepth_dmp);

    CudaHostMemoryHeap<float, 2> bestSim_hmh(depthSimDim);
    bestSim_hmh.copyFrom(bestSim_dmp);

    for(int y = 0; y < depthSimDim.y(); ++y)
    {
        for(int x = 0; x < depthSimDim.x(); ++x)
        {
            DepthSim& out = out_bestDepthSimMap._dsm[y * depthSimDim.x() + x];
            out.depth = bestDepth_hmh(x, y);
            out.sim = bestSim_hmh(x, y);
        }
    }

    ALICEVISION_LOG_INFO("SGM Retrieve best depth in volume done in: " << timer.elapsedMs() << " ms.");
}

void Sgm::exportVolumeInformation(const CudaDeviceMemoryPitched<TSim, 3>& in_volSim_dmp, const std::string& name) const
{
    const IndexT viewId = _mp.getViewId(_rc);
    CudaHostMemoryHeap<TSim, 3> volumeSim_hmh(in_volSim_dmp.getSize());
    volumeSim_hmh.copyFrom(in_volSim_dmp);

    exportSimilarityVolume(volumeSim_hmh, _depths, _mp, _rc, _sgmParams, _mp.getDepthMapsFolder() + std::to_string(viewId) + "_vol_" + name + ".abc");
    exportSimilarityVolumeCross(volumeSim_hmh, _depths, _mp, _rc, _sgmParams, _mp.getDepthMapsFolder() + std::to_string(viewId) + "_vol-cross_" + name + ".abc");
    exportSimilaritySamplesCSV(volumeSim_hmh, _depths, _rc, name, _mp.getDepthMapsFolder() + std::to_string(viewId) + "_9p.csv");
}

void Sgm::computeDepthsAndResetTCams()
{
    ALICEVISION_LOG_DEBUG("Compute depths and reset TCams");

    std::size_t nbObsDepths;
    float minObsDepth, maxObsDepth, midObsDepth;
    _mp.getMinMaxMidNbDepth(_rc, minObsDepth, maxObsDepth, midObsDepth, nbObsDepths, _sgmParams.seedsRangePercentile);

    StaticVector<StaticVector<float>*>* alldepths;

    // all depths from the principal ray provided by target cameras
    if(nbObsDepths < 20)
        alldepths = computeAllDepthsAndResetTCams(-1);
    else
        alldepths = computeAllDepthsAndResetTCams(midObsDepth);

    float minDepthAll = std::numeric_limits<float>::max();
    float maxDepthAll = 0.0f;
    for(int i = 0; i < alldepths->size(); i++)
    {
        for(int j = 0; j < (*alldepths)[i]->size(); j++)
        {
            float depth = (*(*alldepths)[i])[j];
            minDepthAll = std::min(minDepthAll, depth);
            maxDepthAll = std::max(maxDepthAll, depth);
        }
    }

    if(!_sgmParams.useSfmSeeds || _mp.getInputSfMData().getLandmarks().empty())
    {
        ALICEVISION_LOG_DEBUG("Select depth candidates without seeds. Nb observations: " << nbObsDepths);

        computeDepths(minDepthAll, maxDepthAll, (_sgmParams.stepZ > 0.0f ? _sgmParams.stepZ : 1.0f), alldepths);

        if(_sgmParams.maxDepths > 0 && _depths.size() > _sgmParams.maxDepths)
        {
            const float scaleFactor = float(_depths.size()) / float(_sgmParams.maxDepths);
            ALICEVISION_LOG_DEBUG("nbDepths: " << _depths.size() << ", maxDepths: " << _sgmParams.maxDepths
                                               << ", scaleFactor: " << scaleFactor);
            computeDepths(minDepthAll, maxDepthAll, scaleFactor, alldepths);
        }
        if(_sgmParams.saveDepthsToSweepTxtFile)
        {
            const std::string fn = _mp.getDepthMapsFolder() + std::to_string(_mp.getViewId(_rc)) + "depthsAll.txt";
            FILE* f = fopen(fn.c_str(), "w");
            for(int j = 0; j < _depths.size(); j++)
            {
                fprintf(f, "%f\n", _depths[j]);
            }
            fclose(f);
        }
    }
    else
    {
        ALICEVISION_LOG_DEBUG("Select depth candidates from seeds. Nb observations: " << nbObsDepths);
        ALICEVISION_LOG_DEBUG("Depth all: [" << minDepthAll << "-" << maxDepthAll << "]");
        float minDepth = minDepthAll;
        float maxDepth = maxDepthAll;

        // if we get enough information from seeds, adjust min/maxDepth
        if(nbObsDepths > 100)
        {
            minDepth = minObsDepth * (1.0f - _sgmParams.seedsRangeInflate);
            maxDepth = maxObsDepth * (1.0f + _sgmParams.seedsRangeInflate);

            if(maxDepthAll < minDepth || minDepthAll > maxDepth)
            {
                // no intersection between min/maxDepth and min/maxDepthAll
                // keep min/maxDepth value as is
            }
            else
            {
                // min/maxDepth intersection with min/maxDepthAll
                minDepth = std::max(minDepthAll, minDepth);
                maxDepth = std::min(maxDepthAll, maxDepth);
            }
        }

        // build the list of "best" depths for rc, from all tc cameras depths
        computeDepths(minDepth, maxDepth, (_sgmParams.stepZ > 0.0f ? _sgmParams.stepZ : 1.0f), alldepths);

        // filter out depths if computeDepths gave too many values
        if(_sgmParams.maxDepths > 0 && _depths.size() > _sgmParams.maxDepths)
        {
            const float scaleFactor = float(_depths.size()) / float(_sgmParams.maxDepths);
            ALICEVISION_LOG_DEBUG("nbDepths: " << _depths.size() << ", maxDepths: " << _sgmParams.maxDepths
                                               << ", scaleFactor: " << scaleFactor);
            computeDepths(minDepth, maxDepth, scaleFactor, alldepths);
        }
        ALICEVISION_LOG_DEBUG("Selected depth range: [" << minDepth << "-" << maxDepth
                                                        << "], nb selected depths: " << _depths.size());

        if(_sgmParams.saveDepthsToSweepTxtFile)
        {
            const std::string fn = _mp.getDepthMapsFolder() + std::to_string(_mp.getViewId(_rc)) + "depthsAll.txt";
            FILE* f = fopen(fn.c_str(), "w");
            for(int j = 0; j < _depths.size(); j++)
            {
                fprintf(f, "%f\n", _depths[j]);
            }
            fclose(f);
        }
    }

    // fill depthsTcamsLimits member variable with index range of depths to sweep
    computeDepthsTcamsLimits(alldepths);

    if(_sgmParams.saveDepthsToSweepTxtFile)
    {
        const std::string fn = _mp.getDepthMapsFolder() + std::to_string(_mp.getViewId(_rc)) + "depthsTcamsLimits.txt";
        FILE* f = fopen(fn.c_str(), "w");
        for(int j = 0; j < _depthsTcamsLimits.size(); j++)
        {
            Pixel l = _depthsTcamsLimits[j];
            // fprintf(f,"%f %f\n",(*depths)[l.x],(*depths)[l.x+l.y-1]);
            fprintf(f, "%i %i\n", l.x, l.y);
        }
        fclose(f);
    }

    if(_sgmParams.saveDepthsToSweepTxtFile)
    {
        const std::string fn = _mp.getDepthMapsFolder() + std::to_string(_mp.getViewId(_rc)) + "depths.txt";
        FILE* f = fopen(fn.c_str(), "w");
        for(int j = 0; j < _depths.size(); j++)
        {
            fprintf(f, "%f\n", _depths[j]);
        }
        fclose(f);
    }

    if(_sgmParams.saveDepthsToSweepTxtFile)
    {
        for(int i = 0; i < alldepths->size(); i++)
        {
            const std::string fn = _mp.getDepthMapsFolder() + std::to_string(_mp.getViewId(_rc)) + "depths" +
                                   mvsUtils::num2str(i) + ".txt";
            FILE* f = fopen(fn.c_str(), "w");
            for(int j = 0; j < (*alldepths)[i]->size(); j++)
            {
                const float depth = (*(*alldepths)[i])[j];
                fprintf(f, "%f\n", depth);
            }
            fclose(f);
        }
    }

    if(_sgmParams.saveDepthsToSweepTxtFile)
    {
        OrientedPoint rcplane;
        rcplane.p = _mp.CArr[_rc];
        rcplane.n = _mp.iRArr[_rc] * Point3d(0.0, 0.0, 1.0);
        rcplane.n = rcplane.n.normalize();

        const std::string fn = _mp.getDepthMapsFolder() + std::to_string(_mp.getViewId(_rc)) + "rcDepths.txt";
        FILE* f = fopen(fn.c_str(), "w");
        float depth = minDepthAll;
        while(depth < maxDepthAll)
        {
            fprintf(f, "%f\n", depth);
            const Point3d p = rcplane.p + rcplane.n * depth;
            depth = depth + _mp.getCamPixelSize(p, _rc);
        }
        fclose(f);
    }

    deleteArrayOfArrays<float>(&alldepths);

    ALICEVISION_LOG_DEBUG("Compute depths and reset TCams done, rc depths: " << _depths.size());
}

StaticVector<StaticVector<float>*>* Sgm::computeAllDepthsAndResetTCams(float midDepth)
{
    StaticVector<int> tCamsNew;
    StaticVector<StaticVector<float>*>* alldepths = new StaticVector<StaticVector<float>*>();
    alldepths->reserve(_tCams.size());

    for(int c = 0; c < _tCams.size(); c++)
    {
        // depths of all meaningful points on the principal ray of the reference camera regarding the target camera tc
        StaticVector<float>* tcdepths = getDepthsTc(_tCams[c], midDepth);
        if(sizeOfStaticVector<float>(tcdepths) < 50)
        {
            // fallback if we don't have enough valid samples over the epipolar line
            if(tcdepths != nullptr)
            {
                delete tcdepths;
                tcdepths = nullptr;
            }
            float avMinDist, avMidDist, avMaxDist;
            getMinMaxDepths(avMinDist, avMidDist, avMaxDist);
            tcdepths = getDepthsByPixelSize(avMinDist, avMidDist, avMaxDist);

            if(sizeOfStaticVector<float>(tcdepths) < 50)
            {
                if(tcdepths != nullptr)
                {
                    delete tcdepths;
                    tcdepths = nullptr;
                }
            }
        }

        if(tcdepths != nullptr)
        {
            alldepths->push_back(tcdepths);
            tCamsNew.push_back(_tCams[c]);
        }
    }

    _tCams = tCamsNew;

    return alldepths;
}

void Sgm::computeDepthsTcamsLimits(StaticVector<StaticVector<float>*>* alldepths)
{
    _depthsTcamsLimits.resize(_tCams.size());

    for(int c = 0; c < _tCams.size(); c++)
    {
        const float d1 = (*(*alldepths)[c])[0];
        const float d2 = (*(*alldepths)[c])[(*alldepths)[c]->size() - 1];

        int id1 = _depths.indexOfNearestSorted(d1);
        int id2 = _depths.indexOfNearestSorted(d2);

        if(id1 == -1)
            id1 = 0;

        if(id2 == -1)
            id2 = _depths.size() - 1;

        // clamp to keep only the closest depths if we have too much inputs (> _sgmParams.maxDepthsPerTc)
        id2 = std::min(id1 + _sgmParams.maxDepthsPerTc - 1, id2);
        _depthsTcamsLimits[c] = Pixel(id1, id2 - id1 + 1);
    }
}

void Sgm::computeDepths(float minDepth, float maxDepth, float scaleFactor,
                        const StaticVector<StaticVector<float>*>* alldepths)
{
    _depths.clear();

    float depth = minDepth;

    while(depth < maxDepth)
    {
        _depths.push_back(depth);

        // get min tc step at depth
        float minTcStep = maxDepth - minDepth;

        // for each tc camera
        for(int i = 0; i < alldepths->size(); i++)
        {
            // list of valid depths for the tc
            StaticVector<float>* tcDepths = (*alldepths)[i];

            // get the tc depth closest to the current depth
            const int id = tcDepths->indexOfNearestSorted(depth);

            // continue on no result or last element (we need id + 1)
            if(id < 0 || id >= tcDepths->size() - 1)
                continue;

            // consider the enclosing depth range
            const float did = (*tcDepths)[id];     // closest depth
            const float nid = (*tcDepths)[id + 1]; // next depth
            const float tcStep = fabs(did - nid);  // [closest; next] depths distance

            // keep this value if smallest step so far
            minTcStep = std::min(minTcStep, tcStep);
        }

        depth += minTcStep * scaleFactor;
    }
}

void Sgm::getMinMaxDepths(float& minDepth, float& midDepth, float& maxDepth)
{
    if(_sgmParams.prematchinMinMaxDepthDontUseSeeds)
    {
        minDepth = 0.0f;
        maxDepth = 0.0f;
        for(int c = 0; c < _tCams.size(); ++c)
        {
            const int tc = _tCams[c];
            minDepth += (_mp.CArr[_rc] - _mp.CArr[tc]).size() * _sgmParams.prematchingMinCamDist;
            maxDepth += (_mp.CArr[_rc] - _mp.CArr[tc]).size() * _sgmParams.prematchingMaxCamDist;
        }
        minDepth /= static_cast<float>(_tCams.size());
        maxDepth /= static_cast<float>(_tCams.size());
        midDepth = (minDepth + maxDepth) / 2.0f;
    }
    else
    {
        std::size_t nbDepths;
        _mp.getMinMaxMidNbDepth(_rc, minDepth, maxDepth, midDepth, nbDepths, _sgmParams.seedsRangePercentile);
        maxDepth = maxDepth * _sgmParams.prematchingMaxDepthScale;
    }
}

StaticVector<float>* Sgm::getDepthsByPixelSize(float minDepth, float midDepth, float maxDepth)
{
    const int maxDepthsHalf = 1024;

    const float d = float(_sgmParams.scale) * float(_sgmParams.rcDepthsCompStep);

    OrientedPoint rcplane;
    rcplane.p = _mp.CArr[_rc];
    rcplane.n = _mp.iRArr[_rc] * Point3d(0.0, 0.0, 1.0);
    rcplane.n = rcplane.n.normalize();

    int ndepthsMidMax = 0;
    float maxdepth = midDepth;
    while((maxdepth < maxDepth) && (ndepthsMidMax < maxDepthsHalf))
    {
        Point3d p = rcplane.p + rcplane.n * maxdepth;
        float pixSize = _mp.getCamPixelSize(p, _rc, d);
        maxdepth += pixSize;
        ndepthsMidMax++;
    }

    int ndepthsMidMin = 0;
    float mindepth = midDepth;
    while((mindepth > minDepth) && (ndepthsMidMin < maxDepthsHalf * 2 - ndepthsMidMax))
    {
        Point3d p = rcplane.p + rcplane.n * mindepth;
        float pixSize = _mp.getCamPixelSize(p, _rc, d);
        mindepth -= pixSize;
        ndepthsMidMin++;
    }

    // getNumberOfDepths
    float depth = mindepth;
    int ndepths = 0;
    float pixSize = 1.0f;
    while((depth < maxdepth) && (pixSize > 0.0f) && (ndepths < 2 * maxDepthsHalf))
    {
        Point3d p = rcplane.p + rcplane.n * depth;
        pixSize = _mp.getCamPixelSize(p, _rc, d);
        depth += pixSize;
        ndepths++;
    }

    StaticVector<float>* out = new StaticVector<float>();
    out->reserve(ndepths);

    // fill
    depth = mindepth;
    pixSize = 1.0f;
    ndepths = 0;
    while((depth < maxdepth) && (pixSize > 0.0f) && (ndepths < 2 * maxDepthsHalf))
    {
        out->push_back(depth);
        Point3d p = rcplane.p + rcplane.n * depth;
        pixSize = _mp.getCamPixelSize(p, _rc, d);
        depth += pixSize;
        ndepths++;
    }

    // check if it is asc
    for(int i = 0; i < out->size() - 1; i++)
    {
        if((*out)[i] >= (*out)[i + 1])
        {
            for(int j = 0; j <= i + 1; j++)
            {
                ALICEVISION_LOG_TRACE("getDepthsByPixelSize: check if it is asc: " << (*out)[j]);
            }
            throw std::runtime_error("getDepthsByPixelSize not asc.");
        }
    }
    return out;
}

StaticVector<float>* Sgm::getDepthsTc(int tc, float midDepth)
{
    OrientedPoint rcplane;
    rcplane.p = _mp.CArr[_rc];
    rcplane.n = _mp.iRArr[_rc] * Point3d(0.0, 0.0, 1.0);
    rcplane.n = rcplane.n.normalize();

    const Point2d rmid = Point2d((float)_mp.getWidth(_rc) / 2.0f, (float)_mp.getHeight(_rc) / 2.0f);
    Point2d pFromTar, pToTar; // segment of epipolar line of the principal point of the rc camera to the tc camera
    getTarEpipolarDirectedLine(&pFromTar, &pToTar, rmid, _rc, tc, _mp);

    int allDepths = static_cast<int>((pToTar - pFromTar).size());
    ALICEVISION_LOG_DEBUG("allDepths: " << allDepths);

    const Point2d pixelVect = ((pToTar - pFromTar).normalize()) * std::max(1.0f, (float)_sgmParams.scale);
    // printf("%f %f %i %i\n",pixelVect.size(),((float)(scale*step)/3.0f),scale,step);

    Point2d cg = Point2d(0.0f, 0.0f);
    Point3d cg3 = Point3d(0.0f, 0.0f, 0.0f);
    int ncg = 0;
    // navigate through all pixels of the epilolar segment
    // Compute the middle of the valid pixels of the epipolar segment (in rc camera) of the principal point (of the rc
    // camera)
    for(int i = 0; i < allDepths; i++)
    {
        Point2d tpix = pFromTar + pixelVect * (float)i;
        Point3d p;
        if(triangulateMatch(p, rmid, tpix, _rc, tc, _mp)) // triangulate principal point from rc with tpix
        {
            float depth = orientedPointPlaneDistance(
                p, rcplane.p,
                rcplane.n); // todo: can compute the distance to the camera (as it's the principal point it's the same)
            if(_mp.isPixelInImage(tpix, tc) && (depth > 0.0f) &&
               checkPair(p, _rc, tc, _mp, _mp.getMinViewAngle(), _mp.getMaxViewAngle()))
            {
                cg = cg + tpix;
                cg3 = cg3 + p;
                ncg++;
            }
        }
    }
    if(ncg == 0)
    {
        return new StaticVector<float>();
    }
    cg = cg / (float)ncg;
    cg3 = cg3 / (float)ncg;
    allDepths = ncg;

    ALICEVISION_LOG_DEBUG("All correct depths: " << allDepths);

    Point2d midpoint = cg;
    if(midDepth > 0.0f)
    {
        Point3d midPt = rcplane.p + rcplane.n * midDepth;
        _mp.getPixelFor3DPoint(&midpoint, midPt, tc);
    }

    // compute the direction
    float direction = 1.0f;
    {
        Point3d p;
        if(!triangulateMatch(p, rmid, midpoint, _rc, tc, _mp))
        {
            StaticVector<float>* out = new StaticVector<float>();
            return out;
        }

        float depth = orientedPointPlaneDistance(p, rcplane.p, rcplane.n);

        if(!triangulateMatch(p, rmid, midpoint + pixelVect, _rc, tc, _mp))
        {
            StaticVector<float>* out = new StaticVector<float>();
            return out;
        }

        float depthP1 = orientedPointPlaneDistance(p, rcplane.p, rcplane.n);
        if(depth > depthP1)
        {
            direction = -1.0f;
        }
    }

    StaticVector<float>* out1 = new StaticVector<float>();
    out1->reserve(2 * _sgmParams.rcTcDepthsHalfLimit);

    Point2d tpix = midpoint;
    float depthOld = -1.0f;
    int istep = 0;
    bool ok = true;

    // compute depths for all pixels from the middle point to on one side of the epipolar line
    while((out1->size() < _sgmParams.rcTcDepthsHalfLimit) && (_mp.isPixelInImage(tpix, tc) == true) && (ok == true))
    {
        tpix = tpix + pixelVect * direction;

        Point3d refvect = _mp.iCamArr[_rc] * rmid;
        Point3d tarvect = _mp.iCamArr[tc] * tpix;
        float rptpang = angleBetwV1andV2(refvect, tarvect);

        Point3d p;
        ok = triangulateMatch(p, rmid, tpix, _rc, tc, _mp);

        float depth = orientedPointPlaneDistance(p, rcplane.p, rcplane.n);
        if(_mp.isPixelInImage(tpix, tc) && (depth > 0.0f) && (depth > depthOld) &&
           checkPair(p, _rc, tc, _mp, _mp.getMinViewAngle(), _mp.getMaxViewAngle()) &&
           (rptpang >
            _mp.getMinViewAngle()) // WARNING if vects are near parallel thaen this results to strange angles ...
           &&
           (rptpang <
            _mp.getMaxViewAngle())) // this is the propper angle ... beacause is does not depend on the triangluated p
        {
            out1->push_back(depth);
            // if ((tpix.x!=tpixold.x)||(tpix.y!=tpixold.y)||(depthOld>=depth))
            //{
            // printf("after %f %f %f %f %i %f %f\n",tpix.x,tpix.y,depth,depthOld,istep,ang,kk);
            //};
        }
        else
        {
            ok = false;
        }
        depthOld = depth;
        istep++;
    }

    StaticVector<float>* out2 = new StaticVector<float>();
    out2->reserve(2 * _sgmParams.rcTcDepthsHalfLimit);
    tpix = midpoint;
    istep = 0;
    ok = true;

    // compute depths for all pixels from the middle point to the other side of the epipolar line
    while((out2->size() < _sgmParams.rcTcDepthsHalfLimit) && (_mp.isPixelInImage(tpix, tc) == true) && (ok == true))
    {
        const Point3d refvect = _mp.iCamArr[_rc] * rmid;
        const Point3d tarvect = _mp.iCamArr[tc] * tpix;
        const float rptpang = angleBetwV1andV2(refvect, tarvect);

        Point3d p;
        ok = triangulateMatch(p, rmid, tpix, _rc, tc, _mp);

        float depth = orientedPointPlaneDistance(p, rcplane.p, rcplane.n);
        if(_mp.isPixelInImage(tpix, tc) && (depth > 0.0f) && (depth < depthOld) &&
           checkPair(p, _rc, tc, _mp, _mp.getMinViewAngle(), _mp.getMaxViewAngle()) &&
           (rptpang >
            _mp.getMinViewAngle()) // WARNING if vects are near parallel thaen this results to strange angles ...
           &&
           (rptpang <
            _mp.getMaxViewAngle())) // this is the propper angle ... beacause is does not depend on the triangluated p
        {
            out2->push_back(depth);
            // printf("%f %f\n",tpix.x,tpix.y);
        }
        else
        {
            ok = false;
        }

        depthOld = depth;
        tpix = tpix - pixelVect * direction;
    }

    // printf("out2\n");
    StaticVector<float>* out = new StaticVector<float>();
    out->reserve(2 * _sgmParams.rcTcDepthsHalfLimit);
    for(int i = out2->size() - 1; i >= 0; i--)
    {
        out->push_back((*out2)[i]);
        // printf("%f\n",(*out2)[i]);
    }
    // printf("out1\n");
    for(int i = 0; i < out1->size(); i++)
    {
        out->push_back((*out1)[i]);
        // printf("%f\n",(*out1)[i]);
    }

    delete out2;
    delete out1;

    // we want to have it in ascending order
    if(out->size() > 0 && (*out)[0] > (*out)[out->size() - 1])
    {
        StaticVector<float>* outTmp = new StaticVector<float>();
        outTmp->reserve(out->size());
        for(int i = out->size() - 1; i >= 0; i--)
        {
            outTmp->push_back((*out)[i]);
        }
        delete out;
        out = outTmp;
    }

    // check if it is asc
    for(int i = 0; i < out->size() - 1; i++)
    {
        if((*out)[i] > (*out)[i + 1])
        {

            for(int j = 0; j <= i + 1; j++)
            {
                ALICEVISION_LOG_TRACE("getDepthsRcTc: check if it is asc: " << (*out)[j]);
            }
            ALICEVISION_LOG_WARNING("getDepthsRcTc: not asc");

            if(out->size() > 1)
            {
                qsort(&(*out)[0], out->size(), sizeof(float), qSortCompareFloatAsc);
            }
        }
    }

    ALICEVISION_LOG_DEBUG("used depths: " << out->size());

    return out;
}

bool Sgm::selectBestDepthsRange(int nDepthsThr, StaticVector<float>* rcSeedsDistsAsc)
{
    if(_depths.size() <= nDepthsThr)
        return true;

    StaticVector<int> votes;
    votes.reserve(_depths.size() - nDepthsThr);
    for(int i = 0; i < _depths.size() - nDepthsThr; i++)
    {
        const float d1 = _depths[i];
        const float d2 = _depths[i + nDepthsThr - 1];

        int id1 = rcSeedsDistsAsc->indexOfNearestSorted(d1);
        int id2 = rcSeedsDistsAsc->indexOfNearestSorted(d2);

        if(d1 < (*rcSeedsDistsAsc)[0])
            id1 = 0;

        if(d2 > (*rcSeedsDistsAsc)[rcSeedsDistsAsc->size() - 1])
            id2 = rcSeedsDistsAsc->size() - 1;

        if((id1 > -1) && (id2 > -1))
            votes.push_back(abs(id2 - id1));
        else
            votes.push_back(0);
    }

    StaticVector<float> depthsNew;
    depthsNew.reserve(nDepthsThr);

    const int id1 = votes.maxValId();
    const int id2 = id1 + nDepthsThr - 1;

    for(int i = id1; i <= id2; i++)
        depthsNew.push_back(_depths[i]);

    std::swap(_depths, depthsNew);
    return true;
}

bool Sgm::selectBestDepthsRange(int nDepthsThr, StaticVector<StaticVector<float>*>* alldepths)
{
    if(nDepthsThr <= 0 || _depths.size() <= nDepthsThr)
        return true;

    StaticVector<float> votes;
    votes.reserve(_depths.size() - nDepthsThr);

    for(int i = 0; i < _depths.size() - nDepthsThr; i++)
    {
        const float d1 = _depths[i];
        const float d2 = _depths[i + nDepthsThr - 1];
        float overlap = 0.0f;

        for(int c = 0; c < alldepths->size(); c++)
        {
            const StaticVector<float>* tcDepths = (*alldepths)[c];
            const float dd1 = std::max(d1, (*tcDepths)[0]);
            const float dd2 = std::min(d2, (*tcDepths)[tcDepths->size() - 1]);
            if(dd1 < dd2)
                overlap += dd2 - dd1;
        }
        votes.push_back(overlap);
    }

    StaticVector<float> depthsNew;
    depthsNew.reserve(nDepthsThr);

    const int id1 = votes.maxValId();
    const int id2 = id1 + nDepthsThr - 1;

    for(int i = id1; i <= id2; i++)
        depthsNew.push_back(_depths[i]);

    std::swap(_depths, depthsNew);
    return true;
}

} // namespace depthMap
} // namespace aliceVision
