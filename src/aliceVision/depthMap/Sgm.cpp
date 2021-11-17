// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Sgm.hpp"
#include "volumeIO.hpp"

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/gpu/gpu.hpp>

#include <aliceVision/depthMap/SgmParams.hpp>
#include <aliceVision/depthMap/cuda/PlaneSweepingCuda.hpp>
#include <aliceVision/depthMap/cuda/deviceCommon/device_utils.h>

#include <aliceVision/mvsData/OrientedPoint.hpp>
#include <aliceVision/mvsData/Point3d.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/mvsUtils/common.hpp>
#include <aliceVision/mvsUtils/fileIO.hpp>
#include <aliceVision/mvsData/imageIO.hpp>
#include <aliceVision/alicevision_omp.hpp>

#include <boost/filesystem.hpp>

#include <iostream>
#include <sstream>

namespace aliceVision {
namespace depthMap {

namespace bfs = boost::filesystem;

Sgm::Sgm(const SgmParams& sgmParams, const mvsUtils::MultiViewParams& mp, PlaneSweepingCuda& cps, int rc)
    : _rc(rc)
    , _mp(mp)
    , _cps(cps)
    , _sgmParams(sgmParams)
    , _depthSimMap(_rc, _mp, _sgmParams.scale, _sgmParams.stepXY)
{
    _tCams = _mp.findNearestCamsFromLandmarks(_rc, _sgmParams.maxTCams);
    _depthsTcamsLimits.clear();

    computeDepthsAndResetTCams();
}

Sgm::~Sgm()
{}

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

float Sgm::getMinTcStepAtDepth(float depth, float minDepth, float maxDepth,
                                     StaticVector<StaticVector<float>*>* alldepths)
{
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

    return minTcStep;
}

void Sgm::computeDepths(float minDepth, float maxDepth, float scaleFactor, StaticVector<StaticVector<float>*>* alldepths)
{
    _depths.clear();

    float depth = minDepth;
    while(depth < maxDepth)
    {
        _depths.push_back(depth);

        const float step = getMinTcStepAtDepth(depth, minDepth, maxDepth, alldepths);
        depth += step * scaleFactor;
    }
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
        const int startingDepth = std::min_element(depthTcamsLimitsVec.begin(), depthTcamsLimitsVec.end(), MinOffX())->x;
        const auto depth_it = std::max_element(depthTcamsLimitsVec.begin(), depthTcamsLimitsVec.end(), MinOffXplusY());
        const int stoppingDepth = depth_it->x + depth_it->y;

        // The overall starting depth index should always be zero.
        assert(startingDepth == 0);

        // Usually stoppingDepth should be equal to the total number of depths.
        // But due to sgmMaxDepths and sgmMaxDepthPerTc, we can have more depths
        // than we finally use in all TC cameras.
        assert(_rcDepths.size() >= stoppingDepth);
    }
}

StaticVector<StaticVector<float>*>* Sgm::computeAllDepthsAndResetTCams(float midDepth)
{
    StaticVector<int> tCamsNew;
    StaticVector<StaticVector<float>*>* alldepths = new StaticVector<StaticVector<float>*>();
    alldepths->reserve(_tCams.size());

    for(int c = 0; c < _tCams.size(); c++)
    {
        // depths of all meaningful points on the principal ray of the reference camera regarding the target camera tc
        StaticVector<float>* tcdepths = _cps.getDepthsRcTc(_rc, _tCams[c], _sgmParams.scale, midDepth, _sgmParams.rcTcDepthsHalfLimit);
        if(sizeOfStaticVector<float>(tcdepths) < 50)
        {
            // fallback if we don't have enough valid samples over the epipolar line
            if(tcdepths != nullptr)
            {
                delete tcdepths;
                tcdepths = nullptr;
            }
            float avMinDist, avMidDist, avMaxDist;
            _cps.getMinMaxdepths(_rc, _tCams, avMinDist, avMidDist, avMaxDist);
            tcdepths = _cps.getDepthsByPixelSize(_rc, avMinDist, avMidDist, avMaxDist, _sgmParams.scale, _sgmParams.rcDepthsCompStep);

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
            ALICEVISION_LOG_DEBUG("nbDepths: " << _depths.size() << ", maxDepths: " << _sgmParams.maxDepths << ", scaleFactor: " << scaleFactor);
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
            ALICEVISION_LOG_DEBUG("nbDepths: " << _depths.size() << ", maxDepths: " << _sgmParams.maxDepths << ", scaleFactor: " << scaleFactor);
            computeDepths(minDepth, maxDepth, scaleFactor, alldepths);
        }
        ALICEVISION_LOG_DEBUG("Selected depth range: [" << minDepth << "-" << maxDepth << "], nb selected depths: " << _depths.size());

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
            const std::string fn = _mp.getDepthMapsFolder() + std::to_string(_mp.getViewId(_rc)) + "depths" + mvsUtils::num2str(i) + ".txt";
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

bool Sgm::sgmRc()
{
    const system::Timer timer;
    const IndexT viewId = _mp.getViewId(_rc);

    ALICEVISION_LOG_INFO("Estimate depth map (SGM) of view id: " << viewId << " (rc: " << (_rc + 1) << " / " << _mp.ncams << ")");

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
    // this device need also to allocate: 
    // (max_img - 1) * X * Y * dims_at_a_time * sizeof(float) of device memory.
    {
        int devid;
        cudaGetDevice( &devid );
        ALICEVISION_LOG_DEBUG("Allocating 2 volumes (x: " << volDim.x() << ", y: " << volDim.y() << ", z: " << volDim.z() << ") on GPU device " << devid << ".");
    }

    CudaDeviceMemoryPitched<TSim, 3> volumeSecBestSim_d(volDim);
    CudaDeviceMemoryPitched<TSim, 3> volumeBestSim_d(volDim);

    checkStartingAndStoppingDepth();

    _cps.computeDepthSimMapVolume(_rc, volumeBestSim_d, volumeSecBestSim_d, volDim, _tCams.getData(), _depthsTcamsLimits.getData(), _depths.getData(), _sgmParams);

    if (_sgmParams.exportIntermediateResults)
    {
        CudaHostMemoryHeap<TSim, 3> volumeSecBestSim_h(volumeSecBestSim_d.getSize());
        volumeSecBestSim_h.copyFrom(volumeSecBestSim_d);

        exportSimilarityVolume(volumeSecBestSim_h, _depths, _mp, _rc, _sgmParams.scale, _sgmParams.stepXY, _mp.getDepthMapsFolder() + std::to_string(viewId) + "_vol_beforeFiltering.abc");
        exportSimilaritySamplesCSV(volumeSecBestSim_h, _depths, _rc, _sgmParams.scale, _sgmParams.stepXY, "beforeFiltering", _mp.getDepthMapsFolder() + std::to_string(viewId) + "_9p.csv");
    }

    // reuse best sim to put filtered sim volume
    CudaDeviceMemoryPitched<TSim, 3>& volumeFilteredSim_d = volumeBestSim_d;

    // Filter on the 3D volume to weight voxels based on their neighborhood strongness.
    // So it downweights local minimums that are not supported by their neighborhood.
    // this is here for experimental reason ... to show how SGGC work on non
    // optimized depthmaps ... it must equals to true in normal case
    if(_sgmParams.doSgmOptimizeVolume)                      
    {
        _cps.SgmOptimizeSimVolume(_rc, volumeSecBestSim_d, volumeFilteredSim_d, volDim, _sgmParams);
    }
    else
    {
        volumeFilteredSim_d.copyFrom(volumeSecBestSim_d);
    }

    if(_sgmParams.exportIntermediateResults)
    {
        CudaHostMemoryHeap<TSim, 3> volumeSecBestSim_h(volumeFilteredSim_d.getSize());
        volumeSecBestSim_h.copyFrom(volumeFilteredSim_d);

        exportSimilarityVolume(volumeSecBestSim_h, _depths, _mp, _rc, _sgmParams.scale, _sgmParams.stepXY, _mp.getDepthMapsFolder() + std::to_string(viewId) + "_vol_afterFiltering.abc");
        exportSimilaritySamplesCSV(volumeSecBestSim_h, _depths, _rc, _sgmParams.scale, _sgmParams.stepXY, "afterFiltering", _mp.getDepthMapsFolder() + std::to_string(viewId) + "_9p.csv");
    }

    // Retrieve best depth per pixel
    // For each pixel, choose the voxel with the minimal similarity value
    _cps.SgmRetrieveBestDepth(_depthSimMap, volumeFilteredSim_d, _depths, _rc, volDim, _sgmParams);

    if(_sgmParams.exportIntermediateResults)
    {
        // {
        //     // Export RAW SGM results with the depths based on the input planes without interpolation
        //     DepthSimMap depthSimMapRawPlanes(_rc, _mp, _scale, _step);
        //     _sp.cps.SgmRetrieveBestDepth(depthSimMapRawPlanes, volumeSecBestSim_d, _depths, volDimX, volDimY, volDimZ, false); // interpolate=false
        //     depthSimMapRawPlanes.save("_sgmPlanes");
        // }
        _depthSimMap.save("_sgm");
        _depthSimMap.save("_sgmStep1", true);
    }

    ALICEVISION_LOG_INFO("Estimate depth map (SGM) done in: " << timer.elapsedMs() << " ms.");
    return true;
}

std::string Sgm::getDepthMapFileName(IndexT viewId, int scale, int step) const
{
    return _mp.getDepthMapsFolder() + std::to_string(viewId) + "_depthMap_scale" + mvsUtils::num2str(scale) + "_step" + mvsUtils::num2str(step) + "_SGM.bin";
}

std::string Sgm::getSimMapFileName(IndexT viewId, int scale, int step) const
{
    return _mp.getDepthMapsFolder() + std::to_string(viewId) + "_simMap_scale" + mvsUtils::num2str(scale) + "_step" + mvsUtils::num2str(step) + "_SGM.bin";
}

std::string Sgm::getTCamsFileName(IndexT viewId) const
{
    return _mp.getDepthMapsFolder() + std::to_string(viewId) + "_tcams.bin";
}

std::string Sgm::getDepthsFileName(IndexT viewId) const
{
    return _mp.getDepthMapsFolder() + std::to_string(viewId) + "_depths.bin";
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

} // namespace depthMap
} // namespace aliceVision
