// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "SemiGlobalMatchingRc.hpp"
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/gpu/gpu.hpp>

#include <aliceVision/depthMap/SemiGlobalMatchingRcTc.hpp>
#include <aliceVision/depthMap/SemiGlobalMatchingVolume.hpp>
#include <aliceVision/mvsData/OrientedPoint.hpp>
#include <aliceVision/mvsData/Point3d.hpp>
#include <aliceVision/mvsUtils/common.hpp>
#include <aliceVision/mvsUtils/fileIO.hpp>
#include <aliceVision/mvsData/imageIO.hpp>
#include <aliceVision/alicevision_omp.hpp>

#include <boost/filesystem.hpp>

#include <iostream>

namespace aliceVision {
namespace depthMap {

namespace bfs = boost::filesystem;

SemiGlobalMatchingRc::SemiGlobalMatchingRc(int rc, int scale, int step, SemiGlobalMatchingParams* sp)
    : _rc(rc)
    , _scale(scale)
    , _step(step)
    , _sp(sp)
{
    const int nbNearestCams = sp->mp->userParams.get<int>("semiGlobalMatching.maxTCams", 10);
    _width  = sp->mp->getWidth(rc)  / (scale * step);
    _height = sp->mp->getHeight(rc) / (scale * step);
    _sgmTCams  = sp->mp->findNearestCamsFromLandmarks(rc, nbNearestCams);
    _sgmWsh = sp->mp->userParams.get<int>("semiGlobalMatching.wsh", 4);
    _sgmGammaC = static_cast<float>(sp->mp->userParams.get<double>("semiGlobalMatching.gammaC", 5.5));
    _sgmGammaP = static_cast<float>(sp->mp->userParams.get<double>("semiGlobalMatching.gammaP", 8.0));
    _depthsTcamsLimits.clear();

    computeDepthsAndResetTCams();
}

SemiGlobalMatchingRc::~SemiGlobalMatchingRc()
{
    delete _volumeBestIdVal;
}

bool SemiGlobalMatchingRc::selectBestDepthsRange(int nDepthsThr, StaticVector<float>* rcSeedsDistsAsc)
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

bool SemiGlobalMatchingRc::selectBestDepthsRange(int nDepthsThr, StaticVector<StaticVector<float>*>* alldepths)
{
    if(_depths.size() <= nDepthsThr)
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

float SemiGlobalMatchingRc::getMinTcStepAtDepth(float depth, float minDepth, float maxDepth,
                                     StaticVector<StaticVector<float>*>* alldepths)
{
    float minTcStep = maxDepth - minDepth;

    // for each tc depths
    for(int i = 0; i < alldepths->size(); i++)
    {
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

void SemiGlobalMatchingRc::computeDepths(float minDepth, float maxDepth, StaticVector<StaticVector<float>*>* alldepths)
{
    int maxNdetphs = 0;
    {
        float depth = minDepth;
        while(depth < maxDepth)
        {
            maxNdetphs++;
            depth += getMinTcStepAtDepth(depth, minDepth, maxDepth, alldepths);
        }
    }

    _depths.resize(0);
    _depths.reserve(maxNdetphs);

    {
        float depth = minDepth;
        while(depth < maxDepth)
        {
            _depths.push_back(depth);
            depth += getMinTcStepAtDepth(depth, minDepth, maxDepth, alldepths);
        }
    }
}

StaticVector<StaticVector<float>*>* SemiGlobalMatchingRc::computeAllDepthsAndResetTCams(float midDepth)
{
    StaticVector<int> tcamsNew;
    StaticVector<StaticVector<float>*>* alldepths = new StaticVector<StaticVector<float>*>();
    alldepths->reserve(_sgmTCams.size());

    for(int c = 0; c < _sgmTCams.size(); c++)
    {
        // depths of all meaningful points on the principal ray of the reference camera regarding the target camera tc
        StaticVector<float>* tcdepths = _sp->cps.getDepthsRcTc(_rc, _sgmTCams[c], _scale, midDepth, _sp->rcTcDepthsHalfLimit);
        if(sizeOfStaticVector<float>(tcdepths) < 50)
        {
            // fallback if we don't have enough valid samples over the epipolar line
            if(tcdepths != nullptr)
            {
                delete tcdepths;
                tcdepths = nullptr;
            }
            float avMinDist, avMidDist, avMaxDist;
            _sp->cps.getMinMaxdepths(_rc, _sgmTCams, avMinDist, avMidDist, avMaxDist);
            tcdepths = _sp->cps.getDepthsByPixelSize(_rc, avMinDist, avMidDist, avMaxDist, _scale, _sp->rcDepthsCompStep);

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
            tcamsNew.push_back(_sgmTCams[c]);
        }
    }

    _sgmTCams = tcamsNew;

    return alldepths;
}

void SemiGlobalMatchingRc::computeDepthsTcamsLimits(StaticVector<StaticVector<float>*>* alldepths)
{
    _depthsTcamsLimits.resize(_sgmTCams.size());

    for(int c = 0; c < _sgmTCams.size(); c++)
    {
        const float d1 = (*(*alldepths)[c])[0];
        const float d2 = (*(*alldepths)[c])[(*alldepths)[c]->size() - 1];

        int id1 = _depths.indexOfNearestSorted(d1);
        int id2 = _depths.indexOfNearestSorted(d2);

        if(id1 == -1)
            id1 = 0;

        if(id2 == -1)
            id2 = _depths.size() - 1;

        // clamp to keep only the closest depths if we have too much inputs (> maxDepthsToSweep)
        id2 = std::min(id1 + _sp->maxDepthsToSweep - 1, id2);
        _depthsTcamsLimits[c] = Pixel(id1, id2 - id1 + 1);
    }
}

void SemiGlobalMatchingRc::computeDepthsAndResetTCams()
{
    std::size_t nbObsDepths;
    float minObsDepth, maxObsDepth, midObsDepth;
    _sp->mp->getMinMaxMidNbDepth(_rc, minObsDepth, maxObsDepth, midObsDepth, nbObsDepths, _sp->seedsRangePercentile);

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

    if(!_sp->useSeedsToCompDepthsToSweep)
    {
        computeDepths(minDepthAll, maxDepthAll, alldepths);
        if(_sp->saveDepthsToSweepToTxtForVis)
        {
            std::string fn = _sp->mp->getDepthMapsFolder() + std::to_string(_sp->mp->getViewId(_rc)) + "depthsAll.txt";
            FILE* f = fopen(fn.c_str(), "w");
            for(int j = 0; j < _depths.size(); j++)
            {
                float depth = _depths[j];
                fprintf(f, "%f\n", depth);
            }
            fclose(f);
        }
        selectBestDepthsRange(_sp->maxDepthsToStore, alldepths);
    }
    else
    {
        float minDepth = minDepthAll;
        float maxDepth = maxDepthAll;

        // if we get enough information from seeds, adjust min/maxDepth
        if(nbObsDepths > 100)
        {
            minDepth = minObsDepth * (1.0f - _sp->seedsRangeInflate);
            maxDepth = maxObsDepth * (1.0f + _sp->seedsRangeInflate);

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
        computeDepths(minDepth, maxDepth, alldepths);

        if(_sp->saveDepthsToSweepToTxtForVis)
        {
            std::string fn = _sp->mp->getDepthMapsFolder() + std::to_string(_sp->mp->getViewId(_rc)) + "depthsAll.txt";
            FILE* f = fopen(fn.c_str(), "w");
            for(int j = 0; j < _depths.size(); j++)
            {
                float depth = _depths[j];
                fprintf(f, "%f\n", depth);
            }
            fclose(f);
        }

        // filter out depths if computeDepths gave too many values
        selectBestDepthsRange(_sp->maxDepthsToStore, alldepths);
    }

    // fill depthsTcamsLimits member variable with index range of depths to sweep
    computeDepthsTcamsLimits(alldepths);

    if(_sp->saveDepthsToSweepToTxtForVis)
    {
        std::string fn = _sp->mp->getDepthMapsFolder() + std::to_string(_sp->mp->getViewId(_rc)) + "depthsTcamsLimits.txt";
        FILE* f = fopen(fn.c_str(), "w");
        for(int j = 0; j < _depthsTcamsLimits.size(); j++)
        {
            Pixel l = _depthsTcamsLimits[j];
            // fprintf(f,"%f %f\n",(*depths)[l.x],(*depths)[l.x+l.y-1]);
            fprintf(f, "%i %i\n", l.x, l.y);
        }
        fclose(f);
    }

    if(_sp->saveDepthsToSweepToTxtForVis)
    {
        std::string fn = _sp->mp->getDepthMapsFolder() + std::to_string(_sp->mp->getViewId(_rc)) + "depths.txt";
        FILE* f = fopen(fn.c_str(), "w");
        for(int j = 0; j < _depths.size(); j++)
        {
            float depth = _depths[j];
            fprintf(f, "%f\n", depth);
        }
        fclose(f);
    }

    if(_sp->saveDepthsToSweepToTxtForVis)
    {
        for(int i = 0; i < alldepths->size(); i++)
        {
            std::string fn = _sp->mp->getDepthMapsFolder() + std::to_string(_sp->mp->getViewId(_rc)) + "depths" + mvsUtils::num2str(i) + ".txt";
            FILE* f = fopen(fn.c_str(), "w");
            for(int j = 0; j < (*alldepths)[i]->size(); j++)
            {
                float depth = (*(*alldepths)[i])[j];
                fprintf(f, "%f\n", depth);
            }
            fclose(f);
        }
    }

    if(_sp->saveDepthsToSweepToTxtForVis)
    {
        OrientedPoint rcplane;
        rcplane.p = _sp->mp->CArr[_rc];
        rcplane.n = _sp->mp->iRArr[_rc] * Point3d(0.0, 0.0, 1.0);
        rcplane.n = rcplane.n.normalize();

        std::string fn = _sp->mp->getDepthMapsFolder() + std::to_string(_sp->mp->getViewId(_rc)) + "rcDepths.txt";
        FILE* f = fopen(fn.c_str(), "w");
        float depth = minDepthAll;
        while(depth < maxDepthAll)
        {
            fprintf(f, "%f\n", depth);
            Point3d p = rcplane.p + rcplane.n * depth;
            depth = depth + _sp->mp->getCamPixelSize(p, _rc);
        }
        fclose(f);
    }

    if(_sp->mp->verbose)
        ALICEVISION_LOG_DEBUG("rc depths: " << _depths.size());

    deleteArrayOfArrays<float>(&alldepths);
}

void SemiGlobalMatchingRc::getSubDepthsForTCam( int tcamid, std::vector<float>& out )
{
    out.resize(_depthsTcamsLimits[tcamid].y);

    for(int i = 0; i<_depthsTcamsLimits[tcamid].y; i++ )
    {
        out[i] = _depths[_depthsTcamsLimits[tcamid].x + i];
    }
}

bool SemiGlobalMatchingRc::sgmrc(bool checkIfExists)
{
    if(_sp->mp->verbose)
      ALICEVISION_LOG_DEBUG("SGM (rc: " << (_rc + 1) << " / " << _sp->mp->ncams << ")");

    if(_sgmTCams.size() == 0)
    {
      return false;
    }

    if((mvsUtils::FileExists(_sp->getSGM_idDepthMapFileName(_sp->mp->getViewId(_rc), _scale, _step))) && (checkIfExists))
    {
      ALICEVISION_LOG_INFO("Already computed: " + _sp->getSGM_idDepthMapFileName(_sp->mp->getViewId(_rc), _scale, _step));
      return false;
    }

    long tall = clock();

    const int volDimX = _width;
    const int volDimY = _height;
    const int volDimZ = _depths.size();
    float volumeMBinGPUMem = 0.0f;

    StaticVector<unsigned char>* simVolume = nullptr;

    StaticVectorBool* rcSilhoueteMap = nullptr;
    if(_sp->useSilhouetteMaskCodedByColor)
    {
        rcSilhoueteMap = new StaticVectorBool();
        rcSilhoueteMap->reserve(_width * _height);
        rcSilhoueteMap->resize_with(_width * _height, true);
        _sp->cps.getSilhoueteMap(rcSilhoueteMap, _scale, _step, _sp->silhouetteMaskColor, _rc);
    }

    {
        std::vector<float> subDepths;
        getSubDepthsForTCam(0, subDepths);
        SemiGlobalMatchingRcTc srt(subDepths, _rc, _sgmTCams[0], _scale, _step, _sp, rcSilhoueteMap);
        simVolume = srt.computeDepthSimMapVolume(volumeMBinGPUMem, _sgmWsh, _sgmGammaC, _sgmGammaP);
    }

    // recompute to all depths
    volumeMBinGPUMem = ((volumeMBinGPUMem / (float)_depthsTcamsLimits[0].y) * (float)volDimZ);

    SemiGlobalMatchingVolume* svol = new SemiGlobalMatchingVolume(volumeMBinGPUMem, volDimX, volDimY, volDimZ, _sp);
    svol->copyVolume(simVolume, _depthsTcamsLimits[0].x, _depthsTcamsLimits[0].y);
    delete simVolume;

    for(int c = 1; c < _sgmTCams.size(); c++)
    {
        std::vector<float> subDepths;
        getSubDepthsForTCam(c, subDepths);
        SemiGlobalMatchingRcTc* srt = new SemiGlobalMatchingRcTc(subDepths, _rc, _sgmTCams[c], _scale, _step, _sp, rcSilhoueteMap);
        simVolume = srt->computeDepthSimMapVolume(volumeMBinGPUMem, _sgmWsh, _sgmGammaC, _sgmGammaP);
        delete srt;
        svol->addVolumeSecondMin(simVolume,_depthsTcamsLimits[c].x,_depthsTcamsLimits[c].y);
        delete simVolume;
    }

    //if(_sp->exportIntermediateResults)
    //  svol->exportVolume(*_depths, _rc, _scale, _step, _sp->mp->getDepthMapsFolder() + std::to_string(_sp->mp->getViewId(_rc)) + "_vol_beforeReduction.abc");

    // reduction of 'volume' (X, Y, Z) into 'volumeStepZ' (X, Y, Z/step)
    svol->cloneVolumeSecondStepZ();

    if(_sp->exportIntermediateResults)
    {
      svol->exportVolumeStep(_depths, _rc, _scale, _step, _sp->mp->getDepthMapsFolder() + std::to_string(_sp->mp->getViewId(_rc)) + "_vol_afterReduction.abc");
      svol->export9PCSV(_depths, _rc, _scale, _step, "afterReduction", _sp->mp->getDepthMapsFolder() + std::to_string(_sp->mp->getViewId(_rc)) + "_9p.csv");
    }

    // filter on the 3D volume to weight voxels based on their neighborhood strongness.
    // so it downweights local minimums that are not supported by their neighborhood.
    // this is here for experimental reason ... to show how SGGC work on non
    // optimized depthmaps ... it must equals to true in normal case
    if(_sp->doSGMoptimizeVolume)
        svol->SGMoptimizeVolumeStepZ(_rc, _step, 0, 0, _scale);

    if(_sp->exportIntermediateResults)
    {
      svol->exportVolumeStep(_depths, _rc, _scale, _step, _sp->mp->getDepthMapsFolder() + std::to_string(_sp->mp->getViewId(_rc)) + "_vol_afterFiltering.abc");
      svol->export9PCSV(_depths, _rc, _scale, _step, "afterFiltering", _sp->mp->getDepthMapsFolder() + std::to_string(_sp->mp->getViewId(_rc)) + "_9p.csv");
    }

    // for each pixel: choose the voxel with the minimal similarity value
    const int zborder = 2;
    _volumeBestIdVal = svol->getOrigVolumeBestIdValFromVolumeStepZ(zborder);

    delete svol;

    if(rcSilhoueteMap != nullptr)
    {
        for(int i = 0; i < _width * _height; i++)
        {
            if((*rcSilhoueteMap)[i])
            {
                (*_volumeBestIdVal)[i].id = 0;
                (*_volumeBestIdVal)[i].value = 1.0f;
            }
        }
        delete rcSilhoueteMap;
        rcSilhoueteMap = nullptr;
    }

    mvsUtils::printfElapsedTime(tall, "SGM (rc: " + mvsUtils::num2str(_rc) + " / " + mvsUtils::num2str(_sp->mp->ncams) + ")");

    if(_sp->exportIntermediateResults)
    {
        DepthSimMap* depthSimMapFinal = _sp->getDepthSimMapFromBestIdVal(_width, _height, _volumeBestIdVal, _scale, _step, _rc, zborder, _depths);
        depthSimMapFinal->saveToImage(_sp->mp->getDepthMapsFolder() + "sgm_" + std::to_string(_sp->mp->getViewId(_rc)) + "_" + "scale" + mvsUtils::num2str(depthSimMapFinal->scale) + "_step" + mvsUtils::num2str(depthSimMapFinal->step) + ".png", 1.0f);
        delete depthSimMapFinal;

        std::vector<unsigned short> volumeBestId(_volumeBestIdVal->size());
        for(int i = 0; i < _volumeBestIdVal->size(); i++)
          volumeBestId.at(i) = std::max(0, (*_volumeBestIdVal)[i].id);
        imageIO::writeImage(_sp->getSGM_idDepthMapFileName(_sp->mp->getViewId(_rc), _scale, _step), _width, _height, volumeBestId, imageIO::EImageQuality::LOSSLESS, imageIO::EImageColorSpace::NO_CONVERSION);
    }
    return true;
}

} // namespace depthMap
} // namespace aliceVision
