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
#include <aliceVision/imageIO/image.hpp>
#include <aliceVision/imageIO/imageScaledColors.hpp>
#include <aliceVision/alicevision_omp.hpp>

#include <boost/filesystem.hpp>

#include <iostream>
#include <sstream>

namespace aliceVision {
namespace depthMap {

namespace bfs = boost::filesystem;

SemiGlobalMatchingRc::SemiGlobalMatchingRc(bool doComputeDepthsAndResetTCams, int _rc, int _scale, int _step, SemiGlobalMatchingParams* _sp)
    : rc( _rc )
    , scale( _scale )
    , step( _step )
{
    sp = _sp;

    w = sp->mp->getWidth(rc) / (scale * step);
    h = sp->mp->getHeight(rc) / (scale * step);

    int nnearestcams = sp->mp->userParams.get<int>("semiGlobalMatching.maxTCams", 10);
    tcams = sp->pc->findNearestCamsFromLandmarks(rc, nnearestcams);

    wsh = sp->mp->userParams.get<int>("semiGlobalMatching.wsh", 4);
    gammaC = (float)sp->mp->userParams.get<double>("semiGlobalMatching.gammaC", 5.5);
    gammaP = (float)sp->mp->userParams.get<double>("semiGlobalMatching.gammaP", 8.0);

    const IndexT viewId = sp->mp->getViewId(rc);

    tcamsFileName = sp->getSGM_tcamsFileName(viewId);
    depthsFileName = sp->getSGM_depthsFileName(viewId);
    depthsTcamsLimitsFileName =  sp->mp->getDepthMapFolder() + std::to_string(viewId) + "_depthsTcamsLimits.bin";
    SGM_depthMapFileName = sp->getSGM_depthMapFileName(viewId, scale, step);
    SGM_simMapFileName = sp->getSGM_simMapFileName(viewId, scale, step);
    SGM_idDepthMapFileName = sp->getSGM_idDepthMapFileName(viewId, scale, step);

    depths = nullptr;
    depthsTcamsLimits.clear();

    if(doComputeDepthsAndResetTCams)
    {
        computeDepthsAndResetTCams();
    }
    else
    {
        depths = nullptr;
        depthsTcamsLimits.clear();
        if(mvsUtils::FileExists(tcamsFileName) && mvsUtils::FileExists(depthsFileName) && mvsUtils::FileExists(depthsTcamsLimitsFileName))
        {
            loadArrayFromFile<int>( tcams, tcamsFileName, true);
            depths = loadArrayFromFile<float>(depthsFileName);
            loadArrayFromFile<Pixel>( depthsTcamsLimits,  depthsTcamsLimitsFileName );
        }
    }
}

SemiGlobalMatchingRc::~SemiGlobalMatchingRc()
{
    delete depths;
}

bool SemiGlobalMatchingRc::selectBestDepthsRange(int nDepthsThr, StaticVector<float>* rcSeedsDistsAsc)
{
    if(depths->size() <= nDepthsThr)
    {
        return true;
    }

    StaticVector<int>* votes = new StaticVector<int>();
    votes->reserve(depths->size() - nDepthsThr);
    for(int i = 0; i < depths->size() - nDepthsThr; i++)
    {
        float d1 = (*depths)[i];
        float d2 = (*depths)[i + nDepthsThr - 1];
        int id1 = rcSeedsDistsAsc->indexOfNearestSorted(d1);
        if(d1 < (*rcSeedsDistsAsc)[0])
        {
            id1 = 0;
        }
        int id2 = rcSeedsDistsAsc->indexOfNearestSorted(d2);
        if(d2 > (*rcSeedsDistsAsc)[rcSeedsDistsAsc->size() - 1])
        {
            id2 = rcSeedsDistsAsc->size() - 1;
        }

        if((id1 > -1) && (id2 > -1))
        {
            votes->push_back(abs(id2 - id1));
        }
        else
        {
            votes->push_back(0);
        }
    }

    StaticVector<float>* depthsNew = new StaticVector<float>();
    depthsNew->reserve(nDepthsThr);

    int id1 = votes->maxValId();
    int id2 = id1 + nDepthsThr - 1;

    // printf("id1 %i, id2 %i\n",id1,id2);

    for(int i = id1; i <= id2; i++)
    {
        depthsNew->push_back((*depths)[i]);
    }

    delete votes;

    delete depths;
    depths = depthsNew;

    return true;
}

bool SemiGlobalMatchingRc::selectBestDepthsRange(int nDepthsThr, StaticVector<StaticVector<float>*>* alldepths)
{
    if(depths->size() <= nDepthsThr)
    {
        return true;
    }

    StaticVector<float>* votes = new StaticVector<float>();
    votes->reserve(depths->size() - nDepthsThr);

    for(int i = 0; i < depths->size() - nDepthsThr; i++)
    {
        float d1 = (*depths)[i];
        float d2 = (*depths)[i + nDepthsThr - 1];

        float overlap = 0.0f;
        for(int c = 0; c < alldepths->size(); c++)
        {
            StaticVector<float>* tcDepths = (*alldepths)[c];
            float dd1 = std::max(d1, (*tcDepths)[0]);
            float dd2 = std::min(d2, (*tcDepths)[tcDepths->size() - 1]);
            if(dd1 < dd2)
            {
                overlap += dd2 - dd1;
            }
        }
        votes->push_back(overlap);
    }

    StaticVector<float>* depthsNew = new StaticVector<float>();
    depthsNew->reserve(nDepthsThr);

    int id1 = votes->maxValId();
    int id2 = id1 + nDepthsThr - 1;

    // printf("id1 %i, id2 %i\n",id1,id2);

    for(int i = id1; i <= id2; i++)
    {
        depthsNew->push_back((*depths)[i]);
    }

    delete votes;

    delete depths;
    depths = depthsNew;

    return true;
}

float SemiGlobalMatchingRc::getMinTcStepAtDepth(float depth, float minDepth, float maxDepth,
                                     StaticVector<StaticVector<float>*>* alldepths)
{
    float minTcStep = maxDepth - minDepth;

    // For each tc depths
    for(int i = 0; i < alldepths->size(); i++)
    {
        StaticVector<float>* tcDepths = (*alldepths)[i];
        // Get the tc depth closest to the current depth
        int id = tcDepths->indexOfNearestSorted(depth);
        // Continue on no result or last element (we need id + 1)
        if(id < 0 || id >= tcDepths->size() - 1)
            continue;

        // Consider the enclosing depth range 
        float did = (*tcDepths)[id];     // closest depth
        float nid = (*tcDepths)[id + 1]; // next depth
        float tcStep = fabs(did - nid);  // [closest; next] depths distance
        // keep this value if smallest step so far
        minTcStep = std::min(minTcStep, tcStep);

        // WARNING useless, 'id' is < tcDepths->size() - 1
        /*if(depth >= (*tcDepths)[tcDepths->size() - 1])
        {
            printf("WARNING %i %i %f %f %f %f\n", id, tcDepths->size(), did, nid, depth,
                    (*tcDepths)[tcDepths->size() - 1]);
        }*/
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

    depths = new StaticVector<float>();
    depths->reserve(maxNdetphs);

    {
        float depth = minDepth;
        while(depth < maxDepth)
        {
            depths->push_back(depth);
            depth += getMinTcStepAtDepth(depth, minDepth, maxDepth, alldepths);
        }
    }
}

/**
 * @brief Compute depths of the principal ray of reference camera rc visible by a pixel in a target camera tc
 *        providing meaningful 3d information.
 */
StaticVector<StaticVector<float>*>* SemiGlobalMatchingRc::computeAllDepthsAndResetTCams(float midDepth)
{
    StaticVector<int> tcamsNew;
    StaticVector<StaticVector<float>*>* alldepths = new StaticVector<StaticVector<float>*>();
    alldepths->reserve(tcams.size());

    for(int c = 0; c < tcams.size(); c++)
    {
        // depths of all meaningful points on the principal ray of the reference camera regarding the target camera tc
        StaticVector<float>* tcdepths = sp->cps.getDepthsRcTc(rc, tcams[c], scale, midDepth, sp->rcTcDepthsHalfLimit);
        if(sizeOfStaticVector<float>(tcdepths) < 50)
        {
            // fallback if we don't have enough valid samples over the epipolar line
            if(tcdepths != nullptr)
            {
                delete tcdepths;
                tcdepths = nullptr;
            }
            float avMinDist, avMidDist, avMaxDist;
            sp->cps.getMinMaxdepths(rc, tcams, avMinDist, avMidDist, avMaxDist);
            tcdepths = sp->cps.getDepthsByPixelSize(rc, avMinDist, avMidDist, avMaxDist, scale, sp->rcDepthsCompStep);

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
            tcamsNew.push_back(tcams[c]);
        }
    }

    tcams = tcamsNew;

    return alldepths;
}

/**
 * @ brief Fill depthsTcamsLimits member variable with index range of depths to sweep
 */
void SemiGlobalMatchingRc::computeDepthsTcamsLimits(StaticVector<StaticVector<float>*>* alldepths)
{
    depthsTcamsLimits.resize( tcams.size() );

    for(int c = 0; c < tcams.size(); c++)
    {
        float d1 = (*(*alldepths)[c])[0];
        float d2 = (*(*alldepths)[c])[(*alldepths)[c]->size() - 1];
        int id1 = depths->indexOfNearestSorted(d1);
        if(id1 == -1)
        {
            id1 = 0;
        }
        int id2 = depths->indexOfNearestSorted(d2);
        if(id2 == -1)
        {
            id2 = depths->size() - 1;
        }
        // clamp to keep only the closest depths if we have too much inputs (> maxDepthsToSweep)
        id2 = std::min(id1 + sp->maxDepthsToSweep - 1, id2);
        depthsTcamsLimits[c] = Pixel(id1, id2 - id1 + 1);
    }
}

void SemiGlobalMatchingRc::computeDepthsAndResetTCams()
{
    std::size_t nbObsDepths;
    float minObsDepth, maxObsDepth, midObsDepth;
    sp->mp->getMinMaxMidNbDepth(rc, minObsDepth, maxObsDepth, midObsDepth, nbObsDepths, sp->seedsRangePercentile);

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

    if(!sp->useSeedsToCompDepthsToSweep)
    {
        computeDepths(minDepthAll, maxDepthAll, alldepths);
        if(sp->saveDepthsToSweepToTxtForVis)
        {
            std::string fn = tmpDir + std::to_string(sp->mp->getViewId(rc)) + "depthsAll.txt";
            FILE* f = fopen(fn.c_str(), "w");
            for(int j = 0; j < depths->size(); j++)
            {
                float depth = (*depths)[j];
                fprintf(f, "%f\n", depth);
            }
            fclose(f);
        }
        selectBestDepthsRange(sp->maxDepthsToStore, alldepths);
    }
    else
    {
        float minDepth = minDepthAll;
        float maxDepth = maxDepthAll;

        // If we get enough information from seeds, adjust min/maxDepth
        if(nbObsDepths > 100)
        {
            minDepth = minObsDepth * (1.0f - sp->seedsRangeInflate);
            maxDepth = maxObsDepth * (1.0f + sp->seedsRangeInflate);

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

        // Build the list of "best" depths for rc, from all tc cameras depths
        computeDepths(minDepth, maxDepth, alldepths);

        if(sp->saveDepthsToSweepToTxtForVis)
        {
            std::string fn = tmpDir + std::to_string(sp->mp->getViewId(rc)) + "depthsAll.txt";
            FILE* f = fopen(fn.c_str(), "w");
            for(int j = 0; j < depths->size(); j++)
            {
                float depth = (*depths)[j];
                fprintf(f, "%f\n", depth);
            }
            fclose(f);
        }

        // Filter out depths if computeDepths gave too many values
        selectBestDepthsRange(sp->maxDepthsToStore, alldepths);
    }

    // fill depthsTcamsLimits member variable with index range of depths to sweep
    computeDepthsTcamsLimits(alldepths);

    if(sp->saveDepthsToSweepToTxtForVis)
    {
        std::string fn = tmpDir + std::to_string(sp->mp->getViewId(rc)) + "depthsTcamsLimits.txt";
        FILE* f = fopen(fn.c_str(), "w");
        for(int j = 0; j < depthsTcamsLimits.size(); j++)
        {
            Pixel l = depthsTcamsLimits[j];
            // fprintf(f,"%f %f\n",(*depths)[l.x],(*depths)[l.x+l.y-1]);
            fprintf(f, "%i %i\n", l.x, l.y);
        }
        fclose(f);
    }

    if(sp->saveDepthsToSweepToTxtForVis)
    {
        std::string fn = tmpDir + std::to_string(sp->mp->getViewId(rc)) + "depths.txt";
        FILE* f = fopen(fn.c_str(), "w");
        for(int j = 0; j < depths->size(); j++)
        {
            float depth = (*depths)[j];
            fprintf(f, "%f\n", depth);
        }
        fclose(f);
    }

    if(sp->saveDepthsToSweepToTxtForVis)
    {
        for(int i = 0; i < alldepths->size(); i++)
        {
            std::string fn = tmpDir + std::to_string(sp->mp->getViewId(rc)) + "depths" + mvsUtils::num2str(i) + ".txt";
            FILE* f = fopen(fn.c_str(), "w");
            for(int j = 0; j < (*alldepths)[i]->size(); j++)
            {
                float depth = (*(*alldepths)[i])[j];
                fprintf(f, "%f\n", depth);
            }
            fclose(f);
        }
    }

    if(sp->saveDepthsToSweepToTxtForVis)
    {
        OrientedPoint rcplane;
        rcplane.p = sp->mp->CArr[rc];
        rcplane.n = sp->mp->iRArr[rc] * Point3d(0.0, 0.0, 1.0);
        rcplane.n = rcplane.n.normalize();

        std::string fn = tmpDir + std::to_string(sp->mp->getViewId(rc)) + "rcDepths.txt";
        FILE* f = fopen(fn.c_str(), "w");
        float depth = minDepthAll;
        while(depth < maxDepthAll)
        {
            fprintf(f, "%f\n", depth);
            Point3d p = rcplane.p + rcplane.n * depth;
            depth = depth + sp->mp->getCamPixelSize(p, rc);
        }
        fclose(f);
    }

    if(sp->mp->verbose)
        ALICEVISION_LOG_DEBUG("rc depths: " << depths->size());

    deleteArrayOfArrays<float>(&alldepths);
}

bool SemiGlobalMatchingRc::sgmrc(bool checkIfExists)
{
    if(sp->mp->verbose)
        ALICEVISION_LOG_DEBUG("sgmrc: processing " << (rc + 1) << " of " << sp->mp->ncams << ".");

    if(tcams.size() == 0)
    {
        return false;
    }

    if((mvsUtils::FileExists(SGM_idDepthMapFileName)) && (checkIfExists))
    {
        return false;
    }

    long tall = clock();

    const int volDimX = w;
    const int volDimY = h;
    const int volDimZ = depths->size();

    sp->cps.cameraToDevice( rc, tcams );

//
// FORCE_ZDIM_LIMIT exists only to test volume slicing for low-memory CUDA cards
// also on high-memory cards. Make sure to #undef for releases.
//
// #define FORCE_ZDIM_LIMIT 32
#undef  FORCE_ZDIM_LIMIT
#ifndef FORCE_ZDIM_LIMIT
    const long gpu_bytes_reqd_per_plane = volDimX * volDimY * sizeof(float) * 2; // safety margin 100%
    const long gpu_bytes_free = sp->cps.getDeviceMemoryInfo().x * 1024 * 1024;
    int        zDimsAtATime = depths->size();
    const int  camsAtATime  = tcams.size();
    if( gpu_bytes_reqd_per_plane * zDimsAtATime * camsAtATime > gpu_bytes_free )
    {
        while( zDimsAtATime > 1 && gpu_bytes_reqd_per_plane * zDimsAtATime * camsAtATime > gpu_bytes_free )
        {
            zDimsAtATime /= 2;
        }
    }
    if(sp->mp->verbose)
        ALICEVISION_LOG_DEBUG("bytes free on GPU: " << gpu_bytes_free
                           << "(" << (int)(gpu_bytes_free/1024.0f/1024.0f) << " MB)"<< std::endl
                           << "    estimated req'd bytes/plane: " << gpu_bytes_reqd_per_plane << std::endl
                           << "    estimated dims at a time: " << zDimsAtATime << std::endl
                           << "    cams at a time: " << camsAtATime << std::endl
                           << "    estimated total: " << gpu_bytes_reqd_per_plane * zDimsAtATime * camsAtATime
                           << " (" << (int)(gpu_bytes_reqd_per_plane * zDimsAtATime * camsAtATime/1024.0f/1024.0f) << " MB)" );
#else
    int zDimsAtATime = FORCE_ZDIM_LIMIT; // for example FORCE_ZDIM_LIMIT=32
#endif

    StaticVectorBool* rcSilhoueteMap = nullptr;
    if(sp->useSilhouetteMaskCodedByColor)
    {
        rcSilhoueteMap = new StaticVectorBool();
        rcSilhoueteMap->reserve(w * h);
        rcSilhoueteMap->resize_with(w * h, true);
        sp->cps.getSilhoueteMap(rcSilhoueteMap, scale, step, sp->silhouetteMaskColor, rc);
    }

    if(sp->mp->verbose)
    {
        std::ostringstream ostr;
        ostr << "In " << __FUNCTION__ << std::endl
             << "    rc camera " << rc << " has depth " << depths->size() << std::endl;
        for( int c = 0; c < tcams.size(); c++ )
            ostr << "    tc camera " << tcams[c]
                 << " uses " << depthsTcamsLimits[c].y << " depths" << std::endl;

        ALICEVISION_LOG_DEBUG( ostr.str() );
    }

    std::vector<StaticVector<unsigned char> > simVolume;
    simVolume.resize( tcams.size() );

    /* request this device to allocate
     *   (max_img - 1) * X * Y * dims_at_a_time * sizeof(float)
     * of device memory.
     */
    if(sp->mp->verbose)
    {
        int devid;
        cudaGetDevice( &devid );
        ALICEVISION_LOG_DEBUG( "Allocating " << tcams.size()
            << " times " << volDimX << " " << volDimY << " "
            << zDimsAtATime << " on device " << devid );
    }
    std::vector<CudaDeviceMemoryPitched<float, 3>*> volume_tmp_on_gpu;
    sp->cps.allocTempVolume( volume_tmp_on_gpu,
                             tcams.size(),
                             volDimX,
                             volDimY,
                             zDimsAtATime );

    std::vector<int> index_set( tcams.size() );
    for(int c = 0; c < tcams.size(); c++)
    {
        index_set[c] = c;
    }
    SemiGlobalMatchingRcTc srt( index_set,
                                depths->getData(),
                                depthsTcamsLimits.getData(),
                                rc, tcams, scale, step, zDimsAtATime, sp, rcSilhoueteMap );
    srt.computeDepthSimMapVolume( simVolume, volume_tmp_on_gpu, wsh, gammaC, gammaP );

    sp->cps.freeTempVolume( volume_tmp_on_gpu );

    index_set.erase( index_set.begin() );
    SemiGlobalMatchingVolume svol( volDimX, volDimY, volDimZ, zDimsAtATime, sp );
    svol.copyVolume( simVolume[0], depthsTcamsLimits[0] );
    svol.addVolumeSecondMin( index_set, simVolume, depthsTcamsLimits );

    // Reduction of 'volume' (X, Y, Z) into 'volumeStepZ' (X, Y, Z/step)
    svol.cloneVolumeSecondStepZ();

    // Filter on the 3D volume to weight voxels based on their neighborhood strongness.
    // So it downweights local minimums that are not supported by their neighborhood.
    if(sp->doSGMoptimizeVolume) // this is here for experimental reason ... to show how SGGC work on non
                                // optimized depthmaps ... it must equals to true in normal case
    {
        svol.SGMoptimizeVolumeStepZ(rc, step, scale);
    }

    // For each pixel: choose the voxel with the minimal similarity value
    int zborder = 2;
    StaticVector<IdValue>* volumeBestIdVal = svol.getOrigVolumeBestIdValFromVolumeStepZ(zborder);
    svol.freeMem();

    if(rcSilhoueteMap != nullptr)
    {
        for(int i = 0; i < w * h; i++)
        {
            if((*rcSilhoueteMap)[i])
            {
                (*volumeBestIdVal)[i].id = 0;
                (*volumeBestIdVal)[i].value = 1.0f;
            }
        }
        delete rcSilhoueteMap;
        rcSilhoueteMap = nullptr;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    saveArrayToFile<float>(depthsFileName, depths);
    saveArrayToFile<int>(tcamsFileName, tcams);
    saveArrayToFile<Pixel>(depthsTcamsLimitsFileName, depthsTcamsLimits);

    DepthSimMap* depthSimMapFinal =
        sp->getDepthSimMapFromBestIdVal(w, h, volumeBestIdVal, scale, step, rc, zborder, depths);

    // Save to :
    //  - SGM_{RC}_scaleX_stepN_simMap.bin
    //  - SGM_{RC}_scaleX_stepN_depthMap.bin
    //depthSimMapFinal->saveToBin(SGM_depthMapFileName, SGM_simMapFileName);

    // Save to :
    //  - {RC}_simMap_scaleX.exr
    //  - {RC}_dephMap_scaleX.exr
    // depthSimMapFinal->save(rc, tcams);

    {
        std::vector<unsigned short> volumeBestId(volumeBestIdVal->size());
        for(int i = 0; i < volumeBestIdVal->size(); i++)
            volumeBestId.at(i) = std::max(0, (*volumeBestIdVal)[i].id);

        imageIO::writeImage(SGM_idDepthMapFileName, volDimX, volDimY, volumeBestId);

        if(sp->visualizeDepthMaps)
            imageIO::writeImageScaledColors("visualize_" + SGM_idDepthMapFileName, volDimX, volDimY, 0, depths->size(), volumeBestId.data(), true);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    mvsUtils::printfElapsedTime(tall, "PSSGM rc " + mvsUtils::num2str(rc) + " of " + mvsUtils::num2str(sp->mp->ncams));

    if(sp->visualizeDepthMaps)
        depthSimMapFinal->saveToImage(tmpDir + "SemiGlobalMatchingRc_SGM" + std::to_string(sp->mp->getViewId(rc)) + "_" + "scale" +
                                        mvsUtils::num2str(depthSimMapFinal->scale) + "step" + mvsUtils::num2str(depthSimMapFinal->step) +
                                        ".depthSimMap.png", 1.0f);


    delete depthSimMapFinal;
    delete volumeBestIdVal;

    return true;
}

void computeDepthMapsPSSGM(int CUDADeviceNo, mvsUtils::MultiViewParams* mp, mvsUtils::PreMatchCams* pc, const StaticVector<int>& cams)
{
    const int fileScale = 1; // input images scale (should be one)
    int sgmScale = mp->userParams.get<int>("semiGlobalMatching.scale", -1);
    int sgmStep = mp->userParams.get<int>("semiGlobalMatching.step", -1);

    if(sgmScale == -1)
    {
        // Compute the number of scales that will be used in the plane sweeping.
        // The highest scale should have a minimum resolution of 700x550.
        int width = mp->getMaxImageWidth();
        int height = mp->getMaxImageHeight();
        int scaleTmp = computeStep(mp, fileScale, (width > height ? 700 : 550), (width > height ? 550 : 700));
        sgmScale = std::min(2, scaleTmp);
        sgmStep = computeStep(mp, fileScale * sgmScale, (width > height ? 700 : 550), (width > height ? 550 : 700));
        ALICEVISION_LOG_INFO("PSSGM autoScaleStep: scale: " << sgmScale << ", step: " << sgmStep);
    }

    const int bandType = 0;
    
    // load images from files into RAM 
    mvsUtils::ImagesCache ic(mp, bandType, true);
    // load stuff on GPU memory and creates multi-level images and computes gradients
    PlaneSweepingCuda cps(CUDADeviceNo, ic, mp, pc, sgmScale);
    // init plane sweeping parameters
    SemiGlobalMatchingParams sp(mp, pc, cps);

    //////////////////////////////////////////////////////////////////////////////////////////

    for(const int rc : cams)
    {
        std::string depthMapFilepath = sp.getSGM_idDepthMapFileName(mp->getViewId(rc), sgmScale, sgmStep);
        if(!mvsUtils::FileExists(depthMapFilepath))
        {
            ALICEVISION_LOG_INFO("Compute depth map: " << depthMapFilepath);
            SemiGlobalMatchingRc psgr(true, rc, sgmScale, sgmStep, &sp);
            psgr.sgmrc();
        }
        else
        {
            ALICEVISION_LOG_INFO("Depth map already computed: " << depthMapFilepath);
        }
    }
}

void computeDepthMapsPSSGM(mvsUtils::MultiViewParams* mp, mvsUtils::PreMatchCams* pc, const StaticVector<int>& cams)
{
    int num_gpus = listCUDADevices(true);
    int num_cpu_threads = omp_get_num_procs();
    ALICEVISION_LOG_INFO("Number of GPU devices: " << num_gpus << ", number of CPU threads: " << num_cpu_threads);
    int numthreads = std::min(num_gpus, num_cpu_threads);

    int num_gpus_to_use = mp->userParams.get<int>("semiGlobalMatching.num_gpus_to_use", 0);
    if(num_gpus_to_use > 0)
    {
        numthreads = num_gpus_to_use;
    }

    if(numthreads == 1)
    {
        // The GPU sorting is determined by an environment variable named CUDA_DEVICE_ORDER
        // Possible values: FASTEST_FIRST (default) or PCI_BUS_ID
        const int CUDADeviceNo = 0;
        computeDepthMapsPSSGM(CUDADeviceNo, mp, pc, cams);
    }
    else
    {
        omp_set_num_threads(numthreads); // create as many CPU threads as there are CUDA devices
#pragma omp parallel
        {
            int cpu_thread_id = omp_get_thread_num();
            int CUDADeviceNo = cpu_thread_id % numthreads;
            ALICEVISION_LOG_INFO("CPU thread " << cpu_thread_id << " (of " << numthreads << ") uses CUDA device: " << CUDADeviceNo);

            int rcFrom = CUDADeviceNo * (cams.size() / numthreads);
            int rcTo = (CUDADeviceNo + 1) * (cams.size() / numthreads);
            if(CUDADeviceNo == numthreads - 1)
            {
                rcTo = cams.size();
            }
            StaticVector<int> subcams;
            subcams.reserve(cams.size());
            for(int rc = rcFrom; rc < rcTo; rc++)
            {
                subcams.push_back(cams[rc]);
            }
            computeDepthMapsPSSGM(cpu_thread_id, mp, pc, subcams);
        }
    }
}

} // namespace depthMap
} // namespace aliceVision
