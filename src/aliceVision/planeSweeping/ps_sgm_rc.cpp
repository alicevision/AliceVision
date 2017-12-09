// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "ps_sgm_rc.hpp"
#include "ps_sgm_rctc.hpp"
#include "ps_sgm_vol.hpp"

#include <aliceVision/common/fileIO.hpp>
#include <aliceVision/imageIO/imageScaledColors.hpp>
#include <aliceVision/omp.hpp>

#include <boost/filesystem.hpp>

#include <iostream>


namespace bfs = boost::filesystem;

ps_sgm_rc::ps_sgm_rc(bool doComputeDepthsAndResetTCams, int _rc, int _scale, int _step, ps_sgm_params* _sp)
{
    sp = _sp;

    outDir = sp->getSGMOutDir();
    if(!FolderExists(outDir))
    {
        bfs::create_directory(outDir);
    }

    tmpDir = sp->getSGMTmpDir();
    if(!FolderExists(tmpDir))
    {
        bfs::create_directory(tmpDir);
    }

    rc = _rc;
    scale = _scale;
    step = _step;

    w = sp->mp->mip->getWidth(rc) / (scale * step);
    h = sp->mp->mip->getHeight(rc) / (scale * step);

    int nnearestcams = sp->mp->mip->_ini.get<int>("semiGlobalMatching.maxTCams", 10);
    tcams = sp->pc->findNearestCamsFromSeeds(rc, nnearestcams);

    wsh = sp->mp->mip->_ini.get<int>("semiGlobalMatching.wsh", 4);
    gammaC = (float)sp->mp->mip->_ini.get<double>("semiGlobalMatching.gammaC", 5.5);
    gammaP = (float)sp->mp->mip->_ini.get<double>("semiGlobalMatching.gammaP", 8.0);

    sp->cps->verbose = sp->mp->verbose;

    tcamsFileName = sp->getSGM_tcamsFileName(rc);
    depthsFileName = sp->getSGM_depthsFileName(rc);
    depthsTcamsLimitsFileName = outDir + num2strFourDecimal(rc) + "depthsTcamsLimits.bin";
    SGM_depthMapFileName = sp->getSGM_depthMapFileName(rc, scale, step);
    SGM_simMapFileName = sp->getSGM_simMapFileName(rc, scale, step);
    SGM_idDepthMapFileName = sp->getSGM_idDepthMapFileName(rc, scale, step);

    depths = nullptr;
    depthsTcamsLimits = nullptr;

    if(doComputeDepthsAndResetTCams)
    {
        computeDepthsAndResetTCams();
    }
    else
    {
        delete tcams;
        tcams = nullptr;
        depths = nullptr;
        depthsTcamsLimits = nullptr;
        if(FileExists(tcamsFileName) && FileExists(depthsFileName) && FileExists(depthsTcamsLimitsFileName))
        {
            tcams = loadArrayFromFile<int>(tcamsFileName, true);
            depths = loadArrayFromFile<float>(depthsFileName);
            depthsTcamsLimits = loadArrayFromFile<pixel>(depthsTcamsLimitsFileName);
        }
    }
}

ps_sgm_rc::~ps_sgm_rc()
{
    delete tcams;
    delete depths;
    delete depthsTcamsLimits;
}

/**
 * @brief Depths of all seeds (regarding the camera plane and not the camera center).
 */
staticVector<float>* ps_sgm_rc::getTcSeedsRcPlaneDists(int rc, staticVector<int>* tcams)
{
    orientedPoint rcplane;
    rcplane.p = sp->mp->CArr[rc];
    rcplane.n = sp->mp->iRArr[rc] * point3d(0.0, 0.0, 1.0);
    rcplane.n = rcplane.n.normalize();

    int nTcSeeds = 0;
    for(int c = 0; c < tcams->size(); c++)
    {
        staticVector<seedPoint>* seeds;
        loadSeedsFromFile(&seeds, sp->mp->indexes[(*tcams)[c]], sp->mp->mip, EFileType::seeds);
        nTcSeeds += seeds->size();
        delete seeds;
    } // for c

    staticVector<float>* rcDists = new staticVector<float>(nTcSeeds);

    for(int c = 0; c < tcams->size(); c++)
    {
        staticVector<seedPoint>* seeds;
        loadSeedsFromFile(&seeds, sp->mp->indexes[(*tcams)[c]], sp->mp->mip, EFileType::seeds);
        for(int i = 0; i < seeds->size(); i++)
        {
            rcDists->push_back(pointPlaneDistance((*seeds)[i].op.p, rcplane.p, rcplane.n));
        }
        delete seeds;
    } // for c

    return rcDists;
}

bool ps_sgm_rc::selectBestDepthsRange(int nDepthsThr, staticVector<float>* rcSeedsDistsAsc)
{
    if(depths->size() <= nDepthsThr)
    {
        return true;
    }

    staticVector<int>* votes = new staticVector<int>(depths->size() - nDepthsThr);
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

    staticVector<float>* depthsNew = new staticVector<float>(nDepthsThr);

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

bool ps_sgm_rc::selectBestDepthsRange(int nDepthsThr, staticVector<staticVector<float>*>* alldepths)
{
    if(depths->size() <= nDepthsThr)
    {
        return true;
    }

    staticVector<float>* votes = new staticVector<float>(depths->size() - nDepthsThr);
    for(int i = 0; i < depths->size() - nDepthsThr; i++)
    {
        float d1 = (*depths)[i];
        float d2 = (*depths)[i + nDepthsThr - 1];

        float overlap = 0.0f;
        for(int c = 0; c < alldepths->size(); c++)
        {
            staticVector<float>* tcDepths = (*alldepths)[c];
            float dd1 = std::max(d1, (*tcDepths)[0]);
            float dd2 = std::min(d2, (*tcDepths)[tcDepths->size() - 1]);
            if(dd1 < dd2)
            {
                overlap += dd2 - dd1;
            }
        }
        votes->push_back(overlap);
    }

    staticVector<float>* depthsNew = new staticVector<float>(nDepthsThr);

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

float ps_sgm_rc::getMinTcStepAtDepth(float depth, float minDepth, float maxDepth,
                                     staticVector<staticVector<float>*>* alldepths)
{
    float minTcStep = maxDepth - minDepth;

    // For each tc depths
    for(int i = 0; i < alldepths->size(); i++)
    {
        staticVector<float>* tcDepths = (*alldepths)[i];
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

float ps_sgm_rc::getMeanTcStepAtDepth(float depth, float minDepth, float maxDepth,
                                      staticVector<staticVector<float>*>* alldepths)
{
    float meanTcStep = 0.0f;
    float n = 0.0f;
    for(int i = 0; i < alldepths->size(); i++)
    {
        int id = (*alldepths)[i]->indexOfNearestSorted(depth);
        if((id >= 0) && (id < (*alldepths)[i]->size() - 1))
        {
            float did = (*(*alldepths)[i])[id];
            float nid = (*(*alldepths)[i])[id + 1];
            float tcStep = fabs(did - nid);
            meanTcStep += tcStep;
            n += 1.0f;
        }
    }

    if(n > 0.0f)
    {
        return meanTcStep / n;
    }

    return maxDepth - minDepth;
}

void ps_sgm_rc::computeDepths(float minDepth, float maxDepth, staticVector<staticVector<float>*>* alldepths)
{
    int maxNdetphs = 0;
    {
        float depth = minDepth;
        while(depth < maxDepth)
        {
            maxNdetphs++;
            depth += getMinTcStepAtDepth(depth, minDepth, maxDepth, alldepths);
            // depth += getMeanTcStepAtDepth(depth, minDepth, maxDepth, alldepths);
        }
    }

    depths = new staticVector<float>(maxNdetphs);

    {
        float depth = minDepth;
        while(depth < maxDepth)
        {
            depths->push_back(depth);
            depth += getMinTcStepAtDepth(depth, minDepth, maxDepth, alldepths);
            // depth += getMeanTcStepAtDepth(depth, minDepth, maxDepth, alldepths);
        }
    }
}

/**
 * @brief Compute depths of the principal ray of reference camera rc visible by a pixel in a target camera tc
 *        providing meaningful 3d information.
 */
staticVector<staticVector<float>*>* ps_sgm_rc::computeAllDepthsAndResetTCams()
{
    /*
    for (int c=0;c<tcams->size();c++) {
            printf("%i %i\n",c,(*tcams)[c]);
    };
    */

    staticVector<int>* tcamsNew = new staticVector<int>(tcams->size());
    staticVector<staticVector<float>*>* alldepths = new staticVector<staticVector<float>*>(tcams->size());
    float midDepth = getCGDepthFromSeeds(sp->mp, rc);

    for(int c = 0; c < tcams->size(); c++)
    {
        // depths of all meaningful points on the principal ray of the reference camera regarding the target camera tc
        staticVector<float>* tcdepths = sp->cps->getDepthsRcTc(rc, (*tcams)[c], scale, midDepth, sp->rcTcDepthsHalfLimit);
        if(sizeOfStaticVector<float>(tcdepths) < 50)
        {
            // fallback if we don't have enough valid samples over the epipolar line
            if(tcdepths != nullptr)
            {
                delete tcdepths;
                tcdepths = nullptr;
            }
            float avMinDist, avMidDist, avMaxDist;
            sp->cps->getMinMaxdepths(rc, tcams, avMinDist, avMidDist, avMaxDist);
            tcdepths = sp->cps->getDepthsByPixelSize(rc, avMinDist, avMidDist, avMaxDist, scale, sp->rcDepthsCompStep);

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
            tcamsNew->push_back((*tcams)[c]);
        }
    }

    delete tcams;
    tcams = tcamsNew;

    /*
    for (int c=0;c<tcams->size();c++) {
            printf("%i %i\n",c,(*tcams)[c]);
    };
    */

    return alldepths;
}

/**
 * @ brief Fill depthsTcamsLimits member variable with index range of depths to sweep
 */
void ps_sgm_rc::computeDepthsTcamsLimits(staticVector<staticVector<float>*>* alldepths)
{
    depthsTcamsLimits = new staticVector<pixel>(tcams->size());
    for(int c = 0; c < tcams->size(); c++)
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
        depthsTcamsLimits->push_back(pixel(id1, id2 - id1 + 1));
    }
}

void ps_sgm_rc::computeDepthsAndResetTCams()
{
    // all depths from the principal ray provided by target cameras
    staticVector<staticVector<float>*>* alldepths = computeAllDepthsAndResetTCams();

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
            std::string fn = tmpDir + num2strFourDecimal(rc) + "depthsAll.txt";
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
        staticVector<float>* rcSeedsDistsAsc = getTcSeedsRcPlaneDists(rc, tcams);
        float minDepth = minDepthAll;
        float maxDepth = maxDepthAll;

        // If we get enough information from seeds, adjust min/maxDepth
        if(rcSeedsDistsAsc->size() > 100)
        {
            qsort(&(*rcSeedsDistsAsc)[0], rcSeedsDistsAsc->size(), sizeof(float), qSortCompareFloatAsc);

            minDepth = (*rcSeedsDistsAsc)[(int)(float)rcSeedsDistsAsc->size() * (sp->seedsRangePercentile)] *
                (1.0f - sp->seedsRangeInflate);
            
            maxDepth = (*rcSeedsDistsAsc)[(int)(float)rcSeedsDistsAsc->size() * (1.0f - sp->seedsRangePercentile)] *
                             (1.0f + sp->seedsRangeInflate);

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
            std::string fn = tmpDir + num2strFourDecimal(rc) + "depthsAll.txt";
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

        if(sp->saveDepthsToSweepToTxtForVis)
        {
            std::string fn = tmpDir + num2strFourDecimal(rc) + "rcSeedsDists.txt";
            FILE* f = fopen(fn.c_str(), "w");
            for(int j = 0; j < rcSeedsDistsAsc->size(); j++)
            {
                float depth = (*rcSeedsDistsAsc)[j];
                fprintf(f, "%f\n", depth);
            }
            fclose(f);
        }

        delete rcSeedsDistsAsc;
    }

    // fill depthsTcamsLimits member variable with index range of depths to sweep
    computeDepthsTcamsLimits(alldepths);

    if(sp->saveDepthsToSweepToTxtForVis)
    {
        std::string fn = tmpDir + num2strFourDecimal(rc) + "depthsTcamsLimits.txt";
        FILE* f = fopen(fn.c_str(), "w");
        for(int j = 0; j < depthsTcamsLimits->size(); j++)
        {
            pixel l = (*depthsTcamsLimits)[j];
            // fprintf(f,"%f %f\n",(*depths)[l.x],(*depths)[l.x+l.y-1]);
            fprintf(f, "%i %i\n", l.x, l.y);
        }
        fclose(f);
    }

    if(sp->saveDepthsToSweepToTxtForVis)
    {
        std::string fn = tmpDir + num2strFourDecimal(rc) + "depths.txt";
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
            std::string fn = tmpDir + num2strFourDecimal(rc) + "depths" + num2str(i) + ".txt";
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
        orientedPoint rcplane;
        rcplane.p = sp->mp->CArr[rc];
        rcplane.n = sp->mp->iRArr[rc] * point3d(0.0, 0.0, 1.0);
        rcplane.n = rcplane.n.normalize();

        std::string fn = tmpDir + num2strFourDecimal(rc) + "rcDepths.txt";
        FILE* f = fopen(fn.c_str(), "w");
        float depth = minDepthAll;
        while(depth < maxDepthAll)
        {
            fprintf(f, "%f\n", depth);
            point3d p = rcplane.p + rcplane.n * depth;
            depth = depth + sp->mp->getCamPixelSize(p, rc);
        }
        fclose(f);
    }

    if(sp->mp->verbose)
        printf("rc depths %i\n", depths->size());

    deleteArrayOfArrays<float>(&alldepths);
}

staticVector<float>* ps_sgm_rc::getSubDepthsForTCam(int tcamid)
{
    staticVector<float>* out = new staticVector<float>((*depthsTcamsLimits)[tcamid].y);

    for(int i = (*depthsTcamsLimits)[tcamid].x; i < (*depthsTcamsLimits)[tcamid].x + (*depthsTcamsLimits)[tcamid].y;
        i++)
    {
        out->push_back((*depths)[i]);
    }

    return out;
}

bool ps_sgm_rc::sgmrc(bool checkIfExists)
{
    if(sp->mp->verbose)
        printf("processing sgmrc %i of %i\n", rc, sp->mp->ncams);

    if(tcams->size() == 0)
    {
        return false;
    }

    if((FileExists(SGM_idDepthMapFileName)) && (checkIfExists))
    {
        return false;
    }

    long tall = clock();

    int volDimX = w;
    int volDimY = h;
    int volDimZ = depths->size();
    float volumeMBinGPUMem = 0.0f;

    staticVector<unsigned char>* simVolume = nullptr;

    staticVectorBool* rcSilhoueteMap = nullptr;
    if(sp->useSilhouetteMaskCodedByColor)
    {
        rcSilhoueteMap = new staticVectorBool(w * h);
        rcSilhoueteMap->resize_with(w * h, true);
        sp->cps->getSilhoueteMap(rcSilhoueteMap, scale, step, sp->silhouetteMaskColor, rc);
    }

    {
        staticVector<float>* subDepths = getSubDepthsForTCam(0);
        ps_sgm_rctc srt(subDepths, rc, (*tcams)[0], scale, step, sp, rcSilhoueteMap);
        simVolume = srt.computeDepthSimMapVolume(volumeMBinGPUMem, wsh, gammaC, gammaP);
        delete subDepths;
    }

    // recompute to all depths
    volumeMBinGPUMem = ((volumeMBinGPUMem / (float)(*depthsTcamsLimits)[0].y) * (float)volDimZ);

    ps_sgm_vol* svol = new ps_sgm_vol(volumeMBinGPUMem, volDimX, volDimY, volDimZ, sp);
    svol->copyVolume(simVolume, (*depthsTcamsLimits)[0].x, (*depthsTcamsLimits)[0].y);
    delete simVolume;

    for(int c = 1; c < tcams->size(); c++)
    {
        staticVector<float>* subDepths = getSubDepthsForTCam(c);
        ps_sgm_rctc* srt = new ps_sgm_rctc(subDepths, rc, (*tcams)[c], scale, step, sp, rcSilhoueteMap);
        simVolume = srt->computeDepthSimMapVolume(volumeMBinGPUMem, wsh, gammaC, gammaP);
        delete srt;
        delete subDepths;
        svol->addVolumeSecondMin(simVolume,(*depthsTcamsLimits)[c].x,(*depthsTcamsLimits)[c].y);
        delete simVolume;
    }

    // Reduction of 'volume' (X, Y, Z) into 'volumeStepZ' (X, Y, Z/step)
    svol->cloneVolumeSecondStepZ();

    // Filter on the 3D volume to weight voxels based on their neighborhood strongness.
    // So it downweights local minimums that are not supported by their neighborhood.
    if(sp->doSGMoptimizeVolume) // this is here for experimental reason ... to show how SGGC work on non
                                // optimized depthmaps ... it must equals to true in normal case
    {
        svol->SGMoptimizeVolumeStepZ(rc, step, 0, 0, scale);
    }

    // For each pixel: choose the voxel with the minimal similarity value
    int zborder = 2;
    staticVector<idValue>* volumeBestIdVal = svol->getOrigVolumeBestIdValFromVolumeStepZ(zborder);
    delete svol;

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
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    saveArrayToFile<float>(depthsFileName, depths);
    saveArrayToFile<int>(tcamsFileName, tcams);
    saveArrayToFile<pixel>(depthsTcamsLimitsFileName, depthsTcamsLimits);

    ps_depthSimMap* depthSimMapFinal =
        sp->getDepthSimMapFromBestIdVal(w, h, volumeBestIdVal, scale, step, rc, zborder, depths);

    // Save to :
    //  - SGM/SGM_{RC}_scaleX_stepN_simMap.bin
    //  - SGM/SGM_{RC}_scaleX_stepN_depthMap.bin
    depthSimMapFinal->saveToBin(SGM_depthMapFileName, SGM_simMapFileName);

    // Save to :
    //  - {RC}_simMap_scaleX.bin
    //  - {RC}_dephMap_scaleX.bin
    //  - {RC}_simMap.bin
    //  - {RC}_dephMap.bin
    depthSimMapFinal->save(rc, tcams);

    staticVector<unsigned short>* volumeBestId = new staticVector<unsigned short>(volumeBestIdVal->size());
    for(int i = 0; i < volumeBestIdVal->size(); i++)
    {
        volumeBestId->push_back(std::max(0, (*volumeBestIdVal)[i].id));
    }
    saveArrayToFile<unsigned short>(SGM_idDepthMapFileName, volumeBestId);

    if(sp->visualizeDepthMaps)
        imageIO::writeImageScaledColors(SGM_idDepthMapFileName + ".png", volDimX, volDimY, 0, depths->size(), &(*volumeBestId)[0], true);

    delete volumeBestId;

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    printfElapsedTime(tall, "PSSGM rc " + num2str(rc) + " of " + num2str(sp->mp->ncams));

    if(sp->visualizeDepthMaps)
    {
        depthSimMapFinal->saveToImage(tmpDir + "ps_sgm_rc_SGM" + num2strFourDecimal(rc) + "_" + "scale" +
                                        num2str(depthSimMapFinal->scale) + "step" + num2str(depthSimMapFinal->step) +
                                        ".wrl.depthSimMap.png",
                                    1.0f);

        depthSimMapFinal->saveToWrl(tmpDir + "SGM" + num2strFourDecimal(rc) + "_" + "scale" +
                                        num2str(depthSimMapFinal->scale) + "step" +
                                        num2str(depthSimMapFinal->step) + ".wrl",
                                    rc);
    }

    delete depthSimMapFinal;
    delete volumeBestIdVal;

    return true;
}

void computeDepthMapsPSSGM(int CUDADeviceNo, multiviewParams* mp, mv_prematch_cams* pc, const staticVector<int>& cams)
{
    int scale = mp->mip->_ini.get<int>("semiGlobalMatching.scale", -1);
    int step = mp->mip->_ini.get<int>("semiGlobalMatching.step", -1);
    if(scale == -1)
    {
        // Compute the number of scales that will be used in the plane sweeping.
        // The highest scale should have a minimum resolution of 700x550.
        int width = mp->mip->getMaxImageWidth();
        int height = mp->mip->getMaxImageHeight();
        int scaleTmp = computeStep(mp->mip, 1, (width > height ? 700 : 550), (width > height ? 550 : 700));
        scale = std::min(2, scaleTmp);
        step = computeStep(mp->mip, scale, (width > height ? 700 : 550), (width > height ? 550 : 700));
        printf("PSSGM autoScaleStep %i %i\n", scale, step);
    }

    int bandType = 0;
    
    // load images from files into RAM 
    mv_images_cache ic(mp, bandType, true);
    // load stuff on GPU memory and creates multi-level images and computes gradients
    cuda_plane_sweeping cps(CUDADeviceNo,& ic, mp, pc, scale);
    // init plane sweeping parameters
    ps_sgm_params sp(mp, pc, &cps);

    //////////////////////////////////////////////////////////////////////////////////////////

    for(const int rc : cams)
    {
        std::string depthMapFilepath = sp.getSGM_idDepthMapFileName(rc, scale, step);
        if(!FileExists(depthMapFilepath))
        {
            std::cout << "Compute depth map: " << depthMapFilepath << std::endl;
            ps_sgm_rc psgr(true, rc, scale, step, &sp);
            psgr.sgmrc();
        }
        else
        {
            std::cout << "Depth map already computed: " << depthMapFilepath << std::endl;
        }
    }
}

void computeDepthMapsPSSGM(multiviewParams* mp, mv_prematch_cams* pc, const staticVector<int>& cams)
{
    int num_gpus = listCUDADevices(true);
    int num_cpu_threads = omp_get_num_procs();
    std::cout << "Number of GPU devices: " << num_gpus << ", number of CPU threads: " << num_cpu_threads << std::endl;
    int numthreads = std::min(num_gpus, num_cpu_threads);

    int num_gpus_to_use = mp->mip->_ini.get<int>("semiGlobalMatching.num_gpus_to_use", 1);
    if(num_gpus_to_use > 0)
    {
        numthreads = num_gpus_to_use;
    }

    if(numthreads == 1)
    {
        computeDepthMapsPSSGM(mp->CUDADeviceNo, mp, pc, cams);
    }
    else
    {
        omp_set_num_threads(numthreads); // create as many CPU threads as there are CUDA devices
#pragma omp parallel
        {
            int cpu_thread_id = omp_get_thread_num();
            int CUDADeviceNo = cpu_thread_id % numthreads;
            std::cout << "CPU thread " << cpu_thread_id << " (of " << numthreads << ") uses CUDA device: " << CUDADeviceNo << std::endl;

            int rcFrom = CUDADeviceNo * (cams.size() / numthreads);
            int rcTo = (CUDADeviceNo + 1) * (cams.size() / numthreads);
            if(CUDADeviceNo == numthreads - 1)
            {
                rcTo = cams.size();
            }
            staticVector<int> subcams(cams.size());
            for(int rc = rcFrom; rc < rcTo; rc++)
            {
                subcams.push_back(cams[rc]);
            }
            computeDepthMapsPSSGM(cpu_thread_id, mp, pc, subcams);
        }
    }
}
