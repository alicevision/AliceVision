// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "SemiGlobalMatchingParams.hpp"
#include <aliceVision/structures/Pixel.hpp>
#include <aliceVision/structures/Point2d.hpp>
#include <aliceVision/structures/Point3d.hpp>
#include <aliceVision/common/common.hpp>
#include <aliceVision/common/fileIO.hpp>

#include <boost/filesystem.hpp>

namespace bfs = boost::filesystem;

SemiGlobalMatchingParams::SemiGlobalMatchingParams(multiviewParams* _mp, mv_prematch_cams* _pc, PlaneSweepingCuda* _cps)
{
    mp = _mp;
    pc = _pc;
    cps = _cps;
    prt = new RcTc(mp, cps);

    visualizeDepthMaps = mp->mip->_ini.get<bool>("semiGlobalMatching.visualizeDepthMaps", false);
    visualizePartialDepthMaps =
        mp->mip->_ini.get<bool>("semiGlobalMatching.visualizePartialDepthMaps", false);

    doSmooth = mp->mip->_ini.get<bool>("semiGlobalMatching.smooth", true);


    doRefine = mp->mip->_ini.get<bool>("semiGlobalMatching.doRefine", true);
    refineUseTcOrPixSize = mp->mip->_ini.get<bool>("semiGlobalMatching.refineUseTcOrPixSize", true);

    ndepthsToRefine = mp->mip->_ini.get<int>("semiGlobalMatching.ndepthsToRefine", 15);

    P1 = (unsigned char)mp->mip->_ini.get<int>("semiGlobalMatching.P1", 10);
    P2 = (unsigned char)mp->mip->_ini.get<int>("semiGlobalMatching.P2", 125);
    P3 = (unsigned char)mp->mip->_ini.get<int>("semiGlobalMatching.P3", 0);

    maxDepthsToStore = mp->mip->_ini.get<int>("semiGlobalMatching.maxDepthsToStore", 3000);
    maxDepthsToSweep = mp->mip->_ini.get<int>("semiGlobalMatching.maxDepthsToSweep", 1500);
    rcTcDepthsHalfLimit = mp->mip->_ini.get<int>("semiGlobalMatching.rcTcDepthsHalfLimit", 2048);

    rcDepthsCompStep = mp->mip->_ini.get<int>("semiGlobalMatching.rcDepthsCompStep", 6);

    useSeedsToCompDepthsToSweep =
        mp->mip->_ini.get<bool>("semiGlobalMatching.useSeedsToCompDepthsToSweep", true);
    seedsRangePercentile = (float)mp->mip->_ini.get<double>("semiGlobalMatching.seedsRangePercentile", 0.001);
    seedsRangeInflate = (float)mp->mip->_ini.get<double>("semiGlobalMatching.seedsRangeInflate", 0.2);

    saveDepthsToSweepToTxtForVis =
        mp->mip->_ini.get<bool>("semiGlobalMatching.saveDepthsToSweepToTxtForVis", false);

    doSGMoptimizeVolume = mp->mip->_ini.get<bool>("semiGlobalMatching.doSGMoptimizeVolume", true);
    doRefineRc = mp->mip->_ini.get<bool>("semiGlobalMatching.doRefineRc", true);

    modalsMapDistLimit = mp->mip->_ini.get<int>("semiGlobalMatching.modalsMapDistLimit", 2);
    minNumOfConsistentCams = mp->mip->_ini.get<int>("semiGlobalMatching.minNumOfConsistentCams", 2);
    minObjectThickness = mp->mip->_ini.get<int>("semiGlobalMatching.minObjectThickness", 8);
    maxTcRcPixSizeInVoxRatio =
        (float)mp->mip->_ini.get<double>("semiGlobalMatching.maxTcRcPixSizeInVoxRatio", 2.0f);
    nSGGCIters = mp->mip->_ini.get<int>("semiGlobalMatching.nSGGCIters", 0);

    SGMoutDirName = mp->mip->_ini.get<std::string>("semiGlobalMatching.outDirName", "SGM");
    SGMtmpDirName = mp->mip->_ini.get<std::string>("semiGlobalMatching.tmpDirName", "_tmp");

    useSilhouetteMaskCodedByColor = mp->mip->_ini.get<bool>("global.useSilhouetteMaskCodedByColor", false);
    silhouetteMaskColor.r = mp->mip->_ini.get<int>("global.silhouetteMaskColorR", 0);
    silhouetteMaskColor.g = mp->mip->_ini.get<int>("global.silhouetteMaskColorG", 0);
    silhouetteMaskColor.b = mp->mip->_ini.get<int>("global.silhouetteMaskColorB", 0);
}

SemiGlobalMatchingParams::~SemiGlobalMatchingParams()
{
    delete prt;
}

std::string SemiGlobalMatchingParams::getREFINE_photo_depthMapFileName(int cam, int scale, int step)
{
    return mp->mip->_depthMapFolder + num2strFourDecimal(cam + 1) + "_depthMap_scale" + num2str(scale) + "_step" + num2str(step) + "_refinePhoto.exr";
}

std::string SemiGlobalMatchingParams::getREFINE_photo_simMapFileName(int cam, int scale, int step)
{
    return mp->mip->_depthMapFolder + num2strFourDecimal(cam + 1) + "_simMap_scale" + num2str(scale) + "_step" + num2str(step) + "_refinePhoto.exr";
}

std::string SemiGlobalMatchingParams::getREFINE_opt_depthMapFileName(int cam, int scale, int step)
{
    return mp->mip->_depthMapFolder + num2strFourDecimal(cam + 1) + "_depthMap_scale" + num2str(scale) + "_step" + num2str(step) + "_refineOpt.exr";
}

std::string SemiGlobalMatchingParams::getREFINE_opt_simMapFileName(int cam, int scale, int step)
{
    return mp->mip->_depthMapFolder + num2strFourDecimal(cam + 1) + "_simMap_scale" + num2str(scale) + "_step" + num2str(step) + "_refineOpt.exr";
}

std::string SemiGlobalMatchingParams::getSGMTmpDir()
{
    return mp->mip->_depthMapFolder + SGMoutDirName + "/" + SGMtmpDirName + "/";
}

std::string SemiGlobalMatchingParams::getSGM_depthMapFileName(int cam, int scale, int step)
{
    return mp->mip->_depthMapFolder + num2strFourDecimal(cam + 1) + "_depthMap_scale" + num2str(scale) + "_step" + num2str(step) + "_SGM.bin";
}

std::string SemiGlobalMatchingParams::getSGM_simMapFileName(int cam, int scale, int step)
{
    return mp->mip->_depthMapFolder + num2strFourDecimal(cam + 1) + "_simMap_scale" + num2str(scale) + "_step" + num2str(step) + "_SGM.bin";
}

std::string SemiGlobalMatchingParams::getSGM_idDepthMapFileName(int cam, int scale, int step)
{
    return mp->mip->_depthMapFolder + num2strFourDecimal(cam + 1) + "_idDepthMap_scale" + num2str(scale) + "_step" + num2str(step) + "_SGM.png";
}

std::string SemiGlobalMatchingParams::getSGM_tcamsFileName(int cam)
{
    return mp->mip->_depthMapFolder + num2strFourDecimal(cam + 1) + "_tcams.bin";
}

std::string SemiGlobalMatchingParams::getSGM_depthsFileName(int cam)
{
    return mp->mip->_depthMapFolder + num2strFourDecimal(cam + 1) + "_depths.bin";
}

DepthSimMap* SemiGlobalMatchingParams::getDepthSimMapFromBestIdVal(int w, int h, StaticVector<IdValue>* volumeBestIdVal,
                                                           int scale, int step, int rc, int zborder,
                                                           StaticVector<float>* planesDepths)
{
    long tall = clock();

    int volDimX = w;
    int volDimY = h;

    DepthSimMap* depthSimMap = new DepthSimMap(rc, mp, scale, step);

#pragma omp parallel for
    for(int y = 0; y < volDimY; y++)
    {
        for(int x = 0; x < volDimX; x++)
        {
            Pixel pix = Pixel(x * step, y * step);
            Pixel pixScale1 = Pixel(pix.x * scale, pix.y * scale);
            float sim = (*volumeBestIdVal)[y * volDimX + x].value;
            int fpdepthId = (*volumeBestIdVal)[y * volDimX + x].id;
            if((fpdepthId >= zborder) && (fpdepthId < planesDepths->size() - zborder))
            {
                float fpPlaneDepth = (*planesDepths)[fpdepthId];
                Point3d planen = (mp->iRArr[rc] * Point3d(0.0f, 0.0f, 1.0f)).normalize();
                Point3d planep = mp->CArr[rc] + planen * fpPlaneDepth;
                Point3d v = (mp->iCamArr[rc] * Point2d((float)(pixScale1.x), (float)(pixScale1.y))).normalize();
                Point3d p = linePlaneIntersect(mp->CArr[rc], v, planep, planen);
                float depth = (mp->CArr[rc] - p).size();

                // printf("fpdepthId %i, fpPlaneDepth %f, depth %f, x %i y
                // %i\n",fpdepthId,fpPlaneDepth,depth,pixScale1.x,pixScale1.y);

                //(*depthSimMap->dsm)[(pix.y/step)*(depthSimMap->w)+(pix.x/step)].x = depth;
                //(*depthSimMap->dsm)[(pix.y/step)*(depthSimMap->w)+(pix.x/step)].y = sim;
                (*depthSimMap->dsm)[y * volDimX + x].depth = depth;
                (*depthSimMap->dsm)[y * volDimX + x].sim = sim;
            }
            else
            {
                // border cases
                // printf("WARNING fpdepthId == %i\n",fpdepthId);
                // exit(1);
            }
        }
    }

    if(mp->verbose)
        printfElapsedTime(tall, "getDepthSimMapFromBestIdVal");

    return depthSimMap;
}

