#include "ps_sgm_params.h"
#include "stdafx.h"

#include "structures/mv_filesio.h"


#include <boost/filesystem.hpp>

namespace bfs = boost::filesystem;

ps_sgm_params::ps_sgm_params(multiviewParams* _mp, mv_prematch_cams* _pc, cuda_plane_sweeping* _cps)
{
    mp = _mp;
    pc = _pc;
    cps = _cps;
    o3d = new mv_output3D(mp);
    prt = new ps_rctc(mp, cps);

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

ps_sgm_params::~ps_sgm_params()
{
    delete o3d;
    delete prt;
}

std::string ps_sgm_params::getREFINEOutDir()
{
    return mp->mip->mvDir + "REFINERC/";
}

std::string ps_sgm_params::getREFINETmpDir()
{
    return mp->mip->mvDir + "REFINERC/_tmp/";
}

std::string ps_sgm_params::getREFINE_photo_depthMapFileName(int cam, int scale, int step)
{
    return getREFINEOutDir() + "REFINE_photo_" + num2strFourDecimal(cam) + "_scale" + num2str(scale) + "_step" +
           num2str(step) + "_depthMap.bin";
}

std::string ps_sgm_params::getREFINE_photo_simMapFileName(int cam, int scale, int step)
{
    return getREFINEOutDir() + "REFINE_photo_" + num2strFourDecimal(cam) + "_scale" + num2str(scale) + "_step" +
           num2str(step) + "_simMap.bin";
}

std::string ps_sgm_params::getREFINE_opt_depthMapFileName(int cam, int scale, int step)
{
    return getREFINEOutDir() + "REFINE_opt_" + num2strFourDecimal(cam) + "_scale" + num2str(scale) + "_step" +
           num2str(step) + "_depthMap.bin";
}

std::string ps_sgm_params::getREFINE_opt_simMapFileName(int cam, int scale, int step)
{
    return getREFINEOutDir() + "REFINE_opt_" + num2strFourDecimal(cam) + "_scale" + num2str(scale) + "_step" +
           num2str(step) + "_simMap.bin";
}

std::string ps_sgm_params::getSGMOutDir()
{
    return mp->mip->mvDir + SGMoutDirName + "/";
}

std::string ps_sgm_params::getSGMTmpDir()
{
    return mp->mip->mvDir + SGMoutDirName + "/" + SGMtmpDirName + "/";
}

std::string ps_sgm_params::getSGM_depthMapFileName(int cam, int scale, int step)
{
    return getSGMOutDir() + "SGM_" + num2strFourDecimal(cam) + "_scale" + num2str(scale) + "_step" + num2str(step) +
           "_depthMap.bin";
}

std::string ps_sgm_params::getSGM_simMapFileName(int cam, int scale, int step)
{
    return getSGMOutDir() + "SGM_" + num2strFourDecimal(cam) + "_scale" + num2str(scale) + "_step" + num2str(step) +
           "_simMap.bin";
}

std::string ps_sgm_params::getSGM_idDepthMapFileName(int cam, int scale, int step)
{
    return getSGMOutDir() + "SGM_" + num2strFourDecimal(cam) + "_scale" + num2str(scale) + "_step" + num2str(step) +
           "_idDepthMap.bin";
}

std::string ps_sgm_params::getSGM_tcamsFileName(int cam)
{
    return getSGMOutDir() + num2strFourDecimal(cam) + "tcams.bin";
}

std::string ps_sgm_params::getSGM_depthsFileName(int cam)
{
    return getSGMOutDir() + num2strFourDecimal(cam) + "depths.bin";
}

ps_depthSimMap* ps_sgm_params::getDepthSimMapFromBestIdVal(int w, int h, staticVector<idValue>* volumeBestIdVal,
                                                           int scale, int step, int rc, int zborder,
                                                           staticVector<float>* planesDepths)
{
    long tall = clock();

    int volDimX = w;
    int volDimY = h;

    ps_depthSimMap* depthSimMap = new ps_depthSimMap(rc, mp, scale, step);

#pragma omp parallel for
    for(int y = 0; y < volDimY; y++)
    {
        for(int x = 0; x < volDimX; x++)
        {
            pixel pix = pixel(x * step, y * step);
            pixel pixScale1 = pixel(pix.x * scale, pix.y * scale);
            float sim = (*volumeBestIdVal)[y * volDimX + x].value;
            int fpdepthId = (*volumeBestIdVal)[y * volDimX + x].id;
            if((fpdepthId >= zborder) && (fpdepthId < planesDepths->size() - zborder))
            {
                float fpPlaneDepth = (*planesDepths)[fpdepthId];
                point3d planen = (mp->iRArr[rc] * point3d(0.0f, 0.0f, 1.0f)).normalize();
                point3d planep = mp->CArr[rc] + planen * fpPlaneDepth;
                point3d v = (mp->iCamArr[rc] * point2d((float)(pixScale1.x), (float)(pixScale1.y))).normalize();
                point3d p = linePlaneIntersect(mp->CArr[rc], v, planep, planen);
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

