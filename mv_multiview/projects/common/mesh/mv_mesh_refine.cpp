#include "mv_mesh_refine.h"
#include "stdafx.h"



#include <boost/filesystem.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>

namespace bfs = boost::filesystem;

mv_mesh_refine::mv_mesh_refine(multiviewParams* _mp, mv_prematch_cams* _pc, std::string _tmpDir)
    : mv_mesh()
{
    mp = _mp;
    pc = _pc;

    if(_tmpDir == "")
    {
        tmpDir = mp->mip->mvDir + "refine/";
    }
    else
    {
        tmpDir = _tmpDir;
    }
    bfs::create_directory(tmpDir);

    tmpDirOld = mp->mip->mvDir + "refine/old/";
    bfs::create_directory(tmpDirOld);

    meshDepthMapsDir = tmpDir + "meshDepthMapsDir/";
    bfs::create_directory(meshDepthMapsDir);

    o3d = new mv_output3D(mp);

    int bandType = 0;
    ic = new mv_images_cache(mp, bandType, true);
    cps = new cuda_plane_sweeping(mp->CUDADeviceNo, ic, mp, pc, 1);
    prt = new ps_rctc(mp, cps);
}

mv_mesh_refine::~mv_mesh_refine()
{
    delete o3d;
    delete ic;
    delete cps;
    delete prt;
}

void mv_mesh_refine::smoothDepthMapAdaptiveByImage(ps_rctc* prt, int rc, staticVector<float>* tmpDepthMap)
{
    ps_depthSimMap* depthSimMap = new ps_depthSimMap(rc, mp, 1, 1);
    depthSimMap->initJustFromDepthMapT(tmpDepthMap, -1.0f);

    const bool doSmooth = mp->mip->_ini.get<bool>("grow2.smooth", true);
    const int s_wsh = mp->mip->_ini.get<int>("grow2.smoothWsh", 4);
    const float s_gammaC = (float)mp->mip->_ini.get<double>("grow2.smoothGammaC", 15.5);
    const float s_gammaP = (float)mp->mip->_ini.get<double>("grow2.smoothGammaP", 8.0);
    if(doSmooth)
        prt->smoothDepthMap(depthSimMap, rc, s_wsh, s_gammaC, s_gammaP);

    staticVector<float>* depthMaps = depthSimMap->getDepthMapTStep1();
    for(int i = 0; i < depthMaps->size(); i++)
    {
        (*tmpDepthMap)[i] = (*depthMaps)[i];
    }

    delete depthMaps;
    delete depthSimMap;
}

void mv_mesh_refine::smoothDepthMapsAdaptiveByImages(staticVector<int>* usedCams)
{
    for(int c = 0; c < usedCams->size(); c++)
    {
        int rc = (*usedCams)[c];
        std::string fileName1 = meshDepthMapsDir + "depthMap" + num2strFourDecimal(rc) + ".bin";
        staticVector<float>* tmpDepthMap1 = loadArrayFromFile<float>(fileName1);
        smoothDepthMapAdaptiveByImage(prt, rc, tmpDepthMap1);
        saveArrayToFile<float>(fileName1, tmpDepthMap1);
        delete tmpDepthMap1;
    }
}

void mv_mesh_refine::transposeDepthMap(staticVector<float>* depthMapTransposed, staticVector<float>* depthMap, int w,
                                       int h)
{
    depthMapTransposed->resize_with(h * w, -1.0f);
    for(int y = 0; y < h; y++)
    {
        for(int x = 0; x < w; x++)
        {
            (*depthMapTransposed)[x * h + y] = (*depthMap)[y * w + x];
        }
    }
}

void mv_mesh_refine::alignSourceDepthMapToTarget(staticVector<float>* sourceDepthMapT,
                                                 staticVector<float>* targetDepthMapT, int rc, float maxPixelSizeDist)
{
    long t1 = clock();

    int w = mp->mip->getWidth(rc);
    int h = mp->mip->getHeight(rc);

    staticVector<float>* sourceDepthMap = new staticVector<float>(w * h);
    transposeDepthMap(sourceDepthMap, sourceDepthMapT, h, w);

    staticVector<float>* targetDepthMap = new staticVector<float>(w * h);
    transposeDepthMap(targetDepthMap, targetDepthMapT, h, w);

    int wsh = 16;
    float gammaC = 10.0f;

    cps->alignSourceDepthMapToTarget(sourceDepthMap, targetDepthMap, rc, 1, gammaC, wsh, maxPixelSizeDist);

    transposeDepthMap(sourceDepthMapT, sourceDepthMap, w, h);
    delete sourceDepthMap;

    transposeDepthMap(targetDepthMapT, targetDepthMap, w, h);
    delete targetDepthMap;

    if(mp->verbose)
        printfElapsedTime(t1, "alignSourceDepthMapToTarget");
}
