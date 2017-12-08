// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "ps_depthSimMap.hpp"

#include <aliceVision/common/fileIO.hpp>
#include <aliceVision/structures/mv_geometry.hpp>
#include <aliceVision/structures/jetColorMap.hpp>
#include <aliceVision/imageIO/image.hpp>

#include <iostream>


ps_depthSimMap::ps_depthSimMap(int _rc, multiviewParams* _mp, int _scale, int _step)
{
    rc = _rc;
    mp = _mp;
    o3d = new mv_output3D(mp);
    scale = _scale;
    step = _step;
    w = mp->mip->getWidth(rc) / (scale * step);
    h = mp->mip->getHeight(rc) / (scale * step);
    dsm = new staticVector<DepthSim>(w * h);
    dsm->resize_with(w * h, DepthSim(-1.0f, 1.0f));
}

ps_depthSimMap::~ps_depthSimMap()
{
    delete o3d;
    delete dsm;
}

point3d ps_depthSimMap::get3DPtOfPixel(const pixel& pix, int pixScale, int rc)
{
    pixel pix11 = pixel(pix.x * pixScale, pix.y * pixScale);
    int i = (pix11.y / (scale * step)) * w + pix11.x / (scale * step);
    float depth = (*dsm)[i].depth;
    return mp->CArr[rc] +
           (mp->iCamArr[rc] *
            point2d((float)(pix11.x) + (float)pixScale / 2.0f, (float)pix11.y + (float)pixScale / 2.0f))
                   .normalize() *
               depth;
}

float ps_depthSimMap::getFPDepthOfPixel(const pixel& pix, int pixScale, int rc)
{
    point3d p = get3DPtOfPixel(pix, pixScale, rc);
    point3d zVect = (mp->iRArr[rc] * point3d(0.0f, 0.0f, 1.0f)).normalize();
    return pointPlaneDistance(p, mp->CArr[rc], zVect);
}

void ps_depthSimMap::add11(ps_depthSimMap* depthSimMap)
{
    if((scale != 1) || (step != 1))
    {
        printf("Warning you can add just to cale1 step1 map!\n");
        exit(1);
    }

    int k = (depthSimMap->step * depthSimMap->scale) / 2;
    int k1 = (depthSimMap->step * depthSimMap->scale) / 2;
    if((depthSimMap->step * depthSimMap->scale) % 2 == 0)
    {
        k1 -= 1;
    }

    for(int i = 0; i < depthSimMap->dsm->size(); i++)
    {
        int x = (i % depthSimMap->w) * depthSimMap->step * depthSimMap->scale;
        int y = (i / depthSimMap->w) * depthSimMap->step * depthSimMap->scale;
        DepthSim depthSim = (*depthSimMap->dsm)[i];

        if(depthSim.depth > -1.0f)
        {

            bool isBest = true;
            for(int yp = y - k; yp <= y + k1; yp++)
            {
                for(int xp = x - k; xp <= x + k1; xp++)
                {
                    if((xp >= 0) && (xp < w) && (yp >= 0) && (yp < h) && (depthSim.sim > (*dsm)[yp * w + xp].sim))
                    {
                        isBest = false;
                    }
                }
            }

            if(isBest)
            {
                for(int yp = y - k; yp <= y + k1; yp++)
                {
                    for(int xp = x - k; xp <= x + k1; xp++)
                    {
                        if((xp >= 0) && (xp < w) && (yp >= 0) && (yp < h))
                        {
                            (*dsm)[yp * w + xp] = depthSim;
                        }
                    }
                }
            }
        }
    }
}

void ps_depthSimMap::add(ps_depthSimMap* depthSimMap)
{
    if((scale != depthSimMap->scale) || (step != depthSimMap->step))
    {
        printf("Warning you can add just to the same scale and step map!\n");
        exit(1);
    }

    for(int i = 0; i < dsm->size(); i++)
    {
        DepthSim depthSim1 = (*dsm)[i];
        DepthSim depthSim2 = (*depthSimMap->dsm)[i];

        if((depthSim2.depth > -1.0f) && (depthSim2.sim < depthSim1.sim))
        {
            (*dsm)[i] = depthSim2;
        }
    }
}

void ps_depthSimMap::getReconstructedPixelsDepthsSims(staticVector<pixel>* pixels, staticVector<float>* depths,
                                                      staticVector<float>* sims)
{
    for(int j = 0; j < w * h; j++)
    {
        if((*dsm)[j].depth > 0.0f)
        {
            pixels->push_back(pixel((j % w) * step, (j / w) * step));
            depths->push_back((*dsm)[j].depth);
            sims->push_back((*dsm)[j].sim);
        }
    }
}

point2d ps_depthSimMap::getMaxMinDepth()
{
    float maxDepth = -1.0f;
    float minDepth = std::numeric_limits<float>::max();
    for(int j = 0; j < w * h; j++)
    {
        if((*dsm)[j].depth > -1.0f)
        {
            maxDepth = std::max(maxDepth, (*dsm)[j].depth);
            minDepth = std::min(minDepth, (*dsm)[j].depth);
        }
    }
    return point2d(maxDepth, minDepth);
}

point2d ps_depthSimMap::getMaxMinSim()
{
    float maxSim = -1.0f;
    float minSim = std::numeric_limits<float>::max();
    for(int j = 0; j < w * h; j++)
    {
        if((*dsm)[j].sim > -1.0f)
        {
            maxSim = std::max(maxSim, (*dsm)[j].sim);
            minSim = std::min(minSim, (*dsm)[j].sim);
        }
    }
    return point2d(maxSim, minSim);
}

void ps_depthSimMap::setUsedCellsSimTo(float defaultSim)
{
    for(int j = 0; j < w * h; j++)
    {
        if((*dsm)[j].depth > -1.0f)
        {
            (*dsm)[j].sim = defaultSim;
        }
    }
}

float ps_depthSimMap::getPercentileDepth(float perc)
{

    int step = std::max(1, (w * h) / 50000);
    int n = (w * h) / std::max(1, (step - 1));
    // if (mp->verbose) printf("%i\n",n);
    staticVector<float>* depths = new staticVector<float>(n);

    for(int j = 0; j < w * h; j += step)
    {
        if((*dsm)[j].depth > -1.0f)
        {
            depths->push_back((*dsm)[j].depth);
        }
    }

    qsort(&(*depths)[0], depths->size(), sizeof(float), qSortCompareFloatAsc);

    float out = (*depths)[(float)((float)depths->size() * perc)];

    delete depths;

    return out;
}

/**
 * @brief Get depth map at the size of our input image (with scale applied)
 *        from an internal buffer only computed for a subpart (based on the step).
 */
staticVector<float>* ps_depthSimMap::getDepthMapStep1()
{
	// Size of our input image (with scale applied)
    int wdm = mp->mip->getWidth(rc) / scale;
    int hdm = mp->mip->getHeight(rc) / scale;

	// Create a depth map at the size of our input image
    staticVector<float>* depthMap = new staticVector<float>(wdm * hdm);
    depthMap->resize_with(wdm * hdm, -1.0f);

    for(int i = 0; i < wdm * hdm; i++)
    {
        int x = (i % wdm) / step;
        int y = (i / wdm) / step;
        if((x < w) && (y < h))
        {
			// dsm size: (width, height) / (scale*step)
            float depth = (*dsm)[y * w + x].depth;
			// depthMap size: (width, height) / scale
            (*depthMap)[i] = depth;
        }
    }

    return depthMap;
}

staticVector<float>* ps_depthSimMap::getSimMapStep1()
{
    int wdm = mp->mip->getWidth(rc) / scale;
    int hdm = mp->mip->getHeight(rc) / scale;

    staticVector<float>* simMap = new staticVector<float>(wdm * hdm);
    simMap->resize_with(wdm * hdm, -1.0f);
    for(int i = 0; i < wdm * hdm; i++)
    {
        int x = (i % wdm) / step;
        int y = (i / wdm) / step;
        if((x < w) && (y < h))
        {
            float sim = (*dsm)[y * w + x].sim;
            (*simMap)[i] = sim;
        }
    }

    return simMap;
}

staticVector<float>* ps_depthSimMap::getDepthMapStep1XPart(int xFrom, int partW)
{
    int wdm = mp->mip->getWidth(rc) / scale;
    int hdm = mp->mip->getHeight(rc) / scale;

    staticVector<float>* depthMap = new staticVector<float>(wdm * hdm);
    depthMap->resize_with(wdm * hdm, -1.0f);
    for(int yp = 0; yp < hdm; yp++)
    {
        for(int xp = xFrom; xp < xFrom + partW; xp++)
        {
            int x = xp / step;
            int y = yp / step;
            if((x < w) && (y < h))
            {
                float depth = (*dsm)[y * w + x].depth;
                (*depthMap)[yp * partW + (xp - xFrom)] = depth;
            }
        }
    }

    return depthMap;
}

staticVector<float>* ps_depthSimMap::getSimMapStep1XPart(int xFrom, int partW)
{
    int wdm = mp->mip->getWidth(rc) / scale;
    int hdm = mp->mip->getHeight(rc) / scale;

    staticVector<float>* simMap = new staticVector<float>(wdm * hdm);
    simMap->resize_with(wdm * hdm, -1.0f);
    for(int yp = 0; yp < hdm; yp++)
    {
        for(int xp = xFrom; xp < xFrom + partW; xp++)
        {
            int x = xp / step;
            int y = yp / step;
            if((x < w) && (y < h))
            {
                float sim = (*dsm)[y * w + x].sim;
                (*simMap)[yp * partW + (xp - xFrom)] = sim;
            }
        }
    }

    return simMap;
}

void ps_depthSimMap::initJustFromDepthMapT(staticVector<float>* depthMapT, float defaultSim)
{
    int hdm = mp->mip->getHeight(rc) / scale;

    for(int i = 0; i < dsm->size(); i++)
    {
        int x = (i % w) * step;
        int y = (i / w) * step;
        if((x < w) && (y < h))
        {
            (*dsm)[i].depth = (*depthMapT)[x * hdm + y];
            (*dsm)[i].sim = defaultSim;
        }
    }
}

void ps_depthSimMap::initJustFromDepthMap(staticVector<float>* depthMap, float defaultSim)
{
    int wdm = mp->mip->getWidth(rc) / scale;

    for(int i = 0; i < dsm->size(); i++)
    {
        int x = (i % w) * step;
        int y = (i / w) * step;
        if((x < w) && (y < h))
        {
            (*dsm)[i].depth = (*depthMap)[y * wdm + x];
            (*dsm)[i].sim = defaultSim;
        }
    }
}

void ps_depthSimMap::initFromDepthMapTAndSimMapT(staticVector<float>* depthMapT, staticVector<float>* simMapT,
                                                 int depthSimMapsScale)
{
    int wdm = mp->mip->getWidth(rc) / depthSimMapsScale;
    int hdm = mp->mip->getHeight(rc) / depthSimMapsScale;

    for(int i = 0; i < dsm->size(); i++)
    {
        int x = (((i % w) * step) * scale) / depthSimMapsScale;
        int y = (((i / w) * step) * scale) / depthSimMapsScale;
        if((x < wdm) && (y < hdm))
        {
            (*dsm)[i].depth = (*depthMapT)[x * hdm + y];
            (*dsm)[i].sim = (*simMapT)[x * hdm + y];
        }
    }
}

void ps_depthSimMap::initFromDepthMapAndSimMap(staticVector<float>* depthMap, staticVector<float>* simMap,
                                               int depthSimMapsScale)
{
    int wdm = mp->mip->getWidth(rc) / depthSimMapsScale;
    int hdm = mp->mip->getHeight(rc) / depthSimMapsScale;

    for(int i = 0; i < dsm->size(); i++)
    {
        int x = (((i % w) * step) * scale) / depthSimMapsScale;
        int y = (((i / w) * step) * scale) / depthSimMapsScale;
        if((x < wdm) && (y < hdm))
        {
            (*dsm)[i].depth = (*depthMap)[y * wdm + x];
            (*dsm)[i].sim = (*simMap)[y * wdm + x];
        }
    }
}

staticVector<float>* ps_depthSimMap::getDepthMapTStep1()
{
    int wdm = mp->mip->getWidth(rc) / scale;
    int hdm = mp->mip->getHeight(rc) / scale;

    staticVector<float>* depthMap = new staticVector<float>(wdm * hdm);
    depthMap->resize_with(wdm * hdm, -1.0f);
    for(int i = 0; i < wdm * hdm; i++)
    {
        int x = (i / hdm) / step;
        int y = (i % hdm) / step;
        if((x < w) && (y < h))
        {
            float depth = (*dsm)[y * w + x].depth;
            (*depthMap)[i] = depth;
        }
    }

    return depthMap;
}

staticVector<float>* ps_depthSimMap::getDepthMap()
{
    staticVector<float>* depthMap = new staticVector<float>(dsm->size());
    for(int i = 0; i < dsm->size(); i++)
    {
        depthMap->push_back((*dsm)[i].depth);
    }
    return depthMap;
}

staticVector<float>* ps_depthSimMap::getSimMapTStep1()
{
    int wdm = mp->mip->getWidth(rc) / scale;
    int hdm = mp->mip->getHeight(rc) / scale;

    staticVector<float>* simMap = new staticVector<float>(wdm * hdm);
    simMap->resize_with(wdm * hdm, -1.0f);
    for(int i = 0; i < wdm * hdm; i++)
    {
        int x = (i / hdm) / step;
        int y = (i % hdm) / step;
        if((x < w) && (y < h))
        {
            float sim = (*dsm)[y * w + x].sim;
            (*simMap)[i] = sim;
        }
    }

    return simMap;
}

void ps_depthSimMap::saveToWrlPng(std::string wrlFileName, int rc, float simThr)
{
    saveToImage(wrlFileName + ".depthSimMap.png", simThr);
    saveToWrl(wrlFileName, rc);
}

void ps_depthSimMap::saveToImage(std::string filename, float simThr)
{
    const int bufferWidth = 2 * w;
    std::vector<Color> colorBuffer(bufferWidth * h);

    try 
    {
        point2d maxMinDepth;
        maxMinDepth.x = getPercentileDepth(0.9) * 1.1;
        maxMinDepth.y = getPercentileDepth(0.01) * 0.8;

        point2d maxMinSim = point2d(simThr, -1.0f);
        if(simThr < -1.0f)
        {
            point2d autoMaxMinSim = getMaxMinSim();
            // only use it if the default range is valid
            if (std::abs(autoMaxMinSim.x - autoMaxMinSim.y) > std::numeric_limits<float>::epsilon())
                maxMinSim = autoMaxMinSim;

            if(mp->verbose)
                printf("max %f, min %f\n", maxMinSim.x, maxMinSim.y);
        }

        for(int y = 0; y < h; y++)
        {
            for(int x = 0; x < w; x++)
            {
                const DepthSim& depthSim = (*dsm)[y * w + x];
                float depth = (depthSim.depth - maxMinDepth.y) / (maxMinDepth.x - maxMinDepth.y);
                colorBuffer.at(y * bufferWidth + x) = getColorFromJetColorMap(depth);

                float sim = (depthSim.sim - maxMinSim.y) / (maxMinSim.x - maxMinSim.y);
                colorBuffer.at(y * bufferWidth + w + x) = getColorFromJetColorMap(sim);
            }
        }

        imageIO::writeImage(filename, bufferWidth, h, colorBuffer);
    }
    catch(...)
    {
        std::cout << "Failed to save " << filename << " (simThr :" << simThr << ")" << std::endl;
    }
}

void ps_depthSimMap::saveToWrl(std::string wrlFileName, int rc)
{
    point3d cg = point3d(0.0f, 0.0f, 0.0f);
    float ncg = 0.0f;
    staticVector<point3d>* pts = new staticVector<point3d>(w * h);
    staticVector<voxel>* ptsCls = new staticVector<voxel>(w * h);
    for(int i = 0; i < dsm->size(); i++)
    {
        int x = (i % w) * step;
        int y = (i / w) * step;
        float depth = (*dsm)[i].depth;
        float sim = (*dsm)[i].sim;
        if(depth > 0.0f)
        {
            point3d p =
                mp->CArr[rc] +
                (mp->iCamArr[rc] * point2d((float)x * (float)scale, (float)y * (float)scale)).normalize() * depth;
            pts->push_back(p);
            if(sim < mp->simThr)
            {
                ptsCls->push_back(voxel(0, 0, 0));
                cg = cg + p;
                ncg += 1.0f;
            }
            else
            {
                ptsCls->push_back(voxel(255, 0, 0));
            }
        }
    }
    cg = cg / ncg;

    point2d maxMinDepth = getMaxMinDepth();

    point3d rchexah[8];
    getCamHexahedron(mp, rchexah, rc, maxMinDepth.y, maxMinDepth.x);
    staticVector<int>* rcams = new staticVector<int>(1);
    rcams->push_back(rc);

    FILE* f = fopen(wrlFileName.c_str(), "w");
    fprintf(f, "#VRML V2.0 utf8\n");

    o3d->printf_wrl_pts_cls(f, pts, ptsCls, mp);
    o3d->printfHexahedron(rchexah, f, mp);

    {
        mv_mesh* meCams = createMeshForCameras(rcams, mp, 0.000001f, 0, 1, 0.0f);
        o3d->printfGroupCameras(meCams, rcams, f, mp, 0.000001f, 0.0f);
        delete meCams;

        mv_mesh* mecenter = createMeshForFrontPlanePolygonOfCamera(rc, mp, (float)(h / 4), cg);
        rgb colorOfTris;
        colorOfTris.r = 255;
        colorOfTris.g = 0;
        colorOfTris.b = 0;
        o3d->printfMvMeshToWrl(f, colorOfTris, mecenter);
        delete mecenter;
    }

    fclose(f);

    delete rcams;
    delete ptsCls;
    delete pts;

    staticVector<float>* depthMapT = getDepthMapTStep1();
    for(int stepDetail = 1; stepDetail <= 3; stepDetail++)
    {
        mv_mesh* me = new mv_mesh();
        me->initFromDepthMap(stepDetail, mp, &(*depthMapT)[0], rc, scale, 1,
                             10.0f * (float)stepDetail * (float)step * (float)scale);
        o3d->saveMvMeshToWrl(me, wrlFileName + "mesh" + num2str(stepDetail) + ".wrl");
        delete me;
    }
    delete depthMapT;
}

void ps_depthSimMap::save(int rc, staticVector<int>* tcams)
{
    staticVector<float>* depthMap = getDepthMapStep1();
    staticVector<float>* simMap = getSimMapStep1();

    const int width = mp->mip->getWidth(rc) / scale;
    const int height = mp->mip->getHeight(rc) / scale;

    std::cout << "* write image width : " << width << std::endl;
    std::cout << "* write image height : " << height << std::endl;
    std::cout << "* write image size : " << depthMap->size() << std::endl;

    // TODO: remove "+ 1"
    imageIO::writeImage(mv_getFileName(mp->mip, rc + 1, mp->mip->MV_FILE_TYPE_depthMap, scale), width, height, depthMap->getDataWritable());
    imageIO::writeImage(mv_getFileName(mp->mip, rc + 1, mp->mip->MV_FILE_TYPE_simMap, scale), width, height, simMap->getDataWritable());

//    if(scale == 1) // TODO: check if necessary
//    {
//        imageIO::writeImage(mv_getFileName(mp->mip, rc + 1, mp->mip->MV_FILE_TYPE_depthMap, 0), width, height, depthMap->getDataWritable());
//        imageIO::writeImage(mv_getFileName(mp->mip, rc + 1, mp->mip->MV_FILE_TYPE_simMap, 0), width, height, simMap->getDataWritable());
//    }

    {
        point2d maxMinDepth = getMaxMinDepth();
        FILE* f = mv_openFile(mp->mip, mp->indexes[rc], mp->mip->MV_FILE_TYPE_depthMapInfo, "w");
        int nbTCs = tcams ? tcams->size() : 0;
        fprintf(f, "minDepth %f, maxDepth %f, ntcams %i, tcams", maxMinDepth.y, maxMinDepth.x, nbTCs);
        for(int c = 0; c < nbTCs; c++)
        {
            int tc = (*tcams)[c];
            fprintf(f, " %i", tc);
        }
        fclose(f);
    }
}

void ps_depthSimMap::load(int rc, int fromScale)
{
    int width, height;

    staticVector<float> depthMap;
    staticVector<float> simMap;

    imageIO::readImage(mv_getFileName(mp->mip, rc + 1, mp->mip->MV_FILE_TYPE_depthMap, fromScale), width, height, depthMap.getDataWritable());
    imageIO::readImage(mv_getFileName(mp->mip, rc + 1, mp->mip->MV_FILE_TYPE_simMap, fromScale), width, height, simMap.getDataWritable());

    imageIO::transposeImage(width, height, depthMap.getDataWritable());
    imageIO::transposeImage(width, height, simMap.getDataWritable());

    initFromDepthMapTAndSimMapT(&depthMap, &simMap, fromScale);
}

void ps_depthSimMap::saveToBin(std::string depthMapFileName, std::string simMapFileName)
{
    staticVector<float>* _depthMap = new staticVector<float>(dsm->size());
    staticVector<float>* _simMap = new staticVector<float>(dsm->size());
    _depthMap->resize(dsm->size());
    _simMap->resize(dsm->size());

    for(int i = 0; i < dsm->size(); i++)
    {
        (*_depthMap)[i] = (*dsm)[i].depth;
        (*_simMap)[i] = (*dsm)[i].sim;
    }
    saveArrayToFile<float>(depthMapFileName, _depthMap);
    saveArrayToFile<float>(simMapFileName, _simMap);
    delete _depthMap;
    delete _simMap;
}

bool ps_depthSimMap::loadFromBin(std::string depthMapFileName, std::string simMapFileName)
{
    staticVector<float>* _depthMap = loadArrayFromFile<float>(depthMapFileName, true);
    staticVector<float>* _simMap = loadArrayFromFile<float>(simMapFileName, true);
    bool ok = false;

    if((_depthMap != nullptr) && (_simMap != nullptr))
    {
        for(int i = 0; i < dsm->size(); i++)
        {
            (*dsm)[i].depth = (*_depthMap)[i];
            (*dsm)[i].sim = (*_simMap)[i];
        }
        ok = true;
    }
    if(_depthMap != nullptr)
        delete _depthMap;
    if(_simMap != nullptr)
        delete _simMap;

    return ok;
}

bool ps_depthSimMap::loadFromBin(std::string depthMapFileName, float defaultSim)
{
    staticVector<float>* _depthMap = loadArrayFromFile<float>(depthMapFileName, true);
    bool ok = false;

    if(_depthMap != nullptr)
    {
        for(int i = 0; i < dsm->size(); i++)
        {
            (*dsm)[i].depth = (*_depthMap)[i];
            (*dsm)[i].sim = defaultSim;
        }
        ok = true;
    }
    if(_depthMap != nullptr)
        delete _depthMap;

    return ok;
}

mv_universe* ps_depthSimMap::segment(float alpha, int rc)
{
    printf("segmenting to connected components \n");

    staticVector<pixel>* edges = new staticVector<pixel>(w * h * 2);
    for(int y = 0; y < h - 1; y++)
    {
        for(int x = 0; x < w - 1; x++)
        {
            float depth = (*dsm)[y * w + x].depth;
            float depthr = (*dsm)[y * w + (x + 1)].depth;
            float depthd = (*dsm)[(y + 1) * w + x].depth;

            if(depth > 0.0f)
            {
                point3d p =
                    mp->CArr[rc] +
                    (mp->iCamArr[rc] * point2d((float)x * (scale * step), (float)y * (scale * step))).normalize() *
                        depth;
                float pixSize = alpha * mp->getCamPixelSize(p, rc);

                if((depthr > 0.0f) && (fabs(depth - depthr) < pixSize))
                {
                    edges->push_back(pixel(y * w + x, y * w + (x + 1)));
                }
                if((depthd > 0.0f) && (fabs(depth - depthd) < pixSize))
                {
                    edges->push_back(pixel(y * w + x, (y + 1) * w + x));
                }
            }
        }
    }

    // segments
    mv_universe* u = new mv_universe(w * h);
    for(int i = 0; i < edges->size(); i++)
    {
        int a = u->find((*edges)[i].x);
        int b = u->find((*edges)[i].y);
        if(a != b)
        {
            u->join(a, b);
        }
    }

    delete edges;

    return u;
}

void ps_depthSimMap::removeSmallSegments(int minSegSize, float alpha, int rc)
{
    long tall = clock();

    mv_universe* u = ps_depthSimMap::segment(alpha, rc);

    for(int i = 0; i < w * h; i++)
    {
        float depth = (*dsm)[i].depth;
        if(depth > 0.0f)
        {
            int sid = u->find(i);
            if(u->elts[sid].size < minSegSize)
            {
                (*dsm)[i] = DepthSim(-1.0f, 1.0f);
            }
        }
    }

    if(mp->verbose)
        printfElapsedTime(tall, "removeSmallSegments");
}

void ps_depthSimMap::cutout(const pixel& LU, const pixel& RD)
{
    int wdm = mp->mip->getWidth(rc) / scale;
    int hdm = mp->mip->getHeight(rc) / scale;

    for(int i = 0; i < wdm * hdm; i++)
    {
        int x = (i % wdm) / step;
        int y = (i / wdm) / step;
        if((LU.x > x) || (LU.y > y) || (RD.x < x) || (RD.y < y))
        {
            (*dsm)[y * w + x].depth = -1.0f;
            (*dsm)[y * w + x].sim = 1.0f;
        }
    }
}

float ps_depthSimMap::getAngleBetwABandACdepth(int rc, const pixel& cellA, float dA, const pixel& cellB, float dB,
                                               const pixel& cellC, float dC)
{
    point3d pA =
        mp->CArr[rc] +
        (mp->iCamArr[rc] * point2d((float)cellA.x * (scale * step), (float)cellA.y * (scale * step))).normalize() * dA;
    point3d pB =
        mp->CArr[rc] +
        (mp->iCamArr[rc] * point2d((float)cellB.x * (scale * step), (float)cellB.y * (scale * step))).normalize() * dB;
    point3d pC =
        mp->CArr[rc] +
        (mp->iCamArr[rc] * point2d((float)cellC.x * (scale * step), (float)cellC.y * (scale * step))).normalize() * dC;
    return 180.0f - angleBetwABandAC(pA, pB, pC);
}

float ps_depthSimMap::getCellSmoothEnergy(int rc, const int cellId, float defaultE)
{
    return getCellSmoothEnergy(rc, pixel(cellId % w, cellId / w), defaultE);
}

float ps_depthSimMap::getCellSmoothEnergy(int rc, const pixel& cell, float defaultE)
{
    if((cell.x <= 0) || (cell.x >= w - 1) || (cell.y <= 0) || (cell.y >= h - 1))
    {
        return defaultE;
    }

    pixel cell0 = cell;
    pixel cellL = cell0 + pixel(0, -1);
    pixel cellR = cell0 + pixel(0, 1);
    pixel cellU = cell0 + pixel(-1, 0);
    pixel cellB = cell0 + pixel(1, 0);

    float d0 = (*dsm)[cell0.y * w + cell0.x].depth;
    float dL = (*dsm)[cellL.y * w + cellL.x].depth;
    float dR = (*dsm)[cellR.y * w + cellR.x].depth;
    float dU = (*dsm)[cellU.y * w + cellU.x].depth;
    float dB = (*dsm)[cellB.y * w + cellB.x].depth;

    float e = 0.0f;
    bool ok = false;

    if((d0 > 0.0f) && (dL > 0.0f) && (dR > 0.0f))
    {
        e = std::max(e, getAngleBetwABandACdepth(rc, cell0, d0, cellL, dL, cellR, dR));
        ok = true;
    }

    if((d0 > 0.0f) && (dU > 0.0f) && (dB > 0.0f))
    {
        e = std::max(e, getAngleBetwABandACdepth(rc, cell0, d0, cellU, dU, cellB, dB));
        ok = true;
    }

    return (ok) ? e : defaultE;
}

float ps_depthSimMap::getASmoothStepBetwABandACdepth(int rc, const pixel& cellA, float dA, const pixel& cellB, float dB,
                                                     const pixel& cellC, float dC)
{
    point3d pA =
        mp->CArr[rc] +
        (mp->iCamArr[rc] * point2d((float)cellA.x * (scale * step), (float)cellA.y * (scale * step))).normalize() * dA;
    point3d pB =
        mp->CArr[rc] +
        (mp->iCamArr[rc] * point2d((float)cellB.x * (scale * step), (float)cellB.y * (scale * step))).normalize() * dB;
    point3d pC =
        mp->CArr[rc] +
        (mp->iCamArr[rc] * point2d((float)cellC.x * (scale * step), (float)cellC.y * (scale * step))).normalize() * dC;

    point3d vs = ((pB - pA) + (pC - pA)) / 4.0f;
    point3d vcn = (mp->CArr[rc] - pA).normalize();

    point3d A1 = pA + vs;
    point3d pS = closestPointToLine3D(&A1, &pA, &vcn);

    return (mp->CArr[rc] - pS).size() - dA;
}

float ps_depthSimMap::getCellSmoothStep(int rc, const int cellId)
{
    return getCellSmoothStep(rc, pixel(cellId % w, cellId / w));
}

float ps_depthSimMap::getCellSmoothStep(int rc, const pixel& cell)
{
    if((cell.x <= 0) || (cell.x >= w - 1) || (cell.y <= 0) || (cell.y >= h - 1))
    {
        return 0.0f;
    }

    pixel cell0 = cell;
    pixel cellL = cell0 + pixel(0, -1);
    pixel cellR = cell0 + pixel(0, 1);
    pixel cellU = cell0 + pixel(-1, 0);
    pixel cellB = cell0 + pixel(1, 0);

    float d0 = (*dsm)[cell0.y * w + cell0.x].depth;
    float dL = (*dsm)[cellL.y * w + cellL.x].depth;
    float dR = (*dsm)[cellR.y * w + cellR.x].depth;
    float dU = (*dsm)[cellU.y * w + cellU.x].depth;
    float dB = (*dsm)[cellB.y * w + cellB.x].depth;

    point3d cg = point3d(0.0f, 0.0f, 0.0f);
    float n = 0.0f;

    if(dL > 0.0f)
    {
        cg =
            cg +
            (mp->CArr[rc] +
             (mp->iCamArr[rc] * point2d((float)cellL.x * (scale * step), (float)cellL.y * (scale * step))).normalize() *
                 dL);
        n += 1.0f;
    }
    if(dR > 0.0f)
    {
        cg =
            cg +
            (mp->CArr[rc] +
             (mp->iCamArr[rc] * point2d((float)cellR.x * (scale * step), (float)cellR.y * (scale * step))).normalize() *
                 dR);
        n += 1.0f;
    }
    if(dU > 0.0f)
    {
        cg =
            cg +
            (mp->CArr[rc] +
             (mp->iCamArr[rc] * point2d((float)cellU.x * (scale * step), (float)cellU.y * (scale * step))).normalize() *
                 dU);
        n += 1.0f;
    }
    if(dB > 0.0f)
    {
        cg =
            cg +
            (mp->CArr[rc] +
             (mp->iCamArr[rc] * point2d((float)cellB.x * (scale * step), (float)cellB.y * (scale * step))).normalize() *
                 dB);
        n += 1.0f;
    }

    if((d0 > 0.0f) && (n > 1.0f))
    {
        cg = cg / n;
        point3d p0 =
            mp->CArr[rc] +
            (mp->iCamArr[rc] * point2d((float)cell0.x * (scale * step), (float)cell0.y * (scale * step))).normalize() *
                d0;
        point3d vcn = (mp->CArr[rc] - p0).normalize();

        point3d pS = closestPointToLine3D(&cg, &p0, &vcn);

        return (mp->CArr[rc] - pS).size() - d0;
    }

    return 0.0f;
}
