// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "mv_fuse.hpp"

#include <aliceVision/structures/mv_filesio.hpp>

#include <boost/filesystem.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>

#include <iostream>


namespace bfs = boost::filesystem;

mv_fuse::mv_fuse(const multiviewParams* _mp, mv_prematch_cams* _pc)
  : mp(_mp)
  , pc(_pc)
{
}

mv_fuse::~mv_fuse()
{
}

void mv_fuse::visualizeDepthMap(int rc, std::string wrlFileName, staticVector<float>* depthMap,
                                staticVector<float>* simMap, int scale, int step)
{
    float mindepth = std::numeric_limits<float>::max();
    float maxdepth = 0.0f;

    point3d cg = point3d(0.0f, 0.0f, 0.0f);
    float ncg = 0.0f;
    int w = mp->mip->getWidth(rc) / (scale * step);
    int h = mp->mip->getHeight(rc) / (scale * step);
    staticVector<point3d>* pts = new staticVector<point3d>(w * h);
    staticVector<voxel>* ptsCls = new staticVector<voxel>(w * h);
    mp->CArr[rc].doprintf();
    mp->iCamArr[rc].doprintf();

    for(int i = 0; i < depthMap->size(); i++)
    {
        int x = i / h;
        int y = i % h;
        float depth = (*depthMap)[i];
        float sim = simMap != nullptr ? (*simMap)[i] : -1.0;
        if(depth > 0.0f) // && (x % step == 0) && (y % step == 0))
        {
            point3d p =
                mp->CArr[rc] +
                (mp->iCamArr[rc] * point2d((float)x * (float)scale * (float)step, (float)y * (float)scale * (float)step)).normalize() * depth;
            // if (i%3==0) {
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
            //};
            mindepth = std::min(mindepth, depth);
            maxdepth = std::max(maxdepth, depth);
        }
    }
    cg = cg / ncg;

    imagesc(wrlFileName + "depth.png", &(*depthMap)[0], w, h, mindepth, maxdepth);
    if(simMap)
        imagesc(wrlFileName + "sim.png", &(*simMap)[0], w, h, -1.0f, 1.0f);

    point3d rchexah[8];
    getCamHexahedron(mp, rchexah, rc, mindepth, maxdepth);
    staticVector<int>* rcams = new staticVector<int>(1);
    rcams->push_back(rc);

    FILE* f = fopen(wrlFileName.c_str(), "w");
    fprintf(f, "#VRML V2.0 utf8\n");
    mv_output3D* o3d = new mv_output3D(mp);
    o3d->printf_wrl_pts_cls(f, pts, ptsCls, mp);
    o3d->printfHexahedron(rchexah, f, mp);
    mv_mesh* meCams = createMeshForCameras(rcams, mp, 0.000001f, 0, 1, 0.0f);
    o3d->printfGroupCameras(meCams, rcams, f, mp, 0.000001f, 0.0f);
    delete meCams;

    {
        mv_mesh* mecenter = createMeshForFrontPlanePolygonOfCamera(rc, mp, (float)(h / 4), cg);
        rgb colorOfTris;
        colorOfTris.r = 255;
        colorOfTris.g = 0;
        colorOfTris.b = 0;
        o3d->printfMvMeshToWrl(f, colorOfTris, mecenter);
        delete mecenter;
    }

    delete o3d;
    fclose(f);

    delete rcams;

    for(int stepDetail = 1; stepDetail <= 3; stepDetail++)
    {
        mv_mesh* me = new mv_mesh();
        me->initFromDepthMap(stepDetail, mp, &(*depthMap)[0], rc, scale, step,
                             10.0f * (float)stepDetail * (float)step * (float)scale);
        o3d = new mv_output3D(mp);
        o3d->saveMvMeshToWrl(me, wrlFileName + "mesh" + num2str(stepDetail) + ".wrl");
        delete o3d;
    }

    delete ptsCls;
    delete pts;
}

void mv_fuse::visualizeDepthMap(int rc, std::string wrlFileName, int scale, int step)
{
    staticVector<float>* depthMap =
        loadArrayFromFile<float>(mv_getFileName(mp->mip, rc + 1, mp->mip->MV_FILE_TYPE_depthMap, scale));
    staticVector<float>* simMap =
        loadArrayFromFile<float>(mv_getFileName(mp->mip, rc + 1, mp->mip->MV_FILE_TYPE_simMap, scale));
    visualizeDepthMap(rc, wrlFileName, depthMap, simMap, std::max(1, scale), step);
    delete depthMap;
    delete simMap;
}

unsigned long mv_fuse::computeNumberOfAllPoints(int scale)
{
    unsigned long npts = 0;

#pragma omp parallel for reduction(+:npts)
    for(int rc = 0; rc < mp->ncams; rc++)
    {
        staticVector<float>* depthMap =
            loadArrayFromFile<float>(mv_getFileName(mp->mip, rc + 1, mp->mip->MV_FILE_TYPE_depthMap, scale));
        unsigned long rc_npts = 0;
        for(int i = 0; i < sizeOfStaticVector<float>(depthMap); i++)
        {
            rc_npts += (unsigned long)((*depthMap)[i] > 0.0f);
        }

        npts += rc_npts;

        if(depthMap != nullptr)
        {
            delete depthMap;
        }
    } // for i

    return npts;
}


/**
 * @brief 
 * 
 * @param[in]
 * @param[in]
 * @param[in] p: 3d point back projected from tc camera
 * @param[in]
 * @param[in]
 * @param[out] numOfPtsMap
 * @param[in] depthMap
 * @param[in] simMap
 * @param[in] scale
 */
bool mv_fuse::updateInSurr(int pixSizeBall, int pixSizeBallWSP, point3d& p, int rc, int tc,
                           staticVector<int>* numOfPtsMap, staticVector<float>* depthMap, staticVector<float>* simMap,
                           int scale)
{
    int w = mp->mip->getWidth(rc) / scale;
    int h = mp->mip->getHeight(rc) / scale;

    pixel pix;
    mp->getPixelFor3DPoint(&pix, p, rc);
    if(!mp->isPixelInImage(pix, rc))
    {
        return false;
    }

    pixel cell = pix;
    cell.x /= scale;
    cell.y /= scale;

    float pixDepth = (mp->CArr[rc] - p).size();

    int d = pixSizeBall;

    float sim = (*simMap)[cell.x * h + cell.y];
    if(sim >= 1.0f)
    {
        d = pixSizeBallWSP;
    }

    // float pixSize = 2.0f*(float)std::max(d,1)*mp->getCamPixelSize(p,cam);
    float pixSize = 2.0f * mp->getCamPixelSizePlaneSweepAlpha(p, rc, tc, scale, 1);

    pixel ncell;
    for(ncell.x = std::max(0, cell.x - d); ncell.x <= std::min(w - 1, cell.x + d); ncell.x++)
    {
        for(ncell.y = std::max(0, cell.y - d); ncell.y <= std::min(h - 1, cell.y + d); ncell.y++)
        {
            // printf("%i %i %i %i %i %i %i %i\n",ncell.x,ncell.y,w,h,w*h,depthMap->size(),cam,scale);
            float depth = (*depthMap)[ncell.x * h + ncell.y];
            // point3d p1 = mp->CArr[rc] +
            // (mp->iCamArr[rc]*point2d((float)ncell.x*(float)scale,(float)ncell.y*(float)scale)).normalize()*depth;
            // if ( (p1-p).size() < pixSize ) {
            if(fabs(pixDepth - depth) < pixSize)
            {
                (*numOfPtsMap)[ncell.x * h + ncell.y]++;
            }
        }
    }

    return true;
}

// minNumOfModals number of other cams including this cam ... minNumOfModals /in 2,3,...
void mv_fuse::filterGroups(const staticVector<int>& cams)
{
    printf("Precomputing groups\n");
    long t1 = clock();
    int pixSizeBall = mp->mip->_ini.get<int>("filter.pixSizeBall", 0);
    int pixSizeBallWSP = mp->mip->_ini.get<int>("filter.pixSizeBallWSP", 0);
    int nNearestCams = mp->mip->_ini.get<int>("prematching.nNearestCams", 10);

#pragma omp parallel for
    for(int c = 0; c < cams.size(); c++)
    {
        int rc = cams[c];
        filterGroupsRC(rc, pixSizeBall, pixSizeBallWSP, nNearestCams);
    }

    printfElapsedTime(t1);
}

// minNumOfModals number of other cams including this cam ... minNumOfModals /in 2,3,...
bool mv_fuse::filterGroupsRC(int rc, int pixSizeBall, int pixSizeBallWSP, int nNearestCams)
{

    if(FileExists(mv_getFileName(mp->mip, rc + 1, mp->mip->MV_FILE_TYPE_nmodMap)))
    {
        return true;
    }

    long t1 = clock();
    int w = mp->mip->getWidth(rc);
    int h = mp->mip->getHeight(rc);

    staticVector<float>* depthMap =
        loadArrayFromFile<float>(mv_getFileName(mp->mip, rc + 1, mp->mip->MV_FILE_TYPE_depthMap, 1));
    staticVector<float>* simMap =
        loadArrayFromFile<float>(mv_getFileName(mp->mip, rc + 1, mp->mip->MV_FILE_TYPE_simMap, 1));

    staticVector<unsigned char>* numOfModalsMap = new staticVector<unsigned char>(w * h);
    numOfModalsMap->resize_with(w * h, 0);

    if((depthMap == nullptr) || (simMap == nullptr) || (depthMap->size() != w * h) || (simMap->size() != w * h))
    {
        printf("WARNING filterGroupsRC %i \n", rc);
        if(depthMap != nullptr)
        {
            delete depthMap;
        }
        if(simMap != nullptr)
        {
            delete simMap;
        }
        depthMap = new staticVector<float>(w * h);
        depthMap->resize_with(w * h, -1.0f);
        simMap = new staticVector<float>(w * h);
        simMap->resize_with(w * h, 1.0f);
        saveArrayToFile<float>(mv_getFileName(mp->mip, rc + 1, mp->mip->MV_FILE_TYPE_depthMap, 1), depthMap);
        saveArrayToFile<float>(mv_getFileName(mp->mip, rc + 1, mp->mip->MV_FILE_TYPE_simMap, 1), simMap);
        saveArrayToFile<unsigned char>(mv_getFileName(mp->mip, rc + 1, mp->mip->MV_FILE_TYPE_nmodMap), numOfModalsMap);
        delete depthMap;
        delete simMap;
        delete numOfModalsMap;
        return false;
    }

    staticVector<int>* numOfPtsMap = new staticVector<int>(w * h);
    numOfPtsMap->resize_with(w * h, 0);

    // staticVector<int> *tcams = pc->findCamsWhichIntersectsCamHexah(rc);
    // staticVector<int> *tcams = pc->findNearestCams(rc);
    staticVector<int>* tcams = pc->findNearestCamsFromSeeds(rc, nNearestCams);

    for(int c = 0; c < tcams->size(); c++)
    {
        numOfPtsMap->resize_with(w * h, 0);
        int tc = (*tcams)[c];

        staticVector<float>* tcdepthMap =
            loadArrayFromFile<float>(mv_getFileName(mp->mip, tc + 1, mp->mip->MV_FILE_TYPE_depthMap, 1));
        if(tcdepthMap != nullptr)
        {
            for(int i = 0; i < sizeOfStaticVector<float>(tcdepthMap); i++)
            {
                int x = i / h;
                int y = i % h;
                float depth = (*tcdepthMap)[i];
                if(depth > 0.0f)
                {
                    point3d p = mp->CArr[tc] + (mp->iCamArr[tc] * point2d((float)x, (float)y)).normalize() * depth;
                    updateInSurr(pixSizeBall, pixSizeBallWSP, p, rc, tc, numOfPtsMap, depthMap, simMap, 1);
                }
            }
            delete tcdepthMap;
        }

        for(int i = 0; i < w * h; i++)
        {
            (*numOfModalsMap)[i] += static_cast<int>((*numOfPtsMap)[i] > 0);
        }
    }
    delete tcams;

    saveArrayToFile<unsigned char>(mv_getFileName(mp->mip, rc + 1, mp->mip->MV_FILE_TYPE_nmodMap), numOfModalsMap);

    delete numOfPtsMap;
    delete numOfModalsMap;
    delete depthMap;
    delete simMap;

    if(mp->verbose)
        printf("%i solved in ", rc);
    if(mp->verbose)
        printfElapsedTime(t1);

    return true;
}

// minNumOfModals number of other cams including this cam ... minNumOfModals /in 2,3,...
void mv_fuse::filterDepthMaps(const staticVector<int>& cams, int minNumOfModals)
{
    printf("Filtering depth maps\n");
    long t1 = clock();
    int minNumOfModalsWSP2SSP = mp->mip->_ini.get<int>("filter.minNumOfConsistentCamsWSP2SSP", 4);
    int pixSizeBall = mp->mip->_ini.get<int>("filter.pixSizeBall", 0);
    int pixSizeBallWSP = mp->mip->_ini.get<int>("filter.pixSizeBallWSP", 0);

#pragma omp parallel for
    for(int c = 0; c < cams.size(); c++)
    {
        int rc = cams[c];
        filterDepthMapsRC(rc, minNumOfModals, minNumOfModalsWSP2SSP);
    }

    printfElapsedTime(t1);
}

// minNumOfModals number of other cams including this cam ... minNumOfModals /in 2,3,...
bool mv_fuse::filterDepthMapsRC(int rc, int minNumOfModals, int minNumOfModalsWSP2SSP)
{
    long t1 = clock();
    int w = mp->mip->getWidth(rc);
    int h = mp->mip->getHeight(rc);

    staticVector<float>* depthMap =
        loadArrayFromFile<float>(mv_getFileName(mp->mip, rc + 1, mp->mip->MV_FILE_TYPE_depthMap, 1));
    staticVector<float>* simMap =
        loadArrayFromFile<float>(mv_getFileName(mp->mip, rc + 1, mp->mip->MV_FILE_TYPE_simMap, 1));
    staticVector<unsigned char>* numOfModalsMap =
        loadArrayFromFile<unsigned char>(mv_getFileName(mp->mip, rc + 1, mp->mip->MV_FILE_TYPE_nmodMap));

    for(int i = 0; i < w * h; i++)
    {
        // if the reference point is consistent in three target cameras and is denoted as weakly supported point
        // make him strongly supported
        if(((*numOfModalsMap)[i] >= minNumOfModalsWSP2SSP - 1) && ((*simMap)[i] >= 1.0f))
        {
            (*simMap)[i] = (*simMap)[i] - 2.0f;
        }

        // if it is conistent in only one camera and is weakly supported then remove him
        // weakly supported point must be consisten in at least two cameras
        if(((*numOfModalsMap)[i] <= 1) && ((*simMap)[i] >= 1.0f))
        {
            (*depthMap)[i] = -1.0f;
            (*simMap)[i] = 1.0f;
        }

        // if it is not conistent in minimal number of cameras and is strongly supported then remove him
        if(((*numOfModalsMap)[i] < minNumOfModals - 1) && ((*simMap)[i] < 1.0f))
        {
            (*depthMap)[i] = -1.0f;
            (*simMap)[i] = 1.0f;
        }
    }

    saveArrayToFile<float>(mv_getFileName(mp->mip, rc + 1, mp->mip->MV_FILE_TYPE_depthMap, 0), depthMap);
    saveArrayToFile<float>(mv_getFileName(mp->mip, rc + 1, mp->mip->MV_FILE_TYPE_simMap, 0), simMap);

    // visualizeDepthMap(rc, mp->mip->newDir+num2strFourDecimal(rc)+"_scale"+num2str(0)+"fused.wrl",0,1,scales);

    delete numOfModalsMap;
    delete depthMap;
    delete simMap;

    if(mp->verbose)
        printf("%i solved in ", rc);
    if(mp->verbose)
        printfElapsedTime(t1);

    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float mv_fuse::computeAveragePixelSizeInHexahedron(point3d* hexah, int step, int scale)
{
    int scaleuse = std::max(1, scale);

    staticVector<int>* cams = pc->findCamsWhichIntersectsHexahedron(hexah);
    int j = 0;
    float av = 0.0f;
    float nav = 0.0f;
    float minv = std::numeric_limits<float>::max();
    long t1 = initEstimate();
    for(int c = 0; c < cams->size(); c++)
    {
        int rc = (*cams)[c];
        int h = mp->mip->getHeight(rc) / scaleuse;
        staticVector<float>* rcdepthMap =
            loadArrayFromFile<float>(mv_getFileName(mp->mip, rc + 1, mp->mip->MV_FILE_TYPE_depthMap, scale));
        for(int i = 0; i < rcdepthMap->size(); i++)
        {
            int x = i / h;
            int y = i % h;
            float depth = (*rcdepthMap)[i];
            if(depth > 0.0f)
            {
                if(j % step == 0)
                {
                    point3d p = mp->CArr[rc] +
                                (mp->iCamArr[rc] * point2d((float)x * (float)scaleuse, (float)y * (float)scaleuse))
                                        .normalize() *
                                    depth;
                    if(isPointInHexahedron(p, hexah))
                    {
                        float v = mp->getCamPixelSize(p, rc);
                        av += v; // WARNING: the value may be too big for a float
                        nav += 1.0f;
                        minv = std::min(minv, v);
                    }
                }
                j++;
            }
        }
        delete rcdepthMap;
        printfEstimate(c, cams->size(), t1);
    }
    finishEstimate();
    delete cams;

    if(nav == 0.0f)
    {
        return -1.0f;
    }

    return av / nav;

    // return minv;
}

/**
 *@param[out] hexah: table of 8 values
 *@param[out] minPixSize
 */
void mv_fuse::divideSpace(point3d* hexah, float& minPixSize)
{
    printf("Estimate space\n");
    int scale = 0;

    unsigned long npset = computeNumberOfAllPoints(scale);
    int stepPts = std::max(1, (int)(npset / (unsigned long)3000000));

    minPixSize = std::numeric_limits<float>::max();
    long t1 = initEstimate();
    stat3d s3d = stat3d();
    for(int rc = 0; rc < mp->ncams; rc++)
    {
        int h = mp->mip->getHeight(rc);
        staticVector<float>* depthMap =
            loadArrayFromFile<float>(mv_getFileName(mp->mip, rc + 1, mp->mip->MV_FILE_TYPE_depthMap, scale));
        for(int i = 0; i < sizeOfStaticVector<float>(depthMap); i += stepPts)
        {
            int x = i / h;
            int y = i % h;
            float depth = (*depthMap)[i];
            if(depth > 0.0f)
            {
                point3d p = mp->CArr[rc] + (mp->iCamArr[rc] * point2d((float)x, (float)y)).normalize() * depth;
                float pixSize = mp->getCamPixelSize(p, rc);
                minPixSize = std::min(minPixSize, pixSize);
                s3d.update(&p);
            }
        }
        if(depthMap != nullptr)
        {
            delete depthMap;
        }
        printfEstimate(rc, mp->ncams, t1);
    }
    finishEstimate();

    point3d v1, v2, v3, cg;
    float d1, d2, d3;
    s3d.getEigenVectorsDesc(cg, v1, v2, v3, d1, d2, d3);

    using namespace boost::accumulators;
    using Accumulator = accumulator_set<float, stats<
            tag::tail_quantile<right>
            >>;
    const std::size_t cacheSize =  10000;
    Accumulator accX1( tag::tail<right>::cache_size = cacheSize );
    Accumulator accX2( tag::tail<right>::cache_size = cacheSize );
    Accumulator accY1( tag::tail<right>::cache_size = cacheSize );
    Accumulator accY2( tag::tail<right>::cache_size = cacheSize );
    Accumulator accZ1( tag::tail<right>::cache_size = cacheSize );
    Accumulator accZ2( tag::tail<right>::cache_size = cacheSize );

    for(int rc = 0; rc < mp->ncams; ++rc)
    {
        int h = mp->mip->getHeight(rc);
        staticVector<float>* depthMap =
            loadArrayFromFile<float>(mv_getFileName(mp->mip, rc + 1, mp->mip->MV_FILE_TYPE_depthMap, scale));
        for(int i = 0; i < depthMap->size(); i += stepPts)
        {
            int x = i / h;
            int y = i % h;
            float depth = (*depthMap)[i];
            if(depth > 0.0f)
            {
                point3d p = mp->CArr[rc] + (mp->iCamArr[rc] * point2d((float)x, (float)y)).normalize() * depth;
                float d1 = orientedPointPlaneDistance(p, cg, v1);
                float d2 = orientedPointPlaneDistance(p, cg, v2);
                float d3 = orientedPointPlaneDistance(p, cg, v3);

                if(d1 < 0)
                    accX1(fabs(d1));
                else
                    accX2(fabs(d1));

                if(d2 < 0)
                    accY1(fabs(d2));
                else
                    accY2(fabs(d2));

                if(d3 < 0)
                    accZ1(fabs(d3));
                else
                    accZ2(fabs(d3));
            }
        }
        delete depthMap;

        printfEstimate(rc, mp->ncams, t1);
    }
    finishEstimate();

    float perc = (float)mp->mip->_ini.get<double>("largeScale.universePercentile", 0.999f);

    float mind1 = -quantile(accX1, quantile_probability = perc);
    float maxd1 = quantile(accX2, quantile_probability = perc);
    float mind2 = -quantile(accY1, quantile_probability = perc);
    float maxd2 = quantile(accY2, quantile_probability = perc);
    float mind3 = -quantile(accZ1, quantile_probability = perc);
    float maxd3 = quantile(accZ2, quantile_probability = perc);

//    std::cout << "quantile accumulators:" << std::endl;
//    std::cout << "X(" << mind1 << ", " << maxd1 << "), "
//              << "Y(" << mind2 << ", " << maxd2 << "), "
//              << "Z(" << mind3 << ", " << maxd3 << "), "
//              << std::endl;

    hexah[0] = cg + v1 * maxd1 + v2 * maxd2 + v3 * maxd3;
    hexah[1] = cg + v1 * mind1 + v2 * maxd2 + v3 * maxd3;
    hexah[2] = cg + v1 * mind1 + v2 * mind2 + v3 * maxd3;
    hexah[3] = cg + v1 * maxd1 + v2 * mind2 + v3 * maxd3;
    hexah[4] = cg + v1 * maxd1 + v2 * maxd2 + v3 * mind3;
    hexah[5] = cg + v1 * mind1 + v2 * maxd2 + v3 * mind3;
    hexah[6] = cg + v1 * mind1 + v2 * mind2 + v3 * mind3;
    hexah[7] = cg + v1 * maxd1 + v2 * mind2 + v3 * mind3;
}

voxel mv_fuse::estimateDimensions(point3d* vox, point3d* newSpace, int scale, int maxOcTreeDim)
{
    point3d O = (vox[0] + vox[1] + vox[2] + vox[3] + vox[4] + vox[5] + vox[6] + vox[7]) / 8.0f;
    point3d vx = vox[1] - vox[0];
    point3d vy = vox[3] - vox[0];
    point3d vz = vox[4] - vox[0];
    float svx = vx.size();
    float svy = vy.size();
    float svz = vz.size();
    vx = vx.normalize();
    vy = vy.normalize();
    vz = vz.normalize();

    int nAllPts = computeNumberOfAllPoints(scale);
    float pointToJoinPixSizeDist = (float)mp->mip->_ini.get<double>("mv_fuse.pointToJoinPixSizeDist", 2.0f);
    std::cout << "pointToJoinPixSizeDist: " << pointToJoinPixSizeDist << std::endl;
    int maxpts = 1000000;
    int stepPts = nAllPts / maxpts + 1;
    // WARNING perf: reload all depth maps to compute the minPixelSize (minPixelSize consider only points in the hexahedron)
    // Average 3D size for each pixel from all 3D points in the current voxel
    float aAvPixelSize = computeAveragePixelSizeInHexahedron(vox, stepPts, scale) * (float)std::max(scale, 1) * pointToJoinPixSizeDist;

    voxel maxDim;
    maxDim.x = (int)ceil(svx / (aAvPixelSize * (float)maxOcTreeDim));
    maxDim.y = (int)ceil(svy / (aAvPixelSize * (float)maxOcTreeDim));
    maxDim.z = (int)ceil(svz / (aAvPixelSize * (float)maxOcTreeDim));

    point3d vvx = vx * ((float)maxDim.x * ((aAvPixelSize * (float)maxOcTreeDim)));
    point3d vvy = vy * ((float)maxDim.y * ((aAvPixelSize * (float)maxOcTreeDim)));
    point3d vvz = vz * ((float)maxDim.z * ((aAvPixelSize * (float)maxOcTreeDim)));
    newSpace[0] = O - vvx / 2.0f - vvy / 2.0f - vvz / 2.0f;
    newSpace[1] = O + vvx - vvx / 2.0f - vvy / 2.0f - vvz / 2.0f;
    newSpace[2] = O + vvx + vvy - vvx / 2.0f - vvy / 2.0f - vvz / 2.0f;
    newSpace[3] = O + vvy - vvx / 2.0f - vvy / 2.0f - vvz / 2.0f;
    newSpace[4] = O + vvz - vvx / 2.0f - vvy / 2.0f - vvz / 2.0f;
    newSpace[5] = O + vvz + vvx - vvx / 2.0f - vvy / 2.0f - vvz / 2.0f;
    newSpace[6] = O + vvz + vvx + vvy - vvx / 2.0f - vvy / 2.0f - vvz / 2.0f;
    newSpace[7] = O + vvz + vvy - vvx / 2.0f - vvy / 2.0f - vvz / 2.0f;

    printf("Estimated : %s %s %s \n", num2str(maxDim.x).c_str(), num2str(maxDim.y).c_str(), num2str(maxDim.z).c_str());

    if(mp->verbose)
        printf("optimal detail: %i %i %i\n", (int)((vvx.size() / (float)maxDim.x) / aAvPixelSize),
               (int)((vvy.size() / (float)maxDim.y) / aAvPixelSize),
               (int)((vvz.size() / (float)maxDim.z) / aAvPixelSize));

    return maxDim;
}

mv_universe* mv_fuse::segmentDepthMap(float alpha, int rc, staticVector<float>* depthMap, int* segMap, int scale)
{
    printf("segmenting to connected components \n");
    int w = mp->mip->getWidth(rc) / std::max(1, scale);
    int h = mp->mip->getHeight(rc) / std::max(1, scale);

    staticVector<pixel>* edges = new staticVector<pixel>(w * h * 2);
    for(int x = 0; x < w - 1; x++)
    {
        for(int y = 0; y < h - 1; y++)
        {
            float depth = (*depthMap)[x * h + y];
            float depthr = (*depthMap)[(x + 1) * h + y];
            float depthd = (*depthMap)[x * h + (y + 1)];
            point3d p = mp->CArr[rc] + (mp->iCamArr[rc] * point2d((float)x, (float)y)).normalize() * depth;
            float pixSize = alpha * mp->getCamPixelSize(p, rc);

            if(segMap == nullptr)
            {
                if(fabs(depth - depthr) < pixSize)
                {
                    edges->push_back(pixel(x * h + y, (x + 1) * h + y));
                }
                if(fabs(depth - depthd) < pixSize)
                {
                    edges->push_back(pixel(x * h + y, x * h + (y + 1)));
                }
            }
            else
            {
                int seg = segMap[x * h + y];
                int segr = segMap[(x + 1) * h + y];
                int segd = segMap[x * h + (y + 1)];

                if((fabs(depth - depthr) < pixSize) && (seg == segr))
                {
                    edges->push_back(pixel(x * h + y, (x + 1) * h + y));
                }
                if((fabs(depth - depthd) < pixSize) && (seg == segd))
                {
                    edges->push_back(pixel(x * h + y, x * h + (y + 1)));
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


void mv_fuse::filterSmallConnComponents(float alpha, int minSegSize, int scale)
{
    printf("filtering out connected components smaller than %i pixels\n", minSegSize);

    long t1 = initEstimate();
    for(int rc = 0; rc < mp->ncams; rc++)
    {
        int w = mp->mip->getWidth(rc) / std::max(1, scale);
        int h = mp->mip->getHeight(rc) / std::max(1, scale);
        staticVector<float>* depthMap =
            loadArrayFromFile<float>(mv_getFileName(mp->mip, rc + 1, mp->mip->MV_FILE_TYPE_depthMap, scale));
        staticVector<float>* simMap =
            loadArrayFromFile<float>(mv_getFileName(mp->mip, rc + 1, mp->mip->MV_FILE_TYPE_simMap, scale));

        mv_universe* u = segmentDepthMap(alpha, rc, depthMap, nullptr, scale);

        for(int x = 0; x < w; x++)
        {
            for(int y = 0; y < h; y++)
            {
                int id = x * h + y;
                int a = u->find(id);
                int size = u->elts[a].size;
                if(size < minSegSize)
                {
                    (*depthMap)[x * h + y] = -1.0f;
                    (*simMap)[x * h + y] = 1.0f;
                }
            }
        }

        delete u;

        saveArrayToFile<float>(mv_getFileName(mp->mip, rc + 1, mp->mip->MV_FILE_TYPE_depthMap, scale), depthMap);
        saveArrayToFile<float>(mv_getFileName(mp->mip, rc + 1, mp->mip->MV_FILE_TYPE_simMap, scale), simMap);

        // visualizeDepthMap(rc, mp->mip->newDir+num2strFourDecimal(rc)+"fused.wrl");

        delete depthMap;
        delete simMap;

        printfEstimate(rc, mp->ncams, t1);
    }
    finishEstimate();
}

std::string generateTempPtsSimsFiles(std::string tmpDir, multiviewParams* mp, bool addRandomNoise, float percNoisePts,
                                     int noisPixSizeDistHalfThr)
{
    printf("generating temp files\n");
    std::string depthMapsPtsSimsTmpDir = tmpDir + "depthMapsPtsSimsTmp/";

    if(!FolderExists(depthMapsPtsSimsTmpDir))
    {
        bfs::create_directory(depthMapsPtsSimsTmpDir);

        int scale = 0;
        int scaleuse = std::max(1, scale);

        staticVector<point2d>* minMaxDepths = new staticVector<point2d>(mp->ncams);
        minMaxDepths->resize_with(mp->ncams, point2d(-1.0, -1.0));

#pragma omp parallel for
        for(int rc = 0; rc < mp->ncams; rc++)
        {
            int w = mp->mip->getWidth(rc) / scaleuse;
            int h = mp->mip->getHeight(rc) / scaleuse;
            staticVector<point3d>* pts = new staticVector<point3d>(w * h);
            staticVector<float>* sims = new staticVector<float>(w * h);
            staticVector<float>* depthMap =
                loadArrayFromFile<float>(mv_getFileName(mp->mip, rc + 1, mp->mip->MV_FILE_TYPE_depthMap, scale));
            staticVector<float>* simMap =
                loadArrayFromFile<float>(mv_getFileName(mp->mip, rc + 1, mp->mip->MV_FILE_TYPE_simMap, scale));

            if(addRandomNoise)
            {
                staticVector<int>* idsAlive = new staticVector<int>(w * h);
                for(int i = 0; i < w * h; i++)
                {
                    if((*depthMap)[i] > 0.0f)
                    {
                        idsAlive->push_back(i);
                    }
                }

                int nnoisePts = ((percNoisePts / 100.0f) * (float)(idsAlive->size()));
                staticVector<int>* randIdsAlive = createRandomArrayOfIntegers(idsAlive->size());

                srand(time(nullptr));

                long t1 = clock();
                for(int id = 0; id < idsAlive->size(); id++)
                {
                    int i = (*idsAlive)[(*randIdsAlive)[id]];
                    int x = i / h;
                    int y = i % h;
                    double depth = (*depthMap)[i];

                    double sim = (*simMap)[i];
                    if(depth > 0.0f)
                    {
                        point3d p = mp->CArr[rc] +
                                    (mp->iCamArr[rc] * point2d((double)x * (double)scaleuse, (double)y * (double)scaleuse))
                                            .normalize() *
                                        depth;

                        if(id < nnoisePts)
                        {
                            double pixSize = mp->getCamPixelSize(p, rc);
                            int rid = rand() % (2 * noisPixSizeDistHalfThr + 1);
                            rid = rid - noisPixSizeDistHalfThr;
                            double rdepthAdd = pixSize * (double)rid;
                            depth = depth + rdepthAdd;
                            p = mp->CArr[rc] +
                                (mp->iCamArr[rc] * point2d((double)x * (double)scaleuse, (double)y * (double)scaleuse))
                                        .normalize() *
                                    depth;
                        }

                        pts->push_back(p);
                        sims->push_back(sim);
                        if((*minMaxDepths)[rc].x < 0.0f)
                        {
                            (*minMaxDepths)[rc].x = depth;
                        }
                        else
                        {
                            (*minMaxDepths)[rc].x = std::min((*minMaxDepths)[rc].x, depth);
                        }
                        if((*minMaxDepths)[rc].y < 0.0f)
                        {
                            (*minMaxDepths)[rc].y = depth;
                        }
                        else
                        {
                            (*minMaxDepths)[rc].y = std::max((*minMaxDepths)[rc].y, depth);
                        }
                    }
                }
                if(mp->verbose)
                    printf("%i depth map -> 3D pts arr : ", rc);
                if(mp->verbose)
                    printfElapsedTime(t1);

                delete idsAlive;
                delete randIdsAlive;
            }
            else
            {

                long t1 = clock();
                for(int x = 0; x < w; x++)
                {
                    for(int y = 0; y < h; y++)
                    {
                        int i = x * h + y;
                        double depth = (*depthMap)[i];
                        double sim = (*simMap)[i];
                        if(depth > 0.0f)
                        {
                            point3d p =
                                mp->CArr[rc] +
                                (mp->iCamArr[rc] * point2d((double)x * (double)scaleuse, (double)y * (double)scaleuse))
                                        .normalize() *
                                    depth;
                            pts->push_back(p);
                            sims->push_back(sim);
                            if((*minMaxDepths)[rc].x < 0.0f)
                            {
                                (*minMaxDepths)[rc].x = depth;
                            }
                            else
                            {
                                (*minMaxDepths)[rc].x = std::min((*minMaxDepths)[rc].x, depth);
                            }
                            if((*minMaxDepths)[rc].y < 0.0f)
                            {
                                (*minMaxDepths)[rc].y = depth;
                            }
                            else
                            {
                                (*minMaxDepths)[rc].y = std::max((*minMaxDepths)[rc].y, depth);
                            }
                        }
                    }
                }
                if(mp->verbose)
                    printf("%i depth map -> 3D pts arr : ", rc);
                if(mp->verbose)
                    printfElapsedTime(t1);
            }

            delete depthMap;
            delete simMap;
            saveArrayToFile<point3d>(depthMapsPtsSimsTmpDir + num2strFourDecimal(rc) + "pts.bin", pts);
            saveArrayToFile<float>(depthMapsPtsSimsTmpDir + num2strFourDecimal(rc) + "sims.bin", sims);
            delete pts;
            delete sims;
        }

        saveArrayToFile<point2d>(depthMapsPtsSimsTmpDir + "minMaxDepths.bin", minMaxDepths);
        delete minMaxDepths;
    }

    return depthMapsPtsSimsTmpDir;
}

void deleteTempPtsSimsFiles(multiviewParams* mp, std::string depthMapsPtsSimsTmpDir)
{
    for(int rc = 0; rc < mp->ncams; rc++)
    {
        std::string ptsfn = depthMapsPtsSimsTmpDir + num2strFourDecimal(rc) + "pts.bin";
        std::string simsfn = depthMapsPtsSimsTmpDir + num2strFourDecimal(rc) + "sims.bin";
        remove(ptsfn.c_str());
        remove(simsfn.c_str());
    }
    DeleteDirectory(depthMapsPtsSimsTmpDir);
}
