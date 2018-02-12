// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "mv_mesh_orthomosaic.hpp"
#include <aliceVision/imageIO/image.hpp>

mv_mesh_orthomosaic::mv_mesh_orthomosaic(std::string _tmpDir, std::string _demName, multiviewParams* _mp)
    : mv_mesh_dem(_tmpDir, -1, -1, _mp)
{
    demName = _demName;

    initDEMFromFile(demName);

    demPixS.x = dim.x / (float)demW;
    demPixS.y = dim.y / (float)demH;
    demPixS.z = dim.z / 65535.0f;

    // staticVector<staticVector<int>*> *ptsCams = loadArrayOfArraysFromFile<int>(ptsCamsFileName);
    // camsPts = convertObjectsCamsToCamsObjects(mp, ptsCams);
    // deleteArrayOfArrays<int>(&ptsCams);

    percentile = (float)mp->mip->_ini.get<double>("DEM.percentile", 0.999);

    orm = new staticVector<point4d>(demW * demH);
    orm->resize_with(demW * demH, point4d(0.0f, 0.0f, 0.0f, 0.0f));

    pixSizeMap = new staticVector<float>(demW * demH);
    pixSizeMap->resize_with(demW * demH, 0.0f);

    int bandType = 0;
    ic = new mv_images_cache(mp, bandType, true);

    visualizeMetaData = (bool)mp->mip->_ini.get<bool>("orthomosaic.visualizeMetaData", false);
    doAverage = (bool)mp->mip->_ini.get<bool>("orthomosaic.doAverage", true);
}

mv_mesh_orthomosaic::~mv_mesh_orthomosaic()
{
    delete ic;
    // deleteArrayOfArrays<int>(&camsPts);
    delete orm;
    delete pixSizeMap;
}

void mv_mesh_orthomosaic::computeRcOrthomosaic(int rc, staticVector<float>* dem, staticVector<float>* demGlob)
{
    staticVector<point4d>* rcorm = nullptr;

    if(visualizeMetaData)
    {
        rcorm = new staticVector<point4d>(demW * demH);
        rcorm->resize_with(demW * demH, point4d(0.0f, 0.0f, 0.0f, 0.0f));
    }

    for(int y = 0; y < demH; y++)
    {
        for(int x = 0; x < demW; x++)
        {
            float demz = (*dem)[y * demW + x];
            float demzGlob = (*demGlob)[y * demW + x];
            if((demzGlob == demzGlob) && (!std::isnan(demzGlob)) && (demzGlob > 0.0f) && (demzGlob < dim.z) &&
               (demz == demz) && (!std::isnan(demz)) && (demz > 0.0f) && (demz < dim.z))
            {
                point3d p = oax + xax * (demPixS.x * (float)x) + yax * (demPixS.y * (float)y) + zax * demz;
                point2d pix;
                mp->getPixelFor3DPoint(&pix, p, rc);
                if(mp->isPixelInImage(pix))
                {
                    point3d pGlob = oax + xax * (demPixS.x * (float)x) + yax * (demPixS.y * (float)y) + zax * demzGlob;
                    float pixSize = mp->getCamPixelSize(p, rc);
                    float pixSizeThr = 5.0f * pixSize;

                    if((p - pGlob).size() < pixSizeThr)
                    {
                        point3d col = ic->getPixelValueInterpolated(&pix, rc);
                        if(visualizeMetaData)
                        {
                            (*rcorm)[y * demW + x].x += col.x;
                            (*rcorm)[y * demW + x].y += col.y;
                            (*rcorm)[y * demW + x].z += col.z;
                            (*rcorm)[y * demW + x].w += 1.0f;
                        }
                        if(doAverage)
                        {
                            (*orm)[y * demW + x].x += col.x;
                            (*orm)[y * demW + x].y += col.y;
                            (*orm)[y * demW + x].z += col.z;
                            (*orm)[y * demW + x].w += 1.0f;
                            (*pixSizeMap)[y * demW + x] += pixSize;
                        }
                        else
                        {
                            float w = (*orm)[y * demW + x].w;
                            float d = (p - mp->CArr[rc]).size();
                            if((w == 0.0f) || (w > d))
                            {
                                (*orm)[y * demW + x].x = col.x;
                                (*orm)[y * demW + x].y = col.y;
                                (*orm)[y * demW + x].z = col.z;
                                (*orm)[y * demW + x].w = d;
                                (*pixSizeMap)[y * demW + x] = pixSize;
                            }
                        }
                    }
                }
            }
        }
    }

    if(visualizeMetaData)
    {
        for(int i = 0; i < demW * demH; i++)
        {
            if((*rcorm)[i].w > 0.0)
            {
                (*rcorm)[i].x = (*rcorm)[i].x / (*rcorm)[i].w;
                (*rcorm)[i].y = (*rcorm)[i].y / (*rcorm)[i].w;
                (*rcorm)[i].z = (*rcorm)[i].z / (*rcorm)[i].w;
                (*rcorm)[i].w = (*rcorm)[i].w / (*rcorm)[i].w;
            }
        }

        std::vector<Color> colorBuffer(demW * demH);
        std::vector<float> maskBuffer(demW * demH);

        for(int y = 0; y < demH; y++)
        {
            for(int x = 0; x < demW; x++)
            {
                const std::size_t index = y * demW + x;
                const point4d p = (*rcorm)[index];
                colorBuffer.at(index) = Color(p.x, p.y, p.z);
                maskBuffer.at(index) = p.w;
            }
        }

        std::string imageFileName = tmpDir + num2strFourDecimal(rc) + "orthoMosaic.png";
        imageIO::writeImage(imageFileName, demW, demH, colorBuffer);

        std::string imageMaskFileName = tmpDir + num2strFourDecimal(rc) + "orthoMosaicMask.png";
        imageIO::writeImage(imageMaskFileName, demW, demH, maskBuffer);

        delete rcorm;
    }
}
