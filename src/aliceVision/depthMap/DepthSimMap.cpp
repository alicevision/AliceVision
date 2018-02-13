// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "DepthSimMap.hpp"
#include <aliceVision/common/common.hpp>
#include <aliceVision/common/fileIO.hpp>
#include <aliceVision/structures/Color.hpp>
#include <aliceVision/structures/geometry.hpp>
#include <aliceVision/structures/jetColorMap.hpp>
#include <aliceVision/imageIO/image.hpp>

#include <iostream>

DepthSimMap::DepthSimMap(int _rc, multiviewParams* _mp, int _scale, int _step)
{
    rc = _rc;
    mp = _mp;
    scale = _scale;
    step = _step;
    w = mp->mip->getWidth(rc) / (scale * step);
    h = mp->mip->getHeight(rc) / (scale * step);
    dsm = new StaticVector<DepthSim>(w * h);
    dsm->resize_with(w * h, DepthSim(-1.0f, 1.0f));
}

DepthSimMap::~DepthSimMap()
{
    delete dsm;
}

Point3d DepthSimMap::get3DPtOfPixel(const Pixel& pix, int pixScale, int rc)
{
    Pixel pix11 = Pixel(pix.x * pixScale, pix.y * pixScale);
    int i = (pix11.y / (scale * step)) * w + pix11.x / (scale * step);
    float depth = (*dsm)[i].depth;
    return mp->CArr[rc] +
           (mp->iCamArr[rc] *
            Point2d((float)(pix11.x) + (float)pixScale / 2.0f, (float)pix11.y + (float)pixScale / 2.0f))
                   .normalize() *
               depth;
}

float DepthSimMap::getFPDepthOfPixel(const Pixel& pix, int pixScale, int rc)
{
    Point3d p = get3DPtOfPixel(pix, pixScale, rc);
    Point3d zVect = (mp->iRArr[rc] * Point3d(0.0f, 0.0f, 1.0f)).normalize();
    return pointPlaneDistance(p, mp->CArr[rc], zVect);
}

void DepthSimMap::add11(DepthSimMap* depthSimMap)
{
    if((scale != 1) || (step != 1))
    {
        throw std::runtime_error("Error DepthSimMap: You can only add to scale1-step1 map.");
    }

    int k = (depthSimMap->step * depthSimMap->scale) / 2;
    int k1 = k;
    if((depthSimMap->step * depthSimMap->scale) % 2 == 0)
        k -= 1;

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
                    if((xp >= 0) && (xp < w) && (yp >= 0) && (yp < h) && // check image borders
                       (depthSim.sim > (*dsm)[yp * w + xp].sim))
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

void DepthSimMap::add(DepthSimMap* depthSimMap)
{
    if((scale != depthSimMap->scale) || (step != depthSimMap->step))
    {
        throw std::runtime_error("Error DepthSimMap: You can only add to the same scale and step map.");
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

void DepthSimMap::getReconstructedPixelsDepthsSims(StaticVector<Pixel>* pixels, StaticVector<float>* depths,
                                                      StaticVector<float>* sims)
{
    for(int j = 0; j < w * h; j++)
    {
        if((*dsm)[j].depth > 0.0f)
        {
            pixels->push_back(Pixel((j % w) * step, (j / w) * step));
            depths->push_back((*dsm)[j].depth);
            sims->push_back((*dsm)[j].sim);
        }
    }
}

Point2d DepthSimMap::getMaxMinDepth()
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
    return Point2d(maxDepth, minDepth);
}

Point2d DepthSimMap::getMaxMinSim()
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
    return Point2d(maxSim, minSim);
}

void DepthSimMap::setUsedCellsSimTo(float defaultSim)
{
    for(int j = 0; j < w * h; j++)
    {
        if((*dsm)[j].depth > -1.0f)
        {
            (*dsm)[j].sim = defaultSim;
        }
    }
}

float DepthSimMap::getPercentileDepth(float perc)
{

    int step = std::max(1, (w * h) / 50000);
    int n = (w * h) / std::max(1, (step - 1));
    // if (mp->verbose) printf("%i\n",n);
    StaticVector<float>* depths = new StaticVector<float>(n);

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
StaticVector<float>* DepthSimMap::getDepthMapStep1()
{
	// Size of our input image (with scale applied)
    int wdm = mp->mip->getWidth(rc) / scale;
    int hdm = mp->mip->getHeight(rc) / scale;

	// Create a depth map at the size of our input image
    StaticVector<float>* depthMap = new StaticVector<float>(wdm * hdm);
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

StaticVector<float>* DepthSimMap::getSimMapStep1()
{
    int wdm = mp->mip->getWidth(rc) / scale;
    int hdm = mp->mip->getHeight(rc) / scale;

    StaticVector<float>* simMap = new StaticVector<float>(wdm * hdm);
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

StaticVector<float>* DepthSimMap::getDepthMapStep1XPart(int xFrom, int partW)
{
    int wdm = mp->mip->getWidth(rc) / scale;
    int hdm = mp->mip->getHeight(rc) / scale;

    StaticVector<float>* depthMap = new StaticVector<float>(wdm * hdm);
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

StaticVector<float>* DepthSimMap::getSimMapStep1XPart(int xFrom, int partW)
{
    int wdm = mp->mip->getWidth(rc) / scale;
    int hdm = mp->mip->getHeight(rc) / scale;

    StaticVector<float>* simMap = new StaticVector<float>(wdm * hdm);
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

void DepthSimMap::initJustFromDepthMapT(StaticVector<float>* depthMapT, float defaultSim)
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

void DepthSimMap::initJustFromDepthMap(StaticVector<float>* depthMap, float defaultSim)
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

void DepthSimMap::initFromDepthMapTAndSimMapT(StaticVector<float>* depthMapT, StaticVector<float>* simMapT,
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

void DepthSimMap::initFromDepthMapAndSimMap(StaticVector<float>* depthMap, StaticVector<float>* simMap,
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

StaticVector<float>* DepthSimMap::getDepthMapTStep1()
{
    int wdm = mp->mip->getWidth(rc) / scale;
    int hdm = mp->mip->getHeight(rc) / scale;

    StaticVector<float>* depthMap = new StaticVector<float>(wdm * hdm);
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

StaticVector<float>* DepthSimMap::getDepthMap()
{
    StaticVector<float>* depthMap = new StaticVector<float>(dsm->size());
    for(int i = 0; i < dsm->size(); i++)
    {
        depthMap->push_back((*dsm)[i].depth);
    }
    return depthMap;
}

StaticVector<float>* DepthSimMap::getSimMapTStep1()
{
    int wdm = mp->mip->getWidth(rc) / scale;
    int hdm = mp->mip->getHeight(rc) / scale;

    StaticVector<float>* simMap = new StaticVector<float>(wdm * hdm);
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

void DepthSimMap::saveToImage(std::string filename, float simThr)
{
    const int bufferWidth = 2 * w;
    std::vector<Color> colorBuffer(bufferWidth * h);

    try 
    {
        Point2d maxMinDepth;
        maxMinDepth.x = getPercentileDepth(0.9) * 1.1;
        maxMinDepth.y = getPercentileDepth(0.01) * 0.8;

        Point2d maxMinSim = Point2d(simThr, -1.0f);
        if(simThr < -1.0f)
        {
            Point2d autoMaxMinSim = getMaxMinSim();
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

void DepthSimMap::save(int rc, StaticVector<int>* tcams)
{
    StaticVector<float>* depthMap = getDepthMapStep1();
    StaticVector<float>* simMap = getSimMapStep1();

    const int width = mp->mip->getWidth(rc) / scale;
    const int height = mp->mip->getHeight(rc) / scale;

    oiio::ParamValueList metadata;
    metadata.emplace_back(oiio::ParamValue("AliceVision:CArr", oiio::TypeDesc(oiio::TypeDesc::DOUBLE, oiio::TypeDesc::VEC3), 1, mp->CArr[rc].m));
    metadata.emplace_back(oiio::ParamValue("AliceVision:iCamArr", oiio::TypeDesc(oiio::TypeDesc::DOUBLE, oiio::TypeDesc::MATRIX33), 1, mp->iCamArr[rc].m));
    // TODO: remove "+ 1"
    imageIO::writeImage(mv_getFileName(mp->mip, rc + 1, EFileType::depthMap, scale), width, height, depthMap->getDataWritable(), imageIO::EImageQuality::LOSSLESS, metadata);
    imageIO::writeImage(mv_getFileName(mp->mip, rc + 1, EFileType::simMap, scale), width, height, simMap->getDataWritable());

    {
        Point2d maxMinDepth = getMaxMinDepth();
        FILE* f = mv_openFile(mp->mip, mp->indexes[rc], EFileType::depthMapInfo, "w");
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

void DepthSimMap::load(int rc, int fromScale)
{
    int width, height;

    StaticVector<float> depthMap;
    StaticVector<float> simMap;

    imageIO::readImage(mv_getFileName(mp->mip, rc + 1, EFileType::depthMap, fromScale), width, height, depthMap.getDataWritable());
    imageIO::readImage(mv_getFileName(mp->mip, rc + 1, EFileType::simMap, fromScale), width, height, simMap.getDataWritable());

    imageIO::transposeImage(width, height, depthMap.getDataWritable());
    imageIO::transposeImage(width, height, simMap.getDataWritable());

    initFromDepthMapTAndSimMapT(&depthMap, &simMap, fromScale);
}

void DepthSimMap::saveRefine(int rc, std::string depthMapFileName, std::string simMapFileName)
{
    const int width = mp->mip->getWidth(rc);
    const int height = mp->mip->getHeight(rc);
    const int size = width * height;

    std::vector<float> depthMap(size);
    std::vector<float> simMap(size);

    for(int i = 0; i < dsm->size(); ++i)
    {
        depthMap.at(i) = (*dsm)[i].depth;
        simMap.at(i) = (*dsm)[i].sim;
    }

    oiio::ParamValueList metadata;
    metadata.emplace_back(oiio::ParamValue("AliceVision:CArr", oiio::TypeDesc(oiio::TypeDesc::DOUBLE, oiio::TypeDesc::VEC3), 1, mp->CArr[rc].m));
    metadata.emplace_back(oiio::ParamValue("AliceVision:iCamArr", oiio::TypeDesc(oiio::TypeDesc::DOUBLE, oiio::TypeDesc::MATRIX33), 1, mp->iCamArr[rc].m));

    imageIO::writeImage(depthMapFileName, width, height, depthMap, imageIO::EImageQuality::LOSSLESS, metadata);
    imageIO::writeImage(simMapFileName, width, height, simMap);
}

bool DepthSimMap::loadRefine(std::string depthMapFileName, std::string simMapFileName)
{
    int width;
    int height;

    std::vector<float> depthMap;
    std::vector<float> simMap;

    imageIO::readImage(depthMapFileName, width, height, depthMap);
    imageIO::readImage(simMapFileName, width, height, simMap);

    for(int i = 0; i < dsm->size(); ++i)
    {
        (*dsm)[i].depth = depthMap.at(i);
        (*dsm)[i].sim = simMap.at(i);
    }

    return true;
}

bool DepthSimMap::loadRefine(std::string depthMapFileName, float defaultSim)
{
  int width;
  int height;

  std::vector<float> depthMap;

  imageIO::readImage(depthMapFileName, width, height, depthMap);

  for(int i = 0; i < dsm->size(); ++i)
  {
      (*dsm)[i].depth = depthMap.at(i);
      (*dsm)[i].sim = defaultSim;
  }

  return true;
}

Universe* DepthSimMap::segment(float alpha, int rc)
{
    printf("segmenting to connected components \n");

    StaticVector<Pixel>* edges = new StaticVector<Pixel>(w * h * 2);
    for(int y = 0; y < h - 1; y++)
    {
        for(int x = 0; x < w - 1; x++)
        {
            float depth = (*dsm)[y * w + x].depth;
            float depthr = (*dsm)[y * w + (x + 1)].depth;
            float depthd = (*dsm)[(y + 1) * w + x].depth;

            if(depth > 0.0f)
            {
                Point3d p =
                    mp->CArr[rc] +
                    (mp->iCamArr[rc] * Point2d((float)x * (scale * step), (float)y * (scale * step))).normalize() *
                        depth;
                float pixSize = alpha * mp->getCamPixelSize(p, rc);

                if((depthr > 0.0f) && (fabs(depth - depthr) < pixSize))
                {
                    edges->push_back(Pixel(y * w + x, y * w + (x + 1)));
                }
                if((depthd > 0.0f) && (fabs(depth - depthd) < pixSize))
                {
                    edges->push_back(Pixel(y * w + x, (y + 1) * w + x));
                }
            }
        }
    }

    // segments
    Universe* u = new Universe(w * h);
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

void DepthSimMap::removeSmallSegments(int minSegSize, float alpha, int rc)
{
    long tall = clock();

    Universe* u = DepthSimMap::segment(alpha, rc);

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

void DepthSimMap::cutout(const Pixel& LU, const Pixel& RD)
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

float DepthSimMap::getAngleBetwABandACdepth(int rc, const Pixel& cellA, float dA, const Pixel& cellB, float dB,
                                               const Pixel& cellC, float dC)
{
    Point3d pA =
        mp->CArr[rc] +
        (mp->iCamArr[rc] * Point2d((float)cellA.x * (scale * step), (float)cellA.y * (scale * step))).normalize() * dA;
    Point3d pB =
        mp->CArr[rc] +
        (mp->iCamArr[rc] * Point2d((float)cellB.x * (scale * step), (float)cellB.y * (scale * step))).normalize() * dB;
    Point3d pC =
        mp->CArr[rc] +
        (mp->iCamArr[rc] * Point2d((float)cellC.x * (scale * step), (float)cellC.y * (scale * step))).normalize() * dC;
    return 180.0f - angleBetwABandAC(pA, pB, pC);
}

float DepthSimMap::getCellSmoothEnergy(int rc, const int cellId, float defaultE)
{
    return getCellSmoothEnergy(rc, Pixel(cellId % w, cellId / w), defaultE);
}

float DepthSimMap::getCellSmoothEnergy(int rc, const Pixel& cell, float defaultE)
{
    if((cell.x <= 0) || (cell.x >= w - 1) || (cell.y <= 0) || (cell.y >= h - 1))
    {
        return defaultE;
    }

    Pixel cell0 = cell;
    Pixel cellL = cell0 + Pixel(0, -1);
    Pixel cellR = cell0 + Pixel(0, 1);
    Pixel cellU = cell0 + Pixel(-1, 0);
    Pixel cellB = cell0 + Pixel(1, 0);

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

float DepthSimMap::getASmoothStepBetwABandACdepth(int rc, const Pixel& cellA, float dA, const Pixel& cellB, float dB,
                                                     const Pixel& cellC, float dC)
{
    Point3d pA =
        mp->CArr[rc] +
        (mp->iCamArr[rc] * Point2d((float)cellA.x * (scale * step), (float)cellA.y * (scale * step))).normalize() * dA;
    Point3d pB =
        mp->CArr[rc] +
        (mp->iCamArr[rc] * Point2d((float)cellB.x * (scale * step), (float)cellB.y * (scale * step))).normalize() * dB;
    Point3d pC =
        mp->CArr[rc] +
        (mp->iCamArr[rc] * Point2d((float)cellC.x * (scale * step), (float)cellC.y * (scale * step))).normalize() * dC;

    Point3d vs = ((pB - pA) + (pC - pA)) / 4.0f;
    Point3d vcn = (mp->CArr[rc] - pA).normalize();

    Point3d A1 = pA + vs;
    Point3d pS = closestPointToLine3D(&A1, &pA, &vcn);

    return (mp->CArr[rc] - pS).size() - dA;
}

float DepthSimMap::getCellSmoothStep(int rc, const int cellId)
{
    return getCellSmoothStep(rc, Pixel(cellId % w, cellId / w));
}

float DepthSimMap::getCellSmoothStep(int rc, const Pixel& cell)
{
    if((cell.x <= 0) || (cell.x >= w - 1) || (cell.y <= 0) || (cell.y >= h - 1))
    {
        return 0.0f;
    }

    Pixel cell0 = cell;
    Pixel cellL = cell0 + Pixel(0, -1);
    Pixel cellR = cell0 + Pixel(0, 1);
    Pixel cellU = cell0 + Pixel(-1, 0);
    Pixel cellB = cell0 + Pixel(1, 0);

    float d0 = (*dsm)[cell0.y * w + cell0.x].depth;
    float dL = (*dsm)[cellL.y * w + cellL.x].depth;
    float dR = (*dsm)[cellR.y * w + cellR.x].depth;
    float dU = (*dsm)[cellU.y * w + cellU.x].depth;
    float dB = (*dsm)[cellB.y * w + cellB.x].depth;

    Point3d cg = Point3d(0.0f, 0.0f, 0.0f);
    float n = 0.0f;

    if(dL > 0.0f)
    {
        cg =
            cg +
            (mp->CArr[rc] +
             (mp->iCamArr[rc] * Point2d((float)cellL.x * (scale * step), (float)cellL.y * (scale * step))).normalize() *
                 dL);
        n += 1.0f;
    }
    if(dR > 0.0f)
    {
        cg =
            cg +
            (mp->CArr[rc] +
             (mp->iCamArr[rc] * Point2d((float)cellR.x * (scale * step), (float)cellR.y * (scale * step))).normalize() *
                 dR);
        n += 1.0f;
    }
    if(dU > 0.0f)
    {
        cg =
            cg +
            (mp->CArr[rc] +
             (mp->iCamArr[rc] * Point2d((float)cellU.x * (scale * step), (float)cellU.y * (scale * step))).normalize() *
                 dU);
        n += 1.0f;
    }
    if(dB > 0.0f)
    {
        cg =
            cg +
            (mp->CArr[rc] +
             (mp->iCamArr[rc] * Point2d((float)cellB.x * (scale * step), (float)cellB.y * (scale * step))).normalize() *
                 dB);
        n += 1.0f;
    }

    if((d0 > 0.0f) && (n > 1.0f))
    {
        cg = cg / n;
        Point3d p0 =
            mp->CArr[rc] +
            (mp->iCamArr[rc] * Point2d((float)cell0.x * (scale * step), (float)cell0.y * (scale * step))).normalize() *
                d0;
        Point3d vcn = (mp->CArr[rc] - p0).normalize();

        Point3d pS = closestPointToLine3D(&cg, &p0, &vcn);

        return (mp->CArr[rc] - pS).size() - d0;
    }

    return 0.0f;
}
