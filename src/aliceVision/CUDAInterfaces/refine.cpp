// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "refine.hpp"

#include <aliceVision/delaunaycut/mv_delaunay_helpers.hpp>
#include <aliceVision/mesh/mv_plyloader.hpp>
#include <aliceVision/structures/mv_geometry.hpp>
#include <aliceVision/common/ImagesCache.hpp>
#include <aliceVision/common/fileIO.hpp>
#include <aliceVision/imageIO/image.hpp>

#include <boost/filesystem.hpp>


namespace bfs = boost::filesystem;

staticVector<int>* reprojectTargetImageToTheReferenceGetVisTrisRcTc(int rc, int tc, mv_mesh* me, std::string tmpDir)
{
    std::string visTrisFileName = tmpDir + "visTris" + num2strFourDecimal(rc) + ".bin";
    staticVector<int>* visTrisRc = loadArrayFromFile<int>(visTrisFileName);

    // depthMapFileName = tmpDir + "depthMap"+num2strFourDecimal(tc)+".bin";
    // trisMapFileName = tmpDir + "trisMap"+num2strFourDecimal(tc)+".bin";
    // staticVector<int> *visTrisTc = me->getVisibleTrianglesIndexes(depthMapFileName, trisMapFileName, mp, tc, scale,
    // w, h);
    visTrisFileName = tmpDir + "visTris" + num2strFourDecimal(tc) + ".bin";
    staticVector<int>* visTrisTc = loadArrayFromFile<int>(visTrisFileName);

    staticVector<pixel>* visTrisRcTcB = new staticVector<pixel>(me->tris->size());
    visTrisRcTcB->resize_with(me->tris->size(), pixel(0, 0));

    for(int i = 0; i < visTrisRc->size(); i++)
    {
        (*visTrisRcTcB)[(*visTrisRc)[i]].x = 1;
    }
    for(int i = 0; i < visTrisTc->size(); i++)
    {
        (*visTrisRcTcB)[(*visTrisTc)[i]].y = 1;
    }

    int n = 0;
    for(int i = 0; i < visTrisRcTcB->size(); i++)
    {
        n += (((*visTrisRcTcB)[i].x == 1) && ((*visTrisRcTcB)[i].y == 1));
    }

    staticVector<int>* visTrisRcTc = new staticVector<int>(n);
    for(int i = 0; i < visTrisRcTcB->size(); i++)
    {
        if(((*visTrisRcTcB)[i].x == 1) && ((*visTrisRcTcB)[i].y == 1))
        {
            visTrisRcTc->push_back(i);
        }
    }
    delete visTrisRcTcB;
    delete visTrisRc;
    delete visTrisTc;

    return visTrisRcTc;
}

void reprojectTargetImageToTheReferenceRGB(mv_images_cache* ic, int rc, int tc, multiviewParams* mp, mv_mesh* me,
                                           int scale, int w, int h, std::string tmpDir)
{
    staticVector<int>* visTrisRcTc = reprojectTargetImageToTheReferenceGetVisTrisRcTc(rc, tc, me, tmpDir);

    std::vector<Color> img(mp->mip->getSize(rc), Color(-1.0f, -1.0f, -1.0f));
    std::vector<point2d> imgMap(mp->mip->getSize(rc), point2d(-1.0f, -1.0f));

    for(int i = 0; i < visTrisRcTc->size(); i++)
    {
        int triId = (*visTrisRcTc)[i];
        mv_mesh::triangle_proj tp = me->getTriangleProjection(triId, mp, rc, w, h);

        point3d n = me->computeTriangleNormal(triId);
        point3d p = me->computeTriangleCenterOfGravity(triId);
        matrix3x3 H;
        mp->computeHomographyInductedByPlaneRcTc(&H, p, n, rc, tc);

        point2d rpixp;
        mp->getPixelFor3DPoint(&rpixp, p, rc);
        point2d tpix = H ^ rpixp;
        if((mp->isPixelInImage(rpixp, rc)) && (mp->isPixelInImage(tpix, tc)))
        {
            img.at((int)rpixp.x * mp->mip->getHeight(rc) + (int)rpixp.y) = ic->getPixelValueInterpolated(&tpix, tc);
        }

        pixel pix;
        for(pix.x = (tp.lu.x - 1) * scale; pix.x <= (tp.rd.x + 1) * scale; pix.x++)
        {
            for(pix.y = (tp.lu.y - 1) * scale; pix.y <= (tp.rd.y + 1) * scale; pix.y++)
            {
                point2d pixx = point2d(pix.x, pix.y);
                point2d pixxs = point2d((float)pix.x / (float)scale, (float)pix.y / (float)scale);
                if(isPointInTriangle(tp.tp2ds[0], tp.tp2ds[1], tp.tp2ds[2], pixxs))
                {
                    tpix = H ^ pixx;
                    if((mp->isPixelInImage(pixx, rc)) && (mp->isPixelInImage(tpix, tc)))
                    {
                        img.at(pix.x * mp->mip->getHeight(rc) + pix.y) = ic->getPixelValueInterpolated(&tpix, tc);
                        imgMap.at(pix.x * mp->mip->getHeight(rc) + pix.y) = tpix;
                    }
                }
            }
        }
    }

    imageIO::writeImage("rctc.png", mp->mip->getWidth(rc), mp->mip->getHeight(rc), img);

    delete visTrisRcTc;
}

void reprojectTargetImageToTheReferenceRGB(multiviewParams* mp, std::string tmpDir, int rc,
                                           int tc)
{
    int w = mp->mip->getWidth(rc);
    int h = mp->mip->getHeight(rc);

    mv_mesh* me = new mv_mesh();
    me->loadFromBin(tmpDir + "mesh.bin");

    mv_images_cache* ic = new mv_images_cache(mp, mp->mip->_ini.get<int>("global.npreload", 100), 0);
    reprojectTargetImageToTheReferenceRGB(ic, rc, tc, mp, me, 1, w, h, tmpDir);
    delete ic;

    delete me;
}

staticVector<point3d>* loadFromPtsSamplesFromTheLevelFromFile(int sampleLevel, std::string fileName)
{
    FILE* f = fopen(fileName.c_str(), "rb");
    int n, k;
    fread(&n, sizeof(int), 1, f);
    fread(&k, sizeof(int), 1, f);
    staticVector<point3d>* out = new staticVector<point3d>(n);
    for(int i = 0; i < n; i++)
    {
        for(int j = -k; j <= k; j++)
        {
            point3d p;
            fread(&p, sizeof(point3d), 1, f);
            if(j == sampleLevel)
            {
                out->push_back(p);
            }
        }
    }
    fclose(f);
    return out;
}

void initPtsSamplesSimFile(int npts, int kHalf, std::string fileName)
{
    FILE* f = fopen(fileName.c_str(), "wb");
    fwrite(&npts, sizeof(int), 1, f);
    fwrite(&kHalf, sizeof(int), 1, f);
    for(int i = 0; i < npts; i++)
    {
        for(int j = -kHalf; j <= kHalf; j++)
        {
            float sim = 1.0f;
            fwrite(&sim, sizeof(float), 1, f);
        }
    }
    fclose(f);
}

void updatePtsSamplesSimLevel(staticVector<float>* samplesSim, int sampleLevel, std::string fileName)
{
    std::string fileNameTmp = fileName + "tmp";
    FILE* fi = fopen(fileName.c_str(), "rb");
    FILE* fo = fopen(fileNameTmp.c_str(), "wb");
    int n, k;
    fread(&n, sizeof(int), 1, fi);
    fread(&k, sizeof(int), 1, fi);
    fwrite(&n, sizeof(int), 1, fo);
    fwrite(&k, sizeof(int), 1, fo);
    for(int i = 0; i < n; i++)
    {
        for(int j = -k; j <= k; j++)
        {
            float sim;
            fread(&sim, sizeof(float), 1, fi);
            if(j == sampleLevel)
            {
                sim = (*samplesSim)[i];
            }
            fwrite(&sim, sizeof(float), 1, fo);
        }
    }
    fclose(fi);
    fclose(fo);

    remove(fileName.c_str());
    bfs::copy_file(fileNameTmp, fileName, bfs::copy_option::overwrite_if_exists);
    remove(fileNameTmp.c_str());
}

float triangle_area(point3d& pa, point3d& pb, point3d& pc)
{
    /*
    float a = (pb-pa).size();
    float b = (pc-pa).size();
    float c = (pc-pb).size();
    float p = (a+b+c)/2.0;

    return sqrt(p*(p-a)*(p-b)*(p-c));
    */
    point3d e1 = pb - pa;
    point3d e2 = pc - pa;
    return cross(e1, e2).size() / 2.0f;
}

void visualisePtsNms(std::string ptsFileName, std::string nmsFileName, multiviewParams* mp)
{
    staticVector<point3d>* pts = loadArrayFromFile<point3d>(ptsFileName);
    staticVector<point3d>* nms = loadArrayFromFile<point3d>(nmsFileName);

    mv_output3D* o3d = new mv_output3D(mp);
    std::string fileNameWrl = "ptsNms.wrl";
    o3d->create_wrl_pts_nms(pts, nms, mp, fileNameWrl);
    delete o3d;

    delete pts;
    delete nms;
}

void visualisePtsVisFile(std::string wrlPrefix, std::string ptsFileName, std::string ptsVisFileName,
                         multiviewParams* mp)
{
    // TODO visualise ptsVisFile
    for(int rc = 0; rc < mp->ncams; rc++)
    {
        FILE* fi1 = fopen(ptsVisFileName.c_str(), "rb");
        FILE* fi2 = fopen(ptsFileName.c_str(), "rb");
        int npts;
        fread(&npts, sizeof(int), 1, fi1);
        fread(&npts, sizeof(int), 1, fi2);

        staticVector<point3d>* pts = new staticVector<point3d>(npts);

        for(int i = 0; i < npts; i++)
        {
            point3d p;
            fread(&p, sizeof(float), 3, fi2);
            int ncams = 0;
            fread(&ncams, sizeof(int), 1, fi1);
            for(int j = 0; j < ncams; j++)
            {
                int rc1;
                fread(&rc1, sizeof(int), 1, fi1);
                if(rc1 == rc)
                {
                    pts->push_back(p);
                }
            }
        }

        fclose(fi1);
        fclose(fi2);

        mv_output3D* o3d = new mv_output3D(mp);
        std::string fileNameWrl = wrlPrefix + num2strFourDecimal(rc) + ".wrl";
        o3d->create_wrl_pts(pts, mp, fileNameWrl);
        delete o3d;

        delete pts;
    }
}

void visualiseTrisVisFile(std::string wrlPrefix, std::string trisVisFileName, multiviewParams* mp, mv_mesh* me)
{
    staticVector<staticVector<int>*>* trisCams = loadArrayOfArraysFromFile<int>(trisVisFileName);

    for(int rc = 0; rc < mp->ncams; rc++)
    {
        staticVector<int>* visTris = new staticVector<int>(trisCams->size());
        for(int i = 0; i < trisCams->size(); i++)
        {
            for(int j = 0; j < sizeOfStaticVector<int>((*trisCams)[i]); j++)
            {
                int tc = (*(*trisCams)[i])[j];
                if(rc == tc)
                {
                    visTris->push_back(i);
                }
            }
        }

        mv_mesh* me1 = me->generateMeshFromTrianglesSubset(*visTris, NULL);
        mv_output3D* o3d = new mv_output3D(mp);
        std::string fileNameWrl = wrlPrefix + num2strFourDecimal(rc) + ".wrl";
        o3d->saveMvMeshToWrl(me1, fileNameWrl);
        delete o3d;
        delete me1;

        delete visTris;
    }

    deleteArrayOfArrays<int>(&trisCams);
}

void initPtsVisFile(std::string fileName, int npts)
{
    FILE* fo = fopen(fileName.c_str(), "wb");
    fwrite(&npts, sizeof(int), 1, fo);
    for(int i = 0; i < npts; i++)
    {
        int ncams = 0;
        fwrite(&ncams, sizeof(int), 1, fo);
    }
    fclose(fo);
}

void updatePtsVisFile(std::string fileNameIn, std::string fileNameOut, staticVector<staticVector<int>*>* ptsVis)
{
    FILE* fi = fopen(fileNameIn.c_str(), "rb");
    FILE* fo = fopen(fileNameOut.c_str(), "wb");

    int npts;
    fread(&npts, sizeof(int), 1, fi);
    fwrite(&npts, sizeof(int), 1, fo);

    for(int i = 0; i < npts; i++)
    {
        int ncams = 0;
        fread(&ncams, sizeof(int), 1, fi);
        int ncams1 = ncams + sizeOfStaticVector<int>((*ptsVis)[i]);
        fwrite(&ncams1, sizeof(int), 1, fo);
        // read and write old
        for(int j = 0; j < ncams; j++)
        {
            int rc;
            fread(&rc, sizeof(int), 1, fi);
            fwrite(&rc, sizeof(int), 1, fo);
        }

        // write new
        for(int j = 0; j < sizeOfStaticVector<int>((*ptsVis)[i]); j++)
        {
            int rc = (*(*ptsVis)[i])[j];
            fwrite(&rc, sizeof(int), 1, fo);
        }
    }

    fclose(fo);
    fclose(fi);
}

bool isContourPixelInDepthMap(pixel pix, staticVector<float>* depthMap, int w, int h, float mindiscontinuity)
{
    float pixdepth = (*depthMap)[pix.x * h + pix.y];
    int d = 2;
    pixel npix;
    for(npix.x = std::max(0, pix.x - d); npix.x <= std::min(w - 1, pix.x + d); npix.x++)
    {
        for(npix.y = std::max(0, pix.y - d); npix.y <= std::min(h - 1, pix.y + d); npix.y++)
        {
            float npixdepth = (*depthMap)[npix.x * h + npix.y];
            if(pixdepth < npixdepth - mindiscontinuity)
            {
                return true;
            }
        }
    }
    return false;
}


void sortPtsCamsByRcNormAngle(staticVector<staticVector<int>*>* ptsCams, staticVector<point3d>* pts,
                              staticVector<point3d>* nms, multiviewParams* mp, int maxNCams)
{
    for(int i = 0; i < pts->size(); i++)
    {
        point3d p = (*pts)[i];
        point3d n = (*nms)[i].normalize();

        if((*ptsCams)[i] != NULL)
        {
            staticVector<sortedId>* cams = new staticVector<sortedId>(sizeOfStaticVector<int>((*ptsCams)[i]));
            for(int j = 0; j < sizeOfStaticVector<int>((*ptsCams)[i]); j++)
            {
                int rc = (*(*ptsCams)[i])[j];
                float ang = notOrientedangleBetwV1andV2((mp->CArr[rc] - p).normalize(), n);
                cams->push_back(sortedId(rc, ang));
            }
            qsort(&(*cams)[0], cams->size(), sizeof(sortedId), qsortCompareSortedIdAsc);
            (*ptsCams)[i]->resize(std::min(maxNCams, sizeOfStaticVector<int>((*ptsCams)[i])));
            for(int j = 0; j < sizeOfStaticVector<int>((*ptsCams)[i]); j++)
            {
                (*(*ptsCams)[i])[j] = (*cams)[j].id;
            }
            delete cams;
        }
    }
}

staticVector<int>* computeCamPairsHist(staticVector<point3d>* pts, staticVector<point3d>* nms,
                                       staticVector<staticVector<int>*>* ptsVis, multiviewParams* mp)
{
    staticVector<int>* camPairsHist = new staticVector<int>(mp->ncams * mp->ncams);
    camPairsHist->resize_with(mp->ncams * mp->ncams, 0);

    for(int i = 0; i < ptsVis->size(); i++)
    {
        if(sizeOfStaticVector<int>((*ptsVis)[i]) >= 2)
        {
            point3d nn = (*nms)[i];
            point3d pp = (*pts)[i];

            int n = sizeOfStaticVector<int>((*ptsVis)[i]);
            for(int j1 = 0; j1 < n; j1++)
            {
                int rc = (*(*ptsVis)[i])[j1];
                for(int j2 = j1 + 1; j2 < n; j2++)
                {
                    int tc = (*(*ptsVis)[i])[j2];
                    bool canBeVisibleInRc = notOrientedangleBetwV1andV2(nn, (mp->CArr[rc] - pp).normalize()) < 80.0f;
                    bool canBeVisibleInTc = notOrientedangleBetwV1andV2(nn, (mp->CArr[tc] - pp).normalize()) < 80.0f;
                    if((canBeVisibleInRc == true) && (canBeVisibleInTc == true))
                    {
                        (*camPairsHist)[std::min(rc, tc) * mp->ncams + std::max(rc, tc)]++;
                        //(*camPairsHist)[std::max(rc,tc)*mp->ncams+std::min(rc,tc)]++;
                    }
                }
            }
        }
    }

    staticVector<int>* camsh = new staticVector<int>(mp->ncams * mp->ncams);
    for(int rc = 0; rc < mp->ncams; rc++)
    {
        for(int tc = 0; tc < mp->ncams; tc++)
        {
            int id = rc * mp->ncams + tc;
            int val = (*camPairsHist)[id];
            if(val > (int)((float)pts->size() * 0.1f))
            {
                // if (val > 100) {
                camsh->push_back(id);
            }
        }
    }

    delete camPairsHist;

    return camsh;
}

staticVector<int>* computeCamPairsHistRcFirst(staticVector<point3d>* pts, staticVector<point3d>* nms,
                                              staticVector<staticVector<int>*>* ptsVis, multiviewParams* mp)
{
    staticVector<int>* camPairsHist = new staticVector<int>(mp->ncams * mp->ncams);
    camPairsHist->resize_with(mp->ncams * mp->ncams, 0);

    for(int i = 0; i < ptsVis->size(); i++)
    {
        if(sizeOfStaticVector<int>((*ptsVis)[i]) >= 2)
        {
            point3d nn = (*nms)[i];
            point3d pp = (*pts)[i];

            int n = sizeOfStaticVector<int>((*ptsVis)[i]);
            int rc = (*(*ptsVis)[i])[0];
            bool canBeVisibleInRc = notOrientedangleBetwV1andV2(nn, (mp->CArr[rc] - pp).normalize()) < 80.0f;

            for(int j2 = 1; j2 < n; j2++)
            {
                int tc = (*(*ptsVis)[i])[j2];
                bool canBeVisibleInTc = notOrientedangleBetwV1andV2(nn, (mp->CArr[tc] - pp).normalize()) < 80.0f;
                if((canBeVisibleInRc == true) && (canBeVisibleInTc == true))
                {
                    (*camPairsHist)[std::min(rc, tc) * mp->ncams + std::max(rc, tc)]++;
                }
            }
        }
    }

    staticVector<int>* camsh = new staticVector<int>(mp->ncams * mp->ncams);
    for(int rc = 0; rc < mp->ncams; rc++)
    {
        for(int tc = 0; tc < mp->ncams; tc++)
        {
            int id = rc * mp->ncams + tc;
            int val = (*camPairsHist)[id];
            if(val > (int)((float)pts->size() * 0.1f))
            {
                camsh->push_back(id);
            }
        }
    }

    delete camPairsHist;

    return camsh;
}



void filterPtsCamsByMinimalPixelSize(mv_mesh* me, staticVector<staticVector<int>*>* ptsCams, multiviewParams* mp)
{
    std::cout << "filterPtsCamsByMinimalPixelSize" << std::endl;

    for(int i = 0; i < me->pts->size(); i++)
    {
        point3d p = (*me->pts)[i];
        staticVector<int>* ptCams = (*ptsCams)[i];
        int n = sizeOfStaticVector<int>(ptCams);
        if(n > 0)
        {
            float minPixSize = std::numeric_limits<float>::max();
            for(int j = 0; j < n; j++)
            {
                float pixSize = mp->getCamPixelSize(p, (*ptCams)[j]);
                if(pixSize < minPixSize)
                {
                    minPixSize = pixSize;
                }
            }
            staticVector<int>* ptCamsNew = new staticVector<int>(n);
            for(int j = 0; j < n; j++)
            {
                float pixSize = mp->getCamPixelSize(p, (*ptCams)[j]);
                if(pixSize / minPixSize < 1.5f)
                {
                    ptCamsNew->push_back((*ptCams)[j]);
                }
            }
            if(ptCamsNew->size() > 0)
            {
                delete(*ptsCams)[i];
                (*ptsCams)[i] = ptCamsNew;
            }
            else
            {
                delete(*ptsCams)[i];
                (*ptsCams)[i] = NULL;
            }
        }
    }
    std::cout << "filterPtsCamsByMinimalPixelSize end" << std::endl;
}

float curvatureWeightFromSim(float sim)
{
    // sim = [-1:0.1:1]; weight = 1-exp(-10*((sim+1)/2)); plot(sim,weight,'r-');
    return (1.0f - exp(-20.0f * ((sim + 1.0f) / 2.0f)));
}

void nomalizeImages(std::string tmpDir, staticVector<int>* usedcams, staticVector<Color>* camsColScales,
                    staticVector<Color>* camsColShifts, multiviewParams* mp, mv_mesh* me,
                    staticVector<staticVector<int>*>* ptsCams)
{
    if(mp->verbose)
        printf("normalizing images\n");

    staticVector<staticVector<int>*>* camsPts = convertObjectsCamsToCamsObjects(mp, ptsCams);

    mv_images_cache* ic = new mv_images_cache(mp, mp->mip->_ini.get<int>("global.npreload", 100), 1);

    for(int iter = 0; iter < 10; iter++)
    {
        staticVector<Color>* ptsAvCols = new staticVector<Color>(me->pts->size());
        ptsAvCols->resize_with(me->pts->size(), Color(0.0f, 0.0f, 0.0f));
        staticVector<float>* nPtsAvCols = new staticVector<float>(me->pts->size());
        nPtsAvCols->resize_with(me->pts->size(), 0.0f);

        long t1 = initEstimate();
        for(int rc = 0; rc < mp->ncams; rc++)
        {
            for(int j = 0; j < sizeOfStaticVector<int>((*camsPts)[rc]); j++)
            {
                int idPt = (*(*camsPts)[rc])[j];
                point2d pix;
                mp->getPixelFor3DPoint(&pix, (*me->pts)[idPt], rc);
                if(mp->isPixelInImage(pix, rc) == true)
                {
                    Color ccc = ic->getPixelValueInterpolated(&pix, rc);
                    Color col;
                    col.r = ccc.r * (*camsColScales)[rc].r + (*camsColShifts)[rc].r;
                    col.g = ccc.g * (*camsColScales)[rc].g + (*camsColShifts)[rc].g;
                    col.b = ccc.b * (*camsColScales)[rc].b + (*camsColShifts)[rc].b;
                    col.r = std::min(255.0f, std::max(0.0f, col.r));
                    col.g = std::min(255.0f, std::max(0.0f, col.g));
                    col.b = std::min(255.0f, std::max(0.0f, col.b));
                    (*ptsAvCols)[idPt] = (*ptsAvCols)[idPt] + col;
                    (*nPtsAvCols)[idPt] += 1.0f;
                }
            }
            printfEstimate(rc, mp->ncams, t1);
        }
        finishEstimate();

        for(int i = 0; i < me->pts->size(); i++)
        {
            if((*nPtsAvCols)[i] > 0.0f)
            {
                (*ptsAvCols)[i] = (*ptsAvCols)[i] / (*nPtsAvCols)[i];
            }
        }

        delete nPtsAvCols;
        t1 = initEstimate();
        for(int rci = 0; rci < usedcams->size(); rci++)
        {
            int rc = (*usedcams)[rci];

            simStat ssr = simStat();
            simStat ssg = simStat();
            simStat ssb = simStat();
            Color averageColScale = Color(0.0f, 0.0f, 0.0f);
            Color averageColShift = Color(0.0f, 0.0f, 0.0f);
            // staticVector<color> *ptsAvColsDists = new staticVector<color>(me->pts->size());

            float nsamples = 0.0f;
            for(int j = 0; j < sizeOfStaticVector<int>((*camsPts)[rc]); j++)
            {
                int idPt = (*(*camsPts)[rc])[j];
                point2d pix;
                mp->getPixelFor3DPoint(&pix, (*me->pts)[idPt], rc);
                if(mp->isPixelInImage(pix, rc) == true)
                {
                    Color ccc = ic->getPixelValueInterpolated(&pix, rc);
                    Color col;
                    col.r = ccc.r * (*camsColScales)[rc].r + (*camsColShifts)[rc].r;
                    col.g = ccc.g * (*camsColScales)[rc].g + (*camsColShifts)[rc].g;
                    col.b = ccc.b * (*camsColScales)[rc].b + (*camsColShifts)[rc].b;
                    col.r = std::min(255.0f, std::max(0.0f, col.r));
                    col.g = std::min(255.0f, std::max(0.0f, col.g));
                    col.b = std::min(255.0f, std::max(0.0f, col.b));
                    // ptsAvColsDists->push_back((*ptsAvCols)[idPt]-col);

                    averageColShift = averageColShift + ((*ptsAvCols)[idPt] - col);

                    if(col.r > 0.001f)
                    {
                        averageColScale.r = averageColScale.r + ((*ptsAvCols)[idPt].r / col.r);
                    }
                    if(col.g > 0.001f)
                    {
                        averageColScale.g = averageColScale.g + ((*ptsAvCols)[idPt].g / col.g);
                    }
                    if(col.b > 0.001f)
                    {
                        averageColScale.b = averageColScale.b + ((*ptsAvCols)[idPt].b / col.b);
                    }

                    ssr.update((*ptsAvCols)[idPt].r, col.r);
                    ssg.update((*ptsAvCols)[idPt].g, col.g);
                    ssb.update((*ptsAvCols)[idPt].b, col.b);

                    nsamples += 1.0f;
                }
            }

            if(nsamples > 100.0f)
            {

                point2d n1, n2, cg;

                /*
                ssr.getEigenVectors(n1,n2);	cg = ssr.getCG(); k = n2.x/n2.y; k = std::min(std::max(0.8f,k),1.2f);
                l = cg.x-cg.y*k;
                (*camsColScales)[rc].x = std::min(std::max(0.8f,(*camsColScales)[rc].x*k),1.2f);
                (*camsColShifts)[rc].x = (*camsColShifts)[rc].x*k+l;

                ssg.getEigenVectors(n1,n2);	cg = ssg.getCG(); k = n2.x/n2.y; k = std::min(std::max(0.8f,k),1.2f);
                l = cg.x-cg.y*k;
                (*camsColScales)[rc].y = std::min(std::max(0.8f,(*camsColScales)[rc].y*k),1.2f);
                (*camsColShifts)[rc].y = (*camsColShifts)[rc].y*k+l;

                ssb.getEigenVectors(n1,n2);	cg = ssb.getCG(); k = n2.x/n2.y; k = std::min(std::max(0.8f,k),1.2f);
                l = cg.x-cg.y*k;
                (*camsColScales)[rc].z = std::min(std::max(0.8f,(*camsColScales)[rc].z*k),1.2f);
                (*camsColShifts)[rc].z = (*camsColShifts)[rc].z*k+l;
                */

                averageColScale = averageColScale / nsamples;
                averageColShift = averageColShift / nsamples;
                //(*camsColScales)[rc].x = (*camsColScales)[rc].x * averageColScale.x;
                //(*camsColScales)[rc].y = (*camsColScales)[rc].y * averageColScale.y;
                //(*camsColScales)[rc].z = (*camsColScales)[rc].z * averageColScale.z;
                (*camsColShifts)[rc] = (*camsColShifts)[rc] + averageColShift;
                if(rc == (*usedcams)[0])
                {
                    if(mp->verbose)
                        printf("k = (%f %f %f), l = (%f %f %f)\n", (*camsColScales)[rc].r, (*camsColScales)[rc].g,
                               (*camsColScales)[rc].b, (*camsColShifts)[rc].r, (*camsColShifts)[rc].g,
                               (*camsColShifts)[rc].b);
                }
            }

            /*
            if (ptsAvColsDists->size()>100)
            {
                    averageColScale = averageColScale / (float)ptsAvColsDists->size();
                    averageColShift = averageColShift / (float)ptsAvColsDists->size();
                    (*camsColShifts)[rc] = (*camsColShifts)[rc] + averageColShift;
            };
            delete ptsAvColsDists;
            */

            printfEstimate(rci, usedcams->size(), t1);
        }
        finishEstimate();

        if(iter == 9)
        {
            staticVector<rgb>* triColors = new staticVector<rgb>(me->tris->size());
            for(int i = 0; i < me->tris->size(); i++)
            {
                Color c = ((*ptsAvCols)[(*me->tris)[i].i[0]] + (*ptsAvCols)[(*me->tris)[i].i[1]] +
                     (*ptsAvCols)[(*me->tris)[i].i[2]]) /
                    3.0f;
                rgb cc;
                cc.r = (unsigned char)(std::max(0.0f, std::min(254.0f, c.r)));
                cc.g = (unsigned char)(std::max(0.0f, std::min(254.0f, c.g)));
                cc.b = (unsigned char)(std::max(0.0f, std::min(254.0f, c.b)));
                triColors->push_back(cc);
            }

            mv_output3D* o3d = new mv_output3D(mp);
            o3d->saveMvMeshToWrl(me, tmpDir + "meshAvImgColOpt.wrl", triColors);
            me->saveToPly(tmpDir + "meshAvImgColOpt.ply", triColors);
            delete o3d;
            delete triColors;
        }

        delete ptsAvCols;
    }

    delete ic;
    deleteArrayOfArrays<int>(&camsPts);
}

void nomalizeImagesMaps(std::string tmpDir, staticVector<int>* usedcams, multiviewParams* mp, mv_mesh* me,
                        staticVector<staticVector<int>*>* ptsCams)
{
    printf("normalizing images\n");

    staticVector<staticVector<int>*>* camsPts = convertObjectsCamsToCamsObjects(mp, ptsCams);

    mv_images_cache* ic = new mv_images_cache(mp, mp->mip->_ini.get<int>("global.npreload", 100), 1);

    for(int rc = 0; rc < mp->ncams; rc++)
    {
        int w = mp->mip->getWidth(rc) / 50;
        int h = mp->mip->getHeight(rc) / 50;
        staticVector<Color>* camColScalesMap = new staticVector<Color>(w * h);
        staticVector<Color>* camColShiftsMap = new staticVector<Color>(w * h);
        camColScalesMap->resize_with(w * h, Color(1.0f, 1.0f, 1.0f));
        camColShiftsMap->resize_with(w * h, Color(0.0f, 0.0f, 0.0f));
        saveArrayToFile<Color>(tmpDir + "colorScalesMap" + num2strFourDecimal(rc) + ".bin", camColScalesMap);
        saveArrayToFile<Color>(tmpDir + "colorShiftsMap" + num2strFourDecimal(rc) + ".bin", camColShiftsMap);
        delete camColScalesMap;
        delete camColShiftsMap;
    }

    for(int iter = 0; iter < 100; iter++)
    {
        staticVector<Color>* ptsAvCols = new staticVector<Color>(me->pts->size());
        ptsAvCols->resize_with(me->pts->size(), Color(0.0f, 0.0f, 0.0f));
        staticVector<float>* nPtsAvCols = new staticVector<float>(me->pts->size());
        nPtsAvCols->resize_with(me->pts->size(), 0.0f);

        // printf("1");

        long t1 = initEstimate();
        for(int rci = 0; rci < usedcams->size(); rci++)
        {
            int rc = (*usedcams)[rci];
            int w = mp->mip->getWidth(rc) / 50;
            int h = mp->mip->getHeight(rc) / 50;
            staticVector<Color>* camColScalesMap =
                loadArrayFromFile<Color>(tmpDir + "colorScalesMap" + num2strFourDecimal(rc) + ".bin");
            staticVector<Color>* camColShiftsMap =
                loadArrayFromFile<Color>(tmpDir + "colorShiftsMap" + num2strFourDecimal(rc) + ".bin");
            for(int j = 0; j < sizeOfStaticVector<int>((*camsPts)[rc]); j++)
            {
                int idPt = (*(*camsPts)[rc])[j];
                point2d pix;
                mp->getPixelFor3DPoint(&pix, (*me->pts)[idPt], rc);
                int x = (int)(pix.x) / 50;
                int y = (int)(pix.y) / 50;
                if((mp->isPixelInImage(pix, rc) == true) && (x < w) && (y < h))
                {
                    Color scale = (*camColScalesMap)[x * h + y];
                    Color shift = (*camColShiftsMap)[x * h + y];
                    Color ccc = ic->getPixelValueInterpolated(&pix, rc);
                    Color col;
                    col.r = ccc.r * scale.r + shift.r;
                    col.g = ccc.g * scale.g + shift.g;
                    col.b = ccc.b * scale.b + shift.b;
                    col.r = std::min(255.0f, std::max(0.0f, col.r));
                    col.g = std::min(255.0f, std::max(0.0f, col.g));
                    col.b = std::min(255.0f, std::max(0.0f, col.b));
                    (*ptsAvCols)[idPt] = (*ptsAvCols)[idPt] + col;
                    (*nPtsAvCols)[idPt] += 1.0f;
                }
            }
            delete camColScalesMap;
            delete camColShiftsMap;
            printfEstimate(rci, usedcams->size(), t1);
        }
        finishEstimate();

        // printf("2");

        for(int i = 0; i < me->pts->size(); i++)
        {
            if((*nPtsAvCols)[i] > 0.0f)
            {
                (*ptsAvCols)[i] = (*ptsAvCols)[i] / (*nPtsAvCols)[i];
            }
        }
        delete nPtsAvCols;

        t1 = initEstimate();
        for(int rci = 0; rci < usedcams->size(); rci++)
        {
            // printf("3");

            int rc = (*usedcams)[rci];
            int w = mp->mip->getWidth(rc) / 50;
            int h = mp->mip->getHeight(rc) / 50;
            staticVector<Color>* camColScalesMap =
                loadArrayFromFile<Color>(tmpDir + "colorScalesMap" + num2strFourDecimal(rc) + ".bin");
            staticVector<Color>* camColShiftsMap =
                loadArrayFromFile<Color>(tmpDir + "colorShiftsMap" + num2strFourDecimal(rc) + ".bin");
            staticVector<point2d>* camColTrnMapR = new staticVector<point2d>(w * h);
            staticVector<point2d>* camColTrnMapG = new staticVector<point2d>(w * h);
            staticVector<point2d>* camColTrnMapB = new staticVector<point2d>(w * h);
            camColTrnMapR->resize_with(w * h, point2d(0.0f, 0.0f));
            camColTrnMapG->resize_with(w * h, point2d(0.0f, 0.0f));
            camColTrnMapB->resize_with(w * h, point2d(0.0f, 0.0f));

            // printf("4");

            for(int j = 0; j < sizeOfStaticVector<int>((*camsPts)[rc]); j++)
            {
                int idPt = (*(*camsPts)[rc])[j];
                point2d pix;
                mp->getPixelFor3DPoint(&pix, (*me->pts)[idPt], rc);
                int x = (int)(pix.x) / 50;
                int y = (int)(pix.y) / 50;
                if((mp->isPixelInImage(pix, rc) == true) && (x < w) && (y < h))
                {
                    Color scale = (*camColScalesMap)[x * h + y];
                    Color shift = (*camColShiftsMap)[x * h + y];
                    Color ccc = ic->getPixelValueInterpolated(&pix, rc);
                    Color col;
                    col.r = ccc.r * scale.r + shift.r;
                    col.g = ccc.g * scale.g + shift.g;
                    col.b = ccc.b * scale.b + shift.b;
                    col.r = std::min(255.0f, std::max(0.0f, col.r));
                    col.g = std::min(255.0f, std::max(0.0f, col.g));
                    col.b = std::min(255.0f, std::max(0.0f, col.b));
                    (*camColTrnMapR)[x * h + y] =
                        (*camColTrnMapR)[x * h + y] + point2d((*ptsAvCols)[idPt].r - col.r, 1.0f);
                    (*camColTrnMapG)[x * h + y] =
                        (*camColTrnMapG)[x * h + y] + point2d((*ptsAvCols)[idPt].g - col.g, 1.0f);
                    (*camColTrnMapB)[x * h + y] =
                        (*camColTrnMapB)[x * h + y] + point2d((*ptsAvCols)[idPt].b - col.b, 1.0f);
                }
            }

            // printf("5");

            for(int id = 0; id < w * h; id++)
            {
                (*camColShiftsMap)[id].r = (*camColShiftsMap)[id].r + (*camColTrnMapR)[id].x / (*camColTrnMapR)[id].y;
                (*camColShiftsMap)[id].g = (*camColShiftsMap)[id].g + (*camColTrnMapG)[id].x / (*camColTrnMapG)[id].y;
                (*camColShiftsMap)[id].b = (*camColShiftsMap)[id].b + (*camColTrnMapB)[id].x / (*camColTrnMapB)[id].y;
            }

            saveArrayToFile<Color>(tmpDir + "colorScalesMap" + num2strFourDecimal(rc) + ".bin", camColScalesMap);
            saveArrayToFile<Color>(tmpDir + "colorShiftsMap" + num2strFourDecimal(rc) + ".bin", camColShiftsMap);
            delete camColScalesMap;
            delete camColShiftsMap;
            delete camColTrnMapR;
            delete camColTrnMapG;
            delete camColTrnMapB;

            // printf("11");

            printfEstimate(rci, usedcams->size(), t1);
        }
        finishEstimate();

        if(iter == 99)
        {
            staticVector<rgb>* triColors = new staticVector<rgb>(me->tris->size());
            for(int i = 0; i < me->tris->size(); i++)
            {
                Color c;
                c = ((*ptsAvCols)[(*me->tris)[i].i[0]] + (*ptsAvCols)[(*me->tris)[i].i[1]] +
                     (*ptsAvCols)[(*me->tris)[i].i[2]]) /
                    3.0f;
                rgb cc;
                cc.r = (unsigned char)(std::max(0.0f, std::min(254.0f, c.r)));
                cc.g = (unsigned char)(std::max(0.0f, std::min(254.0f, c.g)));
                cc.b = (unsigned char)(std::max(0.0f, std::min(254.0f, c.b)));
                triColors->push_back(cc);
            }

            mv_output3D* o3d = new mv_output3D(mp);
            o3d->saveMvMeshToWrl(me, tmpDir + "meshAvImgColOpt.wrl", triColors);
            me->saveToPly(tmpDir + "meshAvImgColOpt.ply", triColors);
            delete o3d;
            delete triColors;
        }

        delete ptsAvCols;
    }

    delete ic;
    deleteArrayOfArrays<int>(&camsPts);
}

void nomalizeImages1(staticVector<Color>** camsColScalesO, staticVector<Color>** camsColShiftsO,
                     multiviewParams* mp, mv_mesh* me, staticVector<staticVector<int>*>* ptsCams)
{
    mv_images_cache* ic = new mv_images_cache(mp, mp->mip->_ini.get<int>("global.npreload", 100), 1);

    staticVector<Color>* camsColScales = new staticVector<Color>(mp->ncams);
    staticVector<Color>* camsColShifts = new staticVector<Color>(mp->ncams);
    camsColScales->resize_with(mp->ncams, Color(1.0f, 1.0f, 1.0f));
    camsColShifts->resize_with(mp->ncams, Color(0.0f, 0.0f, 0.0f));

    for(int iter = 0; iter < 10; iter++)
    {

        long t1 = initEstimate();
        for(int rc = 0; rc < mp->ncams; rc++)
        {
            staticVector<int>* tcams = new staticVector<int>(mp->ncams);
            staticVector<Color>* ptsRcCols = new staticVector<Color>(me->pts->size());
            for(int i = 0; i < me->pts->size(); i++)
            {
                Color col(0.0f, 0.0f, 0.0f);

                bool ok = false;
                for(int j = 0; j < sizeOfStaticVector<int>((*ptsCams)[i]); j++)
                {
                    int tc = (*(*ptsCams)[i])[j];
                    if(tc == rc)
                    {
                        point2d pix;
                        mp->getPixelFor3DPoint(&pix, (*me->pts)[i], rc);
                        if(mp->isPixelInImage(pix, rc) == true)
                        {
                            ok = true;
                            Color ccc = ic->getPixelValueInterpolated(&pix, rc);
                            col.r = ccc.r * (*camsColScales)[rc].r + (*camsColShifts)[rc].r;
                            col.g = ccc.g * (*camsColScales)[rc].g + (*camsColShifts)[rc].g;
                            col.b = ccc.b * (*camsColScales)[rc].b + (*camsColShifts)[rc].b;
                            col.r = std::min(255.0f, std::max(0.0f, col.r));
                            col.g = std::min(255.0f, std::max(0.0f, col.g));
                            col.b = std::min(255.0f, std::max(0.0f, col.b));
                        }
                    }
                }
                ptsRcCols->push_back(col);
                if(ok == true)
                {
                    for(int j = 0; j < sizeOfStaticVector<int>((*ptsCams)[i]); j++)
                    {
                        int tc = (*(*ptsCams)[i])[j];
                        if(tc != rc)
                        {
                            tcams->push_back_distinct(tc);
                        }
                    }
                }
            }

            simStat sr = simStat();
            simStat sg = simStat();
            simStat sb = simStat();

            for(int c = 0; c < tcams->size(); c++)
            {
                int tc = (*tcams)[c];
                simStat ssr = simStat();
                simStat ssg = simStat();
                simStat ssb = simStat();

                // std::string fileName = mp->mip->newDir + "stat" + num2strFourDecimal(rc) + "_"+
                // num2strFourDecimal(tc) + "_" + num2str(iter) + ".txt";
                // FILE *fr = fopen(fileName.c_str(),"w");

                for(int i = 0; i < me->pts->size(); i++)
                {
                    bool ok = false;
                    for(int j = 0; j < sizeOfStaticVector<int>((*ptsCams)[i]); j++)
                    {
                        int cam = (*(*ptsCams)[i])[j];
                        if(cam == rc)
                        {
                            point2d pix;
                            mp->getPixelFor3DPoint(&pix, (*me->pts)[i], cam);
                            if(mp->isPixelInImage(pix, cam) == true)
                            {
                                ok = true;
                            }
                        }
                    }
                    if(ok == true)
                    {
                        for(int j = 0; j < sizeOfStaticVector<int>((*ptsCams)[i]); j++)
                        {
                            if(tc == (*(*ptsCams)[i])[j])
                            {
                                point2d pix;
                                mp->getPixelFor3DPoint(&pix, (*me->pts)[i], tc);
                                if(mp->isPixelInImage(pix, tc) == true)
                                {
                                    Color ccc = ic->getPixelValueInterpolated(&pix, tc);
                                    Color col;
                                    col.r = ccc.r * (*camsColScales)[tc].r + (*camsColShifts)[tc].r;
                                    col.g = ccc.g * (*camsColScales)[tc].g + (*camsColShifts)[tc].g;
                                    col.b = ccc.b * (*camsColScales)[tc].b + (*camsColShifts)[tc].b;
                                    col.r = std::min(255.0f, std::max(0.0f, col.r));
                                    col.g = std::min(255.0f, std::max(0.0f, col.g));
                                    col.b = std::min(255.0f, std::max(0.0f, col.b));

                                    ssr.update((*ptsRcCols)[i].r, col.r);
                                    ssg.update((*ptsRcCols)[i].g, col.g);
                                    ssb.update((*ptsRcCols)[i].b, col.b);

                                    // fprintf(fr,"%f %f %f %f %f %f\n",(*ptsRcCols)[i].x,col.x,
                                    //	                                                         (*ptsRcCols)[i].y,col.y,
                                    //								 (*ptsRcCols)[i].z,col.z);
                                }
                            }
                        }
                    }
                }

                // fclose(fr);

                if(ssr.count > 1000.0f)
                {
                    point2d n1, n2, cg, lpt;
                    ssr.getEigenVectors(n1, n2);
                    cg = ssr.getCG();
                    lpt = cg + n2 * 10.0f;
                    float scaler = (lpt.y - cg.y) / (lpt.x - cg.x);
                    float shiftr = cg.y - cg.x * scaler;

                    ssg.getEigenVectors(n1, n2);
                    cg = ssg.getCG();
                    lpt = cg + n2 * 10.0f;
                    float scaleg = (lpt.y - cg.y) / (lpt.x - cg.x);
                    float shiftg = cg.y - cg.x * scaleg;

                    ssb.getEigenVectors(n1, n2);
                    cg = ssb.getCG();
                    lpt = cg + n2 * 10.0f;
                    float scaleb = (lpt.y - cg.y) / (lpt.x - cg.x);
                    float shiftb = cg.y - cg.x * scaleb;

                    if((fabs(shiftr) < 40.0f) && (fabs(shiftg) < 40.0f) && (fabs(shiftb) < 40.0f))
                    {
                        sr.update(scaler, shiftr);
                        sg.update(scaleg, shiftg);
                        sb.update(scaleb, shiftb);
                    }

                    if(rc == 0)
                    {
                        printf("scale %f %f %f, shift %f %f %f\n", scaler, scaleg, scaleb, shiftr, shiftg, shiftb);
                    }
                }
            }

            delete tcams;
            delete ptsRcCols;

            (*camsColScales)[rc].r = (*camsColScales)[rc].r * (float)(sr.xsum / sr.count);
            (*camsColScales)[rc].g = (*camsColScales)[rc].g * (float)(sg.xsum / sg.count);
            (*camsColScales)[rc].b = (*camsColScales)[rc].b * (float)(sb.xsum / sb.count);
            (*camsColShifts)[rc] =
                (*camsColShifts)[rc] +
                Color((float)(sr.ysum / sr.count), (float)(sg.ysum / sg.count), (float)(sb.ysum / sb.count));

            if(rc == 0)
            {
                printf("scale %f %f %f, shift %f %f %f\n", (*camsColScales)[rc].r, (*camsColScales)[rc].g,
                       (*camsColScales)[rc].b, (*camsColShifts)[rc].r, (*camsColShifts)[rc].g, (*camsColShifts)[rc].b);
            }

            printfEstimate(rc, mp->ncams, t1);
        }
        finishEstimate();
    }

    delete ic;

    (*camsColScalesO) = camsColScales;
    (*camsColShiftsO) = camsColShifts;
}

void saveMeshToWrlTrisColored(multiviewParams* mp, mv_mesh* me, staticVector<staticVector<int>*>* ptsCams,
                              std::string tmpDir)
{
    if(mp->verbose)
        printf("computing colors \n");

    mv_images_cache* ic = new mv_images_cache(mp, mp->mip->_ini.get<int>("global.npreload", 100), 0);

    staticVector<Color>* ptsAvCols = new staticVector<Color>(me->pts->size());
    ptsAvCols->resize_with(me->pts->size(), Color(0.0f, 0.0f, 0.0f));
    staticVector<float>* nPtsAvCols = new staticVector<float>(me->pts->size());
    nPtsAvCols->resize_with(me->pts->size(), 0.0f);

    staticVector<staticVector<int>*>* camsPts = convertObjectsCamsToCamsObjects(mp, ptsCams);
    long t1 = initEstimate();
    for(int rc = 0; rc < mp->ncams; rc++)
    {
        for(int j = 0; j < sizeOfStaticVector<int>((*camsPts)[rc]); j++)
        {
            int idPt = (*(*camsPts)[rc])[j];
            point2d pix;
            mp->getPixelFor3DPoint(&pix, (*me->pts)[idPt], rc);
            if(mp->isPixelInImage(pix, rc) == true)
            {
                Color col = ic->getPixelValueInterpolated(&pix, rc);
                (*ptsAvCols)[idPt] = (*ptsAvCols)[idPt] + col;
                (*nPtsAvCols)[idPt] += 1.0f;
            }
        }
        printfEstimate(rc, mp->ncams, t1);
    }
    finishEstimate();

    for(int i = 0; i < me->pts->size(); i++)
    {
        if((*nPtsAvCols)[i] > 0.0f)
        {
            (*ptsAvCols)[i] = (*ptsAvCols)[i] / (*nPtsAvCols)[i];
        }
    }

    delete nPtsAvCols;
    deleteArrayOfArrays<int>(&camsPts);
    /*
    //colour pts of the mesh
    long t1=initEstimate();
    for (int i=0;i<me->pts->size();i++) {
            color avcol = color(0.0f,0.0f,0.0f);
            float  navcol = 0.0f;
            for (int j=0;j<sizeOfStaticVector<int>((*ptsCams)[i]); j++) {
                    int rc = (*(*ptsCams)[i])[j];
                    point2d pix; mp->getPixelFor3DPoint(&pix, (*me->pts)[i], rc);
                    if (mp->isPixelInImage(pix)==true) {
                            color col = ic->getPixelValueInterpolated(&pix,rc);
                            avcol=avcol+col;
                            navcol+=1.0f;
                    };
            };
            avcol=avcol/navcol;
            ptsAvCols->push_back(avcol);

            printfEstimate(i, me->pts->size(), t1);
    };
    finishEstimate();
    */

    staticVector<rgb>* triColors = new staticVector<rgb>(me->tris->size());
    for(int i = 0; i < me->tris->size(); i++)
    {
        Color c;
        c = ((*ptsAvCols)[(*me->tris)[i].i[0]] + (*ptsAvCols)[(*me->tris)[i].i[1]] +
             (*ptsAvCols)[(*me->tris)[i].i[2]]) /
            3.0f;
        rgb cc;
        cc.r = (unsigned char)(std::max(0.0f, std::min(254.0f, c.r)));
        cc.g = (unsigned char)(std::max(0.0f, std::min(254.0f, c.g)));
        cc.b = (unsigned char)(std::max(0.0f, std::min(254.0f, c.b)));
        triColors->push_back(cc);
    }

    if(mp->verbose)
        printf("saving colored mesh to wrl \n");

    mv_output3D* o3d = new mv_output3D(mp);
    o3d->saveMvMeshToWrl(me, tmpDir + "meshAvImgCol.wrl", triColors);
    me->saveToPly(tmpDir + "meshAvImgCol.ply", triColors);
    delete o3d;

    delete ptsAvCols;
    delete ic;
    delete triColors;
}

void saveMeshToWrlTrisColoredGrowSeed(staticVector<int>* ptsRcs, int seedPtId, multiviewParams* mp, mv_mesh* me,
                                      staticVector<staticVector<int>*>* ptsCams)
{
    // get nearest rc
    int rc = -1;
    float md = std::numeric_limits<float>::max();

    for(int j = 0; j < sizeOfStaticVector<int>((*ptsCams)[seedPtId]); j++)
    {
        int cam = (*(*ptsCams)[seedPtId])[j];
        float d = ((*me->pts)[seedPtId] - mp->CArr[cam]).size();
        if(d < md)
        {
            rc = cam;
            md = d;
        }
    }

    printf("rc: %i\n", rc);

    // mark rc
    staticVectorBool* rcPts = new staticVectorBool(me->pts->size());
    rcPts->resize_with(me->pts->size(), false);
    for(int i = 0; i < ptsCams->size(); i++)
    {
        for(int j = 0; j < sizeOfStaticVector<int>((*ptsCams)[i]); j++)
        {
            int cam = (*(*ptsCams)[i])[j];
            if(cam == rc)
            {
                (*rcPts)[i] = true;
            }
        }
    }

    // grow nearest RC
    staticVector<staticVector<int>*>* ptsNeighPts = me->getPtsNeighPtsOrdered();
    staticVector<int>* toProcess = new staticVector<int>(me->pts->size());
    toProcess->push_back(seedPtId);

    while(toProcess->size() > 0)
    {
        int ptId = toProcess->pop();
        (*ptsRcs)[ptId] = rc;
        if(sizeOfStaticVector<int>((*ptsNeighPts)[ptId]) > 0)
        {
            for(int j = 0; j < sizeOfStaticVector<int>((*ptsNeighPts)[ptId]); j++)
            {
                int nptId = (*(*ptsNeighPts)[ptId])[j];
                if(((*ptsRcs)[nptId] == -1) && ((*rcPts)[nptId] == true))
                {
                    toProcess->push_back(nptId);
                }
            }
        }
    }
    delete toProcess;
    deleteArrayOfArrays<int>(&ptsNeighPts);

    delete rcPts;
}

void saveMeshToWrlTrisColoredGrow(multiviewParams* mp, mv_mesh* me, staticVector<staticVector<int>*>* ptsCams,
                                  std::string tmpDir)
{
    printf("growing best RCs \n");

    mv_images_cache* ic = new mv_images_cache(mp, mp->mip->_ini.get<int>("global.npreload", 100), 1);

    staticVector<int>* ptsRcs = new staticVector<int>(me->pts->size());
    ptsRcs->resize_with(me->pts->size(), -1);

    long t1 = initEstimate();
    // for (int i=0;i<me->pts->size();i++) {
    for(int i = 0; i < 1; i++)
    {
        if((*ptsRcs)[i] == -1)
        {
            saveMeshToWrlTrisColoredGrowSeed(ptsRcs, i, mp, me, ptsCams);
        }
        printfEstimate(i, me->pts->size(), t1);
    }
    finishEstimate();

    staticVector<Color>* ptsAvCols = new staticVector<Color>(me->pts->size());
    ptsAvCols->resize_with(me->pts->size(), Color(0.0f, 0.0f, 0.0f));

    t1 = initEstimate();
    for(int rc = 0; rc < mp->ncams; rc++)
    {
        int nptsRc = 0;
        for(int i = 0; i < me->pts->size(); i++)
        {
            if((*ptsRcs)[i] == rc)
            {
                point2d pix;
                mp->getPixelFor3DPoint(&pix, (*me->pts)[i], rc);
                if(mp->isPixelInImage(pix, rc) == true)
                {
                    Color col = ic->getPixelValueInterpolated(&pix, rc);
                    (*ptsAvCols)[i] = col;
                    nptsRc++;
                }
            }
        }
        printf("%i %i of %i\n", rc, nptsRc, me->pts->size());
        printfEstimate(rc, mp->ncams, t1);
    }
    finishEstimate();

    staticVector<rgb>* triColors = new staticVector<rgb>(me->tris->size());
    for(int i = 0; i < me->tris->size(); i++)
    {
        Color c;
        c = ((*ptsAvCols)[(*me->tris)[i].i[0]] + (*ptsAvCols)[(*me->tris)[i].i[1]] +
             (*ptsAvCols)[(*me->tris)[i].i[2]]) /
            3.0f;
        rgb cc;
        cc.r = (unsigned char)(std::max(0.0f, std::min(254.0f, c.r)));
        cc.g = (unsigned char)(std::max(0.0f, std::min(254.0f, c.g)));
        cc.b = (unsigned char)(std::max(0.0f, std::min(254.0f, c.b)));
        triColors->push_back(cc);
    }

    printf("saving colored mesh to wrl \n");

    mv_output3D* o3d = new mv_output3D(mp);
    o3d->saveMvMeshToWrl(me, tmpDir + "meshAvImgColGrow.wrl", triColors);
    me->saveToPly(tmpDir + "meshAvImgColGrow.ply", triColors);
    delete o3d;

    delete ptsRcs;
    delete ptsAvCols;
    delete ic;
    delete triColors;
}

void saveMvMeshToWrlPly(int scale, staticVector<int>* usedcams, multiviewParams* mp, mv_mesh* me, std::string tmpDir,
                        std::string fileNameWrl, std::string fileNamePly, staticVector<staticVector<int>*>* trisCams,
                        staticVector<staticVector<int>*>* ptsCams, bool doNormalizeInmages)
{
    mv_output3D* o3d = new mv_output3D(mp);

    staticVector<int>* rcTris = new staticVector<int>(me->tris->size());
    rcTris->resize_with(me->tris->size(), -1);

    long t1 = initEstimate();
    for(int idTri = 0; idTri < me->tris->size(); idTri++)
    {
        staticVector<int>* cams = (*trisCams)[idTri];
        for(int c = 0; c < sizeOfStaticVector<int>(cams); c++)
        {

            int rc = (*cams)[c];
            int rcTri = (*rcTris)[idTri];
            point3d p = me->computeTriangleCenterOfGravity(idTri);

            double dNew = (mp->CArr[rc] - p).size();
            double dOld = dNew + 1.0f;
            if(rcTri >= 0)
            {
                dOld = (mp->CArr[rcTri] - p).size();
            }

            point2d pixa;
            mp->getPixelFor3DPoint(&pixa, (*me->pts)[(*me->tris)[idTri].i[0]], rc);
            point2d pixb;
            mp->getPixelFor3DPoint(&pixb, (*me->pts)[(*me->tris)[idTri].i[1]], rc);
            point2d pixc;
            mp->getPixelFor3DPoint(&pixc, (*me->pts)[(*me->tris)[idTri].i[2]], rc);

            if((dNew < dOld) && (mp->isPixelInImage(pixa, rc) == true) && (mp->isPixelInImage(pixb, rc) == true) &&
               (mp->isPixelInImage(pixc, rc) == true))
            {
                (*rcTris)[idTri] = rc;
            }

            /*
            int rc = (*cams)[c];
            int rcTri = (*rcTris)[idTri];
            point3d p = me->computeTriangleCenterOfGravity(idTri);

            point3d v1 = (p-mp->CArr[rc]).normalize();
            point3d v2 = me->computeTriangleNormal(idTri);
            float angleNew = notOrientedangleBetwV1andV2(v1,v2);

            float dNew = (mp->CArr[rc]-p).size();
            float dOld = dNew+1.0f;
            if (rcTri>=0) {
                    dOld = (mp->CArr[rcTri]-p).size();
            };
            if ((dNew<dOld)
                    &&(angleNew<80)
                    &&(me->isTriangleVisibleInCam(idTri,mp,rc)==true)
                    )
            {
                    (*rcTris)[idTri]=rc;
            };
            */
        }
        printfEstimate(idTri, me->tris->size(), t1);
    }
    finishEstimate();

    for(int i = 0; i < rcTris->size(); i++)
    {
        if((*rcTris)[i] < 0)
        {
            (*rcTris)[i] = 0;
        }
    }

    staticVector<Color>* camsColScales = new staticVector<Color>(mp->ncams);
    staticVector<Color>* camsColShifts = new staticVector<Color>(mp->ncams);
    camsColScales->resize_with(mp->ncams, Color(1.0f, 1.0f, 1.0f));
    camsColShifts->resize_with(mp->ncams, Color(0.0f, 0.0f, 0.0f));
    if(doNormalizeInmages == true)
    {
        nomalizeImages(tmpDir, usedcams, camsColScales, camsColShifts, mp, me, ptsCams);
    }
    saveArrayToFile<Color>(tmpDir + "camsColorScales.bin", camsColScales);
    saveArrayToFile<Color>(tmpDir + "camsColorShifts.bin", camsColShifts);
    // nomalizeImagesMaps(tmpDir, usedcams, mp, me, ptsCams);

    o3d->saveMvMeshToWrl(scale, me, camsColScales, camsColShifts, rcTris, mp, tmpDir + fileNameWrl, tmpDir, 0);
    me->saveToPly(tmpDir + fileNamePly);

    delete rcTris;
    delete camsColScales;
    delete camsColShifts;

    delete o3d;
}
