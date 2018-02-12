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
