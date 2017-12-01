// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "mv_mesh.hpp"

#include <aliceVision/rply/mv_plyloader.hpp>
#include <aliceVision/rply/mv_plysaver.hpp>
#include <aliceVision/structures/mv_geometry.hpp>
#include <aliceVision/structures/quaternion.hpp>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <boost/filesystem.hpp>

#include <fstream>
#include <map>


namespace bfs = boost::filesystem;

mv_mesh::mv_mesh()
{
}

mv_mesh::~mv_mesh()
{
    delete pts;
    delete tris;
}

bool mv_mesh::loadFromPly(std::string plyFileName)
{
    return mv_loadply(plyFileName, this);
}

void mv_mesh::saveToPly(std::string plyFileName)
{
    mv_saveply(plyFileName, this);
}

void mv_mesh::saveToPly(std::string plyFileName, staticVector<rgb>* triColors)
{
    mv_saveply(plyFileName, this, triColors);
}

bool mv_mesh::loadFromBin(std::string binFileName)
{
    FILE* f = fopen(binFileName.c_str(), "rb");

    if(f == nullptr)
    {
        return false;
    }

    int npts;
    fread(&npts, sizeof(int), 1, f);
    pts = new staticVector<point3d>(npts);
    pts->resize(npts);
    fread(&(*pts)[0], sizeof(point3d), npts, f);
    int ntris;
    fread(&ntris, sizeof(int), 1, f);
    tris = new staticVector<mv_mesh::triangle>(ntris);
    tris->resize(ntris);
    fread(&(*tris)[0], sizeof(mv_mesh::triangle), ntris, f);
    fclose(f);

    return true;
}

bool mv_mesh::loadFromTxt(std::string txtFileName)
{
    FILE* fin = fopen(txtFileName.c_str(), "r");

    if(fin == nullptr)
    {
        printf("file %s does not exists!!!\n", txtFileName.c_str());
        return false;
    }

    int npnts;
    fscanf(fin, "%i\n", &npnts); // warning ... here will be 3
    fscanf(fin, "%i\n", &npnts);
    pts = new staticVector<point3d>(npnts);
    for(int i = 0; i < npnts; i++)
    {
        double x, y, z;
        fscanf(fin, "%lf %lf %lf\n", &x, &y, &z);
        pts->push_back(point3d(x, y, z));
    }

    int ntris;
    fscanf(fin, "%i\n", &ntris);

    tris = new staticVector<mv_mesh::triangle>(ntris);

    for(int i = 0; i < ntris; i++)
    {
        int x, y, z;
        fscanf(fin, "%i %i %i\n", &x, &y, &z);

        mv_mesh::triangle t;
        t.i[0] = x;
        t.i[1] = y;
        t.i[2] = z;
        t.alive = true;

        tris->push_back(t);
    }

    fclose(fin);

    return true;
}

void mv_mesh::saveToBin(std::string binFileName)
{
    long t = std::clock();
    std::cout << "save mesh to bin." << std::endl;
    // printf("open\n");
    FILE* f = fopen(binFileName.c_str(), "wb");
    int npts = pts->size();
    // printf("write npts %i\n",npts);
    fwrite(&npts, sizeof(int), 1, f);
    // printf("write pts\n");
    fwrite(&(*pts)[0], sizeof(point3d), npts, f);
    int ntris = tris->size();
    // printf("write ntris %i\n",ntris);
    fwrite(&ntris, sizeof(int), 1, f);
    // printf("write tris\n");
    fwrite(&(*tris)[0], sizeof(mv_mesh::triangle), ntris, f);
    // printf("close\n");
    fclose(f);
    // printf("done\n");
    printfElapsedTime(t, "Save mesh to bin ");
}

void mv_mesh::addMesh(mv_mesh* me)
{
    int npts = sizeOfStaticVector<point3d>(pts);
    int ntris = sizeOfStaticVector<mv_mesh::triangle>(tris);
    int npts1 = sizeOfStaticVector<point3d>(me->pts);
    int ntris1 = sizeOfStaticVector<mv_mesh::triangle>(me->tris);

    //	printf("pts needed %i of %i allocated\n",npts+npts1,pts->reserved());
    //	printf("tris needed %i of %i allocated\n",ntris+ntris1,tris->reserved());

    if((pts != nullptr) && (tris != nullptr) && (npts + npts1 <= pts->capacity()) &&
       (ntris + ntris1 <= tris->capacity()))
    {
        for(int i = 0; i < npts1; i++)
        {
            pts->push_back((*me->pts)[i]);
        }
        for(int i = 0; i < ntris1; i++)
        {
            mv_mesh::triangle t = (*me->tris)[i];
            if((t.i[0] >= 0) && (t.i[0] < npts1) && (t.i[1] >= 0) && (t.i[1] < npts1) && (t.i[2] >= 0) &&
               (t.i[2] < npts1))
            {
                t.i[0] += npts;
                t.i[1] += npts;
                t.i[2] += npts;
                tris->push_back(t);
            }
            else
            {
                printf("WARNING BAD TRIANGLE INDEX %i %i %i, npts : %i \n", t.i[0], t.i[1], t.i[2], npts1);
            }
        }
    }
    else
    {
        staticVector<point3d>* ptsnew = new staticVector<point3d>(npts + npts1);
        staticVector<mv_mesh::triangle>* trisnew = new staticVector<mv_mesh::triangle>(ntris + ntris1);

        for(int i = 0; i < npts; i++)
        {
            ptsnew->push_back((*pts)[i]);
        }
        for(int i = 0; i < npts1; i++)
        {
            ptsnew->push_back((*me->pts)[i]);
        }

        for(int i = 0; i < ntris; i++)
        {
            trisnew->push_back((*tris)[i]);
        }
        for(int i = 0; i < ntris1; i++)
        {
            mv_mesh::triangle t = (*me->tris)[i];
            if((t.i[0] >= 0) && (t.i[0] < npts1) && (t.i[1] >= 0) && (t.i[1] < npts1) && (t.i[2] >= 0) &&
               (t.i[2] < npts1))
            {
                t.i[0] += npts;
                t.i[1] += npts;
                t.i[2] += npts;
                trisnew->push_back(t);
            }
            else
            {
                printf("WARNING BAD TRIANGLE INDEX %i %i %i, npts : %i \n", t.i[0], t.i[1], t.i[2], npts1);
            }
        }

        if(pts != nullptr)
        {
            delete pts;
        }
        if(tris != nullptr)
        {
            delete tris;
        }
        pts = ptsnew;
        tris = trisnew;
    }
}

mv_mesh::triangle_proj mv_mesh::getTriangleProjection(int triid, const multiviewParams* mp, int rc, int w, int h) const
{
    int ow = mp->mip->getWidth(rc);
    int oh = mp->mip->getHeight(rc);

    triangle_proj tp;
    for(int j = 0; j < 3; j++)
    {
        mp->getPixelFor3DPoint(&tp.tp2ds[j], (*pts)[(*tris)[triid].i[j]], rc);
        tp.tp2ds[j].x = (tp.tp2ds[j].x / (float)ow) * (float)w;
        tp.tp2ds[j].y = (tp.tp2ds[j].y / (float)oh) * (float)h;
        tp.tpixs[j].x = (int)floor(tp.tp2ds[j].x);
        tp.tpixs[j].y = (int)floor(tp.tp2ds[j].y);
    }

    tp.lu.x = w;
    tp.lu.y = h;
    tp.rd.x = 0;
    tp.rd.y = 0;
    for(int j = 0; j < 3; j++)
    {
        if((float)tp.lu.x > tp.tp2ds[j].x)
        {
            tp.lu.x = (int)tp.tp2ds[j].x;
        }
        if((float)tp.lu.y > tp.tp2ds[j].y)
        {
            tp.lu.y = (int)tp.tp2ds[j].y;
        }
        if((float)tp.rd.x < tp.tp2ds[j].x)
        {
            tp.rd.x = (int)tp.tp2ds[j].x;
        }
        if((float)tp.rd.y < tp.tp2ds[j].y)
        {
            tp.rd.y = (int)tp.tp2ds[j].y;
        }
    }

    return tp;
}

bool mv_mesh::isTriangleInFrontOfCam(int triid, const multiviewParams* mp, int rc) const
{
    for(int j = 0; j < 3; j++)
    {
        if(!mp->is3DPointInFrontOfCam(&(*pts)[(*tris)[triid].i[j]], rc))
        {
            return false;
        }
    }
    return true;
}

bool mv_mesh::isTriangleVisibleInCam(int triid, const multiviewParams* mp, int rc) const
{
    for(int j = 0; j < 3; j++)
    {
        orientedPoint op;
        op.p = (*pts)[(*tris)[triid].i[0]];
        op.n = cross(((*pts)[(*tris)[triid].i[1]] - (*pts)[(*tris)[triid].i[0]]).normalize(),
                     ((*pts)[(*tris)[triid].i[2]] - (*pts)[(*tris)[triid].i[0]]).normalize());

        if(!isVisibleInCamera(mp, &op, rc))
        {
            return false;
        }
    }
    return true;
}

bool mv_mesh::isTriangleProjectionInImage(mv_mesh::triangle_proj tp, int w, int h) const
{
    for(int j = 0; j < 3; j++)
    {
        if(!((tp.tpixs[j].x > 0) && (tp.tpixs[j].x < w) && (tp.tpixs[j].y > 0) && (tp.tpixs[j].y < h)))
        {
            return false;
        }
    }
    return true;
}

void mv_mesh::updateStatMaps(staticVector<float>* depthMap, staticVector<float>* sigmaMap, pixel lu, pixel rd,
                             const multiviewParams* mp, int rc, int gridSize)
{
    int w = mp->mip->getWidth(rc);
    int h = mp->mip->getHeight(rc);

    staticVector<staticVector<int>*>* tmp = getTrisMap(mp, rc, 1, w, h);

    long tstart = clock();
    printf("updateStatMaps \n");
    int allpixs = (rd.x - lu.x) * (rd.y - lu.y);

    long t1 = initEstimate();
    pixel pix;
    for(pix.x = lu.x; pix.x < rd.x; pix.x++)
    {
        for(pix.y = lu.y; pix.y < rd.y; pix.y++)
        {

            staticVector<int>* ti = (*tmp)[pix.x * h + pix.y];
            if(ti != nullptr)
            {
                // stat3d s3d = stat3d();
                double xxsum = 0.0;
                double xsum = 0.0;
                double count = 0.0;
                int num = 0;

                for(int gi = 0; gi <= gridSize; gi++)
                {
                    for(int gj = 0; gj <= gridSize; gj++)
                    {
                        point2d p;
                        p.x = (float)pix.x + (float)gi / (float)gridSize;
                        p.y = (float)pix.y + (float)gi / (float)gridSize;

                        float mindepth = 10000000.0;
                        point3d minlpi;
                        for(int i = 0; i < ti->size(); i++)
                        {
                            int idTri = (*ti)[i];
                            orientedPoint tri;
                            tri.p = (*pts)[(*tris)[idTri].i[0]];
                            tri.n = cross(((*pts)[(*tris)[idTri].i[1]] - (*pts)[(*tris)[idTri].i[0]]).normalize(),
                                          ((*pts)[(*tris)[idTri].i[2]] - (*pts)[(*tris)[idTri].i[0]]).normalize());

                            triangle_proj tp = getTriangleProjection((*ti)[i], mp, rc, w, h);
                            if(isPointInTriangle(tp.tp2ds[0], tp.tp2ds[1], tp.tp2ds[2], p))
                            {
                                point3d lpi =
                                    linePlaneIntersect(mp->CArr[rc], (mp->iCamArr[rc] * p).normalize(), tri.p, tri.n);
                                float depth = (mp->CArr[rc] - lpi).size();
                                if(depth < mindepth)
                                {
                                    mindepth = depth;
                                    minlpi = lpi;
                                }
                            }
                        }

                        if(mindepth < 10000000.0)
                        {
                            // s3d.update(&minlpi);
                            xxsum += (double)mindepth * (double)mindepth;
                            xsum += (double)mindepth;
                            count += 1.0;
                            num++;
                        }
                    }
                }

                // if (s3d.count > 0)
                if(num > 0)
                {
                    /*
                    point3d cg;
                    cg.x = s3d.xsum / (float)s3d.count;
                    cg.y = s3d.ysum / (float)s3d.count;
                    cg.z = s3d.zsum / (float)s3d.count;

                    (*depthMap)[pix.x*h+pix.y] = (mp->CArr[rc]-cg).size();
                    (*sigmaMap)[pix.x*h+pix.y] = (mp->CArr[rc]-cg).size();
                    */

                    // mean or average
                    (*depthMap)[pix.x * h + pix.y] = (float)(xsum / count);

                    // variance
                    (*sigmaMap)[pix.x * h + pix.y] = (float)(xxsum / count - (xsum * xsum) / (count * count));
                }
            } // ti!=NULL

            printfEstimate(pix.x * (rd.y - lu.y) + pix.y, allpixs, t1);
        } // for pix.y
    }     // for pix.x
    finishEstimate();

    // deallocate
    deleteArrayOfArrays<int>(&tmp);

    printfElapsedTime(tstart);
}

staticVector<point2d>* mv_mesh::getTrianglePixelIntersectionsAndInternalPoints(mv_mesh::triangle_proj* tp,
                                                                               mv_mesh::rectangle* re)
{
    staticVector<point2d>* out = new staticVector<point2d>(20);

    if(isPointInTriangle(tp->tp2ds[0], tp->tp2ds[1], tp->tp2ds[2], re->P[0]))
    {
        out->push_back(re->P[0]);
    }
    if(isPointInTriangle(tp->tp2ds[0], tp->tp2ds[1], tp->tp2ds[2], re->P[1]))
    {
        out->push_back(re->P[1]);
    }
    if(isPointInTriangle(tp->tp2ds[0], tp->tp2ds[1], tp->tp2ds[2], re->P[2]))
    {
        out->push_back(re->P[2]);
    }
    if(isPointInTriangle(tp->tp2ds[0], tp->tp2ds[1], tp->tp2ds[2], re->P[3]))
    {
        out->push_back(re->P[3]);
    }
    if((isPointInTriangle(re->P[0], re->P[1], re->P[2], tp->tp2ds[0])) ||
       (isPointInTriangle(re->P[2], re->P[3], re->P[0], tp->tp2ds[0])))
    {
        out->push_back(tp->tp2ds[0]);
    }
    if((isPointInTriangle(re->P[0], re->P[1], re->P[2], tp->tp2ds[1])) ||
       (isPointInTriangle(re->P[2], re->P[3], re->P[0], tp->tp2ds[1])))
    {
        out->push_back(tp->tp2ds[1]);
    }
    if((isPointInTriangle(re->P[0], re->P[1], re->P[2], tp->tp2ds[2])) ||
       (isPointInTriangle(re->P[2], re->P[3], re->P[0], tp->tp2ds[2])))
    {
        out->push_back(tp->tp2ds[2]);
    }

    point2d lli;
    if(lineSegmentsIntersect2DTest(&lli, &tp->tp2ds[0], &tp->tp2ds[1], &re->P[0], &re->P[1]))
    {
        out->push_back(lli);
    }
    if(lineSegmentsIntersect2DTest(&lli, &tp->tp2ds[1], &tp->tp2ds[2], &re->P[0], &re->P[1]))
    {
        out->push_back(lli);
    }
    if(lineSegmentsIntersect2DTest(&lli, &tp->tp2ds[2], &tp->tp2ds[0], &re->P[0], &re->P[1]))
    {
        out->push_back(lli);
    }

    if(lineSegmentsIntersect2DTest(&lli, &tp->tp2ds[0], &tp->tp2ds[1], &re->P[1], &re->P[2]))
    {
        out->push_back(lli);
    }
    if(lineSegmentsIntersect2DTest(&lli, &tp->tp2ds[1], &tp->tp2ds[2], &re->P[1], &re->P[2]))
    {
        out->push_back(lli);
    }
    if(lineSegmentsIntersect2DTest(&lli, &tp->tp2ds[2], &tp->tp2ds[0], &re->P[1], &re->P[2]))
    {
        out->push_back(lli);
    }

    if(lineSegmentsIntersect2DTest(&lli, &tp->tp2ds[0], &tp->tp2ds[1], &re->P[2], &re->P[3]))
    {
        out->push_back(lli);
    }
    if(lineSegmentsIntersect2DTest(&lli, &tp->tp2ds[1], &tp->tp2ds[2], &re->P[2], &re->P[3]))
    {
        out->push_back(lli);
    }
    if(lineSegmentsIntersect2DTest(&lli, &tp->tp2ds[2], &tp->tp2ds[0], &re->P[2], &re->P[3]))
    {
        out->push_back(lli);
    }

    if(lineSegmentsIntersect2DTest(&lli, &tp->tp2ds[0], &tp->tp2ds[1], &re->P[3], &re->P[0]))
    {
        out->push_back(lli);
    }
    if(lineSegmentsIntersect2DTest(&lli, &tp->tp2ds[1], &tp->tp2ds[2], &re->P[3], &re->P[0]))
    {
        out->push_back(lli);
    }
    if(lineSegmentsIntersect2DTest(&lli, &tp->tp2ds[2], &tp->tp2ds[0], &re->P[3], &re->P[0]))
    {
        out->push_back(lli);
    }

    return out;
}

staticVector<point3d>* mv_mesh::getTrianglePixelIntersectionsAndInternalPoints(const multiviewParams* mp, int idTri,
                                                                               pixel&  /*pix*/, int rc,
                                                                               mv_mesh::triangle_proj* tp,
                                                                               mv_mesh::rectangle* re)
{

    point3d A = (*pts)[(*tris)[idTri].i[0]];
    point3d B = (*pts)[(*tris)[idTri].i[1]];
    point3d C = (*pts)[(*tris)[idTri].i[2]];

    staticVector<point3d>* out = triangleRectangleIntersection(A, B, C, mp, rc, re->P);

    if((isPointInTriangle(re->P[0], re->P[1], re->P[2], tp->tp2ds[0])) ||
       (isPointInTriangle(re->P[2], re->P[3], re->P[0], tp->tp2ds[0])))
    {
        out->push_back(A);
    }
    if((isPointInTriangle(re->P[0], re->P[1], re->P[2], tp->tp2ds[1])) ||
       (isPointInTriangle(re->P[2], re->P[3], re->P[0], tp->tp2ds[1])))
    {
        out->push_back(B);
    }
    if((isPointInTriangle(re->P[0], re->P[1], re->P[2], tp->tp2ds[2])) ||
       (isPointInTriangle(re->P[2], re->P[3], re->P[0], tp->tp2ds[2])))
    {
        out->push_back(C);
    }

    return out;
}

point2d mv_mesh::getTrianglePixelInternalPoint(mv_mesh::triangle_proj* tp, mv_mesh::rectangle* re)
{

    if((isPointInTriangle(re->P[0], re->P[1], re->P[2], tp->tp2ds[0])) ||
       (isPointInTriangle(re->P[2], re->P[3], re->P[0], tp->tp2ds[0])))
    {
        return tp->tp2ds[0];
    }
    if((isPointInTriangle(re->P[0], re->P[1], re->P[2], tp->tp2ds[1])) ||
       (isPointInTriangle(re->P[2], re->P[3], re->P[0], tp->tp2ds[1])))
    {
        return tp->tp2ds[1];
    }
    if((isPointInTriangle(re->P[0], re->P[1], re->P[2], tp->tp2ds[2])) ||
       (isPointInTriangle(re->P[2], re->P[3], re->P[0], tp->tp2ds[2])))
    {
        return tp->tp2ds[2];
    }

    if(isPointInTriangle(tp->tp2ds[0], tp->tp2ds[1], tp->tp2ds[2], re->P[0]))
    {
        return re->P[0];
    }
    if(isPointInTriangle(tp->tp2ds[0], tp->tp2ds[1], tp->tp2ds[2], re->P[1]))
    {
        return re->P[1];
    }
    if(isPointInTriangle(tp->tp2ds[0], tp->tp2ds[1], tp->tp2ds[2], re->P[2]))
    {
        return re->P[2];
    }
    if(isPointInTriangle(tp->tp2ds[0], tp->tp2ds[1], tp->tp2ds[2], re->P[3]))
    {
        return re->P[3];
    }
}

bool mv_mesh::doesTriangleIntersectsRectangle(mv_mesh::triangle_proj* tp, mv_mesh::rectangle* re)
{

    point2d p1[3];
    point2d p2[3];
    p1[0] = re->P[0];
    p1[1] = re->P[1];
    p1[2] = re->P[2];
    p2[0] = re->P[2];
    p2[1] = re->P[3];
    p2[2] = re->P[0];

    return ((TrianglesOverlap(tp->tp2ds, p1)) || (TrianglesOverlap(tp->tp2ds, p2)) ||
            (isPointInTriangle(p1[0], p1[1], p1[2], tp->tp2ds[0])) ||
            (isPointInTriangle(p1[0], p1[1], p1[2], tp->tp2ds[1])) ||
            (isPointInTriangle(p1[0], p1[1], p1[2], tp->tp2ds[2])) ||
            (isPointInTriangle(p2[0], p2[1], p2[2], tp->tp2ds[0])) ||
            (isPointInTriangle(p2[0], p2[1], p2[2], tp->tp2ds[1])) ||
            (isPointInTriangle(p2[0], p2[1], p2[2], tp->tp2ds[2])));

    /*
            point2d p;
            point2d *_p = &p;

            if ( isPointInTriangle(&re->P[0], &re->P[1], &re->P[2], &tp->tp2ds[0]) ||
                 isPointInTriangle(&re->P[1], &re->P[2], &re->P[0], &tp->tp2ds[0]) ||
                     isPointInTriangle(&re->P[0], &re->P[1], &re->P[2], &tp->tp2ds[1]) ||
                 isPointInTriangle(&re->P[1], &re->P[2], &re->P[0], &tp->tp2ds[1]) ||
                     isPointInTriangle(&re->P[0], &re->P[1], &re->P[2], &tp->tp2ds[2]) ||
                 isPointInTriangle(&re->P[1], &re->P[2], &re->P[0], &tp->tp2ds[2]) )
            {
                    *_p = (tp->tp2ds[0]+tp->tp2ds[1]+tp->tp2ds[2])/3.0;
                    return true;
            };

            point2d PS = (re->P[0]+re->P[1]+re->P[2]+re->P[3])/4.0;

            if (isPointInTriangle(&tp->tp2ds[0], &tp->tp2ds[1], &tp->tp2ds[2], &PS) == true)
            {
                    *_p = PS;
                    return true;
            };

            if (isPointInTriangle(&tp->tp2ds[0], &tp->tp2ds[1], &tp->tp2ds[2], &re->P[0]) == true)
            {
                    *_p = tp->tp2ds[0];
                    return true;
            };

            if (isPointInTriangle(&tp->tp2ds[0], &tp->tp2ds[1], &tp->tp2ds[2], &re->P[1]) == true)
            {
                    *_p = tp->tp2ds[1];
                    return true;
            };

            if (isPointInTriangle(&tp->tp2ds[0], &tp->tp2ds[1], &tp->tp2ds[2], &re->P[2]) == true)
            {
                    *_p = tp->tp2ds[2];
                    return true;
            };

            if (isPointInTriangle(&tp->tp2ds[0], &tp->tp2ds[1], &tp->tp2ds[2], &re->P[3]) == true)
            {
                    *_p = tp->tp2ds[3];
                    return true;
            };


            if ( lineSegmentsIntersect2DTest(&tp->tp2ds[0],&tp->tp2ds[1],&re->P[0],&re->P[1]) ||
                     lineSegmentsIntersect2DTest(&tp->tp2ds[0],&tp->tp2ds[1],&re->P[1],&re->P[2]) ||
                     lineSegmentsIntersect2DTest(&tp->tp2ds[0],&tp->tp2ds[1],&re->P[2],&re->P[3]) ||
                     lineSegmentsIntersect2DTest(&tp->tp2ds[0],&tp->tp2ds[1],&re->P[3],&re->P[0]) ||
                     lineSegmentsIntersect2DTest(&tp->tp2ds[1],&tp->tp2ds[2],&re->P[0],&re->P[1]) ||
                     lineSegmentsIntersect2DTest(&tp->tp2ds[1],&tp->tp2ds[2],&re->P[1],&re->P[2]) ||
                     lineSegmentsIntersect2DTest(&tp->tp2ds[1],&tp->tp2ds[2],&re->P[2],&re->P[3]) ||
                     lineSegmentsIntersect2DTest(&tp->tp2ds[1],&tp->tp2ds[2],&re->P[3],&re->P[0]) ||
                     lineSegmentsIntersect2DTest(&tp->tp2ds[2],&tp->tp2ds[0],&re->P[0],&re->P[1]) ||
                     lineSegmentsIntersect2DTest(&tp->tp2ds[2],&tp->tp2ds[0],&re->P[1],&re->P[2]) ||
                     lineSegmentsIntersect2DTest(&tp->tp2ds[2],&tp->tp2ds[0],&re->P[2],&re->P[3]) ||
                     lineSegmentsIntersect2DTest(&tp->tp2ds[2],&tp->tp2ds[0],&re->P[3],&re->P[0]) )
            {
                    return true;
            };


            return false;
    */
}

staticVector<staticVector<int>*>* mv_mesh::getPtsNeighTris()
{
    staticVector<voxel>* tirsPtsIds = new staticVector<voxel>(tris->size() * 3);
    for(int i = 0; i < tris->size(); i++)
    {
        tirsPtsIds->push_back(voxel((*tris)[i].i[0], i, 0));
        tirsPtsIds->push_back(voxel((*tris)[i].i[1], i, 0));
        tirsPtsIds->push_back(voxel((*tris)[i].i[2], i, 0));
    }
    qsort(&(*tirsPtsIds)[0], tirsPtsIds->size(), sizeof(voxel), qSortCompareVoxelByXAsc);

    int i = 0;
    int j = 0;
    int k = 0;
    int firstid = 0;
    while(i < tirsPtsIds->size())
    {
        k++;
        (*tirsPtsIds)[i].z = j;
        if((i == tirsPtsIds->size() - 1) || ((*tirsPtsIds)[i].x != (*tirsPtsIds)[i + 1].x))
        {
            (*tirsPtsIds)[firstid].z = k;
            j++;
            firstid = i + 1;
            k = 0;
        }
        i++;
    }
    int npts = j;

    staticVector<staticVector<int>*>* ptsNeighTris = new staticVector<staticVector<int>*>(pts->size());
    ptsNeighTris->resize_with(pts->size(), nullptr);

    i = 0;
    for(j = 0; j < npts; j++)
    {
        int middlePtId = (*tirsPtsIds)[i].x;
        int numincidenttris = (*tirsPtsIds)[i].z;
        int i0 = i;
        int i1 = i + numincidenttris;
        i = i1;

        staticVector<int>* triTmp = new staticVector<int>(numincidenttris);
        for(int l = i0; l < i1; l++)
        {
            triTmp->push_back((*tirsPtsIds)[l].y);
        }

        (*ptsNeighTris)[middlePtId] = triTmp;
    }

    delete tirsPtsIds;

    return ptsNeighTris;
}

staticVector<staticVector<int>*>* mv_mesh::getPtsNeighPtsOrdered()
{
    staticVector<staticVector<int>*>* ptsNeighTris = getPtsNeighTris();

    staticVector<staticVector<int>*>* ptsNeighPts = new staticVector<staticVector<int>*>(pts->size());
    ptsNeighPts->resize_with(pts->size(), nullptr);

    for(int i = 0; i < pts->size(); i++)
    {
        int middlePtId = i;
        staticVector<int>* triTmp = (*ptsNeighTris)[i];
        if((triTmp != nullptr) && (triTmp->size() > 0))
        {
            staticVector<int>* vhid = new staticVector<int>(triTmp->size() * 2);
            int acttriptid = (*tris)[(*triTmp)[0]].i[0];
            int firsttriptid = acttriptid;
            vhid->push_back(acttriptid);

            bool isThereTWithActtriptid = true;
            while((triTmp->size() > 0) && (isThereTWithActtriptid))
            {
                isThereTWithActtriptid = false;

                // find tri with middlePtId and acttriptid and get remainnig point id
                int l1 = 0;
                while((l1 < triTmp->size()) && (!isThereTWithActtriptid))
                {
                    bool okmiddlePtId = false;
                    bool okacttriptid = false;
                    int remptid = -1; // remaining pt id
                    for(int k = 0; k < 3; k++)
                    {
                        int triptid = (*tris)[(*triTmp)[l1]].i[k];
                        float len = ((*pts)[middlePtId] - (*pts)[triptid]).size();
                        if((triptid != middlePtId) && (triptid != acttriptid) && (len > 0.0) && (!(len != len)) &&
                           (!std::isnan(len)))
                        {
                            remptid = triptid;
                        }
                        if(triptid == middlePtId)
                        {
                            okmiddlePtId = true;
                        }
                        if(triptid == acttriptid)
                        {
                            okacttriptid = true;
                        }
                    }

                    if((okmiddlePtId) && (okacttriptid) && (remptid > -1))
                    {
                        acttriptid = remptid;
                        triTmp->remove(l1);
                        vhid->push_back(acttriptid);
                        isThereTWithActtriptid = true;
                    }
                    l1++;
                }
            } // while (triTmp->size()>0)

            // if ((acttriptid != firsttriptid)||(vhid->size()<3)) {
            //	delete vhid;
            //}else{
            if(vhid->size() == 0)
            {
                delete vhid;
            }
            else
            {
                if(acttriptid == firsttriptid)
                {
                    vhid->pop(); // remove last ... which ist first
                }

                // remove duplicities
                staticVector<int>* vhid1 = new staticVector<int>(vhid->size());
                for(int k1 = 0; k1 < vhid->size(); k1++)
                {
                    if(vhid1->indexOf((*vhid)[k1]) == -1)
                    {
                        vhid1->push_back((*vhid)[k1]);
                    }
                }
                delete vhid;

                (*ptsNeighPts)[middlePtId] = vhid1;

                /*
                FILE *fm=fopen("fans.m","w");

                fprintf(fm,"clear all \n close all \n clc; \n figure; \n hold on \n axis equal \n");

                for (int l=0;l<vhid->size();l++) {
                        point3d p = (*pts)[middlePtId];
                        point3d np = (*pts)[(*vhid)[l]];
                        fprintf(fm,"plot3([%f %f],[%f %f],[%f %f],'r-');\n",np.x,p.x,np.y,p.y,np.z,p.z);
                        fprintf(fm,"text(%f,%f,%f,'%i');\n",np.x,np.y,np.z,l);
                };
                for (int l=0;l<vhid->size()-1;l++) {
                        point3d p = (*pts)[(*vhid)[l]];
                        point3d np = (*pts)[(*vhid)[l+1]];
                        fprintf(fm,"plot3([%f %f],[%f %f],[%f %f],'b-');\n",np.x,p.x,np.y,p.y,np.z,p.z);
                };
                point3d p = (*pts)[(*vhid)[0]];
                point3d np = (*pts)[(*vhid)[vhid->size()-1]];
                fprintf(fm,"plot3([%f %f],[%f %f],[%f %f],'b-');\n",np.x,p.x,np.y,p.y,np.z,p.z);

                fclose(fm);
                */
            }
        }
    }

    deleteArrayOfArrays<int>(&ptsNeighTris);

    return ptsNeighPts;
}

staticVector<staticVector<int>*>* mv_mesh::getTrisMap(const multiviewParams* mp, int rc, int  /*scale*/, int w, int h)
{
    long tstart = clock();
    printf("getTrisMap \n");
    staticVector<int>* nmap = new staticVector<int>(w * h);
    nmap->resize_with(w * h, 0);

    printf("estimating numbers \n");
    long t1 = initEstimate();
    for(int i = 0; i < tris->size(); i++)
    {
        triangle_proj tp = getTriangleProjection(i, mp, rc, w, h);
        if((isTriangleProjectionInImage(tp, w, h))
           //&&(isTriangleInFrontOfCam(i, mp, rc) == true)
           //&&(isTriangleVisibleInCam(i, mp, rc)==true)
           )
        {
            pixel pix;
            for(pix.x = tp.lu.x; pix.x <= tp.rd.x; pix.x++)
            {
                for(pix.y = tp.lu.y; pix.y <= tp.rd.y; pix.y++)
                {
                    mv_mesh::rectangle re = mv_mesh::rectangle(pix, 1);
                    if(doesTriangleIntersectsRectangle(&tp, &re))
                    {
                        (*nmap)[pix.x * h + pix.y] += 1;
                    }
                } // for y
            }     // for x
        }         // isthere
        printfEstimate(i, tris->size(), t1);
    } // for i ntris
    finishEstimate();

    // allocate
    printf("allocating\n");
    staticVector<staticVector<int>*>* tmp = new staticVector<staticVector<int>*>(w * h);
    tmp->resize_with(w * h, nullptr);
    staticVector<int>** ptmp = &(*tmp)[0];
    for(int i = 0; i < w * h; i++)
    {
        if((*nmap)[i] > 0)
        {
            *ptmp = new staticVector<int>((*nmap)[i]);
        }
        ptmp++;
    }
    delete nmap;

    // fill
    printf("filling\n");
    t1 = initEstimate();
    for(int i = 0; i < tris->size(); i++)
    {
        triangle_proj tp = getTriangleProjection(i, mp, rc, w, h);
        if((isTriangleProjectionInImage(tp, w, h))
           //&&(isTriangleInFrontOfCam(i, mp, rc) == true)
           //&&(isTriangleVisibleInCam(i, mp, rc)==true)
           )
        {
            pixel pix;
            for(pix.x = tp.lu.x; pix.x <= tp.rd.x; pix.x++)
            {
                for(pix.y = tp.lu.y; pix.y <= tp.rd.y; pix.y++)
                {
                    mv_mesh::rectangle re = mv_mesh::rectangle(pix, 1);
                    if(doesTriangleIntersectsRectangle(&tp, &re))
                    {
                        (*tmp)[pix.x * h + pix.y]->push_back(i);
                    }
                } // for y
            }     // for x
        }         // isthere
        printfEstimate(i, tris->size(), t1);
    } // for i ntris
    finishEstimate();

    printfElapsedTime(tstart);

    return tmp;
}

staticVector<staticVector<int>*>* mv_mesh::getTrisMap(staticVector<int>* visTris, const multiviewParams* mp, int rc,
                                                      int  /*scale*/, int w, int h)
{
    long tstart = clock();
    printf("getTrisMap \n");
    staticVector<int>* nmap = new staticVector<int>(w * h);
    nmap->resize_with(w * h, 0);

    printf("estimating numbers \n");
    long t1 = initEstimate();
    for(int m = 0; m < visTris->size(); m++)
    {
        int i = (*visTris)[m];
        triangle_proj tp = getTriangleProjection(i, mp, rc, w, h);
        if((isTriangleProjectionInImage(tp, w, h))
           //&&(isTriangleInFrontOfCam(i, mp, rc) == true)
           //&&(isTriangleVisibleInCam(i, mp, rc)==true)
           )
        {
            pixel pix;
            for(pix.x = tp.lu.x; pix.x <= tp.rd.x; pix.x++)
            {
                for(pix.y = tp.lu.y; pix.y <= tp.rd.y; pix.y++)
                {
                    mv_mesh::rectangle re = mv_mesh::rectangle(pix, 1);
                    if(doesTriangleIntersectsRectangle(&tp, &re))
                    {
                        (*nmap)[pix.x * h + pix.y] += 1;
                    }
                } // for y
            }     // for x
        }         // isthere
        printfEstimate(i, tris->size(), t1);
    } // for i ntris
    finishEstimate();

    // allocate
    printf("allocating\n");
    staticVector<staticVector<int>*>* tmp = new staticVector<staticVector<int>*>(w * h);
    tmp->resize_with(w * h, nullptr);
    staticVector<int>** ptmp = &(*tmp)[0];
    for(int i = 0; i < w * h; i++)
    {
        if((*nmap)[i] > 0)
        {
            *ptmp = new staticVector<int>((*nmap)[i]);
        }
        ptmp++;
    }
    delete nmap;

    // fill
    printf("filling\n");
    t1 = initEstimate();
    for(int m = 0; m < visTris->size(); m++)
    {
        int i = (*visTris)[m];
        triangle_proj tp = getTriangleProjection(i, mp, rc, w, h);
        if((isTriangleProjectionInImage(tp, w, h))
           //&&(isTriangleInFrontOfCam(i, mp, rc) == true)
           //&&(isTriangleVisibleInCam(i, mp, rc)==true)
           )
        {
            pixel pix;
            for(pix.x = tp.lu.x; pix.x <= tp.rd.x; pix.x++)
            {
                for(pix.y = tp.lu.y; pix.y <= tp.rd.y; pix.y++)
                {
                    mv_mesh::rectangle re = mv_mesh::rectangle(pix, 1);
                    if(doesTriangleIntersectsRectangle(&tp, &re))
                    {
                        (*tmp)[pix.x * h + pix.y]->push_back(i);
                    }
                } // for y
            }     // for x
        }         // isthere
        printfEstimate(i, tris->size(), t1);
    } // for i ntris
    finishEstimate();

    printfElapsedTime(tstart);

    return tmp;
}

void mv_mesh::getDepthMap(staticVector<float>* depthMap, const multiviewParams* mp, int rc, int scale, int w, int h)
{
    staticVector<staticVector<int>*>* tmp = getTrisMap(mp, rc, scale, w, h);
    getDepthMap(depthMap, tmp, mp, rc, scale, w, h);
    deleteArrayOfArrays<int>(&tmp);
}

void mv_mesh::getDepthMap(staticVector<float>* depthMap, staticVector<staticVector<int>*>* tmp, const multiviewParams* mp,
                          int rc, int scale, int w, int h)
{
    depthMap->resize_with(w * h, -1.0f);

    pixel pix;
    for(pix.x = 0; pix.x < w; pix.x++)
    {
        for(pix.y = 0; pix.y < h; pix.y++)
        {

            staticVector<int>* ti = (*tmp)[pix.x * h + pix.y];
            if((ti != nullptr) && (ti->size() > 0))
            {
                point2d p;
                p.x = (double)pix.x;
                p.y = (double)pix.y;

                double mindepth = 10000000.0;

                for(int i = 0; i < ti->size(); i++)
                {
                    int idTri = (*ti)[i];
                    orientedPoint tri;
                    tri.p = (*pts)[(*tris)[idTri].i[0]];
                    tri.n = cross(((*pts)[(*tris)[idTri].i[1]] - (*pts)[(*tris)[idTri].i[0]]).normalize(),
                                  ((*pts)[(*tris)[idTri].i[2]] - (*pts)[(*tris)[idTri].i[0]]).normalize());

                    mv_mesh::rectangle re = mv_mesh::rectangle(pix, 1);
                    triangle_proj tp = getTriangleProjection(idTri, mp, rc, w, h);

                    /*
                    if ((pix.x==66)&&(pix.y==117))
                    {
                            printf("plot([%f %f],[%f %f],'r-');\n",
                                    tp.tp2ds[0].x,tp.tp2ds[1].x,tp.tp2ds[0].y,tp.tp2ds[1].y);
                            printf("plot([%f %f],[%f %f],'r-');\n",
                                    tp.tp2ds[1].x,tp.tp2ds[2].x,tp.tp2ds[1].y,tp.tp2ds[2].y);
                            printf("plot([%f %f],[%f %f],'r-');\n",
                                    tp.tp2ds[2].x,tp.tp2ds[0].x,tp.tp2ds[2].y,tp.tp2ds[0].y);
                    };


                    if ((pix.x==67)&&(pix.y==118))
                    {
                            printf("%f\n", angleBetwV1andV2((mp->CArr[rc]-tri.p).normalize(),tri.n));
                    }
                    */

                    staticVector<point2d>* tpis = getTrianglePixelIntersectionsAndInternalPoints(&tp, &re);

                    double maxd = -1.0;
                    for(int k = 0; k < tpis->size(); k++)
                    {
                        point3d lpi = linePlaneIntersect(
                            mp->CArr[rc], (mp->iCamArr[rc] * ((*tpis)[k] * (float)scale)).normalize(), tri.p, tri.n);
                        if(!std::isnan(angleBetwV1andV2((mp->CArr[rc] - tri.p).normalize(), tri.n)))
                        {
                            maxd = std::max(maxd, (mp->CArr[rc] - lpi).size());
                        }
                        else
                        {
                            /*
                            printf("A %f %f %f\n", (*pts)[(*tris)[idTri].i[0]].x, (*pts)[(*tris)[idTri].i[0]].y,
                            (*pts)[(*tris)[idTri].i[0]].z);
                            printf("B %f %f %f\n", (*pts)[(*tris)[idTri].i[1]].x, (*pts)[(*tris)[idTri].i[1]].y,
                            (*pts)[(*tris)[idTri].i[1]].z);
                            printf("C %f %f %f\n", (*pts)[(*tris)[idTri].i[2]].x, (*pts)[(*tris)[idTri].i[2]].y,
                            (*pts)[(*tris)[idTri].i[2]].z);
                            printf("n %f %f %f\n", tri.n.x, tri.n.y, tri.n.z);
                            printf("ids %i %i %i\n", (*tris)[idTri].i[0], (*tris)[idTri].i[1], (*tris)[idTri].i[2]);
                            */

                            maxd = std::max(maxd, (mp->CArr[rc] - (*pts)[(*tris)[idTri].i[1]]).size());

                            /*
                            staticVector<point3d> *tpis1 = getTrianglePixelIntersectionsAndInternalPoints(mp, idTri,
                            pix, rc, &tp, &re);

                            float maxd = -1.0;
                            for (int k=0;k<tpis1->size();k++) {
                                    maxd = std::max(maxd,(mp->CArr[rc]-(*tpis1)[k]).size());
                            };

                            delete tpis1;
                            */
                        }
                        //};
                    }
                    mindepth = std::min(mindepth, maxd);

                    delete tpis;
                }

                /*
                //for (int idTri=0;idTri<tris->size();idTri++)
                for (int i=0;i<ti->size();i++)
                {
                        int idTri = (*ti)[i];
                        orientedPoint tri;
                        tri.p = (*pts)[(*tris)[idTri].i[0]];
                        tri.n = cross( ((*pts)[(*tris)[idTri].i[1]]-(*pts)[(*tris)[idTri].i[0]]).normalize(),
                                                   ((*pts)[(*tris)[idTri].i[2]]-(*pts)[(*tris)[idTri].i[0]]).normalize()
                );
                        point3d lpi =
                linePlaneIntersect(mp->CArr[rc],(mp->iCamArr[rc]*point2d(p.x,p.y)).normalize(),tri.p,tri.n);
                        if ((mp->is3DPointInFrontOfCam(&lpi,rc)==true)&&
                                (isLineInTriangle(
                                        &(*pts)[(*tris)[idTri].i[0]],
                                        &(*pts)[(*tris)[idTri].i[1]],
                                        &(*pts)[(*tris)[idTri].i[2]],
                                        &mp->CArr[rc],
                                        &(mp->iCamArr[rc]*point2d(p.x,p.y)).normalize())==true))
                        {
                                mindepth=std::min(mindepth,(mp->CArr[rc]-lpi).size());
                        };
                };
                */

                (*depthMap)[pix.x * h + pix.y] = mindepth;
            }
            else
            {
                (*depthMap)[pix.x * h + pix.y] = -1.0f;
            }
        } // for pix.y
    }     // for pix.x
}

void mv_mesh::loadFromMesh(std::string fileName)
{
    FILE* f = fopen(fileName.c_str(), "r");

    int npts, ntris, c, k;
    fscanf(f, "MeshVersionFormatted 1%d", &c);
    fscanf(f, "Dimension 3%d", &c);
    fscanf(f, "Vertices%d", &c);
    fscanf(f, "%i%d", &npts, &c);
    pts = new staticVector<point3d>(npts);
    for(int i = 0; i < npts; i++)
    {
        point3d p;
        fscanf(f, "%lf %lf %lf %i\n", &p.x, &p.y, &p.z, &k);
        pts->push_back(p);
    }
    fscanf(f, "Triangles%d", &c);
    fscanf(f, "%i%d", &ntris, &c);

    tris = new staticVector<mv_mesh::triangle>(ntris);

    for(int i = 0; i < ntris; i++)
    {
        int x, y, z;
        fscanf(f, "%i %i %i %i\n", &x, &y, &z, &k);

        mv_mesh::triangle t;
        t.i[0] = x;
        t.i[1] = y;
        t.i[2] = z;
        t.alive = true;

        tris->push_back(t);
    }

    fclose(f);
}

void mv_mesh::saveToOFF(std::string fileName)
{
    FILE* f = fopen(fileName.c_str(), "w");
    fprintf(f, "OFF\n");
    fprintf(f, "%i %i 0\n", pts->size(), tris->size());
    for(int i = 0; i < pts->size(); i++)
    {
        fprintf(f, "%lf %lf %lf\n", (*pts)[i].x, (*pts)[i].y, (*pts)[i].z);
    }
    for(int i = 0; i < tris->size(); i++)
    {
        fprintf(f, "3 %i %i %i\n", (*tris)[i].i[0], (*tris)[i].i[1], (*tris)[i].i[2]);
    }
    fclose(f);
}

staticVector<int>* mv_mesh::getVisibleTrianglesIndexes(std::string depthMapFileName, std::string trisMapFileName,
                                                       const multiviewParams* mp, int rc, int w, int h)
{
    staticVector<float>* depthMap = loadArrayFromFile<float>(depthMapFileName);
    staticVector<staticVector<int>*>* trisMap = loadArrayOfArraysFromFile<int>(trisMapFileName);

    staticVector<int>* vistri = getVisibleTrianglesIndexes(trisMap, depthMap, mp, rc, w, h);

    deleteArrayOfArrays<int>(&trisMap);
    delete depthMap;

    return vistri;
}

staticVector<int>* mv_mesh::getVisibleTrianglesIndexes(std::string tmpDir, const multiviewParams* mp, int rc, int w, int h)
{
    std::string depthMapFileName = tmpDir + "depthMap" + num2strFourDecimal(rc) + ".bin";
    std::string trisMapFileName = tmpDir + "trisMap" + num2strFourDecimal(rc) + ".bin";

    staticVector<float>* depthMap = loadArrayFromFile<float>(depthMapFileName);
    staticVector<staticVector<int>*>* trisMap = loadArrayOfArraysFromFile<int>(trisMapFileName);

    staticVector<int>* vistri = getVisibleTrianglesIndexes(trisMap, depthMap, mp, rc, w, h);

    deleteArrayOfArrays<int>(&trisMap);
    delete depthMap;

    return vistri;
}

staticVector<int>* mv_mesh::getVisibleTrianglesIndexes(staticVector<float>* depthMap, const multiviewParams* mp, int rc,
                                                       int w, int h)
{
    int ow = mp->mip->getWidth(rc);
    int oh = mp->mip->getHeight(rc);

    staticVector<int>* out = new staticVector<int>(tris->size());
    for(int i = 0; i < tris->size(); i++)
    {
        point3d cg = computeTriangleCenterOfGravity(i);
        pixel pix;
        mp->getPixelFor3DPoint(&pix, cg, rc);
        if(mp->isPixelInImage(pix, 1, rc))
        {
            pix.x = (int)(((float)pix.x / (float)ow) * (float)w);
            pix.y = (int)(((float)pix.y / (float)oh) * (float)h);
            float depth = (*depthMap)[pix.x * h + pix.y];
            float pixSize = mp->getCamPixelSize(cg, rc) * 6.0f;
            // if (depth-pixSize<(mp->CArr[rc]-cg).size()) {
            if(fabs(depth - (mp->CArr[rc] - cg).size()) < pixSize)
            {
                out->push_back(i);
            }
        }
    }
    return out;
}

staticVector<int>* mv_mesh::getVisibleTrianglesIndexes(staticVector<staticVector<int>*>* trisMap,
                                                       staticVector<float>* depthMap, const multiviewParams* mp, int rc,
                                                       int w, int h)
{
    int ow = mp->mip->getWidth(rc);
    int oh = mp->mip->getHeight(rc);

    staticVectorBool* btris = new staticVectorBool(tris->size());
    btris->resize_with(tris->size(), false);

    pixel pix;
    for(pix.x = 0; pix.x < w; pix.x++)
    {
        for(pix.y = 0; pix.y < h; pix.y++)
        {

            staticVector<int>* ti = (*trisMap)[pix.x * h + pix.y];
            if(ti != nullptr)
            {
                point2d p;
                p.x = (float)pix.x;
                p.y = (float)pix.y;

                float depth = (*depthMap)[pix.x * h + pix.y];
                for(int i = 0; i < ti->size(); i++)
                {
                    int idTri = (*ti)[i];
                    orientedPoint tri;
                    tri.p = (*pts)[(*tris)[idTri].i[0]];
                    tri.n = cross(((*pts)[(*tris)[idTri].i[1]] - (*pts)[(*tris)[idTri].i[0]]).normalize(),
                                  ((*pts)[(*tris)[idTri].i[2]] - (*pts)[(*tris)[idTri].i[0]]).normalize());

                    mv_mesh::rectangle re = mv_mesh::rectangle(pix, 1);
                    triangle_proj tp = getTriangleProjection(idTri, mp, rc, w, h);

                    /*
                    staticVector<point2d> *tpis = getTrianglePixelIntersectionsAndInternalPoints(&tp, &re);
                    float mindepth = 10000000.0f;
                    point3d minlpi;
                    for (int k=0;k<tpis->size();k++) {
                            point3d lpi =
                    linePlaneIntersect(mp->CArr[rc],(mp->iCamArr[rc]*((*tpis)[k]*(float)scale)).normalize(),tri.p,tri.n);
                            if (mindepth>(mp->CArr[rc]-lpi).size()) {
                                    mindepth = (mp->CArr[rc]-lpi).size();
                                    minlpi = lpi;
                            };
                    };
                    float pixSize = mp->getCamPixelSize(minlpi,rc) * 2.0f;
                    if (fabs(depth-mindepth)<pixSize) {
                            (*btris)[idTri] = true;
                    };
                    delete tpis;
                    */

                    point2d tpip = getTrianglePixelInternalPoint(&tp, &re);
                    tpip.x = (tpip.x / (float)w) * (float)ow;
                    tpip.y = (tpip.y / (float)h) * (float)oh;

                    point3d lpi = linePlaneIntersect(mp->CArr[rc], (mp->iCamArr[rc] * tpip).normalize(), tri.p, tri.n);
                    float lpidepth = (mp->CArr[rc] - lpi).size();
                    float pixSize = mp->getCamPixelSize(lpi, rc) * 2.0f;
                    if(fabs(depth - lpidepth) < pixSize)
                    {
                        (*btris)[idTri] = true;
                    }
                }
            }
        } // for pix.y
    }     // for pix.x

    int nvistris = 0;
    for(int i = 0; i < btris->size(); i++)
    {
        if((*btris)[i])
        {
            nvistris++;
        }
    }

    staticVector<int>* out = new staticVector<int>(nvistris);
    for(int i = 0; i < btris->size(); i++)
    {
        if((*btris)[i])
        {
            out->push_back(i);
        }
    }

    // deallocate
    delete btris;

    return out;
}

staticVector<staticVector<int>*>* mv_mesh::getTrisNeighTris()
{
    staticVector<staticVector<int>*>* ptsNeighTris = getPtsNeighTris();

    // compute n
    staticVector<int>* ntrisNeighTris = new staticVector<int>(tris->size());
    ntrisNeighTris->resize_with(tris->size(), 0);

    for(int i = 0; i < pts->size(); i++)
    {
        int middlePtId = i;
        staticVector<int>* triTmp = (*ptsNeighTris)[i];
        if((triTmp != nullptr) && (triTmp->size() > 0))
        {
            for(int l1 = 0; l1 < triTmp->size(); l1++)
            {
                for(int l2 = 0; l2 < triTmp->size(); l2++)
                {
                    int idTri1 = (*triTmp)[l1];
                    int idTri2 = (*triTmp)[l2];
                    if(idTri1 != idTri2)
                    {
                        int nok = 0;
                        for(int k1 = 0; k1 < 3; k1++)
                        {
                            bool ok = false;
                            for(int k2 = 0; k2 < 3; k2++)
                            {
                                if(((*tris)[idTri1].i[k1] == (*tris)[idTri2].i[k2]) &&
                                   ((*tris)[idTri1].i[k1] != middlePtId))
                                {
                                    ok = true;
                                }
                            }
                            nok += static_cast<int>(ok);
                        }
                        if(nok == 1)
                        {
                            (*ntrisNeighTris)[idTri1]++;
                            (*ntrisNeighTris)[idTri2]++;
                        }
                    }
                } // for l2
            }     // for li
        }         // if ((triTmp!=NULL)&&(triTmp->size()>0))
    }

    // allocate
    staticVector<staticVector<int>*>* trisNeighTris = new staticVector<staticVector<int>*>(tris->size());
    trisNeighTris->resize_with(tris->size(), nullptr);
    for(int i = 0; i < tris->size(); i++)
    {
        if((*ntrisNeighTris)[i] > 0)
        {
            (*trisNeighTris)[i] = new staticVector<int>((*ntrisNeighTris)[i]);
        }
    }

    // fill
    for(int i = 0; i < pts->size(); i++)
    {
        int middlePtId = i;
        staticVector<int>* triTmp = (*ptsNeighTris)[i];
        if((triTmp != nullptr) && (triTmp->size() > 0))
        {
            for(int l1 = 0; l1 < triTmp->size(); l1++)
            {
                for(int l2 = 0; l2 < triTmp->size(); l2++)
                {
                    int idTri1 = (*triTmp)[l1];
                    int idTri2 = (*triTmp)[l2];
                    if(idTri1 != idTri2)
                    {
                        int nok = 0;
                        for(int k1 = 0; k1 < 3; k1++)
                        {
                            bool ok = false;
                            for(int k2 = 0; k2 < 3; k2++)
                            {
                                if(((*tris)[idTri1].i[k1] == (*tris)[idTri2].i[k2]) &&
                                   ((*tris)[idTri1].i[k1] != middlePtId))
                                {
                                    ok = true;
                                }
                            }
                            nok += static_cast<int>(ok);
                        }
                        if(nok == 1)
                        {
                            (*trisNeighTris)[idTri1]->push_back(idTri2);
                            (*trisNeighTris)[idTri2]->push_back(idTri1);
                        }
                    }
                } // for l2
            }     // for li
        }         // if ((triTmp!=NULL)&&(triTmp->size()>0))
    }

    // remove duplicities
    for(int i = 0; i < trisNeighTris->size(); i++)
    {
        staticVector<int>* nei = (*trisNeighTris)[i];
        if(nei != nullptr)
        {
            qsort(&(*nei)[0], nei->size(), sizeof(int), qSortCompareIntAsc);
            staticVector<int>* nei1 = new staticVector<int>(nei->size());
            for(int j = 0; j < nei->size(); j++)
            {
                if((j == nei->size() - 1) || ((*nei)[j] != (*nei)[j + 1]))
                {
                    nei1->push_back((*nei)[j]);
                }
            }
            delete nei;
            (*trisNeighTris)[i] = nei1;
        }
    }

    // deallocate
    delete ntrisNeighTris;
    deleteArrayOfArrays(&ptsNeighTris);

    return trisNeighTris;
}

mv_mesh* mv_mesh::generateMeshFromTrianglesSubset(staticVector<int>* visTris, staticVector<int>** ptIdToNewPtId)
{
    mv_mesh* me = new mv_mesh();

    staticVector<int>* pts1 = new staticVector<int>(pts->size());
    pts1->resize_with(pts->size(), -1);
    for(int i = 0; i < visTris->size(); i++)
    {
        int idTri = (*visTris)[i];
        (*pts1)[(*tris)[idTri].i[0]] = 0;
        (*pts1)[(*tris)[idTri].i[1]] = 0;
        (*pts1)[(*tris)[idTri].i[2]] = 0;
    }

    int j = 0;
    for(int i = 0; i < pts->size(); i++)
    {
        if((*pts1)[i] > -1)
        {
            (*pts1)[i] = j;
            j++;
        }
    }

    if(ptIdToNewPtId != nullptr)
    {
        (*ptIdToNewPtId) = new staticVector<int>(pts1->size());
        for(int i = 0; i < pts1->size(); i++)
        {
            (*ptIdToNewPtId)->push_back((*pts1)[i]);
        }
    }

    me->pts = new staticVector<point3d>(j);

    for(int i = 0; i < pts->size(); i++)
    {
        if((*pts1)[i] > -1)
        {
            me->pts->push_back((*pts)[i]);
        }
    }

    me->tris = new staticVector<mv_mesh::triangle>(visTris->size());
    for(int i = 0; i < visTris->size(); i++)
    {
        int idTri = (*visTris)[i];
        mv_mesh::triangle t;
        t.alive = true;
        t.i[0] = (*pts1)[(*tris)[idTri].i[0]];
        t.i[1] = (*pts1)[(*tris)[idTri].i[1]];
        t.i[2] = (*pts1)[(*tris)[idTri].i[2]];
        me->tris->push_back(t);
    }

    delete pts1;

    return me;
}

void mv_mesh::getNotOrientedEdges(staticVector<staticVector<int>*>** edgesNeighTris,
                                  staticVector<pixel>** edgesPointsPairs)
{
    // printf("getNotOrientedEdges\n");
    staticVector<voxel>* edges = new staticVector<voxel>(tris->size() * 3);

    for(int i = 0; i < tris->size(); i++)
    {
        int a = (*tris)[i].i[0];
        int b = (*tris)[i].i[1];
        int c = (*tris)[i].i[2];
        edges->push_back(voxel(std::min(a, b), std::max(a, b), i));
        edges->push_back(voxel(std::min(b, c), std::max(b, c), i));
        edges->push_back(voxel(std::min(c, a), std::max(c, a), i));
    }

    qsort(&(*edges)[0], edges->size(), sizeof(voxel), qSortCompareVoxelByXAsc);

    staticVector<staticVector<int>*>* _edgesNeighTris = new staticVector<staticVector<int>*>(tris->size() * 3);
    staticVector<pixel>* _edgesPointsPairs = new staticVector<pixel>(tris->size() * 3);

    // remove duplicities
    int i0 = 0;
    long t1 = initEstimate();
    for(int i = 0; i < edges->size(); i++)
    {
        if((i == edges->size() - 1) || ((*edges)[i].x != (*edges)[i + 1].x))
        {
            staticVector<voxel>* edges1 = new staticVector<voxel>(i - i0 + 1);
            for(int j = i0; j <= i; j++)
            {
                edges1->push_back((*edges)[j]);
            }
            qsort(&(*edges1)[0], edges1->size(), sizeof(voxel), qSortCompareVoxelByYAsc);

            int j0 = 0;
            for(int j = 0; j < edges1->size(); j++)
            {
                if((j == edges1->size() - 1) || ((*edges1)[j].y != (*edges1)[j + 1].y))
                {
                    _edgesPointsPairs->push_back(pixel((*edges1)[j].x, (*edges1)[j].y));
                    staticVector<int>* neighTris = new staticVector<int>(j - j0 + 1);
                    for(int k = j0; k <= j; k++)
                    {
                        neighTris->push_back((*edges1)[k].z);
                    }
                    _edgesNeighTris->push_back(neighTris);
                    j0 = j + 1;
                }
            }

            delete edges1;
            i0 = i + 1;
        }

        printfEstimate(i, edges->size(), t1);
    }
    finishEstimate();

    (*edgesNeighTris) = _edgesNeighTris;
    (*edgesPointsPairs) = _edgesPointsPairs;

    delete edges;
}

staticVector<pixel>* mv_mesh::getNotOrientedEdgesAsPointsPairs()
{
    staticVector<staticVector<int>*>* edgesNeighTris;
    staticVector<pixel>* edgesPointsPairs;
    getNotOrientedEdges(&edgesNeighTris, &edgesPointsPairs);
    deleteArrayOfArrays<int>(&edgesNeighTris);
    return edgesPointsPairs;
}

staticVector<pixel>* mv_mesh::getNotOrientedEdgesAsTrianglesPairs()
{
    // printf("getNotOrientedEdgesAsTrianglesPairs\n");

    staticVector<staticVector<int>*>* edgesNeighTris;
    staticVector<pixel>* edgesPointsPairs;
    getNotOrientedEdges(&edgesNeighTris, &edgesPointsPairs);
    int ntrispairs = 0;
    for(int i = 0; i < edgesNeighTris->size(); i++)
    {
        staticVector<int>* neighTris = (*edgesNeighTris)[i];
        for(int j = 0; j < sizeOfStaticVector<int>(neighTris); j++)
        {
            for(int k = j + 1; k < sizeOfStaticVector<int>(neighTris); k++)
            {
                ntrispairs++;
            }
        }
    }

    staticVector<pixel>* edges = new staticVector<pixel>(ntrispairs);
    for(int i = 0; i < edgesNeighTris->size(); i++)
    {
        staticVector<int>* neighTris = (*edgesNeighTris)[i];
        for(int j = 0; j < sizeOfStaticVector<int>(neighTris); j++)
        {
            for(int k = j + 1; k < sizeOfStaticVector<int>(neighTris); k++)
            {
                edges->push_back(pixel((*neighTris)[j], (*neighTris)[k]));
            }
        }
    }
    delete edgesPointsPairs;
    deleteArrayOfArrays<int>(&edgesNeighTris);

    return edges;
}

staticVector<point3d>* mv_mesh::getLaplacianSmoothingVectors(staticVector<staticVector<int>*>* ptsNeighPts,
                                                             double maximalNeighDist)
{
    staticVector<point3d>* nms = new staticVector<point3d>(pts->size());

    for(int i = 0; i < pts->size(); i++)
    {
        point3d p = (*pts)[i];
        staticVector<int>* nei = (*ptsNeighPts)[i];
        int nneighs = 0;
        if(nei != nullptr)
        {
            nneighs = nei->size();
        }

        if(nneighs == 0)
        {
            nms->push_back(point3d(0.0, 0.0, 0.0));
        }
        else
        {
            double maxNeighDist = 0.0f;
            // laplacian smoothing vector
            point3d n = point3d(0.0, 0.0, 0.0);
            for(int j = 0; j < nneighs; j++)
            {
                n = n + (*pts)[(*nei)[j]];
                maxNeighDist = std::max(maxNeighDist, (p - (*pts)[(*nei)[j]]).size());
            }
            n = ((n / (float)nneighs) - p);

            float d = n.size();
            n = n.normalize();

            if(std::isnan(d) || std::isnan(n.x) || std::isnan(n.y) || std::isnan(n.z) || (d != d) || (n.x != n.x) ||
               (n.y != n.y) || (n.z != n.z)) // check if is not NaN
            {
                n = point3d(0.0, 0.0, 0.0);
            }
            else
            {
                n = n * d;
            }

            if(std::isnan(d) || std::isnan(n.x) || std::isnan(n.y) || std::isnan(n.z) || (d != d) || (n.x != n.x) ||
               (n.y != n.y) || (n.z != n.z)) // check if is not NaN
            {
                n = point3d(0.0, 0.0, 0.0);
            }

            if((maximalNeighDist > 0.0f) && (maxNeighDist > maximalNeighDist))
            {
                n = point3d(0.0, 0.0, 0.0);
            }

            nms->push_back(n);
        }
    }

    return nms;
}

void mv_mesh::laplacianSmoothPts(float maximalNeighDist)
{
    staticVector<staticVector<int>*>* ptsNei = getPtsNeighPtsOrdered();
    laplacianSmoothPts(ptsNei, maximalNeighDist);
    deleteArrayOfArrays<int>(&ptsNei);
}

void mv_mesh::laplacianSmoothPts(staticVector<staticVector<int>*>* ptsNeighPts, double maximalNeighDist)
{
    staticVector<point3d>* nms = getLaplacianSmoothingVectors(ptsNeighPts, maximalNeighDist);

    // smooth
    for(int i = 0; i < pts->size(); i++)
    {
        (*pts)[i] = (*pts)[i] + (*nms)[i];
    }

    delete nms;
}

point3d mv_mesh::computeTriangleNormal(int idTri)
{
    return cross(((*pts)[(*tris)[idTri].i[1]] - (*pts)[(*tris)[idTri].i[0]]).normalize(),
                 ((*pts)[(*tris)[idTri].i[2]] - (*pts)[(*tris)[idTri].i[0]]).normalize())
        .normalize();
}

point3d mv_mesh::computeTriangleCenterOfGravity(int idTri) const
{
    return ((*pts)[(*tris)[idTri].i[0]] + (*pts)[(*tris)[idTri].i[1]] + (*pts)[(*tris)[idTri].i[2]]) / 3.0f;
}

float mv_mesh::computeTriangleMaxEdgeLength(int idTri) const
{
    return std::max(std::max(((*pts)[(*tris)[idTri].i[0]] - (*pts)[(*tris)[idTri].i[1]]).size(),
                             ((*pts)[(*tris)[idTri].i[1]] - (*pts)[(*tris)[idTri].i[2]]).size()),
                    ((*pts)[(*tris)[idTri].i[2]] - (*pts)[(*tris)[idTri].i[0]]).size());
}

float mv_mesh::computeTriangleAverageEdgeLength(int idTri)
{
    return (((*pts)[(*tris)[idTri].i[0]] - (*pts)[(*tris)[idTri].i[1]]).size() +
            ((*pts)[(*tris)[idTri].i[1]] - (*pts)[(*tris)[idTri].i[2]]).size() +
            ((*pts)[(*tris)[idTri].i[2]] - (*pts)[(*tris)[idTri].i[0]]).size()) /
           3.0f;
}

staticVector<point3d>* mv_mesh::computeNormalsForPts()
{
    staticVector<staticVector<int>*>* ptsNeighTris = getPtsNeighTris();
    staticVector<point3d>* nms = computeNormalsForPts(ptsNeighTris);
    deleteArrayOfArrays<int>(&ptsNeighTris);
    return nms;
}

staticVector<point3d>* mv_mesh::computeNormalsForPts(staticVector<staticVector<int>*>* ptsNeighTris)
{
    staticVector<point3d>* nms = new staticVector<point3d>(pts->size());
    nms->resize_with(pts->size(), point3d(0.0f, 0.0f, 0.0f));

    for(int i = 0; i < pts->size(); i++)
    {
        staticVector<int>* triTmp = (*ptsNeighTris)[i];
        if((triTmp != nullptr) && (triTmp->size() > 0))
        {
            point3d n = point3d(0.0f, 0.0f, 0.0f);
            float nn = 0.0f;
            for(int j = 0; j < triTmp->size(); j++)
            {
                point3d n1 = computeTriangleNormal((*triTmp)[j]);
                n1 = n1.normalize();
                if(std::isnan(n1.x) || std::isnan(n1.y) || std::isnan(n1.z) || (n1.x != n1.x) || (n1.y != n1.y) ||
                   (n1.z != n1.z)) // check if is not NaN
                {
                    //
                }
                else
                {
                    n = n + computeTriangleNormal((*triTmp)[j]);
                    nn += 1.0f;
                }
            }
            n = n / nn;

            n = n.normalize();
            if(std::isnan(n.x) || std::isnan(n.y) || std::isnan(n.z) || (n.x != n.x) || (n.y != n.y) ||
               (n.z != n.z)) // check if is not NaN
            {
                n = point3d(0.0f, 0.0f, 0.0f);
                printf(".");
            }

            (*nms)[i] = n;
        }
    }

    return nms;
}

void mv_mesh::smoothNormals(staticVector<point3d>* nms, staticVector<staticVector<int>*>* ptsNeighPts)
{
    staticVector<point3d>* nmss = new staticVector<point3d>(pts->size());
    nmss->resize_with(pts->size(), point3d(0.0f, 0.0f, 0.0f));

    for(int i = 0; i < pts->size(); i++)
    {
        point3d n = (*nms)[i];
        for(int j = 0; j < sizeOfStaticVector<int>((*ptsNeighPts)[i]); j++)
        {
            n = n + (*nms)[(*(*ptsNeighPts)[i])[j]];
        }
        if(sizeOfStaticVector<int>((*ptsNeighPts)[i]) > 0)
        {
            n = n / (float)sizeOfStaticVector<int>((*ptsNeighPts)[i]);
        }
        n = n.normalize();
        if(std::isnan(n.x) || std::isnan(n.y) || std::isnan(n.z) || (n.x != n.x) || (n.y != n.y) || (n.z != n.z))
        {
            n = point3d(0.0f, 0.0f, 0.0f);
        }
        (*nmss)[i] = n;
    }
    for(int i = 0; i < pts->size(); i++)
    {
        (*nms)[i] = (*nmss)[i];
    }
    delete nmss;
}

staticVector<float>* mv_mesh::computePtsAvNeighEdgeLength(staticVector<staticVector<int>*>* ptsNeighPts)
{
    staticVector<float>* avnel = new staticVector<float>(pts->size());
    avnel->resize_with(pts->size(), 0.0f);

    for(int i = 0; i < pts->size(); i++)
    {
        staticVector<int>* ptsTmp = (*ptsNeighPts)[i];
        if((ptsTmp != nullptr) && (ptsTmp->size() > 0))
        {
            point3d p = (*pts)[i];
            float avs = 0.0f;
            float ns = 0.0f;
            for(int j = 0; j < ptsTmp->size(); j++)
            {
                float s = (p - (*pts)[(*ptsTmp)[j]]).size();
                if(!std::isnan(s))
                {
                    avs += s;
                    ns += 1.0f;
                }
            }
            avs /= ns;
            (*avnel)[i] = avs;
        }
    }

    return avnel;
}

staticVector<point3d>* mv_mesh::computeSmoothUnitVectsForPts(staticVector<staticVector<int>*>* ptsNeighPts)
{
    staticVector<point3d>* nms = getLaplacianSmoothingVectors(ptsNeighPts);
    for(int i = 0; i < nms->size(); i++)
    {
        point3d n = (*nms)[i];
        n = n.normalize();
        if(std::isnan(n.x) || std::isnan(n.y) || std::isnan(n.z) || (n.x != n.x) || (n.y != n.y) ||
           (n.z != n.z)) // check if is not NaN
        {
            n = point3d(0.0f, 0.0f, 0.0f);
        }
        (*nms)[i] = n;
    }

    return nms;
}

void mv_mesh::removeFreePointsFromMesh()
{
    removeFreePointsFromMesh(nullptr);
}

void mv_mesh::removeFreePointsFromMesh(staticVector<int>** ptIdToNewPtId)
{
    printf("removeFreePointsFromMesh\n");

    staticVector<int>* visTris = new staticVector<int>(tris->size());
    for(int i = 0; i < tris->size(); i++)
    {
        visTris->push_back(i);
    }
    mv_mesh* me1 = generateMeshFromTrianglesSubset(visTris, ptIdToNewPtId);
    delete visTris;

    delete pts;
    delete tris;

    pts = new staticVector<point3d>(me1->pts->size());
    for(int i = 0; i < me1->pts->size(); i++)
    {
        pts->push_back((*me1->pts)[i]);
    }

    tris = new staticVector<mv_mesh::triangle>(me1->tris->size());
    for(int i = 0; i < me1->tris->size(); i++)
    {
        tris->push_back((*me1->tris)[i]);
    }

    delete me1;
}

float mv_mesh::computeTriangleProjectionArea(triangle_proj& tp)
{
    // return (float)((tp.rd.x-tp.lu.x+1)*(tp.rd.y-tp.lu.y+1));

    point2d pa = tp.tp2ds[0];
    point2d pb = tp.tp2ds[1];
    point2d pc = tp.tp2ds[2];
    float a = (pb - pa).size();
    float b = (pc - pa).size();
    float c = (pc - pb).size();
    float p = (a + b + c) / 2.0;

    return sqrt(p * (p - a) * (p - b) * (p - c));

    //	point2d e1 = tp.tp2ds[1]-tp.tp2ds[0];
    //	point2d e2 = tp.tp2ds[2]-tp.tp2ds[0];
    //	return  cross(e1,e2).size()/2.0f;
}

float mv_mesh::computeTriangleArea(int idTri)
{
    point3d pa = (*pts)[(*tris)[idTri].i[0]];
    point3d pb = (*pts)[(*tris)[idTri].i[1]];
    point3d pc = (*pts)[(*tris)[idTri].i[2]];
    float a = (pb - pa).size();
    float b = (pc - pa).size();
    float c = (pc - pb).size();
    float p = (a + b + c) / 2.0;

    return sqrt(p * (p - a) * (p - b) * (p - c));
}

staticVector<voxel>* mv_mesh::getTrianglesEdgesIds(staticVector<staticVector<int>*>* edgesNeighTris)
{
    staticVector<voxel>* out = new staticVector<voxel>(tris->size());
    out->resize_with(tris->size(), voxel(-1, -1, -1));

    for(int i = 0; i < edgesNeighTris->size(); i++)
    {
        for(int j = 0; j < (*edgesNeighTris)[i]->size(); j++)
        {
            int idTri = (*(*edgesNeighTris)[i])[j];

            if((*out)[idTri].x == -1)
            {
                (*out)[idTri].x = i;
            }
            else
            {
                if((*out)[idTri].y == -1)
                {
                    (*out)[idTri].y = i;
                }
                else
                {
                    if((*out)[idTri].z == -1)
                    {
                        (*out)[idTri].z = i;
                    }
                    else
                    {
                        printf("warning ... getEdgesIdsToEachTriangle!!!\n");
                    }
                }
            }

        } // for j
    }     // for i

    // check ... each triangle has to have three edge ids
    for(int i = 0; i < tris->size(); i++)
    {
        if(((*out)[i].x == -1) || ((*out)[i].y == -1) || ((*out)[i].z == -1))
        {
            printf("warning ... getEdgesIdsToEachTriangle 1!!!\n");
        }
    }

    return out;
}

void mv_mesh::subdivideMeshCase1(int i, staticVector<pixel>* edgesi, pixel& neptIdEdgeId,
                                 staticVector<mv_mesh::triangle>* tris1)
{
    int ii[5];
    ii[0] = (*tris)[i].i[0];
    ii[1] = (*tris)[i].i[1];
    ii[2] = (*tris)[i].i[2];
    ii[3] = (*tris)[i].i[0];
    ii[4] = (*tris)[i].i[1];
    for(int k = 0; k < 3; k++)
    {
        int a = ii[k];
        int b = ii[k + 1];
        int c = ii[k + 2];
        if((((*edgesi)[neptIdEdgeId.y].x == a) && ((*edgesi)[neptIdEdgeId.y].y == b)) ||
           (((*edgesi)[neptIdEdgeId.y].y == a) && ((*edgesi)[neptIdEdgeId.y].x == b)))
        {
            mv_mesh::triangle t;
            t.alive = true;
            t.i[0] = a;
            t.i[1] = neptIdEdgeId.x;
            t.i[2] = c;
            tris1->push_back(t);

            t.i[0] = neptIdEdgeId.x;
            t.i[1] = b;
            t.i[2] = c;
            tris1->push_back(t);
        }
    }
}

void mv_mesh::subdivideMeshCase2(int i, staticVector<pixel>* edgesi, pixel& neptIdEdgeId1, pixel& neptIdEdgeId2,
                                 staticVector<mv_mesh::triangle>* tris1)
{
    int ii[5];
    ii[0] = (*tris)[i].i[0];
    ii[1] = (*tris)[i].i[1];
    ii[2] = (*tris)[i].i[2];
    ii[3] = (*tris)[i].i[0];
    ii[4] = (*tris)[i].i[1];
    for(int k = 0; k < 3; k++)
    {
        int a = ii[k];
        int b = ii[k + 1];
        int c = ii[k + 2];
        if(((((*edgesi)[neptIdEdgeId1.y].x == a) && ((*edgesi)[neptIdEdgeId1.y].y == b)) ||
            (((*edgesi)[neptIdEdgeId1.y].y == a) && ((*edgesi)[neptIdEdgeId1.y].x == b))) &&
           ((((*edgesi)[neptIdEdgeId2.y].x == b) && ((*edgesi)[neptIdEdgeId2.y].y == c)) ||
            (((*edgesi)[neptIdEdgeId2.y].y == b) && ((*edgesi)[neptIdEdgeId2.y].x == c))))
        {
            mv_mesh::triangle t;
            t.alive = true;
            t.i[0] = a;
            t.i[1] = neptIdEdgeId1.x;
            t.i[2] = neptIdEdgeId2.x;
            tris1->push_back(t);

            t.i[0] = neptIdEdgeId1.x;
            t.i[1] = b;
            t.i[2] = neptIdEdgeId2.x;
            tris1->push_back(t);

            t.i[0] = neptIdEdgeId2.x;
            t.i[1] = c;
            t.i[2] = a;
            tris1->push_back(t);
        }
    }
}

void mv_mesh::subdivideMeshCase3(int i, staticVector<pixel>* edgesi, pixel& neptIdEdgeId1, pixel& neptIdEdgeId2,
                                 pixel& neptIdEdgeId3, staticVector<mv_mesh::triangle>* tris1)
{
    int a = (*tris)[i].i[0];
    int b = (*tris)[i].i[1];
    int c = (*tris)[i].i[2];
    if(((((*edgesi)[neptIdEdgeId1.y].x == a) && ((*edgesi)[neptIdEdgeId1.y].y == b)) ||
        (((*edgesi)[neptIdEdgeId1.y].y == a) && ((*edgesi)[neptIdEdgeId1.y].x == b))) &&
       ((((*edgesi)[neptIdEdgeId2.y].x == b) && ((*edgesi)[neptIdEdgeId2.y].y == c)) ||
        (((*edgesi)[neptIdEdgeId2.y].y == b) && ((*edgesi)[neptIdEdgeId2.y].x == c))) &&
       ((((*edgesi)[neptIdEdgeId3.y].x == c) && ((*edgesi)[neptIdEdgeId3.y].y == a)) ||
        (((*edgesi)[neptIdEdgeId3.y].y == c) && ((*edgesi)[neptIdEdgeId3.y].x == a))))
    {
        mv_mesh::triangle t;
        t.alive = true;
        t.i[0] = a;
        t.i[1] = neptIdEdgeId1.x;
        t.i[2] = neptIdEdgeId3.x;
        tris1->push_back(t);

        t.i[0] = neptIdEdgeId1.x;
        t.i[1] = b;
        t.i[2] = neptIdEdgeId2.x;
        tris1->push_back(t);

        t.i[0] = neptIdEdgeId2.x;
        t.i[1] = c;
        t.i[2] = neptIdEdgeId3.x;
        tris1->push_back(t);

        t.i[0] = neptIdEdgeId1.x;
        t.i[1] = neptIdEdgeId2.x;
        t.i[2] = neptIdEdgeId3.x;
        tris1->push_back(t);
    }
}

void mv_mesh::subdivideMesh(const multiviewParams* mp, float maxTriArea, std::string tmpDir, int maxMeshPts)
{
    staticVector<staticVector<int>*>* trisCams = computeTrisCams(mp, tmpDir);
    staticVector<staticVector<int>*>* trisCams1 = subdivideMesh(mp, maxTriArea, 0.0f, true, trisCams, maxMeshPts);
    deleteArrayOfArrays<int>(&trisCams);
    deleteArrayOfArrays<int>(&trisCams1);
}

void mv_mesh::subdivideMeshMaxEdgeLength(const multiviewParams* mp, float maxEdgeLenght, int maxMeshPts)
{
    subdivideMesh(mp, 0.0f, maxEdgeLenght, false, nullptr, maxMeshPts);
}

staticVector<staticVector<int>*>* mv_mesh::subdivideMesh(const multiviewParams* mp, float maxTriArea, float maxEdgeLength,
                                                         bool useMaxTrisAreaOrAvEdgeLength,
                                                         staticVector<staticVector<int>*>* trisCams, int maxMeshPts)
{
    printf("SUBDIVIDING MESH\n");

    staticVector<int>* trisCamsId = new staticVector<int>(tris->size());
    for(int i = 0; i < tris->size(); i++)
    {
        trisCamsId->push_back(i);
    }

    int nsubd = 1000;
    while((pts->size() < maxMeshPts) && (nsubd > 10))
    {
        nsubd = subdivideMesh(mp, maxTriArea, maxEdgeLength, useMaxTrisAreaOrAvEdgeLength, trisCams, &trisCamsId);
        printf("subdivided %i\n", nsubd);
    }
    // subdivideMesh(mp, maxTriArea, trisCams, &trisCamsId);
    // subdivideMesh(mp, maxTriArea, trisCams, &trisCamsId);

    if(trisCams != nullptr)
    {
        staticVector<staticVector<int>*>* trisCams1 = new staticVector<staticVector<int>*>(tris->size());
        for(int i = 0; i < tris->size(); i++)
        {
            int tcid = (*trisCamsId)[i];
            if((*trisCams)[tcid] != nullptr)
            {
                staticVector<int>* cams = new staticVector<int>((*trisCams)[tcid]->size());
                for(int j = 0; j < (*trisCams)[tcid]->size(); j++)
                {
                    cams->push_back((*(*trisCams)[tcid])[j]);
                }
                trisCams1->push_back(cams);
            }
            else
            {
                trisCams1->push_back(nullptr);
            }
        }
        delete trisCamsId;

        return trisCams1;
    }

    return nullptr;
}

void mv_mesh::subdivideMeshMaxEdgeLengthUpdatePtsCams(const multiviewParams* mp, float maxEdgeLength,
                                                      staticVector<staticVector<int>*>* ptsCams, int maxMeshPts)
{
    printf("SUBDIVIDING MESH\n");

    staticVector<int>* trisCamsId = new staticVector<int>(tris->size());
    for(int i = 0; i < tris->size(); i++)
    {
        trisCamsId->push_back(i);
    }

    int oldNPts = pts->size();
    int oldNTris = tris->size();
    staticVector<triangle>* oldTris = new staticVector<triangle>(tris->size());
    oldTris->push_back_arr(tris);

    int nsubd = 1000;
    while((pts->size() < maxMeshPts) && (nsubd > 10))
    {
        nsubd = subdivideMesh(mp, 0.0f, maxEdgeLength, false, nullptr, &trisCamsId);
        printf("subdivided %i\n", nsubd);
    }
    // subdivideMesh(mp, maxTriArea, trisCams, &trisCamsId);
    // subdivideMesh(mp, maxTriArea, trisCams, &trisCamsId);

    if(pts->size() - oldNPts > 0)
    {
        staticVector<int>* newPtsOldTriId = new staticVector<int>(pts->size() - oldNPts);
        newPtsOldTriId->resize_with(pts->size() - oldNPts, -1);
        for(int i = oldNTris; i < tris->size(); i++)
        {
            int tcid = (*trisCamsId)[i];
            while(tcid > oldNTris)
            {
                tcid = (*trisCamsId)[tcid];
            }
            for(int k = 0; k < 3; k++)
            {
                if((*tris)[i].i[k] >= oldNPts)
                {
                    (*newPtsOldTriId)[(*tris)[i].i[k] - oldNPts] = tcid;
                }
            }
        }

        ptsCams->resizeAdd(newPtsOldTriId->size());
        for(int i = 0; i < newPtsOldTriId->size(); i++)
        {
            staticVector<int>* cams = nullptr;
            int idTri = (*newPtsOldTriId)[i];
            if(idTri > -1)
            {
                if(((*oldTris)[idTri].i[0] >= oldNPts) || ((*oldTris)[idTri].i[1] >= oldNPts) ||
                   ((*oldTris)[idTri].i[2] >= oldNPts))
                {
                    printf("WARNING OUT OF RANGE!\n");
                    exit(1);
                }

                int maxcams = sizeOfStaticVector<int>((*ptsCams)[(*oldTris)[idTri].i[0]]) +
                              sizeOfStaticVector<int>((*ptsCams)[(*oldTris)[idTri].i[1]]) +
                              sizeOfStaticVector<int>((*ptsCams)[(*oldTris)[idTri].i[2]]);
                cams = new staticVector<int>(maxcams);
                for(int k = 0; k < 3; k++)
                {
                    for(int j = 0; j < sizeOfStaticVector<int>((*ptsCams)[(*oldTris)[idTri].i[k]]); j++)
                    {
                        cams->push_back_distinct((*(*ptsCams)[(*oldTris)[idTri].i[k]])[j]);
                    }
                }
                cams->shrink_to_fit();
            }
            ptsCams->push_back(cams);
        }

        if(ptsCams->size() != pts->size())
        {
            printf("WARNING different size!\n");
            exit(1);
        }

        delete newPtsOldTriId;
    }

    delete oldTris;
    delete trisCamsId;
}

int mv_mesh::subdivideMesh(const multiviewParams* mp, float maxTriArea, float maxEdgeLength,
                           bool useMaxTrisAreaOrAvEdgeLength, staticVector<staticVector<int>*>* trisCams,
                           staticVector<int>** trisCamsId)
{

    staticVector<staticVector<int>*>* edgesNeighTris;
    staticVector<pixel>* edgesPointsPairs;
    getNotOrientedEdges(&edgesNeighTris, &edgesPointsPairs);
    staticVector<voxel>* trisEdges = getTrianglesEdgesIds(edgesNeighTris);

    /*
    //check edges ID
    staticVector<staticVector<int>*> *trisNeighTris = getTrisNeighTris();
    for (int i=0;i<tris->size();i++) {
            bool subdivide = false;
            for (int j=0;j<sizeOfStaticVector<int>((*trisNeighTris)[i]);j++)
            {
                    int k = (*(*trisNeighTris)[i])[j];

                    int n=0;
                    for (int k1=0;k1<3;k1++) {
                            for (int k2=0;k2<3;k2++) {
                                    n+=(int)((*tris)[i].i[k1] == (*tris)[k].i[k2]);
                            };
                    };
                    if (n<2) {
                            printf("warning ... strange !!!\n");
                    };

                    staticVector<int> *ed = new staticVector<int>(6);
                    ed->push_back_distinct((*trisEdges)[i].x);
                    ed->push_back_distinct((*trisEdges)[i].y);
                    ed->push_back_distinct((*trisEdges)[i].z);
                    ed->push_back_distinct((*trisEdges)[k].x);
                    ed->push_back_distinct((*trisEdges)[k].y);
                    ed->push_back_distinct((*trisEdges)[k].z);
                    if (ed->size()==6) {
                            printf("warning ... not common egde !!!\n");
                    };
                    delete ed;
            };
    };
    deleteArrayOfArrays<int>(&trisNeighTris);
    */

    // which triangles should be subdivided
    int nTrisToSubdivide = 0;
    staticVectorBool* trisToSubdivide = new staticVectorBool(tris->size());
    for(int i = 0; i < tris->size(); i++)
    {
        int tcid = (*(*trisCamsId))[i];
        bool subdivide = false;

        if(useMaxTrisAreaOrAvEdgeLength)
        {
            for(int j = 0; j < sizeOfStaticVector<int>((*trisCams)[tcid]); j++)
            {
                int rc = (*(*trisCams)[tcid])[j];
                int w = mp->mip->getWidth(rc);
                int h = mp->mip->getHeight(rc);
                triangle_proj tp = getTriangleProjection(i, mp, rc, w, h);
                if(computeTriangleProjectionArea(tp) > maxTriArea)
                {
                    subdivide = true;
                }
            }
        }
        else
        {
            if(computeTriangleMaxEdgeLength(i) > maxEdgeLength)
            {
                subdivide = true;
            }
        }

        (*trisToSubdivide)[i] = subdivide;
        nTrisToSubdivide += static_cast<int>(subdivide);
    }

    // which edges are going to be subdivided
    staticVector<int>* edgesToSubdivide = new staticVector<int>(edgesNeighTris->size());
    for(int i = 0; i < edgesNeighTris->size(); i++)
    {
        bool hasNeigTriToSubdivide = false;
        for(int j = 0; j < (*edgesNeighTris)[i]->size(); j++)
        {
            int idTri = (*(*edgesNeighTris)[i])[j];
            if((*trisToSubdivide)[idTri])
            {
                hasNeigTriToSubdivide = true;
            }
        }
        edgesToSubdivide->push_back((int)(hasNeigTriToSubdivide)-1);
    }

    // assing id-s
    int id = pts->size();
    for(int i = 0; i < edgesToSubdivide->size(); i++)
    {
        if((*edgesToSubdivide)[i] > -1)
        {
            (*edgesToSubdivide)[i] = id;
            id++;
        }
    }
    int nEdgesToSubdivide = id - pts->size();

    // copy old pts
    staticVector<point3d>* pts1 = new staticVector<point3d>(pts->size() + nEdgesToSubdivide);
    for(int i = 0; i < pts->size(); i++)
    {
        pts1->push_back((*pts)[i]);
    }

    // add new pts ... middle of edge
    for(int i = 0; i < edgesToSubdivide->size(); i++)
    {
        if((*edgesToSubdivide)[i] > -1)
        {
            point3d p = ((*pts)[(*edgesPointsPairs)[i].x] + (*pts)[(*edgesPointsPairs)[i].y]) / 2.0f;
            pts1->push_back(p);
        }
    }

    // there might be needed to subdivide more triangles ... find them
    nTrisToSubdivide = 0;
    for(int i = 0; i < tris->size(); i++)
    {
        bool subdivide =
            (((*edgesToSubdivide)[(*trisEdges)[i].x] > -1) || ((*edgesToSubdivide)[(*trisEdges)[i].y] > -1) ||
             ((*edgesToSubdivide)[(*trisEdges)[i].z] > -1));
        (*trisToSubdivide)[i] = subdivide;
        nTrisToSubdivide += static_cast<int>(subdivide);
    }

    printf("number of triangles to subdivide %i\n", nTrisToSubdivide);
    printf("number of pts to add %i\n", nEdgesToSubdivide);

    staticVector<int>* trisCamsId1 = new staticVector<int>(tris->size() - nTrisToSubdivide + 4 * nTrisToSubdivide);
    staticVector<mv_mesh::triangle>* tris1 =
        new staticVector<mv_mesh::triangle>(tris->size() - nTrisToSubdivide + 4 * nTrisToSubdivide);
    for(int i = 0; i < tris->size(); i++)
    {
        if((*trisToSubdivide)[i])
        {
            pixel newPtsIds[3];
            newPtsIds[0].x = (*edgesToSubdivide)[(*trisEdges)[i].x]; // new pt id
            newPtsIds[0].y = (*trisEdges)[i].x;                      // edge id
            newPtsIds[1].x = (*edgesToSubdivide)[(*trisEdges)[i].y];
            newPtsIds[1].y = (*trisEdges)[i].y;
            newPtsIds[2].x = (*edgesToSubdivide)[(*trisEdges)[i].z];
            newPtsIds[2].y = (*trisEdges)[i].z;

            qsort(&newPtsIds[0], 3, sizeof(pixel), qSortComparePixelByXDesc);

            int n = 0;
            while((n < 3) && (newPtsIds[n].x > -1))
            {
                n++;
            }

            if(n == 0)
            {
                printf("warning subdivideMesh 1 !!!");
            }

            if(n == 1)
            {
                subdivideMeshCase1(i, edgesPointsPairs, newPtsIds[0], tris1);

                trisCamsId1->push_back((*(*trisCamsId))[i]);
                trisCamsId1->push_back((*(*trisCamsId))[i]);
            }

            if(n == 2)
            {
                subdivideMeshCase2(i, edgesPointsPairs, newPtsIds[0], newPtsIds[1], tris1);
                subdivideMeshCase2(i, edgesPointsPairs, newPtsIds[1], newPtsIds[0], tris1);

                trisCamsId1->push_back((*(*trisCamsId))[i]);
                trisCamsId1->push_back((*(*trisCamsId))[i]);
                trisCamsId1->push_back((*(*trisCamsId))[i]);
            }

            if(n == 3)
            {
                subdivideMeshCase3(i, edgesPointsPairs, newPtsIds[0], newPtsIds[1], newPtsIds[2], tris1);
                subdivideMeshCase3(i, edgesPointsPairs, newPtsIds[0], newPtsIds[2], newPtsIds[1], tris1);
                subdivideMeshCase3(i, edgesPointsPairs, newPtsIds[1], newPtsIds[0], newPtsIds[2], tris1);
                subdivideMeshCase3(i, edgesPointsPairs, newPtsIds[1], newPtsIds[2], newPtsIds[0], tris1);
                subdivideMeshCase3(i, edgesPointsPairs, newPtsIds[2], newPtsIds[0], newPtsIds[1], tris1);
                subdivideMeshCase3(i, edgesPointsPairs, newPtsIds[2], newPtsIds[1], newPtsIds[0], tris1);

                trisCamsId1->push_back((*(*trisCamsId))[i]);
                trisCamsId1->push_back((*(*trisCamsId))[i]);
                trisCamsId1->push_back((*(*trisCamsId))[i]);
                trisCamsId1->push_back((*(*trisCamsId))[i]);
            }
        }
        else
        {
            tris1->push_back((*tris)[i]);
            trisCamsId1->push_back((*(*trisCamsId))[i]);
        }
    }

    delete pts;
    delete tris;
    pts = pts1;
    tris = tris1;

    delete(*trisCamsId);
    (*trisCamsId) = trisCamsId1;

    deleteArrayOfArrays<int>(&edgesNeighTris);
    delete edgesPointsPairs;
    delete trisEdges;
    delete trisToSubdivide;
    delete edgesToSubdivide;

    return nTrisToSubdivide;
}

void mv_mesh::cutout(const pixel& lu, const pixel& rd, int rc, const multiviewParams* mp,
                     staticVector<staticVector<int>*>* ptsCams)
{
    staticVectorBool* bpts = new staticVectorBool(pts->size());
    bpts->resize_with(pts->size(), false);

    staticVector<mv_mesh::triangle>* tris1 = new staticVector<mv_mesh::triangle>(tris->size());

    for(int i = 0; i < pts->size(); i++)
    {
        pixel pix;
        mp->getPixelFor3DPoint(&pix, (*pts)[i], rc);
        (*bpts)[i] = mp->isPixelInCutOut(&pix, &lu, &rd, 1, rc);
    }

    for(int i = 0; i < tris->size(); i++)
    {
        if((*bpts)[(*tris)[i].i[0]] && (*bpts)[(*tris)[i].i[1]] && (*bpts)[(*tris)[i].i[2]])
        {
            tris1->push_back((*tris)[i]);
        }
    }

    delete tris;
    tris = tris1;

    staticVector<int>* ptIdToNewPtId;
    removeFreePointsFromMesh(&ptIdToNewPtId);

    staticVector<staticVector<int>*>* ptsCamsNew = new staticVector<staticVector<int>*>(pts->size());
    ptsCamsNew->resize_with(pts->size(), nullptr);
    for(int i = 0; i < ptsCams->size(); i++)
    {
        if((*ptIdToNewPtId)[i] == -1)
        {
            delete(*ptsCams)[i];
        }
        else
        {
            (*ptsCamsNew)[(*ptIdToNewPtId)[i]] = (*ptsCams)[i];
        }
        (*ptsCams)[i] = nullptr;
    }

    ptsCams->resize(ptsCamsNew->size());
    for(int i = 0; i < ptsCamsNew->size(); i++)
    {
        (*ptsCams)[i] = (*ptsCamsNew)[i];
        (*ptsCamsNew)[i] = nullptr;
    }

    delete ptIdToNewPtId;
    delete ptsCamsNew;
}

void mv_mesh::cutout(const pixel& lu, const pixel& rd, int rc, const multiviewParams* mp)
{
    staticVectorBool* bpts = new staticVectorBool(pts->size());
    bpts->resize_with(pts->size(), false);

    staticVector<mv_mesh::triangle>* tris1 = new staticVector<mv_mesh::triangle>(tris->size());

    for(int i = 0; i < pts->size(); i++)
    {
        pixel pix;
        mp->getPixelFor3DPoint(&pix, (*pts)[i], rc);
        (*bpts)[i] = mp->isPixelInCutOut(&pix, &lu, &rd, 1, rc);
    }

    for(int i = 0; i < tris->size(); i++)
    {
        if((*bpts)[(*tris)[i].i[0]] && (*bpts)[(*tris)[i].i[1]] && (*bpts)[(*tris)[i].i[2]])
        {
            tris1->push_back((*tris)[i]);
        }
    }

    delete tris;
    tris = tris1;

    removeFreePointsFromMesh(nullptr);
}

float mv_mesh::computeAverageEdgeLength() const
{
    /*
    staticVector<staticVector<int>*> *edgesNeighTris;
    staticVector<pixel> *edgesPointsPairs;
    getNotOrientedEdges(&edgesNeighTris, &edgesPointsPairs);

    float s=0.0f;
    float n=0.0f;
    for (int i=0;i<edgesPointsPairs->size();i++) {
            s += ((*pts)[(*edgesPointsPairs)[i].x]-(*pts)[(*edgesPointsPairs)[i].y]).size();
            n += 1.0f;
    };

    deleteArrayOfArrays<int>(&edgesNeighTris);
    delete edgesPointsPairs;

    if (n==0.0f) {
            return 0.0f;
    }else{
            return (s/n);
    };
    */

    float s = 0.0f;
    float n = 0.0f;
    for(int i = 0; i < tris->size(); i++)
    {
        s += computeTriangleMaxEdgeLength(i);
        n += 1.0f;
    }
    if(n == 0.0f)
    {
        return 0.0f;
    }

    return (s / n);
}

bool mv_mesh::isPointVisibleInRcAndTc(int idPt, staticVector<staticVector<int>*>* ptsCams, int rc, int tc,
                                      const multiviewParams*  /*mp*/)
{
    bool isVisibleInRc = false;
    bool isVisibleInTc = false;
    for(int j = 0; j < sizeOfStaticVector<int>((*ptsCams)[idPt]); j++)
    {
        if((*(*ptsCams)[idPt])[j] == rc)
        {
            isVisibleInRc = true;
        }
        if((*(*ptsCams)[idPt])[j] == tc)
        {
            isVisibleInTc = true;
        }
    }

    return (isVisibleInRc && isVisibleInTc);
}

bool mv_mesh::isTriangleVisibleInRcAndTc(int idTri, staticVector<staticVector<int>*>* ptsCams, int rc, int tc,
                                         const multiviewParams* mp)
{
    return (isPointVisibleInRcAndTc((*tris)[idTri].i[0], ptsCams, rc, tc, mp) &&
            isPointVisibleInRcAndTc((*tris)[idTri].i[1], ptsCams, rc, tc, mp) &&
            isPointVisibleInRcAndTc((*tris)[idTri].i[2], ptsCams, rc, tc, mp));
}

void mv_mesh::getTrianglesIndexesForRcTc(staticVector<int>** trisRcTc, staticVector<staticVector<int>*>* ptsCams,
                                         int rc, int tc, const multiviewParams* mp)
{
    // precompute number of triangles
    int ntris = 0;
    for(int i = 0; i < tris->size(); i++)
    {
        if(isTriangleVisibleInRcAndTc(i, ptsCams, rc, tc, mp))
        {
            ntris++;
        }
    }

    // allocate
    (*trisRcTc) = new staticVector<int>(ntris);

    // fill
    for(int i = 0; i < tris->size(); i++)
    {
        if(isTriangleVisibleInRcAndTc(i, ptsCams, rc, tc, mp))
        {
            (*trisRcTc)->push_back(i);
        }
    }
}

void mv_mesh::letJustTringlesIdsInMesh(staticVector<int>* trisIdsToStay)
{
    // printf("letJustTringlesIdsInMesh %i\n",trisIdsToStay->size());

    staticVector<mv_mesh::triangle>* trisTmp = new staticVector<mv_mesh::triangle>(trisIdsToStay->size());
    for(int i = 0; i < trisIdsToStay->size(); i++)
    {
        trisTmp->push_back((*tris)[(*trisIdsToStay)[i]]);
    }

    delete tris;
    tris = trisTmp;
}

void mv_mesh::removeDepthMaps(const multiviewParams* mp, std::string tmpDir)
{
    if(mp->verbose)
        printf("removing depth maps\n");

    long t1 = initEstimate();
    for(int rc = 0; rc < mp->ncams; rc++)
    {
        std::string fn;
        fn = tmpDir + "depthMap" + num2strFourDecimal(rc) + ".png";
        remove(fn.c_str());
        fn = tmpDir + "depthMap" + num2strFourDecimal(rc) + ".bin";
        remove(fn.c_str());
        fn = tmpDir + "depthMap" + num2strFourDecimal(rc) + "T.bin";
        remove(fn.c_str());
        fn = tmpDir + "visTris" + num2strFourDecimal(rc) + ".bin";
        remove(fn.c_str());
        fn = tmpDir + "trisIdsMap" + num2strFourDecimal(rc) + ".bin";
        remove(fn.c_str());
        fn = tmpDir + "trisIdsMap" + num2strFourDecimal(rc) + ".png";
        remove(fn.c_str());

        printfEstimate(rc, mp->ncams, t1);
    }
    finishEstimate();
}

staticVector<staticVector<int>*>* mv_mesh::computeTrisCams(const multiviewParams* mp, std::string tmpDir)
{
    if(mp->verbose)
        printf("computing tris cams\n");

    staticVector<int>* ntrisCams = new staticVector<int>(tris->size());
    ntrisCams->resize_with(tris->size(), 0);

    long t1 = initEstimate();
    for(int rc = 0; rc < mp->ncams; rc++)
    {
        std::string visTrisFileName = tmpDir + "visTris" + num2strFourDecimal(rc) + ".bin";
        staticVector<int>* visTris = loadArrayFromFile<int>(visTrisFileName);
        if(visTris != nullptr)
        {
            for(int i = 0; i < visTris->size(); i++)
            {
                int idTri = (*visTris)[i];
                (*ntrisCams)[idTri]++;
            }
            delete visTris;
        }
        printfEstimate(rc, mp->ncams, t1);
    }
    finishEstimate();

    staticVector<staticVector<int>*>* trisCams = new staticVector<staticVector<int>*>(tris->size());
    for(int i = 0; i < tris->size(); i++)
    {
        staticVector<int>* cams = nullptr;
        if((*ntrisCams)[i] > 0)
        {
            cams = new staticVector<int>((*ntrisCams)[i]);
        }
        trisCams->push_back(cams);
    }

    t1 = initEstimate();
    for(int rc = 0; rc < mp->ncams; rc++)
    {
        std::string visTrisFileName = tmpDir + "visTris" + num2strFourDecimal(rc) + ".bin";
        staticVector<int>* visTris = loadArrayFromFile<int>(visTrisFileName);
        if(visTris != nullptr)
        {
            for(int i = 0; i < visTris->size(); i++)
            {
                int idTri = (*visTris)[i];
                (*trisCams)[idTri]->push_back(rc);
            }
            delete visTris;
        }
        printfEstimate(rc, mp->ncams, t1);
    }
    finishEstimate();

    delete ntrisCams;

    if(mp->verbose)
        printf("done\n");

    return trisCams;
}

staticVector<staticVector<int>*>* mv_mesh::computeTrisCamsFromPtsCams(staticVector<staticVector<int>*>* ptsCams) const
{
    //std::cout << "computeTrisCamsFromPtsCams" << std::endl;
    // TODO: try intersection
    staticVector<staticVector<int>*>* trisCams = new staticVector<staticVector<int>*>(tris->size());
    for(int idTri = 0; idTri < tris->size(); idTri++)
    {
        int maxcams = sizeOfStaticVector<int>((*ptsCams)[(*tris)[idTri].i[0]]) +
                      sizeOfStaticVector<int>((*ptsCams)[(*tris)[idTri].i[1]]) +
                      sizeOfStaticVector<int>((*ptsCams)[(*tris)[idTri].i[2]]);
        staticVector<int>* cams = new staticVector<int>(maxcams);
        for(int k = 0; k < 3; k++)
        {
            for(int i = 0; i < sizeOfStaticVector<int>((*ptsCams)[(*tris)[idTri].i[k]]); i++)
            {
                cams->push_back_distinct((*(*ptsCams)[(*tris)[idTri].i[k]])[i]);
            }
        }
        trisCams->push_back(cams);
    }

    //std::cout << "computeTrisCamsFromPtsCams end" << std::endl;
    return trisCams;
}

staticVector<staticVector<int>*>* mv_mesh::computePtsCamsFromTrisCams(staticVector<staticVector<int>*>* trisCams)
{
    staticVector<staticVector<int>*>* ptsCams = new staticVector<staticVector<int>*>(pts->size());
    ptsCams->resize_with(pts->size(), nullptr);

    for(int idTri = 0; idTri < tris->size(); idTri++)
    {
        for(int k = 0; k < 3; k++)
        {
            staticVector<int>* ptcams = (*ptsCams)[(*tris)[idTri].i[k]];
            if(ptcams == nullptr)
            {
                ptcams = new staticVector<int>(sizeOfStaticVector<int>((*trisCams)[idTri]) * 2);
            }
            else
            {
                if(ptcams->size() + sizeOfStaticVector<int>((*trisCams)[idTri]) > ptcams->capacity())
                {
                    ptcams->resizeAdd(sizeOfStaticVector<int>((*trisCams)[idTri]));
                }
            }
            for(int c = 0; c < sizeOfStaticVector<int>((*trisCams)[idTri]); c++)
            {
                ptcams->push_back_distinct((*(*trisCams)[idTri])[c]);
            }
            (*ptsCams)[(*tris)[idTri].i[k]] = ptcams;
        }
    }

    return ptsCams;
}

staticVector<staticVector<int>*>* mv_mesh::computePtsCams(const multiviewParams* mp, std::string tmpDir)
{
    printf("computing pts cams\n");

    staticVector<staticVector<int>*>* trisCams = computeTrisCams(mp, tmpDir);
    staticVector<staticVector<int>*>* ptsNeighTris = getPtsNeighTris();

    staticVector<staticVector<int>*>* ptsCams = new staticVector<staticVector<int>*>(pts->size());

    long t1 = initEstimate();
    for(int i = 0; i < pts->size(); i++)
    {
        int maxCams = 0;
        for(int j = 0; j < sizeOfStaticVector<int>((*ptsNeighTris)[i]); j++)
        {
            int idTri = (*(*ptsNeighTris)[i])[j];
            maxCams += sizeOfStaticVector<int>((*trisCams)[idTri]);
        }
        staticVector<int>* cams = new staticVector<int>(maxCams);
        for(int j = 0; j < sizeOfStaticVector<int>((*ptsNeighTris)[i]); j++)
        {
            int idTri = (*(*ptsNeighTris)[i])[j];
            int ncams = sizeOfStaticVector<int>((*trisCams)[idTri]);
            for(int c = 0; c < ncams; c++)
            {
                cams->push_back_distinct((*(*trisCams)[idTri])[c]);
            }
        }
        ptsCams->push_back(cams);
        printfEstimate(i, pts->size(), t1);
    }
    finishEstimate();

    deleteArrayOfArrays<int>(&trisCams);
    deleteArrayOfArrays<int>(&ptsNeighTris);

    return ptsCams;
}

void mv_mesh::initFromDepthMap(const multiviewParams* mp, staticVector<float>* depthMap, int rc, int scale, float alpha)
{
    initFromDepthMap(mp, &(*depthMap)[0], rc, scale, 1, alpha);
}
void mv_mesh::initFromDepthMapT(const multiviewParams* mp, float* depthMap, int rc, int scale, int step, float alpha)
{
    int w = mp->mip->getWidth(rc) / (scale * step);
    int h = mp->mip->getHeight(rc) / (scale * step);

    float* depthMapT = new float[w * h];

    for(int x = 0; x < w; x++)
    {
        for(int y = 0; y < h; y++)
        {
            depthMapT[x * h + y] = depthMap[y * w + x];
        }
    }

    initFromDepthMap(mp, depthMapT, rc, scale, step, alpha);

    delete[] depthMapT;
}

void mv_mesh::initFromDepthMap(const multiviewParams* mp, float* depthMap, int rc, int scale, int step, float alpha)
{
    initFromDepthMap(1, mp, depthMap, rc, scale, step, alpha);
}

void mv_mesh::initFromDepthMap(int stepDetail, const multiviewParams* mp, float* depthMap, int rc, int scale, int step,
                               float alpha)
{
    int w = mp->mip->getWidth(rc) / (scale * step);
    int h = mp->mip->getHeight(rc) / (scale * step);

    pts = new staticVector<point3d>(w * h);
    staticVectorBool* usedMap = new staticVectorBool(w * h);
    for(int i = 0; i < w * h; i++)
    {
        int x = i / h;
        int y = i % h;
        float depth = depthMap[i];
        if(depth > 0.0f)
        {
            point3d p = mp->CArr[rc] +
                        (mp->iCamArr[rc] * point2d((float)x * (float)(scale * step), (float)y * (float)(scale * step)))
                                .normalize() *
                            depth;
            pts->push_back(p);
            usedMap->push_back(true);
        }
        else
        {
            pts->push_back(point3d(0.0f, 0.0f, 0.0f));
            usedMap->push_back(false);
        }
    }

    tris = new staticVector<mv_mesh::triangle>(w * h * 2);
    for(int x = 0; x < w - 1 - stepDetail; x += stepDetail)
    {
        for(int y = 0; y < h - 1 - stepDetail; y += stepDetail)
        {
            point3d p1 = (*pts)[x * h + y];
            point3d p2 = (*pts)[(x + stepDetail) * h + y];
            point3d p3 = (*pts)[(x + stepDetail) * h + y + stepDetail];
            point3d p4 = (*pts)[x * h + y + stepDetail];

            if((*usedMap)[x * h + y] && (*usedMap)[(x + stepDetail) * h + y] &&
               (*usedMap)[(x + stepDetail) * h + y + stepDetail] && (*usedMap)[x * h + y + stepDetail])
            {
                float d = mp->getCamPixelSize(p1, rc, alpha);
                if(((p1 - p2).size() < d) && ((p1 - p3).size() < d) && ((p1 - p4).size() < d) &&
                   ((p2 - p3).size() < d) && ((p3 - p4).size() < d))
                {
                    mv_mesh::triangle t;
                    t.alive = true;
                    t.i[2] = x * h + y;
                    t.i[1] = (x + stepDetail) * h + y;
                    t.i[0] = x * h + y + stepDetail;
                    tris->push_back(t);

                    t.alive = true;
                    t.i[2] = (x + stepDetail) * h + y;
                    t.i[1] = (x + stepDetail) * h + y + stepDetail;
                    t.i[0] = x * h + y + stepDetail;
                    tris->push_back(t);
                }
            }
        }
    }

    delete usedMap;

    removeFreePointsFromMesh(nullptr);
}

void mv_mesh::removeTrianglesInHexahedrons(staticVector<point3d>* hexahsToExcludeFromResultingMesh)
{
    if(hexahsToExcludeFromResultingMesh != nullptr)
    {
        printf("removeTrianglesInHexahedrons %i %i \n", tris->size(),
               (int)(hexahsToExcludeFromResultingMesh->size() / 8));
        staticVector<int>* trisIdsToStay = new staticVector<int>(tris->size());

        long t1 = initEstimate();
        for(int i = 0; i < tris->size(); i++)
        {
            int nin = 0;
            for(int k = 0; k < 3; k++)
            {
                point3d p = (*pts)[(*tris)[i].i[k]];
                bool isThere = false;
                for(int j = 0; j < (int)(hexahsToExcludeFromResultingMesh->size() / 8); j++)
                {
                    if(isPointInHexahedron(p, &(*hexahsToExcludeFromResultingMesh)[j * 8]))
                    {
                        isThere = true;
                    }
                }
                if(isThere)
                {
                    nin++;
                }
            }
            if(nin < 3)
            {
                trisIdsToStay->push_back(i);
            }
            printfEstimate(i, tris->size(), t1);
        }
        finishEstimate();

        letJustTringlesIdsInMesh(trisIdsToStay);

        delete trisIdsToStay;
    }
}

void mv_mesh::removeTrianglesOutsideHexahedron(point3d* hexah)
{
    printf("removeTrianglesOutsideHexahedrons %i \n", tris->size());
    staticVector<int>* trisIdsToStay = new staticVector<int>(tris->size());

    long t1 = initEstimate();
    for(int i = 0; i < tris->size(); i++)
    {
        int nout = 0;
        for(int k = 0; k < 3; k++)
        {
            point3d p = (*pts)[(*tris)[i].i[k]];
            bool isThere = false;
            if(!isPointInHexahedron(p, hexah))
            {
                isThere = true;
            }
            if(isThere)
            {
                nout++;
            }
        }
        if(nout < 1)
        {
            trisIdsToStay->push_back(i);
        }
        printfEstimate(i, tris->size(), t1);
    }
    finishEstimate();

    letJustTringlesIdsInMesh(trisIdsToStay);

    delete trisIdsToStay;
}

staticVector<int>* mv_mesh::getNearPointsIds(int idPt, int maxLevel, staticVector<staticVector<int>*>* ptsNeighPts)
{
    staticVectorBool* processed = new staticVectorBool(pts->size());
    processed->resize_with(pts->size(), false);

    staticVector<pixel>* toProcess = new staticVector<pixel>(pts->size());
    toProcess->push_back(pixel(idPt, 0));

    staticVector<int>* out = new staticVector<int>(pts->size());

    while(toProcess->size() > 0)
    {
        pixel val = toProcess->pop();
        int id = val.x;
        int level = val.y;
        (*processed)[id] = true;
        out->push_back(id);

        staticVector<int>* neighs = (*ptsNeighPts)[id];
        for(int i = 0; i < sizeOfStaticVector<int>(neighs); i++)
        {
            int nid = (*neighs)[i];
            if((level < maxLevel) && (!(*processed)[nid]))
            {
                toProcess->push_back(pixel(nid, level + 1));
            }
        }
    }

    delete processed;
    delete toProcess;

    return out;
}

void mv_mesh::filterLargeEdgeTriangles(float maxEdgelengthThr)
{
    float averageEdgeLength = computeAverageEdgeLength();
    float avelthr = maxEdgelengthThr;

    staticVector<int>* trisIdsToStay = new staticVector<int>(tris->size());
    for(int i = 0; i < tris->size(); i++)
    {
        float triMaxEdgelength = computeTriangleMaxEdgeLength(i);
        if(triMaxEdgelength < averageEdgeLength * avelthr)
        {
            trisIdsToStay->push_back(i);
        }
    }
    letJustTringlesIdsInMesh(trisIdsToStay);
    delete trisIdsToStay;
}

void mv_mesh::hideLargeEdgeTriangles(float maxEdgelengthThr)
{
    float averageEdgeLength = computeAverageEdgeLength();
    float avelthr = maxEdgelengthThr;

    for(int i = 0; i < tris->size(); i++)
    {
        float triMaxEdgelength = computeTriangleMaxEdgeLength(i);
        if(triMaxEdgelength < averageEdgeLength * avelthr)
        {
            (*tris)[i].alive = true;
        }
        else
        {
            (*tris)[i].alive = false;
        }
    }
}

staticVector<int>* mv_mesh::getUsedCams(const multiviewParams* mp, staticVector<staticVector<int>*>* trisCams)
{
    staticVector<int>* usedcams = new staticVector<int>(mp->ncams);
    for(int i = 0; i < trisCams->size(); i++)
    {
        for(int c = 0; c < sizeOfStaticVector<int>((*trisCams)[i]); c++)
        {
            usedcams->push_back_distinct((*(*trisCams)[i])[c]);
        }
    }
    return usedcams;
}

staticVector<int>* mv_mesh::getTargetCams(int rc, const multiviewParams* mp, staticVector<staticVector<int>*>* trisCams)
{
    staticVector<int>* tcams = new staticVector<int>(mp->ncams);
    for(int i = 0; i < trisCams->size(); i++)
    {
        if(((*trisCams)[i] != nullptr) && ((*trisCams)[i]->indexOf(rc) > -1))
        {
            for(int c = 0; c < sizeOfStaticVector<int>((*trisCams)[i]); c++)
            {
                if((*(*trisCams)[i])[c] != rc)
                {
                    tcams->push_back_distinct((*(*trisCams)[i])[c]);
                }
            }
        }
    }
    return tcams;
}

float mv_mesh::getMaxAdjFacesAngle(int idpt, staticVector<staticVector<int>*>* ptsNeighTris,
                                   staticVector<staticVector<int>*>* trisNeighTris)
{
    staticVector<int>* ptNeighTris = (*ptsNeighTris)[idpt];
    if(ptNeighTris == nullptr)
    {
        return -1.0f;
    }

    int maxntris = ptNeighTris->size();
    for(int i = 0; i < ptNeighTris->size(); i++)
    {
        int triid = (*ptNeighTris)[i];
        maxntris += sizeOfStaticVector<int>((*trisNeighTris)[triid]);
    }

    staticVector<voxel>* edges = new staticVector<voxel>(maxntris * 3);

    for(int i = 0; i < ptNeighTris->size(); i++)
    {
        int triid = (*ptNeighTris)[i];
        int a = (*tris)[triid].i[0];
        int b = (*tris)[triid].i[1];
        int c = (*tris)[triid].i[2];
        edges->push_back(voxel(std::min(a, b), std::max(a, b), triid));
        edges->push_back(voxel(std::min(b, c), std::max(b, c), triid));
        edges->push_back(voxel(std::min(c, a), std::max(c, a), triid));

        staticVector<int>* triNeighTris = (*trisNeighTris)[triid];
        for(int j = 0; j < sizeOfStaticVector<int>(triNeighTris); j++)
        {
            triid = (*triNeighTris)[j];
            a = (*tris)[triid].i[0];
            b = (*tris)[triid].i[1];
            c = (*tris)[triid].i[2];
            edges->push_back(voxel(std::min(a, b), std::max(a, b), triid));
            edges->push_back(voxel(std::min(b, c), std::max(b, c), triid));
            edges->push_back(voxel(std::min(c, a), std::max(c, a), triid));
        }
    }

    qsort(&(*edges)[0], edges->size(), sizeof(voxel), qSortCompareVoxelByXAsc);

    float maxAng = 0.0f;

    int i0 = 0;
    for(int i = 0; i < edges->size(); i++)
    {
        if((i == edges->size() - 1) || ((*edges)[i].x != (*edges)[i + 1].x))
        {
            staticVector<voxel>* edges1 = new staticVector<voxel>(i - i0 + 1);
            for(int j = i0; j <= i; j++)
            {
                edges1->push_back((*edges)[j]);
            }
            qsort(&(*edges1)[0], edges1->size(), sizeof(voxel), qSortCompareVoxelByYAsc);

            int j0 = 0;
            for(int j = 0; j < edges1->size(); j++)
            {
                if((j == edges1->size() - 1) || ((*edges1)[j].y != (*edges1)[j + 1].y))
                {
                    //_edgesPointsPairs->push_back(pixel((*edges1)[j].x,(*edges1)[j].y));
                    staticVector<int>* neighTris = new staticVector<int>(j - j0 + 1);
                    for(int k = j0; k <= j; k++)
                    {
                        neighTris->push_back((*edges1)[k].z);
                    }
                    qsort(&(*neighTris)[0], neighTris->size(), sizeof(int), qSortCompareIntAsc);

                    staticVector<int>* neighTrisDist = new staticVector<int>(j - j0 + 1);
                    for(int k = 0; k < neighTris->size(); k++)
                    {
                        if((k == neighTris->size() - 1) || ((*neighTris)[k] != (*neighTris)[k + 1]))
                        {
                            neighTrisDist->push_back((*neighTris)[k]);
                        }
                    }

                    if(neighTrisDist->size() > 2)
                    {
                        delete neighTrisDist;
                        delete neighTris;
                        delete edges1;
                        delete edges;
                        return -1.0f;
                    }

                    if(neighTrisDist->size() == 2)
                    {
                        point3d n1 = computeTriangleNormal((*neighTrisDist)[0]);
                        point3d n2 = computeTriangleNormal((*neighTrisDist)[1]);
                        float ang = angleBetwV1andV2(n1, n2);
                        if(!std::isnan(ang))
                        {
                            maxAng = std::max(maxAng, ang);
                        }
                    }

                    delete neighTrisDist;
                    delete neighTris;
                    j0 = j + 1;
                }
            }
            i0 = i + 1;

            delete edges1;
        }
    }

    delete edges;

    return maxAng;
}

float mv_mesh::getCurvatureAngle(int idpt, staticVector<staticVector<int>*>* ptsNeighPts)
{

    point3d p = (*pts)[idpt];
    staticVector<int>* nei = (*ptsNeighPts)[idpt];
    int nneighs = 0;
    if(nei != nullptr)
    {
        nneighs = nei->size();
    }

    if(nneighs == 0)
    {
        return 0.0f;
    }

    point3d n = point3d(0.0f, 0.0f, 0.0f);
    for(int j = 0; j < nneighs; j++)
    {
        n = n + (*pts)[(*nei)[j]];
    }
    n = ((n / (float)nneighs) - p).normalize();

    if(std::isnan(n.x) || std::isnan(n.y) || std::isnan(n.z) || (n.x != n.x) || (n.y != n.y) ||
       (n.z != n.z)) // check if is not NaN
    {
        return 0.0f;
    }

    float a = 0.0f;
    for(int j = 0; j < nneighs; j++)
    {
        point3d v = ((*pts)[(*nei)[j]] - p).normalize();
        if(std::isnan(v.x) || std::isnan(v.y) || std::isnan(v.z) || (v.x != v.x) || (v.y != v.y) ||
           (v.z != v.z)) // check if is not NaN
        {
            //
        }
        else
        {
            a = std::max(a, (float)angleBetwV1andV2(v, n));
        }
    }

    return a;
}

void mv_mesh::invertTriangleOrientations()
{
    for(int i = 0; i < tris->size(); i++)
    {
        int tmp = (*tris)[i].i[1];
        (*tris)[i].i[1] = (*tris)[i].i[2];
        (*tris)[i].i[2] = tmp;
    }
}

mv_mesh* createMeshForCameras(staticVector<int>* tcams, const multiviewParams* mp, float ps, int shift, int step,
                              float  /*upStep*/)
{
    mv_mesh* me = new mv_mesh();
    me->pts = new staticVector<point3d>(tcams->size() * 5);
    me->tris = new staticVector<mv_mesh::triangle>(tcams->size() * 6);

    for(int c = shift; c < tcams->size(); c += step)
    {
        int rc = (*tcams)[c];
        QVector pos, at, up;
        QVector rot_axis;
        float rot_angle;
        float viewsim;

        point3d viewp;
        point3d viewn;
        viewp = mp->CArr[rc];
        point3d Cl = viewp;

        point3d Xl = mp->RArr[rc].transpose() * point3d(1.0, 0.0, 0.0) * mp->KArr[rc].m13;
        Xl = Xl.normalize();
        point3d Yl = mp->RArr[rc].transpose() * point3d(0.0, 1.0, 0.0) * mp->KArr[rc].m23;
        Yl = Yl.normalize();
        point3d Zl = mp->RArr[rc].transpose() * point3d(0.0, 0.0, 1.0) * mp->KArr[rc].m11;
        Zl = Zl.normalize();

        point3d Ol = Cl + Zl;
        point3d Vl = Yl;
        Vl.x = -Vl.x;
        Vl.y = -Vl.y;
        Vl.z = -Vl.z;

        pos.x = Cl.x;
        pos.y = Cl.y;
        pos.z = Cl.z;
        at.x = Ol.x;
        at.y = Ol.y;
        at.z = Ol.z;
        up.x = Vl.x;
        up.y = Vl.y;
        up.z = Vl.z;

        Convert_Camera_Model(&pos, &at, &up, &rot_axis, &rot_angle);

        viewn.x = rot_axis.x;
        viewn.y = rot_axis.y;
        viewn.z = rot_axis.z;
        viewsim = rot_angle;

        point3d vx, vy;
        point3d n, p;
        n.x = Zl.x;
        n.y = Zl.y;
        n.z = Zl.z;
        n = n.normalize();

        p.x = viewp.x;
        p.y = viewp.y;
        p.z = viewp.z;

        vx = Xl;
        vy = Yl;

        me->pts->push_back(p);
        me->pts->push_back(p + (n - vx - vy) * ps);
        me->pts->push_back(p + (n - vx + vy) * ps);
        me->pts->push_back(p + (n + vx + vy) * ps);
        me->pts->push_back(p + (n + vx - vy) * ps);
    }

    int id = 0;
    for(int c = shift; c < tcams->size(); c += step)
    {
        mv_mesh::triangle t;
        t.i[0] = id * 5;
        t.i[1] = id * 5 + 1;
        t.i[2] = id * 5 + 2;
        me->tris->push_back(t);
        t.i[0] = id * 5;
        t.i[1] = id * 5 + 2;
        t.i[2] = id * 5 + 3;
        me->tris->push_back(t);
        t.i[0] = id * 5;
        t.i[1] = id * 5 + 3;
        t.i[2] = id * 5 + 4;
        me->tris->push_back(t);
        t.i[0] = id * 5;
        t.i[1] = id * 5 + 4;
        t.i[2] = id * 5 + 1;
        me->tris->push_back(t);
        t.i[0] = id * 5 + 1;
        t.i[1] = id * 5 + 3;
        t.i[2] = id * 5 + 2;
        me->tris->push_back(t);
        t.i[0] = id * 5 + 3;
        t.i[1] = id * 5 + 1;
        t.i[2] = id * 5 + 4;
        me->tris->push_back(t);

        id++;
    }

    return me;
}

mv_mesh* createMeshForCameras4(staticVector<matrix3x4>* cams, const multiviewParams* mp, float ps)
{
    mv_mesh* me = new mv_mesh();
    me->pts = new staticVector<point3d>(cams->size() * 5);
    me->tris = new staticVector<mv_mesh::triangle>(cams->size() * 4);

    for(int c = 0; c < cams->size(); c++)
    {
        matrix3x4 P = (*cams)[c];

        point3d Co;
        matrix3x3 Ro;
        matrix3x3 iRo;
        matrix3x3 Ko;
        matrix3x3 iKo;
        matrix3x3 iPo;
        mp->decomposeProjectionMatrix(Co, Ro, iRo, Ko, iKo, iPo, P);

        QVector pos, at, up;
        QVector rot_axis;
        float rot_angle;
        float viewsim;

        point3d viewp;
        point3d viewn;
        viewp = Co;
        point3d Cl = viewp;

        point3d Xl = Ro.transpose() * point3d(1.0, 0.0, 0.0) * Ko.m13;
        Xl = Xl.normalize();
        point3d Yl = Ro.transpose() * point3d(0.0, 1.0, 0.0) * Ko.m23;
        Yl = Yl.normalize();
        point3d Zl = Ro.transpose() * point3d(0.0, 0.0, 1.0) * Ko.m11;
        Zl = Zl.normalize();

        point3d Ol = Cl + Zl;
        point3d Vl = Yl;
        Vl.x = -Vl.x;
        Vl.y = -Vl.y;
        Vl.z = -Vl.z;

        pos.x = Cl.x;
        pos.y = Cl.y;
        pos.z = Cl.z;
        at.x = Ol.x;
        at.y = Ol.y;
        at.z = Ol.z;
        up.x = Vl.x;
        up.y = Vl.y;
        up.z = Vl.z;

        Convert_Camera_Model(&pos, &at, &up, &rot_axis, &rot_angle);

        viewn.x = rot_axis.x;
        viewn.y = rot_axis.y;
        viewn.z = rot_axis.z;
        viewsim = rot_angle;

        point3d vx, vy;
        point3d n, p;
        n.x = Zl.x;
        n.y = Zl.y;
        n.z = Zl.z;
        n = n.normalize();

        p.x = viewp.x;
        p.y = viewp.y;
        p.z = viewp.z;

        vx = Xl;
        vy = Yl;

        me->pts->push_back(p);
        me->pts->push_back(p + (n - vx - vy) * ps);
        me->pts->push_back(p + (n - vx + vy) * ps);
        me->pts->push_back(p + (n + vx + vy) * ps);
        me->pts->push_back(p + (n + vx - vy) * ps);
    }

    int id = 0;
    for(int c = 0; c < cams->size(); c++)
    {
        mv_mesh::triangle t;
        t.i[0] = id * 5;
        t.i[1] = id * 5 + 1;
        t.i[2] = id * 5 + 2;
        me->tris->push_back(t);
        t.i[0] = id * 5;
        t.i[1] = id * 5 + 2;
        t.i[2] = id * 5 + 3;
        me->tris->push_back(t);
        t.i[0] = id * 5;
        t.i[1] = id * 5 + 3;
        t.i[2] = id * 5 + 4;
        me->tris->push_back(t);
        t.i[0] = id * 5;
        t.i[1] = id * 5 + 4;
        t.i[2] = id * 5 + 1;
        me->tris->push_back(t);
        id++;
    }

    return me;
}

mv_mesh* createMeshForCameras4(staticVector<int>* tcams, const multiviewParams* mp, float ps, int shift, int step,
                               float  /*upStep*/)
{
    staticVector<matrix3x4>* cams = new staticVector<matrix3x4>(tcams->size());

    for(int c = shift; c < tcams->size(); c += step)
    {
        int rc = (*tcams)[c];
        cams->push_back(mp->camArr[rc]);
    }

    mv_mesh* me = createMeshForCameras4(cams, mp, ps);

    delete cams;

    return me;
}

mv_mesh* createMeshForFrontPlanePolygonOfCamera(int rc, const multiviewParams* mp, float border, point3d pivot)
{
    orientedPoint rcplane;
    rcplane.p = pivot;
    rcplane.n = mp->iRArr[rc] * point3d(0.0, 0.0, 1.0);
    rcplane.n = rcplane.n.normalize();

    point2d pix;
    mv_mesh* me = new mv_mesh();
    me->pts = new staticVector<point3d>(4);
    me->tris = new staticVector<mv_mesh::triangle>(2);

    pix.x = border;
    pix.y = border;
    me->pts->push_back(linePlaneIntersect(mp->CArr[rc], (mp->iCamArr[rc] * pix).normalize(), rcplane.p, rcplane.n));

    pix.x = (float)mp->mip->getWidth(rc) - border;
    pix.y = border;
    me->pts->push_back(linePlaneIntersect(mp->CArr[rc], (mp->iCamArr[rc] * pix).normalize(), rcplane.p, rcplane.n));

    pix.x = (float)mp->mip->getWidth(rc) - border;
    pix.y = (float)mp->mip->getHeight(rc) - border;
    me->pts->push_back(linePlaneIntersect(mp->CArr[rc], (mp->iCamArr[rc] * pix).normalize(), rcplane.p, rcplane.n));

    pix.x = border;
    pix.y = (float)mp->mip->getHeight(rc) - border;
    me->pts->push_back(linePlaneIntersect(mp->CArr[rc], (mp->iCamArr[rc] * pix).normalize(), rcplane.p, rcplane.n));

    mv_mesh::triangle t;
    t.i[0] = 0;
    t.i[1] = 2;
    t.i[2] = 1;
    me->tris->push_back(t);
    t.i[0] = 0;
    t.i[1] = 3;
    t.i[2] = 2;
    me->tris->push_back(t);

    return me;
}

mv_mesh* computeBoxMeshFromPlaneMeshAndZDimSize(mv_mesh* mePlane, float boxZsize)
{
    point3d n = cross(((*mePlane->pts)[0] - (*mePlane->pts)[1]).normalize(),
                      ((*mePlane->pts)[0] - (*mePlane->pts)[3]).normalize());

    mv_mesh* box = new mv_mesh();
    box->pts = new staticVector<point3d>(8);
    box->tris = new staticVector<mv_mesh::triangle>(12);

    for(int k = 0; k < 4; k++)
    {
        box->pts->push_back((*mePlane->pts)[k] - n * (boxZsize / 2.0f));
    }
    for(int k = 0; k < 4; k++)
    {
        box->pts->push_back((*mePlane->pts)[k] + n * (boxZsize / 2.0f));
    }

    mv_mesh::triangle t;
    t.alive = true;
    t.i[0] = 0;
    t.i[1] = 1;
    t.i[2] = 2;
    box->tris->push_back(t);
    t.i[0] = 2;
    t.i[1] = 3;
    t.i[2] = 0;
    box->tris->push_back(t);
    t.i[0] = 4;
    t.i[1] = 5;
    t.i[2] = 6;
    box->tris->push_back(t);
    t.i[0] = 6;
    t.i[1] = 7;
    t.i[2] = 4;
    box->tris->push_back(t);

    t.i[0] = 0;
    t.i[1] = 1;
    t.i[2] = 5;
    box->tris->push_back(t);
    t.i[0] = 5;
    t.i[1] = 4;
    t.i[2] = 0;
    box->tris->push_back(t);

    t.i[0] = 1;
    t.i[1] = 2;
    t.i[2] = 6;
    box->tris->push_back(t);
    t.i[0] = 6;
    t.i[1] = 5;
    t.i[2] = 1;
    box->tris->push_back(t);

    t.i[0] = 2;
    t.i[1] = 3;
    t.i[2] = 7;
    box->tris->push_back(t);
    t.i[0] = 7;
    t.i[1] = 6;
    t.i[2] = 2;
    box->tris->push_back(t);

    t.i[0] = 3;
    t.i[1] = 0;
    t.i[2] = 4;
    box->tris->push_back(t);
    t.i[0] = 4;
    t.i[1] = 7;
    t.i[2] = 3;
    box->tris->push_back(t);

    return box;
}

point3d mv_mesh::computeCentreOfGravity()
{
    point3d cg;
    for(int i = 0; i < pts->size(); i++)
    {
        cg = cg + (*pts)[i];
    }
    cg = cg / (float)pts->size();
    return cg;
}

staticVector<int>* mv_mesh::getDistinctPtsIdsForTrisIds(staticVector<int>* trisIds)
{
    staticVector<int>* tirsPtsIds = new staticVector<int>(trisIds->size() * 3);
    for(int i = 0; i < trisIds->size(); i++)
    {
        int id = (*trisIds)[i];
        tirsPtsIds->push_back((*tris)[id].i[0]);
        tirsPtsIds->push_back((*tris)[id].i[1]);
        tirsPtsIds->push_back((*tris)[id].i[2]);
    }

    qsort(&(*tirsPtsIds)[0], tirsPtsIds->size(), sizeof(int), qSortCompareIntAsc);

    staticVector<int>* out = new staticVector<int>(tirsPtsIds->size());

    int i = 0;
    while(i < tirsPtsIds->size())
    {
        if((i == tirsPtsIds->size() - 1) || ((*tirsPtsIds)[i] != (*tirsPtsIds)[i + 1]))
        {
            out->push_back((*tirsPtsIds)[i]);
        }
        i++;
    }

    return out;
}

void mv_mesh::changeTriPtId(int triId, int oldPtId, int newPtId)
{
    for(int k = 0; k < 3; k++)
    {
        if(oldPtId == (*tris)[triId].i[k])
        {
            (*tris)[triId].i[k] = newPtId;
        }
    }
}

int mv_mesh::getTriPtIndex(int triId, int ptId, bool failIfDoesNotExists)
{
    for(int k = 0; k < 3; k++)
    {
        if(ptId == (*tris)[triId].i[k])
        {
            return k;
        }
    }

    if(failIfDoesNotExists)
    {
        printf("ERROR getTriPtIndex\n");
        exit(1);
    }
    else
    {
        return -1;
    }
}

pixel mv_mesh::getTriOtherPtsIds(int triId, int _ptId)
{
    int others[3];
    int nothers = 0;
    for(int k = 0; k < 3; k++)
    {
        if(_ptId != (*tris)[triId].i[k])
        {
            others[nothers] = (*tris)[triId].i[k];
            nothers++;
        }
    }

    if(nothers != 2)
    {
        printf("WARNING pt X neighbouring tringle without pt X\n");
        exit(1);
    }

    return pixel(others[0], others[1]);
}

int mv_mesh::getTriRemainingPtIndex(int triId, int ptId1, int ptId2)
{
    for(int k = 0; k < 3; k++)
    {
        if(((*tris)[triId].i[k] != ptId1) && ((*tris)[triId].i[k] != ptId2))
        {
            return k;
        }
    }
    printf("ERROR getTriRemainingPtIndex\n");
    exit(1);
}

int mv_mesh::getTriRemainingPtId(int triId, int ptId1, int ptId2)
{
    for(int k = 0; k < 3; k++)
    {
        int ptid = (*tris)[triId].i[k];
        if((ptid != ptId1) && (ptid != ptId2))
        {
            return ptid;
        }
    }
    printf("ERROR getTriRemainingPtIndex\n");
    exit(1);
}

bool mv_mesh::areTwoTrisSameOriented(int triId1, int triId2, int edgePtId1, int edgePtId2)
{
    int t1ep1Index = getTriPtIndex(triId1, edgePtId1, true);
    int t1ep2Index = getTriPtIndex(triId1, edgePtId2, true);
    int t2ep1Index = getTriPtIndex(triId2, edgePtId1, true);
    int t2ep2Index = getTriPtIndex(triId2, edgePtId2, true);
    int t1Orientation = (t1ep2Index + (3 - t1ep1Index)) % 3; // can be 1 or 2;
    int t2Orientation = (t2ep2Index + (3 - t2ep1Index)) % 3; // can be 1 or 2;

    return (t1Orientation != t2Orientation);
}

int mv_mesh::getTriRemainingTriIndex(int ptIndex1, int ptIndex2)
{
    bool indexes[3] = {false, false, false};
    indexes[ptIndex1] = true;
    indexes[ptIndex2] = true;
    for(int k = 0; k < 3; k++)
    {
        if(!indexes[k])
        {
            return k;
        }
    }
}

int mv_mesh::getTri1EdgeIdWRTNeighTri2(int triId1, int triId2)
{
    int out = -1;
    int n = 0;
    for(int k = 0; k < 3; k++)
    {
        if(getTriPtIndex(triId2, (*tris)[triId1].i[k], false) == -1)
        {
            out = k;
            n++;
        }
    }

    if(n != 1)
    {
        out = -1;
    }

    return out;
}

bool mv_mesh::checkPtsForNaN()
{
    for(int i = 0; i < pts->size(); i++)
    {
        point3d p = (*pts)[i];
        if(std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z) || (p.x != p.x) || (p.y != p.y) ||
           (p.z != p.z)) // check if is not NaN
        {
            return false;
        }
    }
    return true;
}

void mv_mesh::filterWrongTriangles()
{
    float EPS = std::numeric_limits<float>::epsilon();

    staticVector<int>* trisIdsToStay = new staticVector<int>(tris->size());
    for(int i = 0; i < tris->size(); i++)
    {

        if(((*tris)[i].i[0] >= 0) && ((*tris)[i].i[0] < pts->size()) && ((*tris)[i].i[1] >= 0) &&
           ((*tris)[i].i[1] < pts->size()) && ((*tris)[i].i[2] >= 0) && ((*tris)[i].i[2] < pts->size()))
        {

            float d1 = ((*pts)[(*tris)[i].i[0]] - (*pts)[(*tris)[i].i[1]]).size();
            float d2 = ((*pts)[(*tris)[i].i[0]] - (*pts)[(*tris)[i].i[2]]).size();
            float d3 = ((*pts)[(*tris)[i].i[2]] - (*pts)[(*tris)[i].i[1]]).size();
            point3d n = computeTriangleNormal(i);

            if(std::isnan(d1) || std::isnan(d2) || std::isnan(d3) || (d1 != d1) || (d2 != d2) || (d3 != d3) ||
               std::isnan(n.x) || std::isnan(n.y) || std::isnan(n.z) || (n.x != n.x) || (n.y != n.y) ||
               (n.z != n.z)) // check if is not NaN
            {
                printf("WARNING edge length or normal is NaN\n");
            }
            else
            {
                if((d1 > EPS) && (d2 > EPS) && (d3 > EPS))
                {
                    trisIdsToStay->push_back(i);
                }
                else
                {
                    printf("WARNING edge length smaller than EPS\n");
                }
            }
        }
        else
        {
            printf("WARNING triangle index out of range!!!\n");
        }
    }
    letJustTringlesIdsInMesh(trisIdsToStay);
    delete trisIdsToStay;
}

void mv_mesh::filterObtuseTriangles()
{

    staticVector<int>* trisIdsToStay = new staticVector<int>(tris->size());
    for(int i = 0; i < tris->size(); i++)
    {
        if(!isTriangleObtuse(i))
        {
            trisIdsToStay->push_back(i);
        }
        else
        {
            printf("WARNING triangle index out of range!!!\n");
        }
    }
    letJustTringlesIdsInMesh(trisIdsToStay);
    delete trisIdsToStay;
}

bool mv_mesh::isTriangleAngleAtVetexObtuse(int vertexIdInTriangle, int triId)
{
    point3d A = (*pts)[(*tris)[triId].i[(vertexIdInTriangle + 0) % 3]];
    point3d B = (*pts)[(*tris)[triId].i[(vertexIdInTriangle + 1) % 3]];
    point3d C = (*pts)[(*tris)[triId].i[(vertexIdInTriangle + 2) % 3]];
    return dot(B - A, C - A) < 0.0f;
}

bool mv_mesh::isTriangleObtuse(int triId)
{
    return (isTriangleAngleAtVetexObtuse(0, triId)) || (isTriangleAngleAtVetexObtuse(1, triId)) ||
           (isTriangleAngleAtVetexObtuse(2, triId));
}

staticVector<staticVector<int>*>* mv_mesh::computeCamsTris(std::string ptsCamsFileName, const multiviewParams* mp)
{
    staticVector<staticVector<int>*>* trisCams = nullptr;
    {
        staticVector<staticVector<int>*>* ptsCams = loadArrayOfArraysFromFile<int>(ptsCamsFileName);
        trisCams = computeTrisCamsFromPtsCams(ptsCams);
        deleteArrayOfArrays<int>(&ptsCams);
    }

    staticVector<staticVector<int>*>* camsTris = convertObjectsCamsToCamsObjects(mp, trisCams);

    deleteArrayOfArrays<int>(&trisCams);

    return camsTris;
}

staticVector<int>* mv_mesh::getLargestConnectedComponentTrisIds(const multiviewParams& mp)
{
    staticVector<staticVector<int>*>* ptsNeighPtsOrdered = getPtsNeighPtsOrdered();

    staticVector<int>* colors = new staticVector<int>(pts->size());
    colors->resize_with(pts->size(), -1);
    staticVector<int>* buff = new staticVector<int>(pts->size());
    int col = 0;
    int maxNptsOfCol = -1;
    int bestCol = -1;
    for(int i = 0; i < pts->size(); i++)
    {
        if((*colors)[i] == -1)
        {
            buff->resize(0);
            buff->push_back(i);
            int nptsOfCol = 0;
            while(buff->size() > 0)
            {
                int ptid = buff->pop();
                if((*colors)[ptid] == -1)
                {
                    (*colors)[ptid] = col;
                    nptsOfCol++;
                }
                else
                {
                    if((*colors)[ptid] != col)
                    {
                        if(mp.verbose)
                            printf("WARNING should not happen!\n");
                    }
                }
                for(int j = 0; j < sizeOfStaticVector<int>((*ptsNeighPtsOrdered)[ptid]); j++)
                {
                    int nptid = (*(*ptsNeighPtsOrdered)[ptid])[j];
                    if((nptid > -1) && ((*colors)[nptid] == -1))
                    {
                        if(buff->size() >= buff->capacity())
                        {
                            if(mp.verbose)
                                printf("WARNING should not happen but no problem!\n");
                            buff->resizeAdd(pts->size());
                        }
                        buff->push_back(nptid);
                    }
                }
            }

            if(maxNptsOfCol < nptsOfCol)
            {
                maxNptsOfCol = nptsOfCol;
                bestCol = col;
            }
            col++;
        }
    }

    staticVector<int>* out = new staticVector<int>(tris->size());
    for(int i = 0; i < tris->size(); i++)
    {
        if(((*tris)[i].alive) && ((*colors)[(*tris)[i].i[0]] == bestCol) && ((*colors)[(*tris)[i].i[1]] == bestCol) &&
           ((*colors)[(*tris)[i].i[2]] == bestCol))
        {
            out->push_back(i);
        }
    }

    delete colors;
    delete buff;
    deleteArrayOfArrays<int>(&ptsNeighPtsOrdered);

    return out;
}

staticVector<int>* mv_mesh::getTrisIdsMapT(const multiviewParams* mp, int rc, int scale, staticVector<int>* trisIds)
{
    long tstart = clock();
    if(mp->verbose)
        printf("getTrisIdsMapT \n");

    int w = mp->mip->getWidth(rc);
    int h = mp->mip->getHeight(rc);

    staticVector<int>* triIdMap = new staticVector<int>(w * h);
    triIdMap->resize_with(w * h, -1);

    long t1 = initEstimate();
    for(int i = 0; i < trisIds->size(); i++)
    {
        int idTri = (*trisIds)[i];
        triangle_proj tp = getTriangleProjection(idTri, mp, rc, w, h);
        if(isTriangleProjectionInImage(tp, w, h))
        {
            pixel cell;
            for(cell.y = tp.lu.y / scale; cell.y <= tp.rd.y / scale; cell.y++)
            {
                for(cell.x = tp.lu.x / scale; cell.x <= tp.rd.x / scale; cell.x++)
                {
                    mv_mesh::rectangle re = mv_mesh::rectangle(cell, scale);
                    if(doesTriangleIntersectsRectangle(&tp, &re))
                    {
                        (*triIdMap)[cell.y * w + cell.x] = idTri;
                    }
                } // for y
            }     // for x
        }         // isthere
        printfEstimate(i, trisIds->size(), t1);
    } // for i ntris
    finishEstimate();

    if(mp->verbose)
        printfElapsedTime(tstart);
    return triIdMap;
}

void mv_mesh::remeshByTextureAndGetTextureInfo(int& textureSide, int& natlases, bool verbose,
                                               std::string meTextureCorrdsFileName, staticVector<point2d>** meshPtsUVO,
                                               staticVector<int>** meshPtsTexIdsO)
{
    FILE* fb = fopen(meTextureCorrdsFileName.c_str(), "rb");
    fread(&textureSide, sizeof(int), 1, fb);
    fread(&natlases, sizeof(int), 1, fb);

    if(verbose)
        printf("textureSide %i, natlases %i \n", textureSide, natlases);

    staticVector<point2d>* meshPtsUV = new staticVector<point2d>(pts->size());
    staticVector<int>* meshPtsTexIds = new staticVector<int>(pts->size());
    meshPtsUV->resize_with(pts->size(), point2d(-1.0f, -1.0f));
    meshPtsTexIds->resize_with(pts->size(), -1);

    for(int idatlasi = 0; idatlasi < natlases; idatlasi++)
    {
        int ntrisIds;
        fread(&ntrisIds, sizeof(int), 1, fb);

        int idatlas = idatlasi;
        fread(&idatlas, sizeof(int), 1, fb);

        staticVector<int>* trisIds = new staticVector<int>(ntrisIds);
        for(int i = 0; i < ntrisIds; i++)
        {
            int triid;
            fread(&triid, sizeof(int), 1, fb);
            trisIds->push_back(triid);
        }

        if(verbose)
            printf("ntrisIds %i\n", ntrisIds);

        for(int i = 0; i < trisIds->size(); i++)
        {
            int triid = (*trisIds)[i];

            bool hasuv = false;
            for(int k = 0; k < 3; k++)
            {
                point2d pixold = (*meshPtsUV)[(*tris)[triid].i[k]];
                if(pixold.x > 0.0f)
                {
                    hasuv = true;
                }
            }
            if(hasuv)
            {
                if(pts->size() + 3 > pts->capacity())
                {
                    meshPtsUV->resizeAdd(pts->size());
                    meshPtsTexIds->resizeAdd(pts->size());
                    pts->resizeAdd(pts->size());
                }
                if(tris->size() + 1 > tris->capacity())
                {
                    tris->resizeAdd(tris->size());
                }
                mv_mesh::triangle t;
                t.alive = true;
                for(int k = 0; k < 3; k++)
                {
                    t.i[k] = pts->size();
                    pts->push_back((*pts)[(*tris)[triid].i[k]]);
                    meshPtsUV->push_back(point2d(-1.0f, -1.0f));
                    meshPtsTexIds->push_back(-1);
                }
                (*tris)[triid] = t;
            }

            point2d pix;
            for(int k = 0; k < 3; k++)
            {
                fread(&pix, sizeof(point2d), 1, fb);
                (*meshPtsUV)[(*tris)[triid].i[k]] = pix;
                // printf("pix %f %f\n",pix.x,pix.y);
                (*meshPtsTexIds)[(*tris)[triid].i[k]] = idatlas;
            }
        }

        delete(trisIds);
    }

    fclose(fb);

    *meshPtsUVO = meshPtsUV;
    *meshPtsTexIdsO = meshPtsTexIds;
}

bool mv_mesh::loadFromObjAscii(int& nmtls, staticVector<int>** trisMtlIds, staticVector<point3d>** normals,
                               staticVector<voxel>** trisNormalsIds, staticVector<point2d>** uvCoords,
                               staticVector<voxel>** trisUvIds, std::string objAsciiFileName)
{
    std::cout << "Loading mesh from obj file: " << objAsciiFileName << std::endl;
    // read number of points, triangles, uvcoords
    int npts = 0;
    int ntris = 0;
    int nuvs = 0;
    int nnorms = 0;
    int nlines = 0;

    {
        std::ifstream in(objAsciiFileName.c_str());
        std::string line;
        while(getline(in, line))
        {
            if((line[0] == 'v') && (line[1] == ' '))
            {
                npts += 1;
            }
            if((line[0] == 'v') && (line[1] == 'n') && (line[2] == ' '))
            {
                nnorms += 1;
            }
            if((line[0] == 'v') && (line[1] == 't') && (line[2] == ' '))
            {
                nuvs += 1;
            }
            if((line[0] == 'f') && (line[1] == ' '))
            {
                ntris += 1;
            }
            nlines++;
        }
        in.close();
    }

    printf("vertices %i\n", npts);
    printf("normals %i\n", nnorms);
    printf("uv coordinates %i\n", nuvs);
    printf("triangles %i\n", ntris);

    pts = new staticVector<point3d>(npts);
    tris = new staticVector<mv_mesh::triangle>(ntris);
    *uvCoords = new staticVector<point2d>(nuvs);
    *trisUvIds = new staticVector<voxel>(ntris);
    *normals = new staticVector<point3d>(nnorms);
    *trisNormalsIds = new staticVector<voxel>(ntris);
    *trisMtlIds = new staticVector<int>(ntris);

    std::map<std::string, int> materialCache;

    {
        int mtlId = -1;
        std::ifstream in(objAsciiFileName.c_str());
        std::string line;

        long t1 = initEstimate();
        int idline = 0;
        while(getline(in, line))
        {
            if(findNSubstrsInString(line, "usemtl") == 1)
            {
                char buff[5000];
                sscanf(line.c_str(), "usemtl %s", buff);
                auto it = materialCache.find(buff);
                if(it == materialCache.end())
                    materialCache.emplace(buff, ++mtlId); // new material
                else
                    mtlId = it->second;                   // already known material
            }

            if((line[0] == 'v') && (line[1] == ' '))
            {
                point3d pt;
                sscanf(line.c_str(), "v %lf %lf %lf", &pt.x, &pt.y, &pt.z);
                pts->push_back(pt);
            }

            if((line[0] == 'v') && (line[1] == 'n') && (line[2] == ' '))
            {
                point3d pt;
                sscanf(line.c_str(), "vn %lf %lf %lf", &pt.x, &pt.y, &pt.z);
                // printf("%f %f %f\n", pt.x, pt.y, pt.z);
                (*normals)->push_back(pt);
            }

            if((line[0] == 'v') && (line[1] == 't') && (line[2] == ' '))
            {
                point2d pt;
                sscanf(line.c_str(), "vt %lf %lf", &pt.x, &pt.y);
                (*uvCoords)->push_back(pt);
            }

            if((line[0] == 'f') && (line[1] == ' '))
            {
                int n1 = findNSubstrsInString(line, "/");
                int n2 = findNSubstrsInString(line, "//");
                voxel v1, v2, v3;
                bool ok = false;
                bool okn = false;
                bool okuv = false;
                if(n2 == 0)
                {
                    if(n1 == 0)
                    {
                        sscanf(line.c_str(), "f %i %i %i", &v1.x, &v1.y, &v1.z);
                        ok = true;
                    }
                    if(n1 == 3)
                    {
                        sscanf(line.c_str(), "f %i/%i %i/%i %i/%i", &v1.x, &v2.x, &v1.y, &v2.y, &v1.z, &v2.z);
                        ok = true;
                        okuv = true;
                    }
                    if(n1 == 6)
                    {
                        sscanf(line.c_str(), "f %i/%i/%i %i/%i/%i %i/%i/%i", &v1.x, &v2.x, &v3.x, &v1.y, &v2.y, &v3.y,
                               &v1.z, &v2.z, &v3.z);
                        ok = true;
                        okuv = true;
                        okn = true;
                    }
                }
                else
                {
                    if(n2 == 3)
                    {
                        sscanf(line.c_str(), "f %i//%i %i//%i %i//%i", &v1.x, &v3.x, &v1.y, &v3.y, &v1.z, &v3.z);
                        ok = true;
                        okn = true;
                    }
                }

                if(!ok)
                {
                    printf("ERROR occured while reading obj file %s\n", objAsciiFileName.c_str());
                    exit(1);
                }

                triangle t;
                t.i[0] = v1.x - 1;
                t.i[1] = v1.y - 1;
                t.i[2] = v1.z - 1;
                t.alive = true;
                tris->push_back(t);
                (*trisMtlIds)->push_back(mtlId);

                if(okuv)
                {
                    (*trisUvIds)->push_back(v2 - voxel(1, 1, 1));
                }

                if(okn)
                {
                    (*trisNormalsIds)->push_back(v3 - voxel(1, 1, 1));
                }
            }

            printfEstimate(idline, nlines, t1);
            idline++;
        }
        finishEstimate();

        in.close();
        nmtls = materialCache.size();
    }
    return npts != 0 && ntris != 0;
}

bool mv_mesh::getEdgeNeighTrisInterval(pixel& itr, pixel edge, staticVector<voxel>* edgesXStat,
                                       staticVector<voxel>* edgesXYStat)
{
    int ptId1 = std::max(edge.x, edge.y);
    int ptId2 = std::min(edge.x, edge.y);
    itr = pixel(-1, -1);

    int i1 = indexOfSortedVoxelArrByX(ptId1, edgesXStat, 0, edgesXStat->size() - 1);
    if(i1 > -1)
    {
        int i2 = indexOfSortedVoxelArrByX(ptId2, edgesXYStat, (*edgesXStat)[i1].y, (*edgesXStat)[i1].z);
        if(i2 > -1)
        {
            itr = pixel((*edgesXYStat)[i2].y, (*edgesXYStat)[i2].z);
        }
        else
        {
            return false;
        }
    }
    else
    {
        return false;
    }

    return true;
}

staticVector<voxel>* mv_mesh::getTrisNeighsTris()
{
    printf("getTrisNeighsTris ... ");

    staticVector<voxel>* trisNeighTris = new staticVector<voxel>(tris->size());
    trisNeighTris->resize_with(tris->size(), voxel(-1, -1, -1));

    staticVector<voxel>* edgesNeigTris = new staticVector<voxel>(tris->size() * 3);
    staticVector<voxel>* edgesXStat = new staticVector<voxel>(pts->size());
    staticVector<voxel>* edgesXYStat = new staticVector<voxel>(tris->size() * 3);

    for(int i = 0; i < tris->size(); i++)
    {
        int a = (*tris)[i].i[0];
        int b = (*tris)[i].i[1];
        int c = (*tris)[i].i[2];
        edgesNeigTris->push_back(voxel(std::max(a, b), std::min(a, b), i));
        edgesNeigTris->push_back(voxel(std::max(b, c), std::min(b, c), i));
        edgesNeigTris->push_back(voxel(std::max(c, a), std::min(c, a), i));
    }

    qsort(&(*edgesNeigTris)[0], edgesNeigTris->size(), sizeof(voxel), qSortCompareVoxelByXAsc);

    // sort
    int i0 = 0;
    long t1 = initEstimate();
    for(int i = 0; i < edgesNeigTris->size(); i++)
    {

        if((i == edgesNeigTris->size() - 1) || ((*edgesNeigTris)[i].x != (*edgesNeigTris)[i + 1].x))
        {
            if(i - i0 + 1 > 1)
                qsort(&(*edgesNeigTris)[i0], i - i0 + 1, sizeof(voxel), qSortCompareVoxelByYAsc);

            // printf("i0 %i - i %i\n",i0,i);
            int xyI0 = edgesXYStat->size();

            int j0 = i0;
            for(int j = i0; j <= i; j++)
            {
                if((j == i) || ((*edgesNeigTris)[j].y != (*edgesNeigTris)[j + 1].y))
                {
                    // printf("j0 %i - j %i\n",j0,j);
                    if(j - j0 + 1 > 1)
                        qsort(&(*edgesNeigTris)[j0], j - j0 + 1, sizeof(voxel), qSortCompareVoxelByZAsc);

                    // for (int k=j0;k<=j;k++) {
                    //	printf("%i %i %i %i\n",k,(*edgesNeigTris)[k].x,(*edgesNeigTris)[k].y,(*edgesNeigTris)[k].z);
                    //};

                    // printf("%i of %i\n",edgesXYStat->size(),edgesXYStat->reserved());
                    edgesXYStat->push_back(voxel((*edgesNeigTris)[j].y, j0, j));
                    j0 = j + 1;
                }
            }

            int xyI = edgesXYStat->size() - 1;

            // printf("%i of %i\n",edgesXStat->size(),edgesXStat->reserved());
            edgesXStat->push_back(voxel((*edgesNeigTris)[i].x, xyI0, xyI));

            i0 = i + 1;
        }

        printfEstimate(i, edgesNeigTris->size(), t1);
    }
    finishEstimate();

    for(int i = 0; i < tris->size(); i++)
    {
        for(int k = 0; k < 3; k++)
        {
            pixel edge = pixel((*tris)[i].i[(k + 1) % 3], (*tris)[i].i[(k + 2) % 3]);
            pixel itr;
            if(!getEdgeNeighTrisInterval(itr, edge, edgesXStat, edgesXYStat))
            {
                printf("WARNING mv_mesh::getTrisNeighsTris case 0\n");
                exit(1);
            }
            int nneighs = itr.y - itr.x + 1;
            if(nneighs > 2)
            {
                printf("WARNING mv_mesh::getTrisNeighsTris triangle edge has more than one neighbours\n");
            }

            bool wasThere = false;
            for(int y = itr.x; y <= itr.y; y++)
            {
                int neighTri = (*edgesNeigTris)[y].z;
                if(neighTri != i)
                {
                    (*trisNeighTris)[i].m[k] = neighTri;
                }
                else
                {
                    wasThere = true;
                }
            }
            if(!wasThere)
            {
                printf("WARNING mv_mesh_clean2::init case 1\n");
                exit(1);
            }
        }
    }

    delete edgesNeigTris;
    delete edgesXStat;
    delete edgesXYStat;

    printf("done\n");

    return trisNeighTris;
}

void mv_mesh::Transform(matrix3x3 Rs, point3d t)
{
    for(int i = 0; i < pts->size(); i++)
    {
        (*pts)[i] = Rs * (*pts)[i] + t;
    }
}
