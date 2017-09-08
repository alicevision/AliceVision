#include "mv_output3D.h"

#include "stdafx.h"
#include "structures/mv_filesio.h"
#include "structures/mv_geometry.h"
#include <boost/filesystem.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <unordered_map>
#include <iostream>

namespace bfs = boost::filesystem;
using namespace std;

mv_output3D::mv_output3D(const multiviewParams* _mp)
  : mp(_mp)
  , g_border(10)
  , pixelSizeErrThr(5.0)
{
}

mv_output3D::~mv_output3D()
{
}

void mv_output3D::initNtrisNptsAndMap2Pts(int* _ntris, int* _npts, staticVector<float>* depthMap, int rc,
                                          staticVector<int>** _map2pts)
{
    int width = mp->mip->getWidth(rc);
    int height = mp->mip->getHeight(rc);
    float scale = 10.f;

    // compute number of points and triangles
    float* ludm;
    float* rudm;
    float* rddm;
    float* lddm;

    int npts = 0;
    int ntris = 0;

    staticVector<int>* map2pts = new staticVector<int>(width * height);
    map2pts->resize_with(width * height, -1);

    ludm = &(*depthMap)[0 * height + 0];
    rudm = &(*depthMap)[1 * height + 0];
    rddm = &(*depthMap)[1 * height + 1];
    lddm = &(*depthMap)[0 * height + 1];
    for(int x = 0; x < width; x++)
    {
        for(int y = 0; y < height; y++)
        {
            point2d pix;
            pix.x = (float)x * scale + scale / 2.0;
            pix.y = (float)y * scale + scale / 2.0;

            if((*ludm) > 0.0)
            {
                (*map2pts)[x * height + y] = npts;
                npts++;
            }

            if((x < width - 1) && (y < height - 1))
            {

                if(((*ludm) > 0.0) && ((*rudm) > 0.0) && ((*lddm) > 0.0))
                {
                    point3d plu = mp->CArr[rc] + (mp->iCamArr[rc] * point2d(pix.x, pix.y)).normalize() * (*ludm);
                    point3d pru =
                        mp->CArr[rc] + (mp->iCamArr[rc] * point2d(pix.x + scale, pix.y)).normalize() * (*rudm);
                    point3d pld =
                        mp->CArr[rc] + (mp->iCamArr[rc] * point2d(pix.x + scale, pix.y + scale)).normalize() * (*lddm);
                    float pixelSize = pixelSizeErrThr * mp->getCamPixelSize(plu, rc) * scale;
                    if(((plu - pru).size() < pixelSize) && ((plu - pld).size() < pixelSize) &&
                       ((pru - pld).size() < pixelSize))
                    {
                        ntris++;
                    }
                }

                if(((*rudm) > 0.0) && ((*rddm) > 0.0) && ((*lddm) > 0.0))
                {
                    point3d pru =
                        mp->CArr[rc] + (mp->iCamArr[rc] * point2d(pix.x + scale, pix.y)).normalize() * (*rudm);
                    point3d prd =
                        mp->CArr[rc] + (mp->iCamArr[rc] * point2d(pix.x + scale, pix.y + scale)).normalize() * (*rddm);
                    point3d pld =
                        mp->CArr[rc] + (mp->iCamArr[rc] * point2d(pix.x, pix.y + scale)).normalize() * (*lddm);
                    float pixelSize = pixelSizeErrThr * mp->getCamPixelSize(pru, rc) * scale;
                    if(((pru - prd).size() < pixelSize) && ((pru - pld).size() < pixelSize) &&
                       ((prd - pld).size() < pixelSize))
                    {
                        ntris++;
                    }
                }
            }

            ++ludm;
            ++rudm;
            ++rddm;
            ++lddm;
        }
    }

    *_npts = npts;
    *_ntris = ntris;
    *_map2pts = map2pts;
}

int mv_output3D::write2PlyPts(FILE* plyf, bool dobinary, int  /*ntris*/, int  /*npts*/, staticVector<float>* depthMap, int rc,
                              staticVector<int>*  /*map2pts*/, int  /*ptIndexPrefix*/, bool textureOrRc, voxel* col,
                              bool write_color)
{
    int width = mp->mip->getWidth(rc);
    int height = mp->mip->getHeight(rc);
    float scale = 1.0f;

    int nwritten = 0;

    float* ludm;

    // write pts
    const std::string fileNameStrIn = mv_getFileNamePrefix(mp->mip, rc + 1) + "." + mp->mip->imageExt;
    IplImage* bmpimg = cvLoadImage(fileNameStrIn.c_str());

    ludm = &(*depthMap)[0 * height + 0];
    for(int x = 0; x < width; x++)
    {
        for(int y = 0; y < height; y++)
        {
            point2d pix;
            pix.x = (float)x * scale + scale / 2.0;
            pix.y = (float)y * scale + scale / 2.0;

            if((*ludm) > 0.0)
            {
                CvScalar lug;
                point3d p = mp->CArr[rc] + (mp->iCamArr[rc] * point2d(pix.x, pix.y)).normalize() * (*ludm);
                if(textureOrRc)
                {
                    lug = cvGet2D(bmpimg, (int)pix.y, (int)pix.x);
                }
                else
                {
                    lug.val[0] = col->z;
                    lug.val[1] = col->y;
                    lug.val[2] = col->x;
                }

                if(dobinary)
                {
                    float pos[3];
                    pos[0] = (float)(p.x);
                    pos[1] = (float)(p.y);
                    pos[2] = (float)(p.z);
                    fwrite(&pos[0], sizeof(float), 3, plyf);
                    if(write_color)
                    {
                        float coll[3];
                        coll[0] = (unsigned char)(p.x);
                        coll[1] = (unsigned char)(p.y);
                        coll[2] = (unsigned char)(p.z);
                        fwrite(&coll[0], sizeof(unsigned char), 3, plyf);
                    }

                    nwritten++;
                }
                else
                {
                    if(write_color)
                    {
                        fprintf(plyf, "%f %f %f %i %i %i%c", (float)(p.x), (float)(p.y), (float)(p.z),
                                (unsigned char)lug.val[0], (unsigned char)lug.val[1], (unsigned char)lug.val[2], 10);
                    }
                    else
                    {
                        fprintf(plyf, "%f %f %f%c", (float)(p.x), (float)(p.y), (float)(p.z), 10);
                    }

                    nwritten++;
                }
            }
            ++ludm;
        }
    }

    cvReleaseImage(&bmpimg);
    return nwritten;
}

int mv_output3D::write2PlyTris(FILE* plyf, bool dobinary, int ntris, int  /*npts*/, staticVector<float>* depthMap, int rc,
                               staticVector<int>* map2pts, int ptIndexPrefix, int gnpts, bool islast)
{
    int width = mp->mip->getWidth(rc);
    int height = mp->mip->getHeight(rc);
    float scale = 1.0f;

    int nwritten = 0;

    float* ludm;
    float* rudm;
    float* rddm;
    float* lddm;

    // write triangle indexes
    ludm = &(*depthMap)[0 * height + 0];
    rudm = &(*depthMap)[1 * height + 0];
    rddm = &(*depthMap)[1 * height + 1];
    lddm = &(*depthMap)[0 * height + 1];
    int i = 0;
    for(int x = 0; x < width; x++)
    {
        for(int y = 0; y < height; y++)
        {
            point2d pix;
            pix.x = (float)x * scale + scale / 2.0;
            pix.y = (float)y * scale + scale / 2.0;

            if((x < width - 1) && (y < height - 1))
            {
                if(((*ludm) > 0.0) && ((*rudm) > 0.0) && ((*lddm) > 0.0))
                {
                    point3d plu = mp->CArr[rc] + (mp->iCamArr[rc] * point2d(pix.x, pix.y)).normalize() * (*ludm);
                    point3d pru =
                        mp->CArr[rc] + (mp->iCamArr[rc] * point2d(pix.x + scale, pix.y)).normalize() * (*rudm);
                    point3d pld =
                        mp->CArr[rc] + (mp->iCamArr[rc] * point2d(pix.x + scale, pix.y + scale)).normalize() * (*lddm);
                    float pixelSize = pixelSizeErrThr * mp->getCamPixelSize(plu, rc) * scale;
                    if(((plu - pru).size() < pixelSize) && ((plu - pld).size() < pixelSize) &&
                       ((pru - pld).size() < pixelSize))
                    {
                        if(dobinary)
                        {
                            unsigned char npts = 3;
                            fwrite(&npts, sizeof(unsigned char), 1, plyf);
                            int points[3];
                            points[2] = ptIndexPrefix + (*map2pts)[(x + 0) * height + (y + 0)];
                            points[1] = ptIndexPrefix + (*map2pts)[(x + 1) * height + (y + 0)];
                            points[0] = ptIndexPrefix + (*map2pts)[(x + 0) * height + (y + 1)];
                            fwrite(&points[0], sizeof(int), 3, plyf);
                            nwritten++;
                        }
                        else
                        {
                            int points[3];
                            points[2] = ptIndexPrefix + (*map2pts)[(x + 0) * height + (y + 0)];
                            points[1] = ptIndexPrefix + (*map2pts)[(x + 1) * height + (y + 0)];
                            points[0] = ptIndexPrefix + (*map2pts)[(x + 0) * height + (y + 1)];

                            if((points[0] >= 0) && (points[0] < gnpts) && (points[1] >= 0) && (points[1] < gnpts) &&
                               (points[2] >= 0) && (points[2] < gnpts))
                            {
                                int wr = fprintf(plyf, "3 %d %d %d", points[2], points[1], points[0]);

                                if(wr < 0)
                                {
                                    printf("ERROR");
                                }

                                if((i < ntris - 1) || (!islast))
                                {
                                    fprintf(plyf, "%c", 10);
                                }
                                nwritten++;
                            }
                            else
                            {
                                printf("ERROR");
                            }
                        }
                        i++;
                    }
                }

                if(((*rudm) > 0.0) && ((*rddm) > 0.0) && ((*lddm) > 0.0))
                {
                    point3d pru =
                        mp->CArr[rc] + (mp->iCamArr[rc] * point2d(pix.x + scale, pix.y)).normalize() * (*rudm);
                    point3d prd =
                        mp->CArr[rc] + (mp->iCamArr[rc] * point2d(pix.x + scale, pix.y + scale)).normalize() * (*rddm);
                    point3d pld =
                        mp->CArr[rc] + (mp->iCamArr[rc] * point2d(pix.x, pix.y + scale)).normalize() * (*lddm);
                    float pixelSize = pixelSizeErrThr * mp->getCamPixelSize(pru, rc) * scale;
                    if(((pru - prd).size() < pixelSize) && ((pru - pld).size() < pixelSize) &&
                       ((prd - pld).size() < pixelSize))
                    {
                        if(dobinary)
                        {
                            unsigned char npts = 3;
                            fwrite(&npts, sizeof(unsigned char), 1, plyf);
                            int points[3];
                            points[2] = ptIndexPrefix + (*map2pts)[(x + 1) * height + (y + 0)];
                            points[1] = ptIndexPrefix + (*map2pts)[(x + 1) * height + (y + 1)];
                            points[0] = ptIndexPrefix + (*map2pts)[(x + 0) * height + (y + 1)];
                            fwrite(&points[0], sizeof(int), 3, plyf);
                            nwritten++;
                        }
                        else
                        {
                            int points[3];
                            points[2] = ptIndexPrefix + (*map2pts)[(x + 1) * height + (y + 0)];
                            points[1] = ptIndexPrefix + (*map2pts)[(x + 1) * height + (y + 1)];
                            points[0] = ptIndexPrefix + (*map2pts)[(x + 0) * height + (y + 1)];

                            if((points[0] >= 0) && (points[0] < gnpts) && (points[1] >= 0) && (points[1] < gnpts) &&
                               (points[2] >= 0) && (points[2] < gnpts))
                            {
                                int wr = fprintf(plyf, "3 %d %d %d", points[2], points[1], points[0]);

                                if(wr < 0)
                                {
                                    printf("ERROR");
                                }

                                if((i < ntris - 1) || (!islast))
                                {
                                    fprintf(plyf, "%c", 10);
                                }
                                nwritten++;
                            }
                            else
                            {
                                printf("ERROR");
                            }
                        }
                        i++;
                    }
                }
            }

            ++ludm;
            ++rudm;
            ++rddm;
            ++lddm;
        }
    }

    return nwritten;
}

void mv_output3D::depthMap2Ply(const std::string& plyFileName, staticVector<float>* depthMap, int rc)
{

    int ntris;
    int npts;
    staticVector<int>* map2pts;
    initNtrisNptsAndMap2Pts(&ntris, &npts, depthMap, rc, &map2pts);

    // write to ply
    printf("Writing to file %s \n", plyFileName.c_str());

    FILE* plyf = nullptr;

    bool dobinary = false;

    if(dobinary)
    {
        plyf = fopen(plyFileName.c_str(), "wb");
    }
    else
    {
        plyf = fopen(plyFileName.c_str(), "wb");
    }

    if(plyf == nullptr)
    {
        printf("Could not open output ply file %s\n", plyFileName.c_str());
        exit(1);
    }

    fprintf(plyf, "ply%c", 10);
    if(dobinary)
    {
        fprintf(plyf, "format binary_little_endian 1.0%c", 10);
    }
    else
    {
        fprintf(plyf, "format ascii 1.0%c", 10);
    }
    fprintf(plyf, "comment Generated by multiRecon ( http://cmp.felk.cvut.cz/~jancom1 )%c", 10);
    fprintf(plyf, "element vertex %d%c", npts, 10);
    fprintf(plyf, "property float x%c", 10);
    fprintf(plyf, "property float y%c", 10);
    fprintf(plyf, "property float z%c", 10);
    fprintf(plyf, "property uchar diffuse_red%c", 10);
    fprintf(plyf, "property uchar diffuse_green%c", 10);
    fprintf(plyf, "property uchar diffuse_blue%c", 10);
    fprintf(plyf, "element face %d%c", ntris, 10);
    fprintf(plyf, "property list uchar int vertex_indices%c", 10);
    fprintf(plyf, "end_header%c", 10);

    write2PlyPts(plyf, dobinary, ntris, npts, depthMap, rc, map2pts, 0, true, nullptr, true);
    write2PlyTris(plyf, dobinary, ntris, npts, depthMap, rc, map2pts, 0, npts, true);

    fclose(plyf);

    delete map2pts;
}

void mv_output3D::savePrematchToWrl(const std::string& wrname, int shift, int step)
{
    // ALL
    // printf("Creating : %s\n",wrname.c_str());

    // printf("converting\n");

    int npts = 0;
    long t1 = initEstimate();
    for(int rc = shift; rc < mp->ncams; rc += step)
    {
        staticVector<seedPoint>* seeds;
        loadSeedsFromFile(&seeds, mp->indexes[rc], mp->mip, mp->mip->MV_FILE_TYPE_seeds);

        staticVector<orientedPoint>* ops = new staticVector<orientedPoint>(seeds->size());

        for(int i = 0; i < seeds->size(); i++)
        {
            ops->push_back((*seeds)[i].op);
        } // for i

        npts += ops->size();
        saveOrientedPointsToFile(ops, mp->indexes[rc], mp->mip);
        delete ops;

        delete seeds;

        printfEstimate(rc, mp->ncams, t1);
    }
    finishEstimate();

    // printf("all pts %i\n",npts);

    /*
    create_wrl(
            mp,
            1,
            1,
            0,
            -0.6,
            wrname,
            100000,
            false
    );
    */
    create_wrl_pts(mp, wrname, shift, step);
}

void mv_output3D::savePrematchToWrl(const std::string& wrname)
{
    // ALL
    printf("saving seeds to wrl\n");

    printf("converting\n");

    int npts = 0;
    long t1 = initEstimate();
    for(int rc = 0; rc < mp->ncams; rc++)
    {
        staticVector<seedPoint>* seeds;
        loadSeedsFromFile(&seeds, mp->indexes[rc], mp->mip, mp->mip->MV_FILE_TYPE_seeds);

        staticVector<orientedPoint>* ops = new staticVector<orientedPoint>(seeds->size());

        for(int i = 0; i < seeds->size(); i++)
        {
            ops->push_back((*seeds)[i].op);
        } // for i

        npts += ops->size();
        saveOrientedPointsToFile(ops, mp->indexes[rc], mp->mip);
        delete ops;

        delete seeds;

        printfEstimate(rc, mp->ncams, t1);
    }
    finishEstimate();

    printf("all pts %i \n", npts);

    create_wrl(mp, 1, 10, 0, -0.6, wrname, 100000, false);
}

void mv_output3D::printfGroupCameras(mv_mesh* meCams, staticVector<int>* tcams, FILE* f, const multiviewParams* mp, float ps,
                                     float upStep)
{
    printfGroupCameras(meCams, tcams, f, mp, ps, 0, 1, upStep);
}

void mv_output3D::printfGroupCameras(mv_mesh* meCams, staticVector<int>* tcams, FILE* f, const multiviewParams* mp, float  /*ps*/,
                                     int shift, int step, float upStep)
{
    rgb colorOfTris;
    colorOfTris.r = 0;
    colorOfTris.g = 255;
    colorOfTris.b = 0;
    printfMvMeshToWrl(f, colorOfTris, meCams);

    fprintf(f, "Group {\nchildren [\n");

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

        Cl = Cl + Vl.normalize() * upStep;
        viewp = viewp + Vl.normalize() * upStep;

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

        fprintf(f, "Viewpoint {\n");
        fprintf(f, "\tfieldOfView %f \n", angleBetwV1andV2(point3d(mp->KArr[rc].m13, mp->KArr[rc].m11, 0.0),
                                                           point3d(0.0, mp->KArr[rc].m11, 0.0)) *
                                              (M_PI / 180.0) * 2.0);
        fprintf(f, "\tposition %f %f %f \n", viewp.x, viewp.y, viewp.z);
        fprintf(f, "\torientation %f %f %f %f \n", viewn.x, viewn.y, viewn.z, viewsim);
        fprintf(f, "\tdescription \"v%i\" \n", rc);
        fprintf(f, "\tjump TRUE \n");
        fprintf(f, "}\n");
    }

    fprintf(f, "\t\t] #  end of children\n} # end of group\n");

    /*
    fprintf(f, "Group {\nchildren [\n");


    for (int c = shift; c<tcams->size(); c+=step)
    {
            int rc = (*tcams)[c];
            QVector		pos, at, up;
            QVector		rot_axis;
            float		rot_angle;
            float		viewsim;

            point3d viewp;
            point3d viewn;
            viewp = mp->CArr[rc];
            point3d Cl = viewp;

            point3d Xl = mp->RArr[rc].transpose() * point3d(1.0,0.0,0.0) * mp->KArr[rc].m13;
            Xl = Xl.normalize();
            point3d Yl = mp->RArr[rc].transpose() * point3d(0.0,1.0,0.0) * mp->KArr[rc].m23;
            Yl = Yl.normalize();
            point3d Zl = mp->RArr[rc].transpose() * point3d(0.0,0.0,1.0) * mp->KArr[rc].m11;
            Zl = Zl.normalize();

            point3d Ol = Cl + Zl;
            point3d Vl = Yl;
            Vl.x = -Vl.x;
            Vl.y = -Vl.y;
            Vl.z = -Vl.z;

            Cl = Cl + Vl.normalize()*upStep;
            viewp = viewp + Vl.normalize()*upStep;

            pos.x = Cl.x;	pos.y = Cl.y;	pos.z = Cl.z;
            at.x  = Ol.x;	at.y  = Ol.y;	at.z  = Ol.z;
            up.x  = Vl.x;	up.y  = Vl.y;	up.z  = Vl.z;

            Convert_Camera_Model(&pos, &at, &up, &rot_axis, &rot_angle);

            viewn.x = rot_axis.x;
            viewn.y = rot_axis.y;
            viewn.z = rot_axis.z;
            viewsim = rot_angle;


            fprintf(f, "Viewpoint {\n");
            fprintf(f, "\tfieldOfView %f \n",
                            angleBetwV1andV2(
                                    point3d(mp->KArr[rc].m13,mp->KArr[rc].m11,0.0),
                                    point3d(0.0,mp->KArr[rc].m11,0.0)
                            ) * (M_PI/180.0) * 2.0
            );
            fprintf(f, "\tposition %f %f %f \n",viewp.x,viewp.y,viewp.z);
            fprintf(f, "\torientation %f %f %f %f \n",viewn.x,viewn.y,viewn.z,viewsim);
            fprintf(f, "\tdescription \"v%i\" \n",rc);
            fprintf(f, "\tjump TRUE \n");
            fprintf(f, "}\n");
    };

    fprintf(f, "Background {\nskyColor 1 1 1 \n}\n");
    fprintf(f, "\tTransform {\n");
    fprintf(f, "\t\ttranslation %g %g %g\n", 0.0, 0.0, 0.0);
    fprintf(f, "\t\tscale %g %g %g\n", 1.0, 1.0, 1.0);
    fprintf(f, "\n\t\tchildren [ Shape {\n");


    //cameras
    fprintf(f, "\t\t\tappearance Appearance {material Material {emissiveColor 1 1 1 }}\n");
    fprintf(f, "\t\t\tgeometry IndexedLineSet {\n");
    fprintf(f, "\t\t\t\tcoord Coordinate { point [\n");

    for (int c = shift; c<tcams->size(); c+=step)
    {
            int rc = (*tcams)[c];
            QVector		pos, at, up;
            QVector		rot_axis;
            float		rot_angle;
            float		viewsim;

            point3d viewp;
            point3d viewn;
            viewp = mp->CArr[rc];
            point3d Cl = viewp;

            point3d Xl = mp->RArr[rc].transpose() * point3d(1.0,0.0,0.0) * mp->KArr[rc].m13;
            Xl = Xl.normalize();
            point3d Yl = mp->RArr[rc].transpose() * point3d(0.0,1.0,0.0) * mp->KArr[rc].m23;
            Yl = Yl.normalize();
            point3d Zl = mp->RArr[rc].transpose() * point3d(0.0,0.0,1.0) * mp->KArr[rc].m11;
            Zl = Zl.normalize();

            point3d Ol = Cl + Zl;
            point3d Vl = Yl;
            Vl.x = -Vl.x;
            Vl.y = -Vl.y;
            Vl.z = -Vl.z;

            pos.x = Cl.x;	pos.y = Cl.y;	pos.z = Cl.z;
            at.x  = Ol.x;	at.y  = Ol.y;	at.z  = Ol.z;
            up.x  = Vl.x;	up.y  = Vl.y;	up.z  = Vl.z;

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


            fprintf(f, "\t\t\t\t\t%7.8f %7.8f %7.8f \n", p.x, p.y, p.z);
            fprintf(f, "\t\t\t\t\t%7.8f %7.8f %7.8f \n", p.x+(n.x-vx.x-vy.x)*ps,
                                                                                                     p.y+(n.y-vx.y-vy.y)*ps,
                                                                                                     p.z+(n.z-vx.z-vy.z)*ps);
            fprintf(f, "\t\t\t\t\t%7.8f %7.8f %7.8f \n", p.x+(n.x-vx.x+vy.x)*ps,
                                                                                                     p.y+(n.y-vx.y+vy.y)*ps,
                                                                                                     p.z+(n.z-vx.z+vy.z)*ps);
            fprintf(f, "\t\t\t\t\t%7.8f %7.8f %7.8f \n", p.x+(n.x+vx.x+vy.x)*ps,
                                                                                                     p.y+(n.y+vx.y+vy.y)*ps,
                                                                                                     p.z+(n.z+vx.z+vy.z)*ps);
            fprintf(f, "\t\t\t\t\t%7.8f %7.8f %7.8f \n", p.x+(n.x+vx.x-vy.x)*ps,
                                                                                                     p.y+(n.y+vx.y-vy.y)*ps,
                                                                                                     p.z+(n.z+vx.z-vy.z)*ps);
    };

    fprintf(f, "\t\t\t\t]}\n\n");

    fprintf(f, "\t\t\t\tcoordIndex [\n");

    int id=0;
    for (int c = shift; c<tcams->size(); c+=step)
    {
            fprintf(f, "\t\t\t\t\t%i %i %i\n", id*5, id*5+1, -1);
            fprintf(f, "\t\t\t\t\t%i %i %i\n", id*5, id*5+2, -1);
            fprintf(f, "\t\t\t\t\t%i %i %i\n", id*5, id*5+3, -1);
            fprintf(f, "\t\t\t\t\t%i %i %i\n", id*5, id*5+4, -1);

            fprintf(f, "\t\t\t\t\t%i %i %i\n", id*5+1, id*5+2, -1);
            fprintf(f, "\t\t\t\t\t%i %i %i\n", id*5+2, id*5+3, -1);
            fprintf(f, "\t\t\t\t\t%i %i %i\n", id*5+3, id*5+4, -1);
            fprintf(f, "\t\t\t\t\t%i %i %i\n", id*5+4, id*5+1, -1);
            id++;
    };
    fprintf(f, "\t\t\t\t]\n\n");

    fprintf(f, "color Color { color [1 0 0, 0 1 0, 0 0 1] }\n\n");


    fprintf(f, "\t\t\t\tcolorIndex [\n");
    for (int c = shift; c<tcams->size(); c+=step)
    {
            fprintf(f, "\t\t\t\t\t%i\n", 1);
            fprintf(f, "\t\t\t\t\t%i\n", 1);
            fprintf(f, "\t\t\t\t\t%i\n", 1);
            fprintf(f, "\t\t\t\t\t%i\n", 1);

            fprintf(f, "\t\t\t\t\t%i\n", 1);
            fprintf(f, "\t\t\t\t\t%i\n", 1);
            fprintf(f, "\t\t\t\t\t%i\n", 1);
            fprintf(f, "\t\t\t\t\t%i\n", 1);
    };
    fprintf(f, "\t\t\t\t]\n\n");


    fprintf(f, "colorPerVertex FALSE\n\n");


    fprintf(f, "\t\t\t} # end of IndexedFaceSet\n\n");





    fprintf(f,"\t\t}]#end of children shape\n\t}#end of transform\n] # end of children\n} # end of group\n");
    */
}

void mv_output3D::printfGroupCamerasOmniSequnece(staticVector<int>* tcams, FILE* f, const multiviewParams* mp, float ps,
                                                 int camerasPerOneOmni, float upStep)
{
    fprintf(f, "Group {\nchildren [\n");

    for(int s = 0; s < camerasPerOneOmni; s++)
    {
        int ss = s + 1;
        if(s == camerasPerOneOmni - 1)
        {
            ss = 0;
        }
        for(int c = 0; c < tcams->size(); c++)
        {
            int rc = (*tcams)[c];

            if(rc % camerasPerOneOmni == ss)
            {
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

                Cl = Cl + Vl.normalize() * upStep;
                viewp = viewp + Vl.normalize() * upStep;

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

                fprintf(f, "Viewpoint {\n");
                fprintf(f, "\tfieldOfView %f \n", angleBetwV1andV2(point3d(mp->KArr[rc].m13, mp->KArr[rc].m11, 0.0),
                                                                   point3d(0.0, mp->KArr[rc].m11, 0.0)) *
                                                      (M_PI / 180.0) * 2.0);
                fprintf(f, "\tposition %f %f %f \n", viewp.x, viewp.y, viewp.z);
                fprintf(f, "\torientation %f %f %f %f \n", viewn.x, viewn.y, viewn.z, viewsim);
                fprintf(f, "\tdescription \"v%i\" \n", rc);
                fprintf(f, "\tjump TRUE \n");
                fprintf(f, "}\n");
            }
        }
    }

    fprintf(f, "Background {\nskyColor 1 1 1 \n}\n");
    fprintf(f, "\tTransform {\n");
    fprintf(f, "\t\ttranslation %g %g %g\n", 0.0, 0.0, 0.0);
    fprintf(f, "\t\tscale %g %g %g\n", 1.0, 1.0, 1.0);
    fprintf(f, "\n\t\tchildren [ Shape {\n");

    // cameras
    fprintf(f, "\t\t\tappearance Appearance {material Material {emissiveColor 1 1 1 }}\n");
    fprintf(f, "\t\t\tgeometry IndexedLineSet {\n");
    fprintf(f, "\t\t\t\tcoord Coordinate { point [\n");

    for(int c = 0; c < tcams->size(); c++)
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

        fprintf(f, "\t\t\t\t\t%7.8f %7.8f %7.8f \n", p.x, p.y, p.z);
        fprintf(f, "\t\t\t\t\t%7.8f %7.8f %7.8f \n", p.x + (n.x - vx.x - vy.x) * ps, p.y + (n.y - vx.y - vy.y) * ps,
                p.z + (n.z - vx.z - vy.z) * ps);
        fprintf(f, "\t\t\t\t\t%7.8f %7.8f %7.8f \n", p.x + (n.x - vx.x + vy.x) * ps, p.y + (n.y - vx.y + vy.y) * ps,
                p.z + (n.z - vx.z + vy.z) * ps);
        fprintf(f, "\t\t\t\t\t%7.8f %7.8f %7.8f \n", p.x + (n.x + vx.x + vy.x) * ps, p.y + (n.y + vx.y + vy.y) * ps,
                p.z + (n.z + vx.z + vy.z) * ps);
        fprintf(f, "\t\t\t\t\t%7.8f %7.8f %7.8f \n", p.x + (n.x + vx.x - vy.x) * ps, p.y + (n.y + vx.y - vy.y) * ps,
                p.z + (n.z + vx.z - vy.z) * ps);
    }

    fprintf(f, "\t\t\t\t]}\n\n");

    fprintf(f, "\t\t\t\tcoordIndex [\n");

    for(int c = 0; c < tcams->size(); c++)
    {
        fprintf(f, "\t\t\t\t\t%i %i %i\n", c * 5, c * 5 + 1, -1);
        fprintf(f, "\t\t\t\t\t%i %i %i\n", c * 5, c * 5 + 2, -1);
        fprintf(f, "\t\t\t\t\t%i %i %i\n", c * 5, c * 5 + 3, -1);
        fprintf(f, "\t\t\t\t\t%i %i %i\n", c * 5, c * 5 + 4, -1);

        fprintf(f, "\t\t\t\t\t%i %i %i\n", c * 5 + 1, c * 5 + 2, -1);
        fprintf(f, "\t\t\t\t\t%i %i %i\n", c * 5 + 2, c * 5 + 3, -1);
        fprintf(f, "\t\t\t\t\t%i %i %i\n", c * 5 + 3, c * 5 + 4, -1);
        fprintf(f, "\t\t\t\t\t%i %i %i\n", c * 5 + 4, c * 5 + 1, -1);
    }
    fprintf(f, "\t\t\t\t]\n\n");

    fprintf(f, "color Color { color [1 0 0, 0 1 0, 0 0 1] }\n\n");

    fprintf(f, "\t\t\t\tcolorIndex [\n");
    for(int c = 0; c < tcams->size(); c++)
    {
        fprintf(f, "\t\t\t\t\t%i\n", 1);
        fprintf(f, "\t\t\t\t\t%i\n", 1);
        fprintf(f, "\t\t\t\t\t%i\n", 1);
        fprintf(f, "\t\t\t\t\t%i\n", 1);

        fprintf(f, "\t\t\t\t\t%i\n", 1);
        fprintf(f, "\t\t\t\t\t%i\n", 1);
        fprintf(f, "\t\t\t\t\t%i\n", 1);
        fprintf(f, "\t\t\t\t\t%i\n", 1);
    }
    fprintf(f, "\t\t\t\t]\n\n");

    fprintf(f, "colorPerVertex FALSE\n\n");

    fprintf(f, "\t\t\t} # end of IndexedFaceSet\n\n");

    fprintf(f, "\t\t}]#end of children shape\n\t}#end of transform\n] # end of children\n} # end of group\n");
}

void mv_output3D::create_wrl(const multiviewParams* mp, int filerPerc, int winSizeHalf, int colorType, float simThr,
                             const std::string& wrlname, int maxPointsNum, bool savepset)
{
    create_wrl(mp, filerPerc, winSizeHalf, colorType, simThr, wrlname, maxPointsNum, savepset, 0, 1);
}
void mv_output3D::create_wrl(const multiviewParams* mp, int filerPerc, int winSizeHalf, int colorType, float simThr,
                             const std::string& wrlname, int maxPointsNum, bool savepset, int  /*camshift*/, int  /*camstep*/)
{
    // printf("creating wrl\n");

    ///////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////
    // check size and create pset

    int n = 0;
    for(int rc = 0; rc < mp->ncams; rc++)
    {
        n += getNumOrientedPointsFromFile(mp->indexes[rc], mp->mip);
    }

    printf("processing %i points in scene \n", n);

    if(savepset)
    {
        // write pset file
        const std::string psetname = mp->mip->newDir + wrlname + ".pset";
        FILE* f1 = fopen(psetname.c_str(), "w");

        int npset = 0;
        for(int rc = 0; rc < mp->ncams; rc++)
        {
            staticVector<orientedPoint>* ops = loadOrientedPointsFromFile(mp->indexes[rc], mp->mip);

            for(int i = 0; i < ops->size(); i++)
            {
                // if (i%psetstep==0)
                //{
                // print to pset file
                fprintf(f1, "%f %f %f %f %f %f\n", (*ops)[i].p.x, (*ops)[i].p.y, (*ops)[i].p.z, -(*ops)[i].n.x,
                        -(*ops)[i].n.y, -(*ops)[i].n.z);
                npset++;
                //};
            }

            delete ops;
        }

        printf("pset oriented points %i \n", npset);

        fclose(f1);
    }

    if(n < 100)
    {
        return;
    }

    int step = 1;

    if(n > maxPointsNum)
    {
        step = 2;
        while((float)n / (float)step > maxPointsNum)
        {
            step++;
        }

        printf("Warning: scene is too big ... taking only each %i-th point \n", step);
    }

    std::vector<seedPoint> seeds;
    std::vector<int> seedsrc;

    for(int rc = 0; rc < mp->ncams; rc++)
    {
        // load all seeds for reference camera
        staticVector<orientedPoint>* ops = loadOrientedPointsFromFile(mp->indexes[rc], mp->mip);

        for(int i = 0; i < ops->size(); i += step)
        {
            seedPoint sp;
            sp.op = (*ops)[i];
            seeds.push_back(sp);
            seedsrc.push_back(rc);
        }

        delete ops;
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////
    // filter outliers
    if(filerPerc > 0)
    {

        point3d cg = point3d(0.0, 0.0, 0.0);
        for(int i = 0; i < (int)seeds.size(); i++)
        {
            cg = cg + seeds[i].op.p;
        }
        cg = cg / (float)seeds.size();

        std::vector<sortedId> sid;
        sid.resize((int)seeds.size());
        for(int i = 0; i < (int)seeds.size(); i++)
        {
            sid[i].id = i;
            sid[i].value = (seeds[i].op.p - cg).size();
        }
        qsort(&sid[0], (int)seeds.size(), sizeof(sortedId), &compareSortedId);

        // cut out 10% of the most distacned
        int ns = (int)((float)seeds.size() * ((float)filerPerc / 100.0));
        int n = (int)seeds.size();
        std::vector<seedPoint> tmp;
        std::vector<int> tmprc;
        tmp = seeds;
        tmprc = seedsrc;
        seeds.resize(0);
        seedsrc.resize(0);
        seeds.reserve(n - ns);
        seedsrc.reserve(n - ns);

        for(int i = ns; i < n; i++)
        {
            seeds.push_back(tmp[sid[i].id]);
            seedsrc.push_back(tmprc[sid[i].id]);
        }
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////
    // sort it by rc to prevent often image reloading
    std::vector<sortedId> sid;
    sid.resize((int)seeds.size());
    for(int i = 0; i < (int)seeds.size(); i++)
    {
        sid[i].id = i;
        sid[i].value = seedsrc[i];
    }
    qsort(&sid[0], (int)seeds.size(), sizeof(sortedId), &compareSortedId);

    std::vector<seedPoint> tmp;
    std::vector<int> tmprc;
    tmp = seeds;
    tmprc = seedsrc;
    seeds.resize(0);
    seedsrc.resize(0);
    seeds.reserve((int)tmp.size());
    seedsrc.reserve((int)tmp.size());

    for(int i = 0; i < (int)tmp.size(); i++)
    {
        seeds.push_back(tmp[sid[i].id]);
        seedsrc.push_back(tmprc[sid[i].id]);
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////
    // create texture map
    int s = 2 * winSizeHalf + 1;
    int w = (int)floor(1200.0 / (float)s);
    int wp = w * s;
    int h = (int)floor((float)seeds.size() / w) + 1;
    int hp = h * s;

    const std::string fileName = mp->mip->newDir + wrlname + ".wrl";
    const std::string textName = mp->mip->newDir + wrlname + ".png";
    const std::string textFileName = wrlname + ".png";

    IplImage* bmp = cvCreateImage(cvSize(wp, hp), IPL_DEPTH_8U, 3);

    // save texture to file
    FILE* f = fopen(fileName.c_str(), "w");

    fprintf(f, "#VRML V2.0 utf8\n");

    fprintf(f, "Group {\nchildren [\n");
    fprintf(f, "\tWorldInfo {\n");
    fprintf(f, "\tinfo \"test\"\n");
    fprintf(f, "\t}\n\n");

    /*
    for (int rc = 0; rc<mp->ncams; rc++)
    {
            QVector		pos, at, up;
            QVector		rot_axis;
            float		rot_angle;
            float		viewsim;

            point3d viewp;
            point3d viewn;
            viewp = mp->CArr[rc];
            point3d Cl = viewp;

            point3d Xl = mp->RArr[rc].transpose() * point3d(1.0,0.0,0.0) * mp->KArr[rc].m13;
            Xl = Xl.normalize();
            point3d Yl = mp->RArr[rc].transpose() * point3d(0.0,1.0,0.0) * mp->KArr[rc].m23;
            Yl = Yl.normalize();
            point3d Zl = mp->RArr[rc].transpose() * point3d(0.0,0.0,1.0) * mp->KArr[rc].m11;
            Zl = Zl.normalize();

            point3d Ol = Cl + Zl;
            point3d Vl = Yl;
            Vl.x = -Vl.x;
            Vl.y = -Vl.y;
            Vl.z = -Vl.z;

            pos.x = Cl.x;	pos.y = Cl.y;	pos.z = Cl.z;
            at.x  = Ol.x;	at.y  = Ol.y;	at.z  = Ol.z;
            up.x  = Vl.x;	up.y  = Vl.y;	up.z  = Vl.z;

            Convert_Camera_Model(&pos, &at, &up, &rot_axis, &rot_angle);

            viewn.x = rot_axis.x;
            viewn.y = rot_axis.y;
            viewn.z = rot_axis.z;
            viewsim = rot_angle;


            fprintf(f, "Viewpoint {\n");
            fprintf(f, "\tfieldOfView 0.25 \n");
            fprintf(f, "\tposition %f %f %f \n",viewp.x,viewp.y,viewp.z);
            fprintf(f, "\torientation %f %f %f %f \n",viewn.x,viewn.y,viewn.z,viewsim);
            fprintf(f, "\tdescription \"v%i\" \n",rc);
            fprintf(f, "\tjump TRUE \n");
            fprintf(f, "}\n");
    };
*/

    fprintf(f, "Background {\nskyColor 1 1 1 \n}\n");
    fprintf(f, "\tTransform {\n");
    fprintf(f, "\t\ttranslation %g %g %g\n", 0.0, 0.0, 0.0);
    fprintf(f, "\t\tscale %g %g %g\n", 1.0, 1.0, 1.0);
    fprintf(f, "\n\t\tchildren [ Shape {\n");

    // points
    fprintf(f, "\t\t\tappearance Appearance {\n");

    fprintf(f, "\t\t\t\ttexture ImageTexture {\n");
    fprintf(f, "\t\t\t\t\turl \"%s\"\n", textFileName.c_str());
    // fprintf(f, "\t\t\t\t\trepeatS FALSE\n\t\t\t\t\trepeatT FALSE\n");
    fprintf(f, "\t\t\t\t}\n\n");

    fprintf(f, "\t\t\t\t\ttextureTransform TextureTransform {\n");
    fprintf(f, "\t\t\t\t\tscale %.9f %.9f\n", 1.0 / (float)wp, 1.0 / (float)hp);
    fprintf(f, "\t\t\t\t\ttranslation 0.5 0.5\n");
    fprintf(f, "\t\t\t\t}\n");

    /*
    fprintf(f, "\t\t\t\tmaterial Material {\n");
    fprintf(f, "\t\t\t\t\tambientIntensity\t1.0\n");
    fprintf(f, "\t\t\t\t\tdiffuseColor\t1.0 1.0 1.0\n");
    fprintf(f, "\t\t\t\t\temissiveColor\t0 0 0\n");
    fprintf(f, "\t\t\t\t\tshininess\t0.0\n");
    fprintf(f, "\t\t\t\t\tspecularColor\t0 0 0\n");
    fprintf(f, "\t\t\t\t\ttransparency\t0\n\t\t\t\t}\n");
    */

    fprintf(f, "\t\t\t}\n\n");

    fprintf(f, "\t\t\tgeometry IndexedFaceSet {\n");

    // compute colors for cameras
    srand((unsigned)time(nullptr));
    std::vector<point3d> colors;
    colors.resize(mp->ncams);
    for(int rc = 0; rc < mp->ncams; rc++)
    {
        unsigned char col = (unsigned char)(((float)rand() / (float)RAND_MAX) * 256.0);
        colors[rc].x = (float)col;
        col = (unsigned char)(((float)rand() / (float)RAND_MAX) * 256.0);
        colors[rc].y = (float)col;
        col = (unsigned char)(((float)rand() / (float)RAND_MAX) * 256.0);
        colors[rc].z = (float)col;
    }

    fprintf(f, "solid FALSE\n\n");

    fprintf(f, "ccw FALSE\n\n");

    fprintf(f, "creaseAngle 1.57\n\n");

    // std::vector<point3d> points;

    fprintf(f, "\t\t\t\tcoord Coordinate { point [\n");
    int oldrc = -1;
    IplImage* bmpimg = nullptr;

    for(int i = 0; i < (int)seeds.size(); i++)
    {
        float pixSize = mp->getCamPixelSize(seeds[i].op.p, seedsrc[i]);
        point3d vectx, vecty;
        point3d n = seeds[i].op.n;
        computeRotCS(&vectx, &vecty, &n);

        point3d p;
        p = seeds[i].op.p - vectx * pixSize * (float)winSizeHalf - vecty * pixSize * (float)winSizeHalf;
        fprintf(f, "\t\t\t\t\t%.9f %.9f %.9f \n", p.x, p.y, p.z);
        // points.push_back(p);

        p = seeds[i].op.p + vectx * pixSize * (float)winSizeHalf - vecty * pixSize * (float)winSizeHalf;
        fprintf(f, "\t\t\t\t\t%.9f %.9f %.9f \n", p.x, p.y, p.z);
        // points.push_back(p);

        p = seeds[i].op.p + vectx * pixSize * (float)winSizeHalf + vecty * pixSize * (float)winSizeHalf;
        fprintf(f, "\t\t\t\t\t%.9f %.9f %.9f \n", p.x, p.y, p.z);
        // points.push_back(p);

        p = seeds[i].op.p - vectx * pixSize * (float)winSizeHalf + vecty * pixSize * (float)winSizeHalf;
        fprintf(f, "\t\t\t\t\t%.9f %.9f %.9f \n", p.x, p.y, p.z);
        // points.push_back(p);

        int xt = (int)floor((float)i / (float)h);
        int yt = i - xt * h;

        // load original (color) image
        if(oldrc != seedsrc[i])
        {
            if(bmpimg != nullptr)
            {
                cvReleaseImage(&bmpimg);
            }
            const std::string fileNameStrIn = mv_getFileNamePrefix(mp->mip, seedsrc[i] + 1) + "." + mp->mip->imageExt;
            bmpimg = cvLoadImage(fileNameStrIn.c_str());
        }
        oldrc = seedsrc[i];

        if(colorType == 0)
        {
            for(int x = -winSizeHalf; x <= winSizeHalf; x++)
            {
                for(int y = -winSizeHalf; y <= winSizeHalf; y++)
                {
                    point3d X = seeds[i].op.p + vectx * pixSize * (float)x + vecty * pixSize * (float)y;
                    point2d pix;
                    mp->getPixelFor3DPoint(&pix, X, seedsrc[i]);
                    if((pix.x > g_border) && (pix.x < mp->mip->getWidth(seedsrc[i])) && (pix.y > g_border) &&
                       (pix.y < mp->mip->getHeight(seedsrc[i])))
                    {
                        // bmpimg.GetPixel(pix.x, pix.y, &c);

                        int xp = (int)floor(pix.x);
                        int yp = (int)floor(pix.y);

                        // precision to 4 decimal places
                        float ui = pix.x - floor(pix.x);
                        float vi = pix.y - floor(pix.y);

                        // bilinear interpolation of the pixel intensity value
                        CvScalar lug = cvGet2D(bmpimg, yp, xp);
                        CvScalar rug = cvGet2D(bmpimg, yp, (xp + 1));
                        CvScalar rdg = cvGet2D(bmpimg, (yp + 1), (xp + 1));
                        CvScalar ldg = cvGet2D(bmpimg, (yp + 1), xp);

                        float ugr = (float)lug.val[0] + (float)(rug.val[0] - lug.val[0]) * ui;
                        float ugg = (float)lug.val[1] + (float)(rug.val[1] - lug.val[1]) * ui;
                        float ugb = (float)lug.val[2] + (float)(rug.val[2] - lug.val[2]) * ui;

                        float dgr = (float)ldg.val[0] + (float)(rdg.val[0] - ldg.val[0]) * ui;
                        float dgg = (float)ldg.val[1] + (float)(rdg.val[1] - ldg.val[1]) * ui;
                        float dgb = (float)ldg.val[2] + (float)(rdg.val[2] - ldg.val[2]) * ui;

                        float gr = ugr + (dgr - ugr) * vi;
                        float gg = ugg + (dgg - ugg) * vi;
                        float gb = ugb + (dgb - ugb) * vi;

                        CvScalar c;
                        c.val[0] = gb;
                        c.val[1] = gg;
                        c.val[2] = gr;

                        xp = xt * s + x + winSizeHalf;
                        yp = yt * s + y + winSizeHalf;
                        cvSet2D(bmp, yp, xp, c);
                    }
                }
            }
        }

        if(colorType == 1)
        {
            for(int x = -winSizeHalf; x <= winSizeHalf; x++)
            {
                for(int y = -winSizeHalf; y <= winSizeHalf; y++)
                {
                    int xp = xt * s + x + winSizeHalf;
                    int yp = yt * s + y + winSizeHalf;
                    CvScalar c;
                    c.val[0] = (unsigned char)colors[seedsrc[i]].x;
                    c.val[1] = (unsigned char)colors[seedsrc[i]].y;
                    c.val[2] = (unsigned char)colors[seedsrc[i]].z;
                    cvSet2D(bmp, yp, xp, c);
                }
            }
        }

        if(colorType == 2)
        {
            for(int x = -winSizeHalf; x <= winSizeHalf; x++)
            {
                for(int y = -winSizeHalf; y <= winSizeHalf; y++)
                {
                    float mar = 1.0 - fabs(simThr);
                    unsigned char col = (unsigned char)((((-seeds[i].op.sim) - fabs(simThr)) / mar) * 256.0);
                    int xp = xt * s + x + winSizeHalf;
                    int yp = yt * s + y + winSizeHalf;
                    CvScalar c;
                    c.val[0] = 256 - col;
                    c.val[1] = col;
                    c.val[2] = 0;
                    cvSet2D(bmp, yp, xp, c);
                }
            }
        }
    }
    fprintf(f, "\t\t\t\t]}\n\n");

    fprintf(f, "\t\t\t\ttexCoord TextureCoordinate { point [\n");
    for(int i = 0; i < (int)seeds.size(); i++)
    {
        int xt = (int)floor((float)i / (float)h);
        int yt = i - xt * h;
        int xp = xt * s + 1;
        int yp = yt * s + 1;

        // fprintf(f, "\t\t\t\t\t%7.8f %7.8f \n", (float)(xt*s+1)/(float)(wp-1)	 , 1.0 - (float)(yt*s+1)/(float)(hp-1));
        // fprintf(f, "\t\t\t\t\t%7.8f %7.8f \n", (float)(xt*s+1)/(float)(wp-1)     , 1.0 -
        // (float)(yt*s+1+s-1)/(float)(hp-1));
        // fprintf(f, "\t\t\t\t\t%7.8f %7.8f \n", (float)(xt*s+1+s-1)/(float)(wp-1) , 1.0 -
        // (float)(yt*s+1+s-1)/(float)(hp-1));
        // fprintf(f, "\t\t\t\t\t%7.8f %7.8f \n", (float)(xt*s+1+s-1)/(float)(wp-1) , 1.0 -
        // (float)(yt*s+1)/(float)(hp-1));

        // fprintf(f, "\t\t\t\t\t%7.8f %7.8f \n", (float)(xt*s)/(float)(wp)   , 1.0 - (float)(yt*s)/(float)(hp));
        // fprintf(f, "\t\t\t\t\t%7.8f %7.8f \n", (float)(xt*s)/(float)(wp)   , 1.0 - (float)(yt*s+s)/(float)(hp));
        // fprintf(f, "\t\t\t\t\t%7.8f %7.8f \n", (float)(xt*s+s)/(float)(wp) , 1.0 - (float)(yt*s+s)/(float)(hp));
        // fprintf(f, "\t\t\t\t\t%7.8f %7.8f \n", (float)(xt*s+s)/(float)(wp) , 1.0 - (float)(yt*s)/(float)(hp));

        fprintf(f, "\t\t\t\t\t%i %i \n", xp, (hp - 1) - (yp));
        fprintf(f, "\t\t\t\t\t%i %i \n", xp + s - 3, (hp - 1) - (yp));
        fprintf(f, "\t\t\t\t\t%i %i \n", xp + s - 3, (hp - 1) - (yp + s - 3));
        fprintf(f, "\t\t\t\t\t%i %i \n", xp, (hp - 1) - (yp + s - 3));
    }
    fprintf(f, "\t\t\t\t]}\n\n");

    // fprintf(f, "\t\t\t\t ccw TRUE \n\n");
    fprintf(f, "\t\t\t\t colorPerVertex FALSE \n\n");

    fprintf(f, "\t\t\t\tcoordIndex [\n");
    for(int i = 0; i < (int)seeds.size(); i++)
    {
        // fprintf(f, "\t\t\t\t\t%i, %i %i %i %i\n",i*4+0, i*4+1, i*4+2, i*4+3, -1);
        fprintf(f, "\t\t\t\t\t%i, %i %i %i\n", i * 4 + 0, i * 4 + 1, i * 4 + 2, -1);
        fprintf(f, "\t\t\t\t\t%i, %i %i %i\n", i * 4 + 2, i * 4 + 3, i * 4 + 0, -1);
    }
    fprintf(f, "\t\t\t\t]\n\n");

    fprintf(f, "\t\t\t} # end of IndexedFaceSet\n\n");

    fprintf(f, "\t\t}]#end of children shape\n\t}#end of transform\n] # end of children\n} # end of group\n");

    // printfGroupCameras(f,mp,mp->getCamPixelSize(seeds[0].op.p, seedsrc[0]) * 10.0, camshift, camstep);

    fclose(f);

    if(bmpimg != nullptr)
    {
        cvReleaseImage(&bmpimg);
    }

    if(cvSaveImage(textName.c_str(), bmp) == 0)
        printf("Could not save: %s\n", textName.c_str());
    cvReleaseImage(&bmp);
}

void mv_output3D::printfGroupCamera_rc(FILE* f, const multiviewParams* mp, float ps, int rc)
{
    fprintf(f, "Group {\nchildren [\n");

    if(rc >= 0)
    {

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

        fprintf(f, "Viewpoint {\n");
        fprintf(f, "\tfieldOfView %f \n", angleBetwV1andV2(point3d(mp->KArr[rc].m13, mp->KArr[rc].m11, 0.0),
                                                           point3d(0.0, mp->KArr[rc].m11, 0.0)) *
                                              (M_PI / 180.0) * 2.0);
        fprintf(f, "\tposition %f %f %f \n", viewp.x, viewp.y, viewp.z);
        fprintf(f, "\torientation %f %f %f %f \n", viewn.x, viewn.y, viewn.z, viewsim);
        fprintf(f, "\tdescription \"v%i\" \n", rc);
        fprintf(f, "\tjump TRUE \n");
        fprintf(f, "}\n");
    }

    fprintf(f, "Background {\nskyColor 1 1 1 \n}\n");
    fprintf(f, "\tTransform {\n");
    fprintf(f, "\t\ttranslation %g %g %g\n", 0.0, 0.0, 0.0);
    fprintf(f, "\t\tscale %g %g %g\n", 1.0, 1.0, 1.0);
    fprintf(f, "\n\t\tchildren [ Shape {\n");

    // cameras
    fprintf(f, "\t\t\tappearance Appearance {material Material {emissiveColor 1 1 1 }}\n");
    fprintf(f, "\t\t\tgeometry IndexedLineSet {\n");
    fprintf(f, "\t\t\t\tcoord Coordinate { point [\n");

    if(rc >= 0)
    {
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

        fprintf(f, "\t\t\t\t\t%7.8f %7.8f %7.8f \n", p.x, p.y, p.z);
        fprintf(f, "\t\t\t\t\t%7.8f %7.8f %7.8f \n", p.x + (n.x - vx.x - vy.x) * ps, p.y + (n.y - vx.y - vy.y) * ps,
                p.z + (n.z - vx.z - vy.z) * ps);
        fprintf(f, "\t\t\t\t\t%7.8f %7.8f %7.8f \n", p.x + (n.x - vx.x + vy.x) * ps, p.y + (n.y - vx.y + vy.y) * ps,
                p.z + (n.z - vx.z + vy.z) * ps);
        fprintf(f, "\t\t\t\t\t%7.8f %7.8f %7.8f \n", p.x + (n.x + vx.x + vy.x) * ps, p.y + (n.y + vx.y + vy.y) * ps,
                p.z + (n.z + vx.z + vy.z) * ps);
        fprintf(f, "\t\t\t\t\t%7.8f %7.8f %7.8f \n", p.x + (n.x + vx.x - vy.x) * ps, p.y + (n.y + vx.y - vy.y) * ps,
                p.z + (n.z + vx.z - vy.z) * ps);
    }

    fprintf(f, "\t\t\t\t]}\n\n");

    fprintf(f, "\t\t\t\tcoordIndex [\n");

    if(rc >= 0)
    {

        fprintf(f, "\t\t\t\t\t%i %i %i\n", rc * 5, rc * 5 + 1, -1);
        fprintf(f, "\t\t\t\t\t%i %i %i\n", rc * 5, rc * 5 + 2, -1);
        fprintf(f, "\t\t\t\t\t%i %i %i\n", rc * 5, rc * 5 + 3, -1);
        fprintf(f, "\t\t\t\t\t%i %i %i\n", rc * 5, rc * 5 + 4, -1);

        fprintf(f, "\t\t\t\t\t%i %i %i\n", rc * 5 + 1, rc * 5 + 2, -1);
        fprintf(f, "\t\t\t\t\t%i %i %i\n", rc * 5 + 2, rc * 5 + 3, -1);
        fprintf(f, "\t\t\t\t\t%i %i %i\n", rc * 5 + 3, rc * 5 + 4, -1);
        fprintf(f, "\t\t\t\t\t%i %i %i\n", rc * 5 + 4, rc * 5 + 1, -1);
    }

    fprintf(f, "\t\t\t\t]\n\n");

    fprintf(f, "color Color { color [1 0 0, 0 1 0, 0 0 1] }\n\n");

    fprintf(f, "\t\t\t\tcolorIndex [\n");

    if(rc >= 0)
    {

        fprintf(f, "\t\t\t\t\t%i\n", 1);
        fprintf(f, "\t\t\t\t\t%i\n", 1);
        fprintf(f, "\t\t\t\t\t%i\n", 1);
        fprintf(f, "\t\t\t\t\t%i\n", 1);

        fprintf(f, "\t\t\t\t\t%i\n", 1);
        fprintf(f, "\t\t\t\t\t%i\n", 1);
        fprintf(f, "\t\t\t\t\t%i\n", 1);
        fprintf(f, "\t\t\t\t\t%i\n", 1);
    }

    fprintf(f, "\t\t\t\t]\n\n");

    fprintf(f, "colorPerVertex FALSE\n\n");

    fprintf(f, "\t\t\t} # end of IndexedFaceSet\n\n");

    fprintf(f, "\t\t}]#end of children shape\n\t}#end of transform\n] # end of children\n} # end of group\n");
}

void mv_output3D::create_wrl_pts(const multiviewParams* mp, const std::string& wrlname, int shift, int step)
{
    FILE* f = fopen(wrlname.c_str(), "w");

    fprintf(f, "#VRML V2.0 utf8\n");

    fprintf(f, "Group {\nchildren [\n");
    fprintf(f, "\tWorldInfo {\n");
    fprintf(f, "\tinfo \"test\"\n");
    fprintf(f, "\t}\n\n");
    fprintf(f, "Background {\nskyColor 1 1 1 \n}\n");

    fprintf(f, "Background {\nskyColor 1 1 1 \n}\n");
    fprintf(f, "\tTransform {\n");
    fprintf(f, "\t\ttranslation %g %g %g\n", 0.0, 0.0, 0.0);
    fprintf(f, "\t\tscale %g %g %g\n", 1.0, 1.0, 1.0);
    fprintf(f, "\n\t\tchildren [ Shape {\n");
    fprintf(f, "\n\t\tappearance Appearance { material Material {emissiveColor 1 1 1 }}\n");
    fprintf(f, "\t\t\tgeometry PointSet {\n");

    fprintf(f, "\t\t\t\tcoord Coordinate { point [\n");

    // printf("writing pts\n");
    long t1 = initEstimate();
    int npts = 0;
    float meanpixsize = 0.0;
    for(int rc = shift; rc < mp->ncams; rc += step)
    {
        // load all pts for reference camera
        staticVector<orientedPoint>* ops = loadOrientedPointsFromFile(mp->indexes[rc], mp->mip);
        for(int i = 0; i < ops->size(); i++)
        {
            point3d p = (*ops)[i].p;
            fprintf(f, "\t\t\t\t\t%7.8f %7.8f %7.8f \n", p.x, p.y, p.z);
            npts++;
            meanpixsize += mp->getCamPixelSize(p, rc);
        }
        delete ops;

        printfEstimate(rc, mp->ncams, t1);
    }
    finishEstimate();
    meanpixsize /= (float)npts;

    fprintf(f, "\t\t\t\t]}\n\n");

    fprintf(f, "\t\t\t\tcolor Color { color [\n");

    // printf("writing cls\n");

    IplImage* bmpimg = nullptr;

    t1 = initEstimate();
    for(int rc = shift; rc < mp->ncams; rc += step)
    {
        const std::string fileNameStrIn = mv_getFileNamePrefix(mp->mip, rc + 1) + "." + mp->mip->imageExt;
        bmpimg = cvLoadImage(fileNameStrIn.c_str());

        // load all pts for reference camera
        staticVector<orientedPoint>* ops = loadOrientedPointsFromFile(mp->indexes[rc], mp->mip);
        for(int i = 0; i < ops->size(); i++)
        {
            point3d p = (*ops)[i].p;

            point2d pix;
            mp->getPixelFor3DPoint(&pix, p, rc);
            if((pix.x > g_border) && (pix.x < mp->mip->getWidth(rc) - g_border) && (pix.y > g_border) &&
               (pix.y < mp->mip->getHeight(rc) - g_border))
            {
                int xp = (int)floor(pix.x);
                int yp = (int)floor(pix.y);

                // precision to 4 decimal places
                float ui = pix.x - floor(pix.x);
                float vi = pix.y - floor(pix.y);

                // bilinear interpolation of the pixel intensity value
                CvScalar lug = cvGet2D(bmpimg, yp, xp);
                CvScalar rug = cvGet2D(bmpimg, yp, (xp + 1));
                CvScalar rdg = cvGet2D(bmpimg, (yp + 1), (xp + 1));
                CvScalar ldg = cvGet2D(bmpimg, (yp + 1), xp);
                float ugr = (float)lug.val[0] + (float)(rug.val[0] - lug.val[0]) * ui;
                float ugg = (float)lug.val[1] + (float)(rug.val[1] - lug.val[1]) * ui;
                float ugb = (float)lug.val[2] + (float)(rug.val[2] - lug.val[2]) * ui;

                float dgr = (float)ldg.val[0] + (float)(rdg.val[0] - ldg.val[0]) * ui;
                float dgg = (float)ldg.val[1] + (float)(rdg.val[1] - ldg.val[1]) * ui;
                float dgb = (float)ldg.val[2] + (float)(rdg.val[2] - ldg.val[2]) * ui;

                float gb = ugr + (dgr - ugr) * vi;
                float gg = ugg + (dgg - ugg) * vi;
                float gr = ugb + (dgb - ugb) * vi;

                fprintf(f, "\t\t\t\t\t%7.8f %7.8f %7.8f \n", gr / 255.0f, gg / 255.0f, gb / 255.0f);
                // fprintf(f, "\t\t\t\t\t%7.8f %7.8f %7.8f \n", 0.0, 0.0, 0.0);
            }
            else
            {
                fprintf(f, "\t\t\t\t\t%7.8f %7.8f %7.8f \n", 0.0, 0.0, 0.0);
            }
        }

        cvReleaseImage(&bmpimg);

        delete ops;
        printfEstimate(rc, mp->ncams, t1);
    }
    finishEstimate();

    fprintf(f, "\t\t\t\t]}\n\n");

    fprintf(f, "\t\t\t} # end of PointSet\n\n");

    fprintf(f, "\t\t}]#end of children shape\n\t}#end of transform\n] # end of children\n} # end of group\n");

    // printfGroupCameras(f,mp, meanpixsize / 10.0, shift, step);
    // printfGroupCameras(f,mp, meanpixsize * 20, shift, step);

    fclose(f);

    // printf("nseeds: %i\n",npts);
}

void mv_output3D::create_wrl_pts(staticVector<point3d>* pts, const multiviewParams* mp, const std::string& wrlname)
{
    FILE* f = fopen(wrlname.c_str(), "w");
    fprintf(f, "#VRML V2.0 utf8\n");
    printf_wrl_pts(f, pts, mp);
    fclose(f);
}

void mv_output3D::printf_wrl_pts(FILE* f, staticVector<point3d>* pts, const multiviewParams* mp)
{
    fprintf(f, "Group {\nchildren [\n");
    fprintf(f, "\tWorldInfo {\n");
    fprintf(f, "\tinfo \"test\"\n");
    fprintf(f, "\t}\n\n");
    fprintf(f, "Background {\nskyColor 1 1 1 \n}\n");

    fprintf(f, "Background {\nskyColor 1 1 1 \n}\n");
    fprintf(f, "\tTransform {\n");
    fprintf(f, "\t\ttranslation %g %g %g\n", 0.0, 0.0, 0.0);
    fprintf(f, "\t\tscale %g %g %g\n", 1.0, 1.0, 1.0);
    fprintf(f, "\n\t\tchildren [ Shape {\n");
    fprintf(f, "\n\t\tappearance Appearance { material Material {emissiveColor 1 1 1 }}\n");
    fprintf(f, "\t\t\tgeometry PointSet {\n");

    fprintf(f, "\t\t\t\tcoord Coordinate { point [\n");

    // printf("writing pts\n");
    float meanpixsize = 0.0;
    for(int i = 0; i < pts->size(); i++)
    {
        point3d p = (*pts)[i];
        fprintf(f, "\t\t\t\t\t%7.8f %7.8f %7.8f \n", p.x, p.y, p.z);
        meanpixsize += mp->getCamPixelSize(p, 0);
    }
    meanpixsize /= (float)pts->size();

    fprintf(f, "\t\t\t\t]}\n\n");

    fprintf(f, "\t\t\t\tcolor Color { color [\n");

    // printf("writing cls\n");

    // load all pts for reference camera
    for(int i = 0; i < pts->size(); i++)
    {
        fprintf(f, "\t\t\t\t\t%7.8f %7.8f %7.8f \n", 0.0, 0.0, 0.0);
    }

    fprintf(f, "\t\t\t\t]}\n\n");

    fprintf(f, "\t\t\t} # end of PointSet\n\n");

    fprintf(f, "\t\t}]#end of children shape\n\t}#end of transform\n] # end of children\n} # end of group\n");

    // printfGroupCameras(f,mp, meanpixsize / 10.0, shift, step);
    // printfGroupCameras(f,mp, 0.01, 0, 1);

    // printf("npts: %i\n",pts->size());
}

void mv_output3D::create_wrl_pts_cls(staticVector<point3d>* pts, staticVector<voxel>* colors, const multiviewParams* mp,
                                     const std::string& wrlname)
{
    FILE* f = fopen(wrlname.c_str(), "w");
    fprintf(f, "#VRML V2.0 utf8\n");
    printf_wrl_pts_cls(f, pts, colors, mp);
    fclose(f);
}

void mv_output3D::printf_wrl_pts_cls(FILE* f, staticVector<point3d>* pts, staticVector<voxel>* colors, const multiviewParams*  /*mp*/)
{
    fprintf(f, "Group {\nchildren [\n");
    fprintf(f, "\tWorldInfo {\n");
    fprintf(f, "\tinfo \"test\"\n");
    fprintf(f, "\t}\n\n");
    fprintf(f, "Background {\nskyColor 1 1 1 \n}\n");

    fprintf(f, "Background {\nskyColor 1 1 1 \n}\n");
    fprintf(f, "\tTransform {\n");
    fprintf(f, "\t\ttranslation %g %g %g\n", 0.0, 0.0, 0.0);
    fprintf(f, "\t\tscale %g %g %g\n", 1.0, 1.0, 1.0);
    fprintf(f, "\n\t\tchildren [ Shape {\n");
    fprintf(f, "\n\t\tappearance Appearance { material Material {emissiveColor 1 1 1 }}\n");
    fprintf(f, "\t\t\tgeometry PointSet {\n");

    fprintf(f, "\t\t\t\tcoord Coordinate { point [\n");

    // printf("writing pts\n");
    for(int i = 0; i < pts->size(); i++)
    {
        point3d p = (*pts)[i];
        fprintf(f, "\t\t\t\t\t%7.8f %7.8f %7.8f \n", p.x, p.y, p.z);
    }

    fprintf(f, "\t\t\t\t]}\n\n");

    fprintf(f, "\t\t\t\tcolor Color { color [\n");

    // printf("writing cls\n");

    // load all pts for reference camera
    for(int i = 0; i < pts->size(); i++)
    {
        voxel v = (*colors)[i];
        fprintf(f, "\t\t\t\t\t%7.8f %7.8f %7.8f \n", (float)v.x / 255.0f, (float)v.y / 255.0f, (float)v.z / 255.0f);
    }

    fprintf(f, "\t\t\t\t]}\n\n");

    fprintf(f, "\t\t\t} # end of PointSet\n\n");

    fprintf(f, "\t\t}]#end of children shape\n\t}#end of transform\n] # end of children\n} # end of group\n");

    // printf("npts: %i\n",pts->size());
}

void mv_output3D::create_wrl_pts_nms(staticVector<point3d>* pts, staticVector<point3d>* nms, const multiviewParams*  /*mp*/,
                                     const std::string& wrlname, float r, float g, float b)
{
    FILE* f = fopen(wrlname.c_str(), "w");

    fprintf(f, "#VRML V2.0 utf8\n");

    fprintf(f, "Group {\nchildren [\n");
    fprintf(f, "\tWorldInfo {\n");
    fprintf(f, "\tinfo \"test\"\n");
    fprintf(f, "\t}\n\n");
    fprintf(f, "Background {\nskyColor 1 1 1 \n}\n");

    fprintf(f, "Background {\nskyColor 1 1 1 \n}\n");
    fprintf(f, "\tTransform {\n");
    fprintf(f, "\t\ttranslation %g %g %g\n", 0.0, 0.0, 0.0);
    fprintf(f, "\t\tscale %g %g %g\n", 1.0, 1.0, 1.0);
    fprintf(f, "\n\t\tchildren [ Shape {\n");
    fprintf(f, "\n\t\tappearance Appearance { material Material {emissiveColor 1 1 1 }}\n");
    fprintf(f, "\t\t\tgeometry IndexedLineSet {\n");

    ///////////////////////////////////////////////////////////////////////////////
    fprintf(f, "\t\t\t\tcoord Coordinate { point [\n");
    // printf("writing pts\n");
    for(int i = 0; i < pts->size(); i++)
    {
        point3d p = (*pts)[i];
        point3d n = (*nms)[i];
        fprintf(f, "\t\t\t\t\t%7.8f %7.8f %7.8f \n", p.x, p.y, p.z);
        // fprintf(f, "\t\t\t\t\t%7.8f %7.8f %7.8f \n", p.x, p.y, p.z);
        // fprintf(f, "\t\t\t\t\t%7.8f %7.8f %7.8f \n", p.x+0.1, p.y+0.1, p.z+0.1);
        fprintf(f, "\t\t\t\t\t%7.8f %7.8f %7.8f \n", p.x + n.x, p.y + n.y, p.z + n.z);
        // fprintf(f, "\t\t\t\t\t%7.8f %7.8f %7.8f \n", p.x+n.x*normalSize, p.y+n.y*normalSize, p.z+n.z*normalSize);
    }
    fprintf(f, "\t\t\t\t]}\n\n");

    ///////////////////////////////////////////////////////////////////////////////
    fprintf(f, "\t\t\t\tcoordIndex [\n");
    printf("writing normals\n");
    for(int i = 0; i < pts->size(); i++)
    {
        fprintf(f, "\t\t\t\t\t%i %i %i\n", 2 * i, 2 * i + 1, -1);
    }
    fprintf(f, "\t\t\t\t]\n\n");

    ///////////////////////////////////////////////////////////////////////////////
    fprintf(f, "color Color { color [%f %f %f, 0 1 0, 0 0 1] }\n\n", r, g, b);

    ///////////////////////////////////////////////////////////////////////////////
    fprintf(f, "\t\t\t\tcolorIndex [\n");
    for(int i = 0; i < pts->size(); i++)
    {
        fprintf(f, "\t\t\t\t\t%i\n", 0);
    }
    fprintf(f, "\t\t\t\t]\n\n");

    ///////////////////////////////////////////////////////////////////////////////
    fprintf(f, "colorPerVertex FALSE\n\n");

    fprintf(f, "\t\t\t} # end of IndexedLineSet\n\n");

    fprintf(f, "\t\t}]#end of children shape\n\t}#end of transform\n] # end of children\n} # end of group\n");

    /*
    stat3d s3d = stat3d();
    for (int i=0;i<pts->size();i++)
    {
            point3d p = (*pts)[i];
            s3d.update(&p);
    };

    {
            point3d p,n;
            s3d.computePlaneByPCA(&p,&n,100);
    };
    float meanpixsize = s3d.d[2]/(float)pts->size();
    */

    // printfGroupCameras(f,mp, 0.1, 0, 1);

    fclose(f);

    // printf("npts: %i\n",pts->size());
}

void mv_output3D::save_triangulation_to_wrl(const multiviewParams* mp, const std::string& inputFileName, const std::string& wrlFileName)
{

    FILE* f = fopen(wrlFileName.c_str(), "w");

    fprintf(f, "#VRML V2.0 utf8\n");

    fprintf(f, "Group {\nchildren [\n");
    fprintf(f, "\tWorldInfo {\n");
    fprintf(f, "\tinfo \"test\"\n");
    fprintf(f, "\t}\n\n");

    fprintf(f, "Background {\nskyColor 1 1 1 \n}\n");
    fprintf(f, "\tTransform {\n");
    fprintf(f, "\t\ttranslation %g %g %g\n", 0.0, 0.0, 0.0);
    fprintf(f, "\t\tscale %g %g %g\n", 1.0, 1.0, 1.0);
    fprintf(f, "\n\t\tchildren [ Shape {\n");
    fprintf(f, "\n\t\tappearance Appearance { material Material {diffuseColor 1 1 1 }}\n");
    fprintf(f, "\t\t\tgeometry IndexedFaceSet {\n");

    FILE* fin = fopen(inputFileName.c_str(), "r");
    int npts;
    fscanf(fin, "%i\n", &npts);

    fprintf(f, "\t\t\t\tcoord Coordinate { point [\n");
    fscanf(fin, "%i\n", &npts);
    // printf("writing pts\n");
    long t1 = initEstimate();
    float meanpixsize = 0.0;
    for(int i = 0; i < npts; i++)
    {
        float x, y, z;
        fscanf(fin, "%f %f %f\n", &x, &y, &z);
        fprintf(f, "\t\t\t\t\t%7.8f %7.8f %7.8f \n", x, y, z);
        printfEstimate(i, npts, t1);
        meanpixsize += mp->getCamPixelSize(point3d(x, y, z), 0);
    }
    finishEstimate();
    fprintf(f, "\t\t\t\t]}\n\n");
    meanpixsize /= (float)npts;

    fprintf(f, "\t\t\t\tcoordIndex[\n");
    int ntris;
    fscanf(fin, "%i\n", &ntris);
    // printf("writing pts\n");
    t1 = initEstimate();
    for(int i = 0; i < ntris; i++)
    {
        int x, y, z;
        fscanf(fin, "%i %i %i\n", &x, &y, &z);
        fprintf(f, "\t\t\t\t\t%i %i %i -1 \n", x, y, z);
        printfEstimate(i, ntris, t1);
    }
    finishEstimate();
    fprintf(f, "\t\t\t\t]\n\n");

    fprintf(f, "\t\t\t} # end of IndexedFaceSet\n\n");

    fprintf(f, "\t\t}]#end of children shape\n\t}#end of transform\n] # end of children\n} # end of group\n");

    // printfGroupCameras(f, mp, meanpixsize / 100.0, 0, 1);

    fclose(f);
    fclose(fin);

    // printf("nseeds: %i\n",npts);
}

void mv_output3D::convertPly2Wrl(const std::string& wrlFileName, const std::string& plyFileName, bool write_color, int  /*camshift*/,
                                 int  /*camstep*/)
{
    FILE* f = fopen(wrlFileName.c_str(), "w");

    fprintf(f, "#VRML V2.0 utf8\n");

    fprintf(f, "Group {\nchildren [\n");
    fprintf(f, "\tWorldInfo {\n");
    fprintf(f, "\tinfo \"test\"\n");
    fprintf(f, "\t}\n\n");

    fprintf(f, "Background {\nskyColor 1 1 1 \n}\n");
    fprintf(f, "\tTransform {\n");
    fprintf(f, "\t\ttranslation %g %g %g\n", 0.0, 0.0, 0.0);
    fprintf(f, "\t\tscale %g %g %g\n", 1.0, 1.0, 1.0);
    fprintf(f, "\n\t\tchildren [ Shape {\n");
    fprintf(f, "\n\t\tappearance Appearance { material Material {emissiveColor 1 1 1 }}\n");
    fprintf(f, "\t\t\tgeometry IndexedFaceSet {\n");

    FILE* fin = fopen(plyFileName.c_str(), "r");
    int npts, ntris, c;
    fscanf(fin, "ply%d", &c);
    fscanf(fin, "format ascii 1.0%d", &c);
    fscanf(fin, "comment Generated by multiRecon ( http://cmp.felk.cvut.cz/~jancom1 )%d", &c);
    fscanf(fin, "element vertex %d%d", &npts, &c);
    // printf("%i\n",npts);
    fscanf(fin, "property float x%d", &c);
    fscanf(fin, "property float y%d", &c);
    fscanf(fin, "property float z%d", &c);
    if(write_color)
    {
        fscanf(fin, "property uchar diffuse_red%d", &c);
        fscanf(fin, "property uchar diffuse_green%d", &c);
        fscanf(fin, "property uchar diffuse_blue%d", &c);
    }
    fscanf(fin, "element face %d%d", &ntris, &c);
    // printf("%i\n",ntris);
    fscanf(fin, "property list uchar int vertex_indices%d", &c);
    fscanf(fin, "end_header%d", &c);

    fprintf(f, "\t\t\t\tcoord Coordinate { point [\n");
    // printf("writing pts\n");
    long t1 = initEstimate();
    float meanpixsize = 0.0;
    for(int i = 0; i < npts; i++)
    {
        float x, y, z;
        int r, g, b;
        if(write_color)
        {
            fscanf(fin, "%f %f %f %i %i %i%d", &x, &y, &z, &r, &g, &b, &c);
        }
        else
        {
            fscanf(fin, "%f %f %f%d", &x, &y, &z, &c);
        }
        fprintf(f, "\t\t\t\t\t%7.8f %7.8f %7.8f \n", x, y, z);
        printfEstimate(i, npts, t1);
        meanpixsize += mp->getCamPixelSize(point3d(x, y, z), 0);
    }
    finishEstimate();
    fprintf(f, "\t\t\t\t]}\n\n");
    meanpixsize /= (float)npts;

    fprintf(f, "\t\t\t\tcoordIndex[\n");
    // printf("writing pts\n");
    t1 = initEstimate();
    for(int i = 0; i < ntris; i++)
    {
        int x, y, z, nn;
        fscanf(fin, "%i %i %i %i%d", &nn, &x, &y, &z, &c);
        // printf("%i %i %i\n",x,y,z);

        fprintf(f, "\t\t\t\t\t%i %i %i -1 \n", z, y, x);
        printfEstimate(i, ntris, t1);
    }
    finishEstimate();
    fprintf(f, "\t\t\t\t]\n\n");
    fclose(fin);

    if(write_color)
    {
        fin = fopen(plyFileName.c_str(), "r");
        fscanf(fin, "ply%d", &c);
        fscanf(fin, "format ascii 1.0%d", &c);
        fscanf(fin, "comment Generated by multiRecon ( http://cmp.felk.cvut.cz/~jancom1 )%d", &c);
        fscanf(fin, "element vertex %d%d", &npts, &c);
        fscanf(fin, "property float x%d", &c);
        fscanf(fin, "property float y%d", &c);
        fscanf(fin, "property float z%d", &c);
        if(write_color)
        {
            fscanf(fin, "property uchar diffuse_red%d", &c);
            fscanf(fin, "property uchar diffuse_green%d", &c);
            fscanf(fin, "property uchar diffuse_blue%d", &c);
        }
        fscanf(fin, "element face %d%d", &ntris, &c);
        fscanf(fin, "property list uchar int vertex_indices%d", &c);
        fscanf(fin, "end_header%d", &c);

        fprintf(f, "color Color { color [");
        // printf("writing colors\n");
        long t1 = initEstimate();
        for(int i = 0; i < npts; i++)
        {
            float x, y, z;
            int r, g, b;
            fscanf(fin, "%f %f %f %i %i %i%d", &x, &y, &z, &r, &g, &b, &c);
            fprintf(f, "\t\t\t\t\t%f %f %f \n", (float)r / 255.0, (float)g / 255.0, (float)b / 255.0);
            printfEstimate(i, npts, t1);
        }
        fprintf(f, "] }\n\n");
        fclose(fin);
    }

    fprintf(f, "\t\t\t} # end of IndexedFaceSet\n\n");

    fprintf(f, "\t\t}]#end of children shape\n\t}#end of transform\n] # end of children\n} # end of group\n");

    // printfGroupCameras(f, mp, meanpixsize / 200.0, camshift, camstep);

    fclose(f);

    // printf("nseeds: %i\n",npts);
}

void mv_output3D::create_wrl_for_delaunay_cut(const multiviewParams* mp, const std::string& inputFileNameT,
                                              const std::string& inputFileNameRC, const std::string& wrlFileName, const std::string& wrlDir,
                                              int camerasPerOneOmni)
{
    staticVector<int>* cams = new staticVector<int>(mp->ncams);
    for(int i = 0; i < mp->ncams; i++)
    {
        cams->push_back(i);
    }
    create_wrl_for_delaunay_cut(mp, inputFileNameT, inputFileNameRC, wrlFileName, wrlDir, camerasPerOneOmni, cams);
    delete cams;
}

void mv_output3D::create_wrl_for_delaunay_cut(const multiviewParams* mp, const std::string& inputFileNameT,
                                              const std::string& inputFileNameRC, const std::string& wrlFileName, const std::string& wrlDir,
                                              int  /*camerasPerOneOmni*/, staticVector<int>* tcams)
{
    FILE* f = fopen(wrlFileName.c_str(), "w");
    fprintf(f, "#VRML V2.0 utf8\n");
    // printf("creating wrl\n");

    const std::string imgsdir = wrlDir + "imgs";
    bfs::create_directory(imgsdir);

    int scaleFactor = 3;

    for(int C = 0; C < tcams->size(); C++)
    {
        int RC = (*tcams)[C];

        const std::string fileNameStr = mv_getFileNamePrefix(mp->mip, RC + 1) + "." + mp->mip->imageExt;

        IplImage* bmp = cvLoadImage(fileNameStr.c_str());
        if(bmp != nullptr)
        {

            // check image size...
            if((mp->mip->getWidth(RC) == bmp->width) && (mp->mip->getHeight(RC) == bmp->height))
            {
                IplImage* bmpr = cvCreateImage(
                    cvSize(mp->mip->getWidth(RC) / scaleFactor, mp->mip->getHeight(RC) / scaleFactor), IPL_DEPTH_8U, 3);
                cvResize(bmp, bmpr);
                const std::string fileNameStrOut = imgsdir + "/" + num2strFourDecimal(mp->indexes[RC]) + "._c.png";
                if(cvSaveImage(fileNameStrOut.c_str(), bmpr) == 0)
                    printf("Could not save: %s\n", fileNameStrOut.c_str());
                cvReleaseImage(&bmpr);
            }
            else
            {
                printf("!width, height \n");
                exit(EXIT_FAILURE);
            }
            cvReleaseImage(&bmp);
        }
        else
        {
            printf("!LoadBitmapA(g_streamFactory, %s, bmp) \n", fileNameStr.c_str());
            exit(EXIT_FAILURE);
        }
    }

    long t1 = initEstimate();
    int ncamsatonce = 10;
    for(int C = 0; C < tcams->size(); C = C + ncamsatonce)
    {

        int Ncamsatonce = std::min(C + ncamsatonce, tcams->size()) - C;

        int npnts, ntris;

        // load triangles and cameras indexes
        FILE* fin = fopen(inputFileNameT.c_str(), "r");
        fscanf(fin, "%i\n", &npnts); // 3
        fscanf(fin, "%i\n", &npnts);
        staticVector<point3d>* pnts = new staticVector<point3d>(npnts);
        for(int i = 0; i < npnts; i++)
        {
            float x, y, z;
            fscanf(fin, "%f %f %f\n", &x, &y, &z);
            pnts->push_back(point3d(x, y, z));
        }

        staticVector<staticVector<int>*>* usedpnts = new staticVector<staticVector<int>*>(Ncamsatonce);
        for(int i = 0; i < Ncamsatonce; i++)
        {
            (*usedpnts)[i] = new staticVector<int>(npnts);
            (*usedpnts)[i]->resize_with(npnts, -1);
        }

        fscanf(fin, "%i\n", &ntris);
        FILE* finrc = fopen(inputFileNameRC.c_str(), "r");
        fscanf(finrc, "%i\n", &ntris);

        staticVector<staticVector<voxel>*>* tris = new staticVector<staticVector<voxel>*>(Ncamsatonce);
        for(int i = 0; i < Ncamsatonce; i++)
        {
            (*tris)[i] = new staticVector<voxel>(ntris);
        }

        for(int i = 0; i < ntris; i++)
        {
            int x, y, z, rc;
            fscanf(fin, "%i %i %i\n", &x, &y, &z);
            fscanf(finrc, "%i\n", &rc);

            int c = tcams->indexOf(rc);
            if((c >= C) && (c < C + Ncamsatonce))
            {
                (*(*usedpnts)[c - C])[x] = 0;
                (*(*usedpnts)[c - C])[y] = 0;
                (*(*usedpnts)[c - C])[z] = 0;
                (*tris)[c - C]->push_back(voxel(x, y, z));
            }
        }

        for(int c = 0; c < Ncamsatonce; c++)
        {
            int j = 0;
            for(int i = 0; i < (*usedpnts)[c]->size(); i++)
            {
                if((*(*usedpnts)[c])[i] > -1)
                {
                    (*(*usedpnts)[c])[i] = j;
                    j++;
                }
            }
        }

        fclose(fin);
        fclose(finrc);

        /////////////////////////////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////////////////////////////


        for(int c = C; c < C + Ncamsatonce; c++)
        {
            int rc = (*tcams)[c];
            int w = mp->mip->getWidth(rc);
            int h = mp->mip->getHeight(rc);
            staticVector<int>* usedpntsrc = (*usedpnts)[c - C];
            staticVector<voxel>* trisrc = (*tris)[c - C];

            const std::string fileNameStrIn = "imgs/" + num2strFourDecimal(mp->indexes[rc]) + "._c.png";

            fprintf(f, "Group {\nchildren [\n");
            fprintf(f, "\tWorldInfo {\n");
            fprintf(f, "\tinfo \"test\"\n");
            fprintf(f, "\t}\n\n");

            fprintf(f, "Background {\nskyColor 1 1 1 \n}\n");
            fprintf(f, "\tTransform {\n");
            fprintf(f, "\t\ttranslation %g %g %g\n", 0.0, 0.0, 0.0);
            fprintf(f, "\t\tscale %g %g %g\n", 1.0, 1.0, 1.0);
            fprintf(f, "\n\t\tchildren [ Shape {\n");

            // points
            fprintf(f, "\t\t\tappearance Appearance {\n");

            fprintf(f, "\t\t\t\ttexture ImageTexture {\n");
            fprintf(f, "\t\t\t\t\turl \"%s\"\n", fileNameStrIn.c_str());
            // fprintf(f, "\t\t\t\t\trepeatS FALSE\n\t\t\t\t\trepeatT FALSE\n");
            fprintf(f, "\t\t\t\t}\n\n");

            fprintf(f, "\t\t\t\t\ttextureTransform TextureTransform {\n");
            fprintf(f, "\t\t\t\t\tscale %.9f %.9f\n", 1.0 / ((float)w / (float)scaleFactor),
                    1.0 / ((float)h / (float)scaleFactor));
            fprintf(f, "\t\t\t\t\ttranslation 0.5 0.5\n");
            fprintf(f, "\t\t\t\t}\n");

            fprintf(f, "\t\t\t}\n\n");

            fprintf(f, "\t\t\tgeometry IndexedFaceSet {\n");

            fprintf(f, "solid FALSE\n\n");

            fprintf(f, "ccw FALSE\n\n");

            fprintf(f, "creaseAngle 1.57\n\n");

            fprintf(f, "\t\t\t\tcoord Coordinate { point [\n");

            // write points

            // get points of triangles of rc
            for(int i = 0; i < npnts; i++)
            {
                if((*usedpntsrc)[i] > -1)
                {
                    fprintf(f, "\t\t\t\t\t%.9f %.9f %.9f \n", (*pnts)[i].x, (*pnts)[i].y, (*pnts)[i].z);
                }
            }
            fprintf(f, "\t\t\t\t]}\n\n");

            fprintf(f, "\t\t\t\ttexCoord TextureCoordinate { point [\n");
            for(int i = 0; i < npnts; i++)
            {
                if((*usedpntsrc)[i] > -1)
                {
                    point3d pt = (*pnts)[i];
                    pixel pix;
                    mp->getPixelFor3DPoint(&pix, pt, rc);
                    // fprintf(f, "\t\t\t\t\t%i %i \n", pix.x/scaleFactor, pix.y/scaleFactor);
                    fprintf(f, "\t\t\t\t\t%.9f %.9f \n", (float)pix.x / (float)scaleFactor,
                            (float)pix.y / (float)scaleFactor);
                }
            }
            fprintf(f, "\t\t\t\t]}\n\n");

            // fprintf(f, "\t\t\t\t ccw TRUE \n\n");
            fprintf(f, "\t\t\t\t colorPerVertex FALSE \n\n");

            fprintf(f, "\t\t\t\tcoordIndex [\n");

            for(int i = 0; i < trisrc->size(); i++)
            {
                fprintf(f, "\t\t\t\t\t%i, %i %i %i\n", (*usedpntsrc)[(*trisrc)[i].x], (*usedpntsrc)[(*trisrc)[i].y],
                        (*usedpntsrc)[(*trisrc)[i].z], -1);
            }
            fprintf(f, "\t\t\t\t]\n\n");

            fprintf(f, "\t\t\t} # end of IndexedFaceSet\n\n");

            fprintf(f, "\t\t}]#end of children shape\n\t}#end of transform\n] # end of children\n} # end of group\n");
        }

        delete pnts;
        for(int i = 0; i < Ncamsatonce; i++)
        {
            delete(*tris)[i];
            delete(*usedpnts)[i];
        }

        delete tris;
        delete usedpnts;

        printfEstimate(C, tcams->size(), t1);
    }
    finishEstimate();

    /*
    if (camerasPerOneOmni<=1) {
            printfGroupCameras(f, mp, 0.001);
    }else{
            printfGroupCamerasOmniSequnece(f,mp,0.001,camerasPerOneOmni);
    };
    */

    fclose(f);
}

void mv_output3D::create_ply_for_delaunay_cut_strecha(const multiviewParams* mp, const std::string& inputFileNameT,
                                                      const std::string& plyFileName)
{

    if(mp->verbose)
        printf("Writing to file %s \n", plyFileName.c_str());

    FILE* plyf = nullptr;

    bool dobinary = false;

    if(dobinary)
    {
        plyf = fopen(plyFileName.c_str(), "wb");
    }
    else
    {
        plyf = fopen(plyFileName.c_str(), "wb");
    }

    if(plyf == nullptr)
    {
        printf("Could not open output ply file %s\n", plyFileName.c_str());
        return;
    }

    fprintf(plyf, "ply%c", 10);
    if(dobinary)
    {
        fprintf(plyf, "format binary_little_endian 1.0%c", 10);
    }
    else
    {
        fprintf(plyf, "format ascii 1.0%c", 10);
    }

    // load triangles and cameras indexes
    int npnts, ntris;
    FILE* fin = fopen(inputFileNameT.c_str(), "r");
    fscanf(fin, "%i\n", &npnts); // 3
    fscanf(fin, "%i\n", &npnts);
    for(int i = 0; i < npnts; i++)
    {
        float x, y, z;
        fscanf(fin, "%f %f %f\n", &x, &y, &z);
    }
    fscanf(fin, "%i\n", &ntris);
    fclose(fin);

    fprintf(plyf, "comment Generated by multiRecon ( http://cmp.felk.cvut.cz/~jancom1 )%c", 10);
    fprintf(plyf, "element vertex %d%c", npnts, 10);
    fprintf(plyf, "property float x%c", 10);
    fprintf(plyf, "property float y%c", 10);
    fprintf(plyf, "property float z%c", 10);
    fprintf(plyf, "element face %d%c", ntris, 10);
    fprintf(plyf, "property list uchar int vertex_indices%c", 10);
    fprintf(plyf, "end_header%c", 10);

    fin = fopen(inputFileNameT.c_str(), "r");
    fscanf(fin, "%i\n", &npnts); // 3
    fscanf(fin, "%i\n", &npnts);
    for(int i = 0; i < npnts; i++)
    {
        float x, y, z;
        fscanf(fin, "%f %f %f\n", &x, &y, &z);
        if(dobinary)
        {
            float pos[3];
            pos[0] = (float)(x);
            pos[1] = (float)(y);
            pos[2] = (float)(z);
            fwrite(&pos[0], sizeof(float), 3, plyf);
        }
        else
        {
            fprintf(plyf, "%f %f %f%c", (float)(x), (float)(y), (float)(z), 10);
        }
    }

    fscanf(fin, "%i\n", &ntris);

    for(int i = 0; i < ntris; i++)
    {
        int x, y, z;
        fscanf(fin, "%i %i %i\n", &x, &y, &z);

        if(dobinary)
        {
            unsigned char npts = 3;
            fwrite(&npts, sizeof(unsigned char), 1, plyf);
            int points[3];
            points[0] = x;
            points[1] = y;
            points[2] = z;
            fwrite(&points[0], sizeof(int), 3, plyf);
        }
        else
        {
            fprintf(plyf, "3 %d %d %d", x, y, z);

            if(i < ntris - 1)
            {
                fprintf(plyf, "%c", 10);
            }
        }
    }

    fclose(fin);
    fclose(plyf);
}

void mv_output3D::saveMvMeshToWrl(staticVectorBool* triSource, mv_mesh* me, const std::string& wrlFileName)
{
    FILE* f = fopen(wrlFileName.c_str(), "w");

    fprintf(f, "#VRML V2.0 utf8\n");

    fprintf(f, "Group {\nchildren [\n");
    fprintf(f, "\tWorldInfo {\n");
    fprintf(f, "\tinfo \"test\"\n");
    fprintf(f, "\t}\n\n");

    fprintf(f, "Background {\nskyColor 1 1 1 \n}\n");
    fprintf(f, "\tTransform {\n");
    fprintf(f, "\t\ttranslation %g %g %g\n", 0.0, 0.0, 0.0);
    fprintf(f, "\t\tscale %g %g %g\n", 1.0, 1.0, 1.0);
    fprintf(f, "\n\t\tchildren [ Shape {\n");
    fprintf(f, "\n\t\tappearance Appearance { material Material {diffuseColor 1 1 1 }}\n");
    fprintf(f, "\t\t\tgeometry IndexedFaceSet {\n");

    int npts = me->pts->size();
    int ntris = me->tris->size();

    fprintf(f, "\t\t\t\tcoord Coordinate { point [\n");
    // printf("writing pts\n");
    long t1 = initEstimate();
    float meanpixsize = 0.0;
    for(int i = 0; i < npts; i++)
    {
        fprintf(f, "\t\t\t\t\t%7.8f %7.8f %7.8f \n", (*me->pts)[i].x, (*me->pts)[i].y, (*me->pts)[i].z);
        printfEstimate(i, npts, t1);
        meanpixsize += mp->getCamPixelSize((*me->pts)[i], 0);
    }
    finishEstimate();
    fprintf(f, "\t\t\t\t]}\n\n");
    meanpixsize /= (float)npts;

    fprintf(f, "\t\t\t\tcoordIndex[\n");
    t1 = initEstimate();
    for(int i = 0; i < ntris; i++)
    {
        fprintf(f, "\t\t\t\t\t%i %i %i -1 \n", (*me->tris)[i].i[0], (*me->tris)[i].i[1], (*me->tris)[i].i[2]);
        printfEstimate(i, ntris, t1);
    }
    finishEstimate();
    fprintf(f, "\t\t\t\t]\n\n");

    fprintf(f, "colorPerVertex FALSE\n\n");

    fprintf(f, "color Color { color [");
    // printf("writing colors\n");
    for(int i = 0; i < ntris; i++)
    {
        if((*triSource)[i])
        {
            fprintf(f, "\t\t\t\t\t%f %f %f \n", 1.0, 0.0, 0.0);
        }
        else
        {
            fprintf(f, "\t\t\t\t\t%f %f %f \n", 0.0, 1.0, 0.0);
        }
    }
    fprintf(f, "] }\n\n");

    fprintf(f, "\t\t\t} # end of IndexedFaceSet\n\n");

    fprintf(f, "\t\t}]#end of children shape\n\t}#end of transform\n] # end of children\n} # end of group\n");

    // printfGroupCameras(f, mp, me->computeAverageEdgeLength()*5.0f, 0, 1);

    fclose(f);

    // printf("npts: %i\n",npts);
}

void mv_output3D::saveMvMeshToWrl(mv_mesh* me, const std::string& wrlFileName, staticVector<rgb>* colors, bool solid, bool colorPerVertex)
{
    FILE* f = fopen(wrlFileName.c_str(), "w");

    fprintf(f, "#VRML V2.0 utf8\n");

    fprintf(f, "Group {\nchildren [\n");
    fprintf(f, "\tWorldInfo {\n");
    fprintf(f, "\tinfo \"test\"\n");
    fprintf(f, "\t}\n\n");

    fprintf(f, "Background {\nskyColor 1 1 1 \n}\n");
    fprintf(f, "\tTransform {\n");
    fprintf(f, "\t\ttranslation %g %g %g\n", 0.0, 0.0, 0.0);
    fprintf(f, "\t\tscale %g %g %g\n", 1.0, 1.0, 1.0);
    fprintf(f, "\n\t\tchildren [ Shape {\n");
    fprintf(f, "\n\t\tappearance Appearance { material Material {diffuseColor 1 1 1 }}\n");
    fprintf(f, "\t\t\tgeometry IndexedFaceSet {\n");

    if(solid)
    {
        fprintf(f, "solid TRUE\n\n");
    }
    else
    {
        fprintf(f, "solid FALSE\n\n");
    }

    int npts = me->pts->size();
    int ntris = me->tris->size();

    fprintf(f, "\t\t\t\tcoord Coordinate { point [\n");
    // printf("writing pts\n");
    long t1 = initEstimate();
    for(int i = 0; i < npts; i++)
    {
        fprintf(f, "\t\t\t\t\t%7.8f %7.8f %7.8f \n", (*me->pts)[i].x, (*me->pts)[i].y, (*me->pts)[i].z);
        printfEstimate(i, npts, t1);
    }
    finishEstimate();
    fprintf(f, "\t\t\t\t]}\n\n");

    fprintf(f, "\t\t\t\tcoordIndex[\n");
    t1 = initEstimate();
    for(int i = 0; i < ntris; i++)
    {
        fprintf(f, "\t\t\t\t\t%i %i %i -1 \n", (*me->tris)[i].i[0], (*me->tris)[i].i[1], (*me->tris)[i].i[2]);
        printfEstimate(i, ntris, t1);
    }
    finishEstimate();
    fprintf(f, "\t\t\t\t]\n\n");

    if(colors)
    {
        if(colorPerVertex)
        {
            fprintf(f, "colorPerVertex TRUE\n\n");
        }
        else
        {
            fprintf(f, "colorPerVertex FALSE\n\n"); // colorPerFace
        }

        fprintf(f, "color Color { color [");
        // printf("writing colors\n");
        for(int i = 0; i < ntris; i++)
        {
            rgb col = (*colors)[i];
            fprintf(f, "\t\t\t\t\t%f %f %f \n", (float)col.r / 255.0f, (float)col.g / 255.0f, (float)col.b / 255.0f);
        }
        fprintf(f, "] }\n\n");
    }
    fprintf(f, "\t\t\t} # end of IndexedFaceSet\n\n");

    fprintf(f, "\t\t}]#end of children shape\n\t}#end of transform\n] # end of children\n} # end of group\n");

    // printfGroupCameras(f, mp, me->computeAverageEdgeLength()*5.0f, 0, 1);

    fclose(f);

    // printf("npts: %i\n",npts);
}

void mv_output3D::saveMvMeshToWrl(staticVector<float>* ptsValues, float minVal, float maxVal, mv_mesh* me,
                                  const std::string& wrlFileName, bool solid)
{
    FILE* f = fopen(wrlFileName.c_str(), "w");

    fprintf(f, "#VRML V2.0 utf8\n");

    fprintf(f, "Group {\nchildren [\n");
    fprintf(f, "\tWorldInfo {\n");
    fprintf(f, "\tinfo \"test\"\n");
    fprintf(f, "\t}\n\n");

    fprintf(f, "Background {\nskyColor 1 1 1 \n}\n");
    fprintf(f, "\tTransform {\n");
    fprintf(f, "\t\ttranslation %g %g %g\n", 0.0, 0.0, 0.0);
    fprintf(f, "\t\tscale %g %g %g\n", 1.0, 1.0, 1.0);
    fprintf(f, "\n\t\tchildren [ Shape {\n");
    fprintf(f, "\n\t\tappearance Appearance { material Material {diffuseColor 1 1 1 }}\n");
    fprintf(f, "\t\t\tgeometry IndexedFaceSet {\n");

    if(solid)
    {
        fprintf(f, "solid TRUE\n\n");
    }
    else
    {
        fprintf(f, "solid FALSE\n\n");
    }

    int npts = me->pts->size();
    int ntris = me->tris->size();

    fprintf(f, "\t\t\t\tcoord Coordinate { point [\n");
    // printf("writing pts\n");
    long t1 = initEstimate();
    for(int i = 0; i < npts; i++)
    {
        fprintf(f, "\t\t\t\t\t%7.8f %7.8f %7.8f \n", (*me->pts)[i].x, (*me->pts)[i].y, (*me->pts)[i].z);
        printfEstimate(i, npts, t1);
    }
    finishEstimate();
    fprintf(f, "\t\t\t\t]}\n\n");

    fprintf(f, "\t\t\t\tcoordIndex[\n");
    t1 = initEstimate();
    for(int i = 0; i < ntris; i++)
    {
        fprintf(f, "\t\t\t\t\t%i %i %i -1 \n", (*me->tris)[i].i[0], (*me->tris)[i].i[1], (*me->tris)[i].i[2]);
        printfEstimate(i, ntris, t1);
    }
    finishEstimate();
    fprintf(f, "\t\t\t\t]\n\n");

    fprintf(f, "colorPerVertex TRUE\n\n");

    fprintf(f, "color Color { color [");
    // printf("writing colors\n");
    for(int i = 0; i < npts; i++)
    {
        float s = 1.0f - (maxVal - std::max(minVal, (*ptsValues)[i])) / (maxVal - minVal);
        rgb col = getColorFromJetColorMap(s);
        fprintf(f, "\t\t\t\t\t%f %f %f \n", (float)col.r / 255.0f, (float)col.g / 255.0f, (float)col.b / 255.0f);
    }
    fprintf(f, "] }\n\n");

    fprintf(f, "\t\t\t} # end of IndexedFaceSet\n\n");

    fprintf(f, "\t\t}]#end of children shape\n\t}#end of transform\n] # end of children\n} # end of group\n");

    // printfGroupCameras(f, mp, me->computeAverageEdgeLength()*5.0f, 0, 1);

    fclose(f);

    // printf("npts: %i\n",npts);
}

void mv_output3D::saveMvMeshToWrlPtsColors(staticVector<rgb>* ptsColors, mv_mesh* me, const std::string& wrlFileName,
                                           bool solid)
{
    FILE* f = fopen(wrlFileName.c_str(), "w");

    fprintf(f, "#VRML V2.0 utf8\n");

    fprintf(f, "Group {\nchildren [\n");
    fprintf(f, "\tWorldInfo {\n");
    fprintf(f, "\tinfo \"test\"\n");
    fprintf(f, "\t}\n\n");

    fprintf(f, "Background {\nskyColor 1 1 1 \n}\n");
    fprintf(f, "\tTransform {\n");
    fprintf(f, "\t\ttranslation %g %g %g\n", 0.0, 0.0, 0.0);
    fprintf(f, "\t\tscale %g %g %g\n", 1.0, 1.0, 1.0);
    fprintf(f, "\n\t\tchildren [ Shape {\n");
    fprintf(f, "\n\t\tappearance Appearance { material Material {diffuseColor 1 1 1 }}\n");
    fprintf(f, "\t\t\tgeometry IndexedFaceSet {\n");

    if(solid)
    {
        fprintf(f, "solid TRUE\n\n");
    }
    else
    {
        fprintf(f, "solid FALSE\n\n");
    }

    int npts = me->pts->size();
    int ntris = me->tris->size();

    fprintf(f, "\t\t\t\tcoord Coordinate { point [\n");
    // printf("writing pts\n");
    long t1 = initEstimate();
    for(int i = 0; i < npts; i++)
    {
        fprintf(f, "\t\t\t\t\t%7.8f %7.8f %7.8f \n", (*me->pts)[i].x, (*me->pts)[i].y, (*me->pts)[i].z);
        printfEstimate(i, npts, t1);
    }
    finishEstimate();
    fprintf(f, "\t\t\t\t]}\n\n");

    fprintf(f, "\t\t\t\tcoordIndex[\n");
    t1 = initEstimate();
    for(int i = 0; i < ntris; i++)
    {
        fprintf(f, "\t\t\t\t\t%i %i %i -1 \n", (*me->tris)[i].i[0], (*me->tris)[i].i[1], (*me->tris)[i].i[2]);
        printfEstimate(i, ntris, t1);
    }
    finishEstimate();
    fprintf(f, "\t\t\t\t]\n\n");

    fprintf(f, "colorPerVertex TRUE\n\n");

    fprintf(f, "color Color { color [");
    // printf("writing colors\n");
    for(int i = 0; i < npts; i++)
    {
        rgb col = (*ptsColors)[i];
        fprintf(f, "\t\t\t\t\t%f %f %f \n", (float)col.r / 255.0f, (float)col.g / 255.0f, (float)col.b / 255.0f);
    }
    fprintf(f, "] }\n\n");

    fprintf(f, "\t\t\t} # end of IndexedFaceSet\n\n");

    fprintf(f, "\t\t}]#end of children shape\n\t}#end of transform\n] # end of children\n} # end of group\n");

    // printfGroupCameras(f, mp, me->computeAverageEdgeLength()*5.0f, 0, 1);

    fclose(f);

    // printf("npts: %i\n",npts);
}

void mv_output3D::saveMvMeshToWrlPlusPts(staticVector<point3d>* pts, staticVector<rgb>* triColors, mv_mesh* me,
                                         const std::string& wrlFileName)
{
    FILE* f = fopen(wrlFileName.c_str(), "w");

    fprintf(f, "#VRML V2.0 utf8\n");

    fprintf(f, "Group {\nchildren [\n");
    fprintf(f, "\tWorldInfo {\n");
    fprintf(f, "\tinfo \"test\"\n");
    fprintf(f, "\t}\n\n");

    fprintf(f, "Background {\nskyColor 1 1 1 \n}\n");
    fprintf(f, "\tTransform {\n");
    fprintf(f, "\t\ttranslation %g %g %g\n", 0.0, 0.0, 0.0);
    fprintf(f, "\t\tscale %g %g %g\n", 1.0, 1.0, 1.0);
    fprintf(f, "\n\t\tchildren [ Shape {\n");
    fprintf(f, "\n\t\tappearance Appearance { material Material {diffuseColor 1 1 1 }}\n");
    fprintf(f, "\t\t\tgeometry IndexedFaceSet {\n");

    int npts = me->pts->size();
    int ntris = me->tris->size();

    fprintf(f, "\t\t\t\tcoord Coordinate { point [\n");
    // printf("writing pts\n");
    long t1 = initEstimate();
    float meanpixsize = 0.0;
    for(int i = 0; i < npts; i++)
    {
        fprintf(f, "\t\t\t\t\t%7.8f %7.8f %7.8f \n", (*me->pts)[i].x, (*me->pts)[i].y, (*me->pts)[i].z);
        printfEstimate(i, npts, t1);
        meanpixsize += mp->getCamPixelSize((*me->pts)[i], 0);
    }
    finishEstimate();
    fprintf(f, "\t\t\t\t]}\n\n");
    meanpixsize /= (float)npts;

    fprintf(f, "\t\t\t\tcoordIndex[\n");
    t1 = initEstimate();
    for(int i = 0; i < ntris; i++)
    {
        fprintf(f, "\t\t\t\t\t%i %i %i -1 \n", (*me->tris)[i].i[0], (*me->tris)[i].i[1], (*me->tris)[i].i[2]);
        printfEstimate(i, ntris, t1);
    }
    finishEstimate();
    fprintf(f, "\t\t\t\t]\n\n");

    fprintf(f, "colorPerVertex FALSE\n\n");

    fprintf(f, "color Color { color [");
    // printf("writing colors\n");
    for(int i = 0; i < ntris; i++)
    {
        rgb col = (*triColors)[i];
        fprintf(f, "\t\t\t\t\t%f %f %f \n", (float)col.r / 255.0f, (float)col.g / 255.0f, (float)col.b / 255.0f);
    }
    fprintf(f, "] }\n\n");

    fprintf(f, "\t\t\t} # end of IndexedFaceSet\n\n");

    fprintf(f, "\t\t}]#end of children shape\n\t}#end of transform\n] # end of children\n} # end of group\n");

    fprintf(f, "Group {\nchildren [\n");
    fprintf(f, "\tWorldInfo {\n");
    fprintf(f, "\tinfo \"test\"\n");
    fprintf(f, "\t}\n\n");

    fprintf(f, "Background {\nskyColor 1 1 1 \n}\n");
    fprintf(f, "\tTransform {\n");
    fprintf(f, "\t\ttranslation %g %g %g\n", 0.0, 0.0, 0.0);
    fprintf(f, "\t\tscale %g %g %g\n", 1.0, 1.0, 1.0);
    fprintf(f, "\n\t\tchildren [ Shape {\n");
    fprintf(f, "\n\t\tappearance Appearance { material Material {emissiveColor 1 1 1 }}\n");
    fprintf(f, "\t\t\tgeometry PointSet {\n");

    fprintf(f, "\t\t\t\tcoord Coordinate { point [\n");

    // printf("writing pts\n");
    for(int i = 0; i < pts->size(); i++)
    {
        point3d p = (*pts)[i];
        fprintf(f, "\t\t\t\t\t%7.8f %7.8f %7.8f \n", p.x, p.y, p.z);
    }

    fprintf(f, "\t\t\t\t]}\n\n");

    fprintf(f, "\t\t\t\tcolor Color { color [\n");

    // printf("writing cls\n");

    // load all pts for reference camera
    for(int i = 0; i < pts->size(); i++)
    {
        fprintf(f, "\t\t\t\t\t%7.8f %7.8f %7.8f \n", 0.0, 1.0, 0.0);
    }

    fprintf(f, "\t\t\t\t]}\n\n");

    fprintf(f, "\t\t\t} # end of PointSet\n\n");

    fprintf(f, "\t\t}]#end of children shape\n\t}#end of transform\n] # end of children\n} # end of group\n");

    // printfGroupCameras(f, mp, me->computeAverageEdgeLength()*5.0f, 0, 1);

    fclose(f);

    // printf("npts: %i\n",npts);
}

void mv_output3D::printfMvMeshToWrl(FILE* f, rgb& colorOfTris, mv_mesh* me)
{
    fprintf(f, "Group {\nchildren [\n");
    fprintf(f, "\tWorldInfo {\n");
    fprintf(f, "\tinfo \"test\"\n");
    fprintf(f, "\t}\n\n");

    fprintf(f, "Background {\nskyColor 1 1 1 \n}\n");
    fprintf(f, "\tTransform {\n");
    fprintf(f, "\t\ttranslation %g %g %g\n", 0.0, 0.0, 0.0);
    fprintf(f, "\t\tscale %g %g %g\n", 1.0, 1.0, 1.0);
    fprintf(f, "\n\t\tchildren [ Shape {\n");
    fprintf(f, "\n\t\tappearance Appearance { material Material {diffuseColor 1 1 1 }}\n");
    fprintf(f, "\t\t\tgeometry IndexedFaceSet {\n");

    int npts = me->pts->size();
    int ntris = me->tris->size();

    fprintf(f, "\t\t\t\tcoord Coordinate { point [\n");
    // printf("writing pts\n");
    long t1 = initEstimate();
    float meanpixsize = 0.0;
    for(int i = 0; i < npts; i++)
    {
        fprintf(f, "\t\t\t\t\t%7.8f %7.8f %7.8f \n", (*me->pts)[i].x, (*me->pts)[i].y, (*me->pts)[i].z);
        printfEstimate(i, npts, t1);
        meanpixsize += mp->getCamPixelSize((*me->pts)[i], 0);
    }
    finishEstimate();
    fprintf(f, "\t\t\t\t]}\n\n");
    meanpixsize /= (float)npts;

    fprintf(f, "\t\t\t\tcoordIndex[\n");
    t1 = initEstimate();
    for(int i = 0; i < ntris; i++)
    {
        fprintf(f, "\t\t\t\t\t%i %i %i -1 \n", (*me->tris)[i].i[0], (*me->tris)[i].i[1], (*me->tris)[i].i[2]);
        printfEstimate(i, ntris, t1);
    }
    finishEstimate();
    fprintf(f, "\t\t\t\t]\n\n");

    fprintf(f, "colorPerVertex FALSE\n\n");

    fprintf(f, "color Color { color [");
    // printf("writing colors\n");
    for(int i = 0; i < ntris; i++)
    {
        fprintf(f, "\t\t\t\t\t%f %f %f \n", (float)colorOfTris.r / 255.0f, (float)colorOfTris.g / 255.0f,
                (float)colorOfTris.b / 255.0f);
    }
    fprintf(f, "] }\n\n");

    fprintf(f, "\t\t\t} # end of IndexedFaceSet\n\n");

    fprintf(f, "\t\t}]#end of children shape\n\t}#end of transform\n] # end of children\n} # end of group\n");
}

void mv_output3D::saveMvMeshToWrl(int scaleFactor, mv_mesh* me, staticVector<Color>* camsColorScales,
                                  staticVector<Color>* camsColorShifts, staticVector<int>* rcTris,
                                  const multiviewParams* mp, const std::string& wrlFileName, const std::string& wrlDir,
                                  int  /*camerasPerOneOmni*/)
{
    FILE* f = fopen(wrlFileName.c_str(), "w");
    fprintf(f, "#VRML V2.0 utf8\n");
    // printf("creating wrl\n");

    const std::string imgsdir = wrlDir + "imgs";
    bfs::create_directory(imgsdir);

    staticVector<int>* usedcams = new staticVector<int>(mp->ncams);
    for(int i = 0; i < rcTris->size(); i++)
    {
        int rc = (*rcTris)[i];
        usedcams->push_back_distinct(rc);
    }

    // for (int RC=0;RC<mp->ncams;RC++)
    for(int c = 0; c < usedcams->size(); c++)
    {
        int RC = (*usedcams)[c];

        const std::string fileNameStr = mv_getFileNamePrefix(mp->mip, RC + 1) + "." + mp->mip->imageExt;

        IplImage* bmp = cvLoadImage(fileNameStr.c_str());
        // cvSmooth(bmp,bmp,CV_GAUSSIAN,11);

        if(bmp != nullptr)
        {

            // check image size...
            if((mp->mip->getWidth(RC) == bmp->width) && (mp->mip->getHeight(RC) == bmp->height))
            {
                IplImage* bmpr = cvCreateImage(
                    cvSize(mp->mip->getWidth(RC) / scaleFactor, mp->mip->getHeight(RC) / scaleFactor), IPL_DEPTH_8U, 3);
                cvResize(bmp, bmpr);

                // staticVector<color> *camColScalesMap =
                // loadArrayFromFile<color>(wrlDir+"colorScalesMap"+num2strFourDecimal(RC)+".bin");
                // staticVector<color> *camColShiftsMap =
                // loadArrayFromFile<color>(wrlDir+"colorShiftsMap"+num2strFourDecimal(RC)+".bin");

                Color sc = (*camsColorScales)[RC];
                Color sh = (*camsColorShifts)[RC];
                int w = mp->mip->getWidth(RC) / scaleFactor;
                int h = mp->mip->getHeight(RC) / scaleFactor;

                for(int x = 0; x < w; x++)
                {
                    for(int y = 0; y < h; y++)
                    {
                        // color sc = (*camColScalesMap)[xx*hh+yy];
                        // color sh = (*camColShiftsMap)[xx*hh+yy];
                        CvScalar s = cvGet2D(bmpr, y, x);
                        s.val[0] = (unsigned char)(std::max(0.0f, std::min(254.0f, (float)s.val[0] * sc.r + sh.r)));
                        s.val[1] = (unsigned char)(std::max(0.0f, std::min(254.0f, (float)s.val[1] * sc.g + sh.g)));
                        s.val[2] = (unsigned char)(std::max(0.0f, std::min(254.0f, (float)s.val[2] * sc.b + sh.b)));
                        cvSet2D(bmpr, y, x, s);
                    }
                }

                // delete camColScalesMap;
                // delete camColShiftsMap;

                const std::string fileNameStrOut = imgsdir + "/" + num2strFourDecimal(mp->indexes[RC]) + "._c.png";
                if(cvSaveImage(fileNameStrOut.c_str(), bmpr) == 0)
                    printf("Could not save: %s\n", fileNameStrOut.c_str());
                cvReleaseImage(&bmpr);
            }
            else
            {
                printf("!width, height \n");
                exit(EXIT_FAILURE);
            }
            cvReleaseImage(&bmp);
        }
        else
        {
            printf("!LoadBitmapA(g_streamFactory, %s, bmp) \n", fileNameStr.c_str());
            exit(EXIT_FAILURE);
        }
    }

    long t1 = initEstimate();
    int ncamsatonce = 10;
    // for (int RC=0;RC<mp->ncams;RC=RC+ncamsatonce)
    for(int c = 0; c < usedcams->size(); c = c + ncamsatonce)
    {
        // int Ncamsatonce = std::min(RC+ncamsatonce,mp->ncams)-RC;
        int Ncamsatonce = std::min(c + ncamsatonce, usedcams->size()) - c;

        // load triangles and cameras indexes
        staticVector<staticVector<int>*>* usedpnts = new staticVector<staticVector<int>*>(Ncamsatonce);
        for(int i = 0; i < Ncamsatonce; i++)
        {
            (*usedpnts)[i] = new staticVector<int>(me->pts->size());
            (*usedpnts)[i]->resize_with(me->pts->size(), -1);
        }

        staticVector<staticVector<int>*>* tris = new staticVector<staticVector<int>*>(Ncamsatonce);
        for(int i = 0; i < Ncamsatonce; i++)
        {
            (*tris)[i] = new staticVector<int>(me->tris->size());
        }

        for(int i = 0; i < rcTris->size(); i++)
        {
            int rc = (*rcTris)[i];
            int rci = usedcams->indexOf(rc);
            if((rci >= c) && (rci < c + Ncamsatonce))
            {
                (*(*usedpnts)[rci - c])[(*me->tris)[i].i[0]] = 0;
                (*(*usedpnts)[rci - c])[(*me->tris)[i].i[1]] = 0;
                (*(*usedpnts)[rci - c])[(*me->tris)[i].i[2]] = 0;
                (*tris)[rci - c]->push_back(i);
            }
        }

        for(int cx = 0; cx < Ncamsatonce; cx++)
        {
            int j = 0;
            for(int i = 0; i < (*usedpnts)[cx]->size(); i++)
            {
                if((*(*usedpnts)[cx])[i] > -1)
                {
                    (*(*usedpnts)[cx])[i] = j;
                    j++;
                }
            }
        }

        /////////////////////////////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////////////////////////////


        // for (int rc=RC;rc<RC+Ncamsatonce;rc++)
        for(int rci = c; rci < c + Ncamsatonce; rci++)
        {
            int rc = (*usedcams)[rci];
            int w = mp->mip->getWidth(rc);
            int h = mp->mip->getHeight(rc);
            staticVector<int>* usedpntsrc = (*usedpnts)[rci - c];
            staticVector<int>* trisrc = (*tris)[rci - c];

            const std::string fileNameStrIn = "imgs/" + num2strFourDecimal(mp->indexes[rc]) + "._c.png";

            fprintf(f, "Group {\nchildren [\n");
            fprintf(f, "\tWorldInfo {\n");
            fprintf(f, "\tinfo \"test\"\n");
            fprintf(f, "\t}\n\n");

            fprintf(f, "Background {\nskyColor 1 1 1 \n}\n");
            fprintf(f, "\tTransform {\n");
            fprintf(f, "\t\ttranslation %g %g %g\n", 0.0, 0.0, 0.0);
            fprintf(f, "\t\tscale %g %g %g\n", 1.0, 1.0, 1.0);
            fprintf(f, "\n\t\tchildren [ Shape {\n");

            // points
            fprintf(f, "\t\t\tappearance Appearance {\n");

            fprintf(f, "\t\t\t\ttexture ImageTexture {\n");
            fprintf(f, "\t\t\t\t\turl \"%s\"\n", fileNameStrIn.c_str());
            // fprintf(f, "\t\t\t\t\trepeatS FALSE\n\t\t\t\t\trepeatT FALSE\n");
            fprintf(f, "\t\t\t\t}\n\n");

            fprintf(f, "\t\t\t\t\ttextureTransform TextureTransform {\n");
            fprintf(f, "\t\t\t\t\tscale %.9f %.9f\n", 1.0 / ((float)w / (float)scaleFactor),
                    1.0 / ((float)h / (float)scaleFactor));
            fprintf(f, "\t\t\t\t\ttranslation 0.5 0.5\n");
            fprintf(f, "\t\t\t\t}\n");

            fprintf(f, "\t\t\t}\n\n");

            fprintf(f, "\t\t\tgeometry IndexedFaceSet {\n");

            fprintf(f, "solid FALSE\n\n");

            fprintf(f, "ccw FALSE\n\n");

            fprintf(f, "creaseAngle 1.57\n\n");

            fprintf(f, "\t\t\t\tcoord Coordinate { point [\n");

            // write points

            // get points of triangles of rc
            for(int i = 0; i < me->pts->size(); i++)
            {
                if((*usedpntsrc)[i] > -1)
                {
                    fprintf(f, "\t\t\t\t\t%.9f %.9f %.9f \n", (*me->pts)[i].x, (*me->pts)[i].y, (*me->pts)[i].z);
                }
            }
            fprintf(f, "\t\t\t\t]}\n\n");

            fprintf(f, "\t\t\t\ttexCoord TextureCoordinate { point [\n");
            for(int i = 0; i < me->pts->size(); i++)
            {
                if((*usedpntsrc)[i] > -1)
                {
                    point3d pt = (*me->pts)[i];
                    pixel pix;
                    mp->getPixelFor3DPoint(&pix, pt, rc);
                    // fprintf(f, "\t\t\t\t\t%i %i \n", pix.x/scaleFactor, pix.y/scaleFactor);
                    fprintf(f, "\t\t\t\t\t%.9f %.9f \n", (float)pix.x / (float)scaleFactor,
                            (float)pix.y / (float)scaleFactor);
                }
            }
            fprintf(f, "\t\t\t\t]}\n\n");

            // fprintf(f, "\t\t\t\t ccw TRUE \n\n");
            fprintf(f, "\t\t\t\t colorPerVertex FALSE \n\n");

            fprintf(f, "\t\t\t\tcoordIndex [\n");

            for(int i = 0; i < trisrc->size(); i++)
            {
                fprintf(f, "\t\t\t\t\t%i, %i %i %i\n", (*usedpntsrc)[(*me->tris)[(*trisrc)[i]].i[0]],
                        (*usedpntsrc)[(*me->tris)[(*trisrc)[i]].i[1]], (*usedpntsrc)[(*me->tris)[(*trisrc)[i]].i[2]],
                        -1);
            }
            fprintf(f, "\t\t\t\t]\n\n");

            fprintf(f, "\t\t\t} # end of IndexedFaceSet\n\n");

            fprintf(f, "\t\t}]#end of children shape\n\t}#end of transform\n] # end of children\n} # end of group\n");
        }

        for(int i = 0; i < Ncamsatonce; i++)
        {
            delete(*tris)[i];
            delete(*usedpnts)[i];
        }

        delete tris;
        delete usedpnts;

        // printfEstimate(RC, mp->ncams, t1);
        printfEstimate(c, usedcams->size(), t1);
    }
    finishEstimate();

    /*
    if (camerasPerOneOmni<=1) {
            printfGroupCameras(f, mp, me->computeAverageEdgeLength()*5.0f);
    }else{
            printfGroupCamerasOmniSequnece(f,mp,me->computeAverageEdgeLength()*5.0f,camerasPerOneOmni);
    };
    */

    fclose(f);

    delete usedcams;
}

void mv_output3D::printfHexahedron(point3d* hexah, FILE* f, const multiviewParams*  /*mp*/)
{
    fprintf(f, "Group {\nchildren [\n");

    fprintf(f, "Background {\nskyColor 1 1 1 \n}\n");
    fprintf(f, "\tTransform {\n");
    fprintf(f, "\t\ttranslation %g %g %g\n", 0.0, 0.0, 0.0);
    fprintf(f, "\t\tscale %g %g %g\n", 1.0, 1.0, 1.0);
    fprintf(f, "\n\t\tchildren [ Shape {\n");

    // cameras
    fprintf(f, "\t\t\tappearance Appearance {material Material {emissiveColor 1 1 1 }}\n");
    fprintf(f, "\t\t\tgeometry IndexedLineSet {\n");
    fprintf(f, "\t\t\t\tcoord Coordinate { point [\n");

    fprintf(f, "\t\t\t\t\t%7.8f %7.8f %7.8f \n", hexah[0].x, hexah[0].y, hexah[0].z);
    fprintf(f, "\t\t\t\t\t%7.8f %7.8f %7.8f \n", hexah[1].x, hexah[1].y, hexah[1].z);
    fprintf(f, "\t\t\t\t\t%7.8f %7.8f %7.8f \n", hexah[2].x, hexah[2].y, hexah[2].z);
    fprintf(f, "\t\t\t\t\t%7.8f %7.8f %7.8f \n", hexah[3].x, hexah[3].y, hexah[3].z);
    fprintf(f, "\t\t\t\t\t%7.8f %7.8f %7.8f \n", hexah[4].x, hexah[4].y, hexah[4].z);
    fprintf(f, "\t\t\t\t\t%7.8f %7.8f %7.8f \n", hexah[5].x, hexah[5].y, hexah[5].z);
    fprintf(f, "\t\t\t\t\t%7.8f %7.8f %7.8f \n", hexah[6].x, hexah[6].y, hexah[6].z);
    fprintf(f, "\t\t\t\t\t%7.8f %7.8f %7.8f \n", hexah[7].x, hexah[7].y, hexah[7].z);

    fprintf(f, "\t\t\t\t]}\n\n");

    fprintf(f, "\t\t\t\tcoordIndex [\n");

    fprintf(f, "\t\t\t\t\t%i %i %i\n", 0, 1, -1);
    fprintf(f, "\t\t\t\t\t%i %i %i\n", 1, 2, -1);
    fprintf(f, "\t\t\t\t\t%i %i %i\n", 2, 3, -1);
    fprintf(f, "\t\t\t\t\t%i %i %i\n", 3, 0, -1);

    fprintf(f, "\t\t\t\t\t%i %i %i\n", 4, 5, -1);
    fprintf(f, "\t\t\t\t\t%i %i %i\n", 5, 6, -1);
    fprintf(f, "\t\t\t\t\t%i %i %i\n", 6, 7, -1);
    fprintf(f, "\t\t\t\t\t%i %i %i\n", 7, 4, -1);

    fprintf(f, "\t\t\t\t\t%i %i %i\n", 0, 4, -1);
    fprintf(f, "\t\t\t\t\t%i %i %i\n", 1, 5, -1);
    fprintf(f, "\t\t\t\t\t%i %i %i\n", 2, 6, -1);
    fprintf(f, "\t\t\t\t\t%i %i %i\n", 3, 7, -1);

    fprintf(f, "\t\t\t\t]\n\n");

    fprintf(f, "color Color { color [1 0 0, 0 1 0, 0 0 1] }\n\n");

    fprintf(f, "\t\t\t\tcolorIndex [\n");

    fprintf(f, "\t\t\t\t\t%i\n", 1);
    fprintf(f, "\t\t\t\t\t%i\n", 1);
    fprintf(f, "\t\t\t\t\t%i\n", 1);
    fprintf(f, "\t\t\t\t\t%i\n", 1);

    fprintf(f, "\t\t\t\t\t%i\n", 1);
    fprintf(f, "\t\t\t\t\t%i\n", 1);
    fprintf(f, "\t\t\t\t\t%i\n", 1);
    fprintf(f, "\t\t\t\t\t%i\n", 1);

    fprintf(f, "\t\t\t\t\t%i\n", 1);
    fprintf(f, "\t\t\t\t\t%i\n", 1);
    fprintf(f, "\t\t\t\t\t%i\n", 1);
    fprintf(f, "\t\t\t\t\t%i\n", 1);

    fprintf(f, "\t\t\t\t]\n\n");

    fprintf(f, "colorPerVertex FALSE\n\n");

    fprintf(f, "\t\t\t} # end of IndexedFaceSet\n\n");

    fprintf(f, "\t\t}]#end of children shape\n\t}#end of transform\n] # end of children\n} # end of group\n");
}

void mv_output3D::saveVoxelsToWrl(const std::string& wrlFileName, const multiviewParams* mp, staticVector<point3d>* voxels)
{
    FILE* f = fopen(wrlFileName.c_str(), "w");
    fprintf(f, "#VRML V2.0 utf8\n");
    // printf("creating wrl\n");

    for(int i = 0; i < voxels->size() / 8; i++)
    {
        printfHexahedron(&(*voxels)[i * 8], f, mp);
    }
    // printfGroupCameras(f, mp, 0.001);

    fclose(f);
}

void mv_output3D::printfGroupCamerasDepth(FILE* f, const multiviewParams* mp, staticVector<int>* cams, rgb camcolor)
{
    fprintf(f, "Group {\nchildren [\n");

    for(int c = 0; c < cams->size(); c++)
    {
        int rc = (*cams)[c];

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

        float fovy = (atan(mp->KArr[rc].m23 / mp->KArr[rc].m22) * 2.0f) * (M_PI / 180.0);

        fprintf(f, "Viewpoint {\n");
        fprintf(f, "\tfieldOfView %f \n", fovy
                /*angleBetwV1andV2(
                        point3d(mp->KArr[rc].m13,mp->KArr[rc].m11,0.0),
                        point3d(0.0,mp->KArr[rc].m11,0.0)
                ) * (M_PI/180.0) * 2.0*/
                );
        fprintf(f, "\tposition %f %f %f \n", viewp.x, viewp.y, viewp.z);
        fprintf(f, "\torientation %f %f %f %f \n", viewn.x, viewn.y, viewn.z, viewsim);
        fprintf(f, "\tdescription \"v%i\" \n", rc);
        fprintf(f, "\tjump TRUE \n");
        fprintf(f, "}\n");
    }

    fprintf(f, "Background {\nskyColor 1 1 1 \n}\n");
    fprintf(f, "\tTransform {\n");
    fprintf(f, "\t\ttranslation %g %g %g\n", 0.0, 0.0, 0.0);
    fprintf(f, "\t\tscale %g %g %g\n", 1.0, 1.0, 1.0);
    fprintf(f, "\n\t\tchildren [ Shape {\n");

    // cameras
    fprintf(f, "\t\t\tappearance Appearance {material Material {emissiveColor 1 1 1 }}\n");
    fprintf(f, "\t\t\tgeometry IndexedLineSet {\n");
    fprintf(f, "\t\t\t\tcoord Coordinate { point [\n");

    for(int c = 0; c < cams->size(); c++)
    {
        int rc = (*cams)[c];

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

        float mind, maxd;
        getDepthMapInfoDepthLimits(rc + 1, mp->mip, mind, maxd);
        // maxd = (1.0f/(n-vx-vy).size())*maxd;
        maxd = 10.0;

        fprintf(f, "\t\t\t\t\t%7.8f %7.8f %7.8f \n", p.x, p.y, p.z);
        fprintf(f, "\t\t\t\t\t%7.8f %7.8f %7.8f \n", p.x + (n.x - vx.x - vy.x) * maxd, p.y + (n.y - vx.y - vy.y) * maxd,
                p.z + (n.z - vx.z - vy.z) * maxd);
        fprintf(f, "\t\t\t\t\t%7.8f %7.8f %7.8f \n", p.x + (n.x - vx.x + vy.x) * maxd, p.y + (n.y - vx.y + vy.y) * maxd,
                p.z + (n.z - vx.z + vy.z) * maxd);
        fprintf(f, "\t\t\t\t\t%7.8f %7.8f %7.8f \n", p.x + (n.x + vx.x + vy.x) * maxd, p.y + (n.y + vx.y + vy.y) * maxd,
                p.z + (n.z + vx.z + vy.z) * maxd);
        fprintf(f, "\t\t\t\t\t%7.8f %7.8f %7.8f \n", p.x + (n.x + vx.x - vy.x) * maxd, p.y + (n.y + vx.y - vy.y) * maxd,
                p.z + (n.z + vx.z - vy.z) * maxd);
    }

    fprintf(f, "\t\t\t\t]}\n\n");

    fprintf(f, "\t\t\t\tcoordIndex [\n");

    for(int c = 0; c < cams->size(); c++)
    {
        int rc = (*cams)[c];

        fprintf(f, "\t\t\t\t\t%i %i %i\n", rc * 5, rc * 5 + 1, -1);
        fprintf(f, "\t\t\t\t\t%i %i %i\n", rc * 5, rc * 5 + 2, -1);
        fprintf(f, "\t\t\t\t\t%i %i %i\n", rc * 5, rc * 5 + 3, -1);
        fprintf(f, "\t\t\t\t\t%i %i %i\n", rc * 5, rc * 5 + 4, -1);

        fprintf(f, "\t\t\t\t\t%i %i %i\n", rc * 5 + 1, rc * 5 + 2, -1);
        fprintf(f, "\t\t\t\t\t%i %i %i\n", rc * 5 + 2, rc * 5 + 3, -1);
        fprintf(f, "\t\t\t\t\t%i %i %i\n", rc * 5 + 3, rc * 5 + 4, -1);
        fprintf(f, "\t\t\t\t\t%i %i %i\n", rc * 5 + 4, rc * 5 + 1, -1);
    }
    fprintf(f, "\t\t\t\t]\n\n");

    fprintf(f, "color Color { color [%f %f %f, %f %f %f, 0 0 1] }\n\n", (float)camcolor.r / 255.0f,
            (float)camcolor.g / 255.0f, (float)camcolor.b / 255.0f, (float)camcolor.r / 255.0f,
            (float)camcolor.g / 255.0f, (float)camcolor.b / 255.0f);

    fprintf(f, "\t\t\t\tcolorIndex [\n");
    for(int c = 0; c < cams->size(); c++)
    {

        fprintf(f, "\t\t\t\t\t%i\n", 1);
        fprintf(f, "\t\t\t\t\t%i\n", 1);
        fprintf(f, "\t\t\t\t\t%i\n", 1);
        fprintf(f, "\t\t\t\t\t%i\n", 1);

        fprintf(f, "\t\t\t\t\t%i\n", 1);
        fprintf(f, "\t\t\t\t\t%i\n", 1);
        fprintf(f, "\t\t\t\t\t%i\n", 1);
        fprintf(f, "\t\t\t\t\t%i\n", 1);
    }
    fprintf(f, "\t\t\t\t]\n\n");

    fprintf(f, "colorPerVertex FALSE\n\n");

    fprintf(f, "\t\t\t} # end of IndexedFaceSet\n\n");

    fprintf(f, "\t\t}]#end of children shape\n\t}#end of transform\n] # end of children\n} # end of group\n");
}

void mv_output3D::writeCamerasDepthToWrl(const std::string& wrlFileName, const multiviewParams* mp)
{
    FILE* f = fopen(wrlFileName.c_str(), "w");
    fprintf(f, "#VRML V2.0 utf8\n");

    staticVector<int>* cams = new staticVector<int>(mp->ncams);
    for(int rc = 0; rc < mp->ncams; rc++)
    {
        cams->push_back(rc);
    }

    rgb camcolor;
    camcolor.r = 0;
    camcolor.g = 255;
    camcolor.b = 0;
    printfGroupCamerasDepth(f, mp, cams, camcolor);

    fclose(f);

    delete cams;
}

void mv_output3D::writeCamerasToWrl(const std::string& wrlFileName, const multiviewParams* mp, int camerasPerOneOmni, float upStep)
{
    staticVector<int>* tcams = new staticVector<int>(mp->ncams);
    for(int rc = 0; rc < mp->ncams; rc++)
    {
        tcams->push_back(rc);
    }
    writeCamerasToWrl(tcams, wrlFileName, mp, camerasPerOneOmni, upStep);
    delete tcams;
}

void mv_output3D::writeCamerasToWrl(staticVector<int>* tcams, const std::string& wrlFileName, const multiviewParams* mp,
                                    int camerasPerOneOmni, float upStep)
{
    if(mp->verbose)
        printf("Creating : %s\n", wrlFileName.c_str());
    // printf("ACQUIRING AVERAGE PIXEL SIZE FROM SPARSE MATCHES\n");

    float avps = 0.0f;
    float navps = 0.0f;

    long t1 = initEstimate();
    for(int c = 0; c < tcams->size(); c++)
    {
        int rc = (*tcams)[c];
        staticVector<seedPoint>* seeds;
        loadSeedsFromFile(&seeds, mp->indexes[rc], mp->mip, mp->mip->MV_FILE_TYPE_seeds);

        for(int i = 0; i < seeds->size(); i++)
        {
            avps += mp->getCamPixelSize((*seeds)[i].op.p, rc);
            navps += 1.0f;
        }

        delete seeds;
        printfEstimate(c, tcams->size(), t1);
    } // for rc
    finishEstimate();

    float camSize = (avps / navps) * 10.0f;

    if(navps == 0.0f)
    {
        point3d cg = point3d(0.0f, 0.0f, 0.0f);
        for(int c = 0; c < tcams->size(); c++)
        {
            int rc = (*tcams)[c];
            cg = cg + mp->CArr[rc];
        }
        cg = cg / (float)tcams->size();
        avps = 0.0f;
        for(int c = 0; c < tcams->size(); c++)
        {
            int rc = (*tcams)[c];
            avps += (cg - mp->CArr[rc]).size();
        }
        navps = (float)tcams->size();
        camSize = (avps / navps) / 100.0f;
    }

    FILE* f = fopen(wrlFileName.c_str(), "w");
    fprintf(f, "#VRML V2.0 utf8\n");

    if(camerasPerOneOmni <= 1)
    {
        mv_mesh* meCams = createMeshForCameras(tcams, mp, camSize, 0, 1, 0.0f);
        printfGroupCameras(meCams, tcams, f, mp, camSize, camSize * upStep);
        delete meCams;
    }
    else
    {
        printfGroupCamerasOmniSequnece(tcams, f, mp, camSize, camerasPerOneOmni, camSize * upStep);
    }

    fclose(f);
}

void mv_output3D::inline2Wrls(const std::string& wrlFileNameOut, const std::string& wrlFileName1, const std::string& wrlFileName2)
{
    FILE* f = fopen(wrlFileNameOut.c_str(), "w");
    fprintf(f, "#VRML V2.0 utf8\n");
    fprintf(f, "Background {\n skyColor 1 1 1 \n } \n");
    fprintf(f, "Inline{ url [\"%s\"] \n }\n", wrlFileName1.c_str());
    fprintf(f, "Inline{ url [\"%s\"] \n }\n", wrlFileName2.c_str());
    fclose(f);
}

void powerTwoUp(int& max, int& times, int a, int init)
{
    max = init;
    times = 1;
    while(max < a)
    {
        max *= 2;
        times *= 2;
    }
}

void getSubSetOfPtsForSubsetOfTris(staticVector<int>** isubids, staticVector<int>** ihash, mv_mesh* me, int striid,
                                   int etriid)
{
    staticVector<int>* subids = new staticVector<int>(me->pts->size());
    staticVector<int>* hash = new staticVector<int>(me->pts->size());
    hash->resize_with(me->pts->size(), -1);

    for(int i = striid; i < etriid; i++)
    {
        for(int k = 0; k < 3; k++)
        {
            (*hash)[(*me->tris)[i].i[k]] = 0;
        }
    }

    for(int i = 0; i < me->pts->size(); i++)
    {
        if((*hash)[i] > -1)
        {
            subids->push_back(i);
            (*hash)[i] = subids->size() - 1;
        }
    }

    *isubids = subids;
    *ihash = hash;
}

void getSubSetOfPtsForSubsetOfTris(staticVector<int>** isubids, staticVector<int>** ihash, mv_mesh* me,
                                   staticVector<int>* trisIds)
{
    staticVector<int>* subids = new staticVector<int>(me->pts->size());
    staticVector<int>* hash = new staticVector<int>(me->pts->size());
    hash->resize_with(me->pts->size(), -1);

    for(int i = 0; i < trisIds->size(); i++)
    {
        for(int k = 0; k < 3; k++)
        {
            (*hash)[(*me->tris)[(*trisIds)[i]].i[k]] = 0;
        }
    }

    for(int i = 0; i < me->pts->size(); i++)
    {
        if((*hash)[i] > -1)
        {
            subids->push_back(i);
            (*hash)[i] = subids->size() - 1;
        }
    }

    *isubids = subids;
    *ihash = hash;
}

void getPixelForIdTriangle(int& xpg, int& ypg, int& xp, int& yp,
                           // int blockSide, int id2Tri,
                           int blockSide, int idTri, int ti, int tj, int sw, int sh)
{
    int w = blockSide / sw;
    int h = blockSide / sh;
    // int idTri = id2Tri/2;
    int nTrisInBlock = w * h;
    int blockId = idTri / nTrisInBlock;
    int idTriFirstInBlock = blockId * nTrisInBlock;
    int idTriInBlock = idTri - idTriFirstInBlock;

    xp = (idTriInBlock % w) * sw + ti;
    yp = (idTriInBlock / w) * sh + tj;
    // if (id2Tri%2==1) {
    //	xp = (idTriInBlock%w)*sw+(sw-1)-ti;
    //	yp = (idTriInBlock/w)*sh+(sh-1)-tj;
    //};

    xpg = xp;
    ypg = blockId * blockSide + yp;
}

void compParams(int& sh, int& w, int& h, int& nTrisInBlock, int& nblocks, int ntris, int sw, int blockSide)
{
    sh = sw;
    // sh=sw-1;
    // printf("sw %i, sh %i\n",sw,sh);
    w = blockSide / sw;
    h = blockSide / sh;
    nTrisInBlock = w * h;
    // printf("nTrisInBlock %i, blockSide %i, w %i, h %i, ntris %i\n",nTrisInBlock,blockSide,w,h,ntris);
    // nblocks = (ntris/2)/nTrisInBlock+1;
    nblocks = ntris / nTrisInBlock + 1;
    // printf("nblocks %i\n",nblocks);
}

float gigaBytes(int blockpixs, int nblocks, int sizeofTypeInBytes)
{
    float blockgb = (((float)(sizeofTypeInBytes * blockpixs) / 1024.0) / 1024.0) / 1024.0f;

    float gb = blockgb * (float)nblocks;
    // printf("memory needed %f for %i * %i of %i bytes\n",gb,blockpixs,nblocks,sizeofTypeInBytes);
    return gb;
}

int mv_output3D::saveMeshToWrlTrisAverageTextured(const std::string& dirName, const std::string& wrlName, const multiviewParams* mp,
                                                  mv_mesh* me, staticVector<staticVector<int>*>* trisCams)
{
    if(mp->verbose)
        printf("computing colors \n");

    ///////////////////////////////////////////////////////////////////////////////////////
    // compute average edge projection lenght in pixels
    float avEdgeProjLength = 0.0f;
    float navEdgeProjLength = 0.0f;
    for(int i = 0; i < trisCams->size(); i++)
    {
        staticVector<int>* cams = (*trisCams)[i];
        for(int j = 0; j < sizeOfStaticVector<int>(cams); j++)
        {
            int cam = (*cams)[j];
            point2d pixa, pixb, pixc;
            mp->getPixelFor3DPoint(&pixa, (*me->pts)[(*me->tris)[i].i[0]], cam);
            mp->getPixelFor3DPoint(&pixb, (*me->pts)[(*me->tris)[i].i[1]], cam);
            mp->getPixelFor3DPoint(&pixc, (*me->pts)[(*me->tris)[i].i[2]], cam);
            avEdgeProjLength += (pixa - pixb).size();
            navEdgeProjLength += 1.0f;
            avEdgeProjLength += (pixa - pixc).size();
            navEdgeProjLength += 1.0f;
            avEdgeProjLength += (pixb - pixc).size();
            navEdgeProjLength += 1.0f;
        }
    }

    if(navEdgeProjLength == 0)
    {
        if(mp->verbose)
            printf("WARNING navEdgeProjLength==0\n");
        return 1;
    }

    avEdgeProjLength /= navEdgeProjLength;

    avEdgeProjLength = std::min(50.0f, avEdgeProjLength);

    ///////////////////////////////////////////////////////////////////////////////////////
    // compute texture map resolution
    int times, maxsw;
    powerTwoUp(maxsw, times, (int)ceil(avEdgeProjLength), 4);
    int nsq = sqrt((float)me->tris->size()) + 1;
    int blockSide = 2048;

    if(mp->verbose)
        printf("avEdgeProjLength %f, maxsw %i\n", avEdgeProjLength, maxsw);

    int sh, nTrisInBlock, nblocks, w, h;
    int sw = 4;
    compParams(sh, w, h, nTrisInBlock, nblocks, me->tris->size(), sw, blockSide);
    if(mp->verbose)
        printf("init tris %i, nsq %i, sw %i, sh %i, nblocks %i, blockSide %i, nTrisInBlock %i, w %i, h %i\n",
               me->tris->size(), nsq, sw, sh, nblocks, blockSide, nTrisInBlock, w, h);

    while((gigaBytes(blockSide * blockSide, nblocks, sizeof(point3d) + sizeof(float)) < 2.0f) && (sw <= maxsw))
    {
        sw = sw * 2;
        if(blockSide < sw)
        {
            if(mp->verbose)
                printf("WARNING blockSide<sw\n");
            return 1;
        }
        // if (mp->verbose) printf("before tris %i, nsq %i, sw %i, sh %i, nblocks %i, blockSide %i, nTrisInBlock %i, w
        // %i, h %i\n",me->tris->size(),nsq,sw,sh,nblocks,blockSide,nTrisInBlock,w,h);
        compParams(sh, w, h, nTrisInBlock, nblocks, me->tris->size(), sw, blockSide);
        // if (mp->verbose) printf("after tris %i, nsq %i, sw %i, sh %i, nblocks %i, blockSide %i, nTrisInBlock %i, w
        // %i, h %i\n",me->tris->size(),nsq,sw,sh,nblocks,blockSide,nTrisInBlock,w,h);
    }

    sw /= 2;
    compParams(sh, w, h, nTrisInBlock, nblocks, me->tris->size(), sw, blockSide);

    if(mp->verbose)
        printf("tris %i, nsq %i, sw %i, sh %i, nblocks %i, blockSide %i, nTrisInBlock %i, w %i, h %i\n",
               me->tris->size(), nsq, sw, sh, nblocks, blockSide, nTrisInBlock, w, h);

    float gb = gigaBytes(blockSide * blockSide, nblocks, sizeof(point3d) + sizeof(float));

    if(mp->verbose)
        printf("memory needed %f\n", gb);
    if(gb >= 2.0f)
    {
        printf("WARNING out of memory, would need to allocate %f giga bytes, skipping texturing\n", gb);
        return 0;
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    // fill texture map
    staticVector<Color>* trisTextAvCols = new staticVector<Color>(blockSide * blockSide * nblocks);
    trisTextAvCols->resize_with(blockSide * blockSide * nblocks, Color(0.0f, 0.0f, 0.0f));
    staticVector<float>* ntrisTextAvCols = new staticVector<float>(blockSide * blockSide * nblocks);
    ntrisTextAvCols->resize_with(blockSide * blockSide * nblocks, 0.0f);

    staticVector<staticVector<int>*>* camsTris = convertObjectsCamsToCamsObjects(mp, trisCams);
    mv_images_cache* ic = new mv_images_cache(mp, mp->mip->_ini.get<int>("global.npreload", 100), 0);
    long t1 = initEstimate();
    for(int rc = 0; rc < mp->ncams; rc++)
    {
        for(int j = 0; j < sizeOfStaticVector<int>((*camsTris)[rc]); j++)
        {
            int idTri = (*(*camsTris)[rc])[j];
            point3d pa = (*me->pts)[(*me->tris)[idTri].i[0]];
            point3d pb = (*me->pts)[(*me->tris)[idTri].i[1]];
            point3d pc = (*me->pts)[(*me->tris)[idTri].i[2]];
            point2d pixa;
            mp->getPixelFor3DPoint(&pixa, pa, rc);
            point2d pixb;
            mp->getPixelFor3DPoint(&pixb, pb, rc);
            point2d pixc;
            mp->getPixelFor3DPoint(&pixc, pc, rc);
            if((mp->isPixelInImage(pixa, rc)) && (mp->isPixelInImage(pixb, rc)) && (mp->isPixelInImage(pixc, rc)))
            {
                for(int ti = 0; ti < sh; ti++)
                {
                    // for (int tj = 0; tj < sh-ti; tj++) {
                    for(int tj = 0; tj < sh; tj++)
                    {
                        point3d p = pa + ((pb - pa) / (float)(sh - 3)) * (float)(ti - 1) +
                                    ((pc - pa) / (float)(sh - 3)) * (float)(tj - 1);
                        point2d pix;
                        mp->getPixelFor3DPoint(&pix, p, rc);
                        if(mp->isPixelInImage(pix, rc))
                        {
                            Color col = ic->getPixelValueInterpolated(&pix, rc);
                            int xpg, ypg, xp, yp;
                            getPixelForIdTriangle(xpg, ypg, xp, yp, blockSide, idTri, ti, tj, sw, sh);
                            (*trisTextAvCols)[ypg * blockSide + xpg] = (*trisTextAvCols)[ypg * blockSide + xpg] + col;
                            (*ntrisTextAvCols)[ypg * blockSide + xpg] += 1.0f;
                        }
                    }
                }
            }
        }
        printfEstimate(rc, mp->ncams, t1);
    }
    finishEstimate();
    delete ic;

    for(int i = 0; i < trisTextAvCols->size(); i++)
    {
        if((*ntrisTextAvCols)[i] > 0.0f)
        {
            (*trisTextAvCols)[i] = (*trisTextAvCols)[i] / (*ntrisTextAvCols)[i];
        }
    }
    delete ntrisTextAvCols;
    deleteArrayOfArrays<int>(&camsTris);

    ///////////////////////////////////////////////////////////////////////////////////////
    // create wrl
    const std::string fname = dirName + wrlName;
    FILE* f = fopen(fname.c_str(), "w");

    fprintf(f, "#VRML V2.0 utf8\n");

    int startIdTri = 0;

    for(int idBlock = 0; idBlock < nblocks; idBlock++)
    {
        int xpg, ypg, xp, yp;
        getPixelForIdTriangle(xpg, ypg, xp, yp, blockSide, startIdTri, 0, 0, sw, sh);
        if(mp->verbose)
            printf("xpg %i, ypg %i, xp %i, yp %i\n", xpg, ypg, xp, yp);

        if(startIdTri < me->tris->size())
        {
            ///////////////////////////////////////////////////////////////////////////////////////
            // create and fill texture map image
            IplImage* bmp = cvCreateImage(cvSize(blockSide, blockSide), IPL_DEPTH_8U, 3);
            for(yp = 0; yp < blockSide; yp++)
            {
                for(xp = 0; xp < blockSide; xp++)
                {
                    CvScalar c;
                    c.val[0] =
                        (unsigned char)std::min(254.0f, (*trisTextAvCols)[(ypg + yp) * blockSide + (xpg + xp)].b); // b
                    c.val[1] =
                        (unsigned char)std::min(254.0f, (*trisTextAvCols)[(ypg + yp) * blockSide + (xpg + xp)].g); // g
                    c.val[2] =
                        (unsigned char)std::min(254.0f, (*trisTextAvCols)[(ypg + yp) * blockSide + (xpg + xp)].r); // r
                    cvSet2D(bmp, yp, xp, c);
                }
            }
            // int endIdTri = std::min(me->tris->size(),startIdTri+nTrisInBlock*2);
            int endIdTri = std::min(me->tris->size(), startIdTri + nTrisInBlock);
            if(mp->verbose)
                printf("startIdTri %i, endIdTri %i, idBlock %i\n", startIdTri, endIdTri, idBlock);

            const std::string textName = wrlName + "_" + num2str(idBlock) + ".png";
            const std::string textFileName = dirName + textName;

            if(cvSaveImage(textFileName.c_str(), bmp) == 0)
                printf("Could not save: %s\n", textFileName.c_str());
            cvReleaseImage(&bmp);

            fprintf(f, "Group {\nchildren [\n");
            fprintf(f, "\tWorldInfo {\n");
            fprintf(f, "\tinfo \"test\"\n");
            fprintf(f, "\t}\n\n");

            fprintf(f, "Background {\nskyColor 1 1 1 \n}\n");
            fprintf(f, "\tTransform {\n");
            fprintf(f, "\t\ttranslation %g %g %g\n", 0.0, 0.0, 0.0);
            fprintf(f, "\t\tscale %g %g %g\n", 1.0, 1.0, 1.0);
            fprintf(f, "\n\t\tchildren [ Shape {\n");

            fprintf(f, "\t\t\tappearance Appearance {\n");

            fprintf(f, "\t\t\t\ttexture ImageTexture {\n");
            fprintf(f, "\t\t\t\t\turl \"%s\"\n", textName.c_str());
            fprintf(f, "\t\t\t\t}\n\n");

            fprintf(f, "\t\t\t\t\ttextureTransform TextureTransform {\n");
            fprintf(f, "\t\t\t\t\tscale %.9f %.9f\n", 1.0 / (float)blockSide, 1.0 / (float)blockSide);
            fprintf(f, "\t\t\t\t\ttranslation 0.5 0.5\n");
            fprintf(f, "\t\t\t\t}\n");
            fprintf(f, "\t\t\t}\n\n");

            fprintf(f, "\t\t\tgeometry IndexedFaceSet {\n");
            // fprintf(f, "solid FALSE\n\n");
            // fprintf(f, "ccw FALSE\n\n");
            // fprintf(f, "creaseAngle 1.57\n\n");

            fprintf(f, "\t\t\t\tcoord Coordinate { point [\n");
            staticVector<int>* subids = nullptr;
            staticVector<int>* hash = nullptr;
            getSubSetOfPtsForSubsetOfTris(&subids, &hash, me, startIdTri, endIdTri);
            for(int i = 0; i < subids->size(); i++)
            {
                fprintf(f, "\t\t\t\t\t%.9f %.9f %.9f \n", (*me->pts)[(*subids)[i]].x, (*me->pts)[(*subids)[i]].y,
                        (*me->pts)[(*subids)[i]].z);
            }
            fprintf(f, "\t\t\t\t]}\n\n");

            fprintf(f, "\t\t\t\ttexCoord TextureCoordinate { point [\n");
            for(int i = startIdTri; i < endIdTri; i++)
            {
                getPixelForIdTriangle(xpg, ypg, xp, yp, blockSide, i, 1, 1, sw, sh);
                fprintf(f, "\t\t\t\t\t%f %f \n", (float)xp, (float)(blockSide - 1 - yp));
                getPixelForIdTriangle(xpg, ypg, xp, yp, blockSide, i, sh - 1, 1, sw, sh);
                fprintf(f, "\t\t\t\t\t%f %f \n", (float)xp, (float)(blockSide - 1 - yp));
                getPixelForIdTriangle(xpg, ypg, xp, yp, blockSide, i, 1, sh - 1, sw, sh);
                fprintf(f, "\t\t\t\t\t%f %f \n", (float)xp, (float)(blockSide - 1 - yp));
            }
            fprintf(f, "\t\t\t\t]}\n\n");

            fprintf(f, "\t\t\t\tcoordIndex [\n");
            for(int i = startIdTri; i < endIdTri; i++)
            {
                fprintf(f, "\t\t\t\t\t%i %i %i %i\n", (*hash)[(*me->tris)[i].i[0]], (*hash)[(*me->tris)[i].i[1]],
                        (*hash)[(*me->tris)[i].i[2]], -1);
            }
            fprintf(f, "\t\t\t\t]\n\n");

            delete subids;
            delete hash;

            fprintf(f, "\t\t\t\ttexCoordIndex [\n");
            for(int i = startIdTri; i < endIdTri; i++)
            {
                fprintf(f, "\t\t\t\t\t%i %i %i %i\n", (i - startIdTri) * 3, (i - startIdTri) * 3 + 1,
                        (i - startIdTri) * 3 + 2, -1);
            }
            fprintf(f, "\t\t\t\t]\n\n");

            // fprintf(f, "\t\t\t\t ccw TRUE \n\n");
            // fprintf(f, "\t\t\t\t colorPerVertex FALSE \n\n");

            fprintf(f, "\t\t\t} # end of IndexedFaceSet\n\n");

            fprintf(f, "\t\t}]#end of children shape\n\t}#end of transform\n] # end of children\n} # end of group\n");

            // IMPORTANT
            // startIdTri+=nTrisInBlock*2;
            startIdTri += nTrisInBlock;
        }
    }

    delete trisTextAvCols;
    fclose(f);

    return 1;
}

void mv_output3D::saveMvMeshToObj(mv_mesh* me, const std::string& objFileName)
{
    std::cout << "saveMvMeshToObj: " << objFileName << std::endl;
    FILE* f = fopen(objFileName.c_str(), "w");

    fprintf(f, "# \n");
    fprintf(f, "# Wavefront OBJ file\n");
    fprintf(f, "# Created with AliceVision\n");
    fprintf(f, "# \n");
    fprintf(f, "g Mesh\n");
    for(int i = 0; i < me->pts->size(); i++)
    {
        fprintf(f, "v %f %f %f\n", (*me->pts)[i].x, (*me->pts)[i].y, (*me->pts)[i].z);
    }

    for(int i = 0; i < me->tris->size(); i++)
    {
        fprintf(f, "f %i %i %i\n", (*me->tris)[i].i[0] + 1, (*me->tris)[i].i[1] + 1, (*me->tris)[i].i[2] + 1);
    }
    fclose(f);

    std::cout << "saveMvMeshToObj done." << std::endl;
}

void mv_output3D::savePtsAsSpheresToWrl(const std::string& wrlName, staticVector<point3d>* pts, float r, float g, float b,
                                        float radius)
{

    FILE* f = fopen(wrlName.c_str(), "w");
    fprintf(f, "#VRML V2.0 utf8\n");
    fprintf(f, "Group {\nchildren [\n");
    fprintf(f, "\tWorldInfo {\n");
    fprintf(f, "\tinfo \"test\"\n");
    fprintf(f, "\t}\n\n");
    fprintf(f, "Background {\nskyColor 1 1 1 \n}\n");

    for(int i = 0; i < pts->size(); i++)
    {
        point3d p = (*pts)[i];

        fprintf(f, "\tTransform {\n");
        fprintf(f, "\t\ttranslation %f %f %f\n", p.x, p.y, p.z);
        fprintf(f, "\t\tscale %f %f %f\n", 1.0, 1.0, 1.0);
        fprintf(f, "\n\t\tchildren  Shape {\n");
        fprintf(f, "\n\t\t geometry Sphere { radius %f }\n", radius);
        fprintf(f, "\n\t\t appearance Appearance { material Material { emissiveColor %f %f %f } } \n", r, g, b);
        fprintf(f, "\n\t\t}\n");
        fprintf(f, "\t}\n");
    }
    fprintf(f, "\t\t]# end of children\n} # end of group\n");

    fclose(f);
}

void mv_output3D::groupTexturesToOneFileWrl(staticVector<point2d>* meshPtsUV, mv_mesh* me, int newTextureSide,
                                            const std::string& newTextName, const std::string& wrlMeshFileName)
{
    FILE* f = fopen(wrlMeshFileName.c_str(), "w");
    fprintf(f, "#VRML V2.0 utf8\n");

    fprintf(f, "Group {\nchildren [\n");
    fprintf(f, "\tWorldInfo {\n");
    fprintf(f, "\tinfo \"test\"\n");
    fprintf(f, "\t}\n\n");

    fprintf(f, "Background {\nskyColor 1 1 1 \n}\n");
    fprintf(f, "\tTransform {\n");
    fprintf(f, "\t\ttranslation %g %g %g\n", 0.0, 0.0, 0.0);
    fprintf(f, "\t\tscale %g %g %g\n", 1.0, 1.0, 1.0);
    fprintf(f, "\n\t\tchildren [ Shape {\n");

    fprintf(f, "\t\t\tappearance Appearance {\n");

    fprintf(f, "\t\t\t\ttexture ImageTexture {\n");
    fprintf(f, "\t\t\t\t\turl \"%s\"\n", newTextName.c_str());
    fprintf(f, "\t\t\t\t}\n\n");

    fprintf(f, "\t\t\t\t\ttextureTransform TextureTransform {\n");
    fprintf(f, "\t\t\t\t\tscale %.9f %.9f\n", 1.0 / (float)(newTextureSide), 1.0 / (float)(newTextureSide));
    fprintf(f, "\t\t\t\t\ttranslation 0.5 0.5\n");
    fprintf(f, "\t\t\t\t}\n");
    fprintf(f, "\t\t\t}\n\n");

    fprintf(f, "\t\t\tgeometry IndexedFaceSet {\n");
    // fprintf(f, "solid FALSE\n\n");
    // fprintf(f, "ccw FALSE\n\n");
    // fprintf(f, "creaseAngle 1.57\n\n");

    fprintf(f, "\t\t\t\tcoord Coordinate { point [\n");
    for(int i = 0; i < me->pts->size(); i++)
    {
        fprintf(f, "\t\t\t\t\t%.9f %.9f %.9f \n", (*me->pts)[i].x, (*me->pts)[i].y, (*me->pts)[i].z);
    }
    fprintf(f, "\t\t\t\t]}\n\n");

    fprintf(f, "\t\t\t\ttexCoord TextureCoordinate { point [\n");
    for(int i = 0; i < me->pts->size(); i++)
    {
        fprintf(f, "\t\t\t\t\t%f %f \n", (*meshPtsUV)[i].x, (float)(newTextureSide - 1) - (*meshPtsUV)[i].y);
    }
    fprintf(f, "\t\t\t\t]}\n\n");

    fprintf(f, "\t\t\t\tcoordIndex [\n");
    for(int i = 0; i < me->tris->size(); i++)
    {
        fprintf(f, "\t\t\t\t\t%i %i %i %i\n", (*me->tris)[i].i[0], (*me->tris)[i].i[1], (*me->tris)[i].i[2], -1);
    }
    fprintf(f, "\t\t\t\t]\n\n");

    fprintf(f, "\t\t\t\ttexCoordIndex [\n");
    for(int i = 0; i < me->tris->size(); i++)
    {
        fprintf(f, "\t\t\t\t\t%i %i %i %i\n", (*me->tris)[i].i[0], (*me->tris)[i].i[1], (*me->tris)[i].i[2], -1);
    }
    fprintf(f, "\t\t\t\t]\n\n");

    // fprintf(f, "\t\t\t\t ccw TRUE \n\n");
    // fprintf(f, "\t\t\t\t colorPerVertex FALSE \n\n");

    fprintf(f, "\t\t\t} # end of IndexedFaceSet\n\n");

    fprintf(f, "\t\t}]#end of children shape\n\t}#end of transform\n] # end of children\n} # end of group\n");

    fclose(f);
}

void mv_output3D::groupTexturesToOneFile(mv_mesh* me, int newTextureSide, const std::string& meTextureCorrdsFileName,
                                         const std::string& wrlMeshFileName, const std::string& outDir)
{
    int textureSide, natlases;
    staticVector<point2d>* meshPtsUV;
    staticVector<int>* meshPtsTexIds;
    me->remeshByTextureAndGetTextureInfo(textureSide, natlases, mp->verbose, meTextureCorrdsFileName, &meshPtsUV,
                                         &meshPtsTexIds);

    int gridSideSize = 1;
    while(gridSideSize * gridSideSize < natlases)
    {
        gridSideSize++;
    }

    float scale = (float)(gridSideSize * textureSide) / (float)newTextureSide;
    int partScaledSide = (int)((float)textureSide / scale);
    newTextureSide = gridSideSize * partScaledSide;

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // join texture files
    IplImage* bmpo =
        cvCreateImage(cvSize(gridSideSize * partScaledSide, gridSideSize * partScaledSide), IPL_DEPTH_8U, 3);
    for(int gy = 0; gy < gridSideSize; gy++)
    {
        for(int gx = 0; gx < gridSideSize; gx++)
        {
            int atlasId = gy * gridSideSize + gx;
            if(atlasId < natlases)
            {
                const std::string textureFileName = wrlMeshFileName + "_" + num2str(atlasId) + ".png";
                IplImage* bmp = cvLoadImage(textureFileName.c_str());

                IplImage* bmpr = cvCreateImage(cvSize(partScaledSide, partScaledSide), IPL_DEPTH_8U, 3);
                cvResize(bmp, bmpr);

                for(int y = 0; y < partScaledSide; y++)
                {
                    for(int x = 0; x < partScaledSide; x++)
                    {
                        CvScalar c;
                        c = cvGet2D(bmpr, y, x);
                        int xn = gx * partScaledSide + x;
                        int yn = gy * partScaledSide + y;
                        cvSet2D(bmpo, yn, xn, c);
                    }
                }
                cvReleaseImage(&bmp);
                cvReleaseImage(&bmpr);
            }
        }
    }

    const std::string newTextNameOrig = outDir + "meshAvImgTexRes.png";
    if(cvSaveImage(newTextNameOrig.c_str(), bmpo) == 0)
        printf("Could not save: %s\n", newTextNameOrig.c_str());

    cvReleaseImage(&bmpo);

    for(int i = 0; i < meshPtsUV->size(); i++)
    {
        int atlasId = (*meshPtsTexIds)[i];
        // int atlasId = gy*gridSideSize+gx;
        int gx = atlasId % gridSideSize;
        int gy = atlasId / gridSideSize;

        point2d pix = (*meshPtsUV)[i];
        pix.x = (float)(gx * partScaledSide) + pix.x / scale;
        pix.y = (float)(gy * partScaledSide) + pix.y / scale;
        (*meshPtsUV)[i] = pix;
    }

    groupTexturesToOneFileWrl(meshPtsUV, me, gridSideSize * partScaledSide, "meshAvImgTexRes.png",
                              outDir + "meshAvImgTexRes.wrl");

    delete meshPtsUV;
    delete meshPtsTexIds;
}
