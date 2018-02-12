// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "mv_delaunay_GC_grid.hpp"

#include <aliceVision/CUDAInterfaces/refine.hpp>
#include <aliceVision/structures/mv_bin_search.hpp>


void mv_delanuay_GC_grid::addCams(GC_Vertex_handle& vh, staticVector<int>* ptcams)
{
    if(vh->info().cams->size() + ptcams->size() + 1 >= vh->info().cams->reserved())
    {
        vh->info().resizeAdd(max(10, ptcams->size() + 1));
    }

    int id;
    point2d pix;

    for(int i = 0; i < ptcams->size(); i++)
    {
        id = vh->info().cams->push_back_distinct((*ptcams)[i]);
    }

    vh->info().nrc++;
}

void mv_delanuay_GC_grid::createTriangulationFromDepthMapsCamsVoxelGrid(staticVector<int>* cams,
                                                                        std::string fileNameWrl, int scale,
                                                                        staticVector<int>* voxelsIds, point3d voxel[8],
                                                                        int numSubVoxs, bool doMoveIntoGridPosition)
{
    ///////////////////////////////////////////////////////////////////////////////////////
    printf("creating 3D delanuay triangulation\n");

    int w = mp->mip->imp.width;
    int h = mp->mip->imp.height;
    int ng = 0;

    vx = voxel[1] - voxel[0];
    vy = voxel[3] - voxel[0];
    vz = voxel[4] - voxel[0];
    sx = vx.size() / (float)numSubVoxs;
    vx = vx.normalize();
    sy = vy.size() / (float)numSubVoxs;
    vy = vy.normalize();
    sz = vz.size() / (float)numSubVoxs;
    vz = vz.normalize();

    float mind = min(sx, min(sy, sz)) / 10.0f;
    if(doMoveIntoGridPosition == false)
    {
        mind = (sx + sy + sz) / 3.0f;
    }

    int scalePS = mp->mip->_ini.get<int>("global.scalePS", 1);
    float pointToJoinPixSizeDist =
        (float)mp->mip->_ini.get<double>("delanuaycut.pointToJoinPixSizeDist", 2.0) * (float)scalePS;
    printf("grouping threshold is %f pixelSize\n", pointToJoinPixSizeDist);

    ///////////////////////////////////////////////////////////////////////////////////////
    // build tetrahedragonalization

    int nVertices = 0;

    // add points for cam centers
    for(int camid = 0; camid < cams->size(); camid++)
    {
        int rc = (*cams)[camid];
        {
            GC_Point p = GC_Point(mp->CArr[rc].x, mp->CArr[rc].y, mp->CArr[rc].z);

            GC_Vertex_handle v = T.nearest_vertex(p);
            GC_Point np;
            point3d npp;

            if(v != NULL)
            {
                np = v->point();
                npp = convertPointToPoint3d(np);
            }

            if((v == NULL) || ((npp - mp->CArr[rc]).size() > mind))
            {
                GC_Vertex_handle newv = T.insert(p);
                newv->info().id = nVertices;
                newv->info().nrc = 0;
                newv->info().segSize = 0;
                newv->info().segId = -1;
                newv->info().cams = NULL;
                nVertices++;
            }
        }
    }

    // add 6 points to prevent singularities
    {
        point3d vcg = (voxel[0] + voxel[1] + voxel[2] + voxel[3] + voxel[4] + voxel[5] + voxel[6] + voxel[7]) / 8.0f;
        point3d extrPts[6];
        point3d fcg;
        fcg = (voxel[0] + voxel[1] + voxel[2] + voxel[3]) / 4.0f;
        extrPts[0] = fcg + (fcg - vcg) / 2.0f;
        fcg = (voxel[0] + voxel[4] + voxel[7] + voxel[3]) / 4.0f;
        extrPts[1] = fcg + (fcg - vcg) / 2.0f;
        fcg = (voxel[0] + voxel[1] + voxel[5] + voxel[4]) / 4.0f;
        extrPts[2] = fcg + (fcg - vcg) / 2.0f;
        fcg = (voxel[4] + voxel[5] + voxel[6] + voxel[7]) / 4.0f;
        extrPts[3] = fcg + (fcg - vcg) / 2.0f;
        fcg = (voxel[1] + voxel[5] + voxel[6] + voxel[2]) / 4.0f;
        extrPts[4] = fcg + (fcg - vcg) / 2.0f;
        fcg = (voxel[3] + voxel[2] + voxel[6] + voxel[7]) / 4.0f;
        extrPts[5] = fcg + (fcg - vcg) / 2.0f;
        for(int i = 0; i < 6; i++)
        {
            GC_Point p = GC_Point(extrPts[i].x, extrPts[i].y, extrPts[i].z);

            GC_Vertex_handle v = T.nearest_vertex(p);
            GC_Point np;
            point3d npp;

            if(v != NULL)
            {
                np = v->point();
                npp = convertPointToPoint3d(np);
            }

            if((v == NULL) || ((npp - extrPts[i]).size() > mind))
            {
                GC_Vertex_handle newv = T.insert(p);
                newv->info().id = nVertices;
                newv->info().nrc = 0;
                newv->info().segSize = 0;
                newv->info().segId = -1;
                newv->info().cams = NULL;
                nVertices++;
            }
        }
    }

    // add points from voxel
    long t1 = initEstimate();
    for(int i = 0; i < voxelsIds->size(); i++)
    {
        std::string folderName = mp->mip->mvDir + "space\\spacePart" + num2strFourDecimal((*voxelsIds)[i]) + "\\";

        std::string fileNameTracksPts = folderName + "tracksPts.bin";
        std::string fileNameTracksPtsSims = folderName + "tracksPtsSims.bin";
        std::string fileNameTracksPtsNrcs = folderName + "tracksPtsNrcs.bin";
        std::string fileNameTracksPtsCams = folderName + "tracksPtsCams.bin";

        if(FileExists(fileNameTracksPts))
        {
            staticVector<float>* tracksPointsSims = loadArrayFromFile<float>(fileNameTracksPtsSims);
            staticVector<int>* tracksPointsNrcs = loadArrayFromFile<int>(fileNameTracksPtsNrcs);
            staticVector<point3d>* tracksPoints = loadArrayFromFile<point3d>(fileNameTracksPts);
            staticVector<staticVector<int>*>* tracksPointsCams = loadArrayOfArraysFromFile<int>(fileNameTracksPtsCams);

            for(int j = 0; j < tracksPoints->size(); j++)
            {
                point3d tp = (*tracksPoints)[j];

                if(doMoveIntoGridPosition == true)
                {
                    point3d cpx = closestPointOnPlaneToPoint(tp, voxel[0], vx);
                    point3d vpx = tp - cpx;
                    point3d vpy = cpx - closestPointOnPlaneToPoint(cpx, voxel[0], vy);
                    point3d vpz = cpx - closestPointOnPlaneToPoint(cpx, voxel[0], vz);

                    int nx = floor(vpx.size() / sx + 0.5f);
                    int ny = floor(vpy.size() / sy + 0.5f);
                    int nz = floor(vpz.size() / sz + 0.5f);
                    vpx = vpx.normalize() * ((float)nx * sx);
                    vpy = vpy.normalize() * ((float)ny * sy);
                    vpz = vpz.normalize() * ((float)nz * sz);

                    tp = voxel[0] + vpx + vpy + vpz;
                }

                /*
                if ((checkPoint3d(tp)==true)&&
                        (isPointInHexahedron(tp, voxel)&&
                        (nx < numSubVoxs)&&
                        (ny < numSubVoxs)&&
                        (nz < numSubVoxs))
                {
                        int id = bs->indexOf(nx*numSubVoxs*numSubVoxs+ny*numSubVoxs+nz);



                        indexes->push_back(nx*numSubVoxs*numSubVoxs+ny*numSubVoxs+nz);
                };
                */

                // GC_Cell_handle ch = GC_Cell_handle();

                if((isPointInHexahedron(tp, voxel) == true) && (checkPoint3d(tp) == true))
                {
                    GC_Point p = GC_Point(tp.x, tp.y, tp.z);

                    GC_Vertex_handle v = T.nearest_vertex(p);

                    // GC_Vertex_handle v = T.nearest_vertex(p,ch);
                    // ch = v->cell();

                    /*
                    GC_Locate_type lt;
                    int li;
                    int lj;
                    ch = T.locate(p,lt,li,lj,ch);
                    GC_Vertex_handle v = T.nearest_vertex_in_cell(p,ch);
                    */

                    GC_Point np;
                    point3d npp;
                    if(v != NULL)
                    {
                        np = v->point();
                        npp = convertPointToPoint3d(np);
                    }

                    if((v != NULL) && ((npp - tp).size() < mind))
                    {
                        // just in case that it is not a camera center
                        if(v->info().cams != NULL)
                        {
                            staticVector<int>* ptcams = (*tracksPointsCams)[j];
                            if(ptcams != NULL)
                            {
                                addCams(v, ptcams);
                            }
                            v->info().nrc += max(1, (*tracksPointsNrcs)[j]);
                        }
                    }
                    else
                    {
                        staticVector<int>* ptcams = (*tracksPointsCams)[j];
                        if((isPointInHexahedron(tp, voxel) == true) && (ptcams != NULL))
                        {
                            GC_Point p = GC_Point(tp.x, tp.y, tp.z);
                            // GC_Vertex_handle newv = T.insert(p,lt,ch,li,lj);
                            // GC_Vertex_handle newv = T.insert(p,ch);
                            GC_Vertex_handle newv = T.insert(p);
                            newv->info().point = tp;
                            newv->info().id = nVertices;
                            newv->info().nrc = max(1, (*tracksPointsNrcs)[j]);
                            newv->info().segSize = 0;
                            newv->info().segId = -1;

                            newv->info().cams = new staticVector<int>(ptcams->size());

                            for(int c = 0; c < ptcams->size(); c++)
                            {
                                int rc = (*ptcams)[c];
                                newv->info().cams->push_back(rc);
                            }

                            nVertices++;
                        }
                    }
                }

            } // for i

            delete tracksPointsSims;
            delete tracksPointsNrcs;
            delete tracksPoints;
            deleteArrayOfArrays<int>(&tracksPointsCams);
        } // if fileexists

        printfEstimate(i, voxelsIds->size(), t1);
    }
    finishEstimate();

    ///////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////
    initTriangulationDefaults(fileNameWrl);
}

staticVector<int>* mv_delanuay_GC_grid::getCamsForWhoseThePointIsVisible(point3d p, point3d tris[12][3],
                                                                         staticVector<int>* cams)
{

    int n = sizeOfStaticVector<int>(cams);
    staticVector<int>* ptcams = new staticVector<int>(n);

    for(int c = 0; c < n; c++)
    {
        int cam = (*cams)[c];
        point3d linePoint1 = p;
        point3d linePoint2 = mp->CArr[cam];

        bool isVisible = true;
        for(int i = 0; i < 12; i++)
        {
            point3d A = tris[i][0];
            point3d B = tris[i][0];
            point3d C = tris[i][0];

            point3d lpi;
            if(((A - p).size() > 0.0f) && ((B - p).size() > 0.0f) && ((C - p).size() > 0.0f) &&
               (isLineSegmentInTriangle(lpi, A, B, C, linePoint1, linePoint2) == true))
            {
                isVisible = false;
            }
        }

        if(isVisible == true)
        {
            ptcams->push_back(cam);
        }
    }

    return ptcams;
}

void mv_delanuay_GC_grid::createTriangulationFromDepthMapsCamsVoxelGrid1(staticVector<int>* cams,
                                                                         std::string fileNameWrl, int scale,
                                                                         staticVector<int>* voxelsIds, point3d voxel[8],
                                                                         int numSubVoxs)
{
    ///////////////////////////////////////////////////////////////////////////////////////
    printf("creating 3D delanuay triangulation 1\n");

    int w = mp->mip->imp.width;
    int h = mp->mip->imp.height;
    int ng = 0;

    vx = voxel[1] - voxel[0];
    vy = voxel[3] - voxel[0];
    vz = voxel[4] - voxel[0];
    sx = vx.size() / (float)numSubVoxs;
    vx = vx.normalize();
    sy = vy.size() / (float)numSubVoxs;
    vy = vy.normalize();
    sz = vz.size() / (float)numSubVoxs;
    vz = vz.normalize();

    float mind = min(sx, min(sy, sz)) / 10.0f;

    int scalePS = mp->mip->_ini.get<int>("global.scalePS", 1);
    float pointToJoinPixSizeDist =
        (float)mp->mip->_ini.get<double>("delanuaycut.pointToJoinPixSizeDist", 2.0) * (float)scalePS;
    printf("grouping threshold is %f pixelSize\n", pointToJoinPixSizeDist);

    ///////////////////////////////////////////////////////////////////////////////////////
    // build tetrahedragonalization

    int nVertices = 0;

    // add points for cam centers
    for(int camid = 0; camid < cams->size(); camid++)
    {
        int rc = (*cams)[camid];
        {
            GC_Point p = GC_Point(mp->CArr[rc].x, mp->CArr[rc].y, mp->CArr[rc].z);

            GC_Vertex_handle v = T.nearest_vertex(p);
            GC_Point np;
            point3d npp;

            if(v != NULL)
            {
                np = v->point();
                npp = convertPointToPoint3d(np);
            }

            if((v == NULL) || ((npp - mp->CArr[rc]).size() > mind))
            {
                GC_Vertex_handle newv = T.insert(p);
                newv->info().id = nVertices;
                newv->info().nrc = 0;
                newv->info().segSize = 0;
                newv->info().segId = -1;
                newv->info().cams = NULL;
                nVertices++;
            }
        }
    }

    // add 6 points to prevent singularities
    {
        point3d vcg = (voxel[0] + voxel[1] + voxel[2] + voxel[3] + voxel[4] + voxel[5] + voxel[6] + voxel[7]) / 8.0f;
        point3d extrPts[6];
        point3d fcg;
        fcg = (voxel[0] + voxel[1] + voxel[2] + voxel[3]) / 4.0f;
        extrPts[0] = fcg + (fcg - vcg) / 2.0f;
        fcg = (voxel[0] + voxel[4] + voxel[7] + voxel[3]) / 4.0f;
        extrPts[1] = fcg + (fcg - vcg) / 2.0f;
        fcg = (voxel[0] + voxel[1] + voxel[5] + voxel[4]) / 4.0f;
        extrPts[2] = fcg + (fcg - vcg) / 2.0f;
        fcg = (voxel[4] + voxel[5] + voxel[6] + voxel[7]) / 4.0f;
        extrPts[3] = fcg + (fcg - vcg) / 2.0f;
        fcg = (voxel[1] + voxel[5] + voxel[6] + voxel[2]) / 4.0f;
        extrPts[4] = fcg + (fcg - vcg) / 2.0f;
        fcg = (voxel[3] + voxel[2] + voxel[6] + voxel[7]) / 4.0f;
        extrPts[5] = fcg + (fcg - vcg) / 2.0f;
        for(int i = 0; i < 6; i++)
        {
            GC_Point p = GC_Point(extrPts[i].x, extrPts[i].y, extrPts[i].z);

            GC_Vertex_handle v = T.nearest_vertex(p);
            GC_Point np;
            point3d npp;

            if(v != NULL)
            {
                np = v->point();
                npp = convertPointToPoint3d(np);
            }

            if((v == NULL) || ((npp - extrPts[i]).size() > mind))
            {
                GC_Vertex_handle newv = T.insert(p);
                newv->info().id = nVertices;
                newv->info().nrc = 0;
                newv->info().segSize = 0;
                newv->info().segId = -1;
                newv->info().cams = NULL;
                nVertices++;
            }
        }
    }

    // add points from voxel
    long t1 = initEstimate();
    for(int i = 0; i < voxelsIds->size(); i++)
    {
        std::string folderName = mp->mip->mvDir + "space\\spacePart" + num2strFourDecimal((*voxelsIds)[i]) + "\\";

        std::string fileNameTracksPts = folderName + "tracksPts.bin";
        std::string fileNameTracksPtsSims = folderName + "tracksPtsSims.bin";
        std::string fileNameTracksPtsNrcs = folderName + "tracksPtsNrcs.bin";
        std::string fileNameTracksPtsCams = folderName + "tracksPtsCams.bin";

        if(FileExists(fileNameTracksPts))
        {
            staticVector<float>* tracksPointsSims = loadArrayFromFile<float>(fileNameTracksPtsSims);
            staticVector<int>* tracksPointsNrcs = loadArrayFromFile<int>(fileNameTracksPtsNrcs);
            staticVector<point3d>* tracksPoints = loadArrayFromFile<point3d>(fileNameTracksPts);
            staticVector<staticVector<int>*>* tracksPointsCams = loadArrayOfArraysFromFile<int>(fileNameTracksPtsCams);

            for(int j = 0; j < tracksPoints->size(); j++)
            {
                point3d tp = (*tracksPoints)[j];

                point3d cpx = closestPointOnPlaneToPoint(tp, voxel[0], vx);
                point3d vpx = tp - cpx;
                point3d vpy = cpx - closestPointOnPlaneToPoint(cpx, voxel[0], vy);
                point3d vpz = cpx - closestPointOnPlaneToPoint(cpx, voxel[0], vz);

                int nx = floor(vpx.size() / sx);
                int ny = floor(vpy.size() / sy);
                int nz = floor(vpz.size() / sz);
                vpx = vpx.normalize() * ((float)nx * sx);
                vpy = vpy.normalize() * ((float)ny * sy);
                vpz = vpz.normalize() * ((float)nz * sz);
                point3d vx1 = vpx.normalize() * sx;
                point3d vy1 = vpy.normalize() * sy;
                point3d vz1 = vpz.normalize() * sz;

                point3d hexah[8];
                hexah[0] = voxel[0] + vpx + vpy + vpz;
                hexah[1] = voxel[0] + vpx + vpy + vpz + vx1;
                hexah[2] = voxel[0] + vpx + vpy + vpz + vx1 + vy1;
                hexah[3] = voxel[0] + vpx + vpy + vpz + vy1;
                hexah[4] = voxel[0] + vpx + vpy + vpz + vz1;
                hexah[5] = voxel[0] + vpx + vpy + vpz + vz1 + vx1;
                hexah[6] = voxel[0] + vpx + vpy + vpz + vz1 + vx1 + vy1;
                hexah[7] = voxel[0] + vpx + vpy + vpz + vz1 + vy1;

                point3d tris[12][3];
                getHexahedronTriangles(tris, hexah);

                for(int k = 0; k < 8; k++)
                {
                    tp = hexah[k];
                    staticVector<int>* ptcams = getCamsForWhoseThePointIsVisible(tp, tris, (*tracksPointsCams)[j]);

                    if((isPointInHexahedron(tp, voxel) == true) && (checkPoint3d(tp) == true) && (ptcams->size() > 0))
                    {
                        GC_Point p = GC_Point(tp.x, tp.y, tp.z);

                        GC_Vertex_handle v = T.nearest_vertex(p);

                        // GC_Vertex_handle v = T.nearest_vertex(p,ch);
                        // ch = v->cell();

                        /*
                        GC_Locate_type lt;
                        int li;
                        int lj;
                        ch = T.locate(p,lt,li,lj,ch);
                        GC_Vertex_handle v = T.nearest_vertex_in_cell(p,ch);
                        */

                        GC_Point np;
                        point3d npp;
                        if(v != NULL)
                        {
                            np = v->point();
                            npp = convertPointToPoint3d(np);
                        }

                        if((v != NULL) && ((npp - tp).size() < mind))
                        {
                            // just in case that it is not a camera center
                            if(v->info().cams != NULL)
                            {
                                if(ptcams != NULL)
                                {
                                    addCams(v, ptcams);
                                }
                                v->info().nrc += max(1, (*tracksPointsNrcs)[j]);
                            }
                        }
                        else
                        {
                            if((isPointInHexahedron(tp, voxel) == true) && (ptcams != NULL))
                            {
                                GC_Point p = GC_Point(tp.x, tp.y, tp.z);
                                // GC_Vertex_handle newv = T.insert(p,lt,ch,li,lj);
                                // GC_Vertex_handle newv = T.insert(p,ch);
                                GC_Vertex_handle newv = T.insert(p);
                                newv->info().point = tp;
                                newv->info().id = nVertices;
                                newv->info().nrc = max(1, (*tracksPointsNrcs)[j]);
                                newv->info().segSize = 0;
                                newv->info().segId = -1;

                                newv->info().cams = new staticVector<int>(ptcams->size());

                                for(int c = 0; c < ptcams->size(); c++)
                                {
                                    int rc = (*ptcams)[c];
                                    newv->info().cams->push_back(rc);
                                }

                                nVertices++;
                            }
                        } // else

                    } // if

                } // for k

            } // for j

            delete tracksPointsSims;
            delete tracksPointsNrcs;
            delete tracksPoints;
            deleteArrayOfArrays<int>(&tracksPointsCams);
        } // if fileexists

        printfEstimate(i, voxelsIds->size(), t1);
    }
    finishEstimate();

    ///////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////
    initTriangulationDefaults(fileNameWrl);
}

/*
void mv_delanuay_GC_grid::fillBall(point3d pt, float lambda_vis_in, int cam)
{
        for (int x=-1;x<=1;x++) {
                for (int y=-1;y<=1;y++) {
                        for (int z=-z;z<=1;z++)
                        {
                                point3d tp = pt + vx*(sx*(float)x) + vy*(sy*(float)y) + vz*(sz*(float)z);

                                GC_Point p = GC_Point(tp.x,tp.y,tp.z);
                                GC_Vertex_handle v = T.nearest_vertex(p);

                                GC_Point np;
                                point3d npp;
                                if (v!=NULL) {
                                        np = v->point();
                                        npp = convertPointToPoint3d(np);

                                        GC_Facet f1 = getFacetBehindVertexOnTheRayToTheCam((GC_Vertex_handle)v, cam);
                                        if (f1.first!=NULL) {
                                                f1.first->info().cellTWeight += lambda_vis_in;
                                        };
                                };
                        };
                };
        };
}



void mv_delanuay_GC_grid::fillGraph(bool negVisIn, bool negVisOut, bool facetConfPhoto, float sigmaPixelSize)
{

};
*/

bool mv_delanuay_GC_grid::createTriangulationForVoxelGrid(point3d voxel[8], staticVector<int>* voxelsIds,
                                                          std::string folderName, float inflateHexahfactor,
                                                          int numSubVoxs, bool doMoveIntoGridPosition)
{
    point3d hexahInflated[8];

    printf("hexahedron inflate factor is %f\n", inflateHexahfactor);

    if(inflateHexahfactor == 1.0f)
    {
        for(int i = 0; i < 8; i++)
        {
            hexahInflated[i] = voxel[i];
        }
    }
    else
    {
        inflateHexahedron(voxel, hexahInflated, inflateHexahfactor);
    }

    long t1;

    t1 = clock();
    staticVector<int>* cams = pc->findCamsWhichInteresctsHexahedron(hexahInflated);
    printfElapsedTime(t1);

    if(cams->size() < 1)
    {
        delete cams;
        return false;
    }

    int camerasPerOneOmni = 1;
    std::string fileNameDh = folderName + "delanuayTrinagluation.bin";
    std::string fileNameInfo = folderName + "delanuayTrinagluationInfo.bin";
    std::string fileNameStGraph = folderName + "stGraph.bin";
    std::string fileNameStSolution = folderName + "stGraphSolution.bin";
    std::string fileNameTxt = folderName + "delanuayTrinaglesMaxflow.txt";
    std::string fileNameTxtCam = folderName + "delanuayTrinaglesCamerasForColoringMaxflow.txt";
    std::string fileNameDelanuayVerticesWrl = folderName + "delanuayVertices.wrl";
    std::string fileNameDelanuayVerticesSegWrl = folderName + "delanuayVerticesSeg.wrl";
    std::string fileNameCams = folderName + "cams.bin";
    std::string fileNameDelanuayVerticesSegFilteredWrl = folderName + "delanuayVerticesSegFiltered.wrl";

    saveArrayToFile<int>(fileNameCams, cams);

    t1 = clock();
    createTriangulationFromDepthMapsCamsVoxelGrid(cams, fileNameDelanuayVerticesWrl, 0, voxelsIds, hexahInflated,
                                                  numSubVoxs, doMoveIntoGridPosition);
    printfElapsedTime(t1);

    t1 = clock();
    computeVerticesSegSize(fileNameDelanuayVerticesSegWrl, true);
    printfElapsedTime(t1);

    saveDhAndDeallocate(fileNameDh, fileNameInfo);

    delete cams;
}

bool mv_delanuay_GC_grid::reconstructVoxelGrid(point3d hexah[8], staticVector<int>* voxelsIds, std::string folderName,
                                               std::string tmpCamsPtsFolderName, int numSubVoxs,
                                               bool doMoveIntoGridPosition)
{
    staticVector<int>* cams = pc->findCamsWhichInteresctsHexahedron(hexah);

    if(cams->size() < 1)
    {
        delete cams;
        return false;
    }

    int camerasPerOneOmni = 1;
    std::string fileNameDh = folderName + "delanuayTrinagluation.bin";
    std::string fileNameInfo = folderName + "delanuayTrinagluationInfo.bin";
    std::string fileNameStGraph = folderName + "stGraph.bin";
    std::string fileNameStSolution = folderName + "stGraphSolution.bin";
    std::string fileNameTxt = folderName + "delanuayTrinaglesMaxflow.txt";
    std::string fileNameTxtCam = folderName + "delanuayTrinaglesCamerasForColoringMaxflow.txt";
    std::string fileNameDelanuayVerticesWrl = folderName + "delanuayVertices.wrl";
    std::string fileNameDelanuayVerticesSegWrl = folderName + "delanuayVerticesSeg.wrl";
    std::string fileNameCams = folderName + "cams.bin";
    std::string fileNamePly = folderName + "delanuayTrinaglesMaxflow.ply";

    saveArrayToFile<int>(fileNameCams, cams);

    long t1;
    std::string et;

    t1 = clock();
    createTriangulationFromDepthMapsCamsVoxelGrid(cams, fileNameDelanuayVerticesWrl, 0, voxelsIds, hexah, numSubVoxs,
                                                  doMoveIntoGridPosition);
    // createTriangulationFromDepthMapsCamsVoxelGrid1(cams,fileNameDelanuayVerticesWrl, 0, voxelsIds, hexahInflated,
    // numSubVoxs);
    et = printfElapsedTime(t1);
    mp->printStringToLog("createTriangulationFromDepthMapsCamsVoxelGrid: " + et);

    t1 = clock();
    computeVerticesSegSize(fileNameDelanuayVerticesSegWrl, true);
    et = printfElapsedTime(t1);
    mp->printStringToLog("computeVerticesSegSize: " + et);

    // removeSmallSegs(fileNameDelanuayVerticesSegFilteredWrl);

    t1 = clock();
    reconstructExpetiments(cams, folderName, fileNameStGraph, fileNameStSolution, fileNameTxt, fileNameTxtCam,
                           camerasPerOneOmni, true, hexah, tmpCamsPtsFolderName, false, point3d(2.0f, 0.0f, 0.0f));
    et = printfElapsedTime(t1);
    mp->printStringToLog("reconstructExpetiments: " + et);

    t1 = clock();
    saveDhAndDeallocate(fileNameDh, fileNameInfo);
    et = printfElapsedTime(t1);
    mp->printStringToLog("saveDhAndDeallocate: " + et);

    //	loadDh(fileNameDh, fileNameInfo);

    //	reconstructExpetiments(cams, folderName, fileNameStGraph, fileNameStSolution, fileNameTxt, fileNameTxtCam,
    // camerasPerOneOmni, true, hexahInflated, tmpCamsPtsFolderName);

    // smooth(mp,fileNameTxt,"delanuayTrinaglesSmoothTextured.wrl","delanuayTrinaglesSmoothTextured.ply",folderName,1,true);
    // smooth(mp,fileNameTxt,"delanuayTrinaglesSmoothTextured.wrl","delanuayTrinaglesSmoothTextured.ply",folderName,1,false);
    // filterLargeTrianglesMeshDist(mp, pc, folderName, "meshTrisAreaColored.wrl", "meshAreaConsistentTextured.wrl",
    // "meshAreaConsistent.ply", "meshAreaConsistent.wrl");

    delete cams;

    return true;
}
