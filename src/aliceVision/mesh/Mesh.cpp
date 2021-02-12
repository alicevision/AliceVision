// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Mesh.hpp"
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/mvsData/geometry.hpp>
#include <aliceVision/mvsData/OrientedPoint.hpp>
#include <aliceVision/mvsData/Pixel.hpp>

#include <boost/filesystem.hpp>

#include <fstream>
#include <map>

namespace aliceVision {
namespace mesh {

namespace bfs = boost::filesystem;

Mesh::Mesh()
{
}

Mesh::~Mesh()
{
}

void Mesh::saveToObj(const std::string& filename)
{
  ALICEVISION_LOG_INFO("Save mesh to obj: " << filename);
  ALICEVISION_LOG_INFO("Nb points: " << pts.size());
  ALICEVISION_LOG_INFO("Nb triangles: " << tris.size());

  FILE* f = fopen(filename.c_str(), "w");

  fprintf(f, "# \n");
  fprintf(f, "# Wavefront OBJ file\n");
  fprintf(f, "# Created with AliceVision\n");
  fprintf(f, "# \n");
  fprintf(f, "g Mesh\n");

  if(_colors.size() == pts.size())
  {
    std::size_t i = 0;
    for(const auto& point : pts)
    {
      const rgb& col = _colors[i];
      fprintf(f, "v %f %f %f %f %f %f\n", point.x, point.y, point.z, col.r/255.0f, col.g/255.0f, col.b/255.0f);
      ++i;
    }
  }
  else
  {
    for(const auto& point : pts)
      fprintf(f, "v %f %f %f\n", point.x, point.y, point.z);    
  }

  for(int i = 0; i < tris.size(); i++)
  {
      Mesh::triangle& t = tris[i];
      fprintf(f, "f %i %i %i\n", t.v[0] + 1, t.v[1] + 1, t.v[2] + 1);
  }
  fclose(f);
  ALICEVISION_LOG_INFO("Save mesh to obj done.");
}

bool Mesh::loadFromBin(const std::string& binFileName)
{
    FILE* f = fopen(binFileName.c_str(), "rb");

    if(f == nullptr)
        return false;

    int npts;
    fread(&npts, sizeof(int), 1, f);
    pts = StaticVector<Point3d>();
    pts.resize(npts);
    fread(&pts[0], sizeof(Point3d), npts, f);

    int ntris;
    fread(&ntris, sizeof(int), 1, f);
    tris = StaticVector<Mesh::triangle>();
    tris.resize(ntris);
    fread(&tris[0], sizeof(Mesh::triangle), ntris, f);

    fclose(f);
    return true;
}

void Mesh::saveToBin(const std::string& binFileName)
{
    long t = std::clock();
    ALICEVISION_LOG_DEBUG("Save mesh to bin.");
    // printf("open\n");
    FILE* f = fopen(binFileName.c_str(), "wb");

    int npts = pts.size();
    // printf("write npts %i\n",npts);
    fwrite(&npts, sizeof(int), 1, f);
    // printf("write pts\n");
    fwrite(&pts[0], sizeof(Point3d), npts, f);

    int ntris = tris.size();
    // printf("write ntris %i\n",ntris);
    fwrite(&ntris, sizeof(int), 1, f);
    // printf("write tris\n");
    fwrite(&tris[0], sizeof(Mesh::triangle), ntris, f);

    // printf("close\n");
    fclose(f);
    // printf("done\n");
    mvsUtils::printfElapsedTime(t, "Save mesh to bin ");
}

void Mesh::addMesh(const Mesh& mesh)
{
    const std::size_t npts = pts.size();

    pts.reserveAdd(mesh.pts.size());
    std::copy(mesh.pts.begin(), mesh.pts.end(), std::back_inserter(pts.getDataWritable()));

    _colors.reserve(_colors.size() + mesh._colors.size());
    std::copy(mesh._colors.begin(), mesh._colors.end(), std::back_inserter(_colors));

    tris.reserveAdd(mesh.tris.size());
    for(int i = 0; i < mesh.tris.size(); i++)
    {
        Mesh::triangle t = mesh.tris[i];
        // check triangles indices validity
        if(   (t.v[0] >= 0 && t.v[0] < mesh.pts.size())
           && (t.v[1] >= 0 && t.v[1] < mesh.pts.size())
           && (t.v[2] >= 0 && t.v[2] < mesh.pts.size()))
        {
            t.v[0] += npts;
            t.v[1] += npts;
            t.v[2] += npts;
            tris.push_back(t);
        }
        else
        {
            ALICEVISION_LOG_WARNING("addMesh: bad triangle index: " << t.v[0] << " " << t.v[1] << " " << t.v[2] << ", npts: " << mesh.pts.size());
        }
    }

    if(!mesh.uvCoords.empty())
    {
        uvCoords.reserve(mesh.uvCoords.size());
        std::copy(mesh.uvCoords.begin(), mesh.uvCoords.end(), std::back_inserter(uvCoords.getDataWritable()));
    }
    if(!mesh.trisUvIds.empty())
    {
        trisUvIds.reserve(mesh.trisUvIds.size());
        std::copy(mesh.trisUvIds.begin(), mesh.trisUvIds.end(), std::back_inserter(trisUvIds.getDataWritable()));
    }
}

Mesh::triangle_proj Mesh::getTriangleProjection(int triid, const mvsUtils::MultiViewParams& mp, int rc, int w, int h) const
{
    int ow = mp.getWidth(rc);
    int oh = mp.getHeight(rc);

    triangle_proj tp;
    for(int j = 0; j < 3; j++)
    {
        mp.getPixelFor3DPoint(&tp.tp2ds[j], pts[tris[triid].v[j]], rc);
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

bool Mesh::isTriangleProjectionInImage(const mvsUtils::MultiViewParams& mp, const Mesh::triangle_proj& tp, int camId, int margin) const
{
    return (getTriangleNbVertexInImage(mp, tp, camId, margin) == 3);
}

int Mesh::getTriangleNbVertexInImage(const mvsUtils::MultiViewParams& mp, const Mesh::triangle_proj& tp, int camId, int margin) const
{
    int nbVertexInImage = 0;
    for (int j = 0; j < 3; j++)
    {
        if(mp.isPixelInImage(tp.tpixs[j], camId, margin) && mp.isPixelInSourceImage(tp.tpixs[j], camId, margin))
        {
            ++nbVertexInImage;
        }
    }
    return nbVertexInImage;
}

void Mesh::getTrianglePixelIntersectionsAndInternalPoints(Mesh::triangle_proj& tp, Mesh::rectangle& re, StaticVector<Point2d>& out)
{
    out.reserve(20);

    if(isPointInTriangle(tp.tp2ds[0], tp.tp2ds[1], tp.tp2ds[2], re.P[0]))
    {
        out.push_back(re.P[0]);
    }
    if(isPointInTriangle(tp.tp2ds[0], tp.tp2ds[1], tp.tp2ds[2], re.P[1]))
    {
        out.push_back(re.P[1]);
    }
    if(isPointInTriangle(tp.tp2ds[0], tp.tp2ds[1], tp.tp2ds[2], re.P[2]))
    {
        out.push_back(re.P[2]);
    }
    if(isPointInTriangle(tp.tp2ds[0], tp.tp2ds[1], tp.tp2ds[2], re.P[3]))
    {
        out.push_back(re.P[3]);
    }
    if((isPointInTriangle(re.P[0], re.P[1], re.P[2], tp.tp2ds[0])) ||
       (isPointInTriangle(re.P[2], re.P[3], re.P[0], tp.tp2ds[0])))
    {
        out.push_back(tp.tp2ds[0]);
    }
    if((isPointInTriangle(re.P[0], re.P[1], re.P[2], tp.tp2ds[1])) ||
       (isPointInTriangle(re.P[2], re.P[3], re.P[0], tp.tp2ds[1])))
    {
        out.push_back(tp.tp2ds[1]);
    }
    if((isPointInTriangle(re.P[0], re.P[1], re.P[2], tp.tp2ds[2])) ||
       (isPointInTriangle(re.P[2], re.P[3], re.P[0], tp.tp2ds[2])))
    {
        out.push_back(tp.tp2ds[2]);
    }

    Point2d lli;
    if(lineSegmentsIntersect2DTest(lli, tp.tp2ds[0], tp.tp2ds[1], re.P[0], re.P[1]))
    {
        out.push_back(lli);
    }
    if(lineSegmentsIntersect2DTest(lli, tp.tp2ds[1], tp.tp2ds[2], re.P[0], re.P[1]))
    {
        out.push_back(lli);
    }
    if(lineSegmentsIntersect2DTest(lli, tp.tp2ds[2], tp.tp2ds[0], re.P[0], re.P[1]))
    {
        out.push_back(lli);
    }

    if(lineSegmentsIntersect2DTest(lli, tp.tp2ds[0], tp.tp2ds[1], re.P[1], re.P[2]))
    {
        out.push_back(lli);
    }
    if(lineSegmentsIntersect2DTest(lli, tp.tp2ds[1], tp.tp2ds[2], re.P[1], re.P[2]))
    {
        out.push_back(lli);
    }
    if(lineSegmentsIntersect2DTest(lli, tp.tp2ds[2], tp.tp2ds[0], re.P[1], re.P[2]))
    {
        out.push_back(lli);
    }

    if(lineSegmentsIntersect2DTest(lli, tp.tp2ds[0], tp.tp2ds[1], re.P[2], re.P[3]))
    {
        out.push_back(lli);
    }
    if(lineSegmentsIntersect2DTest(lli, tp.tp2ds[1], tp.tp2ds[2], re.P[2], re.P[3]))
    {
        out.push_back(lli);
    }
    if(lineSegmentsIntersect2DTest(lli, tp.tp2ds[2], tp.tp2ds[0], re.P[2], re.P[3]))
    {
        out.push_back(lli);
    }

    if(lineSegmentsIntersect2DTest(lli, tp.tp2ds[0], tp.tp2ds[1], re.P[3], re.P[0]))
    {
        out.push_back(lli);
    }
    if(lineSegmentsIntersect2DTest(lli, tp.tp2ds[1], tp.tp2ds[2], re.P[3], re.P[0]))
    {
        out.push_back(lli);
    }
    if(lineSegmentsIntersect2DTest(lli, tp.tp2ds[2], tp.tp2ds[0], re.P[3], re.P[0]))
    {
        out.push_back(lli);
    }
}

void Mesh::getTrianglePixelIntersectionsAndInternalPoints(const mvsUtils::MultiViewParams& mp, int idTri, Pixel&  /*pix*/,
                                                                int rc, Mesh::triangle_proj& tp, Mesh::rectangle& re,
                                                                StaticVector<Point3d>& out)
{

    Point3d A = pts[tris[idTri].v[0]];
    Point3d B = pts[tris[idTri].v[1]];
    Point3d C = pts[tris[idTri].v[2]];

    triangleRectangleIntersection(A, B, C, mp, rc, re.P, out);

    if((isPointInTriangle(re.P[0], re.P[1], re.P[2], tp.tp2ds[0])) ||
       (isPointInTriangle(re.P[2], re.P[3], re.P[0], tp.tp2ds[0])))
    {
        out.push_back(A);
    }
    if((isPointInTriangle(re.P[0], re.P[1], re.P[2], tp.tp2ds[1])) ||
       (isPointInTriangle(re.P[2], re.P[3], re.P[0], tp.tp2ds[1])))
    {
        out.push_back(B);
    }
    if((isPointInTriangle(re.P[0], re.P[1], re.P[2], tp.tp2ds[2])) ||
       (isPointInTriangle(re.P[2], re.P[3], re.P[0], tp.tp2ds[2])))
    {
        out.push_back(C);
    }
}

Point2d Mesh::getTrianglePixelInternalPoint(Mesh::triangle_proj& tp, Mesh::rectangle& re)
{

    if((isPointInTriangle(re.P[0], re.P[1], re.P[2], tp.tp2ds[0])) ||
       (isPointInTriangle(re.P[2], re.P[3], re.P[0], tp.tp2ds[0])))
    {
        return tp.tp2ds[0];
    }
    if((isPointInTriangle(re.P[0], re.P[1], re.P[2], tp.tp2ds[1])) ||
       (isPointInTriangle(re.P[2], re.P[3], re.P[0], tp.tp2ds[1])))
    {
        return tp.tp2ds[1];
    }
    if((isPointInTriangle(re.P[0], re.P[1], re.P[2], tp.tp2ds[2])) ||
       (isPointInTriangle(re.P[2], re.P[3], re.P[0], tp.tp2ds[2])))
    {
        return tp.tp2ds[2];
    }

    if(isPointInTriangle(tp.tp2ds[0], tp.tp2ds[1], tp.tp2ds[2], re.P[0]))
    {
        return re.P[0];
    }
    if(isPointInTriangle(tp.tp2ds[0], tp.tp2ds[1], tp.tp2ds[2], re.P[1]))
    {
        return re.P[1];
    }
    if(isPointInTriangle(tp.tp2ds[0], tp.tp2ds[1], tp.tp2ds[2], re.P[2]))
    {
        return re.P[2];
    }
    if(isPointInTriangle(tp.tp2ds[0], tp.tp2ds[1], tp.tp2ds[2], re.P[3]))
    {
        return re.P[3];
    }
    throw std::runtime_error("No valid point in triangle.");
}

bool Mesh::doesTriangleIntersectsRectangle(Mesh::triangle_proj& tp, Mesh::rectangle& re)
{

    Point2d p1[3];
    Point2d p2[3];
    p1[0] = re.P[0];
    p1[1] = re.P[1];
    p1[2] = re.P[2];
    p2[0] = re.P[2];
    p2[1] = re.P[3];
    p2[2] = re.P[0];

    return ((TrianglesOverlap(tp.tp2ds, p1)) || (TrianglesOverlap(tp.tp2ds, p2)) ||
            (isPointInTriangle(p1[0], p1[1], p1[2], tp.tp2ds[0])) ||
            (isPointInTriangle(p1[0], p1[1], p1[2], tp.tp2ds[1])) ||
            (isPointInTriangle(p1[0], p1[1], p1[2], tp.tp2ds[2])) ||
            (isPointInTriangle(p2[0], p2[1], p2[2], tp.tp2ds[0])) ||
            (isPointInTriangle(p2[0], p2[1], p2[2], tp.tp2ds[1])) ||
            (isPointInTriangle(p2[0], p2[1], p2[2], tp.tp2ds[2])));

    /*
            Point2d p;
            Point2d *_p = &p;

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

            Point2d PS = (re->P[0]+re->P[1]+re->P[2]+re->P[3])/4.0;

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

void Mesh::getPtsNeighborTriangles(StaticVector<StaticVector<int>>& out_ptsNeighTris) const
{
    // array of tuples <x: vertexIndex, y: triangleIndex, z: numberOfNeighbors>
    StaticVector<Voxel> vertexNeighborhoodPairs;
    vertexNeighborhoodPairs.reserve(tris.size() * 3);
    for(int i = 0; i < tris.size(); ++i)
    {
        vertexNeighborhoodPairs.push_back(Voxel(tris[i].v[0], i, 0));
        vertexNeighborhoodPairs.push_back(Voxel(tris[i].v[1], i, 0));
        vertexNeighborhoodPairs.push_back(Voxel(tris[i].v[2], i, 0));
    }
    qsort(&vertexNeighborhoodPairs[0], vertexNeighborhoodPairs.size(), sizeof(Voxel), qSortCompareVoxelByXAsc);

    int i = 0; // index of the unique pair of <vertex, neighborhood>
    int j = 0; // index of the vertex
    int k = 0; // number of neighbors
    int firstid = 0;
    while(i < vertexNeighborhoodPairs.size())
    {
        k++;
        // (*vertexNeighborhoodPairs)[i].z = j;
        if((i == vertexNeighborhoodPairs.size() - 1) || (vertexNeighborhoodPairs[i].x != vertexNeighborhoodPairs[i + 1].x))
        {
            vertexNeighborhoodPairs[firstid].z = k; // store the number of neighbors
            ++j;
            firstid = i + 1;
            k = 0;
        }
        ++i;
    }
    int npts = j;

    out_ptsNeighTris.reserve(pts.size());
    out_ptsNeighTris.resize(pts.size());

    i = 0;
    for(j = 0; j < npts; ++j)
    {
        int middlePtId = vertexNeighborhoodPairs[i].x;
        int nbNeighbors = vertexNeighborhoodPairs[i].z;
        int i0 = i;
        int i1 = i + nbNeighbors;
        i = i1;

        StaticVector<int>& triTmp = out_ptsNeighTris[middlePtId];
        triTmp.reserve(nbNeighbors);
        for(int l = i0; l < i1; ++l)
        {
            triTmp.push_back(vertexNeighborhoodPairs[l].y); // index of triangle
        }
    }
}

void Mesh::getPtsNeighbors(std::vector<std::vector<int>>& out_ptsNeigh) const
{
    out_ptsNeigh.resize(pts.size());
    for(int triangleId = 0; triangleId < tris.size(); ++triangleId)
    {
        const Mesh::triangle& triangle = tris[triangleId];
        for(int k = 0; k < 3; ++k)
        {
            int ptId = triangle.v[k];
            std::vector<int>& ptNeigh = out_ptsNeigh[ptId];
            if(std::find(ptNeigh.begin(), ptNeigh.end(), triangle.v[(k+1)%3]) == ptNeigh.end())
                ptNeigh.push_back(triangle.v[(k+1)%3]);
            if(std::find(ptNeigh.begin(), ptNeigh.end(), triangle.v[(k+2)%3]) == ptNeigh.end())
                ptNeigh.push_back(triangle.v[(k+2)%3]);
        }
    }
}


void Mesh::getPtsNeighPtsOrdered(StaticVector<StaticVector<int>>& out_ptsNeighPts) const
{
    StaticVector<StaticVector<int>> ptsNeighborTriangles;
    getPtsNeighborTriangles(ptsNeighborTriangles);

    out_ptsNeighPts.resize(pts.size());

    for(int middlePtId = 0; middlePtId < pts.size(); ++middlePtId)
    {
        StaticVector<int>& neighborTriangles = ptsNeighborTriangles[middlePtId];
        if(neighborTriangles.empty())
            continue;

        StaticVector<int> vhid;
        vhid.reserve(neighborTriangles.size() * 2);
        int currentTriPtId = tris[neighborTriangles[0]].v[0];
        int firstTriPtId = currentTriPtId;
        vhid.push_back(currentTriPtId);

        bool isThereTWithCurrentTriPtId = true;
        while(!neighborTriangles.empty() && isThereTWithCurrentTriPtId)
        {
            isThereTWithCurrentTriPtId = false;

            // find triangle with middlePtId and currentTriPtId and get remaining point id
            for(int n = 0; n < neighborTriangles.size(); ++n)
            {
                bool ok_middlePtId = false;
                bool ok_actTriPtId = false;
                int remainingPtId = -1; // remaining pt id
                for(int k = 0; k < 3; ++k)
                {
                    int triPtId = tris[neighborTriangles[n]].v[k];
                    double length = (pts[middlePtId] - pts[triPtId]).size();
                    if((triPtId != middlePtId) && (triPtId != currentTriPtId) && (length > 0.0) && (!std::isnan(length)))
                    {
                        remainingPtId = triPtId;
                    }
                    if(triPtId == middlePtId)
                    {
                        ok_middlePtId = true;
                    }
                    if(triPtId == currentTriPtId)
                    {
                        ok_actTriPtId = true;
                    }
                }

                if(ok_middlePtId && ok_actTriPtId && (remainingPtId > -1))
                {
                    currentTriPtId = remainingPtId;
                    neighborTriangles.remove(n);
                    vhid.push_back(currentTriPtId);
                    isThereTWithCurrentTriPtId = true; // we removed one, so we try again
                    break;
                }
            }
        }

        if(!vhid.empty())
        {
            if(currentTriPtId == firstTriPtId)
            {
                vhid.pop(); // remove last ... which is first
            }

            // remove duplicates
            StaticVector<int>& vhid1 = out_ptsNeighPts[middlePtId];
            vhid1.reserve(vhid.size());
            for(int k1 = 0; k1 < vhid.size(); k1++)
            {
                if(vhid1.indexOf(vhid[k1]) == -1)
                {
                    vhid1.push_back(vhid[k1]);
                }
            }
        }
    }
}

void Mesh::getTrisMap(StaticVector<StaticVector<int>>& out, const mvsUtils::MultiViewParams& mp, int rc, int  /*scale*/, int w, int h)
{
    long tstart = clock();

    ALICEVISION_LOG_INFO("getTrisMap.");
    StaticVector<int> nmap;
    nmap.reserve(w * h);
    nmap.resize_with(w * h, 0);

    long t1 = mvsUtils::initEstimate();
    for(int i = 0; i < tris.size(); i++)
    {
        triangle_proj tp = getTriangleProjection(i, mp, rc, w, h);
        if((isTriangleProjectionInImage(mp, tp, rc, 0)))
        {
            Pixel pix;
            for(pix.x = tp.lu.x; pix.x <= tp.rd.x; pix.x++)
            {
                for(pix.y = tp.lu.y; pix.y <= tp.rd.y; pix.y++)
                {
                    Mesh::rectangle re = Mesh::rectangle(pix, 1);
                    if(doesTriangleIntersectsRectangle(tp, re))
                    {
                        nmap[pix.x * h + pix.y] += 1;
                    }
                } // for y
            }     // for x
        }         // isthere
        mvsUtils::printfEstimate(i, tris.size(), t1);
    } // for i ntris
    mvsUtils::finishEstimate();

    // allocate
    out.reserve(w * h);
    out.resize(w * h);
    for(int i = 0; i < w * h; i++)
    {
        if(nmap[i] > 0)
        {
            out[i].reserve(nmap[i]);
        }
    }

    // fill
    t1 = mvsUtils::initEstimate();
    for(int i = 0; i < tris.size(); i++)
    {
        triangle_proj tp = getTriangleProjection(i, mp, rc, w, h);
        if((isTriangleProjectionInImage(mp, tp, rc, 0)))
        {
            Pixel pix;
            for(pix.x = tp.lu.x; pix.x <= tp.rd.x; pix.x++)
            {
                for(pix.y = tp.lu.y; pix.y <= tp.rd.y; pix.y++)
                {
                    Mesh::rectangle re = Mesh::rectangle(pix, 1);
                    if(doesTriangleIntersectsRectangle(tp, re))
                    {
                        out[pix.x * h + pix.y].push_back(i);
                    }
                } // for y
            }     // for x
        }         // isthere
        mvsUtils::printfEstimate(i, tris.size(), t1);
    } // for i ntris
    mvsUtils::finishEstimate();

    mvsUtils::printfElapsedTime(tstart);
}

void Mesh::getTrisMap(StaticVector<StaticVector<int>>& out, StaticVector<int>& visTris, const mvsUtils::MultiViewParams& mp, int rc,
                                                      int  /*scale*/, int w, int h)
{
    long tstart = clock();

    ALICEVISION_LOG_INFO("getTrisMap.");
    StaticVector<int> nmap;
    nmap.reserve(w * h);
    nmap.resize_with(w * h, 0);

    long t1 = mvsUtils::initEstimate();
    for(int m = 0; m < visTris.size(); m++)
    {
        int i = visTris[m];
        triangle_proj tp = getTriangleProjection(i, mp, rc, w, h);
        if((isTriangleProjectionInImage(mp, tp, rc, 0)))
        {
            Pixel pix;
            for(pix.x = tp.lu.x; pix.x <= tp.rd.x; pix.x++)
            {
                for(pix.y = tp.lu.y; pix.y <= tp.rd.y; pix.y++)
                {
                    Mesh::rectangle re = Mesh::rectangle(pix, 1);
                    if(doesTriangleIntersectsRectangle(tp, re))
                    {
                        nmap[pix.x * h + pix.y] += 1;
                    }
                } // for y
            }     // for x
        }         // isthere
        mvsUtils::printfEstimate(i, tris.size(), t1);
    } // for i ntris
    mvsUtils::finishEstimate();

    // allocate
    out.resize(w * h);
    for(int i = 0; i < w * h; i++)
    {
        if(nmap[i] > 0)
        {
            out[i].reserve(nmap[i]);
        }
    }

    // fill
    t1 = mvsUtils::initEstimate();
    for(int m = 0; m < visTris.size(); m++)
    {
        int i = visTris[m];
        triangle_proj tp = getTriangleProjection(i, mp, rc, w, h);
        if((isTriangleProjectionInImage(mp, tp, rc, 0)))
        {
            Pixel pix;
            for(pix.x = tp.lu.x; pix.x <= tp.rd.x; pix.x++)
            {
                for(pix.y = tp.lu.y; pix.y <= tp.rd.y; pix.y++)
                {
                    Mesh::rectangle re = Mesh::rectangle(pix, 1);
                    if(doesTriangleIntersectsRectangle(tp, re))
                    {
                        out[pix.x * h + pix.y].push_back(i);
                    }
                } // for y
            }     // for x
        }         // isthere
        mvsUtils::printfEstimate(i, tris.size(), t1);
    } // for i ntris
    mvsUtils::finishEstimate();

    mvsUtils::printfElapsedTime(tstart);
}

void Mesh::getDepthMap(StaticVector<float>& depthMap, const mvsUtils::MultiViewParams& mp, int rc, int scale, int w, int h)
{
    StaticVector<StaticVector<int>> tmp;
    getTrisMap(tmp, mp, rc, scale, w, h);
    getDepthMap(depthMap, tmp, mp, rc, scale, w, h);
}

void Mesh::getDepthMap(StaticVector<float>& depthMap, StaticVector<StaticVector<int>>& tmp, const mvsUtils::MultiViewParams& mp,
                          int rc, int scale, int w, int h)
{
    depthMap.resize_with(w * h, -1.0f);

    Pixel pix;
    for(pix.x = 0; pix.x < w; pix.x++)
    {
        for(pix.y = 0; pix.y < h; pix.y++)
        {

            StaticVector<int>& ti = tmp[pix.x * h + pix.y];
            if(!ti.empty())
            {
                Point2d p;
                p.x = (double)pix.x;
                p.y = (double)pix.y;

                double mindepth = 10000000.0;

                for(int i = 0; i < ti.size(); i++)
                {
                    int idTri = ti[i];
                    OrientedPoint tri;
                    tri.p = pts[tris[idTri].v[0]];
                    tri.n = cross((pts[tris[idTri].v[1]] - pts[tris[idTri].v[0]]).normalize(),
                                  (pts[tris[idTri].v[2]] - pts[tris[idTri].v[0]]).normalize());

                    Mesh::rectangle re = Mesh::rectangle(pix, 1);
                    triangle_proj tp = getTriangleProjection(idTri, mp, rc, w, h);

                    StaticVector<Point2d> tpis;
                    getTrianglePixelIntersectionsAndInternalPoints(tp, re, tpis);

                    double maxd = -1.0;
                    for(int k = 0; k < tpis.size(); k++)
                    {
                        Point3d lpi = linePlaneIntersect(
                            mp.CArr[rc], (mp.iCamArr[rc] * (tpis[k] * (float)scale)).normalize(), tri.p, tri.n);
                        if(!std::isnan(angleBetwV1andV2((mp.CArr[rc] - tri.p).normalize(), tri.n)))
                        {
                            maxd = std::max(maxd, (mp.CArr[rc] - lpi).size());
                        }
                        else
                        {
                            maxd = std::max(maxd, (mp.CArr[rc] - pts[tris[idTri].v[1]]).size());

                            /*
                            StaticVector<Point3d> *tpis1 = getTrianglePixelIntersectionsAndInternalPoints(mp, idTri,
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
                        Point3d lpi =
                linePlaneIntersect(mp->CArr[rc],(mp->iCamArr[rc]*Point2d(p.x,p.y)).normalize(),tri.p,tri.n);
                        if ((mp->is3DPointInFrontOfCam(&lpi,rc)==true)&&
                                (isLineInTriangle(
                                        &(*pts)[(*tris)[idTri].i[0]],
                                        &(*pts)[(*tris)[idTri].i[1]],
                                        &(*pts)[(*tris)[idTri].i[2]],
                                        &mp->CArr[rc],
                                        &(mp->iCamArr[rc]*Point2d(p.x,p.y)).normalize())==true))
                        {
                                mindepth=std::min(mindepth,(mp->CArr[rc]-lpi).size());
                        };
                };
                */

                depthMap[pix.x * h + pix.y] = mindepth;
            }
            else
            {
                depthMap[pix.x * h + pix.y] = -1.0f;
            }
        } // for pix.y
    }     // for pix.x
}

void Mesh::getVisibleTrianglesIndexes(StaticVector<int>& out_visTri, const std::string& depthMapFileName, const std::string& trisMapFileName,
                                                       const mvsUtils::MultiViewParams& mp, int rc, int w, int h)
{
    StaticVector<float> depthMap;
    loadArrayFromFile<float>(depthMap, depthMapFileName);
    StaticVector<StaticVector<int>> trisMap;
    loadArrayOfArraysFromFile<int>(trisMap, trisMapFileName);

    getVisibleTrianglesIndexes(out_visTri, trisMap, depthMap, mp, rc, w, h);
}

void Mesh::getVisibleTrianglesIndexes(StaticVector<int>& out_visTri, const std::string& tmpDir, const mvsUtils::MultiViewParams& mp, int rc, int w, int h)
{
    std::string depthMapFileName = tmpDir + "depthMap" + std::to_string(mp.getViewId(rc)) + ".bin";
    std::string trisMapFileName = tmpDir + "trisMap" + std::to_string(mp.getViewId(rc)) + ".bin";

    StaticVector<float> depthMap;
    loadArrayFromFile<float>(depthMap, depthMapFileName);
    StaticVector<StaticVector<int>> trisMap;
    loadArrayOfArraysFromFile<int>(trisMap, trisMapFileName);

    getVisibleTrianglesIndexes(out_visTri, trisMap, depthMap, mp, rc, w, h);
}

void Mesh::getVisibleTrianglesIndexes(StaticVector<int>& out_visTri, StaticVector<float>& depthMap, const mvsUtils::MultiViewParams& mp, int rc,
                                                       int w, int h)
{
    int ow = mp.getWidth(rc);
    int oh = mp.getHeight(rc);

    out_visTri.reserve(tris.size());

    for(int i = 0; i < tris.size(); i++)
    {
        Point3d cg = computeTriangleCenterOfGravity(i);
        Pixel pix;
        mp.getPixelFor3DPoint(&pix, cg, rc);
        if(mp.isPixelInImage(pix, rc, 1))
        {
            pix.x = (int)(((float)pix.x / (float)ow) * (float)w);
            pix.y = (int)(((float)pix.y / (float)oh) * (float)h);
            float depth = depthMap[pix.x * h + pix.y];
            float pixSize = mp.getCamPixelSize(cg, rc) * 6.0f;
            // if (depth-pixSize<(mp->CArr[rc]-cg).size()) {
            if(fabs(depth - (mp.CArr[rc] - cg).size()) < pixSize)
            {
                out_visTri.push_back(i);
            }
        }
    }
}

void Mesh::getVisibleTrianglesIndexes(StaticVector<int>& out_visTri, StaticVector<StaticVector<int>>& trisMap,
                                                       StaticVector<float>& depthMap, const mvsUtils::MultiViewParams& mp, int rc,
                                                       int w, int h)
{
    int ow = mp.getWidth(rc);
    int oh = mp.getHeight(rc);

    StaticVectorBool btris;
    btris.reserve(tris.size());
    btris.resize_with(tris.size(), false);

    Pixel pix;
    for(pix.x = 0; pix.x < w; pix.x++)
    {
        for(pix.y = 0; pix.y < h; pix.y++)
        {
            StaticVector<int>& ti = trisMap[pix.x * h + pix.y];
            if(!ti.empty())
            {
                Point2d p;
                p.x = (float)pix.x;
                p.y = (float)pix.y;

                float depth = depthMap[pix.x * h + pix.y];
                for(int i = 0; i < ti.size(); i++)
                {
                    int idTri = ti[i];
                    OrientedPoint tri;
                    tri.p = pts[tris[idTri].v[0]];
                    tri.n = cross((pts[tris[idTri].v[1]] - pts[tris[idTri].v[0]]).normalize(),
                                  (pts[tris[idTri].v[2]] - pts[tris[idTri].v[0]]).normalize());

                    Mesh::rectangle re = Mesh::rectangle(pix, 1);
                    triangle_proj tp = getTriangleProjection(idTri, mp, rc, w, h);

                    /*
                    StaticVector<Point2d> *tpis = getTrianglePixelIntersectionsAndInternalPoints(&tp, &re);
                    float mindepth = 10000000.0f;
                    Point3d minlpi;
                    for (int k=0;k<tpis->size();k++) {
                            Point3d lpi =
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

                    Point2d tpip = getTrianglePixelInternalPoint(tp, re);
                    tpip.x = (tpip.x / (float)w) * (float)ow;
                    tpip.y = (tpip.y / (float)h) * (float)oh;

                    Point3d lpi = linePlaneIntersect(mp.CArr[rc], (mp.iCamArr[rc] * tpip).normalize(), tri.p, tri.n);
                    float lpidepth = (mp.CArr[rc] - lpi).size();
                    float pixSize = mp.getCamPixelSize(lpi, rc) * 2.0f;
                    if(fabs(depth - lpidepth) < pixSize)
                    {
                        btris[idTri] = true;
                    }
                }
            }
        } // for pix.y
    }     // for pix.x

    int nvistris = 0;
    for(int i = 0; i < btris.size(); i++)
    {
        if(btris[i])
        {
            nvistris++;
        }
    }

    out_visTri.reserve(nvistris);

    for(int i = 0; i < btris.size(); i++)
    {
        if(btris[i])
        {
            out_visTri.push_back(i);
        }
    }
}

void Mesh::generateMeshFromTrianglesSubset(const StaticVector<int>& visTris, Mesh& outMesh, StaticVector<int>& out_ptIdToNewPtId) const
{
    out_ptIdToNewPtId.resize_with(pts.size(), -1); // -1 means unused
    for(int i = 0; i < visTris.size(); i++)
    {
        int idTri = visTris[i];
        out_ptIdToNewPtId[tris[idTri].v[0]] = 0; // 0 means used
        out_ptIdToNewPtId[tris[idTri].v[1]] = 0;
        out_ptIdToNewPtId[tris[idTri].v[2]] = 0;
    }

    int j = 0;
    for(int i = 0; i < pts.size(); i++)
    {
        if(out_ptIdToNewPtId[i] == 0) // if input point used
        {
            out_ptIdToNewPtId[i] = j;
            ++j;
        }
    }

    outMesh.pts.reserve(j);
    
    // also update vertex color data if any
    const bool updateColors = _colors.size() != 0;
    auto& outColors = outMesh.colors();
    outColors.reserve(_colors.size());

    for(int i = 0; i < pts.size(); i++)
    {
        if(out_ptIdToNewPtId[i] > -1)
        {
            outMesh.pts.push_back(pts[i]);
            if(updateColors)
                outColors.push_back(_colors[i]);
        }
    }

    outMesh.tris.reserve(visTris.size());
    for(int i = 0; i < visTris.size(); i++)
    {
        int idTri = visTris[i];
        Mesh::triangle t;
        t.alive = true;
        t.v[0] = out_ptIdToNewPtId[tris[idTri].v[0]];
        t.v[1] = out_ptIdToNewPtId[tris[idTri].v[1]];
        t.v[2] = out_ptIdToNewPtId[tris[idTri].v[2]];
        outMesh.tris.push_back(t);
    }
}

void Mesh::getNotOrientedEdges(StaticVector<StaticVector<int>>& edgesNeighTris, StaticVector<Pixel>& edgesPointsPairs)
{
    StaticVector<Voxel> edges;
    edges.reserve(tris.size() * 3);

    for(int i = 0; i < tris.size(); i++)
    {
        int a = tris[i].v[0];
        int b = tris[i].v[1];
        int c = tris[i].v[2];
        edges.push_back(Voxel(std::min(a, b), std::max(a, b), i));
        edges.push_back(Voxel(std::min(b, c), std::max(b, c), i));
        edges.push_back(Voxel(std::min(c, a), std::max(c, a), i));
    }

    qsort(&edges[0], edges.size(), sizeof(Voxel), qSortCompareVoxelByXAsc);

    edgesNeighTris.reserve(tris.size() * 3);
    edgesPointsPairs.reserve(tris.size() * 3);
    // remove duplicities
    int i0 = 0;
    long t1 = mvsUtils::initEstimate();
    for(int i = 0; i < edges.size(); i++)
    {
        if((i == edges.size() - 1) || (edges[i].x != edges[i + 1].x))
        {
            StaticVector<Voxel> edges1;
            edges1.reserve(i - i0 + 1);
            edges1.getDataWritable().insert(edges1.begin(), edges.begin()+i0, edges.begin()+i+1);
            qsort(&edges1[0], edges1.size(), sizeof(Voxel), qSortCompareVoxelByYAsc);

            int j0 = 0;
            for(int j = 0; j < edges1.size(); j++)
            {
                if((j == edges1.size() - 1) || (edges1[j].y != edges1[j + 1].y))
                {
                    edgesPointsPairs.push_back(Pixel(edges1[j].x, edges1[j].y));
                    edgesNeighTris.resize(edgesNeighTris.size() + 1);
                    StaticVector<int>& neighTris = edgesNeighTris.back();
                    neighTris.reserve(j - j0 + 1);
                    for(int k = j0; k <= j; k++)
                    {
                        neighTris.push_back(edges1[k].z);
                    }
                    j0 = j + 1;
                }
            }
            i0 = i + 1;
        }
        mvsUtils::printfEstimate(i, edges.size(), t1);
    }
    mvsUtils::finishEstimate();
}

void Mesh::getLaplacianSmoothingVectors(StaticVector<StaticVector<int>>& ptsNeighPts, StaticVector<Point3d>& out_nms,
                                        double maximalNeighDist)
{
    out_nms.reserve(pts.size());

    for(int i = 0; i < pts.size(); i++)
    {
        Point3d& p = pts[i];
        StaticVector<int>& nei = ptsNeighPts[i];
        int nneighs = 0;
        if(!nei.empty())
        {
            nneighs = nei.size();
        }

        if(nneighs == 0)
        {
            out_nms.push_back(Point3d(0.0, 0.0, 0.0));
        }
        else
        {
            double maxNeighDist = 0.0f;
            // laplacian smoothing vector
            Point3d n = Point3d(0.0, 0.0, 0.0);
            for(int j = 0; j < nneighs; j++)
            {
                n = n + pts[nei[j]];
                maxNeighDist = std::max(maxNeighDist, (p - pts[nei[j]]).size());
            }
            n = (n / (float)nneighs) - p;

            float d = n.size();
            n = n.normalize();

            if(std::isnan(d) || std::isnan(n.x) || std::isnan(n.y) || std::isnan(n.z)) // check if is not NaN
            {
                n = Point3d(0.0, 0.0, 0.0);
            }
            else
            {
                n = n * d;
            }

            if(std::isnan(d) || std::isnan(n.x) || std::isnan(n.y) || std::isnan(n.z)) // check if is not NaN
            {
                n = Point3d(0.0, 0.0, 0.0);
            }

            if((maximalNeighDist > 0.0f) && (maxNeighDist > maximalNeighDist))
            {
                n = Point3d(0.0, 0.0, 0.0);
            }

            out_nms.push_back(n);
        }
    }
}

void Mesh::laplacianSmoothPts(float maximalNeighDist)
{
    StaticVector<StaticVector<int>> ptsNei;
    getPtsNeighPtsOrdered(ptsNei);
    laplacianSmoothPts(ptsNei, maximalNeighDist);
}

void Mesh::laplacianSmoothPts(StaticVector<StaticVector<int>>& ptsNeighPts, double maximalNeighDist)
{
    StaticVector<Point3d> nms;
    getLaplacianSmoothingVectors(ptsNeighPts, nms, maximalNeighDist);

    // smooth
    for(int i = 0; i < pts.size(); i++)
    {
        pts[i] = pts[i] + nms[i];
    }
}

Point3d Mesh::computeTriangleNormal(int idTri)
{
    const Mesh::triangle& t = tris[idTri];
    return cross((pts[t.v[1]] - pts[t.v[0]]).normalize(),
                 (pts[t.v[2]] - pts[t.v[0]]).normalize())
        .normalize();
}

Point3d Mesh::computeTriangleCenterOfGravity(int idTri) const
{
    const Mesh::triangle& t = tris[idTri];
    return (pts[t.v[0]] + pts[t.v[1]] + pts[t.v[2]]) / 3.0f;
}

double Mesh::computeTriangleMaxEdgeLength(int idTri) const
{
    const Mesh::triangle& t = tris[idTri];
    return std::max(std::max((pts[t.v[0]] - pts[t.v[1]]).size(),
                             (pts[t.v[1]] - pts[t.v[2]]).size()),
                             (pts[t.v[2]] - pts[t.v[0]]).size());
}

double Mesh::computeTriangleMinEdgeLength(int idTri) const
{
    const Mesh::triangle& t = tris[idTri];
    return std::min(std::min((pts[t.v[0]] - pts[t.v[1]]).size(),
                             (pts[t.v[1]] - pts[t.v[2]]).size()),
                             (pts[t.v[2]] - pts[t.v[0]]).size());
}

void Mesh::computeNormalsForPts(StaticVector<Point3d>& out_nms)
{
    StaticVector<StaticVector<int>> ptsNeighTris;
    getPtsNeighborTriangles(ptsNeighTris);
    computeNormalsForPts(ptsNeighTris, out_nms);
}

void Mesh::computeNormalsForPts(StaticVector<StaticVector<int>>& ptsNeighTris, StaticVector<Point3d>& out_nms)
{
    out_nms.reserve(pts.size());
    out_nms.resize_with(pts.size(), Point3d(0.0f, 0.0f, 0.0f));

    for(int i = 0; i < pts.size(); i++)
    {
        StaticVector<int>& triTmp = ptsNeighTris[i];
        if(!triTmp.empty())
        {
            Point3d n = Point3d(0.0f, 0.0f, 0.0f);
            float nn = 0.0f;
            for(int j = 0; j < triTmp.size(); j++)
            {
                Point3d n1 = computeTriangleNormal(triTmp[j]);
                n1 = n1.normalize();
                if(!std::isnan(n1.x) && !std::isnan(n1.y) && std::isnan(n1.z)) // check if is not NaN
                {
                    n = n + computeTriangleNormal(triTmp[j]);
                    nn += 1.0f;
                }
            }
            n = n / nn;

            n = n.normalize();
            if(std::isnan(n.x) || std::isnan(n.y) || std::isnan(n.z)) // check if is not NaN
            {
                n = Point3d(0.0f, 0.0f, 0.0f);
            }

            out_nms[i] = n;
        }
    }
}

void Mesh::smoothNormals(StaticVector<Point3d>& nms, StaticVector<StaticVector<int>>& ptsNeighPts)
{
    for(int i = 0; i < pts.size(); i++)
    {
        Point3d& n = nms[i];
        for(int j = 0; j < sizeOfStaticVector<int>(ptsNeighPts[i]); j++)
        {
            n = n + nms[ptsNeighPts[i][j]];
        }
        if(sizeOfStaticVector<int>(ptsNeighPts[i]) > 0)
        {
            n = n / (float)sizeOfStaticVector<int>(ptsNeighPts[i]);
        }
        n = n.normalize();
        if(std::isnan(n.x) || std::isnan(n.y) || std::isnan(n.z))
        {
            n = Point3d(0.0f, 0.0f, 0.0f);
        }
    }
}

void Mesh::removeFreePointsFromMesh(StaticVector<int>& out_ptIdToNewPtId)
{
    ALICEVISION_LOG_INFO("remove free points from mesh.");

    // declare all triangles as used
    StaticVector<int> visTris;
    visTris.reserve(tris.size());
    for(int i = 0; i < tris.size(); ++i)
    {
        visTris.push_back(i);
    }
    // generate a new mesh from all triangles so all unused points will be removed
    Mesh cleanedMesh;
    generateMeshFromTrianglesSubset(visTris, cleanedMesh, out_ptIdToNewPtId);

    std::swap(cleanedMesh.pts, pts);
    std::swap(cleanedMesh.tris, tris);
    std::swap(cleanedMesh._colors, _colors);
}

double Mesh::computeTriangleProjectionArea(const triangle_proj& tp) const
{
    // return (float)((tp.rd.x-tp.lu.x+1)*(tp.rd.y-tp.lu.y+1));

    Point2d pa = tp.tp2ds[0];
    Point2d pb = tp.tp2ds[1];
    Point2d pc = tp.tp2ds[2];
    double a = (pb - pa).size();
    double b = (pc - pa).size();
    double c = (pc - pb).size();
    double p = (a + b + c) / 2.0;

    return sqrt(p * (p - a) * (p - b) * (p - c));

    //	Point2d e1 = tp.tp2ds[1]-tp.tp2ds[0];
    //	Point2d e2 = tp.tp2ds[2]-tp.tp2ds[0];
    //	return  cross(e1,e2).size()/2.0f;
}

double Mesh::computeTriangleArea(int idTri) const
{
    Point3d pa = pts[tris[idTri].v[0]];
    Point3d pb = pts[tris[idTri].v[1]];
    Point3d pc = pts[tris[idTri].v[2]];
    double a = (pb - pa).size();
    double b = (pc - pa).size();
    double c = (pc - pb).size();
    double p = (a + b + c) / 2.0;

    return sqrt(p * (p - a) * (p - b) * (p - c));
}

void Mesh::getTrianglesEdgesIds(const StaticVector<StaticVector<int>>& edgesNeighTris, StaticVector<Voxel>& out) const
{
    out.reserve(tris.size());
    out.resize_with(tris.size(), Voxel(-1, -1, -1));

    for(int i = 0; i < edgesNeighTris.size(); i++)
    {
        for(int j = 0; j < edgesNeighTris[i].size(); j++)
        {
            int idTri = edgesNeighTris[i][j];

            if(out[idTri].x == -1)
            {
                out[idTri].x = i;
            }
            else
            {
                if(out[idTri].y == -1)
                {
                    out[idTri].y = i;
                }
                else
                {
                    if(out[idTri].z == -1)
                    {
                        out[idTri].z = i;
                    }
                    else
                    {
                        ALICEVISION_LOG_ERROR("getTrianglesEdgesIds: bad condition.");
                    }
                }
            }

        } // for j
    }     // for i

    // check ... each triangle has to have three edge ids
    for(int i = 0; i < tris.size(); i++)
    {
        if((out[i].x == -1) || (out[i].y == -1) || (out[i].z == -1))
        {
            ALICEVISION_LOG_ERROR("triangle " << i << " has to have three edge ids.");
        }
    }
}


namespace subdiv {
struct edge
{
    // local id in the triangle local coordinate system (0, 1 or 2)
    int localIdA;
    int localIdB;
    // if edge to subdivide : global id of the new point
    int new_pointId;

    edge()
    {
        localIdA = -1;
        localIdB = -1;
        new_pointId = -1;
    }

    edge(int a, int b, int newId = -1)
    {
        localIdA = a;
        localIdB = b;
        new_pointId = newId;
    }

    edge& operator=(const edge& other)
    {
        localIdA = other.localIdA;
        localIdB = other.localIdB;
        new_pointId = other.new_pointId;
        return *this;
    }

    // (A,B) = (0,1) or (1,2) or (2,0)
    void orient()
    {
        if(localIdB != (localIdA + 1) % 3)
        {
            localIdA = localIdB;
            localIdB = (localIdA + 1) % 3;
        }
    }
};


void subdivideTriangle(const Mesh& mesh, int triangleId, std::vector<edge>& edgesToSubdivide, StaticVector<Mesh::triangle>& new_tris,
                       StaticVector<Voxel>& new_trisUvIds, StaticVector<Point2d>& new_uvCoords, std::vector<int>& new_trisMtlIds)
{
    int nEdgesToSubdivide = edgesToSubdivide.size();
    // Triangle (A,B,C)
    const Mesh::triangle& triangleToSubdivide = mesh.tris[triangleId];
    int triMtlId = mesh.trisMtlIds()[triangleId];
    // PointA
    int localIdA = edgesToSubdivide[0].localIdA;
    int idA = triangleToSubdivide.v[localIdA];
    int uvIdA = mesh.trisUvIds[triangleId].m[localIdA];
    // PointB
    int localIdB = edgesToSubdivide[0].localIdB;
    int idB = triangleToSubdivide.v[localIdB];
    int uvIdB = mesh.trisUvIds[triangleId].m[localIdB];
    // PointC
    int localIdC = (localIdB + 1) % 3;
    int idC = triangleToSubdivide.v[localIdC];
    int uvIdC = mesh.trisUvIds[triangleId].m[localIdC];

    /*
                 A
                  /|
                 / |
              1 /  |
               /\  |
              /  \ |
           B /____\| C

    */

    // Subdivide into 2 new triangles
    if(nEdgesToSubdivide == 1)
    {
        // New point
        int new_id = edgesToSubdivide[0].new_pointId;
        int new_uvId = new_uvCoords.size();
        Point2d uv = (mesh.uvCoords[uvIdA] + mesh.uvCoords[uvIdB]) / 2.0f;
        new_uvCoords.push_back(uv);

        new_tris.push_back({idA, new_id, idC});
        new_trisUvIds.push_back({uvIdA, new_uvId, uvIdC});
        new_trisMtlIds.push_back(triMtlId);

        new_tris.push_back({new_id, idB, idC});
        new_trisUvIds.push_back({new_uvId, uvIdB, uvIdC});
        new_trisMtlIds.push_back(triMtlId);
    }

    /*
                B
                /|
               / |
            2 /__| 1
             /\  |
            /  \ |
         C /____\| A

    */

    // Subdivide into 3 new triangles
    else if(nEdgesToSubdivide == 2)
    {
        // New point 1
        int new_id1 = edgesToSubdivide[0].new_pointId;
        int new_uvId1 = new_uvCoords.size();
        Point2d uv1 = (mesh.uvCoords[uvIdA] + mesh.uvCoords[uvIdB]) / 2.0;
        new_uvCoords.push_back(uv1);

        // New point 2
        int new_id2 = edgesToSubdivide[1].new_pointId;
        int new_uvId2 = new_uvCoords.size();
        Point2d uv2 = (mesh.uvCoords[uvIdB] + mesh.uvCoords[uvIdC]) / 2.0;
        new_uvCoords.push_back(uv2);

        new_tris.push_back({idA, new_id1, new_id2});
        new_trisUvIds.push_back({uvIdA, new_uvId1, new_uvId2});
        new_trisMtlIds.push_back(triMtlId);

        new_tris.push_back({new_id1, idB, new_id2});
        new_trisUvIds.push_back({new_uvId1, uvIdB, new_uvId2});
        new_trisMtlIds.push_back(triMtlId);

        new_tris.push_back({new_id2, idC, idA});
        new_trisUvIds.push_back({new_uvId2, uvIdC, uvIdA});
        new_trisMtlIds.push_back(triMtlId);
    }

    /*
                 B
                  /\
                 /  \
                /    \
             1 /______\ 2
              / \    / \
             /   \  /   \
          A /_____\/_____\ C
                  3
    */

    // Subdivide into 4 new triangles
    else if(nEdgesToSubdivide == 3)
    {
        // New point 1
        int new_id1 = edgesToSubdivide[0].new_pointId;
        int new_uvId1 = new_uvCoords.size();
        Point2d uv1 = (mesh.uvCoords[uvIdA] + mesh.uvCoords[uvIdB]) / 2.0f;
        new_uvCoords.push_back(uv1);

        // New point 2
        int new_id2 = edgesToSubdivide[1].new_pointId;
        int new_uvId2 = new_uvCoords.size();
        Point2d uv2 = (mesh.uvCoords[uvIdB] + mesh.uvCoords[uvIdC]) / 2.0f;
        new_uvCoords.push_back(uv2);

        // New point 3
        int new_id3 = edgesToSubdivide[2].new_pointId;
        int new_uvId3 = new_uvCoords.size();
        Point2d uv3 = (mesh.uvCoords[uvIdC] + mesh.uvCoords[uvIdA]) / 2.0f;
        new_uvCoords.push_back(uv3);

        new_tris.push_back({idA, new_id1, new_id3});
        new_trisUvIds.push_back({uvIdA, new_uvId1, new_uvId3});
        new_trisMtlIds.push_back(triMtlId);

        new_tris.push_back({new_id1, idB, new_id2});
        new_trisUvIds.push_back({new_uvId1, uvIdB, new_uvId2});
        new_trisMtlIds.push_back(triMtlId);

        new_tris.push_back({new_id2, idC, new_id3});
        new_trisUvIds.push_back({new_uvId2, uvIdC, new_uvId3});
        new_trisMtlIds.push_back(triMtlId);

        new_tris.push_back({new_id1, new_id2, new_id3});
        new_trisUvIds.push_back({new_uvId1, new_uvId2, new_uvId3});
        new_trisMtlIds.push_back(triMtlId);
    }
}

}

int Mesh::subdivideMesh(const Mesh& refMesh, float ratioSubdiv, bool remapVisibilities)
{
    ALICEVISION_LOG_INFO("Subdivide mesh.");
    ALICEVISION_LOG_INFO("nb pts init: " << pts.size());
    ALICEVISION_LOG_INFO("nb tris init: " << tris.size());

    const int targetNbPts = refMesh.pts.size() * ratioSubdiv;
    ALICEVISION_LOG_INFO("nb points in refMesh: " << refMesh.pts.size());
    ALICEVISION_LOG_INFO("targetNbPts: " << targetNbPts);

    GEO::AdaptiveKdTree refMesh_kdTree(3);
    refMesh_kdTree.set_points(refMesh.pts.size(), refMesh.pts.front().m);

    int nbAllSubdiv = 0;
    int nsubd = 0;
    while(pts.size() < targetNbPts)
    {
        // lengthRatio value is 0.5 with a margin to ensure that we will not generate more points than the reference mesh/pointCloud
        const float lengthRatio = 0.45f;
        nsubd = subdivideMeshOnce(refMesh, refMesh_kdTree, lengthRatio);
        nbAllSubdiv += nsubd;
        ALICEVISION_LOG_DEBUG(" - subdivided: " << nsubd);
        ALICEVISION_LOG_DEBUG(" - nb pts: " << pts.size());

        // Stop iteration if we dont have enough subdivisions
        if(nsubd <= 10)
            break;
    }
    if(nbAllSubdiv == 0)
    {
        ALICEVISION_LOG_INFO("No subdivision needed.");
        return 0;
    }

    ALICEVISION_LOG_INFO("Nb points after subdivision: " << pts.size());
    ALICEVISION_LOG_INFO("Nb tris after subdivision: " << tris.size());

    if (remapVisibilities)
    {
        pointsVisibilities.resize(pts.size());
        for (int i = 0; i < pts.size(); ++i)
        {
            int iRef = refMesh_kdTree.get_nearest_neighbor(pts[i].m);
            if (iRef == -1)
                continue;

            PointVisibility& ptVisibilities = pointsVisibilities[i];
            const PointVisibility& refVisibilities = refMesh.pointsVisibilities[iRef];
            std::copy(refVisibilities.begin(), refVisibilities.end(), std::back_inserter(ptVisibilities.getDataWritable()));
        }
    }
    else
    {
        pointsVisibilities.clear();
    }
    return nbAllSubdiv;
}

int Mesh::subdivideMeshOnce(const Mesh& refMesh, const GEO::AdaptiveKdTree& refMesh_kdTree, float lengthRatio)
{
    StaticVector<StaticVector<int>> edgesNeighTris;
    StaticVector<Pixel> edgesPointsPairs;
    getNotOrientedEdges(edgesNeighTris, edgesPointsPairs);

    // for edge (A,B): <A, B, newPointId> with A,B in triangle local system (0, 1 or 2)
    // Edges to subdivise per triangle
    std::map<int, std::vector<subdiv::edge>> trianglesToSubdivide;

    // copy old pts & their uv coords
    StaticVector<Point3d> new_pts;
    StaticVector<Point2d> new_uvCoords;
    new_pts.reserve(pts.size());
    new_uvCoords.reserve(uvCoords.size());
    std::copy(pts.begin(), pts.end(), std::back_inserter(new_pts.getDataWritable()));
    std::copy(uvCoords.begin(), uvCoords.end(), std::back_inserter(new_uvCoords.getDataWritable()));

    int nEdgesToSubdivide = 0;
    // find which edges to subdivide
    for(int i = 0; i < edgesPointsPairs.size(); ++i)
    {
        int idA = edgesPointsPairs[i].x;
        int idB = edgesPointsPairs[i].y;

        double refLocalEdgeLength = 0; // rough estimation of points distances around point A and B
        {
            int j = 0;
            GEO::index_t neighborsId[8];
            double sqDist[8];
            refMesh_kdTree.get_nearest_neighbors(4, pts[idA].m, neighborsId, sqDist);
            refMesh_kdTree.get_nearest_neighbors(4, pts[idB].m, neighborsId + 4, sqDist + 4);
            if (GEO::signed_index_t(neighborsId[0]) == -1 || GEO::signed_index_t(neighborsId[4]) == -1)
                continue;

            for (int i = 1; i < 4; ++i)
            {
                if (GEO::signed_index_t(neighborsId[i]) != -1)
                {
                    refLocalEdgeLength += std::sqrt(sqDist[i]);
                    ++j;
                }
            }
            for (int i = 5; i < 8; ++i)
            {
                if (GEO::signed_index_t(neighborsId[i]) != -1)
                {
                    refLocalEdgeLength += std::sqrt(sqDist[i]);
                    ++j;
                }
            }
            refLocalEdgeLength /= j;
        }

        Point3d& pointA = pts[idA];
        Point3d& pointB = pts[idB];

        const double edgeLength = dist(pointA, pointB);
//        ALICEVISION_LOG_INFO("edge length: " << edgeLength);
//        ALICEVISION_LOG_INFO("refLocalEdgeLength: " << refLocalEdgeLength);

        if(refLocalEdgeLength > 0 && edgeLength * lengthRatio > refLocalEdgeLength)
        {
            // add new point
            Point3d newPoint = (pointA + pointB) * 0.5;
            int newPointId = new_pts.size();
            new_pts.push_back(newPoint);

            // which triangles to subdivide (= edge neighbors triangles)
            for(int triangleId : edgesNeighTris[i])
            {
                const Mesh::triangle& triangle = tris[triangleId];

                int localIdA = std::distance(triangle.v, std::find(triangle.v, triangle.v + 3, idA));
                int localIdB = std::distance(triangle.v, std::find(triangle.v, triangle.v + 3, idB));

                subdiv::edge newEdge(localIdA, localIdB, newPointId);
                newEdge.orient();

                trianglesToSubdivide[triangleId].push_back(newEdge);
            }
            nEdgesToSubdivide++;
        }
    }

    ALICEVISION_LOG_INFO("\t- # triangles to subdivide: " << trianglesToSubdivide.size());
    ALICEVISION_LOG_INFO("\t- # pts to add: " << nEdgesToSubdivide);

    new_uvCoords.reserveAdd(nEdgesToSubdivide);

    StaticVector<Mesh::triangle> new_tris;
    StaticVector<Voxel> new_trisUvIds;
    std::vector<int> new_trisMtlIds;

    std::size_t nTrianglesToSubdivide = trianglesToSubdivide.size();
    new_tris.reserve(tris.size() - nTrianglesToSubdivide + 4 * nTrianglesToSubdivide);
    new_trisUvIds.reserve(trisUvIds.size() - nTrianglesToSubdivide + 4 * nTrianglesToSubdivide);
    new_trisMtlIds.reserve(_trisMtlIds.size() - nTrianglesToSubdivide + 4 * nTrianglesToSubdivide);

    for(int triangleId = 0; triangleId < tris.size(); ++triangleId)
    {
        if(trianglesToSubdivide.find(triangleId) != trianglesToSubdivide.end())
        {
            // sort edges in ascending & adjacent order in the triangle coordinate system
            std::sort(trianglesToSubdivide[triangleId].begin(), trianglesToSubdivide[triangleId].end(), [](const subdiv::edge& a, const subdiv::edge& b) {
                return (a.localIdB == b.localIdA) ;
            });
            subdiv::subdivideTriangle(*this, triangleId, trianglesToSubdivide.at(triangleId), new_tris, new_trisUvIds, new_uvCoords, new_trisMtlIds);
        }
        else
        {
            new_tris.push_back(tris[triangleId]);

            int triMtlId = _trisMtlIds[triangleId];
            int uvIdxa = trisUvIds[triangleId].m[0];
            int uvIdxb = trisUvIds[triangleId].m[1];
            int uvIdxc = trisUvIds[triangleId].m[2];

            new_trisUvIds.push_back({uvIdxa, uvIdxb, uvIdxc});
            new_trisMtlIds.push_back(triMtlId);
        }
    }

    pts.swap(new_pts);
    tris.swap(new_tris);
    uvCoords.swap(new_uvCoords);
    trisUvIds.swap(new_trisUvIds);
    _trisMtlIds.swap(new_trisMtlIds);

    return trianglesToSubdivide.size();
}

double Mesh::computeAverageEdgeLength() const
{
    double s = 0.0;
    double n = 0.0;
    for(int i = 0; i < tris.size(); ++i)
    {
        s += computeTriangleMaxEdgeLength(i);
        n += 1.0;
    }
    if(n == 0.0)
    {
        return 0.0;
    }

    return (s / n);
}

double Mesh::computeLocalAverageEdgeLength(const std::vector<std::vector<int>>& ptsNeighbors, int ptId) const
{
    double localAverageEdgeLength = 0.0;

    const Point3d& point = pts[ptId];
    const std::vector<int>& ptNeighbors = ptsNeighbors[ptId];
    const int nbNeighbors = ptNeighbors.size();

    if(nbNeighbors == 0)
        return -1;

    for(int i = 0; i < nbNeighbors; ++i)
    {
        const Point3d& pointNeighbor = pts[ptNeighbors[i]];
        localAverageEdgeLength += dist(point, pointNeighbor);
    }
    localAverageEdgeLength /= static_cast<double>(nbNeighbors);

    return localAverageEdgeLength;
}

void Mesh::letJustTringlesIdsInMesh(StaticVector<int>& trisIdsToStay)
{
    StaticVector<Mesh::triangle> trisTmp;
    trisTmp.reserve(trisIdsToStay.size());

    for(int i = 0; i < trisIdsToStay.size(); i++)
    {
        trisTmp.push_back(tris[trisIdsToStay[i]]);
    }
    tris.swap(trisTmp);
}

void Mesh::letJustTringlesIdsInMesh(const StaticVectorBool& trisToStay)
{
    int nbTris = 0;
    for(int i = 0; i < trisToStay.size(); ++i)
        if(trisToStay[i])
            ++nbTris;

    StaticVector<Mesh::triangle> trisTmp;
    trisTmp.reserve(nbTris);

    for(int i = 0; i < trisToStay.size(); ++i)
        if(trisToStay[i])
            trisTmp.push_back(tris[i]);

    tris.swap(trisTmp);
}

void Mesh::computeTrisCams(StaticVector<StaticVector<int>>& trisCams, const mvsUtils::MultiViewParams& mp, const std::string tmpDir)
{
    if(mp.verbose)
        ALICEVISION_LOG_DEBUG("Computing tris cams.");

    StaticVector<int> ntrisCams;
    ntrisCams.reserve(tris.size());
    ntrisCams.resize_with(tris.size(), 0);

    long t1 = mvsUtils::initEstimate();
    for(int rc = 0; rc < mp.ncams; ++rc)
    {
        std::string visTrisFileName = tmpDir + "visTris" + std::to_string(mp.getViewId(rc)) + ".bin";
        StaticVector<int> visTris;
        loadArrayFromFile<int>(visTris, visTrisFileName);
        if(visTris.size() != 0)
        {
            for(int i = 0; i < visTris.size(); ++i)
            {
                int idTri = visTris[i];
                ntrisCams[idTri]++;
            }
        }
        mvsUtils::printfEstimate(rc, mp.ncams, t1);
    }
    mvsUtils::finishEstimate();

    trisCams.reserve(tris.size());

    for(int i = 0; i < tris.size(); ++i)
    {
        if(ntrisCams[i] > 0)
        {
            trisCams[i].reserve(ntrisCams[i]);
        }
    }

    t1 = mvsUtils::initEstimate();
    for(int rc = 0; rc < mp.ncams; ++rc)
    {
        std::string visTrisFileName = tmpDir + "visTris" + std::to_string(mp.getViewId(rc)) + ".bin";
        StaticVector<int> visTris;
        loadArrayFromFile<int>(visTris, visTrisFileName);
        if(visTris.size() != 0)
        {
            for(int i = 0; i < visTris.size(); ++i)
            {
                int idTri = visTris[i];
                trisCams[idTri].push_back(rc);
            }
        }
        mvsUtils::printfEstimate(rc, mp.ncams, t1);
    }
    mvsUtils::finishEstimate();
}

void Mesh::computeTrisCamsFromPtsCams(StaticVector<StaticVector<int>>& trisCams) const
{
    // TODO: try intersection
    trisCams.reserve(tris.size());

    for(int idTri = 0; idTri < tris.size(); idTri++)
    {
        const Mesh::triangle& t = tris[idTri];
        StaticVector<int> cams;

        int maxcams = sizeOfStaticVector<int>(pointsVisibilities[t.v[0]]) +
                      sizeOfStaticVector<int>(pointsVisibilities[t.v[1]]) +
                      sizeOfStaticVector<int>(pointsVisibilities[t.v[2]]);
        cams.reserve(maxcams);
        for(int k = 0; k < 3; k++)
        {
            for(int i = 0; i < sizeOfStaticVector<int>(pointsVisibilities[t.v[k]]); i++)
            {
                cams.push_back_distinct(pointsVisibilities[t.v[k]][i]);
            }
        }
        trisCams.push_back(cams);
    }
}

void Mesh::initFromDepthMap(const mvsUtils::MultiViewParams& mp, StaticVector<float>& depthMap, int rc, int scale, float alpha)
{
    initFromDepthMap(mp, &depthMap[0], rc, scale, 1, alpha);
}

void Mesh::initFromDepthMap(const mvsUtils::MultiViewParams& mp, float* depthMap, int rc, int scale, int step, float alpha)
{
    initFromDepthMap(1, mp, depthMap, rc, scale, step, alpha);
}

void Mesh::initFromDepthMap(int stepDetail, const mvsUtils::MultiViewParams& mp, float* depthMap, int rc, int scale, int step,
                               float alpha)
{
    int w = mp.getWidth(rc) / (scale * step);
    int h = mp.getHeight(rc) / (scale * step);

    pts = StaticVector<Point3d>();
    pts.reserve(w * h);
    StaticVectorBool usedMap;
    usedMap.reserve(w * h);
    for(int i = 0; i < w * h; i++)
    {
        int x = i / h;
        int y = i % h;
        float depth = depthMap[i];
        if(depth > 0.0f)
        {
            Point3d p = mp.CArr[rc] +
                        (mp.iCamArr[rc] * Point2d((float)x * (float)(scale * step), (float)y * (float)(scale * step)))
                                .normalize() * depth;
            pts.push_back(p);
            usedMap.push_back(true);
        }
        else
        {
            pts.push_back(Point3d(0.0f, 0.0f, 0.0f));
            usedMap.push_back(false);
        }
    }

    tris = StaticVector<Mesh::triangle>();
    tris.reserve(w * h * 2);
    for(int x = 0; x < w - 1 - stepDetail; x += stepDetail)
    {
        for(int y = 0; y < h - 1 - stepDetail; y += stepDetail)
        {
            Point3d p1 = pts[x * h + y];
            Point3d p2 = pts[(x + stepDetail) * h + y];
            Point3d p3 = pts[(x + stepDetail) * h + y + stepDetail];
            Point3d p4 = pts[x * h + y + stepDetail];

            if(usedMap[x * h + y] && usedMap[(x + stepDetail) * h + y] &&
               usedMap[(x + stepDetail) * h + y + stepDetail] && usedMap[x * h + y + stepDetail])
            {
                float d = mp.getCamPixelSize(p1, rc, alpha);
                if(((p1 - p2).size() < d) && ((p1 - p3).size() < d) && ((p1 - p4).size() < d) &&
                   ((p2 - p3).size() < d) && ((p3 - p4).size() < d))
                {
                    Mesh::triangle t;
                    t.alive = true;
                    t.v[2] = x * h + y;
                    t.v[1] = (x + stepDetail) * h + y;
                    t.v[0] = x * h + y + stepDetail;
                    tris.push_back(t);

                    t.alive = true;
                    t.v[2] = (x + stepDetail) * h + y;
                    t.v[1] = (x + stepDetail) * h + y + stepDetail;
                    t.v[0] = x * h + y + stepDetail;
                    tris.push_back(t);
                }
            }
        }
    }

    StaticVector<int> ptIdToNewPtId;
    removeFreePointsFromMesh(ptIdToNewPtId);
}

void Mesh::removeTrianglesInHexahedrons(StaticVector<Point3d>* hexahsToExcludeFromResultingMesh)
{
    if(hexahsToExcludeFromResultingMesh != nullptr)
    {
        ALICEVISION_LOG_INFO("Remove triangles in hexahedrons: " <<  tris.size() << " " << static_cast<int>(hexahsToExcludeFromResultingMesh->size() / 8));
        StaticVector<int> trisIdsToStay;
        trisIdsToStay.reserve(tris.size());

        long t1 = mvsUtils::initEstimate();
        for(int i = 0; i < tris.size(); i++)
        {
            int nin = 0;
            for(int k = 0; k < 3; k++)
            {
                Point3d p = pts[tris[i].v[k]];
                bool isThere = false;
                for(int j = 0; j < (int)(hexahsToExcludeFromResultingMesh->size() / 8); j++)
                {
                    if(mvsUtils::isPointInHexahedron(p, &(*hexahsToExcludeFromResultingMesh)[j * 8]))
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
                trisIdsToStay.push_back(i);
            }
            mvsUtils::printfEstimate(i, tris.size(), t1);
        }
        mvsUtils::finishEstimate();

        letJustTringlesIdsInMesh(trisIdsToStay);
    }
}

void Mesh::removeTrianglesOutsideHexahedron(Point3d* hexah)
{
    ALICEVISION_LOG_INFO("Remove triangles outside hexahedrons: " << tris.size());
    StaticVector<int> trisIdsToStay;
    trisIdsToStay.reserve(tris.size());

    long t1 = mvsUtils::initEstimate();
    for(int i = 0; i < tris.size(); i++)
    {
        int nout = 0;
        for(int k = 0; k < 3; k++)
        {
            Point3d p = pts[tris[i].v[k]];
            bool isThere = false;
            if(!mvsUtils::isPointInHexahedron(p, hexah))
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
            trisIdsToStay.push_back(i);
        }
        mvsUtils::printfEstimate(i, tris.size(), t1);
    }
    mvsUtils::finishEstimate();

    letJustTringlesIdsInMesh(trisIdsToStay);
}

void Mesh::filterLargeEdgeTriangles(double cutAverageEdgeLengthFactor, const StaticVectorBool& trisToConsider, StaticVectorBool& trisToStay) const
{
    ALICEVISION_LOG_INFO("Filtering large triangles.");

    const double averageEdgeLength = computeAverageEdgeLength();
    const double avelthr = averageEdgeLength * cutAverageEdgeLengthFactor;

    #pragma omp parallel for
    for(int i = 0; i < tris.size(); ++i)
    {
        if(trisToConsider.empty() || trisToConsider[i])
        {
          const double triMaxEdgelength = computeTriangleMaxEdgeLength(i);

          if(triMaxEdgelength >= avelthr)
              trisToStay[i] = false;
        }
    }

    ALICEVISION_LOG_INFO("Filtering large triangles, done.");
}

void Mesh::filterTrianglesByRatio(double ratio, const StaticVectorBool& trisToConsider, StaticVectorBool& trisToStay) const
{
    ALICEVISION_LOG_INFO("Filtering triangles by ratio " << ratio << ".");

    #pragma omp parallel for
    for(int i = 0; i < tris.size(); ++i)
    {
        if(trisToConsider.empty() || trisToConsider[i])
        {
          const double minEdge = computeTriangleMinEdgeLength(i);
          const double maxEdge = computeTriangleMaxEdgeLength(i);
          
          if((minEdge == 0) || ((maxEdge/minEdge) > ratio))
              trisToStay[i] = false;
        }
    }

    ALICEVISION_LOG_INFO("Filtering triangles by ratio, done.");
}

void Mesh::invertTriangleOrientations()
{
    ALICEVISION_LOG_INFO("Invert triangle orientations.");
    for(int i = 0; i < tris.size(); ++i)
    {
        Mesh::triangle& t = tris[i];
        std::swap(t.v[1], t.v[2]);
    }
}

void Mesh::changeTriPtId(int triId, int oldPtId, int newPtId)
{
    for(int k = 0; k < 3; k++)
    {
        if(oldPtId == tris[triId].v[k])
        {
            tris[triId].v[k] = newPtId;
        }
    }
}

int Mesh::getTriPtIndex(int triId, int ptId, bool failIfDoesNotExists) const
{
    for(int k = 0; k < 3; k++)
    {
        if(ptId == tris[triId].v[k])
        {
            return k;
        }
    }

    if(failIfDoesNotExists)
    {
        throw std::runtime_error("Mesh::getTriPtIndex: ptId does not exist: " + std::to_string(ptId));
    }
    return -1;
}

Pixel Mesh::getTriOtherPtsIds(int triId, int _ptId) const
{
    int others[3];
    int nothers = 0;
    for(int k = 0; k < 3; k++)
    {
        if(_ptId != tris[triId].v[k])
        {
            others[nothers] = tris[triId].v[k];
            nothers++;
        }
    }

    if(nothers != 2)
    {
        throw std::runtime_error("Mesh::getTriOtherPtsIds: pt X neighbouring tringle without pt X");
    }

    return Pixel(others[0], others[1]);
}
bool Mesh::areTwoTrisSameOriented(int triId1, int triId2, int edgePtId1, int edgePtId2) const
{
    int t1ep1Index = getTriPtIndex(triId1, edgePtId1, true);
    int t1ep2Index = getTriPtIndex(triId1, edgePtId2, true);
    int t2ep1Index = getTriPtIndex(triId2, edgePtId1, true);
    int t2ep2Index = getTriPtIndex(triId2, edgePtId2, true);
    int t1Orientation = (t1ep2Index + (3 - t1ep1Index)) % 3; // can be 1 or 2;
    int t2Orientation = (t2ep2Index + (3 - t2ep1Index)) % 3; // can be 1 or 2;

    return (t1Orientation != t2Orientation);
}

bool Mesh::isTriangleAngleAtVetexObtuse(int vertexIdInTriangle, int triId) const
{
    Point3d A = pts[tris[triId].v[(vertexIdInTriangle + 0) % 3]];
    Point3d B = pts[tris[triId].v[(vertexIdInTriangle + 1) % 3]];
    Point3d C = pts[tris[triId].v[(vertexIdInTriangle + 2) % 3]];
    return dot(B - A, C - A) < 0.0f;
}

bool Mesh::isTriangleObtuse(int triId) const
{
    return (isTriangleAngleAtVetexObtuse(0, triId)) || (isTriangleAngleAtVetexObtuse(1, triId)) ||
           (isTriangleAngleAtVetexObtuse(2, triId));
}

void Mesh::getLargestConnectedComponentTrisIds(StaticVector<int>& out) const
{
    StaticVector<StaticVector<int>> ptsNeighPtsOrdered;
    getPtsNeighPtsOrdered(ptsNeighPtsOrdered);

    StaticVector<int> colors;
    colors.reserve(pts.size());
    colors.resize_with(pts.size(), -1);

    StaticVector<int> buff;
    buff.reserve(pts.size());

    int col = 0;
    int maxNptsOfCol = -1;
    int bestCol = -1;
    for(int i = 0; i < pts.size(); ++i)
    {
        if(colors[i] != -1) // already labelled with a color id
            continue;

        buff.resize(0);
        buff.push_back(i);
        int nptsOfCol = 0;
        while(buff.size() > 0)
        {
            int ptid = buff.pop();
            if(colors[ptid] == -1)
            {
                colors[ptid] = col;
                ++nptsOfCol;
            }
            else
            {
                if(colors[ptid] != col)
                {
                    throw std::runtime_error("getLargestConnectedComponentTrisIds: bad condition.");
                }
            }
            for(int j = 0; j < sizeOfStaticVector<int>(ptsNeighPtsOrdered[ptid]); ++j)
            {
                int nptid = ptsNeighPtsOrdered[ptid][j];
                if((nptid > -1) && (colors[nptid] == -1))
                {
                    if(buff.size() >= buff.capacity()) // should not happen but no problem
                    {
                        ALICEVISION_LOG_WARNING("getLargestConnectedComponentTrisIds: bad condition.");
                        buff.reserveAdd(pts.size());
                    }
                    buff.push_back(nptid);
                }
            }
        }

        if(maxNptsOfCol < nptsOfCol)
        {
            maxNptsOfCol = nptsOfCol;
            bestCol = col;
        }
        ++col;
    }

    out.reserve(tris.size());
    for(int i = 0; i < tris.size(); i++)
    {
        if((tris[i].alive) &&
           (colors[tris[i].v[0]] == bestCol) &&
           (colors[tris[i].v[1]] == bestCol) &&
           (colors[tris[i].v[2]] == bestCol))
        {
            out.push_back(i);
        }
    }
}

bool Mesh::loadFromObjAscii(const std::string& objAsciiFileName)
{  
    ALICEVISION_LOG_INFO("Loading mesh from obj file: " << objAsciiFileName);
    // read number of points, triangles, uvcoords
    int npts = 0;
    int ntris = 0;
    int nuvs = 0;
    int nnorms = 0;
    int nlines = 0;
    bool useColors = false;

    {
        std::ifstream in(objAsciiFileName.c_str());
        std::string line;
        while(getline(in, line))
        {
            if((line[0] == 'v') && (line[1] == ' '))
            {
                if(npts == 0)
                {
                  useColors = mvsUtils::findNSubstrsInString(line, ".") == 6;
                }
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
                int n1 = mvsUtils::findNSubstrsInString(line, "/");
                int n2 = mvsUtils::findNSubstrsInString(line, "//");
                if((n1 == 3 && n2 == 0) || 
                   (n1 == 6 && n2 == 0) ||
                   (n1 == 0 && n2 == 3) ||
                   (n1 == 0 && n2 == 0))
                    ntris += 1;
                else if((n1 == 4 && n2 == 0) ||
                        (n1 == 8 && n2 == 0) ||
                        (n1 == 0 && n2 == 4))
                    ntris += 2;
            }
            nlines++;
        }
        in.close();
    }

    ALICEVISION_LOG_INFO("\t- # vertices: " << npts << std::endl
      << "\t- # normals: " << nnorms << std::endl
      << "\t- # uv coordinates: " << nuvs << std::endl
      << "\t- # triangles: " << ntris);

    pts = StaticVector<Point3d>();
    pts.reserve(npts);
    tris = StaticVector<Mesh::triangle>();
    tris.reserve(ntris);
    uvCoords.reserve(nuvs);
    trisUvIds.reserve(ntris);
    normals.reserve(nnorms);
    trisNormalsIds.reserve(ntris);
    _trisMtlIds.reserve(ntris);
    if(useColors)
    {
        _colors.reserve(npts);
    }

    std::map<std::string, int> materialCache;

    {
        int mtlId = -1;
        std::ifstream in(objAsciiFileName.c_str());
        std::string line;

        long t1 = mvsUtils::initEstimate();
        int idline = 0;
        while(getline(in, line))
        {
            if(line.size() < 3 || line[0] == '#')
            {
                // nothing to do
            }
            else if(mvsUtils::findNSubstrsInString(line, "usemtl") == 1)
            {
                char buff[5000];
                sscanf(line.c_str(), "usemtl %s", buff);
                auto it = materialCache.find(buff);
                if(it == materialCache.end())
                    materialCache.emplace(buff, ++mtlId); // new material
                else
                    mtlId = it->second;                   // already known material
            }
            else if((line[0] == 'v') && (line[1] == ' '))
            {
                Point3d pt;
                if(useColors)
                {
                    float r, g, b;
                    sscanf(line.c_str(), "v %lf %lf %lf %f %f %f", &pt.x, &pt.y, &pt.z, &r, &g, &b);
                    // convert float color data to uchar
                    _colors.emplace_back(
                      static_cast<unsigned char>(r*255.0f),
                      static_cast<unsigned char>(g*255.0f),
                      static_cast<unsigned char>(b*255.0f)
                    );
                }
                else
                {
                    sscanf(line.c_str(), "v %lf %lf %lf", &pt.x, &pt.y, &pt.z);
                }
                pts.push_back(pt);
            }
            else if((line[0] == 'v') && (line[1] == 'n') && (line[2] == ' '))
            {
                Point3d pt;
                sscanf(line.c_str(), "vn %lf %lf %lf", &pt.x, &pt.y, &pt.z);
                // printf("%f %f %f\n", pt.x, pt.y, pt.z);
                normals.push_back(pt);
            }
            else if((line[0] == 'v') && (line[1] == 't') && (line[2] == ' '))
            {
                Point2d pt;
                sscanf(line.c_str(), "vt %lf %lf", &pt.x, &pt.y);
                uvCoords.push_back(pt);
            }
            else if((line[0] == 'f') && (line[1] == ' '))
            {
                int n1 = mvsUtils::findNSubstrsInString(line, "/");
                int n2 = mvsUtils::findNSubstrsInString(line, "//");
                Voxel vertex, uvCoord, vertexNormal;
                Voxel vertex2, uvCoord2, vertexNormal2;
                bool ok = false;
                bool withNormal = false;
                bool withUV = false;
                bool withQuad = false;
                if(n2 == 0)
                {
                    if(n1 == 0)
                    {
                        sscanf(line.c_str(), "f %i %i %i", &vertex.x, &vertex.y, &vertex.z);
                        ok = true;
                    }
                    else if(n1 == 3)
                    {
                        sscanf(line.c_str(), "f %i/%i %i/%i %i/%i", &vertex.x, &uvCoord.x, &vertex.y, &uvCoord.y, &vertex.z, &uvCoord.z);
                        ok = true;
                        withUV = true;
                    }
                    else if(n1 == 6)
                    {
                        sscanf(line.c_str(), "f %i/%i/%i %i/%i/%i %i/%i/%i", &vertex.x, &uvCoord.x, &vertexNormal.x, &vertex.y, &uvCoord.y, &vertexNormal.y,
                               &vertex.z, &uvCoord.z, &vertexNormal.z);
                        ok = true;
                        withUV = true;
                        withNormal = true;
                    }
                    else if(n1 == 4)
                    {
                        sscanf(line.c_str(), "f %i/%i %i/%i %i/%i %i/%i", &vertex.x, &uvCoord.x, &vertex.y, &uvCoord.y, &vertex.z, &uvCoord.z, &vertex2.z, &uvCoord2.z);
                        vertex2.x = vertex.x; // same first point
                        uvCoord2.x = uvCoord.x;
                        vertex2.y = vertex.z; // 3rd point of the 1st triangle is the 2nd of the 2nd triangle.
                        uvCoord2.y = uvCoord.z;
                        ok = true;
                        withUV = true;
                        withQuad = true;
                    }
                    else if(n1 == 8)
                    {
                        sscanf(line.c_str(), "f %i/%i/%i %i/%i/%i %i/%i/%i %i/%i/%i",
                               &vertex.x, &uvCoord.x, &vertexNormal.x,
                               &vertex.y, &uvCoord.y, &vertexNormal.y,
                               &vertex.z, &uvCoord.z, &vertexNormal.z,
                               &vertex2.z, &uvCoord2.z, &vertexNormal2.z);
                        vertex2.x = vertex.x; // same first point
                        uvCoord2.x = uvCoord.x;
                        vertexNormal2.x = vertexNormal.x;
                        vertex2.y = vertex.z; // 3rd point of the 1st triangle is the 2nd of the 2nd triangle.
                        uvCoord2.y = uvCoord.z;
                        vertexNormal2.y = vertexNormal.z;
                        ok = true;
                        withUV = true;
                        withNormal = true;
                        withQuad = true;
                    }
                }
                else
                {
                    if(n2 == 3)
                    {
                        sscanf(line.c_str(), "f %i//%i %i//%i %i//%i", &vertex.x, &vertexNormal.x, &vertex.y, &vertexNormal.y, &vertex.z, &vertexNormal.z);
                        ok = true;
                        withNormal = true;
                    }
                    else if(n2 == 4)
                    {
                        sscanf(line.c_str(), "f %i//%i %i//%i %i//%i %i//%i",
                               &vertex.x, &vertexNormal.x,
                               &vertex.y, &vertexNormal.y,
                               &vertex.z, &vertexNormal.z,
                               &vertex2.z, &vertexNormal2.z);
                        vertex2.x = vertex.x; // same first point
                        vertexNormal2.x = vertexNormal.x;
                        vertex2.y = vertex.z; // 3rd point of the 1st triangle is the 2nd of the 2nd triangle.
                        vertexNormal2.y = vertexNormal.z;
                        ok = true;
                        withNormal = true;
                        withQuad = true;
                    }
                }
                if(!ok)
                {
                    throw std::runtime_error("Mesh: Unrecognized facet syntax while reading obj file: " + objAsciiFileName);
                }

                // 1st triangle
                {
                    triangle t;
                    t.v[0] = vertex.x - 1;
                    t.v[1] = vertex.y - 1;
                    t.v[2] = vertex.z - 1;
                    t.alive = true;
                    tris.push_back(t);
                    _trisMtlIds.push_back(mtlId);
                    if(withUV)
                    {
                        trisUvIds.push_back(uvCoord - Voxel(1, 1, 1));
                    }
                    if(withNormal)
                    {
                        trisNormalsIds.push_back(vertexNormal - Voxel(1, 1, 1));
                    }
                }

                // potential 2nd triangle
                if(withQuad)
                {
                    triangle t;
                    t.v[0] = vertex2.x - 1;
                    t.v[1] = vertex2.y - 1;
                    t.v[2] = vertex2.z - 1;
                    t.alive = true;
                    tris.push_back(t);
                    _trisMtlIds.push_back(mtlId);
                    if(withUV)
                    {
                        trisUvIds.push_back(uvCoord2 - Voxel(1, 1, 1));
                    }
                    if(withNormal)
                    {
                        trisNormalsIds.push_back(vertexNormal2 - Voxel(1, 1, 1));
                    }
                }
            }

            mvsUtils::printfEstimate(idline, nlines, t1);
            idline++;
        }
        mvsUtils::finishEstimate();

        in.close();
        nmtls = materialCache.size();
    }
    ALICEVISION_LOG_INFO("Mesh loaded: \n\t- #points: " << npts << "\n\t- # triangles: " << ntris);
    return npts != 0 && ntris != 0;
}

bool Mesh::getEdgeNeighTrisInterval(Pixel& itr, Pixel& edge, StaticVector<Voxel>& edgesXStat,
                                       StaticVector<Voxel>& edgesXYStat)
{
    int ptId1 = std::max(edge.x, edge.y);
    int ptId2 = std::min(edge.x, edge.y);
    itr = Pixel(-1, -1);

    int i1 = indexOfSortedVoxelArrByX(ptId1, edgesXStat, 0, edgesXStat.size() - 1);
    if(i1 > -1)
    {
        int i2 = indexOfSortedVoxelArrByX(ptId2, edgesXYStat, edgesXStat[i1].y, edgesXStat[i1].z);
        if(i2 > -1)
        {
            itr = Pixel(edgesXYStat[i2].y, edgesXYStat[i2].z);
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

bool Mesh::lockSurfaceBoundaries(int neighbourIterations, StaticVectorBool& out_ptsCanMove, bool invert) const
{
    using Edge = std::pair<int, int>;  // min vertex index, max vertex index
 
    // qSort lambda for Edge structure
    auto qSortCompareEdgeAsc = [] (const void* ia, const void* ib)
    {
        const Edge a = *(Edge*)ia;
        const Edge b = *(Edge*)ib;

        if(a.first < b.first)
            return -1;
        else if(a.first == b.first)
            return (a.second < b.second) ? -1 : 1;
        return 1;
    };

    ALICEVISION_LOG_INFO("Lock surface " << (invert? "inner part" : "boundaries") << ".");

    StaticVectorBool boundariesVertices(pts.size(), false);

    // Get all edges
    StaticVector<Edge> edges(tris.size() * 3);

    #pragma omp parallel for
    for(int i = 0; i < tris.size(); ++i)
    {
      const int edgeStartIndex = i * 3;
      const Mesh::triangle& t = tris[i];

      edges[edgeStartIndex] = std::make_pair(std::min(t.v[0], t.v[1]), std::max(t.v[0], t.v[1]));
      edges[edgeStartIndex + 1] = std::make_pair(std::min(t.v[1], t.v[2]), std::max(t.v[1], t.v[2]));
      edges[edgeStartIndex + 2] = std::make_pair(std::min(t.v[2], t.v[0]), std::max(t.v[2], t.v[0]));
    }

    // Sort edges by vertex indexes
    qsort(&edges[0], edges.size(), sizeof(Edge), qSortCompareEdgeAsc);

    // Count edge, set is on boundary
    int lastEdgeFirst = edges[0].first;
    int lastEdgeSecond = edges[0].second;
    int edgeCount = 0;
    bool boundary = false;

    for(int i = 0; i < edges.size(); ++i)
    {
      const Edge& edge = edges[i];

      if((edge.first == lastEdgeFirst) && (edge.second == lastEdgeSecond))
      {
         ++edgeCount;
      }
      else
      {
         if(edgeCount < 2)
         {
           boundariesVertices[lastEdgeFirst] = true;
           boundariesVertices[lastEdgeSecond] = true;
           boundary = true;
         }

         lastEdgeFirst = edge.first;
         lastEdgeSecond = edge.second;
         edgeCount = 1;
      }
    }

    // Return false if no boundary
    if(!boundary)
    {
      ALICEVISION_LOG_INFO("No vertex on surface boundary, done.");
      return false;
    }

    // Set is on boundary for neighbours
    for(int n = 0; n < neighbourIterations; ++n)
    {
        StaticVectorBool boundariesVerticesCurrent = boundariesVertices;
        
        #pragma omp parallel for
        for(int i = 0; i < edges.size(); ++i)
        {
            Edge& edge = edges[i];

            if(boundariesVertices[edge.first] && 
               boundariesVertices[edge.second]) // 2 vertices on boundary, skip
                continue;

            if(boundariesVertices[edge.first])
            {
                #pragma OMP_ATOMIC_WRITE
                boundariesVerticesCurrent[edge.second] = true;
            }

            if(boundariesVertices[edge.second])
            {
                #pragma OMP_ATOMIC_WRITE
                boundariesVerticesCurrent[edge.first] = true;
            }
        }
        std::swap(boundariesVertices, boundariesVerticesCurrent);
    }

    // Create output vectors
    if(out_ptsCanMove.empty())
       out_ptsCanMove.resize(pts.size(), true);
    
    #pragma omp parallel for
    for(int i = 0; i < boundariesVertices.size(); ++i)
    {
        if(boundariesVertices[i] == !invert)
            out_ptsCanMove[i] = false;
    }
    
    ALICEVISION_LOG_INFO("Lock surface " << (invert? "inner part" : "boundaries") << ", done.");
    return true;
}

bool Mesh::getSurfaceBoundaries(StaticVectorBool& out_trisToConsider, bool invert) const
{
    // Edge struct
    struct Edge
    {
      int first;  // min vertex index
      int second; // max vertex index
      int triId;  // triangle index

      Edge() : first(0), second(0), triId(0)
      {}

      Edge(int vertexId1, int vertexId2, int triangleId) : triId(triangleId)
      {
        first  = std::min(vertexId1, vertexId2);
        second = std::max(vertexId1, vertexId2);
      }
    };

    // qSort lambda for Edge structure
    auto qSortCompareEdgeAsc = [] (const void* ia, const void* ib)
    {
        const Edge a = *(Edge*)ia;
        const Edge b = *(Edge*)ib;

        if(a.first < b.first)
            return -1;
        else if(a.first == b.first)
            return (a.second < b.second) ? -1 : 1;
        return 1;
    };

    ALICEVISION_LOG_INFO("Get surface " << (invert? "inner part" : "boundaries") << ".");

    StaticVectorBool boundariesEdges(tris.size() * 3, false);

    // Get all edges
    StaticVector<Edge> edges(tris.size() * 3);

    #pragma omp parallel for
    for(int i = 0; i < tris.size(); ++i)
    {
      const int edgeStartIndex = i * 3;
      const Mesh::triangle& t = tris[i];

      edges[edgeStartIndex] = Edge(t.v[0], t.v[1], i);
      edges[edgeStartIndex + 1] = Edge(t.v[1], t.v[2], i);
      edges[edgeStartIndex + 2] = Edge(t.v[2], t.v[0], i);
    }

    // Sort edges by vertex indexes
    qsort(&edges[0], edges.size(), sizeof(Edge), qSortCompareEdgeAsc);

    // Count edge, set is on boundary
    int lastEdgeFirst = edges[0].first;
    int lastEdgeSecond = edges[0].second;
    int edgeCount = 0;
    bool boundary = false;

    for(int i = 0; i < edges.size(); ++i)
    {
      const Edge& edge = edges[i];

      if((edge.first == lastEdgeFirst) && (edge.second == lastEdgeSecond))
      {
         ++edgeCount;
      }
      else
      {
         if(edgeCount < 2)
         {
           boundariesEdges[i-1] = true;
           boundary = true;
         }

         lastEdgeFirst = edge.first;
         lastEdgeSecond = edge.second;
         edgeCount = 1;
      }
    }

    // Return false if no boundary
    if(!boundary)
    {
      ALICEVISION_LOG_INFO("No vertex on surface boundary, done.");
      return false;
    }

    // Create output vectors
    out_trisToConsider.resize(tris.size(), false);

    // Surface triangles
    #pragma omp parallel for
    for(int i = 0; i < edges.size(); ++i)
    {
        Edge& edge = edges[i];

        if(boundariesEdges[i] == !invert)
        {
            #pragma OMP_ATOMIC_WRITE
            out_trisToConsider[edge.triId] = true;
        }
    }

    ALICEVISION_LOG_INFO("Get surface " << (invert? "inner part" : "boundaries") << ", done.");
    return true;
}

} // namespace mesh
} // namespace aliceVision
