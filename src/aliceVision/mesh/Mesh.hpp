// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsData/Matrix3x3.hpp>
#include <aliceVision/mvsData/Point2d.hpp>
#include <aliceVision/mvsData/Point3d.hpp>
#include <aliceVision/mvsData/Rgb.hpp>
#include <aliceVision/mvsData/StaticVector.hpp>
#include <aliceVision/mvsData/Voxel.hpp>
#include <aliceVision/mvsUtils/common.hpp>

namespace aliceVision {
namespace mesh {

class Mesh
{
public:
    struct triangle
    {
        int v[3]; ///< vertex indexes
        bool alive;

        triangle()
        {
            v[0] = -1;
            v[1] = -1;
            v[2] = -1;
            alive = true;
        }

        triangle(int a, int b, int c)
        {
            v[0] = a;
            v[1] = b;
            v[2] = c;
            alive = true;
        }

        triangle& operator=(const triangle& other)
        {
            v[0] = other.v[0];
            v[1] = other.v[1];
            v[2] = other.v[2];
            alive = other.alive;
            return *this;
        }
    };

    struct triangle_proj
    {
        Point2d tp2ds[3];
        Pixel tpixs[3];
        Pixel lu, rd;

        triangle_proj& operator=(const triangle_proj param)
        {
            tp2ds[0] = param.tp2ds[0];
            tp2ds[1] = param.tp2ds[1];
            tp2ds[2] = param.tp2ds[2];
            tpixs[0] = param.tpixs[0];
            tpixs[1] = param.tpixs[1];
            tpixs[2] = param.tpixs[2];
            lu = param.lu;
            rd = param.rd;
            return *this;
        }
    };

    struct rectangle
    {
        Point2d P[4];
        Point2d lu, rd;

        rectangle(Pixel cell, int scale)
        {
            P[0].x = (float)(cell.x * scale + 0);
            P[0].y = (float)(cell.y * scale + 0);
            P[1].x = (float)(cell.x * scale + scale);
            P[1].y = (float)(cell.y * scale + 0);
            P[2].x = (float)(cell.x * scale + scale);
            P[2].y = (float)(cell.y * scale + scale);
            P[3].x = (float)(cell.x * scale + 0);
            P[3].y = (float)(cell.y * scale + scale);
            lu = P[0];
            rd = P[2];
        }

        rectangle& operator=(const rectangle param)
        {
            for(int i = 0; i < 4; i++)
            {
                P[i] = param.P[i];
            };
            lu = param.lu;
            rd = param.rd;
            return *this;
        }
    };

public:
    StaticVector<Point3d>* pts = nullptr;
    StaticVector<Mesh::triangle>* tris = nullptr;
    Matrix3x4 transformGlobal;

    Mesh();
    ~Mesh();

    void saveToObj(const std::string& filename);

    bool loadFromBin(std::string binFileName);
    void saveToBin(std::string binFileName);
    bool loadFromObjAscii(int& nmtls, StaticVector<int>& trisMtlIds, StaticVector<Point3d>& normals,
                          StaticVector<Voxel>& trisNormalsIds, StaticVector<Point2d>& uvCoords,
                          StaticVector<Voxel>& trisUvIds, std::string objAsciiFileName);

    void addMesh(Mesh* me);

    StaticVector<StaticVector<int>*>* getTrisMap(const mvsUtils::MultiViewParams* mp, int rc, int scale, int w, int h);
    StaticVector<StaticVector<int>*>* getTrisMap(StaticVector<int>* visTris, const mvsUtils::MultiViewParams* mp, int rc, int scale,
                                                 int w, int h);
    void getDepthMap(StaticVector<float>* depthMap, const mvsUtils::MultiViewParams* mp, int rc, int scale, int w, int h);
    void getDepthMap(StaticVector<float>* depthMap, StaticVector<StaticVector<int>*>* tmp, const mvsUtils::MultiViewParams* mp, int rc,
                     int scale, int w, int h);

    StaticVector<StaticVector<int>*>* getPtsNeighborTriangles();
    StaticVector<StaticVector<int>*>* getPtsNeighPtsOrdered();

    StaticVector<int>* getVisibleTrianglesIndexes(std::string tmpDir, const mvsUtils::MultiViewParams* mp, int rc, int w, int h);
    StaticVector<int>* getVisibleTrianglesIndexes(std::string depthMapFileName, std::string trisMapFileName,
                                                  const mvsUtils::MultiViewParams* mp, int rc, int w, int h);
    StaticVector<int>* getVisibleTrianglesIndexes(StaticVector<StaticVector<int>*>* trisMap,
                                                  StaticVector<float>* depthMap, const mvsUtils::MultiViewParams* mp, int rc, int w,
                                                  int h);
    StaticVector<int>* getVisibleTrianglesIndexes(StaticVector<float>* depthMap, const mvsUtils::MultiViewParams* mp, int rc, int w,
                                                  int h);

    Mesh* generateMeshFromTrianglesSubset(const StaticVector<int> &visTris, StaticVector<int>** out_ptIdToNewPtId) const;

    void getNotOrientedEdges(StaticVector<StaticVector<int>*>** edgesNeighTris, StaticVector<Pixel>** edgesPointsPairs);
    StaticVector<Voxel>* getTrianglesEdgesIds(StaticVector<StaticVector<int>*>* edgesNeighTris);

    StaticVector<Point3d>* getLaplacianSmoothingVectors(StaticVector<StaticVector<int>*>* ptsNeighPts,
                                                        double maximalNeighDist = -1.0f);
    void laplacianSmoothPts(float maximalNeighDist = -1.0f);
    void laplacianSmoothPts(StaticVector<StaticVector<int>*>* ptsNeighPts, double maximalNeighDist = -1.0f);
    StaticVector<Point3d>* computeNormalsForPts();
    StaticVector<Point3d>* computeNormalsForPts(StaticVector<StaticVector<int>*>* ptsNeighTris);
    void smoothNormals(StaticVector<Point3d>* nms, StaticVector<StaticVector<int>*>* ptsNeighPts);
    Point3d computeTriangleNormal(int idTri);
    Point3d computeTriangleCenterOfGravity(int idTri) const;
    double computeTriangleMaxEdgeLength(int idTri) const;

    void removeFreePointsFromMesh(StaticVector<int>** out_ptIdToNewPtId = nullptr);

    void letJustTringlesIdsInMesh(StaticVector<int>* trisIdsToStay);

    double computeAverageEdgeLength() const;

    bool isTriangleAngleAtVetexObtuse(int vertexIdInTriangle, int triId);
    bool isTriangleObtuse(int triId);

public:
    double computeTriangleProjectionArea(const triangle_proj& tp) const;
    double computeTriangleArea(int idTri) const;
    Mesh::triangle_proj getTriangleProjection(int triid, const mvsUtils::MultiViewParams* mp, int rc, int w, int h) const;
    bool isTriangleProjectionInImage(const Mesh::triangle_proj& tp, int width, int height, int margin) const;
    int getTriangleNbVertexInImage(const Mesh::triangle_proj& tp, int width, int height, int margin) const;
    bool doesTriangleIntersectsRectangle(Mesh::triangle_proj* tp, Mesh::rectangle* re);
    StaticVector<Point2d>* getTrianglePixelIntersectionsAndInternalPoints(Mesh::triangle_proj* tp,
                                                                          Mesh::rectangle* re);
    StaticVector<Point3d>* getTrianglePixelIntersectionsAndInternalPoints(const mvsUtils::MultiViewParams* mp, int idTri, Pixel& pix,
                                                                          int rc, Mesh::triangle_proj* tp,
                                                                          Mesh::rectangle* re);

    Point2d getTrianglePixelInternalPoint(Mesh::triangle_proj* tp, Mesh::rectangle* re);

    void subdivideMesh(const mvsUtils::MultiViewParams* mp, float maxTriArea, std::string tmpDir, int maxMeshPts);
    void subdivideMeshMaxEdgeLengthUpdatePtsCams(const mvsUtils::MultiViewParams* mp, float maxEdgeLength,
                                                 StaticVector<StaticVector<int>*>* ptsCams, int maxMeshPts);
    StaticVector<StaticVector<int>*>* subdivideMesh(const mvsUtils::MultiViewParams* mp, float maxTriArea, float maxEdgeLength,
                                                    bool useMaxTrisAreaOrAvEdgeLength,
                                                    StaticVector<StaticVector<int>*>* trisCams, int maxMeshPts);
    int subdivideMesh(const mvsUtils::MultiViewParams* mp, float maxTriArea, float maxEdgeLength, bool useMaxTrisAreaOrAvEdgeLength,
                      StaticVector<StaticVector<int>*>* trisCams, StaticVector<int>** trisCamsId);
    void subdivideMeshCase1(int i, StaticVector<Pixel>* edgesi, Pixel& neptIdEdgeId,
                            StaticVector<Mesh::triangle>* tris1);
    void subdivideMeshCase2(int i, StaticVector<Pixel>* edgesi, Pixel& neptIdEdgeId1, Pixel& neptIdEdgeId2,
                            StaticVector<Mesh::triangle>* tris1);
    void subdivideMeshCase3(int i, StaticVector<Pixel>* edgesi, Pixel& neptIdEdgeId1, Pixel& neptIdEdgeId2,
                            Pixel& neptIdEdgeId3, StaticVector<Mesh::triangle>* tris1);

    StaticVector<StaticVector<int>*>* computeTrisCams(const mvsUtils::MultiViewParams* mp, std::string tmpDir);
    StaticVector<StaticVector<int>*>* computeTrisCamsFromPtsCams(StaticVector<StaticVector<int>*>* ptsCams) const;

    void initFromDepthMap(const mvsUtils::MultiViewParams* mp, float* depthMap, int rc, int scale, int step, float alpha);
    void initFromDepthMap(const mvsUtils::MultiViewParams* mp, StaticVector<float>* depthMap, int rc, int scale, float alpha);
    void initFromDepthMap(int stepDetail, const mvsUtils::MultiViewParams* mp, float* depthMap, int rc, int scale, int step,
                          float alpha);
    void removeTrianglesInHexahedrons(StaticVector<Point3d>* hexahsToExcludeFromResultingMesh);
    void removeTrianglesOutsideHexahedron(Point3d* hexah);
    void filterLargeEdgeTriangles(double cutAverageEdgeLengthFactor);
    void invertTriangleOrientations();
    void changeTriPtId(int triId, int oldPtId, int newPtId);
    int getTriPtIndex(int triId, int ptId, bool failIfDoesNotExists = true);
    Pixel getTriOtherPtsIds(int triId, int _ptId);
    bool areTwoTrisSameOriented(int triId1, int triId2, int edgePtId1, int edgePtId2);
    StaticVector<int>* getLargestConnectedComponentTrisIds();

    bool getEdgeNeighTrisInterval(Pixel& itr, Pixel edge, StaticVector<Voxel>* edgesXStat,
                                  StaticVector<Voxel>* edgesXYStat);
};

} // namespace mesh
} // namespace aliceVision
