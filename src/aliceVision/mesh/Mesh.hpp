// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/structures/Matrix3x3.hpp>
#include <aliceVision/structures/Point2d.hpp>
#include <aliceVision/structures/Point3d.hpp>
#include <aliceVision/structures/Rgb.hpp>
#include <aliceVision/structures/StaticVector.hpp>
#include <aliceVision/structures/Voxel.hpp>
#include <aliceVision/common/common.hpp>

class Mesh
{
public:
    struct triangle
    {
        int i[3];
        bool alive;

        triangle()
        {
            i[0] = -1;
            i[1] = -1;
            i[2] = -1;
            alive = true;
        }

        triangle(int a, int b, int c)
        {
            i[0] = a;
            i[1] = b;
            i[2] = c;
            alive = true;
        }

        triangle& operator=(const triangle param)
        {
            i[0] = param.i[0];
            i[1] = param.i[1];
            i[2] = param.i[2];
            alive = param.alive;
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

    bool loadFromPly(std::string plyFileName);
    void saveToPly(std::string plyFileName);
    void saveToPly(std::string plyFileName, StaticVector<rgb>* triColors);
    void saveToObj(const std::string& filename);

    void loadFromMesh(std::string fileName);
    void saveToOFF(std::string fileName);

    bool loadFromBin(std::string binFileName);
    bool loadFromTxt(std::string txtFileName);
    void saveToBin(std::string binFileName);
    bool loadFromObjAscii(int& nmtls, StaticVector<int>** trisMtlIds, StaticVector<Point3d>** normals,
                          StaticVector<Voxel>** trisNormalsIds, StaticVector<Point2d>** uvCoords,
                          StaticVector<Voxel>** trisUvIds, std::string objAsciiFileName);

    void addMesh(Mesh* me);

    void updateStatMaps(StaticVector<float>* depthMap, StaticVector<float>* sigmaMap, Pixel lu, Pixel rd,
                        const MultiViewParams* mp, int rc, int gridSize);

    StaticVector<StaticVector<int>*>* getTrisMap(const MultiViewParams* mp, int rc, int scale, int w, int h);
    StaticVector<StaticVector<int>*>* getTrisMap(StaticVector<int>* visTris, const MultiViewParams* mp, int rc, int scale,
                                                 int w, int h);
    void getDepthMap(StaticVector<float>* depthMap, const MultiViewParams* mp, int rc, int scale, int w, int h);
    void getDepthMap(StaticVector<float>* depthMap, StaticVector<StaticVector<int>*>* tmp, const MultiViewParams* mp, int rc,
                     int scale, int w, int h);

    StaticVector<StaticVector<int>*>* getPtsNeighTris();
    StaticVector<StaticVector<int>*>* getPtsNeighPtsOrdered();
    StaticVector<StaticVector<int>*>* getTrisNeighTris();

    StaticVector<int>* getVisibleTrianglesIndexes(std::string tmpDir, const MultiViewParams* mp, int rc, int w, int h);
    StaticVector<int>* getVisibleTrianglesIndexes(std::string depthMapFileName, std::string trisMapFileName,
                                                  const MultiViewParams* mp, int rc, int w, int h);
    StaticVector<int>* getVisibleTrianglesIndexes(StaticVector<StaticVector<int>*>* trisMap,
                                                  StaticVector<float>* depthMap, const MultiViewParams* mp, int rc, int w,
                                                  int h);
    StaticVector<int>* getVisibleTrianglesIndexes(StaticVector<float>* depthMap, const MultiViewParams* mp, int rc, int w,
                                                  int h);

    Mesh* generateMeshFromTrianglesSubset(const StaticVector<int> &visTris, StaticVector<int>** out_ptIdToNewPtId) const;

    void getNotOrientedEdges(StaticVector<StaticVector<int>*>** edgesNeighTris, StaticVector<Pixel>** edgesPointsPairs);
    StaticVector<Pixel>* getNotOrientedEdgesAsPointsPairs();
    StaticVector<Voxel>* getTrianglesEdgesIds(StaticVector<StaticVector<int>*>* edgesNeighTris);
    StaticVector<Pixel>* getNotOrientedEdgesAsTrianglesPairs();

    StaticVector<Point3d>* getLaplacianSmoothingVectors(StaticVector<StaticVector<int>*>* ptsNeighPts,
                                                        double maximalNeighDist = -1.0f);
    void laplacianSmoothPts(float maximalNeighDist = -1.0f);
    void laplacianSmoothPts(StaticVector<StaticVector<int>*>* ptsNeighPts, double maximalNeighDist = -1.0f);
    StaticVector<Point3d>* computeNormalsForPts();
    StaticVector<Point3d>* computeNormalsForPts(StaticVector<StaticVector<int>*>* ptsNeighTris);
    void smoothNormals(StaticVector<Point3d>* nms, StaticVector<StaticVector<int>*>* ptsNeighPts);
    StaticVector<Point3d>* computeSmoothUnitVectsForPts(StaticVector<StaticVector<int>*>* ptsNeighPts);
    StaticVector<float>* computePtsAvNeighEdgeLength(StaticVector<StaticVector<int>*>* ptsNeighPts);
    Point3d computeTriangleNormal(int idTri);
    Point3d computeTriangleCenterOfGravity(int idTri) const;
    float computeTriangleMaxEdgeLength(int idTri) const;
    float computeTriangleAverageEdgeLength(int idTri);

    void removeFreePointsFromMesh(StaticVector<int>** out_ptIdToNewPtId = nullptr);

    void letJustTringlesIdsInMesh(StaticVector<int>* trisIdsToStay);

    void cutout(const Pixel& lu, const Pixel& rd, int rc, const MultiViewParams* mp,
                StaticVector<StaticVector<int>*>* ptsCams);
    void cutout(const Pixel& lu, const Pixel& rd, int rc, const MultiViewParams* mp);

    float computeAverageEdgeLength() const;

    bool isTriangleAngleAtVetexObtuse(int vertexIdInTriangle, int triId);
    bool isTriangleObtuse(int triId);

    void filterObtuseTriangles();

public:
    float computeTriangleProjectionArea(triangle_proj& tp);
    float computeTriangleArea(int idTri);
    Mesh::triangle_proj getTriangleProjection(int triid, const MultiViewParams* mp, int rc, int w, int h) const;
    bool isTriangleProjectionInImage(Mesh::triangle_proj tp, int w, int h) const;
    bool isTriangleInFrontOfCam(int triid, const MultiViewParams* mp, int rc) const;
    bool isTriangleVisibleInCam(int triid, const MultiViewParams* mp, int rc) const;
    bool doesTriangleIntersectsRectangle(Mesh::triangle_proj* tp, Mesh::rectangle* re);
    bool doesTriangleIntersect4Poly(Point2d* _p, Point2d* A, Point2d* B, Point2d* C, Point2d* P0, Point2d* P1,
                                    Point2d* P2, Point2d* P3);
    StaticVector<Point2d>* getTrianglePixelIntersectionsAndInternalPoints(Mesh::triangle_proj* tp,
                                                                          Mesh::rectangle* re);
    StaticVector<Point3d>* getTrianglePixelIntersectionsAndInternalPoints(const MultiViewParams* mp, int idTri, Pixel& pix,
                                                                          int rc, Mesh::triangle_proj* tp,
                                                                          Mesh::rectangle* re);

    Point2d getTrianglePixelInternalPoint(Mesh::triangle_proj* tp, Mesh::rectangle* re);

    void subdivideMesh(const MultiViewParams* mp, float maxTriArea, std::string tmpDir, int maxMeshPts);
    void subdivideMeshMaxEdgeLength(const MultiViewParams* mp, float maxEdgeLenght, int maxMeshPts);
    void subdivideMeshMaxEdgeLengthUpdatePtsCams(const MultiViewParams* mp, float maxEdgeLength,
                                                 StaticVector<StaticVector<int>*>* ptsCams, int maxMeshPts);
    StaticVector<StaticVector<int>*>* subdivideMesh(const MultiViewParams* mp, float maxTriArea, float maxEdgeLength,
                                                    bool useMaxTrisAreaOrAvEdgeLength,
                                                    StaticVector<StaticVector<int>*>* trisCams, int maxMeshPts);
    int subdivideMesh(const MultiViewParams* mp, float maxTriArea, float maxEdgeLength, bool useMaxTrisAreaOrAvEdgeLength,
                      StaticVector<StaticVector<int>*>* trisCams, StaticVector<int>** trisCamsId);
    void subdivideMeshCase1(int i, StaticVector<Pixel>* edgesi, Pixel& neptIdEdgeId,
                            StaticVector<Mesh::triangle>* tris1);
    void subdivideMeshCase2(int i, StaticVector<Pixel>* edgesi, Pixel& neptIdEdgeId1, Pixel& neptIdEdgeId2,
                            StaticVector<Mesh::triangle>* tris1);
    void subdivideMeshCase3(int i, StaticVector<Pixel>* edgesi, Pixel& neptIdEdgeId1, Pixel& neptIdEdgeId2,
                            Pixel& neptIdEdgeId3, StaticVector<Mesh::triangle>* tris1);

    bool isPointVisibleInRcAndTc(int idPt, StaticVector<StaticVector<int>*>* ptsCams, int rc, int tc,
                                 const MultiViewParams* mp);
    bool isTriangleVisibleInRcAndTc(int idTri, StaticVector<StaticVector<int>*>* ptsCams, int rc, int tc,
                                    const MultiViewParams* mp);
    void getTrianglesIndexesForRcTc(StaticVector<int>** trisRcTc, StaticVector<StaticVector<int>*>* ptsCams, int rc,
                                    int tc, const MultiViewParams* mp);

    void removeDepthMaps(const MultiViewParams* mp, std::string tmpDir);
    StaticVector<StaticVector<int>*>* computeTrisCams(const MultiViewParams* mp, std::string tmpDir);
    StaticVector<StaticVector<int>*>* computeTrisCamsFromPtsCams(StaticVector<StaticVector<int>*>* ptsCams) const;
    StaticVector<StaticVector<int>*>* computeCamsTris(std::string ptsCamsFileName, const MultiViewParams* mp);
    StaticVector<StaticVector<int>*>* computePtsCams(const MultiViewParams* mp, std::string tmpDir);
    StaticVector<StaticVector<int>*>* computePtsCamsFromTrisCams(StaticVector<StaticVector<int>*>* trisCams);

    void initFromDepthMapT(const MultiViewParams* mp, float* depthMap, int rc, int scale, int step, float alpha);
    void initFromDepthMap(const MultiViewParams* mp, float* depthMap, int rc, int scale, int step, float alpha);
    void initFromDepthMap(const MultiViewParams* mp, StaticVector<float>* depthMap, int rc, int scale, float alpha);
    void initFromDepthMap(int stepDetail, const MultiViewParams* mp, float* depthMap, int rc, int scale, int step,
                          float alpha);
    void removeTrianglesInHexahedrons(StaticVector<Point3d>* hexahsToExcludeFromResultingMesh);
    void removeTrianglesOutsideHexahedron(Point3d* hexah);
    StaticVector<int>* getNearPointsIds(int idPt, int maxLevel, StaticVector<StaticVector<int>*>* ptsNeighPts);
    void filterLargeEdgeTriangles(float maxEdgelengthThr);
    void hideLargeEdgeTriangles(float maxEdgelengthThr);
    StaticVector<int>* getUsedCams(const MultiViewParams* mp, StaticVector<StaticVector<int>*>* trisCams);
    StaticVector<int>* getTargetCams(int rc, const MultiViewParams* mp, StaticVector<StaticVector<int>*>* trisCams);
    float getMaxAdjFacesAngle(int idpt, StaticVector<StaticVector<int>*>* ptsNeighTris,
                              StaticVector<StaticVector<int>*>* trisNeighTris);
    float getCurvatureAngle(int idpt, StaticVector<StaticVector<int>*>* ptsNeighPts);
    void invertTriangleOrientations();
    StaticVector<int>* getDistinctPtsIdsForTrisIds(StaticVector<int>* trisIds);
    Point3d computeCentreOfGravity();
    void changeTriPtId(int triId, int oldPtId, int newPtId);
    int getTriPtIndex(int triId, int ptId, bool failIfDoesNotExists = true);
    Pixel getTriOtherPtsIds(int triId, int _ptId);
    int getTriRemainingPtId(int triId, int ptId1, int ptId2);
    int getTriRemainingPtIndex(int triId, int ptId1, int ptId2);
    int getTriRemainingTriIndex(int ptIndex1, int ptIndex2);
    int getTri1EdgeIdWRTNeighTri2(int triId1, int triId2);
    bool areTwoTrisSameOriented(int triId1, int triId2, int edgePtId1, int edgePtId2);
    bool checkPtsForNaN();
    void filterWrongTriangles();
    StaticVector<int>* getLargestConnectedComponentTrisIds(const MultiViewParams& mp);
    StaticVector<int>* getTrisIdsMapT(const MultiViewParams* mp, int rc, int scale, StaticVector<int>* trisIds);
    void remeshByTextureAndGetTextureInfo(int& textureSide, int& natlases, bool verbose,
                                          std::string meTextureCorrdsFileName, StaticVector<Point2d>** meshPtsUVO,
                                          StaticVector<int>** meshPtsTexIdsO);

    bool getEdgeNeighTrisInterval(Pixel& itr, Pixel edge, StaticVector<Voxel>* edgesXStat,
                                  StaticVector<Voxel>* edgesXYStat);
    StaticVector<Voxel>* getTrisNeighsTris();
    void Transform(Matrix3x3 Rs, Point3d t);
};

Mesh* createMeshForFrontPlanePolygonOfCamera(int rc, const MultiViewParams* mp, float border, Point3d pivot);
Mesh* computeBoxMeshFromPlaneMeshAndZDimSize(Mesh* mePlane, float boxZsize);
