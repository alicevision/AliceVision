// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/structures/mv_common.hpp>

class mv_mesh
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
        point2d tp2ds[3];
        pixel tpixs[3];
        pixel lu, rd;

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
        point2d P[4];
        point2d lu, rd;

        rectangle(pixel cell, int scale)
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
    staticVector<point3d>* pts = nullptr;
    staticVector<mv_mesh::triangle>* tris = nullptr;
    matrix3x4 transformGlobal;

    mv_mesh();
    ~mv_mesh();

    bool loadFromPly(std::string plyFileName);
    void saveToPly(std::string plyFileName);
    void saveToPly(std::string plyFileName, staticVector<rgb>* triColors);

    void loadFromMesh(std::string fileName);
    void saveToOFF(std::string fileName);

    bool loadFromBin(std::string binFileName);
    bool loadFromTxt(std::string txtFileName);
    void saveToBin(std::string binFileName);
    bool loadFromObjAscii(int& nmtls, staticVector<int>** trisMtlIds, staticVector<point3d>** normals,
                          staticVector<voxel>** trisNormalsIds, staticVector<point2d>** uvCoords,
                          staticVector<voxel>** trisUvIds, std::string objAsciiFileName);

    void addMesh(mv_mesh* me);

    void updateStatMaps(staticVector<float>* depthMap, staticVector<float>* sigmaMap, pixel lu, pixel rd,
                        const multiviewParams* mp, int rc, int gridSize);

    staticVector<staticVector<int>*>* getTrisMap(const multiviewParams* mp, int rc, int scale, int w, int h);
    staticVector<staticVector<int>*>* getTrisMap(staticVector<int>* visTris, const multiviewParams* mp, int rc, int scale,
                                                 int w, int h);
    void getDepthMap(staticVector<float>* depthMap, const multiviewParams* mp, int rc, int scale, int w, int h);
    void getDepthMap(staticVector<float>* depthMap, staticVector<staticVector<int>*>* tmp, const multiviewParams* mp, int rc,
                     int scale, int w, int h);

    staticVector<staticVector<int>*>* getPtsNeighTris();
    staticVector<staticVector<int>*>* getPtsNeighPtsOrdered();
    staticVector<staticVector<int>*>* getTrisNeighTris();

    staticVector<int>* getVisibleTrianglesIndexes(std::string tmpDir, const multiviewParams* mp, int rc, int w, int h);
    staticVector<int>* getVisibleTrianglesIndexes(std::string depthMapFileName, std::string trisMapFileName,
                                                  const multiviewParams* mp, int rc, int w, int h);
    staticVector<int>* getVisibleTrianglesIndexes(staticVector<staticVector<int>*>* trisMap,
                                                  staticVector<float>* depthMap, const multiviewParams* mp, int rc, int w,
                                                  int h);
    staticVector<int>* getVisibleTrianglesIndexes(staticVector<float>* depthMap, const multiviewParams* mp, int rc, int w,
                                                  int h);

    mv_mesh* generateMeshFromTrianglesSubset(staticVector<int>* visTris, staticVector<int>** ptIdToNewPtId);

    void getNotOrientedEdges(staticVector<staticVector<int>*>** edgesNeighTris, staticVector<pixel>** edgesPointsPairs);
    staticVector<pixel>* getNotOrientedEdgesAsPointsPairs();
    staticVector<voxel>* getTrianglesEdgesIds(staticVector<staticVector<int>*>* edgesNeighTris);
    staticVector<pixel>* getNotOrientedEdgesAsTrianglesPairs();

    staticVector<point3d>* getLaplacianSmoothingVectors(staticVector<staticVector<int>*>* ptsNeighPts,
                                                        double maximalNeighDist = -1.0f);
    void laplacianSmoothPts(float maximalNeighDist = -1.0f);
    void laplacianSmoothPts(staticVector<staticVector<int>*>* ptsNeighPts, double maximalNeighDist = -1.0f);
    staticVector<point3d>* computeNormalsForPts();
    staticVector<point3d>* computeNormalsForPts(staticVector<staticVector<int>*>* ptsNeighTris);
    void smoothNormals(staticVector<point3d>* nms, staticVector<staticVector<int>*>* ptsNeighPts);
    staticVector<point3d>* computeSmoothUnitVectsForPts(staticVector<staticVector<int>*>* ptsNeighPts);
    staticVector<float>* computePtsAvNeighEdgeLength(staticVector<staticVector<int>*>* ptsNeighPts);
    point3d computeTriangleNormal(int idTri);
    point3d computeTriangleCenterOfGravity(int idTri) const;
    float computeTriangleMaxEdgeLength(int idTri) const;
    float computeTriangleAverageEdgeLength(int idTri);

    void removeFreePointsFromMesh();
    void removeFreePointsFromMesh(staticVector<int>** ptIdToNewPtId);

    void letJustTringlesIdsInMesh(staticVector<int>* trisIdsToStay);

    void cutout(const pixel& lu, const pixel& rd, int rc, const multiviewParams* mp,
                staticVector<staticVector<int>*>* ptsCams);
    void cutout(const pixel& lu, const pixel& rd, int rc, const multiviewParams* mp);

    float computeAverageEdgeLength() const;

    bool isTriangleAngleAtVetexObtuse(int vertexIdInTriangle, int triId);
    bool isTriangleObtuse(int triId);

    void filterObtuseTriangles();

public:
    float computeTriangleProjectionArea(triangle_proj& tp);
    float computeTriangleArea(int idTri);
    mv_mesh::triangle_proj getTriangleProjection(int triid, const multiviewParams* mp, int rc, int w, int h) const;
    bool isTriangleProjectionInImage(mv_mesh::triangle_proj tp, int w, int h) const;
    bool isTriangleInFrontOfCam(int triid, const multiviewParams* mp, int rc) const;
    bool isTriangleVisibleInCam(int triid, const multiviewParams* mp, int rc) const;
    bool doesTriangleIntersectsRectangle(mv_mesh::triangle_proj* tp, mv_mesh::rectangle* re);
    bool doesTriangleIntersect4Poly(point2d* _p, point2d* A, point2d* B, point2d* C, point2d* P0, point2d* P1,
                                    point2d* P2, point2d* P3);
    staticVector<point2d>* getTrianglePixelIntersectionsAndInternalPoints(mv_mesh::triangle_proj* tp,
                                                                          mv_mesh::rectangle* re);
    staticVector<point3d>* getTrianglePixelIntersectionsAndInternalPoints(const multiviewParams* mp, int idTri, pixel& pix,
                                                                          int rc, mv_mesh::triangle_proj* tp,
                                                                          mv_mesh::rectangle* re);

    point2d getTrianglePixelInternalPoint(mv_mesh::triangle_proj* tp, mv_mesh::rectangle* re);

    void subdivideMesh(const multiviewParams* mp, float maxTriArea, std::string tmpDir, int maxMeshPts);
    void subdivideMeshMaxEdgeLength(const multiviewParams* mp, float maxEdgeLenght, int maxMeshPts);
    void subdivideMeshMaxEdgeLengthUpdatePtsCams(const multiviewParams* mp, float maxEdgeLength,
                                                 staticVector<staticVector<int>*>* ptsCams, int maxMeshPts);
    staticVector<staticVector<int>*>* subdivideMesh(const multiviewParams* mp, float maxTriArea, float maxEdgeLength,
                                                    bool useMaxTrisAreaOrAvEdgeLength,
                                                    staticVector<staticVector<int>*>* trisCams, int maxMeshPts);
    int subdivideMesh(const multiviewParams* mp, float maxTriArea, float maxEdgeLength, bool useMaxTrisAreaOrAvEdgeLength,
                      staticVector<staticVector<int>*>* trisCams, staticVector<int>** trisCamsId);
    void subdivideMeshCase1(int i, staticVector<pixel>* edgesi, pixel& neptIdEdgeId,
                            staticVector<mv_mesh::triangle>* tris1);
    void subdivideMeshCase2(int i, staticVector<pixel>* edgesi, pixel& neptIdEdgeId1, pixel& neptIdEdgeId2,
                            staticVector<mv_mesh::triangle>* tris1);
    void subdivideMeshCase3(int i, staticVector<pixel>* edgesi, pixel& neptIdEdgeId1, pixel& neptIdEdgeId2,
                            pixel& neptIdEdgeId3, staticVector<mv_mesh::triangle>* tris1);

    bool isPointVisibleInRcAndTc(int idPt, staticVector<staticVector<int>*>* ptsCams, int rc, int tc,
                                 const multiviewParams* mp);
    bool isTriangleVisibleInRcAndTc(int idTri, staticVector<staticVector<int>*>* ptsCams, int rc, int tc,
                                    const multiviewParams* mp);
    void getTrianglesIndexesForRcTc(staticVector<int>** trisRcTc, staticVector<staticVector<int>*>* ptsCams, int rc,
                                    int tc, const multiviewParams* mp);

    void removeDepthMaps(const multiviewParams* mp, std::string tmpDir);
    staticVector<staticVector<int>*>* computeTrisCams(const multiviewParams* mp, std::string tmpDir);
    staticVector<staticVector<int>*>* computeTrisCamsFromPtsCams(staticVector<staticVector<int>*>* ptsCams) const;
    staticVector<staticVector<int>*>* computeCamsTris(std::string ptsCamsFileName, const multiviewParams* mp);
    staticVector<staticVector<int>*>* computePtsCams(const multiviewParams* mp, std::string tmpDir);
    staticVector<staticVector<int>*>* computePtsCamsFromTrisCams(staticVector<staticVector<int>*>* trisCams);

    void initFromDepthMapT(const multiviewParams* mp, float* depthMap, int rc, int scale, int step, float alpha);
    void initFromDepthMap(const multiviewParams* mp, float* depthMap, int rc, int scale, int step, float alpha);
    void initFromDepthMap(const multiviewParams* mp, staticVector<float>* depthMap, int rc, int scale, float alpha);
    void initFromDepthMap(int stepDetail, const multiviewParams* mp, float* depthMap, int rc, int scale, int step,
                          float alpha);
    void removeTrianglesInHexahedrons(staticVector<point3d>* hexahsToExcludeFromResultingMesh);
    void removeTrianglesOutsideHexahedron(point3d* hexah);
    staticVector<int>* getNearPointsIds(int idPt, int maxLevel, staticVector<staticVector<int>*>* ptsNeighPts);
    void filterLargeEdgeTriangles(float maxEdgelengthThr);
    void hideLargeEdgeTriangles(float maxEdgelengthThr);
    staticVector<int>* getUsedCams(const multiviewParams* mp, staticVector<staticVector<int>*>* trisCams);
    staticVector<int>* getTargetCams(int rc, const multiviewParams* mp, staticVector<staticVector<int>*>* trisCams);
    float getMaxAdjFacesAngle(int idpt, staticVector<staticVector<int>*>* ptsNeighTris,
                              staticVector<staticVector<int>*>* trisNeighTris);
    float getCurvatureAngle(int idpt, staticVector<staticVector<int>*>* ptsNeighPts);
    void invertTriangleOrientations();
    staticVector<int>* getDistinctPtsIdsForTrisIds(staticVector<int>* trisIds);
    point3d computeCentreOfGravity();
    void changeTriPtId(int triId, int oldPtId, int newPtId);
    int getTriPtIndex(int triId, int ptId, bool failIfDoesNotExists = true);
    pixel getTriOtherPtsIds(int triId, int _ptId);
    int getTriRemainingPtId(int triId, int ptId1, int ptId2);
    int getTriRemainingPtIndex(int triId, int ptId1, int ptId2);
    int getTriRemainingTriIndex(int ptIndex1, int ptIndex2);
    int getTri1EdgeIdWRTNeighTri2(int triId1, int triId2);
    bool areTwoTrisSameOriented(int triId1, int triId2, int edgePtId1, int edgePtId2);
    bool checkPtsForNaN();
    void filterWrongTriangles();
    staticVector<int>* getLargestConnectedComponentTrisIds(const multiviewParams& mp);
    staticVector<int>* getTrisIdsMapT(const multiviewParams* mp, int rc, int scale, staticVector<int>* trisIds);
    void remeshByTextureAndGetTextureInfo(int& textureSide, int& natlases, bool verbose,
                                          std::string meTextureCorrdsFileName, staticVector<point2d>** meshPtsUVO,
                                          staticVector<int>** meshPtsTexIdsO);

    bool getEdgeNeighTrisInterval(pixel& itr, pixel edge, staticVector<voxel>* edgesXStat,
                                  staticVector<voxel>* edgesXYStat);
    staticVector<voxel>* getTrisNeighsTris();
    void Transform(matrix3x3 Rs, point3d t);
};

mv_mesh* createMeshForCameras(staticVector<int>* tcams, const multiviewParams* mp, float ps, int shift, int step,
                              float upStep);
mv_mesh* createMeshForCameras4(staticVector<matrix3x4>* cams, const multiviewParams* mp, float ps);
mv_mesh* createMeshForCameras4(staticVector<int>* tcams, const multiviewParams* mp, float ps, int shift, int step,
                               float upStep);
mv_mesh* createMeshForFrontPlanePolygonOfCamera(int rc, const multiviewParams* mp, float border, point3d pivot);
mv_mesh* computeBoxMeshFromPlaneMeshAndZDimSize(mv_mesh* mePlane, float boxZsize);
