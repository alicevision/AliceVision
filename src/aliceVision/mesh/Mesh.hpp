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
#include <aliceVision/stl/bitmask.hpp>

namespace GEO {
    class AdaptiveKdTree;
}

namespace aliceVision {
namespace mesh {

using PointVisibility = StaticVector<int>;
using PointsVisibility = StaticVector<PointVisibility>;

/**
 * @brief Method to remap visibilities from the reconstruction onto an other mesh.
 */
enum EVisibilityRemappingMethod
{
    Pull = 1, //< For each vertex of the input mesh, pull the visibilities from the closest vertex in the reconstruction.
    Push = 2, //< For each vertex of the reconstruction, push the visibilities to the closest triangle in the input mesh.
    MeshItself = 4,        //< For each vertex of the mesh, test the reprojection in each camera
    PullPush = Pull | Push //< Combine results from Pull and Push results.
};

ALICEVISION_BITMASK(EVisibilityRemappingMethod);

EVisibilityRemappingMethod EVisibilityRemappingMethod_stringToEnum(const std::string& method);
std::string EVisibilityRemappingMethod_enumToString(EVisibilityRemappingMethod method);

/**
 * @brief File type available for exporting mesh
 */
enum class EFileType
{
    OBJ = 0,
    FBX,
    GLTF,
    STL
};

EFileType EFileType_stringToEnum(const std::string& filetype);
std::string EFileType_enumToString(const EFileType filetype);
std::istream& operator>>(std::istream& in, EFileType& meshFileType);
std::ostream& operator<<(std::ostream& os, EFileType meshFileType);


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

protected:
    /// Per-vertex color data
    std::vector<rgb> _colors;
    /// Per triangle material id
    std::vector<int> _trisMtlIds;

public:
    StaticVector<Point3d> pts;
    StaticVector<Mesh::triangle> tris;

    int nmtls = 0;
    StaticVector<Point2d> uvCoords;
    StaticVector<Voxel> trisUvIds;
    StaticVector<Point3d> normals;
    StaticVector<Voxel> trisNormalsIds;
    PointsVisibility pointsVisibilities;

    Mesh();
    ~Mesh();

    void save(const std::string& filepath);

    bool loadFromBin(const std::string& binFilepath);
    void saveToBin(const std::string& binFilepath);
    void load(const std::string& filepath);

    void addMesh(const Mesh& mesh);

    void getTrisMap(StaticVector<StaticVector<int>>& out, const mvsUtils::MultiViewParams& mp, int rc, int scale, int w, int h);
    void getTrisMap(StaticVector<StaticVector<int>>& out, StaticVector<int>& visTris, const mvsUtils::MultiViewParams& mp, int rc, int scale,
                    int w, int h);
    /// Per-vertex color data const accessor
    const std::vector<rgb>& colors() const { return _colors; }
    /// Per-vertex color data accessor
    std::vector<rgb>& colors() { return _colors; }

    /// Per-triangle material ids const accessor
    const std::vector<int>& trisMtlIds() const { return _trisMtlIds; }
    std::vector<int>& trisMtlIds() { return _trisMtlIds; }

    void getDepthMap(StaticVector<float>& depthMap, const mvsUtils::MultiViewParams& mp, int rc, int scale, int w, int h);
    void getDepthMap(StaticVector<float>& depthMap, StaticVector<StaticVector<int>>& tmp, const mvsUtils::MultiViewParams& mp, int rc,
                     int scale, int w, int h);

    void getPtsNeighbors(std::vector<std::vector<int>>& out_ptsNeighTris) const;
    void getPtsNeighborTriangles(StaticVector<StaticVector<int>>& out_ptsNeighTris) const;
    void getPtsNeighPtsOrdered(StaticVector<StaticVector<int>>& out_ptsNeighTris) const;

    void getVisibleTrianglesIndexes(StaticVector<int>& out_visTri, const std::string& tmpDir, const mvsUtils::MultiViewParams& mp, int rc, int w, int h);
    void getVisibleTrianglesIndexes(StaticVector<int>& out_visTri, const std::string& depthMapFilepath, const std::string& trisMapFilepath,
                                                  const mvsUtils::MultiViewParams& mp, int rc, int w, int h);
    void getVisibleTrianglesIndexes(StaticVector<int>& out_visTri, StaticVector<StaticVector<int>>& trisMap,
                                                  StaticVector<float>& depthMap, const mvsUtils::MultiViewParams& mp, int rc, int w,
                                                  int h);
    void getVisibleTrianglesIndexes(StaticVector<int>& out_visTri, StaticVector<float>& depthMap, const mvsUtils::MultiViewParams& mp, int rc, int w,
                                                  int h);

    void generateMeshFromTrianglesSubset(const StaticVector<int>& visTris, Mesh& outMesh, StaticVector<int>& out_ptIdToNewPtId) const;

    void getNotOrientedEdges(StaticVector<StaticVector<int>>& edgesNeighTris, StaticVector<Pixel>& edgesPointsPairs);
    void getTrianglesEdgesIds(const StaticVector<StaticVector<int>>& edgesNeighTris, StaticVector<Voxel>& out) const;

    void getLaplacianSmoothingVectors(StaticVector<StaticVector<int>>& ptsNeighPts, StaticVector<Point3d>& out_nms,
                                      double maximalNeighDist = -1.0f);
    void laplacianSmoothPts(float maximalNeighDist = -1.0f);
    void laplacianSmoothPts(StaticVector<StaticVector<int>>& ptsNeighPts, double maximalNeighDist = -1.0f);
    void computeNormalsForPts(StaticVector<Point3d>& out_nms);
    void computeNormalsForPts(StaticVector<StaticVector<int>>& ptsNeighTris, StaticVector<Point3d>& out_nms);
    void smoothNormals(StaticVector<Point3d>& nms, StaticVector<StaticVector<int>>& ptsNeighPts);
    Point3d computeTriangleNormal(int idTri);
    Point3d computeTriangleCenterOfGravity(int idTri) const;
    double computeTriangleMaxEdgeLength(int idTri) const;
    double computeTriangleMinEdgeLength(int idTri) const;

    void removeFreePointsFromMesh(StaticVector<int>& out_ptIdToNewPtId);

    void letJustTringlesIdsInMesh(StaticVector<int>& trisIdsToStay);
    void letJustTringlesIdsInMesh(const StaticVectorBool& trisToStay);

    double computeAverageEdgeLength() const;
    double computeLocalAverageEdgeLength(const std::vector<std::vector<int>>& ptsNeighbors, int ptId) const;

    bool isTriangleAngleAtVetexObtuse(int vertexIdInTriangle, int triId) const;
    bool isTriangleObtuse(int triId) const;

public:
    double computeTriangleProjectionArea(const triangle_proj& tp) const;
    double computeTriangleArea(int idTri) const;
    Mesh::triangle_proj getTriangleProjection(int triid, const mvsUtils::MultiViewParams& mp, int rc, int w, int h) const;
    bool isTriangleProjectionInImage(const mvsUtils::MultiViewParams& mp, const Mesh::triangle_proj& tp, int camId, int margin) const;
    int getTriangleNbVertexInImage(const mvsUtils::MultiViewParams& mp, const Mesh::triangle_proj& tp, int camId, int margin) const;
    bool doesTriangleIntersectsRectangle(Mesh::triangle_proj& tp, Mesh::rectangle& re);
    void getTrianglePixelIntersectionsAndInternalPoints(Mesh::triangle_proj& tp, Mesh::rectangle& re, StaticVector<Point2d>& out);
    void getTrianglePixelIntersectionsAndInternalPoints(const mvsUtils::MultiViewParams& mp, int idTri, Pixel& pix,
                                                              int rc, Mesh::triangle_proj& tp, Mesh::rectangle& re,
                                                              StaticVector<Point3d>& out);

    Point2d getTrianglePixelInternalPoint(Mesh::triangle_proj& tp, Mesh::rectangle& re);

    int subdivideMesh(const Mesh& refMesh, float ratioSubdiv, bool remapVisibilities);
    int subdivideMeshOnce(const Mesh& refMesh, const GEO::AdaptiveKdTree& refMesh_kdTree, float ratioSubdiv);

    void computeTrisCams(StaticVector<StaticVector<int>>& trisCams, const mvsUtils::MultiViewParams& mp, const std::string tmpDir);
    void computeTrisCamsFromPtsCams(StaticVector<StaticVector<int>>& trisCams) const;

    void initFromDepthMap(const mvsUtils::MultiViewParams& mp, float* depthMap, int rc, int scale, int step, float alpha);
    void initFromDepthMap(const mvsUtils::MultiViewParams& mp, StaticVector<float>& depthMap, int rc, int scale, float alpha);
    void initFromDepthMap(int stepDetail, const mvsUtils::MultiViewParams& mp, float* depthMap, int rc, int scale, int step,
                          float alpha);
    void removeTrianglesInHexahedrons(StaticVector<Point3d>* hexahsToExcludeFromResultingMesh);
    void removeTrianglesOutsideHexahedron(Point3d* hexah);

   /**
    * @brief Find all triangles with an edge length higher than the average in order to be removed.
    * @param[in] cutAverageEdgeLengthFactor The average edge length filtering factor.
    * @param[in] trisToConsider The input triangle group. if empty, all triangles of the mesh.
    * @param[out] trisIdsToStay For each triangle set to true if the triangle should stay.
    * @return false if no boundaries.
    */
    void filterLargeEdgeTriangles(double cutAverageEdgeLengthFactor, const StaticVectorBool& trisToConsider, StaticVectorBool& trisIdsToStay) const;

   /**
    * @brief Find all triangles with [maxEdge/minEdge > ratio] in order to be removed.
    * @param[in] ratio The filtering ratio.
    * @param[in] trisToConsider The input triangle group. if empty, all triangles of the mesh.
    * @param[out] trisIdsToStay For each triangle set to true if the triangle should stay.
    * @return false if no boundaries.
    */
    void filterTrianglesByRatio(double ratio, const StaticVectorBool& trisToConsider, StaticVectorBool& trisIdsToStay) const;

    void invertTriangleOrientations();
    void changeTriPtId(int triId, int oldPtId, int newPtId);
    int getTriPtIndex(int triId, int ptId, bool failIfDoesNotExists = true) const;
    Pixel getTriOtherPtsIds(int triId, int _ptId) const;
    bool areTwoTrisSameOriented(int triId1, int triId2, int edgePtId1, int edgePtId2) const;
    void getLargestConnectedComponentTrisIds(StaticVector<int>& out) const;

    bool getEdgeNeighTrisInterval(Pixel& itr, Pixel& edge, StaticVector<Voxel>& edgesXStat,
                                  StaticVector<Voxel>& edgesXYStat);

   /**
    * @brief Lock mesh vertices on the surface boundaries.
    * @param[in] neighbourIterations Number of boudary neighbours.
    * @param[out] out_ptsCanMove For each mesh vertices set to true if locked. Initialized if empty.
    * @param[in] invert if true lock all vertices not on the surface boundaries.
    * @return false if no boundaries.
    */
    bool lockSurfaceBoundaries(int neighbourIterations, StaticVectorBool& out_ptsCanMove, bool invert = false) const;

   /**
    * @brief Get mesh triangles on the surface boundaries.
    * @param[out] out_trisToConsider For each mesh triangle set to true if on surface.
    * @param[in] invert If true get all triangles not on the surface boundaries.
    * @return false if no boundaries.
    */
    bool getSurfaceBoundaries(StaticVectorBool& out_trisToConsider, bool invert = false) const;
    

    /**
     * @brief Remap visibilities
     *
     * @param[in] remappingMethod the remapping method
     * @param[in] refMesh the reference mesh
     * @param[in] refPointsVisibilities the reference visibilities
     */
    void remapVisibilities(EVisibilityRemappingMethod remappingMethod, const Mesh& refMesh);
};

} // namespace mesh
} // namespace aliceVision
