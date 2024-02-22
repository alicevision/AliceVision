// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/image/Rgb.hpp>
#include <aliceVision/mvsData/Point3d.hpp>
#include <aliceVision/mvsData/StaticVector.hpp>
#include <aliceVision/mvsData/Voxel.hpp>
#include <aliceVision/mvsUtils/common.hpp>
#include <aliceVision/mesh/Mesh.hpp>
#include <aliceVision/fuseCut/delaunayGraphCutTypes.hpp>
#include <aliceVision/fuseCut/Tetrahedralization.hpp>
#include <aliceVision/fuseCut/Intersections.hpp>

#include <geogram/delaunay/delaunay.h>
#include <geogram/delaunay/delaunay_3d.h>
#include <geogram/mesh/mesh.h>
#include <geogram/basic/geometry_nd.h>



#include <map>
#include <set>

namespace aliceVision {

namespace sfmData {
class SfMData;
}

namespace fuseCut {

struct FuseParams
{
    /// Max input points loaded from images
    int maxInputPoints = 50000000;
    /// Max points at the end of the depth maps fusion
    int maxPoints = 5000000;
    /// The step used to load depth values from depth maps is computed from maxInputPts. Here we define the minimal value for this step,
    /// so on small datasets we will not spend too much time at the beginning loading all depth values.
    int minStep = 2;
    /// After fusion, filter points based on their number of observations
    int minVis = 2;

    float simFactor = 15.0f;
    float angleFactor = 15.0f;
    double pixSizeMarginInitCoef = 2.0;
    double pixSizeMarginFinalCoef = 1.0;
    float voteMarginFactor = 4.0f;
    float contributeMarginFactor = 2.0f;
    float simGaussianSizeInit = 10.0f;
    float simGaussianSize = 10.0f;
    double minAngleThreshold = 0.1;
    bool refineFuse = true;
    // Weight for helper points from mask. Do not create helper points if zero.
    float maskHelperPointsWeight = 0.0;
    int maskBorderSize = 1;
};

class DelaunayGraphCut
{
  public:
    using VertexIndex = GEO::index_t;
    using CellIndex = GEO::index_t;


    /**
     * @brief  Used for debug purposes to store count about geometries intersected during fillGraph and forceTedges.
     */
    struct GeometriesCount
    {
        size_t facets = 0;
        size_t vertices = 0;
        size_t edges = 0;

        GeometriesCount& operator+=(const GeometriesCount& gc)
        {
            edges += gc.edges;
            vertices += gc.vertices;
            facets += gc.facets;
            return *this;
        }
        GeometriesCount& operator/=(const size_t v)
        {
            edges /= v;
            vertices /= v;
            facets /= v;
            return *this;
        }
    };

    mvsUtils::MultiViewParams& _mp;

    std::unique_ptr<Tetrahedralization> _tetrahedralization;
    /// 3D points coordinates
    std::vector<Point3d> _verticesCoords;
    /// Information attached to each vertex
    std::vector<GC_vertexInfo> _verticesAttr;
    /// Information attached to each cell
    std::vector<GC_cellInfo> _cellsAttr;
    /// isFull info per cell: true is full / false is empty
    std::vector<bool> _cellIsFull;

    std::vector<int> _camsVertexes;

    static const GEO::index_t NO_TETRAHEDRON = GEO::NO_CELL;

    DelaunayGraphCut(mvsUtils::MultiViewParams& mp);
    virtual ~DelaunayGraphCut();

    /**
     * @brief Retrieve the global vertex index of the localVertexIndex of the facet.
     *
     * @param f the facet
     * @return the global vertex index
     */
    inline VertexIndex getOppositeVertexIndex(const Facet& f) const { return _tetrahedralization->cell_vertex(f.cellIndex, f.localVertexIndex); }

    /**
     * @brief Retrieve the global vertex index of a vertex from a facet and an relative index
     * compared to the localVertexIndex of the facet.
     *
     * @param f the facet
     * @param i the relative index (relative to the localVertexIndex of the facet)
     * @return the global vertex index
     */
    inline VertexIndex getVertexIndex(const Facet& f, int i) const
    {
        return _tetrahedralization->cell_vertex(f.cellIndex, ((f.localVertexIndex + i + 1) % 4));
    }

    inline const std::array<const Point3d*, 3> getFacetsPoints(const Facet& f) const
    {
        return {&(_verticesCoords[getVertexIndex(f, 0)]), &(_verticesCoords[getVertexIndex(f, 1)]), &(_verticesCoords[getVertexIndex(f, 2)])};
    }

    inline std::size_t getNbVertices() const { return _verticesAttr.size(); }

    inline GEO::index_t nearestVertexInCell(GEO::index_t cellIndex, const Point3d& p) const
    {
        GEO::signed_index_t result = NO_TETRAHEDRON;
        double d = std::numeric_limits<double>::max();
        for (GEO::index_t i = 0; i < 4; ++i)
        {
            GEO::signed_index_t currentVertex = _tetrahedralization->cell_vertex(cellIndex, i);
            if (currentVertex < 0)
                continue;
            double currentDist = GEO::Geom::distance2(_verticesCoords[currentVertex].m, p.m, 3);
            if (currentDist < d)
            {
                d = currentDist;
                result = currentVertex;
            }
        }
        return result;
    }

    inline Facet mirrorFacet(const Facet& f) const
    {
        const std::array<VertexIndex, 3> facetVertices = {getVertexIndex(f, 0), getVertexIndex(f, 1), getVertexIndex(f, 2)};

        Facet out;
        out.cellIndex = _tetrahedralization->cell_adjacent(f.cellIndex, f.localVertexIndex);
        if (out.cellIndex != GEO::NO_CELL)
        {
            // Search for the vertex in adjacent cell which doesn't exist in input facet.
            for (int k = 0; k < 4; ++k)
            {
                CellIndex out_vi = _tetrahedralization->cell_vertex(out.cellIndex, k);
                if (std::find(facetVertices.begin(), facetVertices.end(), out_vi) == facetVertices.end())
                {
                    out.localVertexIndex = k;
                    return out;
                }
            }
        }
        return out;
    }

    /**
     * @brief Retrieves the global indexes of neighboring cells around a geometry.
     *
     * @param g the concerned geometry
     * @return a vector of neighboring cell indices
     */
    std::vector<CellIndex> getNeighboringCellsByGeometry(const GeometryIntersection& g) const;

    /**
     * @brief Retrieves the two global indexes of neighboring cells using a facet.
     *
     * @param f the concerned facet
     * @return a vector of neighboring cell indices
     */
    std::vector<CellIndex> getNeighboringCellsByFacet(const Facet& f) const;

    /**
     * @brief Retrieves the global indexes of neighboring cells using the global index of a vertex.
     *
     * @param vi the global vertexIndex
     * @return a vector of neighboring cell indices
     */
    inline const std::vector<CellIndex>& getNeighboringCellsByVertexIndex(VertexIndex vi) const { 
        const auto & neighboringCellsPerVertex = _tetrahedralization->getNeighboringCellsPerVertex();
        return neighboringCellsPerVertex.at(vi); 
    }

    /**
     * @brief Retrieves the global indexes of neighboring cells around one edge.
     *
     * @param e the concerned edge
     * @return a vector of neighboring cell indices
     */
    std::vector<CellIndex> getNeighboringCellsByEdge(const Edge& e) const;

    void computeDelaunay();
    void initCells();
    void displayStatistics();

    StaticVector<StaticVector<int>*>* createPtsCams();
    void createPtsCams(StaticVector<StaticVector<int>>& out_ptsCams);
    StaticVector<int>* getPtsCamsHist();
    StaticVector<int>* getPtsNrcHist();
    StaticVector<int> getIsUsedPerCamera() const;
    StaticVector<int> getSortedUsedCams() const;

    void addPointsFromSfM(const Point3d hexah[8], const StaticVector<int>& cams, const sfmData::SfMData& sfmData);
    void addPointsFromCameraCenters(const StaticVector<int>& cams, float minDist);
    void addPointsToPreventSingularities(const Point3d Voxel[8], float minDist);

    void densifyWithHelperPoints(int nbFront, int nbBack, double scale);

    /**
     * @brief Add volume points to prevent singularities
     */
    void addGridHelperPoints(int helperPointsGridSize, const Point3d Voxel[8], float minDist);

    void addMaskHelperPoints(const Point3d voxel[8], const StaticVector<int>& cams, const FuseParams& params);

    void fuseFromDepthMaps(const StaticVector<int>& cams, const Point3d voxel[8], const FuseParams& params);

    /**
     * @brief Compute connected segments size
     * @param[out] out_segments
     * @param[in] useVertex: sub-set of vertices to compute
     * @param[in] alpha
     */
    void computeVerticesSegSize(std::vector<GC_Seg>& out_segments, const std::vector<bool>& useVertex, float alpha = 0.0f);
    void removeSmallSegs(const std::vector<GC_Seg>& segments, int minSegSize);


    float distFcn(float maxDist, float dist, float distFcnHeight) const;

    inline double conj(double val) const { return val; }
    double facetMaxEdgeLength(Facet& f1) const;
    double maxEdgeLength() const;
    Point3d cellCircumScribedSphereCentre(CellIndex ci) const;
    double getFaceWeight(const Facet& f1) const;


    void fillGraph(double nPixelSizeBehind, float distFcnHeight, float fullWeight);
    
    void rayMarchingGraphEmpty(int vertexIndex,
                                int cam,
                                float weight,
                                float fullWeight,
                                double nPixelSizeBehind,
                                float distFcnHeight);

    void rayMarchingGraphFull(int vertexIndex,
                           int cam,
                           float weight,
                           float fullWeight,
                           double nPixelSizeBehind,
                           float distFcnHeight);

    /**
     * @brief Estimate the cells property "on" based on the analysis of the visibility of neigbouring cells.
     *
     * @param nPixelSizeBehind Used to define the surface margin
     */
    void forceTedgesByGradientIJCV(float nPixelSizeBehind);

    int computeIsOnSurface(std::vector<bool>& vertexIsOnSurface) const;

    void addToInfiniteSw(float sW);

    void maxflow();

    void voteFullEmptyScore(const StaticVector<int>& cams);

    void createDensePointCloud(const Point3d hexah[8],
                               const StaticVector<int>& cams,
                               const sfmData::SfMData* sfmData,
                               const FuseParams* depthMapsFuseParams);

    void createGraphCut(const Point3d hexah[8],
                        const StaticVector<int>& cams);

    /**
     * @brief Invert full/empty status of cells if they represent a too small group after labelling.
     */
    void invertFullStatusForSmallLabels();
    /**
     * @brief Check solid angle ratio between empty/full part around each vertex to reduce local artefacts / improve smoothness.
     */
    void cellsStatusFilteringBySolidAngleRatio(int nbSolidAngleFilteringIterations, double minSolidAngleRatio);

    /**
     * @brief Combine all post-processing steps results to reduce artefacts from the graph-cut (too large triangles, noisy tetrahedrons, isolated
     * cells, etc).
     */
    void graphCutPostProcessing(const Point3d hexah[8]);

    void segmentFullOrFree(bool full, StaticVector<int>& out_fullSegsColor, int& nsegments);
    int removeBubbles();
    int removeDust(int minSegSize);
    void leaveLargestFullSegmentOnly();

    /**
     * Some vertices have not been created by depth maps but
     * have been created for the tetrahedralization like
     * camera centers, helper points, etc.
     * These points have no visibility information (nrc == 0).
     * We want to remove these fake points, but we do not want to create holes.
     * So if the vertex is alone in the middle of valid points, we want to keep it.
     */
    void filterLargeHelperPoints(std::vector<bool>& out_reliableVertices, const std::vector<bool>& vertexIsOnSurface, int maxSegSize);

    /**
     * @brief Create a mesh from the tetrahedral scores
     * @param[in] maxNbConnectedHelperPoints: maximum number of connected helper points before we remove the group. 0 means that we remove all helper
     * points. -1 means that we do not filter helper points at all.
     */
    mesh::Mesh* createMesh(int maxNbConnectedHelperPoints);

    void displayCellsStats() const;
};


std::ostream& operator<<(std::ostream& stream, const DelaunayGraphCut::GeometriesCount& count);

}  // namespace fuseCut
}  // namespace aliceVision
