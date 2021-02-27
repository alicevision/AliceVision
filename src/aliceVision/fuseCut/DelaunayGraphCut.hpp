// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/mvsData/Point3d.hpp>
#include <aliceVision/mvsData/Rgb.hpp>
#include <aliceVision/mvsData/StaticVector.hpp>
#include <aliceVision/mvsData/Voxel.hpp>
#include <aliceVision/mvsUtils/common.hpp>
#include <aliceVision/mesh/Mesh.hpp>
#include <aliceVision/fuseCut/delaunayGraphCutTypes.hpp>
#include <aliceVision/fuseCut/VoxelsGrid.hpp>

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

    struct Facet
    {
        CellIndex cellIndex = GEO::NO_CELL;
        /// local opposite vertex index
        VertexIndex localVertexIndex = GEO::NO_VERTEX;

        Facet(){}
        Facet(CellIndex ci, VertexIndex lvi)
            : cellIndex(ci)
            , localVertexIndex(lvi)
        {}

        bool operator==(const Facet& f) const
        {
            return cellIndex == f.cellIndex && localVertexIndex == f.localVertexIndex;
        }
    };

    struct Edge
    {
        VertexIndex v0 = GEO::NO_VERTEX;
        VertexIndex v1 = GEO::NO_VERTEX;

        Edge() = default;
        Edge(VertexIndex v0_, VertexIndex v1_)
            : v0{v0_}
            , v1{v1_}
        {}

        bool operator==(const Edge& e) const
        {
            return v0 == e.v0 && v1 == e.v1;
        }
        bool isSameUndirectionalEdge(const Edge& e) const
        {
            return (v0 == e.v0 && v1 == e.v1) ||
                   (v0 == e.v1 && v1 == e.v0);
        }
    };

    enum class EGeometryType
    {
        Vertex,
        Edge,
        Facet,
        None
    };

    struct GeometryIntersection
    {
        EGeometryType type = EGeometryType::None;
        union
        {
            Facet facet;
            VertexIndex vertexIndex;
            Edge edge;
        };
        GeometryIntersection() {}
        explicit GeometryIntersection(const Facet& f)
            : facet{f}
            , type{EGeometryType::Facet}
        {}
        explicit GeometryIntersection(const VertexIndex& v)
            : vertexIndex{v}
            , type{EGeometryType::Vertex}
        {}
        explicit GeometryIntersection(const Edge& e)
            : edge{e}
            , type{EGeometryType::Edge}
        {}

        bool operator==(const GeometryIntersection& g) const
        {
            if (type != g.type)
                return false;

            switch (type)
            {
            case EGeometryType::Vertex:
                return vertexIndex == g.vertexIndex;
            case EGeometryType::Edge:
                return edge == g.edge;
            case EGeometryType::Facet:
                return facet == g.facet;
            case EGeometryType::None:
                break;
            }
            return true;
        }
        bool operator!=(const GeometryIntersection& g) const
        {
            return !(*this == g);
        }
    };

    struct IntersectionHistory
    {
        size_t steps = 0;
        Point3d cam;
        Point3d originPt;
        Point3d dirVect;

        std::vector<GeometryIntersection> geometries;
        std::vector<Point3d> intersectPts;
        std::vector<Point3d> vecToCam;
        std::vector<float> distToCam;
        std::vector<float> angleToCam;
        IntersectionHistory(const Point3d& c, const Point3d& oPt, const Point3d& diV) : cam{c}, originPt{oPt}, dirVect{diV} {}
        
        void append(const GeometryIntersection& geom, const Point3d& intersectPt);
    };

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

    GEO::Delaunay_var _tetrahedralization;
    /// 3D points coordinates
    std::vector<Point3d> _verticesCoords;
    /// Information attached to each vertex
    std::vector<GC_vertexInfo> _verticesAttr;
    /// Information attached to each cell
    std::vector<GC_cellInfo> _cellsAttr;
    /// isFull info per cell: true is full / false is empty
    std::vector<bool> _cellIsFull;

    std::vector<int> _camsVertexes;
    std::vector<std::vector<CellIndex>> _neighboringCellsPerVertex;

    bool saveTemporaryBinFiles;

    static const GEO::index_t NO_TETRAHEDRON = GEO::NO_CELL;

    DelaunayGraphCut(mvsUtils::MultiViewParams& mp);
    virtual ~DelaunayGraphCut();

    /**
     * @brief Retrieve the global vertex index of the localVertexIndex of the facet.
     * 
     * @param f the facet
     * @return the global vertex index
     */
    inline VertexIndex getOppositeVertexIndex(const Facet& f) const
    {
        return _tetrahedralization->cell_vertex(f.cellIndex, f.localVertexIndex);
    }

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
        return {&(_verticesCoords[getVertexIndex(f, 0)]),
                &(_verticesCoords[getVertexIndex(f, 1)]),
                &(_verticesCoords[getVertexIndex(f, 2)])};
    }

    inline std::size_t getNbVertices() const
    {
        return _verticesAttr.size();
    }

    inline GEO::index_t nearestVertexInCell(GEO::index_t cellIndex, const Point3d& p) const
    {
        GEO::signed_index_t result = NO_TETRAHEDRON;
        double d = std::numeric_limits<double>::max();
        for(GEO::index_t i = 0; i < 4; ++i)
        {
            GEO::signed_index_t currentVertex = _tetrahedralization->cell_vertex(cellIndex, i);
            if(currentVertex < 0)
                continue;
            double currentDist = GEO::Geom::distance2(_verticesCoords[currentVertex].m, p.m, 3);
            if(currentDist < d)
            {
                d = currentDist;
                result = currentVertex;
            }
        }
        return result;
    }

    inline GEO::index_t locateNearestVertex(const Point3d& p) const
    {
        if(_tetrahedralization->nb_vertices() == 0)
            return GEO::NO_VERTEX;
        /*
        GEO::index_t cellIndex = ((const GEO::Delaunay3d*)_tetrahedralization.get())->locate(p.m); // TODO GEOGRAM: how to??
        if(cellIndex == NO_TETRAHEDRON)
            return GEO::NO_VERTEX;

        return nearestVertexInCell(cellIndex, p);
        */
        return _tetrahedralization->nearest_vertex(p.m); // TODO GEOGRAM: this is a brute force approach!
    }

    /**
     * @brief A cell is infinite if one of its vertices is infinite.
     */
    inline bool isInfiniteCell(CellIndex ci) const
    {
        return _tetrahedralization->cell_is_infinite(ci);
        // return ci < 0 || ci > getNbVertices();
    }
    inline bool isInvalidOrInfiniteCell(CellIndex ci) const
    {
        return ci == GEO::NO_CELL || isInfiniteCell(ci);
        // return ci < 0 || ci > getNbVertices();
    }

    inline Facet mirrorFacet(const Facet& f) const
    {
        const std::array<VertexIndex, 3> facetVertices = {
            getVertexIndex(f, 0),
            getVertexIndex(f, 1),
            getVertexIndex(f, 2)
        };
        
        Facet out;
        out.cellIndex = _tetrahedralization->cell_adjacent(f.cellIndex, f.localVertexIndex);
        if(out.cellIndex != GEO::NO_CELL)
        {
            // Search for the vertex in adjacent cell which doesn't exist in input facet.
            for(int k = 0; k < 4; ++k)
            {
                CellIndex out_vi = _tetrahedralization->cell_vertex(out.cellIndex, k);
                if(std::find(facetVertices.begin(), facetVertices.end(), out_vi) == facetVertices.end())
                {
                  out.localVertexIndex = k;
                  return out;
                }
            }
        }
        return out;
    }

    void updateVertexToCellsCache()
    {
        _neighboringCellsPerVertex.clear();

        std::map<VertexIndex, std::set<CellIndex>> neighboringCellsPerVertexTmp;
        int coutInvalidVertices = 0;
        for (CellIndex ci = 0; ci < _tetrahedralization->nb_cells(); ++ci)
        {
            for(VertexIndex k = 0; k < 4; ++k)
            {
                const VertexIndex vi = _tetrahedralization->cell_vertex(ci, k);
                if(vi == GEO::NO_VERTEX || vi >= _verticesCoords.size())
                {
                    ++coutInvalidVertices;
                    continue;
                }
                neighboringCellsPerVertexTmp[vi].insert(ci);
            }
        }
        ALICEVISION_LOG_INFO("coutInvalidVertices: " << coutInvalidVertices);
        ALICEVISION_LOG_INFO("neighboringCellsPerVertexTmp: " << neighboringCellsPerVertexTmp.size());
        _neighboringCellsPerVertex.resize(_verticesCoords.size());
        ALICEVISION_LOG_INFO("verticesCoords: " << _verticesCoords.size());
        for(const auto& it: neighboringCellsPerVertexTmp)
        {
            const std::set<CellIndex>& input = it.second;
            std::vector<CellIndex>& output = _neighboringCellsPerVertex[it.first];
            output.assign(input.begin(), input.end());
        }
    }

    /**
     * @brief vertexToCells
     *
     * It is a replacement for GEO::Delaunay::next_around_vertex which doesn't work as expected.
     *
     * @param vi
     * @param lvi
     * @return global index of the lvi'th neighboring cell
     */
    inline CellIndex vertexToCells(VertexIndex vi, int lvi) const
    {
        const std::vector<CellIndex>& localCells = _neighboringCellsPerVertex.at(vi);
        if(lvi >= localCells.size())
            return GEO::NO_CELL;
        return localCells[lvi];
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
    inline const std::vector<CellIndex>& getNeighboringCellsByVertexIndex(VertexIndex vi) const
    {
        return _neighboringCellsPerVertex.at(vi);
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

    void saveDhInfo(const std::string& fileNameInfo);
    void saveDh(const std::string& fileNameDh, const std::string& fileNameInfo);

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

    /**
     * @brief Function that returns the next geometry intersected by the ray.
     * The function handles different cases, whether we come from an edge, a facet or a vertex.
     * 
     * @param inGeometry the geometry we come from
     * @param originPt ray origin point
     * @param dirVect ray direction
     * @param intersectPt a reference that will store the computed intersection point for the intersected geometry
     * @param epsilonFactor a multiplicative factor on the smaller side of the facet  used to define the boundary when we
     * have to consider either a collision with an edge/vertex or a facet.
     * @param lastIntersectPt constant reference to the last intersection point used to test the direction.
     * @return 
     */
    GeometryIntersection intersectNextGeom(const GeometryIntersection& inGeometry, const Point3d& originPt,
        const Point3d& dirVect, Point3d& intersectPt, const double epsilonFactor, const Point3d& lastIntersectPt) const;

    /**
     * @brief Function that returns the next geometry intersected by the ray on a given facet or None if there are no intersected geometry.
     * The function distinguishes the intersections cases using epsilon.
     * 
     * @param originPt ray origin point
     * @param DirVec ray direction
     * @param facet the given facet to intersect with
     * @param intersectPt a reference that will store the computed intersection point for the next intersecting geometry
     * @param epsilonFactor a multiplicative factor on the smaller side of the facet  used to define the boundary when we
     * have to consider either a collision with an edge/vertex or a facet.
     * @param ambiguous boolean used to know if our intersection is ambiguous or not
     * @param lastIntersectPt pointer to the last intersection point used to test the direction (if not nulllptr)
     * @return 
     */
    GeometryIntersection rayIntersectTriangle(const Point3d& originPt, const Point3d& DirVec, const Facet& facet,
        Point3d& intersectPt, const double epsilonFactor, bool& ambiguous, const Point3d* lastIntersectPt = nullptr) const;

    float distFcn(float maxDist, float dist, float distFcnHeight) const;

    inline double conj(double val) const { return val; }
    double facetMaxEdgeLength(Facet& f1) const;
    double maxEdgeLength() const;
    Point3d cellCircumScribedSphereCentre(CellIndex ci) const;
    double getFaceWeight(const Facet &f1) const;

    float weightFcn(float nrc, bool labatutWeights, int ncams);

    void fillGraph(double nPixelSizeBehind, bool labatutWeights, bool fillOut, float distFcnHeight,
                           float fullWeight);
    void fillGraphPartPtRc(int& out_nstepsFront, int& out_nstepsBehind, GeometriesCount& outFrontCount, GeometriesCount& outBehindCount, int vertexIndex, int cam, float weight,
                           float fullWeight, double nPixelSizeBehind, bool fillOut, float distFcnHeight);

    /**
     * @brief Estimate the cells property "on" based on the analysis of the visibility of neigbouring cells.
     *
     * @param nPixelSizeBehind Used to define the surface margin
     */
    void forceTedgesByGradientIJCV(float nPixelSizeBehind);

    int computeIsOnSurface(std::vector<bool>& vertexIsOnSurface) const;

    void addToInfiniteSw(float sW);

    void maxflow();

    void voteFullEmptyScore(const StaticVector<int>& cams, const std::string& folderName);

    void createDensePointCloud(const Point3d hexah[8], const StaticVector<int>& cams, const sfmData::SfMData* sfmData, const FuseParams* depthMapsFuseParams);

    void createGraphCut(const Point3d hexah[8], const StaticVector<int>& cams, const std::string& folderName,
                        const std::string& tmpCamsPtsFolderName, bool removeSmallSegments, bool exportDebugTetrahedralization);

    /**
     * @brief Invert full/empty status of cells if they represent a too small group after labelling.
     */
    void invertFullStatusForSmallLabels();
    /**
     * @brief Check solid angle ratio between empty/full part around each vertex to reduce local artefacts / improve smoothness.
     */
    void cellsStatusFilteringBySolidAngleRatio(int nbSolidAngleFilteringIterations, double minSolidAngleRatio);

    /**
     * @brief Combine all post-processing steps results to reduce artefacts from the graph-cut (too large triangles, noisy tetrahedrons, isolated cells, etc).
     */
    void graphCutPostProcessing(const Point3d hexah[8], const std::string& folderName);

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
     * @param[in] maxNbConnectedHelperPoints: maximum number of connected helper points before we remove the group. 0 means that we remove all helper points. -1 means that we do not filter helper points at all.
     */
    mesh::Mesh* createMesh(int maxNbConnectedHelperPoints);
    mesh::Mesh* createTetrahedralMesh(bool filter = true, const float& downscaleFactor = 0.95f, const std::function<float(const GC_cellInfo&)> getScore = [](const GC_cellInfo& c) { return c.emptinessScore; }) const;

    void displayCellsStats() const;
    void exportDebugMesh(const std::string& filename, const Point3d& fromPt, const Point3d& toPt);
    void exportFullScoreMeshs(const std::string& outputFolder, const std::string& name) const;

    void exportBackPropagationMesh(const std::string& filename, std::vector<GeometryIntersection>& intersectedGeom, const Point3d& fromPt, const Point3d& toPt);
    void writeScoreInCsv(const std::string& filePath, const size_t& sizeLimit = 1000);
};


std::ostream& operator<<(std::ostream& stream, const DelaunayGraphCut::EGeometryType type);
std::ostream& operator<<(std::ostream& stream, const DelaunayGraphCut::Facet& facet);
std::ostream& operator<<(std::ostream& stream, const DelaunayGraphCut::Edge& edge);
std::ostream& operator<<(std::ostream& stream, const DelaunayGraphCut::GeometryIntersection& intersection);
std::ostream& operator<<(std::ostream& stream, const DelaunayGraphCut::GeometriesCount& count);

} // namespace fuseCut
} // namespace aliceVision
