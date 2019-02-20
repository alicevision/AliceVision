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
};


class DelaunayGraphCut
{
public:
    using VertexIndex = GEO::index_t;
    using CellIndex = GEO::index_t;

    struct Facet
    {
        Facet(){}
        Facet(CellIndex ci, VertexIndex lvi)
            : cellIndex(ci)
            , localVertexIndex(lvi)
        {}

        CellIndex cellIndex = GEO::NO_CELL;
        /// local opposite vertex index
        VertexIndex localVertexIndex = GEO::NO_VERTEX;
    };

    mvsUtils::MultiViewParams* mp;

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

    DelaunayGraphCut(mvsUtils::MultiViewParams* _mp);
    virtual ~DelaunayGraphCut();

    /// Get absolute opposite vertex index
    inline VertexIndex getOppositeVertexIndex(const Facet& f) const
    {
        return _tetrahedralization->cell_vertex(f.cellIndex, f.localVertexIndex);
    }
    /// Get absolute vertex index
    inline VertexIndex getVertexIndex(const Facet& f, int i) const
    {
        return _tetrahedralization->cell_vertex(f.cellIndex, ((f.localVertexIndex + i + 1) % 4));
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
        for(CellIndex ci = 0, nbCells = _tetrahedralization->nb_cells(); ci < nbCells; ++ci)
        {
            for(VertexIndex k = 0; k < 4; ++k)
            {
                CellIndex vi = _tetrahedralization->cell_vertex(ci, k);
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
    CellIndex vertexToCells(VertexIndex vi, int lvi) const
    {
        const std::vector<CellIndex>& localCells = _neighboringCellsPerVertex.at(vi);
        if(lvi >= localCells.size())
            return GEO::NO_CELL;
        return localCells[lvi];
    }

    void initVertices();
    void computeDelaunay();
    void initCells();
    void displayStatistics();

    void saveDhInfo(const std::string& fileNameInfo);
    void saveDh(const std::string& fileNameDh, const std::string& fileNameInfo);

    StaticVector<StaticVector<int>*>* createPtsCams();
    StaticVector<int>* getPtsCamsHist();
    StaticVector<int>* getPtsNrcHist();
    StaticVector<int> getIsUsedPerCamera() const;
    StaticVector<int> getSortedUsedCams() const;

    void addPointsFromSfM(const Point3d hexah[8], const StaticVector<int>& cams, const sfmData::SfMData& sfmData);
    void addPointsFromCameraCenters(const StaticVector<int>& cams, float minDist);
    void addPointsToPreventSingularities(const Point3d Voxel[], float minDist);

    /**
     * @brief Add volume points to prevent singularities
     */
    void addHelperPoints(int nGridHelperVolumePointsDim, const Point3d Voxel[], float minDist);

    void fuseFromDepthMaps(const StaticVector<int>& cams, const Point3d voxel[8], const FuseParams& params);
    void loadPrecomputedDensePoints(const StaticVector<int>* voxelsIds, const Point3d voxel[8], VoxelsGrid* ls);

    void createTetrahedralizationFromDepthMapsCamsVoxel(const StaticVector<int>& allCams,
                                                   StaticVector<int>* voxelsIds, Point3d Voxel[8], VoxelsGrid* ls);

    void computeVerticesSegSize(bool allPoints, float alpha = 0.0f);
    void removeSmallSegs(int minSegSize);

    bool rayCellIntersection(const Point3d &camC, const Point3d &p, int c, Facet& out_facet, bool nearestFarest,
                            Point3d& out_nlpi) const;
    inline bool nearestNeighCellToTheCamOnTheRay(const Point3d &camC, Point3d& out_p, int tetrahedron, Facet &out_f1, Facet &f2,
                                          Point3d& out_lpi) const;
    inline bool farestNeighCellToTheCamOnTheRay(Point3d& camC, Point3d& p, int tetrahedron, Facet &f1, Facet &f2,
                                         Point3d& lpi) const;
    inline Facet getFacetInFrontVertexOnTheRayToTheCam(int vertexIndex, int cam) const;
    Facet getFacetInFrontVertexOnTheRayToThePoint3d(VertexIndex vi, Point3d& ptt) const;
    Facet getFacetBehindVertexOnTheRayToTheCam(VertexIndex vi, int cam) const;
    int getFirstCellOnTheRayFromCamToThePoint(int cam, Point3d& p, Point3d& lpi) const;

    float distFcn(float maxDist, float dist, float distFcnHeight) const;

    inline double conj(double val) const { return val; }
    double facetMaxEdgeLength(Facet& f1) const;
    double maxEdgeLength() const;
    Point3d cellCircumScribedSphereCentre(CellIndex ci) const;
    double getFaceWeight(const Facet &f1) const;
    float weightFromSim(float sim);

    float weightFcn(float nrc, bool labatutWeights, int ncams);

    virtual void fillGraph(bool fixesSigma, float nPixelSizeBehind, bool allPoints, bool behind, bool labatutWeights,
                           bool fillOut, float distFcnHeight = 0.0f);
    void fillGraphPartPtRc(int& out_nstepsFront, int& out_nstepsBehind, int vertexIndex, int cam, float weight,
                           bool fixesSigma, float nPixelSizeBehind, bool allPoints, bool behind, bool fillOut,
                           float distFcnHeight);

    void forceTedgesByGradientCVPR11(bool fixesSigma, float nPixelSizeBehind);
    void forceTedgesByGradientIJCV(bool fixesSigma, float nPixelSizeBehind);

    void updateGraphFromTmpPtsCamsHexah(const StaticVector<int>& incams, Point3d hexah[8], std::string tmpCamsPtsFolderName,
                                        bool labatutWeights, float distFcnHeight = 0.0f);
    void updateGraphFromTmpPtsCamsHexahRC(int rc, Point3d hexah[8], std::string tmpCamsPtsFolderName,
                                          bool labatutWeights, float distFcnHeight);

    int setIsOnSurface();

    void addToInfiniteSw(float sW);

    void freeUnwantedFullCells(const Point3d* hexah);

    void reconstructGC(const Point3d* hexah);

    void maxflow();

    void reconstructExpetiments(const StaticVector<int>& cams, const std::string& folderName,
                                bool update, Point3d hexahInflated[8], const std::string& tmpCamsPtsFolderName,
                                const Point3d& spaceSteps);


    void createDensePointCloud(Point3d hexah[8], const StaticVector<int>& cams, const sfmData::SfMData* sfmData, const FuseParams* depthMapsFuseParams);
    void createDensePointCloudFromPrecomputedDensePoints(Point3d hexah[8], const StaticVector<int>& cams, StaticVector<int>* voxelsIds, VoxelsGrid* ls);

    void createGraphCut(Point3d hexah[8], const StaticVector<int>& cams, VoxelsGrid* ls, const std::string& folderName, const std::string& tmpCamsPtsFolderName,
                        bool removeSmallSegments, const Point3d& spaceSteps);

    /**
     * @brief Invert full/empty status of cells if they represent a too small group after labelling.
     */
    void invertFullStatusForSmallLabels();

    void graphCutPostProcessing();

    void clearAllPointsInFreeSpace();
    void clearAllPointsNotOnSurface();
    void addNewPointsToOccupiedSpace();
    void clearOutAddIn();
    StaticVector<int>* getNearestTrisFromMeshTris(mesh::Mesh* otherMesh);

    void segmentFullOrFree(bool full, StaticVector<int>** inColors, int& nsegments);
    int removeBubbles();
    int removeDust(int minSegSize);
    void leaveLargestFullSegmentOnly();

    mesh::Mesh* createMesh(bool filterHelperPointsTriangles = true);
};


inline bool DelaunayGraphCut::nearestNeighCellToTheCamOnTheRay(const Point3d& camC, Point3d& out_p, int tetrahedron, Facet& out_f1,
                                                      Facet& out_f2, Point3d& out_lpi) const
{
    out_f1.cellIndex = GEO::NO_CELL;
    out_f1.localVertexIndex = GEO::NO_VERTEX;
    out_f2.cellIndex = GEO::NO_CELL;
    out_f2.localVertexIndex = GEO::NO_VERTEX;
    out_lpi = out_p;

    if(rayCellIntersection(camC, out_p, tetrahedron, out_f1, true, out_lpi) == true)
    {
        out_f2 = mirrorFacet(out_f1);
        out_p = out_lpi;
        return true;
    }
    return false;
}

inline bool DelaunayGraphCut::farestNeighCellToTheCamOnTheRay(Point3d& camC, Point3d& p, int tetrahedron, Facet& f1,
                                                     Facet& f2, Point3d& lpi) const
{
    f1.cellIndex = GEO::NO_CELL;
    f1.localVertexIndex = GEO::NO_VERTEX;
    f2.cellIndex = GEO::NO_CELL;
    f2.localVertexIndex = GEO::NO_VERTEX;
    lpi = p;

    if(rayCellIntersection(camC, p, tetrahedron, f1, false, lpi) == true)
    {
        f2 = mirrorFacet(f1);
        p = lpi;
        return true;
    }

    return false;
}

inline DelaunayGraphCut::Facet DelaunayGraphCut::getFacetInFrontVertexOnTheRayToTheCam(int vertexIndex,
                                                                               int cam) const
{
    // if (btest) {
    //	printf("cam %i pt %f %f %f\n",cam,p.x,p.y,p.z);
    //	printf("ptid %i\n",vp->info().id);
    //	printf("campos %f %f
    //%f\n",mp->CArr[cam].x,mp->CArr[cam].y,mp->CArr[cam].z);
    //};
    if((cam < 0) || (cam >= mp->ncams))
    {
        ALICEVISION_LOG_WARNING("Bad camId, cam: " << cam << ", ptid: " << vertexIndex);
    }

    return getFacetInFrontVertexOnTheRayToThePoint3d(vertexIndex, mp->CArr[cam]);
}

} // namespace fuseCut
} // namespace aliceVision
