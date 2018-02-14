// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/structures/Point3d.hpp>
#include <aliceVision/structures/Rgb.hpp>
#include <aliceVision/structures/StaticVector.hpp>
#include <aliceVision/structures/Voxel.hpp>
#include <aliceVision/common/common.hpp>
#include <aliceVision/common/PreMatchCams.hpp>
#include <aliceVision/delaunayCut/delaunayGraphCutTypes.hpp>
#include <aliceVision/largeScale/VoxelsGrid.hpp>
#include <aliceVision/mesh/Mesh.hpp>

#include <geogram/delaunay/delaunay.h>
#include <geogram/delaunay/delaunay_3d.h>
#include <geogram/mesh/mesh.h>
#include <geogram/basic/geometry_nd.h>

#include <map>
#include <set>


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

    MultiViewParams* mp;
    PreMatchCams* pc;

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
    bool btest;
    bool saveTemporaryBinFiles;

    static const GEO::index_t NO_TETRAHEDRON = GEO::NO_CELL;

    DelaunayGraphCut(MultiViewParams* _mp, PreMatchCams* _pc);
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
        std::set<VertexIndex> facetVertices;
        facetVertices.insert(getVertexIndex(f, 0));
        facetVertices.insert(getVertexIndex(f, 1));
        facetVertices.insert(getVertexIndex(f, 2));
        
        Facet out;
        out.cellIndex = _tetrahedralization->cell_adjacent(f.cellIndex, f.localVertexIndex);
        if(out.cellIndex != GEO::NO_CELL)
        {
            // Search for the vertex in adjacent cell which doesn't exist in input facet.
            for(int k = 0; k < 4; ++k)
            {
                CellIndex out_vi = _tetrahedralization->cell_vertex(out.cellIndex, k);
                if(facetVertices.count(out_vi) == 0)
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
        std::cout << "coutInvalidVertices: " << coutInvalidVertices << std::endl;
        std::cout << "neighboringCellsPerVertexTmp.size(): " << neighboringCellsPerVertexTmp.size() << std::endl;
        _neighboringCellsPerVertex.resize(_verticesCoords.size());
        std::cout << "_verticesCoords.size(): " << _verticesCoords.size() << std::endl;
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

    void loadDhInfo(std::string fileNameInfo);
    void loadDh(std::string fileNameDh, std::string fileNameInfo);

    void saveDhInfo(std::string fileNameInfo);
    void saveDh(std::string fileNameDh, std::string fileNameInfo);

    StaticVector<StaticVector<int>*>* createPtsCams();
    StaticVector<int>* getPtsCamsHist();
    StaticVector<int>* getPtsNrcHist();
    StaticVector<int> getIsUsedPerCamera() const;
    StaticVector<int> getSortedUsedCams() const;

    void addPointsFromCameraCenters(StaticVector<int>* cams, float minDist);

    void addPointsToPreventSingularities(Point3d Voxel[8], float minDist);

    /**
     * @brief Add volume points to prevent singularities
     */
    void addHelperPoints(int nGridHelperVolumePointsDim, Point3d Voxel[8], float minDist);

    void createTetrahedralizationFromDepthMapsCamsVoxel(StaticVector<int>* cams,
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
    bool isIncidentToType(VertexIndex vi, float type) const;
    bool isIncidentToSink(VertexIndex vi, bool sink) const;

    float distFcn(float maxDist, float dist, float distFcnHeight) const;
    double facetArea(const Facet& f) const;

    inline double conj(double val) const { return val; }
    double cellMaxEdgeLength(CellIndex ci) const;
    double cellMinEdgeLength(CellIndex ci);
    double facetMaxEdgeLength(Facet& f1) const;
    double getFacetProjectionMaxEdge(Facet& f, int cam) const;
    double maxEdgeLength() const;
    double averageEdgeLength() const;
    Point3d cellCircumScribedSphereCentre(CellIndex ci) const;
    Point3d cellCentreOfGravity(CellIndex ci) const;
    double cellVolume(CellIndex cv) const;
    double getFaceWeight(const Facet &f1) const;
    float weightFromSim(float sim);

    float weightFcn(float nrc, bool labatutWeights, int ncams);
    bool isCellSmallForPoint(CellIndex ci, VertexIndex vi) const;

    virtual void fillGraph(bool fixesSigma, float nPixelSizeBehind, bool allPoints, bool behind, bool labatutWeights,
                           bool fillOut, float distFcnHeight = 0.0f);
    void fillGraphPartPtRc(int& out_nstepsFront, int& out_nstepsBehind, int vertexIndex, int cam, float weight,
                           bool fixesSigma, float nPixelSizeBehind, bool allPoints, bool behind, bool fillOut,
                           float distFcnHeight);

    void forceTedgesByGradientCVPR11(bool fixesSigma, float nPixelSizeBehind);
    void forceTedgesByGradientIJCV(bool fixesSigma, float nPixelSizeBehind);

    void filterPointsWithHigherPixelSize(bool fixesSigma, float nPixelSizeBehind);

    void updateGraphFromTmpPtsCamsHexah(StaticVector<int>* incams, Point3d hexah[8], std::string tmpCamsPtsFolderName,
                                        bool labatutWeights, float distFcnHeight = 0.0f);
    void updateGraphFromTmpPtsCamsHexahRC(int rc, Point3d hexah[8], std::string tmpCamsPtsFolderName,
                                          bool labatutWeights, float distFcnHeight);

    int setIsOnSurface();

    bool isCamInFrontOfFacet(Facet f, int rc);
    float triangle_area(mv2DTriangle& t);
    float triangle_maxSide(mv2DTriangle& t);
    float triangle_minSide(mv2DTriangle& t);
    float triangle_incircle_area(mv2DTriangle& t);
    float triangle_circumscribed_area(mv2DTriangle& t);

    void addToInfiniteSw(float sW);

    float getAveragePixelSize() const;
    void freeUnwantedFullCells(std::string folderName, Point3d* hexah);
    void saveMaxflowToWrl(std::string dirName, std::string fileNameTxt, std::string fileNameTxtCam,
                          std::string fileNameWrl, std::string fileNameWrlTex, std::string fileNamePly,
                          int camerasPerOneOmni);
    void saveMaxflowToWrl(std::string dirName, std::string fileNameTxt, std::string fileNameTxtCam,
                          std::string fileNameWrl, std::string fileNameWrlTex, std::string fileNamePly,
                          int camerasPerOneOmni, StaticVector<int>* cams);

    void reconstructGC(float alphaQual, std::string baseName, StaticVector<int>* cams, std::string folderName,
                       std::string fileNameStGraph, std::string fileNameStSolution, std::string fileNameTxt,
                       std::string fileNameTxtCam, int camerasPerOneOmni, bool doRemoveBubbles,
                       StaticVector<Point3d>* hexahsToExcludeFromResultingMesh, Point3d* hexah);

    void maxflow();

    void reconstructExpetiments(StaticVector<int>* cams, std::string folderName, std::string fileNameStGraph,
                                std::string fileNameStSolution, std::string fileNameTxt, std::string fileNameTxtCam,
                                int camerasPerOneOmni, bool update, Point3d hexahInflated[8],
                                std::string tmpCamsPtsFolderName,
                                StaticVector<Point3d>* hexahsToExcludeFromResultingMesh, Point3d spaceSteps);

    void reconstructVoxel(Point3d hexah[8], StaticVector<int>* voxelsIds, std::string folderName,
                          std::string tmpCamsPtsFolderName, bool segment,
                          StaticVector<Point3d>* hexahsToExcludeFromResultingMesh, VoxelsGrid* ls, Point3d spaceSteps);

    bool hasVertex(CellIndex ci, VertexIndex vi) const;
    void getIncidentCellsToCellAndVertexOfTheCellIndexes(int vIncident[3], CellIndex ci, VertexIndex vi) const;
    void getIncidentCellsToCellAndEdgeOfTheCellIndexes(int vIncident[2], CellIndex ci, int lvi, int lvj) const;

    /**
     * @brief Invert full/empty status of cells if they represent a too small group after labelling.
     */
    void invertFullStatusForSmallLabels();

    int erosionDilatation(bool sink);
    void graphCutPostProcessing();

    void clearAllPointsInFreeSpace();
    void clearAllPointsNotOnSurface();
    void addNewPointsToOccupiedSpace();
    void clearOutAddIn();
    float computeSurfaceArea();
    StaticVector<int>* getNearestTrisFromMeshTris(Mesh* otherMesh);
    StaticVector<int>* getNearestPtsFromMesh(Mesh& otherMesh);

    void segmentFullOrFree(bool full, StaticVector<int>** inColors, int& nsegments);
    int removeBubbles();
    int removeDust(int minSegSize);
    void leaveLargestFullSegmentOnly();
    StaticVector<float>* computeSegmentsSurfaceArea(bool full, StaticVector<int>& colors, int nsegments);

    Mesh* createMesh(bool filterHelperPointsTriangles = true);
    StaticVector<rgb>* getPtsColorsByNCams();

    void initTetrahedralizationFromMeshTrianglesCenter(Mesh* mesh, bool _addPointsToPreventSingularities);
    void initTetrahedralizationFromMeshVertices(Mesh* mesh, bool _addPointsToPreventSingularities);

    StaticVector<StaticVector<int>*>* createPtsCamsForAnotherMesh(StaticVector<StaticVector<int>*>* refPtsCams, Mesh& otherMesh);
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
        printf("WARNING cam %i, ptid %i\n", cam, vertexIndex);
    }

    return getFacetInFrontVertexOnTheRayToThePoint3d(vertexIndex, mp->CArr[cam]);
}
