// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "mv_delaunay_types.hpp"

#include <aliceVision/output3D/mv_output3D.hpp>
#include <aliceVision/prematching/mv_prematch_cams.hpp>
#include <aliceVision/largeScale/voxelsGrid.hpp>

#include <geogram/delaunay/delaunay.h>
#include <geogram/delaunay/delaunay_3d.h>
#include <geogram/mesh/mesh.h>
#include <geogram/basic/geometry_nd.h>

#include <map>
#include <set>


class mv_delaunay_GC
{
public:
    multiviewParams* mp;
    mv_prematch_cams* pc;

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
        VertexIndex localVertexIndex = GEO::NO_VERTEX; /// local opposite vertex index
    };

    /// Get absolute opposite vertex index
    VertexIndex getOppositeVertexIndex(const Facet& f) const
    {
        return _tetrahedralization->cell_vertex(f.cellIndex, f.localVertexIndex);
    }
    /// Get absolute vertex index
    VertexIndex getVertexIndex(const Facet& f, int i) const
    {
        return _tetrahedralization->cell_vertex(f.cellIndex, ((f.localVertexIndex + i + 1) % 4));
    }
    GEO::Delaunay_var _tetrahedralization;
    std::vector<point3d> _verticesCoords; /// 3D points coordinates
    std::vector<GC_vertexInfo> _verticesAttr; /// Information attached to each vertex
    std::vector<GC_cellInfo> _cellsAttr; /// Information attached to each cell
    std::vector<bool> _cellIsFull; /// isFull info per cell: true is full / false is empty

    std::vector<int> _camsVertexes;
    std::vector<std::vector<CellIndex>> _neighboringCellsPerVertex;

    static const GEO::index_t NO_TETRAHEDRON = GEO::NO_CELL;

    std::size_t getNbVertices() const { return _verticesAttr.size(); }

    GEO::index_t nearestVertexInCell(GEO::index_t cellIndex, const point3d& p) const
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

    GEO::index_t locateNearestVertex(const point3d& p) const
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

    Facet mirrorFacet(const Facet& f) const
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

    bool btest;
    bool saveTemporaryBinFiles;
    bool saveTemporaryWrlFiles;

    mv_delaunay_GC(multiviewParams* _mp, mv_prematch_cams* _pc);
    virtual ~mv_delaunay_GC();

    void initVertices();
    void computeDelaunay();
    void initCells();
    void displayStatistics();

    void loadDhInfo(std::string fileNameInfo);
    void loadDh(std::string fileNameDh, std::string fileNameInfo);

    void saveDhInfo(std::string fileNameInfo);
    void saveDh(std::string fileNameDh, std::string fileNameInfo);

    staticVector<staticVector<int>*>* createPtsCams();
    staticVector<int>* getPtsCamsHist();
    staticVector<int>* getPtsNrcHist();
    staticVector<int> getIsUsedPerCamera() const;
    staticVector<int> getSortedUsedCams() const;

    void addPointsFromCameraCenters(staticVector<int>* cams, float minDist);

    void addPointsToPreventSingularities(point3d voxel[8], float minDist);

    /**
     * @brief Add volume points to prevent singularities
     */
    void addHelperPoints(int nGridHelperVolumePointsDim, point3d voxel[8], float minDist);

    void createTetrahedralizationFromDepthMapsCamsVoxel(staticVector<int>* cams,
                                                   staticVector<int>* voxelsIds, point3d voxel[8], voxelsGrid* ls);

    void computeVerticesSegSize(const std::string &fileNameWrl, bool allPoints, float alpha = 0.0f, bool saveWrl = true);
    void removeSmallSegs(int minSegSize);

    bool rayCellIntersection(const point3d &camC, const point3d &p, int c, Facet& out_facet, bool nearestFarest,
                            point3d& out_nlpi) const;
    inline bool nearestNeighCellToTheCamOnTheRay(const point3d &camC, point3d& out_p, int tetrahedron, Facet &out_f1, Facet &f2,
                                          point3d& out_lpi) const;
    inline bool farestNeighCellToTheCamOnTheRay(point3d& camC, point3d& p, int tetrahedron, Facet &f1, Facet &f2,
                                         point3d& lpi) const;
    inline Facet getFacetInFrontVertexOnTheRayToTheCam(int vertexIndex, int cam) const;
    Facet getFacetInFrontVertexOnTheRayToThePoint3D(VertexIndex vi, point3d& ptt) const;
    Facet getFacetBehindVertexOnTheRayToTheCam(VertexIndex vi, int cam) const;
    int getFirstCellOnTheRayFromCamToThePoint(int cam, point3d& p, point3d& lpi) const;
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
    point3d cellCircumScribedSphereCentre(CellIndex ci) const;
    point3d cellCentreOfGravity(CellIndex ci) const;
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

    void updateGraphFromTmpPtsCamsHexah(staticVector<int>* incams, point3d hexah[8], std::string tmpCamsPtsFolderName,
                                        bool labatutWeights, float distFcnHeight = 0.0f);
    void updateGraphFromTmpPtsCamsHexahRC(int rc, point3d hexah[8], std::string tmpCamsPtsFolderName,
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
    void freeUnwantedFullCells(std::string folderName, point3d* hexah);
    void saveMaxflowToWrl(std::string dirName, std::string fileNameTxt, std::string fileNameTxtCam,
                          std::string fileNameWrl, std::string fileNameWrlTex, std::string fileNamePly,
                          int camerasPerOneOmni);
    void saveMaxflowToWrl(std::string dirName, std::string fileNameTxt, std::string fileNameTxtCam,
                          std::string fileNameWrl, std::string fileNameWrlTex, std::string fileNamePly,
                          int camerasPerOneOmni, staticVector<int>* cams);

    void reconstructGC(float alphaQual, std::string baseName, staticVector<int>* cams, std::string folderName,
                       std::string fileNameStGraph, std::string fileNameStSolution, std::string fileNameTxt,
                       std::string fileNameTxtCam, int camerasPerOneOmni, bool doRemoveBubbles,
                       staticVector<point3d>* hexahsToExcludeFromResultingMesh, bool saveToWrl, point3d* hexah);

    void maxflow();

    void reconstructExpetiments(staticVector<int>* cams, std::string folderName, std::string fileNameStGraph,
                                std::string fileNameStSolution, std::string fileNameTxt, std::string fileNameTxtCam,
                                int camerasPerOneOmni, bool update, point3d hexahInflated[8],
                                std::string tmpCamsPtsFolderName,
                                staticVector<point3d>* hexahsToExcludeFromResultingMesh, point3d spaceSteps);

    void reconstructVoxel(point3d hexah[8], staticVector<int>* voxelsIds, std::string folderName,
                          std::string tmpCamsPtsFolderName, bool segment,
                          staticVector<point3d>* hexahsToExcludeFromResultingMesh, voxelsGrid* ls, point3d spaceSteps);

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
    staticVector<int>* getNearestTrisFromMeshTris(mv_mesh* otherMesh);
    staticVector<int>* getNearestPtsFromMesh(mv_mesh& otherMesh);

    void segmentFullOrFree(bool full, staticVector<int>** inColors, int& nsegments);
    int removeBubbles();
    int removeDust(int minSegSize);
    void leaveLargestFullSegmentOnly();
    staticVector<float>* computeSegmentsSurfaceArea(bool full, staticVector<int>& colors, int nsegments);

    mv_mesh* createMesh();
    staticVector<rgb>* getPtsColorsByNCams();

    void initTetrahedralizationFromMeshTrianglesCenter(mv_mesh* mesh, bool _addPointsToPreventSingularities);
    void initTetrahedralizationFromMeshVertices(mv_mesh* mesh, bool _addPointsToPreventSingularities);

    void saveMeshColoredByCamsConsistency(const std::string &consistencyWrlFilepath, const std::string &nbCamsWrlFilepath);

    void saveSegsWrl(const std::string &fileNameWrl);


    staticVector<staticVector<int>*>* createPtsCamsForAnotherMesh(staticVector<staticVector<int>*>* refPtsCams, mv_mesh& otherMesh);
};


inline bool mv_delaunay_GC::nearestNeighCellToTheCamOnTheRay(const point3d& camC, point3d& out_p, int tetrahedron, Facet& out_f1,
                                                      Facet& out_f2, point3d& out_lpi) const
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

inline bool mv_delaunay_GC::farestNeighCellToTheCamOnTheRay(point3d& camC, point3d& p, int tetrahedron, Facet& f1,
                                                     Facet& f2, point3d& lpi) const
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

inline mv_delaunay_GC::Facet mv_delaunay_GC::getFacetInFrontVertexOnTheRayToTheCam(int vertexIndex,
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

    return getFacetInFrontVertexOnTheRayToThePoint3D(vertexIndex, mp->CArr[cam]);
}
