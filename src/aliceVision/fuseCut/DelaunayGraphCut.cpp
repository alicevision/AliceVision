// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

// To analyse history of intersected geometries during vote

#include "DelaunayGraphCut.hpp"
#include <aliceVision/fuseCut/MaxFlow_AdjList.hpp>
#include <aliceVision/fuseCut/Fuser.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/mvsData/geometry.hpp>
#include <aliceVision/image/jetColorMap.hpp>
#include <aliceVision/mvsData/Pixel.hpp>
#include <aliceVision/mvsData/Point2d.hpp>
#include <aliceVision/mvsData/Universe.hpp>
#include <aliceVision/mvsUtils/fileIO.hpp>
#include <aliceVision/mvsUtils/mapIO.hpp>
#include <aliceVision/image/imageAlgo.hpp>
#include <aliceVision/system/ProgressDisplay.hpp>
#include <aliceVision/alicevision_omp.hpp>

#include "nanoflann.hpp"

#include <geogram/points/kd_tree.h>

#include <cmath>
#include <filesystem>
#include <random>
#include <stdexcept>

#include <boost/math/constants/constants.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <boost/atomic/atomic_ref.hpp>

#include <aliceVision/fuseCut/Kdtree.hpp>

namespace aliceVision {
namespace fuseCut {

namespace fs = std::filesystem;


DelaunayGraphCut::DelaunayGraphCut(mvsUtils::MultiViewParams& mp, const PointCloud & pc, const Tetrahedralization & tetrahedralization)
  : 
  _mp(mp),
  _verticesCoords(pc.getVertices()),
  _verticesAttr(pc.getVerticesAttrs()),
  _camsVertexes(pc.getCameraIndices()),
  _tetrahedralization(tetrahedralization)
{
    initCells();
}

DelaunayGraphCut::~DelaunayGraphCut() {}

std::vector<DelaunayGraphCut::CellIndex> DelaunayGraphCut::getNeighboringCellsByGeometry(const GeometryIntersection& g) const
{
    switch (g.type)
    {
        case EGeometryType::Edge:
            return _tetrahedralization.getNeighboringCellsByEdge(g.edge);
        case EGeometryType::Vertex:
            return _tetrahedralization.getNeighboringCellsByVertexIndex(g.vertexIndex);
        case EGeometryType::Facet:
            return _tetrahedralization.getNeighboringCellsByFacet(g.facet);
        case EGeometryType::None:
            break;
    }

    throw std::runtime_error("[error] getNeighboringCellsByGeometry: an undefined/None geometry has no neighboring cells.");
}


void DelaunayGraphCut::initCells()
{
    _cellsAttr.resize(_tetrahedralization.nb_cells());

    for (int i = 0; i < _cellsAttr.size(); ++i)
    {
        GC_cellInfo& c = _cellsAttr[i];

        c.cellSWeight = 0.0f;
        c.cellTWeight = 0.0f;
        c.on = 0.0f;
        c.emptinessScore = 0.0f;
        for (int s = 0; s < 4; ++s)
        {
            //weights for the 4 faces of the tetrahedron
            c.gEdgeVisWeight[s] = 0.0f;
        }
    }
}

void DelaunayGraphCut::createPtsCams(StaticVector<StaticVector<int>>& out_ptsCams)
{
    long t = std::clock();
    ALICEVISION_LOG_INFO("Extract visibilities.");
    int npts = getNbVertices();

    out_ptsCams.reserve(npts);

    for (const GC_vertexInfo& v : _verticesAttr)
    {
        StaticVector<int> cams;
        cams.reserve(v.getNbCameras());
        for (int c = 0; c < v.getNbCameras(); c++)
        {
            cams.push_back(v.cams[c]);
        }
        out_ptsCams.push_back(cams);
    }  // for i

    ALICEVISION_LOG_INFO("Extract visibilities done.");

    mvsUtils::printfElapsedTime(t, "Extract visibilities ");
}

void DelaunayGraphCut::fillGraph(double nPixelSizeBehind,
                                 float fullWeight)
{
    ALICEVISION_LOG_INFO("Computing s-t graph weights.");
    long t1 = clock();

    // choose random order to prevent waiting
    const unsigned int seed = (unsigned int)_mp.userParams.get<unsigned int>("delaunaycut.seed", 0);
    const std::vector<int> verticesRandIds = mvsUtils::createRandomArrayOfIntegers(_verticesAttr.size(), seed);

#pragma omp parallel for 
    for (int i = 0; i < verticesRandIds.size(); i++)
    {
        const int vertexIndex = verticesRandIds[i];
        const GC_vertexInfo& v = _verticesAttr[vertexIndex];

        if (!v.isReal())
        {  
            continue;
        }
        
        float weight = (float)v.nrc;  // number of cameras

        //Overwrite with forced weight if available
        weight = (float)_mp.userParams.get<double>("LargeScale.forceWeight", weight);

        for (int c = 0; c < v.cams.size(); c++)
        {
            assert(v.cams[c] >= 0);
            assert(v.cams[c] < _mp.ncams);

            rayMarchingGraphEmpty(vertexIndex,
                                v.cams[c],
                                weight);

            rayMarchingGraphFull(vertexIndex,
                                v.cams[c],
                                weight* fullWeight,
                                nPixelSizeBehind);
        }  // for c
    }

    mvsUtils::printfElapsedTime(t1, "s-t graph weights computed : ");
}

void DelaunayGraphCut::rayMarchingGraphEmpty(int vertexIndex,
                                         int cam,
                                         float weight)
{
    const int maxint = std::numeric_limits<int>::max();

    const Point3d& originPt = _verticesCoords[vertexIndex];

    assert(cam >= 0);
    assert(cam < _mp.ncams);

    // Initialisation
    GeometryIntersection geometry(vertexIndex);  // Starting on global vertex index
    Point3d intersectPt = originPt;
    // toTheCam
    const double pointCamDistance = (_mp.CArr[cam] - originPt).size();



    TetrahedronsRayMarching marching(_tetrahedralization, vertexIndex, _camsVertexes[cam], false);

    Facet lastIntersectedFacet;
    bool lastGeoIsVertex = false;
    // Break only when we reach our camera vertex (as long as we find a next geometry)
    while (geometry.type != EGeometryType::Vertex || (_mp.CArr[cam] - intersectPt).size() >= 1.0e-3)
    {
        lastGeoIsVertex = false;
        // Keep previous informations
        const GeometryIntersection previousGeometry = geometry;
        const Point3d lastIntersectPt = intersectPt;

        
        geometry = marching.intersectNextGeom();
        Eigen::Vector3d eintersectPt = marching.getIntersectionPoint();
        intersectPt.x = eintersectPt.x();
        intersectPt.y = eintersectPt.y();
        intersectPt.z = eintersectPt.z();

        if (geometry.type == EGeometryType::None)
        {
            break;
        }


        if (geometry.type == EGeometryType::Facet)
        {
            GeometryIntersection previousGeometry = marching.getPreviousIntersection();

            boost::atomic_ref<float>{_cellsAttr[previousGeometry.facet.cellIndex].emptinessScore} += weight;
            boost::atomic_ref<float>{_cellsAttr[previousGeometry.facet.cellIndex].gEdgeVisWeight[previousGeometry.facet.localVertexIndex]} += weight;

            
            lastIntersectedFacet = geometry.facet;
        }
        else
        {
            if (previousGeometry.type == EGeometryType::Facet)
            {
                boost::atomic_ref<float>{_cellsAttr[previousGeometry.facet.cellIndex].emptinessScore} += weight;
            }

            if (geometry.type == EGeometryType::Vertex)
            {
                lastGeoIsVertex = true;
            }
            else if (geometry.type == EGeometryType::Edge)
            {
            }
        }

        // Declare the last part of the empty path as connected to EMPTY (S node in the graph cut)
        if (lastIntersectedFacet.cellIndex != GEO::NO_CELL && (_mp.CArr[cam] - intersectPt).size() < 0.2 * pointCamDistance)
        {
            boost::atomic_ref<float>{_cellsAttr[lastIntersectedFacet.cellIndex].cellSWeight} = (float)maxint;
        }
    }

    // Vote for the last intersected facet (close to the cam)
    if (lastIntersectedFacet.cellIndex != GEO::NO_CELL)
    {
        boost::atomic_ref<float>{_cellsAttr[lastIntersectedFacet.cellIndex].cellSWeight} = (float)maxint;
    }
}

void DelaunayGraphCut::rayMarchingGraphFull(int vertexIndex,
                                         int cam,
                                         float fullWeight,
                                         double nPixelSizeBehind)
{
    const int maxint = std::numeric_limits<int>::max();
    const Point3d& originPt = _verticesCoords[vertexIndex];
    const double pixSize = _verticesAttr[vertexIndex].pixSize;
    const double maxDist = nPixelSizeBehind * pixSize;

    if (pixSize <= 0.0)
    {
        return;
    }

    // Initialisation
    GeometryIntersection geometry(vertexIndex);  // Starting on global vertex index
    Point3d intersectPt = originPt;

    TetrahedronsRayMarching marching(_tetrahedralization, vertexIndex, _camsVertexes[cam], true);

    bool firstIteration = true;
    Facet lastIntersectedFacet;
    // While we are within the surface margin (as long as we find a next geometry)
    while ((originPt - intersectPt).size() < maxDist)
    {
        // Keep previous informations
        const GeometryIntersection previousGeometry = geometry;
        const Point3d lastIntersectPt = intersectPt;



        geometry = marching.intersectNextGeom();
        Eigen::Vector3d eintersectPt = marching.getIntersectionPoint();
        intersectPt.x = eintersectPt.x();
        intersectPt.y = eintersectPt.y();
        intersectPt.z = eintersectPt.z();

        if (geometry.type == EGeometryType::None)
        {
            break;
        }

        
        if (geometry.type == EGeometryType::Facet)
        {
            lastIntersectedFacet = geometry.facet;
            boost::atomic_ref<float>{_cellsAttr[geometry.facet.cellIndex].gEdgeVisWeight[geometry.facet.localVertexIndex]} += fullWeight;
        }
    }

    // found facet Vote for the last intersected facet (farthest from the camera)
    if (lastIntersectedFacet.cellIndex != GEO::NO_CELL)
    {
        boost::atomic_ref<float>{_cellsAttr[lastIntersectedFacet.cellIndex].cellTWeight} += fullWeight;
    }
}

void DelaunayGraphCut::forceTedgesByGradientIJCV(float nPixelSizeBehind)
{
    ALICEVISION_LOG_INFO("Forcing t-edges");
    long t2 = clock();

    const float forceTEdgeDelta = 0.1f;
    const float minJumpPartRange = 10000.0f;
    const float maxSilentPartRange = 100.0f;
    const float nsigmaJumpPart = 4.0f;
    const float nsigmaFrontSilentPart = 2.0f;
    const float nsigmaBackSilentPart = 2.0f;


    // choose random order to prevent waiting
    const unsigned int seed = (unsigned int)_mp.userParams.get<unsigned int>("delaunaycut.seed", 0);
    const std::vector<int> verticesRandIds = mvsUtils::createRandomArrayOfIntegers(_verticesAttr.size(), seed);


#pragma omp parallel for
    for (int i = 0; i < verticesRandIds.size(); ++i)
    {
        const int vertexIndex = verticesRandIds[i];
        const GC_vertexInfo& v = _verticesAttr[vertexIndex];
        if (v.isVirtual())
            continue;

        const Point3d& originPt = _verticesCoords[vertexIndex];
        // For each camera that has visibility over the vertex v (vertexIndex)
        for (const int cam : v.cams)
        {
            
            const float maxDist = nPixelSizeBehind * _mp.getCamPixelSize(originPt, cam);

            float maxJump = 0.0f;
            float maxSilent = 0.0f;
            float midSilent = 10000000.0f;

            {
                // Initialisation
                GeometryIntersection geometry(vertexIndex);  // Starting on global vertex index
                Point3d intersectPt = originPt;
                // toTheCam
            

                TetrahedronsRayMarching marching(_tetrahedralization, vertexIndex, _camsVertexes[cam], false);

                // As long as we find a next geometry
                Point3d lastIntersectPt = originPt;
                // Iterate on geometries in the direction of camera's vertex within margin defined by maxDist (as long as we find a next geometry)
                while ((geometry.type != EGeometryType::Vertex || (_mp.CArr[cam] - intersectPt).size() > 1.0e-3)  // We reach our camera vertex
                       &&
                       (lastIntersectPt - originPt).size() <= (nsigmaJumpPart + nsigmaFrontSilentPart) * maxDist)  // We are to far from the originPt
                {
                    // Keep previous informations
                    const GeometryIntersection previousGeometry = geometry;
                    lastIntersectPt = intersectPt;

                    geometry = marching.intersectNextGeom();
                    Eigen::Vector3d eintersectPt = marching.getIntersectionPoint();
                    intersectPt.x = eintersectPt.x();
                    intersectPt.y = eintersectPt.y();
                    intersectPt.z = eintersectPt.z();

                    if (geometry.type == EGeometryType::None)
                    {
                        break;
                    }

                    if (geometry.type == EGeometryType::Facet)
                    {
                        GeometryIntersection previousGeometry = marching.getPreviousIntersection();

                        const GC_cellInfo& c = _cellsAttr[previousGeometry.facet.cellIndex];
                        if ((lastIntersectPt - originPt).size() > nsigmaFrontSilentPart * maxDist)  // (p-originPt).size() > 2 * sigma
                        {
                            maxJump = std::max(maxJump, c.emptinessScore);
                        }
                        else
                        {
                            maxSilent = std::max(maxSilent, c.emptinessScore);
                        }
                    }
                }
            }
            {
                // Initialisation
                GeometryIntersection geometry(vertexIndex);
                Point3d intersectPt = originPt;

               
                TetrahedronsRayMarching marching(_tetrahedralization, vertexIndex, _camsVertexes[cam], true);

                Facet lastIntersectedFacet;
                bool firstIteration = true;
                Point3d lastIntersectPt = originPt;

                // While we are within the surface margin defined by maxDist (as long as we find a next geometry)
                while ((lastIntersectPt - originPt).size() <= nsigmaBackSilentPart * maxDist)
                {
                    // Keep previous informations
                    const GeometryIntersection previousGeometry = geometry;
                    lastIntersectPt = intersectPt;


                    geometry = marching.intersectNextGeom();

                    Eigen::Vector3d eintersectPt = marching.getIntersectionPoint();
                    intersectPt.x = eintersectPt.x();
                    intersectPt.y = eintersectPt.y();
                    intersectPt.z = eintersectPt.z();
                    

                    if (geometry.type == EGeometryType::None)
                    {
                        break;
                    }

                   
                    if (geometry.type == EGeometryType::Facet)
                    {
                        GeometryIntersection previousGeometry = marching.getPreviousIntersection();

                        // Vote for the first cell found (only once)
                        if (firstIteration)
                        {
                            midSilent = _cellsAttr[previousGeometry.facet.cellIndex].emptinessScore;
                            firstIteration = false;
                        }

                        const GC_cellInfo& c = _cellsAttr[previousGeometry.facet.cellIndex];
                        maxSilent = std::max(maxSilent, c.emptinessScore);

                        lastIntersectedFacet = geometry.facet;
                    }
                    else
                    {
                        // Vote for the first cell found (only once)
                        // if we come from an edge or vertex to an other we have to vote for the first intersected cell.
                        if (firstIteration)
                        {
                            if (previousGeometry.type != EGeometryType::Vertex)
                            {
                                ALICEVISION_LOG_ERROR("The firstIteration vote could only happen during for "
                                                      "the first cell when we come from the first vertex.");
                                // throw std::runtime_error("[error] The firstIteration vote could only happen during for the first cell when we come
                                // from the first vertex.");
                            }
                            // the information of first intersected cell can only be found by taking intersection of neighbouring cells for both
                            // geometries
                            const std::vector<CellIndex> previousNeighbouring = _tetrahedralization.getNeighboringCellsByVertexIndex(previousGeometry.vertexIndex);
                            const std::vector<CellIndex> currentNeigbouring = getNeighboringCellsByGeometry(geometry);

                            std::vector<CellIndex> neighboringCells;
                            std::set_intersection(previousNeighbouring.begin(),
                                                  previousNeighbouring.end(),
                                                  currentNeigbouring.begin(),
                                                  currentNeigbouring.end(),
                                                  std::back_inserter(neighboringCells));

                            for (const CellIndex& ci : neighboringCells)
                            {
                                midSilent = _cellsAttr[geometry.facet.cellIndex].emptinessScore;
                            }
                            firstIteration = false;
                        }

                    }
                }

                if (lastIntersectedFacet.cellIndex != GEO::NO_CELL)
                {
                    // Equation 6 in paper
                    //   (g / B) < k_rel
                    //   (B - g) > k_abs
                    //   g < k_outl

                    // In the paper:
                    // B (beta): max value before point p
                    // g (gamma): mid-range score behind point p

                    // In the code:
                    // maxJump: max score of emptiness in all the tetrahedron along the line of sight between camera c and 2*sigma before p
                    // midSilent: score of the next tetrahedron directly after p (called T1 in the paper)
                    // maxSilent: max score of emptiness for the tetrahedron around the point p (+/- 2*sigma around p)

                    if ((midSilent / maxJump < forceTEdgeDelta) &&   // (g / B) < k_rel    //// k_rel=0.1
                        (maxJump - midSilent > minJumpPartRange) &&  // (B - g) > k_abs   //// k_abs=10000 // 1000 in the paper
                        (maxSilent < maxSilentPartRange))            // g < k_outl                  //// k_outl=100  // 400 in the paper
                                                                     //(maxSilent-minSilent<maxSilentPartRange))
                    {
                        boost::atomic_ref<float>{_cellsAttr[lastIntersectedFacet.cellIndex].on} += (maxJump - midSilent);
                    }
                }
            }
        }
    }

    for (GC_cellInfo& c : _cellsAttr)
    {
        const float w = std::max(1.0f, c.cellTWeight) * c.on;

        // cellTWeight = clamp(w, cellTWeight, 1000000.0f);
        c.cellTWeight = std::max(c.cellTWeight, std::min(1000000.0f, w));
    }


    mvsUtils::printfElapsedTime(t2, "t-edges forced: ");
}

int DelaunayGraphCut::computeIsOnSurface(std::vector<bool>& vertexIsOnSurface) const
{
    vertexIsOnSurface.resize(_verticesCoords.size(), false);

    int nbSurfaceFacets = 0;
    // loop over all facets
    for (CellIndex ci = 0; ci < _cellIsFull.size(); ++ci)
    {
        for (VertexIndex k = 0; k < 4; ++k)
        {
            const Facet f1(ci, k);
            const bool uo = _cellIsFull[f1.cellIndex];  // get if it is occupied
            if (!uo)
                continue;

            const Facet f2 = _tetrahedralization.mirrorFacet(f1);
            if (_tetrahedralization.isInvalidOrInfiniteCell(f2.cellIndex))
                continue;
            const bool vo = _cellIsFull[f2.cellIndex];  // get if it is occupied

            if (vo)
                continue;

            VertexIndex v1 = _tetrahedralization.getVertexIndex(f1, 0);
            VertexIndex v2 = _tetrahedralization.getVertexIndex(f1, 1);
            VertexIndex v3 = _tetrahedralization.getVertexIndex(f1, 2);
            ++nbSurfaceFacets;
            vertexIsOnSurface[v1] = true;
            vertexIsOnSurface[v2] = true;
            vertexIsOnSurface[v3] = true;

            assert(!(_tetrahedralization.isInfiniteCell(f1.cellIndex) && _tetrahedralization.isInfiniteCell(f2.cellIndex)));  // infinite both cells of finite vertex!
        }
    }
    ALICEVISION_LOG_INFO("computeIsOnSurface nbSurfaceFacets: " << nbSurfaceFacets);
    return nbSurfaceFacets;
}

void DelaunayGraphCut::graphCutPostProcessing(const Point3d hexah[8])
{
    long timer = std::clock();
    ALICEVISION_LOG_INFO("Graph cut post-processing.");

    int minSegmentSize = _mp.userParams.get<int>("hallucinationsFiltering.minSegmentSize", 10);
    int invertTetrahedronBasedOnNeighborsNbIterations =
      _mp.userParams.get<bool>("hallucinationsFiltering.invertTetrahedronBasedOnNeighborsNbIterations", 10);
    double minSolidAngleRatio = _mp.userParams.get<double>("hallucinationsFiltering.minSolidAngleRatio", 0.2);
    int nbSolidAngleFilteringIterations = _mp.userParams.get<double>("hallucinationsFiltering.nbSolidAngleFilteringIterations", 10);

    removeBubbles();

    {
        std::size_t nbFullCells = std::accumulate(_cellIsFull.begin(), _cellIsFull.end(), 0);
        ALICEVISION_LOG_INFO("[" << __LINE__ << "] Nb full cells: " << nbFullCells << " / " << _cellIsFull.size() << " cells.");
    }

    if (true)
    {
        // free all full cell that have a camera vertex
        int nbModifiedCells = 0;
        for (int rc = 0; rc < _mp.ncams; rc++)
        {
            VertexIndex cam_vi = _camsVertexes[rc];
            if (cam_vi == GEO::NO_VERTEX)
                continue;
            for (CellIndex adjCellIndex : _tetrahedralization.getNeighboringCellsByVertexIndex(cam_vi))  // GEOGRAM: set_stores_cicl(true) required
            {
                if (_tetrahedralization.isInvalidOrInfiniteCell(adjCellIndex))
                    continue;

                if (_cellIsFull[adjCellIndex])
                {
                    _cellIsFull[adjCellIndex] = false;
                    ++nbModifiedCells;
                }
            }
        }
        ALICEVISION_LOG_WARNING("Declare empty around camera centers: " << nbModifiedCells << " cells changed to empty within " << _cellIsFull.size()
                                                                        << " cells.");
    }

    removeDust(minSegmentSize);

    invertFullStatusForSmallLabels();

    {
        // Changed status of cells to improve coherence with neighboring tetrahedrons
        // If 3 or 4 facets are connected to cells of the opporite status,
        // it is better to update the current status.
        for (int i = 0; i < invertTetrahedronBasedOnNeighborsNbIterations; ++i)
        {
            StaticVector<CellIndex> toDoInverse;
            toDoInverse.reserve(_cellIsFull.size());

            for (CellIndex ci = 0; ci < _cellIsFull.size(); ++ci)
            {
                int count = 0;
                for (int k = 0; k < 4; ++k)
                {
                    const CellIndex nci = _tetrahedralization.cell_adjacent(ci, k);
                    if (nci == GEO::NO_CELL)
                        continue;
                    count += (_cellIsFull[nci] != _cellIsFull[ci]);
                }
                if (count > 2)
                    toDoInverse.push_back(ci);
            }
            if (toDoInverse.empty())
                break;
            int movedToEmpty = 0;
            int movedToFull = 0;
            for (std::size_t i = 0; i < toDoInverse.size(); ++i)
            {
                CellIndex ci = toDoInverse[i];
                _cellIsFull[ci] = !_cellIsFull[ci];
                if (_cellIsFull[ci])
                    ++movedToFull;
                else
                    ++movedToEmpty;
            }
            ALICEVISION_LOG_WARNING("[" << i << "] Coherence with neighboring tetrahedrons: " << movedToFull << " cells moved to full, "
                                        << movedToEmpty << " cells moved to empty within " << _cellIsFull.size() << " cells.");
        }
    }

    cellsStatusFilteringBySolidAngleRatio(nbSolidAngleFilteringIterations, minSolidAngleRatio);

    ALICEVISION_LOG_INFO("Graph cut post-processing done.");

    mvsUtils::printfElapsedTime(timer, "Graph cut post-processing ");
}

void DelaunayGraphCut::invertFullStatusForSmallLabels()
{
    ALICEVISION_LOG_DEBUG("filling small holes");

    const std::size_t nbCells = _cellIsFull.size();
    StaticVector<int> colorPerCell(nbCells, -1);

    StaticVector<int> nbCellsPerColor;
    nbCellsPerColor.reserve(100);
    nbCellsPerColor.resize_with(1, 0);
    int lastColorId = 0;

    StaticVector<CellIndex> buff;
    buff.reserve(nbCells);

    for (CellIndex ci = 0; ci < nbCells; ++ci)
    {
        if (colorPerCell[ci] == -1)
        {
            // backtrack all connected interior cells
            buff.resize(0);
            buff.push_back(ci);

            colorPerCell[ci] = lastColorId;
            nbCellsPerColor[lastColorId] += 1;

            while (buff.size() > 0)
            {
                CellIndex tmp_ci = buff.pop();

                for (int k = 0; k < 4; ++k)
                {
                    const CellIndex nci = _tetrahedralization.cell_adjacent(tmp_ci, k);
                    if (nci == GEO::NO_CELL)
                        continue;
                    if ((colorPerCell[nci] == -1) && (_cellIsFull[nci] == _cellIsFull[ci]))
                    {
                        colorPerCell[nci] = lastColorId;
                        nbCellsPerColor[lastColorId] += 1;
                        buff.push_back(nci);
                    }
                }
            }
            nbCellsPerColor.push_back(0);  // add new color with 0 cell
            ++lastColorId;
            assert(lastColorId == nbCellsPerColor.size() - 1);
        }
    }
    assert(nbCellsPerColor[nbCellsPerColor.size() - 1] == 0);
    nbCellsPerColor.resize(nbCellsPerColor.size() - 1);  // remove last empty element

    int movedToEmpty = 0;
    int movedToFull = 0;
    for (CellIndex ci = 0; ci < nbCells; ++ci)
    {
        if (nbCellsPerColor[colorPerCell[ci]] < 100)
        {
            _cellIsFull[ci] = !_cellIsFull[ci];
            if (_cellIsFull[ci])
                ++movedToFull;
            else
                ++movedToEmpty;
        }
    }

    ALICEVISION_LOG_WARNING("DelaunayGraphCut::invertFullStatusForSmallLabels: " << movedToFull << " cells moved to full, " << movedToEmpty
                                                                                 << " cells moved to empty within " << _cellIsFull.size()
                                                                                 << " cells.");

    ALICEVISION_LOG_DEBUG("Number of labels: " << nbCellsPerColor.size() << ", Number of cells changed: " << movedToFull + movedToEmpty
                                               << ", full number of cells: " << nbCells);
}

void DelaunayGraphCut::cellsStatusFilteringBySolidAngleRatio(int nbSolidAngleFilteringIterations, double minSolidAngleRatio)
{
    if (nbSolidAngleFilteringIterations <= 0 || minSolidAngleRatio <= 0.0)
        return;

    constexpr double fullSphereSolidAngle = 4.0 * boost::math::constants::pi<double>();

    // Change cells status on surface around vertices to improve smoothness
    // using solid angle ratio between full/empty parts.
    for (int i = 0; i < nbSolidAngleFilteringIterations; ++i)
    {
        std::vector<std::uint8_t> cellsInvertStatus(_cellIsFull.size(), false);
        int toInvertCount = 0;

        std::vector<bool> vertexIsOnSurface;
        const int nbSurfaceFacets = computeIsOnSurface(vertexIsOnSurface);

        const auto & neighboringCellsPerVertex = _tetrahedralization.getNeighboringCellsPerVertex();

#pragma omp parallel for reduction(+ : toInvertCount)
        for (int vi = 0; vi < neighboringCellsPerVertex.size(); ++vi)
        {
            if (!vertexIsOnSurface[vi])
            {
                continue;
            }
                
            const std::vector<CellIndex>& neighboringCells = neighboringCellsPerVertex[vi];
            std::vector<Facet> neighboringFacets;
            neighboringFacets.reserve(neighboringCells.size());
            bool borderCase = false;
            double fullPartSolidAngle = 0.0;
            for (CellIndex ci : neighboringCells)
            {
                // ALICEVISION_LOG_INFO("full cell: " << ci);
                std::vector<VertexIndex> triangle;
                triangle.reserve(3);
                GEO::signed_index_t localVertexIndex = 0;
                for (int k = 0; k < 4; ++k)
                {
                    const GEO::signed_index_t currentVertex = _tetrahedralization.cell_vertex(ci, k);
                    if (currentVertex == GEO::NO_VERTEX)
                        break;
                    if (currentVertex != vi)
                        triangle.push_back(currentVertex);
                    else
                        localVertexIndex = k;
                }
                if (triangle.size() != 3)
                {
                    borderCase = true;
                    break;
                }
                {
                    const Facet f(ci, localVertexIndex);
                    neighboringFacets.push_back(f);
                }

                if (_cellIsFull[ci])
                {
                    const Point3d& O = _verticesCoords[vi];
                    const double s =
                      tetrahedronSolidAngle(_verticesCoords[triangle[0]] - O, _verticesCoords[triangle[1]] - O, _verticesCoords[triangle[2]] - O);
                    fullPartSolidAngle += s;
                }
            }
            if (borderCase)
            {
                // we cannot compute empty/full ratio if we have undefined cells
                continue;
            }
            bool invert = false;
            bool invertFull = false;
            
            if (fullPartSolidAngle < minSolidAngleRatio * fullSphereSolidAngle)
            {
                invert = true;
                invertFull = true;  // we want to invert the FULL cells
            }
            else if (fullPartSolidAngle > (1.0 - minSolidAngleRatio) * fullSphereSolidAngle)
            {
                invert = true;
                invertFull = false;  // we want to invert the EMPTY cells
            }
            if (!invert)
                continue;

            // Ensure that we do not increase inconsitencies (like holes).
            // Check the status coherency with neighbor cells if we swap the cells status.
            for (const Facet& f : neighboringFacets)
            {
                if (_cellIsFull[f.cellIndex] == invertFull)
                {
                    const Facet fv = _tetrahedralization.mirrorFacet(f);
                    if (_tetrahedralization.isInvalidOrInfiniteCell(fv.cellIndex) || _cellIsFull[f.cellIndex] != _cellIsFull[fv.cellIndex])
                    {
                        borderCase = true;
                        break;
                    }
                }
            }
            if (borderCase)
                continue;

            // Invert some cells
            for (CellIndex ci : neighboringCells)
            {
                if (_cellIsFull[ci] == invertFull)
                {
                    boost::atomic_ref<std::uint8_t>{cellsInvertStatus[ci]} = true;
                    ++toInvertCount;
                }
            }
        }
        if (toInvertCount == 0)
            break;
        int movedToEmpty = 0;
        int movedToFull = 0;
        for (CellIndex ci = 0; ci < cellsInvertStatus.size(); ++ci)
        {
            if (!cellsInvertStatus[ci])
                continue;
            _cellIsFull[ci] = !_cellIsFull[ci];
            if (_cellIsFull[ci])
                ++movedToFull;
            else
                ++movedToEmpty;
        }
        ALICEVISION_LOG_WARNING("[" << i << "] Check solid angle full/empty ratio on surface vertices: " << movedToFull << " cells moved to full, "
                                    << movedToEmpty << " cells moved to empty within " << _cellIsFull.size() << " cells.");
    }
}

void DelaunayGraphCut::createGraphCut(const Point3d hexah[8], const StaticVector<int>& cams)
{
    voteFullEmptyScore(cams);
    maxflow();
}

void DelaunayGraphCut::addToInfiniteSw(float sW)
{
    std::size_t nbInfinitCells = 0;
    for (CellIndex ci = 0; ci < _cellsAttr.size(); ++ci)
    {
        if (_tetrahedralization.isInfiniteCell(ci))
        {
            GC_cellInfo& c = _cellsAttr[ci];
            c.cellSWeight += sW;
            ++nbInfinitCells;
        }
    }
    ALICEVISION_LOG_WARNING("DelaunayGraphCut::addToInfiniteSw nbInfinitCells: " << nbInfinitCells);
}

void DelaunayGraphCut::maxflow()
{
    long t_maxflow = clock();

    ALICEVISION_LOG_INFO("Maxflow: start allocation.");
    const std::size_t nbCells = _cellsAttr.size();
    ALICEVISION_LOG_INFO("Number of cells: " << nbCells);

    // MaxFlow_CSR maxFlowGraph(nbCells);
    MaxFlow_AdjList maxFlowGraph(nbCells);

    ALICEVISION_LOG_INFO("Maxflow: add nodes.");
    // fill s-t edges
    int nbSCells = 0;
    int nbTCells = 0;
    for (CellIndex ci = 0; ci < nbCells; ++ci)
    {
        const GC_cellInfo& c = _cellsAttr[ci];
        const float ws = c.cellSWeight;
        const float wt = c.cellTWeight;

        assert(ws >= 0.0f);
        assert(wt >= 0.0f);
        assert(!std::isnan(ws));
        assert(!std::isnan(wt));

        maxFlowGraph.addNode(ci, ws, wt);
        if (ws > wt)
            ++nbSCells;
        else
            ++nbTCells;
    }
    ALICEVISION_LOG_INFO("Maxflow: " << nbSCells << " S cells, " << nbTCells << " T cells.");

    ALICEVISION_LOG_INFO("Maxflow: add edges.");
    const float CONSTalphaVIS = 1.0f;
    const float CONSTalphaPHOTO = 5.0f;

    // fill u-v directed edges
    for (CellIndex ci = 0; ci < nbCells; ++ci)
    {
        for (VertexIndex k = 0; k < 4; ++k)
        {
            Facet fu(ci, k);
            Facet fv = _tetrahedralization.mirrorFacet(fu);
            if (_tetrahedralization.isInvalidOrInfiniteCell(fv.cellIndex))
                continue;

            float a1 = 0.0f;
            float a2 = 0.0f;
            if ((!_tetrahedralization.isInfiniteCell(fu.cellIndex)) && (!_tetrahedralization.isInfiniteCell(fv.cellIndex)))
            {
                // Score for each facet based on the quality of the topology
                a1 = _tetrahedralization.getFaceWeight(fu);
                a2 = _tetrahedralization.getFaceWeight(fv);
            }

            // In output of maxflow the cuts will become the surface.
            // High weight on some facets will avoid cutting them.
            float wFvFu = _cellsAttr[fu.cellIndex].gEdgeVisWeight[fu.localVertexIndex] * CONSTalphaVIS + a1 * CONSTalphaPHOTO;
            float wFuFv = _cellsAttr[fv.cellIndex].gEdgeVisWeight[fv.localVertexIndex] * CONSTalphaVIS + a2 * CONSTalphaPHOTO;

            assert(wFvFu >= 0.0f);
            assert(wFuFv >= 0.0f);
            assert(!std::isnan(wFvFu));
            assert(!std::isnan(wFuFv));

            maxFlowGraph.addEdge(fu.cellIndex, fv.cellIndex, wFuFv, wFvFu);
        }
    }

    std::vector<GC_cellInfo>().swap(_cellsAttr);  // force clear to free some RAM before maxflow

    // Find graph-cut solution
    const float totalFlow = maxFlowGraph.compute();

    _cellIsFull.resize(nbCells);
    std::size_t nbFullCells = 0;
    for (CellIndex ci = 0; ci < nbCells; ++ci)
    {
        _cellIsFull[ci] = maxFlowGraph.isTarget(ci);
        nbFullCells += _cellIsFull[ci];
    }
}

void DelaunayGraphCut::voteFullEmptyScore(const StaticVector<int>& cams)
{
    const int maxint = std::numeric_limits<int>::max();

    const double nPixelSizeBehind = _mp.userParams.get<double>("delaunaycut.nPixelSizeBehind", 4.0);  // sigma value
    const float fullWeight = float(_mp.userParams.get<double>("delaunaycut.fullWeight", 1.0));
    const bool forceTEdge = _mp.userParams.get<bool>("delaunaycut.voteFilteringForWeaklySupportedSurfaces", true);

    fillGraph(nPixelSizeBehind, fullWeight);

    addToInfiniteSw((float)maxint);

    if (forceTEdge)
    {
        forceTedgesByGradientIJCV(nPixelSizeBehind);
    }
}

void DelaunayGraphCut::filterLargeHelperPoints(std::vector<bool>& out_reliableVertices, const std::vector<bool>& vertexIsOnSurface, int maxSegSize)
{
    out_reliableVertices.clear();

    // Do not filter helper points if maxSegSize is negative/infinit
    if (maxSegSize < 0)
    {
        return;
    }

    out_reliableVertices.resize(_verticesAttr.size(), true);

    std::vector<Pixel> edges;
    edges.reserve(_verticesAttr.size());

    // Create edges for all connected helper points
    for (VertexIndex vi = 0; vi < _verticesAttr.size(); ++vi)
    {
        const GC_vertexInfo& v = _verticesAttr[vi];
        const Point3d& p = _verticesCoords[vi];
        if (v.nrc <= 0 && vertexIsOnSurface[vi])
        {
            // go through all the neighbouring points
            GEO::vector<VertexIndex> adjVertices;
            _tetrahedralization.get_neighbors(vi, adjVertices);

            for (VertexIndex nvi : adjVertices)
            {
                // ignore itself
                if (vi == nvi)
                    continue;
                // avoid duplicates
                if (vi < nvi)
                    continue;
                // ignore points not on the surface
                if (!vertexIsOnSurface[vi])
                    continue;
                // ignore valid vertices
                const GC_vertexInfo& nv = _verticesAttr[nvi];
                if (nv.nrc > 0)
                    continue;
                // Declare a new edge between 2 helper points both on the selected surface
                edges.emplace_back(vi, nvi);
            }
        }
    }

    Universe u(_verticesAttr.size());

    if (maxSegSize > 0)
    {
        int s = (int)edges.size();
        // Fuse all edges collected to be merged
        for (int i = 0; i < s; i++)
        {
            int a = u.find(edges[i].x);
            int b = u.find(edges[i].y);
            if (a != b)
            {
                u.join(a, b);
            }
        }
    }

    // Last loop over vertices to update segId
    for (int vi = 0; vi < _verticesAttr.size(); ++vi)
    {
        if (!vertexIsOnSurface[vi])
        {
            // Point is not on the surface
            out_reliableVertices[vi] = false;
            continue;
        }
        const GC_vertexInfo& v = _verticesAttr[vi];
        if (v.nrc > 0)
        {
            // This is not a helper point, so it is reliable.
            out_reliableVertices[vi] = true;
            continue;
        }

        if (maxSegSize > 0)
        {
            // It is an helper point, so it is reliable only if it is a small group.
            const int sigId = u.find(vi);
            const int segSize = u.elts[sigId].size;
            out_reliableVertices[vi] = segSize <= maxSegSize;
        }
        else
        {
            // It is an helper point and should be removed.
            out_reliableVertices[vi] = false;
        }
    }
}

mesh::Mesh* DelaunayGraphCut::createMesh(int maxNbConnectedHelperPoints)
{

    std::vector<bool> vertexIsOnSurface;
    const int nbSurfaceFacets = computeIsOnSurface(vertexIsOnSurface);

    mesh::Mesh* me = new mesh::Mesh();
    me->pts = StaticVector<Point3d>();
    me->pts.reserve(_verticesCoords.size());

    for (const Point3d& p : _verticesCoords)
    {
        me->pts.push_back(p);
    }

    std::vector<bool> reliableVertices;
    filterLargeHelperPoints(reliableVertices, vertexIsOnSurface, maxNbConnectedHelperPoints);

    me->tris = StaticVector<mesh::Mesh::triangle>();
    me->tris.reserve(nbSurfaceFacets);

    // loop over all facets
    for (CellIndex ci = 0; ci < _cellIsFull.size(); ++ci)
    {
        for (VertexIndex k = 0; k < 4; ++k)
        {
            Facet f1(ci, k);
            bool uo = _cellIsFull[f1.cellIndex];  // get if it is occupied
            if (!uo)
            {
                // "f1" is in an EMPTY cell, skip it
                continue;
            }

            Facet f2 = _tetrahedralization.mirrorFacet(f1);
            if (_tetrahedralization.isInvalidOrInfiniteCell(f2.cellIndex))
                continue;
            bool vo = _cellIsFull[f2.cellIndex];  // get if it is occupied

            if (vo)
            {
                // "f2" is in a FULL cell, skip it
                continue;
            }

            // "f1" is in a FULL cell and "f2" is in an EMPTY cell

            VertexIndex vertices[3];
            vertices[0] = _tetrahedralization.getVertexIndex(f1, 0);
            vertices[1] = _tetrahedralization.getVertexIndex(f1, 1);
            vertices[2] = _tetrahedralization.getVertexIndex(f1, 2);

            if (!reliableVertices.empty())
            {
                // We skip triangles if it contains one unreliable vertex.
                bool invalidTriangle = false;
                for (int k = 0; k < 3; ++k)
                {
                    if (!reliableVertices[vertices[k]])
                    {
                        invalidTriangle = true;
                        break;
                    }
                }
                if (invalidTriangle)
                    continue;
            }

            Point3d points[3];
            for (int k = 0; k < 3; ++k)
            {
                points[k] = _verticesCoords[vertices[k]];
            }

            const Point3d D1 = _verticesCoords[_tetrahedralization.getOppositeVertexIndex(f1)];  // in FULL part
            const Point3d D2 = _verticesCoords[_tetrahedralization.getOppositeVertexIndex(f2)];  // in EMPTY part

            const Point3d N = cross((points[1] - points[0]).normalize(), (points[2] - points[0]).normalize()).normalize();

            const double dd1 = orientedPointPlaneDistance(D1, points[0], N);

            const bool clockwise = std::signbit(dd1);

            if (clockwise)
            {
                mesh::Mesh::triangle t;
                t.alive = true;
                t.v[0] = vertices[0];
                t.v[1] = vertices[1];
                t.v[2] = vertices[2];
                me->tris.push_back(t);
            }
            else
            {
                mesh::Mesh::triangle t;
                t.alive = true;
                t.v[0] = vertices[0];
                t.v[1] = vertices[2];
                t.v[2] = vertices[1];
                me->tris.push_back(t);
            }
        }
    }

    return me;
}

void DelaunayGraphCut::segmentFullOrFree(bool full, StaticVector<int>& out_fullSegsColor, int& out_nsegments)
{
    out_fullSegsColor.clear();
    out_fullSegsColor.reserve(_cellIsFull.size());
    out_fullSegsColor.resize_with(_cellIsFull.size(), -1);

    StaticVector<CellIndex> buff;
    buff.reserve(_cellIsFull.size());
    int col = 0;

    // segment connected free space
    for (CellIndex ci = 0; ci < _cellIsFull.size(); ++ci)
    {
        if ((!_tetrahedralization.isInfiniteCell(ci)) && (out_fullSegsColor[ci] == -1) && (_cellIsFull[ci] == full))
        {
            // backtrack all connected interior cells
            buff.resize(0);
            buff.push_back(ci);

            while (buff.size() > 0)
            {
                CellIndex tmp_ci = buff.pop();

                out_fullSegsColor[tmp_ci] = col;

                for (int k = 0; k < 4; ++k)
                {
                    const CellIndex nci = _tetrahedralization.cell_adjacent(tmp_ci, k);
                    if (nci == GEO::NO_CELL)
                        continue;
                    if ((!_tetrahedralization.isInfiniteCell(nci)) && (out_fullSegsColor[nci] == -1) && (_cellIsFull[nci] == full))
                    {
                        buff.push_back(nci);
                    }
                }
            }
            ++col;
        }
    }

    out_nsegments = col;
}

int DelaunayGraphCut::removeBubbles()
{
    int nbEmptySegments = 0;
    StaticVector<int> emptySegColors;
    segmentFullOrFree(false, emptySegColors, nbEmptySegments);

    StaticVectorBool colorsToFill;
    colorsToFill.reserve(nbEmptySegments);
    // all free space segments which contains camera has to remain free all others full
    colorsToFill.resize_with(nbEmptySegments, true);

    // all free space segments which contains camera has to remain free
    for (CellIndex ci = 0; ci < _cellIsFull.size(); ++ci)
    {
        if (_tetrahedralization.isInfiniteCell(ci) || emptySegColors[ci] < 0)
            continue;

        const GC_vertexInfo& a = _verticesAttr[_tetrahedralization.cell_vertex(ci, 0)];
        const GC_vertexInfo& b = _verticesAttr[_tetrahedralization.cell_vertex(ci, 1)];
        const GC_vertexInfo& c = _verticesAttr[_tetrahedralization.cell_vertex(ci, 2)];
        const GC_vertexInfo& d = _verticesAttr[_tetrahedralization.cell_vertex(ci, 3)];
        if (a.isVirtual() || b.isVirtual() || c.isVirtual() || d.isVirtual())
        {
            // TODO FACA: check helper points are not connected to cameras?
            colorsToFill[emptySegColors[ci]] = false;
        }
    }

    int nbBubbles = 0;
    for (int i = 0; i < nbEmptySegments; ++i)
    {
        if (colorsToFill[i])
        {
            ++nbBubbles;
        }
    }

    int nbModifiedCells = 0;
    for (CellIndex ci = 0; ci < _cellIsFull.size(); ++ci)
    {
        if ((!_tetrahedralization.isInfiniteCell(ci)) && (emptySegColors[ci] >= 0) && (colorsToFill[emptySegColors[ci]]))
        {
            _cellIsFull[ci] = true;
            ++nbModifiedCells;
        }
    }

    return nbBubbles;
}

int DelaunayGraphCut::removeDust(int minSegSize)
{
    int nbFullSegments = 0;
    StaticVector<int> fullSegsColor;
    segmentFullOrFree(true, fullSegsColor, nbFullSegments);

    StaticVector<int> colorsSize(nbFullSegments, 0);

    // all free space segments which contains camera has to remain free
    for (CellIndex ci = 0; ci < _cellIsFull.size(); ++ci)
    {
        if (fullSegsColor[ci] >= 0)  // if we have a valid color: non empty and non infinit cell
        {
            colorsSize[fullSegsColor[ci]] += 1;  // count the number of cells in the segment
        }
    }

    int ndust = 0;
    for (CellIndex ci = 0; ci < _cellIsFull.size(); ++ci)
    {
        // if number of cells in the segment is too small, we change the status to "empty"
        if ((fullSegsColor[ci] >= 0) && (colorsSize[fullSegsColor[ci]] < minSegSize))
        {
            _cellIsFull[ci] = false;
            ++ndust;
        }
    }

    return ndust;
}


}  // namespace fuseCut
}  // namespace aliceVision
