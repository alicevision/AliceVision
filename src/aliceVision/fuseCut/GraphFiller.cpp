// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "GraphFiller.hpp"

#include <aliceVision/mvsUtils/common.hpp>
#include <aliceVision/fuseCut/Intersections.hpp>
#include <aliceVision/fuseCut/MaxFlow_AdjList.hpp>

#include <boost/atomic/atomic_ref.hpp>


namespace aliceVision {
namespace fuseCut {

GraphFiller::GraphFiller(mvsUtils::MultiViewParams& mp, 
                        const PointCloud & pc, 
                        const Tetrahedralization & tetrahedralization)
  : 
  _mp(mp),
  _verticesCoords(pc.getVertices()),
  _verticesAttr(pc.getVerticesAttrs()),
  _camsVertexes(pc.getCameraIndices()),
  _tetrahedralization(tetrahedralization)
{
    initCells();
}

void GraphFiller::initCells()
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

void GraphFiller::build(const StaticVector<int>& cams)
{
    const int maxint = std::numeric_limits<int>::max();

    const double nPixelSizeBehind = _mp.userParams.get<double>("delaunaycut.nPixelSizeBehind", 4.0);
    const float fullWeight = float(_mp.userParams.get<double>("delaunaycut.fullWeight", 1.0));
    const bool forceTEdge = _mp.userParams.get<bool>("delaunaycut.voteFilteringForWeaklySupportedSurfaces", true);

    addToInfiniteSw((float)maxint);

    fillGraph(nPixelSizeBehind, fullWeight);

    if (forceTEdge)
    {
        forceTedgesByGradientIJCV(nPixelSizeBehind);
    }
}

void GraphFiller::addToInfiniteSw(float sW)
{
    for (CellIndex ci = 0; ci < _cellsAttr.size(); ++ci)
    {
        if (_tetrahedralization.isInfiniteCell(ci))
        {
            GC_cellInfo& c = _cellsAttr[ci];
            c.cellSWeight += sW;
        }
    }
}

void GraphFiller::fillGraph(double nPixelSizeBehind, float fullWeight)
{
    ALICEVISION_LOG_INFO("Computing s-t graph weights.");

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
            rayMarchingGraphEmpty(vertexIndex, v.cams[c], weight);
            rayMarchingGraphFull(vertexIndex, v.cams[c], weight* fullWeight, nPixelSizeBehind);
        }
    }
}

void GraphFiller::rayMarchingGraphEmpty(int vertexIndex,
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

void GraphFiller::rayMarchingGraphFull(int vertexIndex,
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

void GraphFiller::forceTedgesByGradientIJCV(float nPixelSizeBehind)
{
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
        {
            continue;
        }

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
}

std::vector<CellIndex> GraphFiller::getNeighboringCellsByGeometry(const GeometryIntersection& g) const
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

void GraphFiller::binarize()
{
    const std::size_t nbCells = _cellsAttr.size();

    MaxFlow_AdjList maxFlowGraph(nbCells);

    // fill s-t edges
    for (CellIndex ci = 0; ci < nbCells; ++ci)
    {
        const GC_cellInfo& c = _cellsAttr[ci];
        const float ws = c.cellSWeight;
        const float wt = c.cellTWeight;

        maxFlowGraph.addNode(ci, ws, wt);
    }

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
            {
                continue;
            }

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

            maxFlowGraph.addEdge(fu.cellIndex, fv.cellIndex, wFuFv, wFvFu);
        }
    }

    //Clear graph
    _cellsAttr.clear();

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

}
}