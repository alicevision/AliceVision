// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "GraphFiller.hpp"

#include <boost/atomic/atomic_ref.hpp>
#include <Eigen/Dense>


namespace aliceVision {
namespace fuseCut {



void GraphFiller::createGraphCut(const std::vector<RayInfo> & rayInfos, const Node & node, std::set<std::pair<fuseCut::CellIndex, fuseCut::VertexIndex>> & visited)
{    
    voteFullEmptyScore(rayInfos, node, visited);
}

void GraphFiller::voteFullEmptyScore(const std::vector<RayInfo> & rayInfos, const Node & node, std::set<std::pair<fuseCut::CellIndex, fuseCut::VertexIndex>> & visited)
{
    ALICEVISION_LOG_INFO("DelaunayGraphCut::voteFullEmptyScore");
    const int maxint = std::numeric_limits<int>::max();


    const bool forceTEdge = _mp.userParams.get<bool>("delaunaycut.voteFilteringForWeaklySupportedSurfaces", true);
    const float fullWeight = float(_mp.userParams.get<double>("delaunaycut.fullWeight", 1.0));

    // 0 for distFcn equals 1 all the time
    const float distFcnHeight = (float)_mp.userParams.get<double>("delaunaycut.distFcnHeight", 0.0);

    fillGraph(rayInfos, true, distFcnHeight, fullWeight, node, visited);
    addToInfiniteSw((float)maxint);    
}

void GraphFiller::addToInfiniteSw(float sW)
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
}

void GraphFiller::forceTedgesByGradientIJCV(const std::vector<RayInfo> & rayInfos, const Node & node)
{
    const double nPixelSizeBehind = _mp.userParams.get<double>("delaunaycut.nPixelSizeBehind", 4.0);  // sigma value
    const float forceTEdgeDelta = (float)_mp.userParams.get<double>("delaunaycut.forceTEdgeDelta", 0.1f);
    const float minJumpPartRange = (float)_mp.userParams.get<double>("delaunaycut.minJumpPartRange", 10000.0f);
    const float maxSilentPartRange = (float)_mp.userParams.get<double>("delaunaycut.maxSilentPartRange", 100.0f);
    const float nsigmaJumpPart = (float)_mp.userParams.get<double>("delaunaycut.nsigmaJumpPart", 4.0f);
    const float nsigmaFrontSilentPart = (float)_mp.userParams.get<double>("delaunaycut.nsigmaFrontSilentPart", 2.0f);
    const float nsigmaBackSilentPart = (float)_mp.userParams.get<double>("delaunaycut.nsigmaBackSilentPart", 2.0f);


    const double marginEpsilonFactor = 1.0e-4;


#pragma omp parallel for 
    for (int i = 0; i < rayInfos.size(); ++i)
    {
        

        const int vertexIndex = rayInfos[i].end;
        const GC_vertexInfo& v = _verticesAttr[vertexIndex];
        if (v.isVirtual())
            continue;

        const Point3d& originPt = _verticesCoords[vertexIndex];
        // For each camera that has visibility over the vertex v (vertexIndex)
        
        int cam = rayInfos[i].start;
        {
            GeometriesCount geometriesIntersectedFrontCount;
            GeometriesCount geometriesIntersectedBehindCount;

            const float maxDist = nPixelSizeBehind * _mp.getCamPixelSize(originPt, cam);

            // float minJump = 10000000.0f;
            // float minSilent = 10000000.0f;
            float maxJump = 0.0f;
            float maxSilent = 0.0f;
            float midSilent = 10000000.0f;

            {
                // Initialisation
                GeometryIntersection geometry(vertexIndex);  // Starting on global vertex index
                Point3d intersectPt = originPt;
                // toTheCam
                const Point3d dirVect = (_mp.CArr[cam] - originPt).normalize();

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


                    geometry = intersectNextGeom(previousGeometry, originPt, dirVect, intersectPt, marginEpsilonFactor, lastIntersectPt);

                    if (geometry.type == EGeometryType::None)
                    {
                        break;
                    }

                    if ((intersectPt - originPt).size() <= (lastIntersectPt - originPt).size())
                    {
                        // Inverse direction, stop
                        break;
                    }


                    if (geometry.type == EGeometryType::Facet)
                    {
                        ++geometriesIntersectedFrontCount.facets;
                        const GC_cellInfo& c = _cellsAttr[geometry.facet.cellIndex];
                        if ((lastIntersectPt - originPt).size() > nsigmaFrontSilentPart * maxDist)  // (p-originPt).size() > 2 * sigma
                        {
                            maxJump = std::max(maxJump, c.emptinessScore);
                        }
                        else
                        {
                            maxSilent = std::max(maxSilent, c.emptinessScore);
                        }

                        // Take the mirror facet to iterate over the next cell
                        const Facet mFacet = mirrorFacet(geometry.facet);
                        if (_tetrahedralization.isInvalidOrInfiniteCell(mFacet.cellIndex))
                        {
                            break;
                        }

                        geometry.facet = mFacet;
                        if (previousGeometry.type == EGeometryType::Facet && geometriesIntersectedFrontCount.facets > 10000)
                        {
                            break;
                        }
                    }
                    else if (geometry.type == EGeometryType::Vertex)
                    {
                        ++geometriesIntersectedFrontCount.vertices;
                        if (previousGeometry.type == EGeometryType::Vertex && geometriesIntersectedFrontCount.vertices > 1000)
                        {
                            break;
                        }
                    }
                    else if (geometry.type == EGeometryType::Edge)
                    {
                        ++geometriesIntersectedFrontCount.edges;
                        if (previousGeometry.type == EGeometryType::Edge && geometriesIntersectedFrontCount.edges > 1000)
                        {
                            break;
                        }
                    }
                }
            }
            {
                // Initialisation
                GeometryIntersection geometry(vertexIndex);
                Point3d intersectPt = originPt;
                // behindThePoint
                const Point3d dirVect = (originPt - _mp.CArr[cam]).normalize();


                Facet lastIntersectedFacet;
                bool firstIteration = true;
                Point3d lastIntersectPt = originPt;

                // While we are within the surface margin defined by maxDist (as long as we find a next geometry)
                while ((lastIntersectPt - originPt).size() <= nsigmaBackSilentPart * maxDist)
                {
                    // Keep previous informations
                    const GeometryIntersection previousGeometry = geometry;
                    lastIntersectPt = intersectPt;


                    geometry = intersectNextGeom(previousGeometry, originPt, dirVect, intersectPt, marginEpsilonFactor, lastIntersectPt);

                    if (geometry.type == EGeometryType::None)
                    {
                        break;
                    }

                    if ((intersectPt - originPt).size() <= (lastIntersectPt - originPt).size())
                    {
                        break;
                    }
                    if (geometry.type == EGeometryType::Facet)
                    {
                        ++geometriesIntersectedBehindCount.facets;

                        // Vote for the first cell found (only once)
                        if (firstIteration)
                        {
                            midSilent = _cellsAttr[geometry.facet.cellIndex].emptinessScore;
                            firstIteration = false;
                        }

                        const GC_cellInfo& c = _cellsAttr[geometry.facet.cellIndex];
                        maxSilent = std::max(maxSilent, c.emptinessScore);

                        // Take the mirror facet to iterate over the next cell
                        const Facet mFacet = mirrorFacet(geometry.facet);
                        lastIntersectedFacet = mFacet;
                        geometry.facet = mFacet;
                        if (_tetrahedralization.isInvalidOrInfiniteCell(mFacet.cellIndex))
                        {
                            break;
                        }

                        if (previousGeometry.type == EGeometryType::Facet && geometriesIntersectedBehindCount.facets > 1000)
                        {
                            break;
                        }
                    }
                    else
                    {
                        // Vote for the first cell found (only once)
                        // if we come from an edge or vertex to an other we have to vote for the first intersected cell.
                        if (firstIteration)
                        {
                            if (previousGeometry.type != EGeometryType::Vertex)
                            {
                            }

                            const std::vector<CellIndex> previousNeighbouring = getNeighboringCellsByVertexIndex(previousGeometry.vertexIndex);
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

                        if (geometry.type == EGeometryType::Vertex)
                        {
                            ++geometriesIntersectedBehindCount.vertices;
                            if (previousGeometry.type == EGeometryType::Vertex && geometriesIntersectedBehindCount.vertices > 1000)
                            {
                                break;
                            }
                        }
                        else if (geometry.type == EGeometryType::Edge)
                        {
                            ++geometriesIntersectedBehindCount.edges;
                            if (previousGeometry.type == EGeometryType::Edge && geometriesIntersectedBehindCount.edges > 1000)
                            {
                                break;
                            }
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
}

GeometryIntersection GraphFiller::rayIntersectTriangle(const Point3d& originPt,
                                                                              const Point3d& DirVec,
                                                                              const Facet& facet,
                                                                              Point3d& intersectPt,
                                                                              const double epsilonFactor,
                                                                              bool& ambiguous,
                                                                              const Point3d* lastIntersectPt) const
{
    ambiguous = false;

    const VertexIndex AvertexIndex = getVertexIndex(facet, 0);
    const VertexIndex BvertexIndex = getVertexIndex(facet, 1);
    const VertexIndex CvertexIndex = getVertexIndex(facet, 2);

    const Point3d* A = &_verticesCoords[AvertexIndex];
    const Point3d* B = &_verticesCoords[BvertexIndex];
    const Point3d* C = &_verticesCoords[CvertexIndex];

    const double ABSize = (*A - *B).size();
    const double BCSize = (*B - *C).size();
    const double ACSize = (*A - *C).size();

    const double marginEpsilon = std::min({ABSize, BCSize, ACSize}) * epsilonFactor;
    const double ambiguityEpsilon = (ABSize + BCSize + ACSize) / 3.0 * 1.0e-2;

    Point3d tempIntersectPt;
    const Point2d triangleUv = getLineTriangleIntersectBarycCoords(&tempIntersectPt, A, B, C, &originPt, &DirVec);

    if (!std::isnormal(tempIntersectPt.x) || !std::isnormal(tempIntersectPt.y) || !std::isnormal(tempIntersectPt.z))
    {
        // This is not suppose to happen in real life, we log a warning instead of raising an exeption if we face a border case
        // ALICEVISION_LOG_WARNING("Invalid/notNormal intersection point found during rayIntersectTriangle.");
        return GeometryIntersection();
    }

    const double u = triangleUv.x;  // A to C
    const double v = triangleUv.y;  // A to B

    // If we find invalid uv coordinate
    if (!std::isfinite(u) || !std::isfinite(v))
        return GeometryIntersection();

    // Ouside the triangle with marginEpsilon margin
    if (u < -marginEpsilon || v < -marginEpsilon || (u + v) > (1.0 + marginEpsilon))
        return GeometryIntersection();

    // In case intersectPt is provided, check if intersectPt is in front of lastIntersectionPt
    // in the DirVec direction to ensure that we are moving forward in the right direction
    if (lastIntersectPt != nullptr)
    {
        const Point3d diff = tempIntersectPt - *lastIntersectPt;
        const double dotValue = dot(DirVec, diff.normalize());
        if (dotValue < marginEpsilon || diff.size() < 100 * std::numeric_limits<double>::min())
        {
            return GeometryIntersection();
        }

        if (diff.size() < ambiguityEpsilon)
        {
            ambiguous = true;
        }
    }

    // Change intersection point only if tempIntersectionPt is in the right direction (mean we intersect something)
    intersectPt = tempIntersectPt;

    if (v < marginEpsilon)  // along A C edge
    {
        if (u < marginEpsilon)
        {
            intersectPt = *A;
            return GeometryIntersection(AvertexIndex);  // vertex A
        }
        if (u > 1.0 - marginEpsilon)
        {
            intersectPt = *C;
            return GeometryIntersection(CvertexIndex);  // vertex C
        }

        return GeometryIntersection(Edge(AvertexIndex, CvertexIndex));  // edge AC
    }

    if (u < marginEpsilon)  // along A B edge
    {
        if (v > 1.0 - marginEpsilon)
        {
            intersectPt = *B;
            return GeometryIntersection(BvertexIndex);  // vertex B
        }

        return GeometryIntersection(Edge(AvertexIndex, BvertexIndex));  // edge AB
    }

    if (u + v > 1.0 - marginEpsilon)
        return GeometryIntersection(Edge(BvertexIndex, CvertexIndex));  // edge BC

    return GeometryIntersection(facet);
}

GeometryIntersection GraphFiller::intersectNextGeom(const GeometryIntersection& inGeometry,
                                                                           const Point3d& originPt,
                                                                           const Point3d& dirVect,
                                                                           Point3d& intersectPt,
                                                                           const double epsilonFactor,
                                                                           const Point3d& lastIntersectPt) const
{
    GeometryIntersection bestMatch;
    Point3d bestMatchIntersectPt;

    switch (inGeometry.type)
    {
        case EGeometryType::Facet:
        {
            const CellIndex tetrahedronIndex = inGeometry.facet.cellIndex;

            // Test all facets of the tetrahedron using i as localVertexIndex to define next intersectionFacet
            for (int i = 0; i < 4; ++i)
            {
                // Because we can't intersect with incoming facet (same localVertexIndex)
                if (i == inGeometry.facet.localVertexIndex)
                {
                    continue;
                }

                const Facet intersectionFacet(tetrahedronIndex, i);
                bool ambiguous = false;

                const GeometryIntersection result =
                  rayIntersectTriangle(originPt, dirVect, intersectionFacet, intersectPt, epsilonFactor, ambiguous, &lastIntersectPt);
                if (result.type != EGeometryType::None)
                {
                    if (!ambiguous)
                    {
                        return result;
                    }

                    // Retrieve the best intersected point (farthest from origin point)
                    if (bestMatch.type == EGeometryType::None || (originPt - intersectPt).size() > (originPt - bestMatchIntersectPt).size())
                    {
                        bestMatchIntersectPt = intersectPt;
                        bestMatch = result;
                    }
                }
            }
        }
        break;

        case EGeometryType::Vertex:
        {
            for (CellIndex adjCellIndex : getNeighboringCellsByVertexIndex(inGeometry.vertexIndex))
            {
                if (_tetrahedralization.isInvalidOrInfiniteCell(adjCellIndex))
                    continue;

                // Get local vertex index
                const VertexIndex localVertexIndex = _tetrahedralization.index(adjCellIndex, inGeometry.vertexIndex);

                // Define the facet to intersect
                const Facet facet(adjCellIndex, localVertexIndex);
                bool ambiguous = false;

                const GeometryIntersection result =
                  rayIntersectTriangle(originPt, dirVect, facet, intersectPt, epsilonFactor, ambiguous, &lastIntersectPt);
                if (result.type != EGeometryType::None)
                {
                    if (!ambiguous)
                        return result;

                    // Retrieve the best intersected point (farthest from origin point)
                    if (bestMatch.type == EGeometryType::None || (originPt - intersectPt).size() > (originPt - bestMatchIntersectPt).size())
                    {
                        bestMatchIntersectPt = intersectPt;
                        bestMatch = result;
                    }
                }
            }
        }
        break;

        case EGeometryType::Edge:
        {
            GeometryIntersection result;

            for (CellIndex adjCellIndex : getNeighboringCellsByEdge(inGeometry.edge))
            {
                if (_tetrahedralization.isInvalidOrInfiniteCell(adjCellIndex))
                    continue;
                // Local vertices indices
                const VertexIndex lvi0 = _tetrahedralization.index(adjCellIndex, inGeometry.edge.v0);
                const VertexIndex lvi1 = _tetrahedralization.index(adjCellIndex, inGeometry.edge.v1);

                // The two facets that do not touch this edge need to be tested
                const std::array<Facet, 2> opositeFacets{{{adjCellIndex, lvi0}, {adjCellIndex, lvi1}}};

                for (const Facet& facet : opositeFacets)
                {
                    bool ambiguous = false;
                    const GeometryIntersection result =
                      rayIntersectTriangle(originPt, dirVect, facet, intersectPt, epsilonFactor, ambiguous, &lastIntersectPt);

                    if (result.type == EGeometryType::Edge)
                    {
                        if (result.edge.isSameUndirectionalEdge(inGeometry.edge))
                        {
                            continue;
                        }
                    }
                    if (result.type != EGeometryType::None)
                    {
                        if (!ambiguous)
                            return result;

                        // Retrieve the best intersected point (farthest from origin point)
                        if (bestMatch.type == EGeometryType::None || (originPt - intersectPt).size() > (originPt - bestMatchIntersectPt).size())
                        {
                            bestMatchIntersectPt = intersectPt;
                            bestMatch = result;
                        }
                    }
                }
            }
        }
        break;

        case EGeometryType::None:
            // throw std::runtime_error("[intersectNextGeom] intersection with input none geometry should not happen.");
            break;
    }

    intersectPt = bestMatchIntersectPt;
    return bestMatch;
}

std::vector<CellIndex> GraphFiller::getNeighboringCellsByGeometry(const GeometryIntersection& g) const
{
    switch (g.type)
    {
        case EGeometryType::Edge:
            return getNeighboringCellsByEdge(g.edge);
        case EGeometryType::Vertex:
            return getNeighboringCellsByVertexIndex(g.vertexIndex);
        case EGeometryType::Facet:
            return getNeighboringCellsByFacet(g.facet);
        case EGeometryType::None:
            break;
    }
    throw std::runtime_error("[error] getNeighboringCellsByGeometry: an undefined/None geometry has no neighboring cells.");
}

std::vector<CellIndex> GraphFiller::getNeighboringCellsByFacet(const Facet& f) const
{
    std::vector<CellIndex> neighboringCells;
    neighboringCells.push_back(f.cellIndex);

    const Facet mFacet = mirrorFacet(f);
    if (!_tetrahedralization.isInvalidOrInfiniteCell(mFacet.cellIndex))
        neighboringCells.push_back(mFacet.cellIndex);

    return neighboringCells;
}

void GraphFiller::fillGraph(const std::vector<RayInfo> & rayInfos,

                                 bool fillOut,
                                 float distFcnHeight,
                                 float fullWeight, const Node & node, std::set<std::pair<fuseCut::CellIndex, fuseCut::VertexIndex>> & visited)  // nPixelSizeBehind=2*spaceSteps allPoints=1 behind=0
                                                    // labatutWeights=0 fillOut=1 distFcnHeight=0
{
    ALICEVISION_LOG_INFO("Computing s-t graph weights.");
    long t1 = clock();

    const double nPixelSizeBehind = _mp.userParams.get<double>("delaunaycut.nPixelSizeBehind", 4.0);  // sigma value


#pragma omp parallel for
    for (int i = 0; i < rayInfos.size(); i++)
    {
        const int vertexIndex = rayInfos[i].end;
        const GC_vertexInfo& v = _verticesAttr[vertexIndex];

        if (v.isReal())
        {
            // "weight" is called alpha(p) in the paper
            float weight = (float)v.nrc;  // number of cameras

            //Overwrite with forced weight if available
            weight = (float)_mp.userParams.get<double>("LargeScale.forceWeight", weight);

            {
                

                int stepsFront = 0;
                int stepsBehind = 0;
                fillGraphPartPtRc(stepsFront,
                                  stepsBehind,
                                  vertexIndex,
                                  rayInfos[i].start,
                                  weight,
                                  fullWeight,
                                  nPixelSizeBehind,
                                  fillOut,
                                  distFcnHeight, node, visited);
            }  // for c
        }
    }

}

void GraphFiller::fillGraphPartPtRc(int& outTotalStepsFront,
                                         int& outTotalStepsBehind,
                                         int vertexIndex,
                                         int cam,
                                         float weight,
                                         float fullWeight,
                                         double nPixelSizeBehind,
                                         bool fillOut,
                                         float distFcnHeight, const Node & node, 
                                         std::set<std::pair<fuseCut::CellIndex, fuseCut::VertexIndex>> & visited)  // nPixelSizeBehind=2*spaceSteps allPoints=1 behind=0 fillOut=1 distFcnHeight=0
{
    GeometriesCount frontCount;
    GeometriesCount behindCount;

    const int maxint = std::numeric_limits<int>::max();
    const double marginEpsilonFactor = 1.0e-4;

    const Point3d& originPt = _verticesCoords[vertexIndex];
    double pixSize = _verticesAttr[vertexIndex].pixSize;
    const double maxDist = nPixelSizeBehind * pixSize;

   
    
    assert(cam >= 0);
    assert(cam < _mp.ncams);

    Eigen::Vector3d pt;
    pt.x() = originPt.x;
    pt.y() = originPt.y;
    pt.z() = originPt.z;
    
    Eigen::Vector3d campt;
    campt.x() = _mp.CArr[cam].x;
    campt.y() = _mp.CArr[cam].y;
    campt.z() = _mp.CArr[cam].z;
    
    Point3d sourcePt = originPt;
    GeometryIntersection geometry(vertexIndex);

    bool cameraVisible = node.isInside(campt);

    bool vertexVisible = false;
    if (!node.isInside(pt))
    {
        //Ignore fillinside
        pixSize = -10.0; 

        Eigen::Vector3d outpt;
        bool res = node.getPointLeaving(campt, pt, outpt);
        if (!res)
        {
            ALICEVISION_LOG_ERROR("Invalid");
        }

        CellIndex ci = GEO::NO_CELL;
        if (!_tetrahedralization.locate(_verticesCoords, outpt, ci))
        {
            ALICEVISION_LOG_ERROR("Cannot find node starting cell");
            return;
        }
        

        if (!_tetrahedralization.isInside(ci, _verticesCoords, outpt))
        {
            ALICEVISION_LOG_ERROR("Invalid node starting cell");

            for (CellIndex cid = 0; cid < _tetrahedralization.nb_cells(); cid++)
            {
                if (_tetrahedralization.isInside(cid, _verticesCoords, outpt))
                {
                    ALICEVISION_LOG_ERROR("found instead " << cid << " " << ci << " " << GEO::NO_CELL);
                }
            }
            
            return;
        }

        sourcePt.x = outpt.x();
        sourcePt.y = outpt.y();
        sourcePt.z = outpt.z();
        vertexVisible = true;

        geometry = GeometryIntersection(Facet(ci, -1));
    }

    Eigen::Vector3d source;
    source.x() = sourcePt.x;
    source.y() = sourcePt.y;
    source.z() = sourcePt.z;

    Eigen::Vector3d dir = (campt - source).normalized();

    TetrahedronsRayMarching marching(_tetrahedralization, _verticesCoords, source, dir);
    
    if (fillOut)  // EMPTY part
    {
        // Initialisation
          // Starting on global vertex index
        Point3d intersectPt = sourcePt;
        // toTheCam
        const double pointCamDistance = (_mp.CArr[cam] - sourcePt).size();
        const Point3d dirVect = (_mp.CArr[cam] - sourcePt).normalize();


        outTotalStepsFront = 0;
        Facet lastIntersectedFacet;
        bool lastGeoIsVertex = false;
        
        bool wasFullyInside = false;
        bool entrance = true;

        // Break only when we reach our camera vertex (as long as we find a next geometry)
        while (geometry.type != EGeometryType::Vertex || (_mp.CArr[cam] - intersectPt).size() >= 1.0e-3)
        {
            lastGeoIsVertex = false;
            // Keep previous informations
            const GeometryIntersection previousGeometry = geometry;
            const Point3d lastIntersectPt = intersectPt;


            ++outTotalStepsFront;


            Eigen::Vector3d ipt, lipt;
            lipt.x() = lastIntersectPt.x;
            lipt.y() = lastIntersectPt.y;
            lipt.z() = lastIntersectPt.z;

            ipt.x() = intersectPt.x;
            ipt.y() = intersectPt.y;
            ipt.z() = intersectPt.z;
            
            geometry = marching.intersectNextGeom(previousGeometry, ipt, lipt);
            

            intersectPt.x = ipt.x();
            intersectPt.y = ipt.y();
            intersectPt.z = ipt.z();

            if (geometry.type == EGeometryType::None)
            {
                break;
            }

           
            
            ipt.x() = intersectPt.x;
            ipt.y() = intersectPt.y;
            ipt.z() = intersectPt.z;
            if (!node.isInside(ipt))
            {
                break;
            }

            if ((intersectPt - sourcePt).size() <= (lastIntersectPt - sourcePt).size())
            {
                // Inverse direction, stop
                break;
            }



            if (geometry.type == EGeometryType::Facet)
            {
                ++frontCount.facets;
                boost::atomic_ref<float>{_cellsAttr[geometry.facet.cellIndex].emptinessScore} += weight;
                
                const float dist = distFcn(maxDist, (sourcePt - lastIntersectPt).size(), distFcnHeight);
                boost::atomic_ref<float>{_cellsAttr[geometry.facet.cellIndex].gEdgeVisWeight[geometry.facet.localVertexIndex]} += weight * dist;

                // Take the mirror facet to iterate over the next cell
                const Facet mFacet = mirrorFacet(geometry.facet);
                geometry.facet = mFacet;
                if (_tetrahedralization.isInvalidOrInfiniteCell(mFacet.cellIndex))
                {
                    break;
                }
                
                lastIntersectedFacet = mFacet;
                if (previousGeometry.type == EGeometryType::Facet && frontCount.facets > 10000)
                {
                    break;
                }
            }
            else
            {
                // We have just intersected a vertex or edge.
                // These geometries do not have a cellIndex, so we use the previousGeometry to retrieve the cell between the previous geometry and the
                // current one.
                if (previousGeometry.type == EGeometryType::Facet)
                {
                    boost::atomic_ref<float>{_cellsAttr[previousGeometry.facet.cellIndex].emptinessScore} += weight;
                }

                if (geometry.type == EGeometryType::Vertex)
                {
                    ++frontCount.vertices;
                    lastGeoIsVertex = true;
                    if (previousGeometry.type == EGeometryType::Vertex && frontCount.vertices > 1000)
                    {
                        break;
                    }
                }
                else if (geometry.type == EGeometryType::Edge)
                {
                    ++frontCount.edges;
                    if (previousGeometry.type == EGeometryType::Edge && frontCount.edges > 1000)
                    {
                        break;
                    }
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
            // if(lastGeoIsVertex)
            {
                // lastGeoIsVertex is supposed to be positive in almost all cases.
                // If we do not reach the camera, we still vote on the last tetrehedra.
                // Possible reaisons: the camera is not part of the vertices or we encounter a numerical error in intersectNextGeom
                boost::atomic_ref<float>{_cellsAttr[lastIntersectedFacet.cellIndex].cellSWeight} = (float)maxint;
            }
        }
    }

    if (pixSize > 0.0f)  // fillIn FULL part
    {
        const float fWeight = fullWeight * weight;
        // Initialisation
        GeometryIntersection geometry(vertexIndex);  // Starting on global vertex index
        Point3d intersectPt = originPt;
        // behindThePoint
        const Point3d dirVect = (originPt - _mp.CArr[cam]).normalize();
        outTotalStepsBehind = 0;

        bool found = false;
        bool firstIteration = true;
        Facet lastIntersectedFacet;
        // While we are within the surface margin (as long as we find a next geometry)
        while ((originPt - intersectPt).size() < maxDist)
        {
            // Keep previous informations
            const GeometryIntersection previousGeometry = geometry;
            const Point3d lastIntersectPt = intersectPt;


            ++outTotalStepsBehind;

            geometry = intersectNextGeom(previousGeometry, originPt, dirVect, intersectPt, marginEpsilonFactor, lastIntersectPt);
            if (geometry.type == EGeometryType::None)
            {
                ALICEVISION_LOG_ERROR("end");
                // Break if we reach the end of the tetrahedralization volume
                break;
            }

            if ((intersectPt - originPt).size() <= (lastIntersectPt - originPt).size())
            {
                ALICEVISION_LOG_ERROR("reverse");
                // Inverse direction, stop
                break;
            }

            if (geometry.type == EGeometryType::Facet)
            {
                ++behindCount.facets;

                // Vote for the first cell found (only once)
                if (firstIteration)
                {
                    boost::atomic_ref<float>{_cellsAttr[geometry.facet.cellIndex].on} += fWeight;
                    firstIteration = false;
                }

                boost::atomic_ref<float>{_cellsAttr[geometry.facet.cellIndex].fullnessScore} += fWeight;

                // Take the mirror facet to iterate over the next cell
                const Facet mFacet = mirrorFacet(geometry.facet);
                lastIntersectedFacet = mFacet;
                geometry.facet = mFacet;
                if (_tetrahedralization.isInvalidOrInfiniteCell(mFacet.cellIndex))
                {
                    // Break if we reach the end of the tetrahedralization volume (mirror facet cannot be found)
                    break;
                }

                {
                    const float dist = distFcn(maxDist, (originPt - lastIntersectPt).size(), distFcnHeight);
                    boost::atomic_ref<float>{_cellsAttr[geometry.facet.cellIndex].gEdgeVisWeight[geometry.facet.localVertexIndex]} += fWeight * dist;
                }
                if (previousGeometry.type == EGeometryType::Facet && behindCount.facets > 1000)
                {
                    break;
                }
            }
            else
            {
                // Vote for the first cell found (only once)
                // if we come from an edge or vertex to an other we have to vote for the first intersected cell.
                if (firstIteration)
                {
                    if (previousGeometry.type != EGeometryType::Vertex)
                    {
                        ALICEVISION_LOG_ERROR(
                          "[error] The firstIteration vote could only happen during for the first cell when we come from the first vertex.");
                        // throw std::runtime_error("[error] The firstIteration vote could only happen during for the first cell when we come from the
                        // first vertex.");
                    }
                    // the information of first intersected cell can only be found by taking intersection of neighbouring cells for both geometries
                    const std::vector<CellIndex> previousNeighbouring = getNeighboringCellsByVertexIndex(previousGeometry.vertexIndex);
                    const std::vector<CellIndex> currentNeigbouring = getNeighboringCellsByGeometry(geometry);

                    std::vector<CellIndex> neighboringCells;
                    std::set_intersection(previousNeighbouring.begin(),
                                          previousNeighbouring.end(),
                                          currentNeigbouring.begin(),
                                          currentNeigbouring.end(),
                                          std::back_inserter(neighboringCells));

                    for (const CellIndex& ci : neighboringCells)
                    {
                        boost::atomic_ref<float>{_cellsAttr[neighboringCells[0]].on} += fWeight;
                    }
                    firstIteration = false;
                }

                // We have just intersected a vertex or edge.
                // These geometries do not have a cellIndex, so we use the previousGeometry to retrieve the cell between the previous geometry and the
                // current one.
                if (previousGeometry.type == EGeometryType::Facet)
                {
                    boost::atomic_ref<float>{_cellsAttr[previousGeometry.facet.cellIndex].fullnessScore} += fWeight;
                }

                if (geometry.type == EGeometryType::Vertex)
                {
                    ++behindCount.vertices;
                    if (previousGeometry.type == EGeometryType::Vertex && behindCount.vertices > 1000)
                    {
                        break;
                    }
                }
                else if (geometry.type == EGeometryType::Edge)
                {
                    ++behindCount.edges;
                    if (previousGeometry.type == EGeometryType::Edge && behindCount.edges > 1000)
                    {
                        break;
                    }
                }
            }
        }

        // cv: is the tetrahedron in distance 2*sigma behind the point p in the direction of the camera c (called Lcp in the paper) Vote for the last
        // found facet Vote for the last intersected facet (farthest from the camera)
        if (lastIntersectedFacet.cellIndex != GEO::NO_CELL)
        {
            found = true;
            boost::atomic_ref<float>{_cellsAttr[lastIntersectedFacet.cellIndex].cellTWeight} += fWeight;
        }

        
    }
}

float GraphFiller::distFcn(float maxDist, float dist, float distFcnHeight) const
{
    // distFcnHeight ... 0 for distFcn == 1 for all dist, 0.1 distFcn std::min
    // 0.9, ...., 1.0 dist
    // fcn std::min 0.0f ... check matlab
    // MATLAB: distFcnHeight=0.5; maxDist = 10; dist = 0:0.1:20;
    // plot(dist,1-distFcnHeight*exp(-(dist.*dist)/(2*maxDist)));
    return 1.0f - distFcnHeight * std::exp(-(dist * dist) / (2.0f * maxDist));
    // dist large means distFcn close to 1
    // dist small means distFcn close to 0
}

void GraphFiller::initCells()
{
    _cellsAttr.resize(_tetrahedralization.nb_cells());  // or nb_finite_cells() if keeps_infinite()

    for (int i = 0; i < _cellsAttr.size(); ++i)
    {
        GC_cellInfo& c = _cellsAttr[i];

        c.cellSWeight = 0.0f;
        c.cellTWeight = 0.0f;
        c.on = 0.0f;
        c.emptinessScore = 0.0f;
        for (int s = 0; s < 4; ++s)
        {
            c.gEdgeVisWeight[s] = 0.0f;
        }
    }
}



}  // namespace fuseCut
}  // namespace aliceVision
