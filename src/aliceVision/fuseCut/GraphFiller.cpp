// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "GraphFiller.hpp"

#include <boost/atomic/atomic_ref.hpp>

namespace aliceVision {
namespace fuseCut {

void GraphFiller::createGraphCut(const Point3d hexah[8], const StaticVector<int>& cams)
{
    initCells();
    
    voteFullEmptyScore(cams);
}

void GraphFiller::voteFullEmptyScore(const StaticVector<int>& cams)
{
    ALICEVISION_LOG_INFO("DelaunayGraphCut::voteFullEmptyScore");
    const int maxint = std::numeric_limits<int>::max();


    const bool forceTEdge = _mp.userParams.get<bool>("delaunaycut.voteFilteringForWeaklySupportedSurfaces", true);
    const double nPixelSizeBehind = _mp.userParams.get<double>("delaunaycut.nPixelSizeBehind", 4.0);  // sigma value
    const float fullWeight = float(_mp.userParams.get<double>("delaunaycut.fullWeight", 1.0));

    // 0 for distFcn equals 1 all the time
    const float distFcnHeight = (float)_mp.userParams.get<double>("delaunaycut.distFcnHeight", 0.0);

    fillGraph(nPixelSizeBehind, false, true, distFcnHeight, fullWeight);
    addToInfiniteSw((float)maxint);    

    if (forceTEdge)
    {
        forceTedgesByGradientIJCV(nPixelSizeBehind);
    }
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

void GraphFiller::forceTedgesByGradientIJCV(float nPixelSizeBehind)
{

    const float forceTEdgeDelta = (float)_mp.userParams.get<double>("delaunaycut.forceTEdgeDelta", 0.1f);
    const float minJumpPartRange = (float)_mp.userParams.get<double>("delaunaycut.minJumpPartRange", 10000.0f);
    const float maxSilentPartRange = (float)_mp.userParams.get<double>("delaunaycut.maxSilentPartRange", 100.0f);
    const float nsigmaJumpPart = (float)_mp.userParams.get<double>("delaunaycut.nsigmaJumpPart", 4.0f);
    const float nsigmaFrontSilentPart = (float)_mp.userParams.get<double>("delaunaycut.nsigmaFrontSilentPart", 2.0f);
    const float nsigmaBackSilentPart = (float)_mp.userParams.get<double>("delaunaycut.nsigmaBackSilentPart", 2.0f);

    for (GC_cellInfo& c : _cellsAttr)
    {
        c.on = 0.0f;
    }

    const double marginEpsilonFactor = 1.0e-4;

    // choose random order to prevent waiting
    const unsigned int seed = (unsigned int)_mp.userParams.get<unsigned int>("delaunaycut.seed", 0);
    const std::vector<int> verticesRandIds = mvsUtils::createRandomArrayOfIntegers(_verticesAttr.size(), seed);

    size_t totalStepsFront = 0;
    size_t totalRayFront = 0;
    size_t totalStepsBehind = 0;
    size_t totalRayBehind = 0;

    size_t totalCamHaveVisibilityOnVertex = 0;
    size_t totalOfVertex = 0;

    size_t totalVertexIsVirtual = 0;

    GeometriesCount totalGeometriesIntersectedFrontCount;
    GeometriesCount totalGeometriesIntersectedBehindCount;

#pragma omp parallel for reduction(+ : totalStepsFront, totalRayFront, totalStepsBehind, totalRayBehind)
    for (int i = 0; i < verticesRandIds.size(); ++i)
    {
        const int vertexIndex = verticesRandIds[i];
        const GC_vertexInfo& v = _verticesAttr[vertexIndex];
        if (v.isVirtual())
            continue;

        ++totalVertexIsVirtual;
        const Point3d& originPt = _verticesCoords[vertexIndex];
        // For each camera that has visibility over the vertex v (vertexIndex)
        for (const int cam : v.cams)
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

                    ++totalStepsFront;

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
                            // ALICEVISION_LOG_DEBUG("[Error]: forceTedges(toTheCam) cause: invalidOrInfinite miror facet.");
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
                ++totalRayFront;
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

                    ++totalStepsBehind;

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
                ++totalRayBehind;
            }
        }
        totalCamHaveVisibilityOnVertex += v.cams.size();
        totalOfVertex += 1;
    }

    for (GC_cellInfo& c : _cellsAttr)
    {
        const float w = std::max(1.0f, c.cellTWeight) * c.on;

        // cellTWeight = clamp(w, cellTWeight, 1000000.0f);
        c.cellTWeight = std::max(c.cellTWeight, std::min(1000000.0f, w));
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
                    continue;

                const Facet intersectionFacet(tetrahedronIndex, i);
                bool ambiguous = false;

                const GeometryIntersection result =
                  rayIntersectTriangle(originPt, dirVect, intersectionFacet, intersectPt, epsilonFactor, ambiguous, &lastIntersectPt);
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

void GraphFiller::fillGraph(double nPixelSizeBehind,
                                 bool labatutWeights,
                                 bool fillOut,
                                 float distFcnHeight,
                                 float fullWeight)  // nPixelSizeBehind=2*spaceSteps allPoints=1 behind=0
                                                    // labatutWeights=0 fillOut=1 distFcnHeight=0
{
    ALICEVISION_LOG_INFO("Computing s-t graph weights.");
    long t1 = clock();

    // loop over all cells ... initialize
    for (GC_cellInfo& c : _cellsAttr)
    {
        c.cellSWeight = 0.0f;
        c.cellTWeight = 0.0f;
        c.fullnessScore = 0.0f;
        c.emptinessScore = 0.0f;
        c.on = 0.0f;
        for (int s = 0; s < 4; s++)
        {
            c.gEdgeVisWeight[s] = 0.0f;
        }
    }

    // choose random order to prevent waiting
    const unsigned int seed = (unsigned int)_mp.userParams.get<unsigned int>("delaunaycut.seed", 0);
    const std::vector<int> verticesRandIds = mvsUtils::createRandomArrayOfIntegers(_verticesAttr.size(), seed);

    int64_t totalStepsFront = 0;
    int64_t totalRayFront = 0;
    int64_t totalStepsBehind = 0;
    int64_t totalRayBehind = 0;

    size_t totalCamHaveVisibilityOnVertex = 0;
    size_t totalOfVertex = 0;

    size_t totalIsRealNrc = 0;

    GeometriesCount totalGeometriesIntersectedFrontCount;
    GeometriesCount totalGeometriesIntersectedBehindCount;


    size_t progressStep = verticesRandIds.size() / 100;
    progressStep = std::max(size_t(1), progressStep);
#pragma omp parallel for reduction(+:totalStepsFront,totalRayFront,totalStepsBehind,totalRayBehind,totalCamHaveVisibilityOnVertex,totalOfVertex,totalIsRealNrc)
    for (int i = 0; i < verticesRandIds.size(); i++)
    {
        const int vertexIndex = verticesRandIds[i];
        const GC_vertexInfo& v = _verticesAttr[vertexIndex];

        GeometriesCount subTotalGeometriesIntersectedFrontCount;
        GeometriesCount subTotalGeometriesIntersectedBehindCount;

        if (v.isReal())
        {
            ++totalIsRealNrc;
            // "weight" is called alpha(p) in the paper
            float weight = (float)v.nrc;  // number of cameras

            //Overwrite with forced weight if available
            weight = (float)_mp.userParams.get<double>("LargeScale.forceWeight", weight);

            for (int c = 0; c < v.cams.size(); c++)
            {
                assert(v.cams[c] >= 0);
                assert(v.cams[c] < _mp.ncams);

                int stepsFront = 0;
                int stepsBehind = 0;
                GeometriesCount geometriesIntersectedFrontCount;
                GeometriesCount geometriesIntersectedBehindCount;
                fillGraphPartPtRc(stepsFront,
                                  stepsBehind,
                                  geometriesIntersectedFrontCount,
                                  geometriesIntersectedBehindCount,
                                  vertexIndex,
                                  v.cams[c],
                                  weight,
                                  fullWeight,
                                  nPixelSizeBehind,
                                  fillOut,
                                  distFcnHeight);

                totalStepsFront += stepsFront;
                totalRayFront += 1;
                totalStepsBehind += stepsBehind;
                totalRayBehind += 1;

                subTotalGeometriesIntersectedFrontCount += geometriesIntersectedFrontCount;
                subTotalGeometriesIntersectedBehindCount += geometriesIntersectedBehindCount;
            }  // for c

            totalCamHaveVisibilityOnVertex += v.cams.size();
            totalOfVertex += 1;
        }
    }

}

void GraphFiller::fillGraphPartPtRc(int& outTotalStepsFront,
                                         int& outTotalStepsBehind,
                                         GeometriesCount& outFrontCount,
                                         GeometriesCount& outBehindCount,
                                         int vertexIndex,
                                         int cam,
                                         float weight,
                                         float fullWeight,
                                         double nPixelSizeBehind,
                                         bool fillOut,
                                         float distFcnHeight)  // nPixelSizeBehind=2*spaceSteps allPoints=1 behind=0 fillOut=1 distFcnHeight=0
{
    const int maxint = std::numeric_limits<int>::max();
    const double marginEpsilonFactor = 1.0e-4;

    const Point3d& originPt = _verticesCoords[vertexIndex];
    const double pixSize = _verticesAttr[vertexIndex].pixSize;
    const double maxDist = nPixelSizeBehind * pixSize;

    assert(cam >= 0);
    assert(cam < _mp.ncams);

    if (fillOut)  // EMPTY part
    {
        // Initialisation
        GeometryIntersection geometry(vertexIndex);  // Starting on global vertex index
        Point3d intersectPt = originPt;
        // toTheCam
        const double pointCamDistance = (_mp.CArr[cam] - originPt).size();
        const Point3d dirVect = (_mp.CArr[cam] - originPt).normalize();

#ifdef ALICEVISION_DEBUG_VOTE
        IntersectionHistory history(_mp.CArr[cam], originPt, dirVect);
#endif
        outTotalStepsFront = 0;
        Facet lastIntersectedFacet;
        bool lastGeoIsVertex = false;
        // Break only when we reach our camera vertex (as long as we find a next geometry)
        while (geometry.type != EGeometryType::Vertex || (_mp.CArr[cam] - intersectPt).size() >= 1.0e-3)
        {
            lastGeoIsVertex = false;
            // Keep previous informations
            const GeometryIntersection previousGeometry = geometry;
            const Point3d lastIntersectPt = intersectPt;

#ifdef ALICEVISION_DEBUG_VOTE
            history.append(geometry, intersectPt);
#endif
            ++outTotalStepsFront;

            geometry = intersectNextGeom(previousGeometry, originPt, dirVect, intersectPt, marginEpsilonFactor, lastIntersectPt);

            if (geometry.type == EGeometryType::None)
            {
#ifdef ALICEVISION_DEBUG_VOTE
                // exportBackPropagationMesh("fillGraph_v" + std::to_string(vertexIndex) + "_ToCam_typeNone", history.geometries, originPt,
                // _mp.CArr[cam]);
#endif
                // ALICEVISION_LOG_DEBUG(
                //     "[Error]: fillGraph(toTheCam) cause: geometry cannot be found."
                //     << "Current vertex index: " << vertexIndex
                //     << ", Previous geometry type: " << previousGeometry.type
                //     << ", outFrontCount:" << outFrontCount);
                break;
            }

            if ((intersectPt - originPt).size() <= (lastIntersectPt - originPt).size())
            {
                // Inverse direction, stop
                break;
            }

#ifdef ALICEVISION_DEBUG_VOTE
            {
                const auto end = history.geometries.end();
                auto it = std::find(history.geometries.begin(), end, geometry);
                if (it != end)
                {
                    // exportBackPropagationMesh("fillGraph_ToCam_alreadyIntersected", history.geometries, originPt, _mp.CArr[cam]);
                    ALICEVISION_LOG_DEBUG("[Error]: fillGraph(toTheCam) cause: intersected geometry has already been intersected.");
                    break;
                }
            }
#endif

            if (geometry.type == EGeometryType::Facet)
            {
                ++outFrontCount.facets;
                boost::atomic_ref<float>{_cellsAttr[geometry.facet.cellIndex].emptinessScore} += weight;

                {
                    const float dist = distFcn(maxDist, (originPt - lastIntersectPt).size(), distFcnHeight);
                    boost::atomic_ref<float>{_cellsAttr[geometry.facet.cellIndex].gEdgeVisWeight[geometry.facet.localVertexIndex]} += weight * dist;
                }

                // Take the mirror facet to iterate over the next cell
                const Facet mFacet = mirrorFacet(geometry.facet);
                geometry.facet = mFacet;
                if (_tetrahedralization.isInvalidOrInfiniteCell(mFacet.cellIndex))
                {
#ifdef ALICEVISION_DEBUG_VOTE
                    // exportBackPropagationMesh("fillGraph_ToCam_invalidMirorFacet", history.geometries, originPt, _mp.CArr[cam]);
#endif
                    // ALICEVISION_LOG_DEBUG("[Error]: fillGraph(toTheCam) cause: invalidOrInfinite miror facet.");
                    break;
                }
                lastIntersectedFacet = mFacet;
                if (previousGeometry.type == EGeometryType::Facet && outFrontCount.facets > 10000)
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
                    ++outFrontCount.vertices;
                    lastGeoIsVertex = true;
                    if (previousGeometry.type == EGeometryType::Vertex && outFrontCount.vertices > 1000)
                    {
                        break;
                    }
                }
                else if (geometry.type == EGeometryType::Edge)
                {
                    ++outFrontCount.edges;
                    if (previousGeometry.type == EGeometryType::Edge && outFrontCount.edges > 1000)
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
                // If we come from a facet, the next intersection must exist (even if the mirror facet is invalid, which is verified after taking
                // mirror facet)
                if (previousGeometry.type == EGeometryType::Facet)
                {

                }
                // Break if we reach the end of the tetrahedralization volume
                break;
            }

            if ((intersectPt - originPt).size() <= (lastIntersectPt - originPt).size())
            {
                // Inverse direction, stop
                break;
            }
            if (geometry.type == EGeometryType::Facet)
            {
                ++outBehindCount.facets;

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
                if (previousGeometry.type == EGeometryType::Facet && outBehindCount.facets > 1000)
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
                    ++outBehindCount.vertices;
                    if (previousGeometry.type == EGeometryType::Vertex && outBehindCount.vertices > 1000)
                    {
                        break;
                    }
                }
                else if (geometry.type == EGeometryType::Edge)
                {
                    ++outBehindCount.edges;
                    if (previousGeometry.type == EGeometryType::Edge && outBehindCount.edges > 1000)
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
        c.fullnessScore = 0.0f;
        c.emptinessScore = 0.0f;
        for (int s = 0; s < 4; ++s)
        {
            c.gEdgeVisWeight[s] = 0.0f;
        }
    }
}



}  // namespace fuseCut
}  // namespace aliceVision
