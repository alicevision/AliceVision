// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.


#include "GCOutput.hpp"

#include <aliceVision/system/Logger.hpp>


#include <aliceVision/mvsData/Universe.hpp>
#include <aliceVision/mvsData/geometry.hpp>
#include <boost/atomic/atomic_ref.hpp>

namespace aliceVision {
namespace fuseCut {

void GCOutput::filterLargeHelperPoints(std::vector<bool>& out_reliableVertices, const std::vector<bool>& vertexIsOnSurface, int maxSegSize)
{
    ALICEVISION_LOG_DEBUG("DelaunayGraphCut::filterLargeHelperPoints");
    out_reliableVertices.clear();

    return;

    // Do not filter helper points if maxSegSize is negative/infinit
    if (maxSegSize < 0)
        return;

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
            //_tetrahedralization.get_neighbors(vi, adjVertices);

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

    ALICEVISION_LOG_DEBUG("DelaunayGraphCut::filterLargeHelperPoints done.");
}

void GCOutput::createPtsCams(StaticVector<StaticVector<int>>& out_ptsCams)
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

mesh::Mesh* GCOutput::createMesh(int maxNbConnectedHelperPoints)
{
    ALICEVISION_LOG_INFO("Extract mesh from Graph Cut.");

    std::vector<bool> vertexIsOnSurface;
    const int nbSurfaceFacets = computeIsOnSurface(vertexIsOnSurface);

    ALICEVISION_LOG_INFO("# surface facets: " << nbSurfaceFacets);
    ALICEVISION_LOG_INFO("# vertixes: " << _verticesCoords.size());
    ALICEVISION_LOG_INFO("_cellIsFull.size(): " << _cellIsFull.size());

    mesh::Mesh* me = new mesh::Mesh();

    // TODO: copy only surface points and remap visibilities
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

            Facet f2 = mirrorFacet(f1);
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
            vertices[0] = getVertexIndex(f1, 0);
            vertices[1] = getVertexIndex(f1, 1);
            vertices[2] = getVertexIndex(f1, 2);

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

            // if(T.is_infinite(f1.first->vertex(f1.second)))
            // {
            //    printf("WARNINIG infinite vertex\n");
            // }

            const Point3d D1 = _verticesCoords[getOppositeVertexIndex(f1)];  // in FULL part
            const Point3d D2 = _verticesCoords[getOppositeVertexIndex(f2)];  // in EMPTY part

            const Point3d N = cross((points[1] - points[0]).normalize(), (points[2] - points[0]).normalize()).normalize();

            const double dd1 = orientedPointPlaneDistance(D1, points[0], N);
            // const double dd2 = orientedPointPlaneDistance(D2, points[0], N);

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

    ALICEVISION_LOG_INFO("Extract mesh from Graph Cut done.");
    return me;
}

int GCOutput::computeIsOnSurface(std::vector<bool>& vertexIsOnSurface) const
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

            const Facet f2 = mirrorFacet(f1);
            if (_tetrahedralization.isInvalidOrInfiniteCell(f2.cellIndex))
                continue;
            const bool vo = _cellIsFull[f2.cellIndex];  // get if it is occupied

            if (vo)
                continue;

            VertexIndex v1 = getVertexIndex(f1, 0);
            VertexIndex v2 = getVertexIndex(f1, 1);
            VertexIndex v3 = getVertexIndex(f1, 2);
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

int GCOutput::removeDust(int minSegSize)
{
    ALICEVISION_LOG_DEBUG("Removing dust (isolated full cells).");

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

    ALICEVISION_LOG_INFO("DelaunayGraphCut::removeDust: Number of full segments: " << nbFullSegments);
    ALICEVISION_LOG_WARNING("DelaunayGraphCut::removeDust: " << ndust << " cells changed to empty within " << _cellIsFull.size() << " cells.");

    return ndust;
}

int GCOutput::removeBubbles()
{
    ALICEVISION_LOG_DEBUG("Removing empty bubbles.");

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

    ALICEVISION_LOG_INFO("DelaunayGraphCut::removeBubbles: nbBubbles: " << nbBubbles << ", all empty segments: " << nbEmptySegments);
    ALICEVISION_LOG_WARNING("DelaunayGraphCut::removeBubbles: " << nbModifiedCells << " cells changed to full within " << _cellIsFull.size()
                                                                << " cells.");

    return nbBubbles;
}

void GCOutput::graphCutPostProcessing(const Point3d hexah[8], const std::string& folderName)
{
    long timer = std::clock();
    ALICEVISION_LOG_INFO("Graph cut post-processing.");

    int minSegmentSize = _mp.userParams.get<int>("hallucinationsFiltering.minSegmentSize", 10);
    bool doRemoveBubbles = _mp.userParams.get<bool>("hallucinationsFiltering.doRemoveBubbles", true);
    bool doRemoveDust = _mp.userParams.get<bool>("hallucinationsFiltering.doRemoveDust", true);
    bool doLeaveLargestFullSegmentOnly = _mp.userParams.get<bool>("hallucinationsFiltering.doLeaveLargestFullSegmentOnly", false);
    int invertTetrahedronBasedOnNeighborsNbIterations =
      _mp.userParams.get<bool>("hallucinationsFiltering.invertTetrahedronBasedOnNeighborsNbIterations", 10);
    double minSolidAngleRatio = _mp.userParams.get<double>("hallucinationsFiltering.minSolidAngleRatio", 0.2);
    int nbSolidAngleFilteringIterations = _mp.userParams.get<double>("hallucinationsFiltering.nbSolidAngleFilteringIterations", 10);

    if (doRemoveBubbles)
    {
        // Remove empty spaces that are not connected to at least one camera
        removeBubbles();
    }

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
            for (CellIndex adjCellIndex : getNeighboringCellsByVertexIndex(cam_vi))  // GEOGRAM: set_stores_cicl(true) required
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

    if (doRemoveDust)
    {
        removeDust(minSegmentSize);
    }

    if (doLeaveLargestFullSegmentOnly)
    {
        leaveLargestFullSegmentOnly();
    }

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

void GCOutput::cellsStatusFilteringBySolidAngleRatio(int nbSolidAngleFilteringIterations, double minSolidAngleRatio)
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

#pragma omp parallel for reduction(+ : toInvertCount)
        for (int vi = 0; vi < _tetrahedralization.getNeighboringCellsPerVertex().size(); ++vi)
        {
            if (!vertexIsOnSurface[vi])
                continue;
            // ALICEVISION_LOG_INFO("vertex is on surface: " << vi);
            const std::vector<CellIndex>& neighboringCells = _tetrahedralization.getNeighboringCellsPerVertex()[vi];
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
                    // ALICEVISION_LOG_INFO("tetrahedronSolidAngle: " << s);
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
            // ALICEVISION_LOG_INFO("fullPartSolidAngle: " << fullPartSolidAngle << ", ratio: "
            //                                             << fullPartSolidAngle / fullSphereSolidAngle);
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
                    const Facet fv = mirrorFacet(f);
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

void GCOutput::invertFullStatusForSmallLabels()
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

void GCOutput::segmentFullOrFree(bool full, StaticVector<int>& out_fullSegsColor, int& out_nsegments)
{
    ALICEVISION_LOG_DEBUG("segmentFullOrFree: segmenting connected space.");

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

void GCOutput::leaveLargestFullSegmentOnly()
{
    ALICEVISION_LOG_DEBUG("Largest full segment only.");

    int nsegments;
    StaticVector<int> colors;
    segmentFullOrFree(true, colors, nsegments);

    StaticVector<int> colorsSize(nsegments, 0);

    // all free space segments which contains camera has to remain free
    int largestColor = -1;
    int maxn = 0;
    for (CellIndex ci = 0; ci < _cellIsFull.size(); ++ci)
    {
        int color = colors[ci];
        if (color >= 0)
        {
            colorsSize[color] += 1;
            int n = colorsSize[color];
            if (n > maxn)
            {
                maxn = n;
                largestColor = color;
            }
        }
    }

    for (CellIndex ci = 0; ci < _cellIsFull.size(); ++ci)
    {
        if (colors[ci] != largestColor)
        {
            _cellIsFull[ci] = false;
        }
    }

    ALICEVISION_LOG_DEBUG("Largest full segment only done.");
}

}
}