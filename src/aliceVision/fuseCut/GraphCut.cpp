// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "GraphCut.hpp"

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/fuseCut/MaxFlow_AdjList.hpp>

namespace aliceVision {
namespace fuseCut {

void GraphCut::maxflow()
{
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
            Facet fv = mirrorFacet(fu);
            if (_tetrahedralization.isInvalidOrInfiniteCell(fv.cellIndex))
                continue;

            float a1 = 0.0f;
            float a2 = 0.0f;
            if ((!_tetrahedralization.isInfiniteCell(fu.cellIndex)) && (!_tetrahedralization.isInfiniteCell(fv.cellIndex)))
            {
                // Score for each facet based on the quality of the topology
                a1 = getFaceWeight(fu);
                a2 = getFaceWeight(fv);
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

    std::vector<GC_cellInfo>().swap(_cellsAttr); 

    const float totalFlow = maxFlowGraph.compute();
    ALICEVISION_LOG_INFO("totalFlow: " << totalFlow);

    _cellIsFull.resize(nbCells);
    std::size_t nbFullCells = 0;
    for (CellIndex ci = 0; ci < nbCells; ++ci)
    {
        _cellIsFull[ci] = maxFlowGraph.isTarget(ci);
        nbFullCells += _cellIsFull[ci];
    }
}

Point3d GraphCut::cellCircumScribedSphereCentre(CellIndex ci) const
{
    // http://www.mps.mpg.de/homes/daly/CSDS/t4h/tetra.htm

    const Point3d r0 = _verticesCoords[_tetrahedralization.cell_vertex(ci, 0)];
    const Point3d r1 = _verticesCoords[_tetrahedralization.cell_vertex(ci, 1)];
    const Point3d r2 = _verticesCoords[_tetrahedralization.cell_vertex(ci, 2)];
    const Point3d r3 = _verticesCoords[_tetrahedralization.cell_vertex(ci, 3)];

    const Point3d d1 = r1 - r0;
    const Point3d d2 = r2 - r0;
    const Point3d d3 = r3 - r0;

    float x =
      -(-(d1.x * d2.y * d3.z * conj(d1.x) - d1.x * d2.z * d3.y * conj(d1.x) + d1.y * d2.y * d3.z * conj(d1.y) - d1.y * d2.z * d3.y * conj(d1.y) +
          d1.z * d2.y * d3.z * conj(d1.z) - d1.z * d2.z * d3.y * conj(d1.z) - d1.y * d2.x * d3.z * conj(d2.x) + d1.z * d2.x * d3.y * conj(d2.x) -
          d1.y * d2.y * d3.z * conj(d2.y) + d1.z * d2.y * d3.y * conj(d2.y) - d1.y * d2.z * d3.z * conj(d2.z) + d1.z * d2.z * d3.y * conj(d2.z) +
          d1.y * d2.z * d3.x * conj(d3.x) - d1.z * d2.y * d3.x * conj(d3.x) + d1.y * d2.z * d3.y * conj(d3.y) - d1.z * d2.y * d3.y * conj(d3.y) +
          d1.y * d2.z * d3.z * conj(d3.z) - d1.z * d2.y * d3.z * conj(d3.z)) /
        (2 * d1.x * d2.y * d3.z - 2 * d1.x * d2.z * d3.y - 2 * d1.y * d2.x * d3.z + 2 * d1.y * d2.z * d3.x + 2 * d1.z * d2.x * d3.y -
         2 * d1.z * d2.y * d3.x));
    float y =
      -((d1.x * d2.x * d3.z * conj(d1.x) - d1.x * d2.z * d3.x * conj(d1.x) + d1.y * d2.x * d3.z * conj(d1.y) - d1.y * d2.z * d3.x * conj(d1.y) +
         d1.z * d2.x * d3.z * conj(d1.z) - d1.z * d2.z * d3.x * conj(d1.z) - d1.x * d2.x * d3.z * conj(d2.x) + d1.z * d2.x * d3.x * conj(d2.x) -
         d1.x * d2.y * d3.z * conj(d2.y) + d1.z * d2.y * d3.x * conj(d2.y) - d1.x * d2.z * d3.z * conj(d2.z) + d1.z * d2.z * d3.x * conj(d2.z) +
         d1.x * d2.z * d3.x * conj(d3.x) - d1.z * d2.x * d3.x * conj(d3.x) + d1.x * d2.z * d3.y * conj(d3.y) - d1.z * d2.x * d3.y * conj(d3.y) +
         d1.x * d2.z * d3.z * conj(d3.z) - d1.z * d2.x * d3.z * conj(d3.z)) /
        (2 * d1.x * d2.y * d3.z - 2 * d1.x * d2.z * d3.y - 2 * d1.y * d2.x * d3.z + 2 * d1.y * d2.z * d3.x + 2 * d1.z * d2.x * d3.y -
         2 * d1.z * d2.y * d3.x));
    float z =
      -(-(d1.x * d2.x * d3.y * conj(d1.x) - d1.x * d2.y * d3.x * conj(d1.x) + d1.y * d2.x * d3.y * conj(d1.y) - d1.y * d2.y * d3.x * conj(d1.y) +
          d1.z * d2.x * d3.y * conj(d1.z) - d1.z * d2.y * d3.x * conj(d1.z) - d1.x * d2.x * d3.y * conj(d2.x) + d1.y * d2.x * d3.x * conj(d2.x) -
          d1.x * d2.y * d3.y * conj(d2.y) + d1.y * d2.y * d3.x * conj(d2.y) - d1.x * d2.z * d3.y * conj(d2.z) + d1.y * d2.z * d3.x * conj(d2.z) +
          d1.x * d2.y * d3.x * conj(d3.x) - d1.y * d2.x * d3.x * conj(d3.x) + d1.x * d2.y * d3.y * conj(d3.y) - d1.y * d2.x * d3.y * conj(d3.y) +
          d1.x * d2.y * d3.z * conj(d3.z) - d1.y * d2.x * d3.z * conj(d3.z)) /
        (2 * d1.x * d2.y * d3.z - 2 * d1.x * d2.z * d3.y - 2 * d1.y * d2.x * d3.z + 2 * d1.y * d2.z * d3.x + 2 * d1.z * d2.x * d3.y -
         2 * d1.z * d2.y * d3.x));

    return r0 + Point3d(x, y, z);
}

double GraphCut::getFaceWeight(const Facet& f1) const
{
    const Facet f2 = mirrorFacet(f1);
    const Point3d s1 = cellCircumScribedSphereCentre(f1.cellIndex);
    const Point3d s2 = cellCircumScribedSphereCentre(f2.cellIndex);

    const Point3d A = _verticesCoords[getVertexIndex(f1, 0)];
    const Point3d B = _verticesCoords[getVertexIndex(f1, 1)];
    const Point3d C = _verticesCoords[getVertexIndex(f1, 2)];

    const Point3d n = cross((B - A).normalize(), (C - A).normalize()).normalize();

    double a1 = fabs(angleBetwV1andV2(n, (A - s1).normalize()));
    if (a1 > 90)
    {
        a1 = 180.0 - a1;
    }

    double a2 = fabs(angleBetwV1andV2(n, (A - s2).normalize()));
    if (a2 > 90)
    {
        a2 = 180.0 - a2;
    }

    a1 = a1 * ((double)M_PI / 180.0);
    a2 = a2 * ((double)M_PI / 180.0);

    double wf = 1.0 - std::min(std::cos(a1), std::cos(a2));

    if ((std::isnan(wf)) || (wf < 0.0f) || (wf > 1.0f))
        return 1.0;

    return wf;
}

}
}
