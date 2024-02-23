// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/image/Rgb.hpp>
#include <aliceVision/mvsData/Point3d.hpp>
#include <aliceVision/mvsData/StaticVector.hpp>
#include <aliceVision/mvsData/Voxel.hpp>
#include <aliceVision/mvsUtils/common.hpp>
#include <aliceVision/mesh/Mesh.hpp>
#include <aliceVision/fuseCut/PointCloud.hpp>
#include <aliceVision/fuseCut/delaunayGraphCutTypes.hpp>
#include <aliceVision/fuseCut/Tetrahedralization.hpp>
#include <aliceVision/fuseCut/Intersections.hpp>

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


class DelaunayGraphCut
{
  public:
    using VertexIndex = GEO::index_t;
    using CellIndex = GEO::index_t;

    static const GEO::index_t NO_TETRAHEDRON = GEO::NO_CELL;

    DelaunayGraphCut(mvsUtils::MultiViewParams& mp, const PointCloud & pc);
    virtual ~DelaunayGraphCut();

    /**
     * @brief Retrieve the global vertex index of the localVertexIndex of the facet.
     *
     * @param f the facet
     * @return the global vertex index
     */
    inline VertexIndex getOppositeVertexIndex(const Facet& f) const { return _tetrahedralization->cell_vertex(f.cellIndex, f.localVertexIndex); }

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
        return {&(_verticesCoords[getVertexIndex(f, 0)]), &(_verticesCoords[getVertexIndex(f, 1)]), &(_verticesCoords[getVertexIndex(f, 2)])};
    }

    inline std::size_t getNbVertices() const { return _verticesAttr.size(); }

    inline GEO::index_t nearestVertexInCell(GEO::index_t cellIndex, const Point3d& p) const
    {
        GEO::signed_index_t result = NO_TETRAHEDRON;
        double d = std::numeric_limits<double>::max();
        for (GEO::index_t i = 0; i < 4; ++i)
        {
            GEO::signed_index_t currentVertex = _tetrahedralization->cell_vertex(cellIndex, i);
            if (currentVertex < 0)
                continue;
            double currentDist = GEO::Geom::distance2(_verticesCoords[currentVertex].m, p.m, 3);
            if (currentDist < d)
            {
                d = currentDist;
                result = currentVertex;
            }
        }
        return result;
    }

    /**
     * @brief Retrieves the global indexes of neighboring cells around a geometry.
     *
     * @param g the concerned geometry
     * @return a vector of neighboring cell indices
     */
    std::vector<CellIndex> getNeighboringCellsByGeometry(const GeometryIntersection& g) const;

    

    void computeDelaunay();
    void initCells();

    void createPtsCams(StaticVector<StaticVector<int>>& out_ptsCams);

    float distFcn(float maxDist, float dist, float distFcnHeight) const;
    
    void fillGraph(double nPixelSizeBehind, float fullWeight);
    
    void rayMarchingGraphEmpty(int vertexIndex,
                                int cam,
                                float weight);

    void rayMarchingGraphFull(int vertexIndex,
                           int cam,
                           float fullWeight,
                           double nPixelSizeBehind);

    /**
     * @brief Estimate the cells property "on" based on the analysis of the visibility of neigbouring cells.
     *
     * @param nPixelSizeBehind Used to define the surface margin
     */
    void forceTedgesByGradientIJCV(float nPixelSizeBehind);

    int computeIsOnSurface(std::vector<bool>& vertexIsOnSurface) const;

    void addToInfiniteSw(float sW);

    void maxflow();

    void voteFullEmptyScore(const StaticVector<int>& cams);

    void createGraphCut(const Point3d hexah[8],
                        const StaticVector<int>& cams);

    /**
     * @brief Invert full/empty status of cells if they represent a too small group after labelling.
     */
    void invertFullStatusForSmallLabels();
    /**
     * @brief Check solid angle ratio between empty/full part around each vertex to reduce local artefacts / improve smoothness.
     */
    void cellsStatusFilteringBySolidAngleRatio(int nbSolidAngleFilteringIterations, double minSolidAngleRatio);

    /**
     * @brief Combine all post-processing steps results to reduce artefacts from the graph-cut (too large triangles, noisy tetrahedrons, isolated
     * cells, etc).
     */
    void graphCutPostProcessing(const Point3d hexah[8]);

    void segmentFullOrFree(bool full, StaticVector<int>& out_fullSegsColor, int& nsegments);
    int removeBubbles();
    int removeDust(int minSegSize);

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
     * @param[in] maxNbConnectedHelperPoints: maximum number of connected helper points before we remove the group. 
     * 0 means that we remove all helper points. 
     * -1 means that we do not filter helper points at all.
     */
    mesh::Mesh* createMesh(int maxNbConnectedHelperPoints);

private:
    mvsUtils::MultiViewParams& _mp;
    
    std::unique_ptr<Tetrahedralization> _tetrahedralization;
    std::vector<GC_cellInfo> _cellsAttr;
    std::vector<bool> _cellIsFull;

    const std::vector<Point3d> & _verticesCoords;
    const std::vector<GC_vertexInfo> & _verticesAttr;
    const std::vector<int> & _camsVertexes;
};


}  // namespace fuseCut
}  // namespace aliceVision
