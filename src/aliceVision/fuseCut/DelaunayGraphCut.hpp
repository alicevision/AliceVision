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

    DelaunayGraphCut(mvsUtils::MultiViewParams& mp, const PointCloud & pc, const Tetrahedralization & tetrahedralization);
    virtual ~DelaunayGraphCut();


    inline std::size_t getNbVertices() const { return _verticesAttr.size(); }
    
    void createPtsCams(StaticVector<StaticVector<int>>& out_ptsCams);

    int computeIsOnSurface(std::vector<bool>& vertexIsOnSurface) const;

    void maxflow();

    void createGraphCut(const Point3d hexah[8], const StaticVector<int>& cams);

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

public:
    mvsUtils::MultiViewParams& _mp;
    
    std::vector<GC_cellInfo> _cellsAttr;
    std::vector<bool> _cellIsFull;

    const Tetrahedralization & _tetrahedralization;
    const std::vector<Point3d> & _verticesCoords;
    const std::vector<GC_vertexInfo> & _verticesAttr;
    const std::vector<int> & _camsVertexes;
};


}  // namespace fuseCut
}  // namespace aliceVision
