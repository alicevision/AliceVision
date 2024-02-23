// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/fuseCut/PointCloud.hpp>
#include <aliceVision/fuseCut/Tetrahedralization.hpp>
#include <aliceVision/mvsUtils/MultiViewParams.hpp>
#include <aliceVision/fuseCut/Intersections.hpp>
#include <aliceVision/mesh/Mesh.hpp>

namespace aliceVision {
namespace fuseCut {

class Mesher
{
public:
    Mesher(mvsUtils::MultiViewParams& mp, const PointCloud & pc, const Tetrahedralization & tetrahedralization, std::vector<bool> & cellIsFull);

    /**
     * @brief Create a mesh from the tetrahedral scores
     * @param[in] maxNbConnectedHelperPoints: maximum number of connected helper points before we remove the group. 
     * 0 means that we remove all helper points. 
     * -1 means that we do not filter helper points at all.
     */
    mesh::Mesh * createMesh(int maxNbConnectedHelperPoints);

    /**
     * @brief Combine all post-processing steps results to reduce artefacts from the graph-cut (too large triangles, noisy tetrahedrons, isolated
     * cells, etc).
     */
    void graphCutPostProcessing(const Point3d hexah[8]);

private:
    int computeIsOnSurface(std::vector<bool>& vertexIsOnSurface) const;

    /**
     * Some vertices have not been created by depth maps but 
     * have been created for the tetrahedralization like
     * camera centers, helper points, etc.
     * These points have no visibility information (nrc == 0).
     * We want to remove these fake points, but we do not want to create holes.
     * So if the vertex is alone in the middle of valid points, we want to keep it.
     */
    void filterLargeHelperPoints(std::vector<bool>& out_reliableVertices, 
                                const std::vector<bool>& vertexIsOnSurface, 
                                int maxSegSize);

    int removeBubbles();
    int removeDust(int minSegSize);
    void segmentFullOrFree(bool full, StaticVector<int>& out_fullSegsColor, int& nsegments);
    /**
     * @brief Invert full/empty status of cells if they represent a too small group after labelling.
     */
    void invertFullStatusForSmallLabels();
    /**
     * @brief Check solid angle ratio between empty/full part around each vertex to reduce local artefacts / improve smoothness.
     */
    void cellsStatusFilteringBySolidAngleRatio(int nbSolidAngleFilteringIterations, double minSolidAngleRatio);

private:
    const Tetrahedralization & _tetrahedralization;
    const std::vector<Point3d> & _verticesCoords;
    const std::vector<GC_vertexInfo> & _verticesAttr;
    const std::vector<int> & _camsVertexes;

private:
    std::vector<bool> _cellIsFull;
    mvsUtils::MultiViewParams& _mp;
};

}
}