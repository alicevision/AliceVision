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


namespace aliceVision {
namespace fuseCut {

class GraphFiller
{
public:
    GraphFiller(mvsUtils::MultiViewParams& mp, const PointCloud & pc, const Tetrahedralization & tetrahedralization);

    void build(const StaticVector<int>& cams);

    const std::vector<GC_cellInfo> & getCellsAttributes() const
    {
        return _cellsAttr;
    }

    std::vector<bool> & getCellsStatus()
    {
        return _cellIsFull;
    }

    void binarize();

private:
    void initCells();
    void addToInfiniteSw(float sW);

    void fillGraph(double nPixelSizeBehind, float fullWeight);
    void rayMarchingGraphEmpty(int vertexIndex, int cam, float weight);
    void rayMarchingGraphFull(int vertexIndex, int cam, float fullWeight, double nPixelSizeBehind);
    void forceTedgesByGradientIJCV(float nPixelSizeBehind);
    
    std::vector<CellIndex> getNeighboringCellsByGeometry(const GeometryIntersection& g) const;

private:
    const Tetrahedralization & _tetrahedralization;
    const std::vector<Point3d> & _verticesCoords;
    const std::vector<GC_vertexInfo> & _verticesAttr;
    const std::vector<int> & _camsVertexes;

private:
    mvsUtils::MultiViewParams& _mp;
    std::vector<GC_cellInfo> _cellsAttr;
    std::vector<bool> _cellIsFull;
};

}
}