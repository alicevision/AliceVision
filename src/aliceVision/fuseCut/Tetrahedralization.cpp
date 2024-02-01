// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Tetrahedralization.hpp"

#include <geogram/delaunay/delaunay.h>
#include <geogram/delaunay/delaunay_3d.h>

namespace aliceVision {
namespace fuseCut {

Tetrahedralization::Tetrahedralization()
{
}

void Tetrahedralization::buildFromVertices(const std::vector<Point3d> & vertices)
{
    //Use geogram to build tetrahedrons
    GEO::initialize();
    GEO::Delaunay_var tetrahedralization = GEO::Delaunay::create(3, "BDEL");
    tetrahedralization->set_stores_neighbors(true);
    tetrahedralization->set_vertices(vertices.size(), vertices.front().m);

    //Copy information
    _mesh.resize(tetrahedralization->nb_cells());
    for (CellIndex ci = 0; ci < tetrahedralization->nb_cells(); ci++)
    {
        Cell & c = _mesh[ci];
        c.indices[0] = tetrahedralization->cell_vertex(ci, 0);
        c.indices[1] = tetrahedralization->cell_vertex(ci, 1);
        c.indices[2] = tetrahedralization->cell_vertex(ci, 2);
        c.indices[3] = tetrahedralization->cell_vertex(ci, 3);
        c.adjacent[0] = tetrahedralization->cell_adjacent(ci, 0);
        c.adjacent[1] = tetrahedralization->cell_adjacent(ci, 1);
        c.adjacent[2] = tetrahedralization->cell_adjacent(ci, 2);
        c.adjacent[3] = tetrahedralization->cell_adjacent(ci, 3);
    }

    tetrahedralization.reset();
}

}
}