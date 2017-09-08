#pragma once

#include "mesh/mv_mesh.h"
#include "output3D/mv_output3D.h"
#include "structures/mv_geometry.h"

#include "prematching/mv_prematch_cams.h"

float confKernelVoting(staticVector<float>* confs, float c);
void filterNonConsistentTrianglesMeshDist(std::string tmpDirIters, multiviewParams* mp, int ri,
                                          int niter);
void filterLargeEdgeTriangles(mv_mesh* me, float avelthr);
