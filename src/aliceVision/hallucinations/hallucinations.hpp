#pragma once

#include <aliceVision/mesh/mv_mesh.hpp>
#include <aliceVision/output3D/mv_output3D.hpp>
#include <aliceVision/structures/mv_geometry.hpp>

#include <aliceVision/prematching/mv_prematch_cams.hpp>

float confKernelVoting(staticVector<float>* confs, float c);
void filterNonConsistentTrianglesMeshDist(std::string tmpDirIters, multiviewParams* mp, int ri,
                                          int niter);
void filterLargeEdgeTriangles(mv_mesh* me, float avelthr);
