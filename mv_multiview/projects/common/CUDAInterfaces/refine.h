#pragma once

#include "mesh/mv_mesh.h"
#include "output3D/mv_output3D.h"

void filterPtsCamsByMinimalPixelSize(mv_mesh* me, staticVector<staticVector<int>*>* ptsCams, multiviewParams* mp);
