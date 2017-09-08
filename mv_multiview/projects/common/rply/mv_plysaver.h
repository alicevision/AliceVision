#pragma once

#include "mesh/mv_mesh.h"
#include "rply.h"
//#include "mesh/mv_mesh_scene_point.h"

int mv_saveply(std::string plyFileName, mv_mesh* me, bool diffuse = false);
int mv_saveply(std::string plyFileName, mv_mesh* me, staticVector<rgb>* triColors, bool diffuse = false);
// int mv_saveply(std::string plyFileName, mv_mesh_scene_point *me);

staticVector<rgb>* getTrisColorsRgb(mv_mesh* me, staticVector<rgb>* ptsColors);
staticVector<rgb>* getPtsColorsRgb(mv_mesh* me, staticVector<rgb>* triColors);