#pragma once

#include "rply.h"

#include <aliceVision/mesh/mv_mesh.hpp>

bool mv_loadply(std::string plyFileName, mv_mesh* me);
