// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#ifndef OPENMVG_L_INFINITY_COMPUTER_VISION_H_
#define OPENMVG_L_INFINITY_COMPUTER_VISION_H_

// Structure and motion problem solver
#include "openMVG/linearProgramming/lInfinityCV/tijsAndXis_From_xi_Ri.hpp"
#include "openMVG/linearProgramming/lInfinityCV/tijsAndXis_From_xi_Ri_noise.hpp"
#include "openMVG/linearProgramming/lInfinityCV/triplet_tijsAndXis_kernel.hpp"

// Pose estimation solver
#include "openMVG/linearProgramming/lInfinityCV/resection.hpp"
#include "openMVG/linearProgramming/lInfinityCV/resection_kernel.hpp"
// N-View Triangulation solver
#include "openMVG/linearProgramming/lInfinityCV/triangulation.hpp"

//-------------
//-- Global SfM
//-------------
// Compute from global translation by using 2-views relative translations guess
#include "openMVG/linearProgramming/lInfinityCV/global_translations_fromTij.hpp"
// Compute from global translation by using 3-views relative translations guess
#include "openMVG/linearProgramming/lInfinityCV/global_translations_fromTriplets.hpp"

#endif // OPENMVG_L_INFINITY_COMPUTER_VISION_H_
