// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#ifndef ALICEVISION_L_INFINITY_COMPUTER_VISION_H_
#define ALICEVISION_L_INFINITY_COMPUTER_VISION_H_

// Structure and motion problem solver
#include "aliceVision/linearProgramming/lInfinityCV/tijsAndXis_From_xi_Ri.hpp"
#include "aliceVision/linearProgramming/lInfinityCV/tijsAndXis_From_xi_Ri_noise.hpp"
#include "aliceVision/linearProgramming/lInfinityCV/triplet_tijsAndXis_kernel.hpp"

// Pose estimation solver
#include "aliceVision/linearProgramming/lInfinityCV/resection.hpp"
#include "aliceVision/linearProgramming/lInfinityCV/resection_kernel.hpp"
// N-View Triangulation solver
#include "aliceVision/linearProgramming/lInfinityCV/triangulation.hpp"

//-------------
//-- Global SfM
//-------------
// Compute from global translation by using 2-views relative translations guess
#include "aliceVision/linearProgramming/lInfinityCV/global_translations_fromTij.hpp"
// Compute from global translation by using 3-views relative translations guess
#include "aliceVision/linearProgramming/lInfinityCV/global_translations_fromTriplets.hpp"

#endif // ALICEVISION_L_INFINITY_COMPUTER_VISION_H_
