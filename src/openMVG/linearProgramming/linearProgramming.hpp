// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#ifndef ALICEVISION_LINEAR_PROGRAMMING_H_
#define ALICEVISION_LINEAR_PROGRAMMING_H_

#include <aliceVision/config.hpp>
#include "aliceVision/linearProgramming/linearProgrammingInterface.hpp"
#include "aliceVision/linearProgramming/linearProgrammingOSI_X.hpp"
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_MOSEK)
#include "aliceVision/linearProgramming/linearProgrammingMOSEK.hpp"
#endif

#include "aliceVision/linearProgramming/bisectionLP.hpp"

// Multiple View Geometry solver that rely on Linear programming formulations
#include "aliceVision/linearProgramming/lInfinityCV/lInfinityCV.hpp"

#endif // ALICEVISION_LINEAR_PROGRAMMING_H_
