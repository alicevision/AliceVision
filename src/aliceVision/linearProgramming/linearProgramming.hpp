// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#pragma once

#include <aliceVision/config.hpp>
#include <aliceVision/linearProgramming/bisectionLP.hpp>
#include <aliceVision/linearProgramming/ISolver.hpp>
#include <aliceVision/linearProgramming/OSIXSolver.hpp>

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_MOSEK)
#include <aliceVision/linearProgramming/MOSEKSolver.hpp>
#endif

// Multiple View Geometry solver that rely on Linear programming formulations
#include <aliceVision/linearProgramming/lInfinityCV/lInfinityCV.hpp>

