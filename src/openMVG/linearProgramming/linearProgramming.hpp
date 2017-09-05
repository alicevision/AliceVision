// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#ifndef OPENMVG_LINEAR_PROGRAMMING_H_
#define OPENMVG_LINEAR_PROGRAMMING_H_

#include <openMVG/config.hpp>
#include "openMVG/linearProgramming/linearProgrammingInterface.hpp"
#include "openMVG/linearProgramming/linearProgrammingOSI_X.hpp"
#if OPENMVG_IS_DEFINED(OPENMVG_HAVE_MOSEK)
#include "openMVG/linearProgramming/linearProgrammingMOSEK.hpp"
#endif

#include "openMVG/linearProgramming/bisectionLP.hpp"

// Multiple View Geometry solver that rely on Linear programming formulations
#include "openMVG/linearProgramming/lInfinityCV/lInfinityCV.hpp"

#endif // OPENMVG_LINEAR_PROGRAMMING_H_
