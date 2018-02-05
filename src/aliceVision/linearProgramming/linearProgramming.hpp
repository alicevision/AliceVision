// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

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

