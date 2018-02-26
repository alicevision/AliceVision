// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "aliceVision/numeric/numeric.hpp"
#include "aliceVision/linearProgramming/ISolver.hpp"

extern "C"{
#include "mosek.h"
}

#include <vector>

namespace aliceVision   {
namespace linearProgramming  {

/// MOSEK wrapper for the ISolver
class MOSEKSolver : public ISolver
{
public :
  MOSEKSolver(int nbParams);

  ~MOSEKSolver();

  //--
  // Inherited functions :
  //--

  bool setup(const LPConstraints & constraints);
  bool setup(const LPConstraintsSparse & constraints);

  bool solve();

  bool getSolution(std::vector<double> & estimatedParams);

private :
  //MSKenv_t     env;
  MSKtask_t    task; // Solver object.
};


} // namespace linearProgramming
} // namespace aliceVision
