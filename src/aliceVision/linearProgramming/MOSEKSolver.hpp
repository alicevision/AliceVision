// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

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
