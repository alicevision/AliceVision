// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#ifndef MIMATTE_LINEAR_PROGRAMMING_INTERFACE_MOSEK_H_
#define MIMATTE_LINEAR_PROGRAMMING_INTERFACE_MOSEK_H_


#include "openMVG/numeric/numeric.h"
#include "openMVG/linearProgramming/linearProgrammingInterface.hpp"
extern "C"{
#include "mosek.h"
}
#include <vector>

namespace openMVG   {
namespace linearProgramming  {

/// MOSEK wrapper for the LP_Solver
class MOSEK_SolveWrapper : public LP_Solver
{
public :
  MOSEK_SolveWrapper(int nbParams);

  ~MOSEK_SolveWrapper();

  //--
  // Inherited functions :
  //--

  bool setup(const LP_Constraints & constraints);
  bool setup(const LP_Constraints_Sparse & constraints);

  bool solve();

  bool getSolution(std::vector<double> & estimatedParams);

private :
  //MSKenv_t     env;
  MSKtask_t    task; // Solver object.
};


} // namespace linearProgramming
} // namespace openMVG


#endif // MIMATTE_LINEAR_PROGRAMMING_INTERFACE_MOSEK_H_


