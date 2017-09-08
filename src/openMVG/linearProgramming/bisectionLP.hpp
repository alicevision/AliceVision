// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#ifndef OPENMVG_LINEAR_PROGRAMMING_BISECTIONLP_H_
#define OPENMVG_LINEAR_PROGRAMMING_BISECTIONLP_H_

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/linearProgramming/linearProgrammingInterface.hpp>

#include <iostream>
#include <iterator>
#include <vector>

namespace aliceVision   {
namespace linearProgramming  {

/// Generic Bisection algorithm via Linear Programming.
/// Use dichotomy or mid-point best parameter that fit the solution.
/// http://en.wikipedia.org/wiki/Bisection_method
/// The bisection algorithm continue as long as
///  precision or max iteration number is not reach.
///
template <typename ConstraintBuilder, typename ConstraintType>
bool BisectionLP(
  LP_Solver & solver,
  ConstraintBuilder & cstraintBuilder,
  std::vector<double> * parameters,
  double gammaUp  = 1.0,  // Upper bound
  double gammaLow = 0.0,  // lower bound
  double eps      = 1e-8, // precision that stop dichotomy
  const int maxIteration = 20, // max number of iteration
  double * bestFeasibleGamma = nullptr, // value of best bisection found value
  bool bVerbose = false)
{
  int k = 0;
  bool bModelFound = false;
  ConstraintType constraint;
  do
  {
    ++k; // One more iteration

    double gamma = (gammaLow + gammaUp) / 2.0;

    //-- Setup constraint and solver
    cstraintBuilder.Build(gamma, constraint);
    solver.setup( constraint );
    //--
    // Solving
    bool bFeasible = solver.solve();
    //--

    if (bFeasible)
    {
      gammaUp = gamma;
      if (bestFeasibleGamma)
        *bestFeasibleGamma = gamma;
      solver.getSolution(*parameters);
      bModelFound = true;

      if(bVerbose)
        OPENMVG_LOG_DEBUG(k << "/" << maxIteration
          << "\t gamma " << gamma
          << "\t gammaUp-gammaLow " << gammaUp-gammaLow);
    }
    else
    {
      gammaLow = gamma;
      if(bVerbose)
        OPENMVG_LOG_DEBUG("Not feasible with gamma: " << gamma);
    }
  } while (k < maxIteration && gammaUp - gammaLow > eps);

  return bModelFound;
}

} // namespace linearProgramming
} // namespace aliceVision


#endif // OPENMVG_LINEAR_PROGRAMMING_BISECTIONLP_H_
