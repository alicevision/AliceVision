// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "aliceVision/numeric/numeric.hpp"

#include <vector>
#include <utility>

namespace aliceVision   {
namespace linearProgramming  {

/// Generic container for LP (Linear Programming problems).
/// Embed :
///  - objective function
///  - Constraints (coefficients, Sign, objective value),
///  - Bounds over parameter (<=, =, >=).
///  - minimize or maximize
///
struct LPConstraints
{
  enum eLP_SIGN
  {
    LP_LESS_OR_EQUAL    = 1,  // (<=)
    LP_GREATER_OR_EQUAL = 2,  // (>=)
    LP_EQUAL            = 3,   // (=)
    LP_FREE             = 4 //only supported in MOSEK
  };

  LPConstraints() {
    _bminimize = false;
  }

  int _nbParams; // The number of parameter/variable in constraint.
  Mat _constraintMat; // Constraint under Matrix form.
  Vec _Cst_objective; // Constraint objective value.
  std::vector<eLP_SIGN> _vec_sign; // Constraint sign.
  std::vector< std::pair<double, double> > _vec_bounds; // parameter/variable bounds.

  bool _bminimize; // minimize is true or maximize is false.
  std::vector<double> _vec_cost; // Objective function
};

/// Generic Sparse container for LP (Linear Programming problems).
/// Embed :
///  - Constraints (coefficients, Sign, objective value),
///  - Bounds over parameter (<=, =, >=).
/// Implementation differ from LPConstraints, here constraints are
///  stored as a Sparse matrix.
///
struct LPConstraintsSparse
{
  LPConstraintsSparse() {
    _bminimize = false;
  }

  // Variable part
  int _nbParams; // The number of parameter/variable in constraint.
  std::vector< std::pair<double, double> > _vec_bounds; // parameter/variable bounds.

  // Constraint part
  sRMat _constraintMat; // Constraint under Matrix form.
  Vec _Cst_objective; // Constraint objective value.
  std::vector<LPConstraints::eLP_SIGN> _vec_sign; // Constraint sign.

  bool _bminimize; // minimize is true or maximize is false.
  std::vector<double> _vec_cost; // Objective function
};

/// Generic LP solver (Linear Programming)
/// It's an interface to setup constraint and objective of a Linear Program.
/// Embed constraint setup, problem solving, and parameters getter.
class ISolver
{
public:

  ISolver(int nbParams):_nbParams(nbParams){};

  /// Setup constraint for the given library.
  virtual bool setup(const LPConstraints & constraints) = 0;
  virtual bool setup(const LPConstraintsSparse & constraints) = 0;

  /// Setup the feasibility and found the solution that best fit the constraint.
  virtual bool solve() = 0;

  /// Get back solution. Call it after solve.
  virtual bool getSolution(std::vector<double> & estimatedParams) = 0;

protected :
  int _nbParams; // The number of parameter considered in constraint formulation.
};

} // namespace linearProgramming
} // namespace aliceVision
