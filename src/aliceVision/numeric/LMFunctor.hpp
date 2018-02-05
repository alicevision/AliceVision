// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "aliceVision/numeric/numeric.hpp"
// Levenberg Marquardt Non Linear Optimization
#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>

namespace aliceVision
{
  using namespace Eigen;
// Generic functor Levenberg-Marquardt minimization
template<typename _Scalar, int NX=Dynamic, int NY=Dynamic>
struct LMFunctor
{
  typedef _Scalar Scalar;
  enum {
    InputsAtCompileTime = NX,
    ValuesAtCompileTime = NY
  };
  typedef Matrix<Scalar,InputsAtCompileTime,1> InputType;
  typedef Matrix<Scalar,ValuesAtCompileTime,1> ValueType;
  typedef Matrix<Scalar,ValuesAtCompileTime,InputsAtCompileTime> JacobianType;

  const int m_inputs, m_values;

  LMFunctor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}
  LMFunctor(int inputs, int values) : m_inputs(inputs), m_values(values) {}

  int inputs() const { return m_inputs; }
  int values() const { return m_values; }

  // you should define that in the subclass :
  //  void operator() (const InputType& x, ValueType* v, JacobianType* _j=0) const;
};

}; // namespace aliceVision
