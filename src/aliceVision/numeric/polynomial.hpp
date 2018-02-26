// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <algorithm>
#include <cmath>
#include <stdio.h>

namespace aliceVision {

// Solve the cubic polynomial
//
//   x^3 + a*x^2 + b*x + c = 0
//
// The number of roots (from zero to three) is returned. If the number of roots
// is less than three, then higher numbered x's are not changed. For example,
// if there are 2 roots, only x0 and x1 are set.
//
// The GSL cubic solver was used as a reference for this routine.
template<typename Real>
int SolveCubicPolynomial(Real a, Real b, Real c,
                         Real *x0, Real *x1, Real *x2) {
  Real q = a * a - 3 * b;
  Real r = 2 * a * a * a - 9 * a * b + 27 * c;

  Real Q = q / 9;
  Real R = r / 54;

  Real Q3 = Q * Q * Q;
  Real R2 = R * R;

  Real CR2 = 729 * r * r;
  Real CQ3 = 2916 * q * q * q;

  if (R == 0 && Q == 0) {
    // Tripple root in one place.
    *x0 = *x1 = *x2 = -a / 3 ;
    return 3;

  } else if (CR2 == CQ3) {
    // This test is actually R2 == Q3, written in a form suitable for exact
    // computation with integers.
    //
    // Due to finite precision some double roots may be missed, and considered
    // to be a pair of complex roots z = x +/- epsilon i close to the real
    // axis.
    Real sqrtQ = sqrt (Q);
    if (R > 0) {
      *x0 = -2 * sqrtQ - a / 3;
      *x1 =      sqrtQ - a / 3;
      *x2 =      sqrtQ - a / 3;
    } else {
      *x0 =     -sqrtQ - a / 3;
      *x1 =     -sqrtQ - a / 3;
      *x2 =  2 * sqrtQ - a / 3;
    }
    return 3;

  } else if (CR2 < CQ3) {
    // This case is equivalent to R2 < Q3.
    Real sqrtQ = sqrt (Q);
    Real sqrtQ3 = sqrtQ * sqrtQ * sqrtQ;
    Real theta = acos (R / sqrtQ3);
    Real norm = -2 * sqrtQ;
    *x0 = norm * cos (theta / 3) - a / 3;
    *x1 = norm * cos ((theta + 2.0 * M_PI) / 3) - a / 3;
    *x2 = norm * cos ((theta - 2.0 * M_PI) / 3) - a / 3;

    // Put the roots in ascending order.
    if (*x0 > *x1) {
      std::swap(*x0, *x1);
    }
    if (*x1 > *x2) {
      std::swap(*x1, *x2);
      if (*x0 > *x1) {
        std::swap(*x0, *x1);
      }
    }
    return 3;
  }
  Real sgnR = (R >= 0 ? 1 : -1);
  Real A = -sgnR * pow (fabs (R) + sqrt (R2 - Q3), 1.0/3.0);
  Real B = Q / A ;
  *x0 = A + B - a / 3;
  return 1;
}

// The coefficients are in ascending powers, i.e. coeffs[N]*x^N.
template<typename Real>
int SolveCubicPolynomial(const Real *coeffs, Real *solutions) {
  if (coeffs[0] == 0.0) {
    // TODO(keir): This is a quadratic not a cubic. Implement a quadratic
    // solver!
    return 0;
  }
  Real a = coeffs[2] / coeffs[3];
  Real b = coeffs[1] / coeffs[3];
  Real c = coeffs[0] / coeffs[3];
  return SolveCubicPolynomial(a, b, c,
                              solutions + 0,
                              solutions + 1,
                              solutions + 2);
}
}  // namespace aliceVision
