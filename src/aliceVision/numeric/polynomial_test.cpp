// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "aliceVision/numeric/numeric.hpp"
#include "aliceVision/numeric/polynomial.hpp"

#define BOOST_TEST_MODULE polynomial

#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>

using namespace aliceVision;

// Find the polynomial coefficients of x in the equation
//
//   (x - a)(x - b)(x - c) == 0
//
// by expanding to
//
//   x^3 - (c+b+a) * x^2 + (a*b+(b+a)*c) * x - a*b*c = 0.
//           = p               = q              = r
void CoeffsForCubicZeros(double a, double b, double c,
                    double *p, double *q, double *r) {
  *p = -(c + b + a);
  *q = (a * b + (b + a) * c);
  *r = -a * b * c;
}
// Find the polynomial coefficients of x in the equation
//
//   (x - a)(x - b)(x - c)(x - d) == 0
//
// by expanding to
//
//   x^4 - (d+c+b+a) * x^3 + (d*(c+b+a) + a*b+(b+a)*c) * x^2
//   - (d*(a*b+(b+a)*c)+a*b*c) * x + a*b*c*d = 0.
void CoeffsForQuarticZeros(double a, double b, double c, double d,
                    double *p, double *q, double *r, double *s) {
  *p = -(d + c + b + a);
  *q = (d * (c + b + a) + a * b + (b + a) * c);
  *r = -(d * (a * b + (b + a) * c) + a * b * c);
  *s = a * b * c *d;
}

BOOST_AUTO_TEST_CASE(Poly_SolveCubicPolynomial) {
  double a, b, c, aa, bb, cc;
  double p, q, r;

  a = 1; b = 2; c = 3;
  CoeffsForCubicZeros(a, b, c, &p, &q, &r);
  BOOST_CHECK_EQUAL(3, SolveCubicPolynomial(p,q,r, &aa, &bb, &cc));
  BOOST_CHECK_SMALL(a-aa, 1e-10);
  BOOST_CHECK_SMALL(b-bb, 1e-10);
  BOOST_CHECK_SMALL(c-cc, 1e-10);

  a = 0; b = 1; c = 3;
  CoeffsForCubicZeros(a, b, c, &p, &q, &r);
  BOOST_CHECK_EQUAL(3, SolveCubicPolynomial(p,q,r, &aa, &bb, &cc));
  BOOST_CHECK_SMALL(a-aa, 1e-10);
  BOOST_CHECK_SMALL(b-bb, 1e-10);
  BOOST_CHECK_SMALL(c-cc, 1e-10);

  a = -10; b = 0; c = 1;
  CoeffsForCubicZeros(a, b, c, &p, &q, &r);
  BOOST_CHECK_EQUAL(3, SolveCubicPolynomial(p,q,r, &aa, &bb, &cc));
  BOOST_CHECK_SMALL(a-aa, 1e-10);
  BOOST_CHECK_SMALL(b-bb, 1e-10);
  BOOST_CHECK_SMALL(c-cc, 1e-10);

  a = -8; b = 1; c = 3;
  CoeffsForCubicZeros(a, b, c, &p, &q, &r);
  BOOST_CHECK_EQUAL(3, SolveCubicPolynomial(p,q,r, &aa, &bb, &cc));
  BOOST_CHECK_SMALL(a-aa, 1e-10);
  BOOST_CHECK_SMALL(b-bb, 1e-10);
  BOOST_CHECK_SMALL(c-cc, 1e-10);

  a = 28; b = 28; c = 105;
  CoeffsForCubicZeros(a, b, c, &p, &q, &r);
  BOOST_CHECK_EQUAL(3, SolveCubicPolynomial(p,q,r, &aa, &bb, &cc));
  BOOST_CHECK_SMALL(a-aa, 1e-10);
  BOOST_CHECK_SMALL(b-bb, 1e-10);
  BOOST_CHECK_SMALL(c-cc, 1e-10);
}
