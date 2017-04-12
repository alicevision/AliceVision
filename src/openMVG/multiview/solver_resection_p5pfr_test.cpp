// Copyright (c) 2010 libmv authors.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to
// deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.


// Copyright (c) 2012, 2013 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include <vector>
#include "openMVG/multiview/solver_resection_p5pfr.hpp"
#include "testing/testing.h"

using namespace std;
using namespace openMVG;

TEST(Resection_P5Pfr, RealExample) {
	// DATA
	Mat pt2D = Mat(2, 5);
	Mat pt3D = Mat(3, 5);
	pt2D << 480.379999999999880, 542.209999999999810, 1508.149999999999900, -1640.501000000000000, -402.670000000000070, -1326.581000000000100, -713.868000000000050, -1234.742999999999900, -622.277000000000040, -1461.412100000000000;
	pt3D << 3.17536, 2.53122, 2.42089, 0.75785, 3.88086, -0.46800, 0.19650, -1.86325, 0.16480, 0.41775, -0.86051, -1.50740, -1.82808, 1.20881, 0.29809;

	// PROCESS
	std::vector<resection::M> models;
	resection::P5PfrSolver::Solve(pt2D, pt3D, &models);


	EXPECT_TRUE(false);
}


/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
