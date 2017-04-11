
/*
* Copyright (c) 2011, Laurent Kneip, ETH Zurich
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*     * Neither the name of ETH Zurich nor the
*       names of its contributors may be used to endorse or promote products
*       derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL ETH ZURICH BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

// Copyright (c) 2012, 2013 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef OPENMVG_MULTIVIEW_RESECTION_P5PFR_CPP
#define OPENMVG_MULTIVIEW_RESECTION_P5PFR_CPP

#include "openMVG/multiview/projection.hpp"
#include "openMVG/numeric/numeric.h"
#include "openMVG/multiview/solver_resection_p5pfr.hpp"

#include <cmath>
#include <iostream>

namespace openMVG {
	namespace resection {

		// Solve the resection problem with unknown R,t,r,f
		void P5PfrSolver::Solve(const Mat &pt2Dx, const Mat &pt3Dx, std::vector<M> *models){
			Mat pt2D(pt2Dx);
			Mat pt3D(pt3Dx);
			assert(2 == pt2D.rows());
			assert(3 == pt3D.rows());
			assert(5 == pt3D.cols());
			assert(5 == pt2D.cols());

			// Eliminate all linear stuff
			Mat A = Mat(5, 8); 
			for (int i = 0; i < 5; ++i) {
				Mat X = Mat(1, 8);
				X << -pt2D(1, i)*pt3D(0, i), -pt2D(1, i)*pt3D(1, i), -pt2D(1, i)*pt3D(2, i), -pt2D(1, i), pt2D(0, i)*pt3D(0, i), pt2D(0, i)*pt3D(1, i), pt2D(0, i)*pt3D(2, i), pt2D(0, i);
				A.block(i, 0, 1, 8) = X;
			}
			std::cout << "A:\n" << A << "\n\n";
						



			double end = 0;
		}

		// Compute the residual of the projection distance(pt2D, Project(M,pt3D))
		double P5PfrSolver::Error(const M & model, const Vec2 & pt2D, const Vec3 & pt3D){
			return 0; //(pt2D - Project(model.getP(), pt3D)).norm();
		}

	}  // namespace resection
}  // namespace openMVG

#endif // OPENMVG_MULTIVIEW_RESECTION_P5PFR_CPP