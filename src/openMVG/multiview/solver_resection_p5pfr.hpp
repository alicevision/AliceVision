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

#ifndef OPENMVG_MULTIVIEW_RESECTION_P5PFR_H_
#define OPENMVG_MULTIVIEW_RESECTION_P5PFR_H_

#include <iostream>
#include "openMVG/numeric/numeric.h"

namespace openMVG {
	namespace resection {

		// The structure M contain one output model
		struct M {
			double _f;
			Mat _R;
			Vec3 _t;
			double _r;

			M(Mat R, Vec3 t, double r, double f) : _R(R), _t(t), _r(r), _f(f) {}
		};

		/*
		*      Author: Tomas Pajdla, adapted to openMVG by Michal Polic
		* Description: Compute the absolute pose, focal length and radial distorsion of a camera using three 3D-to-2D correspondences
		*   Reference: [1] Time solution to the absolute pose problem with unknown radial distortion and focal length
		*              Kukelova, Z., Bujnak, M., and Pajdla T.
		*              ICCV 2013
		*
		*       Input: featureVectors: 2x5 matrix with feature vectors with principal point at [0; 0] (each column is a vector)
		*              worldPoints: 3x5 matrix with corresponding 3D world points (each column is a point)
		*
		*      Output: solutions: M x n vector that will contain the each solution in structure M (M._R - rotation matrix,
		*						  M._t - translation vector, M._r - radial distorsion , M._f - focal length).
		*/
		struct P5PfrSolver {
			enum { MINIMUM_SAMPLES = 5 };
			enum { MAX_MODELS = 10 };

			// Solve the problem of camera pose.
			static void Solve(const Mat &pt2Dx, const Mat &pt3Dx, std::vector<M> *models);

			// Compute the residual of the projection distance(pt2D, Project(P,pt3D))
			static double Error(const M & model, const Vec2 & pt2D, const Vec3 & pt3D);
		};

	} // namespace resection
} // namespace openMVG

#endif // OPENMVG_MULTIVIEW_RESECTION_P5PFR_H_