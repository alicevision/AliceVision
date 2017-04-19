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

		// Compute the nullspace, choose the algorithm based on input matrix size
		Mat nulls(Mat &A) {
			Mat N;
			if (A.rows() < A.cols()) {		// LU decomposition
				Eigen::FullPivLU<Mat> lu(A);
				N = lu.kernel();	
			} else {						// SVD decomposition 
				Eigen::JacobiSVD<Mat> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV); 
				N = svd.matrixV();
			}
			return N;
		}

		// Inversion of the radial division undistortion to Brown polynomial distortion model conversion
		static bool rddiv2pol(std::vector<M> *solutions, double dmax, Mat di) {
			// TODO ...

			return true;
		}

		// Compute P5Pfr algorithm with radial division undistortion model
		static bool compute_P5Pfr_Poses_RD(const Mat & featureVectors, const Mat & worldPoints, const int num_r, std::vector<M> *solutions) {
			Mat pt2D(featureVectors);
			Mat pt3D(worldPoints);

			// Eliminate all linear stuff
			Mat A = Mat(5, 8);
			for (int i = 0; i < 5; ++i) {
				Mat X = Mat(1, 8);
				X << -pt2D(1, i)*pt3D(0, i), -pt2D(1, i)*pt3D(1, i), -pt2D(1, i)*pt3D(2, i), -pt2D(1, i), pt2D(0, i)*pt3D(0, i), pt2D(0, i)*pt3D(1, i), pt2D(0, i)*pt3D(2, i), pt2D(0, i);
				A.block(i, 0, 1, 8) = X;
			}
			std::cout << "A:\n" << A << "\n\n";

			// 3D Nullspace    
			Mat N = nulls(A);
			std::cout << "N:\n" << N << "\n\n";

			// Construct the matrix C
			Mat C = Mat(2, 6);
			C << N.block(0, 0, 3, 1).transpose()*N.block(4, 0, 3, 1), N.block(0, 0, 3, 1).transpose()*N.block(4, 1, 3, 1) + N.block(0, 1, 3, 1).transpose()*N.block(4, 0, 3, 1), N.block(0, 0, 3, 1).transpose()*N.block(4, 2, 3, 1) + N.block(0, 2, 3, 1).transpose()*N.block(4, 0, 3, 1), N.block(0, 1, 3, 1).transpose()*N.block(4, 1, 3, 1), N.block(0, 1, 3, 1).transpose()*N.block(4, 2, 3, 1) + N.block(0, 2, 3, 1).transpose()*N.block(4, 1, 3, 1), N.block(0, 2, 3, 1).transpose()*N.block(4, 2, 3, 1), N.block(0, 0, 3, 1).transpose()*N.block(0, 0, 3, 1) - N.block(4, 0, 3, 1).transpose()*N.block(4, 0, 3, 1), N.block(0, 0, 3, 1).transpose()*N.block(0, 1, 3, 1) + N.block(0, 1, 3, 1).transpose()*N.block(0, 0, 3, 1) - (N.block(4, 0, 3, 1).transpose()*N.block(4, 1, 3, 1) + N.block(4, 1, 3, 1).transpose()*N.block(4, 0, 3, 1)), N.block(0, 0, 3, 1).transpose()*N.block(0, 2, 3, 1) + N.block(0, 2, 3, 1).transpose()*N.block(0, 0, 3, 1) - (N.block(4, 0, 3, 1).transpose()*N.block(4, 2, 3, 1) + N.block(4, 2, 3, 1).transpose()*N.block(4, 0, 3, 1)), N.block(0, 1, 3, 1).transpose()*N.block(0, 1, 3, 1) - N.block(4, 1, 3, 1).transpose()*N.block(4, 1, 3, 1), N.block(0, 1, 3, 1).transpose()*N.block(0, 2, 3, 1) + N.block(0, 2, 3, 1).transpose()*N.block(0, 1, 3, 1) - (N.block(4, 1, 3, 1).transpose()*N.block(4, 2, 3, 1) + N.block(4, 2, 3, 1).transpose()*N.block(4, 1, 3, 1)), N.block(0, 2, 3, 1).transpose()*N.block(0, 2, 3, 1) - N.block(4, 2, 3, 1).transpose()*N.block(4, 2, 3, 1);
			std::cout << "C:\n" << C << "\n\n";

			// Normalize C to get reasonable numbers when computing d
			Vec c1(C.row(0));
			Vec c2(C.row(1));
			Mat sC = Mat(2, 2);
			sC << (6 / c1.norm()), 0, 0, (6 / c2.norm());
			C = sC * C;
			std::cout << "C:\n" << C << "\n\n";

			// Determinant coefficients
			Mat d = Mat(5, 1);
			d << C(0, 0)*C(0, 0)*C(1, 3)*C(1, 3) - C(0, 0)*C(0, 1)*C(1, 1)*C(1, 3) - 2 * C(0, 0)*C(0, 3)*C(1, 0)*C(1, 3) + C(0, 0)*C(0, 3)*C(1, 1)*C(1, 1) + C(0, 1)*C(0, 1)*C(1, 0)*C(1, 3) - C(0, 1)*C(0, 3)*C(1, 0)*C(1, 1) + C(0, 3)*C(0, 3)*C(1, 0)*C(1, 0), -C(0, 0)*C(0, 1)*C(1, 3)*C(1, 4) + 2 * C(0, 0)*C(0, 2)*C(1, 3)*C(1, 3) + 2 * C(0, 0)*C(0, 3)*C(1, 1)*C(1, 4) - 2 * C(0, 0)*C(0, 3)*C(1, 2)*C(1, 3) - C(0, 0)*C(0, 4)*C(1, 1)*C(1, 3) + C(0, 1)*C(0, 1)*C(1, 2)*C(1, 3) - C(0, 1)*C(0, 2)*C(1, 1)*C(1, 3) - C(0, 1)*C(0, 3)*C(1, 0)*C(1, 4) - C(0, 1)*C(0, 3)*C(1, 1)*C(1, 2) + 2 * C(0, 1)*C(0, 4)*C(1, 0)*C(1, 3) - 2 * C(0, 2)*C(0, 3)*C(1, 0)*C(1, 3) + C(0, 2)*C(0, 3)*C(1, 1)*C(1, 1) + 2 * C(0, 3)*C(0, 3)*C(1, 0)*C(1, 2) - C(0, 3)*C(0, 4)*C(1, 0)*C(1, 1), -2 * C(0, 0)*C(0, 3)*C(1, 3)*C(1, 5) + C(0, 0)*C(0, 3)*C(1, 4)*C(1, 4) - C(0, 0)*C(0, 4)*C(1, 3)*C(1, 4) + 2 * C(0, 0)*C(0, 5)*C(1, 3)*C(1, 3) + C(0, 1)*C(0, 1)*C(1, 3)*C(1, 5) - C(0, 1)*C(0, 2)*C(1, 3)*C(1, 4) - C(0, 1)*C(0, 3)*C(1, 1)*C(1, 5) - C(0, 1)*C(0, 3)*C(1, 2)*C(1, 4) + 2 * C(0, 1)*C(0, 4)*C(1, 2)*C(1, 3) - C(0, 1)*C(0, 5)*C(1, 1)*C(1, 3) + C(0, 2)*C(0, 2)*C(1, 3)*C(1, 3) + 2 * C(0, 2)*C(0, 3)*C(1, 1)*C(1, 4) - 2 * C(0, 2)*C(0, 3)*C(1, 2)*C(1, 3) - C(0, 2)*C(0, 4)*C(1, 1)*C(1, 3) + 2 * C(0, 3)*C(0, 3)*C(1, 0)*C(1, 5) + C(0, 3)*C(0, 3)*C(1, 2)*C(1, 2) - C(0, 3)*C(0, 4)*C(1, 0)*C(1, 4) - C(0, 3)*C(0, 4)*C(1, 1)*C(1, 2) - 2 * C(0, 3)*C(0, 5)*C(1, 0)*C(1, 3) + C(0, 3)*C(0, 5)*C(1, 1)*C(1, 1) + C(0, 4)*C(0, 4)*C(1, 0)*C(1, 3), -C(0, 1)*C(0, 3)*C(1, 4)*C(1, 5) + 2 * C(0, 1)*C(0, 4)*C(1, 3)*C(1, 5) - C(0, 1)*C(0, 5)*C(1, 3)*C(1, 4) - 2 * C(0, 2)*C(0, 3)*C(1, 3)*C(1, 5) + C(0, 2)*C(0, 3)*C(1, 4)*C(1, 4) - C(0, 2)*C(0, 4)*C(1, 3)*C(1, 4) + 2 * C(0, 2)*C(0, 5)*C(1, 3)*C(1, 3) + 2 * C(0, 3)*C(0, 3)*C(1, 2)*C(1, 5) - C(0, 3)*C(0, 4)*C(1, 1)*C(1, 5) - C(0, 3)*C(0, 4)*C(1, 2)*C(1, 4) + 2 * C(0, 3)*C(0, 5)*C(1, 1)*C(1, 4) - 2 * C(0, 3)*C(0, 5)*C(1, 2)*C(1, 3) + C(0, 4)*C(0, 4)*C(1, 2)*C(1, 3) - C(0, 4)*C(0, 5)*C(1, 1)*C(1, 3), C(0, 3)*C(0, 3)*C(1, 5)*C(1, 5) - C(0, 3)*C(0, 4)*C(1, 4)*C(1, 5) - 2 * C(0, 3)*C(0, 5)*C(1, 3)*C(1, 5) + C(0, 3)*C(0, 5)*C(1, 4)*C(1, 4) + C(0, 4)*C(0, 4)*C(1, 3)*C(1, 5) - C(0, 4)*C(0, 5)*C(1, 3)*C(1, 4) + C(0, 5)*C(0, 5)*C(1, 3)*C(1, 3);
			std::cout << "d:\n" << d << "\n\n";

			// Companion matrix
			d = d * (1.0 / d(0, 0));
			Mat M = Mat(4, 4);
			M << 0, 0, 0, -d(4, 0), 1, 0, 0, -d(3, 0), 0, 1, 0, -d(2, 0), 0, 0, 1, -d(1, 0);
			std::cout << "M:\n" << M << "\n\n";

			// solve it
			Eigen::EigenSolver<Mat> es(M);
			Mat g1_im = es.eigenvalues().imag();
			Mat g1_re = es.eigenvalues().real();
			std::cout << "g1_im:\n" << g1_im << "\n\n";
			std::cout << "g1_re:\n" << g1_re << "\n\n";

			// separate real solutions
			double eps = 2.2204e-16;
			std::vector<double> vec_g1_real;
			for (int i = 0; i < 4; ++i) {
				if (abs(g1_im(i, 0)) < eps)
					vec_g1_real.push_back(g1_re(i, 0));
			}
			if (vec_g1_real.size() == 0)
				return false;
			Vec g1 = Map<Vec>(vec_g1_real.data(), vec_g1_real.size());
			//g1 << -0.144182555488459, -0.086894771975386, 0.398661858288110, 2.444406696200684;
			std::cout << "g1:\n" << g1 << "\n\n";

			//get g2 : Sg1 * <g2 ^ 3, g2 ^ 2, g2, 1 >= 0
			//   SG1 : = << C14 | C12*g1 + C15	| C11*g1 ^ 2 + C13*g1 + C16 | 0							>,
			//	           <  0 | C14			| C12*g1 + C15				| C11*g1 ^ 2 + C13*g1 + C16	>,
			//	           <C24 | C22*g1 + C25	| C21*g1 ^ 2 + C23*g1 + C26 | 0							>,
			//	           <  0 | C24			| C22*g1 + C25				| C21*g1 ^ 2 + C23*g1 + C26 >> ;
			Mat g2 = Mat(g1.rows(), g1.cols());
			for (int i = 0; i < g1.rows(); ++i) {
				Mat M2G = Mat(4, 4);
				M2G << C(0, 3), C(0, 1)*g1(i) + C(0, 4), C(0, 0)*g1(i)*g1(i) + C(0, 2)*g1(i) + C(0, 5), 0, 0, C(0, 3), C(0, 1)*g1(i) + C(0, 4), C(0, 0)*g1(i)*g1(i) + C(0, 2)*g1(i) + C(0, 5), C(1, 3), C(1, 1)*g1(i) + C(1, 4), C(1, 0)*g1(i)*g1(i) + C(1, 2)*g1(i) + C(1, 5), 0, 0, C(1, 3), C(1, 1)*g1(i) + C(1, 4), C(1, 0)*g1(i)*g1(i) + C(1, 2)*g1(i) + C(1, 5);
				std::cout << "M2G:\n" << M2G << "\n\n";

				Mat NM2G = nulls(M2G);
				g2(i) = NM2G(2, NM2G.cols() - 1) / NM2G(3, NM2G.cols() - 1);
				std::cout << i << ") NM2G:\n" << NM2G << "\n\n";
				std::cout << "g2(" << i << "):\n" << g2(i) << "\n\n";
			}
			std::cout << "g2:\n" << g2 << "\n\n";

			// Get P for all pairs of solutions[g1, g2]
			for (int i = 0; i < g1.rows(); ++i) {
				// The first two rows of P(P : = zip((g1, g2)->N[1] * g1 + N[2] * g2 + N[3], G1, G2) : )
				Vec4 p1 = N.block(0, 0, 4, 1)*g1(i) + N.block(0, 1, 4, 1)*g2(i) + N.block(0, 2, 4, 1);
				Vec4 p2 = N.block(4, 0, 4, 1)*g1(i) + N.block(4, 1, 4, 1)*g2(i) + N.block(4, 2, 4, 1);
				std::cout << "p1:\n" << p1 << "\n\n";
				std::cout << "p2:\n" << p2 << "\n\n";

				// P{ i }(3, 1:3) = P{ i }(1, 1:3) x P { i }(2, 1:3)
				Mat34 P;
				P.row(0) = ((1 / p1.block(0, 0, 3, 1).norm()) * p1).transpose();
				P.row(1) = ((1 / p2.block(0, 0, 3, 1).norm()) * p2).transpose();
				P.row(2) << P(0, 1)*P(1, 2) - P(0, 2)*P(1, 1), -P(0, 0)*P(1, 2) + P(0, 2)*P(1, 0), P(0, 0)*P(1, 1) - P(0, 1)*P(1, 0), 0;
				std::cout << "P:\n" << P << "\n\n";

				// Form equations on k p34 and t = 1 / f: B <p34, t, k1, k2 ^ 2, k3 ^ 3, 1> = 0
				Mat B = Mat(5, 6);
				for (int j = 0; j < 5; ++j) {		// for all point pairs[u, X]
					double r2 = pt2D(0, j)*pt2D(0, j) + pt2D(1, j)*pt2D(1, j);	// temporary vals
					double ee11 = (P.block(0, 0, 1, 3) * pt3D.col(j))(0, 0) + P(0, 3);
					double ee21 = (P.block(1, 0, 1, 3) * pt3D.col(j))(0, 0) + P(1, 3);
					double ee31 = (P.block(2, 0, 1, 3) * pt3D.col(j))(0, 0);
					double ee32 = pt2D(1, j) * ee31;
					double ee33 = -pt2D(0, j) * ee31;
					std::cout << "r2: " << r2 << ", ee11: " << ee11 << ", ee21: " << ee21
						<< ", ee31: " << ee31 << ", ee32: " << ee32 << ", ee33: " << ee33 << "\n\n";

					if (abs(pt2D(1, j)) > abs(pt2D(0, j))) {
						B.row(j) << pt2D(1, j), ee32, -ee21*r2, -ee21*r2*r2, -ee21*r2*r2*r2, -ee21;
					}
					else {
						B.row(j) << -pt2D(0, j), ee33, ee11*r2, ee11*r2*r2, ee11*r2*r2*r2, ee11;
					}
				}
				std::cout << "B:\n" << B << "\n\n";

				// select columns
				Mat U;
				switch (num_r) {
				case 1:
					U = Mat(6, 4);
					U << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;
					break;
				case 2:
					U = Mat(6, 5);
					U << 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;
					break;
				case 3:
					U = Mat::Identity(6, 6);
					break;
				otherwise:
					std::cerr << "\nError: the number of radial parameters must be between 1 to 3!\n";
					return false;
				}
				B = B * U;
				std::cout << "B:\n" << B << "\n\n";

				// find the right 1D null space		
				Mat NBfull = nulls(B);
				Mat NB = NBfull.col(NBfull.cols() - 1);
				std::cout << "NB:\n" << NB << "\n\n";

				Mat V = NB.col(NB.cols() - 1);
				std::cout << "V:\n" << V << "\n\n";

				Mat tk = V * (1 / V(V.rows() - 1, V.cols() - 1));
				std::cout << "tk:\n" << tk << "\n\n";

				// make f positive
				if (tk(1, 0) < 0) {
					tk.block(0, 0, 2, 1) = -tk.block(0, 0, 2, 1);
					P.block(0, 0, 2, 4) = -P.block(0, 0, 2, 4);
				}

				P(2, 3) = tk(0, 0) / tk(1, 0);
				Mat K = Mat(3, 3);
				K << 1.0 / tk(1, 0), 0, 0, 0, 1.0 / tk(1, 0), 0, 0, 0, 1;
				Mat R = P.block(0, 0, 3, 3);

				Mat C = -R.transpose() * P.block(0, 3, 3, 1);
				Vec r = Vec(num_r);
				r << tk.block(tk.rows() - num_r - 1, 0, num_r, 1);

				// In[1] we have
				// [u - u0][f 0 0]
				// [v - v0] = [0 f 0][R | -R*C][X]
				// [1 + r*((u - u0) ^ 2 + (v - v0) ^ 2)][0 0 1][1]
				// but we want
				// [(u - u0) / f]
				// [(v - v0) / f] = [R | -R*C][X]
				// [1 + (r*f ^ 2)*((u - u0) ^ 2 + (v - v0) ^ 2) / f ^ 2][1]

				// instead not deal with f dependent r
				for (int j = 0; j < num_r; ++j) {		// f^2, f^4, f^6
					std::cout << "pow f^" << 2 * (j + 1) << ":\n" << pow(K(0, 0), 2 * (j + 1)) << "\n\n";
					r(j) *= pow(K(0, 0), 2 * (j + 1));
				}

				std::cout << "K:\n" << K << "\n\n";
				std::cout << "R:\n" << R << "\n\n";
				std::cout << "C:\n" << C << "\n\n";
				std::cout << "r:\n" << r << "\n\n";

				Vec3 t = P.block(0, 3, 3, 1);
				double f = (1.0 / tk(1, 0));
				std::cout << "t:\n" << t << "\n\n";
				std::cout << "f:\n" << f << "\n\n";

				// output
				resection::M model(R, t, r, f);
				solutions->push_back(model);

				double pause = 1;
			}


			double end = 0;
		}

		// Compute compute_P5Pfr_Poses_RD and transform the radial division undistortion to Brown polynomial distortion model
		static bool compute_P5Pfr_Poses_RP(const Mat & featureVectors, const Mat & worldPoints, const int num_r, std::vector<M> *solutions) {
			if (compute_P5Pfr_Poses_RD(featureVectors, worldPoints, num_r, solutions)) 
				return rddiv2pol(solutions, 1, featureVectors);
			return false;
		}

		// Compute the reprojection error for the radial division undistortion model
		static double reproj_error_RD(const M & m, const Vec2 & pt2D, const Vec3 & pt3D) {
			if (m._r.rows() > 1)
				std::cerr << "Projection function is not implemented for the radial division undistortion model for more than one parameter.\n";

			std::cout << "m._R:\n" << m._R << "\n\n";
			std::cout << "m._t:\n" << m._t << "\n\n";
			std::cout << "pt3D:\n" << pt3D << "\n\n";

			Vec3 v = m._R * pt3D + m._t;							// from delta to epsilon
			std::cout << "v:\n" << v << "\n\n";

			v *= 1.0 / v(2);										// normalize to have v(3, :) = 1
			std::cout << "v:\n" << v << "\n\n";

			double ru2 = v(0)*v(0) + v(1)*v(1);						// undistorted squared radius
			std::cout << "ru2:\n" << ru2 << "\n\n";

			// works for fish - eye, i.e.when distorte image gets smaller on the image plane
			double h1 = sqrt(-4 * m._r(0) * ru2 + 1);
			std::cout << "h1:\n" << h1 << "\n\n";

			double h2 = 0.5 * ((-2 * m._r(0) * ru2 + 1) - h1) * (1 / (m._r(0)*m._r(0)));
			std::cout << "h2:\n" << h2 << "\n\n";

			double rd = sqrt(h2 * (1 / ru2));
			std::cout << "rd:\n" << rd << "\n\n";

			// distort in epsilon
			double h3 = rd / sqrt(ru2);
			Vec2 u;
			u << v(0) * h3, v(1) * h3;
			std::cout << "u:\n" << u << "\n\n";

			// to alpha
			u = m._f * u;
			std::cout << "u:\n" << u << "\n\n";
			std::cout << "res:\n" << (pt2D - u).norm() << "\n\n";
			return (pt2D - u).norm();
		}

		// Compute the reprojection error for Brown polynomial distortion model
		static double reproj_error_RP(const M & m, const Vec2 & pt2D, const Vec3 & pt3D) {
			// ...
			return 0;
		}


		// Solve the resection problem with unknown R,t,r,f
		void P5PfrSolver::Solve(const Mat &pt2Dx, const Mat &pt3Dx, const int num_r, std::vector<M> *models){
			assert(2 == pt2Dx.rows());
			assert(3 == pt3Dx.rows());
			assert(5 == pt3Dx.cols());
			assert(5 == pt2Dx.cols());

			// The radial distorision is represented by: the radial division undistortion
			if( !compute_P5Pfr_Poses_RD(pt2Dx, pt3Dx, num_r, models) )
				models->clear();

			// The radial distorision is represented by: Brown polynomial distortion model
			/*if (!compute_P5Pfr_Poses_RP(pt2Dx, pt3Dx, num_r, models))
				models->clear();*/
		}

		// Compute the residual of the projection distance(pt2D, Project(M,pt3D))
		double P5PfrSolver::Error(const M & m, const Vec2 & pt2D, const Vec3 & pt3D){
			return reproj_error_RD( m, pt2D, pt3D);
			//return reproj_error_RP( m, pt2D, pt3D);
		}

	}  // namespace resection
}  // namespace openMVG

#endif // OPENMVG_MULTIVIEW_RESECTION_P5PFR_CPP