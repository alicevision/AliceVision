// This file is part of the AliceVision project.
// Copyright (c) 2018 AliceVision contributors.
// Copyright (c) 2015 Jan Heller <hellej1@cmp.felk.cvut.cz>, Zuzana Kukelova <zukuke@microsoft.com>
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/numeric/numeric.hpp>

#include <Eigen/Dense>

#include <cmath>
#include <limits>
#include <vector>

namespace aliceVision {

using Mat21 = Eigen::Matrix<double, 2, 1>;

/**
 * @brief Computes the relative pose and two radial disortion coefficients for two cameras from 10 correspondences
 *        Epipolar geometry F + l1 + l2 solver F10e:
 * @code{.unparsed}
 *                          [F11 F12 F13] [u]
 * [x, y, 1+l1*(x^2+y^2)] * [F21 F22 F23] [v]               = 0
 *                          [F31 F32 F33] [1+l2*(u^2+v^2)]
 * @endcode
 * @param[in] X 10x2 matrix of 2D distorted measurements (X == [x y]')
 * @param[in] U 10x2 matrix of 2D distorted measurements (U == [u v]')
 * @param[out] F list of candidate fundamental matrices solutions
 * @param[out] L list of candidate radial disortion solutions (L[i] = [l1 l2]').
 * @return nsols
 *
 * @author	Zuzana Kukelova, Jan Heller, Martin Bujnak, Andrew Fitzgibbon, Tomas Pajdla, adapted to aliceVision by Michal Polic
 * @ref [1]	Zuzana Kukelova, Jan Heller, Martin Bujnak, Andrew Fitzgibbon, Tomas Pajdla:
 *			Efficient Solution to the Epipolar Geometry for Radially Distorted Cameras,
 *			The IEEE International Conference on Computer Vision (ICCV),
 *			December, 2015, Santiago, Chile.
 */
int F10RelativePose(const Mat& X, const Mat& U, std::vector<Mat3>& F, std::vector<Mat21>& L);

} // namespace aliceVision
