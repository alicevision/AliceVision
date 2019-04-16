// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// Copyright (c) 2010 libmv contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/numeric/numeric.hpp>

#include <vector>

namespace aliceVision {

/**
 * @brief Compute the relative camera motion between two cameras.
 *        Given the motion parameters of two cameras, computes the motion parameters
 *        of the second one assuming the first one to be at the origin.
 *        If T1 and T2 are the camera motions, the computed relative motion is
 *
 *          T = T2 T1^{-1}
 */
void relativeCameraMotion(const Mat3& R1,
                          const Vec3& t1,
                          const Mat3& R2,
                          const Vec3& t2,
                          Mat3* R,
                          Vec3* t);
/**
 * @brief Given F, Left/Right K matrix it compute the Essential matrix
 */
void essentialFromFundamental(const Mat3& F,
                              const Mat3& K1,
                              const Mat3& K2,
                              Mat3* E);
/**
 * @brief Compute E as E = [t12]x R12.
 */
void essentialFromRt(const Mat3& R1,
                     const Vec3& t1,
                     const Mat3& R2,
                     const Vec3& t2,
                     Mat3* E);
/**
 * @brief Given E, Left/Right K matrix it compute the Fundamental matrix
 */
void fundamentalFromEssential(const Mat3& E,
                              const Mat3& K1,
                              const Mat3& K2,
                              Mat3* F);
/**
 * @brief Test the possible R|t configuration to have point in front of the cameras
 *        Return false if no possible configuration
 */
bool motionFromEssentialAndCorrespondence(const Mat3& E,
                                          const Mat3& K1,
                                          const Vec2& x1,
                                          const Mat3& K2,
                                          const Vec2& x2,
                                          Mat3* R,
                                          Vec3* t);
/**
 * @brief Choose one of the four possible motion solutions from an essential matrix.
 *        Decides the right solution by checking that the triangulation of a match
 *        x1--x2 lies in front of the cameras.
 *        Return the index of the right solution or -1 if no solution.
 */
int motionFromEssentialChooseSolution(const std::vector<Mat3>& Rs,
                                      const std::vector<Vec3>& ts,
                                      const Mat3& K1,
                                      const Vec2& x1,
                                      const Mat3& K2,
                                      const Vec2& x2);
/**
 * @brief HZ 9.7 page 259 (Result 9.19)
 */
void motionFromEssential(const Mat3& E, std::vector<Mat3>* Rs, std::vector<Vec3>* ts);

} // namespace aliceVision
