// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

namespace aliceVision {
namespace translationAveraging {

/**
 * @brief Compute camera center positions from relative camera translations (translation directions).
 *
 * Implementation of [1] : "5. Solving the Translations Problem" equation (3)
 */
bool solve_translations_problem_l2_chordal(
  const int* edges,
  const double* poses,
  const double* weights,
  int num_edges,
  double loss_width,
  double* X,
  double function_tolerance,
  double parameter_tolerance,
  int max_iterations
);


/**
 * @brief Registration of relative translations to global translations. It implements LInf minimization of  [2]
 *  as a SoftL1 minimization. It can use 2/3 views relative translation vectors (bearing, or triplets of translations).
 *
 * [1] "Robust Global Translations with 1DSfM."
 * Authors: Kyle Wilson and Noah Snavely.
 * Date: September 2014.
 * Conference: ECCV.
 *
 * [2] "Global Fusion of Relative Motions for Robust, Accurate and Scalable Structure from Motion."
 * Authors: Pierre Moulon, Pascal Monasse and Renaud Marlet.
 * Date: December 2013.
 * Conference: ICCV.
 *
 * Based on a BSD implementation from Kyle Wilson.
 * See http://github.com/wilsonkl/SfM_Init
 *
 * @param[in] vec_initial_estimates relative motion information
 * @param[in] b_translation_triplets tell if relative motion comes 3 or 2 views
 *   false: 2-view estimates -> 1 relativeInfo per 2 view estimates,
 *   true:  3-view estimates -> triplet of translations: 3 relativeInfo per triplet.
 * @param[in] nb_poses the number of camera nodes in the relative motion graph
 * @param[out] translations found global camera translations
 * @param[in] d_l1_loss_threshold optionnal threshold for SoftL1 loss (-1: no loss function)
 * @return True if the registration can be solved
 */
bool
solve_translations_problem_softl1
(
  const std::vector<relativeInfo> & vec_initial_estimates,
  const bool b_translation_triplets,
  const int nb_poses,
  std::vector<Eigen::Vector3d> & translations,
  const double d_l1_loss_threshold = 0.01
);

} // namespace translationAveraging
} // namespace aliceVision
