// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2015 openMVG contributors.
// Copyright (c) 2014 Kyle Wilson.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/multiview/translationAveraging/common.hpp>
#include <aliceVision/multiview/translationAveraging/solver.hpp>
#include <aliceVision/config.hpp>
#include <aliceVision/alicevision_omp.hpp>
#include <aliceVision/system/Logger.hpp>

#include "ceres/ceres.h"

#include <ctime>
#include <vector>
#include <set>
#include <map>

namespace aliceVision {
namespace translationAveraging {

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;
using ceres::HuberLoss;

struct ChordFunctor {
  ChordFunctor(const double *direction, double weight)
    : u_(direction), w_(weight){}

  const double *u_; // local estimate of the translation direction
  const double w_;  // weight

  // Residual computation (3x1: a residual along each unit axis)
  template <typename T>
  bool operator()(const T* const x0, const T* const x1, T* residual) const
  {
    // compute ||x1 - x0||_2
    T norm = sqrt((x1[0]-x0[0])*(x1[0]-x0[0]) +
      (x1[1]-x0[1])*(x1[1]-x0[1]) +
      (x1[2]-x0[2])*(x1[2]-x0[2]));
    residual[0] = w_*((x1[0]-x0[0]) / norm - T(u_[0]));
    residual[1] = w_*((x1[1]-x0[1]) / norm - T(u_[1]));
    residual[2] = w_*((x1[2]-x0[2]) / norm - T(u_[2]));
    return true;
  }
};

void reindex_problem(int* edges, int num_edges, std::vector<int> &reindex_lookup);

bool solve_translations_problem_l2_chordal(
  const int* edges,
  const double* poses,
  const double* weights,
  int num_edges,
  double loss_width,
  double* X,
  double function_tolerance,
  double parameter_tolerance,
  int max_iterations)
{

  // re index the edges to be a sequential set
  std::vector<int> _edges(edges, edges+2*num_edges);
  std::vector<int> reindex_lookup;
  reindex_problem(&_edges[0], num_edges, reindex_lookup);
  const int num_nodes = reindex_lookup.size();

  // Init with a random guess solution
  const std::size_t guessSize = 3*num_nodes;
  std::vector<double> x(guessSize);
  Mat randGuesses = Mat::Random(1, guessSize);
  for (std::size_t i = 0; i < guessSize; ++i)
  {
    x[i] = randGuesses(0, i);
  }

  // add the parameter blocks (a 3-vector for each node)
  Problem problem;
  for (int i=0; i<num_nodes; ++i)
    problem.AddParameterBlock(&x[3*i], 3);

  // set the residual function (chordal distance for each edge)
  for (int i=0; i<num_edges; ++i) {
    CostFunction* cost_function =
      new AutoDiffCostFunction<ChordFunctor, 3, 3, 3>(
      new ChordFunctor(poses+3*i, weights[i]));

    if (loss_width == 0.0) {
      // No robust loss function
      problem.AddResidualBlock(cost_function, nullptr, &x[3*_edges[2*i+0]], &x[3*_edges[2*i+1]]);
    } else {
      problem.AddResidualBlock(cost_function, new ceres::HuberLoss(loss_width), &x[3*_edges[2*i+0]], &x[3*_edges[2*i+1]]);
    }
  }

  // Fix first camera in {0,0,0}: fix the translation ambiguity
  x[0] = x[1] = x[2] = 0.0;
  problem.SetParameterBlockConstant(&x[0]);

  // solve
  Solver::Options options;
  // set number of threads, 1 if openMP is not enabled
  options.num_threads = omp_get_max_threads();
#if CERES_VERSION_MAJOR < 2
  options.num_linear_solver_threads = omp_get_max_threads();
#endif

  //options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = max_iterations;
  options.function_tolerance = function_tolerance;
  options.parameter_tolerance = parameter_tolerance;
  if (ceres::IsSparseLinearAlgebraLibraryTypeAvailable(ceres::SUITE_SPARSE) ||
      ceres::IsSparseLinearAlgebraLibraryTypeAvailable(ceres::CX_SPARSE) ||
      ceres::IsSparseLinearAlgebraLibraryTypeAvailable(ceres::EIGEN_SPARSE))
  {
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  }
  else
  {
    options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
  }

  Solver::Summary summary;
  Solve(options, &problem, &summary);

  ALICEVISION_LOG_DEBUG(summary.FullReport());

  if (summary.IsSolutionUsable())
  {
    // undo the re indexing
    for (int i=0; i<num_nodes; ++i) {
      const int j = reindex_lookup[i];
      X[3*j+0] = x[3*i+0];
      X[3*j+1] = x[3*i+1];
      X[3*j+2] = x[3*i+2];
    }
  }
  return summary.IsSolutionUsable();
}

void
reindex_problem(int* edges, int num_edges, std::vector<int> &reindex_lookup)
{
  // get the unique set of nodes
  std::set<int> nodes;
  for (int i=0; i<2*num_edges; ++i)
    nodes.insert(edges[i]);

  reindex_lookup.clear();
  reindex_lookup.reserve(nodes.size());

  std::map<int, int> reindexing_key;
  // iterator through them and assign a new Id to each vertex
  std::set<int>::const_iterator it;
  int n=0;
  for (it = nodes.begin(); it != nodes.end(); ++it, ++n) {
    reindex_lookup.push_back(*it);
    reindexing_key[*it] = n;
  }

  // now renumber the edges
  for (int i=0; i<2*num_edges; ++i)
    edges[i]  = reindexing_key[edges[i]];
}

} // namespace translationAveraging
} // namespace aliceVision
