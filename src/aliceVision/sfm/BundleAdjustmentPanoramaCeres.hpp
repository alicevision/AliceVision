// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/types.hpp>
#include <aliceVision/alicevision_omp.hpp>
#include <aliceVision/sfm/BundleAdjustment.hpp>
#include <aliceVision/sfm/LocalBundleAdjustmentGraph.hpp>

#include <ceres/ceres.h>
#include "liealgebra.hpp"


namespace aliceVision {


namespace sfmData {
class SfMData;
} // namespace sfmData

namespace sfm {

class BundleAdjustmentPanoramaCeres : public BundleAdjustment
{
public:

  /**
   * @brief Contains all ceres parameters.
   */
  struct CeresOptions
  {
    CeresOptions(bool verbose = true, bool multithreaded = true)
      : verbose(verbose)
      , nbThreads(multithreaded ? omp_get_max_threads() : 1) // set number of threads, 1 if OpenMP is not enabled
    {
      setDenseBA(); // use dense BA by default
    }

    void setDenseBA();
    void setSparseBA();

    ceres::LinearSolverType linearSolverType;
    ceres::PreconditionerType preconditionerType;
    ceres::SparseLinearAlgebraLibraryType sparseLinearAlgebraLibraryType;
    unsigned int nbThreads;
    bool summary = false;
    bool verbose = true;
  };

  /**
   * @brief Contains all informations related to the performed bundle adjustment.
   */
  struct Statistics
  {
    Statistics()
    {}

    /**
     * @brief Add a parameter state
     * @param[in] parameter A bundle adjustment parameter
     * @param[in] state A bundle adjustment state
     */
    inline void addState(EParameter parameter, EParameterState state)
    {
      ++parametersStates[parameter][state];
    }

    /**
     * @brief Export statistics about bundle adjustment in a CSV file
     *  The contents of the file have been writen such that it is easy to handle it with
     *  a Python script or any spreadsheets (e.g. by copy/past the full content to LibreOffice)
     * @param[in] folder The folder where you want to save the statistics file
     * @param[in] filename The filename of the statistics file
     * @return false it cannot open the file, true if it succeed
     */
    bool exportToFile(const std::string& folder, const std::string& filename = "statistics.csv") const;

    /**
     * @brief  Display statistics about bundle adjustment in the terminal
     *  Logger need to accept <info> log level
     */
    void show() const;

    // public members

    /// number of successful iterations
    std::size_t nbSuccessfullIterations = 0;
    /// number of unsuccessful iterations
    std::size_t nbUnsuccessfullIterations = 0;
    /// number of resiudal blocks in the Ceres problem
    std::size_t nbResidualBlocks = 0;
    /// RMSEinitial: sqrt(initial_cost / num_residuals)
    double RMSEinitial = 0.0;
    /// RMSEfinal: sqrt(final_cost / num_residuals)
    double RMSEfinal = 0.0;
    /// time spent to solve the BA (s)
    double time = 0.0;
    /// number of states per parameter
    std::map<EParameter, std::map<EParameterState, std::size_t>> parametersStates;
    /// The distribution of the cameras for each graph distance <distance, numOfCam>
    std::map<int, std::size_t> nbCamerasPerDistance;
  };

  /**
   * @brief Bundle adjustment constructor
   * @param[in] options The user Ceres options
   * @see BundleAdjustmentPanoramaCeres::CeresOptions
   */
  BundleAdjustmentPanoramaCeres(const BundleAdjustmentPanoramaCeres::CeresOptions& options = CeresOptions())
    : _ceresOptions(options)
  {
    
  }

  /**
   * @brief Perform a Bundle Adjustment on the SfM scene with refinement of the requested parameters
   * @param[in,out] sfmData The input SfMData contains all the information about the reconstruction
   * @param[in] refineOptions The chosen refine flag
   * @return false if the bundle adjustment failed else true
   * @see BundleAdjustment::Adjust
   */
  bool adjust(sfmData::SfMData& sfmData, ERefineOptions refineOptions = REFINE_ALL);


  /**
   * @brief Get bundle adjustment statistics structure
   * @return statistics structure const ptr
   */
  inline const Statistics& getStatistics() const
  {
    return _statistics;
  }

  /**
   * @brief Return true if the bundle adjustment use an external local graph
   * @return true if use an external local graph
   */
  inline bool useLocalStrategy() const
  {
    return (_localGraph != nullptr);
  }

private:

  /**
   * @brief Clear structures for a new problem
   */
  inline void resetProblem()
  {
    _statistics = Statistics();
    _posesBlocks.clear();
    _intrinsicsBlocks.clear();
  }

  /**
   * @brief Set user Ceres options to the solver
   * @param[in,out] solverOptions The solver options structure
   */
  void setSolverOptions(ceres::Solver::Options& solverOptions) const;

  /**
   * @brief Create a parameter block for each extrinsics according to the Ceres format: [Rx, Ry, Rz, tx, ty, tz]
   * @param[in] sfmData The input SfMData contains all the information about the reconstruction, notably the poses and sub-poses
   * @param[in] refineOptions The chosen refine flag
   * @param[out] problem The Ceres bundle adjustement problem
   */
  void addExtrinsicsToProblem(const sfmData::SfMData& sfmData, ERefineOptions refineOptions, ceres::Problem& problem);

  /**
   * @brief Create a parameter block for each intrinsic according to the Ceres format
   * @param[in] sfmData The input SfMData contains all the information about the reconstruction, notably the intrinsics
   * @param[in] refineOptions The chosen refine flag
   * @param[out] problem The Ceres bundle adjustement problem
   */
  void addIntrinsicsToProblem(const sfmData::SfMData& sfmData, ERefineOptions refineOptions, ceres::Problem& problem);

  /**
   * @brief Create a residual block for each 2D constraints
   * @param[in] sfmData The input SfMData contains all the information about the reconstruction, notably the intrinsics
   * @param[in] refineOptions The chosen refine flag
   * @param[out] problem The Ceres bundle adjustement problem
   */
  void addConstraints2DToProblem(const sfmData::SfMData& sfmData, ERefineOptions refineOptions, ceres::Problem& problem);

  /**
   * @brief Create a residual block for each rotation priors
   * @param[in] sfmData The input SfMData contains all the information about the reconstruction, notably the intrinsics
   * @param[in] refineOptions The chosen refine flag
   * @param[out] problem The Ceres bundle adjustement problem
   */
  void addRotationPriorsToProblem(const sfmData::SfMData& sfmData, ERefineOptions refineOptions, ceres::Problem& problem);

  /**
   * @brief Create the Ceres bundle adjustement problem with:
   *  - extrincics and intrinsics parameters blocks.
   *  - residuals blocks for each observation.
   * @param[in,out] sfmData The input SfMData contains all the information about the reconstruction
   * @param[in] refineOptions The chosen refine flag
   * @param[out] problem The Ceres bundle adjustement problem
   */
  void createProblem(const sfmData::SfMData& sfmData, ERefineOptions refineOptions, ceres::Problem& problem);

  /**
   * @brief Update The given SfMData with the solver solution
   * @param[in,out] sfmData The input SfMData contains all the information about the reconstruction, notably the poses and sub-poses
   * @param[in] refineOptions The chosen refine flag
   */
  void updateFromSolution(sfmData::SfMData& sfmData, ERefineOptions refineOptions) const;

  /**
   * @brief Return the BundleAdjustment::EParameterState for a specific pose.
   * @param[in] poseId The pose id
   * @return BundleAdjustment::EParameterState (always REFINED if no local strategy)
   */
  inline BundleAdjustment::EParameterState getPoseState(IndexT poseId) const
  {
    return (_localGraph != nullptr ? _localGraph->getPoseState(poseId) : BundleAdjustment::EParameterState::REFINED);
  }

  /**
   * @brief Return the BundleAdjustment::EParameterState for a specific intrinsic.
   * @param[in] intrinsicId The intrinsic id
   * @return BundleAdjustment::EParameterState (always REFINED if no local strategy)
   */
  inline BundleAdjustment::EParameterState getIntrinsicState(IndexT intrinsicId) const
  {
    return (_localGraph != nullptr ? _localGraph->getIntrinsicState(intrinsicId) : BundleAdjustment::EParameterState::REFINED);
  }

  // private members

  /// use or not the local budle adjustment strategy
  std::shared_ptr<const LocalBundleAdjustmentGraph> _localGraph = nullptr;

  /// user Ceres options to use in the solver
  CeresOptions _ceresOptions;

  /// last adjustment iteration statisics
  Statistics _statistics;

  // data wrappers for refinement
  /// poses blocks wrapper
  /// block: ceres angleAxis(3) + translation(3)
  HashMap<IndexT, SO3::Matrix> _posesBlocks; //TODO : maybe we can use boost::flat_map instead of HashMap ?

  /// intrinsics blocks wrapper
  /// block: intrinsics params
  HashMap<IndexT, std::vector<double>> _intrinsicsBlocks;

};

} // namespace sfm
} // namespace aliceVision
