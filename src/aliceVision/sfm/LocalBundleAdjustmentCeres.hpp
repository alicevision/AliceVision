// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/sfm/BundleAdjustmentCeres.hpp>
#include <aliceVision/sfm/LocalBundleAdjustmentData.hpp>
#include <aliceVision/sfmData/SfMData.hpp>

namespace aliceVision {
namespace sfm {

class LocalBundleAdjustmentCeres : public BundleAdjustmentCeres
{
public:
  
  /// Contains all the OpenMVG's options to communicate to the Ceres Solver
  struct LocalBA_options : public BundleAdjustmentCeres::BA_options
  {
    LocalBA_options(const bool bVerbose = true, bool bmultithreaded = true) 
      : BundleAdjustmentCeres::BA_options(bVerbose, bmultithreaded)
    {
      _useParametersOrdering = false;
      _useLocalBA = false;
    }
    
    bool _useParametersOrdering; 
    void enableParametersOrdering() {_useParametersOrdering = true;}
    void disableParametersOrdering() {_useParametersOrdering = false;}
    bool isParameterOrderingEnabled() {return _useParametersOrdering;}
    
    bool _useLocalBA;
    void enableLocalBA() {_useLocalBA = true;}
    void disableLocalBA() {_useLocalBA = false;}
    bool isLocalBAEnabled() {return _useLocalBA;}
  };
  
  /// Contains all the informations relating to the last BA performed.
  struct LocalBA_statistics
  {
    LocalBA_statistics(
        const std::set<IndexT>& newlyResectedViewsId = std::set<IndexT>(),
        const std::map<int, std::size_t>& distancesHistogram = std::map<int, std::size_t>()) : 
      _newViewsId(newlyResectedViewsId), 
      _numCamerasPerDistance(distancesHistogram) {;}

    // Parameters returned by Ceres:
    double _time = 0.0;                          ///< The spent time to solve the BA (s)
    std::size_t _numSuccessfullIterations = 0;   ///< The number of successful iterations
    std::size_t _numUnsuccessfullIterations = 0; ///< The number of unsuccessful iterations
    
    std::size_t _numResidualBlocks = 0;          ///< The num. of resiudal blocks in the Ceres problem
    
    double _RMSEinitial = 0.0; ///< sqrt(initial_cost / num_residuals)
    double _RMSEfinal = 0.0;   ///< sqrt(final_cost / num_residuals)
    
    // Parameters specifically used by Local BA:
    std::size_t _numRefinedPoses = 0;           ///< The num. of refined poses    
    std::size_t _numConstantPoses = 0;          ///< The num. of poses set to Constant in the BA solver
    std::size_t _numIgnoredPoses = 0;           ///< The num. of poses not added to the BA solver
    std::size_t _numRefinedIntrinsics = 0;      ///< The num. of refined intrinsics
    std::size_t _numConstantIntrinsics = 0;     ///< The num. of intrinsics set Constant in the BA solver
    std::size_t _numIgnoredIntrinsics = 0;      ///< The num. of intrinsics not added to the BA solver
    std::size_t _numRefinedLandmarks = 0;       ///< The num. of refined landmarks
    std::size_t _numConstantLandmarks = 0;      ///< The num. of landmarks set constant in the BA solver
    std::size_t _numIgnoredLandmarks = 0;       ///< The num. of landmarks not added to the BA solver
    
    std::map<int, std::size_t> _numCamerasPerDistance; ///< The distribution of the cameras for each graph distance <distance, numOfCam>
    
    std::set<IndexT> _newViewsId;               ///< The index of the new views (newly resected)
    
    void show();
  };
  
private:
  // Used for Local BA approach: 
  LocalBA_options _LBAOptions;        ///< Contains all the OpenMVG's options to communicate to the Ceres Solver
  LocalBA_statistics _LBAStatistics;  ///< Contains all the informations relating to the last BA performed.
  
public : 
  
  LocalBundleAdjustmentCeres() {;}
  
  LocalBundleAdjustmentCeres(
      const LocalBundleAdjustmentData& localBA_data, 
      const LocalBundleAdjustmentCeres::LocalBA_options& options, 
      const std::set<IndexT> &newReconstructedViews);
  
  /// @brief Ajust parameters according to the reconstruction graph or refine everything
  /// if graph is empty. 
  /// @param[in,out] sfm_data contains all the information about the reconstruction
  /// @param[in] localBA_data contains all the information about the Local BA approach, notably
  /// the state of each parameter of the solver (refined, constant, ignored)
  /// @return \c false if the refinement failed else \c true
  bool Adjust(sfmData::SfMData& sfm_data, const LocalBundleAdjustmentData& localBA_data);
  
  /// @brief Export statistics about bundle adjustment in a TXT file  \a BaStats_<nameComplement>.txt
  /// The contents of the file have been writen such that it is easy to handle it with
  /// a Python script or any spreadsheets (e.g. by copy/past the full content to LibreOffice) 
  /// @param[in] dir The directory where you want to save the \a BaStats.txt file.
  /// @param[in] nameComplement Add this string at the end of the file's name 
  /// @return false it cannot open the file, true if it succeed
  bool exportStatistics(const std::string& dir, const std::string& filename = "");
  
private:
  
  /// @brief Set BA options to Ceres
  void setSolverOptions(ceres::Solver::Options& solver_options);
  
  /// @brief Create a parameter block for each pose according to the Ceres format: [Rx, Ry, Rz, tx, ty, tz]
  /// @param[in] poses The poses to add in the BA problem
  /// @param[in] problem The Ceres problem
  /// @return The map including all the poses blocks added to the Ceres problem
  std::map<IndexT, std::vector<double>> addPosesToCeresProblem(
      const sfmData::Poses & poses,
      ceres::Problem & problem);
  
  /// @brief Create a parameter block for each intrinsic according to the Ceres format
  /// @param[in] sfm_data All the informations about the recontructions, notably the intrinsics
  /// @param[in] problem The Ceres problem
  /// @return The map including all the intrinsics blocks added to the Ceres problem
  std::map<IndexT, std::vector<double>> addIntrinsicsToCeresProblem(
      const sfmData::SfMData & sfm_data,
      ceres::Problem & problem);
  
  /// @brief Run the Ceres solver
  /// @param problem The Ceres problem
  /// @param options The Ceres options
  /// @param options The Ceres summary, including some statistics about the refinement
  /// @return true if Ceres considers the solution as usable, else false
  bool solveBA(
      ceres::Problem & problem, 
      ceres::Solver::Options & options, 
      ceres::Solver::Summary & summary);
  
  /// @brief Update camera poses with refined data
  /// @param[in] map_poseblocks The refined blocks
  /// @param[in] localBA_data Contains the state of each pose
  /// @param[out] poses The reference to the poses to update
  void updateCameraPoses(
      const std::map<IndexT, std::vector<double>> & map_poseblocks, 
      const LocalBundleAdjustmentData & localBA_data,
      sfmData::Poses & poses);
  
  /// @brief Update camera intrinsics with refined data
  /// @param[in] map_intrinsicblocks The refined blocks
  /// @param[in] localBA_data Contains the state of each intrinsics
  /// @param[out] intrinsics The reference to the intrinsics to update  
  void updateCameraIntrinsics(
      const std::map<IndexT, std::vector<double>> & map_intrinsicblocks,
      const LocalBundleAdjustmentData & localBA_data,
      sfmData::Intrinsics & intrinsics);
};

} // namespace sfm
} // namespace aliceVision
