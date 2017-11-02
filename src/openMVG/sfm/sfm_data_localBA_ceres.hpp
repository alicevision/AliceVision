// Copyright (c) 2015 Pierre Moulon.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef OPENMVG_SFM_DATA_LOCALBA_CERES_HPP
#define OPENMVG_SFM_DATA_LOCALBA_CERES_HPP

#include "openMVG/sfm/sfm_data_BA_ceres.hpp"
#include "openMVG/sfm/sfm_data_localBA.hpp"
#include "openMVG/tracks/tracks.hpp"
#include "lemon/bfs.h"

namespace openMVG {
namespace sfm {

class Local_Bundle_Adjustment_Ceres : public Bundle_Adjustment_Ceres
{
public:
  
  struct LocalBA_options : public Bundle_Adjustment_Ceres::BA_options
  {
    LocalBA_options(const bool bVerbose = true, bool bmultithreaded = true) 
      : Bundle_Adjustment_Ceres::BA_options(bVerbose, bmultithreaded)
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
  
  /// Contain all the information about a Bundle Adjustment loop
  struct LocalBA_statistics
  {
    LocalBA_statistics(
        const std::set<IndexT>& newlyResectedViewsId = std::set<IndexT>(),
        const std::map<int, std::size_t>& distancesHistogram = std::map<int, std::size_t>()) 
    {
      _newViewsId = newlyResectedViewsId;
      _numCamerasPerDistance = distancesHistogram;
    }
    
    // Parameters returned by Ceres:
    double _time = 0.0;                          // spent time to solve the BA (s)
    std::size_t _numSuccessfullIterations = 0;   // number of successfull iterations
    std::size_t _numUnsuccessfullIterations = 0; // number of unsuccessfull iterations
    
    std::size_t _numResidualBlocks = 0;          // num. of resiudal block in the Ceres problem
    
    double _RMSEinitial = 0.0; // sqrt(initial_cost / num_residuals)
    double _RMSEfinal = 0.0;   // sqrt(final_cost / num_residuals)
    
    // Parameters specifically used by Local BA:
    std::size_t _numRefinedPoses = 0;           // number of refined poses among all the estimated views          
    std::size_t _numConstantPoses = 0;          // number of poses set constant in the BA solver
    std::size_t _numIgnoredPoses = 0;           // number of not added poses to the BA solver
    std::size_t _numRefinedIntrinsics = 0;      // num. of refined intrinsics
    std::size_t _numConstantIntrinsics = 0;     // num. of intrinsics set constant in the BA solver
    std::size_t _numIgnoredIntrinsics = 0;      // num. of not added intrinsicsto the BA solver
    std::size_t _numRefinedLandmarks = 0;       // num. of refined landmarks
    std::size_t _numConstantLandmarks = 0;      // num. of landmarks set constant in the BA solver
    std::size_t _numIgnoredLandmarks = 0;       // num. of not added landmarks to the BA solver
    
    std::map<int, std::size_t> _numCamerasPerDistance; // distribution of the cameras for each graph distance
    
    std::set<IndexT> _newViewsId;  // index of the new views added (newly resected)
  };
  
  
private:
  // Used for Local BA approach: 
  LocalBA_options _LBAOptions;
  LocalBA_statistics _LBAStatistics;
  
public : 
  Local_Bundle_Adjustment_Ceres(
      const Local_Bundle_Adjustment_Ceres::LocalBA_options& options, 
      const LocalBA_Data& localBA_data, 
      const std::set<IndexT> &newReconstructedViews);
  
  /**
   * @see Bundle_Adjustment::AdjustPartialReconstruction
   * @brief Ajust parameters according to the reconstruction graph or refine everything
   * if graph is empty. 
   */
  bool Adjust(SfM_Data & sfm_data, const LocalBA_Data& localBA_data);
  
  /// \brief Export statistics about bundle adjustment in a TXT file ("BaStats.txt")
  /// The contents of the file have been writen such that it is easy to handle it with
  /// a Python script or any spreadsheets (e.g. by copy/past the full content to LibreOffice) 
  bool exportStatistics(
      const std::string& path, 
      const std::set<IndexT>& newReconstructedViews,
      const std::size_t& kMinNbOfMatches = 50, 
      const std::size_t kLimitDistance = 1);
  
  void showStatistics();
  
private:
  
  void setSolverOptions(ceres::Solver::Options& solver_options);
  
  // Create a parameter block for each pose according to the Ceres format: [Rx, Ry, Rz, tx, ty, tz]
  Hash_Map<IndexT, std::vector<double>> addPosesToCeresProblem(
      const Poses & poses, 
      ceres::Problem & problem);
  
  // Create a parameter block for each intrinsic according to the Ceres format
  Hash_Map<IndexT, std::vector<double>> addIntrinsicsToCeresProblem(
      const SfM_Data & sfm_data, 
      ceres::Problem & problem);
  
  // Run the Ceres solver
  bool solveBA(
      ceres::Problem & problem, 
      ceres::Solver::Options &options, 
      ceres::Solver::Summary &summary);
  
  // Update camera poses with refined data
  void updateCameraPoses(const Hash_Map<IndexT, 
                         std::vector<double>> & map_poseblocks, 
                         const LocalBA_Data &localBA_data,
                         Poses & poses);
  
  // Update camera intrinsics with refined data
  void updateCameraIntrinsics(
      const Hash_Map<IndexT, std::vector<double>> & map_intrinsicblocks,
      const LocalBA_Data &localBA_data,
      Intrinsics & intrinsics);
};

} // namespace sfm
} // namespace openMVG

#endif // OPENMVG_SFM_DATA_LOCALBA_CERES_HPP
