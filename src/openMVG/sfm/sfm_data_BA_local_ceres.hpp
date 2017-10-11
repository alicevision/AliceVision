// Copyright (c) 2015 Pierre Moulon.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef OPENMVG_SFM_DATA_BA_LOCAL_CERES_HPP
#define OPENMVG_SFM_DATA_BA_LOCAL_CERES_HPP

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
  
  Local_Bundle_Adjustment_Ceres(Local_Bundle_Adjustment_Ceres::LocalBA_options options = LocalBA_options());

  /**
   * @see Bundle_Adjustment::AdjustPartialReconstruction
   * @brief Ajust parameters according to the reconstruction graph or refine everything
   * if graph is empty. 
   */
  bool Adjust(SfM_Data & sfm_data);
  
  bool Adjust(const SfM_Data & const_sfm_data);

  void computeStatesMaps_strategy0(const SfM_Data & sfm_data);

  void computeStatesMaps_strategy1(const SfM_Data & sfm_data, const std::shared_ptr<LocalBA_Data> localBA_data);

  void computeStatesMaps_strategy2(const SfM_Data & sfm_data, const std::shared_ptr<LocalBA_Data> localBA_data);

  void computeStatesMaps_strategy3(const SfM_Data & sfm_data, const std::shared_ptr<LocalBA_Data> localBA_data);

  void computeStatesMaps_strategy4(const SfM_Data & sfm_data, std::shared_ptr<LocalBA_Data> localBA_data);
  
  std::size_t getNumberOfCamerasInTheSolver();
  
  void initStatistics(const std::set<IndexT>& newViewsId) {_LBA_statistics = LocalBA_statistics(newViewsId);}

  /// \brief Export statistics about bundle adjustment in a TXT file ("BaStats.txt")
  /// The contents of the file have been writen such that it is easy to handle it with
  /// a Python script or any spreadsheets (e.g. by copy/past the full content to LibreOffice) 
  bool exportStatistics(const std::string& path);
  
private:

  // Used for Local BA approach: 
  LocalBA_options _LBA_openMVG_options;
  LocalBA_statistics _LBA_statistics;
  
  // Define the state of the all parameter of the reconstruction (structure, poses, intrinsics) in the BA:
  enum ELocalBAState { 
    refined,  //< will be adjuted by the BA solver
    constant, //< will be set as constant in the sover
    ignored   //< will not be set into the BA solver
  };
  
  // Store the ELocalBAState of each parameter (structure, poses, intrinsics) :
  std::map<IndexT, ELocalBAState> _map_poseId_LBAState;
  std::map<IndexT, ELocalBAState> _map_intrinsicId_LBAState;
  std::map<IndexT, ELocalBAState> _map_landmarkId_LBAState;
  
  // Get back the 'ELocalBAState' for a specific parameter :
  ELocalBAState getPoseState(const IndexT poseId)            {return _map_poseId_LBAState.find(poseId)->second;}
  ELocalBAState getIntrinsicsState(const IndexT intrinsicId) {return _map_intrinsicId_LBAState.find(intrinsicId)->second;}
  ELocalBAState getLandmarkState(const IndexT landmarkId)    {return _map_landmarkId_LBAState.find(landmarkId)->second;}


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
  void updateCameraPoses(
    const Hash_Map<IndexT, std::vector<double>> & map_poses,
    Poses & poses);
    
  // Update camera intrinsics with refined data
  void updateCameraIntrinsics(
    const Hash_Map<IndexT, std::vector<double>> & map_intrinsics,
    Intrinsics & intrinsics);
};

} // namespace sfm
} // namespace openMVG

#endif // OPENMVG_SFM_DATA_BA_LOCAL_CERES_HPP
