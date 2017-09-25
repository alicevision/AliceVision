// Copyright (c) 2015 Pierre Moulon.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef OPENMVG_SFM_DATA_HPP
#define OPENMVG_SFM_DATA_HPP

#include "openMVG/types.hpp"
#include "openMVG/sfm/sfm_view.hpp"
#include "openMVG/sfm/sfm_view_metadata.hpp"
#include "openMVG/sfm/sfm_landmark.hpp"
#include "openMVG/geometry/pose3.hpp"
#include "openMVG/cameras/cameras.hpp"
#include <fstream>

namespace openMVG {
namespace sfm {

/// Define a collection of View
typedef Hash_Map<IndexT, std::shared_ptr<View> > Views;

/// Define a collection of Pose (indexed by View::id_pose)
typedef Hash_Map<IndexT, geometry::Pose3> Poses;

/// Define a collection of IntrinsicParameter (indexed by View::id_intrinsic)
typedef Hash_Map<IndexT, std::shared_ptr<cameras::IntrinsicBase> > Intrinsics;

/// Define a collection of landmarks are indexed by their TrackId
typedef Hash_Map<IndexT, Landmark> Landmarks;

/// Contain all the information about a Bundle Adjustment loop
struct LocalBA_stats
{
  LocalBA_stats(const std::set<IndexT>& newlyResectedViewsId = std::set<IndexT>()) {newViewsId = newlyResectedViewsId;}
  
  // Parameters returned by Ceres:
  double time = 0.0;                          // spent time to solve the BA (s)
  std::size_t numSuccessfullIterations = 0;   // number of successfull iterations
  std::size_t numUnsuccessfullIterations = 0; // number of unsuccessfull iterations
  
  std::size_t numResidualBlocks = 0;          // num. of resiudal block in the Ceres problem
  
  double RMSEinitial = 0.0; // sqrt(initial_cost / num_residuals)
  double RMSEfinal = 0.0;   // sqrt(final_cost / num_residuals)
  
  // Parameters specifically used by Local BA:
  std::size_t numRefinedPoses = 0;           // number of refined poses among all the estimated views          
  std::size_t numConstantPoses = 0;          // number of poses set constant in the BA solver
  std::size_t numIgnoredPoses = 0;           // number of not added poses to the BA solver
  std::size_t numRefinedIntrinsics = 0;      // num. of refined intrinsics
  std::size_t numConstantIntrinsics = 0;     // num. of intrinsics set constant in the BA solver
  std::size_t numIgnoredIntrinsics = 0;      // num. of not added intrinsicsto the BA solver
  std::size_t numRefinedLandmarks = 0;       // num. of refined landmarks
  std::size_t numConstantLandmarks = 0;      // num. of landmarks set constant in the BA solver
  std::size_t numIgnoredLandmarks = 0;       // num. of not added landmarks to the BA solver
  
  std::map<int, std::size_t> map_distance_numCameras; // distribution of the cameras for each graph distance
  
  std::set<IndexT> newViewsId;  // index of the new views added (newly resected)
};

struct LocalBA_timeProfiler
{
  double graphUpdating = 0.0;
  double distMapsComputing = 0.0;
  double statesMapsComputing = 0.0;
  double adjusting = 0.0;
  double allLocalBA = 0.0;
  
  bool exportTimes(const std::string& filename)
  {
    std::ofstream os;
    os.open(filename, std::ios::app);
    os.seekp(0, std::ios::end); //put the cursor at the end
    if (!os.is_open())
    {
      OPENMVG_LOG_DEBUG("Unable to open the Time profiling file '" << filename << "'.");
      return false;
    }
    
    if (os.tellp() == 0) // 'tellp' return the cursor's position
    {
      // If the file does't exist: add a header.
      std::vector<std::string> header;
      header.push_back("graphUpdating (s)");
      header.push_back("distMapsComputing (s)"); 
      header.push_back("statesMapsComputing (s)"); 
      header.push_back("adjusting (s)"); 
      header.push_back("allLocalBA (s)"); 
      
      for (std::string & head : header)
        os << head << "\t";
      os << "\n"; 
    }
    
    os << graphUpdating << "\t"
       << distMapsComputing << "\t"
       << statesMapsComputing << "\t"
       << adjusting << "\t"
       << allLocalBA << "\t";
    
    os << "\n";
    os.close();
    return true;
  }
  
  void showTimes()
  {
    std::cout << "\n----- Local BA durations ------" << std::endl;
    std::cout << "graph updating : " << graphUpdating << " ms" << std::endl;
    std::cout << "dist. Maps Computing : " << distMapsComputing << " ms" << std::endl;
    std::cout << "states Maps Computing : " << statesMapsComputing << " ms" << std::endl;
    std::cout << "adjusting : " << adjusting << " ms" << std::endl;
    std::cout << "** all Local BA: " << allLocalBA << " ms" << std::endl;
    std::cout << "-------------------------------\n" << std::endl;
  }
};


/// Generic SfM data container
/// Store structure and camera properties:
struct SfM_Data
{
  /// Considered views
  Views views;
  /// Considered poses (indexed by view.id_pose)
  Poses poses;
  /// Considered camera intrinsics (indexed by view.id_intrinsic)
  Intrinsics intrinsics;
  /// Structure (3D points with their 2D observations)
  Landmarks structure;
  /// Controls points (stored as Landmarks (id_feat has no meaning here))
  Landmarks control_points;
  
  /// Root Views path
  std::string s_root_path;
  std::string _featureFolder;
  std::string _matchingFolder;
  
  //--
  // Accessors
  //--
  const Views & GetViews() const {return views;}
  const Poses & GetPoses() const {return poses;}
  const Intrinsics & GetIntrinsics() const {return intrinsics;}
  
  /**
   * @brief Return a pointer to an intrinsic if available or nullptr otherwise.
   * @param intrinsicId
   */
  const cameras::IntrinsicBase * GetIntrinsicPtr(IndexT intrinsicId) const
  {
    if(intrinsics.count(intrinsicId))
      return intrinsics.at(intrinsicId).get();
    return nullptr;
  }
  
  /**
   * @brief Return a pointer to an intrinsic if available or nullptr otherwise.
   * @param intrinsicId
   */
  cameras::IntrinsicBase * GetIntrinsicPtr(IndexT intrinsicId)
  {
    if(intrinsics.count(intrinsicId))
      return intrinsics.at(intrinsicId).get();
    return nullptr;
  }
  
  std::shared_ptr<cameras::IntrinsicBase> GetIntrinsicSharedPtr(IndexT intrinsicId)
  {
    if(intrinsics.count(intrinsicId))
      return intrinsics.at(intrinsicId);
    return nullptr;
  }
  
  std::map<IndexT, std::size_t> GetIntrinsicsUsage() const
  {
    std::map<IndexT, std::size_t> map_intrinsicId_usageNum;
    
    for (const auto& itView : views)
    {
      const View * view = itView.second.get();
      
      if (IsPoseAndIntrinsicDefined(view))
      {
        auto itIntr = map_intrinsicId_usageNum.find(view->id_intrinsic);
        if (itIntr == map_intrinsicId_usageNum.end())
          map_intrinsicId_usageNum[view->id_intrinsic] = 1;
        else
          map_intrinsicId_usageNum[view->id_intrinsic]++;
      }
    }
    return map_intrinsicId_usageNum;
  }
  
  
  const Landmarks & GetLandmarks() const {return structure;}
  const Landmarks & GetControl_Points() const {return control_points;}
  
  std::set<IndexT> GetViewsKeys() const
  {
    std::set<IndexT> viewKeys;
    for(auto v: views)
      viewKeys.insert(v.first);
    return viewKeys;
  }
  
  /// Check if the View have defined intrinsic and pose
  bool IsPoseAndIntrinsicDefined(const View * view) const
  {
    if (view == nullptr) return false;
    return (
          view->id_intrinsic != UndefinedIndexT &&
        view->id_pose != UndefinedIndexT &&
        intrinsics.find(view->id_intrinsic) != intrinsics.end() &&
        poses.find(view->id_pose) != poses.end());
  }
  
  bool IsPoseAndIntrinsicDefined(IndexT viewID) const
  { 
    return IsPoseAndIntrinsicDefined(views.at(viewID).get());
  }
  
  /// Get the pose associated to a view
  const geometry::Pose3 GetPoseOrDie(const View * view) const
  {
    return poses.at(view->id_pose);
  }
  
  void setFeatureFolder(const std::string& featureFolder)
  {
    _featureFolder = featureFolder;
  }
  void setMatchingFolder(const std::string& matchingFolder)
  {
    _matchingFolder = matchingFolder;
  }
  const std::string& getFeatureFolder() const { return _featureFolder; }
  const std::string& getMatchingFolder() const { return _matchingFolder; }
  
  bool operator==(const SfM_Data& other) const {
    
    // Views
    if(views.size() != other.views.size())
      return false;
    for(Views::const_iterator it = views.begin(); it != views.end(); ++it)
    {
      const View& view1 = *(it->second.get());
      const View& view2 = *(other.views.at(it->first).get());
      if(!(view1 == view2))
        return false;
      
      // Image paths 
      if(s_root_path + view1.s_Img_path != other.s_root_path + view2.s_Img_path)
        return false;
    }
    
    // Poses
    if((poses != other.poses))
      return false;
    
    // Intrinsics
    if(intrinsics.size() != other.intrinsics.size())
      return false;
    
    Intrinsics::const_iterator it = intrinsics.begin();
    Intrinsics::const_iterator otherIt = other.intrinsics.begin();
    for(; it != intrinsics.end() && otherIt != other.intrinsics.end(); ++it, ++otherIt)
    {
      // Index
      if(it->first != otherIt->first)
        return false;
      
      // Intrinsic
      cameras::IntrinsicBase& intrinsic1 = *(it->second.get());
      cameras::IntrinsicBase& intrinsic2 = *(otherIt->second.get());
      if(!(intrinsic1 == intrinsic2))
        return false;
    }
    
    // Points IDs are not preserved
    if(structure.size() != other.structure.size())
      return false;
    Landmarks::const_iterator landMarkIt = structure.begin();
    Landmarks::const_iterator otherLandmarkIt = other.structure.begin();
    for(; landMarkIt != structure.end() && otherLandmarkIt != other.structure.end(); ++landMarkIt, ++otherLandmarkIt)
    {
      // Points IDs are not preserved
      // Landmark
      const Landmark& landmark1 = landMarkIt->second;
      const Landmark& landmark2 = otherLandmarkIt->second;
      if(!(landmark1 == landmark2))
        return false;
    }
    
    // Control points
    if(control_points != other.control_points)
      return false;
    
    // Root path can be reseted during exports
    
    return true;
    
  }
};

/**
 * @brief ColorizeTracks Add the associated color to each 3D point of
 * the sfm_data, using the track to determine the best view from which
 * to get the color.
 * @param sfm_data The container of the data
 * @return true if everything went well
 */
bool ColorizeTracks( SfM_Data & sfm_data );

} // namespace sfm
} // namespace openMVG

#endif // OPENMVG_SFM_DATA_HPP
