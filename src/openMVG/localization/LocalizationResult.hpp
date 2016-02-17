#pragma once

#include <openMVG/sfm/pipelines/localization/SfM_Localizer.hpp>

#include <vector>

namespace openMVG {
namespace localization {
  
class LocalizationResult {
public:
  
  LocalizationResult();
  
  LocalizationResult(
        const sfm::Image_Localizer_Match_Data & matchData,
        const std::vector<pair<IndexT, IndexT> > & indMatch3D2D,
        const geometry::Pose3 & pose,
        const cameras::Pinhole_Intrinsic_Radial_K3 & intrinsics,
        bool isValid = true);
  
  virtual ~LocalizationResult();
  
  // Accessors
  const std::vector<size_t> & getInliers() const;

  const Mat & getPt2D() const;

  const Mat & getPt3D() const;

  const std::vector<pair<IndexT, IndexT> > & getIndMatch3D2D() const;

  const geometry::Pose3 & getPose() const;
  
  void setPose(const geometry::Pose3 & pose);
  
  const cameras::Pinhole_Intrinsic_Radial_K3 & getIntrinsics() const;
  
  cameras::Pinhole_Intrinsic_Radial_K3 & getIntrinsics();
  
  void updateIntrinsics(const std::vector<double> & params);

  bool isValid() const;
  
  Mat2X computeResiduals() const ;
  
private:
  sfm::Image_Localizer_Match_Data _matchData;
                             // Hold all the imaged points, 
                             // their associated 3D points and the inlier indices 
                             // (w.r.t. the F/E robust estimation)

  std::vector<pair<IndexT, IndexT> > _indMatch3D2D; // 3D to 2D index matches in the global index system,
                                                    // i.e. the set of pair (landmark id, index of the associated 2D point)

  geometry::Pose3 _pose; // Computed camera pose
  
  cameras::Pinhole_Intrinsic_Radial_K3 _intrinsics;

  bool _isValid; // True if the localization succeeded, false otherwise
  
private:

};

} // localization
} // openMVG

