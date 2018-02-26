// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/sfm/pipeline/localization/SfMLocalizer.hpp>
#include <aliceVision/voctree/Database.hpp>

#include <vector>
#include <utility>
#include <string>

namespace aliceVision {
namespace localization {

struct IndMatch3D2D
{
  IndMatch3D2D() {}
  IndMatch3D2D(IndexT landmarkId,
      feature::EImageDescriberType descType,
      IndexT featId)
    : landmarkId(landmarkId)
    , descType(descType)
    , featId(featId)
  {}

  bool operator<(const IndMatch3D2D& other) const
  {
    return landmarkId < other.landmarkId;
  }

  IndexT landmarkId = UndefinedIndexT;
  feature::EImageDescriberType descType = feature::EImageDescriberType::UNINITIALIZED;
  IndexT featId = UndefinedIndexT;
};


class LocalizationResult
{
public:
  
  LocalizationResult();
  
  LocalizationResult(const sfm::ImageLocalizerMatchData& matchData,
                     const std::vector<IndMatch3D2D>& indMatch3D2D,
                     const geometry::Pose3& pose,
                     const camera::PinholeRadialK3& intrinsics,
                     const std::vector<voctree::DocMatch>& matchedImages,
                     bool isValid = true);
  
  virtual ~LocalizationResult();
  
  const std::vector<std::size_t>& getInliers() const
  {
    return _matchData.vec_inliers;
  }

  const Mat34& getProjection() const
  {
    return _matchData.projection_matrix;
  }

  const Mat& getPt2D() const
  {
    return _matchData.pt2D;
  }

  const Mat& getPt3D() const
  {
    return _matchData.pt3D;
  }

  const Mat retrieveUndistortedPt2D() const;
  
  const sfm::ImageLocalizerMatchData& getMatchData() const
  {
    return _matchData;
  }

  const std::vector<IndMatch3D2D>& getIndMatch3D2D() const
  {
    return _indMatch3D2D;
  }

  const std::vector<voctree::DocMatch>& getMatchedImages() const
  {
    return _matchedImages;
  }
  
  const geometry::Pose3& getPose() const
  {
    return _pose;
  }

  void setPose(const geometry::Pose3 & pose)
  {
    _pose = pose;
  }

  const camera::PinholeRadialK3& getIntrinsics() const
  {
    return _intrinsics;
  }

  camera::PinholeRadialK3& getIntrinsics()
  {
    return _intrinsics;
  }

  void updateIntrinsics(const std::vector<double>& params)
  {
    _intrinsics.updateFromParams(params);
  }

  bool isValid() const
  {
    return _isValid;
  }
  
  /**
   * @brief Compute the residual for each 2D-3D association.
   * @return A 2xN matrix containing the x-y residual for each point.
   */
  Mat2X computeAllResiduals() const;
  
  /**
   * @brief Compute the residual for the inlier 2D-3D association.
   * @return A 2xNumInliers containing the x-y residual for each inlier point.
   */
  Mat2X computeInliersResiduals() const ;
  
  /**
   * @brief Compute the reprojection error for the inliers.
   * @return A 1xNumInliers vector containing the reprojection error for each inlier point.
   */
  Vec computeReprojectionErrorPerInlier() const;
  
   /**
   * @brief Compute the reprojection error for the all the points.
   * @return A 1xNumInliers vector containing the reprojection error for each point.
   */
  Vec computeReprojectionErrorPerPoint() const;
  
  /**
   * @brief Compute the RMSE for the inlier association.
   * @return The RMSE for the inlier associations.
   */
  double computeInliersRMSE() const ;

  /**
   * @brief Compute the RMSE for the all the associations.
   * @return The RMSE for the inlier associations.
   */
  double computeAllRMSE() const ;
  
  /**
   * @brief Select the best inliers according to the given reprojection error threshold.
   * @param[in] threshold The threshold for the reprojection error in pixels.
   * @return The number of inliers detected with the new threshold.
   */
  std::size_t selectBestInliers(double maxReprojectionError);
  
  /**
   * @brief Select the best inliers according to the reprojection error threshold 
   * used/computed during the resection.
   * @return The number of inliers detected.
   */
  std::size_t selectBestInliers();
  
  double getMaxReprojectionError() const { return _matchData.error_max;}

  static void save(const std::vector<LocalizationResult>& localizationResults, const std::string& filename);
  static void load(std::vector<LocalizationResult>& localizationResults, const std::string& filename);
  
private:
  
  /// Hold all the imaged points, their associated 3D points and the inlier indices 
  /// (w.r.t. the pose robust estimation)
  sfm::ImageLocalizerMatchData _matchData;

  /// 3D to 2D index matches in the global index system,
  /// i.e. the set of pair (landmark id, index of the associated 2D point).
  /// It must have the same size of _matchData.pt3D (and pt2D)
  std::vector<IndMatch3D2D> _indMatch3D2D;
                                                    
  /// Computed camera pose
  geometry::Pose3 _pose; 
  
  /// The camera intrinsics associated 
  camera::PinholeRadialK3 _intrinsics;

  /// the database images that have been used for matching
  std::vector<voctree::DocMatch> _matchedImages;
  
  /// True if the localization succeeded, false otherwise
  bool _isValid; 
};

/**
 * @brief It recompute the pose of each camera in Localization results according
 * to the rigPose given as input. The camera in position 0 is supposed to be the 
 * main camera and it is set to the pose of the rig.
 * @param[out] vec_localizationResults
 * @param[in] rigPose
 * @param[in] vec_subPoses (N-1) vector
 */
void updateRigPoses(std::vector<LocalizationResult>& vec_localizationResults,
                    const geometry::Pose3 &rigPose,
                    const std::vector<geometry::Pose3 > &vec_subPoses);

} // localization
} // aliceVision

