// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "ILocalizer.hpp"
#include "LocalizationResult.hpp"
#include "VoctreeLocalizer.hpp"
#include <aliceVision/config.hpp>
#include <aliceVision/feature/feature.hpp>
#include <aliceVision/feature/ImageDescriber.hpp>
#include <aliceVision/feature/cctag/ImageDescriber_CCTAG.hpp>
#include <aliceVision/sfm/SfMData.hpp>
#include <aliceVision/sfm/pipeline/localization/SfMLocalizer.hpp>
#include <aliceVision/voctree/Database.hpp>

#include <iostream>
#include <bitset>

namespace aliceVision {
namespace localization {


class CCTagLocalizer : public ILocalizer
{
  
  public:
  struct Parameters : public LocalizerParameters
  {

    Parameters() : LocalizerParameters(), 
      _nNearestKeyFrames(4) { }
    
    /// number of best matching images to retrieve from the database
    std::size_t _nNearestKeyFrames;
  };
  
public:
  
  CCTagLocalizer(const std::string &sfmFilePath,
                 const std::string &descriptorsFolder);
   
  void setCudaPipe( int i ) override;

 /**
   * @brief Just a wrapper around the different localization algorithm, the algorith
   * used to localized is chosen using \p param._algorithm
   * 
   * @param[in] imageGrey The input greyscale image.
   * @param[in] param The parameters for the localization.
   * @param[in] useInputIntrinsics Uses the \p queryIntrinsics as known calibration.
   * @param[in,out] queryIntrinsics Intrinsic parameters of the camera, they are used if the
   * flag useInputIntrinsics is set to true, otherwise they are estimated from the correspondences.
   * @param[out] localizationResult The localization result containing the pose and the associations.
   * @param[in] imagePath Optional complete path to the image, used only for debugging purposes.
   * @return  true if the image has been successfully localized.
   */
  bool localize(const image::Image<unsigned char> & imageGrey,
                const LocalizerParameters *parameters,
                bool useInputIntrinsics,
                camera::PinholeRadialK3 &queryIntrinsics,
                LocalizationResult & localizationResult, const std::string& imagePath = std::string()) override;

  bool localize(const feature::MapRegionsPerDesc &queryRegions,
                const std::pair<std::size_t, std::size_t> &imageSize,
                const LocalizerParameters *parameters,
                bool useInputIntrinsics,
                camera::PinholeRadialK3 &queryIntrinsics,
                LocalizationResult & localizationResult,
                const std::string& imagePath = std::string()) override;

  /**
   * @brief Naive implementation of the localizer using the rig. Each image from
   * the rig is localized and then a bundle adjustment is run for optimizing the 
   * global pose.
   * 
   * @param[in] vec_imageGrey A vector containing all the images from the rig
   * @param[in] parameters The parameters for the localization.
   * @param[in,out] vec_queryIntrinsics Vector containing the intrinsic parameters of the cameras
   * @param[in] vec_subPoses A vector containing the N-1 subposes of each camera wrt the main camera
   * @param[out] rigPose The pose of the rig expressed as the pose of the main camera
   * @return true if the rig has been successfully localized.
   */
  bool localizeRig(const std::vector<image::Image<unsigned char> > & vec_imageGrey,
                   const LocalizerParameters *parameters,
                   std::vector<camera::PinholeRadialK3 > &vec_queryIntrinsics,
                   const std::vector<geometry::Pose3 > &vec_subPoses,
                   geometry::Pose3 &rigPose,
                   std::vector<LocalizationResult> & vec_locResults) override;
  

  bool localizeRig(const std::vector<feature::MapRegionsPerDesc> & vec_queryRegions,
                   const std::vector<std::pair<std::size_t, std::size_t> > &imageSize,
                   const LocalizerParameters *param,
                   std::vector<camera::PinholeRadialK3 > &vec_queryIntrinsics,
                   const std::vector<geometry::Pose3 > &vec_subPoses,
                   geometry::Pose3 &rigPose,
                   std::vector<LocalizationResult>& vec_locResults) override;
  
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_OPENGV)
  bool localizeRig_opengv(const std::vector<feature::MapRegionsPerDesc> & vec_queryRegions,
                          const std::vector<std::pair<std::size_t, std::size_t> > &imageSize,
                          const LocalizerParameters *parameters,
                          std::vector<camera::PinholeRadialK3 > &vec_queryIntrinsics,
                          const std::vector<geometry::Pose3 > &vec_subPoses,
                          geometry::Pose3 &rigPose,
                          std::vector<LocalizationResult>& vec_locResults);
#endif
  
  bool localizeRig_naive(const std::vector<feature::MapRegionsPerDesc> & vec_queryRegions,
                        const std::vector<std::pair<std::size_t, std::size_t> > &imageSize,
                        const LocalizerParameters *parameters,
                        std::vector<camera::PinholeRadialK3 > &vec_queryIntrinsics,
                        const std::vector<geometry::Pose3 > &vec_subPoses,
                        geometry::Pose3 &rigPose,
                        std::vector<LocalizationResult>& vec_locResults);

  /**
   * @brief Given the input Regions, it retrieves all the 2D-3D associations from
   * the nearest k-frames in the database. The associations are retrieved in terms
   * of region index and 3D point index along with the number of times (\p occurrences) that the 
   * pair has been found. \p pt2D and \p pt3D contains the coordinates of the corresponding
   * points of the associations, in the same order as in \p occurences.
   * 
   * @param[in] queryRegions The input query regions containing the extracted 
   * markers from the query image.
   * @param[in] imageSize The size of the query image.
   * @param[in] param The parameters to use.
   * @param[out] occurences A map containing for each pair <pt3D_id, pt2D_id> 
   * the number of times that the association has been seen
   * @param[out] pt2D The set of 2D points of the associations as they are given in \p queryRegions.
   * @param[out] pt3D The set of 3D points of the associations.
   * @param[in] The optional path to the query image file, used for debugging.
   */
  void getAllAssociations(const feature::CCTAG_Regions &queryRegions,
                          const std::pair<std::size_t, std::size_t> &imageSize,
                          const CCTagLocalizer::Parameters &param,
                          OccurenceMap & out_occurences,
                          Mat &out_pt2D,
                          Mat &out_pt3D,
                          std::vector<voctree::DocMatch>& out_matchedImages,
                          const std::string& imagePath = std::string()) const;
  
  virtual ~CCTagLocalizer();

private:
  
  bool loadReconstructionDescriptors(
    const sfm::SfMData & sfm_data,
    const std::string & feat_directory);
  
  // for each view index, it contains the cctag features and descriptors that have an
  // associated 3D point
  feature::RegionsPerView _regionsPerView;
  ReconstructedRegionsMappingPerView _reconstructedRegionsMappingPerView;

  // the feature extractor
  feature::ImageDescriber_CCTAG _imageDescriber;
  /// @warning: descType needs to be a CCTAG_Regions
  feature::EImageDescriberType _cctagDescType = feature::EImageDescriberType::CCTAG3;

  // CUDA CCTag supports several parallel pipelines, where each one can
  // processing different image dimensions.
  int _cudaPipe = 0;
  
  //
  //std::map<IndexT, Vec3> _cctagDatabase;
};

 /**
   * @brief Retrieve the k nearest views in a collection of views based on a query
   *        consisting in a set of CCTag regions.
   * 
   * @param[in] queryRegions Set of CCTag regions in the query.
   * @param[in] regionsPerView Collection of views containing a set of cctag regions.
   * @param[in] nNearestKeyFrames Number of nearest neighbours to return.
   * @param[out] out_kNearestFrames Set of computed indices associated to the k nearest views.
   * @param[in] similarityThreshold A threshold to retrieve only the kframes having 
  *  at least \p similarityThreshold similarity score.
   */
void kNearestKeyFrames(
          const feature::CCTAG_Regions & queryRegions,
          feature::EImageDescriberType descType,
          const feature::RegionsPerView & regionsPerView,
          std::size_t nNearestKeyFrames,
          std::vector<IndexT> & out_kNearestFrames,
          float similarityThreshold = 1.0f);

/**
 * @brief Given a set of CCTag descriptors seen in a view, it creates a descriptor for the view: the
 * view descriptor is a 128 bit array (ie the number of possible markers) whose 
 * bits are 0 or 1 whether the corresponding marker is seen or not. E.g. if the 
 * bit in position 8 is 1 it means that the marker with ID 8 has been seen by the view.
 * 
 * @param[in] vCCTagDescriptors The input descriptors associated to the view.
 * @return The view descriptor as a set of bit representing the visibility of
 * each possible marker for that view.
 */
std::bitset<128> constructCCTagViewDescriptor(
        const std::vector<feature::CCTAG_Regions::DescriptorT> & vCCTagDescriptors);

float viewSimilarity(
        const feature::CCTAG_Regions & regionsA,
        const feature::CCTAG_Regions & regionsB);

void viewMatching(
        const feature::CCTAG_Regions & regionsA,
        const feature::CCTAG_Regions & regionsB,
        std::vector<matching::IndMatch> & out_featureMatches);

} // namespace localization
} // aliceVision

