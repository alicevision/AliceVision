
// Copyright (c) 2015 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#pragma once

#include "KDTree.h"
#include "reconstructed_regions.hpp"
#include "LocalizationResult.hpp"
#include "ILocalizer.hpp"

#include <openMVG/features/image_describer.hpp>
#include <openMVG/sfm/sfm_data.hpp>
#include <openMVG/sfm/pipelines/localization/SfM_Localizer.hpp>
#include <openMVG/matching/regions_matcher.hpp>

#define USE_SIFT_FLOAT 0

namespace openMVG {
namespace localization {

//@fixme find a better place or maje the class template?
typedef openMVG::features::Descriptor<float, 128> DescriptorFloat;
#if USE_SIFT_FLOAT
typedef Reconstructed_Regions<features::SIOPointFeature, float, 128> Reconstructed_RegionsT;
#else
typedef Reconstructed_Regions<features::SIOPointFeature, unsigned char, 128> Reconstructed_RegionsT;
#endif

class KDTreeLocalizer : public ILocalizer
{
public:
  struct Parameters : LocalizerParameters
  {

    Parameters() : LocalizerParameters(), 
      _useGuidedMatching(false),
      _useRobustMatching(true),
      //_algorithm(Algorithm::AllResults),
      _numResults(4),
      _maxResults(10),
      _numCommonViews(3),
      _ccTagUseCuda(true),
      _matchingError(std::numeric_limits<double>::infinity()),
      _bufferSize(10),
      _useFrameBufferMatching(false)
    { }
    
    /// Enable/disable guided matching when matching images
    bool _useGuidedMatching;
    /// Enable/disable robust feature matching (geometric validation)
    bool _useRobustMatching;
    /// number of best matching images to retrieve from the database
    std::size_t _numResults;
    /// for algorithm AllResults, it stops the image matching when this number of matched images is reached
    std::size_t _maxResults;
    /// number minimum common images in which a point must be seen to be used in cluster tracking
    std::size_t _numCommonViews;
    /// ccTag-CUDA cannot process frames at different resolutions ATM, so set to false if localizer is used on images of differing sizes
    bool _ccTagUseCuda;
    /// maximum reprojection error allowed for image matching with geometric validation
    double _matchingError;
    /// maximum capacity of the frame buffer
    std::size_t _bufferSize;
    /// enable matching with frame buffer
    bool _useFrameBufferMatching;
  };
  
public:
  
  /**
   * @brief Initialize a localizer based on a vocabulary tree
   * 
   * @param[in] sfmFilePath The path to the sfmdata file containing the scene 
   * reconstruction.
   * @param[in] descriptorsFolder The path to the directory containing the features 
   * of the scene (.desc and .feat files).
   * @param[in] vocTreeFilepath The path to the vocabulary tree (usually a .tree file).
   * @param[in] weightsFilepath Optional path to the weights of the vocabulary 
   * tree (usually a .weights file), if not provided the weights will be recomputed 
   * when all the documents are added.
   * @param[in] useSIFT_CCTAG Optional and enabled only if the CCTAG are available. 
   * It enable the use of combined SIFT and CCTAG features.
   */
  KDTreeLocalizer(const std::string &sfmFilePath,
                  const std::string &descriptorsFolder,
                  size_t leafSize
#ifdef HAVE_CCTAG
                   , bool useSIFT_CCTAG
#endif
                  );
  
  /**
   * @brief Just a wrapper around the different localization algorithm, the algorithm
   * used to localized is chosen using \p param._algorithm. This version extract the
   * sift features from the query image.
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
                const LocalizerParameters *param,
                bool useInputIntrinsics,
                cameras::Pinhole_Intrinsic_Radial_K3 &queryIntrinsics,
                LocalizationResult &localizationResult, 
                const std::string& imagePath = std::string());

  /**
   * @brief Just a wrapper around the different localization algorithm, the algorithm
   * used to localized is chosen using \p param._algorithm. This version takes as
   * input the sift feature already extracted.
   * 
   * @param[in] genQueryRegions The input features of the query image
   * @param[in] imageSize The size of the input image
   * @param[in] param The parameters for the localization.
   * @param[in] useInputIntrinsics Uses the \p queryIntrinsics as known calibration.
   * @param[in,out] queryIntrinsics Intrinsic parameters of the camera, they are used if the
   * flag useInputIntrinsics is set to true, otherwise they are estimated from the correspondences.
   * @param[out] localizationResult The localization result containing the pose and the associations.
   * @param[in] imagePath Optional complete path to the image, used only for debugging purposes.
   * @return  true if the image has been successfully localized.
   */
  bool localize(const std::unique_ptr<features::Regions> &genQueryRegions,
                const std::pair<std::size_t, std::size_t> &imageSize,
                const LocalizerParameters *param,
                bool useInputIntrinsics,
                cameras::Pinhole_Intrinsic_Radial_K3 &queryIntrinsics,
                LocalizationResult & localizationResult,
                const std::string& imagePath = std::string());
  
  
  bool localizeRig(const std::vector<image::Image<unsigned char> > & vec_imageGrey,
                   const LocalizerParameters *param,
                   std::vector<cameras::Pinhole_Intrinsic_Radial_K3 > &vec_queryIntrinsics,
                   const std::vector<geometry::Pose3 > &vec_subPoses,
                   geometry::Pose3 &rigPose,
                   std::vector<LocalizationResult> & vec_locResults);
  
  bool localizeRig(const std::vector<std::unique_ptr<features::Regions> > & vec_queryRegions,
                   const std::vector<std::pair<std::size_t, std::size_t> > &vec_imageSize,
                   const LocalizerParameters *param,
                   std::vector<cameras::Pinhole_Intrinsic_Radial_K3 > &vec_queryIntrinsics,
                   const std::vector<geometry::Pose3 > &vec_subPoses,
                   geometry::Pose3 &rigPose,
                   std::vector<LocalizationResult>& vec_locResults);


#ifdef HAVE_OPENGV
  bool localizeRig_opengv(const std::vector<std::unique_ptr<features::Regions> > & vec_queryRegions,
                          const std::vector<std::pair<std::size_t, std::size_t> > &imageSize,
                          const LocalizerParameters *parameters,
                          std::vector<cameras::Pinhole_Intrinsic_Radial_K3 > &vec_queryIntrinsics,
                          const std::vector<geometry::Pose3 > &vec_subPoses,
                          geometry::Pose3 &rigPose,
                          std::vector<LocalizationResult>& vec_locResults);
#endif

  bool localizeRig_naive(const std::vector<std::unique_ptr<features::Regions> > & vec_queryRegions,
                        const std::vector<std::pair<std::size_t, std::size_t> > &imageSize,
                        const LocalizerParameters *parameters,
                        std::vector<cameras::Pinhole_Intrinsic_Radial_K3 > &vec_queryIntrinsics,
                        const std::vector<geometry::Pose3 > &vec_subPoses,
                        geometry::Pose3 &rigPose,
                        std::vector<LocalizationResult>& vec_locResults);


  /**
   * @brief Try to localize an image in the database: it queries the database to 
   * retrieve \p numResults matching images and it tries to localize the query image
   * wrt the retrieve images in order of their score, collecting all the 2d-3d correspondences
   * and performing the resection with all these correspondences
   *
   * @param[in] siftQueryRegions The input features of the query image
   * @param[in] imageSize The size of the input image
   * @param[in] param The parameters for the localization
   * @param[in] useInputIntrinsics Uses the \p queryIntrinsics as known calibration
   * @param[in,out] queryIntrinsics Intrinsic parameters of the camera, they are used if the
   * flag useInputIntrinsics is set to true, otherwise they are estimated from the correspondences.
   * @param[out] pose The camera pose
   * @param[out] resection_data the 2D-3D correspondences used to compute the pose
   * @param[out] associationIDs the ids of the 2D-3D correspondences used to compute the pose
   * @return true if the localization is successful
   */
  bool localizeAllResults(const features::SIFT_Regions &siftQueryRegions,
                          const std::pair<std::size_t, std::size_t> imageSize,
                          const Parameters &param,
                          bool useInputIntrinsics,
                          cameras::Pinhole_Intrinsic_Radial_K3 &queryIntrinsics,
                          LocalizationResult &localizationResult,
                          const std::string& imagePath = std::string());
  
  
  /**
   * @brief Retrieve matches to all images of the database.
   *
   * @param[in] siftQueryRegions
   * @param[in] imageSize
   * @param[in] param
   * @param[in] useInputIntrinsics
   * @param[in] queryIntrinsics
   * @param[out] occurences
   * @param[out] pt2D output matrix of 2D points
   * @param[out] pt3D output matrix of 3D points
   * @param[out] matchedImages image matches output
   * @param[in] imagePath
   */
  void getAllAssociations(const features::SIFT_Regions &siftQueryRegions,
                          const std::pair<std::size_t, std::size_t> &imageSize,
                          const Parameters &param,
                          bool useInputIntrinsics,
                          const cameras::Pinhole_Intrinsic_Radial_K3 &queryIntrinsics,
                          std::map< std::pair<IndexT, IndexT>, std::size_t > &occurences,
                          Mat &pt2D,
                          Mat &pt3D,
                          std::vector<voctree::DocMatch>& matchedImages,
                          const std::string& imagePath = std::string()) const;

private:
    size_t _leafSize;
    std::vector<popsift::kdtree::KDTreePtr> _kdtrees;
    Hash_Map<IndexT, Reconstructed_RegionsT > _regions_per_view;

  /**
   * @brief Load descriptors and build KD-tree(s) with descriptor->image associations.
   * @return true if everything went ok
   */
  bool initDatabase(const std::string & feat_directory);

  bool robustMatching(matching::RegionsMatcherT<MatcherT> & matcher, 
                      const cameras::IntrinsicBase * queryIntrinsics,// the intrinsics of the image we are using as reference
                      const Reconstructed_RegionsT::RegionsT & regionsToMatch,
                      const cameras::IntrinsicBase * matchedIntrinsics,
                      const float fDistRatio,
                      const double matchingError,
                      const bool robustMatching,
                      const bool b_guided_matching,
                      const std::pair<size_t,size_t> & imageSizeI,     // size of the image in matcher  
                      const std::pair<size_t,size_t> & imageSizeJ,     // size of the query image
                      std::vector<matching::IndMatch> & vec_featureMatches,
                      robust::EROBUST_ESTIMATOR estimator = robust::ROBUST_ESTIMATOR_ACRANSAC) const;
  
public:
  // the feature extractor
  // @fixme do we want a generic image describer?
//  features::SIFT_float_describer _image_describer;
  features::Image_describer* _image_describer;
};

} // localization
} // openMVG
