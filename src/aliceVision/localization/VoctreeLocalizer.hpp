// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/config.hpp>
#include <aliceVision/feature/ImageDescriber.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfm/pipeline/localization/SfMLocalizer.hpp>
#include <aliceVision/stl/mapUtils.hpp>
#include <aliceVision/voctree/VocabularyTree.hpp>
#include <aliceVision/voctree/Database.hpp>
#include <aliceVision/matching/ArrayMatcher_kdtreeFlann.hpp>
#include <aliceVision/matching/RegionsMatcher.hpp>
#include <aliceVision/localization/reconstructed_regions.hpp>
#include <aliceVision/localization/LocalizationResult.hpp>
#include <aliceVision/localization/ILocalizer.hpp>
#include <aliceVision/localization/BoundedBuffer.hpp>

#include <flann/algorithms/dist.h>

namespace aliceVision {
namespace localization {

struct FrameData
{
  FrameData(const LocalizationResult &locResult, const feature::MapRegionsPerDesc& regionsPerDesc);
  
  LocalizationResult _locResult;
  ReconstructedRegionsMappingPerDesc _regionsWith3D;
  feature::MapRegionsPerDesc _regions;
};

class VoctreeLocalizer : public ILocalizer
{
public:
  enum Algorithm : int {FirstBest=0, BestResult=1, AllResults=2, Cluster=3};
  static Algorithm initFromString(const std::string &value);
  
public:
  struct Parameters : public LocalizerParameters
  {

    Parameters()
      : LocalizerParameters()
      , _useGuidedMatching(false)
      , _useRobustMatching(true)
      , _algorithm(Algorithm::AllResults)
      , _numResults(4)
      , _maxResults(10)
      , _numCommonViews(3)
      , _ccTagUseCuda(true)
      , _matchingError(std::numeric_limits<double>::infinity())
      , _nbFrameBufferMatching(10)
    {}
    
    /// Enable/disable guided matching when matching images
    bool _useGuidedMatching;
    /// Enable/disable robust feature matching (geometric validation)
    bool _useRobustMatching;
    /// algorithm to use for localization
    Algorithm _algorithm;
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
    std::size_t _nbFrameBufferMatching;
  };
  
public:
  
  /**
   * @brief Initialize a localizer based on a vocabulary tree
   * 
   * @param[in] sfmData The sfmdata containing the scene
   * reconstruction.
   * @param[in] descriptorsFolder The path to the directory containing the features 
   * of the scene (.desc and .feat files).
   * @param[in] vocTreeFilepath The path to the vocabulary tree (usually a .tree file).
   * @param[in] weightsFilepath Optional path to the weights of the vocabulary 
   * tree (usually a .weights file), if not provided the weights will be recomputed 
   * when all the documents are added.
   * @param[in] matchingDescTypes List of descriptor types to use for feature matching.
   * @param[in] voctreeDescType Descriptor type used for image matching with voctree.
   *
   * It enable the use of combined SIFT and CCTAG features.
   */
  VoctreeLocalizer(const sfmData::SfMData &sfmData,
                   const std::string &descriptorsFolder,
                   const std::string &vocTreeFilepath,
                   const std::string &weightsFilepath,
                   const std::vector<feature::EImageDescriberType>& matchingDescTypes
                  );
  
  void setCudaPipe( int i ) override
  {
      _cudaPipe = i;
  }
  
  /**
   * @brief Just a wrapper around the different localization algorithm, the algorithm
   * used to localized is chosen using \p param._algorithm. This version extract the
   * sift features from the query image.
   * @param[in]  randomNumberGenerator, The random seed
   * @param[in] imageGrey The input greyscale image.
   * @param[in] param The parameters for the localization.
   * @param[in] useInputIntrinsics Uses the \p queryIntrinsics as known calibration.
   * @param[in,out] queryIntrinsics Intrinsic parameters of the camera, they are used if the
   * flag useInputIntrinsics is set to true, otherwise they are estimated from the correspondences.
   * @param[out] localizationResult The localization result containing the pose and the associations.
   * @param[in] imagePath Optional complete path to the image, used only for debugging purposes.
   * @return  true if the image has been successfully localized.
   */
  bool localize(const image::Image<float> & imageGrey,
                const LocalizerParameters *param,
                std::mt19937 & randomNumberGenerator,
                bool useInputIntrinsics,
                camera::PinholeRadialK3 &queryIntrinsics,
                LocalizationResult &localizationResult, 
                const std::string& imagePath = std::string()) override;

  /**
   * @brief Just a wrapper around the different localization algorithm, the algorithm
   * used to localized is chosen using \p param._algorithm. This version takes as
   * input the sift feature already extracted.
   * @param[in]  randomNumberGenerator, The random seed
   * @param[in] queryRegions The input features of the query image
   * @param[in] imageSize The size of the input image
   * @param[in] param The parameters for the localization.
   * @param[in] useInputIntrinsics Uses the \p queryIntrinsics as known calibration.
   * @param[in,out] queryIntrinsics Intrinsic parameters of the camera, they are used if the
   * flag useInputIntrinsics is set to true, otherwise they are estimated from the correspondences.
   * @param[out] localizationResult The localization result containing the pose and the associations.
   * @param[in] imagePath Optional complete path to the image, used only for debugging purposes.
   * @return  true if the image has been successfully localized.
   */
  bool localize(const feature::MapRegionsPerDesc & queryRegions,
                const std::pair<std::size_t, std::size_t> &imageSize,
                const LocalizerParameters *param,
                std::mt19937 & randomNumberGenerator,
                bool useInputIntrinsics,
                camera::PinholeRadialK3 &queryIntrinsics,
                LocalizationResult & localizationResult,
                const std::string& imagePath = std::string()) override;
  
  
  bool localizeRig(const std::vector<image::Image<float>> & vec_imageGrey,
                   const LocalizerParameters *param,
                   std::mt19937 & randomNumberGenerator,
                   std::vector<camera::PinholeRadialK3 > &vec_queryIntrinsics,
                   const std::vector<geometry::Pose3 > &vec_subPoses,
                   geometry::Pose3 &rigPose,
                   std::vector<LocalizationResult> & vec_locResults) override;
  
  bool localizeRig(const std::vector<feature::MapRegionsPerDesc> & vec_queryRegions,
                   const std::vector<std::pair<std::size_t, std::size_t> > &vec_imageSize,
                   const LocalizerParameters *param,
                   std::mt19937 & randomNumberGenerator,
                   std::vector<camera::PinholeRadialK3 > &vec_queryIntrinsics,
                   const std::vector<geometry::Pose3 > &vec_subPoses,
                   geometry::Pose3 &rigPose,
                   std::vector<LocalizationResult>& vec_locResults) override;


#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_OPENGV)
  bool localizeRig_opengv(const std::vector<feature::MapRegionsPerDesc> & vec_queryRegions,
                          const std::vector<std::pair<std::size_t, std::size_t> > &imageSize,
                          const LocalizerParameters *parameters,
                          std::mt19937 & randomNumberGenerator,
                          std::vector<camera::PinholeRadialK3 > &vec_queryIntrinsics,
                          const std::vector<geometry::Pose3 > &vec_subPoses,
                          geometry::Pose3 &rigPose,
                          std::vector<LocalizationResult>& vec_locResults);
#endif

  bool localizeRig_naive(const std::vector<feature::MapRegionsPerDesc> & vec_queryRegions,
                        const std::vector<std::pair<std::size_t, std::size_t> > &imageSize,
                        const LocalizerParameters *parameters,
                        std::mt19937 & randomNumberGenerator,
                        std::vector<camera::PinholeRadialK3 > &vec_queryIntrinsics,
                        const std::vector<geometry::Pose3 > &vec_subPoses,
                        geometry::Pose3 &rigPose,
                        std::vector<LocalizationResult>& vec_locResults);


  /**
   * @brief Try to localize an image in the database: it queries the database to 
   * retrieve \p numResults matching images and it tries to localize the query image
   * wrt the retrieve images in order of their score taking the first best result.
   *
   
   * @param[in] queryRegions The input features of the query image
   * @param[in] imageSize The size of the input image
   * @param[in] param The parameters for the localization
   * @param[in] randomNumberGenerator The random seed
   * @param[in] useInputIntrinsics Uses the \p queryIntrinsics as known calibration
   * @param[in,out] queryIntrinsics Intrinsic parameters of the camera, they are used if the
   * flag useInputIntrinsics is set to true, otherwise they are estimated from the correspondences.
   * @param[out] pose The camera pose
   * @param[out] resection_data the 2D-3D correspondences used to compute the pose
   * @param[out] associationIDs the ids of the 2D-3D correspondences used to compute the pose
   * @return true if the localization is successful
   */
  bool localizeFirstBestResult(const feature::MapRegionsPerDesc &queryRegions,
                               const std::pair<std::size_t, std::size_t> &imageSize,
                               const Parameters &param, 
                               std::mt19937 & randomNumberGenerator,
                               bool useInputIntrinsics,
                               camera::PinholeRadialK3 &queryIntrinsics,
                               LocalizationResult &localizationResult,
                               const std::string &imagePath = std::string());

  /**
   * @brief Try to localize an image in the database: it queries the database to 
   * retrieve \p numResults matching images and it tries to localize the query image
   * wrt the retrieve images in order of their score, collecting all the 2d-3d correspondences
   * and performing the resection with all these correspondences
   *
   * @param[in] queryRegions The input features of the query image
   * @param[in] imageSize The size of the input image
   * @param[in] param The parameters for the localization
   * @param[in] randomNumberGenerator The random seed
   * @param[in] useInputIntrinsics Uses the \p queryIntrinsics as known calibration
   * @param[in,out] queryIntrinsics Intrinsic parameters of the camera, they are used if the
   * flag useInputIntrinsics is set to true, otherwise they are estimated from the correspondences.
   * @param[out] pose The camera pose
   * @param[out] resection_data the 2D-3D correspondences used to compute the pose
   * @param[out] associationIDs the ids of the 2D-3D correspondences used to compute the pose
   * @return true if the localization is successful
   */
  bool localizeAllResults(const feature::MapRegionsPerDesc & queryRegions,
                          const std::pair<std::size_t, std::size_t> & imageSize,
                          const Parameters &param,
                          std::mt19937 & randomNumberGenerator,
                          bool useInputIntrinsics,
                          camera::PinholeRadialK3 &queryIntrinsics,
                          LocalizationResult &localizationResult,
                          const std::string& imagePath = std::string());
  
  
  /**
   * @brief Retrieve matches to all images of the database.
   *
   * @param[in] queryRegions
   * @param[in] imageSize
   * @param[in] param
   * @param[in]  randomNumberGenerator,
   * @param[in] useInputIntrinsics
   * @param[in] queryIntrinsics
   * @param[out] out_occurences
   * @param[out] out_pt2D output matrix of 2D points
   * @param[out] out_pt3D output matrix of 3D points
   * @param[out] out_descTypes output vector of describerType
   * @param[out] out_matchedImages image matches output
   * @param[in] imagePath
   */
  void getAllAssociations(const feature::MapRegionsPerDesc & queryRegions,
                          const std::pair<std::size_t, std::size_t> &imageSize,
                          const Parameters &param,
                          std::mt19937 & randomNumberGenerator,
                          bool useInputIntrinsics,
                          const camera::PinholeRadialK3 &queryIntrinsics,
                          OccurenceMap & out_occurences,
                          Mat &out_pt2D,
                          Mat &out_pt3D,
                          std::vector<feature::EImageDescriberType>& out_descTypes,
                          std::vector<voctree::DocMatch>& out_matchedImages,
                          const std::string& imagePath = std::string()) const;

private:
  /**
   * @brief Load the vocabulary tree.

   * @param[in] vocTreeFilepath The path to the directory containing the features 
   * of the scene (.desc and .feat files).
   * @param[in] weightsFilepath weightsFilepath Optional path to the weights of the vocabulary 
   * tree (usually a .weights file), if not provided the weights will be recomputed 
   * when all the documents are added.
   * @param[in] feat_directory The path to the directory containing the features 
   * of the scene (.desc and .feat files).
   * @return true if everything went ok
   */
  bool initDatabase(const std::string & vocTreeFilepath,
                    const std::string & weightsFilepath,
                    const std::string & featFolder);

  /**
   * @brief robustMatching
   *
   * @param[?] matchers
   * @param[in] queryIntrinsics
   * @param[in] regionsToMatch
   * @param[in] matchedIntrinsics
   * @param[in] fDistRatio
   * @param[in] matchingError
   * @param[in] useGeometricFiltering
   * @param[in] useGuidedMatching
   * @param[in] imageSizeI
   * @param[in] imageSizeJ
   * @param[out] vec_featureMatches
   * @param[in] estimator
   * @return
   */
  bool robustMatching(matching::RegionsDatabaseMatcherPerDesc & matchers,
                      const camera::IntrinsicBase * queryIntrinsics,// the intrinsics of the image we are using as reference
                      const feature::MapRegionsPerDesc & regionsToMatch,
                      const camera::IntrinsicBase * matchedIntrinsics,
                      float fDistRatio,
                      double matchingError,
                      bool useGeometricFiltering,
                      bool useGuidedMatching,
                      const std::pair<size_t,size_t> & imageSizeI,     // size of the image in matcher  
                      const std::pair<size_t,size_t> & imageSizeJ,     // size of the query image
                      std::mt19937 & randomNumberGenerator,
                      matching::MatchesPerDescType & out_featureMatches,
                      robustEstimation::ERobustEstimator estimator = robustEstimation::ERobustEstimator::ACRANSAC) const;
  
  void getAssociationsFromBuffer(matching::RegionsDatabaseMatcherPerDesc& matchers,
                                 const std::pair<std::size_t, std::size_t> & imageSize,
                                 const Parameters &param,
                                 bool useInputIntrinsics,
                                 const camera::PinholeRadialK3 &queryIntrinsics,
                                 OccurenceMap &out_occurences,
                                 std::mt19937 & randomNumberGenerator,
                                 const std::string& imagePath = std::string()) const;
  
  /**
   * @brief Load all the Descriptors who have contributed to the reconstruction.
   * deprecated.. now inside initDatabase
   */
  bool loadReconstructionDescriptors(
    const sfmData::SfMData & sfm_data,
    const std::string & feat_directory);
  
  
public:
  
  /// for each view index, it contains the features and descriptors that have an
  /// associated 3D point
  feature::RegionsPerView _regionsPerView;
  ReconstructedRegionsMappingPerView _reconstructedRegionsMappingPerView;
  
  /// the feature extractor
  std::vector<std::unique_ptr<feature::ImageDescriber>> _imageDescribers;
  
  // CUDA CCTag supports several parallel pipelines, where each one can
  // processing different image dimensions.
  int _cudaPipe = 0;
  
  /// the vocabulary tree used to generate the database and the visual images for
  /// the query images
  std::unique_ptr<voctree::IVocabularyTree> _voctree;
  feature::EImageDescriberType _voctreeDescType = feature::EImageDescriberType::UNINITIALIZED;
  
  /// the database that stores the visual word representation of each image of
  /// the original dataset
  voctree::Database _database;
  
  /// Last frames buffer
  BoundedBuffer<FrameData> _frameBuffer;

  matching::EMatcherType _matcherType = matching::ANN_L2;
};

/**
 * @brief Print the name of the algorithm
 */
std::ostream& operator<<(std::ostream& os, VoctreeLocalizer::Algorithm a);

/**
 * @brief Get the type of algorithm from an integer
 */
std::istream& operator>>(std::istream &in, VoctreeLocalizer::Algorithm &a);


} // localization
} // aliceVision
