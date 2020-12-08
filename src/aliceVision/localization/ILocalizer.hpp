// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/image/Image.hpp>
#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/camera/PinholeRadial.hpp>
#include <aliceVision/feature/ImageDescriber.hpp>
#include <aliceVision/robustEstimation/estimators.hpp>
#include <aliceVision/localization/LocalizationResult.hpp>

#include <random>

namespace aliceVision {
namespace localization {

struct LocalizerParameters
{
  LocalizerParameters()
    : _visualDebug("")
    , _refineIntrinsics(false)
    , _fDistRatio(0.8)
    , _errorMax(std::numeric_limits<double>::infinity())
    , _resectionEstimator(robustEstimation::ERobustEstimator::ACRANSAC)
    , _matchingEstimator(robustEstimation::ERobustEstimator::ACRANSAC)
    , _useLocalizeRigNaive(false)
    , _angularThreshold(degreeToRadian(0.1))
  {
    _featurePreset.setDescPreset(feature::EImageDescriberPreset::ULTRA);
  }

  virtual ~LocalizerParameters() = 0;

  /// enable visual debugging options
  std::string _visualDebug;  
  /// whether or not the Intrinsics of the query camera has to be refined
  bool _refineIntrinsics;
  /// the distance ratio to use when matching feature with the ratio test
  float _fDistRatio;
  /// the preset to use for feature extraction of the query image
  feature::ConfigurationPreset _featurePreset;
  /// maximum reprojection error allowed for resectioning
  double _errorMax;
  /// the type of *sac framework to use for resection
  robustEstimation::ERobustEstimator _resectionEstimator;
  /// the type of *sac framework to use for matching
  robustEstimation::ERobustEstimator _matchingEstimator;
  /// force the use of the rig localization without openGV
  bool _useLocalizeRigNaive;
  /// in rad, it is the maximum angular error for the opengv rig resection
  double _angularThreshold;                       
};

inline LocalizerParameters::~LocalizerParameters() {}

using OccurenceKey = IndMatch3D2D;
using OccurenceMap = std::map<OccurenceKey, std::size_t>;

class ILocalizer
{
public:
    ILocalizer() : _isInit(false) { };

    // Only relevant for CCTagLocalizer
    virtual void setCudaPipe(int) { }
    
    bool isInit() const {return _isInit;}
    
    const sfmData::SfMData& getSfMData() const {return _sfm_data; }
    
    /**
   * @brief Localize one image
   * 
   * @param[in] gen random seed.
   * @param[in] imageGrey The input greyscale image.
   * @param[in] param The parameters for the localization.
   * @param[in] useInputIntrinsics Uses the \p queryIntrinsics as known calibration.
   * @param[in,out] queryIntrinsics Intrinsic parameters of the camera, they are used if the
   * flag useInputIntrinsics is set to true, otherwise they are estimated from the correspondences.
   * @param[out] localizationResult The localization result containing the pose and the associations.
   * @param[in] imagePath Optional complete path to the image, used only for debugging purposes.
   * @return  true if the image has been successfully localized.
   */
  virtual bool localize(const image::Image<float> & imageGrey,
                        const LocalizerParameters *param,
                        std::mt19937 & gen,
                        bool useInputIntrinsics,
                        camera::PinholeRadialK3 &queryIntrinsics,
                        LocalizationResult & localizationResult,
                        const std::string& imagePath = std::string()) = 0;

  virtual bool localize(const feature::MapRegionsPerDesc &queryRegions,
                        const std::pair<std::size_t, std::size_t> &imageSize,
                        const LocalizerParameters *param,
                        std::mt19937 & gen,
                        bool useInputIntrinsics,
                        camera::PinholeRadialK3 &queryIntrinsics,
                        LocalizationResult & localizationResult,
                        const std::string& imagePath = std::string()) = 0;
    
  virtual bool localizeRig(const std::vector<image::Image<float>> & vec_imageGrey,
                           const LocalizerParameters *param,
                           std::mt19937 & gen,
                           std::vector<camera::PinholeRadialK3 > &vec_queryIntrinsics,
                           const std::vector<geometry::Pose3 > &vec_subPoses,
                           geometry::Pose3 &rigPose, 
                           std::vector<LocalizationResult>& vec_locResults)=0;
    
  virtual bool localizeRig(const std::vector<feature::MapRegionsPerDesc> & vec_queryRegions,
                           const std::vector<std::pair<std::size_t, std::size_t> > &imageSize,
                           const LocalizerParameters *param,
                           std::mt19937 & gen,
                           std::vector<camera::PinholeRadialK3 > &vec_queryIntrinsics,
                           const std::vector<geometry::Pose3 > &vec_subPoses,
                           geometry::Pose3 &rigPose,
                           std::vector<LocalizationResult>& vec_locResults)=0;
   
  virtual ~ILocalizer( ) {}

protected:
  bool _isInit;
  sfmData::SfMData _sfm_data;

};

} //namespace aliceVision 
} //namespace localization 

