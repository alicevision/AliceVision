// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/sfm/pipeline/ReconstructionEngine.hpp>
#include <aliceVision/sfm/pipeline/global/GlobalSfMRotationAveragingSolver.hpp>
#include <aliceVision/sfm/pipeline/global/GlobalSfMTranslationAveragingSolver.hpp>

#include <dependencies/htmlDoc/htmlDoc.hpp>

namespace aliceVision{
namespace sfm{

enum ERelativeRotationMethod
{
  RELATIVE_ROTATION_FROM_E,
  RELATIVE_ROTATION_FROM_H
};

inline std::string ERelativeRotationMethod_enumToString(const ERelativeRotationMethod rotationMethod)
{
  switch(rotationMethod)
  {
    case ERelativeRotationMethod::RELATIVE_ROTATION_FROM_E:      return "essential_matrix";
    case ERelativeRotationMethod::RELATIVE_ROTATION_FROM_H:   return "homography_matrix";
  }
  throw std::out_of_range("Invalid method name enum");
}

inline ERelativeRotationMethod ERelativeRotationMethod_stringToEnum(const std::string& rotationMethodName)
{
  std::string methodName = rotationMethodName;
  std::transform(methodName.begin(), methodName.end(), methodName.begin(), ::tolower);

  if(methodName == "essential_matrix")      return ERelativeRotationMethod::RELATIVE_ROTATION_FROM_E;
  if(methodName == "homography_matrix")   return ERelativeRotationMethod::RELATIVE_ROTATION_FROM_H;

  throw std::out_of_range("Invalid method name : '" + rotationMethodName + "'");
}

inline std::ostream& operator<<(std::ostream& os, ERelativeRotationMethod rotationMethodName)
{
  os << ERelativeRotationMethod_enumToString(rotationMethodName);
  return os;
}

inline std::istream& operator>>(std::istream& in, ERelativeRotationMethod& rotationMethod)
{
  std::string token;
  in >> token;
  rotationMethod = ERelativeRotationMethod_stringToEnum(token);
  return in;
}

/// Panorama Pipeline Reconstruction Engine.
/// - Method: Based on Global SfM but with no translations between cameras.
class ReconstructionEngine_panorama : public ReconstructionEngine
{
public:

  ReconstructionEngine_panorama(const sfmData::SfMData& sfmData,
                                 const std::string& outDirectory,
                                 const std::string& loggingFile = "");

  ~ReconstructionEngine_panorama();

  void SetFeaturesProvider(feature::FeaturesPerView* featuresPerView);
  void SetMatchesProvider(matching::PairwiseMatches* provider);

  void SetRotationAveragingMethod(ERotationAveragingMethod eRotationAveragingMethod);
  void SetRelativeRotationMethod(ERelativeRotationMethod eRelativeRotationMethod);

  void setLockAllIntrinsics(bool v) { _lockAllIntrinsics = v; }

  virtual bool process();

protected:
  /// Compute from relative rotations the global rotations of the camera poses
  bool Compute_Global_Rotations(const aliceVision::rotationAveraging::RelativeRotations& vec_relatives_R,
                                HashMap<IndexT, Mat3>& map_globalR);

  /// Compute the initial structure of the scene
  bool Compute_Initial_Structure(matching::PairwiseMatches& tripletWise_matches);

  /// Adjust the scene (& remove outliers)
  bool Adjust();

private:
  /// Compute relative rotations
  void Compute_Relative_Rotations(aliceVision::rotationAveraging::RelativeRotations& vec_relatives_R);

  // Logger
  std::shared_ptr<htmlDocument::htmlDocumentStream> _htmlDocStream;
  std::string _loggingFile;

  // Parameter
  ERotationAveragingMethod _eRotationAveragingMethod;
  ERelativeRotationMethod _eRelativeRotationMethod;
  bool _lockAllIntrinsics = false;

  // Data provider
  feature::FeaturesPerView* _featuresPerView;
  matching::PairwiseMatches* _pairwiseMatches;

  std::shared_ptr<feature::FeaturesPerView> _normalizedFeaturesPerView;
};

} // namespace sfm
} // namespace aliceVision
