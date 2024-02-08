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

namespace aliceVision {
namespace sfm {

/// Global SfM Pipeline Reconstruction Engine.
/// - Method: Global Fusion of Relative Motions.
class ReconstructionEngine_globalSfM : public ReconstructionEngine
{
  public:
    ReconstructionEngine_globalSfM(const sfmData::SfMData& sfmData, const std::string& outDirectory, const std::string& loggingFile = "");

    ~ReconstructionEngine_globalSfM();

    void setFeaturesProvider(feature::FeaturesPerView* featuresPerView);
    void setMatchesProvider(matching::PairwiseMatches* provider);

    void setRotationAveragingMethod(ERotationAveragingMethod eRotationAveragingMethod);
    void setTranslationAveragingMethod(ETranslationAveragingMethod eTranslationAveragingMethod);

    void setLockAllIntrinsics(bool v) { _lockAllIntrinsics = v; }

    virtual bool process();

  protected:
    /// Compute from relative rotations the global rotations of the camera poses
    bool computeGlobalRotations(const aliceVision::rotationAveraging::RelativeRotations& vecRelativesR, std::map<IndexT, Mat3>& mapGlobalR);

    /// Compute/refine relative translations and compute global translations
    bool computeGlobalTranslations(const std::map<IndexT, Mat3>& globalRotations, matching::PairwiseMatches& tripletWiseMatches);

    /// Compute the initial structure of the scene
    bool computeInitialStructure(matching::PairwiseMatches& tripletWiseMatches);

    /// Adjust the scene (& remove outliers)
    bool adjust();

  private:
    /// Compute relative rotations
    void computeRelativeRotations(aliceVision::rotationAveraging::RelativeRotations& vecRelativesR);

    // Logger
    std::shared_ptr<htmlDocument::htmlDocumentStream> _htmlDocStream;
    std::string _loggingFile;

    // Parameter
    ERotationAveragingMethod _eRotationAveragingMethod;
    ETranslationAveragingMethod _eTranslationAveragingMethod;
    bool _lockAllIntrinsics = false;
    EFeatureConstraint _featureConstraint = EFeatureConstraint::BASIC;

    // Data provider
    feature::FeaturesPerView* _featuresPerView;
    matching::PairwiseMatches* _pairwiseMatches;

    std::shared_ptr<feature::FeaturesPerView> _normalizedFeaturesPerView;
};

}  // namespace sfm
}  // namespace aliceVision
