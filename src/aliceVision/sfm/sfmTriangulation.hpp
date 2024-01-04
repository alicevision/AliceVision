// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/types.hpp>
#include <aliceVision/sfmData/SfMData.hpp>

namespace aliceVision {
namespace sfm {

/// Generic basis struct for triangulation of track data contained
///  in the SfMData scene structure.
struct StructureComputationBasis
{
    bool _bConsoleVerbose;

    StructureComputationBasis(bool verbose = false);

    virtual void triangulate(sfmData::SfMData& sfmData, std::mt19937& randomNumberGenerator) const = 0;
};

/// Triangulation of track data contained in the structure of a SfMData scene.
// Use a blind estimation:
// - Triangulate tracks using all observations
// - Inlier/Outlier classification is done by a cheirality test
struct StructureComputationBlind : public StructureComputationBasis
{
    StructureComputationBlind(bool verbose = false);

    virtual void triangulate(sfmData::SfMData& sfmData, std::mt19937& randomNumberGenerator) const;
};

/// Triangulation of track data contained in the structure of a SfMData scene.
// Use a robust estimation:
// - Triangulate tracks using a RANSAC scheme
// - Check cheirality and a pixel residual error (TODO: make it a parameter)
struct StructureComputationRobust : public StructureComputationBasis
{
    StructureComputationRobust(bool verbose = false);

    virtual void triangulate(sfmData::SfMData& sfmData, std::mt19937& randomNumberGenerator) const;

    /// Robust triangulation of track data contained in the structure
    /// All observations must have View with valid Intrinsic and Pose data
    /// Invalid landmark are removed.
    void robustTriangulation(sfmData::SfMData& sfmData, std::mt19937& randomNumberGenerator) const;

    /// Robustly try to estimate the best 3D point using a ransac Scheme
    /// Return true for a successful triangulation
    bool robustTriangulation(const sfmData::SfMData& sfmData,
                             const sfmData::Observations& observations,
                             std::mt19937& randomNumberGenerator,
                             Vec3& X,
                             const IndexT minRequiredInliers = 3,
                             const IndexT minSampleIndex = 3) const;

  private:
    /// Triangulate a given track from a selection of observations
    Vec3 trackSampleTriangulation(const sfmData::SfMData& sfmData, const sfmData::Observations& observations, const std::set<IndexT>& samples) const;
};

}  // namespace sfm
}  // namespace aliceVision
