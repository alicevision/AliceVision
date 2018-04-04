// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

//#include "aliceVision/multiview/homographyKernelSolver.hpp"
//#include "aliceVision/robustEstimation/ACRansac.hpp"
//#include "aliceVision/robustEstimation/ACRansacKernelAdaptator.hpp"
//#include "aliceVision/robustEstimation/guidedMatching.hpp"

//#include "aliceVision/matching/IndMatch.hpp"
//#include "aliceVision/matching/IndMatchDecorator.hpp"
#include "aliceVision/sfm/SfMData.hpp"
#include "aliceVision/feature/RegionsPerView.hpp"
#include "aliceVision/matchingImageCollection/GeometricFilterMatrix.hpp"

namespace aliceVision {
namespace matchingImageCollection {

//-- Multiple homography matrices estimation template functor, based on homography growing, used for filter pair of putative correspondences
struct GeometricFilterMatrix_HGrowing : public GeometricFilterMatrix
{
  GeometricFilterMatrix_HGrowing(
    double dPrecision = std::numeric_limits<double>::infinity(),
    size_t iteration = 1024)
    : GeometricFilterMatrix(dPrecision, std::numeric_limits<double>::infinity(), iteration)
    , _maxNbHomographies(10)
    , _minRemainingMatches(20)
    , _similarityTolerance(10)
    , _affinityTolerance(10)
    , _homographyTolerance(5)
    , _minNbPlanarMatches(6)
    , _nbIterations(8)
    , _maxFractionPlanarMatches(0.7)
  {
    _Hs.push_back(Mat3::Identity());
  }
/**
   * @brief Given two sets of image points, it estimates the homography matrix
   * relating them using a robust method (like A Contrario Ransac).
   */
  template<typename Regions_or_Features_ProviderT>
  EstimationStatus geometricEstimation(
    const sfm::SfMData * sfmData,
    const Regions_or_Features_ProviderT& regionsPerView,
    const Pair pairIndex,
    const matching::MatchesPerDescType & putativeMatchesPerType,
    matching::MatchesPerDescType & out_geometricInliersPerType)
  {
    
    using namespace aliceVision;
    using namespace aliceVision::robustEstimation;
    out_geometricInliersPerType.clear();
    
    // Get back corresponding view index
    const IndexT iIndex = pairIndex.first;
    const IndexT jIndex = pairIndex.second;
    
    const std::vector<feature::EImageDescriberType> descTypes = regionsPerView.getCommonDescTypes(pairIndex);
    if(descTypes.empty())
      return EstimationStatus(false, false);
    
    // Retrieve all 2D features as undistorted positions into flat arrays
    Mat xI, xJ;
    MatchesPairToMat(pairIndex, putativeMatchesPerType, sfmData, regionsPerView, descTypes, xI, xJ);
    std::cout << "Pair id. : " << pairIndex << std::endl;
    std::cout << "|- xI: " << xI.rows() << "x" << xI.cols() << std::endl;
    std::cout << "|- xJ: " << xJ.rows() << "x" << xJ.cols() << std::endl;
    std::size_t nbMatches = xI.cols();
    
    // (?) make a map
    std::vector<Mat3> homographies;
    std::vector<std::vector<IndexT>> planarMatchesPerH;
    
    for (IndexT iTransform = 0; iTransform < _maxNbHomographies; ++iTransform)
    {
      for (IndexT iMatch = 0; iMatch < nbMatches; ++iMatch)
      {
        // [TODO] Add 1st improvment
        
        // Growing a homography from one match ([F.Srajer, 2016] algo. 1, p. 20)  
        std::vector<IndexT> planarMatchesIds;
        Mat3 homographie;
        
        growHomography(xI, xJ, iMatch ,planarMatchesIds, homographie);
        
        if (!planarMatchesIds.empty())
        {
          homographies.push_back(homographie);
          planarMatchesPerH.push_back(planarMatchesIds);
        }
      }
    }
    
    
    // Check if resection has strong support
    const bool hasStrongSupport = true;
    return EstimationStatus(true, hasStrongSupport);
  }
  
  /**
   * @brief Geometry_guided_matching
   * @param sfm_data
   * @param regionsPerView
   * @param pairIndex
   * @param dDistanceRatio
   * @param matches
   * @return
   */
  bool Geometry_guided_matching
  (
    const sfm::SfMData * sfmData,
    const feature::RegionsPerView & regionsPerView,
    const Pair imageIdsPair,
    const double dDistanceRatio,
    matching::MatchesPerDescType & matches) override
  {
    
    /* ... */
    return matches.getNbAllMatches() != 0;
  }

private:

  // Growing a homography from one match ([F.Srajer, 2016] algo. 1, p. 20)  
  //-- See: YASM/relative_pose.h
  void growHomography(const Mat & featuresI, 
                      const Mat & featuresJ, 
                      const IndexT & matchId,
                      std::vector<IndexT> & planarMatchesIndices, 
                      Mat3 & homographie)
  {
  
    planarMatchesIndices.clear();
    homographie = Mat3::Identity();
    
    for (IndexT iRefineStep = 0; iRefineStep < _nbIterations; ++iRefineStep)
    {
      if (iRefineStep == 0)
      {
        computeSimilarityFromMatch();
      }
      else if (iRefineStep <= 4)
      {
        estimateAffinity();
      }
      else
      {
        estimateHomography();
      }
      
      findHomographyInliers();
      
    }
  }
  
  /**
   * @brief computeSimilarityFromMatch
   * see: alicevision::sfm::computeSimilarity() [sfm/utils/alignment.cpp]
   *      alicevision::geometry::ACRansac_FindRTS() [geometry/rigidTransformation3D(_test).hpp]
   */   
  void computeSimilarityFromMatch()
  {
    std::cout << "computeSimilarityFromMatch" << std::endl;
  }
  
  /**
   * @brief estimateAffinity
   * see: alicevision::Affine2DFromCorrespondencesLinear() [multiview/affineSolver(_test).hpp]
   */
  void estimateAffinity()
  {
    std::cout << "estimateAffinity" << std::endl;
  }
  
  /**
   * @brief estimateHomography
   * see: by DLT: alicevision::homography::kernel::FourPointSolver::Solve() [multiview/homographyKernelSolver.hpp]
   *      by RANSAC: alicevision::matchingImageCOllection::geometricEstimation() [matchingImageCollection/GeometricFilterMatrix_H_AC.hpp]
   */
  void estimateHomography()
  {
    std::cout << "estimateHomography" << std::endl;
  }
  
  /**
   * @brief findHomographyInliers Test the reprojection error
   */
  void findHomographyInliers()
  {
    std::cout << "findHomographyInliers" << std::endl;
  }
  
  //-- Stored data
  std::vector<Mat3> _Hs;
  
  //-- Options
  
  std::size_t _maxNbHomographies; // = MaxHoms
  std::size_t _minRemainingMatches; // = MinInsNum
  
  // growHomography function:
  std::size_t _similarityTolerance; // = SimTol
  std::size_t _affinityTolerance;   // = AffTol
  std::size_t _homographyTolerance; // = HomTol
  
  std::size_t _minNbPlanarMatches; // = MinIns
  std::size_t _nbIterations; // = RefIterNum
  std::size_t _maxFractionPlanarMatches; // = StopInsFrac
  
  
};

} // namespace matchingImageCollection
} // namespace aliceVision


