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

#include "aliceVision/matching/IndMatch.hpp"
//#include "aliceVision/matching/IndMatchDecorator.hpp"
#include "aliceVision/sfm/SfMData.hpp"
#include "aliceVision/feature/RegionsPerView.hpp"
#include "aliceVision/matchingImageCollection/GeometricFilterMatrix.hpp"
#include "Eigen/Geometry"


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
    const IndexT viewId_I = pairIndex.first;
    const IndexT viewId_J = pairIndex.second;
    
    const std::vector<feature::EImageDescriberType> descTypes = regionsPerView.getCommonDescTypes(pairIndex);
    if(descTypes.empty())
      return EstimationStatus(false, false);
    
    // Retrieve all 2D features as undistorted positions into flat arrays
    Mat xI, xJ;
    MatchesPairToMat(pairIndex, putativeMatchesPerType, sfmData, regionsPerView, descTypes, xI, xJ);
    
    std::cout << "Pair id. : " << pairIndex << std::endl;
    std::cout << "|- putative: " << putativeMatchesPerType.at(feature::EImageDescriberType::SIFT).size() << std::endl;
    std::cout << "|- xI: " << xI.rows() << "x" << xI.cols() << std::endl;
    std::cout << "|- xJ: " << xJ.rows() << "x" << xJ.cols() << std::endl;
    
    const feature::Regions& regionsSIFT_I = regionsPerView.getRegions(viewId_I, descTypes.at(0));
    const feature::Regions& regionsSIFT_J = regionsPerView.getRegions(viewId_J, descTypes.at(0));
    const std::vector<feature::SIOPointFeature> allSIFTFeaturesI = getSIOPointFeatures(regionsSIFT_I);
    const std::vector<feature::SIOPointFeature> allSIFTfeaturesJ = getSIOPointFeatures(regionsSIFT_J);
    
    matching::IndMatches putativeSIFTMatches = putativeMatchesPerType.at(feature::EImageDescriberType::SIFT);
//    std::vector<feature::SIOPointFeature> putativeFeaturesI, putativeFeaturesJ;
//    putativeFeaturesI.reserve(putativeSIFTMatches.size());
//    putativeFeaturesJ.reserve(putativeSIFTMatches.size());
    
//    for (const matching::IndMatch & idMatch : putativeSIFTMatches)
//    {
//      putativeFeaturesI.push_back(allSIFTFeaturesI.at(idMatch._i));
//      putativeFeaturesJ.push_back(allSIFTfeaturesJ.at(idMatch._j));
//    }
    
    if (viewId_I == 200563944 && viewId_J == 1112206013) // MATLAB exemple
    {
      std::cout << "|- #matches: " << putativeSIFTMatches.size() << std::endl;
      std::cout << "|- allSIFTFeaturesI : " << allSIFTFeaturesI.size() << std::endl;
      std::cout << "|- allSIFTfeaturesJ : " << allSIFTfeaturesJ.size() << std::endl;
//      std::cout << "|- putativeFeaturesI : " << putativeFeaturesI.size() << std::endl;
//      std::cout << "|- putativeFeaturesJ : " << putativeFeaturesJ.size() << std::endl;
      //      std::cout << "-------" << std::endl;
      //      std::cout << "xI : " << std::endl;
      //      std::cout << xI << std::endl;    
      //      std::cout << "putativeFeaturesI : " << std::endl;
      //      std::cout << putativeFeaturesI << std::endl;
      
      std::size_t nbMatches = putativeSIFTMatches.size();
      
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
          
          growHomography(allSIFTFeaturesI, allSIFTfeaturesJ, putativeSIFTMatches, iMatch, planarMatchesIds, homographie);
          
          if (!planarMatchesIds.empty())
          {
            homographies.push_back(homographie);
            planarMatchesPerH.push_back(planarMatchesIds);
          }
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
  void growHomography(const std::vector<feature::SIOPointFeature> & featuresI, 
                      const std::vector<feature::SIOPointFeature> & featuresJ, 
                      const matching::IndMatches & matches,
                      const IndexT & seedMatchId,
                      std::vector<IndexT> & planarMatchesIndices, 
                      Mat3 & transformation)
  {
  
    assert(seedMatchId <= matches.size());
    planarMatchesIndices.clear();
    transformation = Mat3::Identity();
    
    const matching::IndMatch & seedMatch = matches.at(seedMatchId);
    const feature::SIOPointFeature & seedFeatureI = featuresI.at(seedMatch._i);
    const feature::SIOPointFeature & seedFeatureJ = featuresJ.at(seedMatch._j);

    std::size_t currTolerance;

    for (IndexT iRefineStep = 0; iRefineStep < _nbIterations; ++iRefineStep)
    {
      if (iRefineStep == 0)
      {
        computeSimilarityFromMatch(seedFeatureI, seedFeatureJ, transformation);
        std::cout << "featI: " << seedFeatureI << std::endl;
        std::cout << "featJ: " << seedFeatureJ << std::endl;
        std::cout << "S = " << transformation << std::endl;

        currTolerance = _similarityTolerance;
      }
      else if (iRefineStep <= 4)
      {
        estimateAffinity();
        currTolerance = _affinityTolerance;
      }
      else
      {
        estimateHomography();
        currTolerance = _homographyTolerance;
      }
      
      findTransformationInliers(featuresI, featuresJ, matches, transformation, currTolerance, planarMatchesIndices);
      
      std::cout << "#Inliers = " << planarMatchesIndices.size() << std::endl;
      std::cout << planarMatchesIndices << std::endl;
      getchar();
    }
  }
  /**
   * @brief findHomographyInliers Test the reprojection error
   */
  void findTransformationInliers(const std::vector<feature::SIOPointFeature> & featuresI, 
                                 const std::vector<feature::SIOPointFeature> & featuresJ, 
                                 const matching::IndMatches & matches,
                                 const Mat3 & transformation,
                                 const std::size_t tolerance,
                                 std::vector<IndexT> & planarMatchesIndices)
  {
    planarMatchesIndices.clear();

    for (IndexT iMatch = 0; iMatch < matches.size(); ++iMatch)
    {
      const feature::SIOPointFeature & featI = featuresI.at(matches.at(iMatch)._i);
      const feature::SIOPointFeature & featJ = featuresJ.at(matches.at(iMatch)._j);
      
      Vec2 ptI(featI.x(), featI.y());
      Vec2 ptJ(featJ.x(), featJ.y());
      
      Vec3 ptIp_hom = transformation * ptI.homogeneous();

      float dist = (ptJ - ptIp_hom.hnormalized()).squaredNorm();   
      
      if (dist < tolerance * tolerance)
        planarMatchesIndices.push_back(iMatch);
    }
  }
  
  /**
   * @brief computeSimilarityFromMatch
   * see: alicevision::sfm::computeSimilarity() [sfm/utils/alignment.cpp]
   *      alicevision::geometry::ACRansac_FindRTS() [geometry/rigidTransformation3D(_test).hpp]
   */   

  void computeSimilarityFromMatch(const feature::SIOPointFeature & feat1,
                                  const feature::SIOPointFeature & feat2,
                                  Mat3 & S)
  {
    computeSimilarityFromMatch(feat1.coords(), feat1.scale(), feat1.orientation(),
                               feat2.coords(), feat2.scale(), feat2.orientation(),
                               S);
  }

  /**
   * @brief computeSimilarityFromMatch
   * Source: https://github.com/fsrajer/yasfm/blob/master/YASFM/relative_pose.cpp#L1649
   * @param coord1
   * @param scale1
   * @param orientation1
   * @param coord2
   * @param scale2
   * @param orientation2
   * @param S
   */  
  void computeSimilarityFromMatch(const Vec2f & coord1, double scale1, double orientation1,
                                  const Vec2f & coord2, double scale2, double orientation2,
                                  Mat3 & S)
  {
    double c1 = cos(orientation1),
        s1 = sin(orientation1),
        c2 = cos(orientation2),
        s2 = sin(orientation2);
    
    Mat3 A1,A2;
    A1 << scale1*c1,scale1*(-s1),coord1(0),
        scale1*s1,scale1*c1,coord1(1),
        0,0,1;
    A2 << scale2*c2,scale2*(-s2),coord2(0),
        scale2*s2,scale2*c2,coord2(1),
        0,0,1;
    
    S = A2*A1.inverse();
  }
  
  /**
   * @brief estimateAffinity
   * see: alicevision::Affine2DFromCorrespondencesLinear() [multiview/affineSolver(_test).hpp]
   */
  void estimateAffinity()
  {
//    std::cout << "estimateAffinity" << std::endl;
  }
  
  /**
   * @brief estimateHomography
   * see: by DLT: alicevision::homography::kernel::FourPointSolver::Solve() [multiview/homographyKernelSolver.hpp]
   *      by RANSAC: alicevision::matchingImageCOllection::geometricEstimation() [matchingImageCollection/GeometricFilterMatrix_H_AC.hpp]
   */
  void estimateHomography()
  {
//    std::cout << "estimateHomography" << std::endl;
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


