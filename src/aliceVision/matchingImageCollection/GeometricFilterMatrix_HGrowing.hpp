// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "aliceVision/feature/RegionsPerView.hpp"
#include "aliceVision/matching/IndMatch.hpp"
#include "aliceVision/matchingImageCollection/GeometricFilterMatrix.hpp"
#include "aliceVision/sfm/SfMData.hpp"
#include "dependencies/vectorGraphics/svgDrawer.hpp"

#include <Eigen/Geometry>
#include <ceres/ceres.h>
#include <boost/filesystem.hpp>

#include <cmath>


namespace aliceVision {
namespace matchingImageCollection {

/**
 * @brief This functor allows to optimize an Homography.
 * @details It is based on [F.Srajer, 2016] p.20, 21 and its C++ implementation: https://github.com/fsrajer/yasfm/blob/master/YASFM/relative_pose.cpp#L992
 * "The optimization takes into account points with error close to the threshold and does not care about high-error ones."
 */
class RefineHRobustCostFunctor
{
public:

  RefineHRobustCostFunctor(const Vec2& x1, const Vec2& x2,
    double softThresh)
    : x1(x1),x2(x2),_softThresh(softThresh)
  {
  }

  template<typename T>
  bool operator()(const T* const parameters, T* residuals) const
  {
    using Mat3T = Eigen::Matrix<T, 3, 3>;
    using Vec3T = Eigen::Matrix<T, 3, 1>;
    using Vec2T = Eigen::Matrix<T, 2, 1>;

    const Vec2T x(T(x1(0)), T(x1(1)));
    const Vec2T y(T(x2(0)), T(x2(1)));
    
    const Mat3T H(parameters);

    const Vec3T xp = H * x.homogeneous();

    const T errX = y(0) - xp(0)/xp(2);
    const T errY = y(1) - xp(1)/xp(2);
    const T errSq = errX*errX + errY*errY;
    
    // Avoid division by zero in derivatives computation
    const T err = (errSq==0.) ? T(errSq) : T(sqrt(errSq));
    residuals[0] = robustify(_softThresh, err);

    return true;
  }
  
  template<typename T>
  /**
   * @brief robustify
   * Based on: https://github.com/fsrajer/yasfm/blob/3a09bc0ee69b7021910d646386cd92deab504a2c/YASFM/utils.h#L347
   * @param softThresh
   * @param x
   * @return 
   */
  static T robustify(double softThresh, T x)
  {
    const double t = 0.25;
    const double sigma = softThresh / std::sqrt(-std::log(t * t));

    return -ceres::log(ceres::exp(-(x * x) / T(2.0 * sigma * sigma)) + T(t)) + T(std::log(1.0 + t));
  }

  Vec2 x1, x2;
  double _softThresh;
};

//-- Multiple homography matrices estimation template functor, based on homography growing, used for filter pair of putative correspondences
struct GeometricFilterMatrix_HGrowing : public GeometricFilterMatrix
{
  GeometricFilterMatrix_HGrowing(
    double dPrecision = std::numeric_limits<double>::infinity(),
    size_t iteration = 1024)
    : GeometricFilterMatrix(dPrecision, std::numeric_limits<double>::infinity(), iteration)
    , _maxNbHomographies(10)
    , _minNbMatchesPerH(20)
    , _similarityTolerance(20)
    , _affinityTolerance(10)
    , _homographyTolerance(5)
    , _minInliersToRefine(6)
    , _nbRefiningIterations(8)
    , _maxFractionPlanarMatches(0.7)
  {
    assert(_maxFractionPlanarMatches >= 0 && _maxFractionPlanarMatches <= 1);
  }
  
  /**
   * @brief Given two sets of image points, it estimates the homography matrix
   * relating them using a robust method (like A Contrario Ransac).
   * @details It return matches grouped by estimated homographies.
   * Example:
   * - Putative matches id:         [0 1 2 3 4 5 6 7]
   * - Estimated homographies:      [0(h0) 1(h2) 2(h1) 3(nan) 4(h2) 5(h0) 6(h1) 7(nan)]
   * - Sorted matches               [0(h0) 5(h0) 2(h1) 6(h1) 1(h2) 4(h2)]
   * - out_geometricInliersPerType  [0 5 2 6 1 4]
   */
  template<typename Regions_or_Features_ProviderT>
  EstimationStatus geometricEstimation(
    const sfm::SfMData * sfmData,
    const Regions_or_Features_ProviderT& regionsPerView,
    const Pair& pairIndex,
    const matching::MatchesPerDescType & putativeMatchesPerType,
    matching::MatchesPerDescType & out_geometricInliersPerType)
  {
    using namespace aliceVision::feature;
    using namespace aliceVision::matching;
    
    out_geometricInliersPerType.clear();
    
    // ------------------------
    // To draw & save matches groups into .svg images:
    //    enter an existing folder in the following variable ('outputSvgDir'). 
    // File format: <nbHMatches>hmatches_<viewId_I>_<viewId_J>_<descType>.svg
    // * Little white dots = putative matches
    // * Colored dots = geometrically verified matches (1 color per estimated plane)
    std::string outputSvgDir = ""; 
    // ------------------------
    
    const std::vector<feature::EImageDescriberType> descTypes = regionsPerView.getCommonDescTypes(pairIndex);
    if(descTypes.empty())
      return EstimationStatus(false, false);
    
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_CCTAG)
    if (std::find(descTypes.begin(), descTypes.end(),feature::EImageDescriberType::CCTAG3) != descTypes.end() ||
        std::find(descTypes.begin(), descTypes.end(),feature::EImageDescriberType::CCTAG4) != descTypes.end())
    {
      ALICEVISION_LOG_ERROR("Geometric filtering by Homography Growing cannot handle CCTAG descriptors.");
      return EstimationStatus(false, false);
    }
#endif
    
    // Get back corresponding view index
    const IndexT viewId_I = pairIndex.first;
    const IndexT viewId_J = pairIndex.second;
    
    const sfm::View & viewI = *(sfmData->GetViews().at(viewId_I));
    const sfm::View & viewJ = *(sfmData->GetViews().at(viewId_J));
    
    // Setup optional drawer tool:
    bool drawGroupedMatches = false;
    std::vector<std::string> colors {"red","cyan","purple","green","black","brown","blue","pink","grey"};
    
    std::unique_ptr<svg::svgDrawer> svgStream;
    if (!outputSvgDir.empty())
    {
      if (boost::filesystem::exists(outputSvgDir))
      {
        drawGroupedMatches = true;
        svgStream.reset(new svg::svgDrawer(viewI.getWidth() + viewJ.getWidth() , std::max(viewI.getHeight(), viewJ.getHeight())));
      }
      else
      {
        drawGroupedMatches = false;
        ALICEVISION_LOG_WARNING("Cannot save homography-growing matches into '" << outputSvgDir << "': folder does not exist.");
      }
    }
    
    for(size_t d = 0; d < descTypes.size(); ++d)
    {
      const EImageDescriberType& descType = descTypes[d];
      
      if(!putativeMatchesPerType.count(descType))
        continue; // we may have 0 feature for some descriptor types
      
      const Regions & regions_I = regionsPerView.getRegions(viewId_I, descType);
      const Regions & regions_J = regionsPerView.getRegions(viewId_J, descType);
      const std::vector<SIOPointFeature> siofeatures_I = getSIOPointFeatures(regions_I);
      const std::vector<SIOPointFeature> siofeatures_J = getSIOPointFeatures(regions_J);
      
      IndMatches remainingMatches = putativeMatchesPerType.at(descType);
      
      if(drawGroupedMatches)
      {
        svgStream->drawImage(viewI.getImagePath(), viewI.getWidth(), viewI.getHeight());
        svgStream->drawImage(viewJ.getImagePath(), viewJ.getWidth(), viewJ.getHeight(),  viewI.getWidth());
        // draw little white dots representing putative matches
        for(const IndMatch& match : remainingMatches)
        {
          const SIOPointFeature & fI = siofeatures_I.at(match._i);
          const SIOPointFeature & fJ  = siofeatures_J.at(match._j);
          svgStream->drawCircle(fI.x(), fI.y(), 1, svg::svgStyle().stroke("white",2.0));
          svgStream->drawCircle(fJ.x() + viewI.getWidth(), fJ.y(), 1, svg::svgStyle().stroke("white", 2.0));
        }
      }
      
      for(IndexT iH = 0; iH < _maxNbHomographies; ++iH)
      {
        std::set<IndexT> usedMatchesId, bestMatchesId;
        Mat3 bestHomography;
        
        // -- Estimate H using homography-growing approach
        #pragma omp parallel for // (huge optimization but modify results a little)
        for(int iMatch = 0; iMatch < remainingMatches.size(); ++iMatch)
        {
          // Growing a homography from one match ([F.Srajer, 2016] algo. 1, p. 20)  
          // each match is used once only per homography estimation (increases computation time) [1st improvement ([F.Srajer, 2016] p. 20) ] 
          if (usedMatchesId.find(iMatch) != usedMatchesId.end()) 
            continue;
          
          std::set<IndexT> planarMatchesId; // be careful: it contains the id. in the 'remainingMatches' vector not 'putativeMatches' vector.
          Mat3 homography;
          
          if(!growHomography(siofeatures_I, siofeatures_J, remainingMatches, iMatch, planarMatchesId, homography) == EXIT_SUCCESS)
            continue;
          
          #pragma omp critical
          usedMatchesId.insert(planarMatchesId.begin(), planarMatchesId.end());
          
          if (planarMatchesId.size() > bestMatchesId.size())
          {
            #pragma omp critical
            {
              bestMatchesId = planarMatchesId; // be careful: it contains the id. in the 'remainingMatches' vector not 'putativeMatches' vector.
              bestHomography = homography;
            }
          }
        } // 'iMatch'
        
        // -- Refine H using Ceres minimizer
        {
          ceres::Problem problem;
          
          for(IndexT matchId : bestMatchesId)
          {
            IndMatch match = remainingMatches.at(matchId);
            
            const Vec2 x1 = siofeatures_I.at(match._i).coords().cast<double>();
            const Vec2 x2 = siofeatures_J.at(match._j).coords().cast<double>();
            
            RefineHRobustCostFunctor* costFun = 
                new RefineHRobustCostFunctor(x1, x2, _homographyTolerance);
            
            problem.AddResidualBlock(
                new ceres::AutoDiffCostFunction<
                    RefineHRobustCostFunctor,
                    1,
                    9>(costFun),
                nullptr,
                bestHomography.data());
          }
          
          ceres::Solver::Options solverOpt;
          solverOpt.max_num_iterations = 10;
          
          ceres::Solver::Summary summary;
          ceres::Solve(solverOpt,&problem,&summary);
          
          if(std::fabs(bestHomography(2, 2)) > std::numeric_limits<double>::epsilon())
            bestHomography /= bestHomography(2,2);
          
          if (summary.IsSolutionUsable())
            findTransformationInliers(siofeatures_I, siofeatures_J, remainingMatches, bestHomography, _homographyTolerance, bestMatchesId);
            
        } // refinement part
        
        // stop when the models get to small        
        if (bestMatchesId.size() < _minNbMatchesPerH)
          break;

        // Store validated results:
        {
          IndMatches matches;
          Mat3 H = bestHomography;
          for (IndexT id : bestMatchesId) 
          {
            matches.push_back(remainingMatches.at(id));
          }
          _HsAndMatchesPerDesc[descType].emplace_back(H, matches);
        }
        
        if (drawGroupedMatches)
        {
          for (IndexT id : bestMatchesId)
          {
            const IndMatch & match = remainingMatches.at(id);
            const SIOPointFeature & fI = siofeatures_I.at(match._i);
            const SIOPointFeature & fJ  = siofeatures_J.at(match._j);
            std::string color = "grey"; // 0 < iH <= 8: colored; iH > 8  are white (not enougth colors)
            if (iH <= colors.size() - 1)
              color = colors.at(iH);
            
            svgStream->drawCircle(fI.x(), fI.y(), 5, svg::svgStyle().stroke(color, 5.0));
            svgStream->drawCircle(fJ.x() + viewI.getWidth(), fJ.y(), 5, svg::svgStyle().stroke(color, 5.0));
          }
        }
        
        // -- Update not used matches & Save geometrically verified matches
        for (IndexT id : bestMatchesId)
        {
          out_geometricInliersPerType[descType].push_back(remainingMatches.at(id));
        }
        
        // update remaining matches (/!\ Keep ordering)
        std::size_t cpt = 0;
        for (IndexT id : bestMatchesId) 
        {        
          remainingMatches.erase(remainingMatches.begin() + id - cpt);
          ++cpt;
        }
        
        // stop when the number of remaining matches is too small   
        if (remainingMatches.size() < _minNbMatchesPerH)
          break;
          
      } // 'iH'
      
      if (drawGroupedMatches)
      {
        const std::size_t nbMatches = putativeMatchesPerType.at(descType).size() - remainingMatches.size();
        if (nbMatches > 0)
        {
          std::ofstream svgFile(outputSvgDir + "/" + std::to_string(nbMatches) +"hmatches_" + std::to_string(viewI.getViewId()) + "_" + std::to_string(viewJ.getViewId()) + 
                                "_" + EImageDescriberType_enumToString(descType) + ".svg");
          svgFile << svgStream->closeSvgFile().str();
          svgFile.close();
        }
      }         
    } // 'descriptor'
    
    // Check if resection has strong support
    if (out_geometricInliersPerType.empty())
      return EstimationStatus(true, false);
    else
      return EstimationStatus(true, true);
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
  
  /**
   * @brief Get the number of descriptor types including verified matches by the
   * homography growing method.
   * @return The number of descriptor types
   */
  inline std::size_t getNbDescTypes() const {return _HsAndMatchesPerDesc.size();}
  
  /**
   * @brief The number of estimated homographies for the given descriptor.
   * @param[in] descType The wished descriptor type
   * @return The number of estimated homographies (= planes). Return 0 if there is
   * no homography or if the descriptor type does not exist.
   */
  std::size_t getNbHomographies(const feature::EImageDescriberType & descType) const 
  {
    if (_HsAndMatchesPerDesc.find(descType) == _HsAndMatchesPerDesc.end())
        return 0;
    return _HsAndMatchesPerDesc.at(descType).size();
  }
  
  /**
   * @brief Get the number of matches associated to the given descriptor type & homography index.
   * @param[in] descType Descriptor type.
   * @param[in] homographyId The id. of the wished homography / plane.
   * @return The number of matches. Return 0 if the descriptor type or the homography 
   * index do not exist in the result.
   */
  std::size_t getNbVerifiedMatches(const feature::EImageDescriberType & descType, const IndexT homographyId) const
  {
    if (_HsAndMatchesPerDesc.find(descType) == _HsAndMatchesPerDesc.end())
      return 0;
    if (homographyId > _HsAndMatchesPerDesc.at(descType).size() - 1)
      return 0;

    return _HsAndMatchesPerDesc.at(descType).at(homographyId).second.size();
  }
  
  /**
   * @brief Get the number of verified matches for every descriptor and associated homographies.
   * @return The nb of verified matches
   */
  std::size_t getNbAllVerifiedMatches() const
  {
    std::size_t counter = 0;
    for (const auto & HnMs : _HsAndMatchesPerDesc)
    {
      for (const std::pair<Mat3, matching::IndMatches> & HnM : HnMs.second)
      {
        counter += HnM.second.size();
      }
    }
    return counter;
  }
  
  /**
   * @brief Get a copy of the matches for a given desc. type & homography index.
   * @param[in] descType The descriptor type.
   * @param[in] homographyId The id. of the wished homography / plane.
   * @param[in] matches Contains the matches.
   * @return EXIT_SUCCESS the number of matches is up to 0.
   */
  int getMatches(const feature::EImageDescriberType & descType, const IndexT homographyId, matching::IndMatches & matches) const
  {
    matches.clear();
    
    if (_HsAndMatchesPerDesc.find(descType) == _HsAndMatchesPerDesc.end())
      return EXIT_FAILURE;
    if (homographyId > _HsAndMatchesPerDesc.at(descType).size() - 1)
      return EXIT_FAILURE;

    matches = _HsAndMatchesPerDesc.at(descType).at(homographyId).second;
    
    if (matches.size() <= 0)
      return EXIT_FAILURE;

    return EXIT_SUCCESS;
  }
  
private:
  
  /**
   * @brief Return all the matches in the same plane as the match \c seedMatchId with the corresponding homography.
   * @details This algorithm is detailed in [F.Srajer, 2016] algo. 1, p. 20.
   * @param[in] featuresI
   * @param[in] featuresJ
   * @param[in] matches All the putative planar matches.
   * @param[in] seedMatchId The match used to estimate the plane and the corresponding matches.
   * @param[out] planarMatchesIndices The indices (in the \c matches vector) of the really planar matches.
   * @param[out] transformation The homography associated to the plane.
   * @return EXIT_SUCCESS if the \c transformation is different than the identity matrix.
   */
  int growHomography(const std::vector<feature::SIOPointFeature> & featuresI, 
                      const std::vector<feature::SIOPointFeature> & featuresJ, 
                      const matching::IndMatches & matches,
                      const IndexT & seedMatchId,
                      std::set<IndexT> & planarMatchesIndices, 
                      Mat3 & transformation)
  {
    assert(seedMatchId <= matches.size());
   
    planarMatchesIndices.clear();
    transformation = Mat3::Identity();
    
    const matching::IndMatch & seedMatch = matches.at(seedMatchId);
    const feature::SIOPointFeature & seedFeatureI = featuresI.at(seedMatch._i);
    const feature::SIOPointFeature & seedFeatureJ = featuresJ.at(seedMatch._j);

    double currTolerance;

    for (IndexT iRefineStep = 0; iRefineStep < _nbRefiningIterations; ++iRefineStep)
    {
      if (iRefineStep == 0)
      {
        computeSimilarity(seedFeatureI, seedFeatureJ, transformation);
        currTolerance = _similarityTolerance;
      }
      else if (iRefineStep <= 4)
      {
        estimateAffinity(featuresI, featuresJ, matches, transformation, planarMatchesIndices);
        currTolerance = _affinityTolerance;
      }
      else
      {
        estimateHomography(featuresI, featuresJ, matches, transformation, planarMatchesIndices);
        currTolerance = _homographyTolerance;
      }
      
      findTransformationInliers(featuresI, featuresJ, matches, transformation, currTolerance, planarMatchesIndices);
      
      if (planarMatchesIndices.size() < _minInliersToRefine)
        return EXIT_FAILURE;
      
      // Note: the following statement is present in the MATLAB code but not implemented in YASM
//      if (planarMatchesIndices.size() >= _maxFractionPlanarMatches * matches.size())
//        break;
    }
    
    return (transformation != Mat3::Identity()) ? EXIT_SUCCESS : EXIT_FAILURE;
  }
  
  // -- Results container
  
  /// The estimated homographies and their associated planar matches, 
  /// for each descriptor type.
  std::map<feature::EImageDescriberType, std::vector<std::pair<Mat3, matching::IndMatches>>> 
  _HsAndMatchesPerDesc; 
  
  //-- Options
  
  /// Max. number of homographies to estimate.
  std::size_t _maxNbHomographies; 
  
  /// Min. number of matches corresponding to a homography. 
  /// The homography-growing is interrupted when the value is reached.
  std::size_t _minNbMatchesPerH;
  
  /// The maximal reprojection (pixel) error for matches estimated 
  /// from a Similarity transformation.
  double _similarityTolerance; 
  
  /// The maximal reprojection (pixel) error for matches estimated 
  /// from an Affine transformation.
  double _affinityTolerance;   
  
  /// The maximal reprojection (pixel) error for matches estimated 
  /// from a Homography.
  double _homographyTolerance; 
  
  /// Minimum number of inliers to continue in further refining.
  std::size_t _minInliersToRefine; 
  
  /// Number of refine iterations that should be done to a transformation.
  /// 1st iteration is estimated using key coordinates, scale and orientation.
  /// 2-4th iterations are affinities
  /// 5+th iterations are homographies
  std::size_t _nbRefiningIterations; 
  
  /// Value in [0,1]. If there is this fraction of inliers found the
  /// refine phase is terminated.
  std::size_t _maxFractionPlanarMatches; 
  
}; // struct GeometricFilterMatrix_HGrowing

} // namespace matchingImageCollection
} // namespace aliceVision


