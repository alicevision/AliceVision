// This file is part of the AliceVision project.
// Copyright (c) 2018 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "aliceVision/feature/RegionsPerView.hpp"
#include "aliceVision/matching/IndMatch.hpp"
#include "aliceVision/matchingImageCollection/GeometricFilterMatrix.hpp"
#include "aliceVision/matchingImageCollection/geometricFilterUtils.hpp"
#include "aliceVision/sfmData/SfMData.hpp"

#include <boost/filesystem.hpp>

#include <cmath>


namespace aliceVision {
namespace matchingImageCollection {




struct GrowParameters
{
    GrowParameters() = default;

    GrowParameters(double similarityTolerance,
                   double affinityTolerance,
                   double homographyTolerance,
                   std::size_t minInliersToRefine,
                   std::size_t nbRefiningIterations,
                   double maxFractionPlanarMatches)
            : _similarityTolerance(similarityTolerance),
              _affinityTolerance(affinityTolerance),
              _homographyTolerance(homographyTolerance),
              _minInliersToRefine(minInliersToRefine),
              _nbRefiningIterations(nbRefiningIterations),
              _maxFractionPlanarMatches(maxFractionPlanarMatches)
    {
      assert(_maxFractionPlanarMatches >= 0 && _maxFractionPlanarMatches <= 1);
    }

    /// The maximal reprojection (pixel) error for matches estimated
    /// from a Similarity transformation.
    double _similarityTolerance{20};

    /// The maximal reprojection (pixel) error for matches estimated
    /// from an Affine transformation.
    double _affinityTolerance{10};

    /// The maximal reprojection (pixel) error for matches estimated
    /// from a Homography.
    double _homographyTolerance{5};

    /// Minimum number of inliers to continue in further refining.
    std::size_t _minInliersToRefine{6};

    /// Number of refine iterations that should be done to a transformation.
    /// 1st iteration is estimated using key coordinates, scale and orientation.
    /// 2-4th iterations are affinities
    /// 5+th iterations are homographies
    std::size_t _nbRefiningIterations{8};

    /// Value in [0,1]. If there is this fraction of inliers found the
    /// refine phase is terminated.
    double _maxFractionPlanarMatches{0.7};
};

/**
 * @brief Return all the matches in the same plane as the match \c seedMatchId with the corresponding homography.
 * @details This algorithm is detailed in [F.Srajer, 2016] algo. 1, p. 20.
 * @param[in] featuresI The features of the first view.
 * @param[in] featuresJ The features of the second view.
 * @param[in] matches All the putative planar matches.
 * @param[in] seedMatchId The match used to estimate the plane and the corresponding matches.
 * @param[out] planarMatchesIndices The indices (in the \c matches vector) of the really planar matches.
 * @param[out] transformation The homography associated to the plane.
 * @param[in] param The parameters of the algorihm.
 * @return true if the \c transformation is different than the identity matrix.
 */
bool growHomography(const std::vector<feature::PointFeature> &featuresI,
                    const std::vector<feature::PointFeature> &featuresJ,
                    const matching::IndMatches &matches,
                    const IndexT &seedMatchId,
                    std::set<IndexT> &planarMatchesIndices,
                    Mat3 &transformation,
                    const GrowParameters& param);

struct HGrowingFilteringParam
{
    HGrowingFilteringParam() = default;
    HGrowingFilteringParam(std::size_t maxNbHomographies, std::size_t minNbMatchesPerH, const GrowParameters& growParam)
            : _maxNbHomographies(maxNbHomographies),
              _minNbMatchesPerH(minNbMatchesPerH),
              _growParam(growParam) {} ;

    /// Max. number of homographies to estimate.
    std::size_t _maxNbHomographies{10};

    /// Min. number of matches corresponding to a homography.
    /// The homography-growing is interrupted when the value is reached.
    std::size_t _minNbMatchesPerH{20};

    /// Parameters for the growing algorithm
    GrowParameters _growParam{};
};

/**
 * @brief Filter the matches between two images using a growing homography approach.
 * @param[in] featuresI The features of the first view.
 * @param[in] featuresJ The features of the second view.
 * @param[in] putativeMatches The putative matches.
 * @param[out] homographiesAndMatches Contains each found homography and the relevant supporting matches.
 * @param[out] outGeometricInliers All the matches that supports one of the found homographies.
 * @param[in] param The parameters of the algorithm.
 */
void filterMatchesByHGrowing(const std::vector<feature::PointFeature>& siofeatures_I,
                             const std::vector<feature::PointFeature>& siofeatures_J,
                             const matching::IndMatches& putativeMatches,
                             std::vector<std::pair<Mat3,
                             matching::IndMatches>>& homographiesAndMatches,
                             matching::IndMatches& outGeometricInliers,
                             const HGrowingFilteringParam& param);

/**
 * @brief Generate a svg file with the two images and the matches grouped in different color
 * according to their supporting homography.
 * @param[in] outFilename The filename of the svg file to generate.
 * @param[in] viewI The first view.
 * @param[in] viewJ The second view.
 * @param[in] featuresI The features of the first view.
 * @param[in] featuresJ The features of the second view.
 * @param[in] homographiesAndMatches Contains each found homography and the relevant supporting matches.
 * @param[in] putativeMatches The putative matches.
 */
void drawHomographyMatches(const sfmData::View &viewI,
                           const sfmData::View &viewJ,
                           const std::vector<feature::PointFeature> &siofeatures_I,
                           const std::vector<feature::PointFeature> &siofeatures_J,
                           const std::vector<std::pair<Mat3, matching::IndMatches>> &homographiesAndMatches,
                           const matching::IndMatches &putativeMatches,
                           const std::string &outFilename);

//-- Multiple homography matrices estimation template functor, based on homography growing, used for filter pair of putative correspondences
struct GeometricFilterMatrix_HGrowing : public GeometricFilterMatrix
{
    explicit GeometricFilterMatrix_HGrowing(
          double dPrecision = std::numeric_limits<double>::infinity(),
          size_t iteration = 1024)
              : GeometricFilterMatrix(dPrecision, std::numeric_limits<double>::infinity(), iteration)
  { }
  
  /**
   * @brief Given two sets of image points, it estimates the homography matrix
   * relating them using a robust method (like A Contrario Ransac).
   * @details It return matches grouped by estimated homographies.
   * Example:
   * - Putative matches id:         [0 1 2 3 4 5 6 7]
   * - Estimated homographies:      [0(h0) 1(h2) 2(h1) 3(nan) 4(h2) 5(h0) 6(h1) 7(nan)]
   * - Sorted matches               [0(h0) 5(h0) 2(h1) 6(h1) 1(h2) 4(h2)]
   * - out_geometricInliersPerType  [0 5 2 6 1 4]
   *
   * To draw & save matches groups into .svg images:
   *     enter an existing folder in the following variable ('outputSvgDir').
   *  File format: <nbHMatches>hmatches_<viewId_I>_<viewId_J>_<descType>.svg
   *  Little white dots = putative matches
   *  Colored dots = geometrically verified matches (1 color per estimated plane)
   *
   *
   * @tparam Regions_or_Features_ProviderT The Region provider.
   * @param sfmData The sfmData containing the info about the scene.
   * @param regionsPerView The region provider.
   * @param pairIndex The pair of view for which the geometric validation is computed.
   * @param putativeMatchesPerType The putative matches for the considerec views.
   * @param out_geometricInliersPerType The filtered matches validated by the growing homography approach.
   * @param[in] outputSvgDir An existing folder where to save the images.
   * @return The estimation status.
   */
  template<typename Regions_or_Features_ProviderT>
  EstimationStatus geometricEstimation(const sfmData::SfMData * sfmData,
                                       const Regions_or_Features_ProviderT &regionsPerView,
                                       const Pair &pairIndex,
                                       const matching::MatchesPerDescType &putativeMatchesPerType,
                                       std::mt19937 &randomNumberGenerator,
                                       matching::MatchesPerDescType &out_geometricInliersPerType,
                                       const std::string& outputSvgDir = "")
  {
    using namespace aliceVision::feature;
    using namespace aliceVision::matching;
    
    out_geometricInliersPerType.clear();

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
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_APRILTAG)
    if (std::find(descTypes.begin(), descTypes.end(),feature::EImageDescriberType::APRILTAG16H5) != descTypes.end())
    {
      ALICEVISION_LOG_ERROR("Geometric filtering by Homography Growing cannot handle AprilTag descriptors.");
      return EstimationStatus(false, false);
    }
#endif
    
    // Get back corresponding view index
    const IndexT viewId_I = pairIndex.first;
    const IndexT viewId_J = pairIndex.second;
    
    const sfmData::View & viewI = *(sfmData->getViews().at(viewId_I));
    const sfmData::View & viewJ = *(sfmData->getViews().at(viewId_J));


    for (const EImageDescriberType& descType : descTypes)
    {
      if(!putativeMatchesPerType.count(descType))
      {
        continue;
      } // we may have 0 feature for some descriptor types

      const Regions & regions_I = regionsPerView.getRegions(viewId_I, descType);
      const Regions & regions_J = regionsPerView.getRegions(viewId_J, descType);

      std::vector<std::pair<Mat3, matching::IndMatches>> homographiesAndMatches;
      matching::IndMatches outGeometricInliers;
      filterMatchesByHGrowing(regions_I.Features(),
                              regions_J.Features(),
                              putativeMatchesPerType.at(descType),
                              homographiesAndMatches,
                              outGeometricInliers,
                              _parameters);

      out_geometricInliersPerType.emplace(descType, outGeometricInliers);
      _HsAndMatchesPerDesc.emplace(descType, homographiesAndMatches);

      if (outputSvgDir.empty())
      {
        continue;
      }

      if (boost::filesystem::exists(outputSvgDir))
      {
        const std::size_t nbMatches = outGeometricInliers.size();
        const std::string name = std::to_string(nbMatches) + "hmatches_" + std::to_string(viewI.getViewId()) + "_" +
                                 std::to_string(viewJ.getViewId()) +
                                 "_" + EImageDescriberType_enumToString(descType) + ".svg";
        // @FIXME not worth it having boost::filesystem in a header
        const std::string outFilename = (boost::filesystem::path(outputSvgDir) / boost::filesystem::path(name)).string();
        drawHomographyMatches(viewI,
                              viewJ,
                              regions_I.Features(),
                              regions_J.Features(),
                              homographiesAndMatches,
                              putativeMatchesPerType.at(descType),
                              outFilename);
      }
      else
      {
        ALICEVISION_LOG_WARNING("Cannot save homography-growing matches into '" << outputSvgDir << "': folder does not exist.");
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
  bool Geometry_guided_matching(const sfmData::SfMData *sfmData,
                                const feature::RegionsPerView &regionsPerView,
                                const Pair imageIdsPair,
                                const double dDistanceRatio,
                                matching::MatchesPerDescType &matches) override
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
  std::size_t getNbHomographies(const feature::EImageDescriberType & descType) const;
  
  /**
   * @brief Get the number of matches associated to the given descriptor type & homography index.
   * @param[in] descType Descriptor type.
   * @param[in] homographyId The id. of the wished homography / plane.
   * @return The number of matches. Return 0 if the descriptor type or the homography 
   * index do not exist in the result.
   */
  std::size_t getNbVerifiedMatches(const feature::EImageDescriberType & descType, IndexT homographyId) const;
  
  /**
   * @brief Get the number of verified matches for every descriptor and associated homographies.
   * @return The nb of verified matches
   */
  std::size_t getNbAllVerifiedMatches() const;
  
  /**
   * @brief Get a copy of the matches for a given desc. type & homography index.
   * @param[in] descType The descriptor type.
   * @param[in] homographyId The id. of the wished homography / plane.
   * @param[in] matches Contains the matches.
   * @return true the number of matches is up to 0.
   */
  bool getMatches(const feature::EImageDescriberType & descType, const IndexT homographyId, matching::IndMatches & matches) const;
  
private:
  
  // -- Results container
  
  /// The estimated homographies and their associated planar matches, 
  /// for each descriptor type.
  std::map<feature::EImageDescriberType, std::vector<std::pair<Mat3, matching::IndMatches>>> 
  _HsAndMatchesPerDesc; 
  
  //-- Parameters
  HGrowingFilteringParam _parameters{};

}; // struct GeometricFilterMatrix_HGrowing


} // namespace matchingImageCollection
} // namespace aliceVision


