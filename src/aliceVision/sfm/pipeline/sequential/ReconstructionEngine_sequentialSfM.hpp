// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/sfm/pipeline/ReconstructionEngine.hpp>
#include <aliceVision/sfm/LocalBundleAdjustmentGraph.hpp>
#include <aliceVision/sfm/pipeline/localization/SfMLocalizer.hpp>
#include <aliceVision/sfm/pipeline/pairwiseMatchesIO.hpp>
#include <aliceVision/sfm/pipeline/RigSequence.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/feature/FeaturesPerView.hpp>
#include <aliceVision/track/TracksBuilder.hpp>
#include <dependencies/htmlDoc/htmlDoc.hpp>
#include <aliceVision/utils/Histogram.hpp>

#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>

namespace fs = boost::filesystem;
namespace pt = boost::property_tree;

namespace aliceVision {
namespace sfm {

/// Image score contains <ImageId, NbPutativeCommonPoint, score, isIntrinsicsReconstructed>
typedef std::tuple<IndexT, std::size_t, std::size_t, bool> ViewConnectionScore;

/**
 * @brief Sequential SfM Pipeline Reconstruction Engine.
 */
class ReconstructionEngine_sequentialSfM : public ReconstructionEngine
{
public:
  struct Params
  {
    Pair userInitialImagePair = { UndefinedIndexT, UndefinedIndexT };
    int minInputTrackLength = 2;
    int minTrackLength = 2;
    int minPointsPerPose = 30;
    bool useLocalBundleAdjustment = false;
    int localBundelAdjustementGraphDistanceLimit = 1;

    RigParams rig;

    /// Has fixed Intrinsics
    bool lockAllIntrinsics = false;
    int minNbCamerasToRefinePrincipalPoint = 3;

    /// minimum number of obersvations to triangulate a 3d point.
    std::size_t minNbObservationsForTriangulation = 2;
    /// a 3D point must have at least 2 obervations not too much aligned.
    double minAngleForTriangulation = 3.0;
    double minAngleForLandmark = 2.0;
    double maxReprojectionError = 4.0;
    EFeatureConstraint featureConstraint = EFeatureConstraint::BASIC;
    float minAngleInitialPair = 5.0f;
    float maxAngleInitialPair = 40.0f;
    bool filterTrackForks = true;
    robustEstimation::ERobustEstimator localizerEstimator = robustEstimation::ERobustEstimator::ACRANSAC;
    double localizerEstimatorError = std::numeric_limits<double>::infinity();
    size_t localizerEstimatorMaxIterations = 4096;

    // Pyramid scoring

    const int pyramidBase = 2;
    const int pyramidDepth = 5;

    // Local Bundle Adjustment data

    /// The minimum number of shared matches to create an edge between two views (nodes)
    const std::size_t kMinNbOfMatches = 50;

    // Intermediate reconstructions
    /// extension of the intermediate reconstruction files
    std::string sfmStepFileExtension = ".ply";
    /// filter for the intermediate reconstruction files
    sfmDataIO::ESfMData sfmStepFilter = sfmDataIO::ESfMData(
      sfmDataIO::VIEWS |
      sfmDataIO::EXTRINSICS |
      sfmDataIO::INTRINSICS |
      sfmDataIO::STRUCTURE |
      sfmDataIO::OBSERVATIONS |
      sfmDataIO::CONTROL_POINTS);
  };

public:

  ReconstructionEngine_sequentialSfM(const sfmData::SfMData& sfmData,
                                     const Params& params,
                                     const std::string& outputFolder,
                                     const std::string& loggingFile = "");

  void setFeatures(feature::FeaturesPerView* featuresPerView)
  {
    _featuresPerView = featuresPerView;
  }

  void setMatches(matching::PairwiseMatches* pairwiseMatches)
  {
    _pairwiseMatches = pairwiseMatches;
  }

  /**
   * @brief Process the entire incremental reconstruction
   * @return true if done
   */
  virtual bool process();

  /**
   * @brief Initialize pyramid scoring
   */
  void initializePyramidScoring();

  /**
   * @brief Initialize tracks
   * @return number of traks
   */
  std::size_t fuseMatchesIntoTracks();

  /**
   * @brief Get all initial pair candidates
   * @return pair list
   */
  std::vector<Pair> getInitialImagePairsCandidates();

  /**
   * @brief Try all initial pair candidates in order to create an initial reconstruction
   * @param initialPairCandidate The list of all initial pair candidates
   */
  void createInitialReconstruction(const std::vector<Pair>& initialImagePairCandidates);

  /**
   * @brief If we have already reconstructed landmarks in a previous reconstruction,
   * we need to recognize the corresponding tracks and update the landmarkIds accordingly.
   */
  void remapLandmarkIdsToTrackIds();

  /**
   * @brief Loop of reconstruction updates
   * @return the duration of the incremental reconstruction
   */
  double incrementalReconstruction();

  /**
   * @brief Update the reconstruction with a new resection group of images
   * @param[in] resectionId The resection id
   * @param[in] bestViewIds The best remaining view ids
   * @param[in] prevReconstructedViews The previously reconstructed view ids
   * @param[in,out] viewIds The remaining view ids
   * @return new reconstructed view ids
   */
  std::set<IndexT> resection(IndexT resectionId,
                             const std::vector<IndexT>& bestViewIds,
                             const std::set<IndexT>& prevReconstructedViews,
                             std::set<IndexT>& viewIds);

  /**
   * @brief triangulate
   * @param[in] prevReconstructedViews The previously reconstructed view ids
   * @param[in] newReconstructedViews The newly reconstructed view ids
   */
  void triangulate(const std::set<IndexT>& prevReconstructedViews,
                   const std::set<IndexT>& newReconstructedViews);

  /**
   * @brief bundleAdjustment
   * @param[in,out] newReconstructedViews The newly reconstructed view ids
   * @param[in] isInitialPair If true use fixed intrinsics an no nbOutliersThreshold
   * @return true if the bundle adjustment solution is usable
   */
  bool bundleAdjustment(std::set<IndexT>& newReconstructedViews, bool isInitialPair = false);

  /**
   * @brief Export and print statistics of a complete reconstruction
   * @param[in] reconstructionTime The duration of the reconstruction
   */
  void exportStatistics(double reconstructionTime);


  /**
   * @brief calibrateRigs
   * @param[in,out] updatedViews add the updated view ids to the list
   * @return updatedViews view ids
   */
  void calibrateRigs(std::set<IndexT>& updatedViews);

  /**
   * @brief Return all the images containing matches with already reconstructed 3D points.
   * The images are sorted by a score based on the number of features id shared with
   * the reconstruction and the repartition of these points in the image.
   *
   * @param[out] out_connectedViews: output list of view IDs connected with the 3D reconstruction.
   * @param[in] remainingViewIds: input list of remaining view IDs in which we will search for connected views.
   * @return False if there is no view connected.
   */
  bool findConnectedViews(std::vector<ViewConnectionScore>& out_connectedViews,
                          const std::set<IndexT>& remainingViewIds) const;

  /**
   * @brief Estimate the best images on which we can compute the resectioning safely.
   * The images are sorted by a score based on the number of features id shared with
   * the reconstruction and the repartition of these points in the image.
   *
   * @param[out] out_selectedViewIds: output list of view IDs we can use for resectioning.
   * @param[in] remainingViewIds: input list of remaining view IDs in which we will search for the best ones for resectioning.
   * @return False if there is no possible resection.
   */
  bool findNextBestViews(std::vector<IndexT>& out_selectedViewIds,
                         const std::set<IndexT>& remainingViewIds) const;

private:

  struct ResectionData : ImageLocalizerMatchData
  {
    /// tracks index for resection
    std::set<std::size_t> tracksId;
    /// features index for resection
    std::vector<track::FeatureId> featuresId;
    /// pose estimated by the resection
    geometry::Pose3 pose;
    /// intrinsic estimated by resection
    std::shared_ptr<camera::IntrinsicBase> optionalIntrinsic = nullptr;
    /// the instrinsic already exists in the scene or not.
    bool isNewIntrinsic;
  };

  /**
   * @brief Compute the initial 3D seed (First camera t=0; R=Id, second estimated by 5 point algorithm)
   * @param[in] initialPair
   * @return
   */
  bool makeInitialPair3D(const Pair& initialPair);

  /**
   * @brief Automatic initial pair selection (based on a 'baseline' computation score)
   * @param[out] out_bestImagePairs
   * @param[in] filterViewId If defined, each output pairs must contain filterViewId
   * @return
   */
  bool getBestInitialImagePairs(std::vector<Pair>& out_bestImagePairs, IndexT filterViewId = UndefinedIndexT);

  /**
   * @brief Compute a score of the view for a subset of features. This is
   *        used for the next best view choice.
   *
   * The score is based on a pyramid which allows to compute a weighting
   * strategy to promote a good repartition in the image (instead of relying
   * only on the number of features).
   * Inspired by [Schonberger 2016]:
   * "Structure-from-Motion Revisited", Johannes L. Schonberger, Jan-Michael Frahm
   * 
   * http://people.inf.ethz.ch/jschoenb/papers/schoenberger2016sfm.pdf
   * We don't use the same weighting strategy. The weighting choice
   * is not justified in the paper.
   *
   * @param[in] viewId: the ID of the view
   * @param[in] trackIds: set of track IDs contained in viewId
   * @return the computed score
   */
  std::size_t computeCandidateImageScore(IndexT viewId, const std::vector<std::size_t>& trackIds) const;

  /**
   * @brief Apply the resection on a single view.
   * @param[in] viewIndex: image index to add to the reconstruction.
   * @param[out] resectionData: contains the result (P) and all the data used during the resection.
   * @return false if resection failed
   */
  bool computeResection(const IndexT viewIndex, ResectionData& resectionData);

  /**
   * @brief Update the global scene with the new found camera pose, intrinsic (if not defined) and 
   * Update its observations into the global scene structure.
   * @param[in] viewIndex: image index added to the reconstruction.
   * @param[in] resectionData: contains the camera pose and all data used during the resection.
   */
  void updateScene(const IndexT viewIndex, const ResectionData& resectionData);
                   
  /**
   * @brief  Triangulate new possible 2D tracks
   * List tracks that share content with this view and add observations and new 3D track if required.
   * @param previousReconstructedViews
   * @param newReconstructedViews
   */
  void triangulate_2Views(sfmData::SfMData& scene, const std::set<IndexT>& previousReconstructedViews, const std::set<IndexT>& newReconstructedViews);
  
  /**
   * @brief Triangulate new possible 2D tracks
   * List tracks that share content with this view and run a multiview triangulation on them, using the Lo-RANSAC algorithm.
   * @param[in,out] scene All the data about the 3D reconstruction.
   * @param[in] previousReconstructedViews The list of the old reconstructed views (views index).
   * @param[in] newReconstructedViews The list of the new reconstructed views (views index).
   */
  void triangulate_multiViewsLORANSAC(sfmData::SfMData& scene, const std::set<IndexT>& previousReconstructedViews, const std::set<IndexT>& newReconstructedViews);

  /**
   * @brief Check if a 3D points is well located in front of a set of views.
   * @param[in] pt3D A 3D point (euclidian coordinates)
   * @param[in] viewsId A set of views index
   * @param[in] scene All the data about the 3D reconstruction. 
   * @return false if the 3D points is located behind one view (or more), else \c true.
   */
  bool checkChieralities(const Vec3& pt3D, const std::set<IndexT>& viewsId, const sfmData::SfMData& scene);
  
  /**
   * @brief Check if the maximal angle formed by a 3D points and 2 views exceeds a min. angle, among a set of views.
   * @param[in] pt3D A 3D point (euclidian coordinates)
   * @param[in] viewsId A set of views index   
   * @param[in] scene All the data about the 3D reconstruction. 
   * @param[in] kMinAngle The angle limit.
   * @return false if the maximal angle does not exceed the limit, else \c true.
   */
  bool checkAngles(const Vec3& pt3D, const std::set<IndexT>& viewsId, const sfmData::SfMData& scene, const double& kMinAngle);

  /**
   * @brief Select the candidate tracks for the next triangulation step. 
   * @details A track is considered as triangulable if it is visible by at least one new reconsutructed 
   * view and at least \c _minNbObservationsForTriangulation (new and previous) reconstructed view.
   * @param[in] previousReconstructedViews The old reconstructed views.
   * @param[in] newReconstructedViews The newly reconstructed views.
   * @param[out] mapTracksToTriangulate A map with the tracks to triangulate and the observations to do it.
   */
  void getTracksToTriangulate(
      const std::set<IndexT>& previousReconstructedViews,
      const std::set<IndexT>& newReconstructedViews,
      std::map<IndexT, std::set<IndexT> > & mapTracksToTriangulate) const;

  /**
   * @brief Remove observation/tracks that have:
   * - too large residual error
   * - too small angular value
   *
   * @param[in] precision
   * @return number of removed outliers
   */
  std::size_t removeOutliers();

private:

  // Parameters
  Params _params;

  // Data providers

  feature::FeaturesPerView* _featuresPerView;
  matching::PairwiseMatches* _pairwiseMatches;

  // Pyramid scoring

  /// internal cache of precomputed values for the weighting of the pyramid levels
  std::vector<int> _pyramidWeights;
  int _pyramidThreshold;

  // Temporary data

  /// Putative landmark tracks (visibility per potential 3D point)
  track::TracksMap _map_tracks;
  /// Putative tracks per view
  track::TracksPerView _map_tracksPerView;
  /// Precomputed pyramid index for each trackId of each viewId.
  track::TracksPyramidPerView _map_featsPyramidPerView;
  /// Per camera confidence (A contrario estimated threshold error)
  HashMap<IndexT, double> _map_ACThreshold;

  // Local Bundle Adjustment data

  /// Contains all the data used by the Local BA approach
  std::shared_ptr<LocalBundleAdjustmentGraph> _localStrategyGraph;

  // Log

  /// sfm intermediate reconstruction files
  const std::string _sfmStepFolder;

  /// HTML logger
  std::shared_ptr<htmlDocument::htmlDocumentStream> _htmlDocStream;
  /// HTML log file
  std::string _htmlLogFile;
  /// property tree for json stats export
  pt::ptree _jsonLogTree;
};

} // namespace sfm
} // namespace aliceVision

