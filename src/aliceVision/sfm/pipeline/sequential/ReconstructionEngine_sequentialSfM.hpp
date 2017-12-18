// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "aliceVision/sfm/sfmDataIO.hpp"
#include "aliceVision/sfm/pipeline/ReconstructionEngine.hpp"
#include "aliceVision/feature/FeaturesPerView.hpp"
#include "aliceVision/sfm/pipeline/pairwiseMatchesIO.hpp"
#include "aliceVision/track/Track.hpp"
#include "aliceVision/sfm/LocalBundleAdjustmentData.hpp"

#include "dependencies/htmlDoc/htmlDoc.hpp"
#include "dependencies/histogram/histogram.hpp"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
namespace pt = boost::property_tree;

namespace aliceVision {
namespace sfm {

/// Sequential SfM Pipeline Reconstruction Engine.
class ReconstructionEngine_sequentialSfM : public ReconstructionEngine
{
public:
  ReconstructionEngine_sequentialSfM(
    const SfMData & sfm_data,
    const std::string & soutDirectory,
    const std::string & loggingFile = "");

  ~ReconstructionEngine_sequentialSfM();

  void setFeatures(feature::FeaturesPerView * featuresPerView)
  {
    _featuresPerView = featuresPerView;
  }

  void setMatches(matching::PairwiseMatches * pairwiseMatches)
  {
    _pairwiseMatches = pairwiseMatches;
  }

  void RobustResectionOfImages(
    const std::set<size_t>& viewIds,
    std::set<size_t>& set_reconstructedViewId,
    std::set<size_t>& set_rejectedViewId);

  virtual bool Process();

  void setInitialPair(const Pair & initialPair)
  {
    _userInitialImagePair = initialPair;
  }

  /// Initialize tracks
  bool InitLandmarkTracks();

  /// Select a candidate initial pair
  bool ChooseInitialPair(Pair & initialPairIndex) const;

  /// Compute the initial 3D seed (First camera t=0; R=Id, second estimated by 5 point algorithm)
  bool MakeInitialPair3D(const Pair & initialPair);

  /// Automatic initial pair selection (based on a 'baseline' computation score)
  bool getBestInitialImagePairs(std::vector<Pair>& out_bestImagePairs) const;
  
  /**
   * @brief Extension of the file format to store intermediate reconstruction files.
   */
  void setSfmdataInterFileExtension(const std::string& interFileExtension)
  {
    _sfmdataInterFileExtension = interFileExtension;
  }

  void setAllowUserInteraction(bool v)
  {
    _userInteraction = v;
  }

  void setMinInputTrackLength(int minInputTrackLength)
  {
    _minInputTrackLength = minInputTrackLength;
  }

  void setMinTrackLength(int minTrackLength)
  {
    _minTrackLength = minTrackLength;
  }
  
  void setLocalBundleAdjustmentGraphDistance(std::size_t distance)
  {
    if (!_uselocalBundleAdjustment)
      ALICEVISION_LOG_WARNING("Cannot set the local BA graph-distance limit: local BA not enabled.");
    else
      _localBA_data->setGraphDistanceLimit(distance);
  }

  void setUseLocalBundleAdjustmentStrategy(bool v)
  {
    _uselocalBundleAdjustment = v;
    if (v)
    {
      _localBA_data = std::make_shared<LocalBundleAdjustmentData>(_sfm_data);
      _localBA_data->setOutDirectory(stlplus::folder_append_separator(_sOutDirectory)+"localBA/");
      // delete all the previous data about the Local BA.
      if (stlplus::folder_exists(_localBA_data->getOutDirectory()))
        stlplus::folder_delete(_localBA_data->getOutDirectory(), true);
      stlplus::folder_create(_localBA_data->getOutDirectory());
    }
  }
protected:


private:
  /// Image score contains <ImageId, NbPutativeCommonPoint, score, isIntrinsicsReconstructed>
  typedef std::tuple<IndexT, std::size_t, std::size_t, bool> ViewConnectionScore;

  /// Return MSE (Mean Square Error) and a histogram of residual values.
  double ComputeResidualsHistogram(Histogram<double> * histo) const;

  /// Return MSE (Mean Square Error) and a histogram of tracks size.
  double ComputeTracksLengthsHistogram(Histogram<double> * histo) const;

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
  std::size_t computeImageScore(std::size_t viewId, const std::vector<std::size_t>& trackIds) const;

  /**
   * @brief Return all the images containing matches with already reconstructed 3D points.
   * The images are sorted by a score based on the number of features id shared with
   * the reconstruction and the repartition of these points in the image.
   *
   * @param[out] out_connectedViews: output list of view IDs connected with the 3D reconstruction.
   * @param[in] remainingViewIds: input list of remaining view IDs in which we will search for connected views.
   * @return False if there is no view connected.
   */
  bool FindConnectedViews(
    std::vector<ViewConnectionScore>& out_connectedViews,
    const std::set<size_t>& remainingViewIds) const;

  /**
   * @brief Estimate the best images on which we can compute the resectioning safely.
   * The images are sorted by a score based on the number of features id shared with
   * the reconstruction and the repartition of these points in the image.
   *
   * @param[out] out_selectedViewIds: output list of view IDs we can use for resectioning.
   * @param[in] remainingViewIds: input list of remaining view IDs in which we will search for the best ones for resectioning.
   * @return False if there is no possible resection.
   */
  bool FindNextImagesGroupForResection(
    std::vector<size_t>& out_selectedViewIds,
    const std::set<size_t>& remainingViewIds) const;

  /**
   * @brief Add a single Image to the scene and triangulate new possible tracks.
   * @param imageIndex
   * @return false if resection failed
   */
  bool Resection(const size_t imageIndex);

  /**
   * @brief  Triangulate new possible 2D tracks
   * List tracks that share content with this view and add observations and new 3D track if required.
   * @param previousReconstructedViews
   * @param newReconstructedViews
   */
  void triangulate(SfMData& scene, const std::set<IndexT>& previousReconstructedViews, const std::set<IndexT>& newReconstructedViews);

  /**
   * @brief Bundle adjustment to refine Structure; Motion and Intrinsics
   * @param fixedIntrinsics
   */
  bool BundleAdjustment(bool fixedIntrinsics);
  
  /**
     * @brief Apply the bundle adjustment choosing a small amount of parameters to reduce.
     * It reduces drastically the reconstruction time for big dataset of images.
     * @details The parameters to refine (landmarks, intrinsics, poses) are choosen according to the their 
     * proximity to the cameras newly added to the reconstruction.
     */
  bool localBundleAdjustment(const std::set<IndexT>& newReconstructedViews);
    
  /// Discard track with too large residual error
  bool badTrackRejector(double dPrecision, size_t count = 0);

  /// Export statistics in a JSON file
  void exportStatistics(double time_sfm);

  //----
  //-- Data
  //----

  // HTML logger
  std::shared_ptr<htmlDocument::htmlDocumentStream> _htmlDocStream;
  std::string _sLoggingFile;

  // Extension of the file format to store intermediate reconstruction files.
  std::string _sfmdataInterFileExtension = ".ply";
  ESfMData _sfmdataInterFilter = ESfMData(EXTRINSICS | INTRINSICS | STRUCTURE | OBSERVATIONS | CONTROL_POINTS);

  // Parameter
  bool _userInteraction = false;
  Pair _userInitialImagePair;
  int _minInputTrackLength = 2;
  int _minTrackLength = 2;
  int _minPointsPerPose = 30;
  bool _uselocalBundleAdjustment = false;
  
  //-- Data provider
  feature::FeaturesPerView  * _featuresPerView;
  matching::PairwiseMatches  * _pairwiseMatches;

  // Pyramid scoring
  const int _pyramidBase = 2;
  const int _pyramidDepth = 5;
  /// internal cache of precomputed values for the weighting of the pyramid levels
  std::vector<int> _pyramidWeights;
  int _pyramidThreshold;

  // Property tree for json stats export
  pt::ptree _tree;

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
  std::shared_ptr<LocalBundleAdjustmentData> _localBA_data;

  /// Remaining camera index that can be used for resection
  std::set<size_t> _set_remainingViewId;
};

} // namespace sfm
} // namespace aliceVision

