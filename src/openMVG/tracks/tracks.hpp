
// Copyright (c) 2012, 2013 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

// Implementation of [1] an efficient algorithm to compute track from pairwise
//  correspondences.
//
//  [1] Pierre Moulon and Pascal Monasse,
//    "Unordered feature tracking made fast and easy" CVMP 2012.
//
// It tracks the position of features along the series of image from pairwise
//  correspondences.
//
// From map< [imageI,ImageJ], [indexed matches array] > it builds tracks.
//
// Usage :
//  PairWiseMatches map_Matches;
//  PairedIndMatchImport(sMatchFile, map_Matches); // Load series of pairwise matches
//  //---------------------------------------
//  // Compute tracks from matches
//  //---------------------------------------
//  TracksBuilder tracksBuilder;
//  tracks::STLMAPTracks map_tracks;
//  tracksBuilder.Build(map_Matches); // Build: Efficient fusion of correspondences
//  tracksBuilder.Filter();           // Filter: Remove track that have conflict
//  tracksBuilder.ExportToSTL(map_tracks); // Build tracks with STL compliant type
//

#ifndef OPENMVG_TRACKS_H_
#define OPENMVG_TRACKS_H_

#include "lemon/list_graph.h"
#include "lemon/unionfind.h"

#include "openMVG/matching/indMatch.hpp"
#include "openMVG/stl/flatMap.hpp"
#include "openMVG/stl/flatSet.hpp"

#include <algorithm>
#include <iostream>
#include <functional>
#include <vector>
#include <set>
#include <map>
#include <memory>

namespace openMVG {

using namespace openMVG::matching;
using namespace lemon;

/// Lightweight copy of the flat_map of BOOST library
/// Use a vector to speed up insertion (preallocated array)
template<typename T1, typename T2>
class flat_pair_map
{
  typedef std::pair<T1, T2> P;
public:
  typedef typename std::vector< P >::iterator iterator;

  typename std::vector< P >::iterator find(const T1 & val)  {
    return std::lower_bound(m_vec.begin(), m_vec.end(), val, superiorToFirst);
  }

  T2 & operator[](const T1 & val) {
    return std::lower_bound(m_vec.begin(), m_vec.end(), val, superiorToFirst)->second;
  }

  void sort()  {std::sort(m_vec.begin(), m_vec.end(), sortPairAscend);}
  void push_back(const P & val)  { m_vec.push_back(val);  }
  void clear()  { m_vec.clear(); }
  void reserve(size_t count)  { m_vec.reserve(count); }
private:
  std::vector< P > m_vec;

  static bool sortPairAscend(const P &a, const P &b) {return a.first<b.first;}
  static bool superiorToFirst(const P &a, const T1 &b) {return a.first<b;}
};

namespace tracks {

/// Data structure to store a track: collection of {ImageId,FeatureId}
///  The corresponding image points with their imageId and FeatureId.
typedef stl::flat_map<size_t,size_t> submapTrack;
/// A track is a collection of {trackId, submapTrack}
typedef stl::flat_map< size_t, submapTrack > STLMAPTracks;
typedef std::vector<size_t> TrackIdSet;

/**
 * @brief Data structure that contains for each features of each view, its corresponding cell positions for each level of the pyramid, i.e.
 * for each view:
 *   each feature is mapped N times (N=depth of the pyramid)
 *      each times it contains the absolute position P of the cell in the corresponding pyramid level
 *
 * FeatsPyramidPerView contains map<viewId, map<trackId*N, pyramidIndex>>
 *
 * Cell position:
 * Consider the set of all cells of all pyramids, there are M = \sum_{l=1...N} K_l^2 cells with K_l = 2^l and l=1...N
 * We enumerate the cells starting from the first pyramid l=1 (so that they have position from 0 to 3 (ie K^2 - 1))
 * and we go on for increasing values of l so that e.g. the first cell of the pyramid at l=2 has position K^2, the second K^2 + 1 etc...
 * So in general the i-th cell of the pyramid at level l has position P= \sum_{j=1...l-1} K_j^2 + i
 */
typedef stl::flat_map< size_t, stl::flat_map<size_t, size_t> > TracksPyramidPerView;
/**
 * List of visible track ids for each view.
 *
 * TracksPerView contains <viewId, vector<trackId> >
 */
typedef stl::flat_map< size_t, TrackIdSet > TracksPerView;

struct TracksBuilder
{
  typedef std::pair<size_t, size_t> indexedFeaturePair;
  typedef ListDigraph::NodeMap<size_t> IndexMap;
  typedef lemon::UnionFindEnum< IndexMap > UnionFindObject;

  typedef flat_pair_map< lemon::ListDigraph::Node, indexedFeaturePair> MapNodeToIndex;
  typedef flat_pair_map< indexedFeaturePair, lemon::ListDigraph::Node > MapIndexToNode;

  lemon::ListDigraph _graph; //Graph container to create the node
  MapNodeToIndex _map_nodeToIndex; //Node to index map
  std::unique_ptr<IndexMap> _index;
  std::unique_ptr<UnionFindObject> _tracksUF;

  const UnionFindObject & getUnionFindEnum() const {return *_tracksUF; }
  const MapNodeToIndex & getReverseMap() const {return _map_nodeToIndex;}

  /// Build tracks for a given series of pairWise matches
  bool Build( const PairWiseMatches &  map_pair_wise_matches)
  {
    typedef std::set<indexedFeaturePair> SetIndexedPair;
    // Set of all features of all images: (imageIndex, featureIndex)
    SetIndexedPair allFeatures;
    // For each couple of images
    for (PairWiseMatches::const_iterator iter = map_pair_wise_matches.begin();
      iter != map_pair_wise_matches.end();
      ++iter)
    {
      const size_t & I = iter->first.first;
      const size_t & J = iter->first.second;
      // Features correspondences between I and J image.
      const std::vector<IndMatch> & vec_FilteredMatches = iter->second;

      // Retrieve all features
      for( size_t k = 0; k < vec_FilteredMatches.size(); ++k)
      {
        allFeatures.insert(std::make_pair(I,vec_FilteredMatches[k]._i));
        allFeatures.insert(std::make_pair(J,vec_FilteredMatches[k]._j));
      }
    }

    // Build the node indirection for each referenced feature
    MapIndexToNode map_indexToNode;
    map_indexToNode.reserve(allFeatures.size());
    _map_nodeToIndex.reserve(allFeatures.size());
    for (SetIndexedPair::const_iterator iter = allFeatures.begin();
      iter != allFeatures.end();
      ++iter)
    {
      lemon::ListDigraph::Node node = _graph.addNode();
      map_indexToNode.push_back( std::make_pair(*iter, node));
      _map_nodeToIndex.push_back( std::make_pair(node,*iter));
    }

    // Sort the flat_pair_map
    map_indexToNode.sort();
    _map_nodeToIndex.sort();

    // Add the element of myset to the UnionFind insert method.
    _index = std::unique_ptr<IndexMap>( new IndexMap(_graph) );
    _tracksUF = std::unique_ptr<UnionFindObject>( new UnionFindObject(*_index));
    for (ListDigraph::NodeIt it(_graph); it != INVALID; ++it) {
      _tracksUF->insert(it);
    }

    // Make the union according the pair matches
    for (PairWiseMatches::const_iterator iter = map_pair_wise_matches.begin();
      iter != map_pair_wise_matches.end();
      ++iter)
    {
      const size_t & I = iter->first.first;
      const size_t & J = iter->first.second;
      const std::vector<IndMatch> & vec_FilteredMatches = iter->second;
      // We have correspondences between I and J image index.

      for( size_t k = 0; k < vec_FilteredMatches.size(); ++k)
      {
        indexedFeaturePair pairI(I,vec_FilteredMatches[k]._i);
        indexedFeaturePair pairJ(J,vec_FilteredMatches[k]._j);
        _tracksUF->join( map_indexToNode[pairI], map_indexToNode[pairJ] );
      }
    }
    return false;
  }

  /// Remove bad tracks (too short or track with ids collision)
  bool Filter(size_t nLengthSupTo = 2, bool bMultithread = true)
  {
    // Remove bad tracks:
    // - track that are too short,
    // - track with id conflicts (many times the same image index)

    std::set<int> set_classToErase;
#ifdef OPENMVG_USE_OPENMP
    #pragma omp parallel if(bMultithread)
#endif
    for ( lemon::UnionFindEnum< IndexMap >::ClassIt cit(*_tracksUF); cit != INVALID; ++cit) {
#ifdef OPENMVG_USE_OPENMP
    #pragma omp single nowait
#endif
      {
        size_t cpt = 0;
        std::set<size_t> myset;
        for (lemon::UnionFindEnum< IndexMap >::ItemIt iit(*_tracksUF, cit); iit != INVALID; ++iit) {
          myset.insert(_map_nodeToIndex[ iit ].first);
          ++cpt;
        }
        if (myset.size() != cpt || myset.size() < nLengthSupTo)
        {
#ifdef OPENMVG_USE_OPENMP
          #pragma omp critical
#endif
          set_classToErase.insert(cit.operator int());
        }
      }
    }
    std::for_each (set_classToErase.begin(), set_classToErase.end(),
      std::bind1st( std::mem_fun( &UnionFindObject::eraseClass ), _tracksUF.get() ));
    return false;
  }

  bool ExportToStream(std::ostream & os)
  {
    size_t cpt = 0;
    for ( lemon::UnionFindEnum< IndexMap >::ClassIt cit(*_tracksUF); cit != INVALID; ++cit) {
      os << "Class: " << cpt++ << std::endl;
      size_t cptTrackLength = 0;
      for (lemon::UnionFindEnum< IndexMap >::ItemIt iit(*_tracksUF, cit); iit != INVALID; ++iit) {
        ++cptTrackLength;
      }
      os << "\t" << "track length: " << cptTrackLength << std::endl;

      for (lemon::UnionFindEnum< IndexMap >::ItemIt iit(*_tracksUF, cit); iit != INVALID; ++iit) {
        os << _map_nodeToIndex[ iit ].first << "  " << _map_nodeToIndex[ iit ].second << std::endl;
      }
    }
    return os.good();
  }

  /// Return the number of connected set in the UnionFind structure (tree forest)
  size_t NbTracks() const
  {
    size_t cpt = 0;
    for ( lemon::UnionFindEnum< IndexMap >::ClassIt cit(*_tracksUF); cit != INVALID; ++cit)
      ++cpt;
    return cpt;
  }

  /// Export tracks as a map (each entry is a sequence of imageId and featureIndex):
  ///  {TrackIndex => {(imageIndex, featureIndex), ... ,(imageIndex, featureIndex)}
  void ExportToSTL(STLMAPTracks & map_tracks)
  {
    map_tracks.clear();

    size_t cptClass = 0;
    for ( lemon::UnionFindEnum< IndexMap >::ClassIt cit(*_tracksUF); cit != INVALID; ++cit, ++cptClass) {
      std::pair<STLMAPTracks::iterator, bool> ret =
        map_tracks.insert(std::pair<size_t, submapTrack >(cptClass, submapTrack()));
      STLMAPTracks::iterator iterN = ret.first;

      for (lemon::UnionFindEnum< IndexMap >::ItemIt iit(*_tracksUF, cit); iit != INVALID; ++iit) {
        const MapNodeToIndex::iterator iterTrackValue = _map_nodeToIndex.find(iit);
        const indexedFeaturePair & currentPair = iterTrackValue->second;

        iterN->second[currentPair.first] = currentPair.second;
      }
    }
  }
};

struct TracksUtilsMap
{
  /**
   * @brief Find common tracks between images.
   *
   * @param[in] set_imageIndex: set of images we are looking for common tracks
   * @param[in] map_tracksIn: all tracks of the scene
   * @param[out] map_tracksOut: output with only the common tracks
   */
  static bool GetTracksInImages(
    const std::set<size_t> & set_imageIndex,
    const STLMAPTracks & map_tracksIn,
    STLMAPTracks & map_tracksOut)
  {
    assert(!set_imageIndex.empty());
    map_tracksOut.clear();

    // Go along the tracks
    for (auto& trackIn: map_tracksIn)
    {
      // Look if the track contains the provided view index & save the point ids
      submapTrack map_temp;
      for (size_t imageIndex: set_imageIndex)
      {
        submapTrack::const_iterator iterSearch = trackIn.second.find(imageIndex);
        if (iterSearch == trackIn.second.end())
            break; // at least one request image is not in the track
        map_temp[iterSearch->first] = iterSearch->second;
      }
      // if we have a feature for each input image
      // we can add it to the output tracks.
      if (map_temp.size() == set_imageIndex.size())
        map_tracksOut[trackIn.first] = std::move(map_temp);
    }
    return !map_tracksOut.empty();
  }
  
  static void GetCommonTracksInImages(
    const std::set<size_t> & set_imageIndex,
    const TracksPerView & map_tracksPerView,
    std::set<size_t> & set_visibleTracks)
  {
    assert(!set_imageIndex.empty());
    set_visibleTracks.clear();
    
    std::set<size_t>::const_iterator it = set_imageIndex.cbegin();
    {
      TracksPerView::const_iterator tracksPerViewIt = map_tracksPerView.find(*it);
      if(tracksPerViewIt == map_tracksPerView.end())
        return;
      const TrackIdSet& imageTracks = tracksPerViewIt->second;
      set_visibleTracks.insert(imageTracks.cbegin(), imageTracks.cend());
    }
    ++it;
    for(; it != set_imageIndex.cend(); ++it)
    {
      TracksPerView::const_iterator tracksPerViewIt = map_tracksPerView.find(*it);
      if(tracksPerViewIt == map_tracksPerView.end())
        return;
      const TrackIdSet& imageTracks = tracksPerViewIt->second;
      std::set<size_t> tmp;
      std::set_intersection(
          set_visibleTracks.cbegin(), set_visibleTracks.cend(),
          imageTracks.cbegin(), imageTracks.cend(),
          std::inserter(tmp, tmp.begin()));
      set_visibleTracks.swap(tmp);
    }
  }
  
  /**
   * @brief Find common tracks between images.
   *
   * @param[in] set_imageIndex: set of images we are looking for common tracks
   * @param[in] map_tracksIn: all tracks of the scene
   * @param[out] map_tracksOut: output with only the common tracks
   */
  static bool GetTracksInImagesFast(
    const std::set<size_t> & set_imageIndex,
    const STLMAPTracks & map_tracksIn,
    const TracksPerView & map_tracksPerView,
    STLMAPTracks & map_tracksOut)
  {
    assert(!set_imageIndex.empty());
    map_tracksOut.clear();
    
    std::set<size_t> set_visibleTracks;
    GetCommonTracksInImages(set_imageIndex, map_tracksPerView, set_visibleTracks);
    
    // Go along the tracks
    for (size_t visibleTrack: set_visibleTracks)
    {
      STLMAPTracks::const_iterator itTrackIn = map_tracksIn.find(visibleTrack);
      if(itTrackIn == map_tracksIn.end())
        continue;
      const submapTrack& trackFeatsIn = itTrackIn->second;
      submapTrack& trackFeatsOut = map_tracksOut[visibleTrack];
      for (size_t imageIndex: set_imageIndex)
      {
        submapTrack::const_iterator trackFeatsInIt = trackFeatsIn.find(imageIndex);
        if(trackFeatsInIt != trackFeatsIn.end())
          trackFeatsOut[imageIndex] = trackFeatsInIt->second;
      }
    }
    return !map_tracksOut.empty();
  }

  /// Return the tracksId of one image
  static void GetImageTracksId(
    const STLMAPTracks & map_tracks,
    const size_t & imageIndex,
    std::set<size_t> * set_tracksIds)
  {
    set_tracksIds->clear();
    for (auto& track: map_tracks)
    {
      submapTrack::const_iterator iterSearch = track.second.find(imageIndex);
      if (iterSearch != track.second.end())
        set_tracksIds->insert(track.first);
    }
  }

  static void computeTracksPerView(const STLMAPTracks & map_tracks, TracksPerView& map_tracksPerView)
  {
    for (auto& track: map_tracks)
    {
      for (auto& feat: track.second)
      {
        TrackIdSet& tracksSet = map_tracksPerView[feat.first];
        if(tracksSet.empty())
          tracksSet.reserve(1000);
        tracksSet.push_back(track.first);
      }
    }
    // sort tracks Ids in each view
#ifdef OPENMVG_USE_OPENMP
    #pragma omp parallel for
#endif
    for(int i = 0; i < map_tracksPerView.size(); ++i)
    {
      TracksPerView::iterator it = map_tracksPerView.begin();
      std::advance(it, i);
      std::sort(it->second.begin(), it->second.end());
    }
  }

  /// Return the tracksId as a set (sorted increasing)
  static void GetTracksIdVector(
    const STLMAPTracks & map_tracks,
    std::set<size_t> * set_tracksIds)
  {
    set_tracksIds->clear();
    for (STLMAPTracks::const_iterator iterT = map_tracks.begin();
      iterT != map_tracks.end(); ++iterT)
    {
      set_tracksIds->insert(iterT->first);
    }
  }

  /// Get feature index PerView and TrackId
  static bool GetFeatIndexPerViewAndTrackId(
    const STLMAPTracks & map_tracks,
    const std::set<size_t> & set_trackId,
    size_t nImageIndex,
    std::vector<size_t> * pvec_featIndex)
  {
    for (size_t trackId: set_trackId)
    {
      STLMAPTracks::const_iterator iterT = map_tracks.find(trackId);
      // Ignore it if the track doesn't exist
      if(iterT == map_tracks.end())
        continue;
      // Try to find imageIndex
      const submapTrack & map_ref = iterT->second;
      submapTrack::const_iterator iterSearch = map_ref.find(nImageIndex);
      if (iterSearch != map_ref.end())
      {
        pvec_featIndex->emplace_back(iterSearch->second);
      }
    }
    return !pvec_featIndex->empty();
  }

  struct FunctorMapFirstEqual : public std::unary_function <STLMAPTracks , bool>
  {
    size_t id;
    FunctorMapFirstEqual(size_t val):id(val){};
    bool operator()(const std::pair<size_t, submapTrack > & val) {
      return ( id == val.first);
    }
  };

  /**
   * @brief Convert a trackId to a vector of indexed Matches.
   *
   * @param[in]  map_tracks: set of tracks with only 2 elements
   *             (image A and image B) in each submapTrack.
   * @param[in]  vec_filterIndex: the track indexes to retrieve.
   *             Only track indexes contained in this filter vector are kept.
   * @param[out] pvec_index: list of matches
   *             (feature index in image A, feature index in image B).
   *
   * @warning The input tracks must be composed of only two images index.
   * @warning Image index are considered sorted (increasing order).
   */
  static void TracksToIndexedMatches(const STLMAPTracks & map_tracks,
    const std::vector<IndexT> & vec_filterIndex,
    std::vector<IndMatch> * pvec_index)
  {

    std::vector<IndMatch> & vec_indexref = *pvec_index;
    vec_indexref.clear();
    for (size_t i = 0; i < vec_filterIndex.size(); ++i)
    {
      // Retrieve the track information from the current index i.
      STLMAPTracks::const_iterator itF =
        find_if(map_tracks.begin(), map_tracks.end(), FunctorMapFirstEqual(vec_filterIndex[i]));
      // The current track.
      const submapTrack & map_ref = itF->second;

      // We have 2 elements for a track.
      assert(map_ref.size() == 2);
      const IndexT indexI = (map_ref.begin())->second;
      const IndexT indexJ = (++map_ref.begin())->second;

      vec_indexref.emplace_back(indexI, indexJ);
    }
  }

  /// Return the occurrence of tracks length.
  static void TracksLength(const STLMAPTracks & map_tracks,
    std::map<size_t, size_t> & map_Occurence_TrackLength)
  {
    for (STLMAPTracks::const_iterator iterT = map_tracks.begin();
      iterT != map_tracks.end(); ++iterT)
    {
      const size_t trLength = iterT->second.size();
      if (map_Occurence_TrackLength.end() ==
        map_Occurence_TrackLength.find(trLength))
      {
        map_Occurence_TrackLength[trLength] = 1;
      }
      else
      {
        map_Occurence_TrackLength[trLength] += 1;
      }
    }
  }

  /// Return a set containing the image Id considered in the tracks container.
  static void ImageIdInTracks(const TracksPerView & map_tracksPerView,
    std::set<size_t> & set_imagesId)
  {
    for (auto& viewTracks: map_tracksPerView)
    {
      set_imagesId.insert(viewTracks.first);
    }
  }
  
  /// Return a set containing the image Id considered in the tracks container.
  static void ImageIdInTracks(const STLMAPTracks & map_tracks,
    std::set<size_t> & set_imagesId)
  {
    for (STLMAPTracks::const_iterator iterT = map_tracks.begin();
      iterT != map_tracks.end(); ++iterT)
    {
      const submapTrack & map_ref = iterT->second;
      for (submapTrack::const_iterator iter = map_ref.begin();
        iter != map_ref.end();
        ++iter)
      {
        set_imagesId.insert(iter->first);
      }
    }
  }
};

} // namespace tracks
} // namespace openMVG

#endif // OPENMVG_TRACKS_H_
