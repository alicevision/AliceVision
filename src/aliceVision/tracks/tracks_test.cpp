// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#include "CppUnitLite/TestHarness.h"
#include "testing/testing.h"

#include "aliceVision/tracks/tracks.hpp"
#include "aliceVision/matching/IndMatch.hpp"

#include <vector>
#include <utility>


using namespace aliceVision::feature;
using namespace aliceVision::tracks;
using namespace aliceVision::matching;


TEST(Tracks, Simple) {

  // Create some tracks for image (A,B,C)
  // {A,B,C} imageId will be {0,1,2}
  // For those image link some features id depicted below
  //A    B    C
  //0 -> 0 -> 0
  //1 -> 1 -> 6
  //2 -> 3

  // Create the input pairwise correspondences
  PairwiseMatches map_pairwisematches;

  const IndMatch testAB[] = {IndMatch(0,0), IndMatch(1,1), IndMatch(2,3)};
  const IndMatch testBC[] = {IndMatch(0,0), IndMatch(1,6)};

  const std::vector<IndMatch> ab(testAB, testAB+3);
  const std::vector<IndMatch> bc(testBC, testBC+2);
  const int A = 0;
  const int B = 1;
  const int C = 2;
  map_pairwisematches[ std::make_pair(A, B) ][EImageDescriberType::UNKNOWN] = ab;
  map_pairwisematches[ std::make_pair(B, C) ][EImageDescriberType::UNKNOWN] = bc;

  //-- Build tracks using the interface tracksbuilder
  TracksBuilder trackBuilder;
  trackBuilder.Build( map_pairwisematches );

  trackBuilder.ExportToStream(std::cout);

  TracksMap map_tracks;
  trackBuilder.ExportToSTL(map_tracks);

  //-------------------
  // Unit Test check
  //-------------------

  //0, {(0,0) (1,0) (2,0)}
  //1, {(0,1) (1,1) (2,6)}
  //2, {(0,2) (1,3)}
  const std::pair<std::size_t,std::size_t> GT_Tracks[] =
  {
    std::make_pair(0,0), std::make_pair(1,0), std::make_pair(2,0),
    std::make_pair(0,1), std::make_pair(1,1), std::make_pair(2,6),
    std::make_pair(0,2), std::make_pair(1,3)
  };

  CHECK_EQUAL(3,  map_tracks.size());
  std::size_t cpt = 0, i = 0;
  for (TracksMap::const_iterator iterT = map_tracks.begin();
    iterT != map_tracks.end();
    ++iterT, ++i)
  {
    CHECK_EQUAL(i, iterT->first);
    for (auto iter = iterT->second.featPerView.begin();
      iter != iterT->second.featPerView.end();
      ++iter)
    {
      CHECK( GT_Tracks[cpt] == std::make_pair(iter->first, iter->second));
      ++cpt;
    }
  }
}

TEST(Tracks, filter_3viewAtLeast) {

  //
  //A    B    C
  //0 -> 0 -> 0
  //1 -> 1 -> 6
  //2 -> 3
  //

  // Create the input pairwise correspondences
  PairwiseMatches map_pairwisematches;

  IndMatch testAB[] = {IndMatch(0,0), IndMatch(1,1), IndMatch(2,3)};
  IndMatch testBC[] = {IndMatch(0,0), IndMatch(1,6)};


  std::vector<IndMatch> ab(testAB, testAB+3);
  std::vector<IndMatch> bc(testBC, testBC+2);
  const int A = 0;
  const int B = 1;
  const int C = 2;
  map_pairwisematches[ std::make_pair(A,B) ][EImageDescriberType::UNKNOWN] = ab;
  map_pairwisematches[ std::make_pair(B,C) ][EImageDescriberType::UNKNOWN] = bc;

  //-- Build tracks using the interface tracksbuilder
  TracksBuilder trackBuilder;
  trackBuilder.Build( map_pairwisematches );
  CHECK_EQUAL(3, trackBuilder.NbTracks());
  trackBuilder.Filter(3);
  CHECK_EQUAL(2, trackBuilder.NbTracks());
}

TEST(Tracks, Conflict) {

  //
  //A    B    C
  //0 -> 0 -> 0
  //1 -> 1 -> 6
  //{2 -> 3 -> 2
  //      3 -> 8 } This track must be deleted, index 3 appears two times
  //

  // Create the input pairwise correspondences
  PairwiseMatches map_pairwisematches;

  const IndMatch testAB[] = {IndMatch(0,0), IndMatch(1,1), IndMatch(2,3)};
  const IndMatch testBC[] = {IndMatch(0,0), IndMatch(1,6), IndMatch(3,2), IndMatch(3,8)};

  std::vector<IndMatch> ab(testAB, testAB+3);
  std::vector<IndMatch> bc(testBC, testBC+4);
  const int A = 0;
  const int B = 1;
  const int C = 2;
  map_pairwisematches[ std::make_pair(A,B) ][EImageDescriberType::UNKNOWN] = ab;
  map_pairwisematches[ std::make_pair(B,C) ][EImageDescriberType::UNKNOWN] = bc;

  //-- Build tracks using the interface tracksbuilder
  TracksBuilder trackBuilder;
  trackBuilder.Build( map_pairwisematches );

  CHECK_EQUAL(3, trackBuilder.NbTracks());
  trackBuilder.Filter(); // Key feature tested here to kill the conflicted track
  CHECK_EQUAL(2, trackBuilder.NbTracks());

  TracksMap map_tracks;
  trackBuilder.ExportToSTL(map_tracks);

  //-------------------
  // Unit Test check
  //-------------------

  //0, {(0,0) (1,0) (2,0)}
  //1, {(0,1) (1,1) (2,6)}
  const std::pair<std::size_t,std::size_t> GT_Tracks[] =
    {std::make_pair(0,0), std::make_pair(1,0), std::make_pair(2,0),
     std::make_pair(0,1), std::make_pair(1,1), std::make_pair(2,6)};

  CHECK_EQUAL(2,  map_tracks.size());
  std::size_t cpt = 0, i = 0;
  for (TracksMap::const_iterator iterT = map_tracks.begin();
    iterT != map_tracks.end();
    ++iterT, ++i)
  {
    CHECK_EQUAL(i, iterT->first);
    for (auto iter = iterT->second.featPerView.begin();
      iter != iterT->second.featPerView.end();
      ++iter)
    {
      CHECK( GT_Tracks[cpt] == std::make_pair(iter->first, iter->second));
      ++cpt;
    }
  }
}

TEST(Tracks, GetCommonTracksInImages)
{
  {
    std::set<std::size_t> set_imageIndex {15, 20};
    TracksPerView map_tracksPerView;

    std::vector<std::size_t> base{1,2,3,4};
    map_tracksPerView[10] = base;
    map_tracksPerView[15] = base;
    map_tracksPerView[20] = base;
    map_tracksPerView[40] = base;
    map_tracksPerView[15].push_back(5);
    map_tracksPerView[20].push_back(6);

    std::set<std::size_t> set_visibleTracks;
    TracksUtilsMap::GetCommonTracksInImages(set_imageIndex, map_tracksPerView, set_visibleTracks);
    CHECK_EQUAL(base.size(), set_visibleTracks.size());
    set_visibleTracks.clear();
    // test non-existing view index
    TracksUtilsMap::GetCommonTracksInImages({15, 50}, map_tracksPerView, set_visibleTracks);
    CHECK(set_visibleTracks.empty());
  }
  {
    std::set<std::size_t> set_imageIndex {15, 20, 10, 40};
    TracksPerView map_tracksPerView;

    std::vector<std::size_t> base{1,2,3,4};
    map_tracksPerView[10] = base;
    map_tracksPerView[15] = base;
    map_tracksPerView[20] = base;
    map_tracksPerView[40] = base;

    map_tracksPerView[10].push_back(100);
    map_tracksPerView[15].push_back(100);
    map_tracksPerView[20].push_back(100);

    map_tracksPerView[15].push_back(200);
    map_tracksPerView[20].push_back(200);
    map_tracksPerView[40].push_back(200);

    map_tracksPerView[15].push_back(5);
    map_tracksPerView[20].push_back(6);

    std::set<std::size_t> set_visibleTracks;
    TracksUtilsMap::GetCommonTracksInImages(set_imageIndex, map_tracksPerView, set_visibleTracks);
    CHECK_EQUAL(base.size(), set_visibleTracks.size());
  }
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
