// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "colorizeTracks.hpp"
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/stl/indexedSort.hpp>
#include <aliceVision/stl/mapUtils.hpp>
#include <aliceVision/image/io.hpp>

#include <boost/progress.hpp>

#include <map>
#include <vector>

namespace aliceVision {
namespace sfm {

bool colorizeTracks(sfmData::SfMData& sfmData)
{
  // colorize each track:
  // start with the most representative image
  // and iterate to provide a color to each 3D point

  std::vector<Vec3> vec_3dPoints;

  boost::progress_display progressBar(sfmData.getLandmarks().size(), std::cout, "\nCompute scene structure color\n");

  vec_3dPoints.resize(sfmData.getLandmarks().size());

  // build a list of contiguous index for the trackIds
  std::map<IndexT, IndexT> trackIds_to_contiguousIndexes;
  IndexT cpt = 0;
  for(sfmData::Landmarks::const_iterator it = sfmData.getLandmarks().begin(); it != sfmData.getLandmarks().end(); ++it, ++cpt)
  {
    trackIds_to_contiguousIndexes[it->first] = cpt;
    vec_3dPoints[cpt] = it->second.X;
  }

  // the track list that will be colored (point removed during the process)
  std::set<IndexT> remainingTrackToColor;
  std::transform(sfmData.getLandmarks().begin(), sfmData.getLandmarks().end(),
    std::inserter(remainingTrackToColor, remainingTrackToColor.begin()),
    stl::RetrieveKey());

  while(!remainingTrackToColor.empty())
  {
    // find the most representative image (for the remaining 3D points)
    //  a. count the number of observation per view for each 3Dpoint Index
    //  b. sort to find the most representative view index

    std::map<IndexT, IndexT> map_IndexCardinal; // ViewId, Cardinal
    for(std::set<IndexT>::const_iterator iterT = remainingTrackToColor.begin(); iterT != remainingTrackToColor.end(); ++iterT)
    {
      const std::size_t trackId = *iterT;
      const sfmData::Observations& observations = sfmData.getLandmarks().at(trackId).observations;

      for(sfmData::Observations::const_iterator iterObs = observations.begin(); iterObs != observations.end(); ++iterObs)
      {
        const size_t viewId = iterObs->first;
        if (map_IndexCardinal.find(viewId) == map_IndexCardinal.end())
          map_IndexCardinal[viewId] = 1;
        else
          ++map_IndexCardinal[viewId];
      }
    }

    // find the View index that is the most represented
    std::vector<IndexT> vec_cardinal;
    std::transform(map_IndexCardinal.begin(),
      map_IndexCardinal.end(),
      std::back_inserter(vec_cardinal),
      stl::RetrieveValue());
    using namespace stl::indexed_sort;
    std::vector< sort_index_packet_descend< IndexT, IndexT> > packet_vec(vec_cardinal.size());
    sort_index_helper(packet_vec, &vec_cardinal[0], 1);

    // first image index with the most of occurrence
    std::map<IndexT, IndexT>::const_iterator iterTT = map_IndexCardinal.begin();
    std::advance(iterTT, packet_vec[0].index);
    const std::size_t view_index = iterTT->first;
    const sfmData::View * view = sfmData.getViews().at(view_index).get();
    const std::string imagePath = view->getImagePath();
    image::Image<image::RGBColor> image;
    image::readImage(imagePath, image);

    // iterate through the remaining track to color
    // - look if the current view is present to color the track
    std::set<IndexT> set_toRemove;
    for(std::set<IndexT>::const_iterator iterT = remainingTrackToColor.begin(); iterT != remainingTrackToColor.end(); ++iterT)
    {
      const std::size_t trackId = *iterT;
      const sfmData::Observations& observations = sfmData.getLandmarks().at(trackId).observations;
      sfmData::Observations::const_iterator it = observations.find(view_index);

      if(it != observations.end())
      {
        // color the track
        Vec2 pt = it->second.x;
        // clamp the pixel position if the feature/marker center is outside the image.
        pt.x() = clamp(pt.x(), 0.0, double(image.Width()-1));
        pt.y() = clamp(pt.y(), 0.0, double(image.Height()-1));
        sfmData.structure.at(trackId).rgb = image(pt.y(), pt.x());
        set_toRemove.insert(trackId);
        ++progressBar;
      }
    }

    // remove colored track
    for(std::set<IndexT>::const_iterator iter = set_toRemove.begin();
      iter != set_toRemove.end(); ++iter)
    {
      remainingTrackToColor.erase(*iter);
    }
  }
  return true;
}

} // namespace sfm
} // namespace aliceVision
