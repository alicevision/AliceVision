// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "PreMatchCams.hpp"
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/mvsData/Point2d.hpp>
#include <aliceVision/mvsUtils/fileIO.hpp>
#include <aliceVision/mvsUtils/common.hpp>

#include <iostream>

namespace aliceVision {
namespace mvsUtils {

PreMatchCams::PreMatchCams(const MultiViewParams& mp)
  : _mp(mp)
{
  minang = static_cast<float>(_mp.userParams.get<double>("prematching.minAngle", 2.0));
  maxang = static_cast<float>(_mp.userParams.get<double>("prematching.maxAngle", 70.0)); // WARNING: may be too low, especially when using seeds from SFM
}

StaticVector<int> PreMatchCams::findNearestCamsFromLandmarks(int rc, int nbNearestCams)
{
  StaticVector<int> out;
  std::vector<SortedId> ids;
  ids.reserve(_mp.getNbCameras());

  for(int tc = 0; tc < _mp.getNbCameras(); ++tc)
    ids.push_back(SortedId(tc,0));

  const IndexT viewId = _mp.getViewId(rc);

  for(const auto& landmarkPair :_mp.getInputSfMData().getLandmarks())
  {
    const auto& observations = landmarkPair.second.observations;

    if(observations.find(viewId) == observations.end())
      continue;

    for(const auto& observationPair : observations)
    {
      if(observationPair.first != viewId)
      {
        const int tc = _mp.getIndexFromViewId(observationPair.first);
        ++ids.at(tc).value;
      }
    }
  }

  qsort(&ids[0], ids.size(), sizeof(SortedId), qsortCompareSortedIdDesc);

  // ensure the ideal number of target cameras is not superior to the actual number of cameras
  const int maxTc = std::min(std::min(_mp.getNbCameras(), nbNearestCams), static_cast<int>(ids.size()));
  out.reserve(maxTc);

  for(int i = 0; i < maxTc; ++i)
  {
    // a minimum of 10 common points is required (10*2 because points are stored in both rc/tc combinations)
    if(ids[i].value > (10 * 2))
      out.push_back(ids[i].id);
  }

  if(out.size() < nbNearestCams)
    ALICEVISION_LOG_INFO("Found only " << out.size() << "/" << nbNearestCams << " nearest cameras for view id: " << _mp.getViewId(rc));

  return out;
}

// hexahedron format ... 0-3 frontal face, 4-7 back face
StaticVector<int> PreMatchCams::findCamsWhichIntersectsHexahedron(const Point3d hexah[8],
                                                                  const std::string& minMaxDepthsFileName)
{
    StaticVector<Point2d>* minMaxDepths = loadArrayFromFile<Point2d>(minMaxDepthsFileName);
    StaticVector<int> tcams;
    tcams.reserve(_mp.getNbCameras());
    for(int rc = 0; rc < _mp.getNbCameras(); rc++)
    {
        float mindepth = (*minMaxDepths)[rc].x;
        float maxdepth = (*minMaxDepths)[rc].y;
        if((mindepth > 0.0f) && (maxdepth > mindepth))
        {
            Point3d rchex[8];
            getCamHexahedron(&_mp, rchex, rc, mindepth, maxdepth);
            if(intersectsHexahedronHexahedron(rchex, hexah))
            {
                tcams.push_back(rc);
            }
        }
    }
    delete minMaxDepths;
    return tcams;
}

// hexahedron format ... 0-3 frontal face, 4-7 back face
StaticVector<int> PreMatchCams::findCamsWhichIntersectsHexahedron(const Point3d hexah[8])
{
    StaticVector<int> tcams;
    tcams.reserve(_mp.getNbCameras());
    for(int rc = 0; rc < _mp.getNbCameras(); rc++)
    {
        float mindepth, maxdepth;
        StaticVector<int>* pscams;
        if(getDepthMapInfo(rc, &_mp, mindepth, maxdepth, &pscams))
        {
            delete pscams;
            Point3d rchex[8];
            getCamHexahedron(&_mp, rchex, rc, mindepth, maxdepth);

            if(intersectsHexahedronHexahedron(rchex, hexah))
            {
                tcams.push_back(rc);
            }
        }
    }
    return tcams;
}

} // namespace mvsUtils
} // namespace aliceVision
