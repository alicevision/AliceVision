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

PreMatchCams::PreMatchCams(const MultiViewParams& mp, const std::string& camsPairsMatrixFolder)
  : _mp(mp)
  , _camsPairsMatrixFolder(camsPairsMatrixFolder)
{
    minang = static_cast<float>(_mp.userParams.get<double>("prematching.minAngle", 2.0));
    maxang = static_cast<float>(_mp.userParams.get<double>("prematching.maxAngle", 70.0)); // WARNING: may be too low, especially when using seeds from SFM
    minCamsDistance = computeMinCamsDistance();
}

float PreMatchCams::computeMinCamsDistance()
{
    int nd = 0;
    float d = 0.0;
    for(int rc = 0; rc < _mp.getNbCameras(); rc++)
    {
        Point3d rC = _mp.CArr[rc];
        for(int tc = 0; tc < _mp.getNbCameras(); tc++)
        {
            if(rc != tc)
            {
                Point3d tC = _mp.CArr[tc];
                d += (rC - tC).size();
                nd++;
            }
        }
    }

    return (d / static_cast<float>(nd)) / 100.0f;
}

bool PreMatchCams::overlap(int rc, int tc)
{
    if(!checkCamPairAngle(rc, tc, &_mp, 0.0f, 45.0f))
    {
        return false;
    }

    Point2d rmid = Point2d(static_cast<float>(_mp.getWidth(rc)) / 2.0f,
                           static_cast<float>(_mp.getHeight(rc)) / 2.0f);

    Point2d pFromTar, pToTar;

    if(!getTarEpipolarDirectedLine(&pFromTar, &pToTar, rmid, rc, tc, &_mp))
    {
        return false;
    }

    /*
    if (getTarEpipolarDirectedLine(
                    &pFromTar, &pToTar,
                    rmid,
                    tc, rc, mp
                    )==false)
    {
            return false;
    };
    */
    return true;
}

StaticVector<int> PreMatchCams::findNearestCams(int rc, int nbNearestCams)
{
    StaticVector<int> out;
    out.reserve(nbNearestCams);
    StaticVector<SortedId>* ids = new StaticVector<SortedId>();
    ids->reserve(_mp.getNbCameras() - 1);

    for(int c = 0; c < _mp.getNbCameras(); ++c)
    {
        if(c != rc)
        {
            ids->push_back(SortedId(c, (_mp.CArr.at(rc) - _mp.CArr.at(c)).size()));
            // printf("(%i %f) \n", (*ids)[ids->size()-1].id, (*ids)[ids->size()-1].value);
        }
    }

    qsort(&(*ids)[0], ids->size(), sizeof(SortedId), qsortCompareSortedIdAsc);

    /*
    for (int c=0;c<ids->size();c++)
    {
            printf("(%i %f) \n", (*ids)[c].id, (*ids)[c].value);
    };
    */

    {
        int c = 0;
        Point3d rC = _mp.CArr.at(rc);

        while((out.size() < nbNearestCams) && (c < ids->size()))
        {
            int tc = (*ids)[c].id;
            Point3d tC = _mp.CArr[tc];
            float d = (rC - tC).size();

            if((rc != tc) && (d > minCamsDistance) && (overlap(rc, tc)))
            {
                out.push_back(tc);
            }
            c++;
        }
    }
    delete ids;
    return out;
}

StaticVector<int>* PreMatchCams::precomputeIncidentMatrixCamsFromSeeds()
{
  std::string fn = _camsPairsMatrixFolder + "/camsPairsMatrixFromSeeds.bin";
  if(FileExists(fn))
  {
    ALICEVISION_LOG_INFO("Camera pairs matrix file already computed: " << fn);
    return loadArrayFromFile<int>(fn);
  }

  ALICEVISION_LOG_INFO("Compute camera pairs matrix file: " << fn);
  StaticVector<int>* camsmatrix = new StaticVector<int>();
  camsmatrix->reserve(_mp.getNbCameras() * _mp.getNbCameras());
  camsmatrix->resize_with(_mp.getNbCameras() * _mp.getNbCameras(), 0);

  for(int rc = 0; rc < _mp.getNbCameras(); ++rc)
  {
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
          (*camsmatrix)[std::min(rc, tc) *_mp.getNbCameras() + std::max(rc, tc)]++;
        }
      }
    }
  }

  saveArrayToFile<int>(fn, camsmatrix);
  return camsmatrix;
}

StaticVector<int>* PreMatchCams::loadCamPairsMatrix()
{
    std::string fn = _camsPairsMatrixFolder + "/camsPairsMatrixFromSeeds.bin";
    if(!FileExists(fn))
        throw std::runtime_error("Missing camera pairs matrix file: " + fn);
    return loadArrayFromFile<int>(fn);
}


StaticVector<int> PreMatchCams::findNearestCamsFromSeeds(int rc, int nnearestcams)
{
    StaticVector<int> out;
    std::string tarCamsFile = _camsPairsMatrixFolder + "_tarCams/" + num2strFourDecimal(rc) + ".txt";
    if(FileExists(tarCamsFile))
    {
        FILE* f = fopen(tarCamsFile.c_str(), "r");
        int ntcams;
        fscanf(f, "ntcams %i, tcams", &ntcams);
        out.reserve(ntcams);
        for(int c = 0; c < ntcams; c++)
        {
            int tc;
            fscanf(f, " %i", &tc);
            out.push_back(tc);
        }
        fclose(f);
    }
    else
    {
        StaticVector<int>* camsmatrix = loadCamPairsMatrix();
        StaticVector<SortedId> ids;
        ids.reserve(_mp.getNbCameras());
        for(int tc = 0; tc < _mp.getNbCameras(); tc++)
            ids.push_back(SortedId(tc, (float)(*camsmatrix)[std::min(rc, tc) * _mp.getNbCameras() + std::max(rc, tc)]));

        qsort(&ids[0], ids.size(), sizeof(SortedId), qsortCompareSortedIdDesc);
        
        // Ensure the ideal number of target cameras is not superior to the actual number of cameras
        const int maxNumTC = std::min(_mp.getNbCameras(), nnearestcams);
        out.reserve(maxNumTC);

        for(int i = 0; i < maxNumTC; ++i)
        {
            // a minimum of 10 common points is required (10*2 because points are stored in both rc/tc combinations)
            if(ids[i].value > (10 * 2)) 
                out.push_back(ids[i].id);
        }

        delete camsmatrix;
        
        if(out.size() < nnearestcams)
            ALICEVISION_LOG_WARNING("rc: " << rc << " - found only " << out.size() << "/" << nnearestcams << " tc by seeds" );
    }
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
