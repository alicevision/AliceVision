// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Fuser.hpp"
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/mvsData/geometry.hpp>
#include <aliceVision/mvsData/Pixel.hpp>
#include <aliceVision/mvsData/Point2d.hpp>
#include <aliceVision/mvsData/Stat3d.hpp>
#include <aliceVision/mvsUtils/common.hpp>
#include <aliceVision/mvsUtils/fileIO.hpp>
#include <aliceVision/mvsData/imageIO.hpp>
#include <aliceVision/mvsData/imageAlgo.hpp>

#include <boost/filesystem.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>

#include <iostream>

namespace aliceVision {
namespace fuseCut {

namespace bfs = boost::filesystem;

unsigned long computeNumberOfAllPoints(const mvsUtils::MultiViewParams& mp, int scale)
{
    unsigned long npts = 0;

#pragma omp parallel for reduction(+:npts)
    for(int rc = 0; rc < mp.ncams; rc++)
    {
        const std::string filename = mvsUtils::getFileNameFromIndex(mp, rc, mvsUtils::EFileType::depthMap, scale);
        oiio::ParamValueList metadata;
        imageIO::readImageMetadata(filename, metadata);
        int nbDepthValues = metadata.get_int("AliceVision:nbDepthValues", -1);

        if(nbDepthValues < 0)
        {
            int width, height;
            StaticVector<float> depthMap;
            nbDepthValues = 0;

            ALICEVISION_LOG_WARNING("Can't find or invalid 'nbDepthValues' metadata in '" << filename << "'. Recompute the number of valid values.");

            imageIO::readImage(mvsUtils::getFileNameFromIndex(mp, rc, mvsUtils::EFileType::depthMap, scale), width, height, depthMap.getDataWritable(), imageIO::EImageColorSpace::NO_CONVERSION);
            // no need to transpose for this operation
            for(int i = 0; i < sizeOfStaticVector<float>(&depthMap); ++i)
                nbDepthValues += static_cast<unsigned long>(depthMap[i] > 0.0f);
        }

        npts += nbDepthValues;
    }
    return npts;
}

Fuser::Fuser(const mvsUtils::MultiViewParams& mp)
  : _mp(mp)
{}

Fuser::~Fuser()
{
}


/**
 * @brief 
 * 
 * @param[in] pixSizeFactor: pixSize tolerance factor
 * @param[in]
 * @param[in] p: 3d point back projected from tc camera
 * @param[in]
 * @param[in]
 * @param[out] numOfPtsMap
 * @param[in] depthMap
 * @param[in] simMap
 * @param[in] scale
 */
bool Fuser::updateInSurr(float pixToleranceFactor, int pixSizeBall, int pixSizeBallWSP, Point3d& p, int rc, int tc,
                           StaticVector<int>* numOfPtsMap, StaticVector<float>* depthMap, StaticVector<float>* simMap,
                           int scale)
{
    int w =_mp.getWidth(rc) / scale;
    int h =_mp.getHeight(rc) / scale;

    Pixel pix;
   _mp.getPixelFor3DPoint(&pix, p, rc);
    if(!_mp.isPixelInImage(pix, rc))
    {
        return false;
    }

    Pixel cell = pix;
    cell.x /= scale;
    cell.y /= scale;

    float pixDepth = (_mp.CArr[rc] - p).size();

    int d = pixSizeBall;

    float sim = (*simMap)[cell.y * w + cell.x];
    if(sim >= 1.0f)
    {
        d = pixSizeBallWSP;
    }

    // float pixSize = 2.0f*(float)std::max(d,1)*_mp.getCamPixelSize(p,cam);
    float pixSize = pixToleranceFactor *_mp.getCamPixelSizePlaneSweepAlpha(p, rc, tc, scale, 1);

    Pixel ncell;
    for(ncell.x = std::max(0, cell.x - d); ncell.x <= std::min(w - 1, cell.x + d); ncell.x++)
    {
        for(ncell.y = std::max(0, cell.y - d); ncell.y <= std::min(h - 1, cell.y + d); ncell.y++)
        {
            // printf("%i %i %i %i %i %i %i %i\n",ncell.x,ncell.y,w,h,w*h,depthMap->size(),cam,scale);
            float depth = (*depthMap)[ncell.y * w + ncell.x];
            // Point3d p1 = _mp.CArr[rc] +
            // (_mp.iCamArr[rc]*Point2d((float)ncell.x*(float)scale,(float)ncell.y*(float)scale)).normalize()*depth;
            // if ( (p1-p).size() < pixSize ) {
            if(fabs(pixDepth - depth) < pixSize)
            {
                (*numOfPtsMap)[ncell.y * w + ncell.x]++;
            }
        }
    }

    return true;
}

// minNumOfModals number of other cams including this cam ... minNumOfModals /in 2,3,...
void Fuser::filterGroups(const std::vector<int>& cams, float pixToleranceFactor, int pixSizeBall, int pixSizeBallWSP, int nNearestCams)
{
    ALICEVISION_LOG_INFO("Precomputing groups.");
    long t1 = clock();
#pragma omp parallel for
    for(int c = 0; c < cams.size(); c++)
    {
        int rc = cams[c];
        filterGroupsRC(rc, pixToleranceFactor, pixSizeBall, pixSizeBallWSP, nNearestCams);
    }

    mvsUtils::printfElapsedTime(t1);
}

// minNumOfModals number of other cams including this cam ... minNumOfModals /in 2,3,...
bool Fuser::filterGroupsRC(int rc, float pixToleranceFactor, int pixSizeBall, int pixSizeBallWSP, int nNearestCams)
{
    if(mvsUtils::FileExists(getFileNameFromIndex(_mp, rc, mvsUtils::EFileType::nmodMap)))
    {
        return true;
    }

    long t1 = clock();
    int w = _mp.getWidth(rc);
    int h = _mp.getHeight(rc);

    StaticVector<float> depthMap;
    StaticVector<float> simMap;

    {
        int width, height;

        imageIO::readImage(getFileNameFromIndex(_mp, rc, mvsUtils::EFileType::depthMap, 1), width, height, depthMap.getDataWritable(), imageIO::EImageColorSpace::NO_CONVERSION);
        imageIO::readImage(getFileNameFromIndex(_mp, rc, mvsUtils::EFileType::simMap, 1), width, height, simMap.getDataWritable(), imageIO::EImageColorSpace::NO_CONVERSION);
    }

    std::vector<unsigned char> numOfModalsMap(w * h, 0);

    if((depthMap.empty()) || (simMap.empty()) || (depthMap.size() != w * h) || (simMap.size() != w * h))
    {
        std::stringstream s;
        s << "filterGroupsRC: bad image dimension for camera: " << _mp.getViewId(rc) << "\n";
        s << "depthMap size: " << depthMap.size() << ", simMap size: " << simMap.size() << ", width: " << w << ", height: " << h;
       throw std::runtime_error(s.str());
    }

    StaticVector<int>* numOfPtsMap = new StaticVector<int>();
    numOfPtsMap->reserve(w * h);
    numOfPtsMap->resize_with(w * h, 0);

    StaticVector<int> tcams = _mp.findNearestCamsFromLandmarks(rc, nNearestCams);

    for(int c = 0; c < tcams.size(); c++)
    {
        numOfPtsMap->resize_with(w * h, 0);
        int tc = tcams[c];

        StaticVector<float> tcdepthMap;

        {
            int width, height;
            imageIO::readImage(getFileNameFromIndex(_mp, tc, mvsUtils::EFileType::depthMap, 1), width, height, tcdepthMap.getDataWritable(), imageIO::EImageColorSpace::NO_CONVERSION);
        }

        if(!tcdepthMap.empty())
        {
            for(int y = 0; y < h; ++y)
            {
                for(int x = 0; x < w; ++x)
                {
                    float depth = tcdepthMap[y * w + x];
                    if(depth > 0.0f)
                    {
                      Point3d p = _mp.CArr[tc] + (_mp.iCamArr[tc] * Point2d((float)x, (float)y)).normalize() * depth;
                      updateInSurr(pixToleranceFactor, pixSizeBall, pixSizeBallWSP, p, rc, tc, numOfPtsMap, &depthMap, &simMap, 1);
                    }
                }
            }

            for(int i = 0; i < w * h; i++)
            {
                numOfModalsMap.at(i) += static_cast<int>((*numOfPtsMap)[i] > 0);
            }
        }
    }

    {
        using namespace imageIO;
        writeImage(getFileNameFromIndex(_mp, rc, mvsUtils::EFileType::nmodMap), w, h, numOfModalsMap, EImageQuality::LOSSLESS, OutputFileColorSpace(EImageColorSpace::NO_CONVERSION));
    }

    delete numOfPtsMap;

    ALICEVISION_LOG_DEBUG(rc << " solved.");
    mvsUtils::printfElapsedTime(t1);

    return true;
}

// minNumOfModals number of other cams including this cam ... minNumOfModals /in 2,3,...
void Fuser::filterDepthMaps(const std::vector<int>& cams, int minNumOfModals, int minNumOfModalsWSP2SSP)
{
    ALICEVISION_LOG_INFO("Filtering depth maps.");
    long t1 = clock();

#pragma omp parallel for
    for(int c = 0; c < cams.size(); c++)
    {
        int rc = cams[c];
        filterDepthMapsRC(rc, minNumOfModals, minNumOfModalsWSP2SSP);
    }

    mvsUtils::printfElapsedTime(t1);
}

// minNumOfModals number of other cams including this cam ... minNumOfModals /in 2,3,...
bool Fuser::filterDepthMapsRC(int rc, int minNumOfModals, int minNumOfModalsWSP2SSP)
{
    long t1 = clock();
    int w = _mp.getWidth(rc);
    int h = _mp.getHeight(rc);

    std::vector<float> depthMap;
    std::vector<float> simMap;
    std::vector<unsigned char> numOfModalsMap;

    {
        int width, height;

        imageIO::readImage(getFileNameFromIndex(_mp, rc, mvsUtils::EFileType::depthMap, 1), width, height, depthMap, imageIO::EImageColorSpace::NO_CONVERSION);
        imageIO::readImage(getFileNameFromIndex(_mp, rc, mvsUtils::EFileType::simMap, 1), width, height, simMap, imageIO::EImageColorSpace::NO_CONVERSION);
        imageIO::readImage(getFileNameFromIndex(_mp, rc, mvsUtils::EFileType::nmodMap), width, height, numOfModalsMap, imageIO::EImageColorSpace::NO_CONVERSION);
    }

    int nbDepthValues = 0;

    for(int i = 0; i < w * h; i++)
    {
        // if the point is part of a mask (alpha) skip
        if(depthMap.at(i) <= -2.0f)
            continue;

        // if the reference point is consistent in three target cameras and is denoted as weakly supported point
        // make him strongly supported
        if((numOfModalsMap.at(i) >= minNumOfModalsWSP2SSP - 1) && (simMap.at(i) >= 1.0f))
        {
            simMap[i] = simMap[i] - 2.0f;
        }

        // if it is conistent in only one camera and is weakly supported then remove him
        // weakly supported point must be consisten in at least two cameras
        if((numOfModalsMap.at(i) <= 1) && (simMap.at(i) >= 1.0f))
        {
            depthMap[i] = -1.0f;
            simMap[i] = 1.0f;
        }

        // if it is not conistent in minimal number of cameras and is strongly supported then remove him
        if((numOfModalsMap.at(i) < minNumOfModals - 1) && (simMap.at(i) < 1.0f))
        {
            depthMap[i] = -1.0f;
            simMap[i] = 1.0f;
        }

        if(depthMap[i] > 0.0f)
          ++nbDepthValues;
    }

    oiio::ParamValueList metadata = imageIO::getMetadataFromMap(_mp.getMetadata(rc));
    metadata.push_back(oiio::ParamValue("AliceVision:nbDepthValues", oiio::TypeDesc::INT32, 1, &nbDepthValues));
    metadata.push_back(oiio::ParamValue("AliceVision:downscale", _mp.getDownscaleFactor(rc)));
    metadata.push_back(oiio::ParamValue("AliceVision:CArr", oiio::TypeDesc(oiio::TypeDesc::DOUBLE, oiio::TypeDesc::VEC3), 1, _mp.CArr[rc].m));
    metadata.push_back(oiio::ParamValue("AliceVision:iCamArr", oiio::TypeDesc(oiio::TypeDesc::DOUBLE, oiio::TypeDesc::MATRIX33), 1, _mp.iCamArr[rc].m));
    {
        float minDepth, maxDepth, midDepth;
        size_t nbDepths;
        _mp.getMinMaxMidNbDepth(rc, minDepth, maxDepth, midDepth, nbDepths);
        metadata.push_back(oiio::ParamValue("AliceVision:maxDepth", maxDepth));
        metadata.push_back(oiio::ParamValue("AliceVision:minDepth", minDepth));
    }
    {
      std::vector<double> matrixP = _mp.getOriginalP(rc);
      metadata.push_back(oiio::ParamValue("AliceVision:P", oiio::TypeDesc(oiio::TypeDesc::DOUBLE, oiio::TypeDesc::MATRIX44), 1, matrixP.data()));
    }

    using namespace imageIO;
    writeImage(getFileNameFromIndex(_mp, rc, mvsUtils::EFileType::depthMap, 0), w, h, depthMap, EImageQuality::LOSSLESS, OutputFileColorSpace(EImageColorSpace::NO_CONVERSION), metadata);
    writeImage(getFileNameFromIndex(_mp, rc, mvsUtils::EFileType::simMap, 0), w, h, simMap, EImageQuality::OPTIMIZED, OutputFileColorSpace(EImageColorSpace::NO_CONVERSION), metadata);

    ALICEVISION_LOG_DEBUG(rc << " solved.");
    mvsUtils::printfElapsedTime(t1);

    return true;
}

float Fuser::computeAveragePixelSizeInHexahedron(Point3d* hexah, int step, int scale)
{
    int scaleuse = std::max(1, scale);

    StaticVector<int> cams = _mp.findCamsWhichIntersectsHexahedron(hexah);
    int j = 0;
    float av = 0.0f;
    float nav = 0.0f;
    float minv = std::numeric_limits<float>::max();
    //long t1 = mvsUtils::initEstimate();
    for(int c = 0; c < cams.size(); c++)
    {
        int rc = cams[c];
        int h = _mp.getHeight(rc) / scaleuse;
        int w = _mp.getWidth(rc) / scaleuse;
        StaticVector<float> rcdepthMap;

        {
            int width, height;
            imageIO::readImage(getFileNameFromIndex(_mp, rc, mvsUtils::EFileType::depthMap, scale), width, height, rcdepthMap.getDataWritable(), imageIO::EImageColorSpace::NO_CONVERSION);
        }

        for(int y = 0; y < h; y++)
        {
            for(int x = 0; x < w; ++x)
            {
                float depth = rcdepthMap[y * w + x];
                if(depth > 0.0f)
                {
                    if(j % step == 0)
                    {
                        Point3d p = _mp.CArr[rc] +
                                    (_mp.iCamArr[rc] * Point2d((float)x * (float)scaleuse, (float)y * (float)scaleuse))
                                            .normalize() *
                                        depth;
                        if(mvsUtils::isPointInHexahedron(p, hexah))
                        {
                            float v = _mp.getCamPixelSize(p, rc);
                            av += v; // WARNING: the value may be too big for a float
                            nav += 1.0f;
                            minv = std::min(minv, v);
                        }
                    }
                    j++;
                }
            }
        }
        //mvsUtils::printfEstimate(c, cams.size(), t1);
    }
    //mvsUtils::finishEstimate();

    if(nav == 0.0f)
    {
        return -1.0f;
    }

    return av / nav;

    // return minv;
}



float Fuser::computeAveragePixelSizeInHexahedron(Point3d* hexah, const sfmData::SfMData& sfmData)
{
  float av = 0.0f;
  float nav = 0.0f;
  float minv = std::numeric_limits<float>::max();

  for(const auto& landmarkPair : sfmData.getLandmarks())
  {
    const sfmData::Landmark& landmark = landmarkPair.second;
    const Point3d p(landmark.X(0), landmark.X(1), landmark.X(2));

    for(const auto& observationPair : landmark.observations)
    {
      const IndexT viewId = observationPair.first;

      if(mvsUtils::isPointInHexahedron(p, hexah))
      {
        float v = _mp.getCamPixelSize(p, _mp.getIndexFromViewId(viewId));
        av += v; // WARNING: the value may be too big for a float
        nav += 1.0f;
        minv = std::min(minv, v);
      }
    }
  }

  if(nav == 0.0f)
    return -1.0f;

  return av / nav;
}

/**
 *@param[out] hexah: table of 8 values
 *@param[out] minPixSize
 */
void Fuser::divideSpaceFromDepthMaps(Point3d* hexah, float& minPixSize)
{
    ALICEVISION_LOG_INFO("Estimate space from depth maps.");
    int scale = 0;

    unsigned long npset = computeNumberOfAllPoints(_mp, scale);
    int stepPts = std::max(1, (int)(npset / (unsigned long)1000000));

    minPixSize = std::numeric_limits<float>::max();
    //long t1 = mvsUtils::initEstimate();
    Stat3d s3d = Stat3d();
    for(int rc = 0; rc < _mp.ncams; rc++)
    {
        int w = _mp.getWidth(rc);

        StaticVector<float> depthMap;
        {
            int width, height;

            imageIO::readImage(getFileNameFromIndex(_mp, rc, mvsUtils::EFileType::depthMap, scale), width, height, depthMap.getDataWritable(), imageIO::EImageColorSpace::NO_CONVERSION);
        }

        for(int i = 0; i < sizeOfStaticVector<float>(&depthMap); i += stepPts)
        {
            int x = i % w;
            int y = i / w;
            float depth = depthMap[i];
            if(depth > 0.0f)
            {
                Point3d p = _mp.CArr[rc] + (_mp.iCamArr[rc] * Point2d((float)x, (float)y)).normalize() * depth;
                float pixSize = _mp.getCamPixelSize(p, rc);
                minPixSize = std::min(minPixSize, pixSize);
                s3d.update(&p);
            }
        }
        //mvsUtils::printfEstimate(rc, _mp.ncams, t1);
    }
    //mvsUtils::finishEstimate();

    Point3d v1, v2, v3, cg;
    float d1, d2, d3;
    s3d.getEigenVectorsDesc(cg, v1, v2, v3, d1, d2, d3);

    using namespace boost::accumulators;
    using Accumulator = accumulator_set<float, stats<
            tag::tail_quantile<right>
            >>;
    const std::size_t cacheSize =  10000;
    Accumulator accX1( tag::tail<right>::cache_size = cacheSize );
    Accumulator accX2( tag::tail<right>::cache_size = cacheSize );
    Accumulator accY1( tag::tail<right>::cache_size = cacheSize );
    Accumulator accY2( tag::tail<right>::cache_size = cacheSize );
    Accumulator accZ1( tag::tail<right>::cache_size = cacheSize );
    Accumulator accZ2( tag::tail<right>::cache_size = cacheSize );

    for(int rc = 0; rc < _mp.ncams; ++rc)
    {
        int w = _mp.getWidth(rc);

        StaticVector<float> depthMap;
        {
            int width, height;

            imageIO::readImage(getFileNameFromIndex(_mp, rc, mvsUtils::EFileType::depthMap, scale), width, height, depthMap.getDataWritable(), imageIO::EImageColorSpace::NO_CONVERSION);
        }

        for(int i = 0; i < depthMap.size(); i += stepPts)
        {
            int x = i % w;
            int y = i / w;
            float depth = depthMap[i];
            if(depth > 0.0f)
            {
                Point3d p = _mp.CArr[rc] + (_mp.iCamArr[rc] * Point2d((float)x, (float)y)).normalize() * depth;
                float d1 = orientedPointPlaneDistance(p, cg, v1);
                float d2 = orientedPointPlaneDistance(p, cg, v2);
                float d3 = orientedPointPlaneDistance(p, cg, v3);

                if(d1 < 0)
                    accX1(fabs(d1));
                else
                    accX2(fabs(d1));

                if(d2 < 0)
                    accY1(fabs(d2));
                else
                    accY2(fabs(d2));

                if(d3 < 0)
                    accZ1(fabs(d3));
                else
                    accZ2(fabs(d3));
            }
        }
        //mvsUtils::printfEstimate(rc, _mp.ncams, t1);
    }
    //mvsUtils::finishEstimate();

    float perc = (float)_mp.userParams.get<double>("LargeScale.universePercentile", 0.999f);

    float mind1 = -quantile(accX1, quantile_probability = perc);
    float maxd1 = quantile(accX2, quantile_probability = perc);
    float mind2 = -quantile(accY1, quantile_probability = perc);
    float maxd2 = quantile(accY2, quantile_probability = perc);
    float mind3 = -quantile(accZ1, quantile_probability = perc);
    float maxd3 = quantile(accZ2, quantile_probability = perc);

//    std::cout << "quantile accumulators:" << std::endl;
//    std::cout << "X(" << mind1 << ", " << maxd1 << "), "
//              << "Y(" << mind2 << ", " << maxd2 << "), "
//              << "Z(" << mind3 << ", " << maxd3 << "), "
//              << std::endl;

    hexah[0] = cg + v1 * maxd1 + v2 * maxd2 + v3 * maxd3;
    hexah[1] = cg + v1 * mind1 + v2 * maxd2 + v3 * maxd3;
    hexah[2] = cg + v1 * mind1 + v2 * mind2 + v3 * maxd3;
    hexah[3] = cg + v1 * maxd1 + v2 * mind2 + v3 * maxd3;
    hexah[4] = cg + v1 * maxd1 + v2 * maxd2 + v3 * mind3;
    hexah[5] = cg + v1 * mind1 + v2 * maxd2 + v3 * mind3;
    hexah[6] = cg + v1 * mind1 + v2 * mind2 + v3 * mind3;
    hexah[7] = cg + v1 * maxd1 + v2 * mind2 + v3 * mind3;

    const double volume = mvsUtils::computeHexahedronVolume(hexah);

    if(std::isnan(volume) || volume < std::numeric_limits<double>::epsilon())
      throw std::runtime_error("Failed to estimate space from depth maps: The space bounding box is too small.");

    ALICEVISION_LOG_INFO("Estimate space done.");
}

bool checkLandmarkMinObservationAngle(const sfmData::SfMData& sfmData, const sfmData::Landmark& landmark, float minObservationAngle)
{
  for(const auto& observationPairI : landmark.observations)
  {
    const IndexT I = observationPairI.first;
    const sfmData::View& viewI = *(sfmData.getViews().at(I));
    const geometry::Pose3 poseI = sfmData.getPose(viewI).getTransform();
    const camera::IntrinsicBase* intrinsicPtrI = sfmData.getIntrinsicPtr(viewI.getIntrinsicId());

    for(const auto& observationPairJ : landmark.observations)
    {
      const IndexT J = observationPairJ.first;

      // cannot compare the current view with itself
      if(I == J)
        continue;

      const sfmData::View& viewJ = *(sfmData.getViews().at(J));
      const geometry::Pose3 poseJ = sfmData.getPose(viewJ).getTransform();
      const camera::IntrinsicBase* intrinsicPtrJ = sfmData.getIntrinsicPtr(viewJ.getIntrinsicId());

      const double angle = camera::angleBetweenRays(poseI, intrinsicPtrI, poseJ, intrinsicPtrJ, observationPairI.second.x, observationPairJ.second.x);

      // check angle between two observation
      if(angle < minObservationAngle)
        continue;

      return true;
    }
  }

  return false;
}

void Fuser::divideSpaceFromSfM(const sfmData::SfMData& sfmData, Point3d* hexah, std::size_t minObservations, float minObservationAngle) const
{
  ALICEVISION_LOG_INFO("Estimate space from SfM.");

  const std::size_t cacheSize =  10000;
  const double percentile = _mp.userParams.get<double>("LargeScale.universePercentile", 0.999);

  using namespace boost::accumulators;
  using AccumulatorMin = accumulator_set<double, stats<tag::tail_quantile<left>>>;
  using AccumulatorMax = accumulator_set<double, stats<tag::tail_quantile<right>>>;

  AccumulatorMin accMinX(tag::tail<left>::cache_size = cacheSize);
  AccumulatorMin accMinY(tag::tail<left>::cache_size = cacheSize);
  AccumulatorMin accMinZ(tag::tail<left>::cache_size = cacheSize);
  AccumulatorMax accMaxX(tag::tail<right>::cache_size = cacheSize);
  AccumulatorMax accMaxY(tag::tail<right>::cache_size = cacheSize);
  AccumulatorMax accMaxZ(tag::tail<right>::cache_size = cacheSize);

  for(const auto& landmarkPair : sfmData.getLandmarks())
  {
    const sfmData::Landmark& landmark = landmarkPair.second;

    // check number of observations
    if(landmark.observations.size() < minObservations)
      continue;

    // check angle between observations
    if(!checkLandmarkMinObservationAngle(sfmData, landmark, minObservationAngle))
      continue;

    const double x = landmark.X(0);
    const double y = landmark.X(1);
    const double z = landmark.X(2);

    accMinX(x);
    accMinY(y);
    accMinZ(z);
    accMaxX(x);
    accMaxY(y);
    accMaxZ(z);
  }

  // Remove a percentile of the observations (to remove unstable points)
  double xMin = quantile(accMinX, quantile_probability = 1.0 - percentile);
  double yMin = quantile(accMinY, quantile_probability = 1.0 - percentile);
  double zMin = quantile(accMinZ, quantile_probability = 1.0 - percentile);
  double xMax = quantile(accMaxX, quantile_probability = percentile);
  double yMax = quantile(accMaxY, quantile_probability = percentile);
  double zMax = quantile(accMaxZ, quantile_probability = percentile);

  // Add a margin on the result
  const double xMargin = (xMax - xMin) * 0.05;
  const double yMargin = (yMax - yMin) * 0.05;
  const double zMargin = (zMax - zMin) * 0.05;
  xMin -= xMargin;
  yMin -= yMargin;
  zMin -= zMargin;
  xMax += xMargin;
  yMax += yMargin;
  zMax += zMargin;

  hexah[0] = Point3d(xMax, yMax, zMax);
  hexah[1] = Point3d(xMin, yMax, zMax);
  hexah[2] = Point3d(xMin, yMin, zMax);
  hexah[3] = Point3d(xMax, yMin, zMax);
  hexah[4] = Point3d(xMax, yMax, zMin);
  hexah[5] = Point3d(xMin, yMax, zMin);
  hexah[6] = Point3d(xMin, yMin, zMin);
  hexah[7] = Point3d(xMax, yMin, zMin);

  const double volume = mvsUtils::computeHexahedronVolume(hexah);

  if(std::isnan(volume) || volume < std::numeric_limits<double>::epsilon())
    throw std::runtime_error("Failed to estimate space from SfM: The space bounding box is too small.");

  ALICEVISION_LOG_INFO("Estimate space done.");
}

Voxel Fuser::estimateDimensions(Point3d* vox, Point3d* newSpace, int scale, int maxOcTreeDim, const sfmData::SfMData* sfmData)
{
    const Point3d O = (vox[0] + vox[1] + vox[2] + vox[3] + vox[4] + vox[5] + vox[6] + vox[7]) / 8.0f;
    Point3d vx = vox[1] - vox[0];
    Point3d vy = vox[3] - vox[0];
    Point3d vz = vox[4] - vox[0];
    const float svx = vx.size();
    const float svy = vy.size();
    const float svz = vz.size();
    vx = vx.normalize();
    vy = vy.normalize();
    vz = vz.normalize();

    const float pointToJoinPixSizeDist = (float)_mp.userParams.get<double>("Fuser.pointToJoinPixSizeDist", 2.0f);
    ALICEVISION_LOG_INFO("pointToJoinPixSizeDist: " << pointToJoinPixSizeDist);

    float aAvPixelSize;

    if(sfmData == nullptr)
    {
      // WARNING perf: reload all depth maps to compute the minPixelSize (minPixelSize consider only points in the hexahedron)
      // Average 3D size for each pixel from all 3D points in the current voxel
      const int maxPts = 1000000;
      const int nAllPts = computeNumberOfAllPoints(_mp, scale);
      const int stepPts = nAllPts / maxPts + 1;
      aAvPixelSize = computeAveragePixelSizeInHexahedron(vox, stepPts, scale) * (float)std::max(scale, 1) * pointToJoinPixSizeDist;
    }
    else
    {
      aAvPixelSize = computeAveragePixelSizeInHexahedron(vox, *sfmData) * pointToJoinPixSizeDist;
    }

    Voxel maxDim;
    maxDim.x = (int)ceil(svx / (aAvPixelSize * (float)maxOcTreeDim));
    maxDim.y = (int)ceil(svy / (aAvPixelSize * (float)maxOcTreeDim));
    maxDim.z = (int)ceil(svz / (aAvPixelSize * (float)maxOcTreeDim));

    const Point3d vvx = vx * ((float)maxDim.x * ((aAvPixelSize * (float)maxOcTreeDim)));
    const Point3d vvy = vy * ((float)maxDim.y * ((aAvPixelSize * (float)maxOcTreeDim)));
    const Point3d vvz = vz * ((float)maxDim.z * ((aAvPixelSize * (float)maxOcTreeDim)));

    newSpace[0] = O - vvx / 2.0f - vvy / 2.0f - vvz / 2.0f;
    newSpace[1] = O + vvx - vvx / 2.0f - vvy / 2.0f - vvz / 2.0f;
    newSpace[2] = O + vvx + vvy - vvx / 2.0f - vvy / 2.0f - vvz / 2.0f;
    newSpace[3] = O + vvy - vvx / 2.0f - vvy / 2.0f - vvz / 2.0f;
    newSpace[4] = O + vvz - vvx / 2.0f - vvy / 2.0f - vvz / 2.0f;
    newSpace[5] = O + vvz + vvx - vvx / 2.0f - vvy / 2.0f - vvz / 2.0f;
    newSpace[6] = O + vvz + vvx + vvy - vvx / 2.0f - vvy / 2.0f - vvz / 2.0f;
    newSpace[7] = O + vvz + vvy - vvx / 2.0f - vvy / 2.0f - vvz / 2.0f;

    ALICEVISION_LOG_INFO("Estimated: " << mvsUtils::num2str(maxDim.x) << " " << mvsUtils::num2str(maxDim.y) << " " << mvsUtils::num2str(maxDim.z));

    ALICEVISION_LOG_DEBUG("optimal detail: "
            << static_cast<int>((vvx.size() / (float)maxDim.x) / aAvPixelSize) << " "
            << static_cast<int>((vvy.size() / (float)maxDim.y) / aAvPixelSize) << " "
            << static_cast<int>((vvz.size() / (float)maxDim.z) / aAvPixelSize));

    return maxDim;
}

std::string generateTempPtsSimsFiles(std::string tmpDir, mvsUtils::MultiViewParams& mp, bool addRandomNoise, float percNoisePts,
                                     int noisPixSizeDistHalfThr)
{
    ALICEVISION_LOG_INFO("generating temp files.");
    std::string depthMapsPtsSimsTmpDir = tmpDir + "depthMapsPtsSimsTmp/";

    if(!mvsUtils::FolderExists(depthMapsPtsSimsTmpDir))
    {
        bfs::create_directory(depthMapsPtsSimsTmpDir);

        int scale = 0;
        int scaleuse = std::max(1, scale);

        StaticVector<Point2d>* minMaxDepths = new StaticVector<Point2d>();
        minMaxDepths->reserve(mp.ncams);
        minMaxDepths->resize_with(mp.ncams, Point2d(-1.0, -1.0));

#pragma omp parallel for
        for(int rc = 0; rc < mp.ncams; rc++)
        {
            int w = mp.getWidth(rc) / scaleuse;
            int h = mp.getHeight(rc) / scaleuse;

            StaticVector<Point3d>* pts = new StaticVector<Point3d>();
            StaticVector<float>* sims = new StaticVector<float>();

            pts->reserve(w * h);
            sims->reserve(w * h);

            StaticVector<float> depthMap;
            StaticVector<float> simMap;

            {
                int width, height;

                imageIO::readImage(getFileNameFromIndex(mp, rc, mvsUtils::EFileType::depthMap, scale), width, height, depthMap.getDataWritable(), imageIO::EImageColorSpace::NO_CONVERSION);
                imageIO::readImage(getFileNameFromIndex(mp, rc, mvsUtils::EFileType::simMap, scale), width, height, simMap.getDataWritable(), imageIO::EImageColorSpace::NO_CONVERSION);
            }

            if(addRandomNoise)
            {
                StaticVector<int>* idsAlive = new StaticVector<int>();
                idsAlive->reserve(w * h);
                for(int i = 0; i < w * h; i++)
                {
                    if(depthMap[i] > 0.0f)
                    {
                        idsAlive->push_back(i);
                    }
                }

                int nnoisePts = ((percNoisePts / 100.0f) * (float)(idsAlive->size()));
                const std::vector<int> randIdsAlive = mvsUtils::createRandomArrayOfIntegers(idsAlive->size());

                srand(time(nullptr));

                long t1 = clock();
                for(int y = 0; y < h; ++y)
                {
                    for(int x = 0; x < w; ++x)
                    {
                        int id = y * w + x;
                        int i = (*idsAlive)[randIdsAlive[id]];
                        double depth = depthMap[i];

                        double sim = simMap[i];
                        if(depth > 0.0f)
                        {
                            Point3d p = mp.CArr[rc] +
                                        (mp.iCamArr[rc] * Point2d((double)x * (double)scaleuse, (double)y * (double)scaleuse))
                                                .normalize() *
                                            depth;

                            if(id < nnoisePts)
                            {
                                double pixSize = mp.getCamPixelSize(p, rc);
                                int rid = rand() % (2 * noisPixSizeDistHalfThr + 1);
                                rid = rid - noisPixSizeDistHalfThr;
                                double rdepthAdd = pixSize * (double)rid;
                                depth = depth + rdepthAdd;
                                p = mp.CArr[rc] +
                                    (mp.iCamArr[rc] * Point2d((double)x * (double)scaleuse, (double)y * (double)scaleuse))
                                            .normalize() *
                                        depth;
                            }

                            pts->push_back(p);
                            sims->push_back(sim);
                            if((*minMaxDepths)[rc].x < 0.0f)
                            {
                                (*minMaxDepths)[rc].x = depth;
                            }
                            else
                            {
                                (*minMaxDepths)[rc].x = std::min((*minMaxDepths)[rc].x, depth);
                            }
                            if((*minMaxDepths)[rc].y < 0.0f)
                            {
                                (*minMaxDepths)[rc].y = depth;
                            }
                            else
                            {
                                (*minMaxDepths)[rc].y = std::max((*minMaxDepths)[rc].y, depth);
                            }
                        }
                    }
                }
                mvsUtils::printfElapsedTime(t1);

                delete idsAlive;
            }
            else
            {

                long t1 = clock();
                for(int x = 0; x < w; x++)
                {
                    for(int y = 0; y < h; y++)
                    {
                        int i = x * h + y;
                        double depth = depthMap[i];
                        double sim = simMap[i];
                        if(depth > 0.0f)
                        {
                            Point3d p =
                                mp.CArr[rc] +
                                (mp.iCamArr[rc] * Point2d((double)x * (double)scaleuse, (double)y * (double)scaleuse))
                                        .normalize() *
                                    depth;
                            pts->push_back(p);
                            sims->push_back(sim);
                            if((*minMaxDepths)[rc].x < 0.0f)
                            {
                                (*minMaxDepths)[rc].x = depth;
                            }
                            else
                            {
                                (*minMaxDepths)[rc].x = std::min((*minMaxDepths)[rc].x, depth);
                            }
                            if((*minMaxDepths)[rc].y < 0.0f)
                            {
                                (*minMaxDepths)[rc].y = depth;
                            }
                            else
                            {
                                (*minMaxDepths)[rc].y = std::max((*minMaxDepths)[rc].y, depth);
                            }
                        }
                    }
                }
                mvsUtils::printfElapsedTime(t1);
            }

            saveArrayToFile<Point3d>(depthMapsPtsSimsTmpDir + std::to_string(mp.getViewId(rc)) + "pts.bin", pts);
            saveArrayToFile<float>(depthMapsPtsSimsTmpDir + std::to_string(mp.getViewId(rc)) + "sims.bin", sims);
            delete pts;
            delete sims;
        }

        saveArrayToFile<Point2d>(depthMapsPtsSimsTmpDir + "minMaxDepths.bin", minMaxDepths);
        delete minMaxDepths;
    }

    return depthMapsPtsSimsTmpDir;
}

void deleteTempPtsSimsFiles(mvsUtils::MultiViewParams& mp, std::string depthMapsPtsSimsTmpDir)
{
    for(int rc = 0; rc < mp.ncams; rc++)
    {
        std::string ptsfn = depthMapsPtsSimsTmpDir + std::to_string(mp.getViewId(rc)) + "pts.bin";
        std::string simsfn = depthMapsPtsSimsTmpDir + std::to_string(mp.getViewId(rc)) + "sims.bin";
        remove(ptsfn.c_str());
        remove(simsfn.c_str());
    }
    mvsUtils::DeleteDirectory(depthMapsPtsSimsTmpDir);
}

} // namespace fuseCut
} // namespace aliceVision
