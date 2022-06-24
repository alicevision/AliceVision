// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "SgmDepthList.hpp"

#include <aliceVision/alicevision_omp.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/mvsData/OrientedPoint.hpp>
#include <aliceVision/mvsData/Point3d.hpp>
#include <aliceVision/mvsData/geometry.hpp>
#include <aliceVision/mvsUtils/common.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/depthMap/SgmParams.hpp>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>

namespace aliceVision {
namespace depthMap {

SgmDepthList::SgmDepthList(int rc, const std::vector<int>& tCams, const mvsUtils::MultiViewParams& mp, const SgmParams& sgmParams, const ROI& roi)
    : _rc(rc)
    , _roi(roi)
    , _mp(mp)
    , _sgmParams(sgmParams)
{
    if(sgmParams.chooseTCamsPerTile)
    {
      // find nearest T cameras per tile
      _tCams = mp.findTileNearestCams(rc, sgmParams.maxTCamsPerTile, tCams, roi);
    }
    else
    {
      // use previously selected T cameras from the entire image
      _tCams = tCams;
    }
}

void SgmDepthList::computeListRc()
{
    ALICEVISION_LOG_DEBUG("Compute SGM depths list (rc: " << _rc << ")");

    // compute min/max/mid/nb depth from SfM
    std::size_t nbObsDepths;
    float minObsDepth, maxObsDepth, midObsDepth;
    getMinMaxMidNbDepthFromSfM(minObsDepth, maxObsDepth, midObsDepth, nbObsDepths);

    StaticVector<StaticVector<float>*>* alldepths;

    // all depths from the principal ray provided by target cameras
    if(nbObsDepths < 20)
        alldepths = computeAllDepthsAndResetTcs(-1);
    else
        alldepths = computeAllDepthsAndResetTcs(midObsDepth);

    float minDepthAll = std::numeric_limits<float>::max();
    float maxDepthAll = 0.0f;
    for(int i = 0; i < alldepths->size(); i++)
    {
        for(int j = 0; j < (*alldepths)[i]->size(); j++)
        {
            float depth = (*(*alldepths)[i])[j];
            minDepthAll = std::min(minDepthAll, depth);
            maxDepthAll = std::max(maxDepthAll, depth);
        }
    }

    // TODO: handle the case where no depths are found
    if(minDepthAll == std::numeric_limits<float>::max())
    {
        _depths.clear();
        _depthsTcLimits.clear();
        return;
    }

    {
        ALICEVISION_LOG_DEBUG("Depth candidates from seeds for R camera:" << std::endl
                              << "\t- rc: " << _rc << std::endl
                              << "\t- nb observations: " << nbObsDepths <<  std::endl
                              << "\t- all depth range: [" << minDepthAll << "-" << maxDepthAll << "]");

        float minDepth = minDepthAll;
        float maxDepth = maxDepthAll;

        // if we want to use SfM seeds anf if we get enough information from these seeds, adjust min/maxDepth
        if(_sgmParams.useSfmSeeds && !_mp.getInputSfMData().getLandmarks().empty() && nbObsDepths > 100)
        {
            minDepth = minObsDepth * (1.0f - _sgmParams.seedsRangeInflate);
            maxDepth = maxObsDepth * (1.0f + _sgmParams.seedsRangeInflate);

            if(maxDepthAll < minDepth || minDepthAll > maxDepth)
            {
                // no intersection between min/maxDepth and min/maxDepthAll
                // keep min/maxDepth value as is
            }
            else
            {
                // min/maxDepth intersection with min/maxDepthAll
                minDepth = std::max(minDepthAll, minDepth);
                maxDepth = std::min(maxDepthAll, maxDepth);
            }
        }

        // build the list of "best" depths for rc, from all tc cameras depths
        computeDepths(minDepth, maxDepth, (_sgmParams.stepZ > 0.0f ? _sgmParams.stepZ : 1.0f), alldepths);

        // filter out depths if computeDepths gave too many values
        if(_sgmParams.maxDepths > 0 && _depths.size() > _sgmParams.maxDepths)
        {
            const float scaleFactor = float(_depths.size()) / float(_sgmParams.maxDepths);

            ALICEVISION_LOG_DEBUG("Too many values in R camera depth list, filter out with scale factor:" << std::endl
                                  << "\t- rc: " << _rc  << std::endl
                                  << "\t- nb depths: " << _depths.size() << std::endl
                                  << "\t- max depths: " << _sgmParams.maxDepths << std::endl
                                  << "\t- scale factor to apply: " << scaleFactor);

            computeDepths(minDepth, maxDepth, scaleFactor, alldepths);
        }

        if(_sgmParams.saveDepthsToSweepTxtFile)
        {
            const std::string fn = _mp.getDepthMapsFolder() + std::to_string(_mp.getViewId(_rc)) + "depthsAll.txt";
            FILE* f = fopen(fn.c_str(), "w");
            for(int j = 0; j < _depths.size(); j++)
            {
                fprintf(f, "%f\n", _depths[j]);
            }
            fclose(f);
        }

        ALICEVISION_LOG_DEBUG("Final depth range for R camera:" << std::endl
                              << "\t- rc: " << _rc << std::endl
                              << "\t- nb selected depths: " << _depths.size() << std::endl
                              << "\t- selected depth range: [" << minDepth << "-"<< maxDepth << "]");
    }

    // update depth tc limits
    _depthsTcLimits.resize(_tCams.size());

    // fill depthsTcamsLimits member variable with index range of depths to sweep
    for(int c = 0; c < _tCams.size(); c++)
    {
        const float d1 = (*(*alldepths)[c])[0];
        const float d2 = (*(*alldepths)[c])[(*alldepths)[c]->size() - 1];

        int id1 = _depths.indexOfNearestSorted(d1);
        int id2 = _depths.indexOfNearestSorted(d2);

        if(id1 == -1)
            id1 = 0;

        if(id2 == -1)
            id2 = _depths.size() - 1;

        // clamp to keep only the closest depths if we have too much inputs (> _sgmParams.maxDepthsPerTc)
        id2 = std::min(id1 + _sgmParams.maxDepthsPerTc - 1, id2);
        _depthsTcLimits[c] = Pixel(id1, id2 - id1 + 1);
    }

    if(_sgmParams.saveDepthsToSweepTxtFile)
    {
        // export depthsTcLimits txt file
        {
            const std::string fn = _mp.getDepthMapsFolder() + std::to_string(_mp.getViewId(_rc)) + "depthsTcLimits.txt";
            FILE* f = fopen(fn.c_str(), "w");
            for(int j = 0; j < _depthsTcLimits.size(); j++)
            {
                Pixel l = _depthsTcLimits[j];
                // fprintf(f,"%f %f\n",(*depths)[l.x],(*depths)[l.x+l.y-1]);
                fprintf(f, "%i %i\n", l.x, l.y);
            }
            fclose(f);
        }

        // export rc depth txt file
        {
            const std::string fn = _mp.getDepthMapsFolder() + std::to_string(_mp.getViewId(_rc)) + "depths.txt";
            FILE* f = fopen(fn.c_str(), "w");
            for(int j = 0; j < _depths.size(); j++)
            {
                fprintf(f, "%f\n", _depths[j]);
            }
            fclose(f);
        }

        // export all depths per tc txt files
        {
            for(int i = 0; i < alldepths->size(); i++)
            {
                const std::string fn = _mp.getDepthMapsFolder() + std::to_string(_mp.getViewId(_rc)) + "depths" +
                                       mvsUtils::num2str(i) + ".txt";
                FILE* f = fopen(fn.c_str(), "w");
                for(int j = 0; j < (*alldepths)[i]->size(); j++)
                {
                    const float depth = (*(*alldepths)[i])[j];
                    fprintf(f, "%f\n", depth);
                }
                fclose(f);
            }
        }

        // export rc depth txt file
        {
            OrientedPoint rcplane;
            rcplane.p = _mp.CArr[_rc];
            rcplane.n = _mp.iRArr[_rc] * Point3d(0.0, 0.0, 1.0);
            rcplane.n = rcplane.n.normalize();

            const std::string fn = _mp.getDepthMapsFolder() + std::to_string(_mp.getViewId(_rc)) + "rcDepths.txt";
            FILE* f = fopen(fn.c_str(), "w");
            float depth = minDepthAll;
            while(depth < maxDepthAll)
            {
                fprintf(f, "%f\n", depth);
                const Point3d p = rcplane.p + rcplane.n * depth;
                depth = depth + _mp.getCamPixelSize(p, _rc);
            }
            fclose(f);
        }
    }

    deleteArrayOfArrays<float>(&alldepths);
}

void SgmDepthList::logRcTcDepthInformation() const
{
    std::ostringstream ostr;
    ostr << "Camera / Depth information: " << std::endl
         << "\t- R camera:" << std::endl  
         << "\t   - id: " << _rc << std::endl
         << "\t   - view id: " << _mp.getViewId(_rc) << std::endl
         << "\t   - depth planes: " << _depths.size() << std::endl
         << "\t   - depths range: [" << _depths[0] << "-" << _depths[_depths.size() - 1] << "]" << std::endl 
         << "\t- T cameras:" << std::endl;

    for(int c = 0; c < _tCams.size(); c++)
    {
        ostr << "\t   - T camera (" << (c+1) << "/" << _tCams.size() << "):" << std::endl
             << "\t      - id: " << _tCams[c] << std::endl
             << "\t      - view id: " << _mp.getViewId(_tCams[c]) << std::endl
             << "\t      - depth planes: " << _depthsTcLimits[c].y << std::endl
             << "\t      - depths range: [" << _depths[_depthsTcLimits[c].x] << "-" << _depths[_depthsTcLimits[c].x + _depthsTcLimits[c].y - 1] << "]" << std::endl
             << "\t      - depth indexes range: [" << _depthsTcLimits[c].x << "-" << _depthsTcLimits[c].x + _depthsTcLimits[c].y << "]" << std::endl;
    }

    ALICEVISION_LOG_INFO(ostr.str());
}

void SgmDepthList::checkStartingAndStoppingDepth() const
{
    struct MinOffX
    {
        bool operator()(const Pixel& l, const Pixel& r) const { return (l.x < r.x); }
    };

    struct MinOffXplusY
    {
        bool operator()(const Pixel& l, const Pixel& r) const { return (l.x + l.y < r.x + r.y); }
    };

    const std::vector<Pixel>& depthTcamsLimitsVec = _depthsTcLimits.getData();
    const int startingDepth = std::min_element(depthTcamsLimitsVec.begin(), depthTcamsLimitsVec.end(), MinOffX())->x;
    const auto depth_it = std::max_element(depthTcamsLimitsVec.begin(), depthTcamsLimitsVec.end(), MinOffXplusY());
    const int stoppingDepth = depth_it->x + depth_it->y;

    // The overall starting depth index should always be zero.
    assert(startingDepth == 0);

    // Usually stoppingDepth should be equal to the total number of depths.
    // But due to sgmMaxDepths and sgmMaxDepthPerTc, we can have more depths
    // than we finally use in all TC cameras.
    assert(_depths.size() >= stoppingDepth);
}

void SgmDepthList::getMinMaxMidNbDepthFromSfM(float& min, float& max, float& mid, std::size_t& nbDepths) const
{
    using namespace boost::accumulators;

    const std::size_t cacheSize = 1000;
    accumulator_set<float, stats<tag::tail_quantile<left>>> accDistanceMin(tag::tail<left>::cache_size = cacheSize);
    accumulator_set<float, stats<tag::tail_quantile<right>>> accDistanceMax(tag::tail<right>::cache_size = cacheSize);

    const IndexT viewId = _mp.getViewId(_rc);
    const ROI fullsizeRoi = upscaleROI(_roi, _mp.getProcessDownscale()); // landmark observations are in the full-size image coordinate system

    OrientedPoint cameraPlane;
    cameraPlane.p = _mp.CArr[_rc];
    cameraPlane.n = _mp.iRArr[_rc] * Point3d(0.0, 0.0, 1.0);
    cameraPlane.n = cameraPlane.n.normalize();

    Point3d midDepthPoint = Point3d();
    nbDepths = 0;

    // for each landmark
    for(const auto& landmarkPair : _mp.getInputSfMData().getLandmarks())
    {
        const sfmData::Landmark& landmark = landmarkPair.second;
        const Point3d point(landmark.X(0), landmark.X(1), landmark.X(2));

        // for each landmark observation
        for(const auto& observationPair : landmark.observations)
        {
            // is rc observation
            if(observationPair.first == viewId)
            {
                const Vec2& obs2d = observationPair.second.x;

                // if we compute depth list per tile keep only observation located inside the image full-size ROI
                if(!_sgmParams.chooseDepthListPerTile || fullsizeRoi.contains(obs2d.x(), obs2d.y()))
                {
                    const float distance = static_cast<float>(pointPlaneDistance(point, cameraPlane.p, cameraPlane.n));
                    accDistanceMin(distance);
                    accDistanceMax(distance);
                    midDepthPoint = midDepthPoint + point;
                    ++nbDepths;
                }
            }
        }
    }

    if(nbDepths > 0)
    {
      min = quantile(accDistanceMin, quantile_probability = 1.0 - _sgmParams.seedsRangePercentile);
      max = quantile(accDistanceMax, quantile_probability = _sgmParams.seedsRangePercentile);
      midDepthPoint = midDepthPoint / static_cast<float>(nbDepths);
      mid = pointPlaneDistance(midDepthPoint, cameraPlane.p, cameraPlane.n);
    }

    ALICEVISION_LOG_DEBUG("Compute min/max/mid/nb observation depth from SfM for R camera:" << std::endl
                          << "\t- rc: " << _rc << std::endl
                          << "\t- view id: " << viewId << std::endl
                          << "\t- min depth: " << min << std::endl
                          << "\t- max depth: " << max << std::endl
                          << "\t- mid depth: " << mid << std::endl
                          << "\t- nb depth: " << nbDepths << std::endl
                          << "\t- percentile: " << _sgmParams.seedsRangePercentile);
}


StaticVector<StaticVector<float>*>* SgmDepthList::computeAllDepthsAndResetTcs(float midDepth)
{
    std::vector<int> tCamsNew;
    StaticVector<StaticVector<float>*>* alldepths = new StaticVector<StaticVector<float>*>();
    alldepths->reserve(_tCams.size());

    for(int c = 0; c < _tCams.size(); c++)
    {
        // depths of all meaningful points on the principal ray of the reference camera regarding the target camera tc
        StaticVector<float>* tcdepths = getDepthsTc(_tCams[c], midDepth);
        if(sizeOfStaticVector<float>(tcdepths) < 50)
        {
            // fallback if we don't have enough valid samples over the epipolar line
            if(tcdepths != nullptr)
            {
                delete tcdepths;
                tcdepths = nullptr;
            }
            float avMinDist, avMidDist, avMaxDist;
            getPreMatchingMinMaxDepths(avMinDist, avMidDist, avMaxDist);
            tcdepths = getDepthsByPixelSize(avMinDist, avMidDist, avMaxDist);

            if(sizeOfStaticVector<float>(tcdepths) < 50)
            {
                if(tcdepths != nullptr)
                {
                    delete tcdepths;
                    tcdepths = nullptr;
                }
            }
        }

        if(tcdepths != nullptr)
        {
            alldepths->push_back(tcdepths);
            tCamsNew.push_back(_tCams[c]);
        }
    }

    _tCams = tCamsNew;

    return alldepths;
}

void SgmDepthList::computeDepths(float minDepth, float maxDepth, float scaleFactor, const StaticVector<StaticVector<float>*>* alldepths)
{
    _depths.clear();

    float depth = minDepth;

    while(depth < maxDepth)
    {
        _depths.push_back(depth);

        // get min tc step at depth
        float minTcStep = maxDepth - minDepth;

        // for each tc camera
        for(int i = 0; i < alldepths->size(); i++)
        {
            // list of valid depths for the tc
            StaticVector<float>* tcDepths = (*alldepths)[i];

            // get the tc depth closest to the current depth
            const int id = tcDepths->indexOfNearestSorted(depth);

            // continue on no result or last element (we need id + 1)
            if(id < 0 || id >= tcDepths->size() - 1)
                continue;

            // consider the enclosing depth range
            const float did = (*tcDepths)[id];     // closest depth
            const float nid = (*tcDepths)[id + 1]; // next depth
            const float tcStep = fabs(did - nid);  // [closest; next] depths distance

            // keep this value if smallest step so far
            minTcStep = std::min(minTcStep, tcStep);
        }

        depth += minTcStep * scaleFactor;
    }
}

void SgmDepthList::getPreMatchingMinMaxDepths(float& minDepth, float& midDepth, float& maxDepth)
{
    if(_sgmParams.prematchinMinMaxDepthDontUseSeeds)
    {
        minDepth = 0.0f;
        maxDepth = 0.0f;
        for(int c = 0; c < _tCams.size(); ++c)
        {
            const int tc = _tCams[c];
            minDepth += (_mp.CArr[_rc] - _mp.CArr[tc]).size() * _sgmParams.prematchingMinCamDist;
            maxDepth += (_mp.CArr[_rc] - _mp.CArr[tc]).size() * _sgmParams.prematchingMaxCamDist;
        }
        minDepth /= static_cast<float>(_tCams.size());
        maxDepth /= static_cast<float>(_tCams.size());
        midDepth = (minDepth + maxDepth) / 2.0f;
    }
    else
    {
        std::size_t nbDepths;
        getMinMaxMidNbDepthFromSfM(minDepth, maxDepth, midDepth, nbDepths);
        maxDepth = maxDepth * _sgmParams.prematchingMaxDepthScale;
    }
}

StaticVector<float>* SgmDepthList::getDepthsByPixelSize(float minDepth, float midDepth, float maxDepth)
{
    const int maxDepthsHalf = 1024;

    const float d = float(_sgmParams.scale) * float(_sgmParams.rcDepthsCompStep);

    OrientedPoint rcplane;
    rcplane.p = _mp.CArr[_rc];
    rcplane.n = _mp.iRArr[_rc] * Point3d(0.0, 0.0, 1.0);
    rcplane.n = rcplane.n.normalize();

    int ndepthsMidMax = 0;
    float maxdepth = midDepth;
    while((maxdepth < maxDepth) && (ndepthsMidMax < maxDepthsHalf))
    {
        Point3d p = rcplane.p + rcplane.n * maxdepth;
        float pixSize = _mp.getCamPixelSize(p, _rc, d);
        maxdepth += pixSize;
        ndepthsMidMax++;
    }

    int ndepthsMidMin = 0;
    float mindepth = midDepth;
    while((mindepth > minDepth) && (ndepthsMidMin < maxDepthsHalf * 2 - ndepthsMidMax))
    {
        Point3d p = rcplane.p + rcplane.n * mindepth;
        float pixSize = _mp.getCamPixelSize(p, _rc, d);
        mindepth -= pixSize;
        ndepthsMidMin++;
    }

    // getNumberOfDepths
    float depth = mindepth;
    int ndepths = 0;
    float pixSize = 1.0f;
    while((depth < maxdepth) && (pixSize > 0.0f) && (ndepths < 2 * maxDepthsHalf))
    {
        Point3d p = rcplane.p + rcplane.n * depth;
        pixSize = _mp.getCamPixelSize(p, _rc, d);
        depth += pixSize;
        ndepths++;
    }

    StaticVector<float>* out = new StaticVector<float>();
    out->reserve(ndepths);

    // fill
    depth = mindepth;
    pixSize = 1.0f;
    ndepths = 0;
    while((depth < maxdepth) && (pixSize > 0.0f) && (ndepths < 2 * maxDepthsHalf))
    {
        out->push_back(depth);
        Point3d p = rcplane.p + rcplane.n * depth;
        pixSize = _mp.getCamPixelSize(p, _rc, d);
        depth += pixSize;
        ndepths++;
    }

    // check if it is asc
    for(int i = 0; i < out->size() - 1; i++)
    {
        if((*out)[i] >= (*out)[i + 1])
        {
            for(int j = 0; j <= i + 1; j++)
            {
                ALICEVISION_LOG_TRACE("getDepthsByPixelSize: check if it is asc: " << (*out)[j]);
            }
            throw std::runtime_error("getDepthsByPixelSize not asc.");
        }
    }
    return out;
}

StaticVector<float>* SgmDepthList::getDepthsTc(int tc, float midDepth)
{
    OrientedPoint rcplane;
    rcplane.p = _mp.CArr[_rc];
    rcplane.n = _mp.iRArr[_rc] * Point3d(0.0, 0.0, 1.0);
    rcplane.n = rcplane.n.normalize();

    // ROI center 
    const ROI sgmRoi = downscaleROI(_roi, _sgmParams.scale * _sgmParams.stepXY);
    const Point2d sgmRoiCenter((sgmRoi.x.begin + (sgmRoi.width() * 0.5)), sgmRoi.y.begin + (sgmRoi.height() * 0.5));

    // principal point of the rc camera to the tc camera
    const Point2d principalPoint(_mp.getWidth(_rc) * 0.5f, _mp.getHeight(_rc) * 0.5f);
    
    // reference pixel for the epipolar line
    const Point2d rmid = (!_sgmParams.chooseDepthListPerTile) ? principalPoint : sgmRoiCenter;

    // segment of epipolar line
    Point2d pFromTar, pToTar; 

    getTarEpipolarDirectedLine(&pFromTar, &pToTar, rmid, _rc, tc, _mp);

    int allDepths = static_cast<int>((pToTar - pFromTar).size());
    const int allDepthsFound = allDepths; // for debug log

    const Point2d pixelVect = ((pToTar - pFromTar).normalize()) * std::max(1.0f, (float)_sgmParams.scale);

    Point2d cg = Point2d(0.0f, 0.0f);
    Point3d cg3 = Point3d(0.0f, 0.0f, 0.0f);
    int ncg = 0;

    // navigate through all pixels of the epilolar segment
    // Compute the middle of the valid pixels of the epipolar segment (in rc camera) of the principal point (of the rc
    // camera)
    for(int i = 0; i < allDepths; i++)
    {
        Point2d tpix = pFromTar + pixelVect * (float)i;
        Point3d p;
        if(triangulateMatch(p, rmid, tpix, _rc, tc, _mp)) // triangulate principal point from rc with tpix
        {
            float depth = orientedPointPlaneDistance(
                p, rcplane.p,
                rcplane.n); // todo: can compute the distance to the camera (as it's the principal point it's the same)
            if(_mp.isPixelInImage(tpix, tc) && (depth > 0.0f) &&
               checkPair(p, _rc, tc, _mp, _mp.getMinViewAngle(), _mp.getMaxViewAngle()))
            {
                cg = cg + tpix;
                cg3 = cg3 + p;
                ncg++;
            }
        }
    }
    if(ncg == 0)
    {
        return new StaticVector<float>();
    }
    cg = cg / (float)ncg;
    cg3 = cg3 / (float)ncg;
    allDepths = ncg;

    Point2d midpoint = cg;
    if(midDepth > 0.0f)
    {
        Point3d midPt = rcplane.p + rcplane.n * midDepth;
        _mp.getPixelFor3DPoint(&midpoint, midPt, tc);
    }

    // compute the direction
    float direction = 1.0f;
    {
        Point3d p;
        if(!triangulateMatch(p, rmid, midpoint, _rc, tc, _mp))
        {
            StaticVector<float>* out = new StaticVector<float>();
            return out;
        }

        float depth = orientedPointPlaneDistance(p, rcplane.p, rcplane.n);

        if(!triangulateMatch(p, rmid, midpoint + pixelVect, _rc, tc, _mp))
        {
            StaticVector<float>* out = new StaticVector<float>();
            return out;
        }

        float depthP1 = orientedPointPlaneDistance(p, rcplane.p, rcplane.n);
        if(depth > depthP1)
        {
            direction = -1.0f;
        }
    }

    StaticVector<float>* out1 = new StaticVector<float>();
    out1->reserve(2 * _sgmParams.rcTcDepthsHalfLimit);

    Point2d tpix = midpoint;
    float depthOld = -1.0f;
    int istep = 0;
    bool ok = true;

    // compute depths for all pixels from the middle point to on one side of the epipolar line
    while((out1->size() < _sgmParams.rcTcDepthsHalfLimit) && (_mp.isPixelInImage(tpix, tc) == true) && (ok == true))
    {
        tpix = tpix + pixelVect * direction;

        Point3d refvect = _mp.iCamArr[_rc] * rmid;
        Point3d tarvect = _mp.iCamArr[tc] * tpix;
        float rptpang = angleBetwV1andV2(refvect, tarvect);

        Point3d p;
        ok = triangulateMatch(p, rmid, tpix, _rc, tc, _mp);

        float depth = orientedPointPlaneDistance(p, rcplane.p, rcplane.n);
        if(_mp.isPixelInImage(tpix, tc) && (depth > 0.0f) && (depth > depthOld) &&
           checkPair(p, _rc, tc, _mp, _mp.getMinViewAngle(), _mp.getMaxViewAngle()) &&
           (rptpang >
            _mp.getMinViewAngle()) // WARNING if vects are near parallel thaen this results to strange angles ...
           &&
           (rptpang <
            _mp.getMaxViewAngle())) // this is the propper angle ... beacause is does not depend on the triangluated p
        {
            out1->push_back(depth);
            // if ((tpix.x!=tpixold.x)||(tpix.y!=tpixold.y)||(depthOld>=depth))
            //{
            // printf("after %f %f %f %f %i %f %f\n",tpix.x,tpix.y,depth,depthOld,istep,ang,kk);
            //};
        }
        else
        {
            ok = false;
        }
        depthOld = depth;
        istep++;
    }

    StaticVector<float>* out2 = new StaticVector<float>();
    out2->reserve(2 * _sgmParams.rcTcDepthsHalfLimit);
    tpix = midpoint;
    istep = 0;
    ok = true;

    // compute depths for all pixels from the middle point to the other side of the epipolar line
    while((out2->size() < _sgmParams.rcTcDepthsHalfLimit) && (_mp.isPixelInImage(tpix, tc) == true) && (ok == true))
    {
        const Point3d refvect = _mp.iCamArr[_rc] * rmid;
        const Point3d tarvect = _mp.iCamArr[tc] * tpix;
        const float rptpang = angleBetwV1andV2(refvect, tarvect);

        Point3d p;
        ok = triangulateMatch(p, rmid, tpix, _rc, tc, _mp);

        float depth = orientedPointPlaneDistance(p, rcplane.p, rcplane.n);
        if(_mp.isPixelInImage(tpix, tc) && (depth > 0.0f) && (depth < depthOld) &&
           checkPair(p, _rc, tc, _mp, _mp.getMinViewAngle(), _mp.getMaxViewAngle()) &&
           (rptpang >
            _mp.getMinViewAngle()) // WARNING if vects are near parallel thaen this results to strange angles ...
           &&
           (rptpang <
            _mp.getMaxViewAngle())) // this is the propper angle ... beacause is does not depend on the triangluated p
        {
            out2->push_back(depth);
            // printf("%f %f\n",tpix.x,tpix.y);
        }
        else
        {
            ok = false;
        }

        depthOld = depth;
        tpix = tpix - pixelVect * direction;
    }

    // printf("out2\n");
    StaticVector<float>* out = new StaticVector<float>();
    out->reserve(2 * _sgmParams.rcTcDepthsHalfLimit);
    for(int i = out2->size() - 1; i >= 0; i--)
    {
        out->push_back((*out2)[i]);
        // printf("%f\n",(*out2)[i]);
    }
    // printf("out1\n");
    for(int i = 0; i < out1->size(); i++)
    {
        out->push_back((*out1)[i]);
        // printf("%f\n",(*out1)[i]);
    }

    delete out2;
    delete out1;

    // we want to have it in ascending order
    if(out->size() > 0 && (*out)[0] > (*out)[out->size() - 1])
    {
        StaticVector<float>* outTmp = new StaticVector<float>();
        outTmp->reserve(out->size());
        for(int i = out->size() - 1; i >= 0; i--)
        {
            outTmp->push_back((*out)[i]);
        }
        delete out;
        out = outTmp;
    }

    // check if it is asc
    for(int i = 0; i < out->size() - 1; i++)
    {
        if((*out)[i] > (*out)[i + 1])
        {

            for(int j = 0; j <= i + 1; j++)
            {
                ALICEVISION_LOG_TRACE("getDepthsRcTc: check if it is asc: " << (*out)[j]);
            }
            ALICEVISION_LOG_WARNING("getDepthsRcTc: not asc");

            if(out->size() > 1)
            {
                qsort(&(*out)[0], out->size(), sizeof(float), qSortCompareFloatAsc);
            }
        }
    }

    ALICEVISION_LOG_DEBUG("Find depths over the epipolar line segment between R and T cameras:" << std::endl
                          << "\t- rc: " << _rc << std::endl
                          << "\t- tc: " << tc << std::endl
                          << "\t- all depths found: " << allDepthsFound << std::endl
                          << "\t- correct depths found: " << allDepths << std::endl
                          << "\t- depths to use: " << out->size());
    return out;
}

bool SgmDepthList::selectBestDepthsRange(int nDepthsThr, StaticVector<float>* rcSeedsDistsAsc)
{
    if(_depths.size() <= nDepthsThr)
        return true;

    StaticVector<int> votes;
    votes.reserve(_depths.size() - nDepthsThr);
    for(int i = 0; i < _depths.size() - nDepthsThr; i++)
    {
        const float d1 = _depths[i];
        const float d2 = _depths[i + nDepthsThr - 1];

        int id1 = rcSeedsDistsAsc->indexOfNearestSorted(d1);
        int id2 = rcSeedsDistsAsc->indexOfNearestSorted(d2);

        if(d1 < (*rcSeedsDistsAsc)[0])
            id1 = 0;

        if(d2 > (*rcSeedsDistsAsc)[rcSeedsDistsAsc->size() - 1])
            id2 = rcSeedsDistsAsc->size() - 1;

        if((id1 > -1) && (id2 > -1))
            votes.push_back(abs(id2 - id1));
        else
            votes.push_back(0);
    }

    StaticVector<float> depthsNew;
    depthsNew.reserve(nDepthsThr);

    const int id1 = votes.maxValId();
    const int id2 = id1 + nDepthsThr - 1;

    for(int i = id1; i <= id2; i++)
        depthsNew.push_back(_depths[i]);

    std::swap(_depths, depthsNew);
    return true;
}

bool SgmDepthList::selectBestDepthsRange(int nDepthsThr, StaticVector<StaticVector<float>*>* alldepths)
{
    if(nDepthsThr <= 0 || _depths.size() <= nDepthsThr)
        return true;

    StaticVector<float> votes;
    votes.reserve(_depths.size() - nDepthsThr);

    for(int i = 0; i < _depths.size() - nDepthsThr; i++)
    {
        const float d1 = _depths[i];
        const float d2 = _depths[i + nDepthsThr - 1];
        float overlap = 0.0f;

        for(int c = 0; c < alldepths->size(); c++)
        {
            const StaticVector<float>* tcDepths = (*alldepths)[c];
            const float dd1 = std::max(d1, (*tcDepths)[0]);
            const float dd2 = std::min(d2, (*tcDepths)[tcDepths->size() - 1]);
            if(dd1 < dd2)
                overlap += dd2 - dd1;
        }
        votes.push_back(overlap);
    }

    StaticVector<float> depthsNew;
    depthsNew.reserve(nDepthsThr);

    const int id1 = votes.maxValId();
    const int id2 = id1 + nDepthsThr - 1;

    for(int i = id1; i <= id2; i++)
        depthsNew.push_back(_depths[i]);

    std::swap(_depths, depthsNew);
    return true;
}

} // namespace depthMap
} // namespace aliceVision
