// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "SgmDepthList.hpp"

#include <aliceVision/alicevision_omp.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/mvsData/ROI.hpp>
#include <aliceVision/mvsData/Point3d.hpp>
#include <aliceVision/mvsData/OrientedPoint.hpp>
#include <aliceVision/mvsData/geometry.hpp>
#include <aliceVision/mvsUtils/common.hpp>
#include <aliceVision/sfmData/SfMData.hpp>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>

namespace aliceVision {
namespace depthMap {

int indexOfNearestSorted(const std::vector<float>& in_vector, const float value)
{
    // retrieve the first element >= value in _data
    auto it = std::lower_bound(in_vector.begin(), in_vector.end(), value);

    if (it == in_vector.end())
        return -1;

    if (it != in_vector.begin())
    {
        // select the index of the closest value between it (>= value) and prevIt (< value)
        const auto prevIt = std::prev(it);
        it = (value - *prevIt) < (*it - value) ? prevIt : it;
    }
    return std::distance(in_vector.begin(), it);
}

SgmDepthList::SgmDepthList(const mvsUtils::MultiViewParams& mp, const SgmParams& sgmParams, const Tile& tile)
  : _mp(mp),
    _sgmParams(sgmParams),
    _tile(tile)
{}

void SgmDepthList::computeListRc()
{
    ALICEVISION_LOG_DEBUG(_tile << "Compute SGM depths list.");

    // reset member variables
    _depths.clear();
    _depthsTcLimits.clear();

    // compute min/max/mid/nb depth from SfM
    std::size_t nbObsDepths;
    float minObsDepth, maxObsDepth, midObsDepth;
    getMinMaxMidNbDepthFromSfM(minObsDepth, maxObsDepth, midObsDepth, nbObsDepths);

    if (nbObsDepths < 2)
    {
        ALICEVISION_LOG_INFO(_tile << "Cannot get min/max/middle depth from SfM.");
        return;  // nothing to do
    }

    // compute depth list for each T cameras
    std::vector<std::vector<float>> depthsPerTc(_tile.sgmTCams.size());

    for (std::size_t c = 0; c < _tile.sgmTCams.size(); ++c)
    {
        std::vector<float>& tcDepths = depthsPerTc.at(c);

        // compute depths of all meaningful points on the principal ray of the R camera regarding each T cameras
        computeRcTcDepths(_tile.sgmTCams.at(c), (nbObsDepths < 10) ? -1 : midObsDepth, tcDepths);

        if (tcDepths.size() < 10)  // fallback if we don't have enough valid samples over the epipolar line
        {
            ALICEVISION_LOG_DEBUG(_tile << "Not enough valid samples over the epipolar line. Compute depth list from R camera pixel size.");

            tcDepths.clear();

            computePixelSizeDepths(minObsDepth, midObsDepth, maxObsDepth * _sgmParams.prematchingMaxDepthScale, tcDepths);
        }
    }

    // compute min/max for all Rc/Tc depth list
    float minDepthAll = std::numeric_limits<float>::max();
    float maxDepthAll = std::numeric_limits<float>::min();

    for (const std::vector<float>& tcDepths : depthsPerTc)
    {
        for (const float depth : tcDepths)
        {
            minDepthAll = std::min(minDepthAll, depth);
            maxDepthAll = std::max(maxDepthAll, depth);
        }
    }

    // no depths found
    if (minDepthAll > maxDepthAll)
    {
        ALICEVISION_LOG_INFO(_tile << "No depths found.");
        return;  // nothing to do
    }

    ALICEVISION_LOG_DEBUG(_tile << "Depth candidates from seeds for R camera:" << std::endl
                                << "\t- nb observations: " << nbObsDepths << std::endl
                                << "\t- all depth range: [" << minDepthAll << "-" << maxDepthAll << "]" << std::endl
                                << "\t- sfm depth range: [" << minObsDepth << "-" << maxObsDepth << "]");

    float firstDepth = minDepthAll;
    float lastDepth = maxDepthAll;

    // if we want to use SfM seeds anf if we get enough information from these seeds, adjust min/maxDepth
    if (_sgmParams.useSfmSeeds && !_mp.getInputSfMData().getLandmarks().empty() && nbObsDepths > 10)
    {
        const float margin = _sgmParams.seedsRangeInflate * (maxObsDepth - minObsDepth);
        firstDepth = std::max(0.f, minObsDepth - margin);
        lastDepth = maxObsDepth + margin;

        if (maxDepthAll < firstDepth || minDepthAll > lastDepth)
        {
            // no intersection between min/maxDepth and min/maxDepthSample
            // keep min/maxDepth value as is
        }
        else
        {
            // min/maxDepth intersection with min/maxDepthAll
            firstDepth = std::max(minDepthAll, firstDepth);
            lastDepth = std::min(maxDepthAll, lastDepth);
        }
        ALICEVISION_LOG_DEBUG(_tile << "Final depth range (intersection: frustums / landmarks with margin): [" << firstDepth << "-" << lastDepth
                                    << "]");
    }

    // build the list of "best" depths for rc, from all tc cameras depths
    computeRcDepthList(firstDepth, lastDepth, (_sgmParams.stepZ > 0.0f ? _sgmParams.stepZ : 1.0f), depthsPerTc);

    // filter out depths if computeDepths gave too many values
    if (_sgmParams.maxDepths > 0 && _depths.size() > _sgmParams.maxDepths)
    {
        const float scaleFactor = float(_depths.size()) / float(_sgmParams.maxDepths);

        ALICEVISION_LOG_DEBUG(_tile << "Too many values in R camera depth list, filter out with scale factor:" << std::endl
                                    << "\t- nb depths: " << _depths.size() << std::endl
                                    << "\t- max depths: " << _sgmParams.maxDepths << std::endl
                                    << "\t- scale factor to apply: " << scaleFactor);

        computeRcDepthList(firstDepth, lastDepth, scaleFactor, depthsPerTc);

        // ensure depth list size is not greater than maxDepths
        if (_depths.size() > _sgmParams.maxDepths)
            _depths.resize(_sgmParams.maxDepths);  // reduce to depth list first maxDepths elements
    }

    ALICEVISION_LOG_DEBUG(_tile << "Final depth range for R camera:" << std::endl
                                << "\t- nb selected depths: " << _depths.size() << std::endl
                                << "\t- selected depth range: [" << firstDepth << "-" << lastDepth << "]");

    // update depth tc limits
    _depthsTcLimits.resize(_tile.sgmTCams.size());

    // fill depthsTcamsLimits member variable with index range of depths to sweep
    for (std::size_t c = 0; c < _tile.sgmTCams.size(); ++c)
    {
        if (depthsPerTc.empty())
        {
            _depthsTcLimits[c] = Pixel(-1, -1);
            continue;
        }

        const float d1 = depthsPerTc.at(c).front();
        const float d2 = depthsPerTc.at(c).back();

        int id1 = indexOfNearestSorted(_depths, d1);
        int id2 = indexOfNearestSorted(_depths, d2);

        if (id1 == -1)
            id1 = 0;

        if (id2 == -1)
            id2 = _depths.size() - 1;

        _depthsTcLimits[c] = Pixel(id1, id2 - id1 + 1);
    }

    if (_sgmParams.exportDepthsTxtFiles)
        exportTxtFiles(depthsPerTc);

    ALICEVISION_LOG_DEBUG(_tile << "Compute SGM depths list done.");
}

void SgmDepthList::removeTcWithNoDepth(Tile& tile)
{
    assert(tile.rc == _tile.rc);
    assert(tile.sgmTCams.size() == _tile.sgmTCams.size());

    std::vector<int> out_tCams;
    std::vector<Pixel> out_depthsTcLimits;

    for (size_t c = 0; c < tile.sgmTCams.size(); ++c)
    {
        const Pixel& tcLimits = _depthsTcLimits.at(c);
        const int tc = tile.sgmTCams.at(c);

        if (tcLimits.x != -1 && tcLimits.y != -1)
        {
            out_tCams.push_back(tc);
            out_depthsTcLimits.push_back(tcLimits);
        }
        else
        {
            ALICEVISION_LOG_INFO(_tile << "Remove T camera (tc: " << tc << ", view id: " << _mp.getViewId(tc) << ") no depth found.");
        }
    }

    std::swap(tile.sgmTCams, out_tCams);
    std::swap(_depthsTcLimits, out_depthsTcLimits);
}

void SgmDepthList::logRcTcDepthInformation() const
{
    std::ostringstream ostr;
    ostr << "Camera / Depth information: " << std::endl
         << "\t- R camera:" << std::endl
         << "\t   - id: " << _tile.rc << std::endl
         << "\t   - view id: " << _mp.getViewId(_tile.rc) << std::endl
         << "\t   - depth planes: " << _depths.size() << std::endl
         << "\t   - depths range: [" << _depths[0] << "-" << _depths[_depths.size() - 1] << "]" << std::endl
         << "\t- T cameras:" << std::endl;

    for (std::size_t c = 0; c < _tile.sgmTCams.size(); ++c)
    {
        ostr << "\t   - T camera (" << (c + 1) << "/" << _tile.sgmTCams.size() << "):" << std::endl
             << "\t      - id: " << _tile.sgmTCams.at(c) << std::endl
             << "\t      - view id: " << _mp.getViewId(_tile.sgmTCams.at(c)) << std::endl
             << "\t      - depth planes: " << _depthsTcLimits[c].y << std::endl
             << "\t      - depths range: [" << _depths[_depthsTcLimits[c].x] << "-" << _depths[_depthsTcLimits[c].x + _depthsTcLimits[c].y - 1] << "]"
             << std::endl
             << "\t      - depth indexes range: [" << _depthsTcLimits[c].x << "-" << _depthsTcLimits[c].x + _depthsTcLimits[c].y << "]" << std::endl;
    }

    ALICEVISION_LOG_INFO(_tile << ostr.str());
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

    const int startingDepth = std::min_element(_depthsTcLimits.begin(), _depthsTcLimits.end(), MinOffX())->x;
    const auto depth_it = std::max_element(_depthsTcLimits.begin(), _depthsTcLimits.end(), MinOffXplusY());
    const int stoppingDepth = depth_it->x + depth_it->y;

    // The overall starting depth index should always be zero.
    assert(startingDepth == 0);

    // Usually stoppingDepth should be equal to the total number of depths.
    // But due to sgmMaxDepths and sgmMaxDepthPerTc, we can have more depths
    // than we finally use in all TC cameras.
    assert(_depths.size() >= stoppingDepth);
}

void SgmDepthList::getMinMaxMidNbDepthFromSfM(float& out_min, float& out_max, float& out_mid, std::size_t& out_nbDepths) const
{
    using namespace boost::accumulators;

    const std::size_t cacheSize = 1000;
    accumulator_set<float, stats<tag::tail_quantile<left>>> accDistanceMin(tag::tail<left>::cache_size = cacheSize);
    accumulator_set<float, stats<tag::tail_quantile<right>>> accDistanceMax(tag::tail<right>::cache_size = cacheSize);

    const IndexT viewId = _mp.getViewId(_tile.rc);

    const ROI fullsizeRoi = upscaleROI(_tile.roi, _mp.getProcessDownscale());  // landmark observations are in the full-size image coordinate system
    // const ROI selectionRoi = inflateROI(fullsizeRoi, 1.4f); // we can inflate the image full-size roi to be more permissive for common landmark
    // selection

    OrientedPoint cameraPlane;
    cameraPlane.p = _mp.CArr[_tile.rc];
    cameraPlane.n = _mp.iRArr[_tile.rc] * Point3d(0.0, 0.0, 1.0);
    cameraPlane.n = cameraPlane.n.normalize();

    Point3d midDepthPoint;
    out_nbDepths = 0;

    // for each landmark
    for (const auto& landmarkPair : _mp.getInputSfMData().getLandmarks())
    {
        const sfmData::Landmark& landmark = landmarkPair.second;
        const Point3d point(landmark.X(0), landmark.X(1), landmark.X(2));

        // find rc observation
        const auto it = landmark.getObservations().find(viewId);

        // no rc observation
        if (it == landmark.getObservations().end())
            continue;

        // get rc 2d observation
        const Vec2& obs2d = it->second.getCoordinates();

        // if we compute depth list per tile keep only observation located inside the inflated image full-size ROI
        if (!_sgmParams.depthListPerTile || fullsizeRoi.contains(obs2d.x(), obs2d.y()))
        {
            const float distance = static_cast<float>(pointPlaneDistance(point, cameraPlane.p, cameraPlane.n));
            accDistanceMin(distance);
            accDistanceMax(distance);
            midDepthPoint = midDepthPoint + point;
            ++out_nbDepths;
        }
    }

    if (out_nbDepths > 0)
    {
        out_min = quantile(accDistanceMin, quantile_probability = 1.0 - _sgmParams.seedsRangePercentile);
        out_max = quantile(accDistanceMax, quantile_probability = _sgmParams.seedsRangePercentile);
        midDepthPoint = midDepthPoint / static_cast<float>(out_nbDepths);
        out_mid = pointPlaneDistance(midDepthPoint, cameraPlane.p, cameraPlane.n);
    }
    else
    {
        out_min = 0.f;
        out_max = 0.f;
        out_mid = 0.f;
    }

    ALICEVISION_LOG_DEBUG(_tile << "Compute min/max/mid/nb observation depth from SfM for R camera:" << std::endl
                                << "\t- view id: " << viewId << std::endl
                                << "\t- min depth: " << out_min << std::endl
                                << "\t- max depth: " << out_max << std::endl
                                << "\t- mid depth: " << out_mid << std::endl
                                << "\t- nb depth: " << out_nbDepths << std::endl
                                << "\t- percentile: " << _sgmParams.seedsRangePercentile);
}

void SgmDepthList::getRcTcDepthRangeFromSfM(int tc, double& out_zmin, double& out_zmax) const
{
    // get Rc/Tc view ids
    const IndexT rcViewId = _mp.getViewId(_tile.rc);
    const IndexT tcViewId = _mp.getViewId(tc);

    // get R region-of-interest
    // landmark observations are in the full-size image coordinate system, we need to upcscale the tile ROI
    const ROI fullsizeRoi = upscaleROI(_tile.roi, _mp.getProcessDownscale());
    // const ROI selectionRoi = inflateROI(fullsizeRoi, 1.4f); // we can inflate the image full-size roi to be more permissive for common landmark
    // selection

    // build R camera plane
    OrientedPoint cameraPlane;
    cameraPlane.p = _mp.CArr[_tile.rc];
    cameraPlane.n = _mp.iRArr[_tile.rc] * Point3d(0.0, 0.0, 1.0);
    cameraPlane.n = cameraPlane.n.normalize();

    // initialize output min/max depth
    out_zmin = std::numeric_limits<double>::max();
    out_zmax = std::numeric_limits<double>::min();

    // for each landmark
    for (const auto& landmarkPair : _mp.getInputSfMData().getLandmarks())
    {
        const sfmData::Landmark& landmark = landmarkPair.second;
        const Point3d point(landmark.X(0), landmark.X(1), landmark.X(2));

        // no tc observation
        if (landmark.getObservations().find(tcViewId) == landmark.getObservations().end())
            continue;

        // find rc observation
        const auto it = landmark.getObservations().find(rcViewId);

        // no rc observation
        if (it == landmark.getObservations().end())
            continue;

        // get rc 2d observation
        const Vec2& obs2d = it->second.getCoordinates();

        // observation located inside the inflated image full-size ROI
        if (!_sgmParams.depthListPerTile || fullsizeRoi.contains(obs2d.x(), obs2d.y()))
        {
            // compute related depth
            const double depth = pointPlaneDistance(point, cameraPlane.p, cameraPlane.n);

            // update min/max depth
            out_zmin = std::min(out_zmin, depth);
            out_zmax = std::max(out_zmax, depth);
        }
    }

    // no common observations found
    if (out_zmin > out_zmax)
    {
        ALICEVISION_THROW_ERROR(_tile << "Cannot compute min/max depth from common Rc/Tc SfM observations." << std::endl
                                      << "No common observations found (tc view id: " << tcViewId << ").");
    }

    ALICEVISION_LOG_DEBUG(_tile << "Compute min/max depth from common Rc/Tc SfM observations:" << std::endl
                                << "\t- rc: " << _tile.rc << " (view id: " << rcViewId << ")" << std::endl
                                << "\t- tc: " << tc << " (view id: " << tcViewId << ")" << std::endl
                                << "\t- min depth: " << out_zmin << std::endl
                                << "\t- max depth: " << out_zmax);
}

void SgmDepthList::computeRcTcDepths(int tc, float midDepth, std::vector<float>& out_depths) const
{
    assert(out_depths.empty());

    OrientedPoint rcplane;
    rcplane.p = _mp.CArr[_tile.rc];
    rcplane.n = _mp.iRArr[_tile.rc] * Point3d(0.0, 0.0, 1.0);
    rcplane.n = rcplane.n.normalize();

    // ROI center
    const Point2d roiCenter((_tile.roi.x.begin + (_tile.roi.width() * 0.5)), _tile.roi.y.begin + (_tile.roi.height() * 0.5));

    // principal point of the rc camera
    const Point2d principalPoint(_mp.getWidth(_tile.rc) * 0.5, _mp.getHeight(_tile.rc) * 0.5);

    // reference point for the epipolar line
    const Point2d referencePoint = (!_sgmParams.depthListPerTile) ? principalPoint : roiCenter;

    // input middle depth related point
    Point2d tcMidDepthPoint;

    // segment of epipolar line
    Point2d tcFromPoint, tcToPoint;

    {
        const Matrix3x4& rP = _mp.camArr[_tile.rc];
        const Matrix3x4& tP = _mp.camArr[tc];

        Point3d rC;
        Matrix3x3 rR;
        Matrix3x3 riR;
        Matrix3x3 rK;
        Matrix3x3 riK;
        Matrix3x3 riP;
        _mp.decomposeProjectionMatrix(rC, rR, riR, rK, riK, riP, rP);

        _mp.getPixelFor3DPoint(&tcMidDepthPoint, ((riP * referencePoint) * midDepth) + rC, tP);

        double zmin;
        double zmax;

        getRcTcDepthRangeFromSfM(tc, zmin, zmax);

        Point2d tarpix1;
        Point2d tarpix2;

        _mp.getPixelFor3DPoint(&tarpix1, ((riP * referencePoint) * zmin) + rC, tP);
        _mp.getPixelFor3DPoint(&tarpix2, ((riP * referencePoint) * zmax) + rC, tP);

        get2dLineImageIntersection(&tcFromPoint, &tcToPoint, tarpix1, tarpix2, _mp, tc);
    }

    const int nbSegmentPoints = static_cast<int>((tcToPoint - tcFromPoint).size());
    const int nbSegmentPointsAtSgmScale = nbSegmentPoints / _sgmParams.scale;
    const Point2d pixelVect = (tcToPoint - tcFromPoint).normalize() * std::max(1.0, double(_sgmParams.scale));

    // compute the epilolar segment depth direction
    int depthDirection = 1;
    {
        Point3d p;

        // triangulate middle depth point
        if (!triangulateMatch(p, referencePoint, tcMidDepthPoint, _tile.rc, tc, _mp))
            return;

        const float depth = orientedPointPlaneDistance(p, rcplane.p, rcplane.n);

        // triangulate middle depth point + 1 pixelVect
        if (!triangulateMatch(p, referencePoint, tcMidDepthPoint + pixelVect, _tile.rc, tc, _mp))
            return;

        const float depthP1 = orientedPointPlaneDistance(p, rcplane.p, rcplane.n);

        if (depth > depthP1)
            depthDirection = -1;
    }

    out_depths.reserve(nbSegmentPointsAtSgmScale);

    const Point3d refVect = _mp.iCamArr[_tile.rc] * referencePoint;
    float previousDepth = -1.0f;

    // compute depths for all pixels from one side of the epipolar segment to the other
    for (int i = 0; i < nbSegmentPointsAtSgmScale; ++i)
    {
        const Point2d tcPoint = ((depthDirection > 0) ? tcFromPoint : tcToPoint) + (pixelVect * double(i) * double(depthDirection));

        // check if the epipolar segment point is in T camera
        // note: get2dLineImageIntersection can give points slightly out of the picture
        if (!_mp.isPixelInImage(tcPoint, tc))
            continue;

        const Point3d tarVect = _mp.iCamArr[tc] * tcPoint;
        const float refTarVectAngle = angleBetwV1andV2(refVect, tarVect);

        // if vects are near parallel then this results to strange angles
        // this is the proper angle because it does not depend on the triangulated p
        if (refTarVectAngle < _mp.getMinViewAngle() || refTarVectAngle > _mp.getMaxViewAngle())
            continue;

        // epipolar segment point related 3d point
        Point3d p;

        // triangulate principal point from rc with tcPoint
        if (!triangulateMatch(p, referencePoint, tcPoint, _tile.rc, tc, _mp))
            continue;

        // check the difference in pixel size between R and T and the angle size of p
        // note: disabled for now, this test is too strict and rejects too many points.
        // if(!checkPair(p, _tile.rc, tc, _mp, _mp.getMinViewAngle(), _mp.getMaxViewAngle()))
        //    continue;

        // compute related 3d point depth
        const float depth = float(orientedPointPlaneDistance(p, rcplane.p, rcplane.n));

        if ((depth > 0.0f) && (depth > previousDepth))
        {
            out_depths.push_back(depth);
            previousDepth = depth + std::numeric_limits<float>::epsilon();
        }
    }

    out_depths.shrink_to_fit();

    ALICEVISION_LOG_DEBUG(_tile << "Find depths over the epipolar line segment between R and T cameras:" << std::endl
                                << "\t- rc: " << _tile.rc << "(view id: " << _mp.getViewId(_tile.rc) << ")" << std::endl
                                << "\t- tc: " << tc << "(view id: " << _mp.getViewId(tc) << ")" << std::endl
                                << "\t- # points of the epipolar segment: " << nbSegmentPoints << std::endl
                                << "\t- # points of the epipolar segment at SGM scale: " << nbSegmentPointsAtSgmScale << std::endl
                                << "\t- # depths to use: " << out_depths.size());

    if (!out_depths.empty())
        ALICEVISION_LOG_DEBUG(_tile << "Depth to use range [" << out_depths.front() << "-" << out_depths.back() << "]" << std::endl);
}

void SgmDepthList::computePixelSizeDepths(float minObsDepth, float midObsDepth, float maxObsDepth, std::vector<float>& out_depths) const
{
    assert(out_depths.empty());

    const int rcDepthsCompStep = 6;
    const int maxDepthsHalf = 1024;

    const float d = float(_sgmParams.scale) * float(rcDepthsCompStep);

    OrientedPoint rcplane;
    rcplane.p = _mp.CArr[_tile.rc];
    rcplane.n = _mp.iRArr[_tile.rc] * Point3d(0.0, 0.0, 1.0);
    rcplane.n = rcplane.n.normalize();

    int ndepthsMidMax = 0;
    float maxdepth = midObsDepth;
    while ((maxdepth < maxObsDepth) && (ndepthsMidMax < maxDepthsHalf))
    {
        Point3d p = rcplane.p + rcplane.n * maxdepth;
        float pixSize = _mp.getCamPixelSize(p, _tile.rc, d);
        maxdepth += pixSize;
        ndepthsMidMax++;
    }

    int ndepthsMidMin = 0;
    float mindepth = midObsDepth;
    while ((mindepth > minObsDepth) && (ndepthsMidMin < maxDepthsHalf * 2 - ndepthsMidMax))
    {
        Point3d p = rcplane.p + rcplane.n * mindepth;
        float pixSize = _mp.getCamPixelSize(p, _tile.rc, d);
        mindepth -= pixSize;
        ndepthsMidMin++;
    }

    // get number of depths
    float depth = mindepth;
    int ndepths = 0;
    float pixSize = 1.0f;
    while ((depth < maxdepth) && (pixSize > 0.0f) && (ndepths < 2 * maxDepthsHalf))
    {
        Point3d p = rcplane.p + rcplane.n * depth;
        pixSize = _mp.getCamPixelSize(p, _tile.rc, d);
        depth += pixSize;
        ndepths++;
    }

    out_depths.reserve(ndepths);

    // fill
    depth = mindepth;
    pixSize = 1.0f;
    ndepths = 0;
    while ((depth < maxdepth) && (pixSize > 0.0f) && (ndepths < 2 * maxDepthsHalf))
    {
        out_depths.push_back(depth);
        Point3d p = rcplane.p + rcplane.n * depth;
        pixSize = _mp.getCamPixelSize(p, _tile.rc, d);
        depth += pixSize;
        ndepths++;
    }

    // check if it is asc
    for (int i = 0; i < out_depths.size() - 1; i++)
    {
        if (out_depths[i] >= out_depths[i + 1])
        {
            for (int j = 0; j <= i + 1; j++)
            {
                ALICEVISION_LOG_TRACE(_tile << "getDepthsByPixelSize: check if it is asc: " << out_depths[j]);
            }
            throw std::runtime_error("getDepthsByPixelSize not asc.");
        }
    }
}

void SgmDepthList::computeRcDepthList(float firstDepth, float lastDepth, float scaleFactor, const std::vector<std::vector<float>>& dephtsPerTc)
{
    _depths.clear();

    float depth = firstDepth;

    while (depth < lastDepth)
    {
        _depths.push_back(depth);

        // get min tc step at depth
        float minTcStep = lastDepth - firstDepth;

        // for each tc camera
        for (const std::vector<float>& tcDepths : dephtsPerTc)
        {
            // get the tc depth closest to the current depth
            const int id = indexOfNearestSorted(tcDepths, depth);

            // continue on no result or last element (we need id + 1)
            if (id < 0 || id >= tcDepths.size() - 1)
                continue;

            // enclosing depth range
            const float tcStep = fabs(tcDepths.at(id) - tcDepths.at(id + 1));  // (closest - next) depths distance

            // keep this value if smallest step so far
            minTcStep = std::min(minTcStep, tcStep);
        }

        depth += minTcStep * scaleFactor;
    }
}

void SgmDepthList::exportTxtFiles(const std::vector<std::vector<float>>& dephtsPerTc) const
{
    const std::string prefix(_mp.getDepthMapsFolder() + std::to_string(_mp.getViewId(_tile.rc)) + std::string("_"));
    const std::string suffix("_" + std::to_string(_tile.roi.x.begin) + "_" + std::to_string(_tile.roi.y.begin) + ".txt");

    // export depthsTcLimits txt file
    {
        const std::string fn = prefix + "depthsTcLimits" + suffix;
        FILE* f = fopen(fn.c_str(), "w");
        for (int j = 0; j < _depthsTcLimits.size(); j++)
        {
            Pixel l = _depthsTcLimits[j];
            fprintf(f, "%i %i\n", l.x, l.y);
        }
        fclose(f);
    }

    // export rc depth txt file
    {
        const std::string fn = prefix + "depths" + suffix;
        FILE* f = fopen(fn.c_str(), "w");
        for (int j = 0; j < _depths.size(); j++)
        {
            fprintf(f, "%f\n", _depths[j]);
        }
        fclose(f);
    }

    // export all depths per tc txt files
    {
        for (int c = 0; c < dephtsPerTc.size(); ++c)
        {
            const std::string fn = prefix + "depths_tc_" + mvsUtils::num2str(_mp.getViewId(_tile.sgmTCams.at(c))) + suffix;
            FILE* f = fopen(fn.c_str(), "w");
            for (const float depth : dephtsPerTc.at(c))
            {
                fprintf(f, "%f\n", depth);
            }
            fclose(f);
        }
    }
}

}  // namespace depthMap
}  // namespace aliceVision
