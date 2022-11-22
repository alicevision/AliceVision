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

SgmDepthList::SgmDepthList(Tile& tile)
    : _tile(tile)
{}

void SgmDepthList::computeListRc(const mvsUtils::MultiViewParams& mp, const SgmParams& sgmParams)
{
    ALICEVISION_LOG_DEBUG(_tile << "Compute SGM depths list.");

    // compute min/max/mid/nb depth from SfM
    std::size_t nbObsDepths;
    float minObsDepth, maxObsDepth, midObsDepth;
    getMinMaxMidNbDepthFromSfM(mp, sgmParams, minObsDepth, maxObsDepth, midObsDepth, nbObsDepths);

    StaticVector<StaticVector<float>*>* alldepths;

    // all depths from the principal ray provided by target cameras
    if(nbObsDepths < 10)
        alldepths = computeAllDepthsAndResetTcs(mp, sgmParams, -1);
    else
        alldepths = computeAllDepthsAndResetTcs(mp, sgmParams, midObsDepth);

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
        ALICEVISION_LOG_DEBUG(_tile << "Depth candidates from seeds for R camera:" << std::endl
                                    << "\t- nb observations: " << nbObsDepths <<  std::endl
                                    << "\t- all depth range: [" << minDepthAll << "-" << maxDepthAll << "]"
                                    << "\t- sfm depth range: [" << minObsDepth << "-" << maxObsDepth << "]");

        float minDepth = minDepthAll;
        float maxDepth = maxDepthAll;

        // if we want to use SfM seeds anf if we get enough information from these seeds, adjust min/maxDepth
        if(sgmParams.useSfmSeeds && !mp.getInputSfMData().getLandmarks().empty() && nbObsDepths > 10)
        {
            const float margin = sgmParams.seedsRangeInflate * (maxObsDepth-minObsDepth);
            minDepth = minObsDepth - margin;
            maxDepth = maxObsDepth + margin;

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
            ALICEVISION_LOG_DEBUG(_tile << "Final depth range (intersection: frustums / landmarks with margin): [" << minDepth << "-" << maxDepth << "]");
        }

        // build the list of "best" depths for rc, from all tc cameras depths
        computeDepths(minDepth, maxDepth, (sgmParams.stepZ > 0.0f ? sgmParams.stepZ : 1.0f), alldepths);

        // filter out depths if computeDepths gave too many values
        if(sgmParams.maxDepths > 0 && _depths.size() > sgmParams.maxDepths)
        {
            const float scaleFactor = float(_depths.size()) / float(sgmParams.maxDepths);

            ALICEVISION_LOG_DEBUG(_tile << "Too many values in R camera depth list, filter out with scale factor:" << std::endl
                                        << "\t- nb depths: " << _depths.size() << std::endl
                                        << "\t- max depths: " << sgmParams.maxDepths << std::endl
                                        << "\t- scale factor to apply: " << scaleFactor);

            computeDepths(minDepth, maxDepth, scaleFactor, alldepths);

            // ensure depth list size is not greater than maxDepths
            if(_depths.size() > sgmParams.maxDepths)
              _depths.resize(sgmParams.maxDepths); // reduce to depth list first maxDepths elements
        }

        if(sgmParams.saveDepthsToSweepTxtFile)
        {
            const std::string fn = mp.getDepthMapsFolder() + std::to_string(mp.getViewId(_tile.rc)) + "depthsAll.txt";
            FILE* f = fopen(fn.c_str(), "w");
            for(int j = 0; j < _depths.size(); j++)
            {
                fprintf(f, "%f\n", _depths[j]);
            }
            fclose(f);
        }

        ALICEVISION_LOG_DEBUG(_tile << "Final depth range for R camera:" << std::endl
                                    << "\t- nb selected depths: " << _depths.size() << std::endl
                                    << "\t- selected depth range: [" << minDepth << "-"<< maxDepth << "]");
    }

    // update depth tc limits
    _depthsTcLimits.resize(_tile.sgmTCams.size());

    // fill depthsTcamsLimits member variable with index range of depths to sweep
    for(std::size_t c = 0; c < _tile.sgmTCams.size(); ++c)
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
        id2 = std::min(id1 + sgmParams.maxDepthsPerTc - 1, id2);
        _depthsTcLimits[c] = Pixel(id1, id2 - id1 + 1);
    }

    if(sgmParams.saveDepthsToSweepTxtFile)
    {
        // export depthsTcLimits txt file
        {
            const std::string fn = mp.getDepthMapsFolder() + std::to_string(mp.getViewId(_tile.rc)) + "depthsTcLimits.txt";
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
            const std::string fn = mp.getDepthMapsFolder() + std::to_string(mp.getViewId(_tile.rc)) + "depths.txt";
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
                const std::string fn = mp.getDepthMapsFolder() + std::to_string(mp.getViewId(_tile.rc)) + "depths" +
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
            rcplane.p = mp.CArr[_tile.rc];
            rcplane.n = mp.iRArr[_tile.rc] * Point3d(0.0, 0.0, 1.0);
            rcplane.n = rcplane.n.normalize();

            const std::string fn = mp.getDepthMapsFolder() + std::to_string(mp.getViewId(_tile.rc)) + "rcDepths.txt";
            FILE* f = fopen(fn.c_str(), "w");
            float depth = minDepthAll;
            while(depth < maxDepthAll)
            {
                fprintf(f, "%f\n", depth);
                const Point3d p = rcplane.p + rcplane.n * depth;
                depth = depth + mp.getCamPixelSize(p, _tile.rc);
            }
            fclose(f);
        }
    }

    deleteArrayOfArrays<float>(&alldepths);

    ALICEVISION_LOG_DEBUG(_tile << "Compute SGM depths list done.");
}

void SgmDepthList::logRcTcDepthInformation(const mvsUtils::MultiViewParams& mp) const
{
    std::ostringstream ostr;
    ostr << "Camera / Depth information: " << std::endl
         << "\t- R camera:" << std::endl
         << "\t   - id: " << _tile.rc << std::endl
         << "\t   - view id: " << mp.getViewId(_tile.rc) << std::endl
         << "\t   - depth planes: " << _depths.size() << std::endl
         << "\t   - depths range: [" << _depths[0] << "-" << _depths[_depths.size() - 1] << "]" << std::endl
         << "\t- T cameras:" << std::endl;

    for(std::size_t c = 0; c < _tile.sgmTCams.size(); ++c)
    {
        ostr << "\t   - T camera (" << (c+1) << "/" << _tile.sgmTCams.size() << "):" << std::endl
             << "\t      - id: " << _tile.sgmTCams.at(c) << std::endl
             << "\t      - view id: " << mp.getViewId(_tile.sgmTCams.at(c)) << std::endl
             << "\t      - depth planes: " << _depthsTcLimits[c].y << std::endl
             << "\t      - depths range: [" << _depths[_depthsTcLimits[c].x] << "-" << _depths[_depthsTcLimits[c].x + _depthsTcLimits[c].y - 1] << "]" << std::endl
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

void SgmDepthList::getMinMaxMidNbDepthFromSfM(const mvsUtils::MultiViewParams& mp,
                                              const SgmParams& sgmParams,
                                              float& out_min,
                                              float& out_max,
                                              float& out_mid,
                                              std::size_t& out_nbDepths) const
{
    using namespace boost::accumulators;

    const std::size_t cacheSize = 1000;
    accumulator_set<float, stats<tag::tail_quantile<left>>> accDistanceMin(tag::tail<left>::cache_size = cacheSize);
    accumulator_set<float, stats<tag::tail_quantile<right>>> accDistanceMax(tag::tail<right>::cache_size = cacheSize);

    const IndexT viewId = mp.getViewId(_tile.rc);

    const ROI fullsizeRoi = upscaleROI(_tile.roi, mp.getProcessDownscale()); // landmark observations are in the full-size image coordinate system
    const ROI selectionRoi = inflateROI(fullsizeRoi, 1.4f); // inflate the image full-size roi, this ROI is more permissive for common landmark selection

    OrientedPoint cameraPlane;
    cameraPlane.p = mp.CArr[_tile.rc];
    cameraPlane.n = mp.iRArr[_tile.rc] * Point3d(0.0, 0.0, 1.0);
    cameraPlane.n = cameraPlane.n.normalize();

    Point3d midDepthPoint;
    out_nbDepths = 0;

    // for each landmark
    for(const auto& landmarkPair : mp.getInputSfMData().getLandmarks())
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

                // if we compute depth list per tile keep only observation located inside the inflated image full-size ROI
                if(!sgmParams.chooseDepthListPerTile || selectionRoi.contains(obs2d.x(), obs2d.y()))
                {
                    const float distance = static_cast<float>(pointPlaneDistance(point, cameraPlane.p, cameraPlane.n));
                    accDistanceMin(distance);
                    accDistanceMax(distance);
                    midDepthPoint = midDepthPoint + point;
                    ++out_nbDepths;
                }
            }
        }
    }

    if(out_nbDepths > 0)
    {
      out_min = quantile(accDistanceMin, quantile_probability = 1.0 - sgmParams.seedsRangePercentile);
      out_max = quantile(accDistanceMax, quantile_probability = sgmParams.seedsRangePercentile);
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
                                << "\t- percentile: " << sgmParams.seedsRangePercentile);
}

void SgmDepthList::getRcTcDepthRangeFromSfM(const mvsUtils::MultiViewParams& mp,
                                            const SgmParams& sgmParams,
                                            int rc,
                                            int tc,
                                            double& out_zmin,
                                            double& out_zmax) const
{
    // get Rc/Tc view ids
    const IndexT rcViewId = mp.getViewId(_tile.rc);
    const IndexT tcViewId = mp.getViewId(tc);

    // get R region-of-interest
    // landmark observations are in the full-size image coordinate system, we need to upcscale the tile ROI
    // we can inflate the image full-size roi to be more permissive for common landmark selection
    const ROI fullsizeRoi = upscaleROI(_tile.roi, mp.getProcessDownscale());
    const ROI selectionRoi = fullsizeRoi; // TODO: add user parameter, inflateROI(fullsizeRoi, 1.4f);

    // build R camera plane
    OrientedPoint cameraPlane;
    cameraPlane.p = mp.CArr[_tile.rc];
    cameraPlane.n = mp.iRArr[_tile.rc] * Point3d(0.0, 0.0, 1.0);
    cameraPlane.n = cameraPlane.n.normalize();

    // initialize output min/max depth
    out_zmin = std::numeric_limits<double>::max();
    out_zmax = std::numeric_limits<double>::min();

    // for each landmark
    for(const auto& landmarkPair : mp.getInputSfMData().getLandmarks())
    {
        const sfmData::Landmark& landmark = landmarkPair.second;
        const Point3d point(landmark.X(0), landmark.X(1), landmark.X(2));

        // is tc observation
        if(landmark.observations.find(tcViewId) != landmark.observations.end())
        {
            const auto it = landmark.observations.find(rcViewId);

            // is rc observation
            if(it != landmark.observations.end())
            {
                const Vec2& obs2d = it->second.x;

                // observation located inside the inflated image full-size ROI
                if(!sgmParams.chooseDepthListPerTile || selectionRoi.contains(obs2d.x(), obs2d.y()))
                {
                    // compute related depth
                    const double depth = pointPlaneDistance(point, cameraPlane.p, cameraPlane.n);

                    // update min/max depth
                    out_zmin = std::min(out_zmin, depth);
                    out_zmax = std::max(out_zmax, depth);
                }
            }
        }
    }

    // no common observations found
    if(out_zmin > out_zmax)
    {
        ALICEVISION_THROW_ERROR(_tile << "Cannot compute min/max depth from common Rc/Tc SfM observations." << std::endl
                                      << "No common observations found (tc view id: " << tcViewId << ").");
    }

    ALICEVISION_LOG_DEBUG(_tile << "Compute min/max depth from common Rc/Tc SfM observations:" << std::endl
                                << "\t- rc view id: " << rcViewId << std::endl
                                << "\t- tc view id: " << tcViewId << std::endl
                                << "\t- min depth: "  << out_zmin << std::endl
                                << "\t- max depth: "  << out_zmax);
}

StaticVector<StaticVector<float>*>* SgmDepthList::computeAllDepthsAndResetTcs(const mvsUtils::MultiViewParams& mp,
                                                                              const SgmParams& sgmParams,
                                                                              float midDepth)
{
    std::vector<int> tCamsNew;
    StaticVector<StaticVector<float>*>* alldepths = new StaticVector<StaticVector<float>*>();
    alldepths->reserve(_tile.sgmTCams.size());

    for(std::size_t c = 0; c < _tile.sgmTCams.size(); ++c)
    {
        // depths of all meaningful points on the principal ray of the reference camera regarding the target camera tc
        StaticVector<float>* tcdepths = getDepthsTc(mp, sgmParams, _tile.sgmTCams.at(c), midDepth);
        if(sizeOfStaticVector<float>(tcdepths) < 10)
        {
            // fallback if we don't have enough valid samples over the epipolar line
            if(tcdepths != nullptr)
            {
                delete tcdepths;
                tcdepths = nullptr;
            }
            float avMinDist, avMidDist, avMaxDist;
            getPreMatchingMinMaxDepths(mp, sgmParams, avMinDist, avMidDist, avMaxDist);
            tcdepths = getDepthsByPixelSize(mp, sgmParams, avMinDist, avMidDist, avMaxDist);

            if(sizeOfStaticVector<float>(tcdepths) < 10)
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
            tCamsNew.push_back(_tile.sgmTCams.at(c));
        }
    }

    _tile.sgmTCams = tCamsNew; // update SGM T camera list

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

void SgmDepthList::getPreMatchingMinMaxDepths(const mvsUtils::MultiViewParams& mp,
                                              const SgmParams& sgmParams,
                                              float& out_minDepth,
                                              float& out_midDepth,
                                              float& out_maxDepth)
{
    if(sgmParams.prematchinMinMaxDepthDontUseSeeds)
    {
        out_minDepth = 0.0f;
        out_maxDepth = 0.0f;
        for(std::size_t c = 0; c < _tile.sgmTCams.size(); ++c)
        {
            const int tc = _tile.sgmTCams.at(c);
            out_minDepth += (mp.CArr[_tile.rc] - mp.CArr[tc]).size() * sgmParams.prematchingMinCamDist;
            out_maxDepth += (mp.CArr[_tile.rc] - mp.CArr[tc]).size() * sgmParams.prematchingMaxCamDist;
        }
        out_minDepth /= static_cast<float>(_tile.sgmTCams.size());
        out_maxDepth /= static_cast<float>(_tile.sgmTCams.size());
        out_midDepth = (out_minDepth + out_maxDepth) / 2.0f;
    }
    else
    {
        std::size_t nbDepths;
        getMinMaxMidNbDepthFromSfM(mp, sgmParams, out_minDepth, out_maxDepth, out_midDepth, nbDepths);
        out_maxDepth = out_maxDepth * sgmParams.prematchingMaxDepthScale;
    }
}

StaticVector<float>* SgmDepthList::getDepthsByPixelSize(const mvsUtils::MultiViewParams& mp,
                                                        const SgmParams& sgmParams,
                                                        float minDepth,
                                                        float midDepth,
                                                        float maxDepth)
{
    const int maxDepthsHalf = 1024;

    const float d = float(sgmParams.scale) * float(sgmParams.rcDepthsCompStep);

    OrientedPoint rcplane;
    rcplane.p = mp.CArr[_tile.rc];
    rcplane.n = mp.iRArr[_tile.rc] * Point3d(0.0, 0.0, 1.0);
    rcplane.n = rcplane.n.normalize();

    int ndepthsMidMax = 0;
    float maxdepth = midDepth;
    while((maxdepth < maxDepth) && (ndepthsMidMax < maxDepthsHalf))
    {
        Point3d p = rcplane.p + rcplane.n * maxdepth;
        float pixSize = mp.getCamPixelSize(p, _tile.rc, d);
        maxdepth += pixSize;
        ndepthsMidMax++;
    }

    int ndepthsMidMin = 0;
    float mindepth = midDepth;
    while((mindepth > minDepth) && (ndepthsMidMin < maxDepthsHalf * 2 - ndepthsMidMax))
    {
        Point3d p = rcplane.p + rcplane.n * mindepth;
        float pixSize = mp.getCamPixelSize(p, _tile.rc, d);
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
        pixSize = mp.getCamPixelSize(p, _tile.rc, d);
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
        pixSize = mp.getCamPixelSize(p, _tile.rc, d);
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
                ALICEVISION_LOG_TRACE(_tile << "getDepthsByPixelSize: check if it is asc: " << (*out)[j]);
            }
            throw std::runtime_error("getDepthsByPixelSize not asc.");
        }
    }
    return out;
}

StaticVector<float>* SgmDepthList::getDepthsTc(const mvsUtils::MultiViewParams& mp,
                                               const SgmParams& sgmParams,
                                               int tc,
                                               float midDepth)
{
    OrientedPoint rcplane;
    rcplane.p = mp.CArr[_tile.rc];
    rcplane.n = mp.iRArr[_tile.rc] * Point3d(0.0, 0.0, 1.0);
    rcplane.n = rcplane.n.normalize();

    // ROI center 
    const Point2d roiCenter((_tile.roi.x.begin + (_tile.roi.width() * 0.5)), _tile.roi.y.begin + (_tile.roi.height() * 0.5));

    // principal point of the rc camera to the tc camera
    const Point2d principalPoint(mp.getWidth(_tile.rc) * 0.5, mp.getHeight(_tile.rc) * 0.5);
    
    // reference pixel for the epipolar line
    const Point2d rMid = (!sgmParams.chooseDepthListPerTile) ? principalPoint : roiCenter;

    // segment of epipolar line
    Point2d pFromTar, pToTar; 

    {
        double zmin;
        double zmax;

        getRcTcDepthRangeFromSfM(mp, sgmParams, _tile.rc, tc, zmin, zmax);

        const Matrix3x4& rP = mp.camArr[_tile.rc];
        const Matrix3x4& tP = mp.camArr[tc];

        Point3d rC;
        Matrix3x3 rR;
        Matrix3x3 riR;
        Matrix3x3 rK;
        Matrix3x3 riK;
        Matrix3x3 riP;
        mp.decomposeProjectionMatrix(rC, rR, riR, rK, riK, riP, rP);

        Point2d tarpix1;
        Point2d tarpix2;

        mp.getPixelFor3DPoint(&tarpix1, ((riP * rMid) * zmin) + rC, tP);
        mp.getPixelFor3DPoint(&tarpix2, ((riP * rMid) * zmax) + rC, tP);

        get2dLineImageIntersection(&pFromTar, &pToTar, tarpix1, tarpix2, mp, tc);
    }

    int allDepths = static_cast<int>((pToTar - pFromTar).size());
    const int allDepthsFound = allDepths; // for debug log

    const Point2d pixelVect = ((pToTar - pFromTar).normalize()) * std::max(1.0, (double)sgmParams.scale);

    Point2d cg = Point2d(0.0, 0.0);
    Point3d cg3 = Point3d(0.0, 0.0, 0.0);
    int ncg = 0;

    // navigate through all pixels of the epilolar segment
    // Compute the middle of the valid pixels of the epipolar segment (in rc camera) of the principal point (of the rc
    // camera)
    for(int i = 0; i < allDepths; i++)
    {
        Point2d tpix = pFromTar + pixelVect * (float)i;
        Point3d p;
        if(triangulateMatch(p, rMid, tpix, _tile.rc, tc, mp)) // triangulate principal point from rc with tpix
        {
            float depth = orientedPointPlaneDistance(
                p, rcplane.p,
                rcplane.n); // todo: can compute the distance to the camera (as it's the principal point it's the same)
            if(mp.isPixelInImage(tpix, tc) && (depth > 0.0f) &&
               checkPair(p, _tile.rc, tc, mp, mp.getMinViewAngle(), mp.getMaxViewAngle()))
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
        mp.getPixelFor3DPoint(&midpoint, midPt, tc);
    }

    // compute the direction
    float direction = 1.0f;
    {
        Point3d p;
        if(!triangulateMatch(p, rMid, midpoint, _tile.rc, tc, mp))
        {
            StaticVector<float>* out = new StaticVector<float>();
            return out;
        }

        float depth = orientedPointPlaneDistance(p, rcplane.p, rcplane.n);

        if(!triangulateMatch(p, rMid, midpoint + pixelVect, _tile.rc, tc, mp))
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
    out1->reserve(2 * sgmParams.rcTcDepthsHalfLimit);

    Point2d tpix = midpoint;
    float depthOld = -1.0f;
    int istep = 0;
    bool ok = true;

    // compute depths for all pixels from the middle point to on one side of the epipolar line
    while((out1->size() < sgmParams.rcTcDepthsHalfLimit) && (mp.isPixelInImage(tpix, tc) == true) && (ok == true))
    {
        tpix = tpix + pixelVect * direction;

        Point3d refvect = mp.iCamArr[_tile.rc] * rMid;
        Point3d tarvect = mp.iCamArr[tc] * tpix;
        float rptpang = angleBetwV1andV2(refvect, tarvect);

        Point3d p;
        ok = triangulateMatch(p, rMid, tpix, _tile.rc, tc, mp);

        float depth = orientedPointPlaneDistance(p, rcplane.p, rcplane.n);
        if(mp.isPixelInImage(tpix, tc) && (depth > 0.0f) && (depth > depthOld) &&
           checkPair(p, _tile.rc, tc, mp, mp.getMinViewAngle(), mp.getMaxViewAngle()) &&
           (rptpang >
            mp.getMinViewAngle()) // WARNING if vects are near parallel thaen this results to strange angles ...
           &&
           (rptpang <
            mp.getMaxViewAngle())) // this is the propper angle ... beacause is does not depend on the triangluated p
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
    out2->reserve(2 * sgmParams.rcTcDepthsHalfLimit);
    tpix = midpoint;
    istep = 0;
    ok = true;

    // compute depths for all pixels from the middle point to the other side of the epipolar line
    while((out2->size() < sgmParams.rcTcDepthsHalfLimit) && (mp.isPixelInImage(tpix, tc) == true) && (ok == true))
    {
        const Point3d refvect = mp.iCamArr[_tile.rc] * rMid;
        const Point3d tarvect = mp.iCamArr[tc] * tpix;
        const float rptpang = angleBetwV1andV2(refvect, tarvect);

        Point3d p;
        ok = triangulateMatch(p, rMid, tpix, _tile.rc, tc, mp);

        float depth = orientedPointPlaneDistance(p, rcplane.p, rcplane.n);
        if(mp.isPixelInImage(tpix, tc) && (depth > 0.0f) && (depth < depthOld) &&
           checkPair(p, _tile.rc, tc, mp, mp.getMinViewAngle(), mp.getMaxViewAngle()) &&
           (rptpang > mp.getMinViewAngle()) // WARNING if vects are near parallel thaen this results to strange angles ...
           && (rptpang < mp.getMaxViewAngle())) // this is the propper angle ... beacause is does not depend on the triangluated p
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
    out->reserve(2 * sgmParams.rcTcDepthsHalfLimit);
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
                ALICEVISION_LOG_TRACE(_tile << "getDepthsRcTc: check if it is asc: " << (*out)[j]);
            }
            ALICEVISION_LOG_WARNING(_tile << "getDepthsRcTc: not asc.");

            if(out->size() > 1)
            {
                qsort(&(*out)[0], out->size(), sizeof(float), qSortCompareFloatAsc);
            }
        }
    }

    ALICEVISION_LOG_DEBUG(_tile << "Find depths over the epipolar line segment between R and T cameras:" << std::endl
                                << "\t- rc: " << _tile.rc << std::endl
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
