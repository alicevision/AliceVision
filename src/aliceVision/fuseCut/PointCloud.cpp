// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "PointCloud.hpp"

#include <aliceVision/fuseCut/Fuser.hpp>
#include <aliceVision/mvsUtils/mapIO.hpp>
#include <aliceVision/mvsUtils/fileIO.hpp>
#include <aliceVision/mvsUtils/common.hpp>
#include <aliceVision/mvsData/geometry.hpp>

#include <aliceVision/fuseCut/Kdtree.hpp>

#include <geogram/mesh/mesh.h>
#include <geogram/basic/geometry_nd.h>

namespace aliceVision {
namespace fuseCut {

namespace fs = std::filesystem;

/// Filter by pixSize
void filterByPixSize(const std::vector<Point3d>& verticesCoordsPrepare,
                     std::vector<double>& pixSizePrepare,
                     double pixSizeMarginCoef,
                     std::vector<float>& simScorePrepare)
{

    ALICEVISION_LOG_INFO("Build nanoflann KdTree index.");
    PointVectorAdaptator pointCloudRef(verticesCoordsPrepare);
    KdTree kdTree(3 /*dim*/, pointCloudRef, nanoflann::KDTreeSingleIndexAdaptorParams(MAX_LEAF_ELEMENTS));
    kdTree.buildIndex();

    ALICEVISION_LOG_INFO("KdTree created for " << verticesCoordsPrepare.size() << " points.");

#pragma omp parallel for
    for (int vIndex = 0; vIndex < verticesCoordsPrepare.size(); ++vIndex)
    {
        if (pixSizePrepare[vIndex] == -1.0)
        {
            continue;
        }
        const double pixSizeScore = pixSizeMarginCoef * simScorePrepare[vIndex] * pixSizePrepare[vIndex] * pixSizePrepare[vIndex];
        if (pixSizeScore < std::numeric_limits<double>::epsilon())
        {
            pixSizePrepare[vIndex] = -1.0;
            continue;
        }


        static const nanoflann::SearchParameters searchParams(0.f, false);  // false: dont need to sort
        SmallerPixSizeInRadius<double, std::size_t> resultSet(pixSizeScore, pixSizePrepare, simScorePrepare, vIndex);
        kdTree.findNeighbors(resultSet, verticesCoordsPrepare[vIndex].m, searchParams);
        if (resultSet.found)
            pixSizePrepare[vIndex] = -1.0;

    }
    ALICEVISION_LOG_INFO("Filtering done.");
}

/// Remove invalid points based on invalid pixSize
void removeInvalidPoints(std::vector<Point3d>& verticesCoordsPrepare, std::vector<double>& pixSizePrepare, std::vector<float>& simScorePrepare)
{
    std::vector<Point3d> verticesCoordsTmp;
    verticesCoordsTmp.reserve(verticesCoordsPrepare.size());
    std::vector<double> pixSizeTmp;
    pixSizeTmp.reserve(pixSizePrepare.size());
    std::vector<float> simScoreTmp;
    simScoreTmp.reserve(simScorePrepare.size());
    for (int i = 0; i < verticesCoordsPrepare.size(); ++i)
    {
        if (pixSizePrepare[i] != -1.0)
        {
            verticesCoordsTmp.push_back(verticesCoordsPrepare[i]);
            pixSizeTmp.push_back(pixSizePrepare[i]);
            simScoreTmp.push_back(simScorePrepare[i]);
        }
    }
    ALICEVISION_LOG_INFO((verticesCoordsPrepare.size() - verticesCoordsTmp.size()) << " invalid points removed.");
    verticesCoordsPrepare.swap(verticesCoordsTmp);
    pixSizePrepare.swap(pixSizeTmp);
    simScorePrepare.swap(simScoreTmp);
}

void removeInvalidPoints(std::vector<Point3d>& verticesCoordsPrepare,
                         std::vector<double>& pixSizePrepare,
                         std::vector<float>& simScorePrepare,
                         std::vector<GC_vertexInfo>& verticesAttrPrepare)
{
    std::vector<Point3d> verticesCoordsTmp;
    verticesCoordsTmp.reserve(verticesCoordsPrepare.size());
    std::vector<double> pixSizeTmp;
    pixSizeTmp.reserve(pixSizePrepare.size());
    std::vector<float> simScoreTmp;
    simScoreTmp.reserve(simScorePrepare.size());
    std::vector<GC_vertexInfo> verticesAttrTmp;
    verticesAttrTmp.reserve(verticesAttrPrepare.size());
    for (int i = 0; i < verticesCoordsPrepare.size(); ++i)
    {
        if (pixSizePrepare[i] != -1.0)
        {
            verticesCoordsTmp.push_back(verticesCoordsPrepare[i]);
            pixSizeTmp.push_back(pixSizePrepare[i]);
            simScoreTmp.push_back(simScorePrepare[i]);
            verticesAttrTmp.push_back(verticesAttrPrepare[i]);
        }
    }
    ALICEVISION_LOG_INFO((verticesCoordsPrepare.size() - verticesCoordsTmp.size()) << " invalid points removed.");
    verticesCoordsPrepare.swap(verticesCoordsTmp);
    pixSizePrepare.swap(pixSizeTmp);
    simScorePrepare.swap(simScoreTmp);
    verticesAttrPrepare.swap(verticesAttrTmp);
}

void createVerticesWithVisibilities(const StaticVector<int>& cams,
                                    std::vector<Point3d>& verticesCoordsPrepare,
                                    std::vector<double>& pixSizePrepare,
                                    std::vector<float>& simScorePrepare,
                                    std::vector<GC_vertexInfo>& verticesAttrPrepare,
                                    mvsUtils::MultiViewParams& mp,
                                    float simFactor,
                                    float voteMarginFactor,
                                    float contributeMarginFactor,
                                    float simGaussianSize)
{

    PointVectorAdaptator pointCloudRef(verticesCoordsPrepare);
    KdTree kdTree(3 /*dim*/, pointCloudRef, nanoflann::KDTreeSingleIndexAdaptorParams(MAX_LEAF_ELEMENTS));
    kdTree.buildIndex();
    ALICEVISION_LOG_INFO("NANOFLANN: KdTree created.");


    std::vector<omp_lock_t> locks(verticesCoordsPrepare.size());
    for (auto& lock : locks)
        omp_init_lock(&lock);

    omp_set_nested(1);
#pragma omp parallel for num_threads(3)
    for (int c = 0; c < cams.size(); ++c)
    {
        ALICEVISION_LOG_INFO("Create visibilities (" << c << "/" << cams.size() << ")");
        image::Image<float> depthMap;
        image::Image<float> simMap;
        const int width = mp.getWidth(c);
        const int height = mp.getHeight(c);

        // read depth map
        mvsUtils::readMap(c, mp, mvsUtils::EFileType::depthMapFiltered, depthMap);

        if (depthMap.size() <= 0)
        {
            ALICEVISION_LOG_WARNING("Empty depth map (cam id: " << c << ")");
            continue;
        }

        // read similarity map
        try
        {
            mvsUtils::readMap(c, mp, mvsUtils::EFileType::simMapFiltered, simMap);
            image::Image<float> simMapTmp(simMap.width(), simMap.height());
            imageAlgo::convolveImage(simMap, simMapTmp, "gaussian", simGaussianSize, simGaussianSize);
            simMap.swap(simMapTmp);
        }
        catch (const std::exception& e)
        {
            ALICEVISION_LOG_WARNING("Cannot find similarity map file.");
            simMap.resize(width * height, -1);
        }

// Add visibility
#pragma omp parallel for
        for (int y = 0; y < depthMap.height(); ++y)
        {
            for (int x = 0; x < depthMap.width(); ++x)
            {
                const std::size_t index = y * depthMap.width() + x;
                const float depth = depthMap(index);
                if (depth <= 0.0f)
                    continue;

                const Point3d p = mp.backproject(c, Point2d(x, y), depth);
                const double pixSize = mp.getCamPixelSize(p, c);

                nanoflann::KNNResultSet<double, std::size_t> resultSet(1);
                std::size_t nearestVertexIndex = std::numeric_limits<std::size_t>::max();
                double dist = std::numeric_limits<double>::max();
                resultSet.init(&nearestVertexIndex, &dist);
                if (!kdTree.findNeighbors(resultSet, p.m, nanoflann::SearchParameters()))
                {
                    ALICEVISION_LOG_TRACE("Failed to find Neighbors.");
                    continue;
                }

                const float pixSizeScoreI = simScorePrepare[nearestVertexIndex] * pixSize * pixSize;
                const float pixSizeScoreV =
                  simScorePrepare[nearestVertexIndex] * pixSizePrepare[nearestVertexIndex] * pixSizePrepare[nearestVertexIndex];

                if (dist < voteMarginFactor * std::max(pixSizeScoreI, pixSizeScoreV))
                {
                    GC_vertexInfo& va = verticesAttrPrepare[nearestVertexIndex];
                    Point3d& vc = verticesCoordsPrepare[nearestVertexIndex];
                    const float simValue = simMap(index);
                    // remap similarity values from [-1;+1] to [+1;+simFactor]
                    // interpretation is [goodSimilarity;badSimilarity]
                    const float simScore = simValue < -1.0f ? 1.0f : 1.0f + (1.0f + simValue) * simFactor;

                    // Custom locks to limit it to the index: nearestVertexIndex
                    // to avoid using "omp critical"
                    omp_lock_t* lock = &locks[nearestVertexIndex];
                    omp_set_lock(lock);
                    {
                        va.cams.push_back_distinct(c);
                        if (dist < contributeMarginFactor * pixSizeScoreV)
                        {
                            vc = (vc * (double)va.nrc + p) / double(va.nrc + 1);

                            va.nrc += 1;
                        }
                    }
                    omp_unset_lock(lock);
                }
            }
        }
    }
    omp_set_nested(0);

    for (auto& lock : locks)
        omp_destroy_lock(&lock);

// compute pixSize
#pragma omp parallel for
    for (int vi = 0; vi < verticesAttrPrepare.size(); ++vi)
    {
        GC_vertexInfo& v = verticesAttrPrepare[vi];
        v.pixSize = mp.getCamsMinPixelSize(verticesCoordsPrepare[vi], v.cams);
    }

    ALICEVISION_LOG_INFO("Visibilities created.");
}


PointCloud::PointCloud(mvsUtils::MultiViewParams& mp) : _mp(mp)
{
    _camsVertexes.resize(_mp.ncams, -1);
}

void PointCloud::fuseFromDepthMaps(const StaticVector<int>& cams, const Point3d voxel[8], const PointCloudFuseParams& params)
{
     ALICEVISION_LOG_INFO("fuseFromDepthMaps, maxVertices: " << params.maxPoints);

    // Load depth from depth maps, select points per depth maps (1 value per tile).
    // Filter points inside other points (with a volume defined by the pixelSize)
    // If too much points at the end, increment a coefficient factor on the pixel size
    // and iterate to fuse points until we get the right amount of points.

    const unsigned long nbValidDepths = computeNumberOfAllPoints(_mp, _mp.getProcessDownscale());
    ALICEVISION_LOG_INFO("Number of all valid depths in input depth maps: " << nbValidDepths);
    std::size_t nbPixels = 0;
    for (const auto& imgParams : _mp.getImagesParams())
    {
        nbPixels += imgParams.size;
    }
    ALICEVISION_LOG_INFO("Number of pixels from all input images: " << nbPixels);
    int step = std::floor(std::sqrt(double(nbPixels) / double(params.maxInputPoints)));
    step = std::max(step, params.minStep);
    std::size_t realMaxVertices = 0;
    std::vector<int> startIndex(_mp.getNbCameras(), 0);
    for (int i = 0; i < _mp.getNbCameras(); ++i)
    {
        const auto& imgParams = _mp.getImageParams(i);
        startIndex[i] = realMaxVertices;
        realMaxVertices += divideRoundUp(imgParams.width, step) * divideRoundUp(imgParams.height, step);
    }
    std::vector<Point3d> verticesCoordsPrepare(realMaxVertices);
    std::vector<double> pixSizePrepare(realMaxVertices);
    std::vector<float> simScorePrepare(realMaxVertices);

    // counter for filtered points
    int minVisCounter = 0;
    int minAngleCounter = 0;

    ALICEVISION_LOG_INFO("simFactor: " << params.simFactor);
    ALICEVISION_LOG_INFO("maxVertices: " << params.maxPoints);
    ALICEVISION_LOG_INFO("step: " << step << " (minStep: " << params.minStep << ")");
    ALICEVISION_LOG_INFO("realMaxVertices: " << realMaxVertices);
    ALICEVISION_LOG_INFO("minVis: " << params.minVis);

    ALICEVISION_LOG_INFO("Load depth maps and add points.");
    {
        omp_set_nested(1);
#pragma omp parallel for num_threads(3)
        for (int c = 0; c < cams.size(); c++)
        {
            image::Image<float> depthMap;
            image::Image<float> simMap;
            image::Image<unsigned char> numOfModalsMap;

            const int width = _mp.getWidth(c);
            const int height = _mp.getHeight(c);

            {
                // read depth map
                mvsUtils::readMap(c, _mp, mvsUtils::EFileType::depthMapFiltered, depthMap);

                if (depthMap.size() <= 0)
                {
                    ALICEVISION_LOG_WARNING("Empty depth map (cam id: " << c << ")");
                    continue;
                }

                // read similarity map
                try
                {
                    mvsUtils::readMap(c, _mp, mvsUtils::EFileType::simMapFiltered, simMap);
                    image::Image<float> simMapTmp;
                    imageAlgo::convolveImage(simMap, simMapTmp, "gaussian", params.simGaussianSizeInit, params.simGaussianSizeInit);
                    simMap.swap(simMapTmp);
                }
                catch (const std::exception& e)
                {
                    ALICEVISION_LOG_WARNING("simMap file can't be found.");
                    simMap.resize(width, height, true, -1);
                }

                // read nmod map
                int wTmp, hTmp;
                const std::string nmodMapFilepath = getFileNameFromIndex(_mp, c, mvsUtils::EFileType::nmodMap);
                // If we have an nModMap in input (from depthmapfilter) use it,
                // else init with a constant value.
                if (fs::exists(nmodMapFilepath))
                {
                    image::readImage(nmodMapFilepath, numOfModalsMap, image::EImageColorSpace::NO_CONVERSION);
                    if (numOfModalsMap.width() != width || numOfModalsMap.height() != height)
                        throw std::runtime_error("Wrong nmod map dimensions: " + nmodMapFilepath);
                }
                else
                {
                    ALICEVISION_LOG_WARNING("nModMap file can't be found: " << nmodMapFilepath);
                    numOfModalsMap.resize(width, height, true, 1);
                }
            }

            const int syMax = divideRoundUp(height, step);
            const int sxMax = divideRoundUp(width, step);
#pragma omp parallel for
            for (int sy = 0; sy < syMax; ++sy)
            {
                for (int sx = 0; sx < sxMax; ++sx)
                {
                    const int index = startIndex[c] + sy * sxMax + sx;
                    float bestDepth = std::numeric_limits<float>::max();
                    float bestScore = 0;
                    float bestSimScore = 0;
                    int bestX = 0;
                    int bestY = 0;
                    for (int y = sy * step, ymax = std::min((sy + 1) * step, height); y < ymax; ++y)
                    {
                        for (int x = sx * step, xmax = std::min((sx + 1) * step, width); x < xmax; ++x)
                        {
                            const std::size_t index = y * width + x;
                            const float depth = depthMap(index);
                            if (depth <= 0.0f)
                                continue;

                            int numOfModals = 0;
                            const int scoreKernelSize = 1;
                            for (int ly = std::max(y - scoreKernelSize, 0), lyMax = std::min(y + scoreKernelSize, height - 1); ly < lyMax; ++ly)
                            {
                                for (int lx = std::max(x - scoreKernelSize, 0), lxMax = std::min(x + scoreKernelSize, width - 1); lx < lxMax; ++lx)
                                {
                                    if (depthMap(ly * width + lx) > 0.0f)
                                    {
                                        numOfModals += 10 + int(numOfModalsMap(ly * width + lx));
                                    }
                                }
                            }
                            float sim = simMap(index);
                            sim = sim < 0.0f ? 0.0f : sim;  // clamp values < 0
                            // remap similarity values from [-1;+1] to [+1;+simScale]
                            // interpretation is [goodSimilarity;badSimilarity]
                            const float simScore = 1.0f + sim * params.simFactor;

                            const float score = numOfModals + (1.0f / simScore);
                            if (score > bestScore)
                            {
                                bestDepth = depth;
                                bestScore = score;
                                bestSimScore = simScore;
                                bestX = x;
                                bestY = y;
                            }
                        }
                    }
                    if (bestScore < 3 * 13)
                    {
                        // discard the point
                        pixSizePrepare[index] = -1.0;
                    }
                    else
                    {
                        const Point3d p = _mp.CArr[c] + (_mp.iCamArr[c] * Point2d((float)bestX, (float)bestY)).normalize() * bestDepth;

                        // TODO: isPointInHexahedron: here or in the previous loop per pixel to not loose point?
                        if (voxel == nullptr || mvsUtils::isPointInHexahedron(p, voxel))
                        {
                            verticesCoordsPrepare[index] = p;
                            simScorePrepare[index] = bestSimScore;
                            pixSizePrepare[index] = _mp.getCamPixelSize(p, c);
                        }
                        else
                        {
                            // discard the point
                            // verticesCoordsPrepare[index] = p;
                            pixSizePrepare[index] = -1.0;
                        }
                    }
                }
            }
        }
        omp_set_nested(0);
    }

    ALICEVISION_LOG_INFO("Filter initial 3D points by pixel size to remove duplicates.");

    filterByPixSize(verticesCoordsPrepare, pixSizePrepare, params.pixSizeMarginInitCoef, simScorePrepare);
    // remove points if pixSize == -1
    removeInvalidPoints(verticesCoordsPrepare, pixSizePrepare, simScorePrepare);

    ALICEVISION_LOG_INFO("3D points loaded and filtered to " << verticesCoordsPrepare.size() << " points.");

    ALICEVISION_LOG_INFO("Init visibilities to compute angle scores");
    std::vector<GC_vertexInfo> verticesAttrPrepare(verticesCoordsPrepare.size());

    // Compute the vertices positions and simScore from all input depthMap/simMap images,
    // and declare the visibility information (the cameras indexes seeing the vertex).
    createVerticesWithVisibilities(cams,
                                   verticesCoordsPrepare,
                                   pixSizePrepare,
                                   simScorePrepare,
                                   verticesAttrPrepare,
                                   _mp,
                                   params.simFactor,
                                   params.voteMarginFactor,
                                   params.contributeMarginFactor,
                                   params.simGaussianSize);

    ALICEVISION_LOG_INFO("Compute max angle per point");

    ALICEVISION_LOG_INFO("angleFactor: " << params.angleFactor);
    // Compute max visibility angle per point
    // and weight simScore with angular score
#if defined(FUSE_COMPUTE_ANGLE_STATS) && !defined(OMP_HAVE_MIN_MAX_REDUCTION)
    ALICEVISION_LOG_DEBUG("Disable angle stats computation: OpenMP does not provide required min/max reduction clauses.");
    #undef FUSE_COMPUTE_ANGLE_STATS
#endif

#ifdef FUSE_COMPUTE_ANGLE_STATS
    double stat_minAngle = std::numeric_limits<double>::max(), stat_maxAngle = 0.0;
    double stat_minAngleScore = std::numeric_limits<double>::max(), stat_maxAngleScore = 0.0;
    #pragma omp parallel for reduction(max : stat_maxAngle, stat_maxAngleScore) reduction(min : stat_minAngle, stat_minAngleScore)
#else
    #pragma omp parallel for
#endif
    for (int vIndex = 0; vIndex < verticesCoordsPrepare.size(); ++vIndex)
    {
        if (pixSizePrepare[vIndex] == -1.0)
        {
            continue;
        }
        const std::vector<int>& visCams = verticesAttrPrepare[vIndex].cams.getData();
        if (visCams.empty())
        {
            ALICEVISION_LOG_WARNING("BAD: visCams.empty()");
        }
        double maxAngle = 0.0;
        for (int i : visCams)
        {
            for (int j : visCams)
            {
                if (i == j)
                    continue;
                double angle = angleBetwABandAC(verticesCoordsPrepare[vIndex], _mp.CArr[i], _mp.CArr[j]);
                maxAngle = std::max(angle, maxAngle);
            }
        }
        // Kill the point if the angle is too small
        if (maxAngle < params.minAngleThreshold)
        {
            pixSizePrepare[vIndex] = -1;
            minAngleCounter += 1;
            continue;
        }
        // Filter points based on their number of observations
        if (visCams.size() < params.minVis)
        {
            pixSizePrepare[vIndex] = -1;
            minVisCounter += 1;
            continue;
        }

        const double angleScore = 1.0 + params.angleFactor / maxAngle;
        // Combine angleScore with simScore
        simScorePrepare[vIndex] = simScorePrepare[vIndex] * angleScore;

#ifdef FUSE_COMPUTE_ANGLE_STATS
        stat_minAngle = std::min(stat_minAngle, maxAngle);
        stat_maxAngle = std::max(stat_maxAngle, maxAngle);

        stat_minAngleScore = std::min(stat_minAngleScore, angleScore);
        stat_maxAngleScore = std::max(stat_maxAngleScore, angleScore);
#endif
    }
#ifdef FUSE_COMPUTE_ANGLE_STATS
    ALICEVISION_LOG_INFO("Angle min: " << stat_minAngle << ", max: " << stat_maxAngle << ".");
    ALICEVISION_LOG_INFO("Angle score min: " << stat_minAngleScore << ", max: " << stat_maxAngleScore << ".");
#endif
    ALICEVISION_LOG_INFO(minAngleCounter << " filtered points due to low angle of observations (minAngleThreshold=" << params.minAngleThreshold
                                         << "). ");
    ALICEVISION_LOG_INFO(minVisCounter << " filtered points due to low number of observations (minVis=" << params.minVis << "). ");
    removeInvalidPoints(verticesCoordsPrepare, pixSizePrepare, simScorePrepare, verticesAttrPrepare);

    ALICEVISION_LOG_INFO("Filter by angle score and sim score");

    // while more points than the max points (with a limit to 20 iterations).
    double pixSizeMarginFinalCoef = params.pixSizeMarginFinalCoef;
    for (int filteringIt = 0; filteringIt < 20; ++filteringIt)
    {
        ALICEVISION_LOG_INFO("Filter index: " << filteringIt << ", pixSizeMarginFinalCoef: " << pixSizeMarginFinalCoef);
        // Filter points with new simScore
        filterByPixSize(verticesCoordsPrepare, pixSizePrepare, pixSizeMarginFinalCoef, simScorePrepare);

        ALICEVISION_LOG_INFO("Remove invalid points.");
        removeInvalidPoints(verticesCoordsPrepare, pixSizePrepare, simScorePrepare, verticesAttrPrepare);

        if (verticesCoordsPrepare.size() < params.maxPoints)
        {
            ALICEVISION_LOG_INFO("The number of points is below the max number of vertices.");
            break;
        }
        else
        {
            pixSizeMarginFinalCoef *= 1.5;
            ALICEVISION_LOG_INFO("Increase pixel size margin coef to " << pixSizeMarginFinalCoef << ", nb points: " << verticesCoordsPrepare.size()
                                                                       << ", maxVertices: " << params.maxPoints);
        }
    }
    ALICEVISION_LOG_INFO("3D points loaded and filtered to " << verticesCoordsPrepare.size() << " points (maxVertices is " << params.maxPoints
                                                             << ").");

    if (params.refineFuse)
    {
        ALICEVISION_LOG_INFO("Create final visibilities");
        // Initialize the vertice attributes and declare the visibility information
        createVerticesWithVisibilities(cams,
                                       verticesCoordsPrepare,
                                       pixSizePrepare,
                                       simScorePrepare,
                                       verticesAttrPrepare,
                                       _mp,
                                       params.simFactor,
                                       params.voteMarginFactor,
                                       params.contributeMarginFactor,
                                       params.simGaussianSize);
    }

    if (verticesCoordsPrepare.empty())
        throw std::runtime_error("Depth map fusion gives an empty result.");

    ALICEVISION_LOG_WARNING("fuseFromDepthMaps done: " << verticesCoordsPrepare.size() << " points created.");

    // Insert the new elements
    if (_verticesCoords.empty())
    {
        // replace with the new points if emoty
        _verticesCoords.swap(verticesCoordsPrepare);
        _verticesAttr.swap(verticesAttrPrepare);
    }
    else
    {
        // concatenate the new elements with the previous ones
        _verticesCoords.insert(_verticesCoords.end(), verticesCoordsPrepare.begin(), verticesCoordsPrepare.end());
        _verticesAttr.insert(_verticesAttr.end(), verticesAttrPrepare.begin(), verticesAttrPrepare.end());
    }
}

void PointCloud::addPointsFromSfM(const Point3d hexah[8], const StaticVector<int>& cams, const sfmData::SfMData& sfmData)
{
    const std::size_t nbPoints = sfmData.getLandmarks().size();
    const std::size_t verticesOffset = _verticesCoords.size();
    const float forcePixelSize = (float)_mp.userParams.get<double>("LargeScale.forcePixelSize", -1);

    _verticesCoords.resize(verticesOffset + nbPoints);
    _verticesAttr.resize(verticesOffset + nbPoints);

    sfmData::Landmarks::const_iterator landmarkIt = sfmData.getLandmarks().begin();
    std::vector<Point3d>::iterator vCoordsIt = _verticesCoords.begin();
    std::vector<GC_vertexInfo>::iterator vAttrIt = _verticesAttr.begin();

    std::advance(vCoordsIt, verticesOffset);
    std::advance(vAttrIt, verticesOffset);

    std::size_t addedPoints = 0;
    for (std::size_t i = 0; i < nbPoints; ++i)
    {
        const sfmData::Landmark& landmark = landmarkIt->second;
        const Point3d p(landmark.X(0), landmark.X(1), landmark.X(2));

        if (mvsUtils::isPointInHexahedron(p, hexah))
        {
            *vCoordsIt = p;

            vAttrIt->nrc = landmark.getObservations().size();
            vAttrIt->cams.reserve(vAttrIt->nrc);

            for (const auto& observationPair : landmark.getObservations())
                vAttrIt->cams.push_back(_mp.getIndexFromViewId(observationPair.first));

            vAttrIt->pixSize = _mp.getCamsMinPixelSize(p, vAttrIt->cams);
            if (forcePixelSize > 0)
            {
                vAttrIt->pixSize = forcePixelSize;
            }

            ++vCoordsIt;
            ++vAttrIt;
            ++addedPoints;
        }
        ++landmarkIt;
    }
    if (addedPoints != nbPoints)
    {
        _verticesCoords.resize(verticesOffset + addedPoints);
        _verticesAttr.resize(verticesOffset + addedPoints);
    }
    ALICEVISION_LOG_WARNING("Add " << addedPoints << " new points for the SfM.");
}

void PointCloud::addPointsFromCameraCenters(const StaticVector<int>& cams, float minDist)
{
    int addedPoints = 0;
    // Note: For now, we skip the check of collision between cameras and data points to avoid useless computation.
    // const float minDist2 = minDist * minDist;
    // Tree kdTree(_verticesCoords);
    for (int camid = 0; camid < cams.size(); camid++)
    {
        int rc = cams[camid];
        {
            const Point3d p(_mp.CArr[rc].x, _mp.CArr[rc].y, _mp.CArr[rc].z);
            
            const GEO::index_t nvi = _verticesCoords.size();
            _verticesCoords.push_back(p);

            GC_vertexInfo newv;
            newv.nrc = 0;

            _camsVertexes[rc] = nvi;

            _verticesAttr.push_back(newv);
            ++addedPoints;
        }
    }
    ALICEVISION_LOG_WARNING("Add " << addedPoints << " new points for the " << cams.size() << " cameras centers.");
}

void PointCloud::addPointsToPreventSingularities(const Point3d voxel[8], float minDist)
{
    Point3d vcg = (voxel[0] + voxel[1] + voxel[2] + voxel[3] + voxel[4] + voxel[5] + voxel[6] + voxel[7]) / 8.0;
    Point3d extrPts[6];
    Point3d fcg;
    const double s = 0.25;
    fcg = (voxel[0] + voxel[1] + voxel[2] + voxel[3]) * 0.25;  // facet center
    extrPts[0] = fcg + (fcg - vcg) * s;                        // extra point beyond the bbox facet
    fcg = (voxel[0] + voxel[4] + voxel[7] + voxel[3]) * 0.25;
    extrPts[1] = fcg + (fcg - vcg) * s;
    fcg = (voxel[0] + voxel[1] + voxel[5] + voxel[4]) * 0.25;
    extrPts[2] = fcg + (fcg - vcg) * s;
    fcg = (voxel[4] + voxel[5] + voxel[6] + voxel[7]) * 0.25;
    extrPts[3] = fcg + (fcg - vcg) * s;
    fcg = (voxel[1] + voxel[5] + voxel[6] + voxel[2]) * 0.25;
    extrPts[4] = fcg + (fcg - vcg) * s;
    fcg = (voxel[3] + voxel[2] + voxel[6] + voxel[7]) * 0.25;
    extrPts[5] = fcg + (fcg - vcg) * s;

    int addedPoints = 0;
    for (int i = 0; i < 6; ++i)
    {
        const Point3d p(extrPts[i].x, extrPts[i].y, extrPts[i].z);

        // Note: As we create points outside of the bbox, we do not check collisions with data points.
        {
            _verticesCoords.push_back(p);
            GC_vertexInfo newv;
            newv.nrc = 0;

            _verticesAttr.push_back(newv);
            ++addedPoints;
        }
    }
    ALICEVISION_LOG_WARNING("Add " << addedPoints << " points to prevent singularities");
}

void PointCloud::densifyWithHelperPoints(int nbFront, int nbBack, double scale)
{
    if (nbFront <= 0 && nbBack <= 0)
        return;

    const std::size_t nbInputVertices = _verticesCoords.size();

    std::vector<Point3d> newHelperPoints;
    newHelperPoints.reserve((nbFront + nbBack) * nbInputVertices);

    for (std::size_t vi = 0; vi < nbInputVertices; ++vi)
    {
        const Point3d& v = _verticesCoords[vi];
        const GC_vertexInfo& vAttr = _verticesAttr[vi];

        if (vAttr.cams.empty() || vAttr.pixSize <= std::numeric_limits<float>::epsilon())
            continue;

        Point3d mainCamDir;
        for (int camId : vAttr.cams)
        {
            const Point3d& cam = _mp.CArr[camId];
            const Point3d d = (cam - v).normalize();
            mainCamDir += d;
        }
        mainCamDir /= double(vAttr.cams.size());
        mainCamDir = mainCamDir.normalize() * vAttr.pixSize;

        for (int iFront = 1; iFront < nbFront + 1; ++iFront)
            newHelperPoints.push_back(v + mainCamDir * iFront * scale);
        for (int iBack = 1; iBack < nbBack + 1; ++iBack)
            newHelperPoints.push_back(v - mainCamDir * iBack * scale);
    }
    _verticesCoords.resize(nbInputVertices + newHelperPoints.size());
    _verticesAttr.resize(nbInputVertices + newHelperPoints.size());
    for (std::size_t vi = 0; vi < newHelperPoints.size(); ++vi)
    {
        _verticesCoords[nbInputVertices + vi] = newHelperPoints[vi];
        // GC_vertexInfo& vAttr = _verticesAttr[nbInputVertices + vi];
        // Keep vertexInfo with default/empty values, so they will be removed at the end as other helper points
        // vAttr.nrc = 0;
    }

    ALICEVISION_LOG_WARNING("Densify the " << nbInputVertices << " vertices with " << newHelperPoints.size() << " new helper points.");
}

void PointCloud::addGridHelperPoints(int helperPointsGridSize, const Point3d voxel[8], float minDist)
{
    if (helperPointsGridSize <= 0)
        return;

    ALICEVISION_LOG_INFO("Add grid helper points.");
    const int ns = helperPointsGridSize;
    const Point3d vx = (voxel[1] - voxel[0]);
    const Point3d vy = (voxel[3] - voxel[0]);
    const Point3d vz = (voxel[4] - voxel[0]);

    // Add uniform noise on helper points, with 1/8 margin around the vertex.
    const double md = 1.0 / (helperPointsGridSize * 8.0);
    const Point3d maxNoiseSize(md * (voxel[1] - voxel[0]).size(), md * (voxel[3] - voxel[0]).size(), md * (voxel[4] - voxel[0]).size());
    Point3d center = (voxel[0] + voxel[1] + voxel[2] + voxel[3] + voxel[4] + voxel[5] + voxel[6] + voxel[7]) / 8.0;

    const unsigned int seed = (unsigned int)_mp.userParams.get<unsigned int>("delaunaycut.seed", 0);
    std::mt19937 generator(seed != 0 ? seed : std::random_device{}());
    auto rand = std::bind(std::uniform_real_distribution<float>{-1.0, 1.0}, generator);

    const double minDist2 = minDist * minDist;
    ALICEVISION_LOG_TRACE("GRID: Prepare kdtree");
    Tree kdTree(_verticesCoords);

    ALICEVISION_LOG_TRACE("GRID: Allocate vertices");
    std::vector<Point3d> gridVerticesCoords(std::pow(float(ns + 1), 3.f));
    std::vector<bool> valid(gridVerticesCoords.size());

    ALICEVISION_LOG_TRACE("Create helper points.");
#pragma omp parallel for
    for (int x = 0; x <= ns; ++x)
    {
        for (int y = 0; y <= ns; ++y)
        {
            for (int z = 0; z <= ns; ++z)
            {
                int i = x * (ns + 1) * (ns + 1) + y * (ns + 1) + z;
                const Point3d pt = voxel[0] + vx * ((double)x / double(ns)) + vy * ((double)y / double(ns)) + vz * ((double)z / double(ns));
                const Point3d noise(maxNoiseSize.x * rand(), maxNoiseSize.y * rand(), maxNoiseSize.z * rand());
                const Point3d p = pt + noise;
                std::size_t vi{};
                double sq_dist{};

                // if there is no nearest vertex or the nearest vertex is not too close
                if (!kdTree.locateNearestVertex(p, vi, sq_dist) || (sq_dist > minDist2))
                {
                    gridVerticesCoords[i] = p;
                    valid[i] = true;
                }
                else
                {
                    valid[i] = false;
                }
            }
        }
    }
    ALICEVISION_LOG_TRACE("Insert helper points.");
    _verticesCoords.reserve(_verticesCoords.size() + gridVerticesCoords.size());
    _verticesAttr.reserve(_verticesAttr.size() + gridVerticesCoords.size());
    int addedPoints = 0;
    for (int i = 0; i < gridVerticesCoords.size(); ++i)
    {
        if (!valid[i])
            continue;
        _verticesCoords.push_back(gridVerticesCoords[i]);
        GC_vertexInfo newv;
        newv.nrc = 0;
        _verticesAttr.push_back(newv);
        ++addedPoints;
    }

    ALICEVISION_LOG_WARNING("Add " << addedPoints << " new helper points for a 3D grid of " << ns << "x" << ns << "x" << ns << " ("
                                   << std::pow(float(ns + 1), 3.f) << " points).");
}

void PointCloud::addMaskHelperPoints(const Point3d voxel[8], const StaticVector<int>& cams, const PointCloudFuseParams& params)
{
    if (params.maskHelperPointsWeight <= 0.0)
        return;

    ALICEVISION_LOG_INFO("Add Mask Helper Points.");

    Point3d inflatedVoxel[8];
    mvsUtils::inflateHexahedron(voxel, inflatedVoxel, 1.01f);

    std::size_t nbPixels = 0;
    for (const auto& imgParams : _mp.getImagesParams())
    {
        nbPixels += imgParams.size;
    }
    // int step = std::floor(std::sqrt(double(nbPixels) / double(params.maxInputPoints)));
    // step = std::max(step, params.minStep);
    const int step = 1;
    int nbAddedPoints = 0;

    ALICEVISION_LOG_INFO("Load depth maps and add points.");
    {
        for (int c = 0; c < cams.size(); c++)
        {
            image::Image<float> depthMap;
            mvsUtils::readMap(c, _mp, mvsUtils::EFileType::depthMapFiltered, depthMap);

            if (depthMap.size() <= 0)
            {
                ALICEVISION_LOG_WARNING("Empty depth map (cam id: " << c << ")");
                continue;
            }

            const int width = depthMap.width();
            const int height = depthMap.height();
            const int syMax = divideRoundUp(height, step);
            const int sxMax = divideRoundUp(width, step);

            for (int sy = 0; sy < syMax; ++sy)
            {
                for (int sx = 0; sx < sxMax; ++sx)
                {
                    float bestScore = 0;

                    int bestX = 0;
                    int bestY = 0;
                    for (int y = sy * step, ymax = std::min((sy + 1) * step, height); y < ymax; ++y)
                    {
                        for (int x = sx * step, xmax = std::min((sx + 1) * step, width); x < xmax; ++x)
                        {
                            const std::size_t index = y * width + x;
                            const float depth = depthMap(index);

                            // -2 means that the pixels should be masked-out with mask helper points
                            if (depth > -1.5f)
                                continue;

                            int nbValidDepth = 0;
                            const int kernelSize = params.maskBorderSize;
                            for (int ly = std::max(y - kernelSize, 0), lyMax = std::min(y + kernelSize, height - 1); ly < lyMax; ++ly)
                            {
                                for (int lx = std::max(x - kernelSize, 0), lxMax = std::min(x + kernelSize, width - 1); lx < lxMax; ++lx)
                                {
                                    if (depthMap(ly * width + lx) > 0.0f)
                                        ++nbValidDepth;
                                }
                            }

                            const float score = nbValidDepth;  // TODO: best score based on nbValidDepth and kernel size ?
                            if (score > bestScore)
                            {
                                bestScore = score;
                                bestX = x;
                                bestY = y;
                            }
                        }
                    }
                    if (bestScore > 0.0f)
                    {
                        const Point3d& cam = _mp.CArr[c];
                        Point3d maxP = cam + (_mp.iCamArr[c] * Point2d((float)bestX, (float)bestY)).normalize() * 10000000.0;
                        StaticVector<Point3d>* intersectionsPtr = mvsUtils::lineSegmentHexahedronIntersection(cam, maxP, inflatedVoxel);

                        if (intersectionsPtr->size() <= 0)
                            continue;

                        Point3d p;
                        double maxDepth = std::numeric_limits<double>::min();
                        for (Point3d& i : *intersectionsPtr)
                        {
                            const double depth = dist(cam, i);
                            if (depth > maxDepth)
                            {
                                p = i;
                                maxDepth = depth;
                            }
                        }
                        GC_vertexInfo newv;
                        newv.nrc = params.maskHelperPointsWeight;
                        newv.pixSize = 0.0f;
                        newv.cams.push_back_distinct(c);

                        _verticesAttr.push_back(newv);
                        _verticesCoords.emplace_back(p);
                        ++nbAddedPoints;
                    }
                }
            }
        }
    }
    ALICEVISION_LOG_INFO("Added Points: " << nbAddedPoints);
    ALICEVISION_LOG_INFO("Add Mask Helper Points done.");
}

void PointCloud::createDensePointCloud(const Point3d hexah[8],
                                             const StaticVector<int>& cams,
                                             const sfmData::SfMData* sfmData,
                                             const PointCloudFuseParams* depthMapsFuseParams)
{
    assert(sfmData != nullptr || depthMapsFuseParams != nullptr);

    ALICEVISION_LOG_INFO("Creating dense point cloud.");

    const int helperPointsGridSize = _mp.userParams.get<int>("LargeScale.helperPointsGridSize", 10);
    const int densifyNbFront = _mp.userParams.get<int>("LargeScale.densifyNbFront", 0);
    const int densifyNbBack = _mp.userParams.get<int>("LargeScale.densifyNbBack", 0);
    const double densifyScale = _mp.userParams.get<double>("LargeScale.densifyScale", 1.0);

    const float minDist =
      hexah && (helperPointsGridSize > 0)
        ? ((hexah[0] - hexah[1]).size() + (hexah[0] - hexah[3]).size() + (hexah[0] - hexah[4]).size()) / (3. * helperPointsGridSize)
        : 0.00001f;

    // add points from depth maps
    if (depthMapsFuseParams != nullptr)
        fuseFromDepthMaps(cams, hexah, *depthMapsFuseParams);

    // add points from sfm
    if (sfmData != nullptr)
        addPointsFromSfM(hexah, cams, *sfmData);

    // add points for cam centers
    addPointsFromCameraCenters(cams, minDist);

    densifyWithHelperPoints(densifyNbFront, densifyNbBack, densifyScale);

    // add volume points to prevent singularities
    {
        Point3d hexahExt[8];
        mvsUtils::inflateHexahedron(hexah, hexahExt, 1.3);
        addGridHelperPoints(helperPointsGridSize, hexahExt, minDist);

        // add 6 points to prevent singularities (one point beyond each bbox facet)
        addPointsToPreventSingularities(hexahExt, minDist);

        // add point for shape from silhouette
        if (depthMapsFuseParams != nullptr)
            addMaskHelperPoints(hexahExt, cams, *depthMapsFuseParams);
    }

    _verticesCoords.shrink_to_fit();
    _verticesAttr.shrink_to_fit();

    ALICEVISION_LOG_WARNING("Final dense point cloud: " << _verticesCoords.size() << " points.");
}

void PointCloud::createPtsCams(StaticVector<StaticVector<int>>& out_ptsCams)
{
    long t = std::clock();
    int npts = _verticesCoords.size();

    out_ptsCams.reserve(npts);

    for (const GC_vertexInfo& v : _verticesAttr)
    {
        StaticVector<int> cams;
        cams.reserve(v.getNbCameras());
        for (int c = 0; c < v.getNbCameras(); c++)
        {
            cams.push_back(v.cams[c]);
        }

        out_ptsCams.push_back(cams);
    }
}

}
}