// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "PointCloudBuilder.hpp"

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/mvsUtils/MultiViewParams.hpp>
#include <aliceVision/mvsUtils/common.hpp>

#include <aliceVision/fuseCut/Kdtree.hpp>


namespace aliceVision {
namespace fuseCut {

PointCloudBuilder::PointCloudBuilder(mvsUtils::MultiViewParams& mp)
: _mp(mp)
{
    _camsVertexes.resize(_mp.ncams, -1);
}

void PointCloudBuilder::createDensePointCloud(const Point3d hexah[8],
                                             const StaticVector<int>& cams,
                                             const sfmData::SfMData & sfmData)
{
    const int helperPointsGridSize = _mp.userParams.get<int>("LargeScale.helperPointsGridSize", 10);
    const int densifyNbFront = _mp.userParams.get<int>("LargeScale.densifyNbFront", 0);
    const int densifyNbBack = _mp.userParams.get<int>("LargeScale.densifyNbBack", 0);
    const double densifyScale = _mp.userParams.get<double>("LargeScale.densifyScale", 1.0);

    const float minDist =
      hexah && (helperPointsGridSize > 0)
        ? ((hexah[0] - hexah[1]).size() + (hexah[0] - hexah[3]).size() + (hexah[0] - hexah[4]).size()) / (3. * helperPointsGridSize)
        : 0.00001f;
    
    // add points from sfm
    addPointsFromSfM(hexah, cams, sfmData);

    // add points for cam centers
    addPointsFromCameraCenters(cams, minDist);

    //densifyWithHelperPoints(densifyNbFront, densifyNbBack, densifyScale);

    Point3d hexahExt[8];
    mvsUtils::inflateHexahedron(hexah, hexahExt, 1.3);
    addGridHelperPoints(helperPointsGridSize, hexahExt, minDist);

    // add 6 points to prevent singularities (one point beyond each bbox facet)
    addPointsToPreventSingularities(hexahExt, minDist);

    _verticesCoords.shrink_to_fit();
    _verticesAttr.shrink_to_fit();

    

    

    ALICEVISION_LOG_WARNING("Final dense point cloud: " << _verticesCoords.size() << " points.");
}

void PointCloudBuilder::addPointsFromSfM(const Point3d hexah[8], const StaticVector<int>& cams, const sfmData::SfMData& sfmData)
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

    Eigen::Vector3d bbmin;
    bbmin.fill(std::numeric_limits<double>::max());
    Eigen::Vector3d bbmax;
    bbmax.fill(std::numeric_limits<double>::lowest());

    std::size_t addedPoints = 0;
    for (std::size_t i = 0; i < nbPoints; ++i)
    {
        const sfmData::Landmark& landmark = landmarkIt->second;
        const Point3d p(landmark.X(0), landmark.X(1), landmark.X(2));

        if (mvsUtils::isPointInHexahedron(p, hexah))
        {
            *vCoordsIt = p;

            bbmin.x() = std::min(bbmin.x(), landmark.X.x());
            bbmin.y() = std::min(bbmin.y(), landmark.X.y());
            bbmin.z() = std::min(bbmin.z(), landmark.X.z());
            bbmax.x() = std::max(bbmax.x(), landmark.X.x());
            bbmax.y() = std::max(bbmax.y(), landmark.X.y());
            bbmax.z() = std::max(bbmax.z(), landmark.X.z());

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

    _octree = std::make_unique<Node>(bbmin, bbmax);
    /*std::vector<fuseCut::Node::ptr> nodes;
    getNonEmptyNodes(nodes);

    for (auto node : nodes)
    {
        auto bbmin = node->getBBMin();
        auto bbmax = node->getBBMax();
        Eigen::Vector3d center = (bbmin + bbmax) * 0.5;
        
        Eigen::Vector3d borders[2];
        
        borders[0] = center + 1.1 * (bbmin - center);
        borders[1] = center + 1.1 * (bbmax - center);

        GC_vertexInfo newv;
        newv.nrc = 0;

        for (int ix = 0; ix < 2; ix++)
        {
            for (int iy = 0; iy < 2; iy++)
            {
                for (int iz = 0; iz < 2; iz++)
                {
                    Point3d p;
                    p.x = borders[ix].x();
                    p.y = borders[iy].y();
                    p.z = borders[iz].z();

                    _verticesCoords.push_back(p);
                    _verticesAttr.push_back(newv);
                }
            }
        }
    }*/

    ALICEVISION_LOG_WARNING("Add " << addedPoints << " new points for the SfM.");
}

void PointCloudBuilder::addPointsFromCameraCenters(const StaticVector<int>& cams, float minDist)
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
            // std::size_t vi;
            // double sq_dist;

            // if there is no nearest vertex or the nearest vertex is not too close
            // if(!kdTree.locateNearestVertex(p, vi, sq_dist) || (sq_dist > minDist2))
            {
                const size_t = _verticesCoords.size();
                _verticesCoords.push_back(p);

                GC_vertexInfo newv;
                newv.nrc = 0;

                _camsVertexes[rc] = nvi;

                _verticesAttr.push_back(newv);
                ++addedPoints;
            }
            // else
            // {
            //     _camsVertexes[rc] = vi;
            // }
        }
    }
    ALICEVISION_LOG_WARNING("Add " << addedPoints << " new points for the " << cams.size() << " cameras centers.");
}

void PointCloudBuilder::densifyWithHelperPoints(int nbFront, int nbBack, double scale)
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

void PointCloudBuilder::addPointsToPreventSingularities(const Point3d voxel[8], float minDist)
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

void PointCloudBuilder::addGridHelperPoints(int helperPointsGridSize, const Point3d voxel[8], float minDist)
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

}
}