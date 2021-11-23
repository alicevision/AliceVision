// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "OctreeTracks.hpp"
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/mvsData/geometry.hpp>
#include <aliceVision/mvsUtils/common.hpp>
#include <aliceVision/mvsUtils/fileIO.hpp>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>

#include <iostream>

namespace aliceVision {
namespace fuseCut {

OctreeTracks::Node::Node(NodeType type)
{
    type_ = type;
}

OctreeTracks::trackStruct* OctreeTracks::getTrack(int x, int y, int z)
{
    if(!((x >= 0 && x < size_) && (y >= 0 && y < size_) && (z >= 0 && z < size_)))
    {
        return nullptr;
    }

    Node** n = &root_;
    int size = size_;

    while(size != 1)
    {
        if(*n == nullptr)
        {
            return nullptr;
        }

        size /= 2;
        n = &reinterpret_cast<Branch*>(*n)->children[!((x & size) == 0)][!((y & size) == 0)][!((z & size) == 0)];
    }

    if(*n == nullptr)
    {
        return nullptr;
    }

    return reinterpret_cast<trackStruct*>(*n);
}

void OctreeTracks::addPoint(int x, int y, int z, float sim, float pixSize, Point3d& p, int rc)
{
    assert(x >= 0 && x < size_);
    assert(y >= 0 && y < size_);
    assert(z >= 0 && z < size_);

    Node** n = &root_;
    int size = size_;

    while(size != 1)
    {
        if(*n == nullptr)
        {
            *n = new Branch();
        }
        else
        {
            size /= 2;
            n = &reinterpret_cast<Branch*>(*n)->children[!((x & size) == 0)][!((y & size) == 0)][!((z & size) == 0)];
        }
    }

    if(*n == nullptr)
    {
        *n = new trackStruct(sim, pixSize, p, rc);
        leafsNumber_++;
    }
    else
    {
        reinterpret_cast<trackStruct*>(*n)->addPoint(sim, pixSize, p, rc);
    }
}

void OctreeTracks::addTrack(int x, int y, int z, OctreeTracks::trackStruct* t)
{
    assert(x >= 0 && x < size_);
    assert(y >= 0 && y < size_);
    assert(z >= 0 && z < size_);

    Node** n = &root_;
    int size = size_;

    while(size != 1)
    {
        if(*n == nullptr)
        {
            *n = new Branch();
        }
        else
        {
            size /= 2;
            n = &reinterpret_cast<Branch*>(*n)->children[!((x & size) == 0)][!((y & size) == 0)][!((z & size) == 0)];
        }
    }

    if(*n == nullptr)
    {
        *n = new trackStruct(t);
        leafsNumber_++;
    }
    else
    {
        reinterpret_cast<trackStruct*>(*n)->addTrack(t);
    }
}

StaticVector<OctreeTracks::trackStruct*>* OctreeTracks::getAllPoints()
{
    StaticVector<trackStruct*>* out = new StaticVector<trackStruct*>();
    out->reserve(leafsNumber_);
    if(root_ != nullptr)
    {
        getAllPointsRecursive(out, root_);
    }
    return out;
}

void OctreeTracks::getAllPointsRecursive(StaticVector<trackStruct*>* out, Node* node)
{
    assert(node);

    switch(node->type_)
    {
        case BranchNode:
        {
            Branch* b = reinterpret_cast<Branch*>(node);
            for(int i = 0; i < 2; ++i)
            {
                for(int j = 0; j < 2; ++j)
                {
                    for(int k = 0; k < 2; ++k)
                    {
                        if(b->children[i][j][k] != nullptr)
                        {
                            getAllPointsRecursive(out, b->children[i][j][k]);
                        }
                    }
                }
            }
        }
        break;
        case LeafNode:
            out->push_back(reinterpret_cast<trackStruct*>(node));
            break;
    }
}

void OctreeTracks::getNPointsByLevelsRecursive(Node* node, int level, StaticVector<int>* nptsAtLevel)
{
    assert(node);

    switch(node->type_)
    {
        case BranchNode:
        {
            Branch* b = reinterpret_cast<Branch*>(node);
            for(int i = 0; i < 2; ++i)
            {
                for(int j = 0; j < 2; ++j)
                {
                    for(int k = 0; k < 2; ++k)
                    {
                        if(b->children[i][j][k] != nullptr)
                        {
                            (*nptsAtLevel)[level + 1] += 1;
                            getNPointsByLevelsRecursive(b->children[i][j][k], level + 1, nptsAtLevel);
                        }
                    }
                }
            }
        }
        break;
        case LeafNode:
            //
            break;
    }
}

OctreeTracks::Branch::Branch()
    : Node(BranchNode)
{
    for(int i = 0; i < 2; ++i)
    {
        for(int j = 0; j < 2; ++j)
        {
            for(int k = 0; k < 2; ++k)
            {
                children[i][j][k] = nullptr;
            }
        }
    }
}

OctreeTracks::Branch::~Branch()
{
    for(int i = 0; i < 2; ++i)
    {
        for(int j = 0; j < 2; ++j)
        {
            for(int k = 0; k < 2; ++k)
            {
                assert(children[i][j][k] != this);
                /*
                Node *node = children[i][j][k];
                if (node!=NULL) {
                        if ( node->type_ == BranchNode ) {
                                delete reinterpret_cast<Branch*>(node);
                        }else {
                                if ( node->type_ == LeafNode ) {
                                        delete reinterpret_cast<trackStruct*>(node);
                                }else{
                                        printf("WARNING unknows node type\n");
                                };
                        }
                        node = NULL;
                        children[i][j][k] = NULL;
                }
                */
                if(children[i][j][k] != nullptr)
                {
                    if(children[i][j][k]->type_ == BranchNode)
                    {
                        delete reinterpret_cast<Branch*>(children[i][j][k]);
                    }
                    else
                    {
                        if(children[i][j][k]->type_ == LeafNode)
                        {
                            delete reinterpret_cast<trackStruct*>(children[i][j][k]);
                        }
                        else
                        {
                            ALICEVISION_LOG_WARNING("Unknows node type.");
                        }
                    }
                    children[i][j][k] = nullptr;
                }
            }
        }
    }
}

OctreeTracks::trackStruct::trackStruct(float sim, float pixSize, const Point3d& p, int rc)
    : Node(LeafNode)
{
    npts = 1;
    point = p;
    cams.reserve(10);
    cams.push_back(Pixel(rc, 1));
    minPixSize = pixSize;
    minSim = sim;
}

OctreeTracks::trackStruct::trackStruct(trackStruct* t)
    : Node(LeafNode)
{
    npts = t->npts;
    point = t->point;
    cams = t->cams;
    minPixSize = t->minPixSize;
    minSim = t->minSim;
}

OctreeTracks::trackStruct::~trackStruct()
{
}

void OctreeTracks::trackStruct::addPoint(float sim, float pixSize, const Point3d& p, int rc)
{
    int index = indexOf(rc);
    if(index == -1)
    {
        cams.push_back(Pixel(rc, 1));
        if(cams.size() > 1)
        {
            qsort(&cams[0], cams.size(), sizeof(Pixel), qSortComparePixelByXAsc);
        }
    }
    else
    {
        cams[index].y += 1;
    }

    // strategy 1: average all values
    // point = (point * (float)npts + p) / (float)(npts + 1);
    // minPixSize = std::min(minPixSize, pixSize);
    // minSim = std::min(minSim, sim);
    // npts++;

    // strategy 2: keep best from nearest cam
    // if (pixSize < minPixSize * 1.2f) // if equivalent or better than the previous value
    //{
    //	if ((sim < minSim) || (pixSize * 1.2f <= minPixSize))
    //	{
    //		point = p;
    //		minSim = sim;
    //	};
    //	minPixSize = std::min(minPixSize,pixSize);
    //};

    // strategy 3: average values with good precision (precision is given by pixel size)
    if(pixSize < minPixSize * 0.8f) // if strongly better => replace previous values
    {
        point = p;
        minPixSize = pixSize;
        minSim = sim;
        npts = 1;
    }
    else if(pixSize < minPixSize * 1.2f) // if close to the previous value => average
    {
        // average with previous values of the same precision
        point = (point * (float)npts + p) / (float)(npts + 1);
        minPixSize = std::min(minPixSize, pixSize);
        minSim = std::min(minSim, sim);
        npts++;
    }
    // else don't use the position information as it is less accurate.
}

void OctreeTracks::trackStruct::addDistinctNonzeroCamsFromTrackAsZeroCams(trackStruct* t)
{
    for(int i = 0; i < t->cams.size(); i++)
    {
        if(t->cams[i].y > 0)
        {
            int rc = t->cams[i].x;
            int index = indexOf(rc);
            if(index == -1)
            {
                cams.push_back(Pixel(rc, 0));
                if(cams.size() > 1)
                {
                    qsort(&cams[0], cams.size(), sizeof(Pixel), qSortComparePixelByXAsc);
                }
            }
        }
    }
}

void OctreeTracks::trackStruct::addTrack(OctreeTracks::trackStruct* t)
{
    for(int i = 0; i < t->cams.size(); i++)
    {
        int rc = t->cams[i].x;
        int index = indexOf(rc);
        if(index == -1)
        {
            cams.push_back(t->cams[i]);
            if(cams.size() > 1)
            {
                qsort(&cams[0], cams.size(), sizeof(Pixel), qSortComparePixelByXAsc);
            }
        }
        else
        {
            cams[index].y += t->cams[i].y;
        }
    }

    // // to keep CG
    // point = (point * (float)npts + t->point * (float)t->npts) / (float)(npts + t->npts);
    // minPixSize = std::min(minPixSize, t->minPixSize);
    // minSim = std::min(minSim, t->minSim);

    // keep best from nearest cam
    // if (t->minPixSize<minPixSize*1.2f)
    //{
    //	if ((t->minSim<minSim)||(minPixSize>t->minPixSize*1.2f))
    //	{
    //		point = t->point;
    //		minSim = t->minSim;
    //	};
    //	minPixSize = std::min(minPixSize,t->minPixSize);
    //};

    // strategy 3: average values with good precision (precision is given by pixel size)
    if(t->minPixSize < minPixSize * 0.8f) // if strongly better => replace previous values
    {
        point = t->point;
        minPixSize = t->minPixSize;
        minSim = t->minSim;
        npts = t->npts;
    }
    else if(t->minPixSize < minPixSize * 1.2f) // if close to the previous value => average
    {
        // average with previous values of the same precision
        point = (point * (float)npts + t->point * (float)t->npts) / (float)(npts + t->npts);
        minPixSize = std::min(minPixSize, t->minPixSize);
        minSim = std::min(minSim, t->minSim);
        npts += t->npts;
    }
    // else don't use the position information as it is less accurate.
}

int OctreeTracks::trackStruct::indexOf(int val)
{
    if(cams.empty())
    {
        return -1;
    }

    int lef = 0;
    int rig = cams.size() - 1;
    int mid = lef + (rig - lef) / 2;
    while((rig - lef) > 1)
    {
        if((val >= cams[lef].x) && (val < cams[mid].x))
        {
            //lef = lef;
            rig = mid;
            mid = lef + (rig - lef) / 2;
        }
        if((val >= cams[mid].x) && (val <= cams[rig].x))
        {
            lef = mid;
            //rig = rig;
            mid = lef + (rig - lef) / 2;
        }
        if((val < cams[lef].x) || (val > cams[rig].x))
        {
            lef = 0;
            rig = 0;
            mid = 0;
        }
    }

    int id = -1;
    if(val == cams[lef].x)
    {
        id = lef;
    }
    if(val == cams[rig].x)
    {
        id = rig;
    }

    // printf("index of %f is %i\n", (float)val, id);

    return id;
}

void OctreeTracks::trackStruct::doPrintf()
{
    ALICEVISION_LOG_INFO("point: " << point.x << " " << point.y << " " << point.z);
    ALICEVISION_LOG_INFO("ncams: " << cams.size());
    for(int i = 0; i < cams.size(); i++)
        ALICEVISION_LOG_INFO("\t- cam: " << i << ", rc: " << cams[i].x << ", val: " << cams[i].y);
}

OctreeTracks::OctreeTracks(const Point3d* voxel_, mvsUtils::MultiViewParams* mp_, Voxel dimensions)
    : Fuser(*mp_)
{
    numSubVoxsX = dimensions.x;
    numSubVoxsY = dimensions.y;
    numSubVoxsZ = dimensions.z;
    for(int i = 0; i < 8; i++)
    {
        vox[i] = voxel_[i];
    }

    O = vox[0];
    vx = vox[1] - vox[0];
    vy = vox[3] - vox[0];
    vz = vox[4] - vox[0];
    svx = vx.size();
    svy = vy.size();
    svz = vz.size();
    vx = vx.normalize();
    vy = vy.normalize();
    vz = vz.normalize();

    sx = svx / (float)numSubVoxsX;
    sy = svy / (float)numSubVoxsY;
    sz = svz / (float)numSubVoxsZ;

    doFilterOctreeTracks = _mp.userParams.get<bool>("LargeScale.doFilterOctreeTracks", true);
    doUseWeaklySupportedPoints = _mp.userParams.get<bool>("LargeScale.doUseWeaklySupportedPoints", false);
    doUseWeaklySupportedPointCam = _mp.userParams.get<bool>("LargeScale.doUseWeaklySupportedPointCam", false);
    minNumOfConsistentCams = _mp.userParams.get<int>("filter.minNumOfConsistentCams", 2);
    simWspThr = (float)_mp.userParams.get<double>("LargeScale.simWspThr", -0.0f);

    int maxNumSubVoxs = std::max(std::max(numSubVoxsX, numSubVoxsY), numSubVoxsZ);
    size_ = 2;
    while(size_ < maxNumSubVoxs)
    {
        size_ *= 2;
    }

    root_ = nullptr;
    leafsNumber_ = 0;
}

OctreeTracks::~OctreeTracks()
{
    // printf("deleting octree\n");
    if(root_ != nullptr)
    {
        delete reinterpret_cast<Branch*>(root_);
    }
    // printf("deleted\n");
}

bool OctreeTracks::getVoxelOfOctreeFor3DPoint(Voxel& out, Point3d& tp)
{
    out.x = (int)floor(orientedPointPlaneDistance(tp, O, vx) / sx);
    out.y = (int)floor(orientedPointPlaneDistance(tp, O, vy) / sy);
    out.z = (int)floor(orientedPointPlaneDistance(tp, O, vz) / sz);
    return ((out.x >= 0) && (out.x < numSubVoxsX) && (out.y >= 0) && (out.y < numSubVoxsY) && (out.z >= 0) &&
            (out.z < numSubVoxsZ));
}

void OctreeTracks::filterMinNumConsistentCams(StaticVector<trackStruct*>* tracks)
{
    using namespace boost::accumulators;

    StaticVector<trackStruct*> tracksOut;
    tracksOut.reserve(tracks->size());
    typedef accumulator_set<float,
      stats<
        tag::min,
        tag::mean,
        tag::max,
        tag::median(with_p_square_quantile)> > Accumulator;
    Accumulator accMinPixSize;
    Accumulator accMinSim;
    Accumulator accNbCamsA;
    Accumulator accNbCamsB;

    // long t1 = initEstimate();
    for(int i = 0; i < tracks->size(); i++)
    {
        trackStruct* t = (*tracks)[i];
        accNbCamsA(t->cams.size());
        if(t->cams.size() >= minNumOfConsistentCams)
        {
            tracksOut.push_back((*tracks)[i]);
            accMinPixSize(t->minPixSize);
            accMinSim(t->minSim);
            accNbCamsB(t->cams.size());
        } // ELSE DO NOT DELETE BECAUSE IT IS POINTER TO THE STRUCTURE
        // printfEstimate(i, tracks->size(), t1);
    }
    // finishEstimate();

    ALICEVISION_LOG_INFO("filterMinNumConsistentCams: " << std::endl
      << "\t- minPixelSize min: " << boost::accumulators::min(accMinPixSize) << ", max: " << boost::accumulators::max(accMinPixSize) << ", mean: " << boost::accumulators::mean(accMinPixSize) << ", median: " << boost::accumulators::median(accMinPixSize) << std::endl
      << "\t- minSim min: " << boost::accumulators::min(accMinSim) << ", max: " << boost::accumulators::max(accMinSim) << ", mean: " << boost::accumulators::mean(accMinSim) << ", median: " << boost::accumulators::median(accMinSim) << std::endl
      << "\t- accNbCamsA min: " << boost::accumulators::min(accNbCamsA) << ", max: " << boost::accumulators::max(accNbCamsA) << ", mean: " << boost::accumulators::mean(accNbCamsA) << ", median: " << boost::accumulators::median(accNbCamsA) << std::endl
      << "\t- accNbCamsB min: " << boost::accumulators::min(accNbCamsB) << ", max: " << boost::accumulators::max(accNbCamsB) << ", mean: " << boost::accumulators::mean(accNbCamsB) << ", median: " << boost::accumulators::median(accNbCamsB) << std::endl);

    tracks->swap(tracksOut);
}

// TODO this is not working well ...
// if there are two neighbouring voxels and their representants are closer than
// 1.2 minpixsize of each of them then they must have the same set of cameras
// this should preserve the case when the density of points is smaller than sx
void OctreeTracks::updateOctreeTracksCams(StaticVector<trackStruct*>* tracks)
{
    float clusterSizeThr = 1.2f;

    // long t1 = initEstimate();
    for(int i = 0; i < tracks->size(); i++)
    {
        int n = (int)ceil(((*tracks)[i]->minPixSize * clusterSizeThr) / sx);
        Voxel vox;
        if((n > 1) && (getVoxelOfOctreeFor3DPoint(vox, (*tracks)[i]->point)))
        {
            // printf("n %i\n",n);
            for(int xp = -n; xp <= n; xp++)
            {
                for(int yp = -n; yp <= n; yp++)
                {
                    for(int zp = -n; zp <= n; zp++)
                    {
                        if((xp == 0) && (yp == 0) && (zp == 0))
                        {
                            assert(getTrack(vox.x + xp, vox.y + yp, vox.z + zp) == (*tracks)[i]);
                            continue;
                        }
                        trackStruct* neighborTrack = getTrack(vox.x + xp, vox.y + yp, vox.z + zp);
                        if(neighborTrack != nullptr)
                        {
                            trackStruct& currentTrack = *(*tracks)[i];
                            const float dist = (currentTrack.point - neighborTrack->point).size();
                            if((dist < currentTrack.minPixSize * 1.2f) || (dist < neighborTrack->minPixSize * 1.2f))
                               // (nt->minPixSize < (*tracks)[i]->minPixSize * 1.2f) && ((*tracks)[i]->minPixSize < nt->minPixSize * 1.2f)
                            {
                                // currentTrack.cams

                                currentTrack.addDistinctNonzeroCamsFromTrackAsZeroCams(neighborTrack);
                                neighborTrack->addDistinctNonzeroCamsFromTrackAsZeroCams((*tracks)[i]);
                            }
                        }
                    }
                }
            }
        }
    }
    // finishEstimate();
}

/// if there is in near distance to actual track another track that has
/// smaller enough pix size then remove the actual track
void OctreeTracks::filterOctreeTracks2(StaticVector<trackStruct*>* tracks)
{
    ALICEVISION_LOG_DEBUG("filterOctreeTracks2");
    StaticVector<trackStruct*> tracksOut;
    tracksOut.reserve(tracks->size());

    float clusterSizeThr = _mp.userParams.get<double>("OctreeTracks.clusterSizeThr", 2.0f);

    // long t1 = initEstimate();
    for(int i = 0; i < tracks->size(); i++)
    {
        int n = (int)ceil(((*tracks)[i]->minPixSize * clusterSizeThr) / sx);

        bool ok = true;
        Voxel vox;
        if((n > 1) && (getVoxelOfOctreeFor3DPoint(vox, (*tracks)[i]->point)))
        {
            // printf("n %i\n",n);
            for(int xp = -n; xp <= n; xp++)
            {
                for(int yp = -n; yp <= n; yp++)
                {
                    for(int zp = -n; zp <= n; zp++)
                    {
                        trackStruct* nt = getTrack(vox.x + xp, vox.y + yp, vox.z + zp);
                        if((xp == 0) && (yp == 0) && (zp == 0))
                        {
                            assert(nt == (*tracks)[i]);
                        }
                        else
                        {
                            if((nt != nullptr) && ((*tracks)[i]->minPixSize > nt->minPixSize * 1.2f))
                            {
                                ok = false;
                            }
                        }
                    }
                }
            }
        }
        if(ok)
        {
            tracksOut.push_back((*tracks)[i]);
        } // ELSE DO NOT DELETE BECAUSE IT IS POINTER TO THE STRUCTURE
        // printfEstimate(i, tracks->size(), t1);
    }
    // finishEstimate();

    tracks->swap(tracksOut);
}

StaticVector<OctreeTracks::trackStruct*>* OctreeTracks::fillOctree(int maxPts, std::string depthMapsPtsSimsTmpDir)
{
    long t1 = clock();
    StaticVector<int> cams = _mp.findCamsWhichIntersectsHexahedron(vox, depthMapsPtsSimsTmpDir + "minMaxDepths.bin");
    mvsUtils::printfElapsedTime(t1, "findCamsWhichIntersectsHexahedron");
    ALICEVISION_LOG_DEBUG("ncams: " << cams.size());

    t1 = clock();


    // long t1=initEstimate();
    for(int camid = 0; camid < cams.size(); camid++)
    {
        int rc = cams[camid];
        StaticVector<Point3d>* pts =
            loadArrayFromFile<Point3d>(depthMapsPtsSimsTmpDir + std::to_string(_mp.getViewId(rc)) + "pts.bin");
        StaticVector<float>* sims =
            loadArrayFromFile<float>(depthMapsPtsSimsTmpDir + std::to_string(_mp.getViewId(rc)) + "sims.bin");

        // long tpts=initEstimate();
        for(int i = 0; i < pts->size(); i++)
        {
            float sim = (*sims)[i];
            Point3d p = (*pts)[i];

            Voxel otVox;
            if(((doUseWeaklySupportedPoints) || (sim < simWspThr)) && (getVoxelOfOctreeFor3DPoint(otVox, p))) // doUseWeaklySupportedPoints: false by default
            {
                if(doUseWeaklySupportedPointCam)
                {
                    if(sim > 1.0f)
                    {
                        sim -= 2.0f;
                    }
                }
                float pixSize = _mp.getCamPixelSize(p, rc);
                addPoint(otVox.x, otVox.y, otVox.z, sim, pixSize, p, rc);
            }
            if(leafsNumber_ > 2 * maxPts)
            {
                return nullptr;
            }

            // printfEstimate(i, pts->size(), tpts);
        } // for i
          // finishEstimate();

        delete pts;
        delete sims;

        // printfEstimate(camid, cams.size(), t1);
    }
    // finishEstimate();

    StaticVector<trackStruct*>* tracks = getAllPoints();
    mvsUtils::printfElapsedTime(t1, "fillOctree fill");

    // TODO this is not working well ...
    // updateOctreeTracksCams(tracks);
    // if (_mp.verbose) printfElapsedTime(t1,"updateOctreeTracksCams");

    if(doFilterOctreeTracks)
    {
        ALICEVISION_LOG_DEBUG("# tracks before filtering: " << tracks->size());
        long t2 = clock();

        filterMinNumConsistentCams(tracks);
        mvsUtils::printfElapsedTime(t2, "filterMinNumConsistentCams");

        ALICEVISION_LOG_DEBUG("# tracks after filterMinNumConsistentCams: " << tracks->size());

        t2 = clock();
        // filter cameras observations that have a large pixelSize regarding the others
        filterOctreeTracks2(tracks);

        mvsUtils::printfElapsedTime(t2, "filterOctreeTracks2");

        ALICEVISION_LOG_DEBUG("# tracks after filterOctreeTracks2: " << tracks->size());
        ALICEVISION_LOG_DEBUG("# tracks after filtering: " << tracks->size());
    }

    if(tracks->size() > maxPts)
    {
        ALICEVISION_LOG_DEBUG("Too much tracks (" << tracks->size() << "), clear all.");
        delete tracks; // DO NOT DELETE POINTER JUST DELETE THE ARRAY!!!
        return nullptr;
    }

    ALICEVISION_LOG_DEBUG("number of tracks: " << tracks->size());

    return tracks;
}

StaticVector<OctreeTracks::trackStruct*>*
OctreeTracks::fillOctreeFromTracks(StaticVector<OctreeTracks::trackStruct*>* tracksIn)
{
    long t1 = clock();

    for(int i = 0; i < tracksIn->size(); i++)
    {
        trackStruct* t = (*tracksIn)[i];
        Voxel otVox;
        if(getVoxelOfOctreeFor3DPoint(otVox, t->point))
        {
            addTrack(otVox.x, otVox.y, otVox.z, t);
        }
    } // for i

    StaticVector<trackStruct*>* tracks = getAllPoints();

    mvsUtils::printfElapsedTime(t1, "fillOctreeFromTracks");

    return tracks;
}

StaticVector<int>* OctreeTracks::getTracksCams(StaticVector<OctreeTracks::trackStruct*>* tracks)
{
    StaticVectorBool* camsb = new StaticVectorBool();
    camsb->reserve(_mp.ncams);
    camsb->resize_with(_mp.ncams, false);

    for(int i = 0; i < tracks->size(); i++)
    {
        for(int c = 0; c < (*tracks)[i]->cams.size(); c++)
        {
            (*camsb)[(*tracks)[i]->cams[c].x] = true;
        }
    }

    StaticVector<int>* cams = new StaticVector<int>();
    cams->reserve(_mp.ncams);
    for(int i = 0; i < _mp.ncams; i++)
    {
        if((*camsb)[i])
        {
            cams->push_back(i);
        }
    }

    delete camsb;
    return cams;
}

} // namespace fuseCut
} // namespace aliceVision
