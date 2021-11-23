// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "LargeScale.hpp"
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/mvsUtils/common.hpp>
#include <aliceVision/mvsUtils/fileIO.hpp>
#include <aliceVision/fuseCut/DelaunayGraphCut.hpp>

#include <boost/filesystem.hpp>

namespace aliceVision {
namespace fuseCut {

namespace bfs = boost::filesystem;

LargeScale::LargeScale(mvsUtils::MultiViewParams* _mp, std::string _spaceFolderName)
  : mp(_mp)
  , spaceFolderName(_spaceFolderName)
  , spaceVoxelsFolderName(_spaceFolderName + "_data/")
  , spaceFileName(spaceFolderName + "/space.txt")
{
    bfs::create_directory(spaceFolderName);
    bfs::create_directory(spaceVoxelsFolderName);

    doVisualize = mp->userParams.get<bool>("LargeScale.doVisualizeOctreeTracks", false);
}

LargeScale::~LargeScale()
{
}

bool LargeScale::isSpaceSaved()
{
    return mvsUtils::FileExists(spaceFileName);
}

void LargeScale::saveSpaceToFile()
{
    FILE* f = fopen(spaceFileName.c_str(), "w");
    fprintf(f, "%lf %lf %lf %lf %lf %lf %lf %lf\n", space[0].x, space[1].x, space[2].x, space[3].x, space[4].x,
            space[5].x, space[6].x, space[7].x);
    fprintf(f, "%lf %lf %lf %lf %lf %lf %lf %lf\n", space[0].y, space[1].y, space[2].y, space[3].y, space[4].y,
            space[5].y, space[6].y, space[7].y);
    fprintf(f, "%lf %lf %lf %lf %lf %lf %lf %lf\n", space[0].z, space[1].z, space[2].z, space[3].z, space[4].z,
            space[5].z, space[6].z, space[7].z);
    fprintf(f, "%i %i %i\n", dimensions.x, dimensions.y, dimensions.z);
    fprintf(f, "%i\n", maxOcTreeDim);
    fclose(f);
}

void LargeScale::loadSpaceFromFile()
{
    FILE* f = fopen(spaceFileName.c_str(), "r");
    fscanf(f, "%lf %lf %lf %lf %lf %lf %lf %lf\n",
           &space[0].x, &space[1].x, &space[2].x, &space[3].x,
           &space[4].x, &space[5].x, &space[6].x, &space[7].x);
    fscanf(f, "%lf %lf %lf %lf %lf %lf %lf %lf\n",
           &space[0].y, &space[1].y, &space[2].y, &space[3].y,
           &space[4].y, &space[5].y, &space[6].y, &space[7].y);
    fscanf(f, "%lf %lf %lf %lf %lf %lf %lf %lf\n",
           &space[0].z, &space[1].z, &space[2].z, &space[3].z,
           &space[4].z, &space[5].z, &space[6].z, &space[7].z);
    fscanf(f, "%i %i %i\n", &dimensions.x, &dimensions.y, &dimensions.z);
    fscanf(f, "%i\n", &maxOcTreeDim);
    fclose(f);
}

void LargeScale::initialEstimateSpace(int maxOcTreeDim)
{
    float minPixSize;
    Fuser* fs = new Fuser(*mp);
    fs->divideSpaceFromDepthMaps(&space[0], minPixSize);
    dimensions = fs->estimateDimensions(&space[0], &space[0], 0, maxOcTreeDim);
    delete fs;
}

std::string LargeScale::getSpaceCamsTracksDir()
{
    VoxelsGrid* vg = new VoxelsGrid(dimensions, &space[0], mp, spaceVoxelsFolderName);
    std::string out = vg->spaceCamsTracksDir;
    delete vg;
    return out;
}

LargeScale* LargeScale::cloneSpaceIfDoesNotExists(int newOcTreeDim, std::string newSpaceFolderName)
{
    if(isSpaceSaved())
    {
        loadSpaceFromFile();
        
        LargeScale* out = new LargeScale(mp, newSpaceFolderName);

        if(out->isSpaceSaved())
        {
            out->loadSpaceFromFile();
            return out;
        }

        out->space = space;
        out->dimensions = dimensions;
        out->doVisualize = doVisualize;

        out->maxOcTreeDim = (int)((float)maxOcTreeDim / (1024.0f / (float)newOcTreeDim));

        if(mp->verbose)
        {
            ALICEVISION_LOG_DEBUG("maxOcTreeDim new: " << out->maxOcTreeDim);
            ALICEVISION_LOG_DEBUG("maxOcTreeDim old: " << maxOcTreeDim);
        }

        long t1 = clock();

        VoxelsGrid* vgactual = new VoxelsGrid(dimensions, &space[0], mp, spaceVoxelsFolderName, doVisualize);
        if(maxOcTreeDim == out->maxOcTreeDim)
        {
            VoxelsGrid* vgnew = vgactual->copySpace(out->spaceVoxelsFolderName);
            vgnew->generateCamsPtsFromVoxelsTracks();
            delete vgnew;
        }
        else
        {
            VoxelsGrid* vgnew = vgactual->cloneSpace(out->maxOcTreeDim, out->spaceVoxelsFolderName);
            vgnew->generateCamsPtsFromVoxelsTracks();
            delete vgnew;
        }
        delete vgactual;

        out->saveSpaceToFile();

        if(mp->verbose)
            mvsUtils::printfElapsedTime(t1, "space cloned in:");

        return out;
    }

    return nullptr;
}

bool LargeScale::generateSpace(int maxPts, int ocTreeDim, bool generateTracks)
{
    if(isSpaceSaved())
    {
        loadSpaceFromFile();
        return false;
    }

    maxOcTreeDim = 1024;
    initialEstimateSpace(maxOcTreeDim);
    maxOcTreeDim = ocTreeDim;

    if(generateTracks)
    {
        bool addRandomNoise = mp->userParams.get<bool>("LargeScale.addRandomNoise", false);
        float addRandomNoisePercNoisePts =
            (float)mp->userParams.get<double>("LargeScale.addRandomNoisePercNoisePts", 10.0);
        int addRandomNoiseNoisPixSizeDistHalfThr =
            (float)mp->userParams.get<int>("LargeScale.addRandomNoiseNoisPixSizeDistHalfThr", 10);

        std::string depthMapsPtsSimsTmpDir = generateTempPtsSimsFiles(
            spaceFolderName, *mp, addRandomNoise, addRandomNoisePercNoisePts, addRandomNoiseNoisPixSizeDistHalfThr);

        ALICEVISION_LOG_INFO("Creating tracks: " << dimensions.x << ", " << dimensions.y << ", " << dimensions.z);
        StaticVector<Point3d>* ReconstructionPlan = new StaticVector<Point3d>();
        ReconstructionPlan->reserve(1000000);

        std::string tmpdir = spaceFolderName + "tmp/";
        bfs::create_directory(tmpdir);
        VoxelsGrid* vg = new VoxelsGrid(dimensions, &space[0], mp, tmpdir, doVisualize);
        int maxlevel = 0;
        vg->generateTracksForEachVoxel(ReconstructionPlan, maxOcTreeDim, maxPts, 1, maxlevel, depthMapsPtsSimsTmpDir);
        ALICEVISION_LOG_DEBUG("max rec level: " << maxlevel);
        for(int i = 1; i < maxlevel; i++)
        {
            dimensions = dimensions * 2;
            maxOcTreeDim = maxOcTreeDim / 2;
            ALICEVISION_LOG_DEBUG("dimmension: " << dimensions.x << ", " << dimensions.y << ", " << dimensions.z << " max: " << maxOcTreeDim);
        }
        ALICEVISION_LOG_DEBUG("final dimmension: " << dimensions.x << ", " << dimensions.y << ", " << dimensions.z << " max: " << maxOcTreeDim);

        VoxelsGrid* vgnew = new VoxelsGrid(dimensions, &space[0], mp, spaceVoxelsFolderName, doVisualize);
        vg->generateSpace(vgnew, Voxel(0, 0, 0), dimensions, depthMapsPtsSimsTmpDir);
        vgnew->generateCamsPtsFromVoxelsTracks();
        if(doVisualize)
            vgnew->vizualize();

        delete vgnew;
        delete vg;

        mvsUtils::DeleteDirectory(tmpdir);

        deleteTempPtsSimsFiles(*mp, depthMapsPtsSimsTmpDir);

        saveArrayToFile<Point3d>(spaceFolderName + "spacePatitioning.bin", ReconstructionPlan);
        delete ReconstructionPlan;

        saveSpaceToFile();
    }
    return true;
}

Point3d LargeScale::getSpaceSteps()
{
    Point3d vx = space[1] - space[0];
    Point3d vy = space[3] - space[0];
    Point3d vz = space[4] - space[0];
    Point3d sv;
    sv.x = (vx.size() / (float)dimensions.x) / (float)maxOcTreeDim;
    sv.y = (vy.size() / (float)dimensions.y) / (float)maxOcTreeDim;
    sv.z = (vz.size() / (float)dimensions.z) / (float)maxOcTreeDim;
    return sv;
}

} // namespace fuseCut
} // namespace aliceVision
