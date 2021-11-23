// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "VoxelsGrid.hpp"
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/mvsData/Pixel.hpp>
#include <aliceVision/mvsUtils/common.hpp>
#include <aliceVision/mvsUtils/fileIO.hpp>
#include <aliceVision/fuseCut/delaunayGraphCutTypes.hpp>
#include <aliceVision/alicevision_omp.hpp>

#include <boost/filesystem.hpp>

namespace aliceVision {
namespace fuseCut {

namespace bfs = boost::filesystem;

VoxelsGrid::VoxelsGrid()
{
}

VoxelsGrid::VoxelsGrid(const Voxel& dimensions, Point3d* _space, mvsUtils::MultiViewParams* _mp, const std::string& _spaceRootDir, bool _doVisualize)
{
    doVisualize = _doVisualize;
    mp = _mp;
    voxelDim = dimensions;
    for(int k = 0; k < 8; k++)
    {
        space[k] = _space[k]; // TODO faca
    }
    voxels = mvsUtils::computeVoxels(space, dimensions);
    spaceRootDir = _spaceRootDir;
    bfs::create_directory(spaceRootDir);

    spaceCamsTracksDir = _spaceRootDir + "camsTracks/";
    bfs::create_directory(spaceCamsTracksDir);
}

VoxelsGrid* VoxelsGrid::clone(const std::string& _spaceRootDir)
{
    VoxelsGrid* out = new VoxelsGrid();

    out->doVisualize = doVisualize;
    out->mp = mp;
    out->voxelDim = voxelDim;
    out->voxels = new StaticVector<Point3d>();
    out->voxels->resize(voxels->size());
    for(int i = 0; i < voxels->size(); i++)
    {
        out->voxels->push_back((*voxels)[i]);
    }
    for(int k = 0; k < 8; k++)
    {
        out->space[k] = space[k];
    }
    out->spaceRootDir = _spaceRootDir;
    bfs::create_directory(out->spaceRootDir);

    out->spaceCamsTracksDir = _spaceRootDir + "camsTracks/";
    bfs::create_directory(out->spaceCamsTracksDir);

    return out;
}

VoxelsGrid::~VoxelsGrid()
{
    delete voxels;
}

StaticVector<int>* VoxelsGrid::getNVoxelsTracks()
{
    ALICEVISION_LOG_DEBUG("reading number of tracks for each voxel file.");

    StaticVector<int>* nVoxelsTracks = new StaticVector<int>();
    nVoxelsTracks->reserve(voxels->size() / 8);
    //long t1 = mvsUtils::initEstimate();
    for(int i = 0; i < voxels->size() / 8; i++)
    {
        std::string folderName = getVoxelFolderName(i);
        std::string fileNameTracksPts;
        fileNameTracksPts = folderName + "tracksGridPts.bin";
        int n = getArrayLengthFromFile(fileNameTracksPts);
        // printf("%i %i\n",i,n);
        nVoxelsTracks->push_back(n);
        //mvsUtils::printfEstimate(i, voxels->size() / 8, t1);
    }
    //mvsUtils::finishEstimate();

    return nVoxelsTracks;
}

unsigned long VoxelsGrid::getNTracks() const
{
    ALICEVISION_LOG_DEBUG("computing number of all tracks.");

    unsigned long ntracks = 0;
    //long t1 = mvsUtils::initEstimate();
    for(int i = 0; i < voxels->size() / 8; i++)
    {
        const std::string folderName = getVoxelFolderName(i);
        std::string fileNameTracksPts;
        fileNameTracksPts = folderName + "tracksGridPts.bin";
        int n = getArrayLengthFromFile(fileNameTracksPts);
        // printf("%i %i\n",i,n);
        ntracks += (unsigned long)n;
        //mvsUtils::printfEstimate(i, voxels->size() / 8, t1);
    }
    //mvsUtils::finishEstimate();

    return ntracks;
}

int VoxelsGrid::getIdForVoxel(const Voxel& v) const
{
    return v.x * voxelDim.y * voxelDim.z + v.y * voxelDim.z + v.z;
}

Voxel VoxelsGrid::getVoxelForId(int id) const
{
    int xp = id / (voxelDim.y * voxelDim.z);
    return Voxel(xp, (id - xp * voxelDim.y * voxelDim.z) / voxelDim.z,
                 (id - xp * voxelDim.y * voxelDim.z) % voxelDim.z);
}

bool VoxelsGrid::isValidVoxel(const Voxel& v)
{
    return ((v.x >= 0) && (v.x < voxelDim.x) && (v.y >= 0) && (v.y < voxelDim.y) && (v.z >= 0) && (v.z < voxelDim.z));
}

std::string VoxelsGrid::getVoxelFolderName(int id) const
{
    const Voxel v = getVoxelForId(id);
    // std::string fnx = spaceRootDir + "X"+num2str(v.x)+"/";
    // std::string fnxyz = fnx + "Y"+num2str(v.y)+"Z"+num2str(v.z)+"/";
    // bfs::create_directory(fnx);
    // bfs::create_directory(fnxyz);

    std::string fnxyz = spaceRootDir + "X" + mvsUtils::num2str(v.x) + "Y" + mvsUtils::num2str(v.y) + "Z" + mvsUtils::num2str(v.z) + "/";
    // bfs::create_directory(fnxyz);

    // if (FolderExists(fnx)==false) {
    //	printf("Warning folder %s does not exist!\n",fnx.c_str());
    //}

    // if (FolderExists(fnxyz)==false) {
    //	printf("Warning folder %s does not exist!\n",fnxyz.c_str());
    //}

    return fnxyz;
}

StaticVector<OctreeTracks::trackStruct*>* VoxelsGrid::loadTracksFromVoxelFiles(StaticVector<int>** cams, int id)
{
    const std::string folderName = getVoxelFolderName(id);

    const std::string fileNameTracksCams = folderName + "tracksGridCams.bin";
    const std::string fileNameTracksPts = folderName + "tracksGridPts.bin";
    const std::string fileNameTracksPtsCams = folderName + "tracksGridPtsCams.bin";
    const std::string fileNameTracksStat = folderName + "tracksGridStat.bin";

    if(!mvsUtils::FileExists(fileNameTracksPts))
        return nullptr;

    StaticVector<Point3d>* tracksStat = loadArrayFromFile<Point3d>(fileNameTracksStat); // minPixSize, minSim, npts
    StaticVector<Point3d>* tracksPoints = loadArrayFromFile<Point3d>(fileNameTracksPts);
    StaticVector<StaticVector<Pixel>*>* tracksPointsCams = loadArrayOfArraysFromFile<Pixel>(fileNameTracksPtsCams);
    *cams = loadArrayFromFile<int>(fileNameTracksCams);

    StaticVector<OctreeTracks::trackStruct*>* tracks = new StaticVector<OctreeTracks::trackStruct*>();
    tracks->reserve(tracksPoints->size());

    for(int i = 0; i < tracksPoints->size(); i++)
    {
        StaticVector<Pixel>* tcams = (*tracksPointsCams)[i];
        OctreeTracks::trackStruct* t = new OctreeTracks::trackStruct(0.0f, 0.0f, Point3d(), 0);
        t->point = (*tracksPoints)[i];
        t->minPixSize = (*tracksStat)[i].x;
        t->minSim = (*tracksStat)[i].y;
        t->npts = (int)(*tracksStat)[i].z;
        t->cams = *tcams;
        tracks->push_back(t);
    }

    delete tracksStat;
    delete tracksPoints;
    deleteArrayOfArrays<Pixel>(&tracksPointsCams);

    return tracks;
}

bool VoxelsGrid::saveTracksToVoxelFiles(StaticVector<int>* cams, StaticVector<OctreeTracks::trackStruct*>* tracks,
                                        int id)
{
    if(sizeOfStaticVector<OctreeTracks::trackStruct*>(tracks) <= 10)
    {
        return false;
    }

    std::string folderName = getVoxelFolderName(id);

    bfs::create_directory(folderName);
    if(!mvsUtils::FolderExists(folderName))
    {
        ALICEVISION_LOG_WARNING("Folder '" << folderName << "' does not exist.");
    }

    StaticVector<Point3d>* tracksPoints = new StaticVector<Point3d>();
    tracksPoints->reserve(tracks->size());
    StaticVector<StaticVector<Pixel>*>* tracksPointsCams = new StaticVector<StaticVector<Pixel>*>();
    tracksPointsCams->reserve(tracks->size());
    StaticVector<Point3d>* tracksStat = new StaticVector<Point3d>();
    tracksStat->reserve(tracks->size());

    for(int j = 0; j < tracks->size(); j++)
    {
        OctreeTracks::trackStruct* info = (*tracks)[j];
        tracksPoints->push_back(info->point);
        tracksStat->push_back(Point3d(info->minPixSize, info->minSim, info->npts));

        StaticVector<Pixel>* tcams = new StaticVector<Pixel>();
        tcams->reserve(info->cams.size());
        for(int k = 0; k < info->cams.size(); k++)
        {
            tcams->push_back(info->cams[k]);
        }
        tracksPointsCams->push_back(tcams);
    }

    // printf("SAVING %i-th VOXEL TRACKS\n",i);
    std::string fileNameTracksCams, fileNameTracksPts,
        fileNameTracksPtsCams, fileNameTracksStat;

    fileNameTracksCams = folderName + "tracksGridCams.bin";
    fileNameTracksPts = folderName + "tracksGridPts.bin";
    fileNameTracksPtsCams = folderName + "tracksGridPtsCams.bin";
    fileNameTracksStat = folderName + "tracksGridStat.bin";

    saveArrayToFile<int>(fileNameTracksCams, cams);
    saveArrayToFile<Point3d>(fileNameTracksPts, tracksPoints);
    saveArrayToFile<Point3d>(fileNameTracksStat, tracksStat);
    saveArrayOfArraysToFile<Pixel>(fileNameTracksPtsCams, tracksPointsCams);

    delete tracksPoints;
    delete tracksStat;
    deleteArrayOfArrays<Pixel>(&tracksPointsCams);

    return true;
}

void VoxelsGrid::generateTracksForEachVoxel(StaticVector<Point3d>* ReconstructionPlan, int numSubVoxs, int maxPts,
                                            int level, int& maxlevel, const std::string& depthMapsPtsSimsTmpDir)
{
    ALICEVISION_LOG_DEBUG("generateTracksForEachVoxel recursive "
                          << "\t- numSubVoxs: " << numSubVoxs << std::endl
                          << "\t- maxPts: " << maxPts << std::endl
                          << "\t- level: " << level);

    ALICEVISION_LOG_DEBUG("Voxel Dim: " << voxelDim.x << ", " << voxelDim.y << ", " << voxelDim.z);

    maxlevel = std::max(maxlevel, level);

    long tall = clock();
    int nvoxs = voxels->size() / 8;

    StaticVector<int>* toRecurse = new StaticVector<int>();
    toRecurse->reserve(nvoxs);

#pragma omp parallel for
    for(int i = 0; i < nvoxs; i++)
    {
        // printf("GENERATING TRIANGLUATION FOR %i-th VOXEL OF %i\n",i,nvoxs);

        std::string folderName = getVoxelFolderName(i);

        long t1 = clock();
        OctreeTracks* ott = new OctreeTracks(&(*voxels)[i * 8], mp, Voxel(numSubVoxs, numSubVoxs, numSubVoxs));
        StaticVector<OctreeTracks::trackStruct*>* tracks = ott->fillOctree(maxPts, depthMapsPtsSimsTmpDir);
        if(mp->verbose)
            mvsUtils::printfElapsedTime(t1, "fillOctree");
        if(tracks == nullptr)
        {
            if(mp->verbose)
                ALICEVISION_LOG_DEBUG("deleting OTT: " << i);
            delete ott;
            if(mp->verbose)
                ALICEVISION_LOG_DEBUG("deleted " << i);
#pragma omp critical
            {
                toRecurse->push_back(i);
            }
        }
        else
        {
            // printf("SAVING %i-th VOXEL TRACKS FILES\n",i);
            if(tracks->size() > 10)
            {
                if(ReconstructionPlan != nullptr)
                {
#pragma omp critical
                    {
                        for(int k = 0; k < 8; k++)
                        {
                            if(ReconstructionPlan->size() < ReconstructionPlan->capacity())
                            {
                                ReconstructionPlan->push_back((*voxels)[i * 8 + k]);
                            }
                            else
                            {
                                ALICEVISION_LOG_WARNING("reconstruction plan array is full.");
                            }
                        }
                    }
                }
                StaticVector<int>* cams = ott->getTracksCams(tracks);
                saveTracksToVoxelFiles(cams, tracks, i);
                delete cams;
            }
            delete tracks;
            if(mp->verbose)
                ALICEVISION_LOG_DEBUG("deleting OTT: " << i);
            delete ott;
            if(mp->verbose)
                ALICEVISION_LOG_DEBUG("deleted " << i);
        }
    }

    if(mp->verbose)
        ALICEVISION_LOG_DEBUG("toRecurse " << toRecurse->size());

    for(int j = 0; j < toRecurse->size(); j++)
    {
        int i = (*toRecurse)[j];
        // recursion
        std::string folderName = getVoxelFolderName(i);
        // std::string subfn = folderName + "sub/";
        std::string subfn = folderName;
        bfs::create_directory(subfn);

        // create file that indicates that the voxel has subvoxels
        std::string subfnFileMark = folderName + "sub.txt";
        FILE* f = fopen(subfnFileMark.c_str(), "w");
        fclose(f);

        VoxelsGrid* vgnew = new VoxelsGrid(Voxel(2, 2, 2), &(*voxels)[i * 8], mp, subfn, doVisualize);
        vgnew->generateTracksForEachVoxel(ReconstructionPlan, numSubVoxs / 2, maxPts, level + 1, maxlevel,
                                          depthMapsPtsSimsTmpDir);
        delete vgnew;
    }

    delete toRecurse;

    mvsUtils::printfElapsedTime(tall);

    if(doVisualize)
        vizualize();
}

void VoxelsGrid::generateSpace(VoxelsGrid* vgnew, const Voxel& LU, const Voxel& RD,
                               const std::string& depthMapsPtsSimsTmpDir)
{
    if(mp->verbose)
        ALICEVISION_LOG_DEBUG("generateSpace recursive: " << std::endl
                              << "\t- LU: " << LU.x << " " << LU.y << " " << LU.z << std::endl
                              << "\t- RD: " << RD.x << " " << RD.y << " " << RD.z);

    int nvoxs = voxels->size() / 8;
    for(int voxid = 0; voxid < nvoxs; voxid++)
    {
        std::string folderName = getVoxelFolderName(voxid);
        // std::string subfn = folderName + "sub/";
        std::string subfn = folderName;
        std::string subfnFileMark = folderName + "sub.txt";

        Voxel v = getVoxelForId(voxid);
        Voxel ns = RD - LU;
        ns.x /= voxelDim.x;
        ns.y /= voxelDim.y;
        ns.z /= voxelDim.z;
        Voxel subLU = LU + v * ns;
        Voxel subRD = LU + (v + 1) * ns;

        // if (FolderExists(subfn)==true)
        if(mvsUtils::FileExists(subfnFileMark))
        {
            VoxelsGrid* vgrec = new VoxelsGrid(Voxel(2, 2, 2), &(*voxels)[voxid * 8], mp, subfn, doVisualize);
            vgrec->generateSpace(vgnew, subLU, subRD, depthMapsPtsSimsTmpDir);
            delete vgrec;
        }
        else
        {
            Voxel part = subRD - subLU;
            OctreeTracks* ott = new OctreeTracks(&(*voxels)[voxid * 8], mp, part);
            VoxelsGrid* vgg = new VoxelsGrid(part, &(*voxels)[voxid * 8], mp, folderName, doVisualize);
            StaticVector<int>* cams = nullptr;
            StaticVector<OctreeTracks::trackStruct*>* tracks = loadTracksFromVoxelFiles(&cams, voxid);
            Voxel vrel;

            if((tracks != nullptr) && (tracks->size() > 0))
            {
                // estimate nubers // TODO FACA: remove costly estimation of numbers
                StaticVector<int>* nnewVoxsTracks = new StaticVector<int>();
                nnewVoxsTracks->reserve(part.x * part.y * part.z);
                nnewVoxsTracks->resize_with(part.x * part.y * part.z, 0);
                for(int i = 0; i < tracks->size(); i++)
                {
                    if(ott->getVoxelOfOctreeFor3DPoint(vrel, (*tracks)[i]->point))
                    {
                        (*nnewVoxsTracks)[vgg->getIdForVoxel(vrel)] += 1;
                    }
                }

                // allocate
                StaticVector<StaticVector<OctreeTracks::trackStruct*>*>* newVoxsTracks =
                    new StaticVector<StaticVector<OctreeTracks::trackStruct*>*>();
                newVoxsTracks->reserve(part.x * part.y * part.z);
                for(int i = 0; i < part.x * part.y * part.z; i++)
                {
                    auto* newVoxTracks = new StaticVector<OctreeTracks::trackStruct*>();
                    newVoxTracks->reserve((*nnewVoxsTracks)[i]);
                    newVoxsTracks->push_back(newVoxTracks);
                }

                // fill
                for(int i = 0; i < tracks->size(); i++)
                {
                    if(ott->getVoxelOfOctreeFor3DPoint(vrel, (*tracks)[i]->point))
                    {
                        (*newVoxsTracks)[vgg->getIdForVoxel(vrel)]->push_back((*tracks)[i]);
                    }
                }

                for(int i = 0; i < part.x * part.y * part.z; i++)
                {
                    Voxel vact = vgg->getVoxelForId(i);
                    Voxel vglob = subLU + vact;
                    int newid = vgnew->getIdForVoxel(vglob);

                    vgnew->saveTracksToVoxelFiles(cams, (*newVoxsTracks)[i], newid);

                    /*
                    printf("idlocal %i of %i, voxel local %i %i %i, voxel global %i %i %i, id global %i, ntracks %i \n",
                            i,part.x*part.y*part.z, vact.x, vact.y, vact.z, vglob.x, vglob.y, vglob.z, newid,
                            (*newVoxsTracks)[i]->size()
                    );
                    */
                }

                // deallocate
                delete newVoxsTracks;
                delete nnewVoxsTracks;
            }

            if(cams != nullptr)
            {
                delete cams;
            }

            if(tracks != nullptr)
            {
                for(int i = 0; i < tracks->size(); i++)
                {
                    delete(*tracks)[i];
                }
                delete tracks;
            }

            delete vgg;
            delete ott;
        }
    }
}

void VoxelsGrid::cloneSpaceVoxel(int voxelId, int numSubVoxs, VoxelsGrid* newSpace)
{
    std::string folderName = getVoxelFolderName(voxelId);
    std::string fileNameTracksPts = folderName + "tracksGridPts.bin";

    if(mvsUtils::FileExists(fileNameTracksPts))
    {
        OctreeTracks* ott =
            new OctreeTracks(&(*voxels)[voxelId * 8], mp, Voxel(numSubVoxs, numSubVoxs, numSubVoxs));
        StaticVector<int>* tcams;
        StaticVector<OctreeTracks::trackStruct*>* tracksOld = loadTracksFromVoxelFiles(&tcams, voxelId);
        StaticVector<OctreeTracks::trackStruct*>* tracksNew = ott->fillOctreeFromTracks(tracksOld);
        for(int i = 0; i < tracksOld->size(); i++)
        {
            delete(*tracksOld)[i];
        }
        delete tracksOld;
        newSpace->saveTracksToVoxelFiles(tcams, tracksNew, voxelId);

        delete tcams;
        delete tracksNew; // DO NOT NEEDED TO DELETE PARTICULAR POINTERS BECAUSE THEY POINT TO ott STRUCTURES
        delete ott;
    }
}

VoxelsGrid* VoxelsGrid::cloneSpace(int numSubVoxs, std::string newSpaceRootDir)
{
    if(mp->verbose)
        ALICEVISION_LOG_DEBUG("cloning space.");

    VoxelsGrid* out = clone(newSpaceRootDir);

    long t1 = mvsUtils::initEstimate();
    for(int i = 0; i < voxels->size() / 8; i++)
    {
        cloneSpaceVoxel(i, numSubVoxs, out);
        mvsUtils::printfEstimate(i, voxels->size() / 8, t1);
    }
    mvsUtils::finishEstimate();

    if(doVisualize)
        out->vizualize();

    return out;
}

void VoxelsGrid::copySpaceVoxel(int voxelId, VoxelsGrid* newSpace)
{
    std::string folderName = getVoxelFolderName(voxelId);
    std::string fileNameTracksPts = folderName + "tracksGridPts.bin";

    if(mvsUtils::FileExists(fileNameTracksPts))
    {
        StaticVector<int>* tcams;
        StaticVector<OctreeTracks::trackStruct*>* tracksOld = loadTracksFromVoxelFiles(&tcams, voxelId);
        newSpace->saveTracksToVoxelFiles(tcams, tracksOld, voxelId);

        for(int i = 0; i < tracksOld->size(); i++)
        {
            delete(*tracksOld)[i];
        }
        delete tracksOld;
        delete tcams;
    }
}

VoxelsGrid* VoxelsGrid::copySpace(std::string newSpaceRootDir)
{
    if(mp->verbose)
        ALICEVISION_LOG_DEBUG("Copy space.");

    VoxelsGrid* out = clone(newSpaceRootDir);

    long t1 = mvsUtils::initEstimate();
    for(int i = 0; i < voxels->size() / 8; i++)
    {
        copySpaceVoxel(i, out);
        mvsUtils::printfEstimate(i, voxels->size() / 8, t1);
    }
    mvsUtils::finishEstimate();

    if(doVisualize)
        out->vizualize();

    return out;
}

void VoxelsGrid::generateCamsPtsFromVoxelsTracks()
{
    if(mp->verbose)
        ALICEVISION_LOG_DEBUG("Distributing pts from voxels tracks to camera files.");

    long t1 = mvsUtils::initEstimate();
    int nvoxs = voxels->size() / 8;
    for(int i = 0; i < nvoxs; i++)
    {
        std::string folderName = getVoxelFolderName(i);
        std::string fileNameTracksCams, fileNameTracksPts,
            fileNameTracksPtsCams;
        fileNameTracksCams = folderName + "tracksGridCams.bin";
        fileNameTracksPts = folderName + "tracksGridPts.bin";
        fileNameTracksPtsCams = folderName + "tracksGridPtsCams.bin";

        // printf("SAVING %i-th VOXEL POINTS TO CAMS FILES\n",i);

        if(mvsUtils::FileExists(fileNameTracksPts))
        {
            StaticVector<Point3d>* tracksPoints = loadArrayFromFile<Point3d>(fileNameTracksPts);
            StaticVector<StaticVector<Pixel>*>* tracksPointsCams =
                loadArrayOfArraysFromFile<Pixel>(fileNameTracksPtsCams);
            StaticVector<int>* cams = loadArrayFromFile<int>(fileNameTracksCams);

            // printf("distributing %i tracks to %i camspts files  \n", tracksPoints->size(), cams->size());
            StaticVector<StaticVector<Pixel>*>* camsTracksPoints =
                convertObjectsCamsToCamsObjects(*mp, tracksPointsCams);

#pragma omp parallel for
            for(int c = 0; c < cams->size(); c++)
            {
                int rc = (*cams)[c];

                // open camPtsFile for append
                std::string camPtsFileName = spaceCamsTracksDir + "camPtsGrid_" + std::to_string(mp->getViewId(rc)) + ".bin";
                FILE* fin = fopen(camPtsFileName.c_str(), "ab");
                StaticVector<Pixel>* camPtsIds = (*camsTracksPoints)[rc];
                for(int j = 0; j < sizeOfStaticVector<Pixel>(camPtsIds); j++)
                {
                    int ptid = (*camPtsIds)[j].x;
                    int nrc = (*camPtsIds)[j].y;
                    GC_camVertexInfo p;
                    p.point = (*tracksPoints)[ptid];
                    p.sim = -0.91; // TODO FACA: why??
                    p.nrc = nrc;
                    p.ncams = sizeOfStaticVector<Pixel>((*tracksPointsCams)[ptid]);
                    p.fwriteinfo(fin);
                }
                fclose(fin);
            } // for cams

            delete cams;
            delete tracksPoints;
            deleteArrayOfArrays<Pixel>(&camsTracksPoints);
            deleteArrayOfArrays<Pixel>(&tracksPointsCams);
        }

        mvsUtils::printfEstimate(i, nvoxs, t1);
    } // for i
    mvsUtils::finishEstimate();
}

void VoxelsGrid::vizualize()
{
    std::string spaceWrlFileName = spaceRootDir + "tracks.wrl";
    FILE* f = fopen(spaceWrlFileName.c_str(), "w");
    fprintf(f, "#VRML V2.0 utf8\n");
    fprintf(f, "Background {\n skyColor 1 1 1 \n } \n");
    int nvoxs = voxels->size() / 8;
    for(int i = 0; i < nvoxs; i++)
    {
        std::string subFoldeName = getVoxelFolderName(i);
        std::string fname = subFoldeName + "tracks.wrl";
        if(mvsUtils::FolderExists(subFoldeName.c_str()))
        {
            fprintf(f, "Inline{ url [\"%s\"] \n }\n", fname.c_str());
        }
    }
    fclose(f);
}

void VoxelsGrid::getHexah(Point3d* hexahOut, const Voxel& LUi, const Voxel& RDi)
{
    Voxel LU, RD;
    LU.x = std::min(LUi.x, RDi.x);
    LU.y = std::min(LUi.y, RDi.y);
    LU.z = std::min(LUi.z, RDi.z);
    RD.x = std::max(LUi.x, RDi.x);
    RD.y = std::max(LUi.y, RDi.y);
    RD.z = std::max(LUi.z, RDi.z);

    int luid = getIdForVoxel(LU);

    // Point3d *vox = &(*voxels)[0];
    // Point3d O   = vox[0];
    // Point3d vvx = vox[1]-vox[0];
    // Point3d vvy = vox[3]-vox[0];
    // Point3d vvz = vox[4]-vox[0];

    Voxel X, Y, Z;
    int Xid, Yid, Zid;
    X.x = RD.x;
    X.y = LU.y;
    X.z = LU.z;
    Xid = getIdForVoxel(X);
    Y.x = LU.x;
    Y.y = RD.y;
    Y.z = LU.z;
    Yid = getIdForVoxel(Y);
    Z.x = LU.x;
    Z.y = LU.y;
    Z.z = RD.z;
    Zid = getIdForVoxel(Z);

    Point3d O = (*voxels)[luid * 8];
    Point3d vvx = (*voxels)[Xid * 8 + 1] - O;
    Point3d vvy = (*voxels)[Yid * 8 + 3] - O;
    Point3d vvz = (*voxels)[Zid * 8 + 4] - O;

    hexahOut[0] = O;
    hexahOut[1] = O + vvx;
    hexahOut[2] = O + vvx + vvy;
    hexahOut[3] = O + vvy;
    hexahOut[4] = O + vvz;
    hexahOut[5] = O + vvz + vvx;
    hexahOut[6] = O + vvz + vvx + vvy;
    hexahOut[7] = O + vvz + vvy;
}

} // namespace fuseCut
} // namespace aliceVision
