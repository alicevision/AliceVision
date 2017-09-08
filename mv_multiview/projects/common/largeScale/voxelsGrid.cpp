#include "voxelsGrid.h"
#include "stdafx.h"

#include "delaunaycut/mv_delaunay_types.h"
#include "rply/mv_plyloader.h"
#include "structures/mv_filesio.h"
#include "cmpmvs_omp.hpp"

#include <boost/filesystem.hpp>

namespace bfs = boost::filesystem;

voxelsGrid::voxelsGrid()
{
}

voxelsGrid::voxelsGrid(const voxel& dimmensions, point3d* _space, multiviewParams* _mp, mv_prematch_cams* _pc,
                       const std::string& _spaceRootDir, bool _doVisualize)
{
    doVisualize = _doVisualize;
    mp = _mp;
    pc = _pc;
    voxelDim = dimmensions;
    for(int k = 0; k < 8; k++)
    {
        space[k] = _space[k]; // TODO faca
    }
    voxels = computeVoxels(space, dimmensions);
    spaceRootDir = _spaceRootDir;
    bfs::create_directory(spaceRootDir);

    spaceCamsTracksDir = _spaceRootDir + "camsTracks/";
    bfs::create_directory(spaceCamsTracksDir);
}

voxelsGrid* voxelsGrid::clone(const std::string& _spaceRootDir)
{
    voxelsGrid* out = new voxelsGrid();

    out->doVisualize = doVisualize;
    out->mp = mp;
    out->pc = pc;
    out->voxelDim = voxelDim;
    out->voxels = new staticVector<point3d>(voxels->size());
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

voxelsGrid::~voxelsGrid()
{
    delete voxels;
}

staticVector<int>* voxelsGrid::getNVoxelsTracks()
{
    if(mp->verbose)
        printf("reading number of tracks for each voxel file\n");

    staticVector<int>* nVoxelsTracks = new staticVector<int>(voxels->size() / 8);
    long t1 = initEstimate();
    for(int i = 0; i < voxels->size() / 8; i++)
    {
        std::string folderName = getVoxelFolderName(i);
        std::string fileNameTracksPts;
        fileNameTracksPts = folderName + "tracksGridPts.bin";
        int n = getArrayLengthFromFile(fileNameTracksPts);
        // printf("%i %i\n",i,n);
        nVoxelsTracks->push_back(n);
        printfEstimate(i, voxels->size() / 8, t1);
    }
    finishEstimate();

    return nVoxelsTracks;
}

staticVector<int>* voxelsGrid::getVoxelNPointsByLevels(int numSubVoxs, int voxelId)
{
    std::string folderName = getVoxelFolderName(voxelId);
    std::string fileNameTracksPts = folderName + "tracksGridPts.bin";

    if(FileExists(fileNameTracksPts))
    {
        octreeTracks* ott =
            new octreeTracks(&(*voxels)[voxelId * 8], mp, pc, voxel(numSubVoxs, numSubVoxs, numSubVoxs));
        staticVector<point3d>* tracksPoints = loadArrayFromFile<point3d>(fileNameTracksPts);
        for(int i = 0; i < tracksPoints->size(); i++)
        {
            point3d p = (*tracksPoints)[i];
            voxel otVox;
            if(ott->getVoxelOfOctreeFor3DPoint(otVox, p))
            {
                ott->addPoint(otVox.x, otVox.y, otVox.z, 0.0f, 0.0f, p, 0);
            }
            else
            {
                printf("WARNING !!! \n");
            }
        }

        return ott->getNPointsByLevels();

        delete ott;
    }

    return nullptr;
}

unsigned long voxelsGrid::getNTracks() const
{
    if(mp->verbose)
        printf("computing number of all tracks \n");

    unsigned long ntracks = 0;
    long t1 = initEstimate();
    for(int i = 0; i < voxels->size() / 8; i++)
    {
        const std::string folderName = getVoxelFolderName(i);
        std::string fileNameTracksPts;
        fileNameTracksPts = folderName + "tracksGridPts.bin";
        int n = getArrayLengthFromFile(fileNameTracksPts);
        // printf("%i %i\n",i,n);
        ntracks += (unsigned long)n;
        printfEstimate(i, voxels->size() / 8, t1);
    }
    finishEstimate();

    return ntracks;
}

int voxelsGrid::getIdForVoxel(const voxel& v) const
{
    return v.x * voxelDim.y * voxelDim.z + v.y * voxelDim.z + v.z;
}

voxel voxelsGrid::getVoxelForId(int id) const
{
    int xp = id / (voxelDim.y * voxelDim.z);
    return voxel(xp, (id - xp * voxelDim.y * voxelDim.z) / voxelDim.z,
                 (id - xp * voxelDim.y * voxelDim.z) % voxelDim.z);
}

bool voxelsGrid::isValidVoxel(const voxel& v)
{
    return ((v.x >= 0) && (v.x < voxelDim.x) && (v.y >= 0) && (v.y < voxelDim.y) && (v.z >= 0) && (v.z < voxelDim.z));
}

std::string voxelsGrid::getVoxelFolderName(int id) const
{
    const voxel v = getVoxelForId(id);
    // std::string fnx = spaceRootDir + "X"+num2str(v.x)+"/";
    // std::string fnxyz = fnx + "Y"+num2str(v.y)+"Z"+num2str(v.z)+"/";
    // bfs::create_directory(fnx);
    // bfs::create_directory(fnxyz);

    std::string fnxyz = spaceRootDir + "X" + num2str(v.x) + "Y" + num2str(v.y) + "Z" + num2str(v.z) + "/";
    // bfs::create_directory(fnxyz);

    // if (FolderExists(fnx)==false) {
    //	printf("Warning folder %s does not exist!\n",fnx.c_str());
    //}

    // if (FolderExists(fnxyz)==false) {
    //	printf("Warning folder %s does not exist!\n",fnxyz.c_str());
    //}

    return fnxyz;
}

staticVector<octreeTracks::trackStruct*>* voxelsGrid::loadTracksFromVoxelFiles(staticVector<int>** cams, int id)
{
    const std::string folderName = getVoxelFolderName(id);

    const std::string fileNameTracksCams = folderName + "tracksGridCams.bin";
    const std::string fileNameTracksPts = folderName + "tracksGridPts.bin";
    const std::string fileNameTracksPtsCams = folderName + "tracksGridPtsCams.bin";
    const std::string fileNameTracksStat = folderName + "tracksGridStat.bin";

    if(!FileExists(fileNameTracksPts))
        return nullptr;

    staticVector<point3d>* tracksStat = loadArrayFromFile<point3d>(fileNameTracksStat); // minPixSize, minSim, npts
    staticVector<point3d>* tracksPoints = loadArrayFromFile<point3d>(fileNameTracksPts);
    staticVector<staticVector<pixel>*>* tracksPointsCams = loadArrayOfArraysFromFile<pixel>(fileNameTracksPtsCams);
    *cams = loadArrayFromFile<int>(fileNameTracksCams);

    staticVector<octreeTracks::trackStruct*>* tracks =
        new staticVector<octreeTracks::trackStruct*>(tracksPoints->size());
    for(int i = 0; i < tracksPoints->size(); i++)
    {
        staticVector<pixel>* tcams = (*tracksPointsCams)[i];
        octreeTracks::trackStruct* t = new octreeTracks::trackStruct(0.0f, 0.0f, point3d(), 0);
        t->point = (*tracksPoints)[i];
        t->minPixSize = (*tracksStat)[i].x;
        t->minSim = (*tracksStat)[i].y;
        t->npts = (int)(*tracksStat)[i].z;
        t->cams = *tcams;
        tracks->push_back(t);
    }

    delete tracksStat;
    delete tracksPoints;
    deleteArrayOfArrays<pixel>(&tracksPointsCams);

    return tracks;
}

bool voxelsGrid::saveTracksToVoxelFiles(staticVector<int>* cams, staticVector<octreeTracks::trackStruct*>* tracks,
                                        int id)
{
    if(sizeOfStaticVector<octreeTracks::trackStruct*>(tracks) <= 10)
    {
        return false;
    }

    std::string folderName = getVoxelFolderName(id);

    bfs::create_directory(folderName);
    if(!FolderExists(folderName))
    {
        printf("Warning folder %s does not exist!\n", folderName.c_str());
    }

    staticVector<point3d>* tracksPoints = new staticVector<point3d>(tracks->size());
    staticVector<staticVector<pixel>*>* tracksPointsCams = new staticVector<staticVector<pixel>*>(tracks->size());
    staticVector<point3d>* tracksStat = new staticVector<point3d>(tracks->size());

    for(int j = 0; j < tracks->size(); j++)
    {
        octreeTracks::trackStruct* info = (*tracks)[j];
        tracksPoints->push_back(info->point);
        tracksStat->push_back(point3d(info->minPixSize, info->minSim, info->npts));

        staticVector<pixel>* tcams = new staticVector<pixel>(info->cams.size());
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
    saveArrayToFile<point3d>(fileNameTracksPts, tracksPoints);
    saveArrayToFile<point3d>(fileNameTracksStat, tracksStat);
    saveArrayOfArraysToFile<pixel>(fileNameTracksPtsCams, tracksPointsCams);

    delete tracksPoints;
    delete tracksStat;
    deleteArrayOfArrays<pixel>(&tracksPointsCams);

    return true;
}

void voxelsGrid::generateTracksForEachVoxel(staticVector<point3d>* reconstructionPlan, int numSubVoxs, int maxPts,
                                            int level, int& maxlevel, const std::string& depthMapsPtsSimsTmpDir)
{
    if(mp->verbose)
        printf("\n generateTracksForEachVoxel recursive numSubVoxs %i, maxPts %i, level %i \n", numSubVoxs, maxPts,
               level);
    if(mp->verbose)
        printf("dim %i %i %i \n", voxelDim.x, voxelDim.y, voxelDim.z);

    maxlevel = std::max(maxlevel, level);

    long tall = clock();
    int nvoxs = voxels->size() / 8;

    staticVector<int>* toRecurse = new staticVector<int>(nvoxs);

#pragma omp parallel for
    for(int i = 0; i < nvoxs; i++)
    {
        // printf("GENERATING TRIANGLUATION FOR %i-th VOXEL OF %i\n",i,nvoxs);

        std::string folderName = getVoxelFolderName(i);

        long t1 = clock();
        octreeTracks* ott = new octreeTracks(&(*voxels)[i * 8], mp, pc, voxel(numSubVoxs, numSubVoxs, numSubVoxs));
        staticVector<octreeTracks::trackStruct*>* tracks = ott->fillOctree(maxPts, depthMapsPtsSimsTmpDir);
        if(mp->verbose)
            printfElapsedTime(t1, "fillOctree");
        if(tracks == nullptr)
        {
            if(mp->verbose)
                printf("deleting OTT %i\n", i);
            delete ott;
            if(mp->verbose)
                printf("deleted %i\n", i);
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
                if(reconstructionPlan != nullptr)
                {
#pragma omp critical
                    {
                        for(int k = 0; k < 8; k++)
                        {
                            if(reconstructionPlan->size() < reconstructionPlan->capacity())
                            {
                                reconstructionPlan->push_back((*voxels)[i * 8 + k]);
                            }
                            else
                            {
                                printf("WARNINIG reconstruction plan array is full!\n");
                            }
                        }
                    }
                }
                staticVector<int>* cams = ott->getTracksCams(tracks);
                saveTracksToVoxelFiles(cams, tracks, i);
                delete cams;

                if(doVisualize)
                    ott->visualizeTracks(folderName, tracks);
            }
            delete tracks;
            if(mp->verbose)
                printf("deleting OTT %i\n", i);
            delete ott;
            if(mp->verbose)
                printf("deleted %i\n", i);
        }
    }

    if(mp->verbose)
        printf("toRecurse %i\n", toRecurse->size());

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

        voxelsGrid* vgnew = new voxelsGrid(voxel(2, 2, 2), &(*voxels)[i * 8], mp, pc, subfn, doVisualize);
        vgnew->generateTracksForEachVoxel(reconstructionPlan, numSubVoxs / 2, maxPts, level + 1, maxlevel,
                                          depthMapsPtsSimsTmpDir);
        delete vgnew;
    }

    delete toRecurse;

    printfElapsedTime(tall);

    if(doVisualize)
        vizualize();
}

void voxelsGrid::generateSpace(voxelsGrid* vgnew, const voxel& LU, const voxel& RD,
                               const std::string& depthMapsPtsSimsTmpDir)
{
    if(mp->verbose)
        printf("\n generateSpace recursive LU %i %i %i, RD %i %i %i\n", LU.x, LU.y, LU.z, RD.x, RD.y, RD.z);

    int nvoxs = voxels->size() / 8;
    for(int voxid = 0; voxid < nvoxs; voxid++)
    {
        std::string folderName = getVoxelFolderName(voxid);
        // std::string subfn = folderName + "sub/";
        std::string subfn = folderName;
        std::string subfnFileMark = folderName + "sub.txt";

        voxel v = getVoxelForId(voxid);
        voxel ns = RD - LU;
        ns.x /= voxelDim.x;
        ns.y /= voxelDim.y;
        ns.z /= voxelDim.z;
        voxel subLU = LU + v * ns;
        voxel subRD = LU + (v + 1) * ns;

        // if (FolderExists(subfn)==true)
        if(FileExists(subfnFileMark))
        {
            voxelsGrid* vgrec = new voxelsGrid(voxel(2, 2, 2), &(*voxels)[voxid * 8], mp, pc, subfn, doVisualize);
            vgrec->generateSpace(vgnew, subLU, subRD, depthMapsPtsSimsTmpDir);
            delete vgrec;
        }
        else
        {
            voxel part = subRD - subLU;
            octreeTracks* ott = new octreeTracks(&(*voxels)[voxid * 8], mp, pc, part);
            voxelsGrid* vgg = new voxelsGrid(part, &(*voxels)[voxid * 8], mp, pc, folderName, doVisualize);
            staticVector<int>* cams = nullptr;
            staticVector<octreeTracks::trackStruct*>* tracks = loadTracksFromVoxelFiles(&cams, voxid);
            voxel vrel;

            if((tracks != nullptr) && (tracks->size() > 0))
            {
                // estimate nubers // TODO FACA: remove costly estimation of numbers
                staticVector<int>* nnewVoxsTracks = new staticVector<int>(part.x * part.y * part.z);
                nnewVoxsTracks->resize_with(part.x * part.y * part.z, 0);
                for(int i = 0; i < tracks->size(); i++)
                {
                    if(ott->getVoxelOfOctreeFor3DPoint(vrel, (*tracks)[i]->point))
                    {
                        (*nnewVoxsTracks)[vgg->getIdForVoxel(vrel)] += 1;
                    }
                }

                // allocate
                staticVector<staticVector<octreeTracks::trackStruct*>*>* newVoxsTracks =
                    new staticVector<staticVector<octreeTracks::trackStruct*>*>(part.x * part.y * part.z);
                for(int i = 0; i < part.x * part.y * part.z; i++)
                {
                    newVoxsTracks->push_back(new staticVector<octreeTracks::trackStruct*>((*nnewVoxsTracks)[i]));
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
                    voxel vact = vgg->getVoxelForId(i);
                    voxel vglob = subLU + vact;
                    int newid = vgnew->getIdForVoxel(vglob);

                    if(vgnew->saveTracksToVoxelFiles(cams, (*newVoxsTracks)[i], newid))
                    {
                        if(doVisualize)
                        {
                            octreeTracks* ott1 = new octreeTracks(&(*vgnew->voxels)[newid * 8], mp, pc, voxel(1, 1, 1));
                            ott1->visualizeTracks(vgnew->getVoxelFolderName(newid), (*newVoxsTracks)[i]);
                            delete ott1;
                        }
                    }

                    /*
                    printf("idlocal %i of %i, voxel local %i %i %i, voxel global %i %i %i, id global %i, ntracks %i \n",
                            i,part.x*part.y*part.z, vact.x, vact.y, vact.z, vglob.x, vglob.y, vglob.z, newid,
                            (*newVoxsTracks)[i]->size()
                    );

                    octreeTracks *ott1 = new octreeTracks(&(*vgg->voxels)[i*8],mp,pc,voxel(1,1,1));
                    ott1->visualizeTracks(vgnew->getVoxelFolderName(newid), (*newVoxsTracks)[i]);
                    delete ott1;

                    std::string nfn = vgnew->getVoxelFolderName(newid) + "aaa/";
                    bfs::create_directory(nfn);
                    ott1 = new octreeTracks(&(*vgnew->voxels)[newid*8],mp,pc,voxel(1,1,1));
                    ott1->visualizeTracks(nfn, (*newVoxsTracks)[i]);
                    delete ott1;
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

void voxelsGrid::cloneSpaceVoxel(int voxelId, int numSubVoxs, voxelsGrid* newSpace)
{
    std::string folderName = getVoxelFolderName(voxelId);
    std::string fileNameTracksPts = folderName + "tracksGridPts.bin";

    if(FileExists(fileNameTracksPts))
    {
        octreeTracks* ott =
            new octreeTracks(&(*voxels)[voxelId * 8], mp, pc, voxel(numSubVoxs, numSubVoxs, numSubVoxs));
        staticVector<int>* tcams;
        staticVector<octreeTracks::trackStruct*>* tracksOld = loadTracksFromVoxelFiles(&tcams, voxelId);
        staticVector<octreeTracks::trackStruct*>* tracksNew = ott->fillOctreeFromTracks(tracksOld);
        for(int i = 0; i < tracksOld->size(); i++)
        {
            delete(*tracksOld)[i];
        }
        delete tracksOld;
        newSpace->saveTracksToVoxelFiles(tcams, tracksNew, voxelId);

        if(doVisualize)
            ott->visualizeTracks(newSpace->getVoxelFolderName(voxelId), tracksNew);

        delete tcams;
        delete tracksNew; // DO NOT NEEDED TO DELETE PARTICULAR POINTERS BECAUSE THEY POINT TO ott STRUCTURES
        delete ott;
    }
}

voxelsGrid* voxelsGrid::cloneSpace(int numSubVoxs, std::string newSpaceRootDir)
{
    if(mp->verbose)
        printf("cloning space\n");

    voxelsGrid* out = clone(newSpaceRootDir);

    long t1 = initEstimate();
    for(int i = 0; i < voxels->size() / 8; i++)
    {
        cloneSpaceVoxel(i, numSubVoxs, out);
        printfEstimate(i, voxels->size() / 8, t1);
    }
    finishEstimate();

    if(doVisualize)
        out->vizualize();

    return out;
}

void voxelsGrid::copySpaceVoxel(int voxelId, voxelsGrid* newSpace)
{
    std::string folderName = getVoxelFolderName(voxelId);
    std::string fileNameTracksPts = folderName + "tracksGridPts.bin";

    if(FileExists(fileNameTracksPts))
    {
        staticVector<int>* tcams;
        staticVector<octreeTracks::trackStruct*>* tracksOld = loadTracksFromVoxelFiles(&tcams, voxelId);
        newSpace->saveTracksToVoxelFiles(tcams, tracksOld, voxelId);

        if(doVisualize)
        {
            octreeTracks* ott = new octreeTracks(&(*voxels)[voxelId * 8], mp, pc, voxelDim);
            ott->visualizeTracks(newSpace->getVoxelFolderName(voxelId), tracksOld);
            delete ott;
        }

        for(int i = 0; i < tracksOld->size(); i++)
        {
            delete(*tracksOld)[i];
        }
        delete tracksOld;
        delete tcams;
    }
}

voxelsGrid* voxelsGrid::copySpace(std::string newSpaceRootDir)
{
    if(mp->verbose)
        printf("copy space\n");

    voxelsGrid* out = clone(newSpaceRootDir);

    long t1 = initEstimate();
    for(int i = 0; i < voxels->size() / 8; i++)
    {
        copySpaceVoxel(i, out);
        printfEstimate(i, voxels->size() / 8, t1);
    }
    finishEstimate();

    if(doVisualize)
        out->vizualize();

    return out;
}

void voxelsGrid::generateCamsPtsFromVoxelsTracks()
{
    if(mp->verbose)
        printf("distributing pts from voxels tracks to camera files\n");

    long t1 = initEstimate();
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

        if(FileExists(fileNameTracksPts))
        {
            staticVector<point3d>* tracksPoints = loadArrayFromFile<point3d>(fileNameTracksPts);
            staticVector<staticVector<pixel>*>* tracksPointsCams =
                loadArrayOfArraysFromFile<pixel>(fileNameTracksPtsCams);
            staticVector<int>* cams = loadArrayFromFile<int>(fileNameTracksCams);

            // printf("distributing %i tracks to %i camspts files  \n", tracksPoints->size(), cams->size());
            staticVector<staticVector<pixel>*>* camsTracksPoints =
                convertObjectsCamsToCamsObjects(mp, tracksPointsCams);

#pragma omp parallel for
            for(int c = 0; c < cams->size(); c++)
            {
                int rc = (*cams)[c];

                // open camPtsFile for append
                std::string camPtsFileName = spaceCamsTracksDir + "camPtsGrid_" + num2strFourDecimal(rc) + ".bin";
                FILE* fin = fopen(camPtsFileName.c_str(), "ab");
                staticVector<pixel>* camPtsIds = (*camsTracksPoints)[rc];
                for(int j = 0; j < sizeOfStaticVector<pixel>(camPtsIds); j++)
                {
                    int ptid = (*camPtsIds)[j].x;
                    int nrc = (*camPtsIds)[j].y;
                    GC_camVertexInfo p;
                    p.point = (*tracksPoints)[ptid];
                    p.sim = -0.91; // TODO FACA: why??
                    p.nrc = nrc;
                    p.ncams = sizeOfStaticVector<pixel>((*tracksPointsCams)[ptid]);
                    p.fwriteinfo(fin);
                }
                fclose(fin);
            } // for cams

            delete cams;
            delete tracksPoints;
            deleteArrayOfArrays<pixel>(&camsTracksPoints);
            deleteArrayOfArrays<pixel>(&tracksPointsCams);
        }

        printfEstimate(i, nvoxs, t1);
    } // for i
    finishEstimate();
}

void voxelsGrid::vizualize()
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
        if(FolderExists(subFoldeName.c_str()))
        {
            fprintf(f, "Inline{ url [\"%s\"] \n }\n", fname.c_str());
        }
    }
    fclose(f);
}

void voxelsGrid::getHexah(point3d* hexahOut, const voxel& LUi, const voxel& RDi)
{
    voxel LU, RD;
    LU.x = std::min(LUi.x, RDi.x);
    LU.y = std::min(LUi.y, RDi.y);
    LU.z = std::min(LUi.z, RDi.z);
    RD.x = std::max(LUi.x, RDi.x);
    RD.y = std::max(LUi.y, RDi.y);
    RD.z = std::max(LUi.z, RDi.z);

    int luid = getIdForVoxel(LU);

    // point3d *vox = &(*voxels)[0];
    // point3d O   = vox[0];
    // point3d vvx = vox[1]-vox[0];
    // point3d vvy = vox[3]-vox[0];
    // point3d vvz = vox[4]-vox[0];

    voxel X, Y, Z;
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

    point3d O = (*voxels)[luid * 8];
    point3d vvx = (*voxels)[Xid * 8 + 1] - O;
    point3d vvy = (*voxels)[Yid * 8 + 3] - O;
    point3d vvz = (*voxels)[Zid * 8 + 4] - O;

    hexahOut[0] = O;
    hexahOut[1] = O + vvx;
    hexahOut[2] = O + vvx + vvy;
    hexahOut[3] = O + vvy;
    hexahOut[4] = O + vvz;
    hexahOut[5] = O + vvz + vvx;
    hexahOut[6] = O + vvz + vvx + vvy;
    hexahOut[7] = O + vvz + vvy;
}

void voxelsGrid::cretatePSET(std::string psetFileName)
{
    printf("Crating pset file for PoissonRecon\n");

    FILE* f1 = fopen(psetFileName.c_str(), "w");
    int npset = 0;
    long t1 = initEstimate();
    for(int i = 0; i < voxels->size() / 8; i++)
    {
        std::string folderName = getVoxelFolderName(i);
        std::string fileNameTracksCams, fileNameTracksPts, fileNameTracksPtsCams;
        fileNameTracksCams = folderName + "tracksGridCams.bin";
        fileNameTracksPts = folderName + "tracksGridPts.bin";
        fileNameTracksPtsCams = folderName + "tracksGridPtsCams.bin";

        if(FileExists(fileNameTracksPts))
        {
            staticVector<point3d>* tracksPoints = loadArrayFromFile<point3d>(fileNameTracksPts);
            staticVector<staticVector<pixel>*>* tracksPointsCams =
                loadArrayOfArraysFromFile<pixel>(fileNameTracksPtsCams);

            for(int j = 0; j < tracksPoints->size(); j++)
            {
                point3d tp = (*tracksPoints)[j];
                staticVector<pixel>* cams = (*tracksPointsCams)[j];
                if(cams != nullptr)
                {
                    point3d n = point3d(0.0f, 0.0f, 0.0f);
                    for(int c = 0; c < cams->size(); c++)
                    {
                        int rc = (*cams)[c].x;
                        n = n + (mp->CArr[rc] - tp).normalize();
                    }
                    n = n / (float)cams->size();
                    n.normalize();

                    fprintf(f1, "%f %f %f %f %f %f\n", tp.x, tp.y, tp.z, -n.x, -n.y, -n.z);
                    npset++;
                }
            } // for j

            delete tracksPoints;
            deleteArrayOfArrays<pixel>(&tracksPointsCams);
        } // if fileexists

        printfEstimate(i, voxels->size() / 8, t1);
    } // for i
    finishEstimate();

    if(mp->verbose)
        printf("pset oriented points %i \n", npset);

    fclose(f1);
}
