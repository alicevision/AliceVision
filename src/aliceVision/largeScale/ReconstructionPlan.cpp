// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "ReconstructionPlan.hpp"
#include <aliceVision/common/common.hpp>
#include <aliceVision/common/fileIO.hpp>
#include <aliceVision/mesh/plySaver.hpp>
#include <aliceVision/delaunaycut/mv_delaunay_GC.hpp>
#include <aliceVision/delaunaycut/mv_delaunay_meshSmooth.hpp>
#include <aliceVision/largeScale/VoxelsGrid.hpp>

#include <boost/filesystem.hpp>

namespace bfs = boost::filesystem;

ReconstructionPlan::ReconstructionPlan(voxel& dimmensions, point3d* space, multiviewParams* _mp, mv_prematch_cams* _pc,
                                       std::string _spaceRootDir)
    : VoxelsGrid(dimmensions, space, _mp, _pc, _spaceRootDir)
{
    nVoxelsTracks = getNVoxelsTracks();
}

ReconstructionPlan::~ReconstructionPlan()
{
    delete nVoxelsTracks;
}

staticVector<int>* ReconstructionPlan::getNeigboursIds(float dist, int id, bool ceilOrFloor)
{
    staticVector<int>* ids = new staticVector<int>(voxels->size() / 8);
    voxel v = getVoxelForId(id);

    int distN = (int)ceil((dist - 1.0f) / 2.0f);
    if(!ceilOrFloor)
    {
        distN = (int)floor((dist - 1.0f) / 2.0f);
    }
    // printf("dist %i, voxel %i (%i,%i,%i)\n", distN, id, v.x,v.y,v.z);

    for(int x = -distN; x <= distN; x++)
    {
        for(int y = -distN; y <= distN; y++)
        {
            for(int z = -distN; z <= distN; z++)
            {
                voxel v1 = v + voxel(x, y, z);
                // printf("voxel1 (%i,%i,%i)\n", v1.x,v1.y,v1.z);
                if(isValidVoxel(v1))
                {
                    ids->push_back(getIdForVoxel(v1));
                }
            }
        }
    }

    return ids;
}

staticVector<int>* ReconstructionPlan::voxelsIdsIntersectingHexah(point3d* hexah)
{
    staticVector<int>* ids = new staticVector<int>(voxels->size() / 8);

    for(int i = 0; i < voxels->size() / 8; i++)
    {
        if(intersectsHexahedronHexahedron(&(*voxels)[i * 8], hexah))
        {
            ids->push_back(i);
        }
    }

    return ids;
}

int ReconstructionPlan::getPtsCount(float dist, int id)
{
    int npts = 0;
    staticVector<int>* ids = getNeigboursIds(dist, id, false);
    for(int i = 0; i < ids->size(); i++)
    {
        int actVoxId = (*ids)[i];
        npts += (*nVoxelsTracks)[actVoxId];
    }
    delete ids;

    // printf("voxel %i, dist %f, npts %i\n", id, dist, npts);
    return npts;
}

staticVector<float>* ReconstructionPlan::computeMaximaInflateFactors(int maxPts)
{
    float maxDim = std::max(std::max(voxelDim.x, voxelDim.y), voxelDim.z) * 2.0f;
    staticVector<float>* out = new staticVector<float>(voxels->size() / 8);
    for(int i = 0; i < voxels->size() / 8; i++)
    {
        printf("Computing max reconstructable inflate factor for voxel %i\n", i);

        float maxDist = 1.0f;
        int npts = getPtsCount(maxDist, i);
        float lastGoodMaxDist = 0.0f;
        while((npts < maxPts) && (maxDist < maxDim))
        {
            if(npts > 0)
            {
                lastGoodMaxDist = maxDist;
            }
            maxDist += 2.0f;
            npts = getPtsCount(maxDist, i);
        }
        maxDist = lastGoodMaxDist;

        printf("Max reconstructable inflate factor for voxels %i, is %f, npts %i, maxPts %i\n", i, maxDist, npts,
               maxPts);
        out->push_back(maxDist);
    }
    printf("Maximal tracks in a voxel %i\n", nVoxelsTracks->maxVal());

    return out;
}

staticVector<sortedId>* ReconstructionPlan::computeOptimalReconstructionPlan(const staticVector<float>* maximaInflateFactors)
{
    staticVector<sortedId>* out = new staticVector<sortedId>(maximaInflateFactors->size());
    staticVector<sortedId>* voxelsMIFsID = new staticVector<sortedId>(maximaInflateFactors->size());
    float maxif = 0.0f;
    for(int i = 0; i < maximaInflateFactors->size(); i++)
    {
        voxelsMIFsID->push_back(sortedId(i, (*maximaInflateFactors)[i]));
        maxif = std::max(maxif, (*maximaInflateFactors)[i]);
    }
    // take the voxelid with minimal maximal inflate factor because it is the denselly sampled part
    qsort(&(*voxelsMIFsID)[0], voxelsMIFsID->size(), sizeof(sortedId), qsortCompareSortedIdAsc);

    while((*voxelsMIFsID)[0].value <= maxif)
    {
        if((*voxelsMIFsID)[0].value < 1.0f)
        {
            (*voxelsMIFsID)[0].value = maxif + 1.0f;
        }
        else
        {
            int id = (*voxelsMIFsID)[0].id;
            int npts = getPtsCount((*voxelsMIFsID)[0].value, id);
            if(npts > 1000)
            {
                out->push_back((*voxelsMIFsID)[0]);
                printf("reconstruct voxel %i at dist %f npts %i \n", id, (*voxelsMIFsID)[0].value, npts);
            }

            // get interlan voxels
            staticVector<int>* ids = getNeigboursIds((*voxelsMIFsID)[0].value, id, false);

            // sort it by id to allow direct addressing
            qsort(&(*voxelsMIFsID)[0], voxelsMIFsID->size(), sizeof(sortedId), qsortCompareSortedIdByIdAsc);

            // exclude internal voxels from further computation
            for(int i = 0; i < ids->size(); i++)
            {
                int centerVoxId = (*ids)[i];
                (*voxelsMIFsID)[centerVoxId].value = maxif + 1.0f;
            }
            delete ids;
        }

        qsort(&(*voxelsMIFsID)[0], voxelsMIFsID->size(), sizeof(sortedId), qsortCompareSortedIdAsc);
    }

    delete voxelsMIFsID;

    return out;
}

unsigned long ReconstructionPlan::getNTracks(const voxel& LU, const voxel& RD)
{
    unsigned long n = 0;
    voxel v;
    for(v.x = LU.x; v.x <= RD.x; v.x++)
    {
        for(v.y = LU.y; v.y <= RD.y; v.y++)
        {
            for(v.z = LU.z; v.z <= RD.z; v.z++)
            {
                n += (*nVoxelsTracks)[getIdForVoxel(v)];
            }
        }
    }
    return n;
}

bool ReconstructionPlan::divideBox(voxel& LU1o, voxel& RD1o, voxel& LU2o, voxel& RD2o, const voxel& LUi,
                                   const voxel& RDi, unsigned long maxTracks)
{
    unsigned long n = getNTracks(LUi, RDi);
    if(n < maxTracks)
    {
        return false;
    }

    voxel LUt, RDt;

    unsigned long nx1, nx2, ny1, ny2, nz1, nz2;
    nx1 = 0;
    nx2 = 0;
    ny1 = 0;
    ny2 = 0;
    nz1 = 0;
    nz2 = 0;

    if(RDi.x - LUi.x > 0)
    {
        LUt = LUi;
        RDt = RDi;
        RDt.x = LUi.x + (RDi.x - LUi.x) / 2;
        nx1 = getNTracks(LUt, RDt);
        LUt = LUi;
        RDt = RDi;
        LUt.x = LUi.x + (RDi.x - LUi.x) / 2 + 1;
        nx2 = getNTracks(LUt, RDt);
    }

    if(RDi.y - LUi.y > 0)
    {
        LUt = LUi;
        RDt = RDi;
        RDt.y = LUi.y + (RDi.y - LUi.y) / 2;
        ny1 = getNTracks(LUt, RDt);
        LUt = LUi;
        RDt = RDi;
        LUt.y = LUi.y + (RDi.y - LUi.y) / 2 + 1;
        ny2 = getNTracks(LUt, RDt);
    }

    if(RDi.z - LUi.z > 0)
    {
        LUt = LUi;
        RDt = RDi;
        RDt.z = LUi.z + (RDi.z - LUi.z) / 2;
        nz1 = getNTracks(LUt, RDt);
        LUt = LUi;
        RDt = RDi;
        LUt.z = LUi.z + (RDi.z - LUi.z) / 2 + 1;
        nz2 = getNTracks(LUt, RDt);
    }

    if(RDi.x - LUi.x > 0)
    {
        if((abs(RDi.x - LUi.x) >= abs(RDi.y - LUi.y)) && (abs(RDi.x - LUi.x) >= abs(RDi.z - LUi.z)))
        {
            LU1o = LUi;
            RD1o = RDi;
            RD1o.x = LUi.x + (RDi.x - LUi.x) / 2;
            LU2o = LUi;
            RD2o = RDi;
            LU2o.x = LUi.x + (RDi.x - LUi.x) / 2 + 1;
            return true;
        }
    }

    if(RDi.y - LUi.y > 0)
    {
        if((abs(RDi.y - LUi.y) >= abs(RDi.x - LUi.x)) && (abs(RDi.y - LUi.y) >= abs(RDi.z - LUi.z)))
        {
            LU1o = LUi;
            RD1o = RDi;
            RD1o.y = LUi.y + (RDi.y - LUi.y) / 2;
            LU2o = LUi;
            RD2o = RDi;
            LU2o.y = LUi.y + (RDi.y - LUi.y) / 2 + 1;
            return true;
        }
    }

    if(RDi.z - LUi.z > 0)
    {
        if((abs(RDi.z - LUi.z) >= abs(RDi.x - LUi.x)) && (abs(RDi.z - LUi.z) >= abs(RDi.y - LUi.y)))
        {
            LU1o = LUi;
            RD1o = RDi;
            RD1o.z = LUi.z + (RDi.z - LUi.z) / 2;
            LU2o = LUi;
            RD2o = RDi;
            LU2o.z = LUi.z + (RDi.z - LUi.z) / 2 + 1;
            return true;
        }
    }

    printf("WARNING should not happen\n");

    return false;
}

staticVector<point3d>* ReconstructionPlan::computeReconstructionPlanBinSearch(unsigned long maxTracks)
{
    voxel actHexahLU = voxel(0, 0, 0);
    voxel actHexahRD = voxelDim - voxel(1, 1, 1);

    /*
    printf("------------\n");
    printf("actHexahLU %i %i %i\n",actHexahLU.x,actHexahLU.y,actHexahLU.z);
    printf("actHexahRD %i %i %i\n",actHexahRD.x,actHexahRD.y,actHexahRD.z);
    */

    staticVector<point3d>* hexahsToReconstruct = new staticVector<point3d>(nVoxelsTracks->size() * 8);

    staticVector<voxel>* toDivideLU = new staticVector<voxel>(10 * nVoxelsTracks->size());
    staticVector<voxel>* toDivideRD = new staticVector<voxel>(10 * nVoxelsTracks->size());
    toDivideLU->push_back(actHexahLU);
    toDivideRD->push_back(actHexahRD);

    while(toDivideLU->size() > 0)
    {
        actHexahLU = toDivideLU->pop();
        actHexahRD = toDivideRD->pop();

        /*
        printf("------------\n");
        printf("actHexahLU %i %i %i\n",actHexahLU.x,actHexahLU.y,actHexahLU.z);
        printf("actHexahRD %i %i %i\n",actHexahRD.x,actHexahRD.y,actHexahRD.z);
        */

        voxel LU1, RD1, LU2, RD2;
        if(divideBox(LU1, RD1, LU2, RD2, actHexahLU, actHexahRD, maxTracks))
        {
            toDivideLU->push_back(LU1);
            toDivideRD->push_back(RD1);
            toDivideLU->push_back(LU2);
            toDivideRD->push_back(RD2);

            /*
            printf("------------\n");
            printf("actHexahLU %i %i %i\n",actHexahLU.x,actHexahLU.y,actHexahLU.z);
            printf("actHexahRD %i %i %i\n",actHexahRD.x,actHexahRD.y,actHexahRD.z);
            printf("LU1 %i %i %i\n",LU1.x,LU1.y,LU1.z);
            printf("RD1 %i %i %i\n",RD1.x,RD1.y,RD1.z);
            printf("LU2 %i %i %i\n",LU2.x,LU2.y,LU2.z);
            printf("RD2 %i %i %i\n",RD2.x,RD2.y,RD2.z);
            */
        }
        else
        {
            point3d hexah[8], hexahinf[8];

            /*
            printf("------------\n");
            printf("actHexahLU %i %i %i\n",actHexahLU.x,actHexahLU.y,actHexahLU.z);
            printf("actHexahRD %i %i %i\n",actHexahRD.x,actHexahRD.y,actHexahRD.z);
            */

            getHexah(hexah, actHexahLU, actHexahRD);
            inflateHexahedron(hexah, hexahinf, 1.05);
            for(int k = 0; k < 8; k++)
            {
                hexahsToReconstruct->push_back(hexahinf[k]);
            }
        }
    }

    delete toDivideLU;
    delete toDivideRD;

    return hexahsToReconstruct;
}

void ReconstructionPlan::getHexahedronForID(float dist, int id, point3d* out)
{
    inflateHexahedron(&(*voxels)[id * 8], out, dist);
}

void vizualizeReconstructionPlan(int gridLevel, LargeScale* ls, staticVector<sortedId>* optimalReconstructionPlan)
{
    auto subFolderName = ls->mp->mip->_ini.get<std::string>("LargeScale.subFolderName", "");
    if(subFolderName.empty())
    {
        if(ls->mp->mip->_ini.get<bool>("global.LabatutCFG09", false))
        {
            subFolderName = "LabatutCFG09";
        }
        if(ls->mp->mip->_ini.get<bool>("global.JancosekCVPR11", true))
        {
            subFolderName = "JancosekCVPR11";
        }
    }
    subFolderName = subFolderName + "/";

    {
        std::string spaceWrlFileName = ls->spaceFolderName + "reconstructedSpaceLevel" + num2str(gridLevel) + ".wrl";
        FILE* f = fopen(spaceWrlFileName.c_str(), "w");
        fprintf(f, "#VRML V2.0 utf8\n");
        fprintf(f, "Background {\n skyColor 1 1 1 \n } \n");

        for(int i = 0; i < optimalReconstructionPlan->size(); i++)
        {
            int id = (*optimalReconstructionPlan)[i].id;
            float inflateFactor = (*optimalReconstructionPlan)[i].value;
            std::string folderName = "reconstructedSpacePart" + num2strFourDecimal(id) + "/";
            folderName = folderName + "GL_" + num2str(gridLevel) + "_IF_" + num2str((int)inflateFactor) + "/";

            std::string fileName = folderName + subFolderName + "meshAvImgTex.wrl";
            fprintf(f, "Inline{ url [\"%s\"] \n }\n", fileName.c_str());
        }

        std::string fileName = "cameras.wrl";
        fprintf(f, "Inline{ url [\"%s\"] \n }\n", fileName.c_str());
        fclose(f);
    }

    {
        std::string spaceWrlFileName =
            ls->spaceFolderName + "reconstructedSpaceLevel" + num2str(gridLevel) + "Simplyfied10.wrl";
        FILE* f = fopen(spaceWrlFileName.c_str(), "w");
        fprintf(f, "#VRML V2.0 utf8\n");
        fprintf(f, "Background {\n skyColor 1 1 1 \n } \n");

        for(int i = 0; i < optimalReconstructionPlan->size(); i++)
        {
            int id = (*optimalReconstructionPlan)[i].id;
            float inflateFactor = (*optimalReconstructionPlan)[i].value;
            std::string folderName = "reconstructedSpacePart" + num2strFourDecimal(id) + "/";
            folderName += "GL_" + num2str(gridLevel) + "_IF_" + num2str((int)inflateFactor) + "/";

            std::string fileName = folderName + subFolderName + "simplyfied10/meshAvImgTex.wrl";
            fprintf(f, "Inline{ url [\"%s\"] \n }\n", fileName.c_str());
        }

        std::string fileName = "cameras.wrl";
        fprintf(f, "Inline{ url [\"%s\"] \n }\n", fileName.c_str());
        fclose(f);
    }
}

void reconstructAccordingToOptimalReconstructionPlan(int gl, LargeScale* ls)
{
    std::string param = "LargeScale:gridLevel" + num2str(gl);
    int gridLevel = ls->mp->mip->_ini.get<int>(param.c_str(), gl * 300);

    std::string optimalReconstructionPlanFileName =
        ls->spaceFolderName + "optimalReconstructionPlan" + num2str(gridLevel) + ".bin";
    staticVector<sortedId>* optimalReconstructionPlan = loadArrayFromFile<sortedId>(optimalReconstructionPlanFileName);

    std::string optimalReconstructionPlanDimTransformFileName =
        ls->spaceFolderName + "optimalReconstructionPlandimTransform.bin";
    ReconstructionPlan* rp =
        new ReconstructionPlan(ls->dimensions, &ls->space[0], ls->mp, ls->pc, ls->spaceVoxelsFolderName);

    staticVector<point3d>* hexahsToExcludeFromResultingMesh =
        new staticVector<point3d>(optimalReconstructionPlan->size() * 8);
    for(int i = 0; i < optimalReconstructionPlan->size(); i++)
    {
        int id = (*optimalReconstructionPlan)[i].id;
        float inflateFactor = (*optimalReconstructionPlan)[i].value;

        printf("RECONSTRUCTING %i VOXELS AT GRID LEVEL %i AND WITH INFLATE FACTOR %f\n", id, gridLevel, inflateFactor);

        std::string folderName = ls->spaceFolderName + "reconstructedSpacePart" + num2strFourDecimal(id) + "/";
        bfs::create_directory(folderName);
        folderName = folderName + "GL_" + num2str(gridLevel) + "_IF_" + num2str((int)inflateFactor) + "/";
        bfs::create_directory(folderName);

        inflateFactor *= 1.1f;
        point3d hexah[8];
        rp->getHexahedronForID(inflateFactor, id, hexah);

        std::string testFn = folderName + "stGraphSolution.bin";
        // if (FileExists(testFn)==false)
        //{
        staticVector<int>* voxelNeighs = rp->getNeigboursIds(inflateFactor, id, true);
        mv_delaunay_GC* dgc = new mv_delaunay_GC(ls->mp, ls->pc);
        dgc->reconstructVoxel(hexah, voxelNeighs, folderName, ls->getSpaceCamsTracksDir(), false,
                              hexahsToExcludeFromResultingMesh, (VoxelsGrid*)rp, ls->getSpaceSteps());
        delete dgc;
        delete voxelNeighs;
        //};

        point3d hexahThin[8];
        inflateHexahedron(hexah, hexahThin, 0.9);
        for(int k = 0; k < 8; k++)
        {
            hexahsToExcludeFromResultingMesh->push_back(hexahThin[k]);
        }
    }
    delete hexahsToExcludeFromResultingMesh;

    delete optimalReconstructionPlan;
    delete rp;
}

void reconstructSpaceAccordingToVoxelsArray(const std::string& voxelsArrayFileName, LargeScale* ls,
                                            bool doComputeColoredMeshes)
{
    staticVector<point3d>* voxelsArray = loadArrayFromFile<point3d>(voxelsArrayFileName);

    ReconstructionPlan* rp =
        new ReconstructionPlan(ls->dimensions, &ls->space[0], ls->mp, ls->pc, ls->spaceVoxelsFolderName);

    staticVector<point3d>* hexahsToExcludeFromResultingMesh = new staticVector<point3d>(voxelsArray->size());
    for(int i = 0; i < voxelsArray->size() / 8; i++)
    {
        printf("RECONSTRUCTING %i-th VOXEL OF %i \n", i, voxelsArray->size() / 8);

        const std::string folderName = ls->getReconstructionVoxelFolder(i);
        bfs::create_directory(folderName);

        const std::string meshBinFilepath = folderName + "mesh.bin";
        if(!FileExists(meshBinFilepath))
        {
            staticVector<int>* voxelsIds = rp->voxelsIdsIntersectingHexah(&(*voxelsArray)[i * 8]);
            mv_delaunay_GC delaunayGC(ls->mp, ls->pc);
            point3d* hexah = &(*voxelsArray)[i * 8];
            delaunayGC.reconstructVoxel(hexah, voxelsIds, folderName, ls->getSpaceCamsTracksDir(), false,
                                  hexahsToExcludeFromResultingMesh, (VoxelsGrid*)rp, ls->getSpaceSteps());
            delete voxelsIds;

            // Save mesh as .bin and .obj
            Mesh* mesh = delaunayGC.createMesh();
            staticVector<staticVector<int>*>* ptsCams = delaunayGC.createPtsCams();
            staticVector<int> usedCams = delaunayGC.getSortedUsedCams();

            meshPostProcessing(mesh, ptsCams, usedCams, *ls->mp, *ls->pc, ls->mp->mip->mvDir, hexahsToExcludeFromResultingMesh, hexah);
            mesh->saveToBin(folderName + "mesh.bin");
            mesh->saveToObj(folderName + "mesh.obj");

            saveArrayOfArraysToFile<int>(folderName + "meshPtsCamsFromDGC.bin", ptsCams);
            deleteArrayOfArrays<int>(&ptsCams);

            delete mesh;
        }

        /*
        if(doComputeColoredMeshes)
        {
            std::string resultFolderName = folderName + "/";
            computeColoredMesh(resultFolderName, ls);
        }
        */

        point3d hexahThin[8];
        inflateHexahedron(&(*voxelsArray)[i * 8], hexahThin, 0.9);
        for(int k = 0; k < 8; k++)
        {
            hexahsToExcludeFromResultingMesh->push_back(hexahThin[k]);
        }
    }
    delete hexahsToExcludeFromResultingMesh;

    delete rp;
    delete voxelsArray;
}


staticVector<staticVector<int>*>* loadLargeScalePtsCams(const std::vector<std::string>& recsDirs)
{
    staticVector<staticVector<int>*>* ptsCamsFromDct = new staticVector<staticVector<int>*>();
    for(int i = 0; i < recsDirs.size(); ++i)
    {
        std::string folderName = recsDirs[i];

        std::string filePtsCamsFromDCTName = folderName + "meshPtsCamsFromDGC.bin";
        if(!FileExists(filePtsCamsFromDCTName))
            throw std::runtime_error("Missing file: " + filePtsCamsFromDCTName);
        staticVector<staticVector<int>*>* ptsCamsFromDcti = loadArrayOfArraysFromFile<int>(filePtsCamsFromDCTName);
        ptsCamsFromDct->resizeAdd(ptsCamsFromDcti->size());
        for(int i = 0; i < ptsCamsFromDcti->size(); i++)
        {
            ptsCamsFromDct->push_back((*ptsCamsFromDcti)[i]);
        }
        delete ptsCamsFromDcti; //!!!NOT DELETE ARRAYOFARRAYS
    }
    return ptsCamsFromDct;
}


Mesh* joinMeshes(const std::vector<std::string>& recsDirs, staticVector<point3d>* voxelsArray,
                    LargeScale* ls)
{
    ReconstructionPlan rp(ls->dimensions, &ls->space[0], ls->mp, ls->pc, ls->spaceVoxelsFolderName);

    if(ls->mp->verbose)
        printf("Detecting size of merged mesh\n");
    int npts = 0;
    int ntris = 0;
    for(int i = 0; i < recsDirs.size(); i++)
    {
        std::string folderName = recsDirs[i];

        std::string fileName = folderName + "mesh.bin";
        if(FileExists(fileName))
        {
            Mesh* mei = new Mesh();
            mei->loadFromBin(fileName);
            npts += mei->pts->size();
            ntris += mei->tris->size();

            printf("npts %i %i \n", npts, mei->pts->size());
            printf("ntris %i %i \n", ntris, mei->tris->size());

            delete mei;
        }
    }

    if(ls->mp->verbose)
        printf("Creating mesh\n");
    Mesh* me = new Mesh();
    me->pts = new staticVector<point3d>(npts);
    me->tris = new staticVector<Mesh::triangle>(ntris);

    staticVector<rgb>* trisCols = new staticVector<rgb>(ntris);
    staticVector<rgb>* ptsCols = new staticVector<rgb>(npts);

    if(ls->mp->verbose)
        printf("Merging part to one mesh (not connecting them!!!)\n");
    for(int i = 0; i < recsDirs.size(); i++)
    {
        if(ls->mp->verbose)
            printf("Merging part %i\n", i);
        std::string folderName = recsDirs[i];

        std::string fileName = folderName + "mesh.bin";
        if(FileExists(fileName))
        {
            Mesh* mei = new Mesh();
            mei->loadFromBin(fileName);

            // to remove artefacts on the border
            point3d hexah[8];
            float inflateFactor = 0.96;
            inflateHexahedron(&(*voxelsArray)[i * 8], hexah, inflateFactor);
            mei->removeTrianglesOutsideHexahedron(hexah);

            if(ls->mp->verbose)
                printf("Adding mesh part %i to mesh\n", i);
            me->addMesh(mei);

            if(ls->mp->verbose)
                printf("Merging colors of part %i\n", i);
            fileName = folderName + "meshAvImgCol.ply.ptsColors";
            if(FileExists(fileName))
            {
                staticVector<rgb>* ptsColsi = loadArrayFromFile<rgb>(fileName);
                staticVector<rgb>* trisColsi = getTrisColorsRgb(mei, ptsColsi);

                for(int j = 0; j < trisColsi->size(); j++)
                {
                    trisCols->push_back((*trisColsi)[j]);
                }
                for(int j = 0; j < ptsColsi->size(); j++)
                {
                    ptsCols->push_back((*ptsColsi)[j]);
                }
                delete ptsColsi;
                delete trisColsi;
            }

            delete mei;
        }
    }

    // int gridLevel = ls->mp->mip->_ini.get<int>("LargeScale.gridLevel0", 0);

    if(ls->mp->verbose)
        printf("Deleting\n");
    delete ptsCols;
    delete trisCols;

    //if(ls->mp->verbose)
    //    printf("Creating QS\n");
    // createQSfileFromMesh(ls->spaceFolderName + "reconstructedSpaceLevelJoinedMesh"+num2str(gridLevel)+".bin",
    // ls->spaceFolderName+"meshAvImgCol.ply.ptsColors", ls->spaceFolderName +
    // "reconstructedSpaceLevelJoinedMesh"+num2str(gridLevel)+".qs");
#ifdef QSPLAT
    createQSfileFromMesh(spaceBinFileName, spacePtsColsBinFileName, outDir + "mesh.qs");
#endif

    return me;
}

Mesh* joinMeshes(int gl, LargeScale* ls)
{
    ReconstructionPlan* rp =
        new ReconstructionPlan(ls->dimensions, &ls->space[0], ls->mp, ls->pc, ls->spaceVoxelsFolderName);
    std::string param = "LargeScale:gridLevel" + num2str(gl);
    int gridLevel = ls->mp->mip->_ini.get<int>(param.c_str(), gl * 300);

    std::string optimalReconstructionPlanFileName =
        ls->spaceFolderName + "optimalReconstructionPlan" + num2str(gridLevel) + ".bin";
    staticVector<sortedId>* optimalReconstructionPlan = loadArrayFromFile<sortedId>(optimalReconstructionPlanFileName);

    auto subFolderName = ls->mp->mip->_ini.get<std::string>("LargeScale.subFolderName", "");
    if(subFolderName.empty())
    {
        if(ls->mp->mip->_ini.get<bool>("global.LabatutCFG09", false))
        {
            subFolderName = "LabatutCFG09";
        }
        if(ls->mp->mip->_ini.get<bool>("global.JancosekCVPR11", true))
        {
            subFolderName = "JancosekCVPR11";
        }
    }
    subFolderName = subFolderName + "/";

    staticVector<point3d>* voxelsArray = new staticVector<point3d>(optimalReconstructionPlan->size() * 8);
    std::vector<std::string> recsDirs;
    for(int i = 0; i < optimalReconstructionPlan->size(); i++)
    {
        int id = (*optimalReconstructionPlan)[i].id;
        float inflateFactor = (*optimalReconstructionPlan)[i].value;
        std::string folderName = ls->spaceFolderName + "reconstructedSpacePart" + num2strFourDecimal(id) + "/";
        folderName +=  "GL_" + num2str(gridLevel) + "_IF_" + num2str((int)inflateFactor) + "/";

        point3d hexah[8];
        rp->getHexahedronForID(inflateFactor, id, hexah);
        for(int k = 0; k < 8; k++)
        {
            voxelsArray->push_back(hexah[k]);
        }

        recsDirs.push_back(folderName);
    }
    delete optimalReconstructionPlan;
    delete rp;

    Mesh* me = joinMeshes(recsDirs, voxelsArray, ls);
    delete voxelsArray;

    return me;
}

Mesh* joinMeshes(const std::string& voxelsArrayFileName, LargeScale* ls)
{
    staticVector<point3d>* voxelsArray = loadArrayFromFile<point3d>(voxelsArrayFileName);
    std::vector<std::string> recsDirs = ls->getRecsDirs(voxelsArray);

    Mesh* me = joinMeshes(recsDirs, voxelsArray, ls);
    delete voxelsArray;

    return me;
}

