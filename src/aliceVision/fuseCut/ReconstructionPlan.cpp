// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "ReconstructionPlan.hpp"
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/mvsData/Rgb.hpp>
#include <aliceVision/mvsUtils/common.hpp>
#include <aliceVision/mvsUtils/fileIO.hpp>
#include <aliceVision/mesh/meshPostProcessing.hpp>
#include <aliceVision/fuseCut/VoxelsGrid.hpp>
#include <aliceVision/fuseCut/DelaunayGraphCut.hpp>

#include <boost/filesystem.hpp>

namespace aliceVision {
namespace fuseCut {

namespace bfs = boost::filesystem;

ReconstructionPlan::ReconstructionPlan(Voxel& dimmensions, Point3d* space, mvsUtils::MultiViewParams* _mp, std::string _spaceRootDir)
    : VoxelsGrid(dimmensions, space, _mp, _spaceRootDir)
{
    nVoxelsTracks = getNVoxelsTracks();
}

ReconstructionPlan::~ReconstructionPlan()
{
    delete nVoxelsTracks;
}

StaticVector<int>* ReconstructionPlan::voxelsIdsIntersectingHexah(Point3d* hexah)
{
    StaticVector<int>* ids = new StaticVector<int>();
    ids->reserve(voxels->size() / 8);

    for(int i = 0; i < voxels->size() / 8; i++)
    {
        if(mvsUtils::intersectsHexahedronHexahedron(&(*voxels)[i * 8], hexah))
        {
            ids->push_back(i);
        }
    }

    return ids;
}

unsigned long ReconstructionPlan::getNTracks(const Voxel& LU, const Voxel& RD)
{
    unsigned long n = 0;
    Voxel v;
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

bool ReconstructionPlan::divideBox(Voxel& LU1o, Voxel& RD1o, Voxel& LU2o, Voxel& RD2o, const Voxel& LUi,
                                   const Voxel& RDi, unsigned long maxTracks)
{
    unsigned long n = getNTracks(LUi, RDi);
    if(n < maxTracks)
    {
        return false;
    }

    Voxel LUt, RDt;

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

    throw std::runtime_error("divideBox: no valid return condition");

    return false;
}

StaticVector<Point3d>* ReconstructionPlan::computeReconstructionPlanBinSearch(unsigned long maxTracks)
{
    Voxel actHexahLU = Voxel(0, 0, 0);
    Voxel actHexahRD = voxelDim - Voxel(1, 1, 1);

    /*
    printf("------------\n");
    printf("actHexahLU %i %i %i\n",actHexahLU.x,actHexahLU.y,actHexahLU.z);
    printf("actHexahRD %i %i %i\n",actHexahRD.x,actHexahRD.y,actHexahRD.z);
    */

    StaticVector<Point3d>* hexahsToReconstruct = new StaticVector<Point3d>();
    hexahsToReconstruct->reserve(nVoxelsTracks->size() * 8);

    StaticVector<Voxel>* toDivideLU = new StaticVector<Voxel>();
    StaticVector<Voxel>* toDivideRD = new StaticVector<Voxel>();

    toDivideLU->reserve(10 * nVoxelsTracks->size());
    toDivideRD->reserve(10 * nVoxelsTracks->size());

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

        Voxel LU1, RD1, LU2, RD2;
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
            Point3d hexah[8], hexahinf[8];

            /*
            printf("------------\n");
            printf("actHexahLU %i %i %i\n",actHexahLU.x,actHexahLU.y,actHexahLU.z);
            printf("actHexahRD %i %i %i\n",actHexahRD.x,actHexahRD.y,actHexahRD.z);
            */

            getHexah(hexah, actHexahLU, actHexahRD);
            mvsUtils::inflateHexahedron(hexah, hexahinf, 1.05);
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

void ReconstructionPlan::getHexahedronForID(float dist, int id, Point3d* out)
{
    mvsUtils::inflateHexahedron(&(*voxels)[id * 8], out, dist);
}

StaticVector<StaticVector<int>*>* loadLargeScalePtsCams(const std::vector<std::string>& recsDirs)
{
    StaticVector<StaticVector<int>*>* ptsCamsFromDct = new StaticVector<StaticVector<int>*>();
    for(int i = 0; i < recsDirs.size(); ++i)
    {
        std::string folderName = recsDirs[i];

        std::string filePtsCamsFromDCTName = folderName + "meshPtsCamsFromDGC.bin";

        if(!mvsUtils::FileExists(filePtsCamsFromDCTName))
        {
            delete ptsCamsFromDct;
            throw std::runtime_error("Missing file: " + filePtsCamsFromDCTName);
        }
        StaticVector<StaticVector<int>*>* ptsCamsFromDcti = loadArrayOfArraysFromFile<int>(filePtsCamsFromDCTName);
        ptsCamsFromDct->reserveAdd(ptsCamsFromDcti->size());
        for(int i = 0; i < ptsCamsFromDcti->size(); i++)
        {
            ptsCamsFromDct->push_back((*ptsCamsFromDcti)[i]);
        }
        delete ptsCamsFromDcti; //!!!NOT DELETE ARRAYOFARRAYS
    }
    return ptsCamsFromDct;
}

void loadLargeScalePtsCams(const std::vector<std::string>& recsDirs, StaticVector<StaticVector<int>>& out_ptsCams)
{
    for(int i = 0; i < recsDirs.size(); ++i)
    {
        std::string folderName = recsDirs[i];

        std::string filePtsCamsFromDCTName = folderName + "meshPtsCamsFromDGC.bin";

        if(!mvsUtils::FileExists(filePtsCamsFromDCTName))
        {
            throw std::runtime_error("Missing file: " + filePtsCamsFromDCTName);
        }
        loadArrayOfArraysFromFile<int>(out_ptsCams, filePtsCamsFromDCTName);
    }
}

StaticVector<rgb>* getTrisColorsRgb(mesh::Mesh* me, StaticVector<rgb>* ptsColors)
{
    StaticVector<rgb>* trisColors = new StaticVector<rgb>();
    trisColors->resize(me->tris.size());
    for(int i = 0; i < me->tris.size(); i++)
    {
        float r = 0.0f;
        float g = 0.0f;
        float b = 0.0f;
        for(int j = 0; j < 3; j++)
        {
            r += (float)(*ptsColors)[me->tris[i].v[j]].r;
            g += (float)(*ptsColors)[me->tris[i].v[j]].g;
            b += (float)(*ptsColors)[me->tris[i].v[j]].b;
        }
        (*trisColors)[i].r = (unsigned char)(r / 3.0f);
        (*trisColors)[i].g = (unsigned char)(g / 3.0f);
        (*trisColors)[i].b = (unsigned char)(b / 3.0f);
    }
    return trisColors;
}

mesh::Mesh* joinMeshes(const std::vector<std::string>& recsDirs, StaticVector<Point3d>* voxelsArray,
                    LargeScale* ls)
{
    ReconstructionPlan rp(ls->dimensions, &ls->space[0], ls->mp, ls->spaceVoxelsFolderName);

    ALICEVISION_LOG_DEBUG("Detecting size of merged mesh.");
    int npts = 0;
    int ntris = 0;
    for(int i = 0; i < recsDirs.size(); i++)
    {
        std::string folderName = recsDirs[i];

        std::string fileName = folderName + "mesh.bin";
        if(mvsUtils::FileExists(fileName))
        {
            mesh::Mesh* mei = new mesh::Mesh();
            mei->loadFromBin(fileName);
            npts += mei->pts.size();
            ntris += mei->tris.size();

            ALICEVISION_LOG_DEBUG("npts: " << npts << " " << mei->pts.size());
            ALICEVISION_LOG_DEBUG("ntris: " << ntris << " " << mei->tris.size());

            delete mei;
        }
    }

    ALICEVISION_LOG_DEBUG("Creating mesh.");

    mesh::Mesh* me = new mesh::Mesh();

    me->pts = StaticVector<Point3d>();
    me->pts.reserve(npts);
    me->tris = StaticVector<mesh::Mesh::triangle>();
    me->tris.reserve(ntris);

    StaticVector<rgb>* trisCols = new StaticVector<rgb>();
    trisCols->reserve(ntris);
    StaticVector<rgb>* ptsCols = new StaticVector<rgb>();
    ptsCols->reserve(npts);

    ALICEVISION_LOG_DEBUG("Merging part to one mesh without connecting them.");
    for(int i = 0; i < recsDirs.size(); i++)
    {
        if(ls->mp->verbose)
            ALICEVISION_LOG_DEBUG("Merging part: " << i);
        std::string folderName = recsDirs[i];

        std::string fileName = folderName + "mesh.bin";
        if(mvsUtils::FileExists(fileName))
        {
            mesh::Mesh* mei = new mesh::Mesh();
            mei->loadFromBin(fileName);

            // to remove artefacts on the border
            Point3d hexah[8];
            float inflateFactor = 0.96;
            mvsUtils::inflateHexahedron(&(*voxelsArray)[i * 8], hexah, inflateFactor);
            mei->removeTrianglesOutsideHexahedron(hexah);

            ALICEVISION_LOG_DEBUG("Adding mesh part "<< i << " to mesh");
            me->addMesh(*mei);

            ALICEVISION_LOG_DEBUG("Merging colors of part: s" << i);
            fileName = folderName + "meshAvImgCol.ply.ptsColors";
            if(mvsUtils::FileExists(fileName))
            {
                StaticVector<rgb>* ptsColsi = loadArrayFromFile<rgb>(fileName);
                StaticVector<rgb>* trisColsi = getTrisColorsRgb(mei, ptsColsi);

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

    // int gridLevel = ls->mp->_ini.get<int>("LargeScale.gridLevel0", 0);
    ALICEVISION_LOG_DEBUG("Deleting...");
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

mesh::Mesh* joinMeshes(int gl, LargeScale* ls)
{
    ReconstructionPlan* rp =
        new ReconstructionPlan(ls->dimensions, &ls->space[0], ls->mp, ls->spaceVoxelsFolderName);
    std::string param = "LargeScale:gridLevel" + mvsUtils::num2str(gl);
    int gridLevel = ls->mp->userParams.get<int>(param.c_str(), gl * 300);

    std::string optimalReconstructionPlanFileName =
        ls->spaceFolderName + "optimalReconstructionPlan" + mvsUtils::num2str(gridLevel) + ".bin";
    StaticVector<SortedId>* optimalReconstructionPlan = loadArrayFromFile<SortedId>(optimalReconstructionPlanFileName);

    auto subFolderName = ls->mp->userParams.get<std::string>("LargeScale.subFolderName", "");
    if(subFolderName.empty())
    {
        if(ls->mp->userParams.get<bool>("global.LabatutCFG09", false))
        {
            subFolderName = "LabatutCFG09";
        }
    }
    subFolderName = subFolderName + "/";

    StaticVector<Point3d>* voxelsArray = new StaticVector<Point3d>();
    voxelsArray->reserve(optimalReconstructionPlan->size() * 8);

    std::vector<std::string> recsDirs;
    for(int i = 0; i < optimalReconstructionPlan->size(); i++)
    {
        int id = (*optimalReconstructionPlan)[i].id;
        float inflateFactor = (*optimalReconstructionPlan)[i].value;
        std::string folderName = ls->spaceFolderName + "reconstructedSpacePart" + mvsUtils::num2strFourDecimal(id) + "/";
        folderName +=  "GL_" + mvsUtils::num2str(gridLevel) + "_IF_" + mvsUtils::num2str((int)inflateFactor) + "/";

        Point3d hexah[8];
        rp->getHexahedronForID(inflateFactor, id, hexah);
        for(int k = 0; k < 8; k++)
        {
            voxelsArray->push_back(hexah[k]);
        }

        recsDirs.push_back(folderName);
    }
    delete optimalReconstructionPlan;
    delete rp;

    mesh::Mesh* me = joinMeshes(recsDirs, voxelsArray, ls);
    delete voxelsArray;

    return me;
}

mesh::Mesh* joinMeshes(const std::string& voxelsArrayFileName, LargeScale* ls)
{
    StaticVector<Point3d>* voxelsArray = loadArrayFromFile<Point3d>(voxelsArrayFileName);
    std::vector<std::string> recsDirs = ls->getRecsDirs(voxelsArray);

    mesh::Mesh* me = joinMeshes(recsDirs, voxelsArray, ls);
    delete voxelsArray;

    return me;
}

} // namespace fuseCut
} // namespace aliceVision
