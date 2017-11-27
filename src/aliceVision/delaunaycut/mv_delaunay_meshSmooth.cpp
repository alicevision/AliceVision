// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "mv_delaunay_meshSmooth.hpp"
#include "mv_delaunay_GC.hpp"

#include <aliceVision/CUDAInterfaces/cuda_plane_sweeping.hpp>
#include <aliceVision/CUDAInterfaces/refine.hpp>

#include <aliceVision/hallucinations/hallucinations.hpp>
#include <aliceVision/mesh/mv_mesh_energy_opt_photo_mem.hpp>
#include <aliceVision/output3D/mv_output3D.hpp>
#include <aliceVision/planeSweeping/ps_sgm_params.hpp>
#include <aliceVision/structures/mv_images_cache.hpp>
#include <aliceVision/structures/mv_point3d.hpp>
#include <aliceVision/structures/mv_staticVector.hpp>
#include <aliceVision/prematching/mv_prematch_cams.hpp>

#include <boost/filesystem/operations.hpp>

namespace bfs = boost::filesystem;

void meshPostProcessing(mv_mesh*& inout_mesh, staticVector<staticVector<int>*>*& inout_ptsCams, staticVector<int>& usedCams,
                      multiviewParams& mp, mv_prematch_cams& pc,
                      const std::string& resultFolderName,
                      staticVector<point3d>* hexahsToExcludeFromResultingMesh, point3d* hexah)
{
    long timer = std::clock();
    std::cout << "meshPostProcessing" << std::endl;
    mv_output3D o3d(&mp);

    bool exportDebug = (float)mp.mip->_ini.get<bool>("delaunaycut.exportDebugGC", false);

    if(exportDebug)
        o3d.saveMvMeshToObj(inout_mesh, resultFolderName + "rawGraphCut.obj");

    const auto doRemoveHugeTriangles =
            mp.mip->_ini.get<bool>("hallucinationsFiltering.doRemoveHugeTriangles", false);
    if(doRemoveHugeTriangles)
    {
        filterLargeEdgeTriangles(
            inout_mesh, (float)mp.mip->_ini.get<double>("hallucinationsFiltering.NbAverageEdgeLength", 60.0));
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////

    // copy ptsCams
    {
        staticVector<staticVector<int>*>* ptsCamsOld = inout_ptsCams;
        staticVector<int>* ptIdToNewPtId;

        //!!!!!!!!!!!!!
        bool doRemoveTrianglesInhexahsToExcludeFromResultingMesh =
            (bool)mp.mip->_ini.get<bool>("largeScale.doRemoveTrianglesInhexahsToExcludeFromResultingMesh",
                                       false);
        if(doRemoveTrianglesInhexahsToExcludeFromResultingMesh && hexahsToExcludeFromResultingMesh)
        {
            inout_mesh->removeTrianglesInHexahedrons(hexahsToExcludeFromResultingMesh);
        }

        const auto doLeaveLargestFullSegmentOnly = (bool)mp.mip->_ini.get<bool>("hallucinationsFiltering.doLeaveLargestFullSegmentOnly", false);
        if(doLeaveLargestFullSegmentOnly)
        {
            staticVector<int>* trisIdsToStay = inout_mesh->getLargestConnectedComponentTrisIds(mp);
            inout_mesh->letJustTringlesIdsInMesh(trisIdsToStay);
            delete trisIdsToStay;
        }

        inout_mesh->removeFreePointsFromMesh(&ptIdToNewPtId);

        inout_ptsCams = new staticVector<staticVector<int>*>(inout_mesh->pts->size());
        for(int i = 0; i < inout_mesh->pts->size(); i++)
        {
            inout_ptsCams->push_back(nullptr);
        }
        for(int i = 0; i < ptIdToNewPtId->size(); i++)
        {
            int newId = (*ptIdToNewPtId)[i];
            if(newId > -1)
            {
                staticVector<int>* ptCamsNew = new staticVector<int>(sizeOfStaticVector<int>((*ptsCamsOld)[i]));
                for(int j = 0; j < sizeOfStaticVector<int>((*ptsCamsOld)[i]); j++)
                {
                    ptCamsNew->push_back((*(*ptsCamsOld)[i])[j]);
                }
                (*inout_ptsCams)[newId] = ptCamsNew;
            }
        }

        deleteArrayOfArrays<int>(&ptsCamsOld);
        delete ptIdToNewPtId;
    }

    {
        printf("Cleaning mesh\n");

        int bandType = 0;
        mv_images_cache* ic = new mv_images_cache(&mp, bandType, true);
        cuda_plane_sweeping* cps = nullptr; // new cuda_plane_sweeping(mp.CUDADeviceNo, ic, &mp, &pc, 1);
        ps_sgm_params* sp = new ps_sgm_params(&mp, &pc, cps);

        mv_mesh_energy_opt_photo_mem* meOpt = new mv_mesh_energy_opt_photo_mem(&mp, sp, usedCams);
        meOpt->addMesh(inout_mesh);
        delete inout_mesh;

        meOpt->init();
        meOpt->cleanMesh(10);

        if(exportDebug)
            o3d.saveMvMeshToObj(meOpt, resultFolderName + "mesh_clean.obj");

        /////////////////////////////
        {
            // Update pointCams after clean
            inout_ptsCams->resizeAdd(meOpt->newPtsOldPtId->size());
            for(int i = 0; i < meOpt->newPtsOldPtId->size(); i++)
            {
                int oldPtId = (*meOpt->newPtsOldPtId)[i];
                staticVector<int>* ptCams = new staticVector<int>(sizeOfStaticVector<int>((*inout_ptsCams)[oldPtId]));
                for(int j = 0; j < sizeOfStaticVector<int>((*inout_ptsCams)[oldPtId]); j++)
                {
                    ptCams->push_back((*(*inout_ptsCams)[oldPtId])[j]);
                }
                inout_ptsCams->push_back(ptCams);
            }
        }

        /////////////////////////////
        bool doSubdivideMesh = mp.mip->_ini.get<bool>("meshEnergyOpt.doSubdivideMesh", false);
        if(doSubdivideMesh == true)
        {
            float subdivideMeshNTimesAvEdgeLengthThr =
                (float)mp.mip->_ini.get<double>("meshEnergyOpt.doSubdivideMesh", 20.0);
            int subdivideMaxPtsThr =
                mp.mip->_ini.get<int>("meshEnergyOpt.subdivideMaxPtsThr", 6000000);

            meOpt->subdivideMeshMaxEdgeLengthUpdatePtsCams(&mp, subdivideMeshNTimesAvEdgeLengthThr *
                                                                              meOpt->computeAverageEdgeLength(),
                                                           inout_ptsCams, subdivideMaxPtsThr);
            meOpt->deallocateCleaningAttributes();
            meOpt->init();
            meOpt->cleanMesh(1); // has to be here
        }

        /////////////////////////////
        staticVectorBool* ptsCanMove = nullptr;
        if(hexah != nullptr)
        {
            point3d O = hexah[0];
            point3d vx = hexah[1] - hexah[0];
            point3d vy = hexah[3] - hexah[0];
            point3d vz = hexah[4] - hexah[0];
            float svx = vx.size();
            float svy = vy.size();
            float svz = vz.size();
            vx = vx.normalize();
            vy = vy.normalize();
            vz = vz.normalize();
            float avel = 10.0f * meOpt->computeAverageEdgeLength();

            ptsCanMove = new staticVectorBool(meOpt->pts->size());
            ptsCanMove->resize_with(meOpt->pts->size(), true);
            for(int i = 0; i < meOpt->pts->size(); i++)
            {
                float x = pointPlaneDistance((*meOpt->pts)[i], O, vx);
                float y = pointPlaneDistance((*meOpt->pts)[i], O, vy);
                float z = pointPlaneDistance((*meOpt->pts)[i], O, vz);
                bool isHexahBorderPt = ((x < avel) || (x > svx - avel) || (y < avel) || (y > svy - avel) ||
                                        (z < avel) || (z > svz - avel));
                (*ptsCanMove)[i] = ((isHexahBorderPt == false) || (sizeOfStaticVector<int>((*inout_ptsCams)[i]) > 0));
                //(*ptsCanMove)[i] = (isHexahBorderPt==false);
            }
        }

        /////////////////////////////
        bool doSmoothMesh =
            mp.mip->_ini.get<bool>("meshEnergyOpt.doSmoothMesh", true);
        if(doSmoothMesh)
        {
            printf("Smoothing mesh\n");
            float lambda = (float)mp.mip->_ini.get<double>("meshEnergyOpt.lambda", 1.0f);
            float epsilon = (float)mp.mip->_ini.get<double>("meshEnergyOpt.epsilon", 0.1f); // unused in type 3
            int niter = mp.mip->_ini.get<int>("meshEnergyOpt.smoothNbIterations", 10);
            int type = mp.mip->_ini.get<int>("meshEnergyOpt.smoothType", 3); // 0, 1, 2, 3, 4 => only 1 and 3 works
            meOpt->optimizeSmooth(lambda, epsilon, type, niter, ptsCanMove);

            if(exportDebug)
                o3d.saveMvMeshToObj(meOpt, resultFolderName + "mesh_smoothed.obj");
        }

        bool doLaplacianSmoothMesh =
            mp.mip->_ini.get<bool>("meshEnergyOpt.doLaplacianSmoothMesh", false);
        if(doLaplacianSmoothMesh)
        {
            // No support for "ptsCanMove"
            printf("Laplacian smoothing mesh\n");
            staticVector<staticVector<int>*>* ptsNei = meOpt->getPtsNeighPtsOrdered();
            for(int iter = 0; iter < 2; iter++)
            {
                meOpt->laplacianSmoothPts(ptsNei);
            }
            deleteArrayOfArrays<int>(&ptsNei);

            if(exportDebug)
                o3d.saveMvMeshToObj(meOpt, resultFolderName + "mesh_laplacianSmoothed.obj");
        }

        bool doOptimizeMesh =
            mp.mip->_ini.get<bool>("meshEnergyOpt.doOptimizeMesh", false);
        if(doOptimizeMesh)
        {
            printf("Optimize mesh\n");
            staticVector<staticVector<int>*>* camsPts = convertObjectsCamsToCamsObjects(&mp, inout_ptsCams);

            printf("Optimizing mesh\n");
            int niter = mp.mip->_ini.get<int>("meshEnergyOpt.optimizeNbIterations", 50);
            meOpt->allocatePtsStats();
            meOpt->initPtsStats(camsPts);
            meOpt->optimizePhoto(niter, ptsCanMove, camsPts);

            deleteArrayOfArrays<int>(&camsPts);

            if(exportDebug)
                o3d.saveMvMeshToObj(meOpt, resultFolderName + "mesh_optimized.obj");
        }

        delete ptsCanMove;
        meOpt->deallocateCleaningAttributes();

        inout_mesh = new mv_mesh();
        inout_mesh->addMesh(meOpt);

        delete meOpt;

        delete ic;
        delete cps;
        delete sp;
    }
    printfElapsedTime(timer, "Mesh post-processing ");
    std::cout << "meshPostProcessing done" << std::endl;
}
