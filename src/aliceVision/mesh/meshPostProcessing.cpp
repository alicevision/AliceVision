// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "meshPostProcessing.hpp"
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/mvsData/geometry.hpp>
#include <aliceVision/mvsData/Point3d.hpp>
#include <aliceVision/mvsData/StaticVector.hpp>
#include <aliceVision/mvsUtils/PreMatchCams.hpp>
#include <aliceVision/mesh/MeshEnergyOpt.hpp>

#include <boost/filesystem/operations.hpp>

namespace aliceVision {
namespace mesh {

namespace bfs = boost::filesystem;

void filterLargeEdgeTriangles(Mesh* me, float avelthr)
{
    float averageEdgeLength = me->computeAverageEdgeLength();

    StaticVector<int>* trisIdsToStay = new StaticVector<int>();
    trisIdsToStay->reserve(me->tris->size());
    for(int i = 0; i < me->tris->size(); i++)
    {
        float triMaxEdgelength = me->computeTriangleMaxEdgeLength(i);
        if(triMaxEdgelength < averageEdgeLength * avelthr)
        {
            trisIdsToStay->push_back(i);
        }
    }
    me->letJustTringlesIdsInMesh(trisIdsToStay);

    delete trisIdsToStay;
}

void meshPostProcessing(Mesh*& inout_mesh, StaticVector<StaticVector<int>*>*& inout_ptsCams, StaticVector<int>& usedCams,
                      mvsUtils::MultiViewParams& mp, mvsUtils::PreMatchCams& pc,
                      const std::string& resultFolderName,
                      StaticVector<Point3d>* hexahsToExcludeFromResultingMesh, Point3d* hexah)
{
    long timer = std::clock();
    ALICEVISION_LOG_INFO("Mesh post-processing.");


    bool exportDebug = (float)mp._ini.get<bool>("delaunaycut.exportDebugGC", false);

    if(exportDebug)
        inout_mesh->saveToObj(resultFolderName + "rawGraphCut.obj");

    const auto doRemoveHugeTriangles =
            mp._ini.get<bool>("hallucinationsFiltering.doRemoveHugeTriangles", false);

    if(doRemoveHugeTriangles)
    {
        filterLargeEdgeTriangles(
            inout_mesh, (float)mp._ini.get<double>("hallucinationsFiltering.NbAverageEdgeLength", 60.0));
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////

    // copy ptsCams
    {
        StaticVector<StaticVector<int>*>* ptsCamsOld = inout_ptsCams;
        StaticVector<int>* ptIdToNewPtId;

        //!!!!!!!!!!!!!
        bool doRemoveTrianglesInhexahsToExcludeFromResultingMesh =
            (bool)mp._ini.get<bool>("LargeScale.doRemoveTrianglesInhexahsToExcludeFromResultingMesh",
                                       false);
        if(doRemoveTrianglesInhexahsToExcludeFromResultingMesh && hexahsToExcludeFromResultingMesh)
        {
            inout_mesh->removeTrianglesInHexahedrons(hexahsToExcludeFromResultingMesh);
        }

        const auto doLeaveLargestFullSegmentOnly = (bool)mp._ini.get<bool>("hallucinationsFiltering.doLeaveLargestFullSegmentOnly", false);
        if(doLeaveLargestFullSegmentOnly)
        {
            StaticVector<int>* trisIdsToStay = inout_mesh->getLargestConnectedComponentTrisIds(mp);
            inout_mesh->letJustTringlesIdsInMesh(trisIdsToStay);
            delete trisIdsToStay;
        }

        inout_mesh->removeFreePointsFromMesh(&ptIdToNewPtId);

        inout_ptsCams = new StaticVector<StaticVector<int>*>();
        inout_ptsCams->reserve(inout_mesh->pts->size());
        for(int i = 0; i < inout_mesh->pts->size(); i++)
        {
            inout_ptsCams->push_back(nullptr);
        }
        for(int i = 0; i < ptIdToNewPtId->size(); i++)
        {
            int newId = (*ptIdToNewPtId)[i];
            if(newId > -1)
            {
                StaticVector<int>* ptCamsNew = new StaticVector<int>();
                ptCamsNew->reserve(sizeOfStaticVector<int>((*ptsCamsOld)[i]));
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
        ALICEVISION_LOG_INFO("Mesh Cleaning.");

        MeshEnergyOpt* meOpt = new MeshEnergyOpt(&mp);
        meOpt->addMesh(inout_mesh);
        delete inout_mesh;

        meOpt->init();
        meOpt->cleanMesh(10);

        if(exportDebug)
            meOpt->saveToObj(resultFolderName + "MeshClean.obj");

        /////////////////////////////
        {
            // Update pointCams after clean
            inout_ptsCams->resizeAdd(meOpt->newPtsOldPtId->size());
            for(int i = 0; i < meOpt->newPtsOldPtId->size(); i++)
            {
                int oldPtId = (*meOpt->newPtsOldPtId)[i];
                StaticVector<int>* ptCams = new StaticVector<int>();
                ptCams->reserve(sizeOfStaticVector<int>((*inout_ptsCams)[oldPtId]));
                for(int j = 0; j < sizeOfStaticVector<int>((*inout_ptsCams)[oldPtId]); j++)
                {
                    ptCams->push_back((*(*inout_ptsCams)[oldPtId])[j]);
                }
                inout_ptsCams->push_back(ptCams);
            }
        }

        /////////////////////////////
        bool doSubdivideMesh = mp._ini.get<bool>("meshEnergyOpt.doSubdivideMesh", false);
        if(doSubdivideMesh == true)
        {
            float subdivideMeshNTimesAvEdgeLengthThr =
                (float)mp._ini.get<double>("meshEnergyOpt.doSubdivideMesh", 20.0);
            int subdivideMaxPtsThr =
                mp._ini.get<int>("meshEnergyOpt.subdivideMaxPtsThr", 6000000);

            meOpt->subdivideMeshMaxEdgeLengthUpdatePtsCams(&mp, subdivideMeshNTimesAvEdgeLengthThr *
                                                                              meOpt->computeAverageEdgeLength(),
                                                           inout_ptsCams, subdivideMaxPtsThr);
            meOpt->deallocateCleaningAttributes();
            meOpt->init();
            meOpt->cleanMesh(1); // has to be here
        }

        /////////////////////////////
        StaticVectorBool* ptsCanMove = nullptr;
        if(hexah != nullptr)
        {
            Point3d O = hexah[0];
            Point3d vx = hexah[1] - hexah[0];
            Point3d vy = hexah[3] - hexah[0];
            Point3d vz = hexah[4] - hexah[0];
            float svx = vx.size();
            float svy = vy.size();
            float svz = vz.size();
            vx = vx.normalize();
            vy = vy.normalize();
            vz = vz.normalize();
            float avel = 10.0f * meOpt->computeAverageEdgeLength();

            ptsCanMove = new StaticVectorBool();
            ptsCanMove->reserve(meOpt->pts->size());
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
            mp._ini.get<bool>("meshEnergyOpt.doSmoothMesh", true);
        int smoothNIter = mp._ini.get<int>("meshEnergyOpt.smoothNbIterations", 10);
        if(doSmoothMesh && smoothNIter != 0)
        {
            ALICEVISION_LOG_INFO("Mesh smoothing.");
            float lambda = (float)mp._ini.get<double>("meshEnergyOpt.lambda", 1.0f);
            meOpt->optimizeSmooth(lambda, smoothNIter, ptsCanMove);

            if(exportDebug)
                meOpt->saveToObj(resultFolderName + "mesh_smoothed.obj");
        }

        delete ptsCanMove;
        meOpt->deallocateCleaningAttributes();

        inout_mesh = new Mesh();
        inout_mesh->addMesh(meOpt);

        delete meOpt;
    }
    mvsUtils::printfElapsedTime(timer, "Mesh post-processing ");
    ALICEVISION_LOG_INFO("Mesh post-processing done.");
}

} // namespace mesh
} // namespace aliceVision
