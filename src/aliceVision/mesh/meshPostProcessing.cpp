// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "meshPostProcessing.hpp"
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/mvsData/geometry.hpp>
#include <aliceVision/mvsData/Point3d.hpp>
#include <aliceVision/mvsData/StaticVector.hpp>
#include <aliceVision/mesh/MeshEnergyOpt.hpp>

#include <boost/filesystem/operations.hpp>

namespace aliceVision {
namespace mesh {

namespace bfs = boost::filesystem;

void meshPostProcessing(Mesh*& inout_mesh, StaticVector<StaticVector<int>*>*& inout_ptsCams, mvsUtils::MultiViewParams& mp,
                      const std::string& debugFolderName,
                      StaticVector<Point3d>* hexahsToExcludeFromResultingMesh, Point3d* hexah)
{
    long timer = std::clock();
    ALICEVISION_LOG_INFO("Mesh post-processing.");

    bool exportDebug = (float)mp.userParams.get<bool>("delaunaycut.exportDebugGC", false);

    if(exportDebug)
        inout_mesh->saveToObj(debugFolderName + "rawGraphCut.obj");

    // copy ptsCams
    {
        StaticVector<StaticVector<int>*>* ptsCamsOld = inout_ptsCams;
        StaticVector<int>* ptIdToNewPtId;

        bool doRemoveTrianglesInhexahsToExcludeFromResultingMesh =
            (bool)mp.userParams.get<bool>("LargeScale.doRemoveTrianglesInhexahsToExcludeFromResultingMesh",
                                       false);
        if(doRemoveTrianglesInhexahsToExcludeFromResultingMesh && hexahsToExcludeFromResultingMesh)
        {
            inout_mesh->removeTrianglesInHexahedrons(hexahsToExcludeFromResultingMesh);
        }

        inout_mesh->removeFreePointsFromMesh(&ptIdToNewPtId);

        // remap visibilities
        inout_ptsCams = new StaticVector<StaticVector<int>*>();
        inout_ptsCams->resize(inout_mesh->pts->size(), nullptr);
        for(int i = 0; i < ptIdToNewPtId->size(); ++i)
        {
            int newId = (*ptIdToNewPtId)[i];
            if(newId > -1)
            {
                StaticVector<int>* ptCamsNew = new StaticVector<int>();
                ptCamsNew->reserve(sizeOfStaticVector<int>((*ptsCamsOld)[i]));
                for(int j = 0; j < sizeOfStaticVector<int>((*ptsCamsOld)[i]); ++j)
                {
                    ptCamsNew->push_back((*(*ptsCamsOld)[i])[j]);
                }
                (*inout_ptsCams)[newId] = ptCamsNew;
            }
        }

        deleteArrayOfArrays<int>(&ptsCamsOld);
        delete ptIdToNewPtId;
    }

    if(true) // TODO: how to remove it?
    {
        ALICEVISION_LOG_INFO("Mesh Cleaning.");

        MeshEnergyOpt meOpt(&mp);
        meOpt.addMesh(inout_mesh);
        delete inout_mesh;

        meOpt.init();
        meOpt.cleanMesh(10);

        if(exportDebug)
            meOpt.saveToObj(debugFolderName + "MeshClean.obj");

        /////////////////////////////
        {
            // Update pointCams after clean
            inout_ptsCams->reserveAdd(meOpt.newPtsOldPtId->size());
            for(int i = 0; i < meOpt.newPtsOldPtId->size(); i++)
            {
                int oldPtId = (*meOpt.newPtsOldPtId)[i];
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
        bool doSubdivideMesh = mp.userParams.get<bool>("meshEnergyOpt.doSubdivideMesh", false);
        if(doSubdivideMesh == true)
        {
            float subdivideMeshNTimesAvEdgeLengthThr =
                (float)mp.userParams.get<double>("meshEnergyOpt.doSubdivideMesh", 20.0);
            int subdivideMaxPtsThr =
                mp.userParams.get<int>("meshEnergyOpt.subdivideMaxPtsThr", 6000000);

            meOpt.subdivideMeshMaxEdgeLengthUpdatePtsCams(&mp, subdivideMeshNTimesAvEdgeLengthThr *
                                                          meOpt.computeAverageEdgeLength(),
                                                          inout_ptsCams, subdivideMaxPtsThr);
            meOpt.deallocateCleaningAttributes();
            meOpt.init();
            meOpt.cleanMesh(1); // has to be here
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
            float avel = 10.0f * meOpt.computeAverageEdgeLength();

            ptsCanMove = new StaticVectorBool();
            ptsCanMove->reserve(meOpt.pts->size());
            ptsCanMove->resize_with(meOpt.pts->size(), true);
            for(int i = 0; i < meOpt.pts->size(); i++)
            {
                float x = pointPlaneDistance((*meOpt.pts)[i], O, vx);
                float y = pointPlaneDistance((*meOpt.pts)[i], O, vy);
                float z = pointPlaneDistance((*meOpt.pts)[i], O, vz);
                bool isHexahBorderPt = ((x < avel) || (x > svx - avel) || (y < avel) || (y > svy - avel) ||
                                        (z < avel) || (z > svz - avel));
                (*ptsCanMove)[i] = ((isHexahBorderPt == false) || (sizeOfStaticVector<int>((*inout_ptsCams)[i]) > 0));
                //(*ptsCanMove)[i] = (isHexahBorderPt==false);
            }
        }

        /////////////////////////////
        bool doSmoothMesh =
            mp.userParams.get<bool>("meshEnergyOpt.doSmoothMesh", true);
        int smoothNIter = mp.userParams.get<int>("meshEnergyOpt.smoothNbIterations", 0);
        if(doSmoothMesh && smoothNIter != 0)
        {
            ALICEVISION_LOG_INFO("Mesh smoothing.");
            float lambda = (float)mp.userParams.get<double>("meshEnergyOpt.lambda", 1.0f);
            meOpt.optimizeSmooth(lambda, smoothNIter, ptsCanMove);

            if(exportDebug)
                meOpt.saveToObj(debugFolderName + "mesh_smoothed.obj");
        }

        delete ptsCanMove;
        meOpt.deallocateCleaningAttributes();

        inout_mesh = new Mesh();
        inout_mesh->addMesh(&meOpt);
    }
    mvsUtils::printfElapsedTime(timer, "Mesh post-processing ");
    ALICEVISION_LOG_INFO("Mesh post-processing done.");
}

} // namespace mesh
} // namespace aliceVision
