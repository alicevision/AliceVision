// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "PreMatchCams.hpp"
#include <aliceVision/structures/Point2d.hpp>
#include <aliceVision/structures/SeedPoint.hpp>
#include <aliceVision/common/fileIO.hpp>
#include <aliceVision/common/common.hpp>

#include <iostream>

namespace aliceVision {

PreMatchCams::PreMatchCams(MultiViewParams* _mp)
{
    mp = _mp;
    minang = (float)mp->mip->_ini.get<double>("prematching.minAngle", 2.0);
    maxang = (float)mp->mip->_ini.get<double>("prematching.maxAngle", 70.0); // WARNING: may be too low, especially when using seeds from SFM
    minCamsDistance = computeMinCamsDistance();
}

float PreMatchCams::computeMinCamsDistance()
{
    int nd = 0;
    float d = 0.0;
    for(int rc = 0; rc < mp->ncams; rc++)
    {
        Point3d rC = mp->CArr[rc];
        for(int tc = 0; tc < mp->ncams; tc++)
        {
            if(rc != tc)
            {
                Point3d tC = mp->CArr[tc];
                d += (rC - tC).size();
                nd++;
            }
        }
    }
    return (d / (float)nd) / 100.0f;
}

bool PreMatchCams::overlap(int rc, int tc)
{
    if(!checkCamPairAngle(rc, tc, mp, 0.0f, 45.0f))
    {
        return false;
    }

    Point2d rmid = Point2d((float)mp->mip->getWidth(rc) / 2.0f, (float)mp->mip->getHeight(rc) / 2.0f);
    Point2d pFromTar, pToTar;

    if(!getTarEpipolarDirectedLine(&pFromTar, &pToTar, rmid, rc, tc, mp))
    {
        return false;
    }

    /*
    if (getTarEpipolarDirectedLine(
                    &pFromTar, &pToTar,
                    rmid,
                    tc, rc, mp
                    )==false)
    {
            return false;
    };
    */
    return true;
}

StaticVector<int>* PreMatchCams::findNearestCams(int rc, int _nnearestcams)
{
    StaticVector<int>* out;
    out = new StaticVector<int>(_nnearestcams);
    StaticVector<SortedId>* ids = new StaticVector<SortedId>(mp->ncams - 1);
    for(int c = 0; c < mp->ncams; c++)
    {
        if(c != rc)
        {
            ids->push_back(SortedId(c, (mp->CArr[rc] - mp->CArr[c]).size()));
            // printf("(%i %f) \n", (*ids)[ids->size()-1].id, (*ids)[ids->size()-1].value);
        }
    }

    qsort(&(*ids)[0], ids->size(), sizeof(SortedId), qsortCompareSortedIdAsc);

    /*
    for (int c=0;c<ids->size();c++)
    {
            printf("(%i %f) \n", (*ids)[c].id, (*ids)[c].value);
    };
    */

    {
        int c = 0;
        Point3d rC = mp->CArr[rc];

        while((out->size() < _nnearestcams) && (c < ids->size()))
        {
            int tc = (*ids)[c].id;
            Point3d tC = mp->CArr[tc];
            float d = (rC - tC).size();

            if((rc != tc) && (d > minCamsDistance) && (overlap(rc, tc)))
            {
                out->push_back(tc);
            }
            c++;
        }
    }
    delete ids;
    return out;
}

StaticVector<int>* PreMatchCams::precomputeIncidentMatrixCamsFromSeeds()
{
    std::string fn = mp->mip->mvDir + "camsPairsMatrixFromSeeds.bin";
    if(FileExists(fn))
    {
        std::cout << "Camera pairs matrix file already computed: " << fn << std::endl;
        return loadArrayFromFile<int>(fn);
    }
    std::cout << "Compute camera pairs matrix file: " << fn << std::endl;
    StaticVector<int>* camsmatrix = new StaticVector<int>(mp->ncams * mp->ncams);
    camsmatrix->resize_with(mp->ncams * mp->ncams, 0);
    for(int rc = 0; rc < mp->ncams; ++rc)
    {
        StaticVector<SeedPoint>* seeds;
        loadSeedsFromFile(&seeds, rc, mp->mip, EFileType::seeds);
        for(int i = 0; i < seeds->size(); i++)
        {
            SeedPoint* sp = &(*seeds)[i];
            for(int c = 0; c < sp->cams.size(); c++)
            {
                int tc = sp->cams[c];
                (*camsmatrix)[std::min(rc, tc) * mp->ncams + std::max(rc, tc)] += 1;
            }
        }
        delete seeds;
    }
    saveArrayToFile<int>(fn, camsmatrix);
    return camsmatrix;    
}

StaticVector<int>* PreMatchCams::loadCamPairsMatrix()
{
    std::string fn = mp->mip->mvDir + "camsPairsMatrixFromSeeds.bin"; // TODO: store this filename at one place
    if(!FileExists(fn))
        throw std::runtime_error("Missing camera pairs matrix file (see --computeCamPairs): " + fn);
    return loadArrayFromFile<int>(fn);
}


StaticVector<int>* PreMatchCams::findNearestCamsFromSeeds(int rc, int nnearestcams)
{
    StaticVector<int>* out = nullptr;
    std::string tarCamsFile = mp->mip->mvDir + "_tarCams/" + num2strFourDecimal(rc) + ".txt";
    if(FileExists(tarCamsFile))
    {
        FILE* f = fopen(tarCamsFile.c_str(), "r");
        int ntcams;
        fscanf(f, "ntcams %i, tcams", &ntcams);
        out = new StaticVector<int>(ntcams);
        for(int c = 0; c < ntcams; c++)
        {
            int tc;
            fscanf(f, " %i", &tc);
            out->push_back(tc);
        }
        fclose(f);
    }
    else
    {
        StaticVector<int>* camsmatrix = loadCamPairsMatrix();
        StaticVector<SortedId> ids(mp->ncams);
        for(int tc = 0; tc < mp->ncams; tc++)
        {
            ids.push_back(SortedId(tc, (float)(*camsmatrix)[std::min(rc, tc) * mp->ncams + std::max(rc, tc)]));
        }
        qsort(&ids[0], ids.size(), sizeof(SortedId), qsortCompareSortedIdDesc);
        
        // Ensure the ideal number of target cameras is not superior to the actual number of cameras
        const int maxNumTC = std::min(mp->ncams, nnearestcams);
        out = new StaticVector<int>(maxNumTC);

        for(int i = 0; i < maxNumTC; i++)
        {
            // a minimum of 10 common points is required (10*2 because points are stored in both rc/tc combinations)
            if(ids[i].value > (10 * 2)) 
                out->push_back(ids[i].id);
        }

        delete camsmatrix;
        
        if(out->size() < nnearestcams)
            std::cout << "Warning: rc " << rc << " - only found " << out->size() << "/" << nnearestcams << " tc by seeds" << std::endl;
    }
    return out;
}

// hexahedron format ... 0-3 frontal face, 4-7 back face
StaticVector<int>* PreMatchCams::findCamsWhichIntersectsHexahedron(Point3d hexah[8],
                                                                       std::string minMaxDepthsFileName)
{
    StaticVector<Point2d>* minMaxDepths = loadArrayFromFile<Point2d>(minMaxDepthsFileName);
    StaticVector<int>* tcams = new StaticVector<int>(mp->ncams);
    for(int rc = 0; rc < mp->ncams; rc++)
    {
        float mindepth = (*minMaxDepths)[rc].x;
        float maxdepth = (*minMaxDepths)[rc].y;
        if((mindepth > 0.0f) && (maxdepth > mindepth))
        {
            Point3d rchex[8];
            getCamHexahedron(mp, rchex, rc, mindepth, maxdepth);
            if(intersectsHexahedronHexahedron(rchex, hexah))
            {
                tcams->push_back(rc);
            }
        }
    }
    delete minMaxDepths;
    return tcams;
}

// hexahedron format ... 0-3 frontal face, 4-7 back face
StaticVector<int>* PreMatchCams::findCamsWhichIntersectsHexahedron(Point3d hexah[8])
{
    StaticVector<int>* tcams = new StaticVector<int>(mp->ncams);
    for(int rc = 0; rc < mp->ncams; rc++)
    {
        float mindepth, maxdepth;
        StaticVector<int>* pscams;
        if(getDepthMapInfo(rc, mp->mip, mindepth, maxdepth, &pscams))
        {
            delete pscams;
            Point3d rchex[8];
            getCamHexahedron(mp, rchex, rc, mindepth, maxdepth);

            if(intersectsHexahedronHexahedron(rchex, hexah))
            {
                tcams->push_back(rc);
            }
        }
    }
    return tcams;
}

} // namespace aliceVision
