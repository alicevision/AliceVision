// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "mv_prematch_cams.hpp"

#include <aliceVision/structures/mv_filesio.hpp>
#include <aliceVision/structures/mv_common.hpp>

#include <iostream>


mv_prematch_cams::mv_prematch_cams(multiviewParams* _mp)
{
    mp = _mp;
    minang = (float)mp->mip->_ini.get<double>("prematching.minAngle", 2.0);
    maxang = (float)mp->mip->_ini.get<double>("prematching.maxAngle", 70.0); // WARNING: may be too low, especially when using seeds from SFM
    minCamsDistance = computeMinCamsDistance();
}

mv_prematch_cams::~mv_prematch_cams() = default;

float mv_prematch_cams::computeMinCamsDistance()
{

    int nd = 0;
    float d = 0.0;
    for(int rc = 0; rc < mp->ncams; rc++)
    {
        point3d rC = mp->CArr[rc];
        for(int tc = 0; tc < mp->ncams; tc++)
        {
            if(rc != tc)
            {
                point3d tC = mp->CArr[tc];
                d += (rC - tC).size();
                nd++;
            }
        }
    }
    return (d / (float)nd) / 100.0f;
}

bool mv_prematch_cams::overlap(int rc, int tc)
{
    if(!checkCamPairAngle(rc, tc, mp, 0.0f, 45.0f))
    {
        return false;
    }

    point2d rmid = point2d((float)mp->mip->getWidth(rc) / 2.0f, (float)mp->mip->getHeight(rc) / 2.0f);
    point2d pFromTar, pToTar;

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

staticVector<int>* mv_prematch_cams::findNearestCams(int rc, int _nnearestcams)
{
    staticVector<int>* out;

    out = new staticVector<int>(_nnearestcams);

    staticVector<sortedId>* ids = new staticVector<sortedId>(mp->ncams - 1);

    for(int c = 0; c < mp->ncams; c++)
    {
        if(c != rc)
        {
            ids->push_back(sortedId(c, (mp->CArr[rc] - mp->CArr[c]).size()));
            // printf("(%i %f) \n", (*ids)[ids->size()-1].id, (*ids)[ids->size()-1].value);
        }
    }

    qsort(&(*ids)[0], ids->size(), sizeof(sortedId), qsortCompareSortedIdAsc);

    /*
    for (int c=0;c<ids->size();c++)
    {
            printf("(%i %f) \n", (*ids)[c].id, (*ids)[c].value);
    };
    */

    {
        int c = 0;
        point3d rC = mp->CArr[rc];

        while((out->size() < _nnearestcams) && (c < ids->size()))
        {
            int tc = (*ids)[c].id;
            point3d tC = mp->CArr[tc];
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

staticVector<int>* mv_prematch_cams::precomputeIncidentMatrixCamsFromSeeds()
{
    std::string fn = mp->mip->mvDir + "camsPairsMatrixFromSeeds.bin";
    if(FileExists(fn))
    {
        std::cout << "Camera pairs matrix file already computed: " << fn << std::endl;
        return loadArrayFromFile<int>(fn);
    }
    std::cout << "Compute camera pairs matrix file: " << fn << std::endl;
    staticVector<int>* camsmatrix = new staticVector<int>(mp->ncams * mp->ncams);
    camsmatrix->resize_with(mp->ncams * mp->ncams, 0);
    for(int rc = 0; rc < mp->ncams; rc++)
    {
        staticVector<seedPoint>* seeds;
        loadSeedsFromFile(&seeds, mp->indexes[rc], mp->mip, mp->mip->MV_FILE_TYPE_seeds);
        for(int i = 0; i < seeds->size(); i++)
        {
            seedPoint* sp = &(*seeds)[i];
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

staticVector<int>* mv_prematch_cams::loadCamPairsMatrix()
{
    std::string fn = mp->mip->mvDir + "camsPairsMatrixFromSeeds.bin"; // TODO: store this filename at one place
    if(!FileExists(fn))
        throw std::runtime_error("Missing camera pairs matrix file (see --computeCamPairs): " + fn);
    return loadArrayFromFile<int>(fn);
}


staticVector<int>* mv_prematch_cams::findNearestCamsFromSeeds(int rc, int nnearestcams)
{
    staticVector<int>* out = nullptr;

    std::string tarCamsFile = mp->mip->mvDir + "_tarCams/" + num2strFourDecimal(rc) + ".txt";
    if(FileExists(tarCamsFile))
    {
        FILE* f = fopen(tarCamsFile.c_str(), "r");
        int ntcams;
        fscanf(f, "ntcams %i, tcams", &ntcams);
        out = new staticVector<int>(ntcams);
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
        staticVector<int>* camsmatrix = loadCamPairsMatrix();
        staticVector<sortedId> ids(mp->ncams);
        for(int tc = 0; tc < mp->ncams; tc++)
        {
            ids.push_back(sortedId(tc, (float)(*camsmatrix)[std::min(rc, tc) * mp->ncams + std::max(rc, tc)]));
        }
        qsort(&ids[0], ids.size(), sizeof(sortedId), qsortCompareSortedIdDesc);
        
        // Ensure the ideal number of target cameras is not superior to the actual number of cameras
        const int maxNumTC = std::min(mp->ncams, nnearestcams);
        out = new staticVector<int>(maxNumTC);

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

bool mv_prematch_cams::intersectsRcTc(int rc, float rmind, float rmaxd, int tc, float tmind, float tmaxd)
{
    point3d rchex[8];
    point3d tchex[8];
    getCamHexahedron(mp, rchex, rc, rmind, rmaxd);
    getCamHexahedron(mp, tchex, tc, tmind, tmaxd);
    return intersectsHexahedronHexahedron(rchex, tchex);
}

// hexahedron format ... 0-3 frontal face, 4-7 back face
staticVector<int>* mv_prematch_cams::findCamsWhichIntersectsHexahedron(point3d hexah[8],
                                                                       std::string minMaxDepthsFileName)
{
    staticVector<point2d>* minMaxDepths = loadArrayFromFile<point2d>(minMaxDepthsFileName);

    staticVector<int>* tcams = new staticVector<int>(mp->ncams);
    for(int rc = 0; rc < mp->ncams; rc++)
    {
        float mindepth = (*minMaxDepths)[rc].x;
        float maxdepth = (*minMaxDepths)[rc].y;
        if((mindepth > 0.0f) && (maxdepth > mindepth))
        {
            point3d rchex[8];
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
staticVector<int>* mv_prematch_cams::findCamsWhichIntersectsHexahedron(point3d hexah[8])
{

    staticVector<int>* tcams = new staticVector<int>(mp->ncams);
    for(int rc = 0; rc < mp->ncams; rc++)
    {
        float mindepth, maxdepth;
        staticVector<int>* pscams;
        if(getDepthMapInfo(mp->indexes[rc], mp->mip, mindepth, maxdepth, &pscams))
        {
            delete pscams;
            point3d rchex[8];
            getCamHexahedron(mp, rchex, rc, mindepth, maxdepth);

            if(intersectsHexahedronHexahedron(rchex, hexah))
            {
                tcams->push_back(rc);
            }
        }
    }

    return tcams;
}

// hexahedron format ... 0-3 frontal face, 4-7 back face
staticVector<int>* mv_prematch_cams::findCamsWhichAreInHexahedron(point3d hexah[8])
{
    staticVector<int>* tcams = new staticVector<int>(mp->ncams);
    for(int rc = 0; rc < mp->ncams; rc++)
    {
        float mindepth, maxdepth;
        staticVector<int>* pscams;
        getDepthMapInfo(mp->indexes[rc], mp->mip, mindepth, maxdepth, &pscams);
        delete pscams;

        point3d rchex[8];
        getCamHexahedron(mp, rchex, rc, mindepth, maxdepth);

        if(isPointInHexahedron(mp->CArr[rc], hexah))
        {
            tcams->push_back(rc);
        }
    }

    return tcams;
}

staticVector<int>* mv_prematch_cams::findCamsWhichIntersectsCamHexah(int rc)
{
    point3d hexah[8];

    staticVector<int>* tcams;
    float mindepth, maxdepth;
    getDepthMapInfo(rc + 1, mp->mip, mindepth, maxdepth, &tcams);
    delete tcams;

    getCamHexahedron(mp, hexah, rc, mindepth, maxdepth);

    tcams = findCamsWhichIntersectsHexahedron(hexah);
    staticVector<int>* tcams1 = new staticVector<int>(tcams->size());

    for(int c = 0; c < tcams->size(); c++)
    {
        int tc = (*tcams)[c];
        if(tc != rc)
        {
            tcams1->push_back(tc);
        }
    }

    delete tcams;

    return tcams1;
}
