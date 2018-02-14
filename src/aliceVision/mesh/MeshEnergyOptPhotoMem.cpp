// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "MeshEnergyOptPhotoMem.hpp"
#include <aliceVision/structures/Pixel.hpp>
#include <aliceVision/structures/Point2d.hpp>
#include <aliceVision/common/fileIO.hpp>
#include <aliceVision/imageIO/imageScaledColors.hpp>

#include <boost/filesystem.hpp>

namespace bfs = boost::filesystem;

MeshEnergyOptPhotoMem::ptStat::ptStat()
{
    camsDepthMapsPtsSims = new StaticVector<camPtStat>(10);
    historyPtsSims = new StaticVector<Point4d>(10);
}

MeshEnergyOptPhotoMem::ptStat::~ptStat()
{
    delete camsDepthMapsPtsSims;
    delete historyPtsSims;
}

int MeshEnergyOptPhotoMem::ptStat::indexOfNewCamPtStatSortedByCamPixSize(float val)
{
    if((camsDepthMapsPtsSims->size() == 0) || (val < (*camsDepthMapsPtsSims)[0].camPixSize))
    {
        return 0;
    }

    if(val >= (*camsDepthMapsPtsSims)[camsDepthMapsPtsSims->size() - 1].camPixSize)
    {
        return camsDepthMapsPtsSims->size();
    }

    int iter = 0;
    int lef = 0;
    int rig = camsDepthMapsPtsSims->size() - 1;
    int mid = lef + (rig - lef) / 2;
    while((rig - lef) > 2)
    {
        bool ok = false;
        if((val >= (*camsDepthMapsPtsSims)[lef].camPixSize) && (val < (*camsDepthMapsPtsSims)[mid].camPixSize))
        {
            lef = lef;
            rig = mid;
            mid = lef + (rig - lef) / 2;
            ok = true;
        }
        if((val >= (*camsDepthMapsPtsSims)[mid].camPixSize) && (val < (*camsDepthMapsPtsSims)[rig].camPixSize))
        {
            lef = mid;
            rig = rig;
            mid = lef + (rig - lef) / 2;
            ok = true;
        }
        iter++;
        if((iter >= camsDepthMapsPtsSims->size()) || (!ok))
        {
            printf("WARNINIG iter %i, size %i, val %f, OK %i, lef %i, mid %i, rig %i\n", iter,
                   camsDepthMapsPtsSims->size(), val, (int)ok, lef, mid, rig);
            for(int i = 0; i < camsDepthMapsPtsSims->size(); i++)
            {
                printf("id %i val %f\n", i, (*camsDepthMapsPtsSims)[i].camPixSize);
            }
            return -1;
        }
    }

    return rig;
}

void MeshEnergyOptPhotoMem::ptStat::addCamPtStat(camPtStat& cptst)
{
    int id = indexOfNewCamPtStatSortedByCamPixSize(cptst.camPixSize);

    if(camsDepthMapsPtsSims->size() < camsDepthMapsPtsSims->capacity())
    {
        camsDepthMapsPtsSims->push_back(cptst);
    }

    if((id >= 0) && (id < camsDepthMapsPtsSims->size()))
    {
        for(int i = camsDepthMapsPtsSims->size() - 1; i > id; i--)
        {
            (*camsDepthMapsPtsSims)[i] = (*camsDepthMapsPtsSims)[i - 1];
        }
        (*camsDepthMapsPtsSims)[id] = cptst;
    }
    else
    {
        // printf("WARNING ID %i max %i \n", id, camsDepthMapsPtsSims->size());
    }
}

MeshEnergyOptPhotoMem::MeshEnergyOptPhotoMem(MultiViewParams* _mp, SemiGlobalMatchingParams* _sp,
                                                           const StaticVector<int>& _usedCams)
    : MeshEnergyOpt(_mp)
    , usedCams(_usedCams)
    , sp(_sp)
{
    /*
    tmpDir = mp->mip->mvDir + "meshEnergyOptMem/";
    bfs::create_directory(tmpDir);

    meshDepthMapsDir = tmpDir + "meshDepthMapsDir/";
    bfs::create_directory(meshDepthMapsDir);
    */

    ptsStats = nullptr;

    meshPtRcDepthMapPtDistPixSizeLimit = 20.0;

    wsh = sp->mp->mip->_ini.get<int>("meshEnergyOpt.wsh", 2);
    gammaC = (float)sp->mp->mip->_ini.get<double>("meshEnergyOpt.gammaC", 105.5);
    gammaP = (float)sp->mp->mip->_ini.get<double>("meshEnergyOpt.gammaP", 4.0);
    useTcOrRcPixSize = (bool)sp->mp->mip->_ini.get<bool>("meshEnergyOpt.useTcOrRcPixSize", false);
    nSamplesHalf = sp->mp->mip->_ini.get<int>("meshEnergyOpt.nSamplesHalf", 50);
    ndepthsToRefine = sp->mp->mip->_ini.get<int>("meshEnergyOpt.ndepthsToRefine", 11);
    sigma = (float)sp->mp->mip->_ini.get<double>("meshEnergyOpt.sigma", 10.0);
    pixSizeRatioThr = (float)sp->mp->mip->_ini.get<double>("meshEnergyOpt.pixSizeRatioThr", 1.2);
}

MeshEnergyOptPhotoMem::~MeshEnergyOptPhotoMem()
{
    deAllocatePtsStats();
}

void MeshEnergyOptPhotoMem::allocatePtsStats()
{
    if(mp->verbose)
        printf("allocatePtsStats\n");
    ptsStats = new StaticVector<ptStat*>(pts->size());
    for(int i = 0; i < pts->size(); i++)
    {
        ptStat* ptst = new ptStat();
        ptsStats->push_back(ptst);
    }
}

void MeshEnergyOptPhotoMem::deAllocatePtsStats()
{
    if(ptsStats != nullptr)
    {
        for(int i = 0; i < ptsStats->size(); i++)
        {
            delete(*ptsStats)[i];
        }
        delete ptsStats;
        ptsStats = nullptr;
    }
}

double MeshEnergyOptPhotoMem::computeEnergyFairForPt(int ptid)
{
    Point3d Kh;
    double Kg;
    if((getVertexMeanCurvatureNormal(ptid, Kh)) && (getVertexGaussianCurvature(ptid, Kg)))
    {
        double K1, K2;
        getVertexPrincipalCurvatures(Kh.size(), Kg, K1, K2);
        if((!std::isnan(K1)) && (!std::isnan(K2)))
        {
            return K1 * K1 + K2 * K2;
        }
    }

    return 0.0;
}

StaticVector<int>* MeshEnergyOptPhotoMem::getRcTcNVisTrisMap(StaticVector<StaticVector<int>*>* trisCams)
{
    if(sp->mp->verbose)
        printf("getRcTcNVisTrisMap\n");

    StaticVector<int>* out = new StaticVector<int>(sp->mp->ncams * sp->mp->ncams);
    out->resize_with(sp->mp->ncams * sp->mp->ncams, 0);

    int maxVal = 0;
    for(int i = 0; i < tris->size(); i++)
    {
        StaticVector<int>* triCams = (*trisCams)[i];
        int n = sizeOfStaticVector<int>(triCams);
        for(int j = 0; j < n; j++)
        {
            for(int k = j + 1; k < n; k++)
            {
                int rc = (*triCams)[j];
                int tc = (*triCams)[k];
                (*out)[std::min(rc, tc) * sp->mp->ncams + std::max(rc, tc)]++;
                int maxVal = std::max(maxVal, (*out)[std::min(rc, tc) * sp->mp->ncams + std::max(rc, tc)]);
            }
        }
    }

    std::string fn = tmpDir + "_ncTcNVisTrisMap.png";
    imageIO::writeImageScaledColors(fn, sp->mp->ncams, sp->mp->ncams, 0, maxVal, &(*out)[0], true);

    if(sp->mp->verbose)
        printf("done\n");

    return out;
}

void MeshEnergyOptPhotoMem::actualizePtsStats(StaticVector<StaticVector<int>*>* camsPts)
{
    if(mp->verbose)
        printf("actualizePtsStats\n");
    long t1 = clock();

    for(int i = 0; i < ptsStats->size(); i++)
    {
        (*ptsStats)[i]->camsDepthMapsPtsSims->resize(0);
        (*ptsStats)[i]->optDepthMapsPt = (*pts)[i];
    }

    for(int c = 0; c < usedCams.size(); c++)
    {
        int rc = usedCams[c];
        int w = mp->mip->getWidth(rc);

        DepthSimMap* dsmap = new DepthSimMap(rc, sp->mp, 1, 1);
        if(dsmap->loadRefine(sp->getREFINE_photo_depthMapFileName(rc, 1, 1),
                             sp->getREFINE_photo_simMapFileName(rc, 1, 1)))
        {
            // dsmap->load(rc,1); //from opt is worser than from photo ... don't use it
            // long t2 = initEstimate();
            StaticVector<int>* ptsIds = (*camsPts)[rc];
            // for (int i=0;i<pts->size();i++)
            for(int j = 0; j < sizeOfStaticVector<int>(ptsIds); j++)
            {
                int i = (*ptsIds)[j];
                Point3d pt = (*pts)[i];
                Pixel pix;
                mp->getPixelFor3DPoint(&pix, pt, rc);
                if(mp->isPixelInImage(pix, 1, rc))
                {
                    float ptDepth = (mp->CArr[rc] - pt).size();
                    float ptPixSize = mp->getCamPixelSize(pt, rc);
                    DepthSim depthSim = (*dsmap->dsm)[pix.y * w + pix.x];
                    if((depthSim.depth > 0.0f) &&
                       (fabs(ptDepth - depthSim.depth) < meshPtRcDepthMapPtDistPixSizeLimit * ptPixSize))
                    {
                        camPtStat cptst;
                        cptst.cam = rc;
                        cptst.pt = sp->mp->CArr[rc] +
                                   (sp->mp->iCamArr[rc] * Point2d((float)pix.x, (float)pix.y)).normalize() * depthSim.depth;
                        cptst.camPixSize = ptPixSize;

                        /////////////////////////////////////////////////////////////
                        //!!! sigmoid
                        // cptst.sim = sigmoidfcn(0.0f,-1.0f,0.8f,-0.5f,depthSim.y);
                        cptst.sim = depthSim.sim;

                        if((std::isnan(cptst.sim)) || (cptst.sim != cptst.sim))
                        {
                            printf("WARNING %f %f\n", cptst.sim, depthSim.sim);
                        }
                        else
                        {
                            // if (mp->verbose) printf("adding to %i ... ",i);
                            (*ptsStats)[i]->addCamPtStat(cptst);
                            // if (mp->verbose) printf("added\n",i);
                            float step = ptPixSize / 5.0;
                            (*ptsStats)[i]->step =
                                (((*ptsStats)[i]->step < 0.0f) ? step : std::min(step, (*ptsStats)[i]->step));
                        }
                    }
                }
                // printfEstimate(i,pts->size(),t2);
            }
            // finishEstimate();
        }

        delete dsmap;
    }

    if(mp->verbose)
        printfElapsedTime(t1, "actualizePtsStats");
}



void MeshEnergyOptPhotoMem::initPtsStats(StaticVector<StaticVector<int>*>* camsPts)
{
    for(int i = 0; i < ptsStats->size(); i++)
    {
        (*ptsStats)[i]->historyPtsSims->resize(0);
        (*ptsStats)[i]->step = -1.0f;
    }
    actualizePtsStats(camsPts);
}

float MeshEnergyOptPhotoMem::fuse_gaussianKernelVoting_computePtSim(int ptId, int s, float step, Point3d& pt,
                                                                           Point3d& nNormalized, float sigma)
{
    float sum = 0.0;
    for(int c = 0; c < (*ptsStats)[ptId]->camsDepthMapsPtsSims->size(); c++)
    {
        Point3d ptc = (*(*ptsStats)[ptId]->camsDepthMapsPtsSims)[c].pt;
        float sim = (*(*ptsStats)[ptId]->camsDepthMapsPtsSims)[c].sim;
        // sim = sim * dot(nNormalized,(mp->CArr[rc]-ptc).normalize());
        Point3d ptcptn = closestPointToLine3D(&ptc, &pt, &nNormalized);
        float dist = orientedPointPlaneDistance(ptcptn, pt, nNormalized);
        int i = dist / step;
        sum += sim * exp(-((float)(i - s) * (float)(i - s)) / (2.0f * sigma * sigma));
    }
    return sum;
}

void MeshEnergyOptPhotoMem::fuse_gaussianKernelVoting_depthsSimsSamples(double& optDist, double& optSim,
                                                                               double& actSim, int ptId,
                                                                               Point3d& nNormalized, int iter)
{
    double minSum = 10000.0;
    int minSums = 0;
    Point3d pt = (*pts)[ptId];
    double step = (*ptsStats)[ptId]->step;
    double sigma = 5.0;
    // int nSamplesHalf = ((ndepthsToRefine-1)/2)*5.0f;
    int nSamplesHalf = 20;
    if(iter > 0)
    {
        nSamplesHalf = std::min(20, abs((*ptsStats)[ptId]->optS) * 2) + 1;
    }

    if((iter == 0) || (abs((*ptsStats)[ptId]->optS) > 0) ||
       //(angleBetwUnitV1andUnitV2((*ptsStats)[ptId]->optVect,nNormalized)>2.0f)||
       (((*ptsStats)[ptId]->pt - pt).size() > step))
    {
        for(int s = -nSamplesHalf; s <= nSamplesHalf; s++)
        {
            double sum = fuse_gaussianKernelVoting_computePtSim(ptId, s, step, pt, nNormalized, sigma);

            if(sum < minSum)
            {
                minSum = sum;
                minSums = s;
            }

            if(s == 0)
            {
                actSim = sum;
            }
        }

        (*ptsStats)[ptId]->optS = minSums;
        (*ptsStats)[ptId]->pt = pt;
        (*ptsStats)[ptId]->optVect = nNormalized;
        (*ptsStats)[ptId]->optDepthMapsPt = pt + nNormalized * (minSums * step);
        (*ptsStats)[ptId]->optDepthMapsSim = minSum;
        stat_nActivePts++;
    }
    else
    {
        Point3d ptcptn = closestPointToLine3D(&(*ptsStats)[ptId]->optDepthMapsPt, &pt, &nNormalized);
        minSums = (int)(orientedPointPlaneDistance(pt, ptcptn, nNormalized) / step);
        minSum = (*ptsStats)[ptId]->optDepthMapsSim;
        actSim = fuse_gaussianKernelVoting_computePtSim(ptId, 0, step, pt, nNormalized, sigma);
    }

    optDist = (double)minSums * step;
    optSim = minSum;
}

void MeshEnergyOptPhotoMem::optimizePoint(int iter, int ptId, StaticVector<Point3d>* lapPts, bool photoSmooth,
                                                 Point3d& pt, Point3d& normalVectorNormalized,
                                                 double& smoothVal, double& simVal, StaticVectorBool* ptsCanMove)
{
    smoothVal = 0.0;
    simVal = 0.0;

    double lamda = 0.01;
    pt = (*pts)[ptId];
    normalVectorNormalized = Point3d(1.0f, 0.0f, 0.0f);
    Point3d smoothingVector, smoothingVectorNormalized;
    double smoothingVectorSize;
    double K1, K2, area, avNeighEdegeLenth;
    if(((ptsCanMove == nullptr) || ((*ptsCanMove)[ptId])) &&
       //((*ptsStats)[ptId]->camsDepthMapsPtsSims->size()>1)&&
       (!isIsBoundaryPt(ptId)) && (getBiLaplacianSmoothingVectorAndPrincipalCurvatures(
                                      ptId, lapPts, smoothingVector, smoothingVectorNormalized, normalVectorNormalized,
                                      smoothingVectorSize, K1, K2, area, avNeighEdegeLenth)))
    {
        smoothVal = sqrt(K1 * K1 + K2 * K2);
        double smoothValNormalized = smoothVal * sqrt(area);

        if(photoSmooth)
        {
            double photoStep;
            double optSim;
            double actSim;

            fuse_gaussianKernelVoting_depthsSimsSamples(photoStep, optSim, actSim, ptId, normalVectorNormalized, iter);
            /*
            Point3d optMove;
            fuse_gaussianKernelVoting_computeOptMoveVect(optMove, optSim, ptId, pt);
            photoStep = optMove.size();
            normalVectorNormalized = (optMove.normalize()+normalVectorNormalized).normalize();
            */

            photoStep = std::min(double(avNeighEdegeLenth / 10.0), photoStep);

            simVal = actSim;

            double photoWeight = sigmoidfcn(0.0, 1.0, 1.0, 0.1, smoothValNormalized);
            // double photoWeight  = sigmoidfcn(0.0f,1.0f,1.0f,0.2f,smoothValNormalized);
            // double photoWeight = std::max(0.0f,1.0f - smoothValNormalized);
            // double simWeight    = sigmoidfcn(0.0f,1.0f,0.7f,-0.8f,optSim);

            // normalize similarity to -1,0
            // figure; t = -5.0:0.01:0.0; plot(t,sigmoid(0.0,-1.0,6.0,-0.4,t,0));
            double simWeight = -sigmoidfcn(0.0, -1.0, 6.0, -0.4, optSim);

            photoWeight = simWeight * photoWeight;

            // double photoWeight  = sigmoidfcn(0.0f,1.0f,1.0f,0.5f,smoothValNormalized);
            // double simWeight    = sigmoidfcn(0.0f,1.0f,0.7f,-0.8f,optSim);
            // photoWeight = simWeight*photoWeight;

            // double photoWeight  = sigmoidfcn(0.0f,1.0f,1.0f,0.1f,smoothValNormalized);
            // double simWeight    = sigmoidfcn(0.0f,1.0f,0.7f,-2.8f,optSim);
            // photoWeight = std::max(simWeight,photoWeight);

            double smoothWeight = (1.0 - photoWeight);

            Point3d finalStep = normalVectorNormalized * (photoWeight * photoStep) +
                                smoothingVectorNormalized * (smoothWeight * smoothingVectorSize * lamda);

            if(std::isnan(finalStep.x) || std::isnan(finalStep.y) || std::isnan(finalStep.z) ||
               (finalStep.x != finalStep.x) || (finalStep.y != finalStep.y) ||
               (finalStep.z != finalStep.z)) // check if is not NaN
            {
            }
            else
            {
                pt = pt + finalStep;
            }
        }
        else
        {
            pt = pt + smoothingVector * lamda;
        }
    }
}

Point4d MeshEnergyOptPhotoMem::getPtCurvatures(int ptId, StaticVector<Point3d>* lapPts)
{
    Point4d out = Point4d();
    Point3d n;
    if((!isIsBoundaryPt(ptId)) && (getBiLaplacianSmoothingVector(ptId, lapPts, n)))
    {
        Point3d KhVect;
        getVertexMeanCurvatureNormal(ptId, KhVect);
        float Kh = KhVect.size();

        double Kg;
        getVertexGaussianCurvature(ptId, Kg);

        double K1;
        double K2;
        getVertexPrincipalCurvatures(Kh, Kg, K1, K2);

        out.x = Kh;
        out.y = Kg;
        out.z = K1;
        out.w = K2;
    }
    return out;
}

DepthSimMap* MeshEnergyOptPhotoMem::getDepthPixSizeMap(StaticVector<float>* rcDepthMap, int rc,
                                                                 StaticVector<int>* tcams)
{
    int w11 = sp->mp->mip->getWidth(rc);
    int h11 = sp->mp->mip->getHeight(rc);

    DepthSimMap* depthSimMapScale1Step1 = new DepthSimMap(rc, sp->mp, 1, 1);
    depthSimMapScale1Step1->initJustFromDepthMap(rcDepthMap, -1.0f);

    // set sim (y) to pixsize
    for(int y = 0; y < h11; y++)
    {
        for(int x = 0; x < w11; x++)
        {
            Point3d p = sp->mp->CArr[rc] +
                        (sp->mp->iCamArr[rc] * Point2d((float)x, (float)y)).normalize() *
                            (*depthSimMapScale1Step1->dsm)[y * w11 + x].depth;
            if(useTcOrRcPixSize)
            {
                (*depthSimMapScale1Step1->dsm)[y * w11 + x].sim = sp->mp->getCamsMinPixelSize(p, *tcams);
            }
            else
            {
                (*depthSimMapScale1Step1->dsm)[y * w11 + x].sim = sp->mp->getCamPixelSize(p, rc);
            }
        }
    }

    return depthSimMapScale1Step1;
}

void MeshEnergyOptPhotoMem::smoothBiLaplacian(int niters, StaticVectorBool* ptsCanMove)
{
    std::string energyStatFileNames = tmpDir + "energy_stat_smooth.txt";
    FILE* fes = fopen(energyStatFileNames.c_str(), "w");
    for(int iter = 0; iter < niters; iter++)
    {
        long t1 = clock();
        StaticVector<Point3d>* lapPts = computeLaplacianPts();
        StaticVector<Point3d>* newPts = new StaticVector<Point3d>(pts->size());
        newPts->push_back_arr(pts);
        double smoothSum = 0.0;
        double simSum = 0.0;
#pragma omp parallel for reduction(+:smoothSum,simSum)
        for(int i = 0; i < pts->size(); i++)
        {
            Point3d normal;
            double smoothVal = 0.0;
            double simVal = 0.0;
            optimizePoint(iter, i, lapPts, false, (*newPts)[i], normal, smoothVal, simVal, ptsCanMove);
            smoothSum += smoothVal;
            simSum += simVal;
        }
        printf("iter %i smooth %lf + sim %lf = %lf\n", iter, smoothSum, simSum, smoothSum + simSum);
        fprintf(fes, "%i %lf %lf %lf\n", iter, smoothSum, simSum, smoothSum + simSum);

        delete lapPts;
        delete pts;
        pts = newPts;
        printfElapsedTime(t1, "iter" + num2strThreeDigits(iter));
    }
    fclose(fes);
}

void MeshEnergyOptPhotoMem::smoothLaplacian(int niters)
{
    for(int iter = 0; iter < niters; iter++)
    {
        long t1 = clock();
        StaticVector<Point3d>* newPts = new StaticVector<Point3d>(pts->size());
        newPts->push_back_arr(pts);
#pragma omp parallel for
        for(int i = 0; i < pts->size(); i++)
        {
            Point3d normal;
            if(applyLaplacianOperator(i, pts, normal))
            {
                (*newPts)[i] = (*newPts)[i] + normal * 0.1;
            }
        }
        delete pts;
        pts = newPts;
        printfElapsedTime(t1, "iter" + num2strThreeDigits(iter));
    }
}

void MeshEnergyOptPhotoMem::saveIterStat(StaticVector<Point3d>* lapPts, int iter, float avEdgeLength)
{
    std::string iterStatFileName = tmpDir + "iter" + num2strThreeDigits(iter) + "_stat.txt";
    FILE* f = fopen(iterStatFileName.c_str(), "w");
    for(int i = 0; i < pts->size(); i++)
    {
        Point4d ptcurvs = getPtCurvatures(i, lapPts);
        StaticVector<int>* ptNeighPtsOrdered = (*ptsNeighPtsOrdered)[i];
        StaticVector<int>* ptNeighTris = (*ptsNeighTrisSortedAsc)[i];

        float ptNeighsAvEdgeLength = 0.0;
        if(sizeOfStaticVector<int>(ptNeighPtsOrdered) > 0)
        {
            for(int j = 0; j < sizeOfStaticVector<int>(ptNeighPtsOrdered); j++)
            {
                ptNeighsAvEdgeLength += ((*pts)[i] - (*pts)[(*ptNeighPtsOrdered)[j]]).size();
            }
            ptNeighsAvEdgeLength = ptNeighsAvEdgeLength / (float)sizeOfStaticVector<int>(ptNeighPtsOrdered);
        }

        float area = 0.0;
        for(int j = 0; j < ptNeighTris->size(); j++)
        {
            int triId = (*ptNeighTris)[j];
            int vertexIdInTriangle = getVertexIdInTriangleForPtId(i, triId);
            area += getRegionArea(vertexIdInTriangle, triId);
        }

        fprintf(f, "%f %f %f %f %f %f %f %f\n", ptcurvs.x, ptcurvs.y, ptcurvs.z, ptcurvs.w, (*ptsStats)[i]->step,
                ptNeighsAvEdgeLength, avEdgeLength, area);
    }
    fclose(f);
}

bool MeshEnergyOptPhotoMem::optimizePhoto(int niters, StaticVectorBool* ptsCanMove,
                                                 StaticVector<StaticVector<int>*>* camsPts)
{
    if(mp->verbose)
        printf("optimizePhoto\n");

    if(pts->size() <= 4)
    {
        return false;
    }

    float avEdgeLength = computeAverageEdgeLength();

    bool visualizeOptimizationStat = mp->mip->_ini.get<bool>("meshEnergyOpt.visualizeOptimizationStat", false);

    FILE* fe = nullptr;

    if(visualizeOptimizationStat)
    {
        std::string energyStatFileName = tmpDir + "energy_stat.txt";
        fe = fopen(energyStatFileName.c_str(), "w");
    }

    long t1 = clock();
    initEstimate();
    for(int iter = 0; iter < niters; iter++)
    {

        StaticVector<Point3d>* lapPts = computeLaplacianPtsParallel();

        StaticVector<Point3d>* newPts = new StaticVector<Point3d>(pts->size());
        newPts->push_back_arr(pts);

        StaticVector<Point3d>* nms = new StaticVector<Point3d>(pts->size());
        nms->resize(pts->size());

        if((iter > 0) && ((iter % 50 == 0) || (iter == 10) || (iter == 20)))
        {
            // visualizeMeshNormalsSmoothingVectors(iter);
            actualizePtsStats(camsPts);

            if(visualizeOptimizationStat)
            {
                saveIterStat(lapPts, iter, avEdgeLength);
            }
        }

        stat_nActivePts = 0;
        double smoothSum = 0.0;
        double simSum = 0.0;
#pragma omp parallel for reduction(+:smoothSum,simSum)
        for(int i = 0; i < pts->size(); i++)
        {
            double smoothVal = 0.0;
            double simVal = 0.0;
            optimizePoint(iter, i, lapPts, true, (*newPts)[i], (*nms)[i], smoothVal, simVal, ptsCanMove);
            smoothSum += smoothVal;
            simSum += simVal;
        }

        if(visualizeOptimizationStat)
        {
            double activePtsPerc = (double)stat_nActivePts / ((double)pts->size() / 100.0);
            printf("iter %i smooth %lf + sim %lf = %lf, active pts perc %f\n", iter, smoothSum, simSum,
                   smoothSum + simSum, activePtsPerc);
            fprintf(fe, "%i %lf %lf %lf %f\n", iter, smoothSum, simSum, smoothSum + simSum, activePtsPerc);
        }

        delete lapPts;
        delete nms;
        delete pts;
        pts = newPts;

        printfEstimate(iter, niters, t1);
    }
    finishEstimate();

    if(visualizeOptimizationStat)
    {
        fclose(fe);
    }

    return true;
}

StaticVector<StaticVector<int>*>*
MeshEnergyOptPhotoMem::getRcTcamsFromPtsCams(int minPairPts, StaticVector<StaticVector<int>*>* ptsCams)
{
    long tall = clock();
    if(mp->verbose)
        printf("getRcTcamsFromPtsCams\n");

    StaticVector<int>* rctcnpts = new StaticVector<int>(mp->ncams * mp->ncams);
    rctcnpts->resize_with(mp->ncams * mp->ncams, 0);

    long t1 = initEstimate();
    for(int k = 0; k < ptsCams->size(); k++)
    {
        StaticVector<int>* cams = (*ptsCams)[k];
        if(cams != nullptr)
        {
            for(int i = 0; i < cams->size(); i++)
            {
                for(int j = i + 1; j < cams->size(); j++)
                {
                    int rc = (*cams)[i];
                    int tc = (*cams)[j];
                    (*rctcnpts)[std::min(rc, tc) * mp->ncams + std::max(rc, tc)]++;
                }
            }
        }
        printfEstimate(k, ptsCams->size(), t1);
    }
    finishEstimate();

    StaticVector<StaticVector<int>*>* rTcams = new StaticVector<StaticVector<int>*>(mp->ncams);
    t1 = initEstimate();
    for(int rc = 0; rc < mp->ncams; rc++)
    {
        int n = 0;
        for(int tc = 0; tc < mp->ncams; tc++)
        {
            if((*rctcnpts)[std::min(rc, tc) * mp->ncams + std::max(rc, tc)] > minPairPts)
            {
                n++;
            }
        }
        StaticVector<int>* tcams = new StaticVector<int>(n);
        for(int tc = 0; tc < mp->ncams; tc++)
        {
            if((*rctcnpts)[std::min(rc, tc) * mp->ncams + std::max(rc, tc)] > minPairPts)
            {
                tcams->push_back(tc);
            }
        }
        rTcams->push_back(tcams);
        printfEstimate(rc, mp->ncams, t1);
    }
    finishEstimate();

    delete rctcnpts;

    if(mp->verbose)
        printfElapsedTime(tall);

    return rTcams;
}

void MeshEnergyOptPhotoMem::updateAddCamsSorted(StaticVector<int>** cams, int rc)
{
    if(*cams == nullptr)
    {
        *cams = new StaticVector<int>(10);
    }

    if((*cams)->indexOfSorted(rc) == -1)
    {
        if((*cams)->size() == (*cams)->capacity())
        {
            (*cams)->resizeAdd(10);
        }
        (*cams)->push_sorted_asc(rc);
    }
}
