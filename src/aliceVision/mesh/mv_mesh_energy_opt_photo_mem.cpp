// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "mv_mesh_energy_opt_photo_mem.hpp"

#include <aliceVision/common/fileIO.hpp>
#include <aliceVision/imageIO/imageScaledColors.hpp>

#include <boost/filesystem.hpp>


namespace bfs = boost::filesystem;

mv_mesh_energy_opt_photo_mem::ptStat::ptStat()
{
    camsDepthMapsPtsSims = new staticVector<camPtStat>(10);
    historyPtsSims = new staticVector<point4d>(10);
}

mv_mesh_energy_opt_photo_mem::ptStat::~ptStat()
{
    delete camsDepthMapsPtsSims;
    delete historyPtsSims;
}

int mv_mesh_energy_opt_photo_mem::ptStat::indexOfNewCamPtStatSortedByCamPixSize(float val)
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

void mv_mesh_energy_opt_photo_mem::ptStat::addCamPtStat(camPtStat& cptst)
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

mv_mesh_energy_opt_photo_mem::mv_mesh_energy_opt_photo_mem(multiviewParams* _mp, ps_sgm_params* _sp,
                                                           const staticVector<int>& _usedCams)
    : mv_mesh_energy_opt(_mp)
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
    userTcOrPixSize = (bool)sp->mp->mip->_ini.get<bool>("meshEnergyOpt.userTcOrPixSize", false);
    nSamplesHalf = sp->mp->mip->_ini.get<int>("meshEnergyOpt.nSamplesHalf", 50);
    ndepthsToRefine = sp->mp->mip->_ini.get<int>("meshEnergyOpt.ndepthsToRefine", 11);
    sigma = (float)sp->mp->mip->_ini.get<double>("meshEnergyOpt.sigma", 10.0);
    pixSizeRatioThr = (float)sp->mp->mip->_ini.get<double>("meshEnergyOpt.pixSizeRatioThr", 1.2);
}

mv_mesh_energy_opt_photo_mem::~mv_mesh_energy_opt_photo_mem()
{
    deAllocatePtsStats();
}

void mv_mesh_energy_opt_photo_mem::allocatePtsStats()
{
    if(mp->verbose)
        printf("allocatePtsStats\n");
    ptsStats = new staticVector<ptStat*>(pts->size());
    for(int i = 0; i < pts->size(); i++)
    {
        ptStat* ptst = new ptStat();
        ptsStats->push_back(ptst);
    }
}

void mv_mesh_energy_opt_photo_mem::deAllocatePtsStats()
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

double mv_mesh_energy_opt_photo_mem::computeEnergyFairForPt(int ptid)
{
    point3d Kh;
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

staticVector<int>* mv_mesh_energy_opt_photo_mem::getRcTcNVisTrisMap(staticVector<staticVector<int>*>* trisCams)
{
    if(sp->mp->verbose)
        printf("getRcTcNVisTrisMap\n");

    staticVector<int>* out = new staticVector<int>(sp->mp->ncams * sp->mp->ncams);
    out->resize_with(sp->mp->ncams * sp->mp->ncams, 0);

    int maxVal = 0;
    for(int i = 0; i < tris->size(); i++)
    {
        staticVector<int>* triCams = (*trisCams)[i];
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

void mv_mesh_energy_opt_photo_mem::actualizePtsStats(staticVector<staticVector<int>*>* camsPts)
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

        ps_depthSimMap* dsmap = new ps_depthSimMap(rc, sp->mp, 1, 1);
        if(dsmap->loadFromBin(sp->getREFINE_photo_depthMapFileName(rc, 1, 1),
                              sp->getREFINE_photo_simMapFileName(rc, 1, 1)))
        {
            // dsmap->load(rc,1); //from opt is worser than from photo ... don't use it
            // long t2 = initEstimate();
            staticVector<int>* ptsIds = (*camsPts)[rc];
            // for (int i=0;i<pts->size();i++)
            for(int j = 0; j < sizeOfStaticVector<int>(ptsIds); j++)
            {
                int i = (*ptsIds)[j];
                point3d pt = (*pts)[i];
                pixel pix;
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
                                   (sp->mp->iCamArr[rc] * point2d((float)pix.x, (float)pix.y)).normalize() * depthSim.depth;
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



void mv_mesh_energy_opt_photo_mem::initPtsStats(staticVector<staticVector<int>*>* camsPts)
{
    for(int i = 0; i < ptsStats->size(); i++)
    {
        (*ptsStats)[i]->historyPtsSims->resize(0);
        (*ptsStats)[i]->step = -1.0f;
    }
    actualizePtsStats(camsPts);
}

float mv_mesh_energy_opt_photo_mem::fuse_gaussianKernelVoting_computePtSim(int ptId, int s, float step, point3d& pt,
                                                                           point3d& nNormalized, float sigma)
{
    float sum = 0.0;
    for(int c = 0; c < (*ptsStats)[ptId]->camsDepthMapsPtsSims->size(); c++)
    {
        point3d ptc = (*(*ptsStats)[ptId]->camsDepthMapsPtsSims)[c].pt;
        float sim = (*(*ptsStats)[ptId]->camsDepthMapsPtsSims)[c].sim;
        // sim = sim * dot(nNormalized,(mp->CArr[rc]-ptc).normalize());
        point3d ptcptn = closestPointToLine3D(&ptc, &pt, &nNormalized);
        float dist = orientedPointPlaneDistance(ptcptn, pt, nNormalized);
        int i = dist / step;
        sum += sim * exp(-((float)(i - s) * (float)(i - s)) / (2.0f * sigma * sigma));
    }
    return sum;
}

void mv_mesh_energy_opt_photo_mem::fuse_gaussianKernelVoting_depthsSimsSamples(double& optDist, double& optSim,
                                                                               double& actSim, int ptId,
                                                                               point3d& nNormalized, int iter)
{
    double minSum = 10000.0;
    int minSums = 0;
    point3d pt = (*pts)[ptId];
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
        point3d ptcptn = closestPointToLine3D(&(*ptsStats)[ptId]->optDepthMapsPt, &pt, &nNormalized);
        minSums = (int)(orientedPointPlaneDistance(pt, ptcptn, nNormalized) / step);
        minSum = (*ptsStats)[ptId]->optDepthMapsSim;
        actSim = fuse_gaussianKernelVoting_computePtSim(ptId, 0, step, pt, nNormalized, sigma);
    }

    optDist = (double)minSums * step;
    optSim = minSum;
}

void mv_mesh_energy_opt_photo_mem::optimizePoint(int iter, int ptId, staticVector<point3d>* lapPts, bool photoSmooth,
                                                 point3d& pt, point3d& normalVectorNormalized,
                                                 double& smoothVal, double& simVal, staticVectorBool* ptsCanMove)
{
    smoothVal = 0.0;
    simVal = 0.0;

    double lamda = 0.01;
    pt = (*pts)[ptId];
    normalVectorNormalized = point3d(1.0f, 0.0f, 0.0f);
    point3d smoothingVector, smoothingVectorNormalized;
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
            point3d optMove;
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

            point3d finalStep = normalVectorNormalized * (photoWeight * photoStep) +
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

point4d mv_mesh_energy_opt_photo_mem::getPtCurvatures(int ptId, staticVector<point3d>* lapPts)
{
    point4d out = point4d();
    point3d n;
    if((!isIsBoundaryPt(ptId)) && (getBiLaplacianSmoothingVector(ptId, lapPts, n)))
    {
        point3d KhVect;
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

ps_depthSimMap* mv_mesh_energy_opt_photo_mem::getDepthPixSizeMap(staticVector<float>* rcDepthMap, int rc,
                                                                 staticVector<int>* tcams)
{
    int w11 = sp->mp->mip->getWidth(rc);
    int h11 = sp->mp->mip->getHeight(rc);

    ps_depthSimMap* depthSimMapScale1Step1 = new ps_depthSimMap(rc, sp->mp, 1, 1);
    depthSimMapScale1Step1->initJustFromDepthMap(rcDepthMap, -1.0f);

    // set sim (y) to pixsize
    for(int y = 0; y < h11; y++)
    {
        for(int x = 0; x < w11; x++)
        {
            point3d p = sp->mp->CArr[rc] +
                        (sp->mp->iCamArr[rc] * point2d((float)x, (float)y)).normalize() *
                            (*depthSimMapScale1Step1->dsm)[y * w11 + x].depth;
            if(userTcOrPixSize)
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

void mv_mesh_energy_opt_photo_mem::smoothBiLaplacian(int niters, staticVectorBool* ptsCanMove)
{
    std::string energyStatFileNames = tmpDir + "energy_stat_smooth.txt";
    FILE* fes = fopen(energyStatFileNames.c_str(), "w");
    for(int iter = 0; iter < niters; iter++)
    {
        long t1 = clock();
        staticVector<point3d>* lapPts = computeLaplacianPts();
        staticVector<point3d>* newPts = new staticVector<point3d>(pts->size());
        newPts->push_back_arr(pts);
        double smoothSum = 0.0;
        double simSum = 0.0;
#pragma omp parallel for reduction(+:smoothSum,simSum)
        for(int i = 0; i < pts->size(); i++)
        {
            point3d normal;
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

void mv_mesh_energy_opt_photo_mem::smoothLaplacian(int niters)
{
    for(int iter = 0; iter < niters; iter++)
    {
        long t1 = clock();
        staticVector<point3d>* newPts = new staticVector<point3d>(pts->size());
        newPts->push_back_arr(pts);
#pragma omp parallel for
        for(int i = 0; i < pts->size(); i++)
        {
            point3d normal;
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

void mv_mesh_energy_opt_photo_mem::saveIterStat(staticVector<point3d>* lapPts, int iter, float avEdgeLength)
{
    std::string iterStatFileName = tmpDir + "iter" + num2strThreeDigits(iter) + "_stat.txt";
    FILE* f = fopen(iterStatFileName.c_str(), "w");
    for(int i = 0; i < pts->size(); i++)
    {
        point4d ptcurvs = getPtCurvatures(i, lapPts);
        staticVector<int>* ptNeighPtsOrdered = (*ptsNeighPtsOrdered)[i];
        staticVector<int>* ptNeighTris = (*ptsNeighTrisSortedAsc)[i];

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

bool mv_mesh_energy_opt_photo_mem::optimizePhoto(int niters, staticVectorBool* ptsCanMove,
                                                 staticVector<staticVector<int>*>* camsPts)
{
    if(mp->verbose)
        printf("optimizePhoto\n");

    if(pts->size() <= 4)
    {
        return false;
    }

    float avEdgeLength = computeAverageEdgeLength();

    bool visualizeOptimizationWrl = mp->mip->_ini.get<bool>("meshEnergyOpt.visualizeOptimizationWrl", false);
    bool visualizeOptimizationStat =
        mp->mip->_ini.get<bool>("meshEnergyOpt.visualizeOptimizationStat", false);

    if(visualizeOptimizationWrl)
        o3d->saveMvMeshToWrl(this, tmpDir + "iterOrig.wrl");

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

        staticVector<point3d>* lapPts = computeLaplacianPtsParallel();

        staticVector<point3d>* newPts = new staticVector<point3d>(pts->size());
        newPts->push_back_arr(pts);

        staticVector<point3d>* nms = new staticVector<point3d>(pts->size());
        nms->resize(pts->size());

        if((visualizeOptimizationWrl) && (iter % 500 == 0))
        {
            visualizeMeshNormalsSmoothingVectors(iter);
        }

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

    if(visualizeOptimizationWrl)
    {
        visualizeMeshNormalsSmoothingVectors(niters);
    }

    return true;
}

void mv_mesh_energy_opt_photo_mem::visualizeMeshNormalsSmoothingVectors(int iter)
{

    std::string meshFileName = tmpDir + "iter" + num2strFourDecimal(iter) + "mesh.wrl";
    o3d->saveMvMeshToWrl(this, meshFileName);

    staticVector<point3d>* lapPts = computeLaplacianPts();

    staticVector<point3d>* svs = new staticVector<point3d>(pts->size());
    svs->resize_with(pts->size(), point3d(0.0f, 0.0f, 0.0f));

    staticVector<point3d>* nms = new staticVector<point3d>(pts->size());
    nms->resize_with(pts->size(), point3d(0.0f, 0.0f, 0.0f));

    mv_mesh* mePhoto = new mv_mesh();
    mePhoto->addMesh(this);

    for(int i = 0; i < pts->size(); i++)
    {
        float step = (*ptsStats)[i]->step;

        point3d N;
        getVertexSurfaceNormal(i, N);
        N = N.normalize() * (step * 50.0f);
        if(std::isnan(N.x) || std::isnan(N.y) || std::isnan(N.z) || (N.x != N.x) || (N.y != N.y) ||
           (N.z != N.z)) // check if is not NaN
        {
            //
        }
        else
        {
            (*nms)[i] = N;
        }

        point3d tp, n;
        double smoothStep;
        double K1, K2, area, avNeighEdegeLenth;
        if((!isIsBoundaryPt(i)) && (getBiLaplacianSmoothingVectorAndPrincipalCurvatures(
                                       i, lapPts, tp, n, N, smoothStep, K1, K2, area, avNeighEdegeLenth)))
        {
            (*svs)[i] = n * (step * 50.0f);
        }

        (*mePhoto->pts)[i] = (*ptsStats)[i]->optDepthMapsPt;
    }

    std::string normalsFileName = tmpDir + "iter" + num2strFourDecimal(iter) + "normals.wrl";
    o3d->create_wrl_pts_nms(pts, nms, mp, normalsFileName, 0.0f, 1.0f, 0.0f);

    std::string smootingVectorsFileName = tmpDir + "iter" + num2strFourDecimal(iter) + "smoothingNormals.wrl";
    o3d->create_wrl_pts_nms(pts, svs, mp, smootingVectorsFileName, 1.0f, 0.0f, 0.0f);

    std::string photoMeshFileName = tmpDir + "iter" + num2strFourDecimal(iter) + "photoMesh.wrl";
    o3d->saveMvMeshToWrl(mePhoto, photoMeshFileName);

    delete mePhoto;

    std::string allFileName = tmpDir + "iter" + num2strFourDecimal(iter) + "all.wrl";
    FILE* f = fopen(allFileName.c_str(), "w");
    fprintf(f, "#VRML V2.0 utf8\n");
    fprintf(f, "Background {\n skyColor 1 1 1 \n } \n");
    fprintf(f, "Inline{ url [\"%s\"] \n }\n", meshFileName.c_str());
    fprintf(f, "Inline{ url [\"%s\"] \n }\n", normalsFileName.c_str());
    fprintf(f, "Inline{ url [\"%s\"] \n }\n", smootingVectorsFileName.c_str());
    fprintf(f, "Inline{ url [\"%s\"] \n }\n", photoMeshFileName.c_str());
    fclose(f);

    delete lapPts;
    delete svs;
    delete nms;
}

staticVector<staticVector<int>*>*
mv_mesh_energy_opt_photo_mem::getRcTcamsFromPtsCams(int minPairPts, staticVector<staticVector<int>*>* ptsCams)
{
    long tall = clock();
    if(mp->verbose)
        printf("getRcTcamsFromPtsCams\n");

    staticVector<int>* rctcnpts = new staticVector<int>(mp->ncams * mp->ncams);
    rctcnpts->resize_with(mp->ncams * mp->ncams, 0);

    long t1 = initEstimate();
    for(int k = 0; k < ptsCams->size(); k++)
    {
        staticVector<int>* cams = (*ptsCams)[k];
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

    staticVector<staticVector<int>*>* rTcams = new staticVector<staticVector<int>*>(mp->ncams);
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
        staticVector<int>* tcams = new staticVector<int>(n);
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

void mv_mesh_energy_opt_photo_mem::updateAddCamsSorted(staticVector<int>** cams, int rc)
{
    if(*cams == nullptr)
    {
        *cams = new staticVector<int>(10);
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
