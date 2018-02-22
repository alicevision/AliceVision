// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "RefineRc.hpp"
#include <aliceVision/structures/Point2d.hpp>
#include <aliceVision/structures/Point3d.hpp>
#include <aliceVision/common/common.hpp>
#include <aliceVision/common/fileIO.hpp>
#include <aliceVision/imageIO/image.hpp>
#include <aliceVision/omp.hpp>

#include <boost/filesystem.hpp>

namespace aliceVision {
namespace depthMap {

namespace bfs = boost::filesystem;

RefineRc::RefineRc(int _rc, int _scale, int _step, SemiGlobalMatchingParams* _sp)
    : SemiGlobalMatchingRc(false, _rc, _scale, _step, _sp)
{
    _nSamplesHalf = sp->mp->mip->_ini.get<int>("refineRc.nSamplesHalf", 150);
    _ndepthsToRefine = sp->mp->mip->_ini.get<int>("refineRc.ndepthsToRefine", 31);
    _sigma = (float)sp->mp->mip->_ini.get<double>("refineRc.sigma", 15.0);
    _niters = sp->mp->mip->_ini.get<int>("refineRc.niters", 100);

    _userTcOrPixSize = sp->mp->mip->_ini.get<bool>("refineRc.useTcOrRcPixSize", false);
    _wsh = sp->mp->mip->_ini.get<int>("refineRc.wsh", 3);
    _gammaC = (float)sp->mp->mip->_ini.get<double>("refineRc.gammaC", 15.5);
    _gammaP = (float)sp->mp->mip->_ini.get<double>("refineRc.gammaP", 8.0);
    
    int nnearestcams = sp->mp->mip->_ini.get<int>("refineRc.maxTCams", 6);
    tcams = sp->pc->findNearestCamsFromSeeds(rc, nnearestcams);
}

RefineRc::~RefineRc()
{
    //
}

DepthSimMap* RefineRc::getDepthPixSizeMapFromSGM()
{
    int w11 = sp->mp->mip->getWidth(rc);
    int h11 = sp->mp->mip->getHeight(rc);

    int volDimX = w;
    int volDimY = h;
    int volDimZ = depths->size();

    StaticVector<IdValue>* volumeBestIdVal = new StaticVector<IdValue>(volDimX * volDimY);

    {
        std::vector<unsigned short> rcIdDepthMap;

        imageIO::readImage(SGM_idDepthMapFileName, volDimX, volDimY, rcIdDepthMap);

        for(int i = 0; i < volDimX * volDimY; i++)
        {
            // float sim = (*depthSimMapFinal->dsm)[i].y;
            // sim = std::min(sim,mp->simThr);
            float sim = sp->mp->simThr - 0.0001;

            int id = rcIdDepthMap.at(i);
            id = (id > 0) && (id < volDimZ - sp->minObjectThickness - 1) ? id : 0;
            volumeBestIdVal->push_back(IdValue(id, sim));
        }
    }

    int zborder = 2;
    DepthSimMap* depthSimMap =
        sp->getDepthSimMapFromBestIdVal(w, h, volumeBestIdVal, scale, step, rc, zborder, depths);
    delete volumeBestIdVal;

    DepthSimMap* depthSimMapScale1Step1 = new DepthSimMap(rc, sp->mp, 1, 1);
    depthSimMapScale1Step1->add11(depthSimMap);
    delete depthSimMap;

    // set sim (y) to pixsize
    for(int y = 0; y < h11; y++)
    {
        for(int x = 0; x < w11; x++)
        {
            Point3d p = sp->mp->CArr[rc] +
                        (sp->mp->iCamArr[rc] * Point2d((float)x, (float)y)).normalize() *
                            (*depthSimMapScale1Step1->dsm)[y * w11 + x].depth;
            if(_userTcOrPixSize)
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

DepthSimMap* RefineRc::refineAndFuseDepthSimMapCUDA(DepthSimMap* depthPixSizeMapVis)
{
    int w11 = sp->mp->mip->getWidth(rc);
    int h11 = sp->mp->mip->getHeight(rc);

    StaticVector<DepthSimMap*>* dataMaps = new StaticVector<DepthSimMap*>(tcams->size() + 1);
    dataMaps->push_back(depthPixSizeMapVis); //!!DO NOT ERASE!!!

    for(int c = 0; c < tcams->size(); c++)
    {
        int tc = (*tcams)[c];

        DepthSimMap* depthSimMapC = new DepthSimMap(rc, sp->mp, 1, 1);
        StaticVector<float>* depthMap = depthPixSizeMapVis->getDepthMap();
        depthSimMapC->initJustFromDepthMap(depthMap, 1.0f);
        delete depthMap;

        sp->prt->refineRcTcDepthSimMap(_userTcOrPixSize, depthSimMapC, rc, tc, _ndepthsToRefine, _wsh, _gammaC, _gammaP,
                                       0.0f);

        dataMaps->push_back(depthSimMapC);

        if(sp->visualizePartialDepthMaps)
            depthSimMapC->saveToImage(outDir + "refineRc_Photo_" + std::to_string(sp->mp->mip->getViewId(rc)) + "_tc_" +
                                           std::to_string(sp->mp->mip->getViewId(tc)) + ".depthSimMap.png", -2.0f);
    }

    // in order to fit into GPU memory
    DepthSimMap* depthSimMapFused = new DepthSimMap(rc, sp->mp, 1, 1);

    int nhParts = 4;
    int hPartHeightGlob = h11 / nhParts;
    for(int hPart = 0; hPart < nhParts; hPart++)
    {
        int hPartHeight = std::min(h11, (hPart + 1) * hPartHeightGlob) - hPart * hPartHeightGlob;

        // vector of one depthSimMap tile per Tc
        StaticVector<StaticVector<DepthSim>*>* dataMapsHPart =
            new StaticVector<StaticVector<DepthSim>*>(dataMaps->size());
        for(int i = 0; i < dataMaps->size(); i++) // iterate over Tc cameras
        {
            StaticVector<DepthSim>* dataMapHPart = new StaticVector<DepthSim>(w11 * hPartHeight);
            dataMapHPart->resize(w11 * hPartHeight);

#pragma omp parallel for
            for(int y = 0; y < hPartHeight; y++)
            {
                for(int x = 0; x < w11; x++)
                {
                    (*dataMapHPart)[y * w11 + x] = (*(*dataMaps)[i]->dsm)[(y + hPart * hPartHeightGlob) * w11 + x];
                }
            }

            dataMapsHPart->push_back(dataMapHPart);
        }

        StaticVector<DepthSim>* depthSimMapFusedHPart = new StaticVector<DepthSim>(w11 * hPartHeight);
        depthSimMapFusedHPart->resize_with(w11 * hPartHeight, DepthSim(-1.0f, 1.0f));

        sp->cps->fuseDepthSimMapsGaussianKernelVoting(w11, hPartHeight, depthSimMapFusedHPart, dataMapsHPart,
                                                      _nSamplesHalf, _ndepthsToRefine, _sigma);

#pragma omp parallel for
        for(int y = 0; y < hPartHeight; y++)
        {
            for(int x = 0; x < w11; x++)
            {
                (*depthSimMapFused->dsm)[(y + hPart * hPartHeightGlob) * w11 + x] =
                    (*depthSimMapFusedHPart)[y * w11 + x];
            }
        }

        delete depthSimMapFusedHPart;
        deleteArrayOfArrays<DepthSim>(&dataMapsHPart);
    }

    (*dataMaps)[0] = nullptr; // it is input dsmap we dont want to delete it
    for(int c = 0; c < tcams->size(); c++)
    {
        delete(*dataMaps)[c + 1];
    }
    delete dataMaps;

    return depthSimMapFused;
}

DepthSimMap* RefineRc::optimizeDepthSimMapCUDA(DepthSimMap* depthPixSizeMapVis,
                                                      DepthSimMap* depthSimMapPhoto)
{
    int h11 = sp->mp->mip->getHeight(rc);

    StaticVector<DepthSimMap*>* dataMaps = new StaticVector<DepthSimMap*>(2);
    dataMaps->push_back(depthPixSizeMapVis); //!!DO NOT ERASE!!!
    dataMaps->push_back(depthSimMapPhoto);   //!!DO NOT ERASE!!!

    DepthSimMap* depthSimMapOptimized = new DepthSimMap(rc, sp->mp, 1, 1);

    {
        StaticVector<StaticVector<DepthSim>*>* dataMapsPtrs = new StaticVector<StaticVector<DepthSim>*>(dataMaps->size());
        for(int i = 0; i < dataMaps->size(); i++)
        {
            dataMapsPtrs->push_back((*dataMaps)[i]->dsm);
        }

        int nParts = 4;
        int hPart = h11 / nParts;
        for(int part = 0; part < nParts; part++)
        {
            int yFrom = part * hPart;
            int hPartAct = std::min(hPart, h11 - yFrom);
            sp->cps->optimizeDepthSimMapGradientDescent(depthSimMapOptimized->dsm, dataMapsPtrs, rc, _nSamplesHalf,
                                                        _ndepthsToRefine, _sigma, _niters, yFrom, hPartAct);
        }

        for(int i = 0; i < dataMaps->size(); i++)
        {
            (*dataMapsPtrs)[i] = nullptr;
        }
        delete dataMapsPtrs;
    }

    (*dataMaps)[0] = nullptr; // it is input dsmap we dont want to delete it
    (*dataMaps)[1] = nullptr; // it is input dsmap we dont want to delete it
    delete dataMaps;

    return depthSimMapOptimized;
}

bool RefineRc::refinercCUDA(bool checkIfExists)
{
    const IndexT viewId = sp->mp->mip->getViewId(rc);

    if(sp->mp->verbose)
        printf("processing refinercCUDA %i of %i\n", rc + 1, sp->mp->ncams);

    // generate default depthSimMap if rc has no tcam
    if(tcams == nullptr || depths == nullptr)
    {
        DepthSimMap depthSimMapOpt(rc, sp->mp, 1, 1);
        depthSimMapOpt.save(rc, nullptr);
        return true;
    }

    if(checkIfExists && (common::FileExists(sp->getREFINE_opt_simMapFileName(viewId, 1, 1))))
    {
        return false;
    }

    long tall = clock();

    DepthSimMap* depthPixSizeMapVis = getDepthPixSizeMapFromSGM();

    if(sp->visualizeDepthMaps)
        depthPixSizeMapVis->saveToImage(outDir + "refineRc_" + std::to_string(viewId) + "Vis.png", 0.0f);

    DepthSimMap* depthSimMapPhoto = refineAndFuseDepthSimMapCUDA(depthPixSizeMapVis);

    if(sp->visualizeDepthMaps)
        depthSimMapPhoto->saveToImage(outDir + "refineRc_" + std::to_string(viewId) + "Photo.png", 0.0f);

    DepthSimMap* depthSimMapOpt = nullptr;
    if(sp->doRefineRc)
    {
        depthSimMapOpt = optimizeDepthSimMapCUDA(depthPixSizeMapVis, depthSimMapPhoto);
    }
    else
    {
        depthSimMapOpt = new DepthSimMap(rc, sp->mp, 1, 1);
        depthSimMapOpt->add(depthSimMapPhoto);
    }

    if(sp->visualizeDepthMaps)
        depthSimMapOpt->saveToImage(outDir + "refineRc_" + std::to_string(viewId) + "Opt.png", 0.0f);

    depthSimMapOpt->save(rc, tcams);

    if(sp->visualizeDepthMaps)
    {
        depthSimMapPhoto->saveRefine(rc,
                                    sp->getREFINE_photo_depthMapFileName(viewId, 1, 1),
                                    sp->getREFINE_photo_simMapFileName(viewId, 1, 1));

        depthSimMapOpt->saveRefine(rc,
                                   sp->getREFINE_opt_depthMapFileName(viewId, 1, 1),
                                   sp->getREFINE_opt_simMapFileName(viewId, 1, 1));
    }

    common::printfElapsedTime(tall, "refinerc CUDA: " + common::num2str(rc) + " of " + common::num2str(sp->mp->ncams) + ", " + std::to_string(viewId) + " done.");

    delete depthPixSizeMapVis;
    delete depthSimMapPhoto;
    delete depthSimMapOpt;

    return true;
}

void refineDepthMaps(int CUDADeviceNo, common::MultiViewParams* mp, common::PreMatchCams* pc, const StaticVector<int>& cams)
{
    int scale = mp->mip->_ini.get<int>("semiGlobalMatching.scale", -1);
    int step = mp->mip->_ini.get<int>("semiGlobalMatching.step", -1);

    if(scale == -1)
    {
        int width = mp->mip->getMaxImageWidth(); 
        int height = mp->mip->getMaxImageHeight(); 
        int scaleTmp = computeStep(mp->mip, 1, (width > height ? 700 : 550), (width > height ? 550 : 700));
        scale = std::min(2, scaleTmp);
        step = computeStep(mp->mip, scale, (width > height ? 700 : 550), (width > height ? 550 : 700));
        printf("PSSGM autoScaleStep %i %i\n", scale, step);
    }

    int bandType = 0;
    common::ImagesCache* ic = new common::ImagesCache(mp, bandType, true);
    PlaneSweepingCuda* cps = new PlaneSweepingCuda(CUDADeviceNo, ic, mp, pc, scale);
    SemiGlobalMatchingParams* sp = new SemiGlobalMatchingParams(mp, pc, cps);

    //////////////////////////////////////////////////////////////////////////////////////////

    for(const int rc : cams)
    {
        if(!common::FileExists(sp->getREFINE_opt_simMapFileName(mp->mip->getViewId(rc), 1, 1)))
        {
            RefineRc* rrc = new RefineRc(rc, scale, step, sp);
            rrc->refinercCUDA();
            delete rrc;
        }
    }

    delete sp;
    delete ic;
    delete cps;
}

void refineDepthMaps(common::MultiViewParams* mp, common::PreMatchCams* pc, const StaticVector<int>& cams)
{
    int num_gpus = listCUDADevices(true);
    int num_cpu_threads = omp_get_num_procs();
    std::cout << "Number of GPU devices: " << num_gpus << ", number of CPU threads: " << num_cpu_threads << std::endl;
    int numthreads = std::min(num_gpus, num_cpu_threads);

    int num_gpus_to_use = mp->mip->_ini.get<int>("refineRc.num_gpus_to_use", 1);
    if(num_gpus_to_use > 0)
    {
        numthreads = num_gpus_to_use;
    }

    if(numthreads == 1)
    {
        refineDepthMaps(mp->CUDADeviceNo, mp, pc, cams);
    }
    else
    {
        omp_set_num_threads(numthreads); // create as many CPU threads as there are CUDA devices
#pragma omp parallel
        {
            int cpu_thread_id = omp_get_thread_num();
            int CUDADeviceNo = cpu_thread_id % numthreads;
            std::cout << "CPU thread " << cpu_thread_id << " (of " << numthreads << ") uses CUDA device: " << CUDADeviceNo << std::endl;

            int rcFrom = CUDADeviceNo * (cams.size() / numthreads);
            int rcTo = (CUDADeviceNo + 1) * (cams.size() / numthreads);
            if(CUDADeviceNo == numthreads - 1)
            {
                rcTo = cams.size();
            }
            StaticVector<int> subcams(cams.size());
            for(int rc = rcFrom; rc < rcTo; rc++)
            {
                subcams.push_back(cams[rc]);
            }
            refineDepthMaps(cpu_thread_id, mp, pc, subcams);
        }
    }
}

} // namespace depthMap
} // namespace aliceVision
