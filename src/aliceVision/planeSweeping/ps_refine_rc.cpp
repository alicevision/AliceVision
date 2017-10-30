#include "ps_refine_rc.hpp"

#include <aliceVision/structures/mv_filesio.hpp>

#include <aliceVision/omp.hpp>

#include <boost/filesystem.hpp>


namespace bfs = boost::filesystem;

ps_refine_rc::ps_refine_rc(int _rc, int _scale, int _step, ps_sgm_params* _sp)
    : ps_sgm_rc(false, _rc, _scale, _step, _sp)
{
    // change out dir
    outDir = sp->getREFINEOutDir();
    if(!FolderExists(outDir))
    {
        bfs::create_directory(outDir);
    }

    tmpDir = sp->getREFINETmpDir();
    if(!FolderExists(tmpDir))
    {
        bfs::create_directory(tmpDir);
    }

    _nSamplesHalf = sp->mp->mip->_ini.get<int>("refineRc.nSamplesHalf", 150);
    _ndepthsToRefine = sp->mp->mip->_ini.get<int>("refineRc.ndepthsToRefine", 31);
    _sigma = (float)sp->mp->mip->_ini.get<double>("refineRc.sigma", 15.0);
    _niters = sp->mp->mip->_ini.get<int>("refineRc.niters", 100);

    _userTcOrPixSize = sp->mp->mip->_ini.get<bool>("refineRc.userTcOrPixSize", false);
    _wsh = sp->mp->mip->_ini.get<int>("refineRc.wsh", 3);
    _gammaC = (float)sp->mp->mip->_ini.get<double>("refineRc.gammaC", 15.5);
    _gammaP = (float)sp->mp->mip->_ini.get<double>("refineRc.gammaP", 8.0);
    
    int nnearestcams = sp->mp->mip->_ini.get<int>("refineRc.maxTCams", 6);
    tcams = sp->pc->findNearestCamsFromSeeds(rc, nnearestcams);
}

ps_refine_rc::~ps_refine_rc()
{
    //
}

ps_depthSimMap* ps_refine_rc::getDepthPixSizeMapFromSGM()
{
    int w11 = sp->mp->mip->getWidth(rc);
    int h11 = sp->mp->mip->getHeight(rc);

    int volDimX = w;
    int volDimY = h;
    int volDimZ = depths->size();

    staticVector<unsigned short>* rcIdDepthMap = nullptr;

    rcIdDepthMap = loadArrayFromFile<unsigned short>(SGM_idDepthMapFileName);

    staticVector<idValue>* volumeBestIdVal = new staticVector<idValue>(volDimX * volDimY);
    for(int i = 0; i < volDimX * volDimY; i++)
    {
        // float sim = (*depthSimMapFinal->dsm)[i].y;
        // sim = std::min(sim,mp->simThr);
        float sim = sp->mp->simThr - 0.0001;

        int id = (*rcIdDepthMap)[i];
        id = (id > 0) && (id < volDimZ - sp->minObjectThickness - 1) ? id : 0;
        volumeBestIdVal->push_back(idValue(id, sim));
    }
    delete rcIdDepthMap;

    int zborder = 2;
    ps_depthSimMap* depthSimMap =
        sp->getDepthSimMapFromBestIdVal(w, h, volumeBestIdVal, scale, step, rc, zborder, depths);
    delete volumeBestIdVal;

    ps_depthSimMap* depthSimMapScale1Step1 = new ps_depthSimMap(rc, sp->mp, 1, 1);
    depthSimMapScale1Step1->add11(depthSimMap);
    delete depthSimMap;

    // set sim (y) to pixsize
    for(int y = 0; y < h11; y++)
    {
        for(int x = 0; x < w11; x++)
        {
            point3d p = sp->mp->CArr[rc] +
                        (sp->mp->iCamArr[rc] * point2d((float)x, (float)y)).normalize() *
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

ps_depthSimMap* ps_refine_rc::refineAndFuseDepthSimMapCUDA(ps_depthSimMap* depthPixSizeMapVis)
{
    int w11 = sp->mp->mip->getWidth(rc);
    int h11 = sp->mp->mip->getHeight(rc);

    staticVector<ps_depthSimMap*>* dataMaps = new staticVector<ps_depthSimMap*>(tcams->size() + 1);
    dataMaps->push_back(depthPixSizeMapVis); //!!DO NOT ERASE!!!

    for(int c = 0; c < tcams->size(); c++)
    {
        int tc = (*tcams)[c];

        ps_depthSimMap* depthSimMapC = new ps_depthSimMap(rc, sp->mp, 1, 1);
        staticVector<float>* depthMap = depthPixSizeMapVis->getDepthMap();
        depthSimMapC->initJustFromDepthMap(depthMap, 1.0f);
        delete depthMap;

        sp->prt->refineRcTcDepthSimMap(_userTcOrPixSize, depthSimMapC, rc, tc, _ndepthsToRefine, _wsh, _gammaC, _gammaP,
                                       0.0f);

        dataMaps->push_back(depthSimMapC);

        if(sp->visualizePartialDepthMaps)
            depthSimMapC->saveToWrlPng(tmpDir + "refineRc_Photo_" + num2strFourDecimal(rc) + "_tc_" +
                                           num2strFourDecimal(tc) + ".wrl",
                                       rc, -2.0f);

    }

    // in order to fit into GPU memory
    ps_depthSimMap* depthSimMapFused = new ps_depthSimMap(rc, sp->mp, 1, 1);

    int nhParts = 4;
    int hPartHeightGlob = h11 / nhParts;
    for(int hPart = 0; hPart < nhParts; hPart++)
    {
        int hPartHeight = std::min(h11, (hPart + 1) * hPartHeightGlob) - hPart * hPartHeightGlob;

        // vector of one depthSimMap tile per Tc
        staticVector<staticVector<DepthSim>*>* dataMapsHPart =
            new staticVector<staticVector<DepthSim>*>(dataMaps->size());
        for(int i = 0; i < dataMaps->size(); i++) // iterate over Tc cameras
        {
            staticVector<DepthSim>* dataMapHPart = new staticVector<DepthSim>(w11 * hPartHeight);
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

        staticVector<DepthSim>* depthSimMapFusedHPart = new staticVector<DepthSim>(w11 * hPartHeight);
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

ps_depthSimMap* ps_refine_rc::optimizeDepthSimMapCUDA(ps_depthSimMap* depthPixSizeMapVis,
                                                      ps_depthSimMap* depthSimMapPhoto)
{
    int h11 = sp->mp->mip->getHeight(rc);

    staticVector<ps_depthSimMap*>* dataMaps = new staticVector<ps_depthSimMap*>(2);
    dataMaps->push_back(depthPixSizeMapVis); //!!DO NOT ERASE!!!
    dataMaps->push_back(depthSimMapPhoto);   //!!DO NOT ERASE!!!

    ps_depthSimMap* depthSimMapOptimized = new ps_depthSimMap(rc, sp->mp, 1, 1);

    {
        staticVector<staticVector<DepthSim>*>* dataMapsPtrs = new staticVector<staticVector<DepthSim>*>(dataMaps->size());
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

bool ps_refine_rc::refinercCUDA(bool checkIfExists)
{
    if(sp->mp->verbose)
        printf("processing refinercCUDA %i of %i\n", rc, sp->mp->ncams);

    // generate default depthSimMap if rc has no tcam
    if(tcams == nullptr || depths == nullptr)
    {
        ps_depthSimMap depthSimMapOpt(rc, sp->mp, 1, 1);
        depthSimMapOpt.save(rc, nullptr);
        return true;
    }

    if(checkIfExists && (FileExists(sp->getREFINE_opt_simMapFileName(rc, 1, 1))))
    {
        return false;
    }

    long tall = clock();

    ps_depthSimMap* depthPixSizeMapVis = getDepthPixSizeMapFromSGM();
    depthPixSizeMapVis->saveToPng(tmpDir + "refineRc_" + num2strFourDecimal(rc) + "Vis.png", 0.0f);

    ps_depthSimMap* depthSimMapPhoto = refineAndFuseDepthSimMapCUDA(depthPixSizeMapVis);
    depthSimMapPhoto->saveToPng(tmpDir + "refineRc_" + num2strFourDecimal(rc) + "Photo.png", 0.0f);

    ps_depthSimMap* depthSimMapOpt = nullptr;
    if(sp->doRefineRc)
    {
        depthSimMapOpt = optimizeDepthSimMapCUDA(depthPixSizeMapVis, depthSimMapPhoto);
    }
    else
    {
        depthSimMapOpt = new ps_depthSimMap(rc, sp->mp, 1, 1);
        depthSimMapOpt->add(depthSimMapPhoto);
    }

    depthSimMapOpt->saveToPng(tmpDir + "refineRc_" + num2strFourDecimal(rc) + "Opt.png", 0.0f);
    depthSimMapOpt->save(rc, tcams);

    depthSimMapPhoto->saveToBin(sp->getREFINE_photo_depthMapFileName(rc, 1, 1),
                                sp->getREFINE_photo_simMapFileName(rc, 1, 1));
    depthSimMapOpt->saveToBin(sp->getREFINE_opt_depthMapFileName(rc, 1, 1), sp->getREFINE_opt_simMapFileName(rc, 1, 1));

    printfElapsedTime(tall, "REFINERC " + num2str(rc) + " of " + num2str(sp->mp->ncams));

    if(sp->visualizeDepthMaps)
    {
        depthSimMapOpt->saveToWrl(tmpDir + "refineRc_" + num2strFourDecimal(rc) + "Opt.wrl", rc);
        depthSimMapPhoto->saveToWrl(tmpDir + "refineRc_" + num2strFourDecimal(rc) + "Photo.wrl", rc);
        depthPixSizeMapVis->saveToWrl(tmpDir + "refineRc_" + num2strFourDecimal(rc) + "Vis.wrl", rc);
    }

    delete depthPixSizeMapVis;
    delete depthSimMapPhoto;
    delete depthSimMapOpt;

    return true;
}

void refineDepthMaps(int CUDADeviceNo, multiviewParams* mp, mv_prematch_cams* pc, const staticVector<int>& cams)
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
    mv_images_cache* ic = new mv_images_cache(mp, bandType, true);
    cuda_plane_sweeping* cps = new cuda_plane_sweeping(CUDADeviceNo, ic, mp, pc, scale);
    ps_sgm_params* sp = new ps_sgm_params(mp, pc, cps);

    //////////////////////////////////////////////////////////////////////////////////////////

    for(const int rc : cams)
    {
        if(!FileExists(sp->getREFINE_opt_simMapFileName(rc, 1, 1)))
        {
            ps_refine_rc* rrc = new ps_refine_rc(rc, scale, step, sp);
            rrc->refinercCUDA();
            delete rrc;
        }
    }

    delete sp;
    delete ic;
    delete cps;
}

void refineDepthMaps(multiviewParams* mp, mv_prematch_cams* pc, const staticVector<int>& cams)
{
    int num_gpus = listCUDADevices(true);
    int num_cpu_threads = omp_get_num_procs();
    printf("GPU devicse %i, CPU threads %i\n", num_gpus, num_cpu_threads);
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
            printf("CPU thread %d (of %d) uses CUDA device %d\n", cpu_thread_id, numthreads, CUDADeviceNo);

            int rcFrom = CUDADeviceNo * (cams.size() / numthreads);
            int rcTo = (CUDADeviceNo + 1) * (cams.size() / numthreads);
            if(CUDADeviceNo == numthreads - 1)
            {
                rcTo = cams.size();
            }
            staticVector<int> subcams(cams.size());
            for(int rc = rcFrom; rc < rcTo; rc++)
            {
                subcams.push_back(cams[rc]);
            }
            refineDepthMaps(cpu_thread_id, mp, pc, subcams);
        }
    }
}
