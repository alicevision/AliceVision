// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <iostream>
#include <vector>

#include <cmath>

#include <aliceVision/CUDAInterfaces/common_gpu_cpu_structures.hpp>

#include <float.h>

void copyResultToTheHost(Cuda::HostMemoryHeap<float3, 2>* opts_hmh, Cuda::HostMemoryHeap<float3, 2>* onms_hmh,
                         Cuda::HostMemoryHeap<float, 2>* osim_hmh, Cuda::HostMemoryHeap<float3, 2>* oles_hmh,
                         Cuda::DeviceMemoryPitched<float, 3>& pts_dmp, Cuda::DeviceMemoryPitched<float, 3>& nms_dmp,
                         Cuda::DeviceMemoryPitched<float, 2>& sim_dmp, Cuda::DeviceMemoryPitched<float, 2>& les_dmp,
                         int ptsAtGrid, int nbest, int t, int npts)
{
    Cuda::HostMemoryHeap<float, 3> pts_hmh(pts_dmp);
    Cuda::HostMemoryHeap<float, 3> nms_hmh(nms_dmp);
    Cuda::HostMemoryHeap<float, 2> sim_hmh(sim_dmp);
    Cuda::HostMemoryHeap<float, 2> les_hmh(les_dmp);

    for(int i = 0; i < nbest; i++)
    {
        for(int p = 0; p < ptsAtGrid; p++)
        {

            if(ptsAtGrid * t + p < npts)
            {
                if(i == 0)
                {
                    float3 le;
                    le.x = les_hmh.getBuffer()[les_hmh.stride[0] * 0 + p];
                    le.y = les_hmh.getBuffer()[les_hmh.stride[0] * 1 + p];
                    le.z = les_hmh.getBuffer()[les_hmh.stride[0] * 2 + p];
                    oles_hmh->getBuffer()[ptsAtGrid * t + p] = le;
                }

                float3 pt;
                pt.x = pts_hmh.getBuffer()[pts_hmh.stride[0] * pts_hmh.size[1] * 0 + pts_hmh.stride[0] * i + p];
                pt.y = pts_hmh.getBuffer()[pts_hmh.stride[0] * pts_hmh.size[1] * 1 + pts_hmh.stride[0] * i + p];
                pt.z = pts_hmh.getBuffer()[pts_hmh.stride[0] * pts_hmh.size[1] * 2 + pts_hmh.stride[0] * i + p];
                opts_hmh->getBuffer()[opts_hmh->stride[0] * i + ptsAtGrid * t + p] = pt;
                // printf("%f %f %f\n",pt.x,pt.y,pt.z);

                float3 nm;
                nm.x = nms_hmh.getBuffer()[nms_hmh.stride[0] * nms_hmh.size[1] * 0 + nms_hmh.stride[0] * i + p];
                nm.y = nms_hmh.getBuffer()[nms_hmh.stride[0] * nms_hmh.size[1] * 1 + nms_hmh.stride[0] * i + p];
                nm.z = nms_hmh.getBuffer()[nms_hmh.stride[0] * nms_hmh.size[1] * 2 + nms_hmh.stride[0] * i + p];
                onms_hmh->getBuffer()[onms_hmh->stride[0] * i + ptsAtGrid * t + p] = nm;

                float sim;
                sim = sim_hmh.getBuffer()[sim_hmh.stride[0] * i + p];
                osim_hmh->getBuffer()[osim_hmh->stride[0] * i + ptsAtGrid * t + p] = sim;
            }
        }
    }
}

int divUp(unsigned int a, unsigned int b)
{
    return (a % b != 0) ? (a / b + 1) : (a / b);
}

//-----------------------------------------------------------------------------
// forward declarations of CUDA functions
extern void planeSweepingGPU(Cuda::DeviceMemoryPitched<float, 2>& was_dmp, Cuda::DeviceMemoryPitched<float, 3>& pts_dmp,
                             Cuda::DeviceMemoryPitched<float, 3>& nms_dmp, Cuda::DeviceMemoryPitched<float, 2>& sim_dmp,
                             Cuda::DeviceMemoryPitched<float, 2>& les_dmp, Cuda::HostMemoryHeap<float3, 2>& pts_hmh,
                             Cuda::HostMemoryHeap<float3, 2>& nms_hmh, Cuda::HostMemoryHeap<float3, 2>& xas_hmh,
                             Cuda::HostMemoryHeap<float3, 2>& yas_hmh, Cuda::HostMemoryHeap<float, 2>& psz_hmh,
                             Cuda::HostMemoryHeap<float2, 2>** shs_hmh, int ncams, cameraStruct* cams, int nRotHx,
                             int stepRotx, int nRotHy, int stepRoty, int nPosH, int stepPos, int2 rotMapsGrid,
                             int2 workArea, int npts, int nrotPixs, int t, int ptsAtGrid, int nonMaxKernelSizex,
                             int nonMaxKernelSizey, int nbest, int nsteps, bool use_wsh_map, int patch_scale);

extern void deviceAllocate(int ncams, cameraStruct* cams, int2* rotpts, int nrotpts, int2 workArea,
                           Cuda::HostMemoryHeap<unsigned char, 2>& wsh_hmh, int width, int height);
extern void deviceDeallocate(int ncams);
void getWshMap(Cuda::DeviceMemoryPitched<unsigned char, 2>& wshMap_dmp);

//-----------------------------------------------------------------------------
int main(int argc, char* argv[])
{

    if((argc == 1) || (((std::string)argv[1]) == "-help") || (((std::string)argv[1]) == "help"))
    {
        printf("Author: Michal Jancosek\n");
        printf("usage: patch_es_cuda.exe inputFileNameCameras inputFileNamePts outputFileName showResBool\n");
        printf("showResBool: 0-false, 1-true\n");
        return (0);
    }
    std::string inputFileNameCameras = ((std::string)argv[1]).c_str();
    std::string inputFileNamePts = ((std::string)argv[2]).c_str();
    std::string outputFileName = ((std::string)argv[3]).c_str();
    bool show_res = atoi(argv[4]);

    bool printf_info = false;

    // show_res = false;

    int nRotHx = atoi(argv[5]);
    int stepRotx = atoi(argv[6]);
    int nRotHy = atoi(argv[7]);
    int stepRoty = atoi(argv[8]);
    int nPosH = atoi(argv[9]);
    int stepPos = atoi(argv[10]);
    int nonMaxKernelSizex = atoi(argv[11]);
    int nonMaxKernelSizey = atoi(argv[12]);
    int nbest = atoi(argv[13]);
    bool use_wsh_map = atoi(argv[14]);
    int patch_scale = atoi(argv[15]);

    /*
    int nRotH = 20;
    int stepRot = 3;
    int nPosH = 0;
    int stepPos = 1;
    */

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // read the input
    FILE* f = fopen(inputFileNameCameras.c_str(), "rb");
    if(f == NULL)
    {
        exit(1);
    }

    int width, height, ntarcams;
    fread(&width, sizeof(int), 1, f);
    fread(&height, sizeof(int), 1, f);
    fread(&ntarcams, sizeof(int), 1, f);

    // read wshmap
    unsigned char* tex = new unsigned char[width * height];
    fread(tex, sizeof(unsigned char), width * height, f);
    transformImage(tex, width, height);

    Cuda::HostMemoryHeap<unsigned char, 2> wsh_hmh(Cuda::Size<2>(width, height));
    for(int y = 0; y < height; y++)
    {
        for(int x = 0; x < width; x++)
        {
            wsh_hmh.getBuffer()[y * width + x] = tex[y * width + x];
        }
    }
    delete[] tex;

    cameraStruct* cams = new cameraStruct[ntarcams + 1];
    readCamera(&cams[0], f, width, height);

    for(int c = 0; c < ntarcams; c++)
    {
        readCamera(&cams[c + 1], f, width, height);
    }

    // pixels of patch
    int nrotpts;
    fread(&nrotpts, sizeof(int), 1, f);
    int2* rotpts = new int2[nrotpts];
    fread(rotpts, sizeof(int2), nrotpts, f);
    int nrotPixs = min(nrotpts, MAX_PATCH_PIXELS);

    fclose(f);

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // load pts to process
    f = fopen(inputFileNamePts.c_str(), "rb");

    int npts;
    fread(&npts, sizeof(int), 1, f);

    Cuda::HostMemoryHeap<float3, 2> pts_hmh(Cuda::Size<2>(npts, 1));
    Cuda::HostMemoryHeap<float3, 2> xas_hmh(Cuda::Size<2>(npts, 1));
    Cuda::HostMemoryHeap<float3, 2> yas_hmh(Cuda::Size<2>(npts, 1));
    Cuda::HostMemoryHeap<float3, 2> nms_hmh(Cuda::Size<2>(npts, 1));
    Cuda::HostMemoryHeap<float, 2> psz_hmh(Cuda::Size<2>(npts, 1));
    Cuda::HostMemoryHeap<float2, 2>** shs_hmh = new Cuda::HostMemoryHeap<float2, 2>*[ntarcams + 1];
    for(int c = 0; c < ntarcams + 1; c++)
    {
        shs_hmh[c] = new Cuda::HostMemoryHeap<float2, 2>(Cuda::Size<2>(npts, 1));
    }

    fread(pts_hmh.getBuffer(), sizeof(float3), npts, f);
    fread(xas_hmh.getBuffer(), sizeof(float3), npts, f);
    fread(yas_hmh.getBuffer(), sizeof(float3), npts, f);
    fread(nms_hmh.getBuffer(), sizeof(float3), npts, f);
    fread(psz_hmh.getBuffer(), sizeof(float), npts, f);
    for(int c = 0; c < ntarcams + 1; c++)
    {
        fread(shs_hmh[c]->getBuffer(), sizeof(float2), npts, f);
    }

    fclose(f);

    int nRotx = 2 * nRotHx + 1;
    int nRoty = 2 * nRotHy + 1;
    int nPos = 2 * nPosH + 1;

    int2 rotMapsGrid;
    rotMapsGrid.x = 800 / nRotx;
    rotMapsGrid.y = 800 / nRoty;
    int ptsAtGrid = min((rotMapsGrid.x * rotMapsGrid.y) / nPos, MAX_PTS);
    rotMapsGrid.y = divUp(ptsAtGrid * nPos, rotMapsGrid.x);

    if(printf_info)
    {
        printf("npts: %i \n", npts);
        printf("pts at time: %i \n", ptsAtGrid);
    }

    int2 workArea;
    workArea.x = nRotx * rotMapsGrid.x;
    workArea.y = nRoty * rotMapsGrid.y;

    if(printf_info)
    {
        printf("workarea: %i %i\n", workArea.x, workArea.y);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // allocate global on the device
    deviceAllocate(ntarcams + 1, cams, rotpts, nrotpts, workArea, wsh_hmh, width, height);

    if(show_res == true)
    {

        ///////////////////////////////////////////////////////////////////////////////
        // allocate memory wsh map at device
        Cuda::DeviceMemoryPitched<unsigned char, 2> wshMap_dmp(Cuda::Size<2>(width, height));

        getWshMap(wshMap_dmp);

        showImage(wshMap_dmp, "wsh image");
        while(1)
        {
            char key = cvWaitKey(10);
            if(key == 27)
                break;
        }
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // process
    Cuda::HostMemoryHeap<float3, 2> opts_hmh(Cuda::Size<2>(npts, nbest));
    Cuda::HostMemoryHeap<float3, 2> onms_hmh(Cuda::Size<2>(npts, nbest));
    Cuda::HostMemoryHeap<float, 2> osim_hmh(Cuda::Size<2>(npts, nbest));
    Cuda::HostMemoryHeap<float3, 2> oles_hmh(Cuda::Size<2>(npts, 1));

    Cuda::DeviceMemoryPitched<float, 2> was_dmp(Cuda::Size<2>(workArea.x, workArea.y));

    Cuda::DeviceMemoryPitched<float, 3> pts_dmp(Cuda::Size<3>(ptsAtGrid, nbest, 3));
    Cuda::DeviceMemoryPitched<float, 3> nms_dmp(Cuda::Size<3>(ptsAtGrid, nbest, 3));
    Cuda::DeviceMemoryPitched<float, 2> sim_dmp(Cuda::Size<2>(ptsAtGrid, nbest));
    Cuda::DeviceMemoryPitched<float, 2> les_dmp(Cuda::Size<2>(ptsAtGrid, 3));

    int nsteps = divUp(npts, ptsAtGrid);

    double time = getTime();
    for(int t = 0; t < nsteps; t++)
    {
        if(printf_info)
        {
            printf("%i of %i \n", t, nsteps - 1);
        }

        planeSweepingGPU(was_dmp, pts_dmp, nms_dmp, sim_dmp, les_dmp, pts_hmh, nms_hmh, xas_hmh, yas_hmh, psz_hmh,
                         shs_hmh, ntarcams + 1, cams, nRotHx, stepRotx, nRotHy, stepRoty, nPosH, stepPos, rotMapsGrid,
                         workArea, npts, nrotPixs, t, ptsAtGrid, nonMaxKernelSizex, nonMaxKernelSizey, nbest, nsteps,
                         use_wsh_map, patch_scale);

        // copy to the host memory
        copyResultToTheHost(&opts_hmh, &onms_hmh, &osim_hmh, &oles_hmh, pts_dmp, nms_dmp, sim_dmp, les_dmp, ptsAtGrid,
                            nbest, t, npts);

        if(show_res == true)
        {
            showImage(was_dmp, "sim image", 2 * nRotHx + 1, 2 * nRotHy + 1);
            while(1)
            {
                char key = cvWaitKey(10);
                if(key == 27)
                    break;
            }
        }
    }
    time = getTime() - time;

    if(printf_info)
    {
        std::cout << " - processing took " << time << " ms using GPU" << std::endl;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // write the output
    f = fopen(outputFileName.c_str(), "wb");
    fwrite(&npts, sizeof(int), 1, f);
    fwrite(&nbest, sizeof(int), 1, f);
    fwrite(opts_hmh.getBuffer(), sizeof(float3), npts * nbest, f);
    fwrite(onms_hmh.getBuffer(), sizeof(float3), npts * nbest, f);
    fwrite(osim_hmh.getBuffer(), sizeof(float), npts * nbest, f);
    fwrite(oles_hmh.getBuffer(), sizeof(float3), npts, f);
    fclose(f);

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // deallocate
    for(int c = 0; c < ntarcams + 1; c++)
    {
        delete cams[c].tex_hmh;
    }
    delete[] cams;

    if((show_res == true) || (printf_info))
    {
        std::cout << " - press ESC to continue" << std::endl;
        while(1)
        {
            char key = cvWaitKey(10);
            if(key == 27)
                break;
        }
    }

    delete[] rotpts;

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // allocate global on the device
    deviceDeallocate(ntarcams + 1);

    return 0;
}
