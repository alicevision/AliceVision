// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

namespace aliceVision {
namespace depthMap {

__global__ void refine_selectPartOfDepthMapNearFPPlaneDepth_kernel(float* o0depthMap, int o0depthMap_p,
                                                                   float* o1depthMap, int o1depthMap_p,
                                                                   float* idepthMap, int idepthMap_p, int width,
                                                                   int height, float fpPlaneDepth,
                                                                   float fpPlaneDepthNext)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    int2 pix;
    pix.x = x;
    pix.y = y;

    if((x < width) && (y < height))
    {
        float idepth = idepthMap[y * idepthMap_p + x];
        float o0depth = -1.0f;
        float o1depth = -1.0f;

        if(idepth > 0.0f)
        {
            float step = (fpPlaneDepthNext - fpPlaneDepth) / 2.0f;
            float3 p = get3DPointForPixelAndDepthFromRC(pix, idepth);
            float fpPlaneDepthPix = frontoParellePlaneRCDepthFor3DPoint(p);
            if((fpPlaneDepthPix >= fpPlaneDepth - step) && (fpPlaneDepthPix < fpPlaneDepthNext))
            {
                o0depth = size(get3DPointForPixelAndFrontoParellePlaneRC(pix, fpPlaneDepth) - sg_s_rC);
                o1depth = size(get3DPointForPixelAndFrontoParellePlaneRC(pix, fpPlaneDepthNext) - sg_s_rC);
            };
        };

        o0depthMap[y * o0depthMap_p + x] = o0depth;
        o1depthMap[y * o1depthMap_p + x] = o1depth;
    };
}

__global__ void refine_dilateDepthMap_kernel(float* depthMap, int depthMap_p, int width, int height, const float gammaC)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if((x < width) && (y < height))
    {
        float depth = tex2D(depthsTex, x, y);
//        float maxw = -1.0f;
        if(depth < 0.0f)
        {
            // float4 rcm = uchar4_to_float4(tex2D(rTexU4, x, y));
            for(int yp = -1; yp <= +1; yp++)
            {
                for(int xp = -1; xp <= +1; xp++)
                {
                    float depthn = tex2D(depthsTex, x + xp, y + yp);
                    if(depthn > 0.0f)
                    {
                        // float4 rc = uchar4_to_float4(tex2D(rTexU4, x+xp, y+yp));
                        // float w = CostYKfromLab(rcm, rc, gammaC);
                        // depth = (w>maxw?depthn:depth);
                        // maxw = fmaxf(w,maxw);
                        depth = depthn;
                    };
                };
            };
        };
        depthMap[y * depthMap_p + x] = depth;
    };
}

__global__ void refine_dilateFPPlaneDepthMapXpYp_kernel(float* fpPlaneDepthMap, int fpPlaneDepthMap_p, float* maskMap,
                                                        int maskMap_p, int width, int height, int xp, int yp,
                                                        float fpPlaneDepth)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if((x + xp >= 0) && (y + yp >= 0) && (x + xp < width) && (y + yp < height) && (x < width) && (y < height))
    {
        float depth = maskMap[y * maskMap_p + x];
        if(depth > 0.0f)
        {
            fpPlaneDepthMap[(y + yp) * fpPlaneDepthMap_p + (x + xp)] = fpPlaneDepth;
        };
    };
}

__global__ void refine_convertFPPlaneDepthMapToDepthMap_kernel(float* depthMap, int depthMap_p, float* fpPlaneDepthMap,
                                                               int fpPlaneDepthMap_p, int width, int height)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    int2 pix;
    pix.x = x;
    pix.y = y;

    if((x < width) && (y < height))
    {
        float fpPlaneDepth = fpPlaneDepthMap[y * fpPlaneDepthMap_p + x];
        float depth = -1.0f;
        if(fpPlaneDepth > 0.0f)
        {
            depth = size(get3DPointForPixelAndFrontoParellePlaneRC(pix, fpPlaneDepth) - sg_s_rC);
        };
        depthMap[y * depthMap_p + x] = depth;
    };
}

__global__ void refine_computeDepthsMapFromDepthMap_kernel(float3* depthsMap, int depthsMap_p, float* depthMap,
                                                           int depthMap_p, int width, int height, bool moveByTcOrRc,
                                                           float step)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    int2 pix;
    pix.x = x;
    pix.y = y;

    if((x < width) && (y < height))
    {
        float depth = depthMap[y * depthMap_p + x];
        if(depth > 0.0f)
        {
            float3 p = get3DPointForPixelAndDepthFromRC(pix, depth);
            float3 pm1 = p;
            float3 pp1 = p;

            if(moveByTcOrRc == true)
            {
                move3DPointByTcPixStep(pm1, -step);
                move3DPointByTcPixStep(pp1, +step);
            }
            else
            {
                float pixSize = step * computePixSize(p);
                move3DPointByRcPixSize(pm1, -pixSize);
                move3DPointByRcPixSize(pp1, +pixSize);
            };

            depthsMap[y * depthsMap_p + x].x = size(pm1 - sg_s_rC);
            depthsMap[y * depthsMap_p + x].y = size(p - sg_s_rC);
            depthsMap[y * depthsMap_p + x].z = size(pp1 - sg_s_rC);
        };
    };
}

__global__ void refine_reprojTarTexLABByDepthsMap_kernel(float3* depthsMap, int depthsMap_p, uchar4* tex, int tex_p,
                                                         int width, int height, int id)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    int2 pix;
    pix.x = x;
    pix.y = y;

    if((x < width) && (y < height))
    {
        float depth = 0.0f;
        if(id == 0)
        {
            depth = depthsMap[y * depthsMap_p + x].x;
        };
        if(id == 1)
        {
            depth = depthsMap[y * depthsMap_p + x].y;
        };
        if(id == 2)
        {
            depth = depthsMap[y * depthsMap_p + x].z;
        };

        if(depth > 0.0f)
        {
            float3 p = get3DPointForPixelAndDepthFromRC(pix, depth);
            float2 tpc = project3DPoint(sg_s_tP, p);

            if(((tpc.x + 0.5f) > 0.0f) && ((tpc.y + 0.5f) > 0.0f) && ((tpc.x + 0.5f) < (float)width - 1.0f) &&
               ((tpc.y + 0.5f) < (float)height - 1.0f))
            {
                tex[y * tex_p + x] = float4_to_uchar4(255.0f * tex2D(t4tex, (float)tpc.x + 0.5f, (float)tpc.y + 0.5f));
            }
            else
            {
                tex[y * tex_p + x].x = 0;
                tex[y * tex_p + x].y = 0;
                tex[y * tex_p + x].z = 0;
                tex[y * tex_p + x].w = 0;
            };
        };
    };
}

__global__ void refine_reprojTarTexLABByDepthMap_kernel(float* depthMap, int depthMap_p, uchar4* tex, int tex_p,
                                                        int width, int height)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    int2 pix;
    pix.x = x;
    pix.y = y;

    if((x < width) && (y < height))
    {
        float depth = depthMap[y * depthMap_p + x];
        uchar4 ocol = make_uchar4(0, 0, 0, 0);

        if(depth > 0.0f)
        {
            float3 p = get3DPointForPixelAndDepthFromRC(pix, depth);
            float2 tpc = project3DPoint(sg_s_tP, p);
            if(((tpc.x + 0.5f) > 0.0f) && ((tpc.y + 0.5f) > 0.0f) && ((tpc.x + 0.5f) < (float)width - 1.0f) &&
               ((tpc.y + 0.5f) < (float)height - 1.0f))
            {
                ocol = float4_to_uchar4(255.0f * tex2D(t4tex, (float)tpc.x + 0.5f, (float)tpc.y + 0.5f));
            };
        };

        tex[y * tex_p + x] = ocol;
    };
}

__global__ void refine_reprojTarTexLABByDepthMapMovedByStep_kernel(float* depthMap, int depthMap_p, uchar4* tex,
                                                                   int tex_p, int width, int height, bool moveByTcOrRc,
                                                                   float step)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    int2 pix;
    pix.x = x;
    pix.y = y;

    if((x < width) && (y < height))
    {
        float depth = depthMap[y * depthMap_p + x];
        uchar4 ocol = make_uchar4(0, 0, 0, 0);

        if(depth > 0.0f)
        {
            float3 p = get3DPointForPixelAndDepthFromRC(pix, depth);

            if(step != 0.0f)
            {
                if(moveByTcOrRc == true)
                {
                    move3DPointByTcPixStep(p, step);
                }
                else
                {
                    float pixSize = step * computePixSize(p);
                    move3DPointByRcPixSize(p, pixSize);
                };

                depth = size(p - sg_s_rC);
            };

            float2 tpc = project3DPoint(sg_s_tP, p);
            if(((tpc.x + 0.5f) > 0.0f) && ((tpc.y + 0.5f) > 0.0f) && ((tpc.x + 0.5f) < (float)width - 1.0f) &&
               ((tpc.y + 0.5f) < (float)height - 1.0f))
            {
                ocol = float4_to_uchar4(255.0f * tex2D(t4tex, (float)tpc.x + 0.5f, (float)tpc.y + 0.5f));
            };
        };

        depthMap[y * depthMap_p + x] = depth;
        tex[y * tex_p + x] = ocol;
    };
}

__global__ void refine_compYKNCCSimMap_kernel(float* osimMap, int osimMap_p, float* depthMap, int depthMap_p, int width,
                                              int height, int wsh, const float gammaC, const float gammaP)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if((x > wsh) && (y > wsh) && (x < width - wsh) && (y < height - wsh))
    {
        float depth = depthMap[y * depthMap_p + x];
        float osim = 1.0f;

        if(depth > 0.0f)
        {
            simStat sst = simStat();
            float4 rcm = uchar4_to_float4(tex2D(rTexU4, x, y));
            float4 tcm = uchar4_to_float4(tex2D(tTexU4, x, y));
            for(int yp = -wsh; yp <= +wsh; yp++)
            {
                for(int xp = -wsh; xp <= +wsh; xp++)
                {
                    float4 rc = uchar4_to_float4(tex2D(rTexU4, x + xp, y + yp));
                    float4 tc = uchar4_to_float4(tex2D(tTexU4, x + xp, y + yp));
                    float w =
                        CostYKfromLab(xp, yp, rcm, rc, gammaC, gammaP) * CostYKfromLab(xp, yp, tcm, tc, gammaC, gammaP);
                    sst.update((float)rc.x, (float)tc.x, w);
                };
            };
            sst.computeWSim();
            osim = sst.sim;
        };

        osimMap[y * osimMap_p + x] = osim;
    };
}

__global__ void refine_compYKNCCSim_kernel(float3* osims, int osims_p, int id, float* depthMap, int depthMap_p,
                                           int width, int height, int wsh, const float gammaC, const float gammaP)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if((x > wsh) && (y > wsh) && (x < width - wsh) && (y < height - wsh))
    {
        float depth = depthMap[y * depthMap_p + x];
        float3 osim = make_float3(1.0f, 1.0f, 1.0f);
        if(id > 0)
        {
            osim = osims[y * osims_p + x];
        };

        if(depth > 0.0f)
        {
            simStat sst = simStat();
            float4 rcm = uchar4_to_float4(tex2D(rTexU4, x, y));
            float4 tcm = uchar4_to_float4(tex2D(tTexU4, x, y));
            for(int yp = -wsh; yp <= +wsh; yp++)
            {
                for(int xp = -wsh; xp <= +wsh; xp++)
                {
                    float4 rc = uchar4_to_float4(tex2D(rTexU4, x + xp, y + yp));
                    float4 tc = uchar4_to_float4(tex2D(tTexU4, x + xp, y + yp));
                    float w =
                        CostYKfromLab(xp, yp, rcm, rc, gammaC, gammaP) * CostYKfromLab(xp, yp, tcm, tc, gammaC, gammaP);
                    sst.update((float)rc.x, (float)tc.x, w);
                };
            };
            sst.computeWSim();
            if(id == 0)
            {
                osim.x = sst.sim;
            };
            if(id == 1)
            {
                osim.y = sst.sim;
            };
            if(id == 2)
            {
                osim.z = sst.sim;
            };
        };

        osims[y * osims_p + x] = osim;
    };
}

__global__ void refine_compYKNCCSimOptGammaC_kernel(float3* osims, int osims_p, int id, float* depthMap, int depthMap_p,
                                                    int width, int height, int wsh, const float gammaP)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if((x > wsh) && (y > wsh) && (x < width - wsh) && (y < height - wsh))
    {
        float depth = depthMap[y * depthMap_p + x];
        float3 osim = make_float3(1.0f, 1.0f, 1.0f);
        if(id > 0)
        {
            osim = osims[y * osims_p + x];
        };

        if(depth > 0.0f)
        {
            float gammaC[4];
            gammaC[0] = 4.0f;
            gammaC[1] = 8.0f;
            gammaC[2] = 15.0f;
            gammaC[3] = 25.0f;
            simStat sst[4];
            for(int k = 0; k < 4; k++)
            {
                sst[k] = simStat();
            };
            float4 rcm = uchar4_to_float4(tex2D(rTexU4, x, y));
            float4 tcm = uchar4_to_float4(tex2D(tTexU4, x, y));
            for(int yp = -wsh; yp <= +wsh; yp++)
            {
                for(int xp = -wsh; xp <= +wsh; xp++)
                {
                    float4 rc = uchar4_to_float4(tex2D(rTexU4, x + xp, y + yp));
                    float4 tc = uchar4_to_float4(tex2D(tTexU4, x + xp, y + yp));

                    for(int k = 0; k < 4; k++)
                    {
                        float w = CostYKfromLab(xp, yp, rcm, rc, gammaC[k], gammaP) *
                                  CostYKfromLab(xp, yp, tcm, tc, gammaC[k], gammaP);
                        sst[k].update((float)rc.x, (float)tc.x, w);
                    };
                };
            };

            float sim = 1.0f;
            for(int k = 0; k < 4; k++)
            {
                sst[k].computeWSim();
                sim = fminf(sim, sst[k].sim);
            };

            if(id == 0)
            {
                osim.x = sim;
            };
            if(id == 1)
            {
                osim.y = sim;
            };
            if(id == 2)
            {
                osim.z = sim;
            };
        };

        osims[y * osims_p + x] = osim;
    };
}

__global__ void refine_computeBestDepthSimMaps_kernel(float* osim, int osim_p, float* odpt, int odpt_p, float3* isims,
                                                      int isims_p, float3* idpts, int idpts_p, int width, int height,
                                                      float simThr)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if((x < width) && (y < height))
    {
        float3 depths = idpts[y * idpts_p + x];
        float3 sims = isims[y * isims_p + x];

        float outDepth = ((sims.x < sims.y) ? depths.x : depths.y);
        float outSim = ((sims.x < sims.y) ? sims.x : sims.y);
        outDepth = ((sims.z < outSim) ? depths.z : outDepth);
        outSim = ((sims.z < outSim) ? sims.z : outSim);

        float refinedDepth = refineDepthSubPixel(depths, sims);
        if(refinedDepth > 0.0f)
        {
            outDepth = refinedDepth;
        };

        osim[y * osim_p + x] = (outSim < simThr ? outSim : 1.0f);
        odpt[y * odpt_p + x] = (outSim < simThr ? outDepth : -1.0f);
    };
}

__global__ void refine_fuseThreeDepthSimMaps_kernel(float* osim, int osim_p, float* odpt, int odpt_p, float* isimLst,
                                                    int isimLst_p, float* idptLst, int idptLst_p, float* isimAct,
                                                    int isimAct_p, float* idptAct, int idptAct_p, int width, int height,
                                                    float simThr)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if((x < width) && (y < height))
    {
        float dpts[3];
        float sims[3];

        dpts[0] = odpt[y * odpt_p + x];
        dpts[1] = idptLst[y * idptLst_p + x];
        dpts[2] = idptAct[y * idptAct_p + x];

        sims[0] = osim[y * odpt_p + x];
        sims[1] = isimLst[y * isimLst_p + x];
        sims[2] = isimAct[y * isimAct_p + x];

        float outDpt = dpts[0];
        float outSim = simThr;
        outDpt = ((sims[1] < outSim) ? dpts[1] : outDpt);
        outSim = ((sims[1] < outSim) ? sims[1] : outSim);
        outDpt = ((sims[2] < outSim) ? dpts[2] : outDpt);
        outSim = ((sims[2] < outSim) ? sims[2] : outSim);

        // osim[y*osim_p+x]=(outSim<simThr?outSim:1.0f);
        // odpt[y*odpt_p+x]=(outSim<simThr?outDpt:-1.0f);
        osim[y * osim_p + x] = outSim;
        odpt[y * odpt_p + x] = outDpt;
    };
}

__global__ void refine_compUpdateYKNCCSimMapPatch_kernel(float* osimMap, int osimMap_p, float* odptMap, int odptMap_p,
                                                         float* depthMap, int depthMap_p, int width, int height,
                                                         int wsh, const float gammaC, const float gammaP,
                                                         const float epipShift, const float tcStep, int id,
                                                         bool moveByTcOrRc, int xFrom, int imWidth, int imHeight)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    int2 pix;
    pix.x = x + xFrom;
    pix.y = y;

    // if ((pix.x>wsh)&&(pix.y>wsh)&&(pix.x<width-wsh)&&(pix.y<height-wsh))
    if((x >= 0) && (y >= 0) && (x < width) && (y < height))
    {
        float odpt = *get2DBufferAt(depthMap, depthMap_p, x, y);
        float osim = 1.0f;

        // If we have an initial depth value, we can refine it
        if(odpt > 0.0f)
        {
            float3 p = get3DPointForPixelAndDepthFromRC(pix, odpt);
            // move3DPointByTcPixStep(p, tcStep);
            move3DPointByTcOrRcPixStep(pix, p, tcStep, moveByTcOrRc);

            odpt = size(p - sg_s_rC);

            patch ptch;
            ptch.p = p;
            ptch.d = computePixSize(p);
            // TODO: we could compute the orientation of the path from the input depth map instead of relying on the cameras orientations
            computeRotCSEpip(ptch, p);
            osim = compNCCby3DptsYK(ptch, wsh, imWidth, imHeight, gammaC, gammaP, epipShift);
        }

        float* osim_ptr = get2DBufferAt(osimMap, osimMap_p, x, y);
        float* odpt_ptr = get2DBufferAt(odptMap, odptMap_p, x, y);
        if(id == 0)
        {
            // For the first iteration, we initialize the values
            *osim_ptr = osim;
            *odpt_ptr = odpt;
        }
        else
        {
            // Then we update the similarity value if it's better
            float actsim = *osim_ptr;
            if(osim < actsim)
            {
                *osim_ptr = osim;
                *odpt_ptr = odpt;
            }
        }
    }
}

__global__ void refine_coputeDepthStepMap_kernel(float* depthStepMap, int depthStepMap_p, float* depthMap,
                                                 int depthMap_p, int width, int height, bool moveByTcOrRc)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    int2 pix;
    pix.x = x;
    pix.y = y;

    if((x < width) && (y < height))
    {
        float depth = depthMap[y * depthMap_p + x];
        float depthStep = 0.0f;

        if(depth > 0.0f)
        {
            float3 p = get3DPointForPixelAndDepthFromRC(pix, depth);
            float3 p1 = p;
            move3DPointByTcOrRcPixStep(pix, p1, 1.0f, moveByTcOrRc);
            depthStep = size(p - p1);
        };
        depthStepMap[y * depthStepMap_p + x] = depthStep;
    };
}

__global__ void refine_compYKNCCDepthSimMapPatch_kernel(float2* oDepthSimMap, int oDepthSimMap_p, float* depthMap,
                                                        int depthMap_p, int width, int height, int wsh,
                                                        const float gammaC, const float gammaP, const float epipShift,
                                                        const float tcStep, bool moveByTcOrRc)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    int2 pix;
    pix.x = x;
    pix.y = y;

    if((x > wsh) && (y > wsh) && (x < width - wsh) && (y < height - wsh))
    {
        float depth = depthMap[y * depthMap_p + x];
        float2 oDepthSim = make_float2(-1.0f, 1.0f);

        if(depth > 0.0f)
        {
            float3 p = get3DPointForPixelAndDepthFromRC(pix, depth);
            // move3DPointByTcPixStep(p, tcStep);
            move3DPointByTcOrRcPixStep(pix, p, tcStep, moveByTcOrRc);

            patch ptch;
            ptch.p = p;
            ptch.d = computePixSize(p);
            computeRotCSEpip(ptch, p);

            oDepthSim.x = size(sg_s_rC - ptch.p);
            oDepthSim.y = compNCCby3DptsYK(ptch, wsh, width, height, gammaC, gammaP, epipShift);
        };

        oDepthSimMap[y * oDepthSimMap_p + x] = oDepthSim;
    };
}

__global__ void refine_compYKNCCSimMapPatch_kernel(float* osimMap, int osimMap_p, float* depthMap, int depthMap_p,
                                                   int width, int height, int wsh, const float gammaC,
                                                   const float gammaP, const float epipShift, const float tcStep,
                                                   bool moveByTcOrRc, int xFrom, int imWidth, int imHeight)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    int2 pix;
    pix.x = x + xFrom;
    pix.y = y;

    // if ((x>wsh)&&(y>wsh)&&(x<width-wsh)&&(y<height-wsh))
    if((x >= 0) && (y >= 0) && (x < width) && (y < height))
    {
        float depth = *get2DBufferAt(depthMap, depthMap_p, x, y);
        float osim = 1.1f;

        if(depth > 0.0f)
        {
            float3 p = get3DPointForPixelAndDepthFromRC(pix, depth);
            // move3DPointByTcPixStep(p, tcStep);
            move3DPointByTcOrRcPixStep(pix, p, tcStep, moveByTcOrRc);

            patch ptch;
            ptch.p = p;
            ptch.d = computePixSize(p);
            computeRotCSEpip(ptch, p);
            osim = compNCCby3DptsYK(ptch, wsh, imWidth, imHeight, gammaC, gammaP, epipShift);
        };
        *get2DBufferAt(osimMap, osimMap_p, x, y) = osim;
    };
}

__global__ void refine_compYKNCCSimMapPatchDMS_kernel(float* osimMap, int osimMap_p, float* depthMap, int depthMap_p,
                                                      int width, int height, int wsh, const float gammaC,
                                                      const float gammaP, const float epipShift,
                                                      const float depthMapShift)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    int2 pix;
    pix.x = x;
    pix.y = y;

    if((x > wsh) && (y > wsh) && (x < width - wsh) && (y < height - wsh))
    {
        float depth = depthMap[y * depthMap_p + x];
        float osim = 1.1f;

        if(depth > 0.0f)
        {
            float3 p = get3DPointForPixelAndDepthFromRC(pix, depth + depthMapShift);

            patch ptch;
            ptch.p = p;
            ptch.d = computePixSize(p);
            computeRotCSEpip(ptch, p);
            osim = compNCCby3DptsYK(ptch, wsh, width, height, gammaC, gammaP, epipShift);
        };

        osimMap[y * osimMap_p + x] = osim;
    };
}

__global__ void refine_setLastThreeSimsMap_kernel(float3* lastThreeSimsMap, int lastThreeSimsMap_p, float* simMap,
                                                  int simMap_p, int width, int height, int id)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if((x < width) && (y < height))
    {
        float sim = *get2DBufferAt(simMap, simMap_p, x, y);
        float3* lastThreeSims_ptr = get2DBufferAt(lastThreeSimsMap, lastThreeSimsMap_p, x, y);

        if(id == 0)
        {
            lastThreeSims_ptr->x = sim;
        }
        if(id == 1)
        {
            lastThreeSims_ptr->y = sim;
        }
        if(id == 2)
        {
            lastThreeSims_ptr->z = sim;
        }
    }
}

__global__ void refine_computeDepthSimMapFromLastThreeSimsMap_kernel(float* osimMap, int osimMap_p, float* iodepthMap,
                                                                     int iodepthMap_p, float3* lastThreeSimsMap,
                                                                     int lastThreeSimsMap_p, int width, int height,
                                                                     bool moveByTcOrRc, int xFrom)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    int2 pix;
    pix.x = x + xFrom;
    pix.y = y;

    if((x < width) && (y < height))
    {
        float midDepth = *get2DBufferAt(iodepthMap, iodepthMap_p, x, y);
        float3 sims = *get2DBufferAt(lastThreeSimsMap, lastThreeSimsMap_p, x, y);
        float outDepth = midDepth;
        float outSim = sims.y;

        if(outDepth > 0.0f)
        {
            float3 pMid = get3DPointForPixelAndDepthFromRC(pix, midDepth);
            float3 pm1 = pMid;
            float3 pp1 = pMid;
            move3DPointByTcOrRcPixStep(pix, pm1, -1.0f, moveByTcOrRc);
            move3DPointByTcOrRcPixStep(pix, pp1, +1.0f, moveByTcOrRc);

            float3 depths;
            depths.x = size(pm1 - sg_s_rC);
            depths.y = midDepth;
            depths.z = size(pp1 - sg_s_rC);

            float refinedDepth = refineDepthSubPixel(depths, sims);
            if(refinedDepth > 0.0f)
            {
                outDepth = refinedDepth;
            };
        };

        *get2DBufferAt(osimMap, osimMap_p, x, y) = outSim;
        *get2DBufferAt(iodepthMap, iodepthMap_p, x, y) = outDepth;
    };
}

__global__ void refine_updateLastThreeSimsMap_kernel(float3* lastThreeSimsMap, int lastThreeSimsMap_p, float* simMap,
                                                     int simMap_p, int width, int height, int id)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if((x < width) && (y < height))
    {
        float sim = *get2DBufferAt(simMap, simMap_p, x, y);
        float3* lastThreeSims = get2DBufferAt(lastThreeSimsMap, lastThreeSimsMap_p, x, y);

        if(id == 0)
        {
            *lastThreeSims = make_float3(1.1f, 1.1f, 1.1f);
        }

        lastThreeSims->z = sim;
    };
}

__global__ void refine_updateBestStatMap_kernel(float4* bestStatMap, int bestStatMap_p, float3* lastThreeSimsMap,
                                                int lastThreeSimsMap_p, int width, int height, int id, int nids,
                                                float tcStepBefore, float tcStepAct)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if((x < width) && (y < height))
    {
        float3 lastThreeSims = lastThreeSimsMap[y * lastThreeSimsMap_p + x];
        float4 stat = bestStatMap[y * bestStatMap_p + x];

        if(id == 0)
        {
            stat.x = 1.1f;
            stat.y = lastThreeSims.z;
            stat.z = 1.1f;
            stat.w = tcStepAct;
        };

        if((id == 1) && (lastThreeSims.z < stat.y))
        {
            stat.x = 1.1f;
            stat.y = lastThreeSims.z;
            stat.z = 1.1f;
            stat.w = tcStepAct;
        };

        if((id >= 2) && (lastThreeSims.y < stat.y))
        {
            stat.x = lastThreeSims.x;
            stat.y = lastThreeSims.y;
            stat.z = lastThreeSims.z;
            stat.w = tcStepBefore;
        };

        if((id == nids - 1) && (lastThreeSims.z < stat.y))
        {
            stat.x = 1.1f;
            stat.y = lastThreeSims.z;
            stat.z = 1.1f;
            stat.w = tcStepAct;
        };

        bestStatMap[y * bestStatMap_p + x] = stat;
    };
}

__global__ void refine_computeDepthSimMapFromBestStatMap_kernel(float* simMap, int simMap_p, float* depthMap,
                                                                int depthMap_p, float4* bestStatMap, int bestStatMap_p,
                                                                int width, int height, bool moveByTcOrRc)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    int2 pix;
    pix.x = x;
    pix.y = y;

    if((x < width) && (y < height))
    {
        float4 stat = bestStatMap[y * bestStatMap_p + x];
        float outDepth = depthMap[y * depthMap_p + x];
        float outSim = stat.y;

        if(outDepth > 0.0f)
        {
            float tcStep = stat.w;
            float3 porig = get3DPointForPixelAndDepthFromRC(pix, outDepth);
            float3 p = porig;
            // move3DPointByTcPixStep(p, tcStep);
            move3DPointByTcOrRcPixStep(pix, p, tcStep, moveByTcOrRc);
            outDepth = size(p - sg_s_rC);

            if((stat.x < 1.1f) && (stat.z < 1.1f))
            {
                float3 depths;
                float3 pm1 = porig;
                float3 pp1 = porig;
                // move3DPointByTcPixStep(pm1, tcStep-1.0f);
                move3DPointByTcOrRcPixStep(pix, pm1, tcStep - 1.0f, moveByTcOrRc);
                // move3DPointByTcPixStep(pp1, tcStep+1.0f);
                move3DPointByTcOrRcPixStep(pix, pp1, tcStep + 1.0f, moveByTcOrRc);

                depths.x = size(pm1 - sg_s_rC);
                depths.y = outDepth;
                depths.z = size(pp1 - sg_s_rC);

                float3 sims;
                sims.x = stat.x;
                sims.y = stat.y;
                sims.z = stat.z;

                float refinedDepth = refineDepthSubPixel(depths, sims);
                if(refinedDepth > 0.0f)
                {
                    outDepth = refinedDepth;
                }
                else
                {
                    outDepth = outDepth;
                    outSim = stat.y;
                };
            }
            else
            {
                outDepth = -1.0f;
                outSim = 1.0f;
            };
        }
        else
        {
            outDepth = -1.0f;
            outSim = 1.0f;
        };

        simMap[y * simMap_p + x] = outSim;
        depthMap[y * depthMap_p + x] = outDepth;
    };
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

__global__ void refine_reprojTarTexLABByRcTcDepthsMap_kernel(uchar4* tex, int tex_p, float* rcDepthMap,
                                                             int rcDepthMap_p, int width, int height,
                                                             float depthMapShift)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    int2 rpix;
    rpix.x = x;
    rpix.y = y;

    if((x < width) && (y < height))
    {
        float rcDepth = rcDepthMap[y * rcDepthMap_p + x];
        uchar4 col = make_uchar4(0, 0, 0, 255);

        if(rcDepth > 0.0f)
        {
            float3 rp = get3DPointForPixelAndDepthFromRC(rpix, rcDepth);
            float3 rpS = get3DPointForPixelAndDepthFromRC(rpix, rcDepth + depthMapShift);

            float2 tpc = project3DPoint(sg_s_tP, rp);
            float2 tpcS = project3DPoint(sg_s_tP, rpS);

            int2 tpix = make_int2((int)(tpc.x + 0.5f), (int)(tpc.y + 0.5f));
            float tcDepth = tex2D(depthsTex, tpix.x, tpix.y);

            if((tcDepth > 0.0f) && ((tpc.x + 0.5f) > 0.0f) && ((tpc.y + 0.5f) > 0.0f) &&
               ((tpc.x + 0.5f) < (float)width - 1.0f) && ((tpc.y + 0.5f) < (float)height - 1.0f))
            {
                float pixSize = computePixSize(rp);
                float3 tp = get3DPointForPixelAndDepthFromTC(tpc, tcDepth);
                float dist = size(rp - tp);
                if(dist < pixSize)
                {
                    // col = float4_to_uchar4(255.0f*tex2D(t4tex, (float)tpc.x+0.5f, (float)tpc.y+0.5f));
                    // col = float4_to_uchar4(255.0f*tex2D(t4tex, tpc.x, tpc.y));
                    col = float4_to_uchar4(255.0f * tex2D(t4tex, tpcS.x, tpcS.y));
                };
            };
        };

        tex[y * tex_p + x] = col;
    };
}

inline static __device__ double refine_convolveGaussSigma2(float* Im)
{
    double sum = 0.0;
    for(int yp = -2; yp <= +2; yp++)
    {
        for(int xp = -2; xp <= +2; xp++)
        {
            sum = sum + (double)Im[(xp + 2) * 5 + yp + 2] * (double)gauss5[yp + 2] * (double)gauss5[xp + 2];
        };
    };
    return sum;
}

__global__ void refine_compPhotoErr_kernel(float* osimMap, int osimMap_p, float* depthMap, int depthMap_p, int width,
                                           int height, double beta)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    float rim[5 * 5];
    float tim[5 * 5];
    bool ok = false;
    float osim = 2.0f;

    if((x > 2) && (y > 2) && (x < width - 2) && (y < height - 2))
    {
        float depth = depthMap[y * depthMap_p + x];
        if(depth > 0.0f)
        {
            float4 tcm = uchar4_to_float4(tex2D(tTexU4, x, y));
            if(tcm.w == 0)
            {
                for(int yp = -2; yp <= +2; yp++)
                {
                    for(int xp = -2; xp <= +2; xp++)
                    {
                        rim[(xp + 2) * 5 + yp + 2] = (float)(tex2D(rTexU4, x + xp, y + yp).x);
                        tim[(xp + 2) * 5 + yp + 2] = (float)(tex2D(tTexU4, x + xp, y + yp).x);
                    };
                };
                ok = true;
            };
        };
    };

    __syncthreads();

    if(ok == true)
    {
        double omega = sumGauss55;

        double u1 = refine_convolveGaussSigma2(rim) / omega;
        double u2 = refine_convolveGaussSigma2(tim) / omega;

        float tmpIm[5 * 5];
        for(int yp = -2; yp <= +2; yp++)
        {
            for(int xp = -2; xp <= +2; xp++)
            {
                tmpIm[(xp + 2) * 5 + yp + 2] = rim[(xp + 2) * 5 + yp + 2] * rim[(xp + 2) * 5 + yp + 2];
            };
        };
        double v1 = refine_convolveGaussSigma2(tmpIm) / omega - u1 * u1 + beta * beta;

        for(int yp = -2; yp <= +2; yp++)
        {
            for(int xp = -2; xp <= +2; xp++)
            {
                tmpIm[(xp + 2) * 5 + yp + 2] = tim[(xp + 2) * 5 + yp + 2] * tim[(xp + 2) * 5 + yp + 2];
            };
        };
        double v2 = refine_convolveGaussSigma2(tmpIm) / omega - u2 * u2 + beta * beta;

        for(int yp = -2; yp <= +2; yp++)
        {
            for(int xp = -2; xp <= +2; xp++)
            {
                tmpIm[(xp + 2) * 5 + yp + 2] = rim[(xp + 2) * 5 + yp + 2] * tim[(xp + 2) * 5 + yp + 2];
            };
        };
        double v12 = refine_convolveGaussSigma2(tmpIm) / omega - u1 * u2;

        osim = -(float)(v12 / sqrt(v1 * v2));

        osimMap[y * osimMap_p + x] = -osim;
    };

    if((x > 0) && (y > 0) && (x < width) && (y < height))
    {
        osimMap[y * osimMap_p + x] = osim;
    };
}

__global__ void refine_compPhotoErrStat_kernel(float* occMap, int occMap_p, float4* ostat1Map, int ostat1Map_p,
                                               float* depthMap, int depthMap_p, int width, int height, double beta)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    float rim[5 * 5];
    float tim[5 * 5];
    bool ok = false;
    float4 ostat1 = make_float4(0.0f, 0.0f, 0.0f, 0.0f);
    float occ = 0.0f;

    if((x > 2) && (y > 2) && (x < width - 2) && (y < height - 2))
    {
        float depth = depthMap[y * depthMap_p + x];
        if(depth > 0.0f)
        {
            float4 tcm = uchar4_to_float4(tex2D(tTexU4, x, y));
            if(tcm.w == 0)
            {
                for(int yp = -2; yp <= +2; yp++)
                {
                    for(int xp = -2; xp <= +2; xp++)
                    {
                        rim[(xp + 2) * 5 + yp + 2] = (float)(tex2D(rTexU4, x + xp, y + yp).x);
                        tim[(xp + 2) * 5 + yp + 2] = (float)(tex2D(tTexU4, x + xp, y + yp).x);
                    };
                };
                ok = true;
            };
        };
    };

    __syncthreads();

    if(ok == true)
    {
        double omega = sumGauss55;

        double u1 = refine_convolveGaussSigma2(rim) / omega;
        double u2 = refine_convolveGaussSigma2(tim) / omega;

        float tmpIm[5 * 5];
        for(int yp = -2; yp <= +2; yp++)
        {
            for(int xp = -2; xp <= +2; xp++)
            {
                tmpIm[(xp + 2) * 5 + yp + 2] = rim[(xp + 2) * 5 + yp + 2] * rim[(xp + 2) * 5 + yp + 2];
            };
        };
        double v1 = refine_convolveGaussSigma2(tmpIm) / omega - u1 * u1 + beta * beta;

        for(int yp = -2; yp <= +2; yp++)
        {
            for(int xp = -2; xp <= +2; xp++)
            {
                tmpIm[(xp + 2) * 5 + yp + 2] = tim[(xp + 2) * 5 + yp + 2] * tim[(xp + 2) * 5 + yp + 2];
            };
        };
        double v2 = refine_convolveGaussSigma2(tmpIm) / omega - u2 * u2 + beta * beta;

        for(int yp = -2; yp <= +2; yp++)
        {
            for(int xp = -2; xp <= +2; xp++)
            {
                tmpIm[(xp + 2) * 5 + yp + 2] = rim[(xp + 2) * 5 + yp + 2] * tim[(xp + 2) * 5 + yp + 2];
            };
        };
        double v12 = refine_convolveGaussSigma2(tmpIm) / omega - u1 * u2;

        for(int yp = -2; yp <= +2; yp++)
        {
            for(int xp = -2; xp <= +2; xp++)
            {
                tmpIm[(xp + 2) * 5 + yp + 2] = -1.0f / (omega);
            };
        };

        double sqv1v2 = sqrt(v1 * v2);
        double cc = -(v12 / sqv1v2);
        occ = (float)cc;

        ostat1 = make_float4((float)(-1.0 / (omega * sqv1v2)), (float)(cc / (omega * v2)),
                             (float)(u1 / (omega * sqv1v2) - (u2 * cc) / (omega * v2)), 1.0f);
    };

    if((x > 0) && (y > 0) && (x < width) && (y < height))
    {
        occMap[y * occMap_p + x] = occ;
        ostat1Map[y * ostat1Map_p + x] = ostat1;
    };
}

__global__ void refine_compPhotoErrABG_kernel(float* osimMap, int osimMap_p, int width, int height)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    float3 im[5 * 5];
    bool ok = false;
    float osim = 2.0f;

    if((x > 2) && (y > 2) && (x < width - 2) && (y < height - 2))
    {
        float4 a = tex2D(f4Tex, x, y);
        if(a.w > 0.0f)
        {
            for(int yp = -2; yp <= +2; yp++)
            {
                for(int xp = -2; xp <= +2; xp++)
                {
                    a = tex2D(f4Tex, x + xp, y + yp);
                    im[(xp + 2) * 5 + yp + 2].x = a.x;
                    im[(xp + 2) * 5 + yp + 2].y = a.y;
                    im[(xp + 2) * 5 + yp + 2].z = a.z;
                };
            };
            ok = true;
        };
    };

    __syncthreads();

    if(ok == true)
    {
        float4 oabg = make_float4(0.0f, 0.0f, 0.0f, 0.0f);

        float tmpIm[5 * 5];
        for(int yp = -2; yp <= +2; yp++)
        {
            for(int xp = -2; xp <= +2; xp++)
            {
                tmpIm[(xp + 2) * 5 + yp + 2] = im[(xp + 2) * 5 + yp + 2].x;
            };
        };
        oabg.x = refine_convolveGaussSigma2(tmpIm);

        for(int yp = -2; yp <= +2; yp++)
        {
            for(int xp = -2; xp <= +2; xp++)
            {
                tmpIm[(xp + 2) * 5 + yp + 2] = im[(xp + 2) * 5 + yp + 2].y;
            };
        };
        oabg.y = refine_convolveGaussSigma2(tmpIm);

        for(int yp = -2; yp <= +2; yp++)
        {
            for(int xp = -2; xp <= +2; xp++)
            {
                tmpIm[(xp + 2) * 5 + yp + 2] = im[(xp + 2) * 5 + yp + 2].z;
            };
        };
        oabg.z = refine_convolveGaussSigma2(tmpIm);
        oabg.w = 1.0f;

        float ric = (float)(tex2D(rTexU4, x, y).x);
        float tic = (float)(tex2D(tTexU4, x, y).x);

        osim = oabg.x * ric + oabg.y * tic + oabg.z;
        osim = isinf(osim) ? 1.0f : osim;
        osim = fmaxf(fminf(osim, 1.0f), -1.0f);
    };

    if((x > 0) && (y > 0) && (x < width) && (y < height))
    {
        osimMap[y * osimMap_p + x] = osim;
    };
}

/*
 * Buggy code: tTexU4 returns unsigned char and not normalized float
 *
__device__ float2 ComputeSobelTarIm(int x, int y)
{
    unsigned char ul = 255.0f * tex2D(tTexU4, x - 1, y - 1).x; // upper left
    unsigned char um = 255.0f * tex2D(tTexU4, x + 0, y - 1).x; // upper middle
    unsigned char ur = 255.0f * tex2D(tTexU4, x + 1, y - 1).x; // upper right
    unsigned char ml = 255.0f * tex2D(tTexU4, x - 1, y + 0).x; // middle left
    unsigned char mm = 255.0f * tex2D(tTexU4, x + 0, y + 0).x; // middle (unused)
    unsigned char mr = 255.0f * tex2D(tTexU4, x + 1, y + 0).x; // middle right
    unsigned char ll = 255.0f * tex2D(tTexU4, x - 1, y + 1).x; // lower left
    unsigned char lm = 255.0f * tex2D(tTexU4, x + 0, y + 1).x; // lower middle
    unsigned char lr = 255.0f * tex2D(tTexU4, x + 1, y + 1).x; // lower right

    int Horz = ur + 2 * mr + lr - ul - 2 * ml - ll;
    int Vert = ul + 2 * um + ur - ll - 2 * lm - lr;
    return make_float2((float)Vert, (float)Horz);
}
*/

__device__ float2 DPIXTCDRC(const float3& P)
{
    float M3P = sg_s_tP[2] * P.x + sg_s_tP[5] * P.y + sg_s_tP[8] * P.z + sg_s_tP[11];
    float M3P2 = M3P * M3P;

    float m11 = ((sg_s_tP[0] * sg_s_tP[5] - sg_s_tP[2] * sg_s_tP[3]) * P.y +
                 (sg_s_tP[0] * sg_s_tP[8] - sg_s_tP[2] * sg_s_tP[6]) * P.z +
                 (sg_s_tP[0] * sg_s_tP[11] - sg_s_tP[2] * sg_s_tP[9])) /
                M3P2;
    float m12 = ((sg_s_tP[3] * sg_s_tP[2] - sg_s_tP[5] * sg_s_tP[0]) * P.x +
                 (sg_s_tP[3] * sg_s_tP[8] - sg_s_tP[5] * sg_s_tP[6]) * P.z +
                 (sg_s_tP[3] * sg_s_tP[11] - sg_s_tP[5] * sg_s_tP[9])) /
                M3P2;
    float m13 = ((sg_s_tP[6] * sg_s_tP[2] - sg_s_tP[8] * sg_s_tP[0]) * P.x +
                 (sg_s_tP[6] * sg_s_tP[5] - sg_s_tP[8] * sg_s_tP[3]) * P.y +
                 (sg_s_tP[6] * sg_s_tP[11] - sg_s_tP[8] * sg_s_tP[9])) /
                M3P2;

    float m21 = ((sg_s_tP[1] * sg_s_tP[5] - sg_s_tP[2] * sg_s_tP[4]) * P.y +
                 (sg_s_tP[1] * sg_s_tP[8] - sg_s_tP[2] * sg_s_tP[7]) * P.z +
                 (sg_s_tP[1] * sg_s_tP[11] - sg_s_tP[2] * sg_s_tP[10])) /
                M3P2;
    float m22 = ((sg_s_tP[4] * sg_s_tP[2] - sg_s_tP[5] * sg_s_tP[1]) * P.x +
                 (sg_s_tP[4] * sg_s_tP[8] - sg_s_tP[5] * sg_s_tP[7]) * P.z +
                 (sg_s_tP[4] * sg_s_tP[11] - sg_s_tP[5] * sg_s_tP[10])) /
                M3P2;
    float m23 = ((sg_s_tP[7] * sg_s_tP[2] - sg_s_tP[8] * sg_s_tP[1]) * P.x +
                 (sg_s_tP[7] * sg_s_tP[5] - sg_s_tP[8] * sg_s_tP[4]) * P.y +
                 (sg_s_tP[7] * sg_s_tP[11] - sg_s_tP[8] * sg_s_tP[10])) /
                M3P2;

    float3 _drc = P - sg_s_rC;

    float2 op;
    op.x = m11 * _drc.x + m12 * _drc.y + m13;
    op.y = m21 * _drc.x + m22 * _drc.y + m23;

    return op;
};

/*
 * Note: ComputeSobelTarIm called from here is buggy
 *
__global__ void refine_reprojTarSobelAndDPIXTCDRCRcTcDepthsMap_kernel(float4* tex, int tex_p, float* rcDepthMap,
                                                                      int rcDepthMap_p, int width, int height,
                                                                      float depthMapShift)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    int2 rpix;
    rpix.x = x;
    rpix.y = y;

    if((x < width) && (y < height))
    {
        float rcDepth = rcDepthMap[y * rcDepthMap_p + x];
        float4 col = make_float4(0.0f, 0.0f, 0.0f, 0.0f);

        if(rcDepth > 0.0f)
        {
            float3 rp = get3DPointForPixelAndDepthFromRC(rpix, rcDepth);
            float3 rpS = get3DPointForPixelAndDepthFromRC(rpix, rcDepth + depthMapShift);

            float2 tpc = project3DPoint(sg_s_tP, rp);
            int2 tpix = make_int2((int)(tpc.x + 0.5f), (int)(tpc.y + 0.5f));
            float tcDepth = tex2D(depthsTex, tpix.x, tpix.y);

            if((tcDepth > 0.0f) && ((tpc.x + 0.5f) > 0.0f) && ((tpc.y + 0.5f) > 0.0f) &&
               ((tpc.x + 0.5f) < (float)width - 1.0f) && ((tpc.y + 0.5f) < (float)height - 1.0f))
            {
                float pixSize = computePixSize(rp);
                float3 tp = get3DPointForPixelAndDepthFromTC(tpc, tcDepth);
                float dist = size(rp - tp);
                if(dist < pixSize)
                {
                    // col = float4_to_uchar4(255.0f*tex2D(t4tex, (float)tpc.x+0.5f, (float)tpc.y+0.5f));
                    // float2 op = DPIXTCDRC(tp);
                    float2 op = DPIXTCDRC(rpS);
                    float2 so = ComputeSobelTarIm(tpix.x, tpix.y);
                    col = make_float4(op.x, op.y, so.x, so.y);
                }
            }
        }

        tex[y * tex_p + x] = col;
    }
}
*/

__global__ void refine_computeRcTcDepthMap_kernel(float* rcDepthMap, int rcDepthMap_p, int width, int height,
                                                  float pixSizeRatioThr)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    int2 rpix;
    rpix.x = x;
    rpix.y = y;

    if((x < width) && (y < height))
    {
        float rcDepth = rcDepthMap[y * rcDepthMap_p + x];
        float rcDepthOut = -1.0f;
        if(rcDepth > 0.0f)
        {
            float3 rp = get3DPointForPixelAndDepthFromRC(rpix, rcDepth);
            float2 tpc = project3DPoint(sg_s_tP, rp);
            int2 tpix = make_int2((int)(tpc.x + 0.5f), (int)(tpc.y + 0.5f));
            float tcDepth = tex2D(depthsTex, tpc.x + 0.5f, tpc.y + 0.5f);

            if((tcDepth > 0.0f) && (tpix.x > 0) && (tpix.y > 0) && (tpix.x < width) && (tpix.y < height))
            {
                float3 tp = get3DPointForPixelAndDepthFromTC(tpc, tcDepth);
                float rPixSize = computeRcPixSize(rp);
                float tPixSize = computeTcPixSize(tp);
                float pixSizeRatio = fmaxf(rPixSize, tPixSize) / fminf(rPixSize, tPixSize);
                float dist = size(rp - tp);

                rcDepthOut = ((dist / rPixSize < 10.0f) && (pixSizeRatio < pixSizeRatioThr)) ? rcDepth : -1.0f;
            };
        };

        rcDepthMap[y * rcDepthMap_p + x] = rcDepthOut;
    };
}

} // namespace depthMap
} // namespace aliceVision
