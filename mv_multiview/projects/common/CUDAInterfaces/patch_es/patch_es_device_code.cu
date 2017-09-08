#ifndef PATCH_ES_DEVICE_CODE_CU
#define PATCH_ES_DEVICE_CODE_CU

__global__ void patch_kernel(float* out, int out_p, int nRotHx, int stepRotx, int nRotHy, int stepRoty, int nPosH,
                             int stepPos, int2 rotMapsGrid, int2 workArea, int npts, int t, int ptsAtGrid, int c,
                             int wsh, float3* pts, int pts_p, float3* nms, int nms_p, float3* xas, int xas_p,
                             float3* yas, int yas_p, float* psz, int psz_p, int width, int height)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    int nRotx = 2 * nRotHx + 1;
    int nRoty = 2 * nRotHy + 1;
    int nPos = 2 * nPosH + 1;

    int idPtx = x / nRotx;
    int idPty = y / nRoty;
    int idPt = (idPty * rotMapsGrid.x + idPtx) / nPos;

    int rx = stepRotx * (x % nRotx - nRotHx);
    int ry = stepRoty * (y % nRoty - nRotHy);
    int pt = stepPos * ((idPty * rotMapsGrid.x + idPtx) % nPos - nPosH);

    if((x < workArea.x) && (y < workArea.y) && (t * ptsAtGrid + idPt < npts))
    {
        // reading from constant memory
        patch ptch;
        /*
        ptch.p = pospts[idPt];
        ptch.n   = norpts[idPt];
        ptch.x   = xaxpts[idPt];
        ptch.y   = yaxpts[idPt];
        ptch.d   = pxspts[idPt];
        */
        ptch.p = pts[y * pts_p + x];
        ptch.n = nms[y * nms_p + x];
        ptch.x = xas[y * xas_p + x];
        ptch.y = yas[y * yas_p + x];
        ptch.d = psz[y * psz_p + x];

        // ptch.n = rC-ptch.p;
        // normalize(ptch.n);
        // computeRotCS(ptch.x, ptch.y, ptch.n);
        // ptch.d = getPatchPixSize(ptch);

        // move patch by pt steps
        movePatch(ptch, pt);

        // rotate patch by rx, ry angles
        rotatePatch(ptch, rx, ry);

        if(checkPatch(ptch, 80) == true)
        // if (ptch.d>0.0f)
        {
            // compute similarity
            // float sim=compNCCbyH(ptch,wsh);
            float sim = compNCCby3Dpts(ptch, wsh, width, height);

            if(c == 0)
            {
                out[y * out_p + x] = sim;
            }
            else
            {
                out[y * out_p + x] = min(out[y * out_p + x], sim);
            };
        }
        else
        {
            out[y * out_p + x] = 1.0f;
        };

        // float2 g = patchPixCol(ptch, rx, ry);
        // out[y*out_p+x] = g.y;
    };
}

/*
//slow!!!
__global__ void readRange_kernel(float *ole, int ole_p,
                                                                    int nRotHx, int stepRotx, int nRotHy, int stepRoty,
int nPosH, int stepPos,
                                                                        int2 rotMapsGrid, int2 workArea, int npts, int
t, int ptsAtGrid, int nbest)
{
        int idPt = blockIdx.x*blockDim.x + threadIdx.x;

        if ((idPt<ptsAtGrid)&&(t*ptsAtGrid+idPt<npts))
        {
                int nRotx = 2*nRotHx+1;
                int nRoty = 2*nRotHy+1;
                int nPos = 2*nPosH+1;

                float minle =  1.0f;
                float maxle = -1.0f;
                for (int pt=0;pt<nPos;pt++) {
                        for (int rx=0;rx<nRotx;rx++) {
                                for (int ry=0;ry<nRoty;ry++) {
                                        int xpt = (idPt*nPos+pt)%rotMapsGrid.x;
                                        int ypt = (idPt*nPos+pt)/rotMapsGrid.x;
                                        int x = xpt*nRotx+rx;
                                        int y = ypt*nRoty+ry;
                                        float sim = tex2D(watex,x,y);
                                        if (sim<0.9){
                                                minle = min(minle,sim);
                                                maxle = max(maxle,sim);
                                        };
                                };
                        };
                };

                float3 le;
                le.x = minle;
                le.y = maxle;
                le.z = 0.0f;

                ole[0*ole_p+idPt] = le.x;
                ole[1*ole_p+idPt] = le.y;
                ole[2*ole_p+idPt] = le.z;
        };
}
*/

// slow noncoalescent can be done better
__global__ void readMinSupNeigh_kernel(float* opts, int opts_s, int opts_p, float* onms, int onms_s, int onms_p,
                                       float* osim, int osim_p, float* wam, int wam_p, int nRotHx, int stepRotx,
                                       int nRotHy, int stepRoty, int nPosH, int stepPos, int2 rotMapsGrid,
                                       int2 workArea, int npts, int t, int ptsAtGrid, int ibest, int nbest, int kx,
                                       int ky)
{
    int idPt = blockIdx.x * blockDim.x + threadIdx.x;

    if((idPt < ptsAtGrid) && (t * ptsAtGrid + idPt < npts))
    {
        int nRotx = 2 * nRotHx + 1;
        int nRoty = 2 * nRotHy + 1;
        int nPos = 2 * nPosH + 1;

        int xm, ym;
        float3 p;
        float3 n;
        float bestSim = 2.0;
        for(int pt = 0; pt < nPos; pt++)
        {
            for(int rx = 0; rx < nRotx; rx++)
            {
                for(int ry = 0; ry < nRoty; ry++)
                {
                    int xpt = (idPt * nPos + pt) % rotMapsGrid.x;
                    int ypt = (idPt * nPos + pt) / rotMapsGrid.x;
                    int x = xpt * nRotx + rx;
                    int y = ypt * nRoty + ry;
                    float sim = tex2D(watex, x, y);

                    if(bestSim > sim)
                    {

                        // reading from constant memory
                        patch ptch;
                        ptch.p = pospts[idPt];
                        ptch.n = norpts[idPt];
                        ptch.x = xaxpts[idPt];
                        ptch.y = yaxpts[idPt];
                        ptch.d = pxspts[idPt];

                        // move patch by pt steps
                        movePatch(ptch, stepPos * (pt - nPosH));

                        // rotate patch by rx, ry angles
                        rotatePatch(ptch, stepRotx * (rx - nRotHx), stepRoty * (ry - nRotHy));

                        p = ptch.p;
                        n = ptch.n;
                        xm = x;
                        ym = y;
                        bestSim = sim;
                    };
                };
            };
        };

        for(int xp = xm - kx; xp <= xm + kx; xp++)
        {
            for(int yp = ym - ky; yp <= ym + ky; yp++)
            {

                if((xp >= 0) && (xp < workArea.x) && (yp >= 0) && (yp < workArea.y) && (xp / nRotx == xm / nRotx) &&
                   (yp / nRoty == ym / nRoty))
                {
                    wam[yp * wam_p + xp] = 2.0f;
                };
            };
        };
        int c = ibest * opts_p + idPt;

        opts[opts_s * 0 + c] = p.x;
        opts[opts_s * 1 + c] = p.y;
        opts[opts_s * 2 + c] = p.z;
        onms[onms_s * 0 + c] = n.x;
        onms[onms_s * 1 + c] = n.y;
        onms[onms_s * 2 + c] = n.z;
        osim[c] = bestSim;
    };
}

__global__ void ncc_flow_init_kernel(float2* flw, int flw_p, float* sim, int sim_p, int width, int height)
{
    size_t x = blockIdx.x * blockDim.x + threadIdx.x;
    size_t y = blockIdx.y * blockDim.y + threadIdx.y;

    if(x < width && y < height)
    {
        flw[y * flw_p + x].x = 0.0f;
        flw[y * flw_p + x].y = 0.0f;
        sim[y * sim_p + x] = 1.0f;
    };
}

__global__ void ncc_flow_kernel(float2* flw, int flw_p, float* sim, int sim_p, float2* trn, int trn_p, int wsh,
                                float xp, float yp, int width, int height)
{
    size_t x = blockIdx.x * blockDim.x + threadIdx.x;
    size_t y = blockIdx.y * blockDim.y + threadIdx.y;

    if(x < width && y < height)
    {
        float2 t = trn[y * trn_p + x];

        float2 rpix, tpix;
        rpix.x = (float)x;
        rpix.y = (float)y;

        tpix.x = (float)x + t.x + xp;
        tpix.y = (float)y + t.y + yp;

        float sn = compNCC(rpix, tpix, wsh);
        float so = sim[y * sim_p + x];

        if(sn < so)
        {
            flw[y * flw_p + x].x = xp;
            flw[y * flw_p + x].y = yp;
            sim[y * sim_p + x] = sn;
        };
    };
}

__global__ void ncc_flow_kernel_imgs(float2* flw, int flw_p, float* sim, int sim_p, int wsh, float xp, float yp,
                                     int width, int height)
{
    size_t x = blockIdx.x * blockDim.x + threadIdx.x;
    size_t y = blockIdx.y * blockDim.y + threadIdx.y;

    if(x < width && y < height)
    {
        float2 rpix, tpix;
        rpix.x = (float)x;
        rpix.y = (float)y;

        tpix.x = (float)x + xp;
        tpix.y = (float)y + yp;

        float sn = compNCC(rpix, tpix, wsh);
        float so = sim[y * sim_p + x];

        if(sn < so)
        {
            flw[y * flw_p + x].x = xp;
            flw[y * flw_p + x].y = yp;
            sim[y * sim_p + x] = sn;
        };
    };
}

__global__ void ncc_flow_kernel_imgs_pixels(float4* pxs, int pxs_p, float2* flw, int flw_p, float* sim, int sim_p,
                                            int wsh, float xp, float yp, int width, int height, int slicesAtTime,
                                            int ntimes, int npixels, float varThr)
{
    size_t x = blockIdx.x * blockDim.x + threadIdx.x;
    size_t y = blockIdx.y * blockDim.y + threadIdx.y;

    if(x < slicesAtTime && y < ntimes && (y * slicesAtTime + x) < npixels)
    {
        float2 rpix, tpix;
        rpix.x = pxs[y * pxs_p + x].x;
        rpix.y = pxs[y * pxs_p + x].y;
        tpix.x = pxs[y * pxs_p + x].z + xp;
        tpix.y = pxs[y * pxs_p + x].w + yp;

        float sn = compNCCvarThr(rpix, tpix, wsh, varThr);
        float so = sim[y * sim_p + x];

        if(sn < so)
        {
            flw[y * flw_p + x].x = tpix.x - rpix.x;
            flw[y * flw_p + x].y = tpix.y - rpix.y;
            sim[y * sim_p + x] = sn;
        };
    };
}

__global__ void ncc_flow_kernel_imgs_pixels_reset(float4* pxs, int pxs_p, float2* flw, int flw_p, float* sim, int sim_p,
                                                  int wsh, int width, int height, int slicesAtTime, int ntimes,
                                                  int npixels)
{
    size_t x = blockIdx.x * blockDim.x + threadIdx.x;
    size_t y = blockIdx.y * blockDim.y + threadIdx.y;

    if(x < slicesAtTime && y < ntimes && (y * slicesAtTime + x) < npixels)
    {
        float2 rpix, trn;
        rpix.x = pxs[y * pxs_p + x].x;
        rpix.y = pxs[y * pxs_p + x].y;
        trn.x = flw[y * flw_p + x].x;
        trn.y = flw[y * flw_p + x].y;
        pxs[y * pxs_p + x].z = rpix.x + trn.x;
        pxs[y * pxs_p + x].w = rpix.y + trn.y;
    };
}

__global__ void normalize_imgs_kernel(float* scale, int scale_p, float* shift, int shift_p, int wsh, int width,
                                      int height)
{
    size_t x = blockIdx.x * blockDim.x + threadIdx.x;
    size_t y = blockIdx.y * blockDim.y + threadIdx.y;

    if(x < width && y < height)
    {
        float2 pix;
        pix.x = (float)x;
        pix.y = (float)y;

        simStat sst = compSimStat(pix, pix, wsh);

        scale[y * scale_p + x] = 1.0f;
        shift[y * shift_p + x] = sst.xsum / sst.count - sst.ysum / sst.count;
    };
}

__global__ void pes_triangles_kernel(float3* A, float3* B, float3* C, float* out, int ntris)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;

    if(x < ntris)
    {
        // coalescent reading
        float3 a = A[x];
        float3 b = B[x];
        float3 c = C[x];

        ///////////////////////////////////////////////////////
        // compute similarity
        float wsh = 10.0f;
        int iwsh = (int)wsh;
        float3 ab = b - a;
        ab = ab / wsh;
        float3 ac = c - a;
        ac = ac / wsh;

        simStat sst = simStat();
        for(int xi = 0; xi <= iwsh; xi++)
        {
            for(int yi = 0; yi <= iwsh - xi; yi++)
            {
                float fx = (float)xi;
                float fy = (float)yi;
                float3 p = a + ab * fx + ac * fy;
                float2 rp = project3DPoint(sg_s_rP, p);
                float2 tp = project3DPoint(sg_s_tP, p);
                float2 g;
                g.x = tex2D(rtex, rp.x + 0.5, rp.y + 0.5);
                g.y = tex2D(ttex, tp.x + 0.5, tp.y + 0.5);
                sst.update(g);
            }
        }

        sst.computeSim();

        // coalescent writing
        out[x] = sst.sim;
    }
}

/*

__global__ void sum_mode_kernel(
                                        float *xim, int xim_p,
                                        float *yim, int yim_p,
                                        float *output, int output_p,
                                        int width, int height, int mode, int2 shiftyim)
{
        size_t x = blockIdx.x*blockDim.x + threadIdx.x;
        size_t y = blockIdx.y*blockDim.y + threadIdx.y;

        if ((x<width)&&(y<height)&&(x+shiftyim.x>=0)&&(x+shiftyim.x<width)&&(y+shiftyim.y>=0)&&(y+shiftyim.y<height))
        {
                int id_inx = y*xim_p+x;
                int id_iny = (y+shiftyim.y)*yim_p+(x+shiftyim.x);
                int id_out = y*output_p+x;

                if (mode==0) {
                        output[id_out] = xim[id_inx];
                };
                if (mode==1) {
                        output[id_out] = yim[id_iny];
                };
                if (mode==2) {
                        float val = xim[id_inx];
                        output[id_out] = val*val;
                };
                if (mode==3) {
                        float val = yim[id_iny];
                        output[id_out] = val*val;
                };
                if (mode==4) {
                        output[id_out] = xim[id_inx]*yim[id_iny];
                };
        };

}


__global__ void transpose_kernel(float *input, int input_p, float*output, int output_p, int width, int height)
{
        __shared__ float temp[BLOCK_DIM][BLOCK_DIM+1];
        int xIndex = blockIdx.x*BLOCK_DIM + threadIdx.x;
        int yIndex = blockIdx.y*BLOCK_DIM + threadIdx.y;

        if((xIndex < width) && (yIndex < height))
        {
                int id_in = yIndex * input_p + xIndex;
                temp[threadIdx.y][threadIdx.x] = input[id_in];
        }

        __syncthreads();

        xIndex = blockIdx.y * BLOCK_DIM + threadIdx.x;
        yIndex = blockIdx.x * BLOCK_DIM + threadIdx.y;

        if((xIndex < height) && (yIndex < width))
        {
                int id_out = yIndex * output_p + xIndex;
                output[id_out] = temp[threadIdx.x][threadIdx.y];
        };

}

__global__ void scan_kernel(float *input, float*output, int n)
{
        extern __shared__ float temp[];

        int tdx = threadIdx.x; int offset = 1;
        temp[2*tdx] = input[2*tdx];
        temp[2*tdx+1] = input[2*tdx+1];

        for(int d = n>>1; d > 0; d >>= 1)
        {
                __syncthreads();
                if(tdx < d)
                {
                        int ai = offset*(2*tdx+1)-1;
                        int bi = offset*(2*tdx+2)-1;
                        temp[bi] += temp[ai];
                };
                offset *= 2;
        };

        if(tdx == 0) temp[n - 1] = 0;

        for(int d = 1; d < n; d *= 2)
        {
                offset >>= 1; __syncthreads();
                if(tdx < d)
                {
                        int ai = offset*(2*tdx+1)-1;
                        int bi = offset*(2*tdx+2)-1;
                        float t = temp[ai];
                        temp[ai] = temp[bi];
                        temp[bi] += t;
                };
        };

        __syncthreads();

        output[2*tdx] = temp[2*tdx];
        output[2*tdx+1] = temp[2*tdx+1];
}


__global__ void ncc_flow_kernel_imgs_integral(
                float *xsat,  int xsat_p,
                float *ysat,  int ysat_p,
                float *xxsat, int xxsat_p,
                float *yysat, int yysat_p,
                float *xysat, int xysat_p,
                float2 *flw, int flw_p,
                float *sim, int sim_p,
                int wsh, float xp, float yp, int width, int height)
{
        size_t x = blockIdx.x*blockDim.x + threadIdx.x;
        size_t y = blockIdx.y*blockDim.y + threadIdx.y;

        if (x<width && y<height)
        {
                xsat


                float sn = compNCC(rpix, tpix, wsh);
                float so = sim[y*sim_p+x];

                if (sn<so) {
                        flw[y*flw_p+x].x = xp;
                        flw[y*flw_p+x].y = yp;
                        sim[y*sim_p+x] = sn;
                };
        };
}

*/

__global__ void remapRGB_kernel(float2* map, int map_p, float4* out, int out_p, int width, int height)
{
    size_t x = blockIdx.x * blockDim.x + threadIdx.x;
    size_t y = blockIdx.y * blockDim.y + threadIdx.y;

    if(x < width && y < height)
    {
        float2 m = map[y * map_p + x];
        float r = -1.0f;
        float g = -1.0f;
        float b = -1.0f;
        float w = -1.0f;
        if((m.x >= 0.0f) && (m.x < (float)width) && (m.y >= 0.0f) && (m.y < (float)height))
        {
            r = 255.0f * tex2D(rtex, m.x + 0.5f, m.y + 0.5f);
            g = 255.0f * tex2D(gtex, m.x + 0.5f, m.y + 0.5f);
            b = 255.0f * tex2D(btex, m.x + 0.5f, m.y + 0.5f);
            w = 1.0f;
        };
        out[y * out_p + x].x = r;
        out[y * out_p + x].y = g;
        out[y * out_p + x].z = b;
        out[y * out_p + x].w = w;
    };
}

__global__ void remapGray_kernel(float2* map, int map_p, float* out, int out_p, int width, int height)
{
    size_t x = blockIdx.x * blockDim.x + threadIdx.x;
    size_t y = blockIdx.y * blockDim.y + threadIdx.y;

    if(x < width && y < height)
    {
        float2 m = map[y * map_p + x];
        float r = -1.0f;
        if((m.x >= 0.0f) && (m.x < (float)width) && (m.y >= 0.0f) && (m.y < (float)height))
        {
            r = 255.0f * tex2D(rtex, m.x + 0.5f, m.y + 0.5f);
        };
        out[y * out_p + x] = r;
    };
}

__global__ void transpose_float2_kernel(float2* input, int input_p, float2* output, int output_p, int width, int height)
{
    __shared__ float2 temp[BLOCK_DIM][BLOCK_DIM + 1];
    int xIndex = blockIdx.x * BLOCK_DIM + threadIdx.x;
    int yIndex = blockIdx.y * BLOCK_DIM + threadIdx.y;

    if((xIndex < width) && (yIndex < height))
    {
        int id_in = yIndex * input_p + xIndex;
        temp[threadIdx.y][threadIdx.x] = input[id_in];
    }

    __syncthreads();

    xIndex = blockIdx.y * BLOCK_DIM + threadIdx.x;
    yIndex = blockIdx.x * BLOCK_DIM + threadIdx.y;

    if((xIndex < height) && (yIndex < width))
    {
        int id_out = yIndex * output_p + xIndex;
        output[id_out] = temp[threadIdx.x][threadIdx.y];
    };
}

#endif // PATCH_ES_DEVICE_CODE_CU
