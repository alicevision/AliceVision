#ifndef DEVICE_CODE_CU
#define DEVICE_CODE_CU

__global__ void triangles_kernel(float3* tA, float3* tB, float3* tC, float* out, int ntris)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;

    if(x < ntris)
    {
        // coalescent reading
        float3 A = tA[x];
        float3 B = tB[x];
        float3 C = tC[x];

        float2 ra = project3DPoint(rP, A);
        float2 rb = project3DPoint(rP, B);
        float2 rc = project3DPoint(rP, C);

        float2 ta = project3DPoint(tP, A);
        float2 tb = project3DPoint(tP, B);
        float2 tc = project3DPoint(tP, C);

        float3 S = (A + B + C) / 3.0f;
        float2 rs = project3DPoint(rP, S);
        float2 ts = project3DPoint(tP, S);

        float3 vCA = C - A;
        float3 vBA = B - A;
        normalize(vCA);
        normalize(vBA);
        float3 n = cross(vCA, vBA);

        float H[9];
        computeHomography(H, A, n);

        ///////////////////////////////////////////////////////
        // compute similarity
        int wsh = 3;
        simStat sst = simStat();
        for(int x = -wsh; x <= wsh; x++)
        {
            for(int y = -wsh; y <= wsh; y++)
            {
                float2 rp, tp, g;

                /*
                rp.x = ra.x+(float)x; rp.y = ra.y+(float)y;
                tp = V2M3x3mulV2(H,rp);
                g.x = tex2D(rtex, rp.x+0.5, rp.y+0.5);
                g.y = tex2D(ttex, tp.x+0.5, tp.y+0.5);
                sst.update(g);

                rp.x = rb.x+(float)x; rp.y = rb.y+(float)y;
                tp = V2M3x3mulV2(H,rp);
                g.x = tex2D(rtex, rp.x+0.5, rp.y+0.5);
                g.y = tex2D(ttex, tp.x+0.5, tp.y+0.5);
                sst.update(g);

                rp.x = rc.x+(float)x; rp.y = rc.y+(float)y;
                tp = V2M3x3mulV2(H,rp);
                g.x = tex2D(rtex, rp.x+0.5, rp.y+0.5);
                g.y = tex2D(ttex, tp.x+0.5, tp.y+0.5);
                sst.update(g);
                */
                rp.x = rs.x + (float)x;
                rp.y = rs.y + (float)y;
                tp = V2M3x3mulV2(H, rp);
                g.x = tex2D(ref_rtex, rp.x + 0.5, rp.y + 0.5);
                g.y = tex2D(ref_ttex, tp.x + 0.5, tp.y + 0.5);
                sst.update(g);
            };
        };

        ///////////////////////////////////////////////////////
        float2 rvx = rb - ra;
        float rdx = size(rvx);
        normalize(rvx);

        float2 rvy = rc - ra;
        float rdy = size(rvy);
        normalize(rvy);

        int nx = 20;
        int ny = 20;

        for(int x = 0; x < nx; x++)
        {
            for(int y = 0; y < ny - x; y++)
            {
                float2 rp = ra + rvx * ((rdx / (float)nx) * (float)x) + rvy * ((rdy / (float)ny) * (float)y);
                float2 tp = V2M3x3mulV2(H, rp);

                float2 g;
                // g.x = 255.0f*tex2D(rtex, rp.x+0.5, rp.y+0.5);
                // g.y = 255.0f*tex2D(ttex, tp.x+0.5, tp.y+0.5);
                g.x = tex2D(ref_rtex, rp.x + 0.5, rp.y + 0.5);
                g.y = tex2D(ref_ttex, tp.x + 0.5, tp.y + 0.5);
                sst.update(g);
            };
        };

        sst.computeSim();
        // coalescent writing

        out[x] = sst.sim;
    };
}

__global__ void patches_kernel(float3* p, float* out, int npts)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;

    if(x < npts)
    {
        // coalescent reading
        float3 P = p[x];
        float2 rp = project3DPoint(rP, P);
        float2 tp = project3DPoint(tP, P);

        ///////////////////////////////////////////////////////
        // compute similarity
        int wsh = 2;
        simStat sst = simStat();
        for(int x = -wsh; x <= wsh; x++)
        {
            for(int y = -wsh; y <= wsh; y++)
            {
                float2 rpp, tpp, g;

                rpp.x = rp.x + (float)x;
                rpp.y = rp.y + (float)y;
                tpp.x = tp.x + (float)x;
                tpp.y = tp.y + (float)y;
                g.x = tex2D(ref_rtex, rpp.x + 0.5, rpp.y + 0.5);
                g.y = tex2D(ref_ttex, tpp.x + 0.5, tpp.y + 0.5);
                sst.update(g);
            };
        };

        sst.computeSim();

        // coalescent writing
        out[x] = sst.sim;
    };
}

/*
__global__ void triangles_kernel(float *out, int out_p, float2 *t, int t_p, int ntris)
{
        int x = blockIdx.x*blockDim.x + threadIdx.x;
        int cra = 0*t_p+x;
        int crb = 1*t_p+x;
        int crc = 2*t_p+x;
        int cta = 3*t_p+x;
        int ctb = 4*t_p+x;
        int ctc = 5*t_p+x;

        if (x<ntris)
        {
                //coalescent reading
                float2 ra = t[cra];
                float2 rb = t[crb];
                float2 rc = t[crc];

                float2 ta = t[cta];
                float2 tb = t[ctb];
                float2 tc = t[ctc];

                ///////////////////////////////////////////////////////
                float2 rvx = rb-ra;
                float  rdx = size(rvx);
                int	    nx = max(5,(int)rdx);
                normalize(rvx);

                float2 rvy = rc-ra;
                float  rdy = size(rvy);
                int	    ny = max(5,(int)rdy);
                normalize(rvy);

                ///////////////////////////////////////////////////////
                float2 tvx = tb-ta;
                float  tdx = size(tvx);
                                nx = min(100,max(nx,max(5,(int)tdx)));
                normalize(tvx);

                float2 tvy = tc-ta;
                float  tdy = size(tvy);
                                ny = min(100,max(ny,max(5,(int)tdy)));
                normalize(tvy);


                ///////////////////////////////////////////////////////
                //some check

                ///////////////////////////////////////////////////////
                //compute similarity
                simStat sst = simStat();

                int m = max(nx,ny);
                nx = m;
                ny = m;

                for (int x=0;x<nx;x++) {

                        for (int y=0;y<ny-x;y++) {
                                float2 rp = ra + rvx * ((rdx/(float)nx)*(float)x)
                                                           + rvy * ((rdy/(float)ny)*(float)y);
                                float2 tp = ta + tvx * ((tdx/(float)nx)*(float)x)
                                                           + tvy * ((tdy/(float)ny)*(float)y);

                                float2 g;
                                //g.x = 255.0f*tex2D(rtex, rp.x+0.5, rp.y+0.5);
                                //g.y = 255.0f*tex2D(ttex, tp.x+0.5, tp.y+0.5);
                                g.x = tex2D(rtex, rp.x+0.5, rp.y+0.5);
                                g.y = tex2D(ttex, tp.x+0.5, tp.y+0.5);
                                sst.update(g);
                        };
                };

                sst.computeSim();
                //coalescent writing

                out[x] = sst.sim;
        };
}
*/

/*
__global__ void triangles_sizes_kernel(int *out, int out_p, float2 *rt, float2 *tt, int t_p, int ntris)
{
        int x = blockIdx.x*blockDim.x + threadIdx.x;
        int ca = 0*t_p+x;
        int cb = 1*t_p+x;
        int cc = 2*t_p+x;

        if (x<ntris)
        {
                //coalescent reading
                float2 ra = rt[ca];
                float2 rb = rt[cb];
                float2 rc = rt[cc];

                float2 ta = tt[ca];
                float2 tb = tt[cb];
                float2 tc = tt[cc];

                ///////////////////////////////////////////////////////
                float2 rvx = rb-ra;
                float  rdx = size(rvx);
                int	    nx = max(5,(int)rdx);

                float2 rvy = rc-ra;
                float  rdy = size(rvy);
                int	    ny = max(5,(int)rdy);

                ///////////////////////////////////////////////////////
                float2 tvx = tb-ta;
                float  tdx = size(tvx);
                                nx = min(100,min(nx,max(5,(int)tdx)));

                float2 tvy = tc-ta;
                float  tdy = size(tvy);
                                ny = min(100,min(ny,max(5,(int)tdy)));

                sst.computeSim();
                //coalescent writing
                out[0*out_p+x] = nx;
                out[1*out_p+x] = ny;
        };
}
*/

#endif // DEVICE_CODE_CU
