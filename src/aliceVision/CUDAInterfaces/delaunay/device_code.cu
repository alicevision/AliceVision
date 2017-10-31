#ifndef DEVICE_CODE_CU
#define DEVICE_CODE_CU

__global__ void delaunay_triangles_kernel(float* out, float3* A, float3* B, float3* C, int pt, int ntris, int nlines,
                                          int ifrom)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    int id = y * pt + x;

    if(id < ntris)
    {
        float3 a = A[id];
        float3 b = B[id];
        float3 c = C[id];
        float o = out[id];

        float3 v0 = c - a;
        float3 v0n = v0;
        normalize(v0n);

        float3 v1 = b - a;
        float3 v1n = v1;
        normalize(v1n);

        float3 n = cross(v0n, v1n);
        normalize(n);

        float an = dot(a, n);

        for(int i = 0; i < 1000; i++)
        {
            float3 Cam = g_cam[i];
            float3 Pnt = g_pnt[i];

            float3 dir = Cam - Pnt;
            normalize(dir);

            float uu = an - dot(n, Cam);
            float dd = dot(n, dir);
            float kk = uu / dd;
            float3 P = Cam + dir * kk;

            // Compute vectors
            float3 v2 = P - a;

            // Compute dot products
            float dot00 = dot(v0, v0);
            float dot01 = dot(v0, v1);
            float dot02 = dot(v0, v2);
            float dot11 = dot(v1, v1);
            float dot12 = dot(v1, v2);

            // Compute barycentric coordinates
            float invDenom = 1.0 / (dot00 * dot11 - dot01 * dot01);
            float u = (dot11 * dot02 - dot01 * dot12) * invDenom;
            float v = (dot00 * dot12 - dot01 * dot02) * invDenom;

            // Check if point is in triangle
            o += (float)((u > 0.0f) && (v > 0.0f) && (u + v < 1.0f));
        };

        out[id] = o;
    };
}

__global__ void delaunay_tetrahedrons_kernel(float* out, int out_p, float3* A, float3* B, float3* C, float3* D,
                                             float3 cam, float3 point, int ntetrs)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    int id = y * out_p + x;

    if(id < ntetrs)
    {
        float3 a = A[id];
        float3 b = B[id];
        float3 c = C[id];
        float3 d = D[id];
        float3 P;
        float3 dir = cam - point;
        normalize(dir);
        float o = 0.0f;
        int ba = (int)isLineInTriangle(P, a, b, c, cam, dir);
        int bb = (int)isLineInTriangle(P, a, b, d, cam, dir);
        int bc = (int)isLineInTriangle(P, a, c, d, cam, dir);
        int bd = (int)isLineInTriangle(P, b, c, d, cam, dir);

        if(ba + bb + bc + bd > 0)
        {
            o = 1.0f;
        }
        else
        {
            o = 0.0f;
        };

        out[id] = o;
    };
}

__global__ void delaunay_null_kernel(float* out, int out_p, int n)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    int id = y * out_p + x;
    if(id < n)
    {
        out[id] = 0.0f;
    };
}

__global__ void delaunay_add_kernel(float* res, float* src, int p, int n)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    int id = y * p + x;
    if(id < n)
    {
        res[id] = res[id] + src[id];
    };
}

#endif // DEVICE_CODE_CU
