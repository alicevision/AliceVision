// mn MATRIX ADDRESSING: mxy = x*n+y (x-row,y-col), (m-number of rows, n-number of columns)

__device__ float3& M3x3mulV3(float* M3x3, float3& V)
{
    return make_float3(M3x3[0] * V.x + M3x3[3] * V.y + M3x3[6] * V.z, M3x3[1] * V.x + M3x3[4] * V.y + M3x3[7] * V.z,
                       M3x3[2] * V.x + M3x3[5] * V.y + M3x3[8] * V.z);
}

__device__ float3& M3x3mulV2(float* M3x3, float2& V)
{
    return make_float3(M3x3[0] * V.x + M3x3[3] * V.y + M3x3[6], M3x3[1] * V.x + M3x3[4] * V.y + M3x3[7],
                       M3x3[2] * V.x + M3x3[5] * V.y + M3x3[8]);
}

__device__ float3& M3x4mulV3(float* M3x4, float3& V)
{
    return make_float3(M3x4[0] * V.x + M3x4[3] * V.y + M3x4[6] * V.z + M3x4[9],
                       M3x4[1] * V.x + M3x4[4] * V.y + M3x4[7] * V.z + M3x4[10],
                       M3x4[2] * V.x + M3x4[5] * V.y + M3x4[8] * V.z + M3x4[11]);
}

__device__ float2& V2M3x3mulV2(float* M3x3, float2& V)
{
    float d = M3x3[2] * V.x + M3x3[5] * V.y + M3x3[8];
    return make_float2((M3x3[0] * V.x + M3x3[3] * V.y + M3x3[6]) / d, (M3x3[1] * V.x + M3x3[4] * V.y + M3x3[7]) / d);
}

__device__ float2& project3DPoint(float* M3x4, float3& V)
{
    float3 p = M3x4mulV3(M3x4, V);
    return make_float2(p.x / p.z, p.y / p.z);
}

__device__ void M3x3mulM3x3(float* O3x3, float* A3x3, float* B3x3)
{
    O3x3[0] = A3x3[0] * B3x3[0] + A3x3[3] * B3x3[1] + A3x3[6] * B3x3[2];
    O3x3[3] = A3x3[0] * B3x3[3] + A3x3[3] * B3x3[4] + A3x3[6] * B3x3[5];
    O3x3[6] = A3x3[0] * B3x3[6] + A3x3[3] * B3x3[7] + A3x3[6] * B3x3[8];

    O3x3[1] = A3x3[1] * B3x3[0] + A3x3[4] * B3x3[1] + A3x3[7] * B3x3[2];
    O3x3[4] = A3x3[1] * B3x3[3] + A3x3[4] * B3x3[4] + A3x3[7] * B3x3[5];
    O3x3[7] = A3x3[1] * B3x3[6] + A3x3[4] * B3x3[7] + A3x3[7] * B3x3[8];

    O3x3[2] = A3x3[2] * B3x3[0] + A3x3[5] * B3x3[1] + A3x3[8] * B3x3[2];
    O3x3[5] = A3x3[2] * B3x3[3] + A3x3[5] * B3x3[4] + A3x3[8] * B3x3[5];
    O3x3[8] = A3x3[2] * B3x3[6] + A3x3[5] * B3x3[7] + A3x3[8] * B3x3[8];
}

__device__ void M3x3minusM3x3(float* O3x3, float* A3x3, float* B3x3)
{
    for(int i = 0; i < 9; i++)
    {
        O3x3[i] = A3x3[i] - B3x3[i];
    };
}

__device__ void M3x3transpose(float* O3x3, float* A3x3)
{
    O3x3[0] = A3x3[0];
    O3x3[1] = A3x3[3];
    O3x3[2] = A3x3[6];
    O3x3[3] = A3x3[1];
    O3x3[4] = A3x3[4];
    O3x3[5] = A3x3[7];
    O3x3[6] = A3x3[2];
    O3x3[7] = A3x3[5];
    O3x3[8] = A3x3[8];
}

__device__ float3& operator*(float3& a, float& d)
{
    return make_float3(a.x * d, a.y * d, a.z * d);
}

__device__ float3& operator/(float3& a, float d)
{
    return make_float3(a.x / d, a.y / d, a.z / d);
}

__device__ float3& operator+(float3& a, float3& b)
{
    return make_float3(a.x + b.x, a.y + b.y, a.z + b.z);
}

__device__ float3& operator-(float3& a, float3& b)
{
    return make_float3(a.x - b.x, a.y - b.y, a.z - b.z);
}

__device__ float2& operator+(float2& a, float2& b)
{
    return make_float2(a.x + b.x, a.y + b.y);
}

__device__ float2& operator-(float2& a, float2& b)
{
    return make_float2(a.x - b.x, a.y - b.y);
}

__device__ float2& operator*(float2& a, float d)
{
    return make_float2(a.x * d, a.y * d);
}

__device__ float dot(float3& a, float3& b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

__device__ float dot(float2& a, float2& b)
{
    return a.x * b.x + a.y * b.y;
}

__device__ float size(float3& a)
{
    return sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
}

__device__ float size(float2& a)
{
    return sqrt(a.x * a.x + a.y * a.y);
}

__device__ float dist(float3& a, float3& b)
{
    float3 ab = a - b;
    return size(ab);
}

__device__ float dist(float2& a, float2& b)
{
    float2 ab;
    ab.x = a.x - b.x;
    ab.y = a.y - b.y;
    return size(ab);
}

__device__ float3& cross(float3& a, float3& b)
{
    return make_float3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}
__device__ void normalize(float3& a)
{
    float d = sqrt(dot(a, a));
    a.x /= d;
    a.y /= d;
    a.z /= d;
}

__device__ void normalize(float2& a)
{
    float d = sqrt(dot(a, a));
    a.x /= d;
    a.y /= d;
}

__device__ void outerMultiply(float* O3x3, float3& a, float3& b)
{
    O3x3[0] = a.x * b.x;
    O3x3[3] = a.x * b.y;
    O3x3[6] = a.x * b.z;
    O3x3[1] = a.y * b.x;
    O3x3[4] = a.y * b.y;
    O3x3[7] = a.y * b.z;
    O3x3[2] = a.z * b.x;
    O3x3[5] = a.z * b.y;
    O3x3[8] = a.z * b.z;
}

__device__ float3 linePlaneIntersect(float3& linePoint, float3& lineVect, float3& planePoint, float3& planeNormal)
{
    float k = (dot(planePoint, planeNormal) - dot(planeNormal, linePoint)) / dot(planeNormal, lineVect);
    return linePoint + lineVect * k;
}

__device__ float3 lineLineIntersect(float3& p1, float3& p2, float3& p3, float3& p4)
{
    /*
    %  [pa, pb, mua, mub] = LineLineIntersect(p1,p2,p3,p4)
    %
    %   Calculates the line segment pa_pb that is the shortest route
    %   between two lines p1_p2 and p3_p4. Calculates also the values of
    %   mua and mub where
    %        pa = p1 + mua (p2 - p1)
    %        pb = p3 + mub (p4 - p3)
    %
    %   Returns a MATLAB error if no solution exists.
    %
    %   This a simple conversion to MATLAB of the C code posted by Paul
    %   Bourke at
    %   http://astronomy.swin.edu.au/~pbourke/geometry/lineline3d/. The
    %   author of this all too imperfect translation is Cristian Dima
    %   (csd@cmu.edu)
    */

    float d1343, d4321, d1321, d4343, d2121, denom, numer, p13[3], p43[3], p21[3], pa[3], pb[3], muab[2];
    float3 S;

    p13[0] = p1.x - p3.x;
    p13[1] = p1.y - p3.y;
    p13[2] = p1.z - p3.z;

    p43[0] = p4.x - p3.x;
    p43[1] = p4.y - p3.y;
    p43[2] = p4.z - p3.z;

    /*
    if ((abs(p43[0])  < eps) & ...
        (abs(p43[1])  < eps) & ...
        (abs(p43[2])  < eps))
      error('Could not compute LineLineIntersect!');
    end
    */

    p21[0] = p2.x - p1.x;
    p21[1] = p2.y - p1.y;
    p21[2] = p2.z - p1.z;

    /*
    if ((abs(p21[0])  < eps) & ...
        (abs(p21[1])  < eps) & ...
        (abs(p21[2])  < eps))
      error('Could not compute LineLineIntersect!');
    end
    */

    d1343 = p13[0] * p43[0] + p13[1] * p43[1] + p13[2] * p43[2];
    d4321 = p43[0] * p21[0] + p43[1] * p21[1] + p43[2] * p21[2];
    d1321 = p13[0] * p21[0] + p13[1] * p21[1] + p13[2] * p21[2];
    d4343 = p43[0] * p43[0] + p43[1] * p43[1] + p43[2] * p43[2];
    d2121 = p21[0] * p21[0] + p21[1] * p21[1] + p21[2] * p21[2];

    denom = d2121 * d4343 - d4321 * d4321;

    /*
    if (abs(denom) < eps)
      error('Could not compute LineLineIntersect!');
    end
     */

    numer = d1343 * d4321 - d1321 * d4343;

    muab[0] = numer / denom;
    muab[1] = (d1343 + d4321 * muab[0]) / d4343;

    pa[0] = p1.x + muab[0] * p21[0];
    pa[1] = p1.y + muab[0] * p21[1];
    pa[2] = p1.z + muab[0] * p21[2];

    pb[0] = p3.x + muab[1] * p43[0];
    pb[1] = p3.y + muab[1] * p43[1];
    pb[2] = p3.z + muab[1] * p43[2];

    S.x = (pa[0] + pb[0]) / 2.0;
    S.y = (pa[1] + pb[1]) / 2.0;
    S.z = (pa[2] + pb[2]) / 2.0;

    return S;
}

__device__ bool checkProjection(float* H, int x, int y, int width, int height)
{
    float xx = x + 0.5f;
    float yy = y + 0.5f;
    float3 v = make_float3(xx, yy, 1.0f);
    float3 p = M3x3mulV3(H, v);
    p.x /= p.z;
    p.y /= p.z;
    p.z /= p.z;
    return ((p.x >= 0.0f) && (p.x < (float)width) && (p.y >= 0.0f) && (p.y < (float)height));
}

__device__ float3 triangulate(float2& rpix, float2& tpix)
{
    float3 rp = make_float3(rpix.x, rpix.y, 1.0f);
    float3 refvect = M3x3mulV3(riP, rp);
    normalize(refvect);

    float3 tp = make_float3(tpix.x, tpix.y, 1.0f);
    float3 tarvect = M3x3mulV3(tiP, tp);
    normalize(tarvect);

    float3 refpoint = refvect + rC;
    float3 tarpoint = tarvect + tC;

    return lineLineIntersect(rC, refpoint, tC, tarpoint);
}

__device__ void computeHomography(float* _H, float3& _p, float3& _n)
{
    // hartley zisserman second edition p.327 (13.2)
    float3 _tl = make_float3(0.0, 0.0, 0.0) - M3x3mulV3(rR, rC);
    float3 _tr = make_float3(0.0, 0.0, 0.0) - M3x3mulV3(tR, tC);

    float3 p = M3x3mulV3(rR, (_p - rC));
    float3 n = M3x3mulV3(rR, _n);
    normalize(n);
    float d = -dot(n, p);

    float RrT[9];
    M3x3transpose(RrT, rR);

    float tmpRr[9];
    M3x3mulM3x3(tmpRr, tR, RrT);
    float3 tr = _tr - M3x3mulV3(tmpRr, _tl);

    float tmp[9];
    float tmp1[9];
    outerMultiply(tmp, tr, n / d);
    M3x3minusM3x3(tmp, tmpRr, tmp);
    M3x3mulM3x3(tmp1, tK, tmp);
    M3x3mulM3x3(tmp, tmp1, riK);

    for(int i = 0; i < 9; i++)
    {
        _H[i] = tmp[i];
    };
}
