// mn MATRIX ADDRESSING: mxy = x*n+y (x-row,y-col), (m-number of rows, n-number of columns)

__device__ float3 M3x3mulV3(const float* M3x3, const float3& V)
{
    return make_float3(M3x3[0] * V.x + M3x3[3] * V.y + M3x3[6] * V.z, M3x3[1] * V.x + M3x3[4] * V.y + M3x3[7] * V.z,
                       M3x3[2] * V.x + M3x3[5] * V.y + M3x3[8] * V.z);
}

__device__ float3 M3x3mulV2(const float* M3x3, const float2& V)
{
    return make_float3(M3x3[0] * V.x + M3x3[3] * V.y + M3x3[6], M3x3[1] * V.x + M3x3[4] * V.y + M3x3[7],
                       M3x3[2] * V.x + M3x3[5] * V.y + M3x3[8]);
}

__device__ float3 M3x4mulV3(const float* M3x4, const float3& V)
{
    return make_float3(M3x4[0] * V.x + M3x4[3] * V.y + M3x4[6] * V.z + M3x4[9],
                       M3x4[1] * V.x + M3x4[4] * V.y + M3x4[7] * V.z + M3x4[10],
                       M3x4[2] * V.x + M3x4[5] * V.y + M3x4[8] * V.z + M3x4[11]);
}

__device__ float2 V2M3x3mulV2(const float* M3x3, const float2& V)
{
    float d = M3x3[2] * V.x + M3x3[5] * V.y + M3x3[8];
    return make_float2((M3x3[0] * V.x + M3x3[3] * V.y + M3x3[6]) / d, (M3x3[1] * V.x + M3x3[4] * V.y + M3x3[7]) / d);
}

__device__ float2 project3DPoint(const float* M3x4, const float3& V)
{
    float3 p = M3x4mulV3(M3x4, V);
    return make_float2(p.x / p.z, p.y / p.z);
}

__device__ void M3x3mulM3x3(const float* O3x3, const float* A3x3, const float* B3x3)
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

__device__ void M3x3minusM3x3(const float* O3x3, const float* A3x3, const float* B3x3)
{
    for(int i = 0; i < 9; i++)
    {
        O3x3[i] = A3x3[i] - B3x3[i];
    };
}

__device__ void M3x3transpose(const float* O3x3, const float* A3x3)
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

__device__ float3 operator*(const float3& a, const float& d)
{
    return make_float3(a.x * d, a.y * d, a.z * d);
}

__device__ float3 operator/(const float3& a, const float& d)
{
    return make_float3(a.x / d, a.y / d, a.z / d);
}

__device__ float3 operator+(const float3& a, const float3& b)
{
    return make_float3(a.x + b.x, a.y + b.y, a.z + b.z);
}

__device__ float2 operator+(const float2& a, const float2& b)
{
    return make_float2(a.x + b.x, a.y + b.y);
}

__device__ float3 operator-(const float3& a, const float3& b)
{
    return make_float3(a.x - b.x, a.y - b.y, a.z - b.z);
}

__device__ float dot(const float3& a, const float3& b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

__device__ float size(const float3& a)
{
    return sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
}

__device__ float size(const float2& a)
{
    return sqrt(a.x * a.x + a.y * a.y);
}

__device__ float dist(const float3& a, const float3& b)
{
    float3 ab = a - b;
    return size(ab);
}

__device__ float dist(const float2& a, const float2& b)
{
    float2 ab;
    ab.x = a.x - b.x;
    ab.y = a.y - b.y;
    return size(ab);
}

__device__ float3 cross(const float3& a, const float3& b)
{
    return make_float3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}
__device__ void normalize(const float3& a)
{
    float d = sqrt(dot(a, a));
    a.x /= d;
    a.y /= d;
    a.z /= d;
}

__device__ void outerMultiply(float* O3x3, const float3& a, const float3& b)
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

__device__ float3 linePlaneIntersect(const float3& linePoint, const float3& lineVect, const float3& planePoint,
                                     const float3& planeNormal)
{
    float k = (dot(planePoint, planeNormal) - dot(planeNormal, linePoint)) / dot(planeNormal, lineVect);
    return linePoint + lineVect * k;
}

__device__ bool checkProjection(const float* H, int x, int y, int width, int height)
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

__device__ float pointLineDistance3DSquared(const float3& point, const float3& linePoint,
                                            const float3& lineVectNormalized)
{
    float3 a = cross(lineVectNormalized, linePoint - point);
    return dot(a, a);
}

__device__ bool isLineInTriangle(const float3& P, const float3& A, const float3& B, const float3& C,
                                 const float3& linePoint, const float3& lineVect)
{
    float3 v0 = C - A;
    float3 v0n = v0;
    normalize(v0n);

    float3 v1 = B - A;
    float3 v1n = v1;
    normalize(v1n);

    float3 n = cross(v0n, v1n);
    normalize(n);

    P = linePlaneIntersect(linePoint, lineVect, A, n);

    // Compute vectors
    float3 v2 = P - A;

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

    // if ((u==0.0f)||(v==0.0f)||(u+v==1.0f)) {
    //	printf("WARNNG : isLineInTriangle - is on border! \n");
    //};

    // Check if point is in triangle
    return (u > 0.0f) && (v > 0.0f) && (u + v < 1.0f);
}

__device__ void test(float3& P, const float3& A, const float3& B, const float3& C, const float3& linePoint,
                     const float3& lineVect)
{
    float3 v0 = C - A;
    float3 v0n = v0;
    normalize(v0n);

    float3 v1 = B - A;
    float3 v1n = v1;
    normalize(v1n);

    float3 n = cross(v0n, v1n);
    normalize(n);

    float k = (dot(A, n) - dot(n, linePoint)) / dot(n, lineVect);
    P = linePoint + lineVect * k;
}
