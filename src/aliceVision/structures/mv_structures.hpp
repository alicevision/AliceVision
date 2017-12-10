// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "mv_matrix.hpp"
#include "mv_point2d.hpp"
#include "mv_point3d.hpp"
#include "mv_color.hpp"
#include "mv_staticVector.hpp"


#include <queue>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct pixel
{
    union {
        struct
        {
            int x, y;
        };
        int m[2];
    };

    int& operator[](const int index) { return m[index]; }

    pixel()
    {
        x = 0;
        y = 0;
    }

    pixel(const point2d& p)
    {

        x = (int)floor(p.x + 0.5);
        y = (int)floor(p.y + 0.5);
    }

    pixel(const int _x, const int _y)
    {
        x = _x;
        y = _y;
    }

    inline pixel& operator=(const pixel& param)
    {
        x = param.x;
        y = param.y;
        return *this;
    }

    inline bool operator==(const pixel& param) { return ((x == param.x) && (y == param.y)); }

    inline pixel operator-(const pixel& _p) { return pixel(x - _p.x, y - _p.y); }

    inline pixel operator+(const pixel& _p) { return pixel(x + _p.x, y + _p.y); }

    inline pixel operator*(const int& d) { return pixel(x * d, y * d); }

    pixel operator/(int d)
    {
        if(d == 0)
        {
            return pixel(0, 0);
        }
        else
        {
            pixel p;
            p.x = (int)floor((float)x / (float)d + 0.5);
            p.y = (int)floor((float)y / (float)d + 0.5);
            return p;
        };
    }

    friend int dot(const pixel& p1, const pixel& p2) { return p1.x * p2.x + p1.y * p2.y; }

    inline float size() { return sqrt((float)(x * x + y * y)); }
};

template <class T>
class match
{
public:
    T ref;
    T tar;

    match(){}

    match(const T& _ref, const T& _tar)
    {
        ref = _ref;
        tar = _tar;
    }

    inline match& operator=(const match& param)
    {
        ref = param.ref;
        tar = param.tar;
        return *this;
    }
};

struct voxel
{
    union {
        struct
        {
            int x, y, z;
        };
        int m[3];
    };

    voxel()
    {
        x = 0;
        y = 0;
        z = 0;
    }

    voxel(const int _x, const int _y, const int _z)
    {
        x = _x;
        y = _y;
        z = _z;
    }

    int& operator[](const int index) { return m[index]; }

    voxel& operator=(const voxel& param)
    {
        x = param.x;
        y = param.y;
        z = param.z;
        return *this;
    }

    voxel operator-(const voxel& _p) const
    {
        voxel p;
        p.x = x - _p.x;
        p.y = y - _p.y;
        p.z = z - _p.z;

        return p;
    }

    voxel operator+(const voxel& _p) const
    {
        voxel p;
        p.x = x + _p.x;
        p.y = y + _p.y;
        p.z = z + _p.z;

        return p;
    }

    voxel operator+(int _p) const
    {
        voxel p;
        p.x = x + _p;
        p.y = y + _p;
        p.z = z + _p;

        return p;
    }

    voxel operator*(const voxel& _p) const
    {
        voxel p;
        p.x = x * _p.x;
        p.y = y * _p.y;
        p.z = z * _p.z;

        return p;
    }

    voxel operator*(int d) const
    {
        voxel p;
        p.x = x * d;
        p.y = y * d;
        p.z = z * d;

        return p;
    }

    voxel operator/(int d) const
    {
        if(d == 0)
            return voxel(0, 0, 0);

        voxel p;
        p.x = (int)floor((float)x / (float)d + 0.5);
        p.y = (int)floor((float)y / (float)d + 0.5);
        p.z = (int)floor((float)z / (float)d + 0.5);
        return p;
    }

    float size() const
    {
        float d = (float)(x * x + y * y + z * z);
        if(d == 0.0f)
            return 0.0f;
        return sqrt(d);
    }

    bool operator==(const voxel& param) const { return (x == param.x) && (y == param.y) && (z == param.z); }

    bool operator!=(const voxel& param) const { return (x != param.x) || (y != param.y) || (z != param.z); }
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class point4d
{
public:
    union {
        struct
        {
            float x, y, z, w;
        };
        float m[4];
    };

    point4d()
    {
        x = 0.0;
        y = 0.0;
        z = 0.0;
        w = 0.0;
    }

    point4d(const float _x, const float _y, const float _z, const float _w)
    {
        x = _x;
        y = _y;
        z = _z;
        w = _w;
    }

    point4d& operator=(const point4d& param)
    {
        x = param.x;
        y = param.y;
        z = param.z;
        w = param.w;
        return *this;
    }

    bool operator==(const point4d& param) const
    {
        return (x == param.x) && (y == param.y) && (z == param.z) && (w == param.w);
    }

    inline point4d operator-(const point4d& _p) const { return point4d(x - _p.x, y - _p.y, z - _p.z, w - _p.w); }

    inline point4d operator-() const { return point4d(-x, -y, -z, -w); }

    inline point4d operator+(const point4d& _p) const { return point4d(x + _p.x, y + _p.y, z + _p.z, w + _p.w); }

    inline point4d operator*(const float d) const { return point4d(x * d, y * d, z * d, w * d); }

    inline point4d operator/(const float d) const { return point4d(x / d, y / d, z / d, w / d); }

    point4d normalize() const
    {
        float d = sqrt(x * x + y * y + z * z + w * w);
        return point4d(x / d, y / d, z / d, w / d);
    }

    friend float dot(const point4d& p1, const point4d& p2)
    {
        return p1.x * p2.x + p1.y * p2.y + p1.z * p2.z + p1.w * p2.w;
    }

    friend point4d proj(point4d& e, point4d& a) { return e * (dot(e, a) / dot(e, e)); }

    float size() const
    {
        float d = x * x + y * y + z * z + w * w;
        if(d == 0.0f)
            return 0.0f;

        return sqrt(d);
    }

    void doprintf() const { printf("%f %f %f %f\n", x, y, z, w); }

    void saveToFile(const std::string& fileName) const
    {
        FILE* f = fopen(fileName.c_str(), "w");
        fprintf(f, "%f %f %f %f", x, y, z, w);
        fclose(f);
    }

    void loadFromFile(const std::string& fileName)
    {
        FILE* f = fopen(fileName.c_str(), "r");
        fscanf(f, "%f %f %f %f", &x, &y, &z, &w);
        fclose(f);
    }
};

matrix2x3 DPIP(const matrix3x4& M, const point3d& P);
point2d operator*(const matrix2x3& M, const point3d& _p);
point3d operator*(const matrix3x3& M, const point2d& _p);
point2d operator^(const matrix3x3& M, const point2d& _p);
matrix3x4 operator|(const matrix3x3& M, const point3d& p);
matrix3x4 operator*(const matrix3x3& M1, const matrix3x4& M2);
matrix3x3 outerMultiply(const point3d& a, const point3d& b);
point3d operator*(const matrix4x4& M, const point3d& _p);
point3d operator*(const matrix3x3& M, const point3d& _p);
point3d operator*(const matrix3x4& M, const point3d& _p);
point3d mldivide(const matrix3x3& M, const point3d& b);
point2d operator*(const matrix2x2& M, const point2d& _p);
point2d mldivide(const matrix2x2& M, const point2d& b);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct rotation
{
    unsigned char rx; // 1-Byte : 0 to 255
    unsigned char ry; // 1-Byte : 0 to 255

    rotation()
    {
        rx = 90;
        ry = 90;
    }

    rotation(const unsigned char _rx, const unsigned char _ry)
    {
        rx = _rx;
        ry = _ry;
    }

    rotation& operator=(const rotation& param)
    {
        rx = param.rx;
        ry = param.ry;
        return *this;
    }

    friend float size(const rotation& _r1, const rotation& _r2)
    {
        point2d r1 = point2d((float)_r1.rx, (float)_r1.ry);
        point2d r2 = point2d((float)_r2.rx, (float)_r2.ry);
        return (r1 - r2).size();
    }
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct orientedPoint
{
    point3d p; // 3 * float : 3 * 4 = 12 Bytes : (one float is 4 Bytes : 3.4E +/- 38 (7 digits) )
    point3d n; // 3 * float : 3 * 4 = 12  Bytes
    float sim; // 4-Bytes : 3.4E +/- 38 (7 digits)
    // TOTAL: 12 + 12 + 4 = 28 Bytes

    orientedPoint()
    {
        p = point3d();
        n = point3d();
        sim = 1.0f;
    }

    orientedPoint(const point3d& _p, const point3d& _n, float _sim)
    {
        sim = _sim;
        p = _p;
        n = _n;
    }

    orientedPoint& operator=(const orientedPoint& param)
    {
        p = param.p;
        n = param.n;
        sim = param.sim;
        return *this;
    }

    bool operator>(const orientedPoint& param) const { return (sim > param.sim); }
};

class orientedPoint_compare_ped
{
public:
    bool operator()(const orientedPoint& a, const orientedPoint& b) const { return a > b; }
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class seedPointCams
{
public:
    static const int SP_NCAMS = 20;

    unsigned short cams[SP_NCAMS];
    point2d shifts[SP_NCAMS];
    int n;

    seedPointCams() { n = 0; }

    ~seedPointCams(){}

    inline void push_back(unsigned short cam)
    {
        if(n == SP_NCAMS - 1)
        {
            // printf("push_back too many cams\n");
            // exit(1);
        }
        else
        {
            cams[n] = cam;
            shifts[n] = point2d(0.0f, 0.0f);
            n++;
        }
    }

    inline int size() const { return n; }

    inline void resize(int newSize) { n = newSize; }

    inline void reserve(int newSize)
    {
        if(SP_NCAMS < newSize)
        {
            printf("reserve too many cams\n");
            // exit(1);
        }
    }

    inline int indexOf(int what)
    {
        int isthereindex = -1;

        int i = 0;
        while((i < n) && (isthereindex == -1))
        {
            if(cams[i] == what)
            {
                isthereindex = i;
            };
            i++;
        };

        return isthereindex;
    }

    inline const unsigned short& operator[](int index) const { return cams[index]; }

    inline unsigned short& operator[](int index) { return cams[index]; }
};

class seedPoint
{
public:
    orientedPoint op; // 18 Bytes
    point3d xax;
    point3d yax;
    float pixSize;       // 4 bytes
    unsigned long area;  // 4 bytes
    unsigned long segId; // 4-Bytes : 0 to 4,294,967,295
    seedPointCams cams;  // n * 2
    // std::vector<unsigned short> cams;
    // TOTAL: floating

    seedPoint()
    {
        op = orientedPoint();
        pixSize = 0.0;
        area = 0;
    }

    ~seedPoint() { cams.resize(0); }

    seedPoint& operator=(const seedPoint& param)
    {
        op = param.op;
        xax = param.xax;
        yax = param.yax;

        cams.resize(0);
        for(int i = 0; i < param.cams.size(); i++)
        {
            cams.push_back(param.cams[i]);
        };
        for(int i = 0; i < param.cams.size() + 1; i++)
        {
            cams.shifts[i] = param.cams.shifts[i];
        };

        area = param.area;
        pixSize = param.pixSize;
        segId = param.segId;
        return *this;
    }

    bool operator>(const seedPoint& param) const { return (op.sim > param.op.sim); }
    bool isThereCam(int c)
    {
        bool isThere = false;
        int i = 0;
        while((i < (int)cams.size()) && (isThere == false))
        {
            if(cams[i] == c)
            {
                isThere = true;
            };
            i++;
        };

        return isThere;
    }
};

class seedPoint_compare_ped
{
public:
    bool operator()(const seedPoint& a, const seedPoint& b) { return a > b; }
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
struct scenePoint
{
        orientedPoint op;				//12-Bytes
        unsigned short refImgFileId;	//2-Bytes : 0 to 65,535
        unsigned long uniqueId;			//4-Bytes : 0 to 4,294,967,295
        unsigned long seedId;			//4-Bytes : 0 to 4,294,967,295
                                                                        //TOTAL 22-Bytes

        scenePoint()
        {
                op = orientedPoint();
                refImgFileId = 0;
                //uniqueId = 0;
                seedId = 0;
        };

        scenePoint& operator = (const scenePoint &param)
        {
                op = param.op;
                refImgFileId = param.refImgFileId;
                //uniqueId = param.uniqueId;
                seedId = param.seedId;
                return *this;
        };
};

class scenePoint_compare_ped{
public:
        bool operator ()(const scenePoint & a, const scenePoint & b){
                return a.op.sim > b.op.sim;
        };
};

class scenePoint_compare_pedPtr{
public:
        bool operator ()(const scenePoint *a, const scenePoint *b){
                return (*a).op.sim > (*b).op.sim;
        };
};

*/
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct imageParams
{

    int width, height, im_size;
    imageParams(int _width, int _height)
    : width(_width)
    , height(_height)
    , im_size(_width*_height)
    {}

    imageParams& operator=(const imageParams& param)
    {
        width = param.width;
        height = param.height;
        im_size = param.im_size;
        return *this;
    }
};


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <class Type, class Container = std::vector<Type>, class Compare = std::less<typename Container::value_type> >
class my_priority_queue : public std::priority_queue<Type, Container, Compare>
{
private:
public:
    typedef Type value_type;
    void reserve(int size) { this->c.reserve(size); }
    int capacity() { return this->c.capacity(); }
    value_type operator[](const int id) { return this->c[id]; }
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct sortedId
{
    int id;
    float value;

    sortedId(){}

    sortedId(int _id, float _value)
    {
        id = _id;
        value = _value;
    }

    sortedId& operator=(const sortedId& param)
    {
        id = param.id;
        value = param.value;
        return *this;
    }

    bool operator>(const sortedId& param) const { return (value > param.value); }

    bool operator<(const sortedId& param) const { return (value < param.value); }
};

class sortedId_compare_ped
{
public:
    bool operator()(const sortedId& a, const sortedId& b) { return a > b; }
};

int qSortCompareFloatAsc(const void* ia, const void* ib);
int qSortCompareIntAsc(const void* ia, const void* ib);
int qSortCompareFloatAsc(const void* ia, const void* ib);
int qSortCompareIntDesc(const void* ia, const void* ib);
int compareSortedId(const void* ia, const void* ib);
int qsortCompareSortedIdDesc(const void* ia, const void* ib);
int qsortCompareSortedIdAsc(const void* ia, const void* ib);
int qsortCompareSortedIdByIdAsc(const void* ia, const void* ib);
int qsortCompareScenePointPtrBySimAsc(const void* ia, const void* ib);
int qSortCompareVoxelByZDesc(const void* ia, const void* ib);
int qSortCompareVoxelByXAsc(const void* ia, const void* ib);
int qSortCompareVoxelByYAsc(const void* ia, const void* ib);
int qSortCompareVoxelByZAsc(const void* ia, const void* ib);
int qSortComparePixelByXDesc(const void* ia, const void* ib);
int qSortComparePixelByXAsc(const void* ia, const void* ib);
int qSortComparePixelByYAsc(const void* ia, const void* ib);
int qSortComparePixelByYDesc(const void* ia, const void* ib);
int qSortComparePoint3dByXAsc(const void* ia, const void* ib);
int qSortComparePoint3dByYAsc(const void* ia, const void* ib);
int qSortComparePoint3dByZAsc(const void* ia, const void* ib);
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct simStat
{
    double xsum;
    double ysum;
    double xxsum;
    double yysum;
    double xysum;
    double count;
    double sim;

    double censuslMidVal;
    double censusrMidVal;
    int censusSum;

    simStat()
    {
        xsum = 0.0;
        ysum = 0.0;
        xxsum = 0.0;
        yysum = 0.0;
        xysum = 0.0;
        count = 0;
        sim = 1.0;

        censuslMidVal = 0.0;
        censusrMidVal = 0.0;
        censusSum = 0;
    }

    void set(simStat* p)
    {
        xxsum = p->xxsum;
        yysum = p->yysum;
        xysum = p->xysum;
        xsum = p->xsum;
        ysum = p->ysum;
        count = p->count;
    }

    void add(simStat* p)
    {
        xxsum += p->xxsum;
        yysum += p->yysum;
        xysum += p->xysum;
        xsum += p->xsum;
        ysum += p->ysum;
        count += p->count;
    }

    void diff(simStat* p)
    {
        xxsum -= p->xxsum;
        yysum -= p->yysum;
        xysum -= p->xysum;
        xsum -= p->xsum;
        ysum -= p->ysum;
        count -= p->count;
    }

    simStat& operator=(const simStat& param)
    {
        xxsum = param.xxsum;
        yysum = param.yysum;
        xysum = param.xysum;
        xsum = param.xsum;
        ysum = param.ysum;
        count = param.count;
        return *this;
    }

    simStat operator-(const simStat& p)
    {
        simStat m;
        m.xxsum = xxsum - p.xxsum;
        m.yysum = yysum - p.yysum;
        m.xysum = xysum - p.xysum;
        m.xsum = xsum - p.xsum;
        m.ysum = ysum - p.ysum;
        m.count = count - p.count;

        return m;
    }

    simStat operator+(const simStat& p)
    {
        simStat m;
        m.xxsum = xxsum + p.xxsum;
        m.yysum = yysum + p.yysum;
        m.xysum = xysum + p.xysum;
        m.xsum = xsum + p.xsum;
        m.ysum = ysum + p.ysum;
        m.count = count + p.count;

        return m;
    }

    point2d getEigenValues()
    {
        double a = xxsum / count - (xsum * xsum) / (count * count);
        double b = xysum / count - (xsum * ysum) / (count * count);
        double c = yysum / count - (ysum * ysum) / (count * count);
        double v1 = (c + a + sqrt(c * c - 2.0f * a * c + a * a + 4.0f * b * b)) / 2.0f;
        double v2 = (c + a - sqrt(c * c - 2.0f * a * c + a * a + 4.0f * b * b)) / 2.0f;
        point2d out;
        out.x = (float)v1;
        out.y = (float)v2;
        if(v1 > v2)
        {
            out.x = (float)v2;
            out.y = (float)v1;
        };
        return out;
    }

    void getEigenVectors(point2d& n1, point2d& n2)
    {
        point2d ev = getEigenValues();
        n1.x = (float)(-(xysum / count - (xsum * ysum) / (count * count)));
        n1.y = (float)(xxsum / count - (xsum * xsum) / (count * count) - ev.x);
        n2.x = (float)(-(xysum / count - (xsum * ysum) / (count * count)));
        n2.y = (float)(xxsum / count - (xsum * xsum) / (count * count) - ev.y);
        n1 = n1.normalize();
        n2 = n2.normalize();
    }

    void update(float g[2])
    {
        count += 1;
        xsum += (double)g[0];
        ysum += (double)g[1];
        xxsum += (double)g[0] * (double)g[0];
        yysum += (double)g[1] * (double)g[1];
        xysum += (double)g[0] * (double)g[1];
    }

    void update(float g0, float g1)
    {
        float g[2];
        g[0] = g0;
        g[1] = g1;
        update(g);
    }

    point2d getCG() { return point2d((float)(xsum / count), (float)(ysum / count)); }
};

struct stat3d
{
    double xsum;
    double ysum;
    double zsum;
    double xxsum;
    double yysum;
    double zzsum;
    double xysum;
    double xzsum;
    double yzsum;
    int count;

    double A[3][3], V[3][3], d[3], X[4], vx[3], vy[3], vz[3];

    stat3d()
    {
        xsum = 0.0;
        ysum = 0.0;
        zsum = 0.0;
        xxsum = 0.0;
        yysum = 0.0;
        zzsum = 0.0;
        xysum = 0.0;
        xzsum = 0.0;
        yzsum = 0.0;
        count = 0;
    }

    void update(point3d* p)
    {
        xxsum += (double)p->x * (double)p->x;
        yysum += (double)p->y * (double)p->y;
        zzsum += (double)p->z * (double)p->z;
        xysum += (double)p->x * (double)p->y;
        xzsum += (double)p->x * (double)p->z;
        yzsum += (double)p->y * (double)p->z;
        xsum += (double)p->x;
        ysum += (double)p->y;
        zsum += (double)p->z;
        count += 1;
    }

    void add(stat3d* p)
    {
        xxsum += p->xxsum;
        yysum += p->yysum;
        zzsum += p->zzsum;
        xysum += p->xysum;
        xzsum += p->xzsum;
        yzsum += p->yzsum;
        xsum += p->xsum;
        ysum += p->ysum;
        zsum += p->zsum;
        count += p->count;
    }

    double isPlanePerc()
    {
        if(count < 3.0)
        {
            return 0;
        };

        A[0][0] = xxsum / (double)count - (xsum * xsum) / (double)(count * count);
        A[0][1] = xysum / (double)count - (xsum * ysum) / (double)(count * count);
        A[0][2] = xzsum / (double)count - (xsum * zsum) / (double)(count * count);
        A[1][0] = xysum / (double)count - (ysum * xsum) / (double)(count * count);
        A[1][1] = yysum / (double)count - (ysum * ysum) / (double)(count * count);
        A[1][2] = yzsum / (double)count - (ysum * zsum) / (double)(count * count);
        A[2][0] = xzsum / (double)count - (zsum * xsum) / (double)(count * count);
        A[2][1] = yzsum / (double)count - (zsum * ysum) / (double)(count * count);
        A[2][2] = zzsum / (double)count - (zsum * zsum) / (double)(count * count);

        // should be sorted
        eigen_decomposition(A, V[0], V[1], V[2], d);

        // return   (((d[0]/(d[1]/100.0))<5.0)&&((d[1]/(d[2]/100.0))>25.0));
        return (d[0] / (d[1] / 100.0));
    }

    point3d getBiggestEigenVector()
    {
        if(count < 3.0)
        {
            return point3d();
        };

        A[0][0] = xxsum / (double)count - (xsum * xsum) / (double)(count * count);
        A[0][1] = xysum / (double)count - (xsum * ysum) / (double)(count * count);
        A[0][2] = xzsum / (double)count - (xsum * zsum) / (double)(count * count);
        A[1][0] = xysum / (double)count - (ysum * xsum) / (double)(count * count);
        A[1][1] = yysum / (double)count - (ysum * ysum) / (double)(count * count);
        A[1][2] = yzsum / (double)count - (ysum * zsum) / (double)(count * count);
        A[2][0] = xzsum / (double)count - (zsum * xsum) / (double)(count * count);
        A[2][1] = yzsum / (double)count - (zsum * ysum) / (double)(count * count);
        A[2][2] = zzsum / (double)count - (zsum * zsum) / (double)(count * count);

        // should be sorted
        eigen_decomposition(A, V[0], V[1], V[2], d);

        return point3d((float)V[0][2], (float)V[1][2], (float)V[2][2]).normalize();
    }

    void getEigenVectorsDesc(point3d& cg, point3d& v1, point3d& v2, point3d& v3, float& d1, float& d2, float& d3)
    {
        float xmean = xsum / (double)count;
        float ymean = ysum / (double)count;
        float zmean = zsum / (double)count;

        A[0][0] = (xxsum - xsum * xmean - xsum * xmean + xmean * xmean * (double)count) / (double)(count);
        A[0][1] = (xysum - ysum * xmean - xsum * ymean + xmean * ymean * (double)count) / (double)(count);
        A[0][2] = (xzsum - zsum * xmean - xsum * zmean + xmean * zmean * (double)count) / (double)(count);
        A[1][0] = (xysum - xsum * ymean - ysum * xmean + ymean * xmean * (double)count) / (double)(count);
        A[1][1] = (yysum - ysum * ymean - ysum * ymean + ymean * ymean * (double)count) / (double)(count);
        A[1][2] = (yzsum - zsum * ymean - ysum * zmean + ymean * zmean * (double)count) / (double)(count);
        A[2][0] = (xzsum - xsum * zmean - zsum * xmean + zmean * xmean * (double)count) / (double)(count);
        A[2][1] = (yzsum - ysum * zmean - zsum * ymean + zmean * ymean * (double)count) / (double)(count);
        A[2][2] = (zzsum - zsum * zmean - zsum * zmean + zmean * zmean * (double)count) / (double)(count);

        /*
        A[0][0] = xxsum;
        A[0][1] = xysum;
        A[0][2] = xzsum;
        A[1][0] = xysum;
        A[1][1] = yysum;
        A[1][2] = yzsum;
        A[2][0] = xzsum;
        A[2][1] = yzsum;
        A[2][2] = zzsum;
        */

        // should be sorted
        eigen_decomposition(A, V[0], V[1], V[2], d);

        v1 = point3d((float)V[0][2], (float)V[1][2], (float)V[2][2]).normalize();
        v2 = point3d((float)V[0][1], (float)V[1][1], (float)V[2][1]).normalize();
        v3 = point3d((float)V[0][0], (float)V[1][0], (float)V[2][0]).normalize();

        cg.x = (float)(xsum / count);
        cg.y = (float)(ysum / count);
        cg.z = (float)(zsum / count);

        d1 = (float)d[2];
        d2 = (float)d[1];
        d3 = (float)d[0];
    }

    void fprintfcovmatrix(FILE* f)
    {
        A[0][0] = xxsum / (double)count - (xsum * xsum) / (double)(count * count);
        A[0][1] = xysum / (double)count - (xsum * ysum) / (double)(count * count);
        A[0][2] = xzsum / (double)count - (xsum * zsum) / (double)(count * count);
        A[1][0] = xysum / (double)count - (ysum * xsum) / (double)(count * count);
        A[1][1] = yysum / (double)count - (ysum * ysum) / (double)(count * count);
        A[1][2] = yzsum / (double)count - (ysum * zsum) / (double)(count * count);
        A[2][0] = xzsum / (double)count - (zsum * xsum) / (double)(count * count);
        A[2][1] = yzsum / (double)count - (zsum * ysum) / (double)(count * count);
        A[2][2] = zzsum / (double)count - (zsum * zsum) / (double)(count * count);

        fprintf(f, "old \n");
        fprintf(f, "%f %f %f \n", A[0][0], A[0][1], A[0][2]);
        fprintf(f, "%f %f %f \n", A[1][0], A[1][1], A[1][2]);
        fprintf(f, "%f %f %f \n", A[2][0], A[2][1], A[2][2]);

        float xmean = xsum / (double)count;
        float ymean = ysum / (double)count;
        float zmean = zsum / (double)count;

        A[0][0] = (xxsum - xsum * xmean - xsum * xmean + xmean * xmean * (double)count) / (double)(count);
        A[0][1] = (xysum - ysum * xmean - xsum * ymean + xmean * ymean * (double)count) / (double)(count);
        A[0][2] = (xzsum - zsum * xmean - xsum * zmean + xmean * zmean * (double)count) / (double)(count);
        A[1][0] = (xysum - xsum * ymean - ysum * xmean + ymean * xmean * (double)count) / (double)(count);
        A[1][1] = (yysum - ysum * ymean - ysum * ymean + ymean * ymean * (double)count) / (double)(count);
        A[1][2] = (yzsum - zsum * ymean - ysum * zmean + ymean * zmean * (double)count) / (double)(count);
        A[2][0] = (xzsum - xsum * zmean - zsum * xmean + zmean * xmean * (double)count) / (double)(count);
        A[2][1] = (yzsum - ysum * zmean - zsum * ymean + zmean * ymean * (double)count) / (double)(count);
        A[2][2] = (zzsum - zsum * zmean - zsum * zmean + zmean * zmean * (double)count) / (double)(count);

        fprintf(f, "new \n");
        fprintf(f, "%f %f %f \n", A[0][0], A[0][1], A[0][2]);
        fprintf(f, "%f %f %f \n", A[1][0], A[1][1], A[1][2]);
        fprintf(f, "%f %f %f \n", A[2][0], A[2][1], A[2][2]);
    }

    point3d getCG() { return point3d((float)(xsum / count), (float)(ysum / count), (float)(zsum / count)); }
};

struct idValue
{
    int id;
    float value;

    idValue(){}

    idValue(int _id, float _value)
    {
        id = _id;
        value = _value;
    }

    idValue& operator=(const idValue& param)
    {
        id = param.id;
        value = param.value;
        return *this;
    }

    bool operator>(const idValue& param) const { return (value > param.value); }

    bool operator<(const idValue& param) const { return (value < param.value); }
};

struct mv2DTriangle
{
    int cam;
    point2d pts[3];

    mv2DTriangle& operator=(const mv2DTriangle& m)
    {
        pts[0] = m.pts[0];
        pts[1] = m.pts[1];
        pts[2] = m.pts[2];
        cam = m.cam;
        return *this;
    }
};

struct mv3DTriangle
{
    point3d pts[3];

    mv3DTriangle& operator=(const mv3DTriangle& m)
    {
        pts[0] = m.pts[0];
        pts[1] = m.pts[1];
        pts[2] = m.pts[2];
        return *this;
    }
};

struct line2d
{
    point2d pts[2];

    line2d(const point2d& a, const point2d& b)
    {
        pts[0] = a;
        pts[1] = b;
    }

    line2d& operator=(const line2d& m)
    {
        pts[0] = m.pts[0];
        pts[1] = m.pts[1];
        return *this;
    }
};

struct triangle2d
{
    point2d pts[3];

    triangle2d(point2d& a, point2d& b, point2d& c)
    {
        pts[0] = a;
        pts[1] = b;
        pts[2] = c;
    }

    triangle2d& operator=(const triangle2d& m)
    {
        pts[0] = m.pts[0];
        pts[1] = m.pts[1];
        pts[2] = m.pts[2];
        return *this;
    }
};

typedef unsigned char uchar;

struct rgb
{
    uchar r, g, b;
    rgb()
    {
        r = 0;
        g = 0;
        b = 0;
    }
    rgb(uchar _r, uchar _g, uchar _b)
    {
        r = _r;
        g = _g;
        b = _b;
    }

    inline rgb operator-(const rgb& _p) const
    {
        return rgb(r - _p.r, g - _p.g, b - _p.b);
    }

    inline rgb operator+(const rgb& _p) const
    {
        return rgb(r + _p.r, g + _p.g, b + _p.b);
    }
};
rgb rgb_random();

class mv_bites_array
{
public:
    unsigned int* bits;
    int allocated;
    int nbits;
    int sizeofbits;

    mv_bites_array(int _nbits);
    ~mv_bites_array();

    void clear();
    bool getbit(int bitid);
    void setbit(int bitid, bool value);
    void ORBits(mv_bites_array* e);
    void ANDBits(mv_bites_array* e);
    int gentNSumBitsOfANDBits(mv_bites_array* e);
    int getNSumBits();
    void printfBits();
    void copy(mv_bites_array* m)
    {
        for(int i = 0; i < allocated; i++)
        {
            bits[i] = m->bits[i];
        };
    }
};

int getANDBits(mv_bites_array* a1, mv_bites_array* a2);

struct cameraMatrices
{
    matrix3x4 P;
    matrix3x3 R, K, iR, iK, iCam;
    point3d C;
    float f, k1, k2;
};

struct track
{
    point3d p;
    staticVector<int>* cams;

    track()
    {
        p = point3d();
        cams = NULL;
    }

    inline track& operator=(const track& param)
    {
        p = param.p;
        cams = param.cams;
        return *this;
    }
};

int indexOfSortedVoxelArrByX(int val, staticVector<voxel>* values, int startId, int stopId);
