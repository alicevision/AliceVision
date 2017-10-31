#include "mv_double3d.hpp"
#include "mv_matrix3x3.hpp"
#include "mv_matrix3x4.hpp"


double3d::double3d()
{
    x = 0.0;
    y = 0.0;
    z = 0.0;
}

double3d::double3d(const double _x, const double _y, const double _z)
{
    x = _x;
    y = _y;
    z = _z;
}

double3d& double3d::operator=(const double3d& param)
{
    x = param.x;
    y = param.y;
    z = param.z;
    return *this;
}

bool double3d::operator==(const double3d& param) const
{
    return (x == param.x) && (y == param.y) && (z == param.z);
}

double3d double3d::operator-(const double3d& _p) const
{
    return double3d(x - _p.x, y - _p.y, z - _p.z);
}

double3d double3d::operator-() const
{
    return double3d(-x, -y, -z);
}

double3d double3d::operator+(const double3d& _p) const
{
    return double3d(x + _p.x, y + _p.y, z + _p.z);
}

double3d double3d::operator*(const double d) const
{
    return double3d(x * d, y * d, z * d);
}

double3d double3d::operator/(const double d) const
{
    return double3d(x / d, y / d, z / d);
}

double3d double3d::normalize() const
{
    double d = sqrt(x * x + y * y + z * z);
    return double3d(x / d, y / d, z / d);
}

double dot(const double3d& p1, const double3d& p2)
{
    return p1.x * p2.x + p1.y * p2.y + p1.z * p2.z;

    /*
    __declspec(align(16)) double fp1[4];
    fp1[0] = p1.x;
    fp1[1] = p1.y;
    fp1[2] = p1.z;
    fp1[3] = 1.0;
    __declspec(align(16)) double fp2[4];
    fp2[0] = p2.x;
    fp2[1] = p2.y;
    fp2[2] = p2.z;
    fp2[3] = 1.0;


    double r;
    __m128 mv1, mv2;
    mv1 = _mm_load_ps(fp1);
    mv2 = _mm_load_ps(fp2);
    mv1 = _mm_mul_ps(mv1, mv2);
    mv2 = _mm_shuffle_ps(mv1, mv1, _MM_SHUFFLE(0, 0, 2, 1));
    mv1 = _mm_add_ps(mv1, mv2);
    mv2 = _mm_shuffle_ps(mv2, mv2, _MM_SHUFFLE(0, 0, 0, 1));
    mv1 = _mm_add_ss(mv1, mv2);
    _mm_store_ss(&r,mv1);

    return r;
    */
}

double3d cross(const double3d& a, const double3d& b)
{

    double3d vc;
    vc.x = a.y * b.z - a.z * b.y;
    vc.y = a.z * b.x - a.x * b.z;
    vc.z = a.x * b.y - a.y * b.x;

    /*
    __declspec(align(16)) double fp1[4]; fp1[0] = a.x; fp1[1] = a.y; fp1[2] = a.z; fp1[3] = 1.0;
    __declspec(align(16)) double fp2[4]; fp2[0] = b.x; fp2[1] = b.y; fp2[2] = b.z;	fp2[3] = 1.0;

    __declspec(align(16)) double res[4];
    //PSE_ALIGNED pseVector res;
    __m128 mv1, mv2, mbuf1;
    mv1 = _mm_load_ps(fp1);
    mv1 = _mm_shuffle_ps(mv1, mv1, 201);
    mv2 = _mm_load_ps(fp2);
    mbuf1 = _mm_shuffle_ps(mv2, mv2, 210);
    mv2 = _mm_mul_ps(mv2, mv1);
    mbuf1 = _mm_mul_ps(mbuf1, mv1);
    mv2 = _mm_shuffle_ps(mv2, mv2, 201);
    mbuf1 = _mm_sub_ps(mbuf1, mv2);
    _mm_store_ps(res,mbuf1);

    double3d vc;
    vc.x = res[0];
    vc.y = res[1];
    vc.z = res[2];
    */

    return vc;
}

double double3d::size() const
{
    double d = x * x + y * y + z * z;
    if(d == 0.0)
    {
        return 0.0;
    }

    return sqrt(d);
}

double double3d::size2() const
{
    return x * x + y * y + z * z;
}

void double3d::doprintf() const
{
    printf("%lf %lf %lf\n", x, y, z);
}

void double3d::saveToFile(const std::string& fileName) const
{
    FILE* f = fopen(fileName.c_str(), "w");
    fprintf(f, "%lf %lf %lf", x, y, z);
    fclose(f);
}

void double3d::loadFromFile(const std::string& fileName)
{
    FILE* f = fopen(fileName.c_str(), "r");
    fscanf(f, "%lf %lf %lf", &x, &y, &z);
    fclose(f);
}

double3d proj(double3d& e, double3d& a)
{
    return e * (dot(e, a) / dot(e, e));
}

double3d operator*(const matrix3x4& M, const double3d& _p)
{
    return double3d(M.m11 * _p.x + M.m12 * _p.y + M.m13 * _p.z + M.m14,
                    M.m21 * _p.x + M.m22 * _p.y + M.m23 * _p.z + M.m24,
                    M.m31 * _p.x + M.m32 * _p.y + M.m33 * _p.z + M.m34);
}

double3d operator*(const matrix3x3& M, const double3d& _p)
{
    return double3d(M.m11 * _p.x + M.m12 * _p.y + M.m13 * _p.z, M.m21 * _p.x + M.m22 * _p.y + M.m23 * _p.z,
                    M.m31 * _p.x + M.m32 * _p.y + M.m33 * _p.z);
}


double3d point3d2double3d(const point3d& p)
{
    return double3d((double)p.x, (double)p.y, (double)p.z);
}
