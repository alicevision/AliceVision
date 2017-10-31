#include "mv_matrix3x3.hpp"
#include <cstdio>


void matrix3x3::doprintf() const
{
    printf("%f %f %f\n", m11, m12, m13);
    printf("%f %f %f\n", m21, m22, m23);
    printf("%f %f %f\n", m31, m32, m33);
}

void matrix3x3::saveToFile(const std::string& fileName) const
{
    FILE* f = fopen(fileName.c_str(), "w");
    fprintf(f, "%f %f %f\n", m11, m12, m13);
    fprintf(f, "%f %f %f\n", m21, m22, m23);
    fprintf(f, "%f %f %f\n", m31, m32, m33);
    fclose(f);
}

void matrix3x3::loadFromFile(const std::string& fileName)
{
    FILE* f = fopen(fileName.c_str(), "r");
    if(f != nullptr)
    {
        fscanf(f, "%lf %lf %lf\n", &m11, &m12, &m13);
        fscanf(f, "%lf %lf %lf\n", &m21, &m22, &m23);
        fscanf(f, "%lf %lf %lf\n", &m31, &m32, &m33);
        fclose(f);
    }
}
