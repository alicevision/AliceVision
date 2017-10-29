#include "mv_point3d.hpp"

#include <cstdio>


void point3d::doprintf() const
{
    printf("%f %f %f\n", x, y, z);
}

void point3d::saveToFile(const std::string& fileName) const
{
    FILE* f = fopen(fileName.c_str(), "w");
    fprintf(f, "%f %f %f", x, y, z);
    fclose(f);
}

void point3d::loadFromFile(const std::string& fileName)
{
    FILE* f = fopen(fileName.c_str(), "r");
    fscanf(f, "%f %f %f", &x, &y, &z);
    fclose(f);
}
