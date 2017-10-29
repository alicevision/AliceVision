#include "mv_color.hpp"


void Color::doprintf() const
{
    printf("%f %f %f\n", r, g, b);
}

void Color::saveToFile(const std::string& fileName) const
{
    FILE* f = fopen(fileName.c_str(), "w");
    fprintf(f, "%f %f %f", r, g, b);
    fclose(f);
}

void Color::loadFromFile(const std::string& fileName)
{
    FILE* f = fopen(fileName.c_str(), "r");
    fscanf(f, "%f %f %f", &r, &g, &b);
    fclose(f);
}
