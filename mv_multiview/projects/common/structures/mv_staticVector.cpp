#include "mv_staticVector.h"
#include "stdafx.h"

int getArrayLengthFromFile(std::string fileName)
{
    FILE* f = fopen(fileName.c_str(), "rb");
    if(f == nullptr)
    {
        // printf("WARNING: file %s does not exists!\n", fileName.c_str());
        return 0;
    }

    int n = 0;
    fread(&n, sizeof(int), 1, f);
    if(n == -1)
    {
        fread(&n, sizeof(int), 1, f);
    }
    fclose(f);
    return n;
}
