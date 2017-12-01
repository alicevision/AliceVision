// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "mv_staticVector.hpp"
#include "mv_structures.hpp"

template <class T>
class mv_depth_map
{
public:
    staticVector<staticVector<T>*>* map;
    staticVector<int>* occupied;
    int width, height, im_size, maxDepth;

public:
    mv_depth_map(int _width, int _height, int _maxDepth) { allocate(_width, _height, _maxDepth); };

    mv_depth_map(std::string fileName)
    {
        int _width;
        int _height;
        int _maxDepth;

        FILE* f = fopen(fileName.c_str(), "wb");

        fread(&_width, sizeof(int), 1, f);
        fread(&_height, sizeof(int), 1, f);
        fread(&_maxDepth, sizeof(int), 1, f);
        allocate(_width, _height, _maxDepth);

        int nblk;
        fread(&nblk, sizeof(int), 1, f);

        for(int i = 0; i < nblk; i++)
        {
            mv_depth_map::io_block io;
            fread(&io, sizeof(mv_depth_map::io_block), 1, f);

            int id = io.x * height + io.y;

            occupied->(*map)[id] = new staticVector<T>(maxDepth);
            (*map)[id]->resize(io.size);

            fread(&(*(*map)[id])[0], sizeof(T), io.size, f);
        };
        fclose(f);
    };

public:
    ~mv_depth_map(void) { deallocate(); };

    void clear(void)
    {
        while(occupied->size() > 0)
        {
            int id = *occupied->pop();
            delete(*map)[id];
            (*map)[id] = NULL;
        };
    };

private:
    void allocate(int _width, int _height, int _maxDepth)
    {
        width = _width;
        height = _height;
        maxDepth = _maxDepth;
        im_size = width * height;
        map = new staticVector<staticVector<T>*>(im_size);
        map->resize_with(im_size, NULL);
        occupied = new staticVector<int>(im_size);
    };

    void deallocate(void)
    {
        clear();
        delete map;
        delete occupied;
    };

public:
    void saveToFile(std::string fileName)
    {
        FILE* f = fopen(fileName.c_str(), "wb");

        fwrite(&width, sizeof(int), 1, f);
        fwrite(&height, sizeof(int), 1, f);
        fwrite(&maxDepth, sizeof(int), 1, f);

        int nblk = occupied->size();
        fwrite(&nblk, sizeof(int), 1, f);

        for(int i = 0; i < nblk; i++)
        {
            int id = (*occupied)[i];
            mv_depth_map::io_block io;
            io.size = (*map)[id]->size();
            io.x = id / height;
            io.y = id % height;
            fwrite(&io, sizeof(mv_depth_map::io_block), 1, f);
            fwrite(&(*(*map)[id])[0], sizeof(T), io.size, f);
        };
        fclose(f);
    };

    bool add(pixel pix, T& val)
    {
        int id = pix.x * height + pix.y;
        return add(id, &(*map)[id], val);
    };

    bool add(int id, staticVector<T>** pmap, T& val)
    {
        if((*pmap) == NULL)
        {
            (*pmap) = new staticVector<T>(maxDepth);
            occupied->push_back(id);
        };

        if((*pmap)->size() < maxDepth)
        {
            (*pmap)->push_back(val);

            return true;
        }
        else
        {
            return false;
        };
    };

    void setWithoutCheck(pixel pix, int i, T& val)
    {
        int id = pix.x * height + pix.y;
        (*(*map)[id])[i] = val;
    };

private:
    struct io_block
    {
        int size, x, y;
    };
};
