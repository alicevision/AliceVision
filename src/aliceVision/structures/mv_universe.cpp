// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "mv_universe.hpp"


mv_universe::mv_universe(int elements)
{
    elts = new uni_elt[elements];
    allelems = elements;
    num = elements;
    initialize();
}

void mv_universe::initialize()
{
    num = allelems;
    for(int i = 0; i < allelems; i++)
    {
        elts[i].rank = 0;
        elts[i].size = 1;
        elts[i].p = i; // initialized to the index
    }
}

mv_universe::~mv_universe()
{
    delete[] elts;
}

int mv_universe::find(int x)
{
    int y = x;
    while(y != elts[y].p) // follow the index stored in p if not the same that the index
        y = elts[y].p;
    elts[x].p = y; // update x element to the final value (instead of keeping multiple indirections), so next time we will access it directly.
    return y;
}

void mv_universe::join(int x, int y)
{
    // join elements in the one with the highest rank
    if(elts[x].rank > elts[y].rank)
    {
        elts[y].p = x;
        elts[x].size += elts[y].size;
    }
    else
    {
        elts[x].p = y;
        elts[y].size += elts[x].size;
        if(elts[x].rank == elts[y].rank)
            elts[y].rank++;
    }
    num--; // the number of elements has been reduced by one
}

void mv_universe::addEdge(int x, int y)
{
    int a = find(x);
    int b = find(y);
    if(a != b)
    {
        join(a, b);
    }
}
