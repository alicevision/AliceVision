#pragma once

#include "mv_mesh.h"

class mv_mesh_unwrap : public mv_mesh
{
public:
    typedef struct
    {
        int rank;
        int p;
        int size;
        mv_bites_array* cams;
    } universe_elt;

    class universe
    {
    public:
        universe(int elements, int _ncams);
        ~universe();
        void erase();
        void eraseButNotEraseCams();
        void clearNotRootCams();
        int find(int x);
        void join(int x, int y);
        int joinButNotMergeCams(int x, int y);
        void addEdge(int x, int y);
        int size(int x) const { return elts[x].size; }
        int num_sets() const { return num; }

    public:
        universe_elt* elts;
        int num, allelems, ncams;
    };

    bool verbose;
    universe* univ;
    staticVector<int>* usedcams;
    multiviewParams* mp;

    mv_mesh_unwrap(multiviewParams* _mp);
    ~mv_mesh_unwrap();

    int getRefTriangleCommonEdgeId(int idTriRef, int idTriTar);
    void unwrap(staticVector<staticVector<int>*>* trisCams);
    void filterTrisCams(staticVector<staticVector<int>*>* trisCams);
};
