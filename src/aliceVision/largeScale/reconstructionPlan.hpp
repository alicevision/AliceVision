#pragma once
#include "largeScale.hpp"
#include "voxelsGrid.hpp"

class reconstructionPlan : public voxelsGrid
{
public:
    staticVector<int>* nVoxelsTracks;
    reconstructionPlan(voxel& dimmensions, point3d* space, multiviewParams* _mp, mv_prematch_cams* _pc,
                       std::string _spaceRootDir);
    ~reconstructionPlan();

    unsigned long getNTracks(const voxel& LU, const voxel& RD);
    bool divideBox(voxel& LU1o, voxel& RD1o, voxel& LU2o, voxel& RD2o, const voxel& LUi, const voxel& RDi,
                   unsigned long maxTracks);
    staticVector<point3d>* computeReconstructionPlanBinSearch(unsigned long maxTracks);

    staticVector<int>* getNeigboursIds(float dist, int id, bool ceilOrFloor);
    staticVector<int>* voxelsIdsIntersectingHexah(point3d* hexah);
    int getPtsCount(float dist, int id);
    staticVector<float>* computeMaximaInflateFactors(int maxPts);
    staticVector<sortedId>* computeOptimalreconstructionPlan(const staticVector<float>* maximaInflateFactors);
    void getHexahedronForID(float dist, int id, point3d* out);
};

void reconstructAccordingToOptimalReconstructionPlan(int gl, largeScale* ls);
void reconstructSpaceAccordingToVoxelsArray(const std::string& voxelsArrayFileName, largeScale* ls,
                                            bool doComputeColoredMeshes);
mv_mesh* joinMeshes(const std::vector<std::string>& recsDirs, staticVector<point3d>* voxelsArray, largeScale* ls);
mv_mesh* joinMeshes(int gl, largeScale* ls);
mv_mesh* joinMeshes(const std::string& voxelsArrayFileName, largeScale* ls);

staticVector<staticVector<int>*>* loadLargeScalePtsCams(const std::vector<std::string>& recsDirs);

