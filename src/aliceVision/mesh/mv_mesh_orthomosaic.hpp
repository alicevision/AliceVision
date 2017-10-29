#pragma once

#include <aliceVision/CUDAInterfaces/cuda_plane_sweeping.hpp>
#include "mv_mesh_dem.hpp"
#include <aliceVision/output3D/mv_output3D.hpp>
#include <aliceVision/prematching/mv_prematch_cams.hpp>

class mv_mesh_orthomosaic : public mv_mesh_dem
{
public:
    mv_mesh_orthomosaic(std::string _tmpDir, std::string _demName, multiviewParams* _mp);
    ~mv_mesh_orthomosaic();
#ifdef DIRECTX
    void computeOrthomosaic(mv_prematch_cams* pc, mv_mesh* meGlobal, staticVector<staticVector<int>*>* camsTris,
                            std::string imageMagickPath);
#endif
private:
    void saveDem2Wrl(std::string wrlFileName, staticVector<float>* dem);
    void computeRcOrthomosaic(int rc, staticVector<float>* dem, staticVector<float>* demGlob);

    mv_output3D* o3d;
    std::string demName;
    mv_images_cache* ic;
    point3d demPixS;
    // staticVector<staticVector<int>*> *camsPts;
    staticVector<point4d>* orm;
    staticVector<float>* pixSizeMap;
    bool visualizeMetaData;
    bool doAverage;
};
