#pragma once

#include "mv_mesh.h"

#include "output3D/mv_output3D.h"
#include "planeSweeping/ps_depthSimMap.h"
#include "planeSweeping/ps_rctc.h"
#include "prematching/mv_prematch_cams.h"
#include "structures/mv_images_cache.h"

class mv_mesh_refine : public mv_mesh
{
public:
    multiviewParams* mp;
    mv_prematch_cams* pc;
    std::string tmpDir;
    std::string meshDepthMapsDir;
    std::string tmpDirOld;
    staticVector<staticVector<int>*>* ptsCams;
    mv_output3D* o3d;
    mv_images_cache* ic;
    cuda_plane_sweeping* cps;
    ps_rctc* prt;

    mv_mesh_refine(multiviewParams* _mp, mv_prematch_cams* _pc, std::string _tmpDir = "");
    ~mv_mesh_refine();

    void smoothDepthMapsAdaptiveByImages(staticVector<int>* usedCams);
    void smoothDepthMapAdaptiveByImage(ps_rctc* prt, int rc, staticVector<float>* tmpDepthMap);

    void transposeDepthMap(staticVector<float>* depthMapTransposed, staticVector<float>* depthMap, int w, int h);
    void alignSourceDepthMapToTarget(staticVector<float>* sourceDepthMapT, staticVector<float>* targetDepthMapT, int rc,
                                     float maxPixelSizeDist);

};
