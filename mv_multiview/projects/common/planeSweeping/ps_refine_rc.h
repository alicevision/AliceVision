#pragma once

#include "ps_sgm_rc.h"

class ps_refine_rc : public ps_sgm_rc
{
public:
    ps_refine_rc(int _rc, int _scale, int _step, ps_sgm_params* _sp);
    ~ps_refine_rc(void);

    bool refinercCUDA(bool checkIfExists = true);

    std::string outDir;
    std::string tmpDir;
    bool _userTcOrPixSize;
    int _wsh;
    float _gammaC;
    float _gammaP;

private:
    int _nSamplesHalf;
    int _ndepthsToRefine;
    float _sigma;
    int _niters;

    ps_depthSimMap* getDepthPixSizeMapFromSGM();
    ps_depthSimMap* refineAndFuseDepthSimMapCUDA(ps_depthSimMap* depthPixSizeMapVis);
    ps_depthSimMap* optimizeDepthSimMapCUDA(ps_depthSimMap* depthPixSizeMapVis, ps_depthSimMap* depthSimMapPhoto);
};

void refineDepthMaps(multiviewParams* mp, mv_prematch_cams* pc, const staticVector<int>& cams);
void refineDepthMaps(int CUDADeviceNo, multiviewParams* mp, mv_prematch_cams* pc, const staticVector<int>& cams);
