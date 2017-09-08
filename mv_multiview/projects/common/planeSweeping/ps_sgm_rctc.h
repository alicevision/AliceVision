#pragma once

#include "ps_sgm_params.h"

class ps_sgm_rctc
{
public:
    ps_sgm_rctc(staticVector<float>* _rcTcDepths, int _rc, int _tc, int _scale, int _step, ps_sgm_params* _sp,
                staticVectorBool* _rcSilhoueteMap = NULL);
    ~ps_sgm_rctc(void);

    staticVector<unsigned char>* computeDepthSimMapVolume(float& volumeMBinGPUMem, int wsh, float gammaC, float gammaP);

private:
    staticVector<voxel>* getPixels();

    ps_sgm_params* sp;

    int rc, tc, scale, step;
    staticVector<float>* rcTcDepths;
    float epipShift;
    int w, h;
    staticVectorBool* rcSilhoueteMap;
};
