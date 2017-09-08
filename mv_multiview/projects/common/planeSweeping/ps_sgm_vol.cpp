#include "ps_sgm_vol.h"
#include "stdafx.h"

ps_sgm_vol::ps_sgm_vol(float _volGpuMB, int _volDimX, int _volDimY, int _volDimZ, ps_sgm_params* _sp)
{
    sp = _sp;

    volGpuMB = _volGpuMB;
    volDimX = _volDimX;
    volDimY = _volDimY;
    volDimZ = _volDimZ;

    {
        point3d dmi = sp->cps->getDeviceMemoryInfo();
        if(sp->mp->verbose)
            printf("GPU memory : free %f, total %f, used %f\n", dmi.x, dmi.y, dmi.z);
        volStepZ = 1;
        float volumeMB = volGpuMB;
        while(4.0f * volumeMB > dmi.x)
        {
            volStepZ++;
            volumeMB = (volGpuMB / (float)volDimZ) * (volDimZ / volStepZ);
        }
        if(sp->mp->verbose)
            printf("GPU memory : volume %f\n", 4.0f * volumeMB);

        if(volStepZ > 1)
        {
            if(sp->mp->verbose)
                printf("WARNING - low GPU memory - stepz %i\n", volStepZ);
        }
    }

    _volume = new staticVector<unsigned char>(volDimX * volDimY * volDimZ);
    _volume->resize_with(volDimX * volDimY * volDimZ, 255);

    _volumeSecondBest = new staticVector<unsigned char>(volDimX * volDimY * volDimZ);
    _volumeSecondBest->resize_with(volDimX * volDimY * volDimZ, 255);

    _volumeStepZ = new staticVector<unsigned char>(volDimX * volDimY * (volDimZ / volStepZ));
    _volumeStepZ->resize_with(volDimX * volDimY * (volDimZ / volStepZ), 255);

    _volumeBestZ = new staticVector<int>(volDimX * volDimY * (volDimZ / volStepZ));
    _volumeBestZ->resize_with(volDimX * volDimY * (volDimZ / volStepZ), -1);
}

ps_sgm_vol::~ps_sgm_vol()
{
    delete _volume;
    delete _volumeStepZ;
    delete _volumeBestZ;
}

/**
 * @brief Reduction of the similarity volume on the Z axis.
 *        (X, Y, Z) volume is reduced to (X, Y, Z/step).
 *        Inside each chunk of 'step' values, we keep the best similarity value
 *        in 'volumeStepZ' and store the original Z index in 'volumeBestZ'.
 */
void ps_sgm_vol::cloneVolumeStepZ()
{
    long tall = clock();

    //TODO: use a local variable for _volumeStepZ (instead of a member) and replace _volume
    _volumeStepZ->resize_with(volDimX * volDimY * (volDimZ / volStepZ), 255);
    _volumeBestZ->resize_with(volDimX * volDimY * (volDimZ / volStepZ), -1);
    unsigned char* _volumePtr = _volume->getDataWritable().data();
    unsigned char* _volumeStepZPtr = _volumeStepZ->getDataWritable().data();
    int* _volumeBestZPtr = _volumeBestZ->getDataWritable().data();
    for(int z = 0; z < volDimZ; z++)
    {
        for(int y = 0; y < volDimY; y++)
        {
            for(int x = 0; x < volDimX; x++)
            {
                if((z / volStepZ) < (volDimZ / volStepZ))
                {
                    int offs = (z / volStepZ) * volDimX * volDimY + y * volDimX + x;
                    unsigned char oldSim = _volumeStepZPtr[offs];
                    unsigned char newSim = _volumePtr[z * volDimX * volDimY + y * volDimX + x];
                    if(newSim <= oldSim)
                    {
                        _volumeStepZPtr[offs] = newSim;
                        _volumeBestZPtr[offs] = z;
                    }
                }
            }
        }
    }

    delete _volume;
    _volume = nullptr;

    if(sp->mp->verbose)
        printfElapsedTime(tall, "ps_sgm_vol::cloneVolumeStepZ ");
}

void ps_sgm_vol::cloneVolumeSecondStepZ()
{
    long tall = clock();

    _volumeStepZ->resize_with(volDimX * volDimY * (volDimZ / volStepZ), 255);
    _volumeBestZ->resize_with(volDimX * volDimY * (volDimZ / volStepZ), -1);
    unsigned char* _volumeStepZPtr = _volumeStepZ->getDataWritable().data();
    unsigned char* _volumeSecondBestPtr = _volumeSecondBest->getDataWritable().data();
    int* _volumeBestZPtr = _volumeBestZ->getDataWritable().data();
    for(int z = 0; z < volDimZ; z++)
    {
        for(int y = 0; y < volDimY; y++)
        {
            for(int x = 0; x < volDimX; x++)
            {
                if((z / volStepZ) < (volDimZ / volStepZ))
                {
                    int offs = (z / volStepZ) * volDimX * volDimY + y * volDimX + x;
                    unsigned char oldSim = _volumeStepZPtr[offs];
                    unsigned char newSim = _volumeSecondBestPtr[z * volDimX * volDimY + y * volDimX + x];
                    if(newSim <= oldSim)
                    {
                        _volumeStepZPtr[offs] = newSim;
                        _volumeBestZPtr[offs] = z;
                    }
                }
            }
        }
    }

    if (sp->mp->verbose)
        printfElapsedTime(tall, "ps_sgm_vol::cloneVolumeSecondStepZ ");
}

/**
 * @param[in] volStepXY step in the image space
 */
void ps_sgm_vol::SGMoptimizeVolumeStepZ(int rc, int volStepXY, int volLUX, int volLUY, int scale)
{
    long tall = clock();

    sp->cps->SGMoptimizeSimVolume(rc, _volumeStepZ, volDimX, volDimY, volDimZ / volStepZ, volStepXY, volLUX, volLUY,
                                  scale, sp->P1, sp->P2);

    if(sp->mp->verbose)
        printfElapsedTime(tall, "ps_sgm_vol::SGMoptimizeVolumeStepZ");
}

staticVector<idValue>* ps_sgm_vol::getOrigVolumeBestIdValFromVolumeStepZ(int zborder)
{
    long tall = clock();

    staticVector<idValue>* volumeBestIdVal = new staticVector<idValue>(volDimX * volDimY);
    volumeBestIdVal->resize_with(volDimX * volDimY, idValue(-1, 1.0f));
    unsigned char* _volumeStepZPtr = _volumeStepZ->getDataWritable().data();
    int* _volumeBestZPtr = _volumeBestZ->getDataWritable().data();
    idValue* volumeBestIdValPtr = volumeBestIdVal->getDataWritable().data();
    for(int z = zborder; z < volDimZ / volStepZ - zborder; z++)
    {
        for(int y = 1; y < volDimY - 1; y++)
        {
            for(int x = 1; x < volDimX - 1; x++)
            {
                int volumeIndex = z * volDimX * volDimY + y * volDimX + x;
                // value from volumeStepZ converted from (0, 255) to (-1, +1)
                float val = (((float)_volumeStepZPtr[volumeIndex]) / 255.0f) * 2.0f - 1.0f;
                int bestZ = _volumeBestZPtr[volumeIndex]; // TODO: what is bestZ?
                idValue& idVal = volumeBestIdValPtr[y * volDimX + x];
                assert(bestZ >= 0);

                if(idVal.id == -1)
                {
                    // if not initialized, set the value
                    idVal.value = val;
                    idVal.id = bestZ;
                }
                else if (val < idVal.value)
                {
                    // if already initialized, update the value if smaller
                    idVal.value = val;
                    idVal.id = bestZ;
                }
            }
        }
    }

    if(sp->mp->verbose)
        printfElapsedTime(tall, "ps_sgm_vol::getOrigVolumeBestIdValFromVolumeStepZ ");

    return volumeBestIdVal;
}

void ps_sgm_vol::copyVolume(const staticVector<unsigned char>* volume, int zFrom, int nZSteps)
{
    unsigned char* _volumePtr = _volume->getDataWritable().data();
    const unsigned char* volumePtr = volume->getData().data();
#pragma omp parallel for
    for(int z = zFrom; z < zFrom + nZSteps; z++)
    {
        for(int y = 0; y < volDimY; y++)
        {
            for(int x = 0; x < volDimX; x++)
            {
                _volumePtr[z * volDimY * volDimX + y * volDimX + x] =
                    volumePtr[(z - zFrom) * volDimY * volDimX + y * volDimX + x];
            }
        }
    }
}

void ps_sgm_vol::copyVolume(const staticVector<int>* volume)
{
    unsigned char* _volumePtr = _volume->getDataWritable().data();
    const int* volumePtr = volume->getData().data();
#pragma omp parallel for
    for(int z = 0; z < volDimZ; z++)
    {
        for(int y = 0; y < volDimY; y++)
        {
            for(int x = 0; x < volDimX; x++)
            {
                _volumePtr[z * volDimY * volDimX + y * volDimX + x] =
                    (unsigned char)volumePtr[z * volDimY * volDimX + y * volDimX + x];
            }
        }
    }
}

void ps_sgm_vol::addVolumeMin(const staticVector<unsigned char>* volume, int zFrom, int nZSteps)
{
    unsigned char* _volumePtr = _volume->getDataWritable().data();
    const unsigned char* volumePtr = volume->getData().data();
#pragma omp parallel for
    for(int z = zFrom; z < zFrom + nZSteps; z++)
    {
        for(int y = 0; y < volDimY; y++)
        {
            for(int x = 0; x < volDimX; x++)
            {
                int offs = z * volDimY * volDimX + y * volDimX + x;
                unsigned char va = _volumePtr[offs];
                unsigned char vn = volumePtr[(z - zFrom) * volDimY * volDimX + y * volDimX + x];
                _volumePtr[offs] = std::min(va, vn);
            }
        }
    }
}

void ps_sgm_vol::addVolumeSecondMin(const staticVector<unsigned char>* volume, int zFrom, int nZSteps)
{
    unsigned char* _volumePtr = _volume->getDataWritable().data();
    unsigned char* _volumeSecondBestPtr = _volumeSecondBest->getDataWritable().data();
    const unsigned char* volumePtr = volume->getData().data();
#pragma omp parallel for
    for(int z = zFrom; z < zFrom + nZSteps; z++)
    {
        for(int y = 0; y < volDimY; y++)
        {
            for(int x = 0; x < volDimX; x++)
            {
                const int vaIdx = z * volDimY * volDimX + y * volDimX + x;
                const int vnIdx = (z - zFrom) * volDimY * volDimX + y * volDimX + x;
                unsigned char& va = _volumePtr[vaIdx];
                unsigned char& va2 = _volumeSecondBestPtr[vaIdx];
                unsigned char vn = volumePtr[vnIdx];
                if(vn < va)
                {
                    va2 = va;
                    va = vn;
                }
                else if (vn < va2)
                {
                    va2 = vn;
                }
            }
        }
    }
}

void ps_sgm_vol::addVolumeAvg(int n, const staticVector<unsigned char>* volume, int zFrom, int nZSteps)
{
    unsigned char* _volumePtr = _volume->getDataWritable().data();
    const unsigned char* volumePtr = volume->getData().data();
#pragma omp parallel for
    for(int z = zFrom; z < zFrom + nZSteps; z++)
    {
        for(int y = 0; y < volDimY; y++)
        {
            for(int x = 0; x < volDimX; x++)
            {
                unsigned char va = _volumePtr[z * volDimY * volDimX + y * volDimX + x];
                unsigned char vn = volumePtr[(z - zFrom) * volDimY * volDimX + y * volDimX + x];
                float vv = ((float)va * (float)(n - 1) + (float)vn) / (float)n;
                assert(vv >= 0.0);
                assert(vv < 255.0);
                _volumePtr[z * volDimY * volDimX + y * volDimX + x] = (unsigned char)vv;
            }
        }
    }
}



staticVector<unsigned char>* ps_sgm_vol::getZSlice(int z) const
{
    unsigned char* _volumePtr = _volume->getDataWritable().data();
    staticVector<unsigned char>* zs = new staticVector<unsigned char>(volDimY * volDimX);
    unsigned char* zsPtr = zs->getDataWritable().data();
#pragma omp parallel for
    for(int y = 0; y < volDimY; y++)
    {
        for(int x = 0; x < volDimX; x++)
        {
            zsPtr[y * volDimX + x] = _volumePtr[z * volDimY * volDimX + y * volDimX + x];
        }
    }
    return zs;
}

void ps_sgm_vol::showVolume() const
{
    printf("x %i, y %i, z %i, s %i\n", volDimX, volDimY, volDimZ, volStepZ);
    for(int z = 0; z < volDimZ; z++)
    {
        staticVector<unsigned char>* zs = getZSlice(z);
        showImageOpenCVT(&(*zs)[0], volDimX, volDimY, 0, 255, 1, 50);
        delete zs;
    }
    cvDestroyWindow("showImageOpenCVT");
}

