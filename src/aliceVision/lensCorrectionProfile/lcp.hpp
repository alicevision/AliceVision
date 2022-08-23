#pragma once

#include <string>
#include <vector>

enum class LCPCorrectionMode {
    VIGNETTE,
    DISTORTION,
    CA
};

struct settingsInfo
{
    float FocalLength = 0.f;
    float FocusDistance = 0.f;
    float ApertureValue = 0.f;

    void reset()
    {
        FocalLength = 0.f;
        FocusDistance = 0.f;
        ApertureValue = 0.f;
    }
};

struct PerspectiveModel
{
    int Version = -1;
    float FocalLengthX = 0.f;
    float FocalLengthY = 0.f;
    float ResidualMeanError = 0.f;
    float ResidualStandardDeviation = 0.f;
    float RadialDistortParam1 = 0.f;
    float RadialDistortParam2 = 0.f;
    float RadialDistortParam3 = 0.f;
    bool isEmpty = true;

    void reset()
    {
        Version = -1;
        FocalLengthX = 0.f;
        FocalLengthY = 0.f;
        ResidualMeanError = 0.f;
        ResidualStandardDeviation = 0.f;
        RadialDistortParam1 = 0.f;
        RadialDistortParam2 = 0.f;
        RadialDistortParam3 = 0.f;
        isEmpty = true;
    }
};

struct VignetteModel
{
    float FocalLengthX = 0.f;
    float FocalLengthY = 0.f;
    float VignetteModelParam1 = 0.f;
    float VignetteModelParam2 = 0.f;
    float VignetteModelParam3 = 0.f;
    bool isEmpty = true;

    void reset()
    {
        FocalLengthX = 0.f;
        FocalLengthY = 0.f;
        VignetteModelParam1 = 0.f;
        VignetteModelParam2 = 0.f;
        VignetteModelParam3 = 0.f;
        isEmpty = true;
    }
};

struct FisheyeModel
{
    int Version = -1;
    float FocalLengthX = 0.f;
    float FocalLengthY = 0.f;
    float ImageXCenter = 0.f;
    float ImageYCenter = 0.f;
    float ResidualMeanError = 0.f;
    float ResidualStandardDeviation = 0.f;
    float RadialDistortParam1 = 0.f;
    float RadialDistortParam2 = 0.f;
    bool isEmpty = true;

    void reset()
    {
        Version = -1;
        FocalLengthX = 0.f;
        FocalLengthY = 0.f;
        ImageXCenter = 0.f;
        ImageYCenter = 0.f;
        ResidualMeanError = 0.f;
        ResidualStandardDeviation = 0.f;
        RadialDistortParam1 = 0.f;
        RadialDistortParam2 = 0.f;
        isEmpty = true;
    }
};

struct LensParam
{
    bool isFisheye = false;
    bool hasVignetteParams = false;

    FisheyeModel fisheyeParams;
    PerspectiveModel perspParams;
    VignetteModel vignParams;

    settingsInfo camData;

    void reset()
    {
        isFisheye = false;
        hasVignetteParams = false;
        fisheyeParams.reset();
        perspParams.reset();
        vignParams.reset();
        camData.reset();
    }
};

class LCPinfo
{
public:
    LCPinfo(const std::string& filename);
    ~LCPinfo() {};

    bool search(settingsInfo& settings, LCPCorrectionMode mode, int& iLow, int& iHigh, float& weightLow);
    void combine(size_t iLow, size_t iHigh, float weightLow, LCPCorrectionMode mode, LensParam& pOut);

    bool isSeqOpened = false;
    bool isCommonOK = false;
    bool isCamDataOK = false;
    bool inAlternate = false;
    bool waitPerspModeldescription = false;

    int modelCount = 0;

    LensParam currLensParam;

    std::vector<LensParam> v_lensParams;

    std::string Author;
    std::string Make;
    std::string Model;
    std::string UniqueCameraModel;
    bool CameraRawProfile;
    int LensID;
    std::string Lens;
    std::string LensInfo;
    std::string CameraPrettyName;
    std::string LensPrettyName;
    std::string ProfileName;
    float SensorFormatFactor;
    int ImageWidth;
    int ImageLength;

};

