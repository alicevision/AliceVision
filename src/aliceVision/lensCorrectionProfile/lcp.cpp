#include "boost/filesystem.hpp"

#include <expat.h>
#include "aliceVision/lensCorrectionProfile/lcp.hpp"



template<typename T>
constexpr T intp(T a, T b, T c)
{
    // calculate a * b + (1 - a) * c
    return a * (b - c) + c;
}

void XmlStartHandler(void* data, const char* el, const char** attr)
{
    LCPinfo* LCPdata = static_cast<LCPinfo*>(data);

    std::string element(el);

    if (!LCPdata->isSeqOpened)
    {
        LCPdata->isSeqOpened = (element == "photoshop:CameraProfiles");
    }
    else if (LCPdata->inAlternate)
    {
        // do nothing
    }
    else if ((element == "stCamera:AlternateLensIDs") || (element == "stCamera:AlternateLensNames"))
    {
        LCPdata->inAlternate = true;
    }
    else if (element == "rdf:li")
    {
        LCPdata->modelCount++;
    }
    else if ((LCPdata->modelCount == 1) && !LCPdata->isCommonOK && (element == "rdf:Description"))
    {
        for (int i = 0; attr[i]; i += 2)
        {
            std::string key(attr[i]);
            std::string value(attr[i + 1]);

            if (key == "stCamera:Author")
            {
                LCPdata->Author = value;
            }
            else if (key == "stCamera:ProfileName")
            {
                LCPdata->ProfileName = value;
            }
            else if (key == "stCamera:Lens")
            {
                LCPdata->Lens = value;
            }
            else if (key == "stCamera:LensPrettyName")
            {
                LCPdata->LensPrettyName = value;
            }
            else if (key == "stCamera:LensInfo")
            {
                LCPdata->LensInfo = value;
            }
            else if (key == "stCamera:LensID")
            {
                LCPdata->LensID = atoi(value.c_str());
            }
            else if (key == "stCamera:Make")
            {
                LCPdata->Make = value;
            }
            else if (key == "stCamera:Model")
            {
                LCPdata->Model = value;
            }
            else if (key == "stCamera:UniqueCameraModel")
            {
                LCPdata->UniqueCameraModel = value;
            }
            else if (key == "stCamera:CameraPrettyName")
            {
                LCPdata->CameraPrettyName = value;
            }
            else if (key == "stCamera:SensorFormatFactor")
            {
                LCPdata->SensorFormatFactor = atof(value.c_str());
            }
            else if (key == "stCamera:ImageLength")
            {
                LCPdata->ImageLength = atoi(value.c_str());
            }
            else if (key == "stCamera:ImageWidth")
            {
                LCPdata->ImageWidth = atoi(value.c_str());
            }
            else if (key == "stCamera:ApertureValue")
            {
                LCPdata->currLensParam.camData.ApertureValue = atoi(value.c_str());
            }
            else if (key == "stCamera:FocalLength")
            {
                LCPdata->currLensParam.camData.FocalLength = atoi(value.c_str());
            }
            else if (key == "stCamera:FocusDistance")
            {
                LCPdata->currLensParam.camData.FocusDistance = atof(value.c_str());
            }
        }
        LCPdata->isCommonOK = true;
    }
    else if ((LCPdata->modelCount > 1) && !LCPdata->isCamDataOK && (element == "rdf:Description"))
    {
        for (int i = 0; attr[i]; i += 2)
        {
            std::string key(attr[i]);
            std::string value(attr[i + 1]);

            if (key == "stCamera:ApertureValue")
            {
                LCPdata->currLensParam.camData.ApertureValue = atoi(value.c_str());
            }
            else if (key == "stCamera:FocalLength")
            {
                LCPdata->currLensParam.camData.FocalLength = atoi(value.c_str());
            }
            else if (key == "stCamera:FocusDistance")
            {
                LCPdata->currLensParam.camData.FocusDistance = atof(value.c_str());
            }
        }
        LCPdata->isCamDataOK = true;
    }
    else if (LCPdata->isSeqOpened && LCPdata->isCommonOK)
    {
        if ((element == "stCamera:PerspectiveModel") || ((element == "rdf:Description") && LCPdata->waitPerspModeldescription))
        {
            if (!attr[0])
            {
                LCPdata->waitPerspModeldescription = true;
            }
            else if (element == "rdf:Description")
            {
                LCPdata->waitPerspModeldescription = false;
            }

            for (int i = 0; attr[i]; i += 2)
            {
                std::string key(attr[i]);
                std::string value(attr[i + 1]);

                if (key == "stCamera:Version")
                {
                    LCPdata->currLensParam.perspParams.Version = atoi(value.c_str());
                }
                else if (key == "stCamera:FocalLengthX")
                {
                    LCPdata->currLensParam.perspParams.FocalLengthX = atof(value.c_str());
                }
                else if (key == "stCamera:FocalLengthY")
                {
                    LCPdata->currLensParam.perspParams.FocalLengthY = atof(value.c_str());
                }
                else if (key == "stCamera:RadialDistortParam1")
                {
                    LCPdata->currLensParam.perspParams.RadialDistortParam1 = atof(value.c_str());
                }
                else if (key == "stCamera:RadialDistortParam2")
                {
                    LCPdata->currLensParam.perspParams.RadialDistortParam2 = atof(value.c_str());
                }
                else if (key == "stCamera:RadialDistortParam3")
                {
                    LCPdata->currLensParam.perspParams.RadialDistortParam3 = atof(value.c_str());
                }
            }
            LCPdata->currLensParam.perspParams.isEmpty = false;

        }
        else if (element == "stCamera:FisheyeModel")
        {
            LCPdata->currLensParam.isFisheye = true;

            for (int i = 0; attr[i]; i += 2)
            {
                std::string key(attr[i]);
                std::string value(attr[i + 1]);

                if (key == "stCamera:Version")
                {
                    LCPdata->currLensParam.fisheyeParams.Version = atoi(value.c_str());
                }
                else if (key == "stCamera:FocalLengthX")
                {
                    LCPdata->currLensParam.fisheyeParams.FocalLengthX = atof(value.c_str());
                }
                else if (key == "stCamera:FocalLengthY")
                {
                    LCPdata->currLensParam.fisheyeParams.FocalLengthY = atof(value.c_str());
                }
                else if (key == "stCamera:ImageXCenter")
                {
                    LCPdata->currLensParam.fisheyeParams.ImageXCenter = atof(value.c_str());
                }
                else if (key == "stCamera:ImageYCenter")
                {
                    LCPdata->currLensParam.fisheyeParams.ImageYCenter = atof(value.c_str());
                }
                else if (key == "stCamera:ResidualMeanError")
                {
                    LCPdata->currLensParam.fisheyeParams.ResidualMeanError = atof(value.c_str());
                }
                else if (key == "stCamera:ResidualStandardDeviation")
                {
                    LCPdata->currLensParam.fisheyeParams.ResidualStandardDeviation = atof(value.c_str());
                }
                else if (key == "stCamera:RadialDistortParam1")
                {
                    LCPdata->currLensParam.fisheyeParams.RadialDistortParam1 = atof(value.c_str());
                }
                else if (key == "stCamera:RadialDistortParam2")
                {
                    LCPdata->currLensParam.fisheyeParams.RadialDistortParam2 = atof(value.c_str());
                }
            }
            LCPdata->currLensParam.fisheyeParams.isEmpty = false;
        }
        else if (element == "stCamera:VignetteModel")
        {
            LCPdata->currLensParam.hasVignetteParams = true;

            for (int i = 0; attr[i]; i += 2)
            {
                std::string key(attr[i]);
                std::string value(attr[i + 1]);

                if (key == "stCamera:VignetteModelParam1")
                {
                    LCPdata->currLensParam.vignParams.VignetteModelParam1 = atof(value.c_str());
                }
                else if (key == "stCamera:VignetteModelParam2")
                {
                    LCPdata->currLensParam.vignParams.VignetteModelParam2 = atof(value.c_str());
                }
                else if (key == "stCamera:VignetteModelParam3")
                {
                    LCPdata->currLensParam.vignParams.VignetteModelParam3 = atof(value.c_str());
                }
            }
            LCPdata->currLensParam.vignParams.isEmpty = false;
        }
        else if (element == "stCamera:ChromaticRedGreenModel")
        {

        }
        else if (element == "stCamera:ChromaticGreenModel")
        {

        }
        else if (element == "stCamera:ChromaticBlueGreenModel")
        {

        }
    }

}  /* End of start handler */

void XmlEndHandler(void* data, const char* el)
{
    LCPinfo* LCPdata = static_cast<LCPinfo*>(data);

    std::string element(el);

    if (LCPdata->isSeqOpened && (element == "photoshop:CameraProfiles"))
    {
        LCPdata->isSeqOpened = false;
    }
    else if (LCPdata->inAlternate && ((element == "stCamera:AlternateLensIDs") || (element == "stCamera:AlternateLensNames")))
    {
        LCPdata->inAlternate = false;
    }
    else if (LCPdata->isSeqOpened && (element == "rdf:li") && !LCPdata->inAlternate)
    {
        LCPdata->v_lensParams.push_back(LCPdata->currLensParam);
        LCPdata->currLensParam.reset();
        LCPdata->isCamDataOK = false;
    }
}  /* End of end handler */

LCPinfo::LCPinfo(const std::string& filename) :
    isSeqOpened(false),
    isCommonOK(false),
    isCamDataOK(false),
    inAlternate(false),
    waitPerspModeldescription(false),
    modelCount(0),
    Author(""),
    Make(""),
    Model(""),
    UniqueCameraModel(""),
    CameraRawProfile(false),
    LensID(0),
    Lens(""),
    LensInfo(""),
    CameraPrettyName(""),
    LensPrettyName(""),
    ProfileName(""),
    SensorFormatFactor(1.f),
    ImageWidth(0),
    ImageLength(0)
{
    XML_Parser parser = XML_ParserCreate(nullptr);

    if (!parser) {
        throw std::runtime_error("Couldn't allocate memory for XML parser");
    }

    XML_SetElementHandler(parser, XmlStartHandler, XmlEndHandler);
    XML_SetUserData(parser, static_cast<void*>(this));

    FILE* const pFile = fopen(filename.c_str(), "rb");

    if (pFile) {
        constexpr int BufferSize = 8192;
        char buf[BufferSize];
        bool done;

        do {
            int bytesRead = fread(buf, 1, BufferSize, pFile);
            done = feof(pFile);

            if (XML_Parse(parser, buf, bytesRead, done) == XML_STATUS_ERROR) {
                XML_ParserFree(parser);
                throw std::runtime_error("Invalid XML in LCP file");
            }
        } while (!done);

        fclose(pFile);
    }

    XML_ParserFree(parser);
}

bool LCPinfo::search(settingsInfo& settings, LCPCorrectionMode mode, int& iLow, int& iHigh, float& weightLow)
{
    iLow = iHigh = -1;

    std::vector<bool> v_isDistortionValid;
    std::vector<bool> v_isVignetteValid;

    // Search best focal length
    for (size_t i = 0; i < v_lensParams.size(); ++i)
    {
        const LensParam& currParam = v_lensParams[i];
        const float f = currParam.camData.FocalLength;

        v_isDistortionValid.push_back(currParam.isFisheye ? !currParam.fisheyeParams.isEmpty : !currParam.perspParams.isEmpty);
        v_isVignetteValid.push_back(currParam.hasVignetteParams && !currParam.vignParams.isEmpty);

        bool isCurrentValid = (mode == LCPCorrectionMode::DISTORTION && v_isDistortionValid.back()) ||
                              (mode == LCPCorrectionMode::VIGNETTE && v_isVignetteValid.back());

        if (isCurrentValid)
        {
            if (
                f <= settings.FocalLength
                && (
                    iLow == -1
                    || f > v_lensParams[iLow].camData.FocalLength
                    || (settings.FocusDistance == 0 && f == v_lensParams[iLow].camData.FocalLength && v_lensParams[iLow].camData.FocusDistance > currParam.camData.FocusDistance)
                    )
                )
            {
                iLow = i;
            }

            if (
                f >= settings.FocalLength
                && (
                    iHigh == -1
                    || f < v_lensParams[iHigh].camData.FocalLength
                    || (settings.FocusDistance == 0 && f == v_lensParams[iHigh].camData.FocalLength && v_lensParams[iHigh].camData.FocusDistance < currParam.camData.FocusDistance)
                    )
                )
            {
                iHigh = i;
            }
        }
    }

    if (iLow == -1)
    {
        iLow = iHigh;
    }
    else if (iHigh == -1)
    {
        iHigh = iLow;
    }

    bool settingsOK = (mode == LCPCorrectionMode::VIGNETTE && settings.ApertureValue > 0) ||
                      (mode != LCPCorrectionMode::VIGNETTE && settings.FocusDistance > 0);

    if (iLow != -1 && iHigh != -1 && iLow != iHigh && settingsOK)
    {
        std::vector<int> candidatesLow;
        std::vector<int> candidatesHigh;

        for (int i = 0; i < v_lensParams.size(); ++i)
        {
            bool isCurrentValid = (mode == LCPCorrectionMode::DISTORTION && v_isDistortionValid[i]) ||
                                  (mode == LCPCorrectionMode::VIGNETTE && v_isVignetteValid[i]);

            if (isCurrentValid && v_lensParams[i].camData.FocalLength == v_lensParams[iLow].camData.FocalLength)
            {
                candidatesLow.push_back(i);
            }
            if (isCurrentValid && v_lensParams[i].camData.FocalLength == v_lensParams[iHigh].camData.FocalLength)
            {
                candidatesHigh.push_back(i);
            }
        }

        for (int i = 0; i < candidatesLow.size(); ++i)
        {
            bool update = false;
            if (mode == LCPCorrectionMode::VIGNETTE)
            {
                float currAperture = v_lensParams[iLow].camData.ApertureValue;
                float candidateAperture = v_lensParams[candidatesLow[i]].camData.ApertureValue;

                update = (candidateAperture >= settings.ApertureValue && candidateAperture < currAperture&& currAperture > settings.ApertureValue) ||
                    (candidateAperture <= settings.ApertureValue &&
                        (currAperture > settings.ApertureValue || fabs(settings.ApertureValue - candidateAperture) < fabs(settings.ApertureValue - currAperture)));
            }
            else
            {
                float currFocus = v_lensParams[iLow].camData.FocusDistance;
                float candidateFocus = v_lensParams[candidatesLow[i]].camData.FocusDistance;

                update = (candidateFocus >= settings.FocusDistance && candidateFocus < currFocus&& currFocus > settings.FocusDistance) ||
                         (candidateFocus <= settings.FocusDistance &&
                          (currFocus > settings.FocusDistance || fabs(settings.FocusDistance - candidateFocus) < fabs(settings.FocusDistance - currFocus)));
            }

            if (update)
            {
                iLow = candidatesLow[i];
            }
        }

        for (int i = 0; i < candidatesHigh.size(); ++i)
        {
            bool update = false;
            if (mode == LCPCorrectionMode::VIGNETTE)
            {
                float currAperture = v_lensParams[iHigh].camData.ApertureValue;
                float candidateAperture = v_lensParams[candidatesHigh[i]].camData.ApertureValue;

                update = (candidateAperture <= settings.ApertureValue && candidateAperture > currAperture && currAperture < settings.ApertureValue) ||
                         (candidateAperture >= settings.ApertureValue &&
                          (currAperture < settings.ApertureValue || fabs(settings.ApertureValue - candidateAperture) < fabs(settings.ApertureValue - currAperture)));
            }
            else
            {
                float currFocus = v_lensParams[iHigh].camData.FocusDistance;
                float candidateFocus = v_lensParams[candidatesHigh[i]].camData.FocusDistance;

                update = (candidateFocus <= settings.FocusDistance && candidateFocus > currFocus && currFocus < settings.FocusDistance) ||
                         (candidateFocus >= settings.FocusDistance &&
                          (currFocus < settings.FocusDistance || fabs(settings.FocusDistance - candidateFocus) < fabs(settings.FocusDistance - currFocus)));
            }

            if (update)
            {
                iHigh = candidatesHigh[i];
            }
        }

        if (mode == LCPCorrectionMode::VIGNETTE)
        {
            weightLow = (v_lensParams[iHigh].camData.ApertureValue - settings.ApertureValue) / (v_lensParams[iHigh].camData.ApertureValue - v_lensParams[iLow].camData.ApertureValue);
        }
        else
        {
            weightLow = (std::log(v_lensParams[iHigh].camData.FocusDistance) - std::log(settings.FocusDistance)) /
                        (std::log(v_lensParams[iHigh].camData.FocusDistance) - std::log(v_lensParams[iLow].camData.FocusDistance));
        }

        if (v_lensParams[iHigh].camData.FocalLength != v_lensParams[iLow].camData.FocalLength)
        {
            float weightLowFocalLength = (std::log(v_lensParams[iHigh].camData.FocalLength) - std::log(settings.FocalLength)) /
                                         (std::log(v_lensParams[iHigh].camData.FocalLength) - std::log(v_lensParams[iLow].camData.FocalLength));

            if (mode == LCPCorrectionMode::VIGNETTE)
            {
                weightLow = 0.5 * (weightLow + weightLowFocalLength);
            }
            else
            {
                weightLow = 0.2 * weightLow + 0.8 * weightLowFocalLength;
            }
        }

        return true;
    }
    else if (iLow == iHigh && iLow != -1)
    {
        weightLow = 1.0;
        return true;
    }
    else
    {
        return false;
    }
}

void LCPinfo::combine(size_t iLow, size_t iHigh, float weightLow, LCPCorrectionMode mode, LensParam& pOut)
{
    const LensParam& p1 = v_lensParams[iLow];
    const LensParam& p2 = v_lensParams[iHigh];

    switch (mode) {
    case LCPCorrectionMode::VIGNETTE: {
        pOut.hasVignetteParams = true;
        pOut.vignParams.isEmpty = false;
        pOut.vignParams.FocalLengthX = intp<float>(weightLow, p1.vignParams.FocalLengthX, p2.vignParams.FocalLengthX);
        pOut.vignParams.FocalLengthY = intp<float>(weightLow, p1.vignParams.FocalLengthY, p2.vignParams.FocalLengthY);
        pOut.vignParams.VignetteModelParam1 = intp<float>(weightLow, p1.vignParams.VignetteModelParam1, p2.vignParams.VignetteModelParam1);
        pOut.vignParams.VignetteModelParam2 = intp<float>(weightLow, p1.vignParams.VignetteModelParam2, p2.vignParams.VignetteModelParam2);
        pOut.vignParams.VignetteModelParam3 = intp<float>(weightLow, p1.vignParams.VignetteModelParam3, p2.vignParams.VignetteModelParam3);
        pOut.vignParams.isEmpty = false;
        break;
    }

    case LCPCorrectionMode::DISTORTION: {
        pOut.isFisheye = p1.isFisheye;
        if (pOut.isFisheye)
        {
            pOut.fisheyeParams.FocalLengthX = intp<float>(weightLow, p1.fisheyeParams.FocalLengthX, p2.fisheyeParams.FocalLengthX);
            pOut.fisheyeParams.FocalLengthY = intp<float>(weightLow, p1.fisheyeParams.FocalLengthY, p2.fisheyeParams.FocalLengthY);
            pOut.fisheyeParams.RadialDistortParam1 = intp<float>(weightLow, p1.fisheyeParams.RadialDistortParam1, p2.fisheyeParams.RadialDistortParam1);
            pOut.fisheyeParams.RadialDistortParam2 = intp<float>(weightLow, p1.fisheyeParams.RadialDistortParam2, p2.fisheyeParams.RadialDistortParam2);
            pOut.fisheyeParams.isEmpty = false;
        }
        else
        {
            pOut.perspParams.FocalLengthX = intp<float>(weightLow, p1.perspParams.FocalLengthX, p2.perspParams.FocalLengthX);
            pOut.perspParams.FocalLengthY = intp<float>(weightLow, p1.perspParams.FocalLengthY, p2.perspParams.FocalLengthY);
            pOut.perspParams.RadialDistortParam1 = intp<float>(weightLow, p1.perspParams.RadialDistortParam1, p2.perspParams.RadialDistortParam1);
            pOut.perspParams.RadialDistortParam2 = intp<float>(weightLow, p1.perspParams.RadialDistortParam2, p2.perspParams.RadialDistortParam2);
            pOut.perspParams.RadialDistortParam3 = intp<float>(weightLow, p1.perspParams.RadialDistortParam3, p2.perspParams.RadialDistortParam3);
            pOut.perspParams.isEmpty = false;
        }
        break;
    }

    //case LCPCorrectionMode::CA: {
    //	pCorr1->merge(pLow->chromRG, pHigh->chromRG, facLow);
    //	pCorr2->merge(pLow->chromG, pHigh->chromG, facLow);
    //	pCorr3->merge(pLow->chromBG, pHigh->chromBG, facLow);
    //	break;
    //}
    }



}

