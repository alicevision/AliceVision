#include <iostream>

#include "boost/filesystem.hpp"
#include "boost/algorithm/string.hpp"

#include <expat.h>
#include "lcp.hpp"

template<typename T>
constexpr T intp(T a, T b, T c)
{
    // calculate a * b + (1 - a) * c
    return a * (b - c) + c;
}

void LensParam::clear()
{
    _isFisheye = false;
    _hasVignetteParams = false;
    fisheyeParams.reset();
    perspParams.reset();
    vignParams.reset();
    camData.reset();
}

// Set of handlers to be connected with the expat XML parser contained in the "load" method of the "LCPinfo" class.

void XMLCALL LCPinfo::XmlStartHandler(void* pLCPinfo, const char* el, const char** attr)
{
    LCPinfo* LCPdata = static_cast<LCPinfo*>(pLCPinfo);

    std::string element(el);

    if (!LCPdata->isSeqOpened() && (element == "photoshop:CameraProfiles"))
    {
        LCPdata->openSequence();
    }
    else if (element == "stCamera:AlternateLensIDs")
    {
        LCPdata->openAlternateLensIDs();
    }
    else if (element == "stCamera:AlternateLensNames")
    {
        LCPdata->openAlternateLensNames();
    }
    else if (element == "rdf:li")
    {
        if ((LCPdata->isAlternateLensIDsOpened()) && !LCPdata->isCommonOK())
        {
            LCPdata->setGetText();
        }
        else if ((LCPdata->isAlternateLensNamesOpened()) && !LCPdata->isCommonOK())
        {
            LCPdata->setGetText();
        }
        else if (!LCPdata->isAlternateLensIDsOpened() && !LCPdata->isAlternateLensNamesOpened())
        {
            LCPdata->increaseModelCount();
        }      
    }
    else if ((LCPdata->getModelCount() == 1) && !LCPdata->isCommonOK() && (element == "rdf:Description"))
    {
        for (int i = 0; attr[i]; i += 2)
        {
            std::string key(attr[i]);
            std::string value(attr[i + 1]);

            if (key == "stCamera:Author")
            {
                LCPdata->setAuthor(value);
            }
            else if (key == "stCamera:ProfileName")
            {
                LCPdata->setProfileName(value);
            }
            else if (key == "stCamera:Lens")
            {
                LCPdata->addLensModel(value);
            }
            else if (key == "stCamera:LensPrettyName")
            {
                LCPdata->setLensPrettyName(value);
            }
            else if (key == "stCamera:LensInfo")
            {
                LCPdata->setLensInfo(value);
            }
            else if (key == "stCamera:LensID")
            {
                LCPdata->addLensID(atoi(value.c_str()));
            }
            else if (key == "stCamera:Make")
            {
                LCPdata->setCameraMaker(value);
            }
            else if (key == "stCamera:Model")
            {
                LCPdata->setCameraModel(value);
            }
            else if ((key == "stCamera:CameraRawProfile") && (value == "True"))
            {
                LCPdata->setAsRawProfile();
            }
            else if (key == "stCamera:UniqueCameraModel")
            {
                LCPdata->setUniqueCameraModel(value);
            }
            else if (key == "stCamera:CameraPrettyName")
            {
                LCPdata->setCameraPrettyName(value);
            }
            else if (key == "stCamera:SensorFormatFactor")
            {
                LCPdata->setSensorFormatFactor(atof(value.c_str()));
            }
            else if (key == "stCamera:ImageLength")
            {
                LCPdata->setImageLength(atoi(value.c_str()));
            }
            else if (key == "stCamera:ImageWidth")
            {
                LCPdata->setImageWidth(atoi(value.c_str()));
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
    }
    else if ((LCPdata->getModelCount() > 1) && !LCPdata->isCamDataOK() && (element == "rdf:Description"))
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
        LCPdata->setCamDataOK();
    }
    else if (LCPdata->isSeqOpened() && LCPdata->isCommonOK())
    {
        if ((element == "stCamera:PerspectiveModel") || ((element == "rdf:Description") && LCPdata->isWaitPerspModeldescription()))
        {
            if (!attr[0])
            {
                LCPdata->setWaitPerspModeldescription();
            }
            else if (element == "rdf:Description")
            {
                LCPdata->unsetWaitPerspModeldescription();
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
                else if (key == "stCamera:ImageXCenter")
                {
                    LCPdata->currLensParam.perspParams.ImageXCenter = atof(value.c_str());
                }
                else if (key == "stCamera:ImageYCenter")
                {
                    LCPdata->currLensParam.perspParams.ImageYCenter = atof(value.c_str());
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
            LCPdata->currLensParam.setFisheyeStatus(true);

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
            LCPdata->currLensParam.setVignetteParamsStatus(true);

            for (int i = 0; attr[i]; i += 2)
            {
                std::string key(attr[i]);
                std::string value(attr[i + 1]);

                if (key == "stCamera:FocalLengthX")
                {
                    LCPdata->currLensParam.vignParams.FocalLengthX = atof(value.c_str());
                }
                else if (key == "stCamera:FocalLengthY")
                {
                    LCPdata->currLensParam.vignParams.FocalLengthY = atof(value.c_str());
                }
                else if (key == "stCamera:VignetteModelParam1")
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

void XMLCALL LCPinfo::XmlEndHandler(void* pLCPinfo, const char* el)
{
    LCPinfo* LCPdata = static_cast<LCPinfo*>(pLCPinfo);

    std::string element(el);

    if (LCPdata->isSeqOpened() && (element == "photoshop:CameraProfiles"))
    {
        LCPdata->closeSequence();
    }
    else if (LCPdata->isAlternateLensIDsOpened() && (element == "stCamera:AlternateLensIDs"))
    {
        LCPdata->closeAlternateLensIDs();
    }
    else if (LCPdata->isAlternateLensNamesOpened() && ((element == "stCamera:AlternateLensNames")))
    {
        LCPdata->closeAlternateLensNames();
    }
    else if ((element == "rdf:li") && (LCPdata->isAlternateLensIDsOpened() || LCPdata->isAlternateLensNamesOpened()))
    {
        LCPdata->unsetGetText();
    }
    else if (LCPdata->isSeqOpened() && (element == "rdf:li") && !LCPdata->isAlternateLensIDsOpened() && !LCPdata->isAlternateLensNamesOpened())
    {
        LCPdata->storeCurrParams();
        LCPdata->currLensParam.clear();
        LCPdata->unsetCamDataOK();
    }
    else if ((LCPdata->getModelCount() == 1) && !LCPdata->isCommonOK() && (element == "rdf:Description"))
    {
        LCPdata->setCommonOK();
    }
}  /* End of end handler */

void XMLCALL LCPinfo::XmlTextHandler(void* pLCPinfo, const char* s, int len)
{
    LCPinfo* LCPdata = static_cast<LCPinfo*>(pLCPinfo);

    std::ostringstream localtextbuf;

    if (LCPdata->isGetText())
    {
        for (int i = 0; i < len; ++i)
        {
            localtextbuf << s[i];
        }
        if (LCPdata->isAlternateLensIDsOpened())
        {
            LCPdata->addLensID(std::atoi(localtextbuf.str().c_str()));
        }
        else if (LCPdata->isAlternateLensNamesOpened())
        {
            LCPdata->addLensModel(localtextbuf.str());
        }
    }
}

void XMLCALL LCPinfo::XmlStartHandlerCommonOnly(void* pLCPinfo, const char* el, const char** attr)
{
    LCPinfo* LCPdata = static_cast<LCPinfo*>(pLCPinfo);

    std::string element(el);

    if (!LCPdata->isSeqOpened() && (element == "photoshop:CameraProfiles"))
    {
        LCPdata->openSequence();
    }
    else if (element == "stCamera:AlternateLensIDs")
    {
        LCPdata->openAlternateLensIDs();
    }
    else if (element == "stCamera:AlternateLensNames")
    {
        LCPdata->openAlternateLensNames();
    }
    else if (element == "rdf:li")
    {
        if ((LCPdata->isAlternateLensIDsOpened()) && !LCPdata->isCommonOK())
        {
            LCPdata->setGetText();
        }
        else if ((LCPdata->isAlternateLensNamesOpened()) && !LCPdata->isCommonOK())
        {
            LCPdata->setGetText();
        }
        else if (!LCPdata->isAlternateLensIDsOpened() && !LCPdata->isAlternateLensNamesOpened())
        {
            LCPdata->increaseModelCount();
        }
    }
    else if ((LCPdata->getModelCount() == 1) && !LCPdata->isCommonOK() && (element == "rdf:Description"))
    {
        for (int i = 0; attr[i]; i += 2)
        {
            std::string key(attr[i]);
            std::string value(attr[i + 1]);

            if (key == "stCamera:Author")
            {
                LCPdata->setAuthor(value);
            }
            else if (key == "stCamera:ProfileName")
            {
                LCPdata->setProfileName(value);
            }
            else if (key == "stCamera:Lens")
            {
                LCPdata->addLensModel(value);
            }
            else if (key == "stCamera:LensPrettyName")
            {
                LCPdata->setLensPrettyName(value);
            }
            else if (key == "stCamera:LensInfo")
            {
                LCPdata->setLensInfo(value);
            }
            else if (key == "stCamera:LensID")
            {
                LCPdata->addLensID(atoi(value.c_str()));
            }
            else if (key == "stCamera:Make")
            {
                LCPdata->setCameraMaker(value);
            }
            else if (key == "stCamera:Model")
            {
                LCPdata->setCameraModel(value);
            }
            else if ((key == "stCamera:CameraRawProfile") && (value == "True"))
            {
                LCPdata->setAsRawProfile();
            }
            else if (key == "stCamera:UniqueCameraModel")
            {
                LCPdata->setUniqueCameraModel(value);
            }
            else if (key == "stCamera:CameraPrettyName")
            {
                LCPdata->setCameraPrettyName(value);
            }
            else if (key == "stCamera:SensorFormatFactor")
            {
                LCPdata->setSensorFormatFactor(atof(value.c_str()));
            }
            else if (key == "stCamera:ImageLength")
            {
                LCPdata->setImageLength(atoi(value.c_str()));
            }
            else if (key == "stCamera:ImageWidth")
            {
                LCPdata->setImageWidth(atoi(value.c_str()));
            }
        }
    }
}  /* End of start handler common only*/

void XMLCALL LCPinfo::XmlEndHandlerCommonOnly(void* pLCPinfo, const char* el)
{
    LCPinfo* LCPdata = static_cast<LCPinfo*>(pLCPinfo);

    std::string element(el);

    if (LCPdata->isSeqOpened() && (element == "photoshop:CameraProfiles"))
    {
        LCPdata->closeSequence();
    }
    else if (LCPdata->isAlternateLensIDsOpened() && (element == "stCamera:AlternateLensIDs"))
    {
        LCPdata->closeAlternateLensIDs();
    }
    else if (LCPdata->isAlternateLensNamesOpened() && ((element == "stCamera:AlternateLensNames")))
    {
        LCPdata->closeAlternateLensNames();
    }
    else if ((element == "rdf:li") && (LCPdata->isAlternateLensIDsOpened() || LCPdata->isAlternateLensNamesOpened()))
    {
        LCPdata->unsetGetText();
    }
    else if ((LCPdata->getModelCount() == 1) && !LCPdata->isCommonOK() && (element == "rdf:Description"))
    {
        LCPdata->setCommonOK();
    }
}  /* End of end handler common only */

// LCPinfo class implementation

LCPinfo::LCPinfo(const std::string& filename, bool fullParsing) :
    _isSeqOpened(false),
    _isCommonOK(false),
    _isCamDataOK(false),
    _inAlternateLensIDs(false),
    _inAlternateLensNames(false),
    _waitPerspModeldescription(false),
    _getText(false),
    _modelCount(0),
    Author(""),
    Make(""),
    Model(""),
    UniqueCameraModel(""),
    CameraRawProfile(false),
    LensInfo(""),
    CameraPrettyName(""),
    LensPrettyName(""),
    ProfileName(""),
    SensorFormatFactor(1.f),
    ImageWidth(0),
    ImageLength(0)
{
    load(filename, fullParsing);
}

void LCPinfo::load(const std::string& filename, bool fullParsing)
{
    XML_Parser parser = XML_ParserCreate(nullptr);

    if (!parser) {
        throw std::runtime_error("Couldn't allocate memory for XML parser");
    }

    if (fullParsing)
    {
        XML_SetElementHandler(parser, XmlStartHandler, XmlEndHandler);
    }
    else
    {
        XML_SetElementHandler(parser, XmlStartHandlerCommonOnly, XmlEndHandlerCommonOnly);
    }
    
    XML_SetCharacterDataHandler(parser, XmlTextHandler);
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

        v_isDistortionValid.push_back(currParam.isFisheye() ? !currParam.fisheyeParams.isEmpty : !currParam.perspParams.isEmpty);
        v_isVignetteValid.push_back(currParam.hasVignetteParams() && !currParam.vignParams.isEmpty);

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
            if (v_lensParams[iHigh].camData.ApertureValue > v_lensParams[iLow].camData.ApertureValue)
            {
                weightLow = (v_lensParams[iHigh].camData.ApertureValue - settings.ApertureValue) /
                            (v_lensParams[iHigh].camData.ApertureValue - v_lensParams[iLow].camData.ApertureValue);
            }
            else if (v_lensParams[iHigh].camData.ApertureValue == v_lensParams[iLow].camData.ApertureValue)
            {
                if (v_lensParams[iHigh].camData.ApertureValue < settings.ApertureValue)
                {
                    weightLow = 0.f;
                }
                else
                {
                    weightLow = 1.f;
                }
            }
            else
            {
                // Should never occur.
                weightLow = -1.0f;
            }
        }
        else
        {
            if (v_lensParams[iHigh].camData.FocusDistance > v_lensParams[iLow].camData.FocusDistance)
            {
                weightLow = (std::log(v_lensParams[iHigh].camData.FocusDistance) - std::log(settings.FocusDistance)) /
                            (std::log(v_lensParams[iHigh].camData.FocusDistance) - std::log(v_lensParams[iLow].camData.FocusDistance));
            }
            else if (v_lensParams[iHigh].camData.FocusDistance == v_lensParams[iLow].camData.FocusDistance)
            {
                if (v_lensParams[iHigh].camData.FocusDistance < settings.FocusDistance)
                {
                    weightLow = 0.f;
                }
                else
                {
                    weightLow = 1.f;
                }
            }
            else
            {
                // Should never occur.
                weightLow = -1.0f;
            }
        }

        if (v_lensParams[iHigh].camData.FocalLength > v_lensParams[iLow].camData.FocalLength)
        {
            float weightLowFocalLength = (std::log(v_lensParams[iHigh].camData.FocalLength) - std::log(settings.FocalLength)) /
                                         (std::log(v_lensParams[iHigh].camData.FocalLength) - std::log(v_lensParams[iLow].camData.FocalLength));

            if (mode == LCPCorrectionMode::VIGNETTE)
            {
                weightLow = (weightLow != -1.0f) ? 0.5 * (weightLow + weightLowFocalLength) : weightLowFocalLength;
            }
            else
            {
                weightLow = (weightLow != -1.0f) ? 0.2 * weightLow + 0.8 * weightLowFocalLength : weightLowFocalLength;
            }
        }

        return (weightLow != -1.0f);
    }
    else if (iLow == iHigh && iLow != -1)
    {
        weightLow = 1.0;
        return true;
    }
    else if (((mode == LCPCorrectionMode::VIGNETTE && settings.ApertureValue == 0.f) ||
              (mode == LCPCorrectionMode::DISTORTION && settings.FocusDistance == 0.f)) &&
            (v_lensParams[iHigh].camData.FocalLength > v_lensParams[iLow].camData.FocalLength))
    {
        weightLow = (std::log(v_lensParams[iHigh].camData.FocalLength) - std::log(settings.FocalLength)) /
                    (std::log(v_lensParams[iHigh].camData.FocalLength) - std::log(v_lensParams[iLow].camData.FocalLength));
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
        if (p1.hasVignetteParams() && !p1.vignParams.isEmpty && p2.hasVignetteParams() && !p2.vignParams.isEmpty)
        {
            pOut.setVignetteParamsStatus(true);
            pOut.vignParams.FocalLengthX = intp<float>(weightLow, p1.vignParams.FocalLengthX, p2.vignParams.FocalLengthX);
            pOut.vignParams.FocalLengthY = intp<float>(weightLow, p1.vignParams.FocalLengthY, p2.vignParams.FocalLengthY);
            pOut.vignParams.VignetteModelParam1 = intp<float>(weightLow, p1.vignParams.VignetteModelParam1, p2.vignParams.VignetteModelParam1);
            pOut.vignParams.VignetteModelParam2 = intp<float>(weightLow, p1.vignParams.VignetteModelParam2, p2.vignParams.VignetteModelParam2);
            pOut.vignParams.VignetteModelParam3 = intp<float>(weightLow, p1.vignParams.VignetteModelParam3, p2.vignParams.VignetteModelParam3);
            pOut.vignParams.isEmpty = false;
        }
        else
        {
            pOut.setVignetteParamsStatus(false);
            pOut.vignParams.isEmpty = true;
        }
        break;
    }

    case LCPCorrectionMode::DISTORTION: {
        pOut.setFisheyeStatus(p1.isFisheye() && p2.isFisheye() && !p1.fisheyeParams.isEmpty && !p2.fisheyeParams.isEmpty);
        if (pOut.isFisheye())
        {
            pOut.fisheyeParams.FocalLengthX = intp<float>(weightLow, p1.fisheyeParams.FocalLengthX, p2.fisheyeParams.FocalLengthX);
            pOut.fisheyeParams.FocalLengthY = intp<float>(weightLow, p1.fisheyeParams.FocalLengthY, p2.fisheyeParams.FocalLengthY);
            pOut.fisheyeParams.ImageXCenter = intp<float>(weightLow, p1.fisheyeParams.ImageXCenter, p2.fisheyeParams.ImageXCenter);
            pOut.fisheyeParams.ImageYCenter = intp<float>(weightLow, p1.fisheyeParams.ImageYCenter, p2.fisheyeParams.ImageYCenter);
            pOut.fisheyeParams.RadialDistortParam1 = intp<float>(weightLow, p1.fisheyeParams.RadialDistortParam1, p2.fisheyeParams.RadialDistortParam1);
            pOut.fisheyeParams.RadialDistortParam2 = intp<float>(weightLow, p1.fisheyeParams.RadialDistortParam2, p2.fisheyeParams.RadialDistortParam2);
            pOut.fisheyeParams.isEmpty = false;
        }
        else if (!p1.perspParams.isEmpty && !p2.perspParams.isEmpty)
        {
            pOut.perspParams.FocalLengthX = intp<float>(weightLow, p1.perspParams.FocalLengthX, p2.perspParams.FocalLengthX);
            pOut.perspParams.FocalLengthY = intp<float>(weightLow, p1.perspParams.FocalLengthY, p2.perspParams.FocalLengthY);
            pOut.perspParams.ImageXCenter = intp<float>(weightLow, p1.perspParams.ImageXCenter, p2.perspParams.ImageXCenter);
            pOut.perspParams.ImageYCenter = intp<float>(weightLow, p1.perspParams.ImageYCenter, p2.perspParams.ImageYCenter);
            pOut.perspParams.RadialDistortParam1 = intp<float>(weightLow, p1.perspParams.RadialDistortParam1, p2.perspParams.RadialDistortParam1);
            pOut.perspParams.RadialDistortParam2 = intp<float>(weightLow, p1.perspParams.RadialDistortParam2, p2.perspParams.RadialDistortParam2);
            pOut.perspParams.RadialDistortParam3 = intp<float>(weightLow, p1.perspParams.RadialDistortParam3, p2.perspParams.RadialDistortParam3);
            pOut.perspParams.isEmpty = false;
        }
        else
        {
            pOut.fisheyeParams.isEmpty = true;
            pOut.perspParams.isEmpty = true;
        }
        break;
    }
    }
}

void LCPinfo::getDistortionParams(const float& focalLength, const float& focusDistance, LensParam& lparam)
{
    settingsInfo userSettings;
    userSettings.ApertureValue = 0.f;
    userSettings.FocalLength = focalLength;
    userSettings.FocusDistance = focusDistance;

    int iLow, iHigh;
    float weightLow;
    if (search(userSettings, LCPCorrectionMode::DISTORTION, iLow, iHigh, weightLow))
    {
        combine(iLow, iHigh, weightLow, LCPCorrectionMode::DISTORTION, lparam);
    }
}

void LCPinfo::getVignettingParams(const float& focalLength, const float& aperture, LensParam& lparam)
{
    settingsInfo userSettings;
    userSettings.ApertureValue = aperture;
    userSettings.FocalLength = focalLength;
    userSettings.FocusDistance = 0.f;

    int iLow, iHigh;
    float weightLow;
    if (search(userSettings, LCPCorrectionMode::VIGNETTE, iLow, iHigh, weightLow))
    {
        combine(iLow, iHigh, weightLow, LCPCorrectionMode::VIGNETTE, lparam);
    }
}

// Some useful functions when parsing the LCP database

void parseDirectory(const boost::filesystem::path& p, std::vector<boost::filesystem::path>& v)
{
    if (boost::filesystem::is_directory(p))
    {
        for (auto&& x : boost::filesystem::directory_iterator(p))
            parseDirectory(x.path(), v);
    }
    else if (boost::filesystem::is_regular_file(p) && (boost::filesystem::extension(p) == ".lcp"))
    {
        v.push_back(p);
    }
}

std::string reduceString(const std::string& str)
{
    std::string localStr = str;

    // remove all space
    localStr.erase(std::remove(localStr.begin(), localStr.end(), ' '), localStr.end());
    // remove all '/'
    localStr.erase(std::remove(localStr.begin(), localStr.end(), '/'), localStr.end());
    // remove all '.'
    localStr.erase(std::remove(localStr.begin(), localStr.end(), '.'), localStr.end());
    // remove all '_'
    localStr.erase(std::remove(localStr.begin(), localStr.end(), '_'), localStr.end());
    // remove all '-'
    localStr.erase(std::remove(localStr.begin(), localStr.end(), '-'), localStr.end());
    // remove all '*'
    localStr.erase(std::remove(localStr.begin(), localStr.end(), '*'), localStr.end());
    // remove all ','
    localStr.erase(std::remove(localStr.begin(), localStr.end(), ','), localStr.end());
    // remove all ';'
    localStr.erase(std::remove(localStr.begin(), localStr.end(), ';'), localStr.end());
    // remove all ':'
    localStr.erase(std::remove(localStr.begin(), localStr.end(), ':'), localStr.end());
    // to lowercase
    boost::algorithm::to_lower(localStr);

    return localStr;
}

std::vector<std::string> reduceStrings(std::vector<std::string>& v_str)
{
    std::vector<std::string> v_localStr;
    for (auto& s : v_str)
    {
        v_localStr.push_back(reduceString(s));
    }
    return v_localStr;
}

// LCP database parsing implementation

bool findLCPInfo(const std::string& dbDirectoryname, const std::string& cameraModelOrMaker, const std::string& lensModel, const int& lensID, int rawMode, LCPinfo& lcpData, bool omitCameraModel)
{
    std::vector<boost::filesystem::path> v_lcpFilename;
    parseDirectory(dbDirectoryname, v_lcpFilename);

    return findLCPInfo(v_lcpFilename, cameraModelOrMaker, lensModel, lensID, rawMode, lcpData, omitCameraModel);
}

bool findLCPInfo(const std::vector<boost::filesystem::path>& lcpFilenames, const std::string& cameraModelOrMaker, const std::string& lensModel, const int& lensID, int rawMode, LCPinfo& lcpData, bool omitCameraModel)
{
    std::string reducedCameraModel = reduceString(cameraModelOrMaker);
    std::string reducedLensModel = reduceString(lensModel);

    bool lcpFound = false;
    size_t lcpIndex = 0;
    while ((lcpIndex < lcpFilenames.size()) && !lcpFound)
    {
        LCPinfo lcp(lcpFilenames[lcpIndex].string(), false);

        std::string reducedCameraModelLCP = reduceString(omitCameraModel ? lcp.getCameraMaker() : lcp.getCameraModel());
        std::string reducedCameraPrettyNameLCP = reduceString(lcp.getCameraPrettyName());
        std::string reducedLensPrettyNameLCP = reduceString(lcp.getLensPrettyName());

        std::vector<std::string> lensModelsLCP;
        lcp.getLensModels(lensModelsLCP);
        std::vector<std::string> reducedLensModelsLCP = reduceStrings(lensModelsLCP);

        std::vector<int> lensIDsLCP;
        lcp.getLensIDs(lensIDsLCP);

        bool cameraOK = ((reducedCameraModelLCP == reducedCameraModel) || (reducedCameraPrettyNameLCP == reducedCameraModel));
        bool lensOK = ((reducedLensPrettyNameLCP == reducedLensModel) ||
                       (std::find(reducedLensModelsLCP.begin(), reducedLensModelsLCP.end(), reducedLensModel) != reducedLensModelsLCP.end()));
        bool lensIDOK = (std::find(lensIDsLCP.begin(), lensIDsLCP.end(), lensID) != lensIDsLCP.end());
        bool isRaw = lcp.isRawProfile();

        lcpFound = (cameraOK && lensOK && lensIDOK && ((isRaw && rawMode < 2) || (!isRaw && (rawMode%2 == 0))));
        if (lcpFound)
        {
            lcpData.load(lcpFilenames[lcpIndex].string(), true);
        }
        else
        {
            lcpIndex++;
        }
    }

    return lcpFound;
}


