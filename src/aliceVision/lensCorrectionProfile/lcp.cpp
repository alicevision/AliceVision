#include <iostream>

#include "boost/filesystem.hpp"
#include "boost/algorithm/string.hpp"

#include <aliceVision/numeric/numeric.hpp>

#include <expat.h>
#include "lcp.hpp"

template<typename T>
constexpr T interpolate(T a, T b, T c)
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

    if (LCPdata->_currReadingState == LCPReadingState::WaitSequence)
    {
        if (element == "photoshop:CameraProfiles")
        {
            LCPdata->_currReadingState = LCPReadingState::FillCommonAndCameraSettings;
        }
    }
    else if (LCPdata->_currReadingState == LCPReadingState::FillCommonAndCameraSettings)
    {
        if ((element == "stCamera:PerspectiveModel") || (element == "stCamera:FisheyeModel"))
        {
            LCPdata->_currReadingState = LCPReadingState::FillGeometryModel;
            LCPdata->currLensParam.setFisheyeStatus(element == "stCamera:FisheyeModel");
            LCPdata->increaseModelCount();
            if (attr[0])
            {
                for (int i = 0; attr[i]; i += 2)
                {
                    std::string key(attr[i]);
                    std::string value(attr[i + 1]);

                    LCPdata->_currText = value;
                    if (LCPdata->currLensParam.isFisheye())
                    {
                        LCPdata->setFisheyeModel(key);
                    }
                    else
                    {
                        LCPdata->setRectilinearModel(LCPdata->currLensParam.perspParams, key);
                    }
                }
            }
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
        }
        else if (element == "rdf:Description")
        {
            for (int i = 0; attr[i]; i += 2)
            {
                std::string key(attr[i]);
                std::string value(attr[i + 1]);

                LCPdata->_currText = value;
                LCPdata->setCommonSettings(key);
                LCPdata->setCameraSettings(key);
            }
        }
        else
        {
            LCPdata->setGetText();
        }
    }
    else if (LCPdata->_currReadingState == LCPReadingState::FillGeometryModel)
    {
        if (element == "stCamera:VignetteModel")
        {
            LCPdata->_currReadingState = LCPReadingState::FillVignetteModel;
            LCPdata->currLensParam.setVignetteParamsStatus(true);
            if (attr[0])
            {
                for (int i = 0; attr[i]; i += 2)
                {
                    std::string key(attr[i]);
                    std::string value(attr[i + 1]);

                    LCPdata->_currText = value;
                    LCPdata->setVignetteModel(key);
                }
            }
        }
        else if (element == "stCamera:ChromaticGreenModel")
        {
            LCPdata->_currReadingState = LCPReadingState::FillChromaticGreenModel;
            LCPdata->currLensParam.setChromaticParamsStatus(true);
        }
        else if (element == "stCamera:ChromaticRedGreenModel")
        {
            LCPdata->_currReadingState = LCPReadingState::FillChromaticRedGreenModel;
        }
        else if (element == "stCamera:ChromaticBlueGreenModel")
        {
            LCPdata->_currReadingState = LCPReadingState::FillChromaticBlueGreenModel;
        }
        else if (attr[0])
        {
            for (int i = 0; attr[i]; i += 2)
            {
                std::string key(attr[i]);
                std::string value(attr[i + 1]);

                LCPdata->_currText = value;
                if (LCPdata->currLensParam.isFisheye())
                {
                    LCPdata->setFisheyeModel(key);
                }
                else
                {
                    LCPdata->setRectilinearModel(LCPdata->currLensParam.perspParams, key);
                }
            }
        }
        else
        {
            LCPdata->setGetText();
        }
    }
    else if (LCPdata->_currReadingState == LCPReadingState::FillVignetteModel)
    {
        if (!attr[0])
        {
            LCPdata->setGetText();
        }
        else
        {
            for (int i = 0; attr[i]; i += 2)
            {
                std::string key(attr[i]);
                std::string value(attr[i + 1]);

                LCPdata->_currText = value;
                LCPdata->setVignetteModel(key);
            }
        } 
    }
    else if ((LCPdata->_currReadingState == LCPReadingState::FillChromaticGreenModel) ||
        (LCPdata->_currReadingState == LCPReadingState::FillChromaticBlueGreenModel) ||
        (LCPdata->_currReadingState == LCPReadingState::FillChromaticRedGreenModel))
    {
        if (!attr[0])
        {
            LCPdata->setGetText();
        }
        else
        {
            for (int i = 0; attr[i]; i += 2)
            {
                std::string key(attr[i]);
                std::string value(attr[i + 1]);

                LCPdata->_currText = value;
                if (LCPdata->_currReadingState == LCPReadingState::FillChromaticGreenModel)
                {
                    LCPdata->setRectilinearModel(LCPdata->currLensParam.ChromaticGreenParams, key);
                }
                else if (LCPdata->_currReadingState == LCPReadingState::FillChromaticRedGreenModel)
                {
                    LCPdata->setRectilinearModel(LCPdata->currLensParam.ChromaticRedGreenParams, key);
                }
                else if (LCPdata->_currReadingState == LCPReadingState::FillChromaticBlueGreenModel)
                {
                    LCPdata->setRectilinearModel(LCPdata->currLensParam.ChromaticBlueGreenParams, key);
                }

            }
        }
    }
    else if (LCPdata->_currReadingState == LCPReadingState::WaitNewModel)
    {
        if ((element == "stCamera:PerspectiveModel") || (element == "stCamera:FisheyeModel"))
        {
            LCPdata->_currReadingState = LCPReadingState::FillGeometryModel;
            LCPdata->currLensParam.setFisheyeStatus(element == "stCamera:FisheyeModel");
            LCPdata->increaseModelCount();
            if (attr[0])
            {
                for (int i = 0; attr[i]; i += 2)
                {
                    std::string key(attr[i]);
                    std::string value(attr[i + 1]);

                    LCPdata->_currText = value;
                    if (LCPdata->currLensParam.isFisheye())
                    {
                        LCPdata->setFisheyeModel(key);
                    }
                    else
                    {
                        LCPdata->setRectilinearModel(LCPdata->currLensParam.perspParams, key);
                    }
                }
            }
        }
        else if (element == "rdf:Description")
        {
            for (int i = 0; attr[i]; i += 2)
            {
                std::string key(attr[i]);
                std::string value(attr[i + 1]);

                LCPdata->_currText = value;
                LCPdata->setCameraSettings(key);
            }
        }
        else
        {
            LCPdata->setGetText();
        }
    }

}  /* End of start handler */

void XMLCALL LCPinfo::XmlEndHandler(void* pLCPinfo, const char* el)
{
    LCPinfo* LCPdata = static_cast<LCPinfo*>(pLCPinfo);

    std::string element(el);

    if (LCPdata->_currReadingState == LCPReadingState::FillCommonAndCameraSettings)
    {
        if (!LCPdata->_currText.empty())
        {
            LCPdata->setCommonSettings(element);
            LCPdata->setCameraSettings(element);
            LCPdata->_currText.clear();
        }
    }
    else if (LCPdata->_currReadingState == LCPReadingState::WaitNewModel)
    {
        if (!LCPdata->_currText.empty())
        {
            LCPdata->setCameraSettings(element);
            LCPdata->_currText.clear();
        }
    }
    else if (LCPdata->_currReadingState == LCPReadingState::FillGeometryModel)
    {
        if ((element == "stCamera:PerspectiveModel") || (element == "stCamera:FisheyeModel"))
        {
            LCPdata->_currReadingState = LCPReadingState::WaitNewModel;

            LCPdata->storeCurrParams();
            LCPdata->currLensParam.clear();
        }
        else if (!LCPdata->_currText.empty())
        {
            if (LCPdata->currLensParam.isFisheye())
            {
                LCPdata->setFisheyeModel(element);
            }
            else
            {
                LCPdata->setRectilinearModel(LCPdata->currLensParam.perspParams, element);
            }           
            LCPdata->_currText.clear();
        }
    }
    else if (LCPdata->_currReadingState == LCPReadingState::FillChromaticGreenModel)
    {
        if (element == "stCamera:ChromaticGreenModel")
        {
            LCPdata->_currReadingState = LCPReadingState::FillGeometryModel;
        }
        else if (!LCPdata->_currText.empty())
        {
            LCPdata->setRectilinearModel(LCPdata->currLensParam.ChromaticGreenParams, element);
            LCPdata->_currText.clear();
        }
    }
    else if (LCPdata->_currReadingState == LCPReadingState::FillChromaticRedGreenModel)
    {
        if (element == "stCamera:ChromaticRedGreenModel")
        {
            LCPdata->_currReadingState = LCPReadingState::FillGeometryModel;
        }
        else if (!LCPdata->_currText.empty())
        {
            LCPdata->setRectilinearModel(LCPdata->currLensParam.ChromaticRedGreenParams, element);
            LCPdata->_currText.clear();
        }
    }
    else if (LCPdata->_currReadingState == LCPReadingState::FillChromaticBlueGreenModel)
    {
        if (element == "stCamera:ChromaticBlueGreenModel")
        {
            LCPdata->_currReadingState = LCPReadingState::FillGeometryModel;
        }
        else if (!LCPdata->_currText.empty())
        {
            LCPdata->setRectilinearModel(LCPdata->currLensParam.ChromaticBlueGreenParams, element);
            LCPdata->_currText.clear();
        }
    }
    else if (LCPdata->_currReadingState == LCPReadingState::FillVignetteModel)
    {
        if (element == "stCamera:VignetteModel")
        {
            LCPdata->_currReadingState = LCPReadingState::FillGeometryModel;
        }
        else if (!LCPdata->_currText.empty())
        {
            LCPdata->setVignetteModel(element);
            LCPdata->_currText.clear();
        }
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
        else //if (LCPdata->_currReadingState == LCPReadingState::FillCommonAndCameraSettings)
        {
            LCPdata->_currText = localtextbuf.str();
        }
        LCPdata->unsetGetText();
    }
}

void XMLCALL LCPinfo::XmlStartHandlerCommonOnly(void* pLCPinfo, const char* el, const char** attr)
{
    LCPinfo* LCPdata = static_cast<LCPinfo*>(pLCPinfo);

    std::string element(el);

    if (LCPdata->_currReadingState == LCPReadingState::WaitSequence)
    {
        if (element == "photoshop:CameraProfiles")
        {
            LCPdata->_currReadingState = LCPReadingState::FillCommonAndCameraSettings;
        }
    }
    else if (LCPdata->_currReadingState == LCPReadingState::FillCommonAndCameraSettings)
    {
        if (element == "stCamera:AlternateLensIDs")
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
        }
        else if (element == "rdf:Description")
        {
            for (int i = 0; attr[i]; i += 2)
            {
                std::string key(attr[i]);
                std::string value(attr[i + 1]);

                LCPdata->_currText = value;
                LCPdata->setCommonSettings(key);
            }
        }
        else
        {
            LCPdata->setGetText();
        }
    }

}  /* End of start handler */

void XMLCALL LCPinfo::XmlEndHandlerCommonOnly(void* pLCPinfo, const char* el)
{
    LCPinfo* LCPdata = static_cast<LCPinfo*>(pLCPinfo);

    std::string element(el);

    if (LCPdata->_currReadingState == LCPReadingState::FillCommonAndCameraSettings)
    {
        if (!LCPdata->_currText.empty())
        {
            LCPdata->setCommonSettings(element);
            LCPdata->_currText.clear();
        }
    }
}  /* End of end handler */

/*
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
/*
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
            pOut.vignParams.FocalLengthX = interpolate<float>(weightLow, p1.vignParams.FocalLengthX, p2.vignParams.FocalLengthX);
            pOut.vignParams.FocalLengthY = interpolate<float>(weightLow, p1.vignParams.FocalLengthY, p2.vignParams.FocalLengthY);
            pOut.vignParams.VignetteModelParam1 = interpolate<float>(weightLow, p1.vignParams.VignetteModelParam1, p2.vignParams.VignetteModelParam1);
            pOut.vignParams.VignetteModelParam2 = interpolate<float>(weightLow, p1.vignParams.VignetteModelParam2, p2.vignParams.VignetteModelParam2);
            pOut.vignParams.VignetteModelParam3 = interpolate<float>(weightLow, p1.vignParams.VignetteModelParam3, p2.vignParams.VignetteModelParam3);
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
            pOut.fisheyeParams.FocalLengthX = interpolate<float>(weightLow, p1.fisheyeParams.FocalLengthX, p2.fisheyeParams.FocalLengthX);
            pOut.fisheyeParams.FocalLengthY = interpolate<float>(weightLow, p1.fisheyeParams.FocalLengthY, p2.fisheyeParams.FocalLengthY);
            pOut.fisheyeParams.ImageXCenter = interpolate<float>(weightLow, p1.fisheyeParams.ImageXCenter, p2.fisheyeParams.ImageXCenter);
            pOut.fisheyeParams.ImageYCenter = interpolate<float>(weightLow, p1.fisheyeParams.ImageYCenter, p2.fisheyeParams.ImageYCenter);
            pOut.fisheyeParams.RadialDistortParam1 = interpolate<float>(weightLow, p1.fisheyeParams.RadialDistortParam1, p2.fisheyeParams.RadialDistortParam1);
            pOut.fisheyeParams.RadialDistortParam2 = interpolate<float>(weightLow, p1.fisheyeParams.RadialDistortParam2, p2.fisheyeParams.RadialDistortParam2);
            pOut.fisheyeParams.isEmpty = false;
        }
        else if (!p1.perspParams.isEmpty && !p2.perspParams.isEmpty)
        {
            pOut.perspParams.FocalLengthX = interpolate<float>(weightLow, p1.perspParams.FocalLengthX, p2.perspParams.FocalLengthX);
            pOut.perspParams.FocalLengthY = interpolate<float>(weightLow, p1.perspParams.FocalLengthY, p2.perspParams.FocalLengthY);
            pOut.perspParams.ImageXCenter = interpolate<float>(weightLow, p1.perspParams.ImageXCenter, p2.perspParams.ImageXCenter);
            pOut.perspParams.ImageYCenter = interpolate<float>(weightLow, p1.perspParams.ImageYCenter, p2.perspParams.ImageYCenter);
            pOut.perspParams.RadialDistortParam1 = interpolate<float>(weightLow, p1.perspParams.RadialDistortParam1, p2.perspParams.RadialDistortParam1);
            pOut.perspParams.RadialDistortParam2 = interpolate<float>(weightLow, p1.perspParams.RadialDistortParam2, p2.perspParams.RadialDistortParam2);
            pOut.perspParams.RadialDistortParam3 = interpolate<float>(weightLow, p1.perspParams.RadialDistortParam3, p2.perspParams.RadialDistortParam3);
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
    if(search(userSettings, LCPCorrectionMode::DISTORTION, iLow, iHigh, weightLow))
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




void undistortVignetting(aliceVision::image::Image<aliceVision::image::RGBAfColor>& img, const std::vector<float>& vparam, const float focX, const float focY, const float imageXCenter = 0.5, const float imageYCenter = 0.5)
{

#pragma omp parallel for
    for (int j = 0; j < img.Height(); ++j)
        for (int i = 0; i < img.Width(); ++i)
        {
            const aliceVision::Vec2 pix(i, j);
            // compute coordinates with distortion
            const Vec2 disto_pix = intrinsicPtr->get_d_pixel(undisto_pix + ppCorrection);

            aliceVision::Vec2 pp = getPrincipalPoint();

            np(0) = (p(0) - pp(0)) / _scale(0);
            np(1) = (p(1) - pp(1)) / _scale(1);




            // pick pixel if it is in the image domain
            if (imageIn.Contains(disto_pix(1), disto_pix(0)))
                image_ud(j, i) = sampler(imageIn, disto_pix(1), disto_pix(0));
        }




}

