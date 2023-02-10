#include "lcp.hpp"

#include <aliceVision/system/Logger.hpp>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

#include <expat.h>

#include <iostream>
#include <math.h>

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
    if(fullParsing)
    {
        ALICEVISION_LOG_INFO("Load LCP file: \"" << filename << "\"");
    }
    else
    {
        ALICEVISION_LOG_TRACE("Load LCP header: \"" << filename << "\"");
    }
    XML_Parser parser = XML_ParserCreate(nullptr);

    if (!parser)
    {
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

    // Search the best focal lengths with respect to the target one in settings
    // At the end of the loop, iLow is the index in v_lensParam of the lens parameter set with the closest focal length, lower or equal to the one in settings.
    // iHigh is the index in v_lensParam of the lens parameter set with the closest focal length, greater or equal to the one in settings.
    // In case of missing aperture value target and search mode is vignetting, if several parameter sets have the same ideal focal length,
    // iLow and iHigh point out the ones with the lowest aperture value.
    // If search mode is not vignetting and no focus distance target is given the highest focus value is targeted.
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
                f <= settings.FocalLength // v_lensParams[iLow] focal length must always stay lower than or equal to the targeted focal length
                && (
                    iLow == -1 // iLow not yet initialized
                    || f > v_lensParams[iLow].camData.FocalLength // Better candidate than the stored one (higher than but still lower than or equal to the target value)
                    || (f == v_lensParams[iLow].camData.FocalLength && // Same focal lenth value as the stored one
                          (settings.FocusDistance == 0 && // No focus info
                           mode != LCPCorrectionMode::VIGNETTE && // search for other model than vignetting (geometry, chromatic aberation)
                           v_lensParams[iLow].camData.FocusDistance > currParam.camData.FocusDistance) // Higher focus distance value than the stored one
                       || (settings.ApertureValue == 0 && // No aperture info
                           mode == LCPCorrectionMode::VIGNETTE && // search for vignetting model
                           v_lensParams[iLow].camData.ApertureValue < currParam.camData.ApertureValue)) // Lower aperture value than the stored one
                    )
               )
            {
                iLow = i;
            }

            if (
                f >= settings.FocalLength // v_lensParams[iHigh] focal length must always stay greater than or equal to the targeted focal length
                && (
                    iHigh == -1 // iHigh not yet initialized
                    || f < v_lensParams[iHigh].camData.FocalLength // Better candidate than the stored one (lower than but still higher than or equal to the target value)
                    || (f == v_lensParams[iHigh].camData.FocalLength && // Same focal lenth value as the stored one
                          (settings.FocusDistance == 0 && // No focus info
                           mode != LCPCorrectionMode::VIGNETTE && // search for other model than vignetting (geometry, chromatic aberation)
                           v_lensParams[iHigh].camData.FocusDistance > currParam.camData.FocusDistance)
                       || (settings.ApertureValue == 0 && // No aperture info
                           mode == LCPCorrectionMode::VIGNETTE && // search for vignetting model
                           v_lensParams[iHigh].camData.ApertureValue < currParam.camData.ApertureValue)) // Lower aperture value than the stored one
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

    // Amongst all possible low and high candidates, the closest from the targeted focus or aperture value are selected
    if (iLow != -1 && iHigh != -1 && iLow != iHigh && settingsOK)
    {
        std::vector<int> candidatesLow;
        std::vector<int> candidatesHigh;

        // Search all possible low and high candidates
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

        // If search mode is vignetting select low and high candidates with the closest aperture value.
        // If search mode is not vignetting select low and high candidates with the closest focus distance value.
        for (int i = 0; i < candidatesLow.size(); ++i)
        {
            bool update = false;
            if (mode == LCPCorrectionMode::VIGNETTE)
            {
                float currAperture = v_lensParams[iLow].camData.ApertureValue;
                float candidateAperture = v_lensParams[candidatesLow[i]].camData.ApertureValue;

                update = (candidateAperture >= settings.ApertureValue && candidateAperture < currAperture && currAperture > settings.ApertureValue) ||
                         (candidateAperture <= settings.ApertureValue &&
                          (currAperture > settings.ApertureValue || std::fabs(settings.ApertureValue - candidateAperture) < std::fabs(settings.ApertureValue - currAperture)));
            }
            else
            {
                float currFocus = v_lensParams[iLow].camData.FocusDistance;
                float candidateFocus = v_lensParams[candidatesLow[i]].camData.FocusDistance;

                update = (candidateFocus >= settings.FocusDistance && candidateFocus < currFocus && currFocus > settings.FocusDistance) ||
                         (candidateFocus <= settings.FocusDistance &&
                          (currFocus > settings.FocusDistance || std::fabs(settings.FocusDistance - candidateFocus) < std::fabs(settings.FocusDistance - currFocus)));
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
                          (currAperture < settings.ApertureValue || std::fabs(settings.ApertureValue - candidateAperture) < std::fabs(settings.ApertureValue - currAperture)));
            }
            else
            {
                float currFocus = v_lensParams[iHigh].camData.FocusDistance;
                float candidateFocus = v_lensParams[candidatesHigh[i]].camData.FocusDistance;

                update = (candidateFocus <= settings.FocusDistance && candidateFocus > currFocus && currFocus < settings.FocusDistance) ||
                         (candidateFocus >= settings.FocusDistance &&
                          (currFocus < settings.FocusDistance || std::fabs(settings.FocusDistance - candidateFocus) < std::fabs(settings.FocusDistance - currFocus)));
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
    else if ((mode == LCPCorrectionMode::VIGNETTE) && (settings.ApertureValue == 0.f))
    {
        if (v_lensParams[iHigh].camData.FocalLength > v_lensParams[iLow].camData.FocalLength)
        {
            weightLow = (std::log(v_lensParams[iHigh].camData.FocalLength) - std::log(settings.FocalLength)) /
                (std::log(v_lensParams[iHigh].camData.FocalLength) - std::log(v_lensParams[iLow].camData.FocalLength));
            return true;
        }
        else if (v_lensParams[iHigh].camData.FocalLength == v_lensParams[iLow].camData.FocalLength)
        {
            // case of a prime lens without any aperture value available in metadata
            // we keep the highest aperture value
            weightLow = (v_lensParams[iLow].camData.ApertureValue >= v_lensParams[iHigh].camData.ApertureValue) ? 1.0 : 0.0;
            return true;
        }
    }
    else if ((mode == LCPCorrectionMode::DISTORTION) && (settings.FocusDistance == 0.f))
    {
        if (v_lensParams[iHigh].camData.FocalLength > v_lensParams[iLow].camData.FocalLength)
        {
            weightLow = (std::log(v_lensParams[iHigh].camData.FocalLength) - std::log(settings.FocalLength)) /
                (std::log(v_lensParams[iHigh].camData.FocalLength) - std::log(v_lensParams[iLow].camData.FocalLength));
            return true;
        }
        else if (v_lensParams[iHigh].camData.FocalLength == v_lensParams[iLow].camData.FocalLength)
        {
            // case of a prime lens without any focus value available in metadata
            // we retain the lowest focus value
            weightLow = (v_lensParams[iLow].camData.FocusDistance <= v_lensParams[iHigh].camData.FocusDistance) ? 1.0 : 0.0;
            return true;
        }
    }

    return false;
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
            pOut.vignParams.ImageXCenter = interpolate<float>(weightLow, p1.vignParams.ImageXCenter, p2.vignParams.ImageXCenter);
            pOut.vignParams.ImageYCenter = interpolate<float>(weightLow, p1.vignParams.ImageYCenter, p2.vignParams.ImageYCenter);
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

    case LCPCorrectionMode::CA: {
        pOut.ChromaticGreenParams.FocalLengthX = interpolate<float>(weightLow, p1.ChromaticGreenParams.FocalLengthX, p2.ChromaticGreenParams.FocalLengthX);
        pOut.ChromaticGreenParams.FocalLengthY = interpolate<float>(weightLow, p1.ChromaticGreenParams.FocalLengthY, p2.ChromaticGreenParams.FocalLengthY);
        pOut.ChromaticGreenParams.ImageXCenter = interpolate<float>(weightLow, p1.ChromaticGreenParams.ImageXCenter, p2.ChromaticGreenParams.ImageXCenter);
        pOut.ChromaticGreenParams.ImageYCenter = interpolate<float>(weightLow, p1.ChromaticGreenParams.ImageYCenter, p2.ChromaticGreenParams.ImageYCenter);
        pOut.ChromaticGreenParams.RadialDistortParam1 = interpolate<float>(weightLow, p1.ChromaticGreenParams.RadialDistortParam1, p2.ChromaticGreenParams.RadialDistortParam1);
        pOut.ChromaticGreenParams.RadialDistortParam2 = interpolate<float>(weightLow, p1.ChromaticGreenParams.RadialDistortParam2, p2.ChromaticGreenParams.RadialDistortParam2);
        pOut.ChromaticGreenParams.RadialDistortParam3 = interpolate<float>(weightLow, p1.ChromaticGreenParams.RadialDistortParam3, p2.ChromaticGreenParams.RadialDistortParam3);
        pOut.ChromaticGreenParams.isEmpty = false;
        pOut.ChromaticBlueGreenParams.FocalLengthX = interpolate<float>(weightLow, p1.ChromaticBlueGreenParams.FocalLengthX, p2.ChromaticBlueGreenParams.FocalLengthX);
        pOut.ChromaticBlueGreenParams.FocalLengthY = interpolate<float>(weightLow, p1.ChromaticBlueGreenParams.FocalLengthY, p2.ChromaticBlueGreenParams.FocalLengthY);
        pOut.ChromaticBlueGreenParams.ImageXCenter = interpolate<float>(weightLow, p1.ChromaticBlueGreenParams.ImageXCenter, p2.ChromaticBlueGreenParams.ImageXCenter);
        pOut.ChromaticBlueGreenParams.ImageYCenter = interpolate<float>(weightLow, p1.ChromaticBlueGreenParams.ImageYCenter, p2.ChromaticBlueGreenParams.ImageYCenter);
        pOut.ChromaticBlueGreenParams.RadialDistortParam1 = interpolate<float>(weightLow, p1.ChromaticBlueGreenParams.RadialDistortParam1, p2.ChromaticBlueGreenParams.RadialDistortParam1);
        pOut.ChromaticBlueGreenParams.RadialDistortParam2 = interpolate<float>(weightLow, p1.ChromaticBlueGreenParams.RadialDistortParam2, p2.ChromaticBlueGreenParams.RadialDistortParam2);
        pOut.ChromaticBlueGreenParams.RadialDistortParam3 = interpolate<float>(weightLow, p1.ChromaticBlueGreenParams.RadialDistortParam3, p2.ChromaticBlueGreenParams.RadialDistortParam3);
        pOut.ChromaticBlueGreenParams.isEmpty = false;
        pOut.ChromaticRedGreenParams.FocalLengthX = interpolate<float>(weightLow, p1.ChromaticRedGreenParams.FocalLengthX, p2.ChromaticRedGreenParams.FocalLengthX);
        pOut.ChromaticRedGreenParams.FocalLengthY = interpolate<float>(weightLow, p1.ChromaticRedGreenParams.FocalLengthY, p2.ChromaticRedGreenParams.FocalLengthY);
        pOut.ChromaticRedGreenParams.ImageXCenter = interpolate<float>(weightLow, p1.ChromaticRedGreenParams.ImageXCenter, p2.ChromaticRedGreenParams.ImageXCenter);
        pOut.ChromaticRedGreenParams.ImageYCenter = interpolate<float>(weightLow, p1.ChromaticRedGreenParams.ImageYCenter, p2.ChromaticRedGreenParams.ImageYCenter);
        pOut.ChromaticRedGreenParams.RadialDistortParam1 = interpolate<float>(weightLow, p1.ChromaticRedGreenParams.RadialDistortParam1, p2.ChromaticRedGreenParams.RadialDistortParam1);
        pOut.ChromaticRedGreenParams.RadialDistortParam2 = interpolate<float>(weightLow, p1.ChromaticRedGreenParams.RadialDistortParam2, p2.ChromaticRedGreenParams.RadialDistortParam2);
        pOut.ChromaticRedGreenParams.RadialDistortParam3 = interpolate<float>(weightLow, p1.ChromaticRedGreenParams.RadialDistortParam3, p2.ChromaticRedGreenParams.RadialDistortParam3);
        pOut.ChromaticRedGreenParams.isEmpty = false;
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

void LCPinfo::setCommonSettings(const std::string& name)
{
    if (name == "stCamera:Author")
        Author = _currText;
    else if (name == "stCamera:ProfileName")
        ProfileName = _currText;
    else if (name == "stCamera:Make")
        Make = _currText;
    else if (name == "stCamera:Model")
        Model = _currText;
    else if (name == "stCamera:Lens")
        Lens.push_back(_currText);
    else if (name == "stCamera:LensID")
        LensID.push_back(std::stoi(_currText.c_str()));
    else if (name == "stCamera:LensInfo")
        LensInfo = _currText;
    else if (name == "stCamera:ImageWidth")
        ImageWidth = std::stoi(_currText.c_str());
    else if (name == "stCamera:ImageLength")
        ImageLength = std::stoi(_currText.c_str());
    else if (name == "stCamera:XResolution")
        XResolution = std::stof(_currText.c_str());
    else if (name == "stCamera:YResolution")
        YResolution = std::stof(_currText.c_str());
    else if (name == "stCamera:LensPrettyName")
        LensPrettyName = _currText;
    else if (name == "stCamera:CameraPrettyName")
        CameraPrettyName = _currText;
    else if (name == "stCamera:CameraRawProfile")
        CameraRawProfile = ((_currText == "true") || (_currText == "True"));
    else if (name == "stCamera:SensorFormatFactor")
        SensorFormatFactor = std::stof(_currText.c_str());
}

void LCPinfo::setCameraSettings(const std::string& name)
{
    if (name == "stCamera:FocalLength")
        currLensParam.camData.FocalLength = std::stof(_currText.c_str());
    else if (name == "stCamera:ApertureValue")
        currLensParam.camData.ApertureValue = std::stof(_currText.c_str());
    else if (name == "stCamera:FocusDistance")
        currLensParam.camData.FocusDistance = std::stof(_currText.c_str());
}

void LCPinfo::setRectilinearModel(RectilinearModel& model, const std::string& name)
{
    model.isEmpty = false;
    if (name == "stCamera:Version")
        model.Version = std::stoi(_currText.c_str());
    else if (name == "stCamera:FocalLengthX")
        model.FocalLengthX = std::stof(_currText.c_str());
    else if (name == "stCamera:FocalLengthY")
        model.FocalLengthY = std::stof(_currText.c_str());
    else if (name == "stCamera:ImageXCenter")
        model.ImageXCenter = std::stof(_currText.c_str());
    else if (name == "stCamera:ImageYCenter")
        model.ImageYCenter = std::stof(_currText.c_str());
    else if (name == "stCamera:RadialDistortParam1")
        model.RadialDistortParam1 = std::stof(_currText.c_str());
    else if (name == "stCamera:RadialDistortParam2")
        model.RadialDistortParam2 = std::stof(_currText.c_str());
    else if (name == "stCamera:RadialDistortParam3")
        model.RadialDistortParam3 = std::stof(_currText.c_str());
    else if (name == "stCamera:TangentiallDistortParam1")
        model.RadialDistortParam1 = std::stof(_currText.c_str());
    else if (name == "stCamera:TangentiallDistortParam2")
        model.RadialDistortParam2 = std::stof(_currText.c_str());
    else if (name == "stCamera:ScaleFactor")
        model.ScaleFactor = std::stof(_currText.c_str());
}

void LCPinfo::setFisheyeModel(const std::string& name)
{
    currLensParam.fisheyeParams.isEmpty = false;
    if (name == "stCamera:Version")
        currLensParam.fisheyeParams.Version = std::stoi(_currText.c_str());
    else if (name == "stCamera:FocalLengthX")
        currLensParam.fisheyeParams.FocalLengthX = std::stof(_currText.c_str());
    else if (name == "stCamera:FocalLengthY")
        currLensParam.fisheyeParams.FocalLengthY = std::stof(_currText.c_str());
    else if (name == "stCamera:ImageXCenter")
        currLensParam.fisheyeParams.ImageXCenter = std::stof(_currText.c_str());
    else if (name == "stCamera:ImageYCenter")
        currLensParam.fisheyeParams.ImageYCenter = std::stof(_currText.c_str());
    else if (name == "stCamera:RadialDistortParam1")
        currLensParam.fisheyeParams.RadialDistortParam1 = std::stof(_currText.c_str());
    else if (name == "stCamera:RadialDistortParam2")
        currLensParam.fisheyeParams.RadialDistortParam2 = std::stof(_currText.c_str());
}

void LCPinfo::setVignetteModel(const std::string& name)
{
    currLensParam.vignParams.isEmpty = false;
    if (name == "stCamera:FocalLengthX")
        currLensParam.vignParams.FocalLengthX = std::stof(_currText.c_str());
    else if (name == "stCamera:FocalLengthY")
        currLensParam.vignParams.FocalLengthY = std::stof(_currText.c_str());
    else if (name == "stCamera:ImageXCenter")
        currLensParam.vignParams.ImageXCenter = std::stof(_currText.c_str());
    else if (name == "stCamera:ImageYCenter")
        currLensParam.vignParams.ImageYCenter = std::stof(_currText.c_str());
    else if (name == "stCamera:VignetteModelParam1")
        currLensParam.vignParams.VignetteModelParam1 = std::stof(_currText.c_str());
    else if (name == "stCamera:VignetteModelParam2")
        currLensParam.vignParams.VignetteModelParam2 = std::stof(_currText.c_str());
    else if (name == "stCamera:VignetteModelParam3")
        currLensParam.vignParams.VignetteModelParam3 = std::stof(_currText.c_str());
}

void LCPdatabase::loadDirectory(const boost::filesystem::path& p)
{
    if (boost::filesystem::is_directory(p))
    {
        // In some border cases, multiple LCP files could match with the image metadata
        // and we stop the search as soon as we have found a valid match.
        // So to ensure a repeatable behavior, we sort the files by name.
        std::vector<boost::filesystem::path> sortedPaths;
        for (auto&& x : boost::filesystem::directory_iterator(p))
            sortedPaths.push_back(x.path());
        std::sort(sortedPaths.begin(), sortedPaths.end());
        for(auto&& x : sortedPaths)
            loadDirectory(x);
    }
    else if (boost::filesystem::is_regular_file(p) && (boost::filesystem::extension(p) == ".lcp"))
    {
        _lcpFilepaths.push_back(p);
    }
}

std::string reduceString(const std::string& str)
{
    std::string s = str;

    // remove non-alphanumeric characters
    s.erase(std::remove_if(s.begin(), s.end(), [](char c) { return !std::isalnum(c); }), s.end());
    // to lowercase
    boost::algorithm::to_lower(s);

    return s;
}

std::vector<std::string> reduceStrings(const std::vector<std::string>& v_str)
{
    std::vector<std::string> v_localStr;
    for (auto& s : v_str)
    {
        v_localStr.push_back(reduceString(s));
    }
    return v_localStr;
}

LCPinfo* LCPdatabase::retrieveLCP(const std::string& lcpFilepath)
{
    if (lcpFilepath.empty())
    {
        return nullptr;
    }

    // Check if the LCP is already in the cache, or add it
    auto lcpCacheIt = _lcpCache.find(lcpFilepath);
    if(lcpCacheIt == _lcpCache.end())
    {
        // If not already in the cache, add it.
        #pragma omp critical
        _lcpCache[lcpFilepath] = LCPinfo(lcpFilepath, true);
        lcpCacheIt = _lcpCache.find(lcpFilepath);
    }

    // return the LCPinfo from the cache
    return &lcpCacheIt->second;
}

LCPinfo* LCPdatabase::findLCP(
                 const std::string& cameraMake,
                 const std::string& cameraModel,
                 const std::string& lensModel,
                 const int lensID,
                 int rawMode)
{
    const std::string reducedCameraMake = reduceString(cameraMake);
    const std::string reducedCameraModel = _omitCameraModel ? reducedCameraMake : reduceString(cameraModel);
    const std::string reducedLensModel = reduceString(lensModel);

    const std::string lensUidStr = reducedCameraMake + reducedCameraModel + reducedLensModel;
    const auto cachetoLcpPathIt = _lcpCameraMappingCache.find(lensUidStr);
    if(cachetoLcpPathIt != _lcpCameraMappingCache.end())
    {
        return retrieveLCP(cachetoLcpPathIt->second);
    }

    for(const LcpPath& lcpPath : _lcpFilepaths)
    {
        const bool filepathContainsMake = (lcpPath.reducedPath.find(reducedCameraMake) != std::string::npos);
        if(!filepathContainsMake)
            continue;

        auto headerIt = _lcpHeaderCache.find(lcpPath.reducedPath);
        if(headerIt == _lcpHeaderCache.end())
        {
            // If not already in the cache of LCP headers, add it.
            #pragma omp critical
            _lcpHeaderCache[lcpPath.path.string()] = LCPinfo(lcpPath.path.string(), false);
            headerIt = _lcpHeaderCache.find(lcpPath.path.string());
        }

        const LCPinfo& lcpHeader = headerIt->second;

        const std::string reducedCameraModelLCP =
            reduceString(_omitCameraModel ? lcpHeader.getCameraMaker() : lcpHeader.getCameraModel());
        const std::string reducedCameraPrettyNameLCP = reduceString(lcpHeader.getCameraPrettyName());
        const std::string reducedLensPrettyNameLCP = reduceString(lcpHeader.getLensPrettyName());

        std::vector<std::string> lensModelsLCP;
        lcpHeader.getLensModels(lensModelsLCP);
        const std::vector<std::string> reducedLensModelsLCP = reduceStrings(lensModelsLCP);

        std::vector<int> lensIDsLCP;
        lcpHeader.getLensIDs(lensIDsLCP);

        const bool cameraOK =
            ((reducedCameraModelLCP == reducedCameraModel) || (reducedCameraPrettyNameLCP == reducedCameraModel));
        const bool lensOK = ((reducedLensPrettyNameLCP == reducedLensModel) ||
                       (std::find(reducedLensModelsLCP.begin(), reducedLensModelsLCP.end(), reducedLensModel) != reducedLensModelsLCP.end()));
        const bool lensIDOK = (std::find(lensIDsLCP.begin(), lensIDsLCP.end(), lensID) != lensIDsLCP.end());
        const bool isRaw = lcpHeader.isRawProfile();

        const bool lcpFound =
            (cameraOK && lensOK && lensIDOK && ((isRaw && rawMode < 2) || (!isRaw && (rawMode % 2 == 0))));
        if(!lcpFound)
            // The LCP does not match our image metadata
            continue;

        // Add the mapping of the lens ID to the found LCP filepath
        #pragma omp critical
        _lcpCameraMappingCache[lensUidStr] = lcpPath.path.string();

        // Return the LCP from file or cache
        return retrieveLCP(lcpPath.path.string());
    }

    // No LCP has been found, add an empty path for that key in order to speed up next search of it
    if (_lcpCameraMappingCache.find(lensUidStr) == _lcpCameraMappingCache.end())
    {
        #pragma omp critical
        _lcpCameraMappingCache[lensUidStr] = "";
    }

    return nullptr;
}
