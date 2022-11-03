#pragma once

#include <string>
#include <vector>
#include <sstream>

enum class LCPCorrectionMode
{
    VIGNETTE,
    DISTORTION,
    CA
};

enum class LCPReadingState
{
    WaitSequence,
    FillCommonAndCameraSettings,
    FillCameraSettings,
    FillGeometryModel,
    FillVignetteModel,
    FillChromaticGreenModel,
    FillChromaticBlueGreenModel,
    FillChromaticRedGreenModel,
    WaitNewModel
};

inline std::ostream& operator<<(std::ostream& os, const LCPReadingState& state)
{
    if (state == LCPReadingState::WaitSequence)
        os << "WaitSequence";
    else if (state == LCPReadingState::FillCommonAndCameraSettings)
        os << "FillCommonAndCameraSettings";
    else if (state == LCPReadingState::FillCameraSettings)
        os << "FillCameraSettings";
    else if (state == LCPReadingState::FillGeometryModel)
        os << "FillGeometryModel";
    else if (state == LCPReadingState::FillVignetteModel)
        os << "FillVignetteModel";
    else if (state == LCPReadingState::FillChromaticGreenModel)
        os << "FillChromaticGreenModel";
    else if (state == LCPReadingState::FillChromaticBlueGreenModel)
        os << "FillChromaticBlueGreenModel";
    else if (state == LCPReadingState::FillChromaticRedGreenModel)
        os << "FillChromaticRedGreenModel";
    else if (state == LCPReadingState::WaitNewModel)
        os << "WaitNewModel";

    return os;
}


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

struct RectilinearModel
{
    int Version = -1;
    float FocalLengthX = 0.f;
    float FocalLengthY = 0.f;
    float ImageXCenter = 0.5f;
    float ImageYCenter = 0.5f;
    float ResidualMeanError = 0.f;
    float ResidualStandardDeviation = 0.f;
    float RadialDistortParam1 = 0.f;
    float RadialDistortParam2 = 0.f;
    float RadialDistortParam3 = 0.f;
    float TangentialDistortParam1 = 0.f;
    float TangentialDistortParam2 = 0.f;
    float ScaleFactor = 1.f;
    bool isEmpty = true;

    void reset()
    {
        *this = RectilinearModel();
    }
};

struct PerspectiveModel
{
    int Version = -1;
    float FocalLengthX = 0.f;
    float FocalLengthY = 0.f;
    float ImageXCenter = 0.5f;
    float ImageYCenter = 0.5f;
    float ResidualMeanError = 0.f;
    float ResidualStandardDeviation = 0.f;
    float RadialDistortParam1 = 0.f;
    float RadialDistortParam2 = 0.f;
    float RadialDistortParam3 = 0.f;
    bool isEmpty = true;

    void reset()
    {
        *this = PerspectiveModel();
    }
};

struct VignetteModel
{
    float FocalLengthX = 0.f;
    float FocalLengthY = 0.f;
    float ImageXCenter = 0.5f;
    float ImageYCenter = 0.5f;
    float VignetteModelParam1 = 0.f;
    float VignetteModelParam2 = 0.f;
    float VignetteModelParam3 = 0.f;
    bool isEmpty = true;

    void reset()
    {
        *this = VignetteModel();
    }
};

struct FisheyeModel
{
    int Version = -1;
    float FocalLengthX = 0.f;
    float FocalLengthY = 0.f;
    float ImageXCenter = 0.5f;
    float ImageYCenter = 0.5f;
    float ResidualMeanError = 0.f;
    float ResidualStandardDeviation = 0.f;
    float RadialDistortParam1 = 0.f;
    float RadialDistortParam2 = 0.f;
    bool isEmpty = true;

    void reset()
    {
        *this = FisheyeModel();
    }
};

/**
 * @brief LensParam contains parameters of distortion, vignetting and chromatic aberation models
 * for a set of camera settings (focal length, focus distance, aperture value).
 * Detailed information on models can be found in the Adobe technical report
 * "Adobe Camera Model" part of the documentation of the Adobe free tool Lens Profile Creator.
 */
class LensParam
{
public:
    LensParam() = default;

    /**
    * @brief LensParam reset
    */
    void clear();

    /**
     * @brief Indicate that paramaters apply for a fisheye lens
     * @return true if the fisheye model is the valid one
     */
    bool isFisheye() const { return _isFisheye; }

    /**
     * @brief Indicate that a vignetting model is avaialble
     * @return true if a vignetting model is avaialble
     */
    bool hasVignetteParams() const { return _hasVignetteParams; }

    /**
     * @brief Indicate that chromatic models are avaialble
     * @return true if chromatic models are avaialble
     */
    bool hasChromaticParams() const { return _hasChromaticParams; }

    /**
    * @brief Set fisheye status
    * @param[in] fisheye status
    */
    void setFisheyeStatus(bool s) { _isFisheye = s; }

    /**
    * @brief Set vignetting availabilty status
    * @param[in] vignetting availabilty status
    */
    void setVignetteParamsStatus(bool s) { _hasVignetteParams = s; }

    /**
    * @brief Set chromatic models availabilty status
    * @param[in] chromatic models availabilty status
    */
    void setChromaticParamsStatus(bool s) { _hasChromaticParams = s; }

    /**
    * @brief Chromatic Green model parameters
    */
    RectilinearModel ChromaticGreenParams;
    /**
    * @brief Chromatic Red/Green model parameters
    */
    RectilinearModel ChromaticRedGreenParams;
    /**
    * @brief Chromatic Blue/Green model parameters
    */
    RectilinearModel ChromaticBlueGreenParams;

    /**
    * @brief Fisheye model parameters
    */
    FisheyeModel fisheyeParams;
    /**
    * @brief Pinhole model parameters
    */
    //PerspectiveModel perspParams;
    RectilinearModel perspParams;
    /**
    * @brief Vignetting model parameters
    */
    VignetteModel vignParams;

    /**
    * @brief Camera settings
    */
    settingsInfo camData;

private:

    bool _isFisheye = false;
    bool _hasVignetteParams = false;
    bool _hasChromaticParams = false;
};

/**
 * @brief LCPinfo loads and hosts the content of a Lens Correction Profile (LCP) file,
 * parameters of distortion, vignetting and chromatic aberation models for different
 * set of camera settings (focal length, focus distance, aperture value).
 * Detailed information on LCP file content can be found in the Adobe technical report
 * "Adobe Camera Model" part of the documentation of the Adobe free tool Lens Profile Creator.
 */
class LCPinfo
{
public:
    LCPinfo() = default;

    /**
    * @brief LCPinfo constructor
    * @param[in] filename The lcp path on disk
    * @param[in] fullParsing Load only common camera and lens info  and skip all models when set to false (default = true)
    */
    LCPinfo(const std::string& filename, bool fullParsing=true);
    ~LCPinfo() {};

    /**
    * @brief LCPinfo loader
    * @param[in] filename The lcp path on disk
    * @param[in] fullParsing Load only common camera and lens info  and skip all models when set to false (default = true)
    */
    void load(const std::string& filename, bool fullParsing = true);

    /**
    * @brief Get distortion parameters for a given couple focal length, focus distance. Focus distance can set to zero.
    * @param[in] focalLength Focal length in mm
    * @param[in] focusDistance Focus distance in meters
    * @param[in] lparam Lens parameters to be populated with the distortion model 
    */
    void getDistortionParams(const float& focalLength, const float& focusDistance, LensParam& lparam);

    /**
    * @brief Get vignetting parameters for a given couple focal length, aperture value. Aperture value can set to zero.
    * @param[in] focalLength Focal length in mm
    * @param[in] aperture Aperture value
    * @param[in] lparam Lens parameters to be populated with the vignetting model
    */
    void getVignettingParams(const float& focalLength, const float& aperture, LensParam& lparam);

    /**
     * @brief Indicate that no lens paramater set is available
     * @return true if no lens paramater set is available
     */
    inline bool isEmpty() const { return v_lensParams.empty(); }

    /**
    * @brief Get profile author
    * @return author
    */
    inline const std::string& getAuthor() const { return Author; }

    /**
    * @brief Get profile name
    * @return profile name
    */
    inline const std::string& getProfileName() const { return ProfileName; }

    /**
    * @brief Get camera maker
    * @return camera maker
    */
    inline const std::string& getCameraMaker() const { return Make; }

    /**
    * @brief Get camera model
    * @return camera model
    */
    inline const std::string& getCameraModel() const { return Model; }

    /**
    * @brief Get unique camera model
    * @return unique camera model
    */
    inline const std::string& getUniqueCameraModel() const { return UniqueCameraModel; }

    /**
    * @brief Get camera pretty name
    * @return camera pretty name
    */
    inline const std::string& getCameraPrettyName() const { return CameraPrettyName; }

    /**
    * @brief Get lens pretty name
    * @return lens pretty name
    */
    inline const std::string& getLensPrettyName() const { return LensPrettyName; }

    /**
    * @brief Get lens information
    * @return lens information
    */
    inline const std::string& getLensInfo() const { return LensInfo; }

    /**
    * @brief Get all known IDs for the lens
    * @return lens known IDs
    */
    inline void getLensIDs(std::vector<int>& lensIDs) const { lensIDs = LensID; }

    /**
    * @brief Get all known model names for the lens
    * @return lens model known names
    */
    inline void getLensModels(std::vector<std::string>& lensModels) const { lensModels = Lens; }

    /**
    * @brief Get image width
    * @return image width
    */
    inline int getImageWidth() const { return ImageWidth; }

    /**
    * @brief Get image length
    * @return image length
    */
    inline int getImageLength() const { return ImageLength; }

    /**
    * @brief Get sensor format factor
    * @return sensor format factor
    */
    inline float getSensorFormatFactor() const { return SensorFormatFactor; }

    /**
    * @brief Get raw profile status
    * @return true if profile is dedicated to a raw image
    */
    inline bool isRawProfile() const { return CameraRawProfile; }

    /**
    * @brief Get lens information
    * @return lens information
    */
    inline int getModelNumber() const { return v_lensParams.size(); }

    /**
    * @brief Set profile author
    * @param[in] profile author
    */
    inline void setAuthor(const std::string& str) { Author = str; }

    /**
    * @brief Set profile author with current text value
    */
    inline void setAuthor() { Author = _currText; }

    /**
    * @brief Set profile name
    * @param[in] profile name
    */
    inline void setProfileName(const std::string& str) { ProfileName = str; }

    /**
    * @brief Set camera maker
    * @param[in] camera maker
    */
    inline void setCameraMaker(const std::string& str) { Make = str; }

    /**
    * @brief Set camera model
    * @param[in] camera model
    */
    inline void setCameraModel(const std::string& str) { Model = str; }

    /**
    * @brief Set unique camera model
    * @param[in] unique camera model
    */
    inline void setUniqueCameraModel(const std::string& str) { UniqueCameraModel = str; }

    /**
    * @brief Set camera pretty name
    * @param[in] camera pretty name
    */
    inline void setCameraPrettyName(const std::string& str) { CameraPrettyName = str; }

    /**
    * @brief Set lens pretty name
    * @param[in] lens pretty name
    */
    inline void setLensPrettyName(const std::string& str) { LensPrettyName = str; }

    /**
    * @brief Set lens information
    * @param[in] lens information
    */
    inline void setLensInfo(const std::string& str) {  LensInfo = str; }

    /**
    * @brief Set an alernate lens ID for the lens
    * @param[in] alternate lens ID
    */
    inline void addLensID(int lensID) { LensID.push_back(lensID); }

    /**
    * @brief Set an alernate model name for the lens
    * @param[in] alternate model name
    */
    inline void addLensModel(std::string lensModel) { Lens.push_back(lensModel); }

    /**
    * @brief Set image width
    * @param[in] image width
    */
    inline void setImageWidth(int w) { ImageWidth = w; }

    /**
    * @brief Set image length
    * @param[in] image length
    */
    inline void setImageLength(int l) { ImageLength = l; }

    /**
    * @brief Set sensor format factor
    * @param[in] sensor format factor
    */
    inline void setSensorFormatFactor(float f) { SensorFormatFactor = f; }

    /**
    * @brief Set raw profile status
    * @param[in] raw profile status
    */
    inline void setAsRawProfile() { CameraRawProfile = true; }

private:
    // XML handlers
    static void XmlStartHandler(void* pLCPinfo, const char* el, const char** attr);
    static void XmlEndHandler(void* pLCPinfo, const char* el);
    static void XmlStartHandlerCommonOnly(void* pLCPinfo, const char* el, const char** attr);
    static void XmlEndHandlerCommonOnly(void* pLCPinfo, const char* el);
    static void XmlTextHandler(void* pLCPinfo, const char* s, int len);

    // Loading states control
    inline bool isSeqOpened() { return _isSeqOpened; }
    inline void openSequence() { _isSeqOpened = true; }
    inline void closeSequence() { _isSeqOpened = false; }

    inline bool isCommonOK() { return _isCommonOK; }
    inline void setCommonOK() { _isCommonOK = true; }
    inline void unsetCommonOK() { _isCommonOK = false; }

    inline bool isCamDataOK() { return _isCamDataOK; }
    inline void setCamDataOK() { _isCamDataOK = true; }
    inline void unsetCamDataOK() { _isCamDataOK = false; }

    inline bool isAlternateLensIDsOpened() { return _inAlternateLensIDs; }
    inline void openAlternateLensIDs() { _inAlternateLensIDs = true; }
    inline void closeAlternateLensIDs() { _inAlternateLensIDs = false; }

    inline bool isAlternateLensNamesOpened() { return _inAlternateLensNames; }
    inline void openAlternateLensNames() { _inAlternateLensNames = true; }
    inline void closeAlternateLensNames() { _inAlternateLensNames = false; }

    inline bool isWaitPerspModeldescription() { return _waitPerspModeldescription; }
    inline void setWaitPerspModeldescription() { _waitPerspModeldescription = true; }
    inline void unsetWaitPerspModeldescription() { _waitPerspModeldescription = false; }

    inline bool isGetText() { return _getText; }
    inline void setGetText() { _getText = true; }
    inline void unsetGetText() { _getText = false; }

    inline int getModelCount() { return _modelCount; }
    inline void increaseModelCount() { _modelCount++; }

    inline void storeCurrParams() { v_lensParams.push_back(currLensParam); }

    LensParam currLensParam;

    bool search(settingsInfo& settings, LCPCorrectionMode mode, int& iLow, int& iHigh, float& weightLow);
    void combine(size_t iLow, size_t iHigh, float weightLow, LCPCorrectionMode mode, LensParam& pOut);

    // Loading states
    bool _isSeqOpened = false;
    bool _isCommonOK = false;
    bool _isCamDataOK = false;
    bool _inAlternateLensIDs = false;
    bool _inAlternateLensNames = false;
    bool _waitPerspModeldescription = false;
    bool _getText = false;
    int _modelCount = 0;

    LCPReadingState _currReadingState = LCPReadingState::WaitSequence;
    std::string _currText = "";

    // Set of models contained in the LCP file
    std::vector<LensParam> v_lensParams;

    // Camera and Lens information, common for all models
    std::string Author = "";
    std::string Make = "";
    std::string Model = "";
    std::string UniqueCameraModel = "";
    bool CameraRawProfile;
    std::vector<int> LensID;
    std::vector<std::string> Lens;
    std::string LensInfo = "";
    std::string CameraPrettyName = "";
    std::string LensPrettyName = "";
    std::string ProfileName = "";
    float SensorFormatFactor = 1.f;
    int ImageWidth = 0;
    int ImageLength = 0;
    float XResolution = 0.f;
    float YResolution = 0.f;

    void setCommonSettings(const std::string& name)
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
            LensID.push_back(std::atoi(_currText.c_str()));
        else if (name == "stCamera:LensInfo")
            LensInfo = _currText;
        else if (name == "stCamera:ImageWidth")
            ImageWidth = atoi(_currText.c_str());
        else if (name == "stCamera:ImageLength")
            ImageLength = atoi(_currText.c_str());
        else if (name == "stCamera:XResolution")
            XResolution = atof(_currText.c_str());
        else if (name == "stCamera:YResolution")
            YResolution = atof(_currText.c_str());
        else if (name == "stCamera:LensPrettyName")
            LensPrettyName = _currText;
        else if (name == "stCamera:CameraPrettyName")
            CameraPrettyName = _currText;
        else if (name == "stCamera:CameraRawProfile")
            CameraRawProfile = ((_currText == "true") || (_currText == "True"));
        else if (name == "stCamera:SensorFormatFactor")
            SensorFormatFactor = atof(_currText.c_str());
    }

    void setCameraSettings(const std::string& name)
    {
        if (name == "stCamera:FocalLength")
            currLensParam.camData.FocalLength = atof(_currText.c_str());
        else if (name == "stCamera:ApertureValue")
            currLensParam.camData.ApertureValue = atof(_currText.c_str());
        else if (name == "stCamera:FocusDistance")
            currLensParam.camData.FocusDistance = atof(_currText.c_str());
    }

    void setRectilinearModel(RectilinearModel& model, const std::string& name)
    {
        model.isEmpty = false;
        if (name == "stCamera:Version")
            model.Version = atoi(_currText.c_str());
        else if (name == "stCamera:FocalLengthX")
            model.FocalLengthX = atof(_currText.c_str());
        else if (name == "stCamera:FocalLengthY")
            model.FocalLengthY = atof(_currText.c_str());
        else if (name == "stCamera:ImageXCenter")
            model.ImageXCenter = atof(_currText.c_str());
        else if (name == "stCamera:ImageYCenter")
            model.ImageYCenter = atof(_currText.c_str());
        else if (name == "stCamera:RadialDistortParam1")
            model.RadialDistortParam1 = atof(_currText.c_str());
        else if (name == "stCamera:RadialDistortParam2")
            model.RadialDistortParam2 = atof(_currText.c_str());
        else if (name == "stCamera:RadialDistortParam3")
            model.RadialDistortParam3 = atof(_currText.c_str());
        else if (name == "stCamera:TangentiallDistortParam1")
            model.RadialDistortParam1 = atof(_currText.c_str());
        else if (name == "stCamera:TangentiallDistortParam2")
            model.RadialDistortParam2 = atof(_currText.c_str());
        else if (name == "stCamera:ScaleFactor")
            model.ScaleFactor = atof(_currText.c_str());
    }

    void setFisheyeModel(const std::string& name)
    {
        currLensParam.fisheyeParams.isEmpty = false;
        if (name == "stCamera:Version")
            currLensParam.fisheyeParams.Version = atoi(_currText.c_str());
        else if (name == "stCamera:FocalLengthX")
            currLensParam.fisheyeParams.FocalLengthX = atof(_currText.c_str());
        else if (name == "stCamera:FocalLengthY")
            currLensParam.fisheyeParams.FocalLengthY = atof(_currText.c_str());
        else if (name == "stCamera:ImageXCenter")
            currLensParam.fisheyeParams.ImageXCenter = atof(_currText.c_str());
        else if (name == "stCamera:ImageYCenter")
            currLensParam.fisheyeParams.ImageYCenter = atof(_currText.c_str());
        else if (name == "stCamera:RadialDistortParam1")
            currLensParam.fisheyeParams.RadialDistortParam1 = atof(_currText.c_str());
        else if (name == "stCamera:RadialDistortParam2")
            currLensParam.fisheyeParams.RadialDistortParam2 = atof(_currText.c_str());
    }

    void setVignetteModel(const std::string& name)
    {
        currLensParam.vignParams.isEmpty = false;
        if (name == "stCamera:FocalLengthX")
            currLensParam.vignParams.FocalLengthX = atof(_currText.c_str());
        else if (name == "stCamera:FocalLengthY")
            currLensParam.vignParams.FocalLengthY = atof(_currText.c_str());
        else if (name == "stCamera:ImageXCenter")
            currLensParam.vignParams.ImageXCenter = atof(_currText.c_str());
        else if (name == "stCamera:ImageYCenter")
            currLensParam.vignParams.ImageYCenter = atof(_currText.c_str());
        else if (name == "stCamera:VignetteModelParam1")
            currLensParam.vignParams.VignetteModelParam1 = atof(_currText.c_str());
        else if (name == "stCamera:VignetteModelParam2")
            currLensParam.vignParams.VignetteModelParam2 = atof(_currText.c_str());
        else if (name == "stCamera:VignetteModelParam3")
            currLensParam.vignParams.VignetteModelParam3 = atof(_currText.c_str());
    }
};

/**
* @brief Parse a given directory and all subdirectories and try to find an appropriate LCP file for a set of camera and lens information
*        amongst the discovered LCP files. If a file is found, load its content.
* @param[in] dbDirectoryname Directory to explore
* @param[in] cameraModelOrMaker Camera model name or maker
* @param[in] lensModel Lens model name
* @param[in] lensID Lens ID
* @param[in] rawMode Set if raw status of a profile must be considered or not
*            0 : no matter about raw status
*            1 : only raw profile are considered in the search
*            2 : only non raw profile are considered in the search
* @param[in] lcpData LCP info to be populated with the file content
* @param[in] omitCameraModel cameraModelOrMaker contains only the camera maker (default is false)
* @return True if a file is found
*/
bool findLCPInfo(const std::string& dbDirectoryname, const std::string& cameraModelOrMaker, const std::string& lensModel, const int& lensID, int rawMode, LCPinfo& lcpData, bool omitCameraModel = false);

/**
* @brief Try to find an appropriate LCP file for a set of camera and lens information amongst a set of files. If a file is found, load its content.
* @param[in] lcpFilenames Set of all filepathes to explore
* @param[in] cameraModelOrMaker Camera model name or maker
* @param[in] lensModel Lens model name
* @param[in] lensID Lens ID
* @param[in] rawMode Set if raw status of a profile must be considered or not
*            0 : no matter about raw status
*            1 : only raw profile are considered in the search
*            2 : only non raw profile are considered in the search
* @param[in] lcpData LCP info to be populated with the file content
* @param[in] omitCameraModel cameraModelOrMaker contains only the camera maker (default is false)
* @return True if a file is found
*/
bool findLCPInfo(const std::vector<boost::filesystem::path>& lcpFilenames, const std::string& cameraModelOrMaker, const std::string& lensModel, const int& lensID, int rawMode, LCPinfo& lcpData, bool omitCameraModel = false);
