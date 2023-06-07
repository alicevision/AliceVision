#pragma once

#include <boost/filesystem/path.hpp>

#include <string>
#include <vector>
#include <sstream>
#include <map>

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

/**
 * @brief RectilinearModel contains parameters of a rectilinear model of distortion
 * Detailed information on this model can be found in the Adobe technical report
 * "Adobe Camera Model" part of the documentation of the Adobe free tool Lens Profile Creator.
 */
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

/**
 * @brief PerspectiveModel contains parameters of a perspective model of distortion
 * Detailed information on this model can be found in the Adobe technical report
 * "Adobe Camera Model" part of the documentation of the Adobe free tool Lens Profile Creator.
 */
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

/**
 * @brief VignetteModel contains parameters of a vignetting model of distortion
 * Detailed information on this model can be found in the Adobe technical report
 * "Adobe Camera Model" part of the documentation of the Adobe free tool Lens Profile Creator.
 */
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

/**
 * @brief FisheyeModel contains parameters of a fisheye model of distortion
 * Detailed information on this model can be found in the Adobe technical report
 * "Adobe Camera Model" part of the documentation of the Adobe free tool Lens Profile Creator.
 */
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
     * @brief Indicate that no geometric model is available
     * @return true if no geometric model is available
     */
    bool isEmpty() const { return perspParams.isEmpty && fisheyeParams.isEmpty; }

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
    ~LCPinfo() = default;

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
    * @param[out] lparam Lens parameters to be populated with the distortion model 
    */
    void getDistortionParams(const float& focalLength, const float& focusDistance, LensParam& lparam);

    /**
    * @brief Get vignetting parameters for a given couple focal length, aperture value. Aperture value can set to zero.
    * @param[in] focalLength Focal length in mm
    * @param[in] aperture Aperture value
    * @param[out] lparam Lens parameters to be populated with the vignetting model
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

    void setCommonSettings(const std::string& name);
    void setCameraSettings(const std::string& name);
    void setRectilinearModel(RectilinearModel& model, const std::string& name);
    void setFisheyeModel(const std::string& name);
    void setVignetteModel(const std::string& name);
};

std::string reduceString(const std::string& str);
std::vector<std::string> reduceStrings(const std::vector<std::string>& v_str);

/**
 * @brief LCPdatabase allows to access all the LCP files in the database.
 */
class LCPdatabase
{
public:
    LCPdatabase() = default;
    /**
     * @brief LCPdatabase constructor
     * @param[in] folder The folder containing all lcp files
     */
    LCPdatabase(const std::string& folder, bool omitCameraModel = false)
        : _omitCameraModel(omitCameraModel)
    {
        loadDirectory(folder);
    }
    ~LCPdatabase() = default;

    bool empty() const { return _lcpFilepaths.empty(); }

    size_t size() const { return _lcpFilepaths.size(); }

    void loadDirectory(const boost::filesystem::path& p);

    LCPinfo* retrieveLCP() { return retrieveLCP(_lcpFilepaths.begin()->path.string()); }

    /**
     * @brief Get the LCP from filepath. Retrieve it from cache or load it into the cache.
     */
    LCPinfo* retrieveLCP(const std::string& p);

    /**
     * @brief Try to find an appropriate LCP file for a set of camera and lens information amongst a set of files. If a
     * file is found, load its content.
     * 
     * @param[in] cameraMake Camera maker name
     * @param[in] cameraModel Camera model name
     * @param[in] lensModel Lens model name
     * @param[in] lensID Lens ID
     * @param[in] rawMode Set if raw status of a profile must be considered or not
     *            0 : no matter about raw status
     *            1 : only raw profile are considered in the search
     *            2 : only non raw profile are considered in the search
     * @param[in] omitCameraModel cameraModelOrMaker contains only the camera maker (default is false)
     * @return pointer to the found LCPinfo
     */
    LCPinfo* findLCP(const std::string& cameraMake, const std::string& cameraModel,
                     const std::string& lensModel, const int lensID, int rawMode);

private:
    struct LcpPath
    {
        LcpPath(const boost::filesystem::path& p)
            : path(p)
            , reducedPath(reduceString(p.string()))
        {}
        boost::filesystem::path path;
        std::string reducedPath;
    };

    /// List of all LCP files
    std::vector<LcpPath> _lcpFilepaths;
    /// Cache the header of LCP files
    std::map<std::string, LCPinfo> _lcpHeaderCache;
    /// Cache of fully loaded LCP files
    std::map<std::string, LCPinfo> _lcpCache;
    /// Map the label from the camera to the matching LCP filepath
    std::map<std::string, std::string> _lcpCameraMappingCache;
    /// The matching could be strict and fully match the camera Make, Model and Lens.
    /// As we are looking for lens information, we can omit the CameraModel to get generic values valid for more lenses.
    bool _omitCameraModel = false;
};

