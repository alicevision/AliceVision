#pragma once

#include <map>
#include <vector>
#include <array>
#include <memory>
#include <string>

#include <OpenImageIO/imagebuf.h>

#include <aliceVision/image/all.hpp>

namespace aliceVision {
namespace image {

/**
 * @defgroup colorConvertMatrices Color space conversion to/from XYZ
 * color spaces adapted to D50 using Bradford transform
 * Source : Bruce Lindbloom's website : http://www.brucelindbloom.com/
 * @{
 */
constexpr double xyz_sRGB[3][3] = {
    {0.4360747,  0.3850649, 0.1430804},
    {0.2225045,  0.7168786,  0.0606169},
    {0.0139322,  0.0971045,  0.7141733}
};      
constexpr double sRGB_xyz[3][3] = {
    {3.1338561, -1.6168667, -0.4906146},
    { -0.9787684,  1.9161415,  0.0334540},
    {0.0719453, -0.2289914,  1.4052427}
};

constexpr double xyz_prophoto[3][3] = {
    {0.7976749,  0.1351917,  0.0313534},
    {0.2880402,  0.7118741,  0.0000857},
    {0.0000000,  0.0000000,  0.8252100}
};
constexpr double prophoto_xyz[3][3] = {
    {1.3459433, -0.2556075, -0.0511118},
    { -0.5445989,  1.5081673,  0.0205351},
    {0.0000000,  0.0000000,  1.2118128}
};

constexpr double xyz_AdobeRGB[3][3] = {
    {0.6097559,  0.2052401,  0.1492240},
    {0.3111242,  0.6256560,  0.0632197},
    {0.0194811,  0.0608902,  0.7448387}
};
constexpr double AdobeRGB_xyz[3][3] = {
    { 1.9624274, -0.6105343, -0.3413404},
    {-0.9787684,  1.9161415,  0.0334540},
    { 0.0286869, -0.1406752,  1.3487655}
};
/// @}

enum class LightSource
{
    UNKNOWN = 0,
    DAYLIGHT = 1,
    FLUORESCENT = 2,
    TUNGSTEN = 3,
    FLASH = 4,
    FINE_WEATHER = 9,
    CLOUDY_WEATHER = 10,
    SHADE = 11,
    DAYLIGHT_FLUORESCENT = 12,   // D  5700 - 7100K
    DAYWHITE_FLUORESCENT = 13,   // N  4600 - 5500K
    COOL_WHITE_FLUORESCENT = 14, // W  3800 - 4500K
    WHITE_FLUORESCENT = 15,      // WW 3250 - 3800K
    WARM_WHITE_FLUORESCENT = 16, // L  2600 - 3250K
    STANDARD_LIGHT_A = 17,
    STANDARD_LIGHT_B = 18,
    STANDARD_LIGHT_C = 19,
    D55 = 20,
    D65 = 21,
    D75 = 22,
    D50 = 23,
    ISO_STUDIO_TUNGSTEN = 24,
    OTHER = 255
};

double calibrationIlluminantToTemperature(LightSource light);


/**
 * @brief SplineToneCurve represents a tone curve that can be embedded within a DCP color profile
 */
class SplineToneCurve final
{
public:
    SplineToneCurve() = default;
    ~SplineToneCurve() = default;

    void Set(const std::vector<double>& v_xy);
    void Apply(float& ir, float& ig, float& ib) const;
    float operator[](const float idx) const { return getval(idx); }

private:

    void RGBTone(float& maxval, float& medval, float& minval) const;
    float getval(const float idx) const;

    std::vector<float> ffffToneCurve = std::vector<float>(65536, 0.f);
};

/**
 * @brief Aggregate all DCP color profile application options (non linear part)
 */
struct DCPProfileApplyParams
{
    bool apply_hue_shift = true;
    bool apply_baseline_exposure_offset = true;
    bool use_tone_curve = true;
    bool apply_look_table = true;
    std::string working_space = "sRGB";
};

 /**
  * @brief DCPProfileInfo contains information about matrices, table and curves contained in the profile
  */
struct DCPProfileInfo
{
    std::string filename = "";
    std::string profileCalibrationSignature = "";

    bool has_color_matrix_1 = false;
    bool has_color_matrix_2 = false;
    bool has_camera_calibration_1 = false;
    bool has_camera_calibration_2 = false;
    bool has_forward_matrix_1 = false;
    bool has_forward_matrix_2 = false;
    bool has_look_table = false;
    bool has_hue_sat_map = false;
    bool has_tone_curve = false;
    bool has_baseline_exposure_offset = false;

    short light_source_1 = 0;
    short light_source_2 = 0;
    double temperature_1 = 0.0;
    double temperature_2 = 0.0;
};

/**
* @brief DCPProfile contains a Dng Color Profile as specified by Adobe
* DNG specification can be found here: https://helpx.adobe.com/content/dam/help/en/photoshop/pdf/dng_spec_1_6_0_0.pdf
* Profiles with more than 2 illuminants are not supported
*/
class DCPProfile final
{
public:
    DCPProfileInfo info;

    using Triple = std::array<double, 3>;
    using Matrix = std::array<Triple, 3>;

    DCPProfile();

    /**
    * @brief DCPProfile constructor
    * @param[in] filename The dcp path on disk
    */
    explicit DCPProfile(const std::string& filename);
    ~DCPProfile();

    /**
    * @brief DCPProfile loader
    * @param[in] filename The dcp path on disk
    */
    void Load(const std::string& filename);

    /**
     * @brief getMatrices gets some matrices contained in the profile
     * param[in] type The matrices to get, "color" or "forward"
     * param[in] v_Mat A vector of matrices to be populated
     */
    void getMatrices(const std::string& type, std::vector<Matrix>& v_Mat) const;

    /**
     * @brief getMatricesAsStrings gets some matrices contained in the profile in a string format (one string per matrix)
     * param[in] type The matrices to get, "color" or "forward"
     * param[in] v_strMat A vector of std::string to be populated
     */
    void getMatricesAsStrings(const std::string& type, std::vector<std::string>& v_strMat) const;

    /**
     * @brief setMatrices sets some matrices contained in the profile
     * param[in] type The matrices to set, "color" or "forward"
     * param[in] v_Mat A vector of matrices
     */
    void setMatrices(const std::string& type, std::vector<Matrix>& v_Mat);

    /**
     * @brief setMatricesFromStrings sets some matrices contained in the profile from strings (one string per matrix)
     * param[in] type The matrices to set, "color" or "forward"
     * param[in] v_strMat A vector of std::string
     */
    void setMatricesFromStrings(const std::string& type, std::vector<std::string>& v_strMat);

    /**
     * @brief applyLinear applies the linear part of a DCP profile on an OIIO image buffer
     * param[in] image The OIIO image on which the profile must be applied
     * param[in] neutral The neutral value calculated from the camera multiplicators contained in the cam_mul OIIO metadata
     * param[in] sourceIsRaw indicates that the image buffer contains data in raw space (no neutralization <=> cam_mul not applied)
     */
    void applyLinear(OIIO::ImageBuf& image, const Triple& neutral, const bool sourceIsRaw = false, const bool useColorMatrixOnly = false) const;

    /**
     * @brief applyLinear applies the linear part of a DCP profile on an aliceVision image
     * param[in] image The aliceVision image on which the profile must be applied
     * param[in] neutral The neutral value calculated from the camera multiplicators contained in the cam_mul OIIO metadata
     * param[in] sourceIsRaw indicates that the image buffer contains data in raw space (no neutralization <=> cam_mul not applied)
     */
    void applyLinear(Image<image::RGBAfColor>& image, const Triple& neutral, const bool sourceIsRaw = false) const;

    /**
     * @brief apply applies the non linear part of a DCP profile on an OIIO image buffer
     * param[in] image The OIIO image on which the profile must be applied
     * param[in] params contains the application parameters indicating which parts of the profile must be applied
     */
    void apply(OIIO::ImageBuf& image, const DCPProfileApplyParams& params);
    /**
     * @brief apply applies the non linear part of a DCP profile on a rgb pixel
     * param[in] rgb The pixel values on which the profile must be applied
     * param[in] params contains the application parameters indicating which parts of the profile must be applied
     */
    void apply(float* rgb, const DCPProfileApplyParams& params) const;

private:
    struct HsbModify
    {
        float hue_shift;
        float sat_scale;
        float val_scale;
    };

    struct HsdTableInfo
    {
        int hue_divisions;
        int sat_divisions;
        int val_divisions;
        int hue_step;
        int val_step;
        unsigned int array_count;
        bool srgb_gamma;
        struct {
            float h_scale;
            float s_scale;
            float v_scale;
            int max_hue_index0;
            int max_sat_index0;
            int max_val_index0;
            int hue_step;
            int val_step;
        } pc;
    };

    void hsdApply(const HsdTableInfo& table_info, const std::vector<HsbModify>& table_base, float& h, float& s, float& v) const;

    Matrix matMult(const Matrix& A, const Matrix& B) const;
    Triple matMult(const Matrix& M, const Triple& V) const;
    Matrix matInv(const Matrix& M) const;

    Matrix getInterpolatedMatrix(const double cct, const std::string& type) const;
    void getChromaticityCoordinatesFromXyz(const Triple& xyz, double& x, double& y) const;
    Triple getXyzFromChromaticityCoordinates(const double x, const double y) const;
    Triple getXyzFromTemperature(const double cct, const double tint = 0.f) const;
    void setChromaticityCoordinates(const double x, const double y, double& cct, double& tint) const;
    void getChromaticityCoordinates(const double cct, const double tint, double& x, double& y) const;
    void getChromaticityCoordinatesFromCameraNeutral(const Matrix& analogBalance, const Triple& asShotNeutral, double& x, double& y) const;
    Matrix getChromaticAdaptationMatrix(const Triple& xyzSource, const Triple& xyzTarget) const;
    Matrix getCameraToXyzD50Matrix(const double x, const double y) const;
    Matrix getCameraToSrgbLinearMatrix(const double x, const double y) const;
    Matrix getCameraToACES2065Matrix(const Triple& asShotNeutral, const bool sourceIsRaw = false, const bool useColorMatrixOnly = false) const;

    Matrix ws_sRGB; // working color space to sRGB
    Matrix sRGB_ws; // sRGB to working color space
    Matrix color_matrix_1; // Color matrix for illuminant 1
    Matrix color_matrix_2; // Color matrix for illuminant 2
    Matrix camera_calibration_1; // Calibration matrix for illuminant 1
    Matrix camera_calibration_2; // Calibration matrix for illuminant 2
    Matrix analogBalance;
    Matrix forward_matrix_1; // white balanced raw to xyzD50 for illumimant 1
    Matrix forward_matrix_2; // white balanced raw to xyzD50 for illumimant 2
    double baseline_exposure_offset;
    std::vector<HsbModify> deltas_1; // Basic Hue Sat update table for illumimant 1
    std::vector<HsbModify> deltas_2; // Basic Hue Sat update table for illumimant 2
    std::vector<HsbModify> look_table; // Hue Sat update table for look modification
    HsdTableInfo delta_info; // Information for basic Hue Sat updates
    HsdTableInfo look_info;  // Information for look modification 

    SplineToneCurve AS_tone_curve; // Adobe standard tone curve

    SplineToneCurve gammatab_srgb;
    SplineToneCurve igammatab_srgb;
};

/**
* @brief DCPDatabase manages DCP profiles loading and caching
*/
class DCPDatabase final
{
public:
    DCPDatabase() = default;
    DCPDatabase(const std::string& databaseDirPath);

    ~DCPDatabase() = default;

    /**
     * @brief load stores in a file list all the valid dcp filenames found in a folder (including subfolders). 
     * None of them are loaded in cache.
     * param[in] database folder name.
     * param[in] if true loading is forced even if a database with the same folder name is already loaded.
     * return Number of DCP files found.
     */
    int load(const std::string& databaseDirPath, bool force = false);

    /**
     * @brief clear clears the cache and the DCP file list.
     */
    void clear();

    /**
     * @brief Check if the databse is empty.
     * return True if empty.
     */
    inline bool empty() { return dcpFilenamesList.empty(); }

    /**
     * @brief add_or_replace adds or replaces an existing DCP profile in the cache.
     * Update the DCP file list with dcpProf.info.filename. 
     * param[in] DCP profile to be stored.
     * param[in] DSLR Maker.
     * param[in] DSLR Model.
     */
    void add_or_replace(DCPProfile& dcpProf, const std::string& make, const std::string& model);

    /**
     * @brief retrieveDcpForCamera searches for a DCP profile in the database for a given camera.
     * Search first in the cache. If no DCP profile is found search if an appropriate DCP file exists in the file list.
     * If a dcp file is found then load it and store it in the cache.
     * param[in] make The camera maker
     * param[in] model The camera model
     * param[in] dcpProf the DCP profile to be filled in
     * return True if a corresponding profile has been found
     */
    bool retrieveDcpForCamera(const std::string& make, const std::string& model, DCPProfile& dcpProf);

private:

    std::string folderName;

    std::vector<std::string> dcpFilenamesList;

    std::map<std::string, DCPProfile> dcpStore;

};



}
}
