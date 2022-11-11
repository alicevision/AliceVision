#pragma once

#include <map>
#include <vector>
#include <array>
#include <memory>
#include <string>

#include <OpenImageIO/imagebuf.h>

namespace alicevision {
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
 * @brief A class representing a tone curve that can be embedded within a DCP color profile
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
 * @brief Aggregate all color profile application options
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
 * @brief Aggregate all color profile information
 */
struct DCPProfileInfo
{
    bool has_color_matrix_1 = false;
    bool has_color_matrix_2 = false;
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
    * @brief A class representing a DCP color profile
    */
class DCPProfile final
{
public:
    DCPProfileInfo info;

    using Triple = std::array<double, 3>;
    using Matrix = std::array<Triple, 3>;

    explicit DCPProfile(const std::string& filename);
    ~DCPProfile();

    void apply(OIIO::ImageBuf& image, const DCPProfileApplyParams& params);
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

    Matrix matMult(const Matrix& A, const Matrix& B);
    Triple matMult(const Matrix& M, const Triple& V);
    Matrix matInv(const Matrix& M);

    Matrix getInterpolatedMatrix(const float cct, const std::string& type);
    void getChromaticityCoordinatesFromXyz(const Triple& xyz, float& x, float& y);
    Triple getXyzFromChromaticityCoordinates(const float x, const float y);
    Triple getXyzFromTemperature(const float cct, const float tint = 0.f);
    void setChromaticityCoordinates(const float x, const float y, float& cct, float& tint);
    void getChromaticityCoordinates(const float cct, const float tint, float& x, float& y);
    void getChromaticityCoordinatesFromCameraNeutral(const Matrix& analogBalance, const Triple& asShotNeutral, float& x, float& y);
    Matrix getChromaticAdaptationMatrix(const Triple& xyzSource, const Triple& xyzTarget);
    Matrix getCameraToXyzD50Matrix(const float x, const float y);

    Matrix ws_sRGB;
    Matrix sRGB_ws;
    Matrix color_matrix_1;
    Matrix color_matrix_2;
    Matrix calib_matrix_1;
    Matrix calib_matrix_2;
    Matrix analogBalance;
    bool will_interpolate;
    bool valid;
    Matrix forward_matrix_1;
    Matrix forward_matrix_2;
    double baseline_exposure_offset;
    std::vector<HsbModify> deltas_1;
    std::vector<HsbModify> deltas_2;
    std::vector<HsbModify> look_table;
    HsdTableInfo delta_info;
    HsdTableInfo look_info;

    SplineToneCurve AS_tone_curve;

    SplineToneCurve gammatab_srgb;
    SplineToneCurve igammatab_srgb;
};

}
}
