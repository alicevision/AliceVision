#include "dcp.hpp"

#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/system/Logger.hpp>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/foreach.hpp>

#include <iostream>
#include <cstdio>
#include <cstring>
#include <functional>

namespace aliceVision {
namespace image {

using aliceVision::clamp;
namespace bfs = boost::filesystem;

double calibrationIlluminantToTemperature(const LightSource light)
{
    // These temperatures are those found in DNG SDK reference code
    switch(light)
    {
        case LightSource::STANDARD_LIGHT_A:
        case LightSource::TUNGSTEN:
        {
            return 2850.0;
        }

        case LightSource::ISO_STUDIO_TUNGSTEN:
        {
            return 3200.0;
        }

        case LightSource::D50:
        {
            return 5000.0;
        }

        case LightSource::D55:
        case LightSource::DAYLIGHT:
        case LightSource::FINE_WEATHER:
        case LightSource::FLASH:
        case LightSource::STANDARD_LIGHT_B:
        {
            return 5500.0;
        }

        case LightSource::D65:
        case LightSource::STANDARD_LIGHT_C:
        case LightSource::CLOUDY_WEATHER:
        {
            return 6500.0;
        }

        case LightSource::D75:
        case LightSource::SHADE:
        {
            return 7500.0;
        }

        case LightSource::DAYLIGHT_FLUORESCENT:
        {
            return (5700.0 + 7100.0) * 0.5;
        }

        case LightSource::DAYWHITE_FLUORESCENT:
        {
            return (4600.0 + 5500.0) * 0.5;
        }

        case LightSource::COOL_WHITE_FLUORESCENT:
        case LightSource::FLUORESCENT:
        {
            return (3800.0 + 4500.0) * 0.5;
        }

        case LightSource::WHITE_FLUORESCENT:
        {
            return (3250.0 + 3800.0) * 0.5;
        }

        case LightSource::WARM_WHITE_FLUORESCENT:
        {
            return (2600.0 + 3250.0) * 0.5;
        }

        default:
        {
            return 0.0;
        }
    }
}

inline bool rgb2hsvdcp(float r, float g, float b, float& h, float& s, float& v)
{
    const float var_Min = std::min<float>(r, std::min<float>(g, b));

    if (var_Min < 0.f)
    {
        return false;
    }
    else
    {
        const float var_Max = std::max<float>(r, std::max<float>(g, b));
        const float del_Max = var_Max - var_Min;
        v = var_Max / 65535.f;

        if (std::abs(del_Max) < 0.00001f)
        {
            h = 0.f;
            s = 0.f;
        }
        else
        {
            s = del_Max / var_Max;

            if (r == var_Max)
            {
                h = (g - b) / del_Max;
            }
            else if (g == var_Max)
            {
                h = 2.f + (b - r) / del_Max;
            }
            else
            {
                h = 4.f + (r - g) / del_Max;
            }

            if (h < 0.f)
            {
                h += 6.f;
            }
            else if (h > 6.f)
            {
                h -= 6.f;
            }
        }

        return true;
    }
}

inline void rgb2hsvtc(float r, float g, float b, float& h, float& s, float& v)
{
    const float var_Min = std::min<float>(r, std::min<float>(g, b));
    const float var_Max = std::max<float>(r, std::max<float>(g, b));
    const float del_Max = var_Max - var_Min;

    v = var_Max / 65535.f;

    if (del_Max < 0.00001f)
    {
        h = 0.f;
        s = 0.f;
    }
    else
    {
        s = del_Max / var_Max;

        if (r == var_Max)
        {
            h = (g < b ? 6.f : 0.f) + (g - b) / del_Max;
        }
        else if (g == var_Max)
        {
            h = 2.f + (b - r) / del_Max;
        }
        else
        {
            h = 4.f + (r - g) / del_Max;
        }
    }
}

inline void hsv2rgbdcp(float h, float s, float v, float& r, float& g, float& b)
{
    // special version for dcp which saves 1 division (in caller) and six multiplications (inside this function)
    const int sector = h;       // sector 0 to 5, floor() is very slow, and h is always > 0
    const float f = h - sector; // fractional part of h

    v *= 65535.f;
    const float vs = v * s;
    const float p = v - vs;
    const float q = v - f * vs;
    const float t = p + v - q;

    switch (sector)
    {
    case 1:
        r = q;
        g = v;
        b = p;
        break;

    case 2:
        r = p;
        g = v;
        b = t;
        break;

    case 3:
        r = p;
        g = q;
        b = v;
        break;

    case 4:
        r = t;
        g = p;
        b = v;
        break;

    case 5:
        r = v;
        g = p;
        b = q;
        break;

    default:
        r = v;
        g = t;
        b = p;
    }
}

namespace {
/// Wyszecki & Stiles', 'Color Science - Concepts and Methods Data and Formulae - Second Edition', Page 228.
/// (Reciprocal Megakelvin, CIE 1960 Chromaticity Coordinates 'u', CIE 1960 Chromaticity Coordinates 'v', Slope)
const std::vector<std::vector<float>> WYSZECKI_ROBERSTON_TABLE =
    { {0, 0.18006, 0.26352, -0.24341},
      {10, 0.18066, 0.26589, -0.25479},
      {20, 0.18133, 0.26846, -0.26876},
      {30, 0.18208, 0.27119, -0.28539},
      {40, 0.18293, 0.27407, -0.30470},
      {50, 0.18388, 0.27709, -0.32675},
      {60, 0.18494, 0.28021, -0.35156},
      {70, 0.18611, 0.28342, -0.37915},
      {80, 0.18740, 0.28668, -0.40955},
      {90, 0.18880, 0.28997, -0.44278},
      {100, 0.19032, 0.29326, -0.47888},
      {125, 0.19462, 0.30141, -0.58204},
      {150, 0.19962, 0.30921, -0.70471},
      {175, 0.20525, 0.31647, -0.84901},
      {200, 0.21142, 0.32312, -1.0182},
      {225, 0.21807, 0.32909, -1.2168},
      {250, 0.22511, 0.33439, -1.4512},
      {275, 0.23247, 0.33904, -1.7298},
      {300, 0.24010, 0.34308, -2.0637},
      {325, 0.24792, 0.34655, -2.4681}, // 0.24702 ---> 0.24792 Bruce Lindbloom
      {350, 0.25591, 0.34951, -2.9641},
      {375, 0.26400, 0.35200, -3.5814},
      {400, 0.27218, 0.35407, -4.3633},
      {425, 0.28039, 0.35577, -5.3762},
      {450, 0.28863, 0.35714, -6.7262},
      {475, 0.29685, 0.35823, -8.5955},
      {500, 0.30505, 0.35907, -11.324},
      {525, 0.31320, 0.35968, -15.628},
      {550, 0.32129, 0.36011, -23.325},
      {575, 0.32931, 0.36038, -40.770},
      {600, 0.33724, 0.36051, -116.45} };

// Useful matrices
const DCPProfile::Matrix IdentityMatrix = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
const DCPProfile::Matrix CAT02_MATRIX = {0.7328, 0.4296, -0.1624, -0.7036, 1.6975, 0.0061, 0.0030, 0.0136, 0.9834};
const DCPProfile::Matrix xyzD50ToSrgbD65LinearMatrix = { 3.2404542, -1.5371385, -0.4985314, -0.9692660, 1.8760108, 0.0415560, 0.0556434, -0.2040259, 1.0572252 };
const DCPProfile::Matrix xyzD50ToSrgbD50LinearMatrix = { 3.1338561, -1.6168667, -0.4906146, -0.9787684, 1.9161415, 0.0334540, 0.0719453, -0.2289914, 1.4052427 };

// xyzD50ToACES2065Matrix = xyzD60ToACES2065 * xyzD50ToXyzD60
const DCPProfile::Matrix xyzD50ToACES2065Matrix = { 1.019573375, -0.022815668, 0.048147546, -0.503070253, 1.384421764, 0.121965628, 0.000961591, 0.003054793, 1.207019111 };

const double TINT_SCALE = -3000.0;
} // namespace

enum class TagType : int
{
    T_INVALID = 0,
    T_BYTE = 1,
    T_ASCII = 2,
    T_SHORT = 3,
    T_LONG = 4,
    T_RATIONAL = 5,
    T_SBYTE = 6,
    T_UNDEFINED = 7,
    T_SSHORT = 8,
    T_SLONG = 9,
    T_SRATIONAL = 10,
    T_FLOAT = 11,
    T_DOUBLE = 12,
    T_OLYUNDEF = 13,
    T_AUTO = 98,
    T_SUBDIR = 99
};
enum class Endianness : int
{
    UNKNOWN = 0,
    LITTLE = 0x4949,
    BIG = 0x4D4D
};

inline static int getTypeSize(TagType type)
{
    return ((type == TagType::T_INVALID || type == TagType::T_BYTE || type == TagType::T_ASCII ||
             type == TagType::T_SBYTE || type == TagType::T_UNDEFINED)
                ? 1
            : (type == TagType::T_SHORT || type == TagType::T_SSHORT) ? 2
            : (type == TagType::T_LONG || type == TagType::T_SLONG || type == TagType::T_FLOAT ||
               type == TagType::T_OLYUNDEF)
                ? 4
            : (type == TagType::T_RATIONAL || type == TagType::T_SRATIONAL || type == TagType::T_DOUBLE) ? 8
                                                                                                         : 0);
}

enum class TagKey : int
{
    PROFILE_CALIBRATION_SIGNATURE = 50932,
    COLOR_MATRIX_1 = 50721,
    COLOR_MATRIX_2 = 50722,
    CAMERA_CALIBRATION_1 = 50723,
    CAMERA_CALIBRATION_2 = 50724,
    PROFILE_HUE_SAT_MAP_DIMS = 50937,
    PROFILE_HUE_SAT_MAP_DATA_1 = 50938,
    PROFILE_HUE_SAT_MAP_DATA_2 = 50939,
    PROFILE_TONE_CURVE = 50940,
    PROFILE_TONE_COPYRIGHT = 50942,
    CALIBRATION_ILLUMINANT_1 = 50778,
    CALIBRATION_ILLUMINANT_2 = 50779,
    FORWARD_MATRIX_1 = 50964,
    FORWARD_MATRIX_2 = 50965,
    PROFILE_LOOK_TABLE_DIMS = 50981,
    PROFILE_LOOK_TABLE_DATA = 50982,
    PROFILE_HUE_SAT_MAP_ENCODING = 51107,
    PROFILE_LOOK_TABLE_ENCODING = 51108,
    BASELINE_EXPOSURE_OFFSET = 51109
};

//-----------------------------------------------------------------------------
// Reading Endianness dependent data
//-----------------------------------------------------------------------------
unsigned short sget2(const unsigned char* s, Endianness order)
{
    if(order == Endianness::LITTLE)
    {
        return s[0] | s[1] << 8;
    }
    else
    {
        return s[0] << 8 | s[1];
    }
}
int sget4(const unsigned char* s, Endianness order)
{
    if(order == Endianness::LITTLE)
    {
        return s[0] | s[1] << 8 | s[2] << 16 | s[3] << 24;
    }
    else
    {
        return s[0] << 24 | s[1] << 16 | s[2] << 8 | s[3];
    }
}
inline unsigned short get2(FILE* f, Endianness order)
{
    unsigned char str[2] = {0xff, 0xff};
    fread(str, 1, 2, f);
    return sget2(str, order);
}
int get4(FILE* f, Endianness order)
{
    unsigned char str[4] = {0xff, 0xff, 0xff, 0xff};
    fread(str, 1, 4, f);
    return sget4(str, order);
}
short int int2_to_signed(short unsigned int i)
{
    union
    {
        short unsigned int i;
        short int s;
    } u;
    u.i = i;
    return u.s;
}

/**
 * A class representing a single TIFF tag
 */
class Tag
{

public:
    unsigned short tagID{0};
    TagType type{TagType::T_INVALID};
    unsigned int datasize{0};
    std::vector<unsigned char> v_value;
    Endianness order{Endianness::UNKNOWN};

    Tag() = default;
    Tag(unsigned short tag, TagType type, unsigned int datasize, Endianness order, FILE* f);

    ~Tag() = default;

    bool operator==(const Tag& t) const;

    int toInt(int ofs, TagType astype = TagType::T_INVALID) const;
    double toDouble(int ofs = 0) const;
    void toString(char* buffer, std::size_t size, int ofs = 0) const;

    std::string valueToString() const;
};

Tag::Tag(unsigned short tag, TagType type, unsigned int datasize, Endianness order, FILE* f)
    : // file index supposed to be at rigth location to read datasize
    tagID(tag)
    , type(type)
    , datasize(datasize)
    , order(order)
{

    // load value field (possibly seek before)
    const int valuesize = datasize * getTypeSize(type);

    if(valuesize > 4)
    {
        fseek(f, get4(f, order), SEEK_SET);
    }

    // read value
    // value = new unsigned char[valuesize + 1];
    v_value.resize(valuesize + 1);
    auto readSize = fread(v_value.data(), 1, valuesize, f);
    // value[readSize] = '\0';
    v_value[readSize] = '\0';
}

bool Tag::operator==(const Tag& t) const
{
    return (tagID == t.tagID);
}

int Tag::toInt(int ofs, TagType astype) const
{
    if(astype == TagType::T_INVALID)
    {
        astype = type;
    }

    switch(astype)
    {
        case TagType::T_SBYTE:
            return int(static_cast<signed char>(v_value[ofs]));

        case TagType::T_BYTE:
            return v_value[ofs];

        case TagType::T_ASCII:
            return 0;

        case TagType::T_SSHORT:
            return (int)int2_to_signed(sget2(v_value.data() + ofs, order));

        case TagType::T_SHORT:
            return (int)sget2(v_value.data() + ofs, order);

        case TagType::T_SLONG:
        case TagType::T_LONG:
            return (int)sget4(v_value.data() + ofs, order);

        case TagType::T_SRATIONAL:
        {
            int a = (int)sget4(v_value.data() + ofs + 4, order);
            return a == 0 ? 0 : (int)sget4(v_value.data() + ofs, order) / a;
        }

        case TagType::T_RATIONAL:
        {
            uint32_t a = (uint32_t)sget4(v_value.data() + ofs + 4, order);
            return a == 0 ? 0 : (uint32_t)sget4(v_value.data() + ofs, order) / a;
        }

        case TagType::T_FLOAT:
            return (int)toDouble(ofs);

        case TagType::T_UNDEFINED:
            return 0;

        default:
            return 0; // Quick fix for missing cases (INVALID, DOUBLE, OLYUNDEF, SUBDIR)
    }

    return 0;
}

double Tag::toDouble(int ofs) const
{
    union IntFloat
    {
        uint32_t i;
        float f;
    } conv;

    double ud, dd;

    switch(type)
    {
        case TagType::T_SBYTE:
            return (double)(static_cast<signed char>(v_value[ofs]));

        case TagType::T_BYTE:
            return (double)((int)v_value[ofs]);

        case TagType::T_ASCII:
            return 0.0;

        case TagType::T_SSHORT:
            return (double)int2_to_signed(sget2(v_value.data() + ofs, order));

        case TagType::T_SHORT:
            return (double)((int)sget2(v_value.data() + ofs, order));

        case TagType::T_SLONG:
        case TagType::T_LONG:
            return (double)((int)sget4(v_value.data() + ofs, order));

        case TagType::T_SRATIONAL:
            ud = (int)sget4(v_value.data() + ofs, order);
            dd = (int)sget4(v_value.data() + ofs + 4, order);
            return dd == 0. ? 0. : ud / dd;

        case TagType::T_RATIONAL:
            ud = (uint32_t)sget4(v_value.data() + ofs, order);
            dd = (uint32_t)sget4(v_value.data() + ofs + 4, order);
            return dd == 0. ? 0. : ud / dd;

        case TagType::T_FLOAT:
            conv.i = sget4(v_value.data() + ofs, order);
            return conv.f;

        case TagType::T_UNDEFINED:
            return 0.;

        default:
            return 0.;
    }
}

void Tag::toString(char* buffer, std::size_t size, int ofs) const
{
    if(!buffer || !size)
    {
        return;
    }

    if(type == TagType::T_UNDEFINED)
    {
        bool isstring = true;

        for(unsigned int i = 0; (i + ofs < datasize) && (i < 64) && v_value[i + ofs]; ++i)
        {
            if((v_value[i + ofs] < 32) || (v_value[i + ofs] > 126))
            {
                isstring = false;
            }
        }

        if(isstring)
        {
            if(size < 3)
            {
                return;
            }

            std::size_t j = 0;

            for(unsigned int i = 0; (i + ofs < datasize) && (i < 64) && v_value[i + ofs]; ++i)
            {
                if(v_value[i + ofs] == '<' || v_value[i + ofs] == '>')
                {
                    buffer[j++] = '\\';
                    if(j > size - 2)
                    {
                        break;
                    }
                }

                buffer[j++] = v_value[i + ofs];
                if(j > size - 2)
                {
                    break;
                }
            }

            buffer[j++] = 0;
            return;
        }
    }
    else if(type == TagType::T_ASCII)
    {
        snprintf(buffer, size, "%.64s", v_value.data() + ofs);
        return;
    }

    size_t maxcount = std::min<size_t>(datasize, 10);

    buffer[0] = 0;

    for(int i = 0; i < std::min<int>(maxcount, datasize * getTypeSize(type) - ofs); i++)
    {
        std::size_t len = strlen(buffer);

        if(i > 0 && size - len > 2)
        {
            strcat(buffer, ", ");
            len += 2;
        }

        char* b = buffer + len;

        switch(type)
        {
            case TagType::T_UNDEFINED:
            case TagType::T_BYTE:
                snprintf(b, size - len, "%d", v_value[i + ofs]);
                break;

            case TagType::T_SSHORT:
                snprintf(b, size - len, "%d", toInt(2 * i + ofs));
                break;

            case TagType::T_SHORT:
                snprintf(b, size - len, "%u", toInt(2 * i + ofs));
                break;

            case TagType::T_SLONG:
                snprintf(b, size - len, "%d", toInt(4 * i + ofs));
                break;

            case TagType::T_LONG:
                snprintf(b, size - len, "%u", toInt(4 * i + ofs));
                break;

            case TagType::T_SRATIONAL:
                snprintf(b, size - len, "%d/%d", (int)sget4(v_value.data() + 8 * i + ofs, order),
                         (int)sget4(v_value.data() + 8 * i + ofs + 4, order));
                break;

            case TagType::T_RATIONAL:
                snprintf(b, size - len, "%u/%u", (uint32_t)sget4(v_value.data() + 8 * i + ofs, order),
                         (uint32_t)sget4(v_value.data() + 8 * i + ofs + 4, order));
                break;

            case TagType::T_FLOAT:
                snprintf(b, size - len, "%g", toDouble(8 * i + ofs));
                break;

            default:
                break;
        }
    }

    if(datasize > maxcount && size - strlen(buffer) > 3)
    {
        strcat(buffer, "...");
    }
}

std::string Tag::valueToString() const
{
    char buffer[1024];
    toString(buffer, sizeof(buffer));
    return buffer;
}

const Tag* findTag(TagKey tagID, const std::vector<Tag>& v_Tags)
{
    size_t idx = 0;
    while(idx < v_Tags.size() && (v_Tags[idx].tagID != (unsigned short)tagID))
    {
        idx++;
    }
    if(idx == v_Tags.size())
        return nullptr;
    return &(v_Tags[idx]);
}



// Spline interpolation from user data to get the internal tone curve with 65536 values
// https://en.wikipedia.org/wiki/Spline_interpolation
//
void SplineToneCurve::Set(const std::vector<double>& v_xy)
{
    std::vector<double> v_x;
    std::vector<double> v_y;
    std::vector<double> v_ypp;

    const size_t N = v_xy.size() / 2;

    for(int i = 0; i < N; ++i)
    {
        v_x.push_back(v_xy[2 * i]);
        v_y.push_back(v_xy[2 * i + 1]);
        v_ypp.push_back(0.0);
    }
    std::vector<double> v_u(N - 1, 0.0);

    v_ypp[0] = v_u[0] = 0.0; /* Natural spline */

    for(int i = 1; i < N - 1; ++i)
    {
        const double sig = (v_x[i] - v_x[i - 1]) / (v_x[i + 1] - v_x[i - 1]);
        const double p = sig * v_ypp[i - 1] + 2.0;
        v_ypp[i] = (sig - 1.0) / p;
        v_u[i] = ((v_y[i + 1] - v_y[i]) / (v_x[i + 1] - v_x[i]) - (v_y[i] - v_y[i - 1]) / (v_x[i] - v_x[i - 1]));
        v_u[i] = (6.0 * v_u[i] / (v_x[i + 1] - v_x[i - 1]) - sig * v_u[i - 1]) / p;
    }

    v_ypp[N - 1] = 0.0;

    for(int k = N - 2; k >= 0; --k)
    {
        v_ypp[k] = v_ypp[k] * v_ypp[k + 1] + v_u[k];
    }

    int k_hi = 0;

    for(int i = 0; i < 65536; ++i)
    {

        const float t = float(i) / 65535.f;
        while((v_x[k_hi] <= t) && (k_hi < v_x.size() - 1))
            k_hi++;

        const int k_lo = k_hi - 1;
        const double h = v_x[k_hi] - v_x[k_lo];
        const double a = (v_x[k_hi] - t) / h;
        const double b = (t - v_x[k_lo]) / h;
        const double r = a * v_y[k_lo] + b * v_y[k_hi] +
                         ((a * a * a - a) * v_ypp[k_lo] + (b * b * b - b) * v_ypp[k_hi]) * (h * h) *
                             0.1666666666666666666666666666666;

        ffffToneCurve[i] = (float)(r > 0.0 ? (r < 1.0 ? r : 1.0) : 0.0) * 65535.f;
    }
}

void SplineToneCurve::Apply(float& ir, float& ig, float& ib) const
{
    float r = clamp<float>(ir, 0.f, 65535.0f);
    float g = clamp<float>(ig, 0.f, 65535.0f);
    float b = clamp<float>(ib, 0.f, 65535.0f);

    if(r >= g)
    {
        if(g > b)
        {
            RGBTone(r, g, b); // Case 1: r >= g >  b
        }
        else if(b > r)
        {
            RGBTone(b, r, g); // Case 2: b >  r >= g
        }
        else if(b > g)
        {
            RGBTone(r, b, g); // Case 3: r >= b >  g
        }
        else
        { // Case 4: r == g == b
            r = getval(r);
            g = getval(g);
            b = g;
        }
    }
    else
    {
        if(r >= b)
        {
            RGBTone(g, r, b); // Case 5: g >  r >= b
        }
        else if(b > g)
        {
            RGBTone(b, g, r); // Case 6: b >  g >  r
        }
        else
        {
            RGBTone(g, b, r); // Case 7: g >= b >  r
        }
    }

    if(!(ir < 0.0 || ir > 65535.0) || !(ig < 0.0 || ig > 65535.0) || !(ib < 0.0 || ib > 65535.0))
    {
        ir = r;
        ig = g;
        ib = b;
    }
}

void SplineToneCurve::RGBTone(float& maxval, float& medval, float& minval) const
{
    const float minvalold = minval, medvalold = medval, maxvalold = maxval;

    maxval = getval(maxval);
    minval = getval(minval);
    medval = minval + ((maxval - minval) * (medvalold - minvalold) / (maxvalold - minvalold));
}

float SplineToneCurve::getval(const float idx) const
{
    const int idx_int = (int)idx;
    const float idx_dec = idx - idx_int;

    if(idx_int < 0)
        return ffffToneCurve.front();
    else if(idx_int >= 65535)
        return ffffToneCurve.back();
    else
        return idx_dec * ffffToneCurve[idx_int] + (1.f - idx_dec) * ffffToneCurve[idx_int + 1];
}


DCPProfile::DCPProfile()
    : baseline_exposure_offset(0.0)
    , analogBalance(IdentityMatrix)
    , camera_calibration_1(IdentityMatrix)
    , camera_calibration_2(IdentityMatrix)
    , color_matrix_1(IdentityMatrix)
    , color_matrix_2(IdentityMatrix)
    , forward_matrix_1(IdentityMatrix)
    , forward_matrix_2(IdentityMatrix)
    , ws_sRGB(IdentityMatrix)
    , sRGB_ws(IdentityMatrix)
{}

DCPProfile::DCPProfile(const std::string& filename)
    : baseline_exposure_offset(0.0)
    , analogBalance(IdentityMatrix)
    , camera_calibration_1(IdentityMatrix)
    , camera_calibration_2(IdentityMatrix)
    , color_matrix_1(IdentityMatrix)
    , color_matrix_2(IdentityMatrix)
    , forward_matrix_1(IdentityMatrix)
    , forward_matrix_2(IdentityMatrix)
    , ws_sRGB(IdentityMatrix)
    , sRGB_ws(IdentityMatrix)
{
    Load(filename);
}

DCPProfile::~DCPProfile() = default;

void DCPProfile::Load(const std::string& filename)
{
    delta_info.hue_step = delta_info.val_step = look_info.hue_step = look_info.val_step = 0;
    constexpr int tiff_float_size = 4;

    static const float adobe_camera_raw_default_curve[] = {
        0.00000f, 0.00078f, 0.00160f, 0.00242f, 0.00314f, 0.00385f, 0.00460f, 0.00539f, 0.00623f, 0.00712f, 0.00806f,
        0.00906f, 0.01012f, 0.01122f, 0.01238f, 0.01359f, 0.01485f, 0.01616f, 0.01751f, 0.01890f, 0.02033f, 0.02180f,
        0.02331f, 0.02485f, 0.02643f, 0.02804f, 0.02967f, 0.03134f, 0.03303f, 0.03475f, 0.03648f, 0.03824f, 0.04002f,
        0.04181f, 0.04362f, 0.04545f, 0.04730f, 0.04916f, 0.05103f, 0.05292f, 0.05483f, 0.05675f, 0.05868f, 0.06063f,
        0.06259f, 0.06457f, 0.06655f, 0.06856f, 0.07057f, 0.07259f, 0.07463f, 0.07668f, 0.07874f, 0.08081f, 0.08290f,
        0.08499f, 0.08710f, 0.08921f, 0.09134f, 0.09348f, 0.09563f, 0.09779f, 0.09996f, 0.10214f, 0.10433f, 0.10652f,
        0.10873f, 0.11095f, 0.11318f, 0.11541f, 0.11766f, 0.11991f, 0.12218f, 0.12445f, 0.12673f, 0.12902f, 0.13132f,
        0.13363f, 0.13595f, 0.13827f, 0.14061f, 0.14295f, 0.14530f, 0.14765f, 0.15002f, 0.15239f, 0.15477f, 0.15716f,
        0.15956f, 0.16197f, 0.16438f, 0.16680f, 0.16923f, 0.17166f, 0.17410f, 0.17655f, 0.17901f, 0.18148f, 0.18395f,
        0.18643f, 0.18891f, 0.19141f, 0.19391f, 0.19641f, 0.19893f, 0.20145f, 0.20398f, 0.20651f, 0.20905f, 0.21160f,
        0.21416f, 0.21672f, 0.21929f, 0.22185f, 0.22440f, 0.22696f, 0.22950f, 0.23204f, 0.23458f, 0.23711f, 0.23963f,
        0.24215f, 0.24466f, 0.24717f, 0.24967f, 0.25216f, 0.25465f, 0.25713f, 0.25961f, 0.26208f, 0.26454f, 0.26700f,
        0.26945f, 0.27189f, 0.27433f, 0.27676f, 0.27918f, 0.28160f, 0.28401f, 0.28641f, 0.28881f, 0.29120f, 0.29358f,
        0.29596f, 0.29833f, 0.30069f, 0.30305f, 0.30540f, 0.30774f, 0.31008f, 0.31241f, 0.31473f, 0.31704f, 0.31935f,
        0.32165f, 0.32395f, 0.32623f, 0.32851f, 0.33079f, 0.33305f, 0.33531f, 0.33756f, 0.33981f, 0.34205f, 0.34428f,
        0.34650f, 0.34872f, 0.35093f, 0.35313f, 0.35532f, 0.35751f, 0.35969f, 0.36187f, 0.36404f, 0.36620f, 0.36835f,
        0.37050f, 0.37264f, 0.37477f, 0.37689f, 0.37901f, 0.38112f, 0.38323f, 0.38533f, 0.38742f, 0.38950f, 0.39158f,
        0.39365f, 0.39571f, 0.39777f, 0.39982f, 0.40186f, 0.40389f, 0.40592f, 0.40794f, 0.40996f, 0.41197f, 0.41397f,
        0.41596f, 0.41795f, 0.41993f, 0.42191f, 0.42388f, 0.42584f, 0.42779f, 0.42974f, 0.43168f, 0.43362f, 0.43554f,
        0.43747f, 0.43938f, 0.44129f, 0.44319f, 0.44509f, 0.44698f, 0.44886f, 0.45073f, 0.45260f, 0.45447f, 0.45632f,
        0.45817f, 0.46002f, 0.46186f, 0.46369f, 0.46551f, 0.46733f, 0.46914f, 0.47095f, 0.47275f, 0.47454f, 0.47633f,
        0.47811f, 0.47989f, 0.48166f, 0.48342f, 0.48518f, 0.48693f, 0.48867f, 0.49041f, 0.49214f, 0.49387f, 0.49559f,
        0.49730f, 0.49901f, 0.50072f, 0.50241f, 0.50410f, 0.50579f, 0.50747f, 0.50914f, 0.51081f, 0.51247f, 0.51413f,
        0.51578f, 0.51742f, 0.51906f, 0.52069f, 0.52232f, 0.52394f, 0.52556f, 0.52717f, 0.52878f, 0.53038f, 0.53197f,
        0.53356f, 0.53514f, 0.53672f, 0.53829f, 0.53986f, 0.54142f, 0.54297f, 0.54452f, 0.54607f, 0.54761f, 0.54914f,
        0.55067f, 0.55220f, 0.55371f, 0.55523f, 0.55673f, 0.55824f, 0.55973f, 0.56123f, 0.56271f, 0.56420f, 0.56567f,
        0.56715f, 0.56861f, 0.57007f, 0.57153f, 0.57298f, 0.57443f, 0.57587f, 0.57731f, 0.57874f, 0.58017f, 0.58159f,
        0.58301f, 0.58443f, 0.58583f, 0.58724f, 0.58864f, 0.59003f, 0.59142f, 0.59281f, 0.59419f, 0.59556f, 0.59694f,
        0.59830f, 0.59966f, 0.60102f, 0.60238f, 0.60373f, 0.60507f, 0.60641f, 0.60775f, 0.60908f, 0.61040f, 0.61173f,
        0.61305f, 0.61436f, 0.61567f, 0.61698f, 0.61828f, 0.61957f, 0.62087f, 0.62216f, 0.62344f, 0.62472f, 0.62600f,
        0.62727f, 0.62854f, 0.62980f, 0.63106f, 0.63232f, 0.63357f, 0.63482f, 0.63606f, 0.63730f, 0.63854f, 0.63977f,
        0.64100f, 0.64222f, 0.64344f, 0.64466f, 0.64587f, 0.64708f, 0.64829f, 0.64949f, 0.65069f, 0.65188f, 0.65307f,
        0.65426f, 0.65544f, 0.65662f, 0.65779f, 0.65897f, 0.66013f, 0.66130f, 0.66246f, 0.66362f, 0.66477f, 0.66592f,
        0.66707f, 0.66821f, 0.66935f, 0.67048f, 0.67162f, 0.67275f, 0.67387f, 0.67499f, 0.67611f, 0.67723f, 0.67834f,
        0.67945f, 0.68055f, 0.68165f, 0.68275f, 0.68385f, 0.68494f, 0.68603f, 0.68711f, 0.68819f, 0.68927f, 0.69035f,
        0.69142f, 0.69249f, 0.69355f, 0.69461f, 0.69567f, 0.69673f, 0.69778f, 0.69883f, 0.69988f, 0.70092f, 0.70196f,
        0.70300f, 0.70403f, 0.70506f, 0.70609f, 0.70711f, 0.70813f, 0.70915f, 0.71017f, 0.71118f, 0.71219f, 0.71319f,
        0.71420f, 0.71520f, 0.71620f, 0.71719f, 0.71818f, 0.71917f, 0.72016f, 0.72114f, 0.72212f, 0.72309f, 0.72407f,
        0.72504f, 0.72601f, 0.72697f, 0.72794f, 0.72890f, 0.72985f, 0.73081f, 0.73176f, 0.73271f, 0.73365f, 0.73460f,
        0.73554f, 0.73647f, 0.73741f, 0.73834f, 0.73927f, 0.74020f, 0.74112f, 0.74204f, 0.74296f, 0.74388f, 0.74479f,
        0.74570f, 0.74661f, 0.74751f, 0.74842f, 0.74932f, 0.75021f, 0.75111f, 0.75200f, 0.75289f, 0.75378f, 0.75466f,
        0.75555f, 0.75643f, 0.75730f, 0.75818f, 0.75905f, 0.75992f, 0.76079f, 0.76165f, 0.76251f, 0.76337f, 0.76423f,
        0.76508f, 0.76594f, 0.76679f, 0.76763f, 0.76848f, 0.76932f, 0.77016f, 0.77100f, 0.77183f, 0.77267f, 0.77350f,
        0.77432f, 0.77515f, 0.77597f, 0.77680f, 0.77761f, 0.77843f, 0.77924f, 0.78006f, 0.78087f, 0.78167f, 0.78248f,
        0.78328f, 0.78408f, 0.78488f, 0.78568f, 0.78647f, 0.78726f, 0.78805f, 0.78884f, 0.78962f, 0.79040f, 0.79118f,
        0.79196f, 0.79274f, 0.79351f, 0.79428f, 0.79505f, 0.79582f, 0.79658f, 0.79735f, 0.79811f, 0.79887f, 0.79962f,
        0.80038f, 0.80113f, 0.80188f, 0.80263f, 0.80337f, 0.80412f, 0.80486f, 0.80560f, 0.80634f, 0.80707f, 0.80780f,
        0.80854f, 0.80926f, 0.80999f, 0.81072f, 0.81144f, 0.81216f, 0.81288f, 0.81360f, 0.81431f, 0.81503f, 0.81574f,
        0.81645f, 0.81715f, 0.81786f, 0.81856f, 0.81926f, 0.81996f, 0.82066f, 0.82135f, 0.82205f, 0.82274f, 0.82343f,
        0.82412f, 0.82480f, 0.82549f, 0.82617f, 0.82685f, 0.82753f, 0.82820f, 0.82888f, 0.82955f, 0.83022f, 0.83089f,
        0.83155f, 0.83222f, 0.83288f, 0.83354f, 0.83420f, 0.83486f, 0.83552f, 0.83617f, 0.83682f, 0.83747f, 0.83812f,
        0.83877f, 0.83941f, 0.84005f, 0.84069f, 0.84133f, 0.84197f, 0.84261f, 0.84324f, 0.84387f, 0.84450f, 0.84513f,
        0.84576f, 0.84639f, 0.84701f, 0.84763f, 0.84825f, 0.84887f, 0.84949f, 0.85010f, 0.85071f, 0.85132f, 0.85193f,
        0.85254f, 0.85315f, 0.85375f, 0.85436f, 0.85496f, 0.85556f, 0.85615f, 0.85675f, 0.85735f, 0.85794f, 0.85853f,
        0.85912f, 0.85971f, 0.86029f, 0.86088f, 0.86146f, 0.86204f, 0.86262f, 0.86320f, 0.86378f, 0.86435f, 0.86493f,
        0.86550f, 0.86607f, 0.86664f, 0.86720f, 0.86777f, 0.86833f, 0.86889f, 0.86945f, 0.87001f, 0.87057f, 0.87113f,
        0.87168f, 0.87223f, 0.87278f, 0.87333f, 0.87388f, 0.87443f, 0.87497f, 0.87552f, 0.87606f, 0.87660f, 0.87714f,
        0.87768f, 0.87821f, 0.87875f, 0.87928f, 0.87981f, 0.88034f, 0.88087f, 0.88140f, 0.88192f, 0.88244f, 0.88297f,
        0.88349f, 0.88401f, 0.88453f, 0.88504f, 0.88556f, 0.88607f, 0.88658f, 0.88709f, 0.88760f, 0.88811f, 0.88862f,
        0.88912f, 0.88963f, 0.89013f, 0.89063f, 0.89113f, 0.89163f, 0.89212f, 0.89262f, 0.89311f, 0.89360f, 0.89409f,
        0.89458f, 0.89507f, 0.89556f, 0.89604f, 0.89653f, 0.89701f, 0.89749f, 0.89797f, 0.89845f, 0.89892f, 0.89940f,
        0.89987f, 0.90035f, 0.90082f, 0.90129f, 0.90176f, 0.90222f, 0.90269f, 0.90316f, 0.90362f, 0.90408f, 0.90454f,
        0.90500f, 0.90546f, 0.90592f, 0.90637f, 0.90683f, 0.90728f, 0.90773f, 0.90818f, 0.90863f, 0.90908f, 0.90952f,
        0.90997f, 0.91041f, 0.91085f, 0.91130f, 0.91173f, 0.91217f, 0.91261f, 0.91305f, 0.91348f, 0.91392f, 0.91435f,
        0.91478f, 0.91521f, 0.91564f, 0.91606f, 0.91649f, 0.91691f, 0.91734f, 0.91776f, 0.91818f, 0.91860f, 0.91902f,
        0.91944f, 0.91985f, 0.92027f, 0.92068f, 0.92109f, 0.92150f, 0.92191f, 0.92232f, 0.92273f, 0.92314f, 0.92354f,
        0.92395f, 0.92435f, 0.92475f, 0.92515f, 0.92555f, 0.92595f, 0.92634f, 0.92674f, 0.92713f, 0.92753f, 0.92792f,
        0.92831f, 0.92870f, 0.92909f, 0.92947f, 0.92986f, 0.93025f, 0.93063f, 0.93101f, 0.93139f, 0.93177f, 0.93215f,
        0.93253f, 0.93291f, 0.93328f, 0.93366f, 0.93403f, 0.93440f, 0.93478f, 0.93515f, 0.93551f, 0.93588f, 0.93625f,
        0.93661f, 0.93698f, 0.93734f, 0.93770f, 0.93807f, 0.93843f, 0.93878f, 0.93914f, 0.93950f, 0.93986f, 0.94021f,
        0.94056f, 0.94092f, 0.94127f, 0.94162f, 0.94197f, 0.94231f, 0.94266f, 0.94301f, 0.94335f, 0.94369f, 0.94404f,
        0.94438f, 0.94472f, 0.94506f, 0.94540f, 0.94573f, 0.94607f, 0.94641f, 0.94674f, 0.94707f, 0.94740f, 0.94774f,
        0.94807f, 0.94839f, 0.94872f, 0.94905f, 0.94937f, 0.94970f, 0.95002f, 0.95035f, 0.95067f, 0.95099f, 0.95131f,
        0.95163f, 0.95194f, 0.95226f, 0.95257f, 0.95289f, 0.95320f, 0.95351f, 0.95383f, 0.95414f, 0.95445f, 0.95475f,
        0.95506f, 0.95537f, 0.95567f, 0.95598f, 0.95628f, 0.95658f, 0.95688f, 0.95718f, 0.95748f, 0.95778f, 0.95808f,
        0.95838f, 0.95867f, 0.95897f, 0.95926f, 0.95955f, 0.95984f, 0.96013f, 0.96042f, 0.96071f, 0.96100f, 0.96129f,
        0.96157f, 0.96186f, 0.96214f, 0.96242f, 0.96271f, 0.96299f, 0.96327f, 0.96355f, 0.96382f, 0.96410f, 0.96438f,
        0.96465f, 0.96493f, 0.96520f, 0.96547f, 0.96574f, 0.96602f, 0.96629f, 0.96655f, 0.96682f, 0.96709f, 0.96735f,
        0.96762f, 0.96788f, 0.96815f, 0.96841f, 0.96867f, 0.96893f, 0.96919f, 0.96945f, 0.96971f, 0.96996f, 0.97022f,
        0.97047f, 0.97073f, 0.97098f, 0.97123f, 0.97149f, 0.97174f, 0.97199f, 0.97223f, 0.97248f, 0.97273f, 0.97297f,
        0.97322f, 0.97346f, 0.97371f, 0.97395f, 0.97419f, 0.97443f, 0.97467f, 0.97491f, 0.97515f, 0.97539f, 0.97562f,
        0.97586f, 0.97609f, 0.97633f, 0.97656f, 0.97679f, 0.97702f, 0.97725f, 0.97748f, 0.97771f, 0.97794f, 0.97817f,
        0.97839f, 0.97862f, 0.97884f, 0.97907f, 0.97929f, 0.97951f, 0.97973f, 0.97995f, 0.98017f, 0.98039f, 0.98061f,
        0.98082f, 0.98104f, 0.98125f, 0.98147f, 0.98168f, 0.98189f, 0.98211f, 0.98232f, 0.98253f, 0.98274f, 0.98295f,
        0.98315f, 0.98336f, 0.98357f, 0.98377f, 0.98398f, 0.98418f, 0.98438f, 0.98458f, 0.98478f, 0.98498f, 0.98518f,
        0.98538f, 0.98558f, 0.98578f, 0.98597f, 0.98617f, 0.98636f, 0.98656f, 0.98675f, 0.98694f, 0.98714f, 0.98733f,
        0.98752f, 0.98771f, 0.98789f, 0.98808f, 0.98827f, 0.98845f, 0.98864f, 0.98882f, 0.98901f, 0.98919f, 0.98937f,
        0.98955f, 0.98973f, 0.98991f, 0.99009f, 0.99027f, 0.99045f, 0.99063f, 0.99080f, 0.99098f, 0.99115f, 0.99133f,
        0.99150f, 0.99167f, 0.99184f, 0.99201f, 0.99218f, 0.99235f, 0.99252f, 0.99269f, 0.99285f, 0.99302f, 0.99319f,
        0.99335f, 0.99351f, 0.99368f, 0.99384f, 0.99400f, 0.99416f, 0.99432f, 0.99448f, 0.99464f, 0.99480f, 0.99495f,
        0.99511f, 0.99527f, 0.99542f, 0.99558f, 0.99573f, 0.99588f, 0.99603f, 0.99619f, 0.99634f, 0.99649f, 0.99664f,
        0.99678f, 0.99693f, 0.99708f, 0.99722f, 0.99737f, 0.99751f, 0.99766f, 0.99780f, 0.99794f, 0.99809f, 0.99823f,
        0.99837f, 0.99851f, 0.99865f, 0.99879f, 0.99892f, 0.99906f, 0.99920f, 0.99933f, 0.99947f, 0.99960f, 0.99974f,
        0.99987f, 1.00000f };

    FILE* const file = fopen(filename.c_str(), "rb");

    if (file == nullptr)
    {
        ALICEVISION_THROW_ERROR("Unable to load DCP profile " << filename);
    }

    // read tiff header
    fseek(file, 0, SEEK_SET);
    unsigned short bo;
    fread(&bo, 1, 2, file);
    Endianness order = (Endianness)((int)bo);
    get2(file, order);
    int ifdOffset = get4(file, order);

    // read tags
    const int numOfTags = get2(file, order);

    if (numOfTags <= 0 || numOfTags > 1000)
    {
        ALICEVISION_LOG_WARNING("DCP Error: Tag number out of range.");
        fclose(file);
        return;
    }

    std::vector<Tag> v_tag;

    for (int i = 0; i < numOfTags; i++)
    {

        unsigned short tag = get2(file, order);
        TagType type = (TagType)get2(file, order);
        unsigned int datasize = get4(file, order);

        if (!datasize)
        {
            datasize = 1;
        }

        // filter out invalid tags
        // note the large count is to be able to pass LeafData ASCII tag which can be up to almost 10 megabytes,
        // (only a small part of it will actually be parsed though)
        if ((int)type < 1 || (int)type > 14 || datasize > 10 * 1024 * 1024)
        {
            type = TagType::T_INVALID;
            fclose(file);
            ALICEVISION_LOG_WARNING("ERROR : Invalid Tag in dcp file or file too big : datasize = " << datasize);
            return;
        }

        // store next Tag's position in file
        int save = ftell(file) + 4;

        Tag t(tag, type, datasize, order, file);

        v_tag.push_back(t);

        fseek(file, save, SEEK_SET);
    }

    fclose(file);

    info.filename = filename;

    const Tag* tag = findTag(TagKey::CALIBRATION_ILLUMINANT_1, v_tag);
    info.light_source_1 = tag ? tag->toInt(0, TagType::T_SHORT) : -1;
    tag = findTag(TagKey::CALIBRATION_ILLUMINANT_2, v_tag);
    info.light_source_2 = tag ? tag->toInt(0, TagType::T_SHORT) : -1;
    info.temperature_1 = calibrationIlluminantToTemperature(LightSource(info.light_source_1));
    info.temperature_2 = calibrationIlluminantToTemperature(LightSource(info.light_source_2));

    const bool has_second_hue_sat =
        findTag(TagKey::PROFILE_HUE_SAT_MAP_DATA_2, v_tag); // Some profiles have two matrices, but just one huesat

    // Fetch Forward Matrices, if any
    tag = findTag(TagKey::FORWARD_MATRIX_1, v_tag);

    if (tag)
    {
        info.has_forward_matrix_1 = true;

        for (int row = 0; row < 3; ++row)
        {
            for (int col = 0; col < 3; ++col)
            {
                forward_matrix_1[row][col] = tag->toDouble((col + row * 3) * 8);
            }
        }
    }

    tag = findTag(TagKey::FORWARD_MATRIX_2, v_tag);

    if (tag)
    {
        info.has_forward_matrix_2 = true;

        for (int row = 0; row < 3; ++row)
        {
            for (int col = 0; col < 3; ++col)
            {
                forward_matrix_2[row][col] = tag->toDouble((col + row * 3) * 8);
            }
        }
    }

    // Color Matrix (one is always there)
    tag = findTag(TagKey::COLOR_MATRIX_1, v_tag);

    if (!tag)
    {
        ALICEVISION_LOG_WARNING("DCP '" << filename << "' is missing 'ColorMatrix1'. Skipped.");
        return;
    }

    info.has_color_matrix_1 = true;

    for (int row = 0; row < 3; ++row)
    {
        for (int col = 0; col < 3; ++col)
        {
            color_matrix_1[row][col] = tag->toDouble((col + row * 3) * 8);
        }
    }

    tag = findTag(TagKey::CAMERA_CALIBRATION_1, v_tag);

    if (tag)
    {
        info.has_camera_calibration_1 = true;

        for (int row = 0; row < 3; ++row)
        {
            for (int col = 0; col < 3; ++col)
            {
                camera_calibration_1[row][col] = tag->toDouble((col + row * 3) * 8);
            }
        }
    }

    tag = findTag(TagKey::PROFILE_LOOK_TABLE_DIMS, v_tag);

    if (tag)
    {
        look_info.hue_divisions = tag->toInt(0);
        look_info.sat_divisions = tag->toInt(4);
        look_info.val_divisions = tag->toInt(8);

        tag = findTag(TagKey::PROFILE_LOOK_TABLE_ENCODING, v_tag);
        look_info.srgb_gamma = tag && tag->toInt(0);

        tag = findTag(TagKey::PROFILE_LOOK_TABLE_DATA, v_tag);
        look_info.array_count = tag->datasize / 3;

        look_table.resize(look_info.array_count);

        for (unsigned int i = 0; i < look_info.array_count; i++)
        {
            look_table[i].hue_shift = tag->toDouble((i * 3) * tiff_float_size);
            look_table[i].sat_scale = tag->toDouble((i * 3 + 1) * tiff_float_size);
            look_table[i].val_scale = tag->toDouble((i * 3 + 2) * tiff_float_size);
        }

        // Precalculated constants for table application
        look_info.pc.h_scale = look_info.hue_divisions < 2 ? 0.0f : static_cast<float>(look_info.hue_divisions) / 6.0f;
        look_info.pc.s_scale = look_info.sat_divisions - 1;
        look_info.pc.v_scale = look_info.val_divisions - 1;
        look_info.pc.max_hue_index0 = look_info.hue_divisions - 1;
        look_info.pc.max_sat_index0 = look_info.sat_divisions - 2;
        look_info.pc.max_val_index0 = look_info.val_divisions - 2;
        look_info.pc.hue_step = look_info.sat_divisions;
        look_info.pc.val_step = look_info.hue_divisions * look_info.pc.hue_step;

        info.has_look_table = true;
    }

    tag = findTag(TagKey::PROFILE_HUE_SAT_MAP_DIMS, v_tag);

    if (tag)
    {
        delta_info.hue_divisions = tag->toInt(0);
        delta_info.sat_divisions = tag->toInt(4);
        delta_info.val_divisions = tag->toInt(8);

        tag = findTag(TagKey::PROFILE_HUE_SAT_MAP_ENCODING, v_tag);
        delta_info.srgb_gamma = tag && tag->toInt(0);

        tag = findTag(TagKey::PROFILE_HUE_SAT_MAP_DATA_1, v_tag);
        delta_info.array_count = tag->datasize / 3;

        deltas_1.resize(delta_info.array_count);

        for (unsigned int i = 0; i < delta_info.array_count; ++i)
        {
            deltas_1[i].hue_shift = tag->toDouble((i * 3) * tiff_float_size);
            deltas_1[i].sat_scale = tag->toDouble((i * 3 + 1) * tiff_float_size);
            deltas_1[i].val_scale = tag->toDouble((i * 3 + 2) * tiff_float_size);
        }

        delta_info.pc.h_scale =
            delta_info.hue_divisions < 2 ? 0.0f : static_cast<float>(delta_info.hue_divisions) / 6.0f;
        delta_info.pc.s_scale = delta_info.sat_divisions - 1;
        delta_info.pc.v_scale = delta_info.val_divisions - 1;
        delta_info.pc.max_hue_index0 = delta_info.hue_divisions - 1;
        delta_info.pc.max_sat_index0 = delta_info.sat_divisions - 2;
        delta_info.pc.max_val_index0 = delta_info.val_divisions - 2;
        delta_info.pc.hue_step = delta_info.sat_divisions;
        delta_info.pc.val_step = delta_info.hue_divisions * delta_info.pc.hue_step;

        info.has_hue_sat_map = true;
    }

    if (info.light_source_2 != -1)
    {
        tag = findTag(TagKey::COLOR_MATRIX_2, v_tag);

        if (tag) // Second color matrix is not mandatory
        {
            info.has_color_matrix_2 = true;

            for (int row = 0; row < 3; ++row)
            {
                for (int col = 0; col < 3; ++col)
                {
                    color_matrix_2[row][col] = tag ? tag->toDouble((col + row * 3) * 8) : color_matrix_1[row][col];
                }
            }
        }

        tag = findTag(TagKey::CAMERA_CALIBRATION_2, v_tag);

        if (tag)
        {
            info.has_camera_calibration_2 = true;

            for (int row = 0; row < 3; ++row)
            {
                for (int col = 0; col < 3; ++col)
                {
                    camera_calibration_2[row][col] = tag->toDouble((col + row * 3) * 8);
                }
            }
        }

        // Second huesatmap
        if (has_second_hue_sat)
        {
            deltas_2.resize(delta_info.array_count);

            // Saturation maps. Need to be unwinded.
            tag = findTag(TagKey::PROFILE_HUE_SAT_MAP_DATA_2, v_tag);

            for (unsigned int i = 0; i < delta_info.array_count; ++i)
            {
                deltas_2[i].hue_shift = tag->toDouble((i * 3) * tiff_float_size);
                deltas_2[i].sat_scale = tag->toDouble((i * 3 + 1) * tiff_float_size);
                deltas_2[i].val_scale = tag->toDouble((i * 3 + 2) * tiff_float_size);
            }
        }
    }

    tag = findTag(TagKey::BASELINE_EXPOSURE_OFFSET, v_tag);

    if (tag)
    {
        info.has_baseline_exposure_offset = true;
        baseline_exposure_offset = tag->toDouble();
    }

    // Read tone curve points, if any, but disable to RTs own profiles
    tag = findTag(TagKey::PROFILE_TONE_CURVE, v_tag);

    if (tag)
    {
        std::vector<double> AS_curve_points;

        // Push back each X/Y coordinates in a loop
        bool curve_is_linear = true;

        for (int i = 0; i < tag->datasize; i += 2)
        {
            const double x = tag->toDouble((i + 0) * tiff_float_size);
            const double y = tag->toDouble((i + 1) * tiff_float_size);

            if (x != y)
            {
                curve_is_linear = false;
            }

            AS_curve_points.push_back(x);
            AS_curve_points.push_back(y);
        }

        if (!curve_is_linear)
        {
            // Create the curve
            info.has_tone_curve = true;
            AS_tone_curve.Set(AS_curve_points);
        }
    }
    else
    {
        tag = findTag(TagKey::PROFILE_TONE_COPYRIGHT, v_tag);

        if (tag && tag->valueToString().find("Adobe Systems") != std::string::npos)
        {
            // An Adobe profile without tone curve is expected to have the Adobe Default Curve
            std::vector<double> AS_curve_points;

            constexpr size_t tc_len =
                sizeof(adobe_camera_raw_default_curve) / sizeof(adobe_camera_raw_default_curve[0]);

            for (size_t i = 0; i < tc_len; ++i)
            {
                const double x = static_cast<double>(i) / (tc_len - 1);
                const double y = adobe_camera_raw_default_curve[i];
                AS_curve_points.push_back(x);
                AS_curve_points.push_back(y);
            }

            info.has_tone_curve = true;
            AS_tone_curve.Set(AS_curve_points);
        }
    }

    tag = findTag(TagKey::PROFILE_CALIBRATION_SIGNATURE, v_tag);

    if (tag)
    {
        info.profileCalibrationSignature = tag->valueToString();
    }

    std::vector<double> gammatab_srgb_data;
    std::vector<double> igammatab_srgb_data;
    for (int i = 0; i < 65536; i++)
    {
        double x = i / 65535.0;
        gammatab_srgb_data.push_back((x <= 0.003040) ? (x * 12.92310) : (1.055 * exp(log(x) / 2.4) - 0.055)); // from RT
        igammatab_srgb_data.push_back((x <= 0.039286) ? (x / 12.92310) : (exp(log((x + 0.055) / 1.055) * 2.4))); // from RT
    }
    gammatab_srgb.Set(gammatab_srgb_data);
    igammatab_srgb.Set(igammatab_srgb_data);
}

void DCPProfile::apply(OIIO::ImageBuf& image, const DCPProfileApplyParams& params)
{
    // Compute matrices to and from selected working space
    ws_sRGB = { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };
    sRGB_ws = { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };

    if (params.working_space.compare("sRGB"))
    {
        if (params.working_space.compare("prophoto"))
        {
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    double to_ws = 0.0;
                    double from_ws = 0.0;
                    for (int k = 0; k < 3; k++)
                    {
                        to_ws += prophoto_xyz[i][k] * xyz_sRGB[k][j];
                        from_ws += sRGB_xyz[i][k] * xyz_prophoto[k][j];
                    }
                    ws_sRGB[i][j] = to_ws;
                    sRGB_ws[i][j] = from_ws;
                }
            }
        }
        else if (params.working_space.compare("AdobeRGB"))
        {
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    double to_ws = 0.0;
                    double from_ws = 0.0;
                    for (int k = 0; k < 3; k++)
                    {
                        to_ws += AdobeRGB_xyz[i][k] * xyz_sRGB[k][j];
                        from_ws += sRGB_xyz[i][k] * xyz_AdobeRGB[k][j];
                    }
                    ws_sRGB[i][j] = to_ws;
                    sRGB_ws[i][j] = from_ws;
                }
            }
        }
    }

    // Apply DCP profile
#pragma omp parallel for
    for (int i = 0; i < image.spec().height; ++i)
        for (int j = 0; j < image.spec().width; ++j)
        {
            float rgb[3];
            image.getpixel(j, i, rgb, 3);
            for (int c = 0; c < 3; ++c)
            {
                rgb[c] *= 65535.0;
            }
            apply(rgb, params);
            for (int c = 0; c < 3; ++c)
            {
                rgb[c] /= 65535.0;
            }
            image.setpixel(j, i, rgb, 3);
        }
}

void DCPProfile::apply(float* rgb, const DCPProfileApplyParams& params) const
{
    const float exp_scale = (params.apply_baseline_exposure_offset && info.has_baseline_exposure_offset)
        ? std::pow(2.0, baseline_exposure_offset)
        : 1.0f;

    if (!params.use_tone_curve && !params.apply_look_table)
    {
        if (exp_scale == 1.f)
        {
            return;
        }

        for (int c = 0; c < 3; ++c)
        {
            rgb[c] *= exp_scale;
        }
    }
    else
    {

        for (int c = 0; c < 3; ++c)
        {
            rgb[c] *= exp_scale;
        }

        float ws_rgb[3] = { 0.0, 0.0, 0.0 };
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; ++j)
            {
                ws_rgb[i] += ws_sRGB[i][j] * rgb[j];
            }
            if (params.apply_look_table || params.use_tone_curve)
            {
                ws_rgb[i] = std::max<float>(ws_rgb[i], 0.f);
            }
        }

        if (params.apply_look_table)
        {
            float clipped_ws_rgb[3];
            for (int i = 0; i < 3; ++i)
            {
                clipped_ws_rgb[i] = clamp(ws_rgb[i], 0.f, 65535.5f);
            }

            float h, s, v;
            rgb2hsvtc(clipped_ws_rgb[0], clipped_ws_rgb[1], clipped_ws_rgb[2], h, s, v);

            hsdApply(look_info, look_table, h, s, v);
            s = clamp(s, 0.f, 1.f);
            v = clamp(v, 0.f, 1.f);

            h += (h < 0.0f ? 6.0f : (h >= 6.0f ? -6.0f : 0.0f));

            hsv2rgbdcp(h, s, v, clipped_ws_rgb[0], clipped_ws_rgb[1], clipped_ws_rgb[2]);

            if (!(ws_rgb[0] < 0.0 || ws_rgb[0] > 65535.0) ||
                !(ws_rgb[1] < 0.0 || ws_rgb[1] > 65535.0) ||
                !(ws_rgb[2] < 0.0 || ws_rgb[2] > 65535.0))
            {
                ws_rgb[0] = clipped_ws_rgb[0];
                ws_rgb[1] = clipped_ws_rgb[1];
                ws_rgb[2] = clipped_ws_rgb[2];
            }
        }

        if (params.use_tone_curve)
        {
            AS_tone_curve.Apply(ws_rgb[0], ws_rgb[1], ws_rgb[2]);
        }

        for (int i = 0; i < 3; i++)
        {
            rgb[i] = 0.0;
            for (int j = 0; j < 3; j++)
            {
                rgb[i] += sRGB_ws[i][j] * ws_rgb[j];
            }
        }
    }
}

inline void DCPProfile::hsdApply(const HsdTableInfo& table_info, const std::vector<HsbModify>& table_base, float& h,
    float& s, float& v) const
{
    // Apply the HueSatMap. Ported from Adobes reference implementation.
    float hue_shift;
    float sat_scale;
    float val_scale;
    float v_encoded = v;

    if (table_info.val_divisions < 2)
    {
        // Optimize most common case of "2.5D" table
        const float h_scaled = h * table_info.pc.h_scale;
        const float s_scaled = s * table_info.pc.s_scale;

        int h_index0 = std::max<int>(h_scaled, 0);
        const int s_index0 = std::max(std::min<int>(s_scaled, table_info.pc.max_sat_index0), 0);

        int h_index1 = h_index0 + 1;

        if (h_index0 >= table_info.pc.max_hue_index0)
        {
            h_index0 = table_info.pc.max_hue_index0;
            h_index1 = 0;
        }

        const float h_fract1 = h_scaled - static_cast<float>(h_index0);
        const float s_fract1 = s_scaled - static_cast<float>(s_index0);

        const float h_fract0 = 1.0f - h_fract1;
        const float s_fract0 = 1.0f - s_fract1;

        std::vector<HsbModify>::size_type e00_index = h_index0 * table_info.pc.hue_step + s_index0;
        std::vector<HsbModify>::size_type e01_index = e00_index + (h_index1 - h_index0) * table_info.pc.hue_step;

        const float hue_shift0 =
            h_fract0 * table_base[e00_index].hue_shift + h_fract1 * table_base[e01_index].hue_shift;
        const float sat_scale0 =
            h_fract0 * table_base[e00_index].sat_scale + h_fract1 * table_base[e01_index].sat_scale;
        const float val_scale0 =
            h_fract0 * table_base[e00_index].val_scale + h_fract1 * table_base[e01_index].val_scale;

        ++e00_index;
        ++e01_index;

        const float hueShift1 = h_fract0 * table_base[e00_index].hue_shift + h_fract1 * table_base[e01_index].hue_shift;
        const float satScale1 = h_fract0 * table_base[e00_index].sat_scale + h_fract1 * table_base[e01_index].sat_scale;
        const float valScale1 = h_fract0 * table_base[e00_index].val_scale + h_fract1 * table_base[e01_index].val_scale;

        hue_shift = s_fract0 * hue_shift0 + s_fract1 * hueShift1;
        sat_scale = s_fract0 * sat_scale0 + s_fract1 * satScale1;
        val_scale = s_fract0 * val_scale0 + s_fract1 * valScale1;
    }
    else
    {
        const float h_scaled = h * table_info.pc.h_scale;
        const float s_scaled = s * table_info.pc.s_scale;

        if (table_info.srgb_gamma)
        {
            v_encoded = gammatab_srgb[v * 65535.f];
        }

        const float v_scaled = v_encoded * table_info.pc.v_scale;

        int h_index0 = h_scaled;
        const int s_index0 = std::max(std::min<int>(s_scaled, table_info.pc.max_sat_index0), 0);
        const int v_index0 = std::max(std::min<int>(v_scaled, table_info.pc.max_val_index0), 0);

        int h_index1 = h_index0 + 1;

        if (h_index0 >= table_info.pc.max_hue_index0)
        {
            h_index0 = table_info.pc.max_hue_index0;
            h_index1 = 0;
        }

        const float h_fract1 = h_scaled - static_cast<float>(h_index0);
        const float s_fract1 = s_scaled - static_cast<float>(s_index0);
        const float v_fract1 = v_scaled - static_cast<float>(v_index0);

        const float h_fract0 = 1.0f - h_fract1;
        const float s_fract0 = 1.0f - s_fract1;
        const float v_fract0 = 1.0f - v_fract1;

        std::vector<HsbModify>::size_type e00_index =
            v_index0 * table_info.pc.val_step + h_index0 * table_info.pc.hue_step + s_index0;
        std::vector<HsbModify>::size_type e01_index = e00_index + (h_index1 - h_index0) * table_info.pc.hue_step;
        std::vector<HsbModify>::size_type e10_index = e00_index + table_info.pc.val_step;
        std::vector<HsbModify>::size_type e11_index = e01_index + table_info.pc.val_step;

        const float hueShift0 =
            v_fract0 * (h_fract0 * table_base[e00_index].hue_shift + h_fract1 * table_base[e01_index].hue_shift) +
            v_fract1 * (h_fract0 * table_base[e10_index].hue_shift + h_fract1 * table_base[e11_index].hue_shift);
        const float satScale0 =
            v_fract0 * (h_fract0 * table_base[e00_index].sat_scale + h_fract1 * table_base[e01_index].sat_scale) +
            v_fract1 * (h_fract0 * table_base[e10_index].sat_scale + h_fract1 * table_base[e11_index].sat_scale);
        const float valScale0 =
            v_fract0 * (h_fract0 * table_base[e00_index].val_scale + h_fract1 * table_base[e01_index].val_scale) +
            v_fract1 * (h_fract0 * table_base[e10_index].val_scale + h_fract1 * table_base[e11_index].val_scale);

        ++e00_index;
        ++e01_index;
        ++e10_index;
        ++e11_index;

        const float hueShift1 =
            v_fract0 * (h_fract0 * table_base[e00_index].hue_shift + h_fract1 * table_base[e01_index].hue_shift) +
            v_fract1 * (h_fract0 * table_base[e10_index].hue_shift + h_fract1 * table_base[e11_index].hue_shift);
        const float satScale1 =
            v_fract0 * (h_fract0 * table_base[e00_index].sat_scale + h_fract1 * table_base[e01_index].sat_scale) +
            v_fract1 * (h_fract0 * table_base[e10_index].sat_scale + h_fract1 * table_base[e11_index].sat_scale);
        const float valScale1 =
            v_fract0 * (h_fract0 * table_base[e00_index].val_scale + h_fract1 * table_base[e01_index].val_scale) +
            v_fract1 * (h_fract0 * table_base[e10_index].val_scale + h_fract1 * table_base[e11_index].val_scale);

        hue_shift = s_fract0 * hueShift0 + s_fract1 * hueShift1;
        sat_scale = s_fract0 * satScale0 + s_fract1 * satScale1;
        val_scale = s_fract0 * valScale0 + s_fract1 * valScale1;
    }

    hue_shift *= 6.0f / 360.0f; // Convert to internal hue range.

    h += hue_shift;
    s *= sat_scale; // No clipping here, we are RT float :-)

    if (table_info.srgb_gamma)
    {
        v = igammatab_srgb[v_encoded * val_scale * 65535.f];
    }
    else
    {
        v *= val_scale;
    }
}

DCPProfile::Matrix DCPProfile::getInterpolatedMatrix(const double cct, const std::string& type) const
{
    const double a = 1.e6 / info.temperature_1;
    const double b = 1.e6 / info.temperature_2;
    const double c = 1.e6 / cct;

    const Matrix Ma = (type == "color") ? color_matrix_1 : ((type == "calib") ? camera_calibration_1 : forward_matrix_1);
    const Matrix Mb = (type == "color") ? color_matrix_2 : ((type == "calib") ? camera_calibration_2 : forward_matrix_2);
    Matrix interpolatedMatrix;
    for (int i = 0; i < 3; ++i)
    {
        for (int j=0; j < 3; ++j)
        {
            interpolatedMatrix[i][j] = Ma[i][j] + ((c - a) / (b - a)) * (Mb[i][j] - Ma[i][j]);
        }
    }

    return interpolatedMatrix;
}

DCPProfile::Matrix DCPProfile::matMult(const Matrix& A, const Matrix& B) const
{
    Matrix res;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            res[i][j] = 0.f;
            for (int k = 0; k < 3; k++)
            {
                res[i][j] += A[i][k] * B[k][j];
            }
        }
    }
    return res;
}
DCPProfile::Triple DCPProfile::matMult(const Matrix& M, const Triple& V) const
{
    Triple res;
    for (int i = 0; i < 3; ++i)
    {
        res[i] = 0.f;
        for (int k = 0; k < 3; k++)
        {
            res[i] += M[i][k] * V[k];
        }
    }
    return res;
}
DCPProfile::Matrix DCPProfile::matInv(const Matrix& M) const
{
    Matrix inv = M;

    const float det = M[0][0]*M[1][1]*M[2][2] + M[0][1]*M[1][2]*M[2][0] + M[0][2]*M[1][0]*M[2][1] - M[2][0]*M[1][1]*M[0][2] - M[1][0]*M[0][1]*M[2][2] - M[0][0]*M[2][1]*M[1][2];

    if (det != 0.f)
    {
        inv[0][0] = (M[1][1] * M[2][2] - M[2][1] * M[1][2]) / det;
        inv[0][1] = (M[0][2] * M[2][1] - M[0][1] * M[2][2]) / det;
        inv[0][2] = (M[0][1] * M[1][2] - M[0][2] * M[1][1]) / det;
        inv[1][0] = (M[1][2] * M[2][0] - M[1][0] * M[2][2]) / det;
        inv[1][1] = (M[0][0] * M[2][2] - M[2][0] * M[0][2]) / det;
        inv[1][2] = (M[0][2] * M[1][0] - M[0][0] * M[1][2]) / det;
        inv[2][0] = (M[1][0] * M[2][1] - M[1][1] * M[2][0]) / det;
        inv[2][1] = (M[0][1] * M[2][0] - M[0][0] * M[2][1]) / det;
        inv[2][2] = (M[0][0] * M[1][1] - M[1][0] * M[0][1]) / det;
    }

    return inv;
}

/**
 *  Sets the temperature and tint using given chromaticity coordinates.
 *
 *  'Adobe DNG SDK 1.3.0.0': dng_sdk_1_3/dng_sdk/source/dng_temperature.cpp: 'dng_temperature::Set_xy_coord'.
 *
 *  Usage::
 *
 *      >>> Temperature().setChromaticityCoordinates(0.25, 0.25)
 *      (28847.66573353418, 2.0587866255277527)
 *
 *  Args:
 *      x (float): X chromaticity coordinate.
 *      y (float): Y chromaticity coordinate.
 *
 *  Returns:
 *      tuple. Temperature, tint
 */
void DCPProfile::setChromaticityCoordinates(const double x, const double y, double& cct, double& tint) const
{
    const double u = 2.0 * x / (1.5 - x + 6.0 * y);
    const double v = 3.0 * y / (1.5 - x + 6.0 * y);

    double lastDt = 0.f;
    double lastDv = 0.f;
    double lastDu = 0.f;

    for (int i = 1; i < 31; i++)
    {
        const std::vector<float> wrRuvt = WYSZECKI_ROBERSTON_TABLE[i];
        const std::vector<float> wrRuvtPrevious = WYSZECKI_ROBERSTON_TABLE[i - 1];

        double du = 1.f;
        double dv = wrRuvt[3];
        double len = std::sqrt(1.f + dv * dv);

        du /= len;
        dv /= len;
        double uu = u - wrRuvt[1];
        double vv = v - wrRuvt[2];
        double dt = -uu * dv + vv * du;

        if (dt <= 0.f || i == 30)
        {

            dt = -std::min<double>(dt, 0.f);

            const double f = (i == 1) ? 0.f : dt / (lastDt + dt);

            cct = 1.0e6 / (wrRuvtPrevious[0] * f + wrRuvt[0] * (1.f - f));

            const double uu = u - (wrRuvtPrevious[1] * f + wrRuvt[1] * (1.0 - f));
            const double vv = v - (wrRuvtPrevious[2] * f + wrRuvt[2] * (1.0 - f));

            du = du * (1.0 - f) + lastDu * f;
            dv = dv * (1.0 - f) + lastDv * f;
            len = std::sqrt(du * du + dv * dv);
            du /= len;
            dv /= len;

            tint = (uu * du + vv * dv) * 3000.f;

            break;
        }

        lastDt = dt;
        lastDu = du;
        lastDv = dv;
    }
}

/**
 * @brief Returns the chromaticity coordinates.
 *
 *  'Adobe DNG SDK 1.3.0.0': dng_sdk_1_3/dng_sdk/source/dng_temperature.cpp: 'dng_temperature::Get_xy_coord'.
 *
 *  Usage::
 *
 *      >>> Temperature(6500, 25).getChromaticityCoordinates()
 *      (0.3115291328160886, 0.33790935423997026)
 *
 *  Returns:
 *      tuple. Chromaticity coordinates.
*/
void DCPProfile::getChromaticityCoordinates(const double cct, const double tint, double& x, double& y) const
{
    const double r = 1.0e6 / cct;
    const double offset = tint * (1.0 / TINT_SCALE);

    for (int i = 0; i < 30; ++i)
    {
        std::vector<float> wrRuvt = WYSZECKI_ROBERSTON_TABLE[i];
        std::vector<float> wrRuvtNext = WYSZECKI_ROBERSTON_TABLE[i + 1];

        if (r < wrRuvtNext[0] || i == 29)
        {

            const double f = (wrRuvtNext[0] - r) / (wrRuvtNext[0] - wrRuvt[0]);

            double u = wrRuvt[1] * f + wrRuvtNext[1] * (1.0 - f);
            double v = wrRuvt[2] * f + wrRuvtNext[2] * (1.0 - f);

            double uu1 = 1.0;
            double uu2 = 1.0;
            double vv1 = wrRuvt[3];
            double vv2 = wrRuvtNext[3];

            double len1 = sqrt(1.0 + vv1 * vv1);
            double len2 = sqrt(1.0 + vv2 * vv2);

            uu1 /= len1;
            vv1 /= len1;

            uu2 /= len2;
            vv2 /= len2;

            double uu3 = uu1 * f + uu2 * (1.0 - f);
            double vv3 = vv1 * f + vv2 * (1.0 - f);

            double len3 = sqrt(uu3 * uu3 + vv3 * vv3);

            uu3 /= len3;
            vv3 /= len3;

            u += uu3 * offset;
            v += vv3 * offset;

            x = 1.5 * u / (u - 4.0 * v + 2.0);
            y = v / (u - 4.0 * v + 2.0);

            break;
        }
    }
}


void DCPProfile::getChromaticityCoordinatesFromXyz(const Triple& xyz, double& x, double& y) const
{
    x = xyz[0] / (xyz[0] + xyz[1] + xyz[2]);
    y = xyz[1] / (xyz[0] + xyz[1] + xyz[2]);
}

DCPProfile::Triple DCPProfile::getXyzFromChromaticityCoordinates(const double x, const double y) const
{
    Triple xyz;
    xyz[0] = x * 1.0 / y;
    xyz[1] = 1.0;
    xyz[2] = (1.0 - x - y) * 1.0 / y;
    return xyz;
}

DCPProfile::Triple DCPProfile::getXyzFromTemperature(const double cct, const double tint) const
{
    double x, y;
    getChromaticityCoordinates(cct, tint, x, y);
    Triple xyz = getXyzFromChromaticityCoordinates(x, y);
    return xyz;
}


/**
 * @brief Returns the chromaticity coordinates from 'As Shot Neutral' matrix.
 *
 * 'Adobe DNG SDK 1.3.0.0': dng_sdk_1_3 / dng_sdk / documents / dng_spec_1_3_0_0.pdf : 'Mapping Camera Color Space to CIE XYZ Space'.
 *
 * Usage::
 *
 * >> > getChromaticityCoordinatesFromCameraNeutral(2850, \
 *     6500, \
 *     numpy.matrix([0.5309, -0.0229, -0.0336, \
 *         - 0.6241, 1.3265, 0.3337, \
 *         - 0.0817, 0.1215, 0.6664]).reshape((3, 3)), \
 *     numpy.matrix([0.4716, 0.0603, -0.083, \
 *         - 0.7798, 1.5474, 0.248, \
 *         - 0.1496, 0.1937, 0.6651]).reshape((3, 3)), \
 *     numpy.matrix([0.9603, 0., 0., \
 *         0., 1., 0., \
 *         0., 0., 0.9664]).reshape((3, 3)), \
 *     numpy.matrix([0.9603, 0., 0., \
 *         0., 1., 0., \
 *         0., 0., 0.9664]).reshape((3, 3)), \
 *     numpy.identity(3), \
 *     numpy.matrix([0.305672, 1., 0.905393]).reshape((3, 1)))
 *     (0.27389027140925454, 0.28376817722941361)
 * 
 *     Args:
 * calibrationIlluminant1Cct(float) : Calibration Illuminant 1 correlated color temperature.
 * calibrationIlluminant2Cct(float) : Calibration Illuminant 2 correlated color temperature.
 * colorMatrix1(Matrix) : Color Matrix 1 matrix(3 x 3).
 * colorMatrix2(Matrix) : Color Matrix 2 matrix(3 x 3).
 * cameraCalibration1(Matrix) : Camera Calibration 1 matrix(3 x 3).
 * cameraCalibration2(Matrix) : Camera Calibration 2 matrix(3 x 3).
 * analogBalance(Matrix) : Analog Balance matrix(3 x 3).
 * asShotNeutral(Matrix) : As Shot Neutral matrix(3 x 3).
 *
 * Kwargs :
 *     verbose(bool) : Verbose value
 * 
 *     Returns :
 * tuple.Chromaticity coordinates.
 */
void DCPProfile::getChromaticityCoordinatesFromCameraNeutral(const Matrix& analogBalance, const Triple& asShotNeutral, double& x, double& y) const
{
    x = 0.24559589702841558;
    y = 0.24240399461846152;

    double residual = 1.0;
    int itNb = 0;

    while ((residual > 1e-7) && (itNb < 100))
    {
        double xPrevious = x;
        double yPrevious = y;

        double cct, tint;
        setChromaticityCoordinates(x, y, cct, tint);

        // Check that at least one color matrix is mandatory in a dcp
        Matrix interpolatedColorMatrix = IdentityMatrix;

        if (info.has_color_matrix_1 && info.has_color_matrix_2)
        {
            interpolatedColorMatrix = getInterpolatedMatrix(cct, "color");
        }
        else
        {
            interpolatedColorMatrix = info.has_color_matrix_1 ? color_matrix_1 : color_matrix_2;
        }

        Matrix interpolatedCalibMatrix = IdentityMatrix;

        if (info.has_camera_calibration_1 && info.has_camera_calibration_2)
        {
            interpolatedCalibMatrix = getInterpolatedMatrix(cct, "calib");
        }
        else if (info.has_camera_calibration_1)
        {
            interpolatedCalibMatrix = camera_calibration_1;
        }

        Matrix xyzToCamera = matMult(analogBalance, matMult(interpolatedCalibMatrix, interpolatedColorMatrix));
        Triple xyz = matMult(matInv(xyzToCamera), asShotNeutral);

        getChromaticityCoordinatesFromXyz(xyz, x, y);

        residual = std::abs(x - xPrevious) + std::abs(y - yPrevious);

        itNb++;
    }
}

/**
 * @brief Returns the 'Chromatic Adaptation' matrix using given 'XYZ' source and target matrices.
 * 
 *     'http://brucelindbloom.com/index.html?Eqn_ChromAdapt.html'
 * 
 *     Usage::
 * 
 *         >>> getChromaticAdaptationMatrix(numpy.matrix([1.09923822, 1.000, 0.35445412]).reshape((3, 1)), \
 *         numpy.matrix([0.96907232, 1.000, 1.121792157]).reshape((3, 1)))
 *         matrix([[ 0.87145615, -0.13204674,  0.40394832],
 *                 [-0.09638805,  1.04909781,  0.1604033 ],
 *                 [ 0.0080207 ,  0.02826367,  3.06023196]])
 *
 * 
 *     Args:
 *         xyzSource (Matrix): XYZ source matrix ( 3 x 1 ).
 *         xyzTarget (Matrix): XYZ target matrix ( 3 x 1 ).
 * 
 *     Kwargs:
 *         verbose (bool):	Verbose value
 * 
 *     Returns:
 *         Matrix. Chromatic Adaptation matrix ( 3 x 3 ).
 */
DCPProfile::Matrix DCPProfile::getChromaticAdaptationMatrix(const Triple& xyzSource, const Triple& xyzTarget) const
{
    const Triple pybSource = matMult(CAT02_MATRIX, xyzSource);
    const Triple pybTarget = matMult(CAT02_MATRIX, xyzTarget);

    Matrix crd = IdentityMatrix;
    crd[0][0] = pybTarget[0] / pybSource[0];
    crd[1][1] = pybTarget[1] / pybSource[1];
    crd[2][2] = pybTarget[2] / pybSource[2];

    const Matrix cat = matMult(matMult(matInv(CAT02_MATRIX), crd), CAT02_MATRIX);

    return cat;
}

/**
 * @brief Returns the 'camera to XYZ ( D50 )' matrix.
 *     'colorMatrix1', 'colorMatrix2', 'forwardMatrix1' and 'forwardMatrix2' are considered as non existing if given as
 *  identity matrices.
 * 
 *     'Adobe DNG SDK 1.3.0.0': dng_sdk_1_3/dng_sdk/documents/dng_spec_1_3_0_0.pdf: 'Mapping Camera Color Space to CIE
 *  XYZ Space'.
 * 
 *     Usage::
 * 
 *         >>> getCameraToXyzD50Matrix(0.24559589702841558, \
 *                                  0.24240399461846152, \
 *                                  2850, \
 *                                  6500, \
 *                                  numpy.matrix([0.5309, -0.0229, -0.0336, \
 *                                        -0.6241, 1.3265, 0.3337, \
 *                                        -0.0817, 0.1215, 0.6664]).reshape((3, 3)), \
 *                                  numpy.matrix([0.4716, 0.0603, -0.083, \
 *                                        -0.7798, 1.5474, 0.248, \
 *                                        -0.1496, 0.1937, 0.6651]).reshape((3, 3)), \
 *                                  numpy.matrix([0.9603, 0., 0., \
 *                                              0., 1., 0., \
 *                                              0., 0., 0.9664]).reshape((3, 3)), \
 *                                  numpy.matrix([0.9603, 0., 0., \
 *                                              0., 1., 0., \
 *                                              0., 0., 0.9664]).reshape((3, 3)), \
 *                                  numpy.identity(3), \
 *                                  numpy.matrix([0.8924, -0.1041, 0.176, \
 *                                          0.4351, 0.6621, -0.0972, \
 *                                          0.0505, -0.1562, 0.9308]).reshape((3, 3)), \
 *                                  numpy.matrix([0.8924, -0.1041, 0.176, \
 *                                          0.4351, 0.6621, -0.0972, \
 *                                          0.0505, -0.1562, 0.9308]).reshape((3, 3)))
 *         matrix([[ 3.48428482, -0.1041    ,  0.14605003],
 *                 [ 1.69880359,  0.6621    , -0.08065945],
 *                 [ 0.1971721 , -0.1562    ,  0.77240552]])
 * 
 *     Args:
 *         x (float): X chromaticity coordinate
 *         y (float): Y chromaticity coordinate
 *         calibrationIlluminant1Cct (float): Calibration Illuminant 1 correlated color temperature.
 *         calibrationIlluminant2Cct (float): Calibration Illuminant 2 correlated color temperature.
 *         colorMatrix1 (Matrix): Color Matrix 1 matrix ( 3 x 3 ).
 *         colorMatrix2 (Matrix): Color Matrix 2 matrix ( 3 x 3 ).
 *         cameraCalibration1 (Matrix): Camera Calibration 1 matrix ( 3 x 3 ).
 *         cameraCalibration2 (Matrix): Camera Calibration 2 matrix ( 3 x 3 ).
 *         analogBalance (Matrix): Analog Balance matrix ( 3 x 3 ).
 *         forwardMatrix1 (Matrix): Forward Matrix 1 matrix ( 3 x 3 ).
 *         forwardMatrix2 (Matrix): Forward Matrix 2 matrix ( 3 x 3 ).
 * 
 *     Kwargs:
 *         verbose (bool): Verbose value
 * 
 *     Returns:
 *         Matrix.  Camera to XYZ ( D50 ) matrix ( 3 x 3 ).
 */
DCPProfile::Matrix DCPProfile::getCameraToXyzD50Matrix(const double x, const double y) const
{
    double cct, tint;
    setChromaticityCoordinates(x, y, cct, tint);
    const Triple xyz = {
        x * 1.f / y,
        1.f,
        (1.f - x - y) * 1.f / y};

    Matrix interpolatedColorMatrix;
    if (info.has_color_matrix_1 && info.has_color_matrix_2)
    {
        interpolatedColorMatrix = getInterpolatedMatrix(cct, "color");
    }
    else
    {
        interpolatedColorMatrix = info.has_color_matrix_1 ? color_matrix_1 : color_matrix_2;
    }
    const Matrix interpolatedCalibMatrix = getInterpolatedMatrix(cct, "calib");

    const Triple rgb = matMult(interpolatedColorMatrix, xyz);
    const Triple asShotNeutral = {
        rgb[0] / rgb[1],
        rgb[1] / rgb[1],
        rgb[2] / rgb[1]};

    const Triple referenceNeutral = matMult(matInv(matMult(analogBalance, interpolatedCalibMatrix)), asShotNeutral);

    Matrix d = IdentityMatrix;
    d[0][0] = 1.0 / referenceNeutral[0];
    d[1][1] = 1.0 / referenceNeutral[1];
    d[2][2] = 1.0 / referenceNeutral[2];

    Matrix cameraToXyzD50;

    if ((!info.has_forward_matrix_1) && (!info.has_forward_matrix_2))
    {
        Matrix interpolatedForwardMatrix = IdentityMatrix;

        const Matrix xyzToCamera = matMult(analogBalance, matMult(interpolatedCalibMatrix, interpolatedColorMatrix));
        const Matrix cameraToXyz = matInv(xyzToCamera);

        const double D50_cct = 5000.706605070579;  //
        const double D50_tint = 9.562965495510433; // Using x, y = 0.3457, 0.3585
        const Matrix cat = getChromaticAdaptationMatrix(getXyzFromChromaticityCoordinates(x, y), getXyzFromTemperature(D50_cct, D50_tint));
        cameraToXyzD50 = matMult(cat, cameraToXyz);
    }
    else if ((!info.has_forward_matrix_1) || (!info.has_forward_matrix_2))
    {
        const Matrix interpolatedForwardMatrix = info.has_forward_matrix_2 ? forward_matrix_2 : forward_matrix_1;
        cameraToXyzD50 = matMult(interpolatedForwardMatrix, matMult(d, matInv(matMult(analogBalance, interpolatedCalibMatrix))));
    }
    else
    {
        const Matrix interpolatedForwardMatrix = getInterpolatedMatrix(cct, "forward");
        cameraToXyzD50 = matMult(interpolatedForwardMatrix, matMult(d, matInv(matMult(analogBalance, interpolatedCalibMatrix))));
    }

    return cameraToXyzD50;
}

DCPProfile::Matrix DCPProfile::getCameraToSrgbLinearMatrix(const double x, const double y) const
{
    double cct, tint;
    setChromaticityCoordinates(x, y, cct, tint);
    const Triple xyz = {
        x * 1.f / y,
        1.f,
        (1.f - x - y) * 1.f / y };

    Matrix interpolatedColorMatrix;
    if (info.has_color_matrix_1 && info.has_color_matrix_2)
    {
        interpolatedColorMatrix = getInterpolatedMatrix(cct, "color");
    }
    else
    {
        interpolatedColorMatrix = info.has_color_matrix_1 ? color_matrix_1 : color_matrix_2;
    }

    Matrix cameraToXyzD50;

    if ((!info.has_forward_matrix_1) && (!info.has_forward_matrix_2))
    {
        const Matrix interpolatedForwardMatrix = IdentityMatrix;

        const Matrix xyzToCamera = interpolatedColorMatrix;
        const Matrix cameraToXyz = matInv(xyzToCamera);

        const double D50_cct = 5000.706605070579;  //
        const double D50_tint = 9.562965495510433; // Using x, y = 0.3457, 0.3585
        const Matrix cat = getChromaticAdaptationMatrix(getXyzFromChromaticityCoordinates(x, y), getXyzFromTemperature(D50_cct, D50_tint));
        cameraToXyzD50 = matMult(cat, cameraToXyz);
    }
    else if ((!info.has_forward_matrix_1) || (!info.has_forward_matrix_2))
    {
        const Matrix interpolatedForwardMatrix = info.has_forward_matrix_2 ? forward_matrix_2 : forward_matrix_1;
        cameraToXyzD50 = interpolatedForwardMatrix;
    }
    else
    {
        const Matrix interpolatedForwardMatrix = getInterpolatedMatrix(cct, "forward");
        cameraToXyzD50 = interpolatedForwardMatrix;
    }

    const Matrix cameraToSrgbLinear = matMult(xyzD50ToSrgbD65LinearMatrix, cameraToXyzD50);

    return cameraToSrgbLinear;
}

DCPProfile::Matrix DCPProfile::getCameraToACES2065Matrix(const Triple& asShotNeutral, const bool sourceIsRaw, const bool useColorMatrixOnly) const
{
    const Triple asShotNeutralInv = { 1.0 / asShotNeutral[0] , 1.0 / asShotNeutral[1] , 1.0 / asShotNeutral[2] };

    double x, y;
    getChromaticityCoordinatesFromCameraNeutral(IdentityMatrix, asShotNeutralInv, x, y);
    double cct, tint;
    setChromaticityCoordinates(x, y, cct, tint);

    ALICEVISION_LOG_INFO("Estimated illuminant (cct; tint) : (" << cct << "; " << tint << ")");

    Matrix neutral = IdentityMatrix;

    if (sourceIsRaw)
    {
        neutral[0][0] = asShotNeutral[0];
        neutral[1][1] = asShotNeutral[1];
        neutral[2][2] = asShotNeutral[2];
    }

    Matrix cameraToXyzD50 = IdentityMatrix;

    if (useColorMatrixOnly || ((!info.has_forward_matrix_1) && (!info.has_forward_matrix_2)))
    {
        Matrix xyzToCamera = IdentityMatrix;
        if (info.has_color_matrix_1 && info.has_color_matrix_2)
        {
            xyzToCamera = getInterpolatedMatrix(cct, "color");
        }
        else if (info.has_color_matrix_1)
        {
            xyzToCamera = color_matrix_1;
        }
        const Matrix cameraToXyz = matInv(xyzToCamera);

        const double D50_cct = 5000.706605070579;  //
        const double D50_tint = 9.562965495510433; // Using x, y = 0.3457, 0.3585
        const Matrix cat = getChromaticAdaptationMatrix(getXyzFromChromaticityCoordinates(x, y), getXyzFromTemperature(D50_cct, D50_tint));
        cameraToXyzD50 = matMult(cat, cameraToXyz);
    }
    else if ((info.has_forward_matrix_1) && (info.has_forward_matrix_2))
    {
        cameraToXyzD50 = matMult(getInterpolatedMatrix(cct, "forward"), neutral);
    }
    else if (info.has_forward_matrix_1)
    {
        cameraToXyzD50 = matMult(forward_matrix_1, neutral);
    }
    Matrix cameraToACES2065 = matMult(xyzD50ToACES2065Matrix, cameraToXyzD50);

    return cameraToACES2065;
}


void DCPProfile::getMatrices(const std::string& type, std::vector<Matrix>& v_Mat) const
{
    v_Mat.clear();

    if (type == "color")
    {
        if (info.has_color_matrix_1)
            v_Mat.push_back(color_matrix_1);
        if (info.has_color_matrix_2)
            v_Mat.push_back(color_matrix_2);
    }
    else if (type == "forward")
    {
        if (info.has_forward_matrix_1)
            v_Mat.push_back(forward_matrix_1);
        if (info.has_forward_matrix_2)
            v_Mat.push_back(forward_matrix_2);
    }
    else if (type == "calib")
    {
        if (info.has_camera_calibration_1)
            v_Mat.push_back(camera_calibration_1);
        if (info.has_camera_calibration_2)
            v_Mat.push_back(camera_calibration_2);
    }
}

void DCPProfile::getMatricesAsStrings(const std::string& type, std::vector<std::string>& v_strMat) const
{
    v_strMat.clear();
    std::vector<Matrix> v_Mat;
    getMatrices(type, v_Mat);

    for (const auto &mat : v_Mat)
    {
        std::string strMat = "";
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                strMat += (std::to_string(mat[i][j]) + " ");
        v_strMat.push_back(strMat);
    }
}

void DCPProfile::setMatrices(const std::string& type, std::vector<Matrix>& v_Mat)
{
    if (type == "forward")
    {
        info.has_forward_matrix_1 = false;
        info.has_forward_matrix_2 = false;
        if (v_Mat.size() > 0)
        {
            info.has_forward_matrix_1 = true;
            forward_matrix_1 = v_Mat[0];
            if (v_Mat.size() > 1)
            {
                info.has_forward_matrix_2 = true;
                forward_matrix_2 = v_Mat[1];
            }
        }
    }
    else if (type == "color")
    {
        info.has_color_matrix_1 = false;
        info.has_color_matrix_2 = false;
        if (v_Mat.size() > 0)
        {
            info.has_color_matrix_1 = true;
            color_matrix_1 = v_Mat[0];
            if (v_Mat.size() > 1)
            {
                info.has_color_matrix_2 = true;
                color_matrix_2 = v_Mat[1];
            }
        }
    }
    else if (type == "calib")
    {
        info.has_camera_calibration_1 = false;
        info.has_camera_calibration_2 = false;
        if (v_Mat.size() > 0)
        {
            info.has_camera_calibration_1 = true;
            camera_calibration_1 = v_Mat[0];
            if (v_Mat.size() > 1)
            {
                info.has_camera_calibration_2 = true;
                camera_calibration_2 = v_Mat[1];
            }
        }
    }
}

void DCPProfile::setMatricesFromStrings(const std::string& type, std::vector<std::string>& v_strMat)
{
    std::vector<Matrix> v_Mat;

    for (const std::string& strMat : v_strMat)
    {
        std::vector<std::string> v;
        boost::algorithm::split(v, strMat, boost::algorithm::is_space());
        Matrix mat;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                mat[i][j] = std::stof(v[3*i + j]);
        v_Mat.push_back(mat);
    }

    setMatrices(type, v_Mat);
}

void DCPProfile::applyLinear(OIIO::ImageBuf& image, const Triple& neutral, const bool sourceIsRaw, const bool useColorMatrixOnly) const
{
    const Matrix cameraToACES2065Matrix = getCameraToACES2065Matrix(neutral, sourceIsRaw, useColorMatrixOnly);

    ALICEVISION_LOG_INFO("cameraToACES2065Matrix : " << cameraToACES2065Matrix);

    #pragma omp parallel for
    for (int i = 0; i < image.spec().height; ++i)
        for (int j = 0; j < image.spec().width; ++j)
        {
            float rgb[3];
            image.getpixel(j, i, rgb, 3);

            float rgbOut[3];
            for (int r = 0; r < 3; ++r)
            {
                rgbOut[r] = 0.0;
                for (int c = 0; c < 3; ++c)
                {
                    rgbOut[r] += cameraToACES2065Matrix[r][c] * rgb[c];
                }
            }
            image.setpixel(j, i, rgbOut, 3);
        }
}

void DCPProfile::applyLinear(Image<image::RGBAfColor>& image, const Triple& neutral, const bool sourceIsRaw) const
{
    const Matrix cameraToACES2065Matrix = getCameraToACES2065Matrix(neutral, sourceIsRaw);

    #pragma omp parallel for
    for (int i = 0; i < image.Height(); ++i)
        for (int j = 0; j < image.Width(); ++j)
        {
            const RGBAfColor rgb = image(i, j);

            RGBAfColor rgbOut = rgb;
            for (int r = 0; r < 3; ++r)
            {
                rgbOut[r] = 0.0;
                for (int c = 0; c < 3; ++c)
                {
                    rgbOut[r] += cameraToACES2065Matrix[r][c] * rgb[c];
                }
            }
            image(i, j) = rgbOut;
        }
}

DCPDatabase::DCPDatabase(const std::string& databaseDirPath)
{
    load(databaseDirPath, true);
}

int DCPDatabase::load(const std::string& databaseDirPath, bool force)
{
    if (!databaseDirPath.compare(folderName) && !force)
    {
        return dcpFilenamesList.size();
    }

    clear();

    folderName = databaseDirPath;

    if (!bfs::is_directory(databaseDirPath))
    {
        return 0;
    }

    bfs::path targetDir(databaseDirPath);
    bfs::directory_iterator it(targetDir), eod;
    BOOST_FOREACH(bfs::path const& p, std::make_pair(it, eod))
    {
        if (bfs::is_regular_file(p))
        {
            dcpFilenamesList.emplace_back(p.generic_string());
        }
    }

    return dcpFilenamesList.size();
}

void DCPDatabase::clear()
{
    dcpFilenamesList.clear();
    dcpStore.clear();
    folderName = "";
}

bool DCPDatabase::retrieveDcpForCamera(const std::string& make, const std::string& model, DCPProfile& dcpProf)
{
    const std::string dcpKey = make + "_" + model;

    {
        // Retrieve preloaded DCPProfile
        std::map<std::string, image::DCPProfile>::iterator it = dcpStore.find(dcpKey);
        if (it != dcpStore.end())
        {
            dcpProf = it->second;
            return true;
        }
    }

    {
        // Load DCPProfile from disk
        const std::vector<std::string>::iterator it = std::find_if(dcpFilenamesList.begin(), dcpFilenamesList.end(), [make, model](const std::string& s)
            { return (s.find(make) != std::string::npos) && (s.find(model) != std::string::npos); });

        if (it != dcpFilenamesList.end())
        {
            dcpProf.Load(*it);
            dcpStore.insert(std::pair<std::string, image::DCPProfile>(dcpKey, dcpProf));
            return true;
        }
    }

    return false;
}

void DCPDatabase::add_or_replace(DCPProfile& dcpProf, const std::string& make, const std::string& model)
{
    const std::string dcpKey = make + "_" + model;

    std::map<std::string, image::DCPProfile>::iterator it = dcpStore.find(dcpKey);
    if (it != dcpStore.end())
    {
        dcpStore[dcpKey] = dcpProf;
        std::vector<std::string>::iterator it = find(dcpFilenamesList.begin(), dcpFilenamesList.end(), dcpProf.info.filename);
        if (it != dcpFilenamesList.end())
        {
            dcpFilenamesList[it - dcpFilenamesList.begin()] = dcpProf.info.filename;
        }
    }
    else
    {
        dcpStore.insert(std::pair<std::string, image::DCPProfile>(dcpKey, dcpProf));
        dcpFilenamesList.push_back(dcpProf.info.filename);
    }
}

} // namespace image
} // namespace aliceVision




