#include <iostream>
#include <cstdio>
#include <cstring>
#include <functional>

#include <aliceVision/system/Logger.hpp>
#include "dcp.hpp"

using namespace alicevision::image;

namespace
{
double calibrationIlluminantToTemperature(int light)
{
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

    // These temperatures are those found in DNG SDK reference code
    switch(LightSource(light))
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

            if (fabsf(del_Max) < 0.00001f)
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
    COLOR_MATRIX_1 = 50721,
    COLOR_MATRIX_2 = 50722,
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

/*
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
        unsigned int i = 0;

        for(i = 0; i + ofs < datasize && i < 64 && v_value[i + ofs]; i++)
            if(v_value[i + ofs] < 32 || v_value[i + ofs] > 126)
            {
                isstring = false;
            }

        if(isstring)
        {
            if(size < 3)
            {
                return;
            }

            std::size_t j = 0;

            for(i = 0; i + ofs < datasize && i < 64 && v_value[i + ofs]; i++)
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

Tag* findTag(TagKey tagID, std::vector<Tag>& v_Tags)
{
    size_t idx = 0;
    while(idx < v_Tags.size() && (v_Tags[idx].tagID != (unsigned short)tagID))
    {
        idx++;
    }
    return (idx == v_Tags.size()) ? nullptr : &v_Tags[idx];
}

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

        float t = float(i) / 65535.f;
        while((v_x[k_hi] <= t) && (k_hi < v_x.size() - 1))
            k_hi++;

        int k_lo = k_hi - 1;
        double h = v_x[k_hi] - v_x[k_lo];
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
    float r = std::max<float>(0.0, std::min<float>(65535.0, ir));
    float g = std::max<float>(0.0, std::min<float>(65535.0, ig));
    float b = std::max<float>(0.0, std::min<float>(65535.0, ib));

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

DCPProfile::DCPProfile(const std::string filename)
    : will_interpolate(false)
    , valid(false)
    , baseline_exposure_offset(0.0)
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
        0.99987f, 1.00000f};

    FILE* const file = fopen(filename.c_str(), "rb");

    if(file == nullptr)
    {
        ALICEVISION_LOG_WARNING("Unable to load DCP profile " << filename);
        return;
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

    if(numOfTags <= 0 || numOfTags > 1000)
    {
        ALICEVISION_LOG_WARNING("DCP Error: Tag number out of range.");
        fclose(file);
        return;
    }

    std::vector<Tag> v_tag;

    for(int i = 0; i < numOfTags; i++)
    {

        unsigned short tag = get2(file, order);
        TagType type = (TagType)get2(file, order);
        unsigned int datasize = get4(file, order);

        if(!datasize)
        {
            datasize = 1;
        }

        // filter out invalid tags
        // note the large count is to be able to pass LeafData ASCII tag which can be up to almost 10 megabytes,
        // (only a small part of it will actually be parsed though)
        if((int)type < 1 || (int)type > 14 || datasize > 10 * 1024 * 1024)
        {
            type = TagType::T_INVALID;
            fclose(file);
            ALICEVISION_LOG_WARNING("ERROR : Invalid Tag in dcp file !");
            return;
        }

        // store next Tag's position in file
        int save = ftell(file) + 4;

        Tag t(tag, type, datasize, order, file);

        v_tag.push_back(t);

        fseek(file, save, SEEK_SET);
    }

    fclose(file);

    Tag* tag = findTag(TagKey::CALIBRATION_ILLUMINANT_1, v_tag);
    info.light_source_1 = tag ? tag->toInt(0, TagType::T_SHORT) : -1;
    tag = findTag(TagKey::CALIBRATION_ILLUMINANT_2, v_tag);
    info.light_source_2 = tag ? tag->toInt(0, TagType::T_SHORT) : -1;
    info.temperature_1 = calibrationIlluminantToTemperature(info.light_source_1);
    info.temperature_2 = calibrationIlluminantToTemperature(info.light_source_2);

    const bool has_second_hue_sat =
        findTag(TagKey::PROFILE_HUE_SAT_MAP_DATA_2, v_tag); // Some profiles have two matrices, but just one huesat

    // Fetch Forward Matrices, if any
    tag = findTag(TagKey::FORWARD_MATRIX_1, v_tag);

    if(tag)
    {
        info.has_forward_matrix_1 = true;

        for(int row = 0; row < 3; ++row)
        {
            for(int col = 0; col < 3; ++col)
            {
                forward_matrix_1[row][col] = tag->toDouble((col + row * 3) * 8);
            }
        }
    }

    tag = findTag(TagKey::FORWARD_MATRIX_2, v_tag);

    if(tag)
    {
        info.has_forward_matrix_2 = true;

        for(int row = 0; row < 3; ++row)
        {
            for(int col = 0; col < 3; ++col)
            {
                forward_matrix_2[row][col] = tag->toDouble((col + row * 3) * 8);
            }
        }
    }

    // Color Matrix (one is always there)
    tag = findTag(TagKey::COLOR_MATRIX_1, v_tag);

    if(!tag)
    {
        ALICEVISION_LOG_WARNING("DCP '" << filename << "' is missing 'ColorMatrix1'. Skipped.");
        fclose(file);
        return;
    }

    info.has_color_matrix_1 = true;

    for(int row = 0; row < 3; ++row)
    {
        for(int col = 0; col < 3; ++col)
        {
            color_matrix_1[row][col] = tag->toDouble((col + row * 3) * 8);
        }
    }

    tag = findTag(TagKey::PROFILE_LOOK_TABLE_DIMS, v_tag);

    if(tag)
    {
        look_info.hue_divisions = tag->toInt(0);
        look_info.sat_divisions = tag->toInt(4);
        look_info.val_divisions = tag->toInt(8);

        tag = findTag(TagKey::PROFILE_LOOK_TABLE_ENCODING, v_tag);
        look_info.srgb_gamma = tag && tag->toInt(0);

        tag = findTag(TagKey::PROFILE_LOOK_TABLE_DATA, v_tag);
        look_info.array_count = tag->datasize / 3;

        look_table.resize(look_info.array_count);

        for(unsigned int i = 0; i < look_info.array_count; i++)
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

    if(tag)
    {
        delta_info.hue_divisions = tag->toInt(0);
        delta_info.sat_divisions = tag->toInt(4);
        delta_info.val_divisions = tag->toInt(8);

        tag = findTag(TagKey::PROFILE_HUE_SAT_MAP_ENCODING, v_tag);
        delta_info.srgb_gamma = tag && tag->toInt(0);

        tag = findTag(TagKey::PROFILE_HUE_SAT_MAP_DATA_1, v_tag);
        delta_info.array_count = tag->datasize / 3;

        deltas_1.resize(delta_info.array_count);

        for(unsigned int i = 0; i < delta_info.array_count; ++i)
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

    if(info.light_source_2 != -1)
    {
        // Second matrix
        info.has_color_matrix_2 = true;

        tag = findTag(TagKey::COLOR_MATRIX_2, v_tag);

        for(int row = 0; row < 3; ++row)
        {
            for(int col = 0; col < 3; ++col)
            {
                color_matrix_2[row][col] = tag ? tag->toDouble((col + row * 3) * 8) : color_matrix_1[row][col];
            }
        }

        // Second huesatmap
        if(has_second_hue_sat)
        {
            deltas_2.resize(delta_info.array_count);

            // Saturation maps. Need to be unwinded.
            tag = findTag(TagKey::PROFILE_HUE_SAT_MAP_DATA_2, v_tag);

            for(unsigned int i = 0; i < delta_info.array_count; ++i)
            {
                deltas_2[i].hue_shift = tag->toDouble((i * 3) * tiff_float_size);
                deltas_2[i].sat_scale = tag->toDouble((i * 3 + 1) * tiff_float_size);
                deltas_2[i].val_scale = tag->toDouble((i * 3 + 2) * tiff_float_size);
            }
        }
    }

    tag = findTag(TagKey::BASELINE_EXPOSURE_OFFSET, v_tag);

    if(tag)
    {
        info.has_baseline_exposure_offset = true;
        baseline_exposure_offset = tag->toDouble();
    }

    // Read tone curve points, if any, but disable to RTs own profiles
    tag = findTag(TagKey::PROFILE_TONE_CURVE, v_tag);

    if(tag)
    {
        std::vector<double> AS_curve_points;

        // Push back each X/Y coordinates in a loop
        bool curve_is_linear = true;

        for(int i = 0; i < tag->datasize; i += 2)
        {
            const double x = tag->toDouble((i + 0) * tiff_float_size);
            const double y = tag->toDouble((i + 1) * tiff_float_size);

            if(x != y)
            {
                curve_is_linear = false;
            }

            AS_curve_points.push_back(x);
            AS_curve_points.push_back(y);
        }

        if(!curve_is_linear)
        {
            // Create the curve
            info.has_tone_curve = true;
            AS_tone_curve.Set(AS_curve_points);
        }
    }
    else
    {
        tag = findTag(TagKey::PROFILE_TONE_COPYRIGHT, v_tag);

        if(tag && tag->valueToString().find("Adobe Systems") != std::string::npos)
        {
            // An Adobe profile without tone curve is expected to have the Adobe Default Curve
            std::vector<double> AS_curve_points;

            constexpr size_t tc_len =
                sizeof(adobe_camera_raw_default_curve) / sizeof(adobe_camera_raw_default_curve[0]);

            for(size_t i = 0; i < tc_len; ++i)
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

    will_interpolate = false;

    if(info.has_forward_matrix_1)
    {
        if(info.has_forward_matrix_2)
        {
            if(forward_matrix_1 != forward_matrix_2)
            {
                // Common that forward matrices are the same!
                will_interpolate = true;
            }

            if(!deltas_1.empty() && !deltas_2.empty())
            {
                // We assume tables are different
                will_interpolate = true;
            }
        }
    }

    if(info.has_color_matrix_1 && info.has_color_matrix_2)
    {
        if(color_matrix_1 != color_matrix_2)
        {
            will_interpolate = true;
        }

        if(!deltas_1.empty() && !deltas_2.empty())
        {
            will_interpolate = true;
        }
    }

    if(file)
    {
        fclose(file);
    }

    valid = true;

    std::vector<double> gammatab_srgb_data;
    std::vector<double> igammatab_srgb_data;
    for(int i = 0; i < 65536; i++)
    {
        double x = i / 65535.0;
        gammatab_srgb_data.push_back((x <= 0.003040) ? (x * 12.92310) : (1.055 * exp(log(x) / 2.4) - 0.055)); // from RT
        igammatab_srgb_data.push_back((x <= 0.039286) ? (x / 12.92310)
                                                      : (exp(log((x + 0.055) / 1.055) * 2.4))); // from RT
    }
    gammatab_srgb.Set(gammatab_srgb_data);
    igammatab_srgb.Set(igammatab_srgb_data);
}

DCPProfile::~DCPProfile() = default;

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
#define CLIPF_0_65535(a) ((a) > 0.f ? ((a) < 65535.5f ? (a) : 65535.5f) : 0.f)
#define CLIPF_0_1(a) ((a) > 0 ? ((a) < 1 ? (a) : 1) : 0)

    const float exp_scale = (params.apply_baseline_exposure_offset && info.has_baseline_exposure_offset)
        ? powf(2.0, baseline_exposure_offset)
        : 1.0;

    if (!params.use_tone_curve && !params.apply_look_table)
    {
        if (exp_scale == 1.f)
        {
            return;
        }

        for (int c = 0; c < 3; c++)
        {
            rgb[c] *= exp_scale;
        }
    }
    else
    {

        for (int c = 0; c < 3; c++)
        {
            rgb[c] *= exp_scale;
        }

        float ws_rgb[3] = { 0.0, 0.0, 0.0 };
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
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
            for (int i = 0; i < 3; i++)
            {
                clipped_ws_rgb[i] = CLIPF_0_65535(ws_rgb[i]);
            }

            float h, s, v;
            rgb2hsvtc(clipped_ws_rgb[0], clipped_ws_rgb[1], clipped_ws_rgb[2], h, s, v);

            hsdApply(look_info, look_table, h, s, v);
            s = CLIPF_0_1(s);
            v = CLIPF_0_1(v);

            h += (h < 0.0f ? 6.0f : (h >= 6.0f ? -6.0f : 0.0f));

            hsv2rgbdcp(h, s, v, clipped_ws_rgb[0], clipped_ws_rgb[1], clipped_ws_rgb[2]);

            if (!(ws_rgb[0] < 0.0 || ws_rgb[0] > 65535.0) || !(ws_rgb[1] < 0.0 || ws_rgb[1] > 65535.0) ||
                !(ws_rgb[2] < 0.0 || ws_rgb[2] > 65535.0))
            {
                ws_rgb[0] = clipped_ws_rgb[0];
                ws_rgb[1] = clipped_ws_rgb[1];
                ws_rgb[2] = clipped_ws_rgb[2];
            }
        }

        if (params.use_tone_curve)
        {
            // tone_curve.Apply(ws_rgb[0], ws_rgb[1], ws_rgb[2]);
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


