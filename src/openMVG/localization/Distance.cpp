#include "KDTree.h"
#include <stdint.h>
#include <algorithm>
#include <Eigen/Core>
#include <vector>
#include <tuple>
#include <random>

namespace popsift {
namespace kdtree {

static_assert(SPLIT_DIMENSION_COUNT < 128, "Invalid split dimension count");

static unsigned L1Distance_AVX2(const U8Descriptor& ad, const U8Descriptor& bd)
{
    const __m256i* af = ad.features;
    const __m256i* bf = bd.features;
    __m256i acc = _mm256_setzero_si256();

    // 32 components per iteration.
    for (int i = 0; i < 4; ++i) {
        __m256i d = _mm256_sad_epu8(
            _mm256_load_si256(af + i),
            _mm256_load_si256(bf + i));
        acc = _mm256_add_epi64(acc, d);
    }

    ALIGNED32 uint64_t buf[4];
    _mm256_store_si256((__m256i*)buf, acc);
    unsigned int sum = buf[0] + buf[1] + buf[2] + buf[3];
    return sum;
}

static unsigned L1Distance_scalar(const U8Descriptor& ad, const U8Descriptor& bd) {
    unsigned sum = 0;
    for (int i = 0; i < 128; i++) {
        sum += abs((int)ad.ufeatures[i] - (int)bd.ufeatures[i]);
    }
    return sum;
}

// Returns the least possible distance between descriptor and BB.  The code uses the following:
// ----X1----m----X2-------M-----X3--- ; bb.min <= bb.max (componentwise)
// We observe that 
//                 / 0, when x >= m                         / 0, when x <= M
// m - min(m,X) = |                        max(M,x) - x = |
//                 \ m-x, when x < m                        \ x-M when x > M
//
static unsigned L1Distance_AVX2(const U8Descriptor& d, const BoundingBox& bb)
{
    __m256i acc = _mm256_setzero_si256();

    for (int i = 0; i < 4; ++i) {
        __m256i d1 = _mm256_sad_epu8(bb.min.features[i], _mm256_min_epu8(bb.min.features[i], d.features[i]));
        __m256i d2 = _mm256_sad_epu8(bb.max.features[i], _mm256_max_epu8(bb.max.features[i], d.features[i]));
        acc = _mm256_add_epi64(acc, _mm256_add_epi64(d1, d2));
    }

    ALIGNED32 uint64_t buf[4];
    _mm256_store_si256((__m256i*)buf, acc);
    unsigned int sum = buf[0] + buf[1] + buf[2] + buf[3];
    return sum;
}

static unsigned L1Distance_scalar(const U8Descriptor& d, const BoundingBox& bb)  {
    unsigned sum = 0;
    for (int i = 0; i < 128; i++) {
        if (d.ufeatures[i] < bb.min.ufeatures[i]) {
            sum += bb.min.ufeatures[i] - d.ufeatures[i];
        }
        else if (d.ufeatures[i] > bb.max.ufeatures[i]) {
            sum += d.ufeatures[i] - bb.max.ufeatures[i];
        }
    }
    return sum;
}

static inline unsigned reduce(__m256i reg)
{
    __m128i l = _mm256_extracti128_si256(reg, 0);
    __m128i h = _mm256_extracti128_si256(reg, 1);
    __m128i r = _mm_hadd_epi32(_mm_add_epi32(h, l), _mm_setzero_si128());
    return _mm_extract_epi32(r, 0) + _mm_extract_epi32(r, 1);
}

// AVX2 implementation for U8 descriptors.
// 128 components fit in 4 AVX2 registers.  Must expand components from 8-bit
// to 16-bit in order to do arithmetic without overflow. Also, AVX2 doesn't
// support vector multiplication of 8-bit elements.
static unsigned L2DistanceSquared_AVX2(const U8Descriptor& ad, const U8Descriptor& bd)
{
    __m256i acc = _mm256_setzero_si256();

    // 32 components per iteration.
    for (int i = 0; i < 4; ++i) {
        // Must avoid overflow. Max value after squaring is 65025.
        __m256i min = _mm256_min_epu8(ad.features[i], bd.features[i]);
        __m256i max = _mm256_max_epu8(ad.features[i], bd.features[i]);
        __m256i d = _mm256_sub_epi8(max, min);

        // Squared elements, 0..15
        __m256i dl = _mm256_unpacklo_epi8(d, _mm256_setzero_si256());
        dl = _mm256_madd_epi16(dl, dl);

        // Squared elements, 15..31
        __m256i dh = _mm256_unpackhi_epi8(d, _mm256_setzero_si256());
        dh = _mm256_madd_epi16(dh, dh);
        
        acc = _mm256_add_epi32(acc, _mm256_add_epi32(dl, dh));
    }

    return reduce(acc);
}

static unsigned L2DistanceSquared_AVX2(const U8Descriptor& d, const BoundingBox& bb)
{
    __m256i acc = _mm256_setzero_si256();
    
    for (int i = 0; i < 4; ++i) {
        __m256i d1 = _mm256_sub_epi8(bb.min.features[i], _mm256_min_epu8(bb.min.features[i], d.features[i]));
        __m256i d2 = _mm256_sub_epi8(_mm256_max_epu8(bb.max.features[i], d.features[i]), bb.max.features[i]);

        __m256i add = _mm256_add_epi8(d1, d2);

        __m256i up1 = _mm256_unpacklo_epi8(add, _mm256_setzero_si256());
        up1 = _mm256_madd_epi16(up1, up1); //8x32bit res
        
        __m256i up2 = _mm256_unpackhi_epi8(add, _mm256_setzero_si256());
        up2 = _mm256_madd_epi16(up2, up2); //8x32bit res

        acc = _mm256_add_epi32(acc, _mm256_add_epi32(up1, up2));
    }

    return reduce(acc);
}

unsigned L2DistanceSquared_scalar(const U8Descriptor& ad, const U8Descriptor& bd) {
    unsigned sum = 0;

    for (int i = 0; i < 128; i++) {
        int a = (int)ad.ufeatures[i] - (int)bd.ufeatures[i];
        sum += a*a;
    }
    return sum;
}

unsigned L2DistanceSquared_scalar(const U8Descriptor& d, const BoundingBox& bb) {
    unsigned sum = 0;
    for (int i = 0; i < 128; i++) {
        int a = 0;
        if (d.ufeatures[i] < bb.min.ufeatures[i]) {
            a = (int)bb.min.ufeatures[i] - (int)d.ufeatures[i];
        }
        else if (d.ufeatures[i] > bb.max.ufeatures[i]) {
            a = (int)d.ufeatures[i] - (int)bb.max.ufeatures[i];
        }
        sum += a*a;
    }
    return sum;
}

unsigned L1Distance(const U8Descriptor& ad, const U8Descriptor& bd)
{
#ifdef __AVX2__
    return L1Distance_AVX2(ad, bd);
#else
    return L1Distance_scalar(ad, bd);
#endif
}

unsigned L1Distance(const U8Descriptor& ad, const BoundingBox& bb)
{
#ifdef __AVX2__
    return L1Distance_AVX2(ad, bb);
#else
    return L1Distance_scalar(ad, bb);
#endif
}

unsigned L2DistanceSquared(const U8Descriptor& ad, const U8Descriptor& bd)
{
#ifdef __AVX2__
    return L2DistanceSquared_AVX2(ad, bd);
#else
    return L2DistanceSquared_scalar(ad, bd);
#endif
}

unsigned L2DistanceSquared(const U8Descriptor& ad, const BoundingBox& bb)
{
#ifdef __AVX2__
    return L2DistanceSquared_AVX2(ad, bb);
#else
    return L2DistanceSquared_scalar(ad, bb);
#endif
}

static void verifyOne(U8Descriptor& a, U8Descriptor& b) {
    POPSIFT_KDASSERT(L2DistanceSquared_scalar(a, b) == L2DistanceSquared_AVX2(a, b));
    POPSIFT_KDASSERT(L2DistanceSquared_scalar(a, b) == L2DistanceSquared_AVX2(b, a));

    POPSIFT_KDASSERT(L1Distance_scalar(a, b) == L1Distance_AVX2(a, b));
    POPSIFT_KDASSERT(L1Distance_scalar(a, b) == L1Distance_AVX2(b, a));
}

static void verifyOne(U8Descriptor& a, BoundingBox& b) {
    POPSIFT_KDASSERT(L2DistanceSquared_scalar(a, b) == L2DistanceSquared_AVX2(a, b));
    POPSIFT_KDASSERT(L1Distance_scalar(a, b) == L1Distance_AVX2(a, b));
}

void VerifyL2DistanceAVX() {
    using namespace popsift::kdtree;
    U8Descriptor descs[3]; //rand, null, one
    BoundingBox boxes[5]; //rand_rand, rand_null, rand_one, one_one, null_null
    
    static std::mt19937_64 rng_engine;
    std::uniform_int_distribution<int> rng(0, 255);

    for (int i = 0; i < 128; i++) {
        descs[0].ufeatures[i] = rng(rng_engine);
        descs[1].ufeatures[i] = 0;
        descs[2].ufeatures[i] = 255;
        
        boxes[0].min.ufeatures[i] = rng(rng_engine);
        boxes[0].max.ufeatures[i] = rng(rng_engine);

        boxes[1].min.ufeatures[i] = rng(rng_engine);
        boxes[1].max.ufeatures[i] = 0;

        for (int x = 0; x < 128; x++) {
            if (boxes[0].min.ufeatures[x] > boxes[0].max.ufeatures[x])
                std::swap(boxes[0].min.ufeatures[x], boxes[0].max.ufeatures[x]);
            if (boxes[1].min.ufeatures[x] > boxes[1].max.ufeatures[x])
                std::swap(boxes[1].min.ufeatures[x], boxes[1].max.ufeatures[x]);
        }

        boxes[2].min.ufeatures[i] = rng(rng_engine);
        boxes[2].max.ufeatures[i] = 255;

        boxes[3].min.ufeatures[i] = 255;
        boxes[3].max.ufeatures[i] = 255;

        boxes[4].min.ufeatures[i] = 0;
        boxes[4].max.ufeatures[i] = 0;

    }
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            verifyOne(descs[i], descs[j]);
        }
        for (int j = 0; j < 5; j++) {
            verifyOne(descs[i], boxes[j]);
        }
    }
}

// Returns a pair: first contains dimension indices, second contains mean values for them.
std::pair<SplitDimensions, SplitDimensions> GetSplitDimensions(const U8Descriptor* descriptors, const unsigned* indexes, size_t count)
{
    using namespace Eigen;

    using U8Point = Map<Array<unsigned char, 1, 128>, Aligned32>;
    using DPoint = Array<double, 1, 128>;

    DPoint mean = DPoint::Zero();
    for (size_t i = 0; i < count; ++i) {
        unsigned char* p = const_cast<unsigned char*>(descriptors[indexes[i]].ufeatures.data());
        mean += U8Point(p).cast<double>();
    }
    mean /= count;

    DPoint var = DPoint::Zero();
    for (size_t i = 0; i < count; ++i) {
        unsigned char* p = const_cast<unsigned char*>(descriptors[indexes[i]].ufeatures.data());
        auto d = U8Point(p).cast<double>() - mean;
        var += d*d;
    }

    using vd_tup = std::tuple<double, unsigned>;
    std::array<vd_tup, 128> vardim;
    for (int i = 0; i < 128; ++i)
        vardim[i] = std::make_tuple(var[i], i);
    std::partial_sort(vardim.begin(), vardim.begin() + SPLIT_DIMENSION_COUNT, vardim.end(),
        [](const vd_tup& v1, const vd_tup& v2) { return std::get<0>(v1) > std::get<0>(v2); });

    SplitDimensions dims, means;
    for (size_t i = 0; i < SPLIT_DIMENSION_COUNT; ++i) {
        dims[i] = std::get<1>(vardim[i]);
        means[i] = static_cast<unsigned char>(mean(dims[i]));
    }

    return std::make_pair(dims, means);
}

//! Compute BB of descriptors referenced by count indexes.
BoundingBox GetBoundingBox(const U8Descriptor* descriptors, const unsigned* indexes, size_t count)
{
    U8Descriptor min, max;

#ifdef __AVX2__
    for (int i = 0; i < 4; i++) {
        min.features[i] = _mm256_set1_epi8(-1);
        max.features[i] = _mm256_setzero_si256();
    }
    for (size_t i = 0; i < count; ++i)
    for (int j = 0; j < 4; ++j) {
        min.features[j] = _mm256_min_epu8(min.features[j], descriptors[indexes[i]].features[j]);
        max.features[j] = _mm256_max_epu8(max.features[j], descriptors[indexes[i]].features[j]);
    }
#else
    for (int x = 0; x < 128; x++) {
        min.ufeatures[x] = -1;
        max.ufeatures[x] = 0;
    }
    for (size_t i = 0; i < count; ++i)
    for (int j = 0; j < 128; ++j) {
        min.ufeatures[j] = std::min(min.ufeatures[j], descriptors[indexes[i]].ufeatures[j]);
        max.ufeatures[j] = std::max(max.ufeatures[j], descriptors[indexes[i]].ufeatures[j]);
    }
#endif
    return BoundingBox{ min, max };
}

BoundingBox Union(const BoundingBox& a, const BoundingBox& b)
{
    BoundingBox r;
#ifdef __AVX2__
    for (int i = 0; i < 4; ++i) {
        r.min.features[i] = _mm256_min_epu8(a.min.features[i], b.min.features[i]);
        r.max.features[i] = _mm256_max_epu8(a.max.features[i], b.max.features[i]);
    }
#else
    for (int i = 0; i < 128; i++) {
        r.min.ufeatures[i] = std::min(a.min.ufeatures[i], b.min.ufeatures[i]);
        r.max.ufeatures[i] = std::max(a.max.ufeatures[i], b.max.ufeatures[i]);
    }
#endif
    return r;
}

}   // kdtree
}   // popsift