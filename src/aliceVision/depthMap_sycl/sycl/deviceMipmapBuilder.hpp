// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

// this used to be a lookup table based on std::expf, but constexpr sycl::exp allows for cleaner code and faster execution

static constexpr int delta = 1;
inline static constexpr float gauss(int x) {
    return sycl::exp(-float(x * x) / float(2 * delta * delta));
}

//template<int TRadius>
static constexpr int TRadius = 2;
inline sycl::event sycl_mipmapBuildNextLevel(DeviceMipmapImage& mipmapImage,
                                             unsigned int downscale,
                                             sycl::queue& queue, sycl::event prerequisite)
{
    // consts we can calculate outside of the kernel
    const int level = mipmapImage.getLevelInt(downscale);
    assert(level >= 1);
    const SyclSize<2> size_h = mipmapImage.getDimensions(downscale);
    const sycl::uint2 size = sycl::uint2(size_h.x(), size_h.y());
    const sycl::float2 invSize = 1.f / size.convert<float>();

    // Accessors
    const MipmapImageAccess mipmapAcc{mipmapImage};

    // Kernel submission
    return queue.submit([&](sycl::handler& h) {
        h.depends_on(prerequisite);

        h.parallel_for(sycl::range<1>(size_h.x() * size_h.y()), [=](auto idx) {
            SyclRGBA sumColor{0};
            float sumFactor = 0.0f;
            const sycl::uint2 coords = sycl::uint2(idx[0] % size.x(), idx[0] / size.x());

            for(int j = -TRadius; j <= TRadius; j++)
            {
                for(int i = -TRadius; i <= TRadius; i++)
                {
                    // Local offset
                    const sycl::int2 offset = sycl::int2(i, j);

                    // domain factor
                    const float factor = gauss(i) * gauss(j);

                    // current pixel color
                    const SyclRGBA color = mipmapAcc.bilinear((coords.convert<int>() + offset).convert<float>() * invSize,
                                                               level - 1);

                    // sum color
                    sumColor += color * factor;

                    // sum factor
                    sumFactor += factor;
                }
            }

            mipmapAcc(coords, level) = (sumColor / sumFactor);
        });
    });
}

inline sycl::event sycl_mipmapFirstLevelCopy(const SyclDeviceMemoryPitched<sycl::float4, 2>& in_img_dmp,
                                             DeviceMipmapImage& mipmapImage,
                                             int downscale,
                                             sycl::queue& queue, sycl::event prerequisite)
{
    // consts we can calculate outside of the kernel
    const int level = mipmapImage.getLevelInt(downscale);
    const SyclSize<2> size_h = mipmapImage.getDimensions(downscale);
    const sycl::uint2 size = sycl::uint2(size_h.x(), size_h.y());

    const bool computeDownscale = downscale>1;

    // Accessors/host mapped image properties
    const MipmapImageAccess mipmapAcc{mipmapImage};
    const SyclDevicePitchedAccess in_img_acc{in_img_dmp};

    // Kernel submission
    return queue.submit([&](sycl::handler& h) {
        h.depends_on(prerequisite);

        h.parallel_for(sycl::range<1>(size_h.x() * size_h.y()), [=](auto idx) {
            sycl::uint2 coords = sycl::uint2(idx[0] % size.x(), idx[0] / size.x());

            sycl::float4 inPix{0};
            if(computeDownscale)
            {
                // compute gausian blur
                float sumFactor = 0.0f;

                for(int j = -downscale; j <= downscale; j++) // Note: gausian radius is downscale level
                {
                    for(int i = -downscale; i <= downscale; i++)
                    {
                        const sycl::float4 sampPix = __readonly_load(&in_img_acc((coords.convert<int>() * downscale + sycl::int2(i, j)).convert<uint>()));

                        const float factor = gauss(i) * gauss(j); // domain factor

                        inPix += sampPix * factor;
                        sumFactor += factor;
                    }
                }
                inPix /= sumFactor;
            }
            else
            {
                inPix = __readonly_load(&in_img_acc(coords));
            }

            // compute output CIELAB
            // RGB(0, 1) to XYZ(0, 1) to CIELAB(0, 255)
            const sycl::float3 flab = xyz2lab(rgb2xyz(sycl::float3(inPix.r(), inPix.g(), inPix.b())));

            mipmapAcc(coords, level) = sycl::float4(flab.x(), flab.y(), flab.z(), inPix.a()*255.f).convert<SyclColorBaseType>();
        });
    });
}
