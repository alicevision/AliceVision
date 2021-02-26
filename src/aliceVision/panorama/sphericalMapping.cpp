#include "sphericalMapping.hpp"

namespace aliceVision
{

namespace SphericalMapping
{
/**
 * Map from equirectangular to spherical coordinates
 * @param equirectangular equirectangular coordinates
 * @param width number of pixels used to represent longitude
 * @param height number of pixels used to represent latitude
 * @return spherical coordinates
 */
Vec3 fromEquirectangular(const Vec2& equirectangular, int width, int height)
{
    const double latitude = (equirectangular(1) / double(height)) * M_PI - M_PI_2;
    const double longitude = ((equirectangular(0) / double(width)) * 2.0 * M_PI) - M_PI;

    const double Px = cos(latitude) * sin(longitude);
    const double Py = sin(latitude);
    const double Pz = cos(latitude) * cos(longitude);

    return Vec3(Px, Py, Pz);
}

/**
 * Map from Spherical to equirectangular coordinates
 * @param spherical spherical coordinates
 * @param width number of pixels used to represent longitude
 * @param height number of pixels used to represent latitude
 * @return equirectangular coordinates
 */
Vec2 toEquirectangular(const Vec3& spherical, int width, int height)
{

    double vertical_angle = asin(spherical(1));
    double horizontal_angle = atan2(spherical(0), spherical(2));

    double latitude = ((vertical_angle + M_PI_2) / M_PI) * height;
    double longitude = ((horizontal_angle + M_PI) / (2.0 * M_PI)) * width;

    return Vec2(longitude, latitude);
}

} // namespace SphericalMapping
} // namespace aliceVision