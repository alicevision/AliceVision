#include "coordinatesMap.hpp"

#include "sphericalMapping.hpp"

namespace aliceVision
{

bool CoordinatesMap::build(const std::pair<int, int>& panoramaSize, const geometry::Pose3& pose,
                           const aliceVision::camera::IntrinsicBase& intrinsics, const BoundingBox& coarseBbox)
{

    /* Effectively compute the warping map */
    _coordinates = aliceVision::image::Image<Eigen::Vector2d>(coarseBbox.width, coarseBbox.height, false);
    _mask = aliceVision::image::Image<unsigned char>(coarseBbox.width, coarseBbox.height, true, 0);

    int max_x = 0;
    int max_y = 0;
    int min_x = std::numeric_limits<int>::max();
    int min_y = std::numeric_limits<int>::max();

    for(int y = 0; y < coarseBbox.height; y++)
    {

        int cy = y + coarseBbox.top;
        if (cy < 0 || cy >= panoramaSize.second)
        {
            continue;
        }

        for(int x = 0; x < coarseBbox.width; x++)
        {

            int cx = x + coarseBbox.left;

            Vec3 ray = SphericalMapping::fromEquirectangular(Vec2(cx, cy), panoramaSize.first, panoramaSize.second);

            /**
             * Check that this ray should be visible.
             * This test is camera type dependent
             */
            Vec3 transformedRay = pose(ray);
            if(!intrinsics.isVisibleRay(transformedRay))
            {
                continue;
            }

            /**
             * Project this ray to camera pixel coordinates
             */
            const Vec2 pix_disto = intrinsics.project(pose, ray.homogeneous(), true);

            /**
             * Ignore invalid coordinates
             */
            if(!intrinsics.isVisible(pix_disto))
            {
                continue;
            }

            _coordinates(y, x) = pix_disto;
            _mask(y, x) = 1;

            min_x = std::min(cx, min_x);
            min_y = std::min(cy, min_y);
            max_x = std::max(cx, max_x);
            max_y = std::max(cy, max_y);
        }
    }

    _offset_x = coarseBbox.left;
    _offset_y = coarseBbox.top;

    _boundingBox.left = min_x;
    _boundingBox.top = min_y;
    _boundingBox.width = std::max(0, max_x - min_x + 1);
    _boundingBox.height = std::max(0, max_y - min_y + 1);

    return true;
}

bool CoordinatesMap::computeScale(double& result, float ratioUpscale)
{

    std::vector<double> scales;
    size_t real_height = _coordinates.Height();
    size_t real_width = _coordinates.Width();

    for(int i = 1; i < real_height - 2; i++)
    {
        for(int j = 1; j < real_width - 2; j++)
        {
            if(!_mask(i, j) || !_mask(i, j + 1) || !_mask(i + 1, j))
            {
                continue;
            }

            double dxx = _coordinates(i, j + 1).x() - _coordinates(i, j).x();
            double dxy = _coordinates(i + 1, j).x() - _coordinates(i, j).x();
            double dyx = _coordinates(i, j + 1).y() - _coordinates(i, j).y();
            double dyy = _coordinates(i + 1, j).y() - _coordinates(i, j).y();

            double det = std::abs(dxx * dyy - dxy * dyx);
            if(det < 1e-12)
                continue;

            scales.push_back(det);
        }
    }

    if(scales.empty())
        return false;

    std::sort(scales.begin(), scales.end());
    int selected_index = int(floor(float(scales.size() - 1) * ratioUpscale));
    result = sqrt(scales[selected_index]);

    return true;
}

} // namespace aliceVision
