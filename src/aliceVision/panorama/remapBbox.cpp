#include "remapBbox.hpp"
#include "sphericalMapping.hpp"

namespace aliceVision
{

bool isPoleInTriangle(const Vec3& pt1, const Vec3& pt2, const Vec3& pt3)
{

    double a = (pt2.x() * pt3.z() - pt3.x() * pt2.z()) / (pt1.x() * pt2.z() - pt1.x() * pt3.z() - pt2.x() * pt1.z() +
                                                          pt2.x() * pt3.z() + pt3.x() * pt1.z() - pt3.x() * pt2.z());
    double b = (-pt1.x() * pt3.z() + pt3.x() * pt1.z()) / (pt1.x() * pt2.z() - pt1.x() * pt3.z() - pt2.x() * pt1.z() +
                                                           pt2.x() * pt3.z() + pt3.x() * pt1.z() - pt3.x() * pt2.z());
    double c = 1.0 - a - b;

    if(a < 0.0 || a > 1.0)
        return false;
    if(b < 0.0 || b > 1.0)
        return false;
    if(c < 0.0 || c > 1.0)
        return false;

    return true;
}

bool crossHorizontalLoop(const Vec3& pt1, const Vec3& pt2)
{
    Vec3 direction = pt2 - pt1;

    /*Vertical line*/
    if(std::abs(direction(0)) < 1e-12)
    {
        return false;
    }

    double t = -pt1(0) / direction(0);
    Vec3 cross = pt1 + direction * t;

    if(t >= 0.0 && t <= 1.0)
    {
        if(cross(2) < 0.0)
        {
            return true;
        }
    }

    return false;
}

Vec3 getExtremaY(const Vec3& pt1, const Vec3& pt2)
{

    Vec3 delta = pt2 - pt1;
    double dx = delta(0);
    double dy = delta(1);
    double dz = delta(2);
    double sx = pt1(0);
    double sy = pt1(1);
    double sz = pt1(2);

    double ot_y = -(dx * sx * sy - (dy * sx) * (dy * sx) - (dy * sz) * (dy * sz) + dz * sy * sz) /
                  (dx * dx * sy - dx * dy * sx - dy * dz * sz + dz * dz * sy);

    Vec3 pt_extrema = pt1 + ot_y * delta;

    return pt_extrema.normalized();
}

bool computeCoarseBB_Equidistant(BoundingBox& coarse_bbox, const std::pair<int, int>& panoramaSize,
                                 const geometry::Pose3& pose, const aliceVision::camera::IntrinsicBase& intrinsics)
{

    const aliceVision::camera::EquiDistant& cam = dynamic_cast<const camera::EquiDistant&>(intrinsics);

    bool loop = false;
    std::vector<bool> vec_bool(panoramaSize.second, false);

    for(int i = 0; i < panoramaSize.second; i++)
    {

        {
            Vec3 ray = SphericalMapping::fromEquirectangular(Vec2(0, i), panoramaSize.first, panoramaSize.second);

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
        }

        {
            Vec3 ray = SphericalMapping::fromEquirectangular(Vec2(panoramaSize.first - 1, i), panoramaSize.first,
                                                             panoramaSize.second);

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

            vec_bool[i] = true;
            loop = true;
        }
    }

    if(vec_bool[0] || vec_bool[panoramaSize.second - 1])
    {
        loop = false;
    }

    if(!loop)
    {
        coarse_bbox.left = 0;
        coarse_bbox.top = 0;
        coarse_bbox.width = panoramaSize.first;
        coarse_bbox.height = panoramaSize.second;
        return true;
    }

    int last_x = 0;

    for(int x = panoramaSize.first - 1; x >= 0; x--)
    {

        size_t count = 0;

        for(int i = 0; i < panoramaSize.second; i++)
        {

            if(vec_bool[i] == false)
            {
                continue;
            }

            Vec3 ray = SphericalMapping::fromEquirectangular(Vec2(x, i), panoramaSize.first, panoramaSize.second);

            /**
             * Check that this ray should be visible.
             * This test is camera type dependent
             */
            Vec3 transformedRay = pose(ray);
            if(!intrinsics.isVisibleRay(transformedRay))
            {
                vec_bool[i] = false;
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
                vec_bool[i] = false;
                continue;
            }

            count++;
        }

        if(count == 0)
        {
            break;
        }

        last_x = x;
    }

    coarse_bbox.left = last_x;
    coarse_bbox.top = 0;
    coarse_bbox.width = panoramaSize.first;
    coarse_bbox.height = panoramaSize.second;

    return true;
}

bool computeCoarseBB_Pinhole(BoundingBox& coarse_bbox, const std::pair<int, int>& panoramaSize,
                             const geometry::Pose3& pose, const aliceVision::camera::IntrinsicBase& intrinsics)
{

    int bbox_left, bbox_top;
    int bbox_right, bbox_bottom;
    int bbox_width, bbox_height;

    /*Estimate distorted maximal distance from optical center*/
    Vec2 pts[] = {{0.0f, 0.0f}, {intrinsics.w(), 0.0f}, {intrinsics.w(), intrinsics.h()}, {0.0f, intrinsics.h()}};
    float max_radius = 0.0;
    for(int i = 0; i < 4; i++)
    {

        Vec2 ptmeter = intrinsics.ima2cam(pts[i]);
        float radius = ptmeter.norm();
        max_radius = std::max(max_radius, radius);
    }

    /* Estimate undistorted maximal distance from optical center */
    float max_radius_distorted = max_radius; // intrinsics.getMaximalDistortion(0.0, max_radius);

    /*
    Coarse rectangle bouding box in camera space
    We add intermediate points to ensure arclength between 2 points is never more than 180°
    */
    Vec2 pts_radius[] = {{-max_radius_distorted, -max_radius_distorted}, {0, -max_radius_distorted},
                         {max_radius_distorted, -max_radius_distorted},  {max_radius_distorted, 0},
                         {max_radius_distorted, max_radius_distorted},   {0, max_radius_distorted},
                         {-max_radius_distorted, max_radius_distorted},  {-max_radius_distorted, 0}};

    /*
    Transform bounding box into the panorama frame.
    Point are on a unit sphere.
    */
    Vec3 rotated_pts[8];
    for(int i = 0; i < 8; i++)
    {
        Vec3 pt3d = intrinsics.toUnitSphere(pts_radius[i]);
        rotated_pts[i] = pose.rotation().transpose() * pt3d;
    }

    /* Vertical Default solution : no pole*/
    bbox_top = panoramaSize.second;
    bbox_bottom = 0;

    for(int i = 0; i < 8; i++)
    {
        int i2 = (i + 1) % 8;

        Vec3 extremaY = getExtremaY(rotated_pts[i], rotated_pts[i2]);

        Vec2 res;
        res = SphericalMapping::toEquirectangular(extremaY, panoramaSize.first, panoramaSize.second);
        bbox_top = std::min(int(floor(res(1))), bbox_top);
        bbox_bottom = std::max(int(ceil(res(1))), bbox_bottom);

        res = SphericalMapping::toEquirectangular(rotated_pts[i], panoramaSize.first, panoramaSize.second);
        bbox_top = std::min(int(floor(res(1))), bbox_top);
        bbox_bottom = std::max(int(ceil(res(1))), bbox_bottom);
    }

    /*
    Check if our region circumscribe a pole of the sphere :
    Check that the region projected on the Y=0 plane contains the point (0, 0)
    This is a special projection case
    */
    bool pole = isPoleInTriangle(rotated_pts[0], rotated_pts[1], rotated_pts[7]);
    pole |= isPoleInTriangle(rotated_pts[1], rotated_pts[2], rotated_pts[3]);
    pole |= isPoleInTriangle(rotated_pts[3], rotated_pts[4], rotated_pts[5]);
    pole |= isPoleInTriangle(rotated_pts[7], rotated_pts[5], rotated_pts[6]);
    pole |= isPoleInTriangle(rotated_pts[1], rotated_pts[3], rotated_pts[5]);
    pole |= isPoleInTriangle(rotated_pts[1], rotated_pts[5], rotated_pts[7]);

    if(pole)
    {
        Vec3 normal = (rotated_pts[1] - rotated_pts[0]).cross(rotated_pts[3] - rotated_pts[0]);
        if(normal(1) > 0)
        {
            // Lower pole
            bbox_bottom = panoramaSize.second - 1;
        }
        else
        {
            // upper pole
            bbox_top = 0;
        }
    }

    bbox_height = bbox_bottom - bbox_top + 1;

    /*Check if we cross the horizontal loop*/
    bool crossH = false;
    for(int i = 0; i < 8; i++)
    {
        int i2 = (i + 1) % 8;

        bool cross = crossHorizontalLoop(rotated_pts[i], rotated_pts[i2]);
        crossH |= cross;
    }

    if(pole)
    {
        /*Easy : if we cross the pole, the width is full*/
        bbox_left = 0;
        bbox_right = panoramaSize.first - 1;
        bbox_width = bbox_right - bbox_left + 1;
    }
    else if(crossH)
    {
        int first_cross = 0;
        for(int i = 0; i < 8; i++)
        {
            int i2 = (i + 1) % 8;
            bool cross = crossHorizontalLoop(rotated_pts[i], rotated_pts[i2]);
            if(cross)
            {
                first_cross = i;
                break;
            }
        }

        bbox_left = panoramaSize.first - 1;
        bbox_right = 0;
        bool is_right = true;
        for(int index = 0; index < 8; index++)
        {

            int i = (index + first_cross) % 8;
            int i2 = (i + 1) % 8;

            Vec2 res_1 = SphericalMapping::toEquirectangular(rotated_pts[i], panoramaSize.first, panoramaSize.second);
            Vec2 res_2 = SphericalMapping::toEquirectangular(rotated_pts[i2], panoramaSize.first, panoramaSize.second);

            /*[----right ////  left-----]*/
            bool cross = crossHorizontalLoop(rotated_pts[i], rotated_pts[i2]);
            if(cross)
            {
                if(res_1(0) > res_2(0))
                { /*[----res2 //// res1----]*/
                    bbox_left = std::min(int(res_1(0)), bbox_left);
                    bbox_right = std::max(int(res_2(0)), bbox_right);
                    is_right = true;
                }
                else
                { /*[----res1 //// res2----]*/
                    bbox_left = std::min(int(res_2(0)), bbox_left);
                    bbox_right = std::max(int(res_1(0)), bbox_right);
                    is_right = false;
                }
            }
            else
            {
                if(is_right)
                {
                    bbox_right = std::max(int(res_1(0)), bbox_right);
                    bbox_right = std::max(int(res_2(0)), bbox_right);
                }
                else
                {
                    bbox_left = std::min(int(res_1(0)), bbox_left);
                    bbox_left = std::min(int(res_2(0)), bbox_left);
                }
            }
        }
        
        bbox_width = bbox_right + (panoramaSize.first - bbox_left);
    }
    else
    {
        /*horizontal default solution : no border crossing, no pole*/
        bbox_left = panoramaSize.first;
        bbox_right = 0;
        for(int i = 0; i < 8; i++)
        {
            Vec2 res = SphericalMapping::toEquirectangular(rotated_pts[i], panoramaSize.first, panoramaSize.second);
            bbox_left = std::min(int(floor(res(0))), bbox_left);
            bbox_right = std::max(int(ceil(res(0))), bbox_right);
        }
        bbox_width = bbox_right - bbox_left + 1;
    }

    /*Assign solution to result*/
    coarse_bbox.left = bbox_left;
    coarse_bbox.top = bbox_top;
    coarse_bbox.width = bbox_width;
    coarse_bbox.height = bbox_height;

    return true;
}

bool computeCoarseBB(BoundingBox& coarse_bbox, const std::pair<int, int>& panoramaSize, const geometry::Pose3& pose,
                     const aliceVision::camera::IntrinsicBase& intrinsics)
{

    bool ret = true;

    if(isPinhole(intrinsics.getType()))
    {
        ret = computeCoarseBB_Pinhole(coarse_bbox, panoramaSize, pose, intrinsics);
    }
    else if(isEquidistant(intrinsics.getType()))
    {
        ret = computeCoarseBB_Equidistant(coarse_bbox, panoramaSize, pose, intrinsics);
    }
    else
    {
        coarse_bbox.left = 0;
        coarse_bbox.top = 0;
        coarse_bbox.width = panoramaSize.first;
        coarse_bbox.height = panoramaSize.second;
        ret = true;
    }

    return ret;
}

} // namespace aliceVision
