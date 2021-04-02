#include "checkerDetector.hpp"



namespace aliceVision{
namespace calibration{

bool CheckerDetector::process(const image::Image<image::RGBColor> & source)
{
    image::Image<float> grayscale;
    image::ConvertPixelType(source, &grayscale);

    if (!processLevel(grayscale))
    {
        return false;
    }
    
    return true;
}

bool CheckerDetector::processLevel(const image::Image<float> & input) 
{   

    /*image::Image<float> norm(input.Width(), input.Height());
    image::Image<float> angle(input.Width(), input.Height());

    float min = std::numeric_limits<float>::max();
    float max = std::numeric_limits<float>::min();
    for (int y = 0; y < gy.Height(); y++)
    {
        for (int x = 0; x < gy.Width(); x++)
        {
            double dx = gx(y, x);
            double dy = gy(y, x);
            norm(y, x) = sqrt(dx*dx+dy*dy);
            angle(y, x) = atan2(dy, dx);

            min = std::min(min, input(y, x));
            max = std::max(max, input(y, x));
        }
    }*/

    
    image::Image<float> normalized;
    normalizeImage(normalized, input);

    // A New Sub-Pixel Detector for X-Corners in Camera Calibration Targets
    // @inproceedings{inproceedings,
    // author = {Chen, Dazhi and Zhang, Guangjun},
    // title = {A New Sub-Pixel Detector for X-Corners in Camera Calibration Targets.}
    image::Image<float> hessian;
    computeHessianResponse(hessian, normalized);

    // Liu, Y., Liu, S., Cao, Y., & Wang, Z. (2016). Automatic chessboard corner detection method. IET Image Processing, 10(1), 16-23.
    std::vector<Vec2> raw_corners;
    extractCorners(raw_corners, hessian);

    // Geiger, Andreas & Moosmann, Frank & Car, Omer & Schuster, Bernhard. (2012). Automatic camera and range sensor calibration using a single shot. 
    std::vector<Vec2> refined_corners;
    refineCorners(refined_corners, raw_corners, normalized);

    // David Fleet and Tomas Pajdla and Bernt Schiele and Tinne Tuytelaars. ROCHADE: Robust Checkerboard Advanced Detection for Camera Calibration
    std::vector<Vec2> fitted_corners;
    fitCorners(refined_corners, refined_corners, normalized);

    return true;
}

bool CheckerDetector::normalizeImage(image::Image<float> & output, const image::Image<float> & input) {
    
    float min, max;
    getMinMax(min, max, input);
    
    output.resize(input.Width(), input.Height());
    for (int y = 0; y < output.Height(); y++)
    {
        for (int x = 0; x < output.Width(); x++)
        {
            output(y, x) = (input(y, x) - min) / (max - min);
        }
    }

    return true;
}

bool CheckerDetector::computeHessianResponse(image::Image<float> & output, const image::Image<float> & input)
{
    image::Image<float> smoothed;    
    image::ImageGaussianFilter(input, 1.5, smoothed, 2);

    image::Image<float> gx, gy;
    ImageXDerivative(smoothed, gx, true);
    ImageYDerivative(smoothed, gy, true);

    image::Image<float> gxx, gxy, gyy;
    ImageXDerivative(gx, gxx, true);
    ImageXDerivative(gy, gxy, true);
    ImageYDerivative(gy, gyy, true);

    output.resize(input.Width(), input.Height());
    for (int y = 0; y < input.Height(); y++)
    {
        for (int x = 0; x < input.Width(); x++)
        {            
            output(y, x) = std::abs(gxx(y, x) * gyy(y, x) - 2.0 * gxy(y, x));
        }
    }

    return true;
}

bool CheckerDetector::extractCorners(std::vector<Vec2> & raw_corners, const image::Image<float> & hessianResponse)
{
    float min, max;
    getMinMax(min, max, hessianResponse);
    
    float threshold = max * 0.1;
    const int radius = 7;


    image::Image<float> output(hessianResponse.Width(), hessianResponse.Height(), true, 0.0f);
    for (int i = radius; i < hessianResponse.Height() - radius; i++)
    {
        for (int j = radius; j < hessianResponse.Width() - radius; j++)
        {
            bool isMinimal = true;
            float val = hessianResponse(i, j);

            for (int k = -radius; k <= radius; k++)
            {
                for (int l = -radius; l <= radius; l++)
                {
                    if (hessianResponse(i + k, j + l) > val)
                    {
                        isMinimal = false;
                    }
                }
            }

            if (!isMinimal)
            {
               continue;
            }

            if (val > threshold)
            {
                Vec2 pt;
                pt.x() = j;
                pt.y() = i;

                raw_corners.push_back(pt);
            }
        }
    }

    return true;
}

void CheckerDetector::getMinMax(float &min, float &max, const image::Image<float> & input)
{
    min = std::numeric_limits<float>::max();
    max = std::numeric_limits<float>::min();
    for (int y = 0; y < input.Height(); y++)
    {
        for (int x = 0; x < input.Width(); x++)
        {
            min = std::min(min, input(y, x));
            max = std::max(max, input(y, x));
        }
    }
}

bool CheckerDetector::refineCorners(std::vector<Vec2> & refined_corners, const std::vector<Vec2> & raw_corners, const image::Image<float> & input)
{
    image::Image<float> gx, gy;
    ImageXDerivative(input, gx, true);
    ImageYDerivative(input, gy, true);

    const int radius = 5;

    for (const Vec2 & pt: raw_corners)
    {
        if (pt.x() < radius) continue;
        if (pt.y() < radius) continue;
        if (pt.x() >= gx.Width() - radius) continue;
        if (pt.y() >= gx.Height() - radius) continue;

        Eigen::Matrix2d A = Eigen::Matrix2d::Zero();
        Eigen::Vector2d b = Eigen::Vector2d::Zero();
        for (int k = -radius; k <= radius; k++)
        {
            int i = pt.y() + k;

            for (int l = -radius; l <= radius; l++)
            {
                if (l == 0 && k == 0) continue;

                int j = pt.x() + l;

                float du = gx(i, j);
                float dv = gy(i, j);
                float norm = sqrt(du * du + dv * dv);
                if (norm < 0.1) continue;

                du /= norm;
                dv /= norm;

                A(0, 0) += du * du;
                A(0, 1) += du * dv;
                A(1, 0) += du * dv;
                A(1, 1) += dv * dv;

                b(0) += du * du * double(j) + du * dv * double(i);
                b(1) += dv * du * double(j) + dv * dv * double(i);
            }
        }

        Vec2 update = A.inverse() * b;

        //Make sure the update is coherent
        double dist = (update - pt).norm();
        if (dist > radius) continue;
        if (dist != dist) continue;

        refined_corners.push_back(update);
    }

    return true;
}

bool CheckerDetector::fitCorners(std::vector<Vec2> & refined_corners, const std::vector<Vec2> & raw_corners, const image::Image<float> & input)
{
    //Build kernel
    const int radius = 4;
    const int diameter = 2 * radius + 1;
    Eigen::MatrixXd kernel(diameter, diameter);

    double norm = 0.0;
    for (int i = -radius; i <= radius; i++)
    {
        int di = i + radius;
        for (int j = -radius; j <= radius; j++)
        {
            int dj = j + radius;

            double val = std::max(0.0, radius + 1 - std::sqrt(i * i + j * j));
            norm += val;

            kernel(di, dj) = val;
        }
    }

    kernel /= norm;

    image::Image<float> filtered;
    image::ImageConvolution(input, kernel, filtered);

    Eigen::MatrixXd AtA(6, 6);
    Eigen::Vector<double, 6> Atb;

    const image::Sampler2d<image::SamplerLinear> sampler;

    for (Vec2 corner : raw_corners)
    {
        bool isValid = true;

        double cx = corner(0);
        double cy = corner(1);

        Vec2 ori_x = Eigen::Vector2d::UnitX();
        Vec2 ori_y = Eigen::Vector2d::UnitY();

        for (int iter = 0; iter < 20; iter++)
        {
            AtA.fill(0);
            Atb.fill(0);      

            for (int i = -radius; i <= radius; i++)
            {
                int di = corner(1) + i;
                for (int j = -radius; j <= radius; j++)
                {
                    int dj = corner(0) + j;

                    Eigen::Vector<double, 6> rowA;
                    rowA(0) = j * j;
                    rowA(1) = i * j;
                    rowA(2) = i * i;
                    rowA(3) = j;
                    rowA(4) = i;
                    rowA(5) = 1.0;

                    AtA += rowA * rowA.transpose();
                    Atb += rowA * sampler(filtered, di, dj);
                }                
            }
            
            Eigen::Vector<double, 6> x = AtA.inverse() * Atb;

            //f(x,y) = a1x**2 + a2xy + a3y**2 + a4x + a5y + a6
            //df(x)/dx = 2a1x + a2y + a4
            //df(x)/dy = 2a3y + a2x + a5
            //H = |2a1 a2 |
            //    |a2  2a3|
            //det(H) = 2*2*a1*a3 - 2*a2
            //inv(H) = |2a3 -a2|/det(H)
            //         |-a2 2a1|
            double a1 = x(0);
            double a2 = x(1);
            double a3 = x(2);
            double a4 = x(3);
            double a5 = x(4);
            double a6 = x(5);


            double determinantH = 4.0*a1*a3 - 2.0*a2;
            if (std::abs(determinantH) < 1e-6)
            {
                isValid = false;
                break;
            }

            Eigen::Matrix2d H;
            H(0, 0) = 2.0 * a1;
            H(0, 1) = a2;
            H(1, 0) = a2;
            H(1, 1) = 2.0 * a3;

            Eigen::Vector2d vecB;
            vecB(0) = -a4;
            vecB(1) = -a5;

            Vec2 update = H.inverse() * vecB;
            corner += update;
        
            Eigen::EigenSolver<Eigen::Matrix2d> solver(H, true);
            double l1 = solver.eigenvalues()(0).real();
            double l2 = solver.eigenvalues()(1).real();
            if (l1 * l2 > -1e-12)
            {
                isValid = false;
                break;
            }


            if (corner(0) < radius || corner(0) >= input.Width() - radius || corner(1) < radius || corner(1) >= input.Height() - radius)
            {
                isValid = false;
                break;
            }

            if (update.norm() < 1e-5)
            {
                break;
            }
        }

        
        if (isValid)   
        {
            refined_corners.push_back(corner);
        }
    }

    return true;
}

}//namespace calibration
}//namespace aliceVision
