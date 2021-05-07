#include <aliceVision/image/io.hpp>

#include <iostream>
#include <sstream>
#include <fstream>
#include <algorithm>
#include <math.h>

#include <Eigen/Dense>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/eigen.hpp>

#include "normalIntegration.hpp"

void normal2PQ(aliceVision::image::Image<aliceVision::image::RGBfColor>& normals, Eigen::MatrixXf& p, Eigen::MatrixXf& q, bool perspective, Eigen::Matrix3f K){

	aliceVision::image::Image<float> normalsX(p.cols(), p.rows());
	aliceVision::image::Image<float> normalsY(p.cols(), p.rows());
	aliceVision::image::Image<float> normalsZ(p.cols(), p.rows());

    for (size_t j = 0; j < p.cols(); ++j)
    {
        for (size_t i = 0; i < p.rows(); ++i)
        {
            normalsX(i,j) = normals(i,j)(0);
            normalsY(i,j) = normals(i,j)(1);
            normalsZ(i,j) = normals(i,j)(2);
        }
    }

    if(perspective)
    {
        float f = (K(0,0)+K(1,1))/2;

        for (size_t j = 0; j < p.cols(); ++j)
        {
            float u = j - K(0,2);

            for (size_t i = 0; i < p.rows(); ++i)
            {
                float v = i - K(1,2);

                float denom = u*normalsX(i,j) + v*normalsY(i,j) + f*normalsZ(i,j);

                if (denom == 0)
                {
                    p(i,j) = 0;
                    q(i,j) = 0;
                } else {
                    p(i,j) = -normalsX(i,j)/denom;
                    q(i,j) = -normalsY(i,j)/denom;
                }
           }
        }
    } else {
        for (size_t j = 0; j < p.cols(); ++j)
        {
            for (size_t i = 0; i < p.rows(); ++i)
            {
                if (normalsZ(i,j) == 0)
                {
                    p(i,j) = 0;
                    q(i,j) = 0;
                } else {
                    p(i,j) = -normalsX(i,j)/normalsZ(i,j);
                    q(i,j) = -normalsY(i,j)/normalsZ(i,j);
                }
            }
        }
    }
}

void getDivergenceField(const Eigen::MatrixXf& p, const Eigen::MatrixXf& q, Eigen::MatrixXf& f){

    int nbRows = p.rows();
    int nbCols = p.cols();

    Eigen::MatrixXf qy_below(nbRows, nbCols);
    qy_below = q;
    qy_below.block(0,0,nbRows-1,nbCols) = q.block(1,0,nbRows-1,nbCols);

    Eigen::MatrixXf qy_above(nbRows, nbCols);
    qy_above = q;
    qy_above.block(1,0,nbRows-1,nbCols) = q.block(0,0,nbRows-1,nbCols);

    Eigen::MatrixXf qy = 0.5*(qy_below-qy_above);

    Eigen::MatrixXf px_right = p;
    px_right.block(0,0,nbRows,nbCols-1) = p.block(0,1,nbRows,nbCols-1);

    Eigen::MatrixXf px_left = p;
    px_left.block(0,1,nbRows,nbCols-1) = p.block(0,0,nbRows,nbCols-1);

    Eigen::MatrixXf px = 0.5*(px_right-px_left);

    // Div(p,q) 
    f = px+qy;
}

void setBoundaryConditions(const Eigen::MatrixXf& p, const Eigen::MatrixXf& q, Eigen::MatrixXf& f){

    int nbRows = p.rows();
    int nbCols = p.cols();

    // Right hand side of the boundary condition
    Eigen::MatrixXf b = Eigen::MatrixXf::Zero(nbRows, nbCols);

    b.block(0,1,1,nbCols-2) = -q.block(0,1,1,nbCols-2);
    b.block(nbRows-1, 1, 1, nbCols-2) = q.block(nbRows-1, 1, 1, nbCols-2);
    b.block(1,0,nbRows-2, 1) = -p.block(1,0,nbRows-2, 1);
    b.block(1, nbCols-1, nbRows-2, 1) = p.block(1, nbCols-1, nbRows-2, 1);
    b(0,0) = (1/sqrt(2))*(-q(0,0)-p(0,0));
    b(0,nbCols-1) = (1/sqrt(2))*(-q(0,nbCols-1)+p(0,nbCols-1));
    b(nbRows-1, nbCols-1) = (1/sqrt(2))*(q(nbRows-1, nbCols-1)+p(nbRows-1, nbCols-1));
    b(nbRows-1,0) = (1/sqrt(2))*(q(nbRows-1,0)-p(nbRows-1,0));

    //Modification near the boundaries to enforce the non-homogeneous Neumann BC
    f.block(0,1,1,nbCols-2) = f.block(0,1,1,nbCols-2) - b.block(0,1,1,nbCols-2);
    f.block(nbRows-1, 1, 1, nbCols-2) = f.block(nbRows-1, 1, 1, nbCols-2) - b.block(nbRows-1, 1, 1, nbCols-2);
    f.block(1,0,nbRows-2, 1) = f.block(1,0,nbRows-2, 1) - b.block(1,0,nbRows-2, 1);
    f.block(1,nbCols-1,nbRows-2, 1) = f.block(1,nbCols-1,nbRows-2, 1) - b.block(1,nbCols-1,nbRows-2, 1);
    
    // Modification near the corners
    f(0,0) = f(0,0)-sqrt(2)*b(0,0);
    f(0,nbCols-1) = f(0,nbCols-1)-sqrt(2)*b(0,nbCols-1);
    f(nbRows-1,nbCols-1) = f(nbRows-1,nbCols-1)-sqrt(2)*b(nbRows-1,nbCols-1);
    f(nbRows-1,0) = f(nbRows-1,0)-sqrt(2)*b(nbRows-1,0);
}

void normalIntegration(aliceVision::image::Image<aliceVision::image::RGBfColor>& normals, aliceVision::image::Image<float>& depth, bool perspective, const Eigen::Matrix3f& K)
{
    int nbCols = normals.cols();
    int nbRows = normals.rows();

    Eigen::MatrixXf p(nbRows, nbCols);
    Eigen::MatrixXf q(nbRows, nbCols);

    Eigen::MatrixXf f(nbRows, nbCols);

    // Prepare normal integration :
    normal2PQ(normals, p, q, perspective, K);
    getDivergenceField(p, q, f);
    setBoundaryConditions(p, q, f);

    // Convert f to OpenCV matrix :
    cv::Mat f_openCV(nbRows, nbCols, CV_32FC1);
    cv::eigen2cv(f, f_openCV);

    // Cosine transform of f :
    cv::Mat fcos(nbRows, nbCols, CV_32FC1);
    cv::dct(f_openCV, fcos);

    //Cosine transform of z :
    cv::Mat z_bar_bar(nbRows, nbCols, CV_32FC1);

    for (int j = 0; j < nbCols; j++)
    {
        for (int i = 0; i < nbRows; i++)
        {
            double denom = 4*(pow(sin(0.5*M_PI*j/nbCols),2) + pow(sin(0.5*M_PI*i/nbRows),2));
            denom = std::max(denom,0.00001);
            z_bar_bar.at<float>(i,j) = fcos.at<float>(i,j)/denom;
        }
	}

    // Inverse cosine transform :
    cv::Mat z(nbRows, nbCols, CV_32FC1);
    cv::idct(z_bar_bar, z);

    for (int j = 0; j < nbCols; ++j)
    {
        for (int i = 0; i < nbRows; ++i)
        {
            if(perspective)
            {
                depth(i,j) = std::exp(z.at<float>(i,j));
            } else {
                depth(i,j) = z.at<float>(i,j);
            }
        }
    }
}
