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

void normal2PQ(const aliceVision::image::Image<aliceVision::image::RGBfColor>& normals, Eigen::MatrixXf& p, Eigen::MatrixXf& q){

    for (size_t j = 0; j < p.cols(); ++j)
    {
        for (size_t i = 0; i < p.rows(); ++i)
        {
            if (normals(i,j)(2) < 0.001)
            {
                p(i,j) = 0;
                q(i,j) = 0;
            } else
            {
                p(i,j) = normals(i,j)(1)/normals(i,j)(2);
                q(i,j) = -normals(i,j)(0)/normals(i,j)(2);
            }
        }
    }
}

void getDivergenceField(const Eigen::MatrixXf& p, const Eigen::MatrixXf& q, Eigen::MatrixXf& f){

    int nbRows = p.rows();
    int nbCols = p.cols();

    Eigen::MatrixXf px_below(nbRows, nbCols);
    px_below = p;
    px_below.block(0,0,nbRows-1,nbCols) = p.block(1,0,nbRows-1,nbCols);

    Eigen::MatrixXf px_above = p;
    px_above.block(1,0,nbRows-1,nbCols) = p.block(0,0,nbRows-1,nbCols);

    Eigen::MatrixXf px = 0.5*(px_below-px_above);

    Eigen::MatrixXf qy_right = q;
    qy_right.block(0,0,nbRows,nbCols-1) = q.block(0,1,nbRows,nbCols-1);

    Eigen::MatrixXf qy_left = q;
    qy_left.block(0,1,nbRows,nbCols-1) = q.block(0,0,nbRows,nbCols-1);

    Eigen::MatrixXf qy = 0.5*(qy_right-qy_left);

    // Div(p,q) 
    f = px+qy;
}

void setBoundaryConditions(const Eigen::MatrixXf& p, const Eigen::MatrixXf& q, Eigen::MatrixXf& f){

    int nbRows = p.rows();
    int nbCols = p.cols();

    // Right hand side of the boundary condition
    Eigen::MatrixXf b = Eigen::MatrixXf::Zero(nbRows, nbCols);
    
    b.block(0,1,1,nbCols-2) = -p.block(0,1,1,nbCols-2);
    b.block(nbRows-1, 1, 1, nbCols-2) = p.block(nbRows-1, 1, 1, nbCols-2);
    b.block(1,0,nbRows-2, 1) = -q.block(1,0,nbRows-2, 1);
    b.block(1, nbCols-1, nbRows-2, 1) = q.block(1, nbCols-1, nbRows-2, 1);
    b(0,0) = (1/sqrt(2))*(-p(0,0)-q(0,0));
    b(0,nbCols-1) = (1/sqrt(2))*(-p(0,nbCols-1)+q(0,nbCols-1));
    b(nbRows-1, nbCols-1) = (1/sqrt(2))*(p(nbRows-1, nbCols-1)+q(nbRows-1, nbCols-1));
    b(nbRows-1,0) = (1/sqrt(2))*(p(nbRows-1,0)-q(nbRows-1,0));

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

void normalIntegration(aliceVision::image::Image<aliceVision::image::RGBfColor>& normals, aliceVision::image::Image<float>& solution){

    int nbCols = normals.cols();
    int nbRows = normals.rows();
	
    Eigen::MatrixXf p(nbRows, nbCols);
    Eigen::MatrixXf q(nbRows, nbCols);

    Eigen::MatrixXf f(nbRows, nbCols);

    // Prepare normal integration :
    normal2PQ(normals, p, q);

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
            denom = std::max(denom,0.001);
            z_bar_bar.at<float>(i,j) = -fcos.at<float>(i,j)/denom;
        }
    }

    // Inverse cosine transform :
    cv::Mat z;
    cv::idct(z_bar_bar, z);

    for (int j = 0; j < nbCols; ++j)
    {
        for (int i = 0; i < nbRows; ++i)
        {
            solution(i,j) = z.at<float>(i,j);
        }
    }
}
