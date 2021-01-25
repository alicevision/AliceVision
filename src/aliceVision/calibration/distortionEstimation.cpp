#include "distortionEstimation.hpp"

#include <ceres/ceres.h>

using namespace aliceVision;

namespace aliceVision{
namespace calibration{

class CostLine : public ceres::CostFunction
{
public:
    CostLine(std::shared_ptr<camera::Pinhole> & camera, const Vec2& pt)
        : _pt(pt)
        , _camera(camera)
    {

        set_num_residuals(1);

        mutable_parameter_block_sizes()->push_back(1);
        mutable_parameter_block_sizes()->push_back(1);
        mutable_parameter_block_sizes()->push_back(2);
        mutable_parameter_block_sizes()->push_back(camera->getDistortionParams().size());
    }

    bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const override
    {

        const double* parameter_angle_line = parameters[0];
        const double* parameter_dist_line = parameters[1];
        const double* parameter_center = parameters[2];
        const double* parameter_disto = parameters[3];

        double angle = parameter_angle_line[0];
        double distanceToLine = parameter_dist_line[0];

        double cangle = cos(angle);
        double sangle = sin(angle);

        int distortionSize = _camera->getDistortionParams().size();

        //Read parameters and update camera
        _camera->setOffset(parameter_center[0], parameter_center[1]);
        std::vector<double> cameraDistortionParams = _camera->getDistortionParams();

        for (int idParam = 0; idParam < distortionSize; idParam++)
        {
            cameraDistortionParams[idParam] = parameter_disto[idParam];
        }
        _camera->setDistortionParams(cameraDistortionParams);


        //Estimate measure
        Vec2 cpt = _camera->ima2cam(_pt);
        Vec2 distorted = _camera->addDistortion(cpt);

        residuals[0] = cangle * distorted.x() + sangle * distorted.y() - distanceToLine;

        if(jacobians == nullptr)
        {
            return true;
        }

        if(jacobians[0] != nullptr)
        {
            Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> J(jacobians[0]);

            J(0, 0) = distorted.x() * -sangle + distorted.y() * cangle;
        }

        if(jacobians[1] != nullptr)
        {
            Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> J(jacobians[1]);
            J(0, 0) = -1.0;
        }

        if(jacobians[2] != nullptr)
        {
            Eigen::Map<Eigen::Matrix<double, 1, 2, Eigen::RowMajor>> J(jacobians[2]);

            Eigen::Matrix<double, 1, 2> Jline;
            Jline(0, 0) = cangle;
            Jline(0, 1) = sangle;

            J = Jline * _camera->getDerivativeAddDistoWrtPt(cpt) * _camera->getDerivativeIma2CamWrtPrincipalPoint();
        }

        if(jacobians[3] != nullptr)
        {
            Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> J(jacobians[3], 1, distortionSize);

            Eigen::Matrix<double, 1, 2> Jline;
            Jline(0, 0) = cangle;
            Jline(0, 1) = sangle;

            J = Jline * _camera->getDerivativeAddDistoWrtDisto(cpt);
        }

        return true;
    }

private:
    std::shared_ptr<camera::Pinhole> _camera;
    Vec2 _pt;
};

bool estimate(std::shared_ptr<camera::Pinhole> & cameraToEstimate, std::vector<LineWithPoints> & lines)
{
    if (!cameraToEstimate)
    {
        return false; 
    }

    if (lines.size() == 0)
    {
        return false;
    }

    ceres::Problem problem;
    ceres::LossFunction* lossFunction = nullptr;

    std::vector<double> params = cameraToEstimate->getParams();
    
    double * center = &params[1];
    double * distortionParameters = &params[3];

    problem.AddParameterBlock(center, 2);
    problem.AddParameterBlock(distortionParameters, cameraToEstimate->getDistortionParams().size());

    
    for (auto & l : lines)
    {
        problem.AddParameterBlock(&l.angle, 1);
        problem.AddParameterBlock(&l.dist, 1);

        for (Vec2 pt : l.points)
        {
            ceres::CostFunction * costFunction = new CostLine(cameraToEstimate, pt);   
            problem.AddResidualBlock(costFunction, lossFunction, &l.angle, &l.dist, center, distortionParameters);
        }
    }

    ceres::Solver::Options options;
    options.use_inner_iterations = false;
    options.max_num_iterations = 100;

    ceres::Solver::Summary summary;  
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.FullReport() << std::endl;

    if (!summary.IsSolutionUsable())
    {
        return false;
    }

    
    cameraToEstimate->updateFromParams(params);

    return true;
}

}//namespace calibration
}//namespace aliceVision