#include "checkerDetector.hpp"

#include <aliceVision/sfm/liealgebra.hpp>
#include <aliceVision/camera/IntrinsicBase.hpp>
#include <aliceVision/system/Logger.hpp>

#include <OpenImageIO/imagebufalgo.h>

namespace aliceVision{
namespace calibration{



bool CheckerDetector::process(const image::Image<image::RGBColor> & source)
{
    image::Image<float> grayscale;
    image::ConvertPixelType(source, &grayscale);

    double scales[] = {1.0, 0.75, 0.5, 0.25};

    std::vector<Vec2> allCorners;
    for (double scale : scales)
    {
        std::vector<Vec2> corners;
        if (!processLevel(corners, grayscale, scale))
        {
            return false;
        }

        //Merge with previous level corners
        for (Vec2 c : corners)
        {
            double distmin = 5.0;

            for (Vec2 oc : allCorners)
            {
                double dist = (oc - c).norm();
                
                if (dist < distmin)
                {
                    distmin = dist;
                    break;
                }
            }

            if (distmin < 5.0)
            {
                continue;
            } 

            allCorners.push_back(c);
        }
    }

    //Normalize image between 0 and 1
    image::Image<float> normalized;
    normalizeImage(normalized, grayscale);

    // David Fleet and Tomas Pajdla and Bernt Schiele and Tinne Tuytelaars. ROCHADE: Robust Checkerboard Advanced Detection for Camera Calibration
    std::vector<CheckerBoardCorner> fitted_corners;
    fitCorners(fitted_corners, allCorners, normalized);

    //Remove multiple points at the same position
    for (int i = 0; i < fitted_corners.size(); i++)
    {
        const CheckerBoardCorner & ci = fitted_corners[i];

        bool found = false;
        for (int j = i + 1; j < fitted_corners.size(); j++)
        {
            const CheckerBoardCorner & cj = fitted_corners[j];

            if ((ci.center - cj.center).norm() < 2.0) 
            {
                found = true;
            }
        }

        if (!found)
        {
            _corners.push_back(ci);
        }
    }

    // Andreas Geiger and Frank Moosmann and Oemer Car and Bernhard Schuster. Automatic Calibration of Range and Camera Sensors using a single Shot
    if (!buildCheckerboards(_boards, _corners, normalized))
    {
        return false;
    }

    //Try to merge together checkerboards if connected
    if (!mergeCheckerboards())
    {
        return false;
    }
    
    return true;
}

bool CheckerDetector::processLevel(std::vector<Vec2> & corners, const image::Image<float> & input, double scale) 
{       
    //Get resized size
    const unsigned int w = input.Width();
    const unsigned int h = input.Height();
    const unsigned int nw = (unsigned int)(floor(float(w) * scale));
    const unsigned int nh = (unsigned int)(floor(float(h) * scale));

    //Resize image
    image::Image<float> toUpdate = input;
    image::Image<float> rescaled(nw, nh);
    const oiio::ImageSpec imageSpecResized(nw, nh, 1, oiio::TypeDesc::FLOAT);
    const oiio::ImageSpec imageSpecOrigin(w, h, 1, oiio::TypeDesc::FLOAT);
    const oiio::ImageBuf inBuf(imageSpecOrigin, toUpdate.data());
    oiio::ImageBuf outBuf(imageSpecResized, rescaled.data());
    oiio::ImageBufAlgo::resize(outBuf, inBuf);


    //Normalize image between 0 and 1
    image::Image<float> normalized;
    normalizeImage(normalized, rescaled);

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
    // Abdulrahman S. Alturki, John S. Loomis. X-Corner Detection for Camera Calibration Using Saddle Points
    std::vector<Vec2> refined_corners;
    refineCorners(refined_corners, raw_corners, normalized);

    // Yunsu Bok, Hyowon Ha, In So Kweon. Automated checkerboard detection and indexing using circular boundaries
    pruneCorners(corners, refined_corners, normalized);
    //corners = refined_corners;

    for (Vec2 & v : corners)
    {
        v.x() /= scale;
        v.y() /= scale;
    } 

    

    return true;
}

bool CheckerDetector::pruneCorners(std::vector<Vec2> & pruned_corners, const std::vector<Vec2> & raw_corners, const image::Image<float> & input)
{
    const image::Sampler2d<image::SamplerLinear> sampler;
    const int radius = 5;
    const int samples = 50;

    float vector[samples];

    for (const Vec2 & corner : raw_corners)
    {
        float min = std::numeric_limits<float>::max();
        float max = std::numeric_limits<float>::min();

        for (int sample = 0; sample < samples; sample++)
        {
            double angle = 2.0 * M_PI * double(sample) / double(samples);

            double x = corner(0) + cos(angle) * double(radius);
            double y = corner(1) + sin(angle) * double(radius);

            vector[sample] = sampler(input, y, x);

            min = std::min(min, vector[sample]);
            max = std::max(max, vector[sample]);
        }

        for (int sample = 0; sample < samples; sample++)
        {
            vector[sample] = 2.0 * ((vector[sample] - min) / (max - min)) - 1.0;
        }

        int count = 0;
        for (int sample = 0; sample < samples; sample++)
        {
            int next = sample + 1;
            if (next == samples) next = 0;

            float test = vector[sample] * vector[next];
            if (test < 0.0)
            {
                count++;
            }
        }

        if (count != 4)
        {
            continue;
        }

        pruned_corners.push_back(corner);
    }

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
                if (norm < 0.01) continue;

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

bool CheckerDetector::fitCorners(std::vector<CheckerBoardCorner> & refined_corners, const std::vector<Vec2> & raw_corners, const image::Image<float> & input)
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
            CheckerBoardCorner c;
            c.center = corner;
            refined_corners.push_back(c);
        }
    }

    for (CheckerBoardCorner & corner : refined_corners)
    {
        bool isValid = true;

        double cx = corner.center(0);
        double cy = corner.center(1);

        AtA.fill(0);
        Atb.fill(0);

        for (int i = -radius; i <= radius; i++)
        {
            int di = corner.center(1) + i;
            for (int j = -radius; j <= radius; j++)
            {
                if (i == j) continue;
                
                int dj = corner.center(0) + j;

                Eigen::Vector<double, 6> rowA;
                rowA(0) = 1.0;
                rowA(1) = j;
                rowA(2) = i;
                rowA(3) = 2.0 * j * i;
                rowA(4) = j * j - i * i;
                rowA(5) = j * j + i * i;

                AtA += rowA * rowA.transpose();
                Atb += rowA * (2.0 * sampler(filtered, di, dj) - 1.0);
            }                
        }
        
        Eigen::Vector<double, 6> x = AtA.inverse() * Atb;    
        double c1 = x(0);
        double c2 = x(1);
        double c3 = x(2);
        double c4 = x(3);
        double c5 = x(4);
        double c6 = x(5);

        double K = sqrt(c5*c5 + c4*c4);

        double cos2phi = (-c6 / K);
        double cos2theta = (c5 / K);
        double sin2theta = (c4 / K);

        double phi = acos(cos2phi) * 0.5;
        double theta = atan2(sin2theta, cos2theta) * 0.5;

        if (theta != theta || phi != phi)
        {
            continue;
        }

        double angle = theta - phi;
        corner.dir1(0) = cos(angle);
        corner.dir1(1) = sin(angle);

        angle = theta + phi;
        corner.dir2(0) = cos(angle);
        corner.dir2(1) = sin(angle);
    } 
    
    return true;
}

IndexT CheckerDetector::findClosestCorner(const Vec2 & center, const Vec2 & dir, const std::vector<CheckerBoardCorner> & refined_corners)
{   
    IndexT ret = UndefinedIndexT;
    double min = std::numeric_limits<double>::max();
    double angle = 0.0;

    for (IndexT cid = 0; cid < refined_corners.size(); cid++)
    {
        const CheckerBoardCorner & c = refined_corners[cid];

        Vec2 diff = c.center - center;
        double dist = diff.norm();
        if (dist < 5.0)
        {
            continue;
        }
        
        if (std::abs(atan2(diff.y(), diff.x()) - atan2(dir.y(), dir.x())) > M_PI_4)
        {
            continue;
        }

        if (dist < min)
        {
            min = dist;
            ret = cid;
            angle = std::abs(atan2(diff.y(), diff.x()) - atan2(dir.y(), dir.x()));
        }
    }

    if (angle > M_PI / 8.0)
    {
        return UndefinedIndexT;
    }

    return ret;
}

bool CheckerDetector::getSeedCheckerboard(Eigen::Matrix<IndexT, -1, -1> & board, IndexT seed, const std::vector<CheckerBoardCorner> & refined_corners)
{
    IndexT right, bottom, bottom_right, check;
    const CheckerBoardCorner & referenceCorner = refined_corners[seed];

    right = findClosestCorner(referenceCorner.center, referenceCorner.dir1, refined_corners);
    if (right == UndefinedIndexT)
    {
        return false;
    }

    bottom = findClosestCorner(referenceCorner.center, referenceCorner.dir2, refined_corners);
    if (bottom == UndefinedIndexT)
    {
        return false;
    }

    const CheckerBoardCorner & bottomCorner = refined_corners[bottom];
    const CheckerBoardCorner & rightCorner = refined_corners[right];

    bottom_right = findClosestCorner(bottomCorner.center, bottomCorner.dir2, refined_corners);
    if (bottom_right == UndefinedIndexT)
    {
        return false;
    }
    
    check = findClosestCorner(rightCorner.center, -rightCorner.dir1, refined_corners);
    if (check == UndefinedIndexT)
    {
        return false;
    }

    if (check != bottom_right)
    {
        return false;
    }

    board.resize(2, 2);
    board(0, 0) = seed;
    board(0, 1) = right;
    board(1, 0) = bottom;
    board(1, 1) = bottom_right;

    return true;
}


bool CheckerDetector::getCandidates(std::vector<NewPoint> & candidates, Eigen::Matrix<IndexT, -1, -1> & board)
{   
    if (board.rows() < 2)
    {
        return false;
    }

    
    for (int col = 0; col < board.cols(); col++)
    {
        NewPoint p;
        p.col = col;
        p.row = -1;
        p.score = std::numeric_limits<double>::max();

        for (int row = 0; row < board.rows() - 2; row++)
        {
            if (board(row, col) != UndefinedIndexT)
            {
                break;
            }

            p.row = row;
        }

        if (p.row < 0) continue;
        if (board(p.row + 1, col) == UndefinedIndexT) continue;
        if (board(p.row + 2, col) == UndefinedIndexT) continue;

        candidates.push_back(p);
    }

    return true;
}


bool CheckerDetector::growIterationUp(Eigen::Matrix<IndexT, -1, -1> & board, const std::vector<CheckerBoardCorner> & refined_corners)
{
    double referenceEnergy = computeEnergy(board, refined_corners);

    //Enlarge board and fill with empty indices
    Eigen::Matrix<IndexT, -1, -1> nboard(board.rows()  + 1, board.cols());
    nboard.fill(UndefinedIndexT);
    nboard.block(1, 0, board.rows(), board.cols()) = board;
    board = nboard;


    std::unordered_set<IndexT> used;
    for (int i = 0; i < board.rows(); i++)
    {
        for (int j = 0; j < board.cols(); j++)
        {
            IndexT val = board(i, j);
            used.insert(val);
        }
    }

    std::vector<NewPoint> candidates;
    if (!getCandidates(candidates, board))
    {
        return false;
    }

    //Search for new corners
    for (auto & candidate : candidates)
    {
        IndexT rci = board(candidate.row + 2, candidate.col);
        IndexT rcj = board(candidate.row + 1, candidate.col);

        const CheckerBoardCorner & ci = refined_corners[rci];
        const CheckerBoardCorner & cj = refined_corners[rcj];

        IndexT minIndex = UndefinedIndexT;
        double minE = std::numeric_limits<double>::max();
        
        for (int id = 0; id < refined_corners.size(); id++)
        {            
            if (used.find(id) != used.end())
            {
                continue;
            }

            const CheckerBoardCorner & ck = refined_corners[id];
            
            double E = ((ci.center - cj.center) + (ck.center - cj.center)).norm() / (ck.center - ci.center).norm();
            if (E < minE)
            {
                minE = E;
                minIndex = id;
            }
        }   

        //Check if this neighboor is aimed by another candidate (keep the best in this case)
        bool alreadyTook = false;
        for (auto & candidateCheck : candidates)
        {
            IndexT idx = board(candidateCheck.row, candidateCheck.col);
            if (idx != minIndex)
            {
                continue;
            }
                
            if (candidateCheck.score < minE)
            {
                alreadyTook = true;
                continue;
            }

            board(candidateCheck.row, candidateCheck.col) = UndefinedIndexT;
            candidate.score = std::numeric_limits<double>::max();
        }

        if (alreadyTook)
        {
            continue;
        }

        board(candidate.row, candidate.col) = minIndex;
        candidate.score = minE;
    }


    if (computeEnergy(board, refined_corners) < referenceEnergy)
    {
        return true;
    }


    std::sort(candidates.begin(), candidates.end(), [](const NewPoint & p1, const NewPoint p2) { return p1.score < p2.score; });
    while (candidates.size() > 0)
    {
        //Get the worst candidate and remove it
        IndexT j = candidates.back().col;
        IndexT i = candidates.back().row;
        candidates.pop_back();
        board(i, j) = UndefinedIndexT;

        //If the next candidate is very bad, continue
        if (candidates.back().score > 1e6)
        {
            continue;
        }
        

        //After removal, some corners may be isolated
        //And we want them to disappear !
        bool changed = false;
        for (int id = 0; id < candidates.size(); id++)
        {
            int ni = candidates[id].row;
            int nj = candidates[id].col;

            int countH = 0;
            if (nj - 1 >= 0)
            {
                if (board(ni, nj - 1) != UndefinedIndexT)
                {
                    countH++;
                }
            }

            if (nj + 1 < board.cols())
            {
                if (board(ni, nj + 1) != UndefinedIndexT)
                {
                    countH++;
                }
            }

            if (countH == 0)
            {
                candidates[id].score = std::numeric_limits<double>::max();
                changed = true;
            }
        }
        
        if (changed)
        {
            std::sort(candidates.begin(), candidates.end(), [](const NewPoint & p1, const NewPoint p2) { return p1.score < p2.score; });
            continue;
        }

        double E = computeEnergy(board, refined_corners);
        if (E < referenceEnergy)
        {
            return true;
        }
    }


    return false;
}

double CheckerDetector::computeEnergy(Eigen::Matrix<IndexT, -1, -1> & board, const std::vector<CheckerBoardCorner> & refined_corners)
{   
    size_t countValid = 0;
    for (int i = 0; i < board.rows(); i++)
    {
        for (int j = 0; j < board.cols(); j++)
        {
            IndexT id = board(i, j);
            if (id != UndefinedIndexT)
            {
                countValid++;
            }
        }
    }

    double maxE = 0;

    if (board.cols() > 2)
    {
        for (int i = 0; i < board.rows(); i++)
        {
            for (int j = 0; j < board.cols() - 2; j++)
            {
                IndexT id1 = board(i, j);
                IndexT id2 = board(i, j + 1);
                IndexT id3 = board(i, j + 2);

                if (id1 == UndefinedIndexT || id2 == UndefinedIndexT || id3 == UndefinedIndexT)
                {
                    continue;
                }

                const CheckerBoardCorner & ci = refined_corners[id1];
                const CheckerBoardCorner & cj = refined_corners[id2];
                const CheckerBoardCorner & ck = refined_corners[id3];

                double E = ((ci.center - cj.center) + (ck.center - cj.center)).norm() / (ck.center - ci.center).norm();
                if (E > maxE)
                {
                    maxE = E;
                }
            }

            for (int j = 2; j < board.cols(); j++)
            {
                IndexT id1 = board(i, j);
                IndexT id2 = board(i, j - 1);
                IndexT id3 = board(i, j - 2);

                if (id1 == UndefinedIndexT || id2 == UndefinedIndexT || id3 == UndefinedIndexT)
                {
                    continue;
                }

                const CheckerBoardCorner & ci = refined_corners[id1];
                const CheckerBoardCorner & cj = refined_corners[id2];
                const CheckerBoardCorner & ck = refined_corners[id3];

                double E = ((ci.center - cj.center) + (ck.center - cj.center)).norm() / (ck.center - ci.center).norm();
                if (E > maxE)
                {
                    maxE = E;
                }
            }
        }
    }

    if (board.rows() > 2)
    {
        for (int i = 0; i < board.rows() - 2; i++)
        {
            for (int j = 0; j < board.cols(); j++)
            {
                IndexT id1 = board(i, j);
                IndexT id2 = board(i + 1, j);
                IndexT id3 = board(i + 2, j);

                if (id1 == UndefinedIndexT || id2 == UndefinedIndexT || id3 == UndefinedIndexT)
                {
                    continue;
                }

                const CheckerBoardCorner & ci = refined_corners[id1];
                const CheckerBoardCorner & cj = refined_corners[id2];
                const CheckerBoardCorner & ck = refined_corners[id3];

                double E = ((ci.center - cj.center) + (ck.center - cj.center)).norm() / (ck.center - ci.center).norm();
                if (E > maxE)
                {
                    maxE = E;
                }
            }
        }

        for (int i = 2; i < board.rows(); i++)
        {
            for (int j = 0; j < board.cols(); j++)
            {
                IndexT id1 = board(i - 0, j);
                IndexT id2 = board(i - 1, j);
                IndexT id3 = board(i - 2, j);

                if (id1 == UndefinedIndexT || id2 == UndefinedIndexT || id3 == UndefinedIndexT)
                {
                    continue;
                }

                const CheckerBoardCorner & ci = refined_corners[id1];
                const CheckerBoardCorner & cj = refined_corners[id2];
                const CheckerBoardCorner & ck = refined_corners[id3];

                double E = ((ci.center - cj.center) + (ck.center - cj.center)).norm() / (ck.center - ci.center).norm();
                if (E > maxE)
                {
                    maxE = E;
                }
            }
        }
    }

    return -double(countValid) + double(countValid) * maxE;
}

bool CheckerDetector::growIteration(Eigen::Matrix<IndexT, -1, -1> & board, const std::vector<CheckerBoardCorner> & refined_corners)
{
    if (board.rows() < 2) return false;
    if (board.cols() < 2) return false;


    double originalE = computeEnergy(board, refined_corners);
    double minE = std::numeric_limits<double>::max();

    Eigen::Matrix<IndexT, -1, -1> board_original = board;

    Eigen::Matrix<IndexT, -1, -1> board_up = board_original;
    if (growIterationUp(board_up, refined_corners))
    {
        double E = computeEnergy(board_up, refined_corners);
        if (E < minE)
        {
            board = board_up;
            minE = E;
        }
    }

    Eigen::Matrix<IndexT, -1, -1> board_down = board_original.colwise().reverse();
    if (growIterationUp(board_down, refined_corners))
    {
        double E = computeEnergy(board_down, refined_corners);
        if (E < minE)
        {
            board = board_down.colwise().reverse();
            minE = E;
        }
    }

    Eigen::Matrix<IndexT, -1, -1> board_right = board_original.transpose().colwise().reverse();
    if (growIterationUp(board_right, refined_corners))
    {
        double E = computeEnergy(board_right, refined_corners);
        if (E < minE)
        {
            board = board_right.colwise().reverse().transpose();
            minE = E;
        }
    }

    Eigen::Matrix<IndexT, -1, -1> board_left = board_original.transpose();
    if (growIterationUp(board_left, refined_corners))
    {
        double E = computeEnergy(board_left, refined_corners);
        if (E < minE)
        {
            board = board_left.transpose();
            minE = E;
        }
    }

    if (minE < originalE)
    {
        return true;
    }

    board = board_original;

    return false;
}

bool CheckerDetector::buildCheckerboards(std::vector<CheckerBoard> & boards, const std::vector<CheckerBoardCorner> & refined_corners, const image::Image<float> & input)
{
    double minE = std::numeric_limits<double>::max();


    std::vector<bool> used(refined_corners.size(), false);
    for (IndexT cid = 0; cid < refined_corners.size(); cid++)
    {
        const CheckerBoardCorner & seed = refined_corners[cid];
        if (used[cid])
        {
            continue;
        }
        
        Eigen::Matrix<IndexT, -1, -1> board;
        if (!getSeedCheckerboard(board, cid, refined_corners))
        {
            continue;
        }

        bool valid = true;
        for (int i = 0; i < board.rows(); i++)
        {
            for (int j = 0; j < board.cols(); j++)
            {
                if (used[board(i, j)])
                {
                    valid = false;
                }
            }
        }

        if (!valid) 
        {
            continue;
        }

        while (growIteration(board, refined_corners))
        {
        }


        int count = 0;
        for (int i = 0; i < board.rows(); i++)
        {
            for (int j = 0; j < board.cols(); j++)
            {
                if (board(i, j) == UndefinedIndexT) continue;
                count++;
            }
        }
        
        if (count < 10) continue;

        for (int i = 0; i < board.rows(); i++)
        {
            for (int j = 0; j < board.cols(); j++)
            {
                if (board(i, j) == UndefinedIndexT) continue;

                IndexT id = board(i, j);
                used[id] = true;
            }
        }
        
        if (computeEnergy(board, refined_corners) / double(count) > -0.8)
        {
            continue;
        }
        
        boards.push_back(board);
    }


    return true;
}

void CheckerDetector::drawCheckerBoard(image::Image<image::RGBColor> & img)
{
    for (auto c : _corners)
    {
        image::DrawLine(c.center.x() + 2.0, c.center.y(), c.center.x() - 2.0, c.center.y(), image::RGBColor(255,255,0), &img);
        image::DrawLine(c.center.x(), c.center.y() + 2.0, c.center.x(), c.center.y()- 2.0, image::RGBColor(255,255,0), &img);
    }

    for (auto board : _boards)
    {
        for (int i = 0; i < board.rows(); i++)
        {
            for (int j = 0; j < board.cols() - 1; j++)
            {
                IndexT p1 = board(i, j);
                IndexT p2 = board(i, j + 1);

                if (p1 == UndefinedIndexT || p2 == UndefinedIndexT)
                {
                    continue;
                }

                const CheckerBoardCorner & c1 = _corners[p1];
                const CheckerBoardCorner & c2 = _corners[p2];

                image::DrawLineThickness(c1.center.x(), c1.center.y(), c2.center.x(), c2.center.y(), image::RGBColor(255,0,0), 5, &img);
            }
        }

        for (int i = 0; i < board.rows() - 1; i++)
        {
            for (int j = 0; j < board.cols(); j++)
            {
                IndexT p1 = board(i, j);
                IndexT p2 = board(i + 1, j);

                if (p1 == UndefinedIndexT || p2 == UndefinedIndexT)
                {
                    continue;
                }

                const CheckerBoardCorner & c1 = _corners[p1];
                const CheckerBoardCorner & c2 = _corners[p2];

                image::DrawLineThickness(c1.center.x(), c1.center.y(), c2.center.x(), c2.center.y(), image::RGBColor(255,0,0), 5, &img);
            }
        }
    }
}

bool CheckerDetector::mergeCheckerboards()
{
    using CheckerBoardWithScore = std::pair<CheckerBoard, double>;
    std::vector<CheckerBoardWithScore> checkers;

    if (_boards.size() <= 1) 
    {
        return true;
    }

    for (auto b : _boards)
    {
        CheckerBoardWithScore cbws;
        cbws.first = b;
        cbws.second = computeEnergy(b, _corners);
        checkers.push_back(cbws);
    }

    bool hadMerged;
    do 
    {
        hadMerged = false;
        std::sort(checkers.begin(), checkers.end(), [](const CheckerBoardWithScore & cb1, const CheckerBoardWithScore & cb2) { return cb1.second < cb2.second; } );

        for (int idRef = 0; idRef < checkers.size(); idRef++)
        {
            CheckerBoard baseBoard = checkers[idRef].first;

            //Build dictionnary of corners for faster lookup
            std::unordered_map<IndexT, Vec2i> baseCorners;
            for (int i = 0; i < baseBoard.rows(); i++)
            {
                for (int j = 0; j < baseBoard.cols(); j++)
                {
                    if (baseBoard(i, j) != UndefinedIndexT)
                    {
                        baseCorners[baseBoard(i, j)] = Vec2i(j, i);
                    }
                }
            }

            for (int idCur = idRef + 1; idCur < checkers.size(); idCur++)
            {
                //Find a common corner
                const CheckerBoard & currentBoard = checkers[idCur].first;

                bool foundCommon = false;
                Vec2i coordsRef;
                Vec2i coordsCur;

                for (int i = 0; i < currentBoard.rows(); i++)
                {
                    for (int j = 0; j < currentBoard.cols(); j++)
                    {
                        if (currentBoard(i, j) == UndefinedIndexT)
                        {
                            continue;
                        }

                        auto lookup = baseCorners.find(currentBoard(i, j));
                        if (lookup == baseCorners.end())
                        {
                            continue;
                        }

                        foundCommon = true;
                        coordsRef = lookup->second;
                        coordsCur = Vec2i(j, i);
                        break;
                    }

                    if (foundCommon) 
                    {
                        break;
                    }
                }
                
                if (!foundCommon) 
                {
                    continue;
                }

                Vec2i offset = coordsRef - coordsCur;
                
                int right = std::max(baseBoard.cols() - 1, currentBoard.cols() + offset.x() - 1);
                int bottom = std::max(baseBoard.rows() - 1, currentBoard.rows() + offset.y() - 1);
                int left = std::min(0, offset.x());
                int top = std::min(0, offset.y());
                int nwidth = right - left + 1;
                int nheight = bottom - top + 1;

                int shiftRefX = std::max(0, -offset.x());
                int shiftRefY = std::max(0, -offset.y());

                int shiftCurX = std::max(0, offset.x());
                int shiftCurY = std::max(0, offset.y());

                CheckerBoard newBoard(nheight, nwidth);
                newBoard.fill(UndefinedIndexT);
                newBoard.block(shiftRefY, shiftRefX, baseBoard.rows(), baseBoard.cols()) = baseBoard;
                
                bool hasConflict = false;
                for (int i = 0; i < currentBoard.rows(); i++)
                {
                    for (int j = 0; j < currentBoard.cols(); j++)
                    {   
                        IndexT curval = currentBoard(i, j);
                        if (curval == UndefinedIndexT)
                        {
                            continue;
                        }

                        int rx = j + shiftCurX;
                        int ry = i + shiftCurY;

                        IndexT compare = newBoard(ry, rx);
                        if (compare != UndefinedIndexT && compare != curval)
                        {
                            hasConflict = true;
                            break;
                        }

                        newBoard(ry, rx) = curval;
                    }
                }

                if (hasConflict)
                {
                    continue;
                }

                double newEnergy = computeEnergy(newBoard, _corners);
                if (newEnergy < checkers[idRef].second)
                {
                    hadMerged = true;
                    
                    checkers[idRef].first = newBoard;
                    checkers[idRef].second = newEnergy;
                    checkers.erase(checkers.begin() + idCur);

                    break;
                }
            }

            if (hadMerged) 
            {
                break;
            }
        }

    }
    while (hadMerged);

    
    //Copy result
    _boards.clear();
    for (auto b : checkers)
    {
        _boards.push_back(b.first);
    }

    return true;
}


}//namespace calibration
}//namespace aliceVision
