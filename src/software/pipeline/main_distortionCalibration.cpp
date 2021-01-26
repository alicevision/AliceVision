/*Command line parameters*/
#include <Eigen/Dense>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <sstream>

#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/system/Timer.hpp>

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

#include <aliceVision/image/all.hpp>
#include <OpenImageIO/imagebufalgo.h>

#include <opencv2/opencv.hpp>

#include <aliceVision/calibration/distortionEstimation.hpp>

#include "libcbdetect/boards_from_corners.h"
#include "libcbdetect/config.h"
#include "libcbdetect/find_corners.h"
#include "libcbdetect/plot_boards.h"
#include "libcbdetect/plot_corners.h"
#include <chrono>
#include <opencv2/opencv.hpp>
#include <vector>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 0
#define ALICEVISION_SOFTWARE_VERSION_MINOR 1

namespace po = boost::program_options;

using namespace aliceVision;



struct EdgeInformation 
{
    int x;
    int y;
    float cosAngle;
    float sinAngle;
};

struct EdgeStatistics
{
    unsigned int neighboorhoodSize;
    double crossCorrelation;
};

struct Line
{
    double angleDegree;
    double distance;
    std::vector<EdgeInformation> edges;
};

struct Segment : Line
{
    Vec2 start;
    Vec2 end;
};



image::Image<float> distort(const image::Image<float> & source) 
{
    double w = source.Width();
    double h = source.Height();
    double d = sqrt(w*w + h*h);

    camera::PinholeRadialK1 radial(source.Width(), source.Height(), d, source.Width() / 2 + 10, source.Height() / 2 - 20, 0.8);

    double minx = source.Width();
    double maxx = 0;
    double miny = source.Height();
    double maxy = 0;

    for (int i = 0; i < source.Height(); i++) 
    {
        for (int j = 0; j < source.Width(); j++)
        {
            Vec2 pos = radial.get_d_pixel(Vec2(j, i));
            minx = std::min(minx, pos.x());
            maxx = std::max(maxx, pos.x());
            miny = std::min(miny, pos.y());
            maxy = std::max(maxy, pos.y());
        }
    }

    int width = maxx - minx + 1;
    int height = maxy - miny + 1;

    std::cout << width << " " << height << std::endl;

    const image::Sampler2d<image::SamplerLinear> sampler;
    image::Image<float> result(width, height, true, 1.0f);

    for (int i = 0; i < result.Height(); i++) 
    {
        double y = miny + double(i);

        for (int j = 0; j < result.Width(); j++)
        {
            double x = minx + double(j);

            Vec2 pos(x, y);
            Vec2 undist = radial.get_ud_pixel(pos);

            if (undist.x() < 0 || undist.x() >= source.Width()) continue;
            if (undist.y() < 0 || undist.y() >= source.Height()) continue;
            
            
            result(i, j) = sampler(source, undist(1), undist(0));
        }
    }

    return result;
}

cv::Mat undistort(const std::shared_ptr<camera::Pinhole> & camera, const cv::Mat & source) 
{
    double minx = source.cols;
    double maxx = 0;
    double miny = source.rows;
    double maxy = 0;

    for (int i = 0; i < source.rows; i++) 
    {
        for (int j = 0; j < source.cols; j++)
        {
            Vec2 pos = camera->get_d_pixel(Vec2(j, i));
            minx = std::min(minx, pos.x());
            maxx = std::max(maxx, pos.x());
            miny = std::min(miny, pos.y());
            maxy = std::max(maxy, pos.y());
        }
    }

    int width = maxx - minx + 1;
    int height = maxy - miny + 1;

    std::cout << minx << " " << miny << std::endl;
    std::cout << "size:" << width << " " << height << std::endl;

    cv::Mat result(height, width, CV_32FC1);

    for (int i = 0; i < height; i++) 
    {
        double y = miny + double(i);

        for (int j = 0; j < width; j++)
        {
            double x = minx + double(j);

            Vec2 pos(x, y);
            Vec2 dist = camera->get_ud_pixel(pos);

            if (dist.x() < 0 || dist.x() >= source.cols)
            {
                continue;
            } 
            if (dist.y() < 0 || dist.y() >= source.rows) continue;
            
            cv::Mat patch;
            cv::getRectSubPix(source, cv::Size(1,1), cv::Point2d(dist.x(), dist.y()), patch);
            result.at<float>(i, j) = patch.at<float>(0, 0) * 255.0f;
        }
    }

    return result;
}


double unsignedAngleDistance(double angle1, double angle2) 
{
    double diff = std::abs(angle1 - angle2);
    while (diff > 360.0) diff -= 360.0;
    if (diff > 180.0) diff = 360 - diff;

    return diff;
}

bool preprocessImage(cv::Mat & preprocessed, const cv::Mat & input)
{
    cv::Mat filtered;
    cv::GaussianBlur(input, filtered, cv::Size(5, 5), 2.0, 2.0);

    cv::Mat converted(input.rows, input.cols, CV_8UC1);
    filtered.convertTo(converted, CV_8UC1, 255);

    /*cv::Ptr<cv::CLAHE> app = cv::createCLAHE();
    app->apply(converted, preprocessed);*/
    preprocessed = converted;

    cv::imwrite("distorted.png", preprocessed);

    return true;
}

bool buildEdgesList(std::vector<EdgeInformation> & edgeInformations, const cv::Mat & input)
{
    // Build gradient images using a simple sobel
    cv::Mat gradientX, gradientY;
    cv::Sobel(input, gradientX, CV_32FC1, 1, 0);
    cv::Sobel(input, gradientY, CV_32FC1, 0, 1);

    // Compute the gradient norm for each pixel
    cv::Mat gradientNorm(input.rows, input.cols, CV_32FC1);
    gradientNorm = gradientX.mul(gradientX) + gradientY.mul(gradientY);
    gradientNorm.forEach<float>([](float &p, const int * position) -> void { p = sqrt(p); } );

    // Compute Histogram for gradient norm
    cv::Mat hist;
    int dimensions = 512;
    int channels[] = {0};
    const float gradientRange[] = { 0.0f, 512.0f };
    const float * rangeValues[] = { gradientRange };
    cv::calcHist(&gradientNorm, 1, channels, cv::Mat(), hist, 1, &dimensions, rangeValues, true, false); 

    // Use this histogram to compute statistics for canny thresholds
    size_t totalSize = input.rows * input.cols;
    double lowQuantile = 0.7 * double(totalSize);
    double highQuantile = 0.8 * double(totalSize);

    //Estimate low threshold
    int lowPos = 0;
    int highPos = 0;
    double sum = 0.0;
    for (int i = 0; i < dimensions; i++)
    {
        sum += hist.at<float>(i, 0);
        if (sum > lowQuantile) 
        {
            lowPos = i;
            break;
        }
    }

    //Estimate high threshold
    sum = 0.0;
    for (int i = 0; i < dimensions; i++)
    {
        sum += hist.at<float>(i, 0);
        if (sum > highQuantile) 
        {
            highPos = i;
            break;
        }
    }

    // We now have low and high threshold. Compute canny.
    cv::Mat edges;
    cv::Canny(input, edges, lowPos, highPos);

    // Build list of edges
    for (int i = 0; i < edges.rows; i++) 
    {
        for (int j = 0; j < edges.cols; j++)
        {
            if (!edges.at<uint8_t>(i, j))
            {
                continue;
            }

            float norm = gradientNorm.at<float>(i, j);

            EdgeInformation ei;
            ei.x = j; 
            ei.y = i;
            ei.cosAngle = (norm < 1e-6)?0.0:gradientX.at<float>(i, j) / norm;
            ei.sinAngle = (norm < 1e-6)?0.0:gradientY.at<float>(i, j) / norm;
            edgeInformations.push_back(ei);
        }
    }

    return true;
}

bool computeStatistics(std::vector<EdgeStatistics> & edgeStatistics, const std::vector<EdgeInformation> & edgeInformations, size_t width, size_t height)
{
    const int radiusLookup = 4;

    // prepare output
    edgeStatistics.resize(edgeInformations.size());
    for (auto & edgeStatistic : edgeStatistics)
    {
        edgeStatistic.crossCorrelation = 0.0;
        edgeStatistic.neighboorhoodSize = 0;
    }

    // Build spatial map of edges
    cv::Mat indices(height, width, CV_32SC1, cv::Scalar(-1));
    for (int id = 0; id < edgeInformations.size(); id++) 
    {
        indices.at<int>(edgeInformations[id].y, edgeInformations[id].x) = id;
    }

    // Compute statistics
    for (int i = radiusLookup; i < indices.rows - radiusLookup; i++) 
    {
        for (int j = radiusLookup; j < indices.cols - radiusLookup; j++)
        {
            int indexCurrent = indices.at<int>(i, j);
            if (indexCurrent < 0)
            {
                continue;
            }

            const EdgeInformation & eiCurrent = edgeInformations[indexCurrent];
            EdgeStatistics & esCurrent = edgeStatistics[indexCurrent];

            esCurrent.neighboorhoodSize = 0;
            esCurrent.crossCorrelation = 0.0f;

            for (int k = -radiusLookup; k <= radiusLookup; k++)
            {
                int y = i + k;

                for (int l = -radiusLookup; l <= radiusLookup; l++)
                {
                    int x = j + l;

                    int indexOther = indices.at<int>(y, x);
                    if (indexOther < 0)
                    {
                        continue;
                    }

                    if (indexOther == indexCurrent) 
                    {
                        continue;
                    }

                    const EdgeInformation & eiOther = edgeInformations[indexOther];

                    esCurrent.neighboorhoodSize++;
                    esCurrent.crossCorrelation += eiCurrent.cosAngle * eiOther.cosAngle + eiCurrent.sinAngle * eiOther.sinAngle;
                }
            }
        }
    }

    return true;
}

bool filterEdgesListStep(std::vector<EdgeInformation> & edgeInformationsFiltered, std::vector<EdgeStatistics> & edgeStatisticsFiltered, const std::vector<EdgeStatistics> & edgeStatistics, const std::vector<EdgeInformation> & edgeInformations, bool withAngle)
{
    for (int index = 0; index < edgeStatistics.size(); index++)
    {
        const EdgeStatistics & es = edgeStatistics[index];
        if (es.neighboorhoodSize < 4) 
        {
            continue;
        }

        double cc = es.crossCorrelation / float(es.neighboorhoodSize);
        if (cc < 0.95) 
        {
            continue;
        }

        edgeInformationsFiltered.push_back(edgeInformations[index]);
        edgeStatisticsFiltered.push_back(edgeStatistics[index]);
    }

    return true;
}

bool nonMaximalSuppression(std::vector<EdgeInformation> & edgeInformations, std::vector<EdgeStatistics> & edgeStatistics, size_t width, size_t height)
{
    const int minDistance = 1;

    std::vector<EdgeInformation> returned;

    // Build spatial map of edges
    cv::Mat indices(height, width, CV_32SC1, cv::Scalar(-1));
    for (int id = 0; id < edgeInformations.size(); id++) 
    {
        indices.at<int>(edgeInformations[id].y, edgeInformations[id].x) = id;
    }


    // Non maximal suppression with respect to crosscorrelation
    for (int i = minDistance; i < indices.rows - minDistance; i++) 
    {
        for (int j = minDistance; j < indices.cols - minDistance; j++)
        {
            int indexCurrent = indices.at<int>(i, j);
            if (indexCurrent < 0)
            {
                continue;
            } 

            EdgeInformation & eiCurrent = edgeInformations[indexCurrent];
            EdgeStatistics & esCurrent = edgeStatistics[indexCurrent];

            float max = 0.0f;
            for (int k = -minDistance; k <= minDistance; k++)
            {
                int y = i + k;

                for (int l = -minDistance; l <= minDistance; l++)
                {
                    int x = j + l;

                    int indexOther = indices.at<int>(y, x);
                    if (indexOther < 0)
                    {
                        continue;
                    }

                    if (indexOther == indexCurrent) 
                    {
                        continue;
                    }

                    EdgeStatistics & eiOther = edgeStatistics[indexOther];
                    if (eiOther.crossCorrelation > max) 
                    {
                        max = eiOther.crossCorrelation;
                    }
                }
            }

            if (esCurrent.crossCorrelation < max) 
            {
                continue;
            }

            returned.push_back(eiCurrent);
        }
    }

    edgeInformations = returned;

    return true;
}

bool filterEdgesList(std::vector<EdgeInformation> & edgeInformations, size_t width, size_t height)
{   
    bool first = true;
    size_t previousSize = edgeInformations.size();
    std::vector<EdgeStatistics> edgeStatistics;
    
    while (1)
    {
        if (!computeStatistics(edgeStatistics, edgeInformations, width, height)) 
        {
            return false;
        }

        std::vector<EdgeInformation> edgeInformationsFiltered;
        std::vector<EdgeStatistics> edgeStatisticsFiltered;
        if (!filterEdgesListStep(edgeInformationsFiltered, edgeStatisticsFiltered, edgeStatistics, edgeInformations, first)) 
        {
            return false;
        }

        edgeInformations = edgeInformationsFiltered;
        edgeStatistics = edgeStatisticsFiltered;
        if (edgeInformations.size() == previousSize) 
        {
            break;
        }

        previousSize = edgeInformations.size();
        first = false;
    }   


    if (!nonMaximalSuppression(edgeInformations, edgeStatistics, width, height)) 
    {
        return false;
    }

    cv::Mat res(height, width, CV_8UC1, cv::Scalar(0));
    for (auto edge : edgeInformations) 
    {
        res.at<uchar>(edge.y, edge.x) = 255;
    }

    cv::imwrite("/home/servantf/edges.png",res);

    return true;
}

bool computeHoughLines(std::vector<Line> & lines, const std::vector<EdgeInformation> & edgeInformations, size_t width, size_t height)
{
    const double stepDistance = 1.0;
    const double stepAngle = 0.2;
    const double maxAngleEdgeToLine = 2.0;
    const double maxDistanceEdgeToLine = 3.0;
    const double minAngleBetweenLines = 2;
    const double minDistanceBetweenLines = 20;
    const double minRawLineSize = 10;
    const double minScore = minRawLineSize / (1.0 + maxDistanceEdgeToLine);
    const int minEdgePerLine = 20;

    double hwidth = double(width) / 2.0;    
    double hheight = double(height) / 2.0;
    double minAngle = 0.0;
    double maxAngle = 360.0;    
    
    double mindistance = round(-sqrt(hwidth * hwidth + hheight * hheight));
    double maxdistance = round(sqrt(hwidth * hwidth + hheight * hheight));    

    int countBinsAngle = int((maxAngle - minAngle) / stepAngle);
    int countBinsDistance = int((maxdistance - mindistance) / stepDistance);

    struct binInfo
    {
        double angle;
        double distance;
        double score;
    };

    //Initialize all bins for voting
    std::vector<binInfo> bins(countBinsAngle * countBinsDistance);
    for (int idAngle = 0; idAngle < countBinsAngle; idAngle++) 
    {
        for (int idDistance = 0; idDistance < countBinsDistance; idDistance++) 
        {
            bins[idAngle * countBinsDistance + idDistance].angle = minAngle + double(idAngle) * stepAngle;
            bins[idAngle * countBinsDistance + idDistance].distance = mindistance + double(idDistance) * stepDistance;
            bins[idAngle * countBinsDistance + idDistance].score = 0;
        }
    }

    // Loop over edges to assign them to bins
    for (const EdgeInformation & einfo : edgeInformations) 
    {
        double edgeAngle = radianToDegree(atan2(einfo.sinAngle, einfo.cosAngle));
        double edgeAngleMinBound = edgeAngle - maxAngleEdgeToLine;
        double edgeAngleMaxBound = edgeAngle + maxAngleEdgeToLine;
        int binAngleEdgeMinBound = int(edgeAngleMinBound / stepAngle);
        int binAngleEdgeMaxBound = int(edgeAngleMaxBound / stepAngle);



        //Loop over possible angles
        for (int binAngle = binAngleEdgeMinBound; binAngle <= binAngleEdgeMaxBound; binAngle++) 
        {
            int correctedBinAngle = binAngle;
            if (binAngle >= countBinsAngle)
            {
                correctedBinAngle = binAngle - countBinsAngle;
            }

            if (binAngle < 0)
            {
                correctedBinAngle = binAngle + countBinsAngle;
            }

            double curAngle = (double(correctedBinAngle) * stepAngle) * M_PI / 180.0;
            double sinAngle = sin(curAngle);
            double cosAngle = cos(curAngle);

            

            double x = einfo.x - hwidth;
            double y = einfo.y - hheight;

            double md = cosAngle * x + sinAngle * y;


            int distanceMinBound = md - minDistanceBetweenLines;
            int distanceMaxBound = md + minDistanceBetweenLines;

            int binDistanceEdgeMinBound = int(round((distanceMinBound - mindistance) / stepDistance));
            int binDistanceEdgeMaxBound = int(round((distanceMaxBound - mindistance) / stepDistance));

            // Loop over possible distances for this angle 
            for (int binDistance = std::max(0, binAngleEdgeMinBound); binDistance <= std::min(countBinsDistance, binDistanceEdgeMaxBound); binDistance++)
            {
                double distance = mindistance + double(binDistance) * stepDistance;
                double error = fabs(md - distance);

                //Assign a score inversely proportionnal to this error
                bins[correctedBinAngle * countBinsDistance + binDistance].score += 1.0 / (1.0 + error);
            }
        }
    }

    

    // Sort bins by decreasing score
    std::sort(bins.begin(), bins.end(), [](const binInfo &a, const binInfo &b) { return a.score > b.score; } );

    // Add lines and make sure we don't have too much duplicates
    std::vector<Line> added;
    for (int id = 0; id < bins.size(); id++)
    {
        if (bins[id].score < minScore)
        {
            break;
        }

        bool found = false;
        for (const Line & prev : added) 
        {
            double diff = unsignedAngleDistance(bins[id].angle, prev.angleDegree);
            double diffd = std::abs(bins[id].distance - prev.distance);

            if (diff < minAngleBetweenLines && diffd < minDistanceBetweenLines)
            {
                found = true;
            }

            //Remove discrimation between opposed normals
            diff = fabs(unsignedAngleDistance(bins[id].angle, prev.angleDegree) - 180.0);
            diffd = std::abs(bins[id].distance + prev.distance);
            if (diff < minAngleBetweenLines && diffd < minDistanceBetweenLines)
            {
                found = true;
            }
        }


        if (found) 
        {
            continue;
        }

        Line l;
        l.angleDegree = bins[id].angle;
        l.distance = bins[id].distance;

        added.push_back(l);
    }


    // Attach measured edges to those lines
    lines.clear();
    for (Line & l : added) 
    {
        double lineAngleRadian = degreeToRadian(l.angleDegree);
        double coslineAngleRadian = cos(lineAngleRadian);
        double sinlineAngleRadian = sin(lineAngleRadian);

        l.edges.clear();

        for (const EdgeInformation & einfo : edgeInformations) 
        {
            double edgeAngle = radianToDegree(atan2(einfo.sinAngle, einfo.cosAngle));
            double lineAngle = radianToDegree(atan2(sinlineAngleRadian, coslineAngleRadian));

            double cosa = coslineAngleRadian * einfo.cosAngle + sinlineAngleRadian * einfo.sinAngle;
            double diffA = radianToDegree(std::acos(std::abs(cosa)));
            if (diffA > maxAngleEdgeToLine) 
            {
                continue;
            }

            EdgeInformation centeredEdge;
            centeredEdge.x = einfo.x - hwidth;
            centeredEdge.y = einfo.y - hheight;
            centeredEdge.cosAngle = einfo.cosAngle;
            centeredEdge.sinAngle = einfo.sinAngle;

            if (std::abs(coslineAngleRadian * centeredEdge.x + sinlineAngleRadian * centeredEdge.y - l.distance) > maxDistanceEdgeToLine)
            {
                continue;
            }

            l.edges.push_back(centeredEdge);
        }


        if (l.edges.size() < minEdgePerLine) 
        {
            continue;
        }

        lines.push_back(l);
    }

    return true;
}

bool createSegments(std::vector<Segment> & segments, const std::vector<Line> & lines) 
{
    const double maxHoleLength = 10;

    segments.clear();

    // Loop over found lines
    for (auto line : lines) 
    {
        double langle = degreeToRadian(line.angleDegree);
        double coslangle = cos(langle);
        double sinlangle = sin(langle);

        Vec2 lineDirection;
        lineDirection.x() = -sinlangle;
        lineDirection.y() = coslangle;

        Vec2 base;
        bool first = true;
        double minscale = std::numeric_limits<double>::max();
        double maxscale = -std::numeric_limits<double>::max();

        std::vector<double> scales;
        for (const EdgeInformation & e : line.edges) 
        {
            //Project point on line
            double lambda = (coslangle * e.x + sinlangle * e.y) - line.distance;
            double lineEdgeX = e.x - lambda * coslangle;
            double lineEdgeY = e.y - lambda * sinlangle;

            //Store the first point as the line base point
            if (first) 
            {
                base.x() = lineEdgeX;
                base.y() = lineEdgeY;
                first = false;
            }

            //Compute the point position on the directed line
            double scale = lineDirection.x() * (lineEdgeX - base.x()) + lineDirection.y() * (lineEdgeY - base.y()); 
            scales.push_back(scale);
        }

        std::sort(scales.begin(), scales.end());

        Segment s;
        s.angleDegree = line.angleDegree;
        s.distance = line.distance;

        int lastvalid = 0;
        for (int i = 1; i  < scales.size(); i++) 
        {
            double dscale = scales[i] - scales[i-1];
            if (dscale > maxHoleLength)
            {
                double start = scales[lastvalid];
                double end = scales[i - 1];

                for (int pos = lastvalid; pos < i; pos++)
                {
                    s.edges.push_back(EdgeInformation());
                }

                s.start = base + start * lineDirection;
                s.end = base + end * lineDirection;

                segments.push_back(s);
                lastvalid = i;
            }
        }

        double start = scales[lastvalid];
        double end = scales[scales.size() - 1];
        
        s.start = base + start * lineDirection;
        s.end = base + end * lineDirection;
        
        segments.push_back(s);
    }

    return true;
} 

std::list<int> findLargestPath(const std::map<int, std::vector<int>> & connections, const std::vector<Segment> & segments)
{
    typedef struct 
    {
        int segmentId;
        int childRank;
        std::list<int> visited;
    }
    ElementInfo;

    std::list<int> maxpath;
    int maxcost = 0;

    //Looking for paths
    for (auto & referenceSegment : connections)
    {
        int idSegmentRef = referenceSegment.first;

        if (connections.find(idSegmentRef) == connections.end()) 
        {
            continue;
        }

        if (connections.at(idSegmentRef).size() == 0) 
        {
            continue;
        }

        std::list<ElementInfo> stack;

        //Add this segment first child on top of the stack
        ElementInfo ei;
        ei.segmentId = idSegmentRef;
        ei.visited.push_back(idSegmentRef);
        ei.childRank = 0;
        stack.push_back(ei);

        while (!stack.empty())
        {
            ElementInfo & topElement = stack.back();
            const std::vector<int> & children = connections.at(topElement.segmentId);

            if (topElement.childRank >= children.size())
            {
                stack.pop_back();
                continue;
            } 

            int childId = children.at(topElement.childRank);
            topElement.childRank++;

            if (connections.find(childId) == connections.end()) 
            {
                continue;
            }
            

            const std::vector<int> & grandchildren = connections.at(childId);
            if (grandchildren.size() > 0) 
            {
                ei.segmentId = childId;
                ei.childRank = 0;
                ei.visited = topElement.visited;
                ei.visited.push_back(childId);
                stack.push_back(ei);
            }
            else 
            {
                std::list<int> path = topElement.visited;
                path.push_back(childId);

                int cost = 0;
                for (int id : path)
                {
                    cost += segments[id].edges.size();
                }

                if (cost > maxcost) 
                {
                    maxcost = cost;
                    maxpath = path;
                }
            }
        }
    }   

    return maxpath;
}

bool mergeSegments(std::vector<Segment> & mergedSegments, std::vector<Segment> & segments, int  rows, int cols)
{
    const double maxDistanceBetweenConnectedSegments = 100;
    const double maxOrthogonalDistanceToLine = 5;
    const double maxAngleDifferenceDegree = 15;

    mergedSegments.clear();

    std::map<int, std::vector<int>> connections;
    

    for (int idSegmentRef = 0; idSegmentRef < segments.size(); idSegmentRef++)
    {
        Segment & segmentRef = segments[idSegmentRef];

        double angleRef = degreeToRadian(segmentRef.angleDegree);
        double car = cos(angleRef);
        double sar = sin(angleRef);
        double dr = segmentRef.distance;
        Vec2 dirRef = (segmentRef.end - segmentRef.start).normalized();

        std::vector<int> nextSegments;

        for (int idSegmentOther = idSegmentRef + 1; idSegmentOther < segments.size(); idSegmentOther++)
        {
            Segment & segmentOther = segments[idSegmentOther];
            double diff = unsignedAngleDistance(segmentOther.angleDegree, segmentRef.angleDegree);

            //Do they share a similar normal ? 
            if (diff < maxAngleDifferenceDegree) 
            {
                //Ignore points too far away
                double dist = (segmentRef.end - segmentOther.start).norm();
                if (dist > maxDistanceBetweenConnectedSegments) 
                {
                    continue;
                }

                //Make sure the connection is not too far away from the line
                double orthoDist = car * segmentOther.start.x() + sar * segmentOther.start.y() - dr;
                if (orthoDist > maxOrthogonalDistanceToLine)
                {
                    continue;
                }

                //Check that the next point is not behind
                Vec2 dirNext = (segmentOther.start - segmentRef.end).normalized();
                double cosDir = dirNext.x() * dirRef.x() + dirNext.y() * dirRef.y();
                if (cosDir < 0.7)
                {
                    continue;
                }

                nextSegments.push_back(idSegmentOther);
            }

            //Do they share a similar normal but with opposite direction ?  
            if (fabs(diff - 180) < maxAngleDifferenceDegree) 
            {
                //Ignore points too far away
                double dist = (segmentRef.start - segmentOther.end).norm();
                if (dist > maxDistanceBetweenConnectedSegments) 
                {
                    continue;
                }

                //Make sure the connection is not too far away from the line
                double orthoDist = car * segmentOther.end.x() + sar * segmentOther.end.y() - dr;
                if (orthoDist > maxOrthogonalDistanceToLine)
                {
                    continue;
                }

                //Check that the next point is not behind
                Vec2 dirNext = (segmentOther.end - segmentRef.end).normalized();
                double cosDir = dirNext.x() * dirRef.x() + dirNext.y() * dirRef.y();
                if (cosDir < 0.7)
                {
                    continue;
                }

                nextSegments.push_back(idSegmentOther);
            }
        }

        connections[idSegmentRef] = nextSegments;
    }

    int count = 0;
    
    while (1) 
    {
        std::list<int> maxpath = findLargestPath(connections, segments);
        if (maxpath.size() <6) break;

        {
            cv::Mat res(rows, cols, CV_8UC3);
            res = 0;

            Vec2 half(cols/2, rows/2);

            mergedSegments.clear();
            for (auto id : maxpath)
            {
                mergedSegments.push_back(segments[id]);
            }

            for (int i = 0; i < mergedSegments.size(); i++)
            {
                Vec2 start = half + mergedSegments[i].start;
                Vec2 end = half + mergedSegments[i].end;

                cv::line(res, cv::Point(start.x(), start.y()), cv::Point(end.x(), end.y()), cv::Scalar(255,255,255));
            }
            
            char filename[FILENAME_MAX];
            sprintf(filename, "lines%d.png", count);
            cv::imwrite(filename, res);
            count++;
        }


        for (int id : maxpath) 
        {
            connections.erase(id);
        }

        for (auto id : maxpath)
        {
            mergedSegments.push_back(segments[id]);
        }
    }

    

    return true;
}

bool process(const cv::Mat & input) 
{
    cv::Mat preprocessed;
    if (!preprocessImage(preprocessed, input))
    {
        return false;
    }

    std::vector<EdgeInformation> edgeInformations;
    if (!buildEdgesList(edgeInformations, preprocessed))
    {
        return false;
    }

    if (!filterEdgesList(edgeInformations, preprocessed.cols, preprocessed.rows)) 
    {
        return false;
    }
    
    std::vector<Line> lines;
    if (!computeHoughLines(lines, edgeInformations, preprocessed.cols, preprocessed.rows))
    {
        return false;
    }   

    std::vector<Segment> segments;
    if (!createSegments(segments, lines))
    {
        return false;
    }   

    std::vector<Segment> mergedSegments;
       

    mergedSegments = segments;

    cv::Mat res(input.rows, input.cols, CV_8UC3);
    res = 0;

    Vec2 half(input.cols/2, input.rows/2);

    for (int i = 0; i < mergedSegments.size(); i++)
    {
        Vec2 start = half + mergedSegments[i].start;
        Vec2 end = half + mergedSegments[i].end;

        cv::line(res, cv::Point(start.x(), start.y()), cv::Point(end.x(), end.y()), cv::Scalar(255,255,255));
    }
    
    cv::imwrite("lines.png", res);


    if (!mergeSegments(mergedSegments, segments, input.rows, input.cols))
    {
        return false;
    }
    
    
    return true;
}

int aliceVision_main(int argc, char* argv[])
{
    std::string sfmInputDataFilepath;
    std::string sfmOutputDataFilepath;
    std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());

    // Command line parameters
    po::options_description allParams(
    "Parse external information about cameras used in a panorama.\n"
    "AliceVision PanoramaInit");

    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmInputDataFilepath)->required(), "SfMData file input.")
    ("outSfMData,o", po::value<std::string>(&sfmOutputDataFilepath)->required(), "SfMData file output.")
    ;

    po::options_description logParams("Log parameters");
    logParams.add_options()
    ("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
        "verbosity level (fatal, error, warning, info, debug, trace).");

    allParams.add(requiredParams).add(logParams);

    // Parse command line
    po::variables_map vm;
    try
    {
        po::store(po::parse_command_line(argc, argv, allParams), vm);

        if(vm.count("help") || (argc == 1))
        {
            ALICEVISION_COUT(allParams);
            return EXIT_SUCCESS;
        }
        po::notify(vm);
    }
    catch(boost::program_options::required_option& e)
    {
        ALICEVISION_CERR("ERROR: " << e.what());
        ALICEVISION_COUT("Usage:\n\n" << allParams);
        return EXIT_FAILURE;
    }
    catch(boost::program_options::error& e)
    {
        ALICEVISION_CERR("ERROR: " << e.what());
        ALICEVISION_COUT("Usage:\n\n" << allParams);
        return EXIT_FAILURE;
    }

    ALICEVISION_COUT("Program called with the following parameters:");
    ALICEVISION_COUT(vm);

    system::Logger::get()->setLogLevel(verboseLevel);

    sfmData::SfMData sfmData;
    if(!sfmDataIO::Load(sfmData, sfmInputDataFilepath, sfmDataIO::ESfMData(sfmDataIO::ALL)))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmInputDataFilepath << "' cannot be read.");
        return EXIT_FAILURE;
    }

    /*std::shared_ptr<camera::Pinhole> cam = std::make_shared<camera::PinholeRadialK3>(640, 480, 320.0, 320.0, 240.0, 0.8, -0.1, 0.105);

    std::vector<calibration::LineWithPoints> lineWithPoints;
    
    // Create horizontal lines
    for (int i = 0; i < 480; i += 20)
    {
        calibration::LineWithPoints line;
        line.angle = M_PI_4;
        line.dist = 1;

        for (int j = 0; j < 640; j += 20)
        {
            Vec2 pt(j, i);
            
            Vec2 pos = cam->cam2ima(cam->addDistortion(cam->ima2cam(pt)));
            line.points.push_back(pos);
        }

        lineWithPoints.push_back(line);
    }

    // Create vertical lines
    for (int j = 0; j < 640; j += 20)
    {
        calibration::LineWithPoints line;
        line.angle = M_PI_4;
        line.dist = 1;

        for (int i = 0; i < 480; i += 20)
        {
            Vec2 pt(j, i);
            
            Vec2 pos = cam->cam2ima(cam->addDistortion(cam->ima2cam(pt)));
            line.points.push_back(pos);
        }

        lineWithPoints.push_back(line);
    }
    
    std::shared_ptr<camera::Pinhole> toEstimate = std::make_shared<camera::PinholeRadialK1>(640, 480, 320.0, 300.0, 260.0, 0.0);

    if (!calibration::estimate(toEstimate, lineWithPoints, true, false))
    {
        ALICEVISION_LOG_ERROR("Failed to calibrate");
        return EXIT_FAILURE;
    }

    std::cout << toEstimate->getDistortionParams()[0] << std::endl;

    std::shared_ptr<camera::Pinhole> toEstimate2 = std::make_shared<camera::PinholeRadialK3>(640, 480, 320.0, toEstimate->getOffset().x(), toEstimate->getOffset().y(), toEstimate->getDistortionParams()[0], 0.0, 0.0);

    if (!calibration::estimate(toEstimate2, lineWithPoints, true, false))
    {
        ALICEVISION_LOG_ERROR("Failed to calibrate");
        return EXIT_FAILURE;
    }

    if (!calibration::estimate(toEstimate2, lineWithPoints, false, false))
    {
        ALICEVISION_LOG_ERROR("Failed to calibrate");
        return EXIT_FAILURE;
    }

    std::cout << toEstimate2->getDistortionParams()[0] << std::endl;*/

    int pos = 0;
    for (auto v : sfmData.getViews()) 
    {
        auto view = v.second;

        std::string pathImage = view->getImagePath();

        image::Image<float> input;
        image::readImage(v.second->getImagePath(), input, image::EImageColorSpace::SRGB);

        image::Image<float> distorted = input;//distort(input);
        cv::Mat inputOpencvWrapper(distorted.Height(), distorted.Width(), CV_32FC1, distorted.data());
        

        float pixelRatio = view->getDoubleMetadata({"PixelAspectRatio"});
        if (pixelRatio < 0.0) 
        {
            pixelRatio = 1.0f;
        }

        if (pixelRatio != 1.0f)
        {
            cv::Mat resized;
            cv::resize(inputOpencvWrapper, resized, cv::Size(input.Width() * pixelRatio, input.Height()), 0.0, 0.0, cv::INTER_CUBIC);   
            cv::swap(inputOpencvWrapper, resized);
        }
        

        cv::Mat color;
        cv::cvtColor(inputOpencvWrapper, color, cv::COLOR_GRAY2RGB);

        color *= 255.0f;
        cv::imwrite("/home/servantf/distorted.png", color);

        //color = cv::imread(v.second->getImagePath(), cv::IMREAD_COLOR);

        cbdetect::Params params;
        params.show_debug_image = false;
        params.corner_type = cbdetect::SaddlePoint;
        params.norm = true;

        cbdetect::Corner corners;
        cbdetect::find_corners(color, corners, params);
        cbdetect::plot_corners(color, corners);

        std::vector<cbdetect::Board> boards;
        cbdetect::boards_from_corners(color, corners, boards, params);

        cbdetect::plot_boards(color, corners, boards, params);

        std::vector<calibration::LineWithPoints> lineWithPoints;
    
        for (cbdetect::Board & b : boards)
        {
            int height = b.idx.size();
            int width = b.idx[0].size();

            std::cout << width << " " << height << std::endl;

            // Create horizontal lines
            for (int i = 0; i < height; i ++)
            {
                calibration::LineWithPoints line;
                line.angle = M_PI_4;
                line.dist = 1;

                for (int j = 0; j < width; j++)
                {
                    int idx = b.idx[i][j];
                    if (idx < 0) continue;

                    cv::Point2d p = corners.p[idx];

                    Vec2 pt;
                    pt.x() = p.x;
                    pt.y() = p.y;

                    line.points.push_back(pt);
                }

                //std::cout << line.points.size() << std::endl;
                if (line.points.size() < 10) continue;

                lineWithPoints.push_back(line);
            }

            // Create horizontal lines
            for (int j = 0; j < width; j++)
            {
                calibration::LineWithPoints line;
                line.angle = M_PI_4;
                line.dist = 1;

                for (int i = 0; i < height; i++)
                {
                    int idx = b.idx[i][j];
                    if (idx < 0) continue;

                    cv::Point2d p = corners.p[idx];

                    Vec2 pt;
                    pt.x() = p.x;
                    pt.y() = p.y;

                    line.points.push_back(pt);
                }

                if (line.points.size() < 10) continue;

                //std::cout << line.points.size() << std::endl;

                lineWithPoints.push_back(line);
            }
        }

        double w = input.Width();
        double h = input.Height();
        double d = sqrt(w*w + h*h);


        std::shared_ptr<camera::Pinhole> camera = std::make_shared<camera::PinholeRadialK1>(input.Width(), input.Height(), d, input.Width() / 2, input.Height() / 2, 0.0);
        if (!calibration::estimate(camera, lineWithPoints, true, false))
        {
            ALICEVISION_LOG_ERROR("Failed to calibrate");
            return EXIT_FAILURE;
        }

        if (!calibration::estimate(camera, lineWithPoints, false, false))
        {
            ALICEVISION_LOG_ERROR("Failed to calibrate");
            return EXIT_FAILURE;
        }

        std::shared_ptr<camera::Pinhole> camera2 = std::make_shared<camera::PinholeRadialK3>(input.Width(), input.Height(), d, camera->getOffset().x(), camera->getOffset().y(), camera->getDistortionParams()[0], 0, 0);
        if (!calibration::estimate(camera2, lineWithPoints, false, false))
        {
            ALICEVISION_LOG_ERROR("Failed to calibrate");
            return EXIT_FAILURE;
        }

        cv::Mat undistorted = undistort(camera2, inputOpencvWrapper);
        cv::imwrite("/home/servantf/undistorted.png", undistorted);

        pos++;
    }

    return EXIT_SUCCESS;

}
