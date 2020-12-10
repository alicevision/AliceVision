/*Command line parameters*/
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

    cv::Ptr<cv::CLAHE> app = cv::createCLAHE(2, cv::Size(16, 16));
    app->apply(converted, preprocessed);

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
    
    
    /*

    std::remove_if(einfos.begin(), einfos.end(), [](const EdgeInformation & ei) { return !ei.valid;});

    char filename[FILENAME_MAX];
    sprintf(filename, "/home/servantf/test%d.png", iter);
    cv::imwrite(filename, edges);   

    double hwidth = double(input.cols) / 2.0;    
    double hheight = double(input.rows) / 2.0;
    double minAngle = 0.0;
    double maxAngle = 359.0;    
    double stepAngle = 1;
    double mindistance = 0.0;
    double maxdistance = sqrt(hwidth * hwidth + hheight * hheight);
    double stepDistance = 1.0;
    double minParameter = 0.5;
    double maxParameter = 1.5;
    double stepParameter = 0.1;

    int countBinsAngle = int((maxAngle - minAngle) / stepAngle);
    int countBinsDistance = int((maxdistance - mindistance) / stepDistance);
    int countBinsParameter = int((maxParameter - minParameter) / stepParameter);

    struct binInfo
    {
        double angle;
        double distance;
        double score;
    };
    
    std::vector<binInfo> bins(countBinsAngle * countBinsDistance);
    for (int idAngle = 0; idAngle < countBinsAngle; idAngle++) 
    {
        for (int idDistance = 0; idDistance < countBinsDistance; idDistance++) 
        {
            bins[idAngle * countBinsDistance + idDistance].angle = double(idAngle) * stepAngle;
            bins[idAngle * countBinsDistance + idDistance].distance = double(idDistance) * stepDistance;
            bins[idAngle * countBinsDistance + idDistance].score = 0;
        }
    }
    

    for (const EdgeInformation & einfo : einfos) 
    {
        double edgeAngle = einfo.angle;
        int binAngleEdge = int(edgeAngle / stepAngle);

        for (int binAngle = binAngleEdge - 3; binAngle <= binAngleEdge + 3; binAngle++) 
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

            double md = std::abs(cosAngle * x + sinAngle * y);
            int currentDistance = int(md / stepDistance);


            for (int binDistance = std::max(0, currentDistance - 4); binDistance < std::min(countBinsDistance, currentDistance + 4); binDistance++)
            {
                double distance = double(binDistance) * stepDistance;
                double error = fabs(md - distance);

                bins[correctedBinAngle * countBinsDistance + binDistance].score += 1.0 / (1.0 + error);
            }
        }
    }

    std::sort(bins.begin(), bins.end(), [](const binInfo &a, const binInfo &b) { return a.score > b.score; } );
    
    struct Line
    {
        double angle;
        double distance;

        std::vector<EdgeInformation> edges;
    };

    std::vector<Line> added;
    for (int id = 0; id < bins.size(); id++)
    {
        if (bins[id].score < 10.0) break;

        bool found = false;
        for (const Line & prev : added) 
        {
            double diff = unsignedAngleDistance(bins[id].angle, prev.angle);
            double diffd = std::abs(bins[id].distance - prev.distance);

            if (diff < 2 && diffd < 20)
            {
                found = true;
            }
        }


        if (found) 
        {
            continue;
        }

        Line l;
        l.angle = bins[id].angle;
        l.distance = bins[id].distance;

        added.push_back(l);
    }
    
    for (Line & l : added) 
    {
        for (const EdgeInformation & einfo : einfos) 
        {
            if (unsignedAngleDistance(l.angle, einfo.angle) > 2)
            {
                continue;
            }

            if (std::abs(std::abs(cos(l.angle) * (einfo.x - hwidth) + sin(l.angle) * (einfo.y - hheight)) - l.distance) > 10)
            {
                continue;
            }

            l.edges.push_back(einfo);
        }
    }

    cv::Mat res(input.rows, input.cols, CV_8UC1);
    res = 0;

    for (auto bin : added) 
    {
        if (bin.edges.size() < 100) continue;

        for (EdgeInformation & e : bin.edges) 
        {
            cv::line(res, cv::Point(e.x - 1, e.y), cv::Point(e.x + 1, e.y), cv::Scalar(255));
            cv::line(res, cv::Point(e.x, e.y - 1), cv::Point(e.x, e.y + 1), cv::Scalar(255));
        }
        double ca = cos(bin.angle * M_PI / 180.0);
        double sa = sin(bin.angle * M_PI / 180.0);
        
        //ca * x + sa * y - d = 0
        if (std::abs(ca) < 0.1) 
        {
            //y = (d - ca * x) / sa
            //x = (d - sa * y) / ca

            cv::Point2i p1, p2;
            p1.x = 0;
            p1.y = hheight + (bin.distance - ca * (p1.x - hwidth)) / sa;
            p2.x = input.cols - 1;
            p2.y = hheight + (bin.distance - ca * (p2.x - hwidth)) / sa;

            cv::line(res, p1, p2, cv::Scalar(255,255,255));
            
        }
        else 
        {
            

            cv::Point2i p1, p2;
            p1.y = 0;
            p1.x = hwidth + (bin.distance - sa * (p1.y - hheight)) / ca;
            p2.y = input.rows - 1;
            p2.x = hwidth + (bin.distance - sa * (p2.y - hheight)) / ca;

            cv::line(res, p1, p2, cv::Scalar(255,255,255));
        }
    }

    cv::imwrite("lines.png", res);*/
    
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

    

    int pos = 0;
    for (auto v : sfmData.getViews()) 
    {
        auto view = v.second;

        std::string pathImage = view->getImagePath();

        image::Image<float> input;
        image::readImage(v.second->getImagePath(), input, image::EImageColorSpace::SRGB);
        cv::Mat inputOpencvWrapper(input.Height(), input.Width(), CV_32FC1, input.data());


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

        process(inputOpencvWrapper);
        pos++;
    }

    return EXIT_SUCCESS;

}
