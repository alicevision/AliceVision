#pragma once

#include <boost/json.hpp>
#include <aliceVision/calibration/checkerDetector.hpp>

namespace aliceVision
{
namespace calibration
{

CheckerDetector::CheckerBoardCorner tag_invoke(boost::json::value_to_tag<CheckerDetector::CheckerBoardCorner>,
                                               boost::json::value const& jv)
{
    boost::json::object const& obj = jv.as_object();

    calibration::CheckerDetector::CheckerBoardCorner ret;
    ret.center.x() = boost::json::value_to<double>(obj.at("center_x"));
    ret.center.y() = boost::json::value_to<double>(obj.at("center_y"));
    ret.dir1.x() = boost::json::value_to<double>(obj.at("dir1_x"));
    ret.dir1.y() = boost::json::value_to<double>(obj.at("dir1_y"));
    ret.dir2.x() = boost::json::value_to<double>(obj.at("dir2_x"));
    ret.dir2.y() = boost::json::value_to<double>(obj.at("dir2_y"));
    ret.scale = boost::json::value_to<double>(obj.at("scale"));

    return ret;
}

void tag_invoke(const boost::json::value_from_tag&, boost::json::value& jv,
                CheckerDetector::CheckerBoardCorner const& t)
{
    jv = {{"center_x", t.center.x()}, {"center_y", t.center.y()}, {"dir1_x", t.dir1.x()},
          {"dir1_y", t.dir1.y()},     {"dir2_x", t.dir2.x()},     {"dir2_y", t.dir2.y()},
          {"scale", t.scale}};
}

CheckerDetector tag_invoke(boost::json::value_to_tag<CheckerDetector>, boost::json::value const& jv)
{
    CheckerDetector ret;
    boost::json::object const& obj = jv.as_object();

    ret.getCorners() = boost::json::value_to<std::vector<CheckerDetector::CheckerBoardCorner>>(obj.at("corners"));
    
    std::vector<boost::json::value> jvs = boost::json::value_to<std::vector<boost::json::value>>(obj.at("boards"));

    for(boost::json::value & ljv : jvs)
    {
        boost::json::object const & lobj = ljv.as_object();

        Eigen::Index rows = boost::json::value_to<Eigen::Index>(lobj.at("rows"));
        Eigen::Index cols = boost::json::value_to<Eigen::Index>(lobj.at("cols"));
        std::vector<IndexT> values = boost::json::value_to<std::vector<IndexT>>(lobj.at("data"));
        
        CheckerDetector::CheckerBoard board(rows, cols);
        int pos = 0;
        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < cols; j++)
            {
                board(i, j) = values[pos];
                pos++;
            }
        }

        ret.getBoards().push_back(board);
    }

    return ret;
}

void tag_invoke(const boost::json::value_from_tag&, boost::json::value& jv, CheckerDetector const& t)
{
    

    std::vector<boost::json::value> jvs;

    for(auto b : t.getBoards())
    {
        std::vector<IndexT> values;
        for (int i = 0; i < b.rows(); i++)
        {
            for (int j = 0; j < b.cols(); j++)
            {
                values.push_back(b(i, j));
            }
        }

        boost::json::value sjv = { 
            {"rows", b.rows()}, {"cols", b.cols()}, {"data", values}
        };


        jvs.push_back(sjv);
    }

    jv = {{"corners", t.getCorners()}, {"boards", jvs}};
}

} // namespace calibration
} // namespace aliceVision

