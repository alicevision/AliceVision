#include "json.hpp"

namespace aliceVision 
{
std::vector<boost::json::value> readJsons(std::istream& is, boost::json::error_code& ec)
{
    std::vector<boost::json::value> jvs;
    boost::json::stream_parser p;
    std::string line;
    std::size_t n = 0;

    while(true)
    {
        if(n == line.size())
        {
            if(!std::getline(is, line))
            {
                break;
            }

            n = 0;
        }

        //Consume at least part of the line
        n += p.write_some( line.data() + n, line.size() - n, ec);

        //If the parser found a value, add it
        if (p.done())
        {
            jvs.push_back(p.release());
            p.reset();
        }
    }

    if (!p.done())
    {
        //Try to extract the end
        p.finish(ec);
        if (ec.failed())
        {
            return jvs;
        }

        jvs.push_back(p.release());
    }

    return jvs;
}
}