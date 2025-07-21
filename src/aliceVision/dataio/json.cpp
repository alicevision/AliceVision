#include "json.hpp"

namespace aliceVision {
std::vector<boost::json::value> readJsons(std::istream& is, boost::system::error_code& ec)
{
    std::vector<boost::json::value> jvs;
    boost::json::stream_parser p;
    std::string content, line;
    std::size_t totalRead;

    while (std::getline(is, line))
    {   
        content += line;
        
        //Try to read all available items in the buffer
        while (true)
        {
            totalRead = p.write_some(content, ec);

            // If the parser did not find a value, then it won't
            // find anything more.
            if (!p.done())
            {
                break;
            }

            // Store the value, and try to find another one in the remaining
            // content
            jvs.push_back(p.release());
            p.reset();

            //remove processed string from content
            content = content.substr(totalRead);
        }
    }

    if (!p.done())
    {
        // Try to extract the end
        p.finish(ec);
        if (ec.failed())
        {
            return jvs;
        }

        jvs.push_back(p.release());
    }

    return jvs;
}
}  // namespace aliceVision