// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "Logger.hpp"
#include "Timer.hpp"

// This file is header only, so the module don't need to have program_options as a dependency
#include <boost/program_options/variables_map.hpp>
#include <boost/program_options/option.hpp>
#include <boost/program_options/errors.hpp>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>

#include <functional>
#include <ostream>

#define ALICEVISION_COMMANDLINE_START \
\
try { \
system::Timer commandLineTimer; \


#define ALICEVISION_COMMANDLINE_END \
\
    ALICEVISION_LOG_INFO("Task done in (s): " + std::to_string(commandLineTimer.elapsed())); \
    return EXIT_SUCCESS; \
\
} catch(std::exception& e) \
{ \
  ALICEVISION_CERR("================================================================================"); \
  ALICEVISION_CERR("====================== Command line failed with an error ======================="); \
  ALICEVISION_CERR(e.what()); \
  ALICEVISION_CERR("================================================================================"); \
  return EXIT_FAILURE; \
} catch(...) \
{ \
  ALICEVISION_CERR("================================================================================"); \
  ALICEVISION_CERR("============== Command line failed with an unrecognized exception =============="); \
  ALICEVISION_CERR("================================================================================"); \
  return EXIT_FAILURE; \
}


namespace aliceVision {

template <class T>
std::function<void(T)> optInRange(T min, T max, const char* opt_name)
{
    return [=](T v) {
        if(v < min || v > max)
        {
            throw boost::program_options::validation_error(
                boost::program_options::validation_error::invalid_option_value, opt_name,
                                       std::to_string(v));
        }
    };
};
}

namespace boost {

inline std::ostream& operator<<(std::ostream& os, const boost::any& value)
{
    bool is_char;
    try
    {
        boost::any_cast<const char *>(value);
        is_char = true;
    }
    catch(const boost::bad_any_cast &)
    {
        is_char = false;
    }
    bool is_str;
    try
    {
        boost::any_cast<std::string>(value);
        is_str = true;
    }
    catch(const boost::bad_any_cast &)
    {
        is_str = false;
    }

    if(value.type() == typeid(int))
    {
        os << boost::any_cast<int>(value);
    }
    else if(value.type() == typeid(std::size_t))
    {
        os << boost::any_cast<std::size_t>(value);
    }
    else if(((boost::any)value).type() == typeid(bool))
    {
        os << boost::any_cast<bool>(value);
    }
    else if(value.type() == typeid(float))
    {
        os << boost::any_cast<float>(value);
    }
    else if(value.type() == typeid(double))
    {
        os << boost::any_cast<double>(value);
    }
    else if(is_char)
    {
        os << "\"" << boost::any_cast<const char *>(value) << "\"";
    }
    else if(is_str)
    {
        os << "\"" << boost::any_cast<std::string>(value) << "\"";
    }
    else
    {
        // Assumes that the only remainder is vector<string>
        try
        {
            std::vector<std::string> vect = boost::any_cast<std::vector<std::string>>(value);
            os << " = [";
            if(!vect.empty())
            {
                os << vect[0];
                for(int i = 1; i < vect.size(); ++i)
                    os << ", " << vect[i];
            }
            os << "]";
        }
        catch(const boost::bad_any_cast &)
        {
            os << " Unknown Type \"" << value.type().name() << "\"";
        }
    }
    return os;
}

namespace program_options {

inline std::ostream& operator<<(std::ostream& os, const variables_map& vm)
{
    for(const auto& v: vm)
    {
        const std::string& optionName = v.first;
        const boost::program_options::variable_value& var = v.second;

        os << " * " << optionName << " = " << var.value();

        if(var.value().empty())
        {
            os << " (empty)";
        }
        if (vm[optionName].defaulted() || var.defaulted())
        {
            os << " (default)";
        }
        os << std::endl;
    }
    return os;
}

}
}

namespace aliceVision {

class CmdLine
{
public:
    CmdLine(const std::string& name) :
        _allParams(name)
    {
    }

    void add(const boost::program_options::options_description& options)
    {
        _allParams.add(options);
    }

    bool execute(int argc, char** argv)
    {
        std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());

        boost::program_options::options_description logParams("Log parameters");
        logParams.add_options()
            ("verboseLevel,v", boost::program_options::value<std::string>(&verboseLevel)->default_value(verboseLevel), "verbosity level (fatal, error, warning, info, debug, trace).");

        _allParams.add(logParams);

        boost::program_options::variables_map vm;
        try
        {
            boost::program_options::store(boost::program_options::parse_command_line(argc, argv, _allParams), vm);

            if (vm.count("help") || (argc == 1))
            {
                ALICEVISION_COUT(_allParams);
                return false;
            }
            boost::program_options::notify(vm);
        }
        catch (boost::program_options::required_option& e)
        {
            ALICEVISION_CERR("ERROR: " << e.what());
            ALICEVISION_COUT("Usage:\n\n" << _allParams);
            return false;
        }
        catch (boost::program_options::error& e)
        {
            ALICEVISION_CERR("ERROR: " << e.what());
            ALICEVISION_COUT("Usage:\n\n" << _allParams);
            return false;
        }

        ALICEVISION_COUT("Program called with the following parameters:");
        ALICEVISION_COUT(vm);

        // set verbose level
        system::Logger::get()->setLogLevel(verboseLevel);

        return true;
    }

private:
    boost::program_options::options_description _allParams;
};

}
