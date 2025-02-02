// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/image/io.hpp>

#include <boost/program_options.hpp>

#include <string>
#include <sstream>
#include <random>
#include <filesystem>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;


namespace {

/**
 * @brief operator method enum
 */
enum class EMaskOperator : unsigned char
{
    OR = 0,
    AND,
    NOT
};

/**
 * @brief Convert an EMaskOperator enum to its corresponding string
 * @param[in] maskOperator The given EMaskOperator enum
 * @return string
 */
std::string EMaskOperator_enumToString(EMaskOperator maskOperator)
{
    switch (maskOperator)
    {
        case EMaskOperator::OR:
            return "or";
        case EMaskOperator::AND:
            return "and";
        case EMaskOperator::NOT:
            return "not";
    }
    throw std::out_of_range("Invalid EMaskOperator enum");
}

/**
 * @brief Convert a string to its corresponding EMaskOperator enum
 * @param[in] MaskOperator The given string
 * @return EMaskOperator enum
 */
EMaskOperator EMaskOperator_stringToEnum(const std::string& maskOperator)
{
    std::string op = maskOperator;
    std::transform(op.begin(), op.end(), op.begin(), ::tolower);  // tolower

    if (op == "or")
        return EMaskOperator::OR;
    if (op == "and")
        return EMaskOperator::AND;
    if (op == "not")
        return EMaskOperator::NOT;


    throw std::out_of_range("Invalid mask operator : " + maskOperator);
}

inline std::istream& operator>>(std::istream& in, EMaskOperator& maskOperator)
{
    std::string token(std::istreambuf_iterator<char>(in), {});
    maskOperator = EMaskOperator_stringToEnum(token);
    return in;
}

inline std::ostream& operator<<(std::ostream& os, EMaskOperator e) { return os << EMaskOperator_enumToString(e); }

}  // namespace


int aliceVision_main(int argc, char** argv)
{
    // command-line parameters
    std::vector<std::string> directoryNames;
    std::string outDirectory;
    EMaskOperator maskOperator = EMaskOperator::OR;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("inputs,i", po::value<std::vector<std::string>>(&directoryNames)->multitoken(),
         "Path to directories to process.")
        ("output,o", po::value<std::string>(&outDirectory)->required(),
         "Output directory.");
        
    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("operator", po::value<EMaskOperator>(&maskOperator)->default_value(maskOperator), "");
    // clang-format on

    CmdLine cmdline("AliceVision sfmMerge");
    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    if (directoryNames.empty())
    {
        ALICEVISION_LOG_ERROR("At least one directory should be given.");
        return EXIT_FAILURE;
    }

    std::string path = directoryNames[0];
    for (auto &p : std::filesystem::recursive_directory_iterator(path))
    {
        const std::filesystem::path refpath = p.path();
        if (p.path().extension() != ".exr")
        {
            continue;
        }

        ALICEVISION_LOG_INFO("Processing " << refpath.string());

        std::filesystem::path outputDirectoryPath(outDirectory);
        std::filesystem::path outputPath = outputDirectoryPath / refpath.filename();


        image::Image<unsigned char> img;
        aliceVision::image::readImage(refpath.string(), img, image::EImageColorSpace::NO_CONVERSION);

        if (maskOperator == EMaskOperator::NOT)
        {
            for (int i = 0; i < img.height(); i++)
            {
                for (int j = 0; j < img.width(); j++)
                {
                    img(i, j) = (img(i, j) > 0)?0:255;
                }
            }
        }
        else 
        {
            for (int otherDirIndex = 1; otherDirIndex < directoryNames.size(); otherDirIndex++)
            {
                std::filesystem::path otherPath(directoryNames[otherDirIndex]);

                std::filesystem::path otherMaskPath = otherPath / refpath.filename();
                if (!std::filesystem::exists(otherMaskPath))
                {
                    continue;
                }

                image::Image<unsigned char> otherImg;
                aliceVision::image::readImage(otherMaskPath.string(), otherImg, image::EImageColorSpace::NO_CONVERSION);

                if (otherImg.width() != img.width())
                {
                    continue;
                }

                if (otherImg.height() != img.height())
                {
                    continue;
                }
                
                
                for (int i = 0; i < img.height(); i++)
                {
                    for (int j = 0; j < img.width(); j++)
                    {
                        if (maskOperator == EMaskOperator::AND)
                        {
                            img(i, j) = img(i, j) & otherImg(i, j);
                        }
                        else if (maskOperator == EMaskOperator::OR)
                        {
                            img(i, j) = img(i, j) | otherImg(i, j);
                        }
                    }
                }
            }
        }

        aliceVision::image::ImageWriteOptions wopt;
        aliceVision::image::writeImage(outputPath.string(), img, wopt);
    }

    
    
    return EXIT_SUCCESS;
}
