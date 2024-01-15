// This file is part of the AliceVision project.
// Copyright (c) 2020 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <codecvt>
#include <filesystem>
#include <functional>
#include <iostream>
#include <random>
#include <string>
#include <sys/stat.h>
#include <vector>

namespace aliceVision {
namespace utils {

namespace fs = std::filesystem;

/**
 * @brief Allows to retrieve the files paths that validates a specific predicate by searching in a folder.
 * @param[in] the folders path
 * @param[in] the predicate
 * @return the paths list to the corresponding files if they validate the predicate, otherwise it returns an empty list.
 */
inline std::vector<std::string> getFilesPathsFromFolder(const std::string& folder, const std::function<bool(const fs::path&)>& predicate)
{
    // Get all files paths in folder
    std::vector<std::string> paths;

    // If the path isn't a folder path
    if (!fs::is_directory(folder))
        throw std::invalid_argument("The path '" + folder + "' is not a valid folder path.");

    for (const auto& pathIt : fs::directory_iterator(folder))
    {
        const fs::path path = pathIt.path();
        if (is_regular_file(path) && predicate(path))
            paths.push_back(path.generic_string());
    }

    return paths;
}

/**
 * @brief Allows to retrieve the files paths that validates a specific predicate by searching through a list of folders.
 * @param[in] the folders paths list
 * @param[in] the predicate
 * @return the paths list to the corresponding files if they validate the predicate, otherwise it returns an empty list.
 */
inline std::vector<std::string> getFilesPathsFromFolders(const std::vector<std::string>& folders,
                                                         const std::function<bool(const fs::path&)>& predicate)
{
    std::vector<std::string> paths;
    for (const std::string& folder : folders)
    {
        const std::vector<std::string> subPaths = getFilesPathsFromFolder(folder, predicate);
        paths.insert(paths.end(), subPaths.begin(), subPaths.end());
    }

    return paths;
}

/**
 * @brief Generates a random filename of a specified length that is suitable for creating temporary files.
 * This is meant to be an alternative to boost::filesystem::unique_path() as long as std::filesystem does not contain any alternative.
 * @param[in] length The length of the random filename to generate
 * @return A string of random characters
 */
inline std::string generateUniqueFilename(const int length = 16)
{
    static const char characters[] = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";

    std::random_device rd;
    std::mt19937 randomTwEngine(rd());
    int nbChars = sizeof(characters) - 1;  // -1 to exclude the null character at the end of the string from the count
    std::uniform_int_distribution<> randomDist(0, nbChars - 1);

    std::string filename;
    filename.resize(length);

    for (int i = 0; i < length; ++i)
        filename[i] = characters[randomDist(randomTwEngine)];

    return filename;
}

/**
 * @brief Returns the last time a file was modified (based on OIIO's implementation).
 * @param[in] path The path to get the last write time from
 * @return The last time the file was modified as an std::time_t if it exists, 0 otherwise
 */
inline std::time_t getLastWriteTime(const std::string& path)
{
#ifdef _WIN32
    struct __stat64 st;
    std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>, wchar_t> conv;
    std::wstring str = conv.from_bytes(path.data(), path.data() + path.size());
    auto r = _wstat64(std::filesystem::path(str).c_str(), &st);
#else
    struct stat st;
    auto r = stat(std::filesystem::path(path).c_str(), &st);
#endif

    if (r == 0)  // success
    {
        return st.st_mtime;
    }
    else  // failure
    {
        return 0;
    }
}

}  // namespace utils
}  // namespace aliceVision