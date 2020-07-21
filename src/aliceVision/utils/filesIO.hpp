// This file is part of the AliceVision project.
// Copyright (c) 2020 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "boost/filesystem.hpp"

#include <vector>
#include <string>
#include <functional>

namespace fs = boost::filesystem;

namespace aliceVision {
namespace utils {
/**
 * @brief Allows to retrieve the files paths that validates a specific predicate by searching in a folder.
 * @param[in] the folders path
 * @param[in] the predicate
 * @return the paths list to the corresponding files if they validate the predicate, otherwise it returns an empty list.
 */
inline std::vector<std::string> getFilesPathsFromFolder(const std::string& folder,
                                                 const std::function<bool(const boost::filesystem::path&)>& predicate)
{
    // Get all files paths in folder
    std::vector<std::string> paths;

    // If the path isn't a folder path
    if(!fs::is_directory(folder))
        throw std::invalid_argument("The path '" + folder + "' is not a valid folder path.");

    for(const auto& pathIt : fs::directory_iterator(folder))
    {
        const fs::path path = pathIt.path();
        if(is_regular_file(path) && predicate(path))
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
                                                  const std::function<bool(const boost::filesystem::path&)>& predicate)
{
    std::vector<std::string> paths;
    for(const std::string& folder : folders)
    {
        const std::vector<std::string> subPaths = getFilesPathsFromFolder(folder, predicate);
        paths.insert(paths.end(), subPaths.begin(), subPaths.end());
    }

    return paths;
}

} // namespace utils
} // namespace aliceVision