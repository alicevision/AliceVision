// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "IFeed.hpp"

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace aliceVision {
namespace dataio {

/**
 * @brief Registry of the feeds FeedProvider can create.
 * Each feed type registers itself explicitly (registerImageFeed(), registerVideoFeed(),
 * sfmDataIO::registerSfMDataFeed(), ...), so dataio does not depend on the modules providing feeds.
 * Registration is explicit rather than done by static initializers, which are dropped when linking static libraries.
 */
class FeedFactory
{
  public:
    using Predicate = std::function<bool(const std::string& path)>;
    using Creator = std::function<std::unique_ptr<IFeed>(const std::string& path, const std::string& calibPath)>;

    static FeedFactory& instance();

    /**
     * @brief Register a feed. Registering a name a second time is a no-op.
     * @param[in] name The unique name of the feed
     * @param[in] canHandle Return true if the feed can be created from the given path
     * @param[in] create Create the feed from the given path and calibration path
     */
    void registerFeed(const std::string& name, Predicate canHandle, Creator create);

    /**
     * @brief Create the first registered feed that can handle the given path.
     * @return The created feed, or nullptr if no registered feed handles the path
     */
    std::unique_ptr<IFeed> create(const std::string& path, const std::string& calibPath) const;

  private:
    FeedFactory() = default;

    struct Entry
    {
        std::string name;
        Predicate canHandle;
        Creator create;
    };

    mutable std::mutex _mutex;
    std::vector<Entry> _entries;
};

}  // namespace dataio
}  // namespace aliceVision
