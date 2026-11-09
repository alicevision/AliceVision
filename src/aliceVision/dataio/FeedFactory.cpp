// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "FeedFactory.hpp"

namespace aliceVision {
namespace dataio {

FeedFactory& FeedFactory::instance()
{
    static FeedFactory factory;
    return factory;
}

void FeedFactory::registerFeed(const std::string& name, Predicate canHandle, Creator create)
{
    std::lock_guard<std::mutex> lock(_mutex);
    for (const Entry& entry : _entries)
    {
        if (entry.name == name)
            return;
    }
    _entries.push_back({name, std::move(canHandle), std::move(create)});
}

std::unique_ptr<IFeed> FeedFactory::create(const std::string& path, const std::string& calibPath) const
{
    // Copy the entries so that opening a (possibly slow) feed is done without holding the lock
    std::vector<Entry> entries;
    {
        std::lock_guard<std::mutex> lock(_mutex);
        entries = _entries;
    }
    for (const Entry& entry : entries)
    {
        if (entry.canHandle(path))
            return entry.create(path, calibPath);
    }
    return nullptr;
}

}  // namespace dataio
}  // namespace aliceVision
