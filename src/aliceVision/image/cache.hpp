// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "aliceVision/numeric/numeric.hpp"
#include <memory>

#include <unordered_map>
#include <boost/multi_index_container.hpp>
#include <boost/multi_index/hashed_index.hpp>
#include <boost/multi_index/identity.hpp>
#include <boost/multi_index/sequenced_index.hpp>
#include <boost/multi_index/member.hpp>

#include <fstream>
#include <sstream>
#include <queue>

namespace aliceVision
{
namespace image
{


class TileCacheManager;

/**
 * A cached tile is an object containing a rectangular region of an image
 * It has a tile width and a tile height. This is the memory used by this tile
 * It also has a required width and a required height. This is the really used part of the tile
 * This is because a tile may be on a border of the image and not fully used.
 * It contains a pointer to its data which may be null if the data is out of core
 */
class CachedTile {
public:
  using weak_pointer = std::weak_ptr<CachedTile>;
  using smart_pointer = std::shared_ptr<CachedTile>;

public:

  CachedTile() = delete;
  
  CachedTile(const std::shared_ptr<TileCacheManager> & manager, size_t uid, size_t tileWidth, size_t tileHeight, size_t width, size_t height, size_t depth) 
  : _manager(manager),_uid(uid), _tileWidth(tileWidth), _tileHeight(tileHeight), _requiredWidth(width), _requiredHeight(height), _depth(depth) {

    /*Make sure the required size is less or equal than the tile size*/
    _requiredWidth = std::min(_requiredWidth, _tileWidth);
    _requiredHeight = std::min(_requiredHeight, _tileHeight);
  } 

  ~CachedTile();

  size_t getUid() const {
    return _uid;
  }

  size_t getTileWidth() const {
    return _tileWidth;
  }

  size_t getTileHeight() const {
    return _tileHeight;
  }

  size_t getRequiredWidth() const {
    return _requiredWidth;
  }

  size_t getRequiredHeight() const {
    return _requiredHeight;
  }

  size_t getDepth() const {
    return _depth;
  }
  
  /*
  Tells the system that we need the data for this tile.
  This means that the data is out of core, we want it back.
  @return false if the process failed to grab data.
  */
  bool acquire();

  /**
   * Update data with a new buffer
   * Move the data parameter to the _data property.
   * @note the parameter is invalidated !
   */
  void setData(std::unique_ptr<unsigned char> && data) {
    _data = std::move(data);
  }

  /**
   * Get a pointer to the contained data
   * @return nullptr if the data is cached
   */
  unsigned char * getDataPointer() const {

    if (!_data) {
      return nullptr;
    }

    return _data.get();
  }  

  /**
   * Move the data.
   * The data is returned and the object property is set to nullptr
   */
  std::unique_ptr<unsigned char> getData() {
    return std::move(_data);
  }

private:
  std::unique_ptr<unsigned char> _data = nullptr;
  std::weak_ptr<TileCacheManager> _manager;

  size_t _uid;
  size_t _tileWidth;
  size_t _tileHeight;
  size_t _requiredWidth;
  size_t _requiredHeight;
  size_t _depth;
};

/*
An abstract concept of cache management for generic objects
*/
class CacheManager {
public:
  using IndexedStoragePaths = std::unordered_map<size_t, std::string>;
  using IndexedFreeBlocks = std::unordered_map<size_t, std::list<size_t>>;

  /* 
  An item of the Most Recently used container
  */
  struct MRUItem
  {
    size_t objectId;
    size_t objectSize;
  };

  /*
  An item of the object to memory block associative array
  */
  struct MemoryItem
  {
    size_t startBlockId;
    size_t countBlock;
  };

  using MemoryMap = std::map<size_t, MemoryItem>;

  /**
   Most recently used object container
   Used to know which objects have not been used for some time
   */
  using MRUType = boost::multi_index::multi_index_container<
                    MRUItem, 
                    boost::multi_index::indexed_by<
                      boost::multi_index::sequenced<>,
                      boost::multi_index::hashed_unique<
                        boost::multi_index::member<
                          MRUItem, 
                          size_t, 
                          &MRUItem::objectId>
                        >
                      >
                    >;

public:
  CacheManager() = delete;

  /**
   * Create a cache manager
   * Each created object is associated to this manager.
   * @param pathStorage the path to the directory where the file will be stored
   * @param blockSize the base size of an object
   * @param maxTilesPerIndex the maximal number of blocks for a given file (give a maximal size for a cache file)
   */
  CacheManager(const std::string & pathStorage, size_t blockSize, size_t maxBlocksPerIndex);
  virtual ~CacheManager();

  /**
   * Set the maximal memory size
   * @param max the maximal memory size
   */
  void setMaxMemory(size_t maxMemorySize);

  /**
   * Set the maximal number number of items simultaneously in core
   * @param max the maximal number of items
   */
  void setInCoreMaxObjectCount(size_t max);

  /**
   * Create a new object of size block count
   * @param objectId the created object index
   * @param blockCount the required size for this object (In number of blocks)
   * @return true if the object was created
   */
  bool createObject(size_t & objectId, size_t blockCount);

  /**
   * Acquire a given object
   * @param data the result data acquired
   * @param objectId the object index to acquire
   * @return true if the object was acquired
   */
  bool acquireObject(std::unique_ptr<unsigned char> & data, size_t objectId);

  /**
   * Get the number of managed blocks
   * @return a block count
   */
  size_t getActiveBlocks() const;

protected:

  std::string getPathForIndex(size_t indexId);
  void deleteIndexFiles();
  void wipe();

  bool prepareBlockGroup(size_t startBlockId, size_t blocksCount);
  std::unique_ptr<unsigned char> load(size_t startBlockId, size_t blocksCount);
  bool save(std::unique_ptr<unsigned char> && data, size_t startBlockId, size_t blockCount);
  bool saveObject(std::unique_ptr<unsigned char> && data, size_t objectId);

  virtual void onRemovedFromMRU(size_t objectId) = 0;

  void addFreeBlock(size_t blockId, size_t blockCount);
  size_t getFreeBlockId(size_t blockCount);

  

protected:
  size_t _blockSize{0};
  size_t _incoreBlockUsageCount{0};
  size_t _incoreBlockUsageMax{0};
  size_t _blockCountPerIndex{0};
  size_t _nextStartBlockId{0};
  size_t _nextObjectId{0};

  std::string _basePathStorage;
  IndexedStoragePaths _indexPaths;
  IndexedFreeBlocks _freeBlocks;

  MRUType _mru;
  MemoryMap _memoryMap;
};

/**
 * A cache manager specialized for image tiles
 * All tiles in this image have a size multiple of a given base tile size.
 */
class TileCacheManager : public CacheManager, public std::enable_shared_from_this<TileCacheManager> {
public:
  using shared_ptr = std::shared_ptr<TileCacheManager>;
  using MapCachedTile = std::map<size_t, CachedTile::weak_pointer>;
public:

  TileCacheManager() = delete;

  /**
   * There is no explicit constructor.
   * We want to force the use of shared pointer for the manager
   * @param pathStorage the path to the directory where the file will be stored
   * @param tileWidth the base width of a tile
   * @param tileWidth the base height of a tile
   * @param maxTilesPerIndex the maximal number of tiles for a given file (give a maximal size for a cache file)
   * @retuurn the manager shared pointer
   */
  static std::shared_ptr<TileCacheManager> create(const std::string & pathStorage, size_t tileWidth, size_t tileHeight, size_t maxTilesPerIndex);

  /**
   * Notify the manager that a given tile was destroyed.
   * @param tileId the tile index which was destroyed
   * @note this method should not be explicitly called except by CachedTile
   */
  void notifyDestroy(size_t tileId);

  /**
   * Acquire a given tile
   * @param tileId the tile index to acquire
   * @return true if the tile was acquired
   */
  bool acquire(size_t tileId);

  /**
   * Acquire a given tile
   * @param width the requested tile size (less or equal to the base tile size)
   * @param height the requested tile size (less or equal to the base tile size)
   * @param blockCount the requested tile size in depth
   * @return a pointer to the cached tile created
   */
  std::shared_ptr<CachedTile> requireNewCachedTile(size_t width, size_t height, size_t blockCount);

  template <class T>
  std::shared_ptr<CachedTile> requireNewCachedTile(size_t width, size_t height) {
    return requireNewCachedTile(width, height, sizeof(T));
  }

  /**
   * Return the base tile width
   * @return a width
   */
  size_t getTileWidth() const {
    return _tileWidth;
  }

  /**
   * Return the base tile height
   * @return a width
   */
  size_t getTileHeight() const {
    return _tileHeight;
  }

protected:

  TileCacheManager(const std::string & pathStorage, size_t tileWidth, size_t tileHeight, size_t maxTilesPerIndex);

  virtual void onRemovedFromMRU(size_t objectId);

protected:

  size_t _tileWidth;
  size_t _tileHeight;

  MapCachedTile _objectMap;
};

}
}
