#include "cache.hpp"

#include <aliceVision/system/Logger.hpp>

#include <boost/filesystem.hpp>


namespace aliceVision
{
namespace image
{

CacheManager::CacheManager(const std::string & pathStorage, size_t blockSize, size_t maxBlocksPerIndex) :
_blockSize(blockSize),
_incoreBlockUsageCount(0),
_incoreBlockUsageMax(10),
_blockCountPerIndex(maxBlocksPerIndex),
_basePathStorage(pathStorage)
{
  wipe();
}

CacheManager::~CacheManager() {
  wipe();
}

void CacheManager::wipe() {
  deleteIndexFiles();

  _mru.clear();
  
  _incoreBlockUsageCount = 0;
  _nextStartBlockId = 0;
  _nextObjectId = 0;
}


void CacheManager::setMaxMemory(size_t maxMemorySize) {
  _incoreBlockUsageMax = maxMemorySize / _blockSize;
}

void CacheManager::setInCoreMaxObjectCount(size_t max) {
  _incoreBlockUsageMax = max;
}

std::string CacheManager::getPathForIndex(size_t indexId) {

  if (_indexPaths.find(indexId) == _indexPaths.end()) {

    boost::filesystem::path path(_basePathStorage);
    path /= boost::filesystem::unique_path();
    path += ".idx";

    _indexPaths[indexId] = path.string();
  }
  
  return _indexPaths[indexId];
}

void CacheManager::deleteIndexFiles() {

  std::size_t cacheSize = 0;
  for (std::pair<const size_t, std::string> & p : _indexPaths)
  {
    const std::size_t s = boost::filesystem::file_size(p.second);
    ALICEVISION_LOG_TRACE("CacheManager::deleteIndexFiles: '" << p.second << "': " << s / (1024*1024) << "MB.");
    cacheSize += s;
  }
  ALICEVISION_LOG_DEBUG("CacheManager::deleteIndexFiles: cache size is " << cacheSize / (1024*1024) << "MB.");

  // Remove all cache files
  for (std::pair<const size_t, std::string> & p : _indexPaths)
  {
    boost::filesystem::path path(p.second);
    boost::filesystem::remove(path);
  }

  // Remove list of cache files
  _indexPaths.clear();
}

bool CacheManager::prepareBlockGroup(size_t startBlockId, size_t blocksCount) {

  size_t index_id = startBlockId / _blockCountPerIndex;
  size_t block_id_in_index = startBlockId % _blockCountPerIndex;
  size_t position_in_index = block_id_in_index * _blockSize;
  size_t len = _blockSize * blocksCount;

  std::string pathname = getPathForIndex(index_id);
  boost::filesystem::path path(pathname);

  std::ofstream file_index;
  if (boost::filesystem::exists(path)) {
    file_index.open(pathname, std::ios::binary  | std::ios::out | std::ios::in);
  }
  else {
    file_index.open(pathname, std::ios::binary  | std::ios::out);
  }
  
  if (!file_index.is_open()) {
    return false;
  }

  ALICEVISION_LOG_TRACE("CacheManager::prepareBlockGroup: " << blocksCount * _blockSize << " bytes to '" << path << "'.");

  /*write a dummy byte at the end of the tile to "book" this place on disk*/
  file_index.seekp(position_in_index + len - 1, file_index.beg);
  if (!file_index) {
    return false;
  }

  char c[1];
  c[0] = 0xff;
  file_index.write(c, 1);

  return true;
}

std::unique_ptr<unsigned char> CacheManager::load(size_t startBlockId, size_t blockCount) {
    
  const size_t indexId = startBlockId / _blockCountPerIndex;
  const size_t blockIdInIndex = startBlockId % _blockCountPerIndex;
  const size_t positionInIndex = blockIdInIndex * _blockSize;
  const size_t groupLength = _blockSize * blockCount;

  const std::string path = getPathForIndex(indexId);
  
  std::ifstream file_index(path, std::ios::binary);
  if (!file_index.is_open()) {
    return std::unique_ptr<unsigned char>();
  }  

  file_index.seekg(positionInIndex, std::ios::beg);
  if (file_index.fail()) {
    return std::unique_ptr<unsigned char>();
  }

  ALICEVISION_LOG_TRACE("CacheManager::load: read " << groupLength << " bytes from '" << path << "' at position " << positionInIndex << ".");

  std::unique_ptr<unsigned char> data(new unsigned char[groupLength]);
  file_index.read(reinterpret_cast<char *>(data.get()), groupLength);
  if (!file_index) {
    return std::unique_ptr<unsigned char>();
  }

  return data;
}

bool CacheManager::save(std::unique_ptr<unsigned char> && data, size_t startBlockId, size_t blockCount) {
    
  const size_t indexId = startBlockId / _blockCountPerIndex;
  const size_t blockIdInIndex = startBlockId % _blockCountPerIndex;
  const size_t positionInIndex = blockIdInIndex * _blockSize;
  const size_t groupLength = _blockSize * blockCount;

  const std::string path = getPathForIndex(indexId);

  std::ofstream file_index(path, std::ios::binary | std::ios::out | std::ios::in);
  if (!file_index.is_open()) {
    return false;
  }

  file_index.seekp(positionInIndex, std::ios::beg);
  if (file_index.fail()) {
    return false;
  }

  const unsigned char * bytesToWrite = data.get();
  if (bytesToWrite == nullptr) {
    return false;
  }
  else {
    // Write data
    ALICEVISION_LOG_TRACE("CacheManager::save: write " << groupLength << " bytes to '" << path << "' at position " << positionInIndex << ".");
    file_index.write(reinterpret_cast<const char*>(bytesToWrite), groupLength);
  }

  if (!file_index) {
    return false;
  }

  file_index.close();

  return true;
}

size_t CacheManager::getFreeBlockId(size_t blockCount) {
  
  size_t ret;
  std::list<size_t> & freeBlocksForCount = _freeBlocks[blockCount];

  if (freeBlocksForCount.empty()) {
    ret = _nextStartBlockId;
    _nextStartBlockId += blockCount;
  }
  else {
    ret = freeBlocksForCount.front();
    freeBlocksForCount.pop_front();
  }

  return ret;
}

bool CacheManager::createObject(size_t & objectId, size_t blockCount) {
    
  objectId = _nextObjectId;
  _nextObjectId++;

  MemoryItem item;
  item.startBlockId = ~0;
  item.countBlock = blockCount;
  _memoryMap[objectId] = item;

  return true;
}

bool CacheManager::acquireObject(std::unique_ptr<unsigned char> & data, size_t objectId) {

  MemoryMap::iterator itfind = _memoryMap.find(objectId);
  if (itfind == _memoryMap.end()) {
    return false;
  }

  MemoryItem memitem = itfind->second;  
  MRUItem item;
  item.objectId = objectId;
  item.objectSize = memitem.countBlock;

  /* Check mru */
  std::pair<MRUType::iterator, bool> p = _mru.push_front(item);
  if (p.second) {
    
    /*
    Effectively added to the mru.
    This means that we have to find this in the storage
    */
    if (memitem.startBlockId == ~0) {
      std::unique_ptr<unsigned char> buffer(new unsigned char[_blockSize * memitem.countBlock]);
      data = std::move(buffer);
    }
    else {
      data = std::move(load(memitem.startBlockId, memitem.countBlock));
    }

    /*Update memory usage*/
    _incoreBlockUsageCount += memitem.countBlock;
  }
  else {
    /*
    The uid is present in the mru, put it in first position.
    Note that the item may contain a previously deleted info
    */
    _mru.relocate(_mru.begin(), p.first);
  }

  while (_incoreBlockUsageCount > _incoreBlockUsageMax && _mru.size() > 1) {
    
    MRUItem item = _mru.back();

    /*Remove item from mru*/
    _mru.pop_back();

    /*Update memory usage*/
    _incoreBlockUsageCount -= item.objectSize;

    onRemovedFromMRU(item.objectId);
  }

  return true;
}

bool CacheManager::saveObject(std::unique_ptr<unsigned char> && data, size_t objectId) {
  
  MemoryMap::iterator itfind = _memoryMap.find(objectId);
  if (itfind == _memoryMap.end()) {
    return false;
  }

  MemoryItem item = itfind->second;

  if (itfind->second.startBlockId == ~0) {
    
    item.startBlockId = getFreeBlockId(item.countBlock);
    _memoryMap[objectId] = item;

    prepareBlockGroup(item.startBlockId, item.countBlock);
  }

  if (!save(std::move(data), item.startBlockId, item.countBlock)) {
    return false;
  }

  return true;
}

void CacheManager::addFreeBlock(size_t blockId, size_t blockCount) {

  _freeBlocks[blockCount].push_back(blockId);
}

size_t CacheManager::getActiveBlocks() const {
  return _memoryMap.size();
}

CachedTile::~CachedTile() {
  
  std::shared_ptr<TileCacheManager> manager = _manager.lock();
  if (manager) {
    manager->notifyDestroy(_uid);
  }
}

bool CachedTile::acquire() {

  std::shared_ptr<TileCacheManager> manager = _manager.lock();
  if (!manager) {
    return false;
  }
  
  
  return manager->acquire(_uid);

  return true;
}

TileCacheManager::TileCacheManager(const std::string & pathStorage, size_t tileWidth, size_t tileHeight, size_t maxTilesPerIndex) :
CacheManager(pathStorage, tileWidth * tileHeight, maxTilesPerIndex),
_tileWidth(tileWidth), _tileHeight(tileHeight)
{
}

static unsigned int bitCount (unsigned int value) 
{
    unsigned int count = 0;

    while (value > 0) 
    {
        if ((value & 1) == 1)
        {
            count++;
        }

        value >>= 1;
    }

    return count;
}


std::shared_ptr<TileCacheManager> TileCacheManager::create(const std::string & path_storage, size_t tileWidth, size_t tileHeight, size_t maxTilesPerIndex) {

  if (bitCount(tileWidth) != 1) 
  {
    return nullptr;
  }

  if (bitCount(tileHeight) != 1) 
  {
    return nullptr;
  }

  TileCacheManager * obj = new TileCacheManager(path_storage, tileWidth, tileHeight, maxTilesPerIndex);
    
  return std::shared_ptr<TileCacheManager>(obj);
}

std::shared_ptr<CachedTile> TileCacheManager::requireNewCachedTile(size_t width, size_t height, size_t blockCount) {


  CachedTile::smart_pointer ret;
  size_t uid;

  if (!CacheManager::createObject(uid, blockCount)) {
    return ret;
  }

  /* Create container */
  std::shared_ptr<TileCacheManager> sptr = shared_from_this();
  ret.reset(new CachedTile(sptr, uid, _tileWidth, _tileHeight, width, height, blockCount));

  /*Store weak pointer internally*/
  _objectMap[uid] = ret;

  return ret;
}

void TileCacheManager::notifyDestroy(size_t tileId) {
  
  /* Remove weak pointer */
  _objectMap.erase(tileId);

  /* Remove map from object to block id*/
  MemoryMap::iterator it = _memoryMap.find(tileId);
  if (it == _memoryMap.end()) {
    return;
  }
  size_t blockId = it->second.startBlockId;
  size_t blockCount = it->second.countBlock;
  _memoryMap.erase(it);

  /*If memory block is valid*/
  if (blockId != ~0) {

    /*Add block to list of available*/
    addFreeBlock(blockId, blockCount);
  }
}

bool TileCacheManager::acquire(size_t tileId) {


  MapCachedTile::iterator itfind = _objectMap.find(tileId);
  if (itfind == _objectMap.end()) {
    return false;
  }

  CachedTile::smart_pointer tile = itfind->second.lock();
  if (!tile) {
    return false;
  }
  
  /*Acquire the object*/
  std::unique_ptr<unsigned char> content = tile->getData();
  if (!CacheManager::acquireObject(content, tileId)) {
    return false;
  }

  /*Update tile data*/
  tile->setData(std::move(content));


  return true;
}

void TileCacheManager::onRemovedFromMRU(size_t objectId) {

  MapCachedTile::iterator itfind = _objectMap.find(objectId);
  if (itfind == _objectMap.end()) {
    return;
  }

  CachedTile::smart_pointer tile = itfind->second.lock();
  if (!tile) {
    return;
  }  

  /* Save object and set the tile data to nullptr */
  std::unique_ptr<unsigned char> content = tile->getData();
  CacheManager::saveObject(std::move(content), objectId);
} 

}
}
