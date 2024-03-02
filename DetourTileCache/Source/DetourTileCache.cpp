#include "DetourTileCache.h"

#include <DetourAlloc.h>
#include <DetourAssert.h>
#include <DetourCommon.h>
#include <DetourNavMesh.h>
#include <DetourNavMeshBuilder.h>
#include <DetourTileCacheBuilder.h>

#include <cstring>

dtTileCache *dtAllocTileCache() {
  void *mem = dtAlloc(sizeof(dtTileCache), DT_ALLOC_PERM);
  if (!mem)
    return nullptr;
  return new (mem) dtTileCache;
}

void dtFreeTileCache(dtTileCache *tc) {
  if (!tc)
    return;
  tc->~dtTileCache();
  dtFree(tc);
}

namespace {
bool contains(const dtCompressedTileRef *a, const int n, const dtCompressedTileRef v) {
  for (int i = 0; i < n; ++i)
    if (a[i] == v)
      return true;
  return false;
}
} // namespace

inline int computeTileHash(const int x, const int y, const int mask) {
  constexpr uint32_t h1 = 0x8da6b343; // Large multiplicative constants;
  constexpr uint32_t h2 = 0xd8163841; // here arbitrarily chosen primes
  const uint32_t n = h1 * x + h2 * y;
  return static_cast<int>(n & mask);
}

struct NavMeshTileBuildContext {
  explicit NavMeshTileBuildContext(dtTileCacheAlloc *a) : alloc{a} {}
  ~NavMeshTileBuildContext() { purge(); }
  void purge() {
    dtFreeTileCacheLayer(alloc, layer);
    layer = nullptr;
    dtFreeTileCacheContourSet(alloc, lcset);
    lcset = nullptr;
    dtFreeTileCachePolyMesh(alloc, lmesh);
    lmesh = nullptr;
  }
  dtTileCacheLayer *layer{};
  dtTileCacheContourSet *lcset{};
  dtTileCachePolyMesh *lmesh{};
  dtTileCacheAlloc *alloc;
};

dtTileCache::dtTileCache() {
  std::memset(&m_params, 0, sizeof(m_params));
  std::memset(m_reqs, 0, sizeof(ObstacleRequest) * MAX_REQUESTS);
}

dtTileCache::~dtTileCache() {
  for (int i = 0; i < m_params.maxTiles; ++i) {
    if (m_tiles[i].flags & DT_COMPRESSEDTILE_FREE_DATA) {
      dtFree(m_tiles[i].data);
      m_tiles[i].data = nullptr;
    }
  }
  dtFree(m_obstacles);
  m_obstacles = nullptr;
  dtFree(m_posLookup);
  m_posLookup = nullptr;
  dtFree(m_tiles);
  m_tiles = nullptr;
  m_nreqs = 0;
  m_nupdate = 0;
}

const dtCompressedTile *dtTileCache::getTileByRef(const dtCompressedTileRef ref) const {
  if (!ref)
    return nullptr;
  const uint32_t tileIndex = decodeTileIdTile(ref);
  const uint32_t tileSalt = decodeTileIdSalt(ref);
  if (static_cast<int>(tileIndex) >= m_params.maxTiles)
    return nullptr;
  const dtCompressedTile *tile = &m_tiles[tileIndex];
  if (tile->salt != tileSalt)
    return nullptr;
  return tile;
}

dtStatus dtTileCache::init(const dtTileCacheParams *params,
                           dtTileCacheAlloc *talloc,
                           dtTileCacheCompressor *tcomp,
                           dtTileCacheMeshProcess *tmproc) {
  m_talloc = talloc;
  m_tcomp = tcomp;
  m_tmproc = tmproc;
  m_nreqs = 0;
  std::memcpy(&m_params, params, sizeof(m_params));

  // Alloc space for obstacles.
  m_obstacles = static_cast<dtTileCacheObstacle *>(dtAlloc(sizeof(dtTileCacheObstacle) * m_params.maxObstacles, DT_ALLOC_PERM));
  if (!m_obstacles)
    return DT_FAILURE | DT_OUT_OF_MEMORY;
  memset(m_obstacles, 0, sizeof(dtTileCacheObstacle) * m_params.maxObstacles);
  m_nextFreeObstacle = nullptr;
  for (int i = m_params.maxObstacles - 1; i >= 0; --i) {
    m_obstacles[i].salt = 1;
    m_obstacles[i].next = m_nextFreeObstacle;
    m_nextFreeObstacle = &m_obstacles[i];
  }

  // Init tiles
  m_tileLutSize = dtNextPow2(m_params.maxTiles / 4);
  if (!m_tileLutSize)
    m_tileLutSize = 1;
  m_tileLutMask = m_tileLutSize - 1;

  m_tiles = static_cast<dtCompressedTile *>(dtAlloc(sizeof(dtCompressedTile) * m_params.maxTiles, DT_ALLOC_PERM));
  if (!m_tiles)
    return DT_FAILURE | DT_OUT_OF_MEMORY;
  m_posLookup = static_cast<dtCompressedTile **>(dtAlloc(sizeof(dtCompressedTile *) * m_tileLutSize, DT_ALLOC_PERM));
  if (!m_posLookup)
    return DT_FAILURE | DT_OUT_OF_MEMORY;
  std::memset(m_tiles, 0, sizeof(dtCompressedTile) * m_params.maxTiles);
  std::memset(m_posLookup, 0, sizeof(dtCompressedTile *) * m_tileLutSize);
  m_nextFreeTile = nullptr;
  for (int i = m_params.maxTiles - 1; i >= 0; --i) {
    m_tiles[i].salt = 1;
    m_tiles[i].next = m_nextFreeTile;
    m_nextFreeTile = &m_tiles[i];
  }

  // Init ID generator values.
  m_tileBits = dtIlog2(dtNextPow2(static_cast<uint32_t>(m_params.maxTiles)));
  // Only allow 31 salt bits, since the salt mask is calculated using 32bit uint and it will overflow.
  m_saltBits = dtMin(static_cast<uint32_t>(31), 32 - m_tileBits);
  if (m_saltBits < 10)
    return DT_FAILURE | DT_INVALID_PARAM;

  return DT_SUCCESS;
}

int dtTileCache::getTilesAt(const int tx, const int ty, dtCompressedTileRef *tiles, const int maxTiles) const {
  int n = 0;

  // Find tile based on hash.
  const int h = computeTileHash(tx, ty, m_tileLutMask);
  const dtCompressedTile *tile = m_posLookup[h];
  while (tile) {
    if (tile->header &&
        tile->header->tx == tx &&
        tile->header->ty == ty) {
      if (n < maxTiles)
        tiles[n++] = getTileRef(tile);
    }
    tile = tile->next;
  }

  return n;
}

dtCompressedTile *dtTileCache::getTileAt(const int tx, const int ty, const int tlayer) const {
  // Find tile based on hash.
  const int h = computeTileHash(tx, ty, m_tileLutMask);
  dtCompressedTile *tile = m_posLookup[h];
  while (tile) {
    if (tile->header &&
        tile->header->tx == tx &&
        tile->header->ty == ty &&
        tile->header->tlayer == tlayer) {
      return tile;
    }
    tile = tile->next;
  }
  return nullptr;
}

dtCompressedTileRef dtTileCache::getTileRef(const dtCompressedTile *tile) const {
  if (!tile)
    return 0;
  const uint32_t it = static_cast<uint32_t>(tile - m_tiles);
  return encodeTileId(tile->salt, it);
}

dtObstacleRef dtTileCache::getObstacleRef(const dtTileCacheObstacle *ob) const {
  if (!ob)
    return 0;
  const uint32_t idx = static_cast<uint32_t>(ob - m_obstacles);
  return encodeObstacleId(ob->salt, idx);
}

const dtTileCacheObstacle *dtTileCache::getObstacleByRef(const dtObstacleRef ref) const {
  if (!ref)
    return nullptr;
  const uint32_t idx = decodeObstacleIdObstacle(ref);
  if (static_cast<int>(idx) >= m_params.maxObstacles)
    return nullptr;
  const dtTileCacheObstacle *ob = &m_obstacles[idx];
  if (ob->salt != decodeObstacleIdSalt(ref))
    return nullptr;
  return ob;
}

dtTileCacheMeshProcess::~dtTileCacheMeshProcess() {
  // Defined out of line to fix the weak v-tables warning
}

dtStatus dtTileCache::addTile(uint8_t *data, const int dataSize, const uint8_t flags, dtCompressedTileRef *result) {
  // Make sure the data is in right format.
  const dtTileCacheLayerHeader *header = reinterpret_cast<dtTileCacheLayerHeader *>(data);
  if (header->magic != DT_TILECACHE_MAGIC)
    return DT_FAILURE | DT_WRONG_MAGIC;
  if (header->version != DT_TILECACHE_VERSION)
    return DT_FAILURE | DT_WRONG_VERSION;

  // Make sure the location is free.
  if (getTileAt(header->tx, header->ty, header->tlayer))
    return DT_FAILURE;

  // Allocate a tile.
  dtCompressedTile *tile = nullptr;
  if (m_nextFreeTile) {
    tile = m_nextFreeTile;
    m_nextFreeTile = tile->next;
    tile->next = nullptr;
  }

  // Make sure we could allocate a tile.
  if (!tile)
    return DT_FAILURE | DT_OUT_OF_MEMORY;

  // Insert tile into the position lut.
  const int h = computeTileHash(header->tx, header->ty, m_tileLutMask);
  tile->next = m_posLookup[h];
  m_posLookup[h] = tile;

  // Init tile.
  const int headerSize = dtAlign4(sizeof(dtTileCacheLayerHeader));
  tile->header = reinterpret_cast<dtTileCacheLayerHeader *>(data);
  tile->data = data;
  tile->dataSize = dataSize;
  tile->compressed = tile->data + headerSize;
  tile->compressedSize = tile->dataSize - headerSize;
  tile->flags = flags;

  if (result)
    *result = getTileRef(tile);

  return DT_SUCCESS;
}

dtStatus dtTileCache::removeTile(const dtCompressedTileRef ref, uint8_t **data, int *dataSize) {
  if (!ref)
    return DT_FAILURE | DT_INVALID_PARAM;
  const uint32_t tileIndex = decodeTileIdTile(ref);
  const uint32_t tileSalt = decodeTileIdSalt(ref);
  if (static_cast<int>(tileIndex) >= m_params.maxTiles)
    return DT_FAILURE | DT_INVALID_PARAM;
  dtCompressedTile *tile = &m_tiles[tileIndex];
  if (tile->salt != tileSalt)
    return DT_FAILURE | DT_INVALID_PARAM;

  // Remove tile from hash lookup.
  const int h = computeTileHash(tile->header->tx, tile->header->ty, m_tileLutMask);
  dtCompressedTile *prev = nullptr;
  dtCompressedTile *cur = m_posLookup[h];
  while (cur) {
    if (cur == tile) {
      if (prev)
        prev->next = cur->next;
      else
        m_posLookup[h] = cur->next;
      break;
    }
    prev = cur;
    cur = cur->next;
  }

  // Reset tile.
  if (tile->flags & DT_COMPRESSEDTILE_FREE_DATA) {
    // Owns data
    dtFree(tile->data);
    tile->data = nullptr;
    tile->dataSize = 0;
    if (data)
      *data = nullptr;
    if (dataSize)
      *dataSize = 0;
  } else {
    if (data)
      *data = tile->data;
    if (dataSize)
      *dataSize = tile->dataSize;
  }

  tile->header = nullptr;
  tile->data = nullptr;
  tile->dataSize = 0;
  tile->compressed = nullptr;
  tile->compressedSize = 0;
  tile->flags = 0;

  // Update salt, salt should never be zero.
  tile->salt = (tile->salt + 1) & ((1 << m_saltBits) - 1);
  if (tile->salt == 0)
    tile->salt++;

  // Add to free list.
  tile->next = m_nextFreeTile;
  m_nextFreeTile = tile;

  return DT_SUCCESS;
}

dtStatus dtTileCache::addObstacle(const float *pos, const float radius, const float height, dtObstacleRef *result) {
  if (m_nreqs >= MAX_REQUESTS)
    return DT_FAILURE | DT_BUFFER_TOO_SMALL;

  dtTileCacheObstacle *ob = nullptr;
  if (m_nextFreeObstacle) {
    ob = m_nextFreeObstacle;
    m_nextFreeObstacle = ob->next;
    ob->next = nullptr;
  }
  if (!ob)
    return DT_FAILURE | DT_OUT_OF_MEMORY;

  const uint16_t salt = ob->salt;
  std::memset(ob, 0, sizeof(dtTileCacheObstacle));
  ob->salt = salt;
  ob->state = DT_OBSTACLE_PROCESSING;
  ob->type = DT_OBSTACLE_CYLINDER;
  dtVcopy(ob->cylinder.pos, pos);
  ob->cylinder.radius = radius;
  ob->cylinder.height = height;

  ObstacleRequest *req = &m_reqs[m_nreqs++];
  std::memset(req, 0, sizeof(ObstacleRequest));
  req->action = REQUEST_ADD;
  req->ref = getObstacleRef(ob);

  if (result)
    *result = req->ref;

  return DT_SUCCESS;
}

dtStatus dtTileCache::addBoxObstacle(const float *bmin, const float *bmax, dtObstacleRef *result) {
  if (m_nreqs >= MAX_REQUESTS)
    return DT_FAILURE | DT_BUFFER_TOO_SMALL;

  dtTileCacheObstacle *ob = nullptr;
  if (m_nextFreeObstacle) {
    ob = m_nextFreeObstacle;
    m_nextFreeObstacle = ob->next;
    ob->next = nullptr;
  }
  if (!ob)
    return DT_FAILURE | DT_OUT_OF_MEMORY;

  const uint16_t salt = ob->salt;
  std::memset(ob, 0, sizeof(dtTileCacheObstacle));
  ob->salt = salt;
  ob->state = DT_OBSTACLE_PROCESSING;
  ob->type = DT_OBSTACLE_BOX;
  dtVcopy(ob->box.bmin, bmin);
  dtVcopy(ob->box.bmax, bmax);

  ObstacleRequest *req = &m_reqs[m_nreqs++];
  std::memset(req, 0, sizeof(ObstacleRequest));
  req->action = REQUEST_ADD;
  req->ref = getObstacleRef(ob);

  if (result)
    *result = req->ref;

  return DT_SUCCESS;
}

dtStatus dtTileCache::addBoxObstacle(const float *center, const float *halfExtents, const float yRadians, dtObstacleRef *result) {
  if (m_nreqs >= MAX_REQUESTS)
    return DT_FAILURE | DT_BUFFER_TOO_SMALL;

  dtTileCacheObstacle *ob = nullptr;
  if (m_nextFreeObstacle) {
    ob = m_nextFreeObstacle;
    m_nextFreeObstacle = ob->next;
    ob->next = nullptr;
  }
  if (!ob)
    return DT_FAILURE | DT_OUT_OF_MEMORY;

  const uint16_t salt = ob->salt;
  std::memset(ob, 0, sizeof(dtTileCacheObstacle));
  ob->salt = salt;
  ob->state = DT_OBSTACLE_PROCESSING;
  ob->type = DT_OBSTACLE_ORIENTED_BOX;
  dtVcopy(ob->orientedBox.center, center);
  dtVcopy(ob->orientedBox.halfExtents, halfExtents);

  const float coshalf = cosf(0.5f * yRadians);
  const float sinhalf = sinf(-0.5f * yRadians);
  ob->orientedBox.rotAux[0] = coshalf * sinhalf;
  ob->orientedBox.rotAux[1] = coshalf * coshalf - 0.5f;

  ObstacleRequest *req = &m_reqs[m_nreqs++];
  std::memset(req, 0, sizeof(ObstacleRequest));
  req->action = REQUEST_ADD;
  req->ref = getObstacleRef(ob);

  if (result)
    *result = req->ref;

  return DT_SUCCESS;
}

dtStatus dtTileCache::removeObstacle(const dtObstacleRef ref) {
  if (!ref)
    return DT_SUCCESS;
  if (m_nreqs >= MAX_REQUESTS)
    return DT_FAILURE | DT_BUFFER_TOO_SMALL;

  ObstacleRequest *req = &m_reqs[m_nreqs++];
  std::memset(req, 0, sizeof(ObstacleRequest));
  req->action = REQUEST_REMOVE;
  req->ref = ref;

  return DT_SUCCESS;
}

dtStatus dtTileCache::queryTiles(const float *bmin, const float *bmax,
                                 dtCompressedTileRef *results, int *resultCount, const int maxResults) const {
  int n = 0;

  const float tw = m_params.width * m_params.cs;
  const float th = m_params.height * m_params.cs;
  const int tx0 = static_cast<int>(dtMathFloorf((bmin[0] - m_params.orig[0]) / tw));
  const int tx1 = static_cast<int>(dtMathFloorf((bmax[0] - m_params.orig[0]) / tw));
  const int ty0 = static_cast<int>(dtMathFloorf((bmin[2] - m_params.orig[2]) / th));
  const int ty1 = static_cast<int>(dtMathFloorf((bmax[2] - m_params.orig[2]) / th));

  for (int ty = ty0; ty <= ty1; ++ty) {
    for (int tx = tx0; tx <= tx1; ++tx) {
      constexpr int MAX_TILES = 32;
      dtCompressedTileRef tiles[MAX_TILES];
      const int ntiles = getTilesAt(tx, ty, tiles, MAX_TILES);

      for (int i = 0; i < ntiles; ++i) {
        const dtCompressedTile *tile = &m_tiles[decodeTileIdTile(tiles[i])];
        float tbmin[3], tbmax[3];
        calcTightTileBounds(tile->header, tbmin, tbmax);

        if (dtOverlapBounds(bmin, bmax, tbmin, tbmax)) {
          if (n < maxResults)
            results[n++] = tiles[i];
        }
      }
    }
  }

  *resultCount = n;

  return DT_SUCCESS;
}

dtStatus dtTileCache::update(const float /*dt*/, dtNavMesh *navmesh,
                             bool *upToDate) {
  if (m_nupdate == 0) {
    // Process requests.
    for (int i = 0; i < m_nreqs; ++i) {
      const ObstacleRequest *req = &m_reqs[i];

      const uint32_t idx = decodeObstacleIdObstacle(req->ref);
      if (static_cast<int>(idx) >= m_params.maxObstacles)
        continue;
      dtTileCacheObstacle *ob = &m_obstacles[idx];
      if (ob->salt != decodeObstacleIdSalt(req->ref))
        continue;

      if (req->action == REQUEST_ADD) {
        // Find touched tiles.
        float bmin[3], bmax[3];
        getObstacleBounds(ob, bmin, bmax);

        int ntouched = 0;
        queryTiles(bmin, bmax, ob->touched, &ntouched, DT_MAX_TOUCHED_TILES);
        ob->ntouched = static_cast<uint8_t>(ntouched);
        // Add tiles to update list.
        ob->npending = 0;
        for (int j = 0; j < ob->ntouched; ++j) {
          if (m_nupdate < MAX_UPDATE) {
            if (!contains(m_update, m_nupdate, ob->touched[j]))
              m_update[m_nupdate++] = ob->touched[j];
            ob->pending[ob->npending++] = ob->touched[j];
          }
        }
      } else if (req->action == REQUEST_REMOVE) {
        // Prepare to remove obstacle.
        ob->state = DT_OBSTACLE_REMOVING;
        // Add tiles to update list.
        ob->npending = 0;
        for (int j = 0; j < ob->ntouched; ++j) {
          if (m_nupdate < MAX_UPDATE) {
            if (!contains(m_update, m_nupdate, ob->touched[j]))
              m_update[m_nupdate++] = ob->touched[j];
            ob->pending[ob->npending++] = ob->touched[j];
          }
        }
      }
    }

    m_nreqs = 0;
  }

  dtStatus status = DT_SUCCESS;
  // Process updates
  if (m_nupdate) {
    // Build mesh
    const dtCompressedTileRef ref = m_update[0];
    status = buildNavMeshTile(ref, navmesh);
    m_nupdate--;
    if (m_nupdate > 0)
      std::memmove(m_update, m_update + 1, m_nupdate * sizeof(dtCompressedTileRef));

    // Update obstacle states.
    for (int i = 0; i < m_params.maxObstacles; ++i) {
      dtTileCacheObstacle *ob = &m_obstacles[i];
      if (ob->state == DT_OBSTACLE_PROCESSING || ob->state == DT_OBSTACLE_REMOVING) {
        // Remove handled tile from pending list.
        for (int j = 0; j < static_cast<int>(ob->npending); j++) {
          if (ob->pending[j] == ref) {
            ob->pending[j] = ob->pending[static_cast<int>(ob->npending) - 1];
            ob->npending--;
            break;
          }
        }

        // If all pending tiles processed, change state.
        if (ob->npending == 0) {
          if (ob->state == DT_OBSTACLE_PROCESSING) {
            ob->state = DT_OBSTACLE_PROCESSED;
          } else if (ob->state == DT_OBSTACLE_REMOVING) {
            ob->state = DT_OBSTACLE_EMPTY;
            // Update salt, salt should never be zero.
            ob->salt = (ob->salt + 1) & ((1 << 16) - 1);
            if (ob->salt == 0)
              ob->salt++;
            // Return obstacle to free list.
            ob->next = m_nextFreeObstacle;
            m_nextFreeObstacle = ob;
          }
        }
      }
    }
  }

  if (upToDate)
    *upToDate = m_nupdate == 0 && m_nreqs == 0;

  return status;
}

dtStatus dtTileCache::buildNavMeshTilesAt(const int tx, const int ty, dtNavMesh *navmesh) const {
  constexpr int MAX_TILES = 32;
  dtCompressedTileRef tiles[MAX_TILES];
  const int ntiles = getTilesAt(tx, ty, tiles, MAX_TILES);

  for (int i = 0; i < ntiles; ++i) {
    const dtStatus status = buildNavMeshTile(tiles[i], navmesh);
    if (dtStatusFailed(status))
      return status;
  }

  return DT_SUCCESS;
}

dtStatus dtTileCache::buildNavMeshTile(const dtCompressedTileRef ref, dtNavMesh *navmesh) const {
  dtAssert(m_talloc);
  dtAssert(m_tcomp);
  if (!m_talloc || !m_tcomp)
    return DT_FAILURE | DT_INVALID_PARAM;

  const uint32_t idx = decodeTileIdTile(ref);
  if (idx > static_cast<uint32_t>(m_params.maxTiles))
    return DT_FAILURE | DT_INVALID_PARAM;
  const dtCompressedTile *tile = &m_tiles[idx];
  if (tile->salt != decodeTileIdSalt(ref))
    return DT_FAILURE | DT_INVALID_PARAM;

  m_talloc->reset();

  NavMeshTileBuildContext bc(m_talloc);
  const int walkableClimbVx = static_cast<int>(m_params.walkableClimb / m_params.ch);

  // Decompress tile layer data.
  dtStatus status = dtDecompressTileCacheLayer(m_talloc, m_tcomp, tile->data, tile->dataSize, &bc.layer);
  if (dtStatusFailed(status))
    return status;

  // Rasterize obstacles.
  for (int i = 0; i < m_params.maxObstacles; ++i) {
    const dtTileCacheObstacle *ob = &m_obstacles[i];
    if (ob->state == DT_OBSTACLE_EMPTY || ob->state == DT_OBSTACLE_REMOVING)
      continue;
    if (contains(ob->touched, ob->ntouched, ref)) {
      if (ob->type == DT_OBSTACLE_CYLINDER) {
        dtMarkCylinderArea(*bc.layer, tile->header->bmin, m_params.cs, m_params.ch,
                           ob->cylinder.pos, ob->cylinder.radius, ob->cylinder.height, 0);
      } else if (ob->type == DT_OBSTACLE_BOX) {
        dtMarkBoxArea(*bc.layer, tile->header->bmin, m_params.cs, m_params.ch,
                      ob->box.bmin, ob->box.bmax, 0);
      } else if (ob->type == DT_OBSTACLE_ORIENTED_BOX) {
        dtMarkBoxArea(*bc.layer, tile->header->bmin, m_params.cs, m_params.ch,
                      ob->orientedBox.center, ob->orientedBox.halfExtents, ob->orientedBox.rotAux, 0);
      }
    }
  }

  // Build navmesh
  status = dtBuildTileCacheRegions(m_talloc, *bc.layer, walkableClimbVx);
  if (dtStatusFailed(status))
    return status;

  bc.lcset = dtAllocTileCacheContourSet(m_talloc);
  if (!bc.lcset)
    return DT_FAILURE | DT_OUT_OF_MEMORY;
  status = dtBuildTileCacheContours(m_talloc, *bc.layer, walkableClimbVx,
                                    m_params.maxSimplificationError, *bc.lcset);
  if (dtStatusFailed(status))
    return status;

  bc.lmesh = dtAllocTileCachePolyMesh(m_talloc);
  if (!bc.lmesh)
    return DT_FAILURE | DT_OUT_OF_MEMORY;
  status = dtBuildTileCachePolyMesh(m_talloc, *bc.lcset, *bc.lmesh);
  if (dtStatusFailed(status))
    return status;

  // Early out if the mesh tile is empty.
  if (!bc.lmesh->npolys) {
    // Remove existing tile.
    navmesh->removeTile(navmesh->getTileRefAt(tile->header->tx, tile->header->ty, tile->header->tlayer), nullptr, nullptr);
    return DT_SUCCESS;
  }

  dtNavMeshCreateParams params = {};
  params.verts = bc.lmesh->verts;
  params.vertCount = bc.lmesh->nverts;
  params.polys = bc.lmesh->polys;
  params.polyAreas = bc.lmesh->areas;
  params.polyFlags = bc.lmesh->flags;
  params.polyCount = bc.lmesh->npolys;
  params.nvp = DT_VERTS_PER_POLYGON;
  params.walkableHeight = m_params.walkableHeight;
  params.walkableRadius = m_params.walkableRadius;
  params.walkableClimb = m_params.walkableClimb;
  params.tileX = tile->header->tx;
  params.tileY = tile->header->ty;
  params.tileLayer = tile->header->tlayer;
  params.cs = m_params.cs;
  params.ch = m_params.ch;
  params.buildBvTree = false;
  dtVcopy(params.bmin, tile->header->bmin);
  dtVcopy(params.bmax, tile->header->bmax);

  if (m_tmproc) {
    m_tmproc->process(&params, bc.lmesh->areas, bc.lmesh->flags);
  }

  uint8_t *navData = nullptr;
  int navDataSize = 0;
  if (!dtCreateNavMeshData(&params, &navData, &navDataSize))
    return DT_FAILURE;

  // Remove existing tile.
  navmesh->removeTile(navmesh->getTileRefAt(tile->header->tx, tile->header->ty, tile->header->tlayer), nullptr, nullptr);

  // Add new tile, or leave the location empty.
  if (navData) {
    // Let the navmesh own the data.
    status = navmesh->addTile(navData, navDataSize, DT_TILE_FREE_DATA, 0, nullptr);
    if (dtStatusFailed(status)) {
      dtFree(navData);
      return status;
    }
  }

  return DT_SUCCESS;
}

void dtTileCache::calcTightTileBounds(const dtTileCacheLayerHeader *header, float *bmin, float *bmax) const {
  const float cs = m_params.cs;
  bmin[0] = header->bmin[0] + header->minx * cs;
  bmin[1] = header->bmin[1];
  bmin[2] = header->bmin[2] + header->miny * cs;
  bmax[0] = header->bmin[0] + (header->maxx + 1) * cs;
  bmax[1] = header->bmax[1];
  bmax[2] = header->bmin[2] + (header->maxy + 1) * cs;
}

void dtTileCache::getObstacleBounds(const dtTileCacheObstacle *ob, float *bmin, float *bmax) {
  if (ob->type == DT_OBSTACLE_CYLINDER) {
    const dtObstacleCylinder &cl = ob->cylinder;
    bmin[0] = cl.pos[0] - cl.radius;
    bmin[1] = cl.pos[1];
    bmin[2] = cl.pos[2] - cl.radius;
    bmax[0] = cl.pos[0] + cl.radius;
    bmax[1] = cl.pos[1] + cl.height;
    bmax[2] = cl.pos[2] + cl.radius;
  } else if (ob->type == DT_OBSTACLE_BOX) {
    dtVcopy(bmin, ob->box.bmin);
    dtVcopy(bmax, ob->box.bmax);
  } else if (ob->type == DT_OBSTACLE_ORIENTED_BOX) {
    const dtObstacleOrientedBox &orientedBox = ob->orientedBox;
    const float maxr = 1.41f * dtMax(orientedBox.halfExtents[0], orientedBox.halfExtents[2]);
    bmin[0] = orientedBox.center[0] - maxr;
    bmax[0] = orientedBox.center[0] + maxr;
    bmin[1] = orientedBox.center[1] - orientedBox.halfExtents[1];
    bmax[1] = orientedBox.center[1] + orientedBox.halfExtents[1];
    bmin[2] = orientedBox.center[2] - maxr;
    bmax[2] = orientedBox.center[2] + maxr;
  }
}
