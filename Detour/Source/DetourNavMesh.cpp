//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//

#include "DetourNavMesh.h"

#include "DetourAlloc.h"
#include "DetourAssert.h"
#include "DetourCommon.h"

#include <cfloat>
#include <cstring>

inline bool overlapSlabs(const float *amin, const float *amax,
                         const float *bmin, const float *bmax,
                         const float px, const float py) {
  // Check for horizontal overlap.
  // The segment is shrunken a little so that slabs which touch
  // at end points are not connected.
  const float minx = dtMax(amin[0] + px, bmin[0] + px);
  const float maxx = dtMin(amax[0] - px, bmax[0] - px);
  if (minx > maxx)
    return false;

  // Check vertical overlap.
  const float ad = (amax[1] - amin[1]) / (amax[0] - amin[0]);
  const float ak = amin[1] - ad * amin[0];
  const float bd = (bmax[1] - bmin[1]) / (bmax[0] - bmin[0]);
  const float bk = bmin[1] - bd * bmin[0];
  const float aminy = ad * minx + ak;
  const float amaxy = ad * maxx + ak;
  const float bminy = bd * minx + bk;
  const float bmaxy = bd * maxx + bk;
  const float dmin = bminy - aminy;
  const float dmax = bmaxy - amaxy;

  // Crossing segments always overlap.
  if (dmin * dmax < 0)
    return true;

  // Check for overlap at endpoints.
  const float thr = dtSqr(py * 2);
  if (dmin * dmin <= thr || dmax * dmax <= thr)
    return true;

  return false;
}

namespace {
float getSlabCoord(const float *va, const int side) {
  if (side == 0 || side == 4)
    return va[0];
  if (side == 2 || side == 6)
    return va[2];
  return 0;
}

void calcSlabEndPoints(const float *va, const float *vb, float *bmin, float *bmax, const int side) {
  if (side == 0 || side == 4) {
    if (va[2] < vb[2]) {
      bmin[0] = va[2];
      bmin[1] = va[1];
      bmax[0] = vb[2];
      bmax[1] = vb[1];
    } else {
      bmin[0] = vb[2];
      bmin[1] = vb[1];
      bmax[0] = va[2];
      bmax[1] = va[1];
    }
  } else if (side == 2 || side == 6) {
    if (va[0] < vb[0]) {
      bmin[0] = va[0];
      bmin[1] = va[1];
      bmax[0] = vb[0];
      bmax[1] = vb[1];
    } else {
      bmin[0] = vb[0];
      bmin[1] = vb[1];
      bmax[0] = va[0];
      bmax[1] = va[1];
    }
  }
}
} // namespace

inline int computeTileHash(const int x, const int y, const int mask) {
  constexpr uint32_t h1 = 0x8da6b343; // Large multiplicative constants;
  constexpr uint32_t h2 = 0xd8163841; // here arbitrarily chosen primes
  const uint32_t n = h1 * x + h2 * y;
  return static_cast<int>(n & mask);
}

inline uint32_t allocLink(dtMeshTile *tile) {
  if (tile->linksFreeList == DT_NULL_LINK)
    return DT_NULL_LINK;
  const uint32_t link = tile->linksFreeList;
  tile->linksFreeList = tile->links[link].next;
  return link;
}

inline void freeLink(dtMeshTile *tile, const uint32_t link) {
  tile->links[link].next = tile->linksFreeList;
  tile->linksFreeList = link;
}


dtNavMesh *dtAllocNavMesh() {
  void *mem = dtAlloc(sizeof(dtNavMesh), DT_ALLOC_PERM);
  if (!mem)
    return nullptr;
  return new(mem) dtNavMesh;
}

/// @par
///
/// This function will only free the memory for tiles with the #DT_TILE_FREE_DATA
/// flag set.
void dtFreeNavMesh(dtNavMesh *navmesh) {
  if (!navmesh)
    return;
  navmesh->~dtNavMesh();
  dtFree(navmesh);
}

//////////////////////////////////////////////////////////////////////////////////////////

/**
@class dtNavMesh

The navigation mesh consists of one or more tiles defining three primary types of structural data:

A polygon mesh which defines most of the navigation graph. (See rcPolyMesh for its structure.)
A detail mesh used for determining surface height on the polygon mesh. (See rcPolyMeshDetail for its structure.)
Off-mesh connections, which define custom point-to-point edges within the navigation graph.

The general build process is as follows:

-# Create rcPolyMesh and rcPolyMeshDetail data using the Recast build pipeline.
-# Optionally, create off-mesh connection data.
-# Combine the source data into a dtNavMeshCreateParams structure.
-# Create a tile data array using dtCreateNavMeshData().
-# Allocate at dtNavMesh object and initialize it. (For single tile navigation meshes,
   the tile data is loaded during this step.)
-# For multi-tile navigation meshes, load the tile data using dtNavMesh::addTile().

Notes:

- This class is usually used in conjunction with the dtNavMeshQuery class for pathfinding.
- Technically, all navigation meshes are tiled. A 'solo' mesh is simply a navigation mesh initialized
  to have only a single tile.
- This class does not implement any asynchronous methods. So the ::dtStatus result of all methods will
  always contain either a success or failure flag.

@see dtNavMeshQuery, dtCreateNavMeshData, dtNavMeshCreateParams, #dtAllocNavMesh, #dtFreeNavMesh
*/

dtNavMesh::dtNavMesh() :
  m_tileWidth(0),
  m_tileHeight(0),
  m_maxTiles(0),
  m_tileLutSize(0),
  m_tileLutMask(0),
  m_posLookup(nullptr),
  m_nextFree(nullptr),
  m_tiles(nullptr) {
#ifndef DT_POLYREF64
  m_saltBits = 0;
  m_tileBits = 0;
  m_polyBits = 0;
#endif
  std::memset(&m_params, 0, sizeof(dtNavMeshParams));
  m_orig[0] = 0;
  m_orig[1] = 0;
  m_orig[2] = 0;
}

dtNavMesh::~dtNavMesh() {
  for (int i = 0; i < m_maxTiles; ++i) {
    if (m_tiles[i].flags & DT_TILE_FREE_DATA) {
      dtFree(m_tiles[i].data);
      m_tiles[i].data = nullptr;
      m_tiles[i].dataSize = 0;
    }
  }
  dtFree(m_posLookup);
  dtFree(m_tiles);
}

dtStatus dtNavMesh::init(const dtNavMeshParams *params) {
  std::memcpy(&m_params, params, sizeof(dtNavMeshParams));
  dtVcopy(m_orig, params->orig);
  m_tileWidth = params->tileWidth;
  m_tileHeight = params->tileHeight;

  // Init tiles
  m_maxTiles = params->maxTiles;
  m_tileLutSize = dtNextPow2(params->maxTiles / 4);
  if (!m_tileLutSize)
    m_tileLutSize = 1;
  m_tileLutMask = m_tileLutSize - 1;

  m_tiles = static_cast<dtMeshTile *>(dtAlloc(sizeof(dtMeshTile) * m_maxTiles, DT_ALLOC_PERM));
  if (!m_tiles)
    return DT_FAILURE | DT_OUT_OF_MEMORY;
  m_posLookup = static_cast<dtMeshTile **>(dtAlloc(sizeof(dtMeshTile *) * m_tileLutSize, DT_ALLOC_PERM));
  if (!m_posLookup)
    return DT_FAILURE | DT_OUT_OF_MEMORY;
  std::memset(m_tiles, 0, sizeof(dtMeshTile) * m_maxTiles);
  std::memset(m_posLookup, 0, sizeof(dtMeshTile *) * m_tileLutSize);
  m_nextFree = nullptr;
  for (int i = m_maxTiles - 1; i >= 0; --i) {
    m_tiles[i].salt = 1;
    m_tiles[i].next = m_nextFree;
    m_nextFree = &m_tiles[i];
  }

  // Init ID generator values.
#ifndef DT_POLYREF64
  m_tileBits = dtIlog2(dtNextPow2(static_cast<uint32_t>(params->maxTiles)));
  m_polyBits = dtIlog2(dtNextPow2(static_cast<uint32_t>(params->maxPolys)));
  // Only allow 31 salt bits, since the salt mask is calculated using 32bit uint and it will overflow.
  m_saltBits = dtMin(static_cast<uint32_t>(31), 32 - m_tileBits - m_polyBits);

  if (m_saltBits < 10)
    return DT_FAILURE | DT_INVALID_PARAM;
#endif

  return DT_SUCCESS;
}

dtStatus dtNavMesh::init(uint8_t *data, const int dataSize, const int flags) {
  // Make sure the data is in right format.
  const dtMeshHeader *header = reinterpret_cast<dtMeshHeader *>(data);
  if (header->magic != DT_NAVMESH_MAGIC)
    return DT_FAILURE | DT_WRONG_MAGIC;
  if (header->version != DT_NAVMESH_VERSION)
    return DT_FAILURE | DT_WRONG_VERSION;

  dtNavMeshParams params;
  dtVcopy(params.orig, header->bmin);
  params.tileWidth = header->bmax[0] - header->bmin[0];
  params.tileHeight = header->bmax[2] - header->bmin[2];
  params.maxTiles = 1;
  params.maxPolys = header->polyCount;

  const dtStatus status = init(&params);
  if (dtStatusFailed(status))
    return status;

  return addTile(data, dataSize, flags, 0, nullptr);
}

/// @par
///
/// @note The parameters are created automatically when the single tile
/// initialization is performed.
const dtNavMeshParams *dtNavMesh::getParams() const {
  return &m_params;
}

//////////////////////////////////////////////////////////////////////////////////////////
int dtNavMesh::findConnectingPolys(const float *va, const float *vb,
                                   const dtMeshTile *tile, const int side,
                                   dtPolyRef *con, float *conarea, const int maxcon) const {
  if (!tile)
    return 0;

  float amin[2];
  float amax[2];
  calcSlabEndPoints(va, vb, amin, amax, side);
  const float apos = getSlabCoord(va, side);

  // Remove links pointing to 'side' and compact the links array.
  const uint16_t m = DT_EXT_LINK | static_cast<uint16_t>(side);
  int n = 0;

  const dtPolyRef base = getPolyRefBase(tile);

  for (int i = 0; i < tile->header->polyCount; ++i) {
    const dtPoly *poly = &tile->polys[i];
    const int nv = poly->vertCount;
    for (int j = 0; j < nv; ++j) {
      float bmax[2];
      float bmin[2];
      // Skip edges which do not point to the right side.
      if (poly->neis[j] != m)
        continue;

      const float *vc = &tile->verts[poly->verts[j] * 3];
      const float *vd = &tile->verts[poly->verts[(j + 1) % nv] * 3];

      // Segments are not close enough.
      if (dtAbs(apos - getSlabCoord(vc, side)) > 0.01f)
        continue;

      // Check if the segments touch.
      calcSlabEndPoints(vc, vd, bmin, bmax, side);

      if (!overlapSlabs(amin, amax, bmin, bmax, 0.01f, tile->header->walkableClimb))
        continue;

      // Add return value.
      if (n < maxcon) {
        conarea[n * 2 + 0] = dtMax(amin[0], bmin[0]);
        conarea[n * 2 + 1] = dtMin(amax[0], bmax[0]);
        con[n] = base | static_cast<dtPolyRef>(i);
        n++;
      }
      break;
    }
  }
  return n;
}

void dtNavMesh::unconnectLinks(dtMeshTile *tile, const dtMeshTile *target) const {
  if (!tile || !target)
    return;

  const uint32_t targetNum = decodePolyIdTile(getTileRef(target));

  for (int i = 0; i < tile->header->polyCount; ++i) {
    dtPoly *poly = &tile->polys[i];
    uint32_t j = poly->firstLink;
    uint32_t pj = DT_NULL_LINK;
    while (j != DT_NULL_LINK) {
      if (decodePolyIdTile(tile->links[j].ref) == targetNum) {
        // Remove link.
        const uint32_t nj = tile->links[j].next;
        if (pj == DT_NULL_LINK)
          poly->firstLink = nj;
        else
          tile->links[pj].next = nj;
        freeLink(tile, j);
        j = nj;
      } else {
        // Advance
        pj = j;
        j = tile->links[j].next;
      }
    }
  }
}

void dtNavMesh::connectExtLinks(dtMeshTile *tile, const dtMeshTile *target, const int side) const {
  if (!tile)
    return;

  // Connect border links.
  for (int i = 0; i < tile->header->polyCount; ++i) {
    dtPoly *poly = &tile->polys[i];

    // Create new links.
    //		uint16_t m = DT_EXT_LINK | (uint16_t)side;

    const int nv = poly->vertCount;
    for (int j = 0; j < nv; ++j) {
      // Skip non-portal edges.
      if ((poly->neis[j] & DT_EXT_LINK) == 0)
        continue;

      const int dir = poly->neis[j] & 0xff;
      if (side != -1 && dir != side)
        continue;

      // Create new links
      const float *va = &tile->verts[poly->verts[j] * 3];
      const float *vb = &tile->verts[poly->verts[(j + 1) % nv] * 3];
      dtPolyRef nei[4];
      float neia[4 * 2];
      const int nnei = findConnectingPolys(va, vb, target, dtOppositeTile(dir), nei, neia, 4);
      for (int k = 0; k < nnei; ++k) {
        const uint32_t idx = allocLink(tile);
        if (idx != DT_NULL_LINK) {
          dtLink *link = &tile->links[idx];
          link->ref = nei[k];
          link->edge = static_cast<uint8_t>(j);
          link->side = static_cast<uint8_t>(dir);

          link->next = poly->firstLink;
          poly->firstLink = idx;

          // Compress portal limits to a byte value.
          if (dir == 0 || dir == 4) {
            float tmin = (neia[k * 2 + 0] - va[2]) / (vb[2] - va[2]);
            float tmax = (neia[k * 2 + 1] - va[2]) / (vb[2] - va[2]);
            if (tmin > tmax)
              dtSwap(tmin, tmax);
            link->bmin = static_cast<uint8_t>(roundf(dtClamp(tmin, 0.0f, 1.0f) * 255.0f));
            link->bmax = static_cast<uint8_t>(roundf(dtClamp(tmax, 0.0f, 1.0f) * 255.0f));
          } else if (dir == 2 || dir == 6) {
            float tmin = (neia[k * 2 + 0] - va[0]) / (vb[0] - va[0]);
            float tmax = (neia[k * 2 + 1] - va[0]) / (vb[0] - va[0]);
            if (tmin > tmax)
              dtSwap(tmin, tmax);
            link->bmin = static_cast<uint8_t>(roundf(dtClamp(tmin, 0.0f, 1.0f) * 255.0f));
            link->bmax = static_cast<uint8_t>(roundf(dtClamp(tmax, 0.0f, 1.0f) * 255.0f));
          }
        }
      }
    }
  }
}

void dtNavMesh::connectExtOffMeshLinks(dtMeshTile *tile, dtMeshTile *target, const int side) const {
  if (!tile)
    return;

  // Connect off-mesh links.
  // We are interested on links which land from target tile to this tile.
  const uint8_t oppositeSide = side == -1 ? 0xff : static_cast<uint8_t>(dtOppositeTile(side));

  for (int i = 0; i < target->header->offMeshConCount; ++i) {
    const dtOffMeshConnection *targetCon = &target->offMeshCons[i];
    if (targetCon->side != oppositeSide)
      continue;

    dtPoly *targetPoly = &target->polys[targetCon->poly];
    // Skip off-mesh connections which start location could not be connected at all.
    if (targetPoly->firstLink == DT_NULL_LINK)
      continue;

    const float halfExtents[3] = {targetCon->rad, target->header->walkableClimb, targetCon->rad};

    // Find polygon to connect to.
    const float *p = &targetCon->pos[3];
    float nearestPt[3];
    const dtPolyRef ref = findNearestPolyInTile(tile, p, halfExtents, nearestPt);
    if (!ref)
      continue;
    // findNearestPoly may return too optimistic results, further check to make sure.
    if (dtSqr(nearestPt[0] - p[0]) + dtSqr(nearestPt[2] - p[2]) > dtSqr(targetCon->rad))
      continue;
    // Make sure the location is on current mesh.
    float *v = &target->verts[targetPoly->verts[1] * 3];
    dtVcopy(v, nearestPt);

    // Link off-mesh connection to target poly.
    const uint32_t idx = allocLink(target);
    if (idx != DT_NULL_LINK) {
      dtLink *link = &target->links[idx];
      link->ref = ref;
      link->edge = static_cast<uint8_t>(1);
      link->side = oppositeSide;
      link->bmin = link->bmax = 0;
      // Add to linked list.
      link->next = targetPoly->firstLink;
      targetPoly->firstLink = idx;
    }

    // Link target poly to off-mesh connection.
    if (targetCon->flags & DT_OFFMESH_CON_BIDIR) {
      const uint32_t tidx = allocLink(tile);
      if (tidx != DT_NULL_LINK) {
        const uint16_t landPolyIdx = static_cast<uint16_t>(decodePolyIdPoly(ref));
        dtPoly *landPoly = &tile->polys[landPolyIdx];
        dtLink *link = &tile->links[tidx];
        link->ref = getPolyRefBase(target) | static_cast<dtPolyRef>(targetCon->poly);
        link->edge = 0xff;
        link->side = static_cast<uint8_t>(side == -1 ? 0xff : side);
        link->bmin = link->bmax = 0;
        // Add to linked list.
        link->next = landPoly->firstLink;
        landPoly->firstLink = tidx;
      }
    }
  }

}

void dtNavMesh::connectIntLinks(dtMeshTile *tile) const {
  if (!tile)
    return;

  const dtPolyRef base = getPolyRefBase(tile);

  for (int i = 0; i < tile->header->polyCount; ++i) {
    dtPoly *poly = &tile->polys[i];
    poly->firstLink = DT_NULL_LINK;

    if (poly->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)
      continue;

    // Build edge links backwards so that the links will be
    // in the linked list from lowest index to highest.
    for (int j = poly->vertCount - 1; j >= 0; --j) {
      // Skip hard and non-internal edges.
      if (poly->neis[j] == 0 || poly->neis[j] & DT_EXT_LINK)
        continue;

      const uint32_t idx = allocLink(tile);
      if (idx != DT_NULL_LINK) {
        dtLink *link = &tile->links[idx];
        link->ref = base | static_cast<dtPolyRef>(poly->neis[j] - 1);
        link->edge = static_cast<uint8_t>(j);
        link->side = 0xff;
        link->bmin = link->bmax = 0;
        // Add to linked list.
        link->next = poly->firstLink;
        poly->firstLink = idx;
      }
    }
  }
}

void dtNavMesh::baseOffMeshLinks(dtMeshTile *tile) const {
  if (!tile)
    return;

  const dtPolyRef base = getPolyRefBase(tile);

  // Base off-mesh connection start points.
  for (int i = 0; i < tile->header->offMeshConCount; ++i) {
    const dtOffMeshConnection *con = &tile->offMeshCons[i];
    dtPoly *poly = &tile->polys[con->poly];

    const float halfExtents[3] = {con->rad, tile->header->walkableClimb, con->rad};

    // Find polygon to connect to.
    const float *p = &con->pos[0]; // First vertex
    float nearestPt[3];
    const dtPolyRef ref = findNearestPolyInTile(tile, p, halfExtents, nearestPt);
    if (!ref)
      continue;
    // findNearestPoly may return too optimistic results, further check to make sure.
    if (dtSqr(nearestPt[0] - p[0]) + dtSqr(nearestPt[2] - p[2]) > dtSqr(con->rad))
      continue;
    // Make sure the location is on current mesh.
    float *v = &tile->verts[poly->verts[0] * 3];
    dtVcopy(v, nearestPt);

    // Link off-mesh connection to target poly.
    const uint32_t idx = allocLink(tile);
    if (idx != DT_NULL_LINK) {
      dtLink *link = &tile->links[idx];
      link->ref = ref;
      link->edge = static_cast<uint8_t>(0);
      link->side = 0xff;
      link->bmin = link->bmax = 0;
      // Add to linked list.
      link->next = poly->firstLink;
      poly->firstLink = idx;
    }

    // Start end-point is always connect back to off-mesh connection.
    const uint32_t tidx = allocLink(tile);
    if (tidx != DT_NULL_LINK) {
      const uint16_t landPolyIdx = static_cast<uint16_t>(decodePolyIdPoly(ref));
      dtPoly *landPoly = &tile->polys[landPolyIdx];
      dtLink *link = &tile->links[tidx];
      link->ref = base | static_cast<dtPolyRef>(con->poly);
      link->edge = 0xff;
      link->side = 0xff;
      link->bmin = link->bmax = 0;
      // Add to linked list.
      link->next = landPoly->firstLink;
      landPoly->firstLink = tidx;
    }
  }
}

namespace {
template <bool onlyBoundary>
void closestPointOnDetailEdges(const dtMeshTile *tile, const dtPoly *poly, const float *pos, float *closest) {
  const uint32_t ip = static_cast<uint32_t>(poly - tile->polys);
  const dtPolyDetail *pd = &tile->detailMeshes[ip];

  float dmin = FLT_MAX;
  float tmin = 0;
  const float *pmin = nullptr;
  const float *pmax = nullptr;

  for (int i = 0; i < pd->triCount; i++) {
    const uint8_t *tris = &tile->detailTris[(pd->triBase + i) * 4];
    constexpr int ANY_BOUNDARY_EDGE =
        DT_DETAIL_EDGE_BOUNDARY << 0 |
        DT_DETAIL_EDGE_BOUNDARY << 2 |
        DT_DETAIL_EDGE_BOUNDARY << 4;
    if (onlyBoundary && (tris[3] & ANY_BOUNDARY_EDGE) == 0)
      continue;

    const float *v[3];
    for (int j = 0; j < 3; ++j) {
      if (tris[j] < poly->vertCount)
        v[j] = &tile->verts[poly->verts[tris[j]] * 3];
      else
        v[j] = &tile->detailVerts[(pd->vertBase + (tris[j] - poly->vertCount)) * 3];
    }

    for (int k = 0, j = 2; k < 3; j = k++) {
      if ((dtGetDetailTriEdgeFlags(tris[3], j) & DT_DETAIL_EDGE_BOUNDARY) == 0 &&
          (onlyBoundary || tris[j] < tris[k])) {
        // Only looking at boundary edges and this is internal, or
        // this is an inner edge that we will see again or have already seen.
        continue;
      }

      float t;
      const float d = dtDistancePtSegSqr2D(pos, v[j], v[k], t);
      if (d < dmin) {
        dmin = d;
        tmin = t;
        pmin = v[j];
        pmax = v[k];
      }
    }
  }

  dtVlerp(closest, pmin, pmax, tmin);
}
} // namespace

bool dtNavMesh::getPolyHeight(const dtMeshTile *tile, const dtPoly *poly, const float *pos, float *height) {
  // Off-mesh connections do not have detail polys and getting height
  // over them does not make sense.
  if (poly->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)
    return false;

  const uint32_t ip = static_cast<uint32_t>(poly - tile->polys);
  const dtPolyDetail *pd = &tile->detailMeshes[ip];

  float verts[DT_VERTS_PER_POLYGON * 3];
  const int nv = poly->vertCount;
  for (int i = 0; i < nv; ++i)
    dtVcopy(&verts[i * 3], &tile->verts[poly->verts[i] * 3]);

  if (!dtPointInPolygon(pos, verts, nv))
    return false;

  if (!height)
    return true;

  // Find height at the location.
  for (int j = 0; j < pd->triCount; ++j) {
    const uint8_t *t = &tile->detailTris[(pd->triBase + j) * 4];
    const float *v[3];
    for (int k = 0; k < 3; ++k) {
      if (t[k] < poly->vertCount)
        v[k] = &tile->verts[poly->verts[t[k]] * 3];
      else
        v[k] = &tile->detailVerts[(pd->vertBase + (t[k] - poly->vertCount)) * 3];
    }
    float h;
    if (dtClosestHeightPointTriangle(pos, v[0], v[1], v[2], h)) {
      *height = h;
      return true;
    }
  }

  // If all triangle checks failed above (can happen with degenerate triangles
  // or larger floating point values) the point is on an edge, so just select
  // closest. This should almost never happen so the extra iteration here is
  // ok.
  float closest[3];
  closestPointOnDetailEdges<false>(tile, poly, pos, closest);
  *height = closest[1];
  return true;
}

void dtNavMesh::closestPointOnPoly(const dtPolyRef ref, const float *pos, float *closest, bool *posOverPoly) const {
  const dtMeshTile *tile = nullptr;
  const dtPoly *poly = nullptr;
  getTileAndPolyByRefUnsafe(ref, &tile, &poly);

  dtVcopy(closest, pos);
  if (getPolyHeight(tile, poly, pos, &closest[1])) {
    if (posOverPoly)
      *posOverPoly = true;
    return;
  }

  if (posOverPoly)
    *posOverPoly = false;

  // Off-mesh connections don't have detail polygons.
  if (poly->getType() == DT_POLYTYPE_OFFMESH_CONNECTION) {
    const float *v0 = &tile->verts[poly->verts[0] * 3];
    const float *v1 = &tile->verts[poly->verts[1] * 3];
    float t;
    dtDistancePtSegSqr2D(pos, v0, v1, t);
    dtVlerp(closest, v0, v1, t);
    return;
  }

  // Outside poly that is not an offmesh connection.
  closestPointOnDetailEdges<true>(tile, poly, pos, closest);
}

dtPolyRef dtNavMesh::findNearestPolyInTile(const dtMeshTile *tile,
                                           const float *center, const float *halfExtents,
                                           float *nearestPt) const {
  float bmin[3], bmax[3];
  dtVsub(bmin, center, halfExtents);
  dtVadd(bmax, center, halfExtents);

  // Get nearby polygons from proximity grid.
  dtPolyRef polys[128];
  const int polyCount = queryPolygonsInTile(tile, bmin, bmax, polys, 128);

  // Find nearest polygon amongst the nearby polygons.
  dtPolyRef nearest = 0;
  float nearestDistanceSqr = FLT_MAX;
  for (int i = 0; i < polyCount; ++i) {
    const dtPolyRef ref = polys[i];
    float closestPtPoly[3];
    float diff[3];
    bool posOverPoly = false;
    float d;
    closestPointOnPoly(ref, center, closestPtPoly, &posOverPoly);

    // If a point is directly over a polygon and closer than
    // climb height, favor that instead of straight line nearest point.
    dtVsub(diff, center, closestPtPoly);
    if (posOverPoly) {
      d = dtAbs(diff[1]) - tile->header->walkableClimb;
      d = d > 0 ? d * d : 0;
    } else {
      d = dtVlenSqr(diff);
    }

    if (d < nearestDistanceSqr) {
      dtVcopy(nearestPt, closestPtPoly);
      nearestDistanceSqr = d;
      nearest = ref;
    }
  }

  return nearest;
}

int dtNavMesh::queryPolygonsInTile(const dtMeshTile *tile, const float *qmin, const float *qmax,
                                   dtPolyRef *polys, const int maxPolys) const {
  if (tile->bvTree) {
    const dtBVNode *node = &tile->bvTree[0];
    const dtBVNode *end = &tile->bvTree[tile->header->bvNodeCount];
    const float *tbmin = tile->header->bmin;
    const float *tbmax = tile->header->bmax;
    const float qfac = tile->header->bvQuantFactor;

    // Calculate quantized box
    uint16_t bmin[3], bmax[3];
    // dtClamp query box to world box.
    const float minx = dtClamp(qmin[0], tbmin[0], tbmax[0]) - tbmin[0];
    const float miny = dtClamp(qmin[1], tbmin[1], tbmax[1]) - tbmin[1];
    const float minz = dtClamp(qmin[2], tbmin[2], tbmax[2]) - tbmin[2];
    const float maxx = dtClamp(qmax[0], tbmin[0], tbmax[0]) - tbmin[0];
    const float maxy = dtClamp(qmax[1], tbmin[1], tbmax[1]) - tbmin[1];
    const float maxz = dtClamp(qmax[2], tbmin[2], tbmax[2]) - tbmin[2];
    // Quantize
    bmin[0] = static_cast<uint16_t>(qfac * minx) & 0xfffe;
    bmin[1] = static_cast<uint16_t>(qfac * miny) & 0xfffe;
    bmin[2] = static_cast<uint16_t>(qfac * minz) & 0xfffe;
    bmax[0] = static_cast<uint16_t>(qfac * maxx + 1) | 1;
    bmax[1] = static_cast<uint16_t>(qfac * maxy + 1) | 1;
    bmax[2] = static_cast<uint16_t>(qfac * maxz + 1) | 1;

    // Traverse tree
    const dtPolyRef base = getPolyRefBase(tile);
    int n = 0;
    while (node < end) {
      const bool overlap = dtOverlapQuantBounds(bmin, bmax, node->bmin, node->bmax);
      const bool isLeafNode = node->i >= 0;

      if (isLeafNode && overlap) {
        if (n < maxPolys)
          polys[n++] = base | static_cast<dtPolyRef>(node->i);
      }

      if (overlap || isLeafNode)
        node++;
      else {
        const int escapeIndex = -node->i;
        node += escapeIndex;
      }
    }

    return n;
  }
  int n = 0;
  const dtPolyRef base = getPolyRefBase(tile);
  for (int i = 0; i < tile->header->polyCount; ++i) {
    float bmin[3];
    float bmax[3];
    const dtPoly *p = &tile->polys[i];
    // Do not return off-mesh connection polygons.
    if (p->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)
      continue;
    // Calc polygon bounds.
    const float *v = &tile->verts[p->verts[0] * 3];
    dtVcopy(bmin, v);
    dtVcopy(bmax, v);
    for (int j = 1; j < p->vertCount; ++j) {
      v = &tile->verts[p->verts[j] * 3];
      dtVmin(bmin, v);
      dtVmax(bmax, v);
    }
    if (dtOverlapBounds(qmin, qmax, bmin, bmax)) {
      if (n < maxPolys)
        polys[n++] = base | static_cast<dtPolyRef>(i);
    }
  }
  return n;
}

/// @par
///
/// The add operation will fail if the data is in the wrong format, the allocated tile
/// space is full, or there is a tile already at the specified reference.
///
/// The lastRef parameter is used to restore a tile with the same tile
/// reference it had previously used.  In this case the #dtPolyRef's for the
/// tile will be restored to the same values they were before the tile was
/// removed.
///
/// The nav mesh assumes exclusive access to the data passed and will make
/// changes to the dynamic portion of the data. For that reason the data
/// should not be reused in other nav meshes until the tile has been successfully
/// removed from this nav mesh.
///
/// @see dtCreateNavMeshData, #removeTile
dtStatus dtNavMesh::addTile(uint8_t *data, const int dataSize, const int flags,
                            const dtTileRef lastRef, dtTileRef *result) {
  // Make sure the data is in right format.
  auto *const header = reinterpret_cast<dtMeshHeader *>(data);
  if (header->magic != DT_NAVMESH_MAGIC)
    return DT_FAILURE | DT_WRONG_MAGIC;
  if (header->version != DT_NAVMESH_VERSION)
    return DT_FAILURE | DT_WRONG_VERSION;

#ifndef DT_POLYREF64
  // Do not allow adding more polygons than specified in the NavMesh's maxPolys constraint.
  // Otherwise, the poly ID cannot be represented with the given number of bits.
  if (m_polyBits < dtIlog2(dtNextPow2(static_cast<uint32_t>(header->polyCount))))
    return DT_FAILURE | DT_INVALID_PARAM;
#endif

  // Make sure the location is free.
  if (getTileAt(header->x, header->y, header->layer))
    return DT_FAILURE | DT_ALREADY_OCCUPIED;

  // Allocate a tile.
  dtMeshTile *tile = nullptr;
  if (!lastRef) {
    if (m_nextFree) {
      tile = m_nextFree;
      m_nextFree = tile->next;
      tile->next = nullptr;
    }
  } else {
    // Try to relocate the tile to specific index with same salt.
    const int tileIndex = static_cast<int>(decodePolyIdTile(lastRef));
    if (tileIndex >= m_maxTiles)
      return DT_FAILURE | DT_OUT_OF_MEMORY;
    // Try to find the specific tile id from the free list.
    const dtMeshTile *target = &m_tiles[tileIndex];
    dtMeshTile *prev = nullptr;
    tile = m_nextFree;
    while (tile && tile != target) {
      prev = tile;
      tile = tile->next;
    }
    // Could not find the correct location.
    if (!tile || tile != target)
      return DT_FAILURE | DT_OUT_OF_MEMORY;
    // Remove from freelist
    if (!prev)
      m_nextFree = tile->next;
    else
      prev->next = tile->next;

    // Restore salt.
    tile->salt = decodePolyIdSalt(lastRef);
  }

  // Make sure we could allocate a tile.
  if (!tile)
    return DT_FAILURE | DT_OUT_OF_MEMORY;

  // Insert tile into the position lut.
  const int h = computeTileHash(header->x, header->y, m_tileLutMask);
  tile->next = m_posLookup[h];
  m_posLookup[h] = tile;

  // Patch header pointers.
  const int headerSize = dtAlign4(sizeof(dtMeshHeader));
  const int vertsSize = dtAlign4(sizeof(float) * 3 * header->vertCount);
  const int polysSize = dtAlign4(sizeof(dtPoly) * header->polyCount);
  const int linksSize = dtAlign4(sizeof(dtLink) * header->maxLinkCount);
  const int detailMeshesSize = dtAlign4(sizeof(dtPolyDetail) * header->detailMeshCount);
  const int detailVertsSize = dtAlign4(sizeof(float) * 3 * header->detailVertCount);
  const int detailTrisSize = dtAlign4(sizeof(uint8_t) * 4 * header->detailTriCount);
  const int bvtreeSize = dtAlign4(sizeof(dtBVNode) * header->bvNodeCount);
  const int offMeshLinksSize = dtAlign4(sizeof(dtOffMeshConnection) * header->offMeshConCount);

  uint8_t *d = data + headerSize;
  tile->verts = dtGetThenAdvanceBufferPointer<float>(d, vertsSize);
  tile->polys = dtGetThenAdvanceBufferPointer<dtPoly>(d, polysSize);
  tile->links = dtGetThenAdvanceBufferPointer<dtLink>(d, linksSize);
  tile->detailMeshes = dtGetThenAdvanceBufferPointer<dtPolyDetail>(d, detailMeshesSize);
  tile->detailVerts = dtGetThenAdvanceBufferPointer<float>(d, detailVertsSize);
  tile->detailTris = dtGetThenAdvanceBufferPointer<uint8_t>(d, detailTrisSize);
  tile->bvTree = dtGetThenAdvanceBufferPointer<dtBVNode>(d, bvtreeSize);
  tile->offMeshCons = dtGetThenAdvanceBufferPointer<dtOffMeshConnection>(d, offMeshLinksSize);

  // If there are no items in the bvtree, reset the tree pointer.
  if (!bvtreeSize)
    tile->bvTree = nullptr;

  // Build links freelist
  tile->linksFreeList = 0;
  tile->links[header->maxLinkCount - 1].next = DT_NULL_LINK;
  for (int i = 0; i < header->maxLinkCount - 1; ++i)
    tile->links[i].next = i + 1;

  // Init tile.
  tile->header = header;
  tile->data = data;
  tile->dataSize = dataSize;
  tile->flags = flags;

  connectIntLinks(tile);

  // Base off-mesh connections to their starting polygons and connect connections inside the tile.
  baseOffMeshLinks(tile);
  connectExtOffMeshLinks(tile, tile, -1);

  // Create connections with neighbour tiles.
  static constexpr int MAX_NEIS = 32;
  dtMeshTile *neis[MAX_NEIS];

  // Connect with layers in current tile.
  int nneis = getTilesAt(header->x, header->y, neis, MAX_NEIS);
  for (int j = 0; j < nneis; ++j) {
    if (neis[j] == tile)
      continue;

    connectExtLinks(tile, neis[j], -1);
    connectExtLinks(neis[j], tile, -1);
    connectExtOffMeshLinks(tile, neis[j], -1);
    connectExtOffMeshLinks(neis[j], tile, -1);
  }

  // Connect with neighbour tiles.
  for (int i = 0; i < 8; ++i) {
    nneis = getNeighbourTilesAt(header->x, header->y, i, neis, MAX_NEIS);
    for (int j = 0; j < nneis; ++j) {
      connectExtLinks(tile, neis[j], i);
      connectExtLinks(neis[j], tile, dtOppositeTile(i));
      connectExtOffMeshLinks(tile, neis[j], i);
      connectExtOffMeshLinks(neis[j], tile, dtOppositeTile(i));
    }
  }

  if (result)
    *result = getTileRef(tile);

  return DT_SUCCESS;
}

const dtMeshTile *dtNavMesh::getTileAt(const int x, const int y, const int layer) const {
  // Find tile based on hash.
  const int h = computeTileHash(x, y, m_tileLutMask);
  const dtMeshTile *tile = m_posLookup[h];
  while (tile) {
    if (tile->header &&
        tile->header->x == x &&
        tile->header->y == y &&
        tile->header->layer == layer) {
      return tile;
    }
    tile = tile->next;
  }
  return nullptr;
}

int dtNavMesh::getNeighbourTilesAt(const int x, const int y, const int side, dtMeshTile **tiles, const int maxTiles) const {
  int nx = x, ny = y;
  switch (side) {
  case 0:
    nx++;
    break;
  case 1:
    nx++;
    ny++;
    break;
  case 2:
    ny++;
    break;
  case 3:
    nx--;
    ny++;
    break;
  case 4:
    nx--;
    break;
  case 5:
    nx--;
    ny--;
    break;
  case 6:
    ny--;
    break;
  case 7:
    nx++;
    ny--;
    break;
  default:
    break;
  }

  return getTilesAt(nx, ny, tiles, maxTiles);
}

int dtNavMesh::getTilesAt(const int x, const int y, dtMeshTile **tiles, const int maxTiles) const {
  int n = 0;

  // Find tile based on hash.
  const int h = computeTileHash(x, y, m_tileLutMask);
  dtMeshTile *tile = m_posLookup[h];
  while (tile) {
    if (tile->header &&
        tile->header->x == x &&
        tile->header->y == y) {
      if (n < maxTiles)
        tiles[n++] = tile;
    }
    tile = tile->next;
  }

  return n;
}

/// @par
///
/// This function will not fail if the tiles array is too small to hold the
/// entire result set.  It will simply fill the array to capacity.
int dtNavMesh::getTilesAt(const int x, const int y, dtMeshTile const **tiles, const int maxTiles) const {
  int n = 0;

  // Find tile based on hash.
  const int h = computeTileHash(x, y, m_tileLutMask);
  const dtMeshTile *tile = m_posLookup[h];
  while (tile) {
    if (tile->header &&
        tile->header->x == x &&
        tile->header->y == y) {
      if (n < maxTiles)
        tiles[n++] = tile;
    }
    tile = tile->next;
  }

  return n;
}


dtTileRef dtNavMesh::getTileRefAt(const int x, const int y, const int layer) const {
  // Find tile based on hash.
  const int h = computeTileHash(x, y, m_tileLutMask);
  const dtMeshTile *tile = m_posLookup[h];
  while (tile) {
    if (tile->header &&
        tile->header->x == x &&
        tile->header->y == y &&
        tile->header->layer == layer) {
      return getTileRef(tile);
    }
    tile = tile->next;
  }
  return 0;
}

const dtMeshTile *dtNavMesh::getTileByRef(const dtTileRef ref) const {
  if (!ref)
    return nullptr;
  const uint32_t tileIndex = decodePolyIdTile(ref);
  const uint32_t tileSalt = decodePolyIdSalt(ref);
  if (static_cast<int>(tileIndex) >= m_maxTiles)
    return nullptr;
  const dtMeshTile *tile = &m_tiles[tileIndex];
  if (tile->salt != tileSalt)
    return nullptr;
  return tile;
}

int dtNavMesh::getMaxTiles() const {
  return m_maxTiles;
}

dtMeshTile *dtNavMesh::getTile(const int i) {
  return &m_tiles[i];
}

const dtMeshTile *dtNavMesh::getTile(const int i) const {
  return &m_tiles[i];
}

void dtNavMesh::calcTileLoc(const float *pos, int *tx, int *ty) const {
  *tx = static_cast<int>(floorf((pos[0] - m_orig[0]) / m_tileWidth));
  *ty = static_cast<int>(floorf((pos[2] - m_orig[2]) / m_tileHeight));
}

dtStatus dtNavMesh::getTileAndPolyByRef(const dtPolyRef ref, const dtMeshTile **tile, const dtPoly **poly) const {
  if (!ref)
    return DT_FAILURE;
  uint32_t salt, it, ip;
  decodePolyId(ref, salt, it, ip);
  if (it >= static_cast<uint32_t>(m_maxTiles))
    return DT_FAILURE | DT_INVALID_PARAM;
  if (m_tiles[it].salt != salt || m_tiles[it].header == nullptr)
    return DT_FAILURE | DT_INVALID_PARAM;
  if (ip >= static_cast<uint32_t>(m_tiles[it].header->polyCount))
    return DT_FAILURE | DT_INVALID_PARAM;
  *tile = &m_tiles[it];
  *poly = &m_tiles[it].polys[ip];
  return DT_SUCCESS;
}

/// @par
///
/// @warning Only use this function if it is known that the provided polygon
/// reference is valid. This function is faster than #getTileAndPolyByRef, but
/// it does not validate the reference.
void dtNavMesh::getTileAndPolyByRefUnsafe(const dtPolyRef ref, const dtMeshTile **tile, const dtPoly **poly) const {
  uint32_t salt, it, ip;
  decodePolyId(ref, salt, it, ip);
  *tile = &m_tiles[it];
  *poly = &m_tiles[it].polys[ip];
}

bool dtNavMesh::isValidPolyRef(const dtPolyRef ref) const {
  if (!ref)
    return false;
  uint32_t salt, it, ip;
  decodePolyId(ref, salt, it, ip);
  if (it >= static_cast<uint32_t>(m_maxTiles))
    return false;
  if (m_tiles[it].salt != salt || m_tiles[it].header == nullptr)
    return false;
  if (ip >= static_cast<uint32_t>(m_tiles[it].header->polyCount))
    return false;
  return true;
}

/// @par
///
/// This function returns the data for the tile so that, if desired,
/// it can be added back to the navigation mesh at a later point.
///
/// @see #addTile
dtStatus dtNavMesh::removeTile(const dtTileRef ref, uint8_t **data, int *dataSize) {
  if (!ref)
    return DT_FAILURE | DT_INVALID_PARAM;
  const uint32_t tileIndex = decodePolyIdTile(ref);
  const uint32_t tileSalt = decodePolyIdSalt(ref);
  if (static_cast<int>(tileIndex) >= m_maxTiles)
    return DT_FAILURE | DT_INVALID_PARAM;
  dtMeshTile *tile = &m_tiles[tileIndex];
  if (tile->salt != tileSalt)
    return DT_FAILURE | DT_INVALID_PARAM;

  // Remove tile from hash lookup.
  const int h = computeTileHash(tile->header->x, tile->header->y, m_tileLutMask);
  dtMeshTile *prev = nullptr;
  dtMeshTile *cur = m_posLookup[h];
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

  // Remove connections to neighbour tiles.
  static constexpr int MAX_NEIS = 32;
  dtMeshTile *neis[MAX_NEIS];

  // Disconnect from other layers in current tile.
  int nneis = getTilesAt(tile->header->x, tile->header->y, neis, MAX_NEIS);
  for (int j = 0; j < nneis; ++j) {
    if (neis[j] == tile)
      continue;
    unconnectLinks(neis[j], tile);
  }

  // Disconnect from neighbour tiles.
  for (int i = 0; i < 8; ++i) {
    nneis = getNeighbourTilesAt(tile->header->x, tile->header->y, i, neis, MAX_NEIS);
    for (int j = 0; j < nneis; ++j)
      unconnectLinks(neis[j], tile);
  }

  // Reset tile.
  if (tile->flags & DT_TILE_FREE_DATA) {
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
  tile->flags = 0;
  tile->linksFreeList = 0;
  tile->polys = nullptr;
  tile->verts = nullptr;
  tile->links = nullptr;
  tile->detailMeshes = nullptr;
  tile->detailVerts = nullptr;
  tile->detailTris = nullptr;
  tile->bvTree = nullptr;
  tile->offMeshCons = nullptr;

  // Update salt, salt should never be zero.
#ifdef DT_POLYREF64
	tile->salt = (tile->salt+1) & ((1<<DT_SALT_BITS)-1);
#else
  tile->salt = tile->salt + 1 & (1 << m_saltBits) - 1;
#endif
  if (tile->salt == 0)
    tile->salt++;

  // Add to free list.
  tile->next = m_nextFree;
  m_nextFree = tile;

  return DT_SUCCESS;
}

dtTileRef dtNavMesh::getTileRef(const dtMeshTile *tile) const {
  if (!tile)
    return 0;
  const uint32_t it = static_cast<uint32_t>(tile - m_tiles);
  return encodePolyId(tile->salt, it, 0);
}

/// @par
///
/// Example use case:
/// @code
///
/// const dtPolyRef base = navmesh->getPolyRefBase(tile);
/// for (int i = 0; i < tile->header->polyCount; ++i)
/// {
///     const dtPoly* p = &tile->polys[i];
///     const dtPolyRef ref = base | (dtPolyRef)i;
///
///     // Use the reference to access the polygon data.
/// }
/// @endcode
dtPolyRef dtNavMesh::getPolyRefBase(const dtMeshTile *tile) const {
  if (!tile)
    return 0;
  const uint32_t it = static_cast<uint32_t>(tile - m_tiles);
  return encodePolyId(tile->salt, it, 0);
}

struct dtTileState {
  int magic; // Magic number, used to identify the data.
  int version; // Data version number.
  dtTileRef ref; // Tile ref at the time of storing the data.
};

struct dtPolyState {
  uint16_t flags; // Flags (see dtPolyFlags).
  uint8_t area; // Area ID of the polygon.
};

///  @see #storeTileState
int dtNavMesh::getTileStateSize(const dtMeshTile *tile) {
  if (!tile)
    return 0;
  const int headerSize = dtAlign4(sizeof(dtTileState));
  const int polyStateSize = dtAlign4(sizeof(dtPolyState) * tile->header->polyCount);
  return headerSize + polyStateSize;
}

/// @par
///
/// Tile state includes non-structural data such as polygon flags, area ids, etc.
/// @note The state data is only valid until the tile reference changes.
/// @see #getTileStateSize, #restoreTileState
dtStatus dtNavMesh::storeTileState(const dtMeshTile *tile, uint8_t *data, const int maxDataSize) const {
  // Make sure there is enough space to store the state.
  if (maxDataSize < getTileStateSize(tile))
    return DT_FAILURE | DT_BUFFER_TOO_SMALL;

  dtTileState *tileState = dtGetThenAdvanceBufferPointer<dtTileState>(data, dtAlign4(sizeof(dtTileState)));
  dtPolyState *polyStates = dtGetThenAdvanceBufferPointer<dtPolyState>(data, dtAlign4(sizeof(dtPolyState) * tile->header->polyCount));

  // Store tile state.
  tileState->magic = DT_NAVMESH_STATE_MAGIC;
  tileState->version = DT_NAVMESH_STATE_VERSION;
  tileState->ref = getTileRef(tile);

  // Store per poly state.
  for (int i = 0; i < tile->header->polyCount; ++i) {
    const dtPoly *p = &tile->polys[i];
    dtPolyState *s = &polyStates[i];
    s->flags = p->flags;
    s->area = p->getArea();
  }

  return DT_SUCCESS;
}

/// @par
///
/// Tile state includes non-structural data such as polygon flags, area ids, etc.
/// @note This function does not impact the tile's #dtTileRef and #dtPolyRef's.
/// @see #storeTileState
dtStatus dtNavMesh::restoreTileState(const dtMeshTile *tile, const uint8_t *data, const int maxDataSize) const {
  // Make sure there is enough space to store the state.
  if (maxDataSize < getTileStateSize(tile))
    return DT_FAILURE | DT_INVALID_PARAM;

  const dtTileState *tileState = dtGetThenAdvanceBufferPointer<const dtTileState>(data, dtAlign4(sizeof(dtTileState)));
  const dtPolyState *polyStates = dtGetThenAdvanceBufferPointer<const dtPolyState>(data, dtAlign4(sizeof(dtPolyState) * tile->header->polyCount));

  // Check that the restore is possible.
  if (tileState->magic != DT_NAVMESH_STATE_MAGIC)
    return DT_FAILURE | DT_WRONG_MAGIC;
  if (tileState->version != DT_NAVMESH_STATE_VERSION)
    return DT_FAILURE | DT_WRONG_VERSION;
  if (tileState->ref != getTileRef(tile))
    return DT_FAILURE | DT_INVALID_PARAM;

  // Restore per poly state.
  for (int i = 0; i < tile->header->polyCount; ++i) {
    dtPoly *p = &tile->polys[i];
    const dtPolyState *s = &polyStates[i];
    p->flags = s->flags;
    p->setArea(s->area);
  }

  return DT_SUCCESS;
}

/// @par
///
/// Off-mesh connections are stored in the navigation mesh as special 2-vertex
/// polygons with a single edge. At least one of the vertices is expected to be
/// inside a normal polygon. So an off-mesh connection is "entered" from a
/// normal polygon at one of its endpoints. This is the polygon identified by
/// the prevRef parameter.
dtStatus dtNavMesh::getOffMeshConnectionPolyEndPoints(const dtPolyRef prevRef, const dtPolyRef polyRef, float *startPos, float *endPos) const {
  uint32_t salt, it, ip;

  if (!polyRef)
    return DT_FAILURE;

  // Get current polygon
  decodePolyId(polyRef, salt, it, ip);
  if (it >= static_cast<uint32_t>(m_maxTiles))
    return DT_FAILURE | DT_INVALID_PARAM;
  if (m_tiles[it].salt != salt || m_tiles[it].header == nullptr)
    return DT_FAILURE | DT_INVALID_PARAM;
  const dtMeshTile *tile = &m_tiles[it];
  if (ip >= static_cast<uint32_t>(tile->header->polyCount))
    return DT_FAILURE | DT_INVALID_PARAM;
  const dtPoly *poly = &tile->polys[ip];

  // Make sure that the current poly is indeed off-mesh link.
  if (poly->getType() != DT_POLYTYPE_OFFMESH_CONNECTION)
    return DT_FAILURE;

  // Figure out which way to hand out the vertices.
  int idx0 = 0, idx1 = 1;

  // Find link that points to first vertex.
  for (uint32_t i = poly->firstLink; i != DT_NULL_LINK; i = tile->links[i].next) {
    if (tile->links[i].edge == 0) {
      if (tile->links[i].ref != prevRef) {
        idx0 = 1;
        idx1 = 0;
      }
      break;
    }
  }

  dtVcopy(startPos, &tile->verts[poly->verts[idx0] * 3]);
  dtVcopy(endPos, &tile->verts[poly->verts[idx1] * 3]);

  return DT_SUCCESS;
}


const dtOffMeshConnection *dtNavMesh::getOffMeshConnectionByRef(const dtPolyRef ref) const {
  uint32_t salt, it, ip;

  if (!ref)
    return nullptr;

  // Get current polygon
  decodePolyId(ref, salt, it, ip);
  if (it >= static_cast<uint32_t>(m_maxTiles))
    return nullptr;
  if (m_tiles[it].salt != salt || m_tiles[it].header == nullptr)
    return nullptr;
  const dtMeshTile *tile = &m_tiles[it];
  if (ip >= static_cast<uint32_t>(tile->header->polyCount))
    return nullptr;

  // Make sure that the current poly is indeed off-mesh link.
  if (tile->polys[ip].getType() != DT_POLYTYPE_OFFMESH_CONNECTION)
    return nullptr;

  const uint32_t idx = ip - tile->header->offMeshBase;
  dtAssert(idx < static_cast<uint32_t>(tile->header->offMeshConCount));
  return &tile->offMeshCons[idx];
}


dtStatus dtNavMesh::setPolyFlags(const dtPolyRef ref, const uint16_t flags) const {
  if (!ref)
    return DT_FAILURE;
  uint32_t salt, it, ip;
  decodePolyId(ref, salt, it, ip);
  if (it >= static_cast<uint32_t>(m_maxTiles))
    return DT_FAILURE | DT_INVALID_PARAM;
  if (m_tiles[it].salt != salt || m_tiles[it].header == nullptr)
    return DT_FAILURE | DT_INVALID_PARAM;
  const dtMeshTile *tile = &m_tiles[it];
  if (ip >= static_cast<uint32_t>(tile->header->polyCount))
    return DT_FAILURE | DT_INVALID_PARAM;
  dtPoly *poly = &tile->polys[ip];

  // Change flags.
  poly->flags = flags;

  return DT_SUCCESS;
}

dtStatus dtNavMesh::getPolyFlags(const dtPolyRef ref, uint16_t *resultFlags) const {
  if (!ref)
    return DT_FAILURE;
  uint32_t salt, it, ip;
  decodePolyId(ref, salt, it, ip);
  if (it >= static_cast<uint32_t>(m_maxTiles))
    return DT_FAILURE | DT_INVALID_PARAM;
  if (m_tiles[it].salt != salt || m_tiles[it].header == nullptr)
    return DT_FAILURE | DT_INVALID_PARAM;
  const dtMeshTile *tile = &m_tiles[it];
  if (ip >= static_cast<uint32_t>(tile->header->polyCount))
    return DT_FAILURE | DT_INVALID_PARAM;
  const dtPoly *poly = &tile->polys[ip];

  *resultFlags = poly->flags;

  return DT_SUCCESS;
}

dtStatus dtNavMesh::setPolyArea(const dtPolyRef ref, const uint8_t area) const {
  if (!ref)
    return DT_FAILURE;
  uint32_t salt, it, ip;
  decodePolyId(ref, salt, it, ip);
  if (it >= static_cast<uint32_t>(m_maxTiles))
    return DT_FAILURE | DT_INVALID_PARAM;
  if (m_tiles[it].salt != salt || m_tiles[it].header == nullptr)
    return DT_FAILURE | DT_INVALID_PARAM;
  const dtMeshTile *tile = &m_tiles[it];
  if (ip >= static_cast<uint32_t>(tile->header->polyCount))
    return DT_FAILURE | DT_INVALID_PARAM;
  dtPoly *poly = &tile->polys[ip];

  poly->setArea(area);

  return DT_SUCCESS;
}

dtStatus dtNavMesh::getPolyArea(const dtPolyRef ref, uint8_t *resultArea) const {
  if (!ref)
    return DT_FAILURE;
  uint32_t salt, it, ip;
  decodePolyId(ref, salt, it, ip);
  if (it >= static_cast<uint32_t>(m_maxTiles))
    return DT_FAILURE | DT_INVALID_PARAM;
  if (m_tiles[it].salt != salt || m_tiles[it].header == nullptr)
    return DT_FAILURE | DT_INVALID_PARAM;
  const dtMeshTile *tile = &m_tiles[it];
  if (ip >= static_cast<uint32_t>(tile->header->polyCount))
    return DT_FAILURE | DT_INVALID_PARAM;
  const dtPoly *poly = &tile->polys[ip];

  *resultArea = poly->getArea();

  return DT_SUCCESS;
}