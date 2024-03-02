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

#include "NavMeshPruneTool.h"

#include "imgui.h"

#include <DetourAlloc.h>
#include <DetourAssert.h>
#include <DetourCommon.h>
#include <DetourDebugDraw.h>
#include <DetourNavMesh.h>
#include <DetourNavMeshQuery.h>

#include <cstring>
#include <vector>

struct duDebugDraw;
class NavmeshFlags {
  struct TileFlags {
    void purge() const { dtFree(flags); }
    uint8_t *flags{};
    int nflags{};
    dtPolyRef base{};
  };

  const dtNavMesh *m_nav;
  TileFlags *m_tiles;
  int m_ntiles;

public:
  NavmeshFlags() : m_nav(nullptr), m_tiles(nullptr), m_ntiles(0) {
  }

  ~NavmeshFlags() {
    for (int i = 0; i < m_ntiles; ++i)
      m_tiles[i].purge();
    dtFree(m_tiles);
  }

  bool init(const dtNavMesh *nav) {
    m_ntiles = nav->getMaxTiles();
    if (!m_ntiles)
      return true;
    m_tiles = static_cast<TileFlags *>(dtAlloc(sizeof(TileFlags) * m_ntiles, DT_ALLOC_TEMP));
    if (!m_tiles) {
      return false;
    }
    std::memset(m_tiles, 0, sizeof(TileFlags) * m_ntiles);

    // Alloc flags for each tile.
    for (int i = 0; i < nav->getMaxTiles(); ++i) {
      const dtMeshTile *tile = nav->getTile(i);
      if (!tile->header)
        continue;
      TileFlags *tf = &m_tiles[i];
      tf->nflags = tile->header->polyCount;
      tf->base = nav->getPolyRefBase(tile);
      if (tf->nflags) {
        tf->flags = static_cast<uint8_t *>(dtAlloc(tf->nflags, DT_ALLOC_TEMP));
        if (!tf->flags)
          return false;
        std::memset(tf->flags, 0, tf->nflags);
      }
    }

    m_nav = nav;

    return false;
  }

  void clearAllFlags() const {
    for (int i = 0; i < m_ntiles; ++i) {
      const TileFlags *tf = &m_tiles[i];
      if (tf->nflags)
        std::memset(tf->flags, 0, tf->nflags);
    }
  }

  uint8_t getFlags(const dtPolyRef ref) const {
    dtAssert(m_nav);
    dtAssert(m_ntiles);
    if (!m_nav || !m_ntiles)
      return 0;
    // Assume the ref is valid, no bounds checks.
    uint32_t salt, it, ip;
    m_nav->decodePolyId(ref, salt, it, ip);
    return m_tiles[it].flags[ip];
  }

  void setFlags(const dtPolyRef ref, const uint8_t flags) const {
    dtAssert(m_nav);
    dtAssert(m_ntiles);
    if (!m_nav || !m_ntiles)
      return;
    // Assume the ref is valid, no bounds checks.
    uint32_t salt, it, ip;
    m_nav->decodePolyId(ref, salt, it, ip);
    m_tiles[it].flags[ip] = flags;
  }
};

namespace {
void floodNavmesh(const dtNavMesh *nav, const NavmeshFlags *flags, const dtPolyRef start, const uint8_t flag) {
  // If already visited, skip.
  if (flags->getFlags(start))
    return;

  flags->setFlags(start, flag);

  std::vector<dtPolyRef> openList;
  openList.push_back(start);

  while (openList.size()) {
    const dtPolyRef ref = openList.back();
    openList.pop_back();

    // Get current poly and tile.
    // The API input has been checked already, skip checking internal data.
    const dtMeshTile *tile = nullptr;
    const dtPoly *poly = nullptr;
    nav->getTileAndPolyByRefUnsafe(ref, &tile, &poly);

    // Visit linked polygons.
    for (uint32_t i = poly->firstLink; i != DT_NULL_LINK; i = tile->links[i].next) {
      const dtPolyRef neiRef = tile->links[i].ref;
      // Skip invalid and already visited.
      if (!neiRef || flags->getFlags(neiRef))
        continue;
      // Mark as visited
      flags->setFlags(neiRef, flag);
      // Visit neighbours
      openList.push_back(neiRef);
    }
  }
}

void disableUnvisitedPolys(const dtNavMesh *nav, const NavmeshFlags *flags) {
  for (int i = 0; i < nav->getMaxTiles(); ++i) {
    const dtMeshTile *tile = nav->getTile(i);
    if (!tile->header)
      continue;
    const dtPolyRef base = nav->getPolyRefBase(tile);
    for (int j = 0; j < tile->header->polyCount; ++j) {
      const dtPolyRef ref = base | static_cast<uint32_t>(j);
      if (!flags->getFlags(ref)) {
        uint16_t f = 0;
        nav->getPolyFlags(ref, &f);
        nav->setPolyFlags(ref, f | SAMPLE_POLYFLAGS_DISABLED);
      }
    }
  }
}
} // namespace

NavMeshPruneTool::~NavMeshPruneTool() {
  delete m_flags;
}

void NavMeshPruneTool::init(Sample *sample) {
  m_sample = sample;
}

void NavMeshPruneTool::reset() {
  m_hitPosSet = false;
  delete m_flags;
  m_flags = nullptr;
}

void NavMeshPruneTool::handleMenu() {
  dtNavMesh *nav = m_sample->getNavMesh();
  if (!nav)
    return;
  if (!m_flags)
    return;

  if (imguiButton("Clear Selection")) {
    m_flags->clearAllFlags();
  }

  if (imguiButton("Prune Unselected")) {
    disableUnvisitedPolys(nav, m_flags);
    delete m_flags;
    m_flags = nullptr;
  }
}

void NavMeshPruneTool::handleClick(const float *s, const float *p, const bool shift) {
  rcIgnoreUnused(s);
  rcIgnoreUnused(shift);

  if (!m_sample)
    return;
  if (!m_sample->getInputGeom())
    return;
  const dtNavMesh *nav = m_sample->getNavMesh();
  if (!nav)
    return;
  const dtNavMeshQuery *query = m_sample->getNavMeshQuery();
  if (!query)
    return;

  dtVcopy(m_hitPos, p);
  m_hitPosSet = true;

  if (!m_flags) {
    m_flags = new NavmeshFlags;
    m_flags->init(nav);
  }

  constexpr float halfExtents[3] = {2, 4, 2};
  const dtQueryFilter filter;
  dtPolyRef ref = 0;
  query->findNearestPoly(p, halfExtents, &filter, &ref, nullptr);

  floodNavmesh(nav, m_flags, ref, 1);
}

void NavMeshPruneTool::handleToggle() {
}

void NavMeshPruneTool::handleStep() {
}

void NavMeshPruneTool::handleUpdate(const float /*dt*/) {
}

void NavMeshPruneTool::handleRender() {
  duDebugDraw &dd = m_sample->getDebugDraw();

  if (m_hitPosSet) {
    const float s = m_sample->getAgentRadius();
    const uint32_t col = duRGBA(255, 255, 255, 255);
    dd.begin(DU_DRAW_LINES);
    dd.vertex(m_hitPos[0] - s, m_hitPos[1], m_hitPos[2], col);
    dd.vertex(m_hitPos[0] + s, m_hitPos[1], m_hitPos[2], col);
    dd.vertex(m_hitPos[0], m_hitPos[1] - s, m_hitPos[2], col);
    dd.vertex(m_hitPos[0], m_hitPos[1] + s, m_hitPos[2], col);
    dd.vertex(m_hitPos[0], m_hitPos[1], m_hitPos[2] - s, col);
    dd.vertex(m_hitPos[0], m_hitPos[1], m_hitPos[2] + s, col);
    dd.end();
  }

  const dtNavMesh *nav = m_sample->getNavMesh();
  if (m_flags && nav) {
    for (int i = 0; i < nav->getMaxTiles(); ++i) {
      const dtMeshTile *tile = nav->getTile(i);
      if (!tile->header)
        continue;
      const dtPolyRef base = nav->getPolyRefBase(tile);
      for (int j = 0; j < tile->header->polyCount; ++j) {
        const dtPolyRef ref = base | static_cast<uint32_t>(j);
        if (m_flags->getFlags(ref)) {
          duDebugDrawNavMeshPoly(&dd, *nav, ref, duRGBA(255, 255, 255, 128));
        }
      }
    }
  }
}

void NavMeshPruneTool::handleRenderOverlay(double *proj, double *model, int *view) {
  rcIgnoreUnused(model);
  rcIgnoreUnused(proj);

  // Tool help
  const int h = view[3];

  imguiDrawText(280, h - 40, IMGUI_ALIGN_LEFT, "LMB: Click fill area.", imguiRGBA(255, 255, 255, 192));
}
