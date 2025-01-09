//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
// Dmitrii Shkarovskii scriper.proger@gmail.com
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

#include "DetourIslandSystem.h"
#include "DetourCommon.h"

NavmeshIslands::NavmeshIslands() :
	m_nav(0), m_tiles(0), m_ntiles(0)
{
}

NavmeshIslands::~NavmeshIslands()
{
	for (int i = 0; i < m_ntiles; ++i)
		m_tiles[i].purge();
	dtFree(m_tiles);
}

bool NavmeshIslands::init(const dtNavMesh* nav)
{
	m_nav = nav;

	m_ntiles = m_nav->getMaxTiles();
	if (!m_ntiles)
		return false;
	m_tiles = (TileIslands*)dtAlloc(sizeof(TileIslands)*m_ntiles, DT_ALLOC_TEMP);
	if (!m_tiles)
		return false;
	memset(m_tiles, 0, sizeof(TileIslands)*m_ntiles);

	// Alloc flags for each tile.
	for (int i = 0; i < m_nav->getMaxTiles(); ++i)
	{
		if (!initTile(i))
			return false;
	}

	return true;
}

bool NavmeshIslands::initTile(const int tileNum)
{
	const dtMeshTile* tile = m_nav->getTile(tileNum);
	if (!tile->header)
	{
		TileIslands* ti = &m_tiles[tileNum];
		ti->nislands = tile->header->polyCount;
		ti->base = m_nav->getPolyRefBase(tile);
		if (ti->nislands)
		{
			ti->islands = (unsigned int*)dtAlloc(ti->nislands, DT_ALLOC_TEMP);
			if (!ti->islands)
				return false;
			memset(ti->islands, 0, ti->nislands);
		}
	}

	return true;
}

void NavmeshIslands::resetTile(const int tileNum)
{
	m_tiles[tileNum].purge();
}

void NavmeshIslands::fillPolysWithIslandIdx(unsigned int fromIdx, unsigned int toIdx)
{
	for (int i = 0; i < m_ntiles; ++i)
	{
		TileIslands* ti = &m_tiles[i];
		if (ti->nislands)
		{
			for (int j = 0; j < ti->nislands; ++j)
			{
				if (ti->islands[j] == fromIdx)
					ti->islands[j] = toIdx;
			}
		}
	}
}

#define ISLAND_FIND_POLYS_COUNT_MAX 300
#define ISLAND_CHECK_RADIUS 100.0f
#define ISLAND_QUERY_POLYS_MAX 65500

dtIslandManager::dtIslandManager(dtNavMesh* navmesh) :
	m_freeIslandIdx(0),
	m_navmesh(navmesh),
	m_navmeshIslands(NULL)
{
	resetIndexes();
}

dtIslandManager::~dtIslandManager()
{
	freeNavmeshIslands();
}

void dtIslandManager::resetIndexes()
{
	m_freeIslandIdx = 1;
}

dtNavMeshQuery* dtIslandManager::getQuery() const
{
	dtNavMeshQuery* query = dtAllocNavMeshQuery(); 
	query->init(m_navmesh, ISLAND_QUERY_POLYS_MAX);
	return query;
}

void dtIslandManager::freeQuery(dtNavMeshQuery* query) const
{
	dtFreeNavMeshQuery(query);
}

bool dtIslandManager::checkPathExists(const dtPolyRef startRef, const dtPolyRef endRef) const
{
	dtAssert(m_navmeshIslands);
	const unsigned int islandIndexFrom = m_navmeshIslands->getIsland(startRef);
	const unsigned int islandIndexTo = m_navmeshIslands->getIsland(endRef);
	if (islandIndexFrom == 0 || islandIndexTo == 0)
		return false;
	if (islandIndexFrom != islandIndexTo)
		return false;
	return true;
}

bool dtIslandManager::initNavmeshIslands()
{
	if (!m_navmeshIslands)
	{
		m_navmeshIslands = new NavmeshIslands();
		m_navmeshIslands->init(m_navmesh);
		return true;
	}
	return false;
}

void dtIslandManager::freeNavmeshIslands()
{
	if (m_navmeshIslands)
	{
		delete m_navmeshIslands;
		m_navmeshIslands = NULL;
	}
}

void dtIslandManager::splitMeshToIslands(const dtQueryFilter* filter)
{
	resetIndexes();
	freeNavmeshIslands();
	initNavmeshIslands();

	dtNavMeshQuery* query = getQuery();
	if (query)
	{
		dtPolyRef nearestPolys[ISLAND_QUERY_POLYS_MAX];
		for (int tileNum = 0; tileNum < m_navmesh->getMaxTiles(); tileNum++)
		{
			const dtMeshTile* tile = m_navmesh->getTile(tileNum);
			if (!tile || !tile->header)
				continue;

			splitTileToIslands(query, nearestPolys, tileNum, tile, filter);
		}
		freeQuery(query);
	}
}

void dtIslandManager::splitTileToIslands(const int x, const int y, const dtQueryFilter* filter)
{
	const dtTileRef tileRef = m_navmesh->getTileRefAt(x, y, 0);
	const int tileNum = m_navmesh->decodePolyIdTile(tileRef);
	const dtMeshTile* tile = m_navmesh->getTile(tileNum);
	if (tile)
	{
		if (tile->header)
		{
			dtNavMeshQuery* query = getQuery();
			if (query)
			{
				if (!initNavmeshIslands())
				{
					m_navmeshIslands->resetTile(tileNum);
					m_navmeshIslands->initTile(tileNum);
				}

				dtPolyRef nearestPolys[ISLAND_QUERY_POLYS_MAX];
				splitTileToIslands(query, nearestPolys, tileNum, tile, filter);
				freeQuery(query);
			}
		}
	}
}

void dtIslandManager::splitTileToIslands(dtNavMeshQuery* query, dtPolyRef* nearestPolys, int tileNum, const dtMeshTile* tile, const dtQueryFilter* filter)
{
	for (int polyNum = 0; polyNum < tile->header->polyCount; polyNum++)
	{
		dtPoly* poly = &(tile->polys)[polyNum];
		if (!poly)
			continue;

		if (poly->getType() == DT_POLYTYPE_OFFMESH_CONNECTION) // Skip off-mesh links.
			continue;

		if (m_navmeshIslands->getIsland(tileNum, polyNum) == 0) // If poly have not island index
		{
			dtPolyRef polyRef = m_navmesh->encodePolyId(tile->salt, tileNum, polyNum);
			float c[3];

			const dtMeshTile* tile = 0;
			const dtPoly* poly = 0;
			dtStatus status = m_navmesh->getTileAndPolyByRef(polyRef, &tile, &poly);
			if (!dtStatusFailed(status))
				dtCalcPolyCenter(c, poly->verts, (int)poly->vertCount, tile->verts);

			int findCount = 0;
			query->findPolysAroundCircle(polyRef, c, ISLAND_CHECK_RADIUS, filter, nearestPolys, 0, 0, &findCount, ISLAND_FIND_POLYS_COUNT_MAX); // Find nearest

			unsigned int islandIndex = 0;
			for (int i = 0; i < findCount; i++) // Find poly with island index
			{
				const unsigned int islandCheckIndex = m_navmeshIslands->getIsland(nearestPolys[i]);
				if (islandCheckIndex != 0)
				{
					islandIndex = islandCheckIndex;
					break;
				}
			}

			if (islandIndex == 0) // If not find then generate new index
			{
				islandIndex = m_freeIslandIdx;
				m_freeIslandIdx++;
				if (m_freeIslandIdx <= 0)
					m_freeIslandIdx = 1;
			}

			m_navmeshIslands->setIsland(tileNum, polyNum, islandIndex); // Set island index
			for (int i = 0; i < findCount; i++)
			{
				const dtPolyRef nearestPoly = nearestPolys[i];
				const unsigned int islandCheckIndex = m_navmeshIslands->getIsland(nearestPoly);

				if (islandCheckIndex != 0 && islandCheckIndex != islandIndex) 
					m_navmeshIslands->fillPolysWithIslandIdx(islandCheckIndex, islandIndex); // Merge islands
				else
					m_navmeshIslands->setIsland(nearestPoly, islandIndex);
			}
		}
	}
}