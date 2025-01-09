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

// Use in quick check passability from one polygon to another polygon
// This system works on compares polygon island indices
// To activate this system you need uncomment define NAVMESH_ISLAND_SYSTEM 

#ifndef DETOURISLANDSYSTEM_H
#define DETOURISLANDSYSTEM_H

#include <cstring>
#include "DetourAssert.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"

class NavmeshIslands
{
public:
	NavmeshIslands();
	~NavmeshIslands();

	bool init(const dtNavMesh* nav);
	bool initTile(const int tileNum);
	void resetTile(const int tileNum);

	inline void clearAllIslands()
	{
		for (int i = 0; i < m_ntiles; ++i)
		{
			TileIslands* ti = &m_tiles[i];
			if (ti->nislands)
				memset(ti->islands, 0, ti->nislands);
		}
	}

	inline unsigned int getIsland(const dtPolyRef ref) const
	{
		dtAssert(m_nav);
		dtAssert(m_ntiles);
		// Assume the ref is valid, no bounds checks.
		unsigned int salt, it, ip;
		m_nav->decodePolyId(ref, salt, it, ip);
		return m_tiles[it].islands[ip];
	}

	inline unsigned int getIsland(const int tileNum, const int polyNum) const
	{
		dtAssert(m_nav);
		dtAssert(m_ntiles);
		return m_tiles[tileNum].islands[polyNum];
	}

	inline void setIsland(const dtPolyRef ref, const unsigned int island)
	{
		dtAssert(m_nav);
		dtAssert(m_ntiles);
		// Assume the ref is valid, no bounds checks.
		unsigned int salt, it, ip;
		m_nav->decodePolyId(ref, salt, it, ip);
		m_tiles[it].islands[ip] = island;
	}

	inline void setIsland(const int tileNum, const int polyNum, const unsigned int island)
	{
		dtAssert(m_nav);
		dtAssert(m_ntiles);
		m_tiles[tileNum].islands[polyNum] = island;
	}

	void fillPolysWithIslandIdx(unsigned int fromIdx, unsigned int toIdx);

private:
	struct TileIslands
	{
		inline void purge() { dtFree(islands); }
		unsigned int* islands;
		int nislands;
		dtPolyRef base;
	};

	const dtNavMesh* m_nav;
	TileIslands* m_tiles;
	int m_ntiles;
};

class dtIslandManager
{
public:
	dtIslandManager(dtNavMesh* navmesh);
	~dtIslandManager();

	void splitMeshToIslands(const dtQueryFilter* filter);
	void splitTileToIslands(const int x, const int y, const dtQueryFilter* filter);

	bool checkPathExists(const dtPolyRef startRef, const dtPolyRef endRef) const;

private:
	void splitTileToIslands(dtNavMeshQuery* query, dtPolyRef* nearestPolys, int tileNum, const dtMeshTile* tile, const dtQueryFilter* filter);
	void resetIndexes();

	bool initNavmeshIslands();
	void freeNavmeshIslands();

	dtNavMeshQuery* getQuery() const;
	void freeQuery(dtNavMeshQuery* query) const;

private:
	unsigned int m_freeIslandIdx;
	const dtNavMesh* m_navmesh;
	NavmeshIslands* m_navmeshIslands;
};

#endif // DETOURISLANDSYSTEM_H