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

#include <cstdio>
#include "Sample_Debug.h"

#include <format>
#include <iostream>

#include "Recast.h"
#include "DetourNavMesh.h"
#include "RecastDebugDraw.h"
#include "DetourDebugDraw.h"
#include "RecastDump.h"

/*
static int loadBin(const char* path, unsigned char** data)
{
	FILE* fp = fopen(path, "rb");
	if (!fp) return 0;
	fseek(fp, 0, SEEK_END);
	int size = ftell(fp);
	fseek(fp, 0, SEEK_SET);
	*data = new unsigned char[size];
	fread(*data, size, 1, fp);
	fclose(fp);
	return size;
} 
*/

Sample_Debug::Sample_Debug() :
	m_chf(nullptr),
	m_cset(nullptr),
	m_pmesh(nullptr)
{
	resetCommonSettings();

	// Test
	{
		m_cset = rcAllocContourSet();
		if (m_cset)
		{
			FileIO io;
			if (io.openForRead("PathSet_TMP_NA_PathingTestAReg1_1_2_CS.rc"))
			{
				duReadContourSet(*m_cset, &io);
				std::cout << std::format("bmin=({},{},{}) bmax=({},{},{})\n",
					m_cset->bmin[0], m_cset->bmin[1], m_cset->bmin[2],
					   m_cset->bmax[0], m_cset->bmax[1], m_cset->bmax[2])
				<< std::format("cs=%f ch=%f\n", m_cset->cs, m_cset->ch) << std::endl;
			}
			else
			{
				std::cout << "could not open test.cset" << std::endl;
			}
		}
		else
		{
			std::cout << "Could not alloc cset" << std::endl;
		}
	}
}

Sample_Debug::~Sample_Debug()
{
	rcFreeCompactHeightfield(m_chf);
	rcFreeContourSet(m_cset);
	rcFreePolyMesh(m_pmesh);
}

void Sample_Debug::handleSettings()
{
}

void Sample_Debug::handleTools()
{
}

void Sample_Debug::handleDebugMode()
{
}

void Sample_Debug::handleRender()
{
	if (m_chf)
	{
		duDebugDrawCompactHeightfieldRegions(&m_dd, *m_chf);
//		duDebugDrawCompactHeightfieldSolid(&dd, *m_chf);
	}
		
	if (m_navMesh)
		duDebugDrawNavMesh(&m_dd, *m_navMesh, DU_DRAWNAVMESH_OFFMESHCONS);

	if (m_ref && m_navMesh)
		duDebugDrawNavMeshPoly(&m_dd, *m_navMesh, m_ref, duRGBA(255,0,0,128));

/*	float bmin[3], bmax[3];
	rcVsub(bmin, m_center, m_halfExtents);
	rcVadd(bmax, m_center, m_halfExtents);
	duDebugDrawBoxWire(&dd, bmin[0],bmin[1],bmin[2], bmax[0],bmax[1],bmax[2], duRGBA(255,255,255,128), 1.0f);
	duDebugDrawCross(&dd, m_center[0], m_center[1], m_center[2], 1.0f, duRGBA(255,255,255,128), 2.0f);*/

	if (m_cset)
	{
		duDebugDrawRawContours(&m_dd, *m_cset, 0.25f);
		duDebugDrawContours(&m_dd, *m_cset);
	}
	
	if (m_pmesh)
	{
		duDebugDrawPolyMesh(&m_dd, *m_pmesh);
	}
	
	/*
	dd.depthMask(false);
	{
		const float bmin[3] = {-32.000004f,-11.488281f,-115.343544f};
		const float cs = 0.300000f;
		const float ch = 0.200000f;
		const int verts[] = {
			158,46,336,0,
			157,47,331,0,
			161,53,330,0,
			162,52,335,0,
			158,46,336,0,
			154,46,339,5,
			161,46,365,5,
			171,46,385,5,
			174,46,400,5,
			177,46,404,5,
			177,46,410,5,
			183,46,416,5,
			188,49,416,5,
			193,52,411,6,
			194,53,382,6,
			188,52,376,6,
			188,57,363,6,
			174,57,349,6,
			174,60,342,6,
			168,58,336,6,
			167,59,328,6,
			162,55,324,6,
			159,53,324,5,
			152,46,328,5,
			151,46,336,5,
			154,46,339,5,
			158,46,336,0,
			160,46,340,0,
			164,52,339,0,
			168,55,343,0,
			168,50,351,0,
			182,54,364,0,
			182,47,378,0,
			188,50,383,0,
			188,49,409,0,
			183,46,409,0,
			183,46,403,0,
			180,46,399,0,
			177,46,384,0,
			165,46,359,0,
			160,46,340,0,
		};
		const int nverts = sizeof(verts)/(sizeof(int)*4);

		const unsigned int colln = duRGBA(255,255,255,128);
		dd.begin(DU_DRAW_LINES, 1.0f);
		for (int i = 0, j = nverts-1; i < nverts; j=i++)
		{
			const int* va = &verts[j*4];
			const int* vb = &verts[i*4];
			dd.vertex(bmin[0]+va[0]*cs, bmin[1]+va[1]*ch+j*0.01f, bmin[2]+va[2]*cs, colln);
			dd.vertex(bmin[0]+vb[0]*cs, bmin[1]+vb[1]*ch+i*0.01f, bmin[2]+vb[2]*cs, colln);
		}
		dd.end();

		const unsigned int colpt = duRGBA(255,255,255,255);
		dd.begin(DU_DRAW_POINTS, 3.0f);
		for (int i = 0, j = nverts-1; i < nverts; j=i++)
		{
			const int* va = &verts[j*4];
			dd.vertex(bmin[0]+va[0]*cs, bmin[1]+va[1]*ch+j*0.01f, bmin[2]+va[2]*cs, colpt);
		}
		dd.end();

		extern int triangulate(int n, const int* verts, int* indices, int* tris);

		static int indices[nverts];
		static int tris[nverts*3];
		for (int j = 0; j < nverts; ++j)
			indices[j] = j;
			
		static int ntris = 0;
		if (!ntris)
		{
			ntris = triangulate(nverts, verts, &indices[0], &tris[0]);
			if (ntris < 0) ntris = -ntris;
		}
				
		const unsigned int coltri = duRGBA(255,255,255,64);
		dd.begin(DU_DRAW_TRIS);
		for (int i = 0; i < ntris*3; ++i)
		{
			const int* va = &verts[indices[tris[i]]*4];
			dd.vertex(bmin[0]+va[0]*cs, bmin[1]+va[1]*ch, bmin[2]+va[2]*cs, coltri);
		}
		dd.end();
		
	}
	dd.depthMask(true);*/
}

void Sample_Debug::handleRenderOverlay(double* /*proj*/, double* /*model*/, int* /*view*/)
{
}

void Sample_Debug::handleMeshChanged(InputGeom* geom)
{
	m_geom = geom;
}

const float* Sample_Debug::getBoundsMin() const
{
	if (m_cset)
		return m_cset->bmin;
	if (m_chf)
		return m_chf->bmin;
	if (m_navMesh)
		return m_bmin;
	return nullptr;
}

const float* Sample_Debug::getBoundsMax() const
{
	if (m_cset)
		return m_cset->bmax;
	if (m_chf)
		return m_chf->bmax;
	if (m_navMesh)
		return m_bmax;
	return nullptr;
}

void Sample_Debug::handleClick(const float* s, const float* p, const bool shift)
{
	if (m_tool)
		m_tool->handleClick(s, p, shift);
}

void Sample_Debug::handleToggle()
{
	if (m_tool)
		m_tool->handleToggle();
}

bool Sample_Debug::handleBuild()
{

	if (m_chf)
	{
		rcFreeContourSet(m_cset);
		m_cset = nullptr;
		
		// Create contours.
		m_cset = rcAllocContourSet();
		if (!m_cset)
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'cset'.");
			return false;
		}
		if (!rcBuildContours(m_ctx, *m_chf, /*m_cfg.maxSimplificationError*/1.3f, /*m_cfg.maxEdgeLen*/12, *m_cset))
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not create contours.");
			return false;
		}
	}
		
	return true;
}
