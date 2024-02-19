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

#include "ChunkyTriMesh.h"

#include <cmath>
#include <cstdlib>
#include <new>

struct BoundsItem {
  float bmin[2];
  float bmax[2];
  int i;
};

namespace {
int compareItemX(const void *va, const void *vb) {
  const auto *a = static_cast<const BoundsItem *>(va);
  const auto *b = static_cast<const BoundsItem *>(vb);
  if (a->bmin[0] < b->bmin[0])
    return -1;
  if (a->bmin[0] > b->bmin[0])
    return 1;
  return 0;
}

int compareItemY(const void *va, const void *vb) {
  const auto *a = static_cast<const BoundsItem *>(va);
  const auto *b = static_cast<const BoundsItem *>(vb);
  if (a->bmin[1] < b->bmin[1])
    return -1;
  if (a->bmin[1] > b->bmin[1])
    return 1;
  return 0;
}

void calcExtends(const BoundsItem *items, const int /*nitems*/, const int imin, const int imax, float *bmin, float *bmax) {
  bmin[0] = items[imin].bmin[0];
  bmin[1] = items[imin].bmin[1];

  bmax[0] = items[imin].bmax[0];
  bmax[1] = items[imin].bmax[1];

  for (int i = imin + 1; i < imax; ++i) {
    const BoundsItem &it = items[i];
    if (it.bmin[0] < bmin[0])
      bmin[0] = it.bmin[0];
    if (it.bmin[1] < bmin[1])
      bmin[1] = it.bmin[1];

    if (it.bmax[0] > bmax[0])
      bmax[0] = it.bmax[0];
    if (it.bmax[1] > bmax[1])
      bmax[1] = it.bmax[1];
  }
}

int longestAxis(const float x, const float y) {
  return y > x ? 1 : 0;
}

void subdivide(BoundsItem *items, const int nitems, const int imin, const int imax, const int trisPerChunk, int &curNode, rcChunkyTriMeshNode *nodes, const int maxNodes, int &curTri, int *outTris, const int *inTris) {
  const int inum = imax - imin;
  const int icur = curNode;

  if (curNode >= maxNodes)
    return;

  rcChunkyTriMeshNode &node = nodes[curNode++];

  if (inum <= trisPerChunk) {
    // Leaf
    calcExtends(items, nitems, imin, imax, node.bmin, node.bmax);

    // Copy triangles.
    node.i = curTri;
    node.n = inum;

    for (int i = imin; i < imax; ++i) {
      const int *src = &inTris[items[i].i * 3];
      int *dst = &outTris[curTri * 3];
      curTri++;
      dst[0] = src[0];
      dst[1] = src[1];
      dst[2] = src[2];
    }
  } else {
    // Split
    calcExtends(items, nitems, imin, imax, node.bmin, node.bmax);

    const int axis = longestAxis(node.bmax[0] - node.bmin[0],
                                 node.bmax[1] - node.bmin[1]);

    if (axis == 0) {
      // Sort along x-axis
      std::qsort(items + imin, static_cast<std::size_t>(inum), sizeof(BoundsItem), compareItemX);
    } else if (axis == 1) {
      // Sort along y-axis
      std::qsort(items + imin, static_cast<std::size_t>(inum), sizeof(BoundsItem), compareItemY);
    }

    const int isplit = imin + inum / 2;

    // Left
    subdivide(items, nitems, imin, isplit, trisPerChunk, curNode, nodes, maxNodes, curTri, outTris, inTris);
    // Right
    subdivide(items, nitems, isplit, imax, trisPerChunk, curNode, nodes, maxNodes, curTri, outTris, inTris);

    const int iescape = curNode - icur;
    // Negative index means escape.
    node.i = -iescape;
  }
}

bool checkOverlapSegment(const float p[2], const float q[2], const float bmin[2], const float bmax[2]) {
  static constexpr float EPSILON = 1e-6f;

  float tmin = 0;
  float tmax = 1;
  float d[2];
  d[0] = q[0] - p[0];
  d[1] = q[1] - p[1];

  for (int i = 0; i < 2; i++) {
    if (std::abs(d[i]) < EPSILON) {
      // Ray is parallel to slab. No hit if origin not within slab
      if (p[i] < bmin[i] || p[i] > bmax[i])
        return false;
    } else {
      // Compute intersection t value of ray with near and far plane of slab
      const float ood = 1.0f / d[i];
      float t1 = (bmin[i] - p[i]) * ood;
      float t2 = (bmax[i] - p[i]) * ood;
      if (t1 > t2) {
        const float tmp = t1;
        t1 = t2;
        t2 = tmp;
      }
      if (t1 > tmin)
        tmin = t1;
      if (t2 < tmax)
        tmax = t2;
      if (tmin > tmax)
        return false;
    }
  }
  return true;
}
} // namespace

bool rcCreateChunkyTriMesh(const float *verts, const int *tris, const int ntris, const int trisPerChunk, rcChunkyTriMesh *cm) {
  const int nchunks = (ntris + trisPerChunk - 1) / trisPerChunk;

  cm->nodes = new (std::nothrow) rcChunkyTriMeshNode[nchunks * 4];
  if (!cm->nodes)
    return false;

  cm->tris = new (std::nothrow) int[ntris * 3];
  if (!cm->tris)
    return false;

  cm->ntris = ntris;

  // Build tree
  auto *items = new (std::nothrow) BoundsItem[ntris];
  if (!items)
    return false;

  for (int i = 0; i < ntris; i++) {
    const int *t = &tris[i * 3];
    BoundsItem &it = items[i];
    it.i = i;
    // Calc triangle XZ bounds.
    it.bmin[0] = it.bmax[0] = verts[t[0] * 3 + 0];
    it.bmin[1] = it.bmax[1] = verts[t[0] * 3 + 2];
    for (int j = 1; j < 3; ++j) {
      const float *v = &verts[t[j] * 3];
      if (v[0] < it.bmin[0])
        it.bmin[0] = v[0];
      if (v[2] < it.bmin[1])
        it.bmin[1] = v[2];

      if (v[0] > it.bmax[0])
        it.bmax[0] = v[0];
      if (v[2] > it.bmax[1])
        it.bmax[1] = v[2];
    }
  }

  int curTri = 0;
  int curNode = 0;
  subdivide(items, ntris, 0, ntris, trisPerChunk, curNode, cm->nodes, nchunks * 4, curTri, cm->tris, tris);

  delete[] items;

  cm->nnodes = curNode;

  // Calc max tris per node.
  cm->maxTrisPerChunk = 0;
  for (int i = 0; i < cm->nnodes; ++i) {
    const rcChunkyTriMeshNode &node = cm->nodes[i];
    const bool isLeaf = node.i >= 0;
    if (!isLeaf)
      continue;
    if (node.n > cm->maxTrisPerChunk)
      cm->maxTrisPerChunk = node.n;
  }

  return true;
}

inline bool checkOverlapRect(const float amin[2], const float amax[2], const float bmin[2], const float bmax[2]) {
  bool overlap = true;
  overlap = (amin[0] > bmax[0] || amax[0] < bmin[0]) ? false : overlap;
  overlap = (amin[1] > bmax[1] || amax[1] < bmin[1]) ? false : overlap;
  return overlap;
}

int rcGetChunksOverlappingRect(const rcChunkyTriMesh *cm, float bmin[2], float bmax[2], int *ids, const int maxIds) {
  // Traverse tree
  int i = 0;
  int n = 0;
  while (i < cm->nnodes) {
    const rcChunkyTriMeshNode *node = &cm->nodes[i];
    const bool overlap = checkOverlapRect(bmin, bmax, node->bmin, node->bmax);
    const bool isLeafNode = node->i >= 0;

    if (isLeafNode && overlap) {
      if (n < maxIds) {
        ids[n] = i;
        n++;
      }
    }

    if (overlap || isLeafNode)
      i++;
    else {
      const int escapeIndex = -node->i;
      i += escapeIndex;
    }
  }

  return n;
}

int rcGetChunksOverlappingSegment(const rcChunkyTriMesh *cm, float p[2], float q[2], int *ids, const int maxIds) {
  // Traverse tree
  int i = 0;
  int n = 0;
  while (i < cm->nnodes) {
    const rcChunkyTriMeshNode *node = &cm->nodes[i];
    const bool overlap = checkOverlapSegment(p, q, node->bmin, node->bmax);
    const bool isLeafNode = node->i >= 0;

    if (isLeafNode && overlap) {
      if (n < maxIds) {
        ids[n] = i;
        n++;
      }
    }

    if (overlap || isLeafNode)
      i++;
    else {
      const int escapeIndex = -node->i;
      i += escapeIndex;
    }
  }

  return n;
}
