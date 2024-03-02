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

#pragma once
#include <cstdint>
struct dtObstacleCircle {
  float p[3];         ///< Position of the obstacle
  float vel[3];       ///< Velocity of the obstacle
  float dvel[3];      ///< Velocity of the obstacle
  float rad;          ///< Radius of the obstacle
  float dp[3], np[3]; ///< Use for side selection during sampling.
};

struct dtObstacleSegment {
  float p[3], q[3]; ///< End points of the obstacle segment
  bool touch;
};

class dtObstacleAvoidanceDebugData {
public:
  dtObstacleAvoidanceDebugData();
  ~dtObstacleAvoidanceDebugData();

  bool init(int maxSamples);
  void reset();
  void addSample(const float *vel, float ssize, float pen,
                 float vpen, float vcpen, float spen, float tpen);

  void normalizeSamples() const;

  int getSampleCount() const { return m_nsamples; }
  const float *getSampleVelocity(const int i) const { return &m_vel[i * 3]; }
  float getSampleSize(const int i) const { return m_ssize[i]; }
  float getSamplePenalty(const int i) const { return m_pen[i]; }
  float getSampleDesiredVelocityPenalty(const int i) const { return m_vpen[i]; }
  float getSampleCurrentVelocityPenalty(const int i) const { return m_vcpen[i]; }
  float getSamplePreferredSidePenalty(const int i) const { return m_spen[i]; }
  float getSampleCollisionTimePenalty(const int i) const { return m_tpen[i]; }

  // Explicitly disabled copy constructor and copy assignment operator.
  dtObstacleAvoidanceDebugData(const dtObstacleAvoidanceDebugData &) = delete;
  dtObstacleAvoidanceDebugData &operator=(const dtObstacleAvoidanceDebugData &) = delete;

private:
  int m_nsamples;
  int m_maxSamples;
  float *m_vel;
  float *m_ssize;
  float *m_pen;
  float *m_vpen;
  float *m_vcpen;
  float *m_spen;
  float *m_tpen;
};

dtObstacleAvoidanceDebugData *dtAllocObstacleAvoidanceDebugData();
void dtFreeObstacleAvoidanceDebugData(dtObstacleAvoidanceDebugData *ptr);

static constexpr int DT_MAX_PATTERN_DIVS = 32; ///< Max numver of adaptive divs.
static constexpr int DT_MAX_PATTERN_RINGS = 4; ///< Max number of adaptive rings.

struct dtObstacleAvoidanceParams {
  float velBias;
  float weightDesVel;
  float weightCurVel;
  float weightSide;
  float weightToi;
  float horizTime;
  uint8_t gridSize;      ///< grid
  uint8_t adaptiveDivs;  ///< adaptive
  uint8_t adaptiveRings; ///< adaptive
  uint8_t adaptiveDepth; ///< adaptive
};

class dtObstacleAvoidanceQuery {
public:
  dtObstacleAvoidanceQuery() = default;
  ~dtObstacleAvoidanceQuery();

  bool init(int maxCircles, int maxSegments);

  void reset();

  void addCircle(const float *pos, float rad,
                 const float *vel, const float *dvel);

  void addSegment(const float *p, const float *q);

  int sampleVelocityGrid(const float *pos, float rad, float vmax,
                         const float *vel, const float *dvel, float *nvel,
                         const dtObstacleAvoidanceParams *params,
                         dtObstacleAvoidanceDebugData *debug = nullptr);

  int sampleVelocityAdaptive(const float *pos, float rad, float vmax,
                             const float *vel, const float *dvel, float *nvel,
                             const dtObstacleAvoidanceParams *params,
                             dtObstacleAvoidanceDebugData *debug = nullptr);

  int getObstacleCircleCount() const { return m_ncircles; }
  const dtObstacleCircle *getObstacleCircle(const int i) const { return &m_circles[i]; }

  int getObstacleSegmentCount() const { return m_nsegments; }
  const dtObstacleSegment *getObstacleSegment(const int i) const { return &m_segments[i]; }

  // Explicitly disabled copy constructor and copy assignment operator.
  dtObstacleAvoidanceQuery(const dtObstacleAvoidanceQuery &) = delete;
  dtObstacleAvoidanceQuery &operator=(const dtObstacleAvoidanceQuery &) = delete;

private:
  void prepare(const float *pos, const float *dvel) const;

  float processSample(const float *vcand, float cs, const float *pos, float rad, const float *vel, const float *dvel, float minPenalty, dtObstacleAvoidanceDebugData *debug) const;

  dtObstacleAvoidanceParams m_params{};
  float m_invHorizTime{};
  float m_vmax{};
  float m_invVmax{};

  int m_maxCircles{};
  dtObstacleCircle *m_circles{nullptr};
  int m_ncircles{};

  int m_maxSegments{};
  dtObstacleSegment *m_segments{nullptr};
  int m_nsegments{};
};

dtObstacleAvoidanceQuery *dtAllocObstacleAvoidanceQuery();
void dtFreeObstacleAvoidanceQuery(dtObstacleAvoidanceQuery *ptr);
