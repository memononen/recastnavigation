#pragma once
#include <cstdint>

class ValueHistory
{
	static constexpr int MAX_HISTORY = 256;
	float m_samples[MAX_HISTORY];
	int m_hsamples;
public:
	ValueHistory();

  void addSample(const float val) {
          m_hsamples = (m_hsamples + MAX_HISTORY - 1) % MAX_HISTORY;
          m_samples[m_hsamples] = val;
        }

        static int getSampleCount() {
		return MAX_HISTORY;
	}

  float getSample(const int i) const
	{
		return m_samples[(m_hsamples+i) % MAX_HISTORY];
	}
	
	float getSampleMin() const;
	float getSampleMax() const;
	float getAverage() const;
};

struct GraphParams
{
	void setRect(int ix, int iy, int iw, int ih, int ipad);
	void setValueRange(float ivmin, float ivmax, int indiv, const char *iunits);

        int x{};
        int y{};
        int w{};
        int h{};
        int pad{};
        float vmin{};
        float vmax{};
        int ndiv{};
	char units[16]{};
};

void drawGraphBackground(const GraphParams* p);

void drawGraph(const GraphParams* p, const ValueHistory* graph, int idx, const char* label, uint32_t col);
